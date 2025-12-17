#include <LiquidCrystal_I2C.h> 
#include <EEPROM.h>
#include <DHT.h>

// Definiciones de la LCD I2C
LiquidCrystal_I2C lcd(0x27, 16, 2); 

// Configuración del sensor DHT
#define PIN_DHT 7     // Pin digital para el DHT11
#define TIPO_DHT DHT11
DHT dht(PIN_DHT, TIPO_DHT);

// Definimos los valores umbrales para los sensores
#define TEMP_ADVERTENCIA 28 // Grados °C
#define TEMP_ALERTA 38     // Grados °C

#define LUZ_ADVERTENCIA 600
#define LUZ_ALERTA 800

#define NIVEL_ADVERTENCIA 120
#define NIVEL_ALERTA 200

// Bloqueo Atómico (cli/sei): El tiempo que se "pausan" las interrupciones en el loop para guardar la temperatura es de microsegundos (solo copiar 2 bytes), así que no afecta en absoluto al funcionamiento del resto del sistema


// Constantes para almacenamiento y lógica
const uint8_t MAX_MUESTRAS = 10;
const uint8_t BYTES_POR_MUESTRA = 6; 
const uint16_t EEPROM_DIRECCION_INICIO = 0; 
const uint16_t DIR_INDICE_PERSISTENTE = 100; // Dirección para guardar el índice
const uint16_t TIEMPO_BUZZER_MAX = 1500; // 15 segundos * 100 ticks/seg (10ms cada tick)
const uint8_t TIEMPO_DEBOUNCE = 20;      // 200 ms * 100 ticks/seg (Anti-rebote)


// Estructura de datos para las muestras (Global)
struct Muestra {
    uint16_t t;
    uint16_t l;
    uint16_t n;
};

// LCD = 0: Tiempo Real, LCD = 1: Histórico
volatile uint8_t modo_lcd = 0; 
volatile uint8_t indice_historial = 0; // Índice para navegar el histórico (0-9)
volatile uint8_t indice_eeprom = 0;    // Índice de la última muestra guardada (0-9)
volatile uint8_t indice_snapshot = 0;  // Captura del índice para congelar la vista del historial en la pantalla LCD

// Buffer en RAM para visualización estática del historial
Muestra historial_ram[MAX_MUESTRAS];

// Variables de sensores
// valores_adc[0] = Temperatura (DHT), [1] = Luz (ADC1), [2] = Nivel (ADC2)
volatile uint16_t valores_sensores[3] = {0, 0, 0}; 
volatile uint8_t canal_actual_adc = 1; // Arrancamos en 1 porque 0 es temperatura digital



// Variables de estado y control
volatile bool alarma_activa = false;
volatile uint8_t estado_sistema = 0;  // 0: OK, 1: Advertencia, 2: Alerta (Global para uso en LCD)
volatile bool buzzer_encendido = false;
volatile uint16_t contador_tiempo_buzzer = 0; // Cuenta hasta 15s
volatile uint8_t contador_debounce = 0;       // Para evitar rebotes del botón
volatile bool guardar_eeprom = false;         // Bandera para guardar en EEPROM desde el loop

// Temporizador para leer DHT en el loop (no bloqueante)
unsigned long ultimo_tiempo_dht = 0;
const long INTERVALO_DHT = 2000; // Leer cada 2 segundos

// Configurar ADC en modo Free-Running con interrupciones
void adc_init() {
    ADMUX = (1 << REFS0);       // Selecciona a VCC (5V) como voltaje de referencia para q el ADC sepa que el max es 1023, inicialmente apunta al canal 0 (ADC0)
    ADCSRA = (1 << ADEN) |      // Prende el ADC

             (1 << ADATE) |     // Habilita el Auto trigger (disparo automático), significa que cuando termina una conversión, puede arrancar otra automáticamente

             (1 << ADIE) |      // Habilita la Interrupción del ADC. Esto es clave porque cada vez que termine de medir y hacer la conversión, se ejecuta la función ISR(ADC_vect)

             (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Configura el Prescaler a 128. Configurar el prescaler a 128 en los registros del Arduino tiene un impacto en la Frecuencia de funcionamiento de componentes específicos del microcontrolador (como en el ADC o los Timers/Contadores). El prescaler es un divisor de frecuencia, toma la señal del reloj principal del Arduino (16 MHz) y la divide por un factor preestablecido (128 en este caso)
             
    ADCSRB = 0x00; // Selecciona el modo "Free Running". En cuanto termina una conversión, inicia la siguiente inmediatamente. Tan pronto como el ADC termina de convertir la lectura de un pin analógico (ej el A0) y el resultado se guarda en el registro de datos, el ADC se dispara automáticamente (inicia) la siguiente conversión sin necesidad de una nueva instrucción de software o un evento externo
    //Free Running se refiere a la configuración del Modo de Disparo (Trigger Mode) del ADC en el Arduino. ADCSRB = 0x00; hace lo siguiente: Modo Seleccionado: "Free Running" (Funcionamiento Libre)
    // Al escribir 0x00  en el registro ADCSRB (ADC Control and Status Register B), se configuran los bits ADTS2:0 (ADC Auto Trigger Source) a 000. Este valor 000 corresponde al modo Free Running.

    // Configurar primer canal analógico a leer: Canal 1 (Luz)
    // El canal 0 ya no se usa via ADC porque es digital (DHT)
    canal_actual_adc = 1; 
    ADMUX = (ADMUX & 0xF8) | canal_actual_adc;

    ADCSRA |= (1 << ADSC); // Inicia la primer conversión (dsp el modo Free Running dispara automaticamente las otras conversiones, pero la primera conversion debe inciarse manualmente). Es el boton de arranque para comenzar el ciclo continuo de conversiones
     // Esta instrucción tiene dos partes:
        // 1 - Registro y Bit
            // ADCSRA: Es el registro de Control y Estado A del ADC
            // ADSC: Es el bit ADC Start Conversion dentro de ese registro
        // 2 - Acción: Iniciar la Conversión(1 << ADSC): Crea una máscara binaria donde solo el bit ADSC está en 1. |=: Es el operador OR a nivel de bits (Bitwise OR) y asignación. Lo que hace es establecer (poner en 1) el bit ADSC sin modificar el estado de ningún otro bit en el registro ADCSRA.El Efecto: Al establecer el bit ADSC a 1, le estás dando la orden al hardware del microcontrolador de Iniciar la Conversión Analógica-Digital
}


// La funcion timer1_init ejecuta la función ISR (para hacer la interrupcion) cada vez que el Timer 1 cuenta 20.000 pulsos de su reloj pre-escalado
// Se usa el Timer1 (16 bits) para generar una base de tiempo exacta de 10ms (100 Hz)
// Modo CTC (Clear Timer on Compare Match):
    // TCCR1B = (1 << WGM12): El timer cuenta hasta un valor tope y se reinicia
// Prescaler:
        // TCCR1B |= (1 << CS11): Divide el reloj principal por 8 (16MHz / 8 = 2MHz)
// Valor de Comparación (Tope):
    // OCR1A = 20000: Con el reloj a 2MHz, contar 20000 pulsos toma 10ms (20.000/2.000.000 = 0.01s)
// Interrupción:
    // TIMSK1 = (1 << OCIE1A): Habilita la interrupción cuando el timer llega a 20.000 pulsos para ello Ejecuta ISR(TIMER1_COMPA_vect)


void timer1_init() {
    TCCR1A = 0x00;
    TCCR1B = (1 << WGM12) | (1 << CS11); // Activa el modo CTC (Clear Timer on Compare Match). El timer cuenta desde 0 hasta el valor de OCR1A. Cuando llega (es pq matcheo el valor del timer 1 "TCNT1" con el valor de OCR1A), el contador del timer vuelve a 0 automáticamente
    OCR1A = 20000; // 16MHz / (8*100) = 20000 (100 Hz -> 10 ms)
    TIMSK1 = (1 << OCIE1A); // Habilita la interrupción, cuando el contador llega a 20000 ejecuta la ISR(TIMER1_COMPA_vect)

}



// Interrupción externa realizada por Boton/Pulsador - por cambio de estado en el pin D8 (PCINT0)
// Esta funcion se usa para detectar el botón en el pin digital 8 (Puerto B, bit 0)
    // PCICR |= (1 << PCIE0): Habilita el grupo de interrupciones 0 (que controla los pines D8 a D13)
    // PCMSK0 |= (1 << PCINT0): Habilita específicamente la interrupción para el pin D8. Cualquier cambio de estado (LOW a HIGH o viceversa) dispara ISR(PCINT0_vect) la cual es la función que se ejecuta cuando se produce la interrupción
void pcint_init() {
    PCICR |= (1 << PCIE0);    // Habilita el Grupo 0 de interrupciones, que corresponde a los pines D8 hasta D13 (Puerto B)
    PCMSK0 |= (1 << PCINT0); // Dentro de ese grupo, habilita específicamente el pin D8. Si el estado del pin 8 cambia (0 a 1 o 1 a 0), se dispara la interrupción
}



// SETUP (definimos pines de entrada/salida, inicializamos perifericos y habilitamos interrupciones globales)
void setup() {
    // Inicializamos LCD
    lcd.init();
    lcd.backlight();
    lcd.print("Iniciando...");
    delay(1000);
    lcd.clear();

    // Inicializar sensor DHT
    dht.begin();

    // Configuración de PINES
    // D4, D5, D6 como SALIDAS (LEDs y Buzzer)
    DDRD |= (1 << DDD4) | (1 << DDD5) | (1 << DDD6);

    // PINES DE ENTRADA
    DDRB &= ~(1 << DDB0); // Pone el bit 0 del Puerto B (Pin 8) como Entrada (el del Boton/Pulsador)

    // Activar RESISTENCIAS PULL-UP internas para sensores analógicos
    // A1 = PC1 (Pin analógico 1), A2 = PC2 (Pin analógico 2)
    PORTC |= (1 << PORTC1) | (1 << PORTC2);
    
    // Recuperar índice de EEPROM para mantener continuidad en el historial
    indice_eeprom = EEPROM.read(DIR_INDICE_PERSISTENTE);
    // Validación por si es la primera vez (0xFF) o basura
    if (indice_eeprom >= MAX_MUESTRAS) {
        indice_eeprom = 0;
    }

    adc_init();
    timer1_init();
    pcint_init();

    sei(); // Habilitar interrupciones globales
}


// LOOP (Mostramos los valores en la pantalla LCD)
void loop() {
    static uint8_t ultimo_modo = 255; // Para detectar cambio de modo y limpiar pantalla

    // Detectar cambio de modo para limpiar pantalla y CARGAR SNAPSHOT
    if (modo_lcd != ultimo_modo) {
        lcd.clear();
        ultimo_modo = modo_lcd;
        
        // Si entramos al modo Historial (1), cargamos la RAM desde la EEPROM
        if (modo_lcd == 1) {
            // Recorremos los 10 registros y los ordenamos en RAM
            // historial_ram[0] será el mpas reciente, historial_ram[9] el más antiguo
            uint8_t ptr_escritura = indice_eeprom; // Donde se escribirá el próximo (o se estaba por escribir)
            
            for (int i = 0; i < MAX_MUESTRAS; i++) {
                // Formula circular inversa: (Puntero - 1 - i)
                int16_t index_calc = ptr_escritura - 1 - i;
                if (index_calc < 0) index_calc += MAX_MUESTRAS;
                
                uint16_t addr = EEPROM_DIRECCION_INICIO + (index_calc * BYTES_POR_MUESTRA);
                EEPROM.get(addr, historial_ram[i]);
            }
        }

        // Pequeño delay para asegurar que el clear se procese visualmente antes de escribir
        delay(10); 
    }

    // Gestion de EEPROM
    if (guardar_eeprom) {
        guardar_eeprom = false; // Bajamos la bandera

        // Calculamos dirección usando la constante correcta EEPROM_DIRECCION_INICIO
        uint16_t addr_base = EEPROM_DIRECCION_INICIO + (indice_eeprom * BYTES_POR_MUESTRA);
        
        Muestra datos_actuales;
        
        // Copia atómica de los valores volátiles
        // Y escritura protegida para evitar corrupción por interrupciones
        uint8_t sreg_old = SREG;
        cli();  
        
        datos_actuales.t = valores_sensores[0];
        datos_actuales.l = valores_sensores[1]; // A1 (Luz) Directo
        datos_actuales.n = valores_sensores[2]; // A2 (Nivel) Directo

        EEPROM.put(addr_base, datos_actuales);
        
        SREG = sreg_old; // Reactivar interrupciones (sei implícito si estaban activas)
        
        // Guardamos el NUEVO índice para la próxima vez
        // Nota: indice_eeprom apunta al lugar donde escribiremos LA PROXIMA.
        // Lo guardamos ANTES de incrementar? No, guardamos el que acabamos de usar?
        // Queremos saber dónde escribir al reiniciar.
        // Entoces: Escribimos en X. Incrementamos a X+1. Guardamos X+1.
        
        // Avanzar índice circular
        indice_eeprom++;
        if (indice_eeprom >= MAX_MUESTRAS) {
            indice_eeprom = 0;
        }
        
        // Persistir el índice del puntero de escritura
        EEPROM.write(DIR_INDICE_PERSISTENTE, indice_eeprom);
    }

    // Lectura del DHT11 (Digital, lento, fuera de ISR)
    unsigned long tiempo_actual = millis();
    if (tiempo_actual - ultimo_tiempo_dht >= INTERVALO_DHT) {
        ultimo_tiempo_dht = tiempo_actual;
        
        // Leemos temperatura
        float t = dht.readTemperature();
        
        // Si la lectura es válida, actualizamos variable global
        // Guardamos en valores_sensores[0] casteado a int para compatibilidad
        if (!isnan(t)) {
            // Deshabilitamos interrupciones momentáneamente para escritura atómica de variable compartida de 16bits
            uint8_t sreg_old = SREG;
            cli();
            valores_sensores[0] = (uint16_t)t;
            SREG = sreg_old;
        }
    }

    // Actualizamos Pantalla LCD
    if (modo_lcd == 0) {
        // MODO 0: TIEMPO REAL
        uint16_t temp, luz, nivel; //Declaramos variables locales para almacenar los valores leidos
        
        //Lectura atomica de las variables volatiles
        uint8_t sreg_old = SREG;
        cli();
        temp = valores_sensores[0];
        luz  = valores_sensores[1]; // A1 (Luz)
        nivel = valores_sensores[2]; // A2 (Nivel)
        SREG = sreg_old;
        
        lcd.setCursor(0, 0); 
        lcd.print("T:");
        lcd.print(temp);
        lcd.print("  L:");
        lcd.print(luz);
        lcd.print("   "); // Limpiar residuos
        
        lcd.setCursor(0, 1);
        lcd.print("N:");
        lcd.print(nivel);
        lcd.print(" ");
        
        if (estado_sistema == 2) {
            lcd.print("ALERTA!!!");
        } else if (estado_sistema == 1) {
            lcd.print("Warning");
        } else {
            lcd.print("OK      "); 
        }

    } else {
        // LCD en Modo 1: Historico de muestras
        // Leemos directamente del buffer RAM que cargamos al entrar al modo.
        // Esto garantiza ESTABILIDAD (Snapshot) y RAPIDEZ.
        
        Muestra datos_historicos = historial_ram[indice_historial];

        lcd.setCursor(0, 0);
        lcd.print("R:-");             // R:-X indica "X muestras atrás"
        lcd.print(indice_historial);  // 0 a 9
        lcd.print(" T:");             // 3 chars
        lcd.print(datos_historicos.t); // 2-3 chars
        lcd.print("C");               // 1 char -> Total ~10-11 chars (Entra en 16)
        
        lcd.setCursor(0, 1);
        lcd.print("L:");              // 2 chars
        lcd.print(datos_historicos.l); // 1-4 chars
        lcd.print(" N:");             // 3 chars
        lcd.print(datos_historicos.n); // 1-4 chars
    }
    
    // Retardo para no saturar la comunicación I2C (POSIBLEMENTE CAMBIAR A FUNCION MILLIS)
    delay(200); 
}



//INTERRUPCIONES

//ISR(ADC_vect) - Muestreo Rotativo: esta rutina se ejecuta automaticamente al ocurrir la interrupcion del ADC (la que toma valores y hace la conversion ADC)
// ISR(ADC_vect) hace la Rotación del Canal para tomar los valores de los 3 sensores uno a la vez (usando un canal analogico para cada sensor)
// Esta rutina permite que los 3 sensores se actualicen constantemente de fondo/en segundo plano mediante interrupciones ISR sin q el loop haga nada, asi el loop() principal esta libre para manejar otras tareas sin preocuparse por los tiempos de espera de las lecturas de los sensores

//La interrupción salta cada vez que se termina de leer UN solo sensor. La configuracion es:
// Frecuencia del Arduino: 16 MHz con Prescaler = 128, entonces la velocidad del ADC es de 125 kHz (16 Mhz / 128). Una conversion toma 13 ciclos de ese tiempo, entonces tiempo de conversion = 13 * 125 Khz = 104 uS
// Por lo tanto la funcion ISR(ADC_vect) se ejecuta cada 104 uS (aprox. 9600 veces por segundo)
// La interrupción salta después de leer UN SOLO sensor. Funciona así:
// Termina conversion sensor 0 -> INTERRUPCIÓN -> Guardas valor 0 -> Cambias multiplexor al 1, pasan 104 µs
// Termina conversión sensor 1 -> INTERRUPCIÓN -> Guardas valor 1 -> Cambias multiplexor al 2, pasan 104 µs
// Termina conversión sensor 2 -> INTERRUPCIÓN -> Guardas valor 2 -> Cambias multiplexor al 0, pasan 104 µs
// ISR ADC: Muestreo de sensores analogicos (SOLO LUZ y NIVEL)
ISR(ADC_vect) {
    uint16_t lectura = ADC;
    
    // Asignación DIRECTA: Lo que leo del canal X va al índice X
    valores_sensores[canal_actual_adc] = lectura;
    
    // Alternar solo entre Canal 1 (Luz) y Canal 2 (Nivel)
    if (canal_actual_adc == 1) {
        canal_actual_adc = 2; // Siguiente: Canal 2
    } else {
        canal_actual_adc = 1; // Siguiente: Canal 1
    }
    
    // Configurar multiplexor para la PRÓXIMA conversión
    ADMUX = (ADMUX & 0xF8) | canal_actual_adc;
}

//ISR TIMER1: Logica de Alarma, Buzzer Temporizado y Debounce
ISR(TIMER1_COMPA_vect) {
    static uint16_t contador_1s = 0; // uint16_t para contar hasta 400 ms
    static uint8_t contador_parpadeo = 0;
    
    // Decrementar contador de debounce si está activo
    if (contador_debounce > 0) {
        contador_debounce--;
    }

    uint16_t temp = valores_sensores[0];
    uint16_t luz  = valores_sensores[1]; // A1 (Luz)
    uint16_t nivel = valores_sensores[2]; // A2 (Nivel)
    
    // 0: Normal, 1: Advertencia, 2: Alerta
    uint8_t estado = 0; 

    // Chequeo de condiciones (Prioridad: Alerta > Advertencia > Normal)
    
    // Chequeo de Advertencias
    if (temp > TEMP_ADVERTENCIA) estado = 1;
    if (luz > LUZ_ADVERTENCIA) estado = 1;    // Luz alta es el problema (> 600)
    if (nivel > NIVEL_ADVERTENCIA) estado = 1;

    // Chequeo de Alertas (sobrescribe advertencia)
    if (temp > TEMP_ALERTA) estado = 2;
    if (luz > LUZ_ALERTA) estado = 2;         // Luz muy alta es alerta (> 800)
    if (nivel > NIVEL_ALERTA) estado = 2;

    estado_sistema = estado; // Actualizamos variable global para el LCD
    alarma_activa = (estado == 2); 
    
    // LÓGICA DE BUZZER CON LATCH (Memoria de 15 segundos)
    // Si se dispara la alarma (estado 2) y no estaba contando, inicia la cuenta.
    if (estado == 2 && contador_tiempo_buzzer == 0) {
        contador_tiempo_buzzer = 1; 
    }
    
    // Si la cuenta está en progreso (1 a 1500), mantén el buzzer prendido
    if (contador_tiempo_buzzer > 0 && contador_tiempo_buzzer <= 1500) {
        buzzer_encendido = true;
        contador_tiempo_buzzer++;
        
        // Al terminar los 15s (1500 ticks), bloqueamos el buzzer
        if (contador_tiempo_buzzer > 1500) {
            contador_tiempo_buzzer = 1501; // Estado "Apagado"
        }
    } else {
        // Estado 0 (Reposo) o 1501 (Apagado) -> Buzzer Apagado
        buzzer_encendido = false;
        
        // Reset del Latch: Solo si la alarma SE FUE (estado != 2) y ya habíamos terminado (1501)
        // permitimos volver a 0 para que una futura alarma vuelva a sonar.
        if (estado != 2 && contador_tiempo_buzzer == 1501) {
            contador_tiempo_buzzer = 0;
        }
    }
    
    // Control de Hardware (LEDs y Buzzer)
    // D4 (OK), D5 (Alerta), D6 (Buzzer)
    
    contador_parpadeo++;
    if (contador_parpadeo >= 50) contador_parpadeo = 0; 

    // Apagar todo por defecto para setear segun estado
    // Buzzer se controla por la variable 'buzzer_encendido' derivada del latch
    if (buzzer_encendido) {
        PORTD |= (1 << PORTD6);
    } else {
        PORTD &= ~(1 << PORTD6);
    }

    switch (estado) {
        case 0: // NORMAL
            PORTD |= (1 << PORTD4);  // Amarillo ON
            PORTD &= ~(1 << PORTD5); // Rojo OFF
            break;
            
        case 1: // ADVERTENCIA
            PORTD |= (1 << PORTD4);  // Amarillo ON
            // Rojo Parpadea
            if (contador_parpadeo < 25) {
                PORTD |= (1 << PORTD5);
            } else {
                PORTD &= ~(1 << PORTD5);
            }
            break;
            
        case 2: // ALERTA
            PORTD &= ~(1 << PORTD4); // Amarillo OFF
            PORTD |= (1 << PORTD5);  // Rojo ON
            break;
    }
    
    // LOGICA DE GUARDADO EN EEPROM cada 30 segundos
    // Cada 3000 llamadas (10ms * 3000 = 30s)
    contador_1s++;
    if (contador_1s >= 3000) {
        contador_1s = 0;
        guardar_eeprom = true;
    }
}



// ISR PCINT0 - Boton (D8): cuando apretamos el boton, cambia el estado de silencio y el modo LCD pasa de 0 a 1 donde muestra los valores historicos de las muestras (cada vez q apretamos el boton nos movemos hasta el registro 10 y cuando llega a ese, al apretar el boton otra vez se cambia el modo LCD a 0 y se vuelve al modo Tiempo Real mostrando los valores actuales de los sensores)
// La lectura inicial en el modo historico comienza mostrando la ultima muestra guardada (la de hace 1 segundo. Esto es porque inicializamos "history_index" para apuntar un lugar antes de donde se guardara la siguiente muestra
ISR(PCINT0_vect) {
    // Verificar si el debounce permite una nueva pulsación
    if (contador_debounce == 0) {
        // Chequear si el pin está realmente en el estado activo (Asumiendo activo ALTO por defecto o cambio de estado)
        // Como es interrupción por cambio (Change), entra al soltar y al apretar.
        // Si el usuario tiene pull-down (boton a VCC): leer 1 es presionado
        // Si el usuario tiene pull-up (boton a GND): leer 0 es presionado
        // Vamos a asumir que detectamos el cambio y actuamos, pero bloqueamos por un tiempo.
        // Simplemente actuamos ante el evento y bloqueamos.
        
        // Seteamos tiempo de bloqueo (200ms aprox)
        contador_debounce = TIEMPO_DEBOUNCE;
        
        // Lógica de navegación del LCD
        if (modo_lcd == 0) {
            modo_lcd = 1; // Ir a histórico
            indice_snapshot = indice_eeprom; // CONGELAR el estado actual para la visualización
            indice_historial = 0; // Mostrar la más reciente (0 pasos atrás)
        } else {
            // Estamos en histórico, retrocedemos una muestra
            indice_historial++;
            
            // Si pasamos las 10 muestras (0 a 9), volvemos a tiempo real
            if (indice_historial >= MAX_MUESTRAS) {
                modo_lcd = 0; 
            }
        }
    }
}