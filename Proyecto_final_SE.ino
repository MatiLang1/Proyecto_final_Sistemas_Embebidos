// Importamos librerias, Definimos constantes y variables globales, Creamos las interrupciones con trabajo de registros, hacemos el setup, hacemos el loop, hacemos las subrutinas q se ejecutan tras las interrupciones

#include <LiquidCrystal_I2C.h> // Libreria para la LCD I2C
#include <EEPROM.h> // Libreria para la EEPROM
#include <DHT.h> // Libreria para el sensor de Temperatura DHT11

// Definiciones de la LCD I2C
LiquidCrystal_I2C lcd(0x27, 16, 2); 

// Configuramos el sensor de Temperatura
#define PIN_DHT 7     // Pin digital 7 para el DHT11
#define TIPO_DHT DHT11 // Le indicamos a la libreria DHT que tipo de sensor es (en este caso DHT11)
DHT dht(PIN_DHT, TIPO_DHT); // DHT es la clase, dht es una instancia de la clase DHT, a dicho objeto le pasamos el pin y el tipo de sensor (asi le indicamos al objeto en q pin trabajar y q tipo de sensor es). Esto nos permi q cuando quiera leer temperatura, solo llamamos a dht.readTemperature() el cual es uno de los metodos que tiene la clase DHT

// Definimos los valores umbrales para los sensores
#define TEMP_ADVERTENCIA 28 // Grados °C
#define TEMP_ALERTA 38     // Grados °C
#define LUZ_ADVERTENCIA 600
#define LUZ_ALERTA 800
#define NIVEL_ADVERTENCIA 120
#define NIVEL_ALERTA 200

// Para poder guardar valores en la EEPROM sin ser interrumpido por las interrupciones, realizamos un Bloqueo Atómico (cli/sei) . El tiempo que se pausan las interrupciones en el loop para guardar la temperatura es de microsegundos (solo copiar 2 bytes), por lo q no afecta en absoluto al funcionamiento del sistema


// Constantes para almacenamiento y lógica
const uint8_t MAX_MUESTRAS = 10; // Muestras que se guardan en la EEPROM
const uint8_t BYTES_POR_MUESTRA = 6; // Cantidad de bytes por muestra (2 para la Temperatura, 2 para la Luz y 2 para el Nivel)
const uint16_t EEPROM_DIRECCION_INICIO = 0; // Dirección inicial de la EEPROM
const uint16_t DIR_INDICE_PERSISTENTE = 100; // Dirección para guardar el índice. Lo usamos para recordar en que posición (0 al 9) nos quedamos escribiendo antes de apagarse el Arduino (si no guardáramos este número, al reiniciar el Arduino empezaría siempre desde el 0)

const uint16_t TIEMPO_BUZZER_MAX = 1500; // El Timer1 hace 100 ticks por segundo (10ms cada tick) por ello, para q el buzzer dure 15 S, se usa 1500 ticks. El N 1500 indica cuántas veces tiene que despertarse el timer antes de apagar el buzzer

const uint8_t TIEMPO_DEBOUNCE = 20; // 20 ticks por 10 ms = 200 ms q esperamos para liberar al boton (antirebote)


// Creamos una Estructura de datos para las muestras (Global)
struct Muestra {
    uint16_t t;
    uint16_t l;
    uint16_t n;
};


// Variables globales (son volatiles pq pueden ser afectadas por interrupciones asiq no deben ser cacheadas en el registro sino que siempre deben ser leidas de la RAM)
volatile uint8_t modo_lcd = 0; // Para controlar el modo de la LCD (LCD = 0 muestras en Tiempo Real, LCD = 1 Historial de muestras)
volatile uint8_t indice_historial = 0; // Indice para navegar el histórico (0-9), lo controlamos con el boton

volatile uint8_t indice_eeprom = 0;    // Indice q apunta a la posición (0-9) donde se va a guardar el siguiente dato nuevo. Avanza automaticamente cada 30 segundos

volatile uint8_t indice_snapshot = 0;  // Es una copia de "indice_eeprom" que se toma en el instante en que entras al "Historico de muestras". Sirve para congelar la referencia (aunque el sistema siga escribiendo nuevos datos y moviendo indice_eeprom, nosotros vamos a ver en la LCD los datos guardados en la EEPROM hasta el momento en q presionamos el boton para acceder al "Historico de muestras")

// Buffer en RAM para visualizacion estatica del historial. Reservamos espacio en la RAMpara guardar 10 copias exactas de los datos (MAX_MUESTRAS). Cuando apretas el botón de historial, el Arduino va a la EEPROM, copia todo en RAM y muestra esta copia. Asi logramos el efecto "Snapshot" (miramos esta copia quieta en RAM mientras el Arduino sigue escribiendo en la EEPROM real)
Muestra historial_ram[MAX_MUESTRAS]; // "historial_ram" es un array de estructuras (de "Muestra"), su tamaño es de MAX_MUESTRAS (q es 10)


// VARIABLES DE SENSORES
// valores_adc[0] = Temperatura (DHT), valores_adc[1] = Luz (ADC1), valores_adc[2] = Nivel (ADC2)
volatile uint16_t valores_sensores[3] = {0, 0, 0}; 
volatile uint8_t canal_actual_adc = 1; // Arrancamos en 1 porque 0 es temperatura digital


// VARIABLES DE ESTADO Y CONTROL
volatile bool alarma_activa = false; // Flag para controlar el estado de la alarma (arranca apagada)
volatile uint8_t estado_sistema = 0; // 0 = OK, 1 = Advertencia, 2 = Alerta
volatile bool buzzer_encendido = false; // Flag para controlar el estado del buzzer (arranca apagado)
volatile uint16_t contador_tiempo_buzzer = 0; // Cuenta hasta 15s y apaga el buzzer
volatile uint8_t contador_debounce = 0; // Para evitar rebotes del botón (cuenta 200 ms, acepta clics del boton cada minimo 200 ms)
volatile bool guardar_eeprom = false; // Flag para guardar en EEPROM desde el loop (la interrupcion de ADC activa esta flag y cuando eso pasa, el loop guarda en la EEPROM)


// TEMPORIZADOR PARA LEER DHT EN EL LOOP, es NO BLOQUEANTE osea q no bloquea el loop sino q sigue ejecutando lineas hasta q pasen los 2 segundos, para ello usamos millis()
// Cada vez q pasa por la linea del loop, verifica si pasaron 2s (sino pasaron, sigue ejecutando cosas, por ello las interrupciones son + rapidas ya que recien ejecuta la linea de codigo cuando vuelve a pasar por ella y ya pasaron > 2s)
unsigned long ultimo_tiempo_dht = 0;
const long INTERVALO_DHT = 2000; // Leemos el valor del sensor de Temp cada 2 segundos




// INTERRUPCIONES

//INTERRUPCION ADC (para tomar valores de los sensores analogicos)

//Constantes de Arduino para los valores decimales: REFS0 es el numero 6 en decimal (indica la posicion 6 de los 8 bits, cuando haces el << REFS0, el bit 1 se desplaza 6 lugares a la izquierda), ADEN es el numero 7, ADATE es el numero 5, ADIE es el numero 3 (podriamos usar los valores en decimal directamente en el registro)
// ADCSRA = (1 << ADEN) | (1 << ADATE) | (1 << ADIE); habilitamos el ADC (con ADEN), el disparo automático (con ADATE) y la interrupción (con ADIE)"
//el | es para sumar los bits (001 + 010 = 0111)

// Inicializamos/Configuramos el ADC en modo Free-Running con interrupciones
void adc_init() {

    //1 es 00000001 y REFS0 es el número 6, que indica la posición del bit. Entonces (1 << REFS0 = correr el bit 1 6 lugares a la izquierda), resultado = 01000000 el cual asignamos al registro "ADMUX". Asi igual con todos

    ADMUX = (1 << REFS0); //Elegimos VCC (5V) como voltaje de referencia para q el ADC sepa que el max es 1023, inicialmente apunta al canal 0 (ADC0)
    
    ADCSRA = (1 << ADEN) | // Prende el ADC

             (1 << ADATE) | // Habilita el Auto trigger (disparo automático), significa que cuando termina una conversión, puede arrancar otra automáticamente

             (1 << ADIE) | // Habilita la Interrupción del ADC. Esto es clave porque cada vez que termine de medir y hacer la conversión, se ejecuta la función ISR(ADC_vect)

             (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Configura el Prescaler a 128 (el prescaler divide la señal del reloj principal del Arduino (16 MHz) por el factor indicado, 128 aca)
             
    ADCSRB = 0x00; // Selecciona el modo "Free Running". En cuanto termina una conversión, inicia la siguiente inmediatamente. Apenas el ADC termina de convertir la lectura de un pin analógico (ej el A0) en digital y el resultado se guarda en el registro de datos, el ADC se dispara automáticamente iniciando la siguiente conversión sin necesidad de una nueva instrucción o un evento externo
    //Free Running se refiere a la configuración del Modo de Disparo/Trigger del ADC en el Arduino
    // Al escribir 0x00  en el registro ADCSRB, se configuran los bits 000 en el registro ADTS (ADC Auto Trigger Source), este 000 corresponde al modo Free Running

    
    // Configuramos el primer canal analogico a leer - Canal 1 (Luz)
    canal_actual_adc = 1;
    ADMUX = (ADMUX & 0xF8) | canal_actual_adc; //borramos los bits correspondientes al canal (conserva los primeros 5) y le agregamos los 3 del canal actual

    // ADSC es el Bit 6 del registro ADCSRA
    //(REFS0 es el Bit 6 pero del registro ADMUX)

    ADCSRA |= (1 << ADSC); // Inicia la primer conversión (dsp el modo Free Running dispara automaticamente las otras conversiones, pero la primera conversion tenemos q iniciarla manualmente). Es el boton de arranque para comenzar el ciclo continuo de conversiones
     // Esta instrucción tiene dos partes:
        // 1 - Registro y Bit
            // ADCSRA: Es el registro de Control y Estado A del ADC
            // ADSC: Es el bit ADC Start Conversion dentro de ese registro
        // 2 - Acción: Iniciar la Conversión(1 << ADSC): Crea una máscara binaria donde solo el bit ADSC está en 1. |= es el operador OR a nivel de bits. Lo que hace es poner en 1 el bit ADSC sin modificar el estado de ningún otro bit en el registro ADCSRA. Al establecer el bit ADSC a 1, le damos la orden al hardware del microcontrolador de Iniciar la Conversión Analogica-Digital
}



//INTERRUPCION TIMER1 (mide tiempo, cada 10 ms lanza la interrupcion para ejecutar logica de Alarma, Advertencia, etc)
// La funcion timer1_init() ejecuta la subrutina establecida en la funcion ISR (dado q se dispara la interrupcion) cada vez que el Timer 1 cuenta 20.000 pulsos de su reloj preescalado
// Usamos el Timer1 para generar una base de tiempo exacta de 10ms (100 Hz)
// Modo CTC (Clear Timer on Compare Match):
    // TCCR1B = (1 << WGM12): El timer cuenta hasta un valor tope y se reinicia
// Prescaler:
        // TCCR1B |= (1 << CS11): Divide el reloj principal por 8 (16MHz / 8 = 2MHz)
// Valor de Comparación:
    // OCR1A = 20000: Con el reloj a 2MHz, contar 20000 pulsos toma 10ms (20.000/2.000.000 = 0,01s)
// Interrupción:
    // TIMSK1 = (1 << OCIE1A): Habilita la interrupción cuando el timer llega a 20.000 pulsos para ello Ejecuta ISR(TIMER1_COMPA_vect)
void timer1_init() {
    TCCR1A = 0x00;
    TCCR1B = (1 << WGM12) | (1 << CS11); // Activa el modo CTC (Clear Timer on Compare Match). El timer cuenta desde 0 hasta el valor de OCR1A. Cuando llega (es pq matcheo el valor del timer 1 "TCNT1" con el valor de OCR1A), el contador del timer vuelve a 0 automáticamente
    OCR1A = 20000; // 16MHz / (8*100) = 20000 (100 Hz -> 10 ms)
    TIMSK1 = (1 << OCIE1A); // Habilita la interrupción, cuando el contador llega a 20000 se lanza la interrupcion y se ejecuta ISR(TIMER1_COMPA_vect) la cual es la subrutina que se ejecuta cada vez que se dispara esta interrupcion

}




// Inicializamos Timer2 para generar el tono del Buzzer (Buzzer Pasivo)
// Configuración: CTC Mode, Prescaler 64
// Frecuencia objetivo: ~1kHz (2kHz toggle)
// 16MHz / 64 = 250kHz -> 250 ticks = 1ms. Queremos 0.5ms (500us) para togglear -> 125 ticks.
void timer2_init() {
    TCCR2A = (1 << WGM21); // CTC Mode
    TCCR2B = (1 << CS22);  // Prescaler 64
    OCR2A = 125;           // Valor de comparación
    TIMSK2 = 0;            // Arranca silenciado (Interrupción apagada)
}



// Interrupción EXTERNA REALIZADA POR BOTON/PULSADOR - por cambio de estado en el pin D8 (PCINT0)
// Esta funcion se usa para detectar el botón en el pin digital 8 (Puerto B, bit 0)
// El registro PCMSK0 tiene 8 bits y controla D8 al D13, el bit 0 de ese registro, corresponde al D8
void pcint_init() {
    PCICR |= (1 << PCIE0); // Habilita el Grupo 0 de interrupciones, que corresponde a los pines D8 hasta D13 (Puerto B)
    PCMSK0 |= (1 << PCINT0); // Dentro de ese grupo 0, habilita específicamente el pin D8. Si el estado del pin 8 cambia (0 a 1 o 1 a 0), se dispara la interrupción y se ejecuta la subrutina ISR(PCINT0_vect)
}



// SETUP (definimos pines de entrada/salida, inicializamos perifericos y habilitamos interrupciones globales)
void setup() {
    // Inicializamos LCD
    lcd.init();
    lcd.backlight();
    lcd.print("Iniciando...");
    delay(1000);
    lcd.clear();

    // Inicializamos sensor de Temp
    dht.begin();

    // Configuración de PINES
    // D4, D5, D6 como SALIDAS (LEDs y Buzzer)
    DDRD |= (1 << DDD4) | (1 << DDD5) | (1 << DDD6);

    // PINES DE ENTRADA
    DDRB &= ~(1 << DDB0); // Pone el bit 0 del Puerto B (Pin digital 8) como Entrada (el del Boton)

    // Activamos las resistencias pull-up internas para sensores analógicos en los pines A1 y A2
    PORTC |= (1 << PORTC1) | (1 << PORTC2);
    
    // Recuperamos el indice de EEPROM para mantener continuidad en el historico de muestras
    indice_eeprom = EEPROM.read(DIR_INDICE_PERSISTENTE);

    if (indice_eeprom >= MAX_MUESTRAS) {
        indice_eeprom = 0;
    }

    // Inicializamos las interrupciones
    adc_init();
    timer1_init();
    timer2_init(); // Inicializamos Timer del Buzzer
    pcint_init();

    sei(); // Habilitamos interrupciones (Hasta que no ejecutamos sei(), las interrupciones estan "muteadas" aunque esten configuradas. Al ejecutar sei(), damos permiso al CPU para que atienda a las interrupciones)
}


// LOOP (Mostramos los valores en la pantalla LCD)
void loop() {
    static uint8_t ultimo_modo = 255; // Para detectar cambio de modo y limpiar pantalla

    // Detectar cambio de modo para limpiar pantalla y CARGAR SNAPSHOT
    if (modo_lcd != ultimo_modo) {
        lcd.clear();
        ultimo_modo = modo_lcd;
        
        // Si entramos al modo Historial, cargamos los valores desde la EEPROM a la RAM
        if (modo_lcd == 1) {
            // historial_ram[0] contiene la muestra mas reciente
            uint8_t ptr_escritura = indice_eeprom; // Donde se escribira la siguiente muestra
            
            for (int i = 0; i < MAX_MUESTRAS; i++) {
                // Formula circular inversa: (Puntero - 1 - i)
                int16_t index_calc = ptr_escritura - 1 - i;
                if (index_calc < 0) index_calc += MAX_MUESTRAS; //para q el indice vuelva a 10
                
                uint16_t addr = EEPROM_DIRECCION_INICIO + (index_calc * BYTES_POR_MUESTRA); // Calculamos la direccion de la muestra
                EEPROM.get(addr, historial_ram[i]); // Leemos la muestra de la EEPROM y la guardamos en la RAM
                    //GET va la direccion "addr" y lo q obtiene de ahi lo mete en "historial_ram[i]", q es un vector de la estructura "Muestra" por lo q almacena la muestra en la struct i
            }
        }

        // Delay para asegurar que el clear se procese visualmente antes de escribir en la LCD
        delay(10); 
    }

    // Gestion de EEPROM
    if (guardar_eeprom) {
        guardar_eeprom = false; // Bajamos la bandera

        // Calculamos dirección usando la constante EEPROM_DIRECCION_INICIO
        uint16_t addr_base = EEPROM_DIRECCION_INICIO + (indice_eeprom * BYTES_POR_MUESTRA);
        
        Muestra datos_actuales; //creamos el objeto "datos_actuales" del tipo struct "Muestra"
        
        // Copia atómica de los valores volatiles y escritura protegida para evitar corrupcion de valores por interrupciones
        uint8_t sreg_old = SREG; // guardamos una copia del estado actual del sistema (incluyendo si las interrupciones estaban prendidas o apagadas). SREG es una buena practica
        cli(); //bloqueamos las interrupciones (nada interrumpe al CPU), para que la copia de los datos que dura varios ciclos sea Atomica (indivisible)
        
        datos_actuales.t = valores_sensores[0];
        datos_actuales.l = valores_sensores[1]; // A1 (Luz) Directo
        datos_actuales.n = valores_sensores[2]; // A2 (Nivel) Directo

        EEPROM.put(addr_base, datos_actuales);
        
        SREG = sreg_old; // Reactivar interrupciones
        

        indice_eeprom++;
        if (indice_eeprom >= MAX_MUESTRAS) {
            indice_eeprom = 0;
        }
        
        // Persistimos el nuevo indice del puntero de escritura (asi el Arduino arranca de ahi)
        EEPROM.write(DIR_INDICE_PERSISTENTE, indice_eeprom);
    }

    // Lectura del sensor de Temp
    unsigned long tiempo_actual = millis();
    if (tiempo_actual - ultimo_tiempo_dht >= INTERVALO_DHT) {
        ultimo_tiempo_dht = tiempo_actual;
        
        // Leemos temperatura (usamos float para verificar q no sea NAN)
        float t = dht.readTemperature();
        
        // Si t no es NAN (puede dar NAN si el sensor no esta funcionando), guardamos el valor de "t" en valores_sensores[0] conviertiendolo a uint16_t primero (ya que valores_sensores es un array de tipo uint16_t)
        if (!isnan(t)) {
            // Deshabilitamos interrupciones momentáneamente para escritura atómica de variable compartida de 16bits
            uint8_t sreg_old = SREG; // Otra vez usamos SREG para guardar el estado de las interrupciones (guardamos q estaban habilitadas)
            cli(); //bloqueamos interrupciones
            valores_sensores[0] = (uint16_t)t;
            SREG = sreg_old; // volvemos al estado anterior (reactivamos interrupciones)
        }
    }

    // Actualizamos Pantalla LCD
    if (modo_lcd == 0) {
        // MODO 0: TIEMPO REAL
        uint16_t temp, luz, nivel; //Declaramos variables locales para almacenar los valores leidos
        
        //Lectura atomica de las variables volatiles
        uint8_t sreg_old = SREG; // guardamos estado de interrupciones
        cli(); //bloqueamos interrupciones
        temp = valores_sensores[0];
        luz  = valores_sensores[1]; // A1 (Luz)
        nivel = valores_sensores[2]; // A2 (Nivel)
        SREG = sreg_old; // volvemos al estado anterior (reactivamos interrupciones)
        
        lcd.setCursor(0, 0); 
        lcd.print("T:");
        lcd.print(temp);
        lcd.print("  L:");
        lcd.print(luz);
        lcd.print("   "); // Limpiamos residuos
        
        lcd.setCursor(0, 1);
        lcd.print("N:");
        lcd.print(nivel);
        lcd.print(" ");
        
        if (estado_sistema == 2) {
            lcd.print("ALERTA!!!");
        } else if (estado_sistema == 1) {
            lcd.print("Aviso");
        } else {
            lcd.print("OK      "); 
        }

    } else {
        // LCD en Modo 1: Historico de muestras
        // Leemos del buffer RAM que cargamos al entrar al modo, lo cual garantiza estabilidad (Snapshot) y velocidad
        
        // historial_ram esta lleno de las 10 structs "Muestra" con cada struct conteniendo sus 3 campos, le decimos dame la struct del historial_ram que contenga el indice_historial (supongamos q indice_historial es 5, me da la struct 5 completa y la guardo en datos_historicos)
        //indice_historial es la posicion del vector historial_ram a la q quiero acceder
        Muestra datos_historicos = historial_ram[indice_historial];

        lcd.setCursor(0, 0);
        lcd.print("R:-");             // R:-X indica "X muestras atras"
        lcd.print(indice_historial);  // 0 a 9
        lcd.print(" T:");
        lcd.print(datos_historicos.t);
        lcd.print("°C");
        
        lcd.setCursor(0, 1);
        lcd.print("L:");
        lcd.print(datos_historicos.l);
        lcd.print(" N:");
        lcd.print(datos_historicos.n);
    }
    
    // Retardo para no saturar la comunicación I2C
    delay(200); 
}



//SUBRUTINAS (se ejecutan tras dispararse su interrupcion correspondiente)

//ISR(ADC_vect) - Muestreo Rotativo: esta rutina se ejecuta automaticamente al ocurrir la interrupcion del ADC (la que toma valores y hace la conversion ADC)
// Hacemos la Rotación del Canal para tomar los valores de los 2 sensores uno a la vez (usando un canal analogico para cada sensor)
// Esta rutina permite que los 2 sensores se actualicen constantemente de fondo/en segundo plano mediante interrupciones ISR sin q el loop haga nada, asi el loop() esta libre para manejar otras tareas sin preocuparse por los tiempos de espera de las lecturas de los sensores

//La interrupción ocurre cada vez que se termina de leer UN SOLO sensor. La configuracion es:
// Frecuencia del Arduino: 16 MHz con Prescaler = 128, entonces la velocidad del ADC es de 125 kHz (16 Mhz / 128). Una conversion toma 13 ciclos de ese tiempo, entonces tiempo de conversion = 13/125 Khz = 104 uS. Por lo tanto la funcion ISR(ADC_vect) se ejecuta cada 104 uS (aprox 9600 veces por s)
// La interrupcion Funciona así:
// Termina conversión sensor 1 -> INTERRUPCIÓN -> Guarda valor 1 -> Cambia multiplexor al 2, pasan 104 µs
// Termina conversión sensor 2 -> INTERRUPCIÓN -> Guardas valor 2 -> Cambias multiplexor al 1, pasan 104 µs
ISR(ADC_vect) {
    //ADC es un registro que almacena el valor leido del sensor analogico
    uint16_t lectura = ADC;
    
    // Guardamos el valor leido del sensor analogico, en el array "valores_sensores" en su indice correspondiente
    valores_sensores[canal_actual_adc] = lectura;
    
    // Alternar entre Canal 1 (Luz) y Canal 2 (Nivel), el canal 0 de Temp esta en el loop. Arduino tiene 1 solo ADC por lo q para leer mas de un sensor, hay q usar canales distintos conectados al ADC
    if (canal_actual_adc == 1) {
        canal_actual_adc = 2; // Cambia a Canal 2
    } else {
        canal_actual_adc = 1; // Cambia a Canal 1
    }
    
    // Hacemos el cambio de canal en el ADC (& 0xF8: desconecta el cable anterior. | canal: conecta el cable nuevo)
    ADMUX = (ADMUX & 0xF8) | canal_actual_adc;
}

//ISR TIMER1: Logica de Alarma, Buzzer Temporizado y el antirebote del boton
ISR(TIMER1_COMPA_vect) {
    static uint16_t contador_3s = 0; //para contar hasta 3000 ms
    static uint8_t contador_parpadeo = 0;
    
    // Decrementar contador de debounce (antirebote) si está activo
    if (contador_debounce > 0) {
        contador_debounce--;
    }

    uint16_t temp = valores_sensores[0]; // D7 (Temp)
    uint16_t luz  = valores_sensores[1]; // A1 (Luz)
    uint16_t nivel = valores_sensores[2]; // A2 (Nivel)
    
    // 0: Normal, 1: Advertencia, 2: Alerta
    uint8_t estado = 0;
    
    // Chequeo de Advertencias
    if (temp > TEMP_ADVERTENCIA) estado = 1;
    if (luz > LUZ_ADVERTENCIA) estado = 1;
    if (nivel > NIVEL_ADVERTENCIA) estado = 1;

    // Chequeo de Alertas (sobrescribe advertencia)
    if (temp > TEMP_ALERTA) estado = 2;
    if (luz > LUZ_ALERTA) estado = 2;
    if (nivel > NIVEL_ALERTA) estado = 2;

    estado_sistema = estado; // Actualizamos variable global "estado_sistema" para el LCD
    alarma_activa = (estado == 2); // si estado == 2 es true, alarma_activa = True, sino False
    
    // LOGICA DE BUZZER CON LATCH (Memoria de 15 segundos). Si se dispara la alarma (estado 2) y no estaba contando, inicia la cuenta
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
        
        // Reset del Latch: Solo si la alarma SE FUE (estado != 2) y ya habíamos terminado (1501) permitimos volver a 0 para que una futura alarma vuelva a sonar.
        if (estado != 2 && contador_tiempo_buzzer == 1501) {
            contador_tiempo_buzzer = 0;
        }
    }
    
// CONTROL DE HARDWARE (LEDs y Buzzer)
//PORTD es el "Tablero de Interruptores" de los pines digitales 0 al 7. Escribir un 1 aca es poner ese pin en HIGH, escribir un 0 significa ponerlo en LOW
// Si el contador es menor a 25 (0-250ms) -> PRENDO. Si es mayor (250-500ms) -> APAGO". Asi parpadea el LED rojo
// 1 << PORTD4 (LED Verde en D4) PORTD4 es el numero 4 entonces es 1 << 4 lo cual crea la mascara 00010000 y al hacer la asignacion |=  "OR" estamos comparando el PORTD con la mascara q creamos (esto para no borrar el valor que ya estaba en PORTD, si hicieramos PORTD = (1 << 4) perderiamos el valor de los demas pines digitales que ya estaban en el registro PORTD, al hacer el OR solo modificamos el valor del pin 4 (Verde) y mantenemos los demas valores iguales ya q compara los bits con el OR
// PORTD &= ~(1 << PORTD5) PORTD5 es el numero 5 entonces queda PORTD &= ~(1 << 5) lo cual genera la mascara 00100000 al correr el bit "1" 5 posiciones. "~" da vuelta la mascara 00100000 a 11011111, luego al hacer la asignacion & "AND" entre esta mascara y el PORTD, obtenemos el mismo PORTD pero con el bit 5 en "0" (lo cual corresponde al PORTD5 q es el del LED Rojo y asi lo ponemos en LOW)  
    contador_parpadeo++;
    if (contador_parpadeo >= 50) contador_parpadeo = 0; 

    // Apagar todo por defecto para setear segun estado
    // Buzzer se controla por la variable 'buzzer_encendido' derivada del latch
    // Buzzer se controla habilitando/deshabilitando la interrupcion del Timer2 (que genera el tono)
    if (buzzer_encendido) {
        // Habilitamos la interrupcion del Timer2 (suena)
        TIMSK2 |= (1 << OCIE2A);
    } else {
        // Deshabilitamos la interrupcion del Timer2 (silencio) y aseguramos pin LOW
        TIMSK2 &= ~(1 << OCIE2A);
        PORTD &= ~(1 << PORTD6);
    }

    switch (estado) {
        case 0: // NORMAL
            PORTD |= (1 << PORTD4);  // Verde encendido
            PORTD &= ~(1 << PORTD5); // Rojo apagado
            break;
            
        case 1: // ADVERTENCIA
            PORTD |= (1 << PORTD4);  // Verde encendido
            // Rojo parpadea
            if (contador_parpadeo < 25) {
                PORTD |= (1 << PORTD5);
            } else {
                PORTD &= ~(1 << PORTD5);
            }
            break;
            
        case 2: // ALERTA
            PORTD |= (1 << PORTD4); // Verde encendido
            PORTD |= (1 << PORTD5);  // Rojo encendido
            break;
    }
    
    // LOGICA DE GUARDADO EN EEPROM cada 30 segundos - Cada 3000 ticks/llamadas (10ms * 3000 = 30s)
    contador_3s++;
    if (contador_3s >= 3000) {
        contador_3s = 0;
        guardar_eeprom = true;
    }
}



// ISR TIMER2: Generación de onda cuadrada para Buzzer Pasivo
// Se ejecuta cada 0.5ms aprox para invertir el estado del pin
ISR(TIMER2_COMPA_vect) {
    PORTD ^= (1 << PORTD6); // Toggle (Invertir) pin D6
}


// ISR PCINT0 - Boton (D8): cuando apretamos el boton el modo LCD pasa de 0 a 1 donde muestra los valores historicos de las muestras (cada vez q apretamos el boton nos movemos 1 registro desde el 1 al 10 y cuando llegamos a ese, al apretar el boton otra vez se cambia el modo LCD a 0 y se vuelve al modo Tiempo Real)
// La lectura del primer registro en el modo historico comienza mostrando la ultima muestra guardada (la de hace 4 segundos. Esto es porque inicializamos "indice_eeprom" para apuntar un lugar antes de donde se guardara la siguiente muestra
//PCINT0_vect significa Pin Change Interrupt 0 Vector, detecta el cambio de Voltaje en e pin y ahi ejecuta la alarma/evento. Es el "ID" del evento (le dice al Arduino, ejecuta esta funcion cuando se dispare la alarma del Grupo 0 (algun cambio de Voltaje en pines D8-D13)
// El fabricante del chip reservo una dirección fija en memoria (dirección 0x0004) llamada "Vector PCINT0". Cuando el hardware detecta el evento (el cambio de V en pines D8-D13), salta a la dirección 0x0004. La palabra ISR(PCINT0_vect) es una orden al compilador de poner el codigo de esta función exactamente en la dirección 0x0004
ISR(PCINT0_vect) {
    // Verificar si el antirebote permite una nueva pulsacion del boton
    if (contador_debounce == 0) {
        
        // Ponemos tiempo de bloqueo para antirebote 200 ms (los 20 de TIEMPO_DEBOUNCE q van bajando de a 1 por cada vez que se ejecuta ISR(TIMER1), la cual se ejecuta cada 10ms)
        contador_debounce = TIEMPO_DEBOUNCE;
        
        // Logica de navegacion de la LCD
        if (modo_lcd == 0) {
            modo_lcd = 1; // Ir a historico
            indice_snapshot = indice_eeprom; // CONGELAR el estado actual para la visualizacion
            indice_historial = 0; // Mostrar la muestra mas reciente (0 pasos atras)
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