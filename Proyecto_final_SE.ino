// Codigo completo (con interrupciones, registros I2C y valores de los 3 sensores guardados en EEPROM)
#include <LiquidCrystal_I2C.h> 
#include <EEPROM.h>

// Definiciones de la LCD I2C
LiquidCrystal_I2C lcd(0x27, 16, 2); 

// GESTIÓN DE EEPROM Y MODOS DE VISUALIZACIÓN
// Cada dato uint16_t ocupa 2 bytes. 3 sensores * 2 bytes por sensor = 6 bytes por muestra
const uint8_t MAX_SAMPLES = 10;
const uint8_t BYTES_PER_SAMPLE = 6; 
const uint16_t EEPROM_START_ADDR = 0; 

//LCD = 0 indica datos en Tiempo Real, LCD = 1 indica acceso a las muestras anteriores almacenada en EEPROM
volatile uint8_t lcd_mode = 0; 
volatile uint8_t history_index = 0; // Índice de la muestra histórica a mostrar (0-9)
volatile uint8_t eeprom_address_index = 0; // Índice de la última muestra guardada (0-9)

// Variables de interrupciones
volatile uint16_t adc_values[3] = {0,0,0}; //Es un array para guardar las lecturas de los 3 sensores, es volatile porque se llena dentro de una interrupción y se lee en el loop
volatile uint8_t current_channel = 0; //Lleva la cuenta de qué sensor se está leyendo (0, 1 o 2)
volatile bool alarma_activa = false;
volatile bool silencio = false;


// FUNCIONES A NIVEL DE REGISTROS

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
    // Inicialización LCD I2C (Alto Nivel)
    lcd.init();
    lcd.backlight();
    lcd.print("Monitoreo Amb.");

    // PINES DE SALIDA
    // (Data Direction Register) configura si un pin es entrada (0) o salida (1). Aca ponemos los bits 4, 5 y 6 del Puerto D como Salidas (para LEDs y Buzzer)
    DDRD |= (1 << DDD4) | (1 << DDD5) | (1 << DDD6);

    // PINES DE ENTRADA
    DDRB &= ~(1 << DDB0); // Pone el bit 0 del Puerto B (Pin 8) como Entrada (el del Boton/Pulsador)

    adc_init();
    timer1_init();
    pcint_init();

    sei(); // Habilitar interrupciones globales
}


// LOOP (Muestra los valores en la pantalla LCD)
void loop() {
    
    // Muestra la actividad mínima (apagar el buzzer si silencio es true)
    if (silencio) {
        PORTD &= ~(1 << PORTD6);
    }

    if (lcd_mode == 0) {
        // LCD en Modo 0: Valores de sensores en tiempo real
        uint16_t temp = adc_values[0];
        uint16_t luz  = adc_values[1];
        uint16_t nivel = adc_values[2];
        
        lcd.setCursor(0, 0); 
        lcd.print("T: ");
        lcd.print(temp);
        lcd.print("   L: ");
        lcd.print(luz);
        
        lcd.setCursor(0, 1);
        lcd.print("N: ");
        lcd.print(nivel);
        lcd.print(" ");
        
        if (silencio) {
            lcd.print("SILENCIO");
        } else if (alarma_activa) {
            lcd.print("ALERTA! ");
        } else {
            lcd.print("OK      "); 
        }
    } else {
        // LCD en Modo 1: Historico de muestras (Leemos los valores de la EEPROM con EEPROM.get())
        

        // Calculamos la dirección base de la EEPROM (para determinar la dirección de memoria exacta en la EEPROM donde se deben guardar/leer los datos)
        // Permite el acceso secuencial a registros de datos que tienen un tamaño fijo dentro de la EEPROM. El valor de base_addr es la dirección de la primera celda de memoria de la EEPROM donde se almacena el conjunto de datos correspondiente a ese índice histórico
        uint16_t base_addr = EEPROM_START_ADDR + (history_index * BYTES_PER_SAMPLE);



        // Estructura para leer 6 bytes juntos (la muestra entera con los valores de los 3 sensores)
        // Creamos la estructura "Muestra" con los campos t (temperatura), l (luz) y n (nivel), luego creamos la variable "h_data" de tipo "Muestra"
        struct Muestra {
            uint16_t t;
            uint16_t l;
            uint16_t n;
        } h_data;


        // Usando el metodo GET, Leemos la muestra completa de 6 bytes (t, l, y n) de la EEPROM a partir de la direccion "base_addr" y la cargamos en el objeto "h_data" (h_data es una instancia de la struct "Muestra")
        EEPROM.get(base_addr, h_data);
        
        lcd.setCursor(0, 0);
        lcd.print("REG: ");
        lcd.print(history_index + 1); // Muestras 1-10 (sumamos 1 pq el indice arranca en 0)
        lcd.print("/10 T: ");
        lcd.print(h_data.t); // Muestra la temperatura (el campo t de la estructura h_data)
        
        lcd.setCursor(0, 1);
        lcd.print("L: ");
        lcd.print(h_data.l); // Muestra la luz (el campo l de la estructura h_data)
        lcd.print(" N: ");
        lcd.print(h_data.n); // Muestra el nivel (el campo n de la estructura h_data)
    }
    
    // Retardo para no saturar la comunicación I2C (POSIBLEMENTE CAMBIAR A FUNCION MILLIS)
    delay(200); 
}


// Rutina = lo q se ejecuta cuando se da una interrupcion
// La rutina es el codigo de una funcion q se ejecuta cuando sucede una interrupcion, en este caso ISR(ADC_vect) es la funcion q se ejecuta cuando se da la interrupcion del ADC

//ISR(ADC_vect) - Muestreo Rotativo: esta rutina se ejecuta automaticamente al finalizar la interrupcion del ADC (la que toma valores y hace la conversion ADC)
// ISR(ADC_vect) hace la Rotación del Canal para tomar los valores de los 3 sensores uno a la vez (usando un canal analogico para cada sensor)
// Esta rutina permite que los 3 sensores se actualicen constantemente de fondo/en segundo plano mediante interrupciones ISR sin q el loop haga nada, asi el loop() principal esta libre para manejar otras tareas sin preocuparse por los tiempos de espera de las lecturas de los sensores
ISR(ADC_vect) {

    // "Lectura": lee el valor del registro ADC y lo guarda en el array "adc_values"
    uint16_t lectura = ADC;
    adc_values[current_channel] = lectura;
    
    // Avanzar al siguiente canal
    current_channel++;
    if (current_channel >= 3) current_channel = 0;
    
    ADMUX = (1 << REFS0) | current_channel;
    // Reconfiguración: escribe en ADMUX el nuevo canal (como el ADC está en modo Free-Running, la siguiente conversion que inicie automaticamente, usará este nuevo canal)
}

// ISR TIMER1 (Lógica de Alarma y Guardado Periódico)
ISR(TIMER1_COMPA_vect) {
    static uint8_t counter_1s = 0;
    
    // Logica de Alarma
    uint16_t temp = adc_values[0];
    uint16_t luz  = adc_values[1];
    uint16_t nivel = adc_values[2];
    bool alerta = false;

    // Logica de alarmas y control de LEDs/Buzzer con registros. Esta lógica establece 'alerta'

    alarma_activa = alerta; 
    // Buzzer control (manteniendo el uso de registros):
    if (alerta && !silencio) {
        PORTD |= (1 << PORTD6); 
    } else {
        PORTD &= ~(1 << PORTD6);
    }
    
    // Logica de guardado cada 1 segundo (usando EEPROM.put())
    counter_1s++;
    if (counter_1s >= 100) {
        counter_1s = 0;
        
        uint16_t base_addr = EEPROM_START_ADDR + (eeprom_address_index * BYTES_PER_SAMPLE);
        
        // Estructura temporal para guardar
        struct Sample {
            uint16_t t;
            uint16_t l;
            uint16_t n;
        } current_data;
        
        current_data.t = adc_values[0];
        current_data.l = adc_values[1];
        current_data.n = adc_values[2];
        
        // Guardado de las muestras con el meotodo put de la librería EEPROM
        EEPROM.put(base_addr, current_data);

        // Mover al siguiente índice circularmente
        eeprom_address_index++;
        if (eeprom_address_index >= MAX_SAMPLES) {
            eeprom_address_index = 0;
        }
    }
}

// ISR PCINT0 - Boton (D8)
ISR(PCINT0_vect) {
    // 1 - Poner en silencio
    silencio = true; 

    // 2 - Cambiar Modo LCD y navegar en el Historico de muestras
    if (lcd_mode == 0) {
        lcd_mode = 1; // Cambia a modo Historico de muestras
        // Inicializa para mostrar el ultimo dato guardado
        history_index = (eeprom_address_index == 0) ? MAX_SAMPLES - 1 : eeprom_address_index - 1; 
    } else {
        // En modo histórico, avanza al registro anterior (navegación)
        if (history_index == 0) {
            history_index = MAX_SAMPLES - 1;
        } else {
            history_index--;
        }
    }
}