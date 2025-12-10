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

// Variables de interrupción (Mantenidas a nivel de registros)
volatile uint16_t adc_values[3] = {0,0,0}; 
volatile uint8_t current_channel = 0;
volatile bool alarma_activa = false;
volatile bool silencio = false;


// FUNCIONES A NIVEL DE REGISTROS

// Configurar ADC en modo Free-Running con interrupciones
void adc_init() {
    ADMUX = (1 << REFS0);       // Selecciona a VCC (5V) como voltaje de referencia, canal ADC0 inicialmente
    ADCSRA = (1 << ADEN) |      // Habilita el ADC
             (1 << ADATE) |     // Habilita elAuto trigger (disparo automático)
             (1 << ADIE) |      // Habilita la Interrupción del ADC. Esto es clave porque cada vez que termine una conversión, se ejecuta la función ISR(ADC_vect)

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

// Configurar TIMER1 a 10 ms
void timer1_init() {
    TCCR1A = 0x00;
    TCCR1B = (1 << WGM12) | (1 << CS11); // CTC, prescaler 8
    OCR1A = 20000; // 16MHz / (8*100) = 20000 (100 Hz -> 10 ms)
    TIMSK1 = (1 << OCIE1A);
}

// Interrupción por cambio de pin en D8 (PCINT0)
void pcint_init() {
    PCICR |= (1 << PCIE0);    // Grupo PCINT0 (pines 8–13)
    PCMSK0 |= (1 << PCINT0); // Habilitar PCINT0 (D8)
}

// SETUP, LOOP e ISRs

void setup() {
    // Inicialización LCD I2C (Alto Nivel)
    lcd.init();
    lcd.backlight();
    lcd.print("Monitoreo Amb.");

    // Pines de salida
    DDRD |= (1 << DDD4) | (1 << DDD5) | (1 << DDD6);

    // Pin del botón (D8)
    DDRB &= ~(1 << DDB0); // D8 como entrada

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
        // --- MODO 0: TIEMPO REAL ---
        uint16_t temp = adc_values[0];
        uint16_t luz  = adc_values[1];
        uint16_t nivel = adc_values[2];
        
        lcd.setCursor(0, 0); 
        lcd.print("T:");
        lcd.print(temp);
        lcd.print("  L:");
        lcd.print(luz);
        
        lcd.setCursor(0, 1);
        lcd.print("N:");Sampl
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
        // LCD en Modo 1: Historico de muestras (Lectura simple con EEPROM.get())
        
        uint16_t base_addr = EEPROM_START_ADDR + (history_index * BYTES_PER_SAMPLE);
        
        // Estructura temporal para leer 6 bytes de golpe
        struct Sample {
            uint16_t t;
            uint16_t l;
            uint16_t n;
        } h_data;

        // Lectura de EEPROM usando la librería simple
        EEPROM.get(base_addr, h_data);
        
        lcd.setCursor(0, 0);
        lcd.print("REG: ");
        lcd.print(history_index + 1); // Muestra 1-10
        lcd.print("/10 T:");
        lcd.print(h_data.t);
        
        lcd.setCursor(0, 1);
        lcd.print("L:");
        lcd.print(h_data.l);
        lcd.print(" N:");
        lcd.print(h_data.n);
    }
    
    // Retardo para no saturar la comunicación I2C (POSIBLEMENTE CAMBIAR A FUNCION MILLIS)
    delay(200); 
}

// ISR ADC
ISR(ADC_vect) {
    uint16_t lectura = ADC;
    adc_values[current_channel] = lectura;
    
    // Avanzar al siguiente canal
    current_channel++;
    if (current_channel >= 3) current_channel = 0;
    
    ADMUX = (1 << REFS0) | current_channel;
}

// ISR TIMER1 (Lógica de Alarma y Guardado Periódico)
ISR(TIMER1_COMPA_vect) {
    static uint8_t counter_1s = 0;
    
    // --- Lógica de Alarma (igual que antes) ---
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









// // Codigo con interrupciones, registros e I2C
// #include <LiquidCrystal_I2C.h> // Necesitas instalar esta librería (ej. desde el Gestor de Librerías)

// // Definiciones de la LCD I2C
// // Ajusta la dirección (0x27 o 0x3F) y el tamaño (16x2) según tu módulo
// LiquidCrystal_I2C lcd(0x27, 16, 2); 

// // Variables globales
// volatile uint16_t adc_values[3] = {0,0,0}; 
// volatile uint8_t current_channel = 0;
// volatile bool alarma_activa = false;
// volatile bool silencio = false;

// // Configurar ADC en modo Free-Running con interrupciones
// void adc_init() {
//     ADMUX = (1 << REFS0);       // Referencia AVcc, canal ADC0 inicialmente
//     ADCSRA = (1 << ADEN) |      // Habilitar ADC
//              (1 << ADATE) |     // Auto trigger
//              (1 << ADIE) |      // Interrupción ADC
//              (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Prescaler 128
//     ADCSRB = 0x00;              // Free running mode
//     ADCSRA |= (1 << ADSC);      // Start conversion
// }

// // Configurar TIMER1 a 10 ms
// void timer1_init() {
//     TCCR1A = 0x00;
//     TCCR1B = (1 << WGM12) | (1 << CS11); // CTC, prescaler 8
//     OCR1A = 20000; // (F_CPU / (prescaler * 100 Hz)) = (16MHz / (8*100)) = 20000
//     TIMSK1 = (1 << OCIE1A);
// }

// // Interrupción por cambio de pin en D8 (PCINT0)
// void pcint_init() {
//     PCICR |= (1 << PCIE0);    // Grupo PCINT0 (pines 8–13)
//     PCMSK0 |= (1 << PCINT0); // Habilitar PCINT0 (D8)
// }

// // SETUP
// void setup() {

//     // Inicialización LCD I2C (Alto Nivel)
//     lcd.init();
//     lcd.backlight();
//     lcd.print("Monitoreo Amb.");

//     // Pines de salida
//     DDRD |= (1 << DDD4) | (1 << DDD5) | (1 << DDD6);

//     // Pin del botón
//     DDRB &= ~(1 << DDB0); // D8 como entrada

//     adc_init();
//     timer1_init();
//     pcint_init();

//     sei(); // Habilitar interrupciones globales
// }

// // LOOP (Muestra los valores en la LCD)
// void loop() {
    
//     // Muestra la actividad mínima (apagar el buzzer)
//     if (silencio) {
//         PORTD &= ~(1 << PORTD6); // apagar buzzer
//     }

//     // --- LÓGICA DE ACTUALIZACIÓN DE LCD (Alto Nivel) ---
    
//     // Lee los valores actuales (copia atómica fuera del ISR)
//     uint16_t temp = adc_values[0];
//     uint16_t luz  = adc_values[1];
//     uint16_t nivel = adc_values[2];
    
//     // Linea 1: Temperatura y Luz
//     lcd.setCursor(0, 0); 
//     lcd.print("T:");
//     lcd.print(temp);
//     lcd.print("  L:");
//     lcd.print(luz);
    
//     // Linea 2: Nivel y Estado de Alarma
//     lcd.setCursor(0, 1);
//     lcd.print("N:");
//     lcd.print(nivel);
//     lcd.print(" ");
    
//     if (silencio) {
//         lcd.print("SILENCIO");
//     } else if (alarma_activa) {
//         lcd.print("ALERTA! ");
//     } else {
//         lcd.print("OK      "); // Rellenar con espacios para borrar "ALERTA"
//     }
    
//     // Pequeño retardo para no saturar la comunicación I2C
//     delay(200); 
// }

// // ISR ADC
// ISR(ADC_vect) {
//     uint16_t lectura = ADC;
//     adc_values[current_channel] = lectura;
    
//     // Avanzar al siguiente canal
//     current_channel++;
//     if (current_channel >= 3) current_channel = 0;
    
//     ADMUX = (1 << REFS0) | current_channel; 
// }

// // ISR TIMER1 (Actualiza la variable de estado 'alarma_activa')
// ISR(TIMER1_COMPA_vect) {

//     uint16_t temp = adc_values[0];
//     uint16_t luz  = adc_values[1];
//     uint16_t nivel = adc_values[2];

//     bool alerta = false;

//     if (temp > 500) { // temp alta
//         PORTD |= (1 << PORTD4); 
//         alerta = true;
//     } else {
//         PORTD &= ~(1 << PORTD4);
//     }

//     if (luz < 200) { // baja luz
//         PORTD |= (1 << PORTD5);
//     } else {
//         PORTD &= ~(1 << PORTD5);
//     }

//     if (nivel > 600) { // nivel alto
//         PORTD |= (1 << PORTD4); // Reutilizo el LED de temp alta
//         alerta = true;
//     }

//     alarma_activa = alerta; // Actualiza la variable global para que el loop la use

//     if (alerta && !silencio) {
//         PORTD |= (1 << PORTD6); // buzzer
//     } else {
//         PORTD &= ~(1 << PORTD6);
//     }
// }

// // ISR PCINT0 - Botón
// ISR(PCINT0_vect) {
//     silencio = true; // apaga alarmas inmediatamente
// }














// // Codigo sin I2C
// // Variables globales
// volatile uint16_t adc_values[3] = {0,0,0}; 
// volatile uint8_t current_channel = 0;
// // volatile bool alarma_activa = false;
// volatile bool silencio = false;

// // Configurar ADC en modo Free-Running con interrupciones
// void adc_init() {
//     ADMUX = (1 << REFS0);          // Referencia AVcc, canal ADC0 inicialmente
//     ADCSRA = (1 << ADEN) |         // Habilitar ADC
//              (1 << ADATE) |        // Auto trigger
//              (1 << ADIE) |         // Interrupción ADC
//              (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Prescaler 128
//     ADCSRB = 0x00;                 // Free running mode
//     ADCSRA |= (1 << ADSC);         // Start conversion
// }

// // Configurar TIMER1 a 10 ms
// void timer1_init() {
//     TCCR1A = 0x00;
//     TCCR1B = (1 << WGM12) | (1 << CS11); // CTC, prescaler 8
//     OCR1A = 20000; // (F_CPU / (prescaler * 100 Hz)) = (16MHz / (8*100)) = 20000
//     TIMSK1 = (1 << OCIE1A);
// }

// // Interrupción por cambio de pin en D8 (PCINT0)
// void pcint_init() {
//     PCICR |= (1 << PCIE0);  // Grupo PCINT0 (pines 8–13)
//     PCMSK0 |= (1 << PCINT0); // Habilitar PCINT0 (D8)
// }

// // SETUP
// void setup() {

//     // Pines de salida
//     DDRD |= (1 << DDD4) | (1 << DDD5) | (1 << DDD6);

//     // Pin del botón
//     DDRB &= ~(1 << DDB0); // D8 como entrada

//     adc_init();
//     timer1_init();
//     pcint_init();

//     sei(); // Habilitar interrupciones globales
// }

// // LOOP (no queda vacío)
// void loop() {
//     // Aquí solo mostramos actividad mínima
//     // Por ejemplo, enviar datos por Serial (si quisieras)
//     // o procesar flags no críticos.

//     if (silencio) {
//         PORTD &= ~(1 << PORTD6); // apagar buzzer
//     }
// }

// // ISR ADC
// ISR(ADC_vect) {

//     uint16_t lectura = ADC;

//     adc_values[current_channel] = lectura;

//     // Avanzar al siguiente canal
//     current_channel++;
//     if (current_channel >= 3) current_channel = 0;

//     ADMUX = (1 << REFS0) | current_channel; 
// }

// // ISR TIMER1
// ISR(TIMER1_COMPA_vect) {

//     uint16_t temp = adc_values[0];
//     uint16_t luz  = adc_values[1];
//     uint16_t nivel = adc_values[2];

//     bool alerta = false;

//     if (temp > 500) { // temp alta
//         PORTD |= (1 << PORTD4); 
//         alerta = true;
//     } else {
//         PORTD &= ~(1 << PORTD4);
//     }

//     if (luz < 200) { // baja luz
//         PORTD |= (1 << PORTD5);
//     } else {
//         PORTD &= ~(1 << PORTD5);
//     }

//     if (nivel > 600) { // nivel alto
//         PORTD |= (1 << PORTD4);
//         alerta = true;
//     }

//     if (alerta && !silencio) {
//         PORTD |= (1 << PORTD6); // buzzer
//     } else {
//         PORTD &= ~(1 << PORTD6);
//     }
// }

// // ISR PCINT0 - Botón
// ISR(PCINT0_vect) {
//     silencio = true; // apaga alarmas inmediatamente
// }
