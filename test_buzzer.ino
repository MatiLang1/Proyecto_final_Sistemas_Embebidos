void setup() {
    // Configuramos PIN 6 como SALIDA
    // (Equivalente a DDRD |= (1 << DDD6))
    pinMode(6, OUTPUT);
}

void loop() {
    // ------ INTENTO 1: TRATARLO COMO BUZZER PASIVO ------
    // Un Buzzer Pasivo es como un parlante: Necesita una ONDA (Prender/Apagar rapido) para sonar.
    // Si mandamos solo HIGH constante, hace "Click" y se calla.
    // Generamos una onda cuadrada de 1kHz (aprox) manualmente:
    
    // Suena por 5 segundos
    for(long i = 0; i < 2500; i++) { 
        digitalWrite(6, HIGH);       // Empuja el diafragma
        delayMicroseconds(200);      // Espera
        digitalWrite(6, LOW);        // Suelta el diafragma
        delayMicroseconds(200);      // Espera
    }
    
    delay(2000); // Silencio de 2 segundos...

    // ------ INTENTO 2: TRATARLO COMO BUZZER ACTIVO ------
    // Un Buzzer Activo tiene oscilador interno. Solo necesita 5V fijos.
    // Si tu buzzer es Activo, aqui sonara un pito limpio y fuerte.
    // Si es Pasivo, solo hará un "Click" y silencio (o un zumbido muy leve).
    
    digitalWrite(6, HIGH); // 5V Constantes
    delay(5000);           // Esperar 5 seg
    digitalWrite(6, LOW);  // Apagar
    
    delay(2000); // Silencio de 2 segundos...
}
