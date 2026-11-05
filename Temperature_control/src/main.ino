const int lm35Pin = 
volatile float temperatura = 0.0;
volatile int lecturaADC = 0;

const int resistencia = 

const int numReadings = 3;
int readings[numReadings];      // Lecturas del interruptor analógico
int readIndex = 0;              // Índice de la lectura actual
int total = 0;                  // Total de la suma
int media = 0; 


void lectura(){
  float tiempo = millis();
  float actual = 0;
  while (actual - tiempo < 10000){
    // Lee el valor del conversor analógico-digital (0 a 1023)
    lecturaADC = analogRead(lm35Pin);

    // Convierte la lectura a voltaje (5.0V / 1024) y luego a grados Celsius (10 mV por ºC -> * 100)
    temperatura = (lecturaADC * 5.0 * 100.0) / 1024.0;

    total = total - readings[readIndex];
    readings[readIndex] = temperatura;
    total = total + readings[readIndex];
    readIndex = readIndex + 1;

    if (readIndex >= numReadings) {
      readIndex = 0;
    }

    media = total / numReadings;

    Serial.println(media);

    if (media > 80.0){
      digitalWrite(resistencia, LOW);
    } else {
      digitalWrite(resistencia, HIGH);
    }

    delay(40);

    tiempo = millis();
  } 
}


void setup() {
  Serial.begin(115200); // Inicializa el monitor serie
  Serial.setTimeout(10);

  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }
}

void loop() {
  while (Serial.available() > 0){
    String c = Serial.readStringUntil('\n');
    c.trim();

    if (c = "A"){
      lectura();
    }

  }
  delay(50);
}

