#include <MeanFilterLib.h>


const int corrientePin = 5;
const int encoderPinA = 3;
const int encoderPinB = 2;
const int motorPin = 4;

const int PULSOS_POR_VUELTA = 600; // Cambia según tu encoder
volatile long contadorPulsos = 0;
unsigned long tiempoAnterior = 0;
float velocidadRPM = 0;
float velRadianes = 0;

const int ADC_OFFSET = 1921; // Tu offset de cero Amperios
const float SENSITIVIDAD = 0.0347; // TU sensibilidad calculada experimentalmente (V/A)

const float VOLTAJE_REFERENCIA = 3.3;
const int RESOLUCION_ADC = 4095;


MeanFilter<float> filtro(3);

void IRAM_ATTR encoderISR() {
  int estadoB = digitalRead(encoderPinB);
  if (estadoB == HIGH) {
    contadorPulsos++;
  } else {
    contadorPulsos--;
  }
}

float leerCorrienteInstantanea() {
    int valorADC = analogRead(corrientePin);
    float voltaje = ((float)valorADC - ADC_OFFSET) * VOLTAJE_REFERENCIA / RESOLUCION_ADC;
    return voltaje / SENSITIVIDAD;
}

void setup(){
    Serial.begin(115200);

    pinMode(encoderPinA, INPUT_PULLUP);
    pinMode(encoderPinB, INPUT_PULLUP);
    pinMode(corrientePin, INPUT);
    pinMode(motorPin, OUTPUT);

    attachInterrupt(digitalPinToInterrupt(encoderPinA), encoderISR, RISING);

}



void loop(){
    unsigned long tActual = millis();

    float corrienteActual = leerCorrienteInstantanea();

    if (tActual - tiempoAnterior >= 100) {
        noInterrupts();
        long pulsos = contadorPulsos;
        contadorPulsos = 0;
        interrupts();

        // Velocidad en RPM
        velocidadRPM = (pulsos * (60000.0 / (tActual - tiempoAnterior))) / PULSOS_POR_VUELTA;
        velRadianes = (velocidadRPM * 6.2832) / 60;

        tiempoAnterior = tActual;
    }

    float iFiltrada = filtro.AddValue(corrienteActual);

    Serial.print(iFiltrada);
    Serial.print(",");
    Serial.println(velRadianes);

}

//Calibración de sensor de corriente
//https://github.com/KevinAntezana/ESP32-ACS712