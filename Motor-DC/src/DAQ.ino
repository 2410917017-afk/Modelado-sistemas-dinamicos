/*
 * Código para adquisición de datos (DAQ) de un motor DC.
 * Mide corriente y velocidad usando un sensor INA219 y encoder incremental.
 * Envía datos filtrados por serial para análisis en Python.
 */

#include <MeanFilterLib.h>  // Librería para filtrado de media móvil
#include "Wire.h"
#include "Adafruit_INA219.h"

Adafruit_INA219 ina219;

// Definición de pines
const int corrientePin = 5;  // Pin analógico para el sensor de corriente ACS712
const int encoderPinA = 4;   // Pin digital A del encoder incremental
const int encoderPinB = 2;   // Pin digital B del encoder incremental
const int motorPin = 4;      // Pin de salida para controlar el motor (PWM o señal)

// Constantes del encoder
const int PULSOS_POR_VUELTA = 600;  // Número de pulsos por vuelta del encoder (ajustar según modelo)

// Variables para el encoder y velocidad
volatile long contadorPulsos = 0;  // Contador de pulsos (volatile para interrupciones)
unsigned long tiempoAnterior = 0;  // Tiempo del último cálculo de velocidad (ms)
float velocidadRPM = 0;            // Velocidad en revoluciones por minuto
float velRadianes = 0;             // Velocidad en radianes por segundo


// Filtro de media móvil para la corriente
MeanFilter<float> filtro(3);  // Filtro con ventana de 3 muestras

// Interrupción para el encoder: se activa en flanco ascendente de pin A
// Determina dirección basada en el estado de pin B
void IRAM_ATTR encoderISR() {
  int estadoB = digitalRead(encoderPinB);
  if (estadoB == HIGH) {
    contadorPulsos++;  // Dirección positiva
  } else {
    contadorPulsos--;  // Dirección negativa
  }
}


void setup(){
    Serial.begin(115200);  // Iniciar comunicación serial a 115200 baudios

    if (! ina219.begin()) {
        Serial.println("Failed to find INA219 chip");  
    }

    // Configurar pines
    pinMode(encoderPinA, INPUT_PULLUP);  // Pin A del encoder como entrada con pull-up
    pinMode(encoderPinB, INPUT_PULLUP);  // Pin B del encoder como entrada con pull-up
    pinMode(corrientePin, INPUT);        // Pin de corriente como entrada analógica
    pinMode(motorPin, OUTPUT);           // Pin del motor como salida

    // Configurar interrupción para el encoder
    attachInterrupt(digitalPinToInterrupt(encoderPinA), encoderISR, RISING);
}



void loop(){
    unsigned long tActual = millis();  // Obtener tiempo actual en ms
    float current_mA = 0;


    // Calcular velocidad cada 100 ms
    if (tActual - tiempoAnterior >= 100) {
        noInterrupts();  // Deshabilitar interrupciones para leer contador de forma segura
        long pulsos = contadorPulsos;
        contadorPulsos = 0;  // Reiniciar contador
        interrupts();  // Rehabilitar interrupciones

        // Calcular velocidad en RPM
        velocidadRPM = (pulsos * (60000.0 / (tActual - tiempoAnterior))) / PULSOS_POR_VUELTA;
        velRadianes = (velocidadRPM * 6.2832) / 60;  // Convertir a rad/s (2*pi/60)

        tiempoAnterior = tActual;  // Actualizar tiempo anterior
    }

    current_mA = ina219.getCurrent_mA();

    // Aplicar filtro de media móvil a la corriente
    float iFiltrada = filtro.AddValue(current_mA);

    // Enviar datos por serial: corriente filtrada, velocidad en rad/s
    Serial.print(iFiltrada);
    Serial.print(",");
    Serial.println(velRadianes);
}
