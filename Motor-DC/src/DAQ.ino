
/*
 * DAQ Motor DC — ESP32 Dual Core
 * ────────────────────────────────────────────────────────────
 * Núcleo 0 → Lectura INA219 (I2C) + Envío Serial
 * Núcleo 1 → Cálculo de velocidad del encoder (default de Arduino)
 * ────────────────────────────────────────────────────────────
 */

#include "Wire.h"
#include "WiFi.h"

// =============================================================================
// CONFIGURACIÓN
// =============================================================================
const int PIN_ENCODER_A = 18;   // Encoder canal A
const int PIN_ENCODER_B = 19;   // Encoder canal B
const int motorPin     = 15;   // PWM motor
const int PPR           = 600;  // Pulsos por vuelta

bool inicio = true;

// I2C INA219
const uint8_t INA219_ADDR  = 0x40;
const float   R_SHUNT      = 0.1;   // Ohms
const float   LSB_SHUNT_uV = 10.0;  // µV por bit

// Intervalo de cálculo de velocidad
const uint32_t T_VELOCIDAD_MS = 20;  // ms

uint32_t tInicio = 0;

// =============================================================================
// VARIABLES COMPARTIDAS ENTRE NÚCLEOS
// Usar volatile + mutex para acceso seguro entre tareas
// =============================================================================
volatile float g_corriente = 0.0;
volatile float g_radps     = 0.0;
volatile long  g_pulsos    = 0;

portMUX_TYPE mux_pulsos    = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE mux_corriente = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE mux_radps     = portMUX_INITIALIZER_UNLOCKED;

// =============================================================================
// INA219 — Lectura directa por registros (sin librería)
// =============================================================================
void resetI2C() {
  Wire.end();
  delay(100);
  Wire.begin(21, 22);
}

void recoverI2C() {
  pinMode(22, OUTPUT); // SCL
  for (int i = 0; i < 9; i++) {
    digitalWrite(22, HIGH);
    delayMicroseconds(5);
    digitalWrite(22, LOW);
    delayMicroseconds(5);
  }
  Wire.begin(21, 22);
}

float ina219_leerCorriente() {
  Wire.beginTransmission(INA219_ADDR);
  Wire.write(0x01);  // Registro voltaje shunt
  Wire.endTransmission();
  Wire.requestFrom(INA219_ADDR, (uint8_t)2);
  int16_t raw = (Wire.read() << 8) | Wire.read();
  return ((raw * LSB_SHUNT_uV) / 1e6) / R_SHUNT;  // Amperes
}
// =============================================================================
// ESTADO DEL MOTOR
// =============================================================================
volatile bool g_motorOn = false;
portMUX_TYPE mux_motorOn = portMUX_INITIALIZER_UNLOCKED;

// =============================================================================
// ENCODER — ISR (las ISR del ESP32 corren en el núcleo que las registró)
// =============================================================================
void IRAM_ATTR encoderISR() {
  static uint32_t ultimo_us = 0;
  uint32_t ahora = micros();
  if (ahora - ultimo_us < 500) return;  // Anti-rebote: ignorar pulsos < 500µs
  ultimo_us = ahora;

  portENTER_CRITICAL_ISR(&mux_pulsos);
  digitalRead(PIN_ENCODER_B) ? g_pulsos++ : g_pulsos--;
  portEXIT_CRITICAL_ISR(&mux_pulsos);
}

// =============================================================================
// TAREA NÚCLEO 0 — Lectura INA219 + Envío Serial
// =============================================================================
void TaskSensores(void *pvParameters) {
  Wire.begin(21, 22);
  Wire.setClock(100000);  // Fast mode real 400 kHz

  // Verificar presencia del INA219
  Wire.beginTransmission(INA219_ADDR);
  uint8_t err = Wire.endTransmission();
  if (err != 0) {
    Serial.printf("INA219 no encontrado (error %d)\n", err);
    resetI2C();
    recoverI2C();
    vTaskDelete(NULL);
    return;
  }

  Serial.println("LISTO");

  for (;;) {
    float corriente = ina219_leerCorriente();

    portENTER_CRITICAL(&mux_corriente);
    g_corriente = corriente;
    portEXIT_CRITICAL(&mux_corriente);

    float radps_local;
    portENTER_CRITICAL(&mux_radps);
    radps_local = g_radps;
    portEXIT_CRITICAL(&mux_radps);

    bool motorOn_local;
    portENTER_CRITICAL(&mux_motorOn);
    motorOn_local = g_motorOn;
    portEXIT_CRITICAL(&mux_motorOn);

    float voltaje = motorOn_local ? 12.0 : 0.0;

    Serial.printf("%lu,%.2f,%.4f,%.4f\n",
                  millis(), voltaje, corriente, radps_local);

    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

// =============================================================================
// TAREA NÚCLEO 1 — Cálculo de velocidad del encoder
// Mismo núcleo que las ISR → no hay contención en el mutex de pulsos
// =============================================================================
void TaskVelocidad(void *pvParameters) {
  uint32_t tAnterior = millis();

  for (;;) {
    vTaskDelay(T_VELOCIDAD_MS / portTICK_PERIOD_MS);

    uint32_t tActual = millis();
    uint32_t dt      = tActual - tAnterior;

    long pulsos_local;
    portENTER_CRITICAL(&mux_pulsos);
    pulsos_local = g_pulsos;
    g_pulsos     = 0;
    portEXIT_CRITICAL(&mux_pulsos);

    float rpm   = (pulsos_local * 60000.0 / dt) / PPR;
    float radps = (rpm * 6.2832 / 60.0)*(-1);

    portENTER_CRITICAL(&mux_radps);
    g_radps = radps;
    portEXIT_CRITICAL(&mux_radps);

    tAnterior = tActual;
  }
}

// =============================================================================
// TAREA NÚCLEO 1 — Conmutación del motor (cada 1 segundo)
// =============================================================================
void TaskMotor(void *pvParameters) {
  // Esperar handshake del PC (el script Python envía un byte al abrir el puerto)
  while (Serial.available() == 0) {
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
  Serial.read(); // Consumir el byte

  // Ahora sí esperar 1 segundo
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  digitalWrite(motorPin, HIGH);
  portENTER_CRITICAL(&mux_motorOn);
  g_motorOn = true;
  portEXIT_CRITICAL(&mux_motorOn);

  // Tarea terminó su trabajo
  vTaskDelete(NULL);
}

// =============================================================================
// SETUP
// =============================================================================
void setup() {
  WiFi.mode(WIFI_OFF);  // Apagar WiFi y Bluetooth libera ~80KB RAM
  btStop();

  Serial.begin(115200);

  // Configurar GPIO del motor como salida
  pinMode(motorPin, OUTPUT);
  digitalWrite(motorPin, LOW);  // Inicialmente apagado

  // Configurar encoder
  pinMode(PIN_ENCODER_A, INPUT_PULLUP);
  pinMode(PIN_ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_A), encoderISR, RISING);

  // Crear tareas FreeRTOS
  //                     Nombre       Stack   Param  Prior  Handle  Núcleo
  xTaskCreatePinnedToCore(TaskSensores,  "I2C+TX",  4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(TaskVelocidad, "Encoder", 2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(TaskMotor,     "Motor",   2048, NULL, 1, NULL, 1);
}

// loop() vacío — todo corre en las tareas FreeRTOS
void loop() {
  vTaskDelay(portMAX_DELAY);  // No hacer nada
}