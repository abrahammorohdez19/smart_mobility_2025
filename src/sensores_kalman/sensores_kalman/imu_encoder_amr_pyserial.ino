/*
 * ESP32 - Encoder + BNO055 Integrado
 * Publica datos combinados en JSON para ROS2
 */

#include "BNO055_support.h"
#include <Wire.h>

// ============ PINES ENCODER ============
#define ENCODER_A 34
#define ENCODER_B 35
#define ENCODER_Z 32

// ============ PINES I2C (BNO055) ============
#define SDA_PIN 21
#define SCL_PIN 22
#define I2C_FREQ 400000

// ============ LED INDICADOR ============
#define LED_PIN 2

// ============ VARIABLES ENCODER ============
volatile long contador = 0;
volatile int lastA = 0;
float K_DIST = 0.000174;  // Calibrado con 10200 pulsos/vuelta
long lastCount = 0;

// ============ ESTRUCTURAS BNO055 ============
struct bno055_t myBNO;
struct bno055_euler myEulerData;
struct bno055_linear_accel myAccelData;
struct bno055_gyro myGyroData;

// ============ TIMING ============
unsigned long lastTime = 0;

// ============ ISR ENCODER ============
void IRAM_ATTR encoderA_ISR() {
  int A = digitalRead(ENCODER_A);
  int B = digitalRead(ENCODER_B);
  if (A != lastA) {
    if (A == B) contador++;
    else        contador--;
  }
  lastA = A;
}

void IRAM_ATTR encoderZ_ISR() {
  // Opcional: resetear contador en índice Z
}

void setup() {
  // ============ CONFIGURAR LED ============
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  // ============ CONFIGURAR ENCODER ============
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  pinMode(ENCODER_Z, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_Z), encoderZ_ISR, RISING);
  
  // ============ INICIALIZAR I2C ============
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(I2C_FREQ);
  
  // ============ INICIALIZAR SERIAL ============
  Serial.begin(115200);
  delay(1000);
  
  // ============ INICIALIZAR BNO055 ============
  BNO_Init(&myBNO);
  bno055_set_operation_mode(OPERATION_MODE_NDOF);
  
  // Indicador visual de inicialización
  digitalWrite(LED_PIN, HIGH);
  delay(500);
  digitalWrite(LED_PIN, LOW);
  
  // Mensaje inicial
  Serial.println("{\"status\":\"ESP32 Encoder + BNO055 Online\"}");
  
  lastCount = contador;
  lastTime = millis();
}

void loop() {
  unsigned long now = millis();
  
  // Publicar a 20Hz (50ms)
  if (now - lastTime >= 50) {
    // Toggle LED
    static bool ledState = false;
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState);
    
    // ============ LEER ENCODER ============
    long current = contador;
    long delta = current - lastCount;
    lastCount = current;
    float dt = (now - lastTime) / 1000.0;
    float dist_m = current * K_DIST;
    float vel_mps = (delta * K_DIST) / dt;
    
    // ============ LEER BNO055 ============
    bno055_read_euler_hrp(&myEulerData);
    bno055_read_linear_accel_xyz(&myAccelData);
    bno055_read_gyro_xyz(&myGyroData);
    
    // ============ CONSTRUIR JSON COMBINADO ============
    Serial.print("{");
    
    // Datos del Encoder
    Serial.print("\"pulsos\":");
    Serial.print(current);
    Serial.print(",\"dist\":");
    Serial.print(dist_m, 4);
    Serial.print(",\"vel\":");
    Serial.print(vel_mps, 4);
    
    // Aceleración lineal (m/s²)
    Serial.print(",\"ax\":");
    Serial.print(float(myAccelData.x) / 100.0, 3);
    Serial.print(",\"ay\":");
    Serial.print(float(myAccelData.y) / 100.0, 3);
    Serial.print(",\"az\":");
    Serial.print(float(myAccelData.z) / 100.0, 3);
    
    // Velocidad angular (rad/s)
    Serial.print(",\"gx\":");
    Serial.print(float(myGyroData.x) / 900.0, 4);
    Serial.print(",\"gy\":");
    Serial.print(float(myGyroData.y) / 900.0, 4);
    Serial.print(",\"gz\":");
    Serial.print(float(myGyroData.z) / 900.0, 4);
    
    // Orientación Euler (grados)
    Serial.print(",\"yaw\":");
    Serial.print(float(myEulerData.h) / 16.0, 2);
    Serial.print(",\"pitch\":");
    Serial.print(float(myEulerData.p) / 16.0, 2);
    Serial.print(",\"roll\":");
    Serial.print(float(myEulerData.r) / 16.0, 2);
    
    // Timestamp
    Serial.print(",\"timestamp\":");
    Serial.print(now);
    
    Serial.println("}");
    
    lastTime = now;
  }
}

/*
 * ============ CONEXIONES ============
 * 
 * ENCODER:
 * - Canal A -> GPIO 34
 * - Canal B -> GPIO 35
 * - Canal Z -> GPIO 32
 * 
 * BNO055:
 * - VDD -> 3.3V
 * - GND -> GND
 * - SDA -> GPIO 21
 * - SCL -> GPIO 22
 * - PS0 -> GND (modo I2C)
 * - PS1 -> GND (modo I2C)
 * 
 * LED:
 * - GPIO 2 (LED interno ESP32)
 * 
 * ============ FORMATO JSON SALIDA ============
 * {
 *   "pulsos": 1234,
 *   "dist": 0.2148,
 *   "vel": 0.5234,
 *   "ax": 0.123,
 *   "ay": -0.045,
 *   "az": 9.810,
 *   "gx": 0.0012,
 *   "gy": -0.0034,
 *   "gz": 0.0056,
 *   "yaw": 45.23,
 *   "pitch": 1.45,
 *   "roll": -0.89,
 *   "timestamp": 12345
 * }
 */