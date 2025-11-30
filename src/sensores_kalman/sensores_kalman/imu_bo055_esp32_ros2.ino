/*
 * BNO055 con librería Bosch para ESP32
 * Publica datos en JSON para ROS2
 */

#include "BNO055_support.h"
#include <Wire.h>

// Pines I2C para ESP32
#define SDA_PIN 21
#define SCL_PIN 22
#define I2C_FREQ 400000

// LED para indicar estado
#define LED_PIN 2

// Estructuras del BNO055
struct bno055_t myBNO;
struct bno055_euler myEulerData;      // Yaw, Pitch, Roll
struct bno055_linear_accel myAccelData;  // Aceleración lineal (sin gravedad)
struct bno055_gyro myGyroData;        // Giroscopio

unsigned long lastTime = 0;

void setup()
{
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  // Inicializar I2C
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(I2C_FREQ);
  
  // Inicializar Serial
  Serial.begin(115200);
  delay(1000);
  
  // Inicializar BNO055
  BNO_Init(&myBNO);
  bno055_set_operation_mode(OPERATION_MODE_NDOF);
  
  digitalWrite(LED_PIN, HIGH);
  delay(500);
  digitalWrite(LED_PIN, LOW);
  
  // Mensaje inicial en JSON
  Serial.println("{\"status\":\"BNO055 initialized\"}");
}

void loop()
{
  // Publicar a 20Hz (50ms) - más frecuencia para Kalman
  if ((millis() - lastTime) >= 50)
  {
    lastTime = millis();
    
    // Toggle LED
    static bool ledState = false;
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState);
    
    // Leer datos del sensor
    bno055_read_euler_hrp(&myEulerData);           // Orientación
    bno055_read_linear_accel_xyz(&myAccelData);    // Aceleración lineal (sin gravedad)
    bno055_read_gyro_xyz(&myGyroData);             // Velocidad angular
    
    // Construir JSON compacto
    Serial.print("{");
    
    // Aceleración lineal (en m/s²) - sin gravedad
    Serial.print("\"ax\":");
    Serial.print(float(myAccelData.x) / 100.0, 3);  // Convertir a m/s²
    Serial.print(",\"ay\":");
    Serial.print(float(myAccelData.y) / 100.0, 3);
    Serial.print(",\"az\":");
    Serial.print(float(myAccelData.z) / 100.0, 3);
    
    // Velocidad angular (en rad/s)
    Serial.print(",\"gx\":");
    Serial.print(float(myGyroData.x) / 900.0, 4);  // Convertir a rad/s
    Serial.print(",\"gy\":");
    Serial.print(float(myGyroData.y) / 900.0, 4);
    Serial.print(",\"gz\":");
    Serial.print(float(myGyroData.z) / 900.0, 4);
    
    // Orientación Euler (en grados)
    Serial.print(",\"yaw\":");
    Serial.print(float(myEulerData.h) / 16.0, 2);
    Serial.print(",\"pitch\":");
    Serial.print(float(myEulerData.p) / 16.0, 2);
    Serial.print(",\"roll\":");
    Serial.print(float(myEulerData.r) / 16.0, 2);
    
    // Timestamp
    Serial.print(",\"timestamp\":");
    Serial.print(lastTime);
    
    Serial.println("}");
  }
}

/*
 * NOTAS:
 * - bno055_read_linear_accel_xyz() lee aceleración SIN gravedad
 * - Conversión: raw / 100.0 = m/s² para aceleración
 * - Conversión: raw / 900.0 = rad/s para giroscopio  
 * - Conversión: raw / 16.0 = grados para Euler
 * 
 * CONEXIONES:
 * BNO055 VDD -> ESP32 3.3V
 * BNO055 GND -> ESP32 GND
 * BNO055 SDA -> GPIO 21
 * BNO055 SCL -> GPIO 22
 * BNO055 PS0 -> GND (modo I2C)
 * BNO055 PS1 -> GND (modo I2C)
 */