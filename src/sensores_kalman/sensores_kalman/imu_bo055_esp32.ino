#include "BNO055_support.h"
#include <Wire.h>

// Definir pines I2C para ESP32 (opcional si usas los por defecto)
#define SDA_PIN 21
#define SCL_PIN 22

struct bno055_t myBNO;
struct bno055_euler myEulerData;

unsigned long lastTime = 0;

void setup()
{
  // Inicializar I2C con pines específicos de ESP32
  Wire.begin(SDA_PIN, SCL_PIN);
  
  BNO_Init(&myBNO);
  bno055_set_operation_mode(OPERATION_MODE_NDOF);
  
  delay(1);
  
  Serial.begin(115200);
}

void loop()
{
  if ((millis() - lastTime) >= 100)
  {
    lastTime = millis();
    bno055_read_euler_hrp(&myEulerData);
    
    Serial.print("Time Stamp: ");
    Serial.println(lastTime);
    Serial.print("Heading(Yaw): ");
    Serial.println(float(myEulerData.h) / 16.00);
    Serial.print("Roll: ");
    Serial.println(float(myEulerData.r) / 16.00);
    Serial.print("Pitch: ");
    Serial.println(float(myEulerData.p) / 16.00);
    Serial.println();
  }
}