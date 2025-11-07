#include <Arduino.h>
#include <math.h>

const int pinX = A0;
const int pinY = A1;
const int pinZ = A2;

float xG, yG, zG;
float roll, pitch;
float ax_lin, ay_lin, az_lin;
float vx = 0.0, vy = 0.0;
unsigned long prevT = 0;

float G = 9.81;

// Calibración ADXL335
#define X_OFFSET 355.69
#define Y_OFFSET 352.09
#define Z_OFFSET 367.08
#define X_COUNTS_PER_G -70.78
#define Y_COUNTS_PER_G -71.01
#define Z_COUNTS_PER_G 70.62

void setup() {
  Serial.begin(115200);
  delay(1000);
  prevT = micros();
  //Serial.println("IMU Node online");
}

void loop() {
  unsigned long now = micros();
  float dt = (now - prevT) / 1e6;
  prevT = now;

  leerIMU();
  calcularAngulos();
  calcularAcelLineal();

  // Integración simple para velocidad (m/s)
  vx += ax_lin * dt;
  vy += ay_lin * dt;

  Serial.print("{\"ax\":"); Serial.print(ax_lin,3);
  Serial.print(",\"ay\":"); Serial.print(ay_lin,3);
  Serial.print(",\"az\":"); Serial.print(az_lin,3);
  Serial.print(",\"vx\":"); Serial.print(vx,3);
  Serial.print(",\"vy\":"); Serial.print(vy,3);
  Serial.print(",\"roll\":"); Serial.print(roll,2);
  Serial.print(",\"pitch\":"); Serial.print(pitch,2);
  Serial.println("}");
  delay(50);
}

void leerIMU() {
  int xRaw = analogRead(pinX);
  int yRaw = analogRead(pinY);
  int zRaw = analogRead(pinZ);
  xG = (xRaw - X_OFFSET) / X_COUNTS_PER_G;
  yG = (yRaw - Y_OFFSET) / Y_COUNTS_PER_G;
  zG = (zRaw - Z_OFFSET) / Z_COUNTS_PER_G;
}

void calcularAngulos() {
  roll  = atan2(yG, zG) * 57.2958;
  pitch = atan2(-xG, sqrt(yG*yG + zG*zG)) * 57.2958;
}

void calcularAcelLineal() {
  float ax = xG * G;
  float ay = yG * G;
  float az = zG * G;

  float rollRad  = roll * DEG_TO_RAD;
  float pitchRad = pitch * DEG_TO_RAD;

  ax_lin = ax + G * sin(pitchRad);
  ay_lin = ay - G * sin(rollRad);
  az_lin = az - G * cos(pitchRad) * cos(rollRad);
}
