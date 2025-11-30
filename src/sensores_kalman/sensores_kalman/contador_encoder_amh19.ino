// Pines del encoder en ESP32 DEVKIT V1 (38 pines)
#define ENCODER_A 34
#define ENCODER_B 35
#define ENCODER_Z 32   // opcional

#define PPR 1024  // tu encoder

volatile long contador = 0;
volatile int lastA = 0;

// ISR canal A
void IRAM_ATTR encoderA_ISR() {
  int A = digitalRead(ENCODER_A);
  int B = digitalRead(ENCODER_B);

  if (A != lastA) {
    if (A == B) contador++;
    else        contador--;
  }
  lastA = A;
}

// ISR canal Z (index)
volatile long indexPos = 0;

void IRAM_ATTR encoderZ_ISR() {
  indexPos = contador;  // guarda el valor exacto
}

void setup() {
  Serial.begin(115200);

  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  pinMode(ENCODER_Z, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_Z), encoderZ_ISR, RISING);

  Serial.println("Encoder 1024PPR listo 🔥");
}

long lastCount = 0;
unsigned long lastTime = 0;

void loop() {
  unsigned long now = millis();
  if (now - lastTime >= 100) {  // 100 ms

    long delta = contador - lastCount;
    lastCount = contador;

    float vueltas = (float)delta / PPR;   // vueltas en 0.1s
    float rpm = (vueltas / 0.1) * 60.0;   // convertir a RPM

    Serial.print("Pulsos: ");
    Serial.print(contador);
    Serial.print(" | Index en: ");
    Serial.print(indexPos);
    Serial.print(" | RPM: ");
    Serial.println(rpm);

    lastTime = now;
  }
}