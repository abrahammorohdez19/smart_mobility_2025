#define ENCODER_A 34
#define ENCODER_B 35
#define ENCODER_Z 32

volatile long contador = 0;
volatile int lastA = 0;

// (calibrado con 10200 pulsos/vuelta) del amr
float K_DIST = 0.000174;     

void IRAM_ATTR encoderA_ISR() {
  int A = digitalRead(ENCODER_A);
  int B = digitalRead(ENCODER_B);
  if (A != lastA) {
    if (A == B) contador++;
    else        contador--;
  }
  lastA = A;
}

void IRAM_ATTR encoderZ_ISR() {}

void setup() {
  Serial.begin(115200);
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  pinMode(ENCODER_Z, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_Z), encoderZ_ISR, RISING);

  Serial.println("ESP32 Encoder Online");
}

long lastCount = 0;
unsigned long lastTime = 0;

void loop() {
  unsigned long now = millis();

  if (now - lastTime >= 50) {
    long current = contador;
    long delta = current - lastCount;
    lastCount = current;

    float dt = (now - lastTime) / 1000.0;

    float dist_m = current * K_DIST;
    float vel_mps = (delta * K_DIST) / dt;

    // Enviar como JSON
    Serial.print("{\"pulsos\":");
    Serial.print(current);
    Serial.print(",\"dist\":");
    Serial.print(dist_m, 4);
    Serial.print(",\"vel\":");
    Serial.print(vel_mps, 4);
    Serial.println("}");

    lastTime = now;
  }
}
