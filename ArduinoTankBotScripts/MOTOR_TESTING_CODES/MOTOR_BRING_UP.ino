// ====== L298N 4-channel motor bring-up (Open-loop) ======

struct Motor {
  uint8_t en;   // PWM pin (or digital if no PWM)
  uint8_t in1;
  uint8_t in2;
  bool enIsPWM;
};

Motor m1 = {5, 7, 8, true};    // L298 #1 channel A
Motor m2 = {6, 9, 10, true};   // L298 #1 channel B
Motor m3 = {3, 11, 12, true};  // L298 #2 channel A
Motor m4 = {4, 13, A0, false}; // L298 #2 channel B (no PWM on D4 in this mapping)

// speed: -255..255
void setMotor(const Motor &m, int speed) {
  speed = constrain(speed, -255, 255);

  if (speed > 0) {
    digitalWrite(m.in1, HIGH);
    digitalWrite(m.in2, LOW);
  } else if (speed < 0) {
    digitalWrite(m.in1, LOW);
    digitalWrite(m.in2, HIGH);
  } else {
    // Brake (both HIGH) or coast (both LOW). Start with coast:
    digitalWrite(m.in1, LOW);
    digitalWrite(m.in2, LOW);
  }

  int pwm = abs(speed);
  if (m.enIsPWM) {
    analogWrite(m.en, pwm);
  } else {
    // fallback: on/off
    digitalWrite(m.en, (pwm > 0) ? HIGH : LOW);
  }
}

void setup() {
  Serial.begin(115200);

  // Setup pins
  Motor motors[] = {m1, m2, m3, m4};
  for (auto &m : motors) {
    pinMode(m.in1, OUTPUT);
    pinMode(m.in2, OUTPUT);
    pinMode(m.en, OUTPUT);
    setMotor(m, 0);
  }

  Serial.println("Motor bring-up test starting...");
}

void loop() {
  // Test each motor one-by-one
  Serial.println("M1 forward");
  setMotor(m1, 180); setMotor(m2, 0); setMotor(m3, 0); setMotor(m4, 0);
  delay(1500);

  Serial.println("M1 reverse");
  setMotor(m1, -180);
  delay(1500);

  Serial.println("M2 forward");
  setMotor(m1, 0); setMotor(m2, 180);
  delay(1500);

  Serial.println("M2 reverse");
  setMotor(m2, -180);
  delay(1500);

  Serial.println("M3 forward");
  setMotor(m2, 0); setMotor(m3, 180);
  delay(1500);

  Serial.println("M3 reverse");
  setMotor(m3, -180);
  delay(1500);

  Serial.println("M4 forward (ON/OFF if no PWM)");
  setMotor(m3, 0); setMotor(m4, 255);
  delay(1500);

  Serial.println("M4 reverse");
  setMotor(m4, -255);
  delay(1500);

  Serial.println("All stop");
  setMotor(m1, 0); setMotor(m2, 0); setMotor(m3, 0); setMotor(m4, 0);
  delay(1500);
}
