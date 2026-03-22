// ===== 2-Belt L298N Bring-up (Arduino UNO) =====
// Left motor: ENA, IN1, IN2
// Right motor: ENB, IN3, IN4

const uint8_t L_EN = 5;   // PWM
const uint8_t L_IN1 = 7;
const uint8_t L_IN2 = 8;

const uint8_t R_EN = 6;   // PWM
const uint8_t R_IN3 = 9;
const uint8_t R_IN4 = 10;

void setMotor(int pwm, uint8_t en, uint8_t inA, uint8_t inB) {
  pwm = constrain(pwm, -255, 255);

  if (pwm > 0) {          // forward
    digitalWrite(inA, HIGH);
    digitalWrite(inB, LOW);
  } else if (pwm < 0) {   // reverse
    digitalWrite(inA, LOW);
    digitalWrite(inB, HIGH);
  } else {                // coast
    digitalWrite(inA, LOW);
    digitalWrite(inB, LOW);
  }
  analogWrite(en, abs(pwm));
}

void setLeft(int pwm)  { setMotor(pwm, L_EN, L_IN1, L_IN2); }
void setRight(int pwm) { setMotor(pwm, R_EN, R_IN3, R_IN4); }

void setup() {
  Serial.begin(115200);

  pinMode(L_EN, OUTPUT); pinMode(L_IN1, OUTPUT); pinMode(L_IN2, OUTPUT);
  pinMode(R_EN, OUTPUT); pinMode(R_IN3, OUTPUT); pinMode(R_IN4, OUTPUT);

  setLeft(0); setRight(0);
  Serial.println("2-belt motor bring-up starting...");
}

void loop() {
  Serial.println("Forward");
  setLeft(180); setRight(180);
  delay(1500);

  Serial.println("Stop");
  setLeft(0); setRight(0);
  delay(700);

  Serial.println("Reverse");
  setLeft(-180); setRight(-180);
  delay(1500);

  Serial.println("Pivot turn (L fwd, R rev)");
  setLeft(180); setRight(-180);
  delay(1500);

  Serial.println("Stop");
  setLeft(0); setRight(0);
  delay(1500);
}
