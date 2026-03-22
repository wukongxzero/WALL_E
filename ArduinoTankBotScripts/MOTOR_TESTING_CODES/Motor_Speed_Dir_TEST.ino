const int ENA = 5;   // Left motor PWM
const int IN1 = 22;
const int IN2 = 23;

const int ENB = 6;   // Right motor PWM
const int IN3 = 25;
const int IN4 = 24;

enum Mode { MODE_STOP, MODE_START, MODE_LEFT, MODE_RIGHT };
Mode mode = MODE_STOP;

// Dial value 0..10 (float input)
float speedDial = 5.0f;

// Convert dial 0..10 to signed PWM -255..+255
// 5.0 -> 0
// 0.0 -> -255
// 10.0 -> +255
int dialToPWM(float dial) {
  dial = constrain(dial, 0.0f, 10.0f);

  // Small deadband around 5 to prevent jitter
  const float deadband = 0.05f;
  if (abs(dial - 5.0f) <= deadband) return 0;

  if (dial < 5.0f) {
    // Backward: 5->0 maps to 0->-255
    float frac = (5.0f - dial) / 5.0f;     // 0..1
    int pwm = (int)(frac * 255.0f + 0.5f);
    return -constrain(pwm, 0, 255);
  } else {
    // Forward: 5->10 maps to 0->+255
    float frac = (dial - 5.0f) / 5.0f;     // 0..1
    int pwm = (int)(frac * 255.0f + 0.5f);
    return constrain(pwm, 0, 255);
  }
}

void setMotorRaw(int enPin, int inA, int inB, int signedPwm) {
  signedPwm = constrain(signedPwm, -255, 255);

  if (signedPwm == 0) {
    analogWrite(enPin, 0);
    digitalWrite(inA, LOW);
    digitalWrite(inB, LOW);
    return;
  }

  if (signedPwm > 0) {
    // Forward
    digitalWrite(inA, HIGH);
    digitalWrite(inB, LOW);
    analogWrite(enPin, signedPwm);
  } else {
    // Backward
    digitalWrite(inA, LOW);
    digitalWrite(inB, HIGH);
    analogWrite(enPin, -signedPwm);
  }
}

void applyMode() {
  int pwm = dialToPWM(speedDial);

  switch (mode) {
    case MODE_STOP:
      setMotorRaw(ENA, IN1, IN2, 0);
      setMotorRaw(ENB, IN3, IN4, 0);
      break;

    case MODE_START:
      // both motors same direction/speed
      setMotorRaw(ENA, IN1, IN2, pwm);
      setMotorRaw(ENB, IN3, IN4, pwm);
      break;

    case MODE_LEFT:
      // LEFT motor OFF, RIGHT motor runs
      setMotorRaw(ENA, IN1, IN2, 0);
      setMotorRaw(ENB, IN3, IN4, pwm);
      break;

    case MODE_RIGHT:
      // RIGHT motor OFF, LEFT motor runs
      setMotorRaw(ENA, IN1, IN2, pwm);
      setMotorRaw(ENB, IN3, IN4, 0);
      break;
  }
}

void printStatus() {
  Serial.print("Mode=");
  switch (mode) {
    case MODE_STOP:  Serial.print("STOP"); break;
    case MODE_START: Serial.print("START"); break;
    case MODE_LEFT:  Serial.print("LEFT"); break;
    case MODE_RIGHT: Serial.print("RIGHT"); break;
  }
  Serial.print(" | speedDial=");
  Serial.print(speedDial, 2);
  Serial.print(" | pwm=");
  Serial.println(dialToPWM(speedDial));
}

void setup() {
  Serial.begin(115200);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  mode = MODE_STOP;
  speedDial = 5.0f;
  applyMode();

  Serial.println("L298N Mega Test Ready (with speed dial).");
  Serial.println("Commands: start | stop | left | right | speed X");
  Serial.println("Speed X: 0.0..10.0 (5.0=stop, <5 backward, >5 forward)");
  Serial.println("Set Serial Monitor line ending to 'Newline'.");
  printStatus();
}

void loop() {
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();
  line.toLowerCase();
  if (line.length() == 0) return;

  if (line == "start") {
    mode = MODE_START;
    applyMode();
    Serial.println("CMD: start");
    printStatus();
  }
  else if (line == "stop") {
    mode = MODE_STOP;
    applyMode();
    Serial.println("CMD: stop");
    printStatus();
  }
  else if (line == "left") {
    mode = MODE_LEFT;
    applyMode();
    Serial.println("CMD: left (left motor off)");
    printStatus();
  }
  else if (line == "right") {
    mode = MODE_RIGHT;
    applyMode();
    Serial.println("CMD: right (right motor off)");
    printStatus();
  }
  else if (line.startsWith("speed")) {
    // Expect: "speed 7.5"
    int sp = line.indexOf(' ');
    if (sp < 0) {
      Serial.println("Usage: speed X   (X=0..10)");
      return;
    }
    String val = line.substring(sp + 1);
    float d = val.toFloat();
    speedDial = constrain(d, 0.0f, 10.0f);
    applyMode();
    Serial.print("CMD: speed ");
    Serial.println(speedDial, 2);
    printStatus();
  }
  else {
    Serial.print("Unknown: ");
    Serial.println(line);
    Serial.println("Valid: start | stop | left | right | speed X");
  }
}