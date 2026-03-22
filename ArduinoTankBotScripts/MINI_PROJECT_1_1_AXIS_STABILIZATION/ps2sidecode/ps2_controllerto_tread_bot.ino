// this code is only controller + motors, to test if it goes forward , backward, left , right !

#include <PS2X_lib.h>
PS2X ps2x;

// PS2 on Mega pins
#define PS2_DAT 38
#define PS2_CMD 39
#define PS2_SEL 40
#define PS2_CLK 41

// L298N pins
const int ENA = 5;   // PWM
const int IN1 = 23;
const int IN2 = 22;

const int ENB = 6;   // PWM
const int IN3 = 25;
const int IN4 = 24;

const int DEADZONE = 12;
const int PRINT_MS = 120;

int ps2_error = 1;  // start as "not connected"

// If turning is reversed or one side fights the other, flip these:
const bool INVERT_LEFT_MOTOR  = false;
const bool INVERT_RIGHT_MOTOR = false;

int clampInt(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

int applyDeadzone(int v, int dz) {
  if (abs(v) <= dz) return 0;
  return v;
}

const char* dirStr(int cmd) {
  if (cmd > 0) return "FWD";
  if (cmd < 0) return "REV";
  return "STOP";
}

void setMotorL298(int en, int a, int b, int signedSpeed) {
  int pwm = abs(signedSpeed);
//   pwm = clampInt(pwm, 0, 255);
    pwm = clamperInt(pwm,0,200);

  if (pwm == 0) {
    digitalWrite(a, LOW);
    digitalWrite(b, LOW);
    analogWrite(en, 0);
    return;
  }

  if (signedSpeed > 0) {        // forward
    digitalWrite(a, HIGH);
    digitalWrite(b, LOW);
  } else {                      // reverse
    digitalWrite(a, LOW);
    digitalWrite(b, HIGH);
  }
  analogWrite(en, pwm);
}

void stopAll() {
  setMotorL298(ENA, IN1, IN2, 0);
  setMotorL298(ENB, IN3, IN4, 0);
}

void tryConnectPS2() {
  ps2_error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, false, false);
  Serial.print("PS2 error = "); Serial.println(ps2_error);
  if (ps2_error == 0) Serial.println(" Controller connected");
  else                Serial.println(" Controller NOT connected");
}

void setup() {
  Serial.begin(115200);

  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  stopAll();
  delay(300);

  tryConnectPS2();
}

void loop() {
  // Retry connection every 1s if not connected
  static unsigned long lastTry = 0;
  if (ps2_error != 0 && millis() - lastTry > 1000) {
    lastTry = millis();
    tryConnectPS2();
  }

  // If not connected => keep motors stopped
  if (ps2_error != 0) {
    stopAll();
    delay(50);
    return;
  }

  ps2x.read_gamepad(false, 0);

  //  FIX: correct axes
  int rawLY = ps2x.Analog(PSS_RX); // Left stick Y (throttle)
  int rawRX = ps2x.Analog(PSS_LY); // Right stick X (turn)

  // Optional safety: only stop if BOTH are extreme at the same time (rare)
  bool looksBad = ((rawLY == 0 || rawLY == 255) && (rawRX == 0 || rawRX == 255));
  // if (looksBad) {
  //   stopAll();
  //   delay(50);
  //   return;
  // }

  // Convert to signed centered values (-128..127)
  int throttle = -(rawLY - 128);  // push up = forward
  int turn     =  (rawRX - 128);  // right = positive

  throttle = applyDeadzone(throttle, DEADZONE);
  turn     = applyDeadzone(turn, DEADZONE);

  // Scale to about -255..255
  int throttle255 = throttle * 2;
  int turn255     = turn * 2;

  // Differential mix
  int leftCmd  = clampInt(throttle255 + turn255, -255, 255);
  int rightCmd = clampInt(throttle255 - turn255, -255, 255);

  // Emergency stop
  if (ps2x.Button(PSB_START)) {
    leftCmd = 0; rightCmd = 0;
  }

  // Optional motor inversion (for wiring/mechanical differences)
  if (INVERT_LEFT_MOTOR)  leftCmd  = -leftCmd;
  if (INVERT_RIGHT_MOTOR) rightCmd = -rightCmd;

  // Send to motors (keep your mapping: ENA=right, ENB=left)
  setMotorL298(ENA, IN1, IN2, rightCmd);
  setMotorL298(ENB, IN3, IN4, leftCmd);

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= PRINT_MS) {
    lastPrint = millis();
    // Serial.print("LEFT "); Serial.print(dirStr(leftCmd));
    // Serial.print(" pwm="); Serial.print(abs(leftCmd));
    // Serial.print(" | RIGHT "); Serial.print(dirStr(rightCmd));
    // Serial.print(" pwm="); Serial.print(abs(rightCmd));
    // Serial.print(" | rawLY="); Serial.print(rawLY);
    // Serial.print(" rawRX="); Serial.print(rawRX);
    // Serial.println();
  }
}