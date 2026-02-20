#include <PS2X_lib.h>
PS2X ps2x;

// ===== PS2 on Mega hardware SPI pins =====
#define PS2_DAT 50
#define PS2_CMD 51
#define PS2_SEL 53
#define PS2_CLK 52

// ===== L298N pins =====
const int ENA = 5;   // PWM
const int IN1 = 22;
const int IN2 = 23;

const int ENB = 6;   // PWM
const int IN3 = 24;
const int IN4 = 25;

// ===== Tuning =====
const int DEADZONE = 12;
const int PRINT_MS = 120;

// ramp limits (PWM units per loop). Increase for snappier response.
const int RAMP_STEP = 12;

int ps2_error = 1;

// current motor commands after ramp
int leftOut = 0;
int rightOut = 0;

int clampInt(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

int applyDeadzone(int v, int dz) {
  return (abs(v) <= dz) ? 0 : v;
}

const char* dirStr(int cmd) {
  if (cmd > 0) return "FWD";
  if (cmd < 0) return "REV";
  return "STOP";
}

void setMotorL298(int en, int a, int b, int signedSpeed) {
  int pwm = abs(signedSpeed);
  pwm = clampInt(pwm, 0, 255);

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
  leftOut = 0;
  rightOut = 0;
  setMotorL298(ENA, IN1, IN2, 0);
  setMotorL298(ENB, IN3, IN4, 0);
}

int rampToward(int current, int target, int step) {
  if (current < target) return min(current + step, target);
  if (current > target) return max(current - step, target);
  return current;
}

void tryConnectPS2() {
  ps2_error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, false, false);
  Serial.print("PS2 error = "); Serial.println(ps2_error);
  if (ps2_error == 0) Serial.println("✅ Controller connected");
  else                Serial.println("❌ Controller NOT connected");
}

void setup() {
  Serial.begin(115200);

  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  stopAll();     // stop immediately on boot
  delay(300);

  tryConnectPS2();
}

void loop() {
  // reconnect attempts
  static unsigned long lastTry = 0;
  if (ps2_error != 0 && millis() - lastTry > 1000) {
    lastTry = millis();
    tryConnectPS2();
  }

  // if not connected => always stop
  if (ps2_error != 0) {
    stopAll();
    delay(50);
    return;
  }

  ps2x.read_gamepad(false, 0);

  int rawLY = ps2x.Analog(PSS_LY);
  int rawRX = ps2x.Analog(PSS_RX);

  // If readings look invalid, stop (extra safety)
  bool looksBad = (rawLY == 0 || rawLY == 255) && (rawRX == 0 || rawRX == 255);
  if (looksBad) {
    stopAll();
    delay(50);
    return;
  }

  // Map sticks to throttle/turn
  int throttle = -(rawLY - 128);  // up = forward
  int turn     =  (rawRX - 128);

  throttle = applyDeadzone(throttle, DEADZONE);
  turn     = applyDeadzone(turn, DEADZONE);

  int throttle255 = throttle * 2;
  int turn255     = turn * 2;

  int leftCmd  = clampInt(throttle255 + turn255, -255, 255);
  int rightCmd = clampInt(throttle255 - turn255, -255, 255);

  // START = hard stop
  if (ps2x.Button(PSB_START)) {
    leftCmd = 0;
    rightCmd = 0;
  }

  // Ramp to avoid jerk/brownout
  leftOut  = rampToward(leftOut,  leftCmd,  RAMP_STEP);
  rightOut = rampToward(rightOut, rightCmd, RAMP_STEP);

  // Apply to motors
  setMotorL298(ENA, IN1, IN2, leftOut);
  setMotorL298(ENB, IN3, IN4, rightOut);

  // Print
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= PRINT_MS) {
    lastPrint = millis();
    Serial.print("LEFT "); Serial.print(dirStr(leftOut));
    Serial.print(" pwm="); Serial.print(abs(leftOut));
    Serial.print(" | RIGHT "); Serial.print(dirStr(rightOut));
    Serial.print(" pwm="); Serial.print(abs(rightOut));
    Serial.print(" | rawLY="); Serial.print(rawLY);
    Serial.print(" rawRX="); Serial.print(rawRX);
    Serial.println();
  }
}