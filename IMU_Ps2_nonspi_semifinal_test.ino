/*
  MEGA: PS2 (NON-SPI) + L298N tank drive + MPU6050 pitch PD + Servo stabilizer
  --------------------------------------------------------------------------
  PS2 receiver wiring (NON-SPI pins):
    DAT -> D38
    CMD -> D39
    ATT/SEL -> D40
    CLK -> D41
    VCC -> 3.3V
    GND -> GND

  MPU6050 wiring (MEGA fixed I2C pins):
    SDA -> D20
    SCL -> D21
    VCC/GND accordingly

  L298N wiring:
    ENA -> D5 (PWM)   IN1 -> D23   IN2 -> D22   (Left motor)
    ENB -> D6 (PWM)   IN3 -> D25   IN4 -> D24   (Right motor)

  Servo:
    Signal -> D9, power from proper 5V supply, common GND.
*/

#include <Arduino.h>
#include <Wire.h>
#include <PS2X_lib.h>
#include <Servo.h>

// ===================== PS2 (NON-SPI pins) =====================
PS2X ps2x;
#define PS2_DAT 38
#define PS2_CMD 39
#define PS2_SEL 40
#define PS2_CLK 41

// ===================== L298N pins =====================
const int ENA = 5;
const int IN1 = 23;
const int IN2 = 22;

const int ENB = 6;
const int IN3 = 25;
const int IN4 = 24;

// ===================== Servo =====================
const int SERVO_PIN = 9;
const int SERVO_CENTER_US = 1500;
const int SERVO_RANGE_US  = 300;   // +/- around center
Servo platformServo;

// ===================== MPU6050 =====================
const uint8_t MPU_ADDR = 0x68;
const float GYRO_SENS = 131.0f;    // LSB/(deg/s) for +/-250 dps

// ===================== Stabilizer tuning =====================
const float TARGET_PITCH_DEG = 0.0f;
const float KP = 3.0f;
const float KD = 0.08f;
const float CMD_LIMIT = 25.0f;
const float COMP_ALPHA = 0.98f;

// IMU loop
const uint32_t IMU_HZ = 200;
const uint32_t IMU_US = 1000000UL / IMU_HZ;

// PS2 loop (separate, to avoid lag)
const uint32_t PS2_MS = 10;  // 100 Hz

// Motor control tuning
const int DEADZONE  = 12;
const int RAMP_STEP = 30;    // faster response (was 12)
const uint32_t DBG_MS = 120;

// ===================== State =====================
int ps2_error = 1;

uint32_t lastImuUs = 0;
float pitchDeg = 0.0f;
float gyroPitchDps = 0.0f;
float accPitchDeg = 0.0f;
float servoCmd = 0.0f;

int leftOut = 0;
int rightOut = 0;

// ===================== Helpers =====================
static float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static int clampInt(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static int applyDeadzone(int v, int dz) {
  return (abs(v) <= dz) ? 0 : v;
}

static int rampToward(int cur, int tgt, int step) {
  if (cur < tgt) return min(cur + step, tgt);
  if (cur > tgt) return max(cur - step, tgt);
  return cur;
}

static const char* dirStr(int c) {
  if (c > 0) return "FWD";
  if (c < 0) return "REV";
  return "STOP";
}

// ===================== L298N =====================
static void setMotorL298(int en, int a, int b, int signedSpeed) {
  int pwm = clampInt(abs(signedSpeed), 0, 255);

  if (pwm == 0) {
    digitalWrite(a, LOW);
    digitalWrite(b, LOW);
    analogWrite(en, 0);
    return;
  }

  if (signedSpeed > 0) {   // forward
    digitalWrite(a, HIGH);
    digitalWrite(b, LOW);
  } else {                 // reverse
    digitalWrite(a, LOW);
    digitalWrite(b, HIGH);
  }
  analogWrite(en, pwm);
}

static void stopAll() {
  leftOut = 0;
  rightOut = 0;
  setMotorL298(ENA, IN1, IN2, 0);
  setMotorL298(ENB, IN3, IN4, 0);
}

// ===================== Servo =====================
static void applyServo(float u) {
  u = clampf(u, -CMD_LIMIT, CMD_LIMIT);
  int pulse = SERVO_CENTER_US + (int)(u * (SERVO_RANGE_US / CMD_LIMIT));
  platformServo.writeMicroseconds(pulse);
}

// ===================== MPU6050 low-level =====================
static void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission(true);
}

static void mpuReadBytes(uint8_t reg, uint8_t n, uint8_t* buf) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);

  // 2-arg form avoids requestFrom overload noise on AVR
  uint8_t got = Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)n);
  for (uint8_t i = 0; i < got; i++) buf[i] = Wire.read();
}

static void mpuInit() {
  mpuWrite(0x6B, 0x00); // wake
  delay(50);
  mpuWrite(0x1B, 0x00); // gyro +/-250 dps
  mpuWrite(0x1C, 0x00); // accel +/-2g
  mpuWrite(0x1A, 0x03); // DLPF ~42Hz
}

static void readImu(int16_t& ax, int16_t& ay, int16_t& az,
                    int16_t& gx, int16_t& gy, int16_t& gz) {
  uint8_t buf[14] = {0};
  mpuReadBytes(0x3B, 14, buf);

  ax = (int16_t)((buf[0] << 8) | buf[1]);
  ay = (int16_t)((buf[2] << 8) | buf[3]);
  az = (int16_t)((buf[4] << 8) | buf[5]);
  gx = (int16_t)((buf[8] << 8) | buf[9]);
  gy = (int16_t)((buf[10] << 8) | buf[11]);
  gz = (int16_t)((buf[12] << 8) | buf[13]);
}

// ===================== PS2 connect =====================
static void tryConnectPS2() {
  ps2_error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, false, false);
  Serial.print("PS2 error = ");
  Serial.println(ps2_error);
  if (ps2_error == 0) Serial.println("✅ Controller connected");
  else                Serial.println("❌ Controller NOT connected");
}

// ===================== setup =====================
void setup() {
  Serial.begin(115200);
  delay(800);

  // I2C (MEGA uses SDA=20, SCL=21)
  Wire.begin();
  Wire.setClock(400000);
  mpuInit();

  // Servo
  platformServo.attach(SERVO_PIN);
  platformServo.writeMicroseconds(SERVO_CENTER_US);

  // Motor pins
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  stopAll();

  tryConnectPS2();

  lastImuUs = micros();

  Serial.println("READY: PS2+Motors+IMU+Servo");
}

// ===================== loop =====================
void loop() {
  // -------- PS2 read on its own schedule (snappy controls) --------
  static uint32_t lastPs2Ms = 0;
  if (ps2_error == 0 && (uint32_t)(millis() - lastPs2Ms) >= PS2_MS) {
    lastPs2Ms = millis();
    ps2x.read_gamepad(false, 0);
  }

  // -------- reconnect attempts --------
  static uint32_t lastTryMs = 0;
  if (ps2_error != 0 && (uint32_t)(millis() - lastTryMs) > 1000) {
    lastTryMs = millis();
    tryConnectPS2();
  }
  if (ps2_error != 0) {
    stopAll();
    return;
  }

  // -------- IMU loop at fixed rate --------
  uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - lastImuUs) >= IMU_US) {
    float dt = (nowUs - lastImuUs) * 1e-6f;
    lastImuUs = nowUs;

    int16_t ax, ay, az, gx, gy, gz;
    readImu(ax, ay, az, gx, gy, gz);

    float fax = (float)ax, fay = (float)ay, faz = (float)az;

    // accel pitch (deg)
    accPitchDeg = atan2f(-fax, sqrtf(fay * fay + faz * faz)) * 57.29578f;

    // gyro pitch rate (deg/s) (Gy is typical pitch axis)
    gyroPitchDps = ((float)gy) / GYRO_SENS;

    // complementary filter
    float gyroIntegrated = pitchDeg + gyroPitchDps * dt;
    pitchDeg = COMP_ALPHA * gyroIntegrated + (1.0f - COMP_ALPHA) * accPitchDeg;

    // PD control
    float err = TARGET_PITCH_DEG - pitchDeg;
    float u = KP * err - KD * gyroPitchDps;
    servoCmd = clampf(u, -CMD_LIMIT, CMD_LIMIT);
    applyServo(servoCmd);
  }

  // -------- Motor commands (runs every loop, fast) --------
  // Default mapping: throttle = LEFT Y, turn = RIGHT X
  int rawThr  = ps2x.Analog(PSS_LY);
  int rawTurn = ps2x.Analog(PSS_RX);

  bool looksBad = (rawThr == 0 || rawThr == 255) && (rawTurn == 0 || rawTurn == 255);
  if (looksBad) {
    stopAll();
    return;
  }

  int throttle = -(rawThr - 128);     // up = forward
  int turn     =  (rawTurn - 128);    // right = turn right

  throttle = applyDeadzone(throttle, DEADZONE);
  turn     = applyDeadzone(turn, DEADZONE);

  int throttle255 = clampInt(throttle * 2, -255, 255);
  int turn255     = clampInt(turn * 2,     -255, 255);

  int leftCmd  = clampInt(throttle255 + turn255, -255, 255);
  int rightCmd = clampInt(throttle255 - turn255, -255, 255);

  // START = emergency stop
  if (ps2x.Button(PSB_START)) {
    leftCmd = 0;
    rightCmd = 0;
  }

  leftOut  = rampToward(leftOut,  leftCmd,  RAMP_STEP);
  rightOut = rampToward(rightOut, rightCmd, RAMP_STEP);

  setMotorL298(ENA, IN1, IN2, leftOut);
  setMotorL298(ENB, IN3, IN4, rightOut);

  // -------- Debug at low rate --------
  static uint32_t lastDbgMs = 0;
  if ((uint32_t)(millis() - lastDbgMs) >= DBG_MS) {
    lastDbgMs = millis();
    Serial.print("pitch="); Serial.print(pitchDeg, 2);
    Serial.print(" u="); Serial.print(servoCmd, 2);
    Serial.print(" | L "); Serial.print(dirStr(leftOut)); Serial.print(" "); Serial.print(abs(leftOut));
    Serial.print(" | R "); Serial.print(dirStr(rightOut)); Serial.print(" "); Serial.print(abs(rightOut));
    Serial.print(" | Thr="); Serial.print(rawThr);
    Serial.print(" Turn="); Serial.println(rawTurn);
  }
}