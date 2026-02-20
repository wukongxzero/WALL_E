#include <wire.h>
#include <PS2X_lib.h>
#include <servo.h>
PS2X ps2x;

// ===== PS2 on Mega hardware SPI pins =====
#define PS2_DAT 50
#define PS2_CMD 51
#define PS2_SEL 53
#define PS2_CLK 52


// user tuning parameters
static const float TARGET_PITCH_DEG = 0.0f; // "level" target
static const float KP = 3.0f;               // start 2.0–6.0
static const float KD = 0.08f;              // start 0.02–0.15 (set 0 to disable)
static const float CMD_LIMIT = 25.0f;       // limit output command


// Complementary filter (gyro vs accel blend)
static const float COMP_ALPHA = 0.98f;      // 0.97–0.995 typical

// Loop rates
static const uint32_t LOOP_HZ = 200;        // 100–400 is fine
static const uint32_t LOOP_US = 1000000UL / LOOP_HZ;

// Servo config
static const int SERVO_PIN = 9;
static const int SERVO_CENTER_US = 1500;
static const int SERVO_RANGE_US  = 300;     // +/- pulse range around center

// MPU6050 config
static const uint8_t MPU_ADDR = 0x68;
static const float GYRO_SENS = 131.0f;      // LSB/(deg/s) for +/-250 dps

//state
Servo platformServo;
uint32_t lastLoopUs = 0;

float pitchDeg = 0.0f;        // filtered estimate
float gyroPitchDps = 0.0f;    // deg/s
float accPitchDeg = 0.0f;     // accel-only
float cmd = 0.0f;

// L298N pins
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


// ================== MPU6050 LOW LEVEL ==================
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
  Wire.requestFrom(MPU_ADDR, n, true);
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
}

static void mpuInit() {
  // Wake
  mpuWrite(0x6B, 0x00);
  delay(50);

  // Gyro +/-250 dps
  mpuWrite(0x1B, 0x00);
  // Accel +/-2g
  mpuWrite(0x1C, 0x00);
  // DLPF ~42Hz gyro / 44Hz accel (helps noise)
  mpuWrite(0x1A, 0x03);
}


static void readImu(int16_t& ax, int16_t& ay, int16_t& az,
                    int16_t& gx, int16_t& gy, int16_t& gz) {
  uint8_t buf[14];
  mpuReadBytes(0x3B, 14, buf);
  ax = (int16_t)((buf[0] << 8) | buf[1]);
  ay = (int16_t)((buf[2] << 8) | buf[3]);
  az = (int16_t)((buf[4] << 8) | buf[5]);
  gx = (int16_t)((buf[8] << 8) | buf[9]);
  gy = (int16_t)((buf[10] << 8) | buf[11]);
  gz = (int16_t)((buf[12] << 8) | buf[13]);
}

// ================== HELPERS ==================
static float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

// Converts "cmd" (in deg-like units) into servo PWM
static void applyServo(float u) {
  u = clampf(u, -CMD_LIMIT, CMD_LIMIT);
  int pulse = SERVO_CENTER_US + (int)(u * (SERVO_RANGE_US / CMD_LIMIT));
  platformServo.writeMicroseconds(pulse);
}

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
  if (ps2_error == 0) 
  Serial.println("✅ Controller connected");
  else 
  Serial.println("❌ Controller NOT connected");
}

void setup() {
  Serial.begin(115200);
  wire.begin();
  wire.setClock(400000);
  mpuInit();

  platformServo.attach(SERVO_PIN);
  platformServo.writeMicroseconds(SERVO_CENTER_US);

  // Print header for debugging plots
  Serial.println("t_ms,pitch_deg,gyro_pitch_dps,acc_pitch_deg,cmd");
  
  lastLoopUs = micros();

  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  stopAll();     // stop immediately on boot
  delay(300);

  tryConnectPS2();
}

void loop() {
  uint32_t nowUs = micros();
  
  if ((uint32_t)(nowUs - lastLoopUs) < LOOP_US) return;
  float dt = (nowUs - lastLoopUs) * 1e-6f;
  lastLoopUs = nowUs;

  // Read IMU
  int16_t ax, ay, az, gx, gy, gz;
  readImu(ax, ay, az, gx, gy, gz);

  // --- Accel-only pitch (deg) ---
  // This assumes your board is mounted such that X points forward.
  // If your pitch axis is different, you'll swap axes here.
  float fax = (float)ax, fay = (float)ay, faz = (float)az;
  accPitchDeg = atan2f(-fax, sqrtf(fay*fay + faz*faz)) * 57.29578f;

  // --- Gyro pitch rate (deg/s) ---
  // Many MPU6050 boards use Gy as pitch when X forward, Z up.
  gyroPitchDps = ((float)gy) / GYRO_SENS;

  // --- Complementary filter ---
  float gyroIntegrated = pitchDeg + gyroPitchDps * dt;
  pitchDeg = COMP_ALPHA * gyroIntegrated + (1.0f - COMP_ALPHA) * accPitchDeg;

  // --- Control (PD) ---
  float err = TARGET_PITCH_DEG - pitchDeg;

  // D-term using gyro (already a rate around pitch axis)
  float u = KP * err - KD * gyroPitchDps;
  cmd = clampf(u, -CMD_LIMIT, CMD_LIMIT);

  applyServo(cmd);

  // Debug stream (~200Hz; you can lower if needed)
  Serial.print(millis());
  Serial.print(",");
  Serial.print(pitchDeg, 3);
  Serial.print(",");
  // Serial.print(gyroPitchDps, 3);
  // Serial.print(",");
  // Serial.print(accPitchDeg, 3);
  // Serial.print(",");
  Serial.println(cmd, 3);
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