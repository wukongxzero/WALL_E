/*
==================== WIRING MAP (IMU -> SERVO STABILIZER) ====================

BOARD: Arduino Mega 2560 (Powered via USB)
SENSOR: MPU6050 (GY-521)
ACTUATOR: 1x Hobby Servo

--- MPU6050 (I2C) to MEGA ---
MPU6050 VCC  -> MEGA 5V
MPU6050 GND  -> MEGA GND
MPU6050 SDA  -> MEGA Pin 20 (SDA)
MPU6050 SCL  -> MEGA Pin 21 (SCL)
MPU6050 INT  -> Not used

--- SERVO to MEGA ---
Servo Signal (orange/yellow) -> MEGA Pin 9   (SERVO_PIN = 9)
Servo V+     (red)           -> MEGA 5V
Servo GND    (brown/black)   -> MEGA GND

NOTE:
- IMU and Servo share the SAME Arduino 5V and GND.
- All grounds are common (single power source).
- Suitable for small servos (e.g., SG90) during testing.

==============================================================================
*/

#include <Wire.h>
#include <Servo.h>

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

// ================== STATE ==================
Servo platformServo;
uint32_t lastLoopUs = 0;

float pitchDeg = 0.0f;        // filtered estimate
float gyroPitchDps = 0.0f;    // deg/s
float accPitchDeg = 0.0f;     // accel-only
float cmd = 0.0f;

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

// Setup
void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  mpuInit();

  platformServo.attach(SERVO_PIN);
  platformServo.writeMicroseconds(SERVO_CENTER_US);

  // Print header for debugging plots
  Serial.println("t_ms,pitch_deg,gyro_pitch_dps,acc_pitch_deg,cmd");
  lastLoopUs = micros();

  // Give sensor a moment
  delay(200);
}

// main loop 
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
}
