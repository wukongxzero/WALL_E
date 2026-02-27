/*
==================== UNO IMU SERVO STABILIZER ====================

Board: Arduino Uno (ATmega328P)
IMU: MPU6050 (I2C: SDA=A4, SCL=A5)
Servo: Pin 9

Watchdog enabled (250ms timeout)
If loop freezes → board auto resets

===============================================================
*/

#include <Wire.h>
#include <Servo.h>
#include <avr/wdt.h>

// ================= USER TUNING =================

//low pass filter

float filtered_cmd = 0.0f;
static const float TARGET_PITCH_DEG = 0.0f;
// tuning parameters. 
static const float KP = 3.0f;
static const float KD = 0.00f;
static const float KI = 0.001f;

static const float COMP_ALPHA = 0.999f;



static const float CMD_LIMIT = 25.0f;


static const uint32_t LOOP_HZ = 200;
static const uint32_t LOOP_US = 1000000UL / LOOP_HZ;

static const int SERVO_PIN = 9;
static const int SERVO_CENTER_US = 1500;
static const int SERVO_RANGE_US  = 300;

static const uint8_t MPU_ADDR = 0x68;
static const float GYRO_SENS = 131.0f;

// ================= STATE =================
Servo platformServo;

uint32_t lastLoopUs = 0;
float pitchDeg = 0.0f;
float gyroPitchDps = 0.0f;
float accPitchDeg = 0.0f;
float cmd = 0.0f;
float integralerror = 0.0f;

// ================= MPU FUNCTIONS =================
void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission(true);
}

void mpuReadBytes(uint8_t reg, uint8_t n, uint8_t* buf) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, n, true);
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
}

void mpuInit() {
  mpuWrite(0x6B, 0x00);  // wake
  delay(50);
  mpuWrite(0x1B, 0x00);  // gyro ±250 dps
  mpuWrite(0x1C, 0x00);  // accel ±2g
//   mpuWrite(0x1A, 0x03);  // DLPF ~42Hz
  mpuWrite(0x1A,0X05);
}

// ================= HELPERS =================
float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

void applyServo(float u) {
  u = clampf(u, -CMD_LIMIT, CMD_LIMIT);
  int pulse = SERVO_CENTER_US + (int)(u * (SERVO_RANGE_US / CMD_LIMIT));
  platformServo.writeMicroseconds(pulse);
}

// ================= SETUP =================
void setup() {

  // --- Detect watchdog reset ---
  if (MCUSR & (1 << WDRF)) {
    MCUSR = 0;
  }

  wdt_disable();               // safety first
  wdt_enable(WDTO_250MS);      // 250ms timeout

  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  mpuInit();

  platformServo.attach(SERVO_PIN);
  platformServo.writeMicroseconds(SERVO_CENTER_US);

  lastLoopUs = micros();
  delay(200);
}

// ================= LOOP =================
void loop() {

  uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - lastLoopUs) < LOOP_US) return;

  float dt = (nowUs - lastLoopUs) * 1e-6f;
  lastLoopUs = nowUs;

  // --- Refresh watchdog ---
  wdt_reset();

  // --- Read IMU ---
  int16_t ax, ay, az, gx, gy, gz;
  readImu:
  {
    uint8_t buf[14];
    mpuReadBytes(0x3B, 14, buf);

    ay = (int16_t)((buf[0] << 8) | buf[1]);
    ax = (int16_t)((buf[2] << 8) | buf[3]);
    az = (int16_t)((buf[4] << 8) | buf[5]);
    gx = (int16_t)((buf[8] << 8) | buf[9]);
    gy = (int16_t)((buf[10] << 8) | buf[11]);
    gz = (int16_t)((buf[12] << 8) | buf[13]);
  }

  // --- Accelerometer pitch ---
  float fax = (float)ax;
  float fay = (float)ay;
  float faz = (float)az;

  accPitchDeg = atan2f(-fax, sqrtf(fay*fay + faz*faz)) * 57.29578f;

  // --- Gyro pitch rate ---
  gyroPitchDps = ((float)gy) / GYRO_SENS;

  // --- Complementary filter ---
  float gyroIntegrated = pitchDeg + gyroPitchDps * dt;
  pitchDeg = COMP_ALPHA * gyroIntegrated +
             (1.0f - COMP_ALPHA) * accPitchDeg;

  // --- PD Control ---
  float err = TARGET_PITCH_DEG - pitchDeg;

  if(abs(err) < 1.5f){
    err = 0;
    integralerror = 0;
  }
  else{
  integralerror += err*dt;
  } 

  integralerror = clampf(integralerror, -10.0f,10.0f); //anti wind up logic
  float u = KP * err - KD * gyroPitchDps + KI*integralerror;



  cmd = clampf(u, -CMD_LIMIT, CMD_LIMIT);
  filtered_cmd = (0.7f*filteredCmd) + (0.3f*cmd); //Higher 0.7 = smoother 
  applyServo(cmd);

  // --- Debug output (optional) ---
//   Serial.print(millis());
//   Serial.print(",");millis());
//   Serial.print(",");
//   Serial.print(pitchDeg, 3);
//   Serial.print(",");
//   Serial.println(cmd, 3);
//   Serial.print(pitchDeg, 3);
//   Serial.print(",");
//   Serial.println(cmd, 3);
}