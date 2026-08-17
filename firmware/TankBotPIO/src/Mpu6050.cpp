#include "Mpu6050.h"
#include "Arduino.h"
#include "config.h"
#include <Wire.h>

void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission(true);
}

void mpuReadBytes(uint8_t reg, uint8_t n, uint8_t *buf) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);

  // 2-arg form avoids requestFrom overload noise on AVR
  // TODO:remove hardcoded MPU_ADDR to support multiple mpus
  uint8_t got = Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)n);
  for (uint8_t i = 0; i < got; i++)
    buf[i] = Wire.read();
}

void mpuInit() {
  mpuWrite(0x6B, 0x00); // wake
  delay(50);
  mpuWrite(0x1B, 0x00); // gyro +/-250 dps
  mpuWrite(0x1C, 0x00); // accel +/-2g
  mpuWrite(0x1A, 0x03); // DLPF ~42Hz
}

void readImu(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy,
             int16_t &gz) {
  uint8_t buf[14] = {0};
  mpuReadBytes(0x3B, 14, buf);
  ax = (int16_t)((buf[0] << 8) | buf[1]);
  ay = (int16_t)((buf[2] << 8) | buf[3]);
  az = (int16_t)((buf[4] << 8) | buf[5]);
  gx = (int16_t)((buf[8] << 8) | buf[9]);
  gy = (int16_t)((buf[10] << 8) | buf[11]);
  gz = (int16_t)((buf[12] << 8) | buf[13]);
}

void readImu(IMU &mpu) {
  uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - mpu.lastImuUs) >= IMU_US) {
    float dt = (nowUs - mpu.lastImuUs) * 1e-6f;
    mpu.lastImuUs = nowUs;

    int16_t ax, ay, az, gx, gy, gz;
    readImu(ax, ay, az, gx, gy, gz);

    float fax = (float)ax, fay = (float)ay, faz = (float)az;

    // accel pitch (deg)
    mpu.accPitchDeg = atan2f(-fax, sqrtf(fay * fay + faz * faz)) * 57.29578f;

    // gyro pitch rate (deg/s) (Gy is typical pitch axis)
    mpu.gyroPitchDps = ((float)gy) / GYRO_SENS;

    // complementary filter
    float gyroIntegrated = mpu.pitchDeg + mpu.gyroPitchDps * dt;
    // this looks like a digital highpass filter...very cool
    mpu.pitchDeg =
        COMP_ALPHA * gyroIntegrated + (1.0f - COMP_ALPHA) * mpu.accPitchDeg;
  }
}
