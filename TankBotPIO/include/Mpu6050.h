#ifndef MPU6050
#define MPU6050
#include <stdint.h>
struct IMU {
  char addr;
  unsigned long lastImuUs; //= millis();

  float pitchDeg;
  float rollDeg;
  float yawDeg;

  float gyroPitchDps; // gx
  float gyroRollDps;  // gy
  float gyroYawDps;   // gz

  float accPitchDeg; // ax
  float accRollDeg;  // ay
  float accYawDeg;   // az

  float servoCmd;
};

void mpuWrite(unsigned int &reg, unsigned int &val);
void mpuReadBytes(uint8_t &reg, uint8_t &n, uint8_t *buf);
void mpuInit();
void readImu(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy,
             int16_t &gz);
void readImu(IMU &mpu);

/*#ifdef __cplusplus
extern "C" {
#endif
#include "stdint.h"
*/

/*
#ifdef __cplusplus
}
#endif
*/

#endif
