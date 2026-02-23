#ifndef L289NBOARD
#define L289NBOARD
#include "Arduino.h"

struct Motor {
  int ena;
  int in1;
  int in2;
};

struct MotorPorts {
  byte ena;
  byte in1;
  byte in2;
};

struct L289n {
  Motor *leftMotor;
  Motor *rightMotor;
  // Yes this is bloat but added for future functionality
  MotorPorts *leftMotorPorts;
  MotorPorts *rightMotorPorts;

  int trmLeftPWM;
  int trmRightPWM;
};

void setMotorL298n(Motor &motor, const int &signedSpeed);
void setMotorL298n(MotorPorts &motor, const int &signedSpeed);
void setMotorL298n(Motor &motor, int &signedSpeed);

void stopAll(Motor &motorA, Motor &motorB);
void stopAll(MotorPorts &motorA, MotorPorts &motorB);
void stopAll(L289n &driver);

void twoAxisDrive(L289n &driver, const int &xAxis, const int &yAxis);
void twoAxisDrive(L289n &driver, volatile int &xAxis, volatile int &yAxis);

#endif // L289N
