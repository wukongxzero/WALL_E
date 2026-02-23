#include "L829n.h"
#include "Arduino.h"

void setMotorL298n(Motor &motor, const int &signedSpeed) {
  int pwm = max(min(abs(signedSpeed), 255), 0);

  if (pwm == 0) {
    digitalWrite(motor.in1, LOW);
    digitalWrite(motor.in2, LOW);
    analogWrite(motor.ena, 0);
    return;
  }

  if (signedSpeed > 0) { // forward
    digitalWrite(motor.in1, HIGH);
    digitalWrite(motor.in2, LOW);
  } else { // reverse
    digitalWrite(motor.in1, LOW);
    digitalWrite(motor.in2, HIGH);
  }

  analogWrite(motor.ena, pwm);
}
void setMotorL298n(Motor &motor, int &signedSpeed) {
  int pwm = max(min(abs(signedSpeed), 255), 0);

  if (pwm == 0) {
    digitalWrite(motor.in1, LOW);
    digitalWrite(motor.in2, LOW);
    analogWrite(motor.ena, 0);
    return;
  }

  if (signedSpeed > 0) { // forward
    digitalWrite(motor.in1, HIGH);
    digitalWrite(motor.in2, LOW);
  } else { // reverse
    digitalWrite(motor.in1, LOW);
    digitalWrite(motor.in2, HIGH);
  }

  analogWrite(motor.ena, pwm);
}

void stopAll(Motor &motorA, Motor &motorB) {
  setMotorL298n(motorA, 0);
  setMotorL298n(motorB, 0);
}

void twoAxisDrive(L289n &driver, const int &xAxis, const int &yAxis) {
  // throttles come in at - 128 and 128
  setMotorL298n(*(driver.leftMotor), yAxis + xAxis);
  setMotorL298n(*(driver.rightMotor), yAxis - xAxis);
}
void twoAxisDrive(L289n &driver, int xAxis, int yAxis) {
  // throttles come in at - 128 and 128
  setMotorL298n(*(driver.leftMotor), yAxis + xAxis);
  setMotorL298n(*(driver.rightMotor), yAxis - xAxis);
}

void stopAll(L289n &driver) { stopAll(*driver.leftMotor, *driver.rightMotor); }
