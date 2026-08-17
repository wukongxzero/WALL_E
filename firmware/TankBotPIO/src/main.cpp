#include "Arduino.h"
#include "L829n.h"
#include "Mpu6050.h"
#include "PS2Rcv.h"
#include "ServoTimer2.h"
#include "TimerInturruptSetup.h"
#include "config.h"
#include <FeedbackLoop.h>
#include <PS2X_lib.h>
#include <Wire.h>
#include <ps2helper.h>
// timer inturupt
template <typename TArg>
bool setInterval(unsigned long interval, void (*callback)(TArg), TArg params,
                 unsigned long duration = 0);

#include "TimerInterrupt.h"

ServoTimer2 platformServo;
IMU platformIMU;
struct Motor tankWheelL;
struct Motor tankWheelR;
struct L289n tankDrive;
PS2X ps2;
struct ControllerMotorMap ps2Mapping;

void TimerHandler5(void);
void tryConnectPS2(ControllerMotorMap &ps2ToWheels);
void twoAxisDrive(L289n &driver, volatile int &xAxis, volatile int &yAxis);
template <typename T>
void PorportionalControl(T &error, T &output, unsigned long &deltaTime);

void setup() {
  Serial.begin(115200);
  delay(800);

  // I2C (MEGA uses SDA=20, SCL=21)
  Wire.begin();
  Wire.setClock(400000);
  mpuInit();

  // Servo
  platformServo.attach(SERVO_PIN);
  platformServo.write(SERVO_CENTER_US);

  // PS2
  ps2 = PS2X();
  // Motor pins
  // TODO:will optimize by port manip there are 6 vals on one register
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  tankWheelL.in1 = IN1;
  tankWheelL.in2 = IN2;
  tankWheelL.ena = ENA;
  tankWheelR.in1 = IN3;
  tankWheelR.in2 = IN4;
  tankWheelR.ena = ENB;

  tankDrive.leftMotor = &tankWheelL;
  tankDrive.rightMotor = &tankWheelR;
  stopAll(tankWheelL, tankWheelR);
  ps2Mapping.ps2Input = &ps2;
  ps2Mapping.driverOutput = &tankDrive;
  ps2Mapping.ps2ErrorStatus = 1;

  while (ps2Mapping.ps2ErrorStatus == 1) {
    tryConnectPS2(ps2Mapping);
    delay(500);
  }

  platformIMU.lastImuUs = micros();

  ITimer5.init();
  if (ITimer5.attachInterruptInterval(PS2_MS, TimerHandler5)) {
    Serial.print(F("Starting  ITimer5 OK, millis() = "));
    Serial.println(millis());

#if (TIMER_INTERRUPT_DEBUG > 1)
    Serial.print(F("OutputPin1 = "));
    Serial.print(outputPin1);
    Serial.print(F(" address: "));
    Serial.println((uint32_t)&outputPin1);
#endif
  } else
    Serial.println(F("Can't set ITimer5. Select another freq. or timer"));

  Serial.println("READY: PS2+Motors+IMU+Servo");
}

void loop() {

  static float output = SERVO_CENTER_US;
  unsigned long nowUs = millis();
  if ((uint32_t)(nowUs - lastIMUControllerTime) >= IMU_US) {

    readImu(platformIMU);

    // scale degree to servoMicroSecondSignal
    float microsecDegInput =
        map(platformIMU.pitchDeg, -90, 90, SERVO_CENTER_US - SERVO_RANGE_US,
            SERVO_CENTER_US + SERVO_RANGE_US);

    float error = -(microsecDegInput - SERVO_CENTER_US);
    unsigned long deltaT = nowUs - lastIMUControllerTime;

    PorportionalControl<float>(error, output, deltaT);

    platformServo.write(static_cast<int>(SERVO_CENTER_US + output));
  }
  static uint32_t lastDbgMs = 0;
  if ((uint32_t)(millis() - lastDbgMs) >= DBG_MS * 10) {
    lastDbgMs = millis();
    Serial.print("pitchDeg");
    Serial.print(platformIMU.pitchDeg);
    Serial.print("servoMicrosecondsOut: ");
    Serial.println(SERVO_CENTER_US + output);
  }
}

void TimerHandler5(void) {
  // Serial.println("timer trigger");

  if (ps2Mapping.ps2ErrorStatus == 1) {
    tryConnectPS2(ps2Mapping);
  }
  processPS2Inputs(ps2Mapping);
}
