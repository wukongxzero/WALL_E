#include "PS2Rcv.h"
#include "L829n.h"
#include "config.h"
#include <PS2X_lib.h>
#include <ps2helper.h>

void tryConnectPS2(ControllerMotorMap &ps2ToWheels) {
  ps2ToWheels.ps2ErrorStatus = ps2ToWheels.ps2Input->config_gamepad(
      PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, false, false);

  Serial.print("PS2 error = ");
  Serial.println(ps2ToWheels.ps2ErrorStatus);
  if (ps2ToWheels.ps2ErrorStatus == 0)
    Serial.println("✅ Controller connected");
  else
    Serial.println("❌ Controller NOT connected");
}

void processPS2Inputs(ControllerMotorMap &ps2ToWheels) {
  // Serial.println("process begin");
  ps2ToWheels.ps2ErrorStatus = ps2ToWheels.ps2Input->config_gamepad(
      PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, false, false);

  int rawThrDrive = ps2ToWheels.ps2Input->Analog(PSS_LY);
  int rawThrDiff = ps2ToWheels.ps2Input->Analog(PSS_LX);

  int rawTurn = ps2ToWheels.ps2Input->Analog(PSS_RX);

  bool looksBad = (rawThrDrive <= 0 || rawThrDrive >= 255) &&
                  (rawTurn <= 0 || rawTurn >= 255) &&
                  (rawThrDiff <= 0 || rawThrDrive >= 255);
  if (looksBad) {
    // stopAll(ps2ToWheels.driverOutput->leftMotor,
    //         ps2ToWheels.driverOutput->rightMotor);
    Serial.print("looksbad");
    Serial.println(looksBad);
    stopAll(*ps2ToWheels.driverOutput);
    return;
  }

  int throttle = -(rawThrDrive - 128);    // up = forward
  int throttleDiff = -(rawThrDiff - 128); // up = forward
  int turn = (rawTurn - 128);             // right = turn right

  throttle = applyDeadzone(throttle, DEADZONE);
  throttleDiff = applyDeadzone(throttleDiff, DEADZONE);
  turn = applyDeadzone(turn, DEADZONE);

  // int throttle255 = clamp<int>(throttle * 2, -255, 255);
  // int turn255 = clampInt(turn * 2, -255, 255);

  int throttle255 = (max(min(throttle * 2, -255), 255));
  int throttleDiff255 = (max(min(throttle * 2, -255), 255));
  int turn255 = (max(min(throttle * 2, -255), 255));

  // int leftCmd = clampInt(throttle255 + turn255, -255, 255);
  // int rightCmd = clampInt(throttle255 - turn255, -255, 255);
  int leftCmd = max(min(throttle255 + turn255, -255), 255);
  int rightCmd = max(min(throttle255 - turn255, -255), 255);

  // START = emergency stop
  if (ps2ToWheels.ps2Input->Button(PSB_START)) {
    leftCmd = 0;
    rightCmd = 0;
  }

  // ps2ToWheels.xAxis = rampToward(leftOut, leftCmd, RAMP_STEP);
  // ps2ToWheels.yAxis = rampToward(rightOut, rightCmd, RAMP_STEP);

  // twoAxisDrive(*ps2ToWheels.driverOutput, 255, 255);
  twoAxisDrive(*ps2ToWheels.driverOutput, throttleDiff255 - 0, throttle255 - 0);
  static uint32_t lastDbgMs = 0;
  if ((uint32_t)(millis() - lastDbgMs) >= DBG_MS) {
    lastDbgMs = millis();
    Serial.print(" | Thr=");
    Serial.print(rawThrDrive);
    Serial.print(" | ThrDif=");
    Serial.println(rawThrDiff);
    Serial.print(" | Turn=");
    Serial.println(rawTurn);
  }
}
