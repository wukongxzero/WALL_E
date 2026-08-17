#pragma once
// I know this probably won't make it over to C so not using the header guards
// like in the other files
//
#include "L829n.h"
#include "PS2X_lib.h"

struct ControllerMotorMap {
  PS2X *ps2Input;
  L289n *driverOutput;
  volatile int ps2ErrorStatus = 1;
};

void tryConnectPS2(ControllerMotorMap &ps2ToWheels);

// void processPS2Inputs(ControllerMotorMap *ps2ToWheels);

void processPS2Inputs(ControllerMotorMap &ps2ToWheels);
