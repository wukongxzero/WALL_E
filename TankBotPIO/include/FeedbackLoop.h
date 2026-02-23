#pragma once

#include "config.h"
unsigned long lastIMUControllerTime = 0;

template <typename T>
void PorportionalControl(T &error, T &output, unsigned long &deltaTime) {
  static T output_1 = 0;
  static T error_1 = 0;
  // zero order hold??
  output = ((error) + KI * error_1 + KD * output_1) / KP; // *
  //(static_cast<T>(deltaTime)) / 1000;
  output_1 = output;
  error_1 = error;
}
