#include "TankStatus/ATankStatusPublisher.h"
#include "TankStatus/TankStatus.h"
#include <JoyStickPublisher.h>

#ifndef SIMULATION_SCREEN

#include <Propellor/adc.h>
#include <propeller.h>
#include <stdio.h>

#include <simpletools.h>

void calibrateCenter(struct JoyStickPublisher *self) {

  print("calibrate joystick");
  unsigned long sumX = 0;
  unsigned long sumY = 0;
  int samples = 64;

  // Take 16 readings and average them to find the true resting center
  for (int i = 0; i < samples; i++) {

    int xRead = (adc_in((int)self->_channelX));
    int yRead = (adc_in((int)self->_channelY));
    printf("reading{%i}{%i}\n", xRead, yRead);

    sumX += xRead;
    sumY += yRead;

    xRead = SHIFT_JOYSTICK_VALUE(xRead);
    yRead = SHIFT_JOYSTICK_VALUE(yRead);

    self->_rangeXHigh =
        MAX(xRead, self->_rangeXHigh) + AUTO_CALIBRATION_RANGE_THRESH;
    self->_rangeYHigh =
        MAX(yRead, self->_rangeYHigh) + AUTO_CALIBRATION_RANGE_THRESH;
    self->_rangeXLow =
        MIN(xRead, self->_rangeXLow) - AUTO_CALIBRATION_RANGE_THRESH;
    self->_rangeYLow =
        MIN(yRead, self->_rangeYLow) - AUTO_CALIBRATION_RANGE_THRESH;
  }

  self->_centerX = (sumX / samples);
  self->_centerY = (sumY / samples);

  self->trimX = self->_centerX - CENTER_JOYSTICK;
  self->trimY = self->_centerY - CENTER_JOYSTICK;

  // printf("calibrate joystick Center{%i}{%i}", self->trimX, self->trimY);
  // printf("calibrate joystick done{%i}{%i}", sumX / samples, sumY / samples);
}
// TODO:read joystick x and y and update
void readJoystick(struct JoyStickPublisher *self) {

  int xRead = adc_in((int)self->_channelX) - self->trimX;
  int yRead = adc_in((int)self->_channelY) - self->trimY;

  // (4096 * 255) / 4096 = 255
  applyDeadzone(&xRead, self->_rangeXHigh, self->_rangeXLow);
  applyDeadzone(&yRead, self->_rangeYHigh, self->_rangeYLow);

  unsigned char scaled_charX = (unsigned char)((xRead * 255) / 4096);
  unsigned char scaled_charY = (unsigned char)((yRead * 255) / 4096);

  // Note:I added trim after I forgot that I cleared the deadzone....
  //  so the reason for the self trim being here is that these would have been
  //  recorded before the -2046 trim and before its reduced to a 8 bit number
  //  the idea is to have the joystick read 0 when in the deadzone
  int xaxis = CLAMP_JOYSTICK(SHIFT_JOYSTICK_VALUE(xRead + self->trimX) /
                             ADC_RES_CHAR_SCALAR);
  int yaxis = CLAMP_JOYSTICK(SHIFT_JOYSTICK_VALUE(yRead + self->trimY) /
                             ADC_RES_CHAR_SCALAR);

  // y is throttle, x is differenrtial
  // Note: maybe I should decouple this... but itll be faster and more
  // performant doing this
  self->publisher->_localStatus.driveRight = CLAMP_JOYSTICK(scaled_charX);

  self->publisher->_localStatus.driveLeft = CLAMP_JOYSTICK(scaled_charY);

  // printf("xaxis %d|yaxis %d | driveX %d | driveY %d \n", scaled_charX,
  //        scaled_charY, xRead, yRead);
  //  notify(self->publisher);
}
#endif

void constructJoystick(struct JoyStickPublisher *self, unsigned char channelX,
                       unsigned char channelY,
                       struct TankStatusPublisher *localPublisher) {
  self->_channelX = channelX;
  self->_channelY = channelY;
  self->_xAxis = CENTER_JOYSTICK / ADC_RES_CHAR_SCALAR;
  self->_yAxis = CENTER_JOYSTICK / ADC_RES_CHAR_SCALAR;
  self->trimX = 0;
  self->trimY = 0;
  self->publisher = localPublisher;
  constructTankStatus(&(localPublisher->_localStatus));
}

void applyDeadzone(int *input, int rangeHigh, int rangeLow) {
  // use value of pointer and multiply the input by if its outside the deadzone
  // print("inputDeadzone %i", *input);
  if ((*input < rangeHigh || *input > rangeLow)) {
    *input = *input;
  } else {
    *input = CENTER_JOYSTICK;
  }
}
