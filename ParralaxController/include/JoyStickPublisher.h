// #pragma once
// #include "TankStatus/ATankStatusPublisher.h"
//
// class AJoyStickPublisher : public ATankStatusPublisher {
// private:
//   int _xAxis;
//   int _yAxis;
//   unsigned char _pinX;
//   unsigned char pinY;
//
//   void applyDeadzone(int &inputIO);
//
// public:
//   AJoyStickPublisher(const int &joystickPortHorizontal,
//                      const int &joystikPortverital);
//   // Assume propellor specific can make this class generic if need to port
//   // to arduino
//   void read();
//   int getXStick();
//   int getYStick();
// };
#ifndef PARALLAX_ADC_JOYSTICK
#define PARALLAX_ADC_JOYSTICK

#define MAX_JOYSTICK 254
#define MIN_JOYSTICK 2
#define ADC_RES_CHAR_SCALAR 16 // use after process
// TODO:refine threshold
#define AUTO_CALIBRATION_RANGE_THRESH 1

#define CENTER_JOYSTICK 4094 / 2
#define SHIFT_JOYSTICK_VALUE(a) (a - CENTER_JOYSTICK)
#define CLAMP_JOYSTICK(a) (MAX(MIN(a, MAX_JOYSTICK), MIN_JOYSTICK))

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

// this is kind of weird but imagine we are inheriting a publisher only now we
// use composition to spoof inheritance/bridge patterns
// tldr: TankStatusPublisher is a structure we update from the joystick
// publisher
#include "TankStatus/ATankStatusPublisher.h"

struct JoyStickPublisher {
  // call Notify() found in TankStatusPublisher on this struct
  //
  // this structure contains the TankStatus sync state needed for streaming
  struct TankStatusPublisher *publisher;

  unsigned char _xAxis;
  unsigned char _yAxis;

  unsigned char _pinX; // thinking about using to shut joystick off
  unsigned char _pinY;

  int _centerX;
  int _centerY;

  int trimX;
  int trimY;

  int _rangeXLow;
  int _rangeYLow;
  int _rangeXHigh;
  int _rangeYHigh;

  unsigned char _channelX; // Paralax specific but doesn't hurt to have
  unsigned char _channelY; // Paralax specific but doesn't hurt to have
  //
};

void calibrateCenter(struct JoyStickPublisher *self);
void constructJoystick(struct JoyStickPublisher *self, unsigned char channelX,
                       unsigned char channelY,
                       struct TankStatusPublisher *localPublisher);

// TODO: see arduino calculation
void applyDeadzone(int *input, int rangeHigh, int rangeLow);

// TODO:read joystick x and y and update
void readJoystick(struct JoyStickPublisher *self);

#endif
