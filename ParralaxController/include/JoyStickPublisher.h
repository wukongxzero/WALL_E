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

// this is kind of weird but imagine we are inheriting a publisher only now we
// use composition to spoof inheritance/bridge patterns
// tldr: TankStatusPublisher is a structure we update from the joystick
// publisher
#include "TankStatus/ATankStatusPublisher.h"

struct JoyStickPublisher {
  // call Notify() found in TankStatusPublisher on this struct
  //
  // this structure contains the TankStatus sync state needed for streaming
  TankStatusPublisher *publisher;

  unsigned char _xAxis;
  unsigned char _yAxis;
  unsigned char _pinX;
  unsigned char _pinY;
};

// TODO: see arduino calculation
void applyDeadzoneHorizontal(JoyStickPublisher *self);
void applyDeadzoneVertical(JoyStickPublisher *self);

// TODO:read joystick x and y and update
void read(JoyStickPublisher *self);
int getXStick(JoyStickPublisher *self);
int getYStick(JoyStickPublisher *self);

#endif
