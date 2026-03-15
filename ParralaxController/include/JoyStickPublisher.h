#pragma once
#include "TankStatus/ATankStatusPublisher.h"

class AJoyStickPublisher : public ATankStatusPublisher {
private:
  int _xAxis;
  int _yAxis;
  unsigned char _pinX;
  unsigned char pinY;

  void applyDeadzone(int &inputIO);

public:
  AJoyStickPublisher(const int &joystickPortHorizontal,
                     const int &joystickPortverital);
  // Assume propellor specific can make this class generic if need to port
  // to arduino
  void read();
  int getXStick();
  int getYStick();
};
