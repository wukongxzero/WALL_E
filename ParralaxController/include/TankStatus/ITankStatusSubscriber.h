#pragma once
#include <TankStatus/ITankStatus.h>

class ITankStatusSubscriber {
public:
  // locally stored tank status
  //
  struct TankStatus tankStatusSync;

private:
};
