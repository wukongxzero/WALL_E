#pragma once
#include "ITankStatus.h"

class ITankStatusSubscriber {
public:
  // locally stored tank status
  TankStatus tankStatusSync;

private:
};
