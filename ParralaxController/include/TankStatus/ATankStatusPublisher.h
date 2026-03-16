#pragma once

#include "ITankStatus.h"
#include <TankStatus/ITankStatusSubscriber.h>

class ATankStatusPublisher {
private:
  class ITankStatusSubscriber *tsSubuscriberList[2];

public:
  volatile struct TankStatus originalTankStatus;

public:
  void notify();
  void subscribe(class ITankStatusSubscriber &tsSubuscriber);
};
