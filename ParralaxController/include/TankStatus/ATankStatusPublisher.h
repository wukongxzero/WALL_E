#pragma once

#include "ITankStatus.h"
#include "ITankStatusSubscriber.h"
class ATankStatusPublisher {
private:
  ITankStatusSubscriber *tsSubuscriberList[2];

public:
  volatile TankStatus originalTankStatus;

public:
  void notify();
  void subscribe(ITankStatusSubscriber &tsSubuscriber);
};
