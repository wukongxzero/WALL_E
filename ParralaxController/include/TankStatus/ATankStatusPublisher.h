#ifndef TANKSTATUS_PUBLISH
#define TANKSTATUS_PUBLISH

#include "TankStatus.h"
//
// class ATankStatusPublisher {
// private:
//  class ITankStatusSubscriber *tsSubuscriberList[2];
//
// public:
//  volatile struct TankStatus originalTankStatus;
//
// public:
//  void notify();
//  void subscribe(class ITankStatusSubscriber &tsSubuscriber);
//};
//
struct TankStatusPublisher {
  // populate via pub/sub
  TankStatus *tankStatusSubscriberList[8];
  int subscriberCount = 0;
};

// TODO:add the other object's subscriber pointer directly to this publisher so
// it can act as a managing object, increment subscriberCount by 1
void notify(TankStatusPublisher *self, TankStatus *subscriberPtr);

#endif // !TANKSTATUS_PUBLISH
