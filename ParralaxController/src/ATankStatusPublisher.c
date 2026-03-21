#include <TankStatus/ATankStatusPublisher.h>
#include <TankStatus/TankStatus.h>
#include <string.h>
#ifdef _PROPELLER_H
#include <propellor.h>

#endif

void constructTankStatusPublisher(struct TankStatusPublisher *self) {
  self->subscriberCount = 0;
  constructTankStatus(&(self->_localStatus));
}

void notify(struct TankStatusPublisher *self) {

#ifdef _PROPELLER_H_
  //  while (__builtin_propeller_lockset(self->lockID)) {
  // Spin/Wait for our turn in the Hub rotation
  //}
#endif
  // TODO: implement clang mutex pthreads lock
  for (int i = 0; i < self->subscriberCount; i++) {
    memcpy((void *)self->tankStatusSubscriberList[i],
           (const void *)&(self->_localStatus), sizeof(struct TankStatus));
  }

#ifdef _PROPELLER_H_
  //__builtin_propeller_lockclr(self->lockID);
#endif
}

void subscribe(struct TankStatusPublisher *self,
               struct TankStatus *subscriberPtr) {
  self->tankStatusSubscriberList[self->subscriberCount] = subscriberPtr;
  self->subscriberCount++;
}
