#include <TankStatus/ATankStatusPublisher.h>
#include <TankStatus/TankStatus.h>
#include <string.h>

#ifndef SIMULATION_SCREEN
#include <propeller.h>

#endif

void constructTankStatusPublisher(struct TankStatusPublisher *self) {
  self->subscriberCount = 0;
  constructTankStatus(&(self->_localStatus));
}

void notify(struct TankStatusPublisher *self) {

#ifndef SIMULATION_SCREEN_
  /* while (__builtin_propeller_lockset(self->lockID)) {
     // Spin/Wait for our turn in the Hub rotation
   }
   */
#endif
  // TODO: implement clang mutex pthreads lock
  for (int i = 0; i < self->subscriberCount; i++) {
    memcpy((void *)self->tankStatusSubscriberList[i],
           (const void *)&(self->_localStatus), sizeof(struct TankStatus));
    self->tankStatusSubscriberList[i]->changeFlag = 1;
  }

#ifndef SIMULATION_SCREEN
  //__builtin_propeller_lockclr(self->lockID);
#endif
}

void subscribe(struct TankStatusPublisher *self,
               struct TankStatus *subscriberPtr) {
  self->tankStatusSubscriberList[self->subscriberCount] = subscriberPtr;
  self->subscriberCount++;
}
