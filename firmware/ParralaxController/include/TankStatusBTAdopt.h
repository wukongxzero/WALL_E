#ifndef TANKSTATUS_BT_PACKAGE_H
#define TANKSTATUS_BT_PACKAGE_H
#ifndef SIMULATION_SCREEN
#include <Propellor/hc05.h>
#include <TankStatus/ATankStatusPublisher.h>
#include <TankStatus/TankStatus.h>

// use TankStatus length for total bit size

typedef struct {
  struct TankStatus *subscriberPart;
  struct TankStatusPublisher *publisher;

  char bufferRead[TANKSTATUS_PACKET_LENGTH];
  char bufferWrite[TANKSTATUS_PACKET_LENGTH];
  hc05_t *btInterface;

  unsigned char bitCountElement;

} TankStatusBTAdopter;
// wait for recv on cog and pupulate pointer tank status... wont do as a
// publisher so I can reuse what I can for aruduino need to listen for
// incloiming byte,m and recycle everytime
void constructTankStatusBTAdopter(TankStatusBTAdopter *self,
                                  struct TankStatus *ts,
                                  struct TankStatusPublisher *tsPub,
                                  hc05_t *bt);
// untested
// should probably relocate this function to hc05.h lib
void switchBaudrate(hc05_t *bt, int newbaud);

// modifying so blocking happens on a seperate cog
void readTankStatusBT(void *arg);
// write tank status
void writeTankStatusBT(TankStatusBTAdopter *self);
#endif
#endif
