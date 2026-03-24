
#include <Propellor/PrintStatusSubscriber.h>
#ifndef SIMULATION_SCREEN
#include <propeller.h>
#include <stdio.h>

void printTankStatus(struct TankStatus *ts) {
  if (ts->changeFlag) {
    printf("changeFlag");
    // printf("ts wheelL %c | ts wheelR %c| ts eulerY %i", ts->driveLeft,
    //       ts->driveRight, ts->eulerY);

    ts->changeFlag = 0;
  }
}
#endif
