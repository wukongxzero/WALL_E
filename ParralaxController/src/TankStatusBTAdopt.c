#ifndef SIMULATION_SCREEN
#include "Propellor/hc05.h"
#include "TankStatus/ATankStatusPublisher.h"
#include <TankStatus/TankStatus.h>
#include <TankStatusBTAdopt.h>
#include <propeller.h>
#include <simpletools.h>
void constructTankStatusBTAdopter(TankStatusBTAdopter *self,
                                  struct TankStatus *ts,
                                  struct TankStatusPublisher *tsPub,
                                  hc05_t *bt) {
  self->publisher = tsPub;
  self->subscriberPart = ts;
  self->bitCountElement = 0;
  // dont forget to init this
  self->btInterface = bt;
  if (hc05_start_rx_cog(self->btInterface) == -1) {
    while (1)
      ;
  }
}

void switchBaudrate(int newbaud) {
  // Put this in a temporary main() function and run it ONCE
  // hc05_init(&bt, RX_PIN, TX_PIN, EN_PIN, 38400); // AT mode is always 38400
  // baud
  // hc05_set_at_mode(&bt, 1);

  // For HC-05:
  // hc05_send_command(&bt, "UART=115200,0,0");

  // Note: If you actually have an HC-06 (as one of your old comments
  // suggested), the command might be AT+BAUD8 instead.

  // hc05_set_at_mode(&bt, 0);
}

// void readTankStatusBT(TankStatusBTAdopter *self) {
void readTankStatusBT(void *arg) {
  TankStatusBTAdopter *self = (TankStatusBTAdopter *)arg;
  waitcnt(CNT + CLKFREQ);
  // print("Cog %d: Reporting for duty!\n", 2);
  while (1) {

    // Fast, non-blocking check of the circular buffer
    int incoming = hc05_rx_check(self->btInterface);

    if (incoming != -1) {
      unsigned char c = (unsigned char)incoming;
      self->bufferRead[self->bitCountElement] = c;
      self->bitCountElement++;
    }
    if (self->bitCountElement == TANKSTATUS_PACKET_LENGTH) {

      readByteTankStatus(self->bufferRead, TANKSTATUS_PACKET_LENGTH,
                         &(self->publisher->_localStatus));
      notify(self->publisher);

      self->bitCountElement = 0;
    }

  } // write tank status
}
void writeTankStatusBT(TankStatusBTAdopter *self) {}
#endif
