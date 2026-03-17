// TODO: move this file to the Propellor folder
#ifndef BT_USART
#define BT_USART
#include "TankStatus.h"
#include <simpletools.h>
// BT hc-06
// https://www.parallax.com/multiple-serial-port-16-object/
struct PropellorUSART {
  // TODO: add USART Hand
  // puitchar and getchar are used to call placement of char into usart(pin 31)
  // TODO: figure out pins and stuff I think defaulted to 30 and 31 for usart
  // wityh cogs
  //
  unsigned char rx;
  unsigned char tx;

  unsigned char bufferOut[TANKSTATUS_PACKET_LENGTH];
  unsigned char bufferIn[TANKSTATUS_PACKET_LENGTH];
};

// naming self becuase i like self from python
void write(struct PropellorUSART *self);
void read(struct PropellorUSART *self);

#endif
