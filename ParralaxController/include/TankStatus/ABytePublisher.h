
#ifndef USART_BYTE_PUBLISHER
#define USART_BYTE_PUBLISHER

#include "ATankStatusPublisher.h"
#include "BTUSART.h"
#include "TankStatus.h"
#include <simpletools.h>
// BT hc-06
// https://www.parallax.com/multiple-serial-port-16-object/
struct UsartBytePublisher {
  // TODO: add USART Hand
  // putchar and getchar are used to call placement of char into usart(pin 31)
  PropellorUSART *btInterface;
  // loop over TankStatusPublisher->TankStatisSubscriberList to notify all
  // subscriber
  TankStatusPublisher *ptrWithSubscriberList;

  // store serial read usart data into this struct which will be used to notify
  TankStatus buffer;
};

// TODO:take in a message Struct refrence and update all the
// TankStatusSubscribers' TanksStatus pointers to update to the messageStruct
// input
void notify(UsartBytePublisher *self, TankStatus &messageStruct);

// TODO:Update the tankstatus buffer using the readByteTankStatus() method after
// gathering getchar() data from the btInterface
void readInterface(UsartBytePublisher *self);

#endif
