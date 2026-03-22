#ifndef TANKSTATUS_SYNC_STRUCT
#define TANKSTATUS_SYNC_STRUCT

#define TANKSTATUS_PACKET_LENGTH                                               \
  16 // bytes,1 +1+4+4+4 + 2 bits for extra room/ keep as multiple of 8

// todo: rename file as from ITankStatus to tankStatus
struct TankStatus {
  volatile unsigned char driveLeft;  // = 0;  // 1 byte 0-255
  volatile unsigned char driveRight; // 1 byte 0-255

  // TODO: can optimize float into unsigned decimal point char and a signed char
  // for the non decimal portion range would be +/- 128.(0-255) degrees/radians
  // and only transmit as 2 bytes
  // radians would be more compressed for bit streamuing
  volatile float eulerX; // 4 byte 32 bit
  volatile float eulerY; // 4 byte
  volatile float eulerZ; // 4 byte
  volatile int lockID;
  // unsigned char isLocked; // adding a pthreads equivelent of a thread lock
  // int lockID;
  //  (mutex lock) based on ai telling me to lol.
  //  I should not have a problem unless I use multiple publishers
};
void constructTankStatus(struct TankStatus *self);
// TODO: should turn all chars into one char byte array as a buffer to be
// streamed over usart
void makeByteTankStatus(unsigned char *buffer, int byteLength,
                        struct TankStatus *ts);

// TODO: should take a buffer array and package it into a TankStatus structure
void readByteTankStatus(unsigned char *buffer, int byteLength,
                        struct TankStatus *ts);
#endif
