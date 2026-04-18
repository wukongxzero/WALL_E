#ifndef TANKSTATUS_SYNC_STRUCT
#define TANKSTATUS_SYNC_STRUCT

#define TANKSTATUS_PACKET_LENGTH                                               \
  16 // bytes,1 +1+4+4+4 + 2 bits for extra room/ keep as multiple of 8, adding
     // 8 extra bytes of padding
// floating point arithmitic
#define DECODE_SHORT(data) ((float)(data) / 256.0f)
#define ENCODE_SHORT(reading) ((short)((reading) * 256.0f))

// todo: rename file as from ITankStatus to tankStatus
struct TankStatus {
  volatile unsigned char driveLeft;  // = 0;  // 1 byte 0-255
  volatile unsigned char driveRight; // 1 byte 0-255
  volatile short eulerX;             // 2 byte 32 bit
  volatile short eulerY;             // 2 byte
  volatile short eulerZ;             // 2 byte
  volatile unsigned char changeFlag;
};
void constructTankStatus(struct TankStatus *self);
// TODO: should turn all chars into one char byte array as a buffer to be
// streamed over usart
void makeByteTankStatus(unsigned char *buffer, int byteLength,
                        struct TankStatus *ts);

// TODO: should take a buffer array and package it into a TankStatus structure
void readByteTankStatus(unsigned char *buffer, int byteLength,
                        struct TankStatus *ts);

int get_tankstatus_packet_length();
float getDecodedShortToFloat(short data);
short getEncodedFloatToShort(float reading);

#endif
