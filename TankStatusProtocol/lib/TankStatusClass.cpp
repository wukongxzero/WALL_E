#include "TankStatusClass.h"
#include "TankStatus.h"

#include <cstring> // Required for memcpy

TankStatusClass::TankStatusClass() {
  this->ts.driveLeft = 0;
  this->ts.driveRight = 0;
  this->ts.eulerX = 0;
  this->ts.eulerY = 0;
  this->ts.eulerZ = 0;
  this->changeFlag = 0;
  this->driveLeft = &ts.driveLeft;
  this->driveRight = &ts.driveRight;
  this->eulerX = &ts.eulerX;
  this->eulerY = &ts.eulerY;
  this->eulerZ = &ts.eulerZ;
  this->changeFlag = (volatile bool *)&this->ts.changeFlag;
}
TankStatusClass::~TankStatusClass() {}
unsigned char *TankStatusClass::MakeIntoBytes() {
  this->ts.eulerX = ENCODE_SHORT(this->eulerXFloat);
  this->ts.eulerY = ENCODE_SHORT(this->eulerYFloat);
  this->ts.eulerZ = ENCODE_SHORT(this->eulerZFloat);
  makeByteTankStatus(this->buffer, TANKSTATUS_PACKET_LENGTH, &this->ts);
  return this->buffer;
}

void TankStatusClass::BuildFromBytes(unsigned char *incoming_buffer) {
  memcpy(buffer, incoming_buffer, TANKSTATUS_PACKET_LENGTH);
  readByteTankStatus(this->buffer, TANKSTATUS_PACKET_LENGTH, &this->ts);
  // 2. SYNC (Decode): Pull the C struct data back out into the Python floats
  this->eulerXFloat = DECODE_SHORT(this->ts.eulerX);
  this->eulerYFloat = DECODE_SHORT(this->ts.eulerY);
  this->eulerZFloat = DECODE_SHORT(this->ts.eulerZ);
}
