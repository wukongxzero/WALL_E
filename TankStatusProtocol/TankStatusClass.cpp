#include "TankStatusClass.h"
#include "lib/TankStatus.h"

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
}
TankStatusClass::~TankStatusClass() {}
unsigned char *TankStatusClass::MakeIntoBytes() {
  makeByteTankStatus(this->buffer, TANKSTATUS_PACKET_LENGTH, &this->ts);
  return this->buffer;
}

void TankStatusClass::BuildFromBytes(unsigned char *incoming_buffer) {
  memcpy(&buffer, incoming_buffer, TANKSTATUS_PACKET_LENGTH);
  readByteTankStatus(this->buffer, TANKSTATUS_PACKET_LENGTH, &this->ts);
}
