#include "TankStatusClass.h"
#ifndef ARDUINO
#include <cstring>
#else
#include <Arduino.h>
#endif


// Constructor
TankStatusClass::TankStatusClass() {
  // Initialize the internal C struct
  this->ts.driveLeft = 0;
  this->ts.driveRight = 0;
  this->ts.eulerX = 0;
  this->ts.eulerY = 0;
  this->ts.eulerZ = 0;
  this->ts.changeFlag = 0;

  // Sync UP to the class variables
  this->driveLeft = this->ts.driveLeft;
  this->driveRight = this->ts.driveRight;
  this->eulerX = this->ts.eulerX;
  this->eulerY = this->ts.eulerY;
  this->eulerZ = this->ts.eulerZ;
  this->changeFlag = this->ts.changeFlag;

  this->eulerXFloat = 0.0f;
  this->eulerYFloat = 0.0f;
  this->eulerZFloat = 0.0f;
  
  this->packetLength = TANKSTATUS_PACKET_LENGTH;
}

TankStatusClass::~TankStatusClass() {}

unsigned char *TankStatusClass::MakeIntoBytes() {
  // 1. Sync wrapper variables DOWN into the C struct
  this->ts.driveLeft = this->driveLeft;
  this->ts.driveRight = this->driveRight;
  this->ts.changeFlag = this->changeFlag;

  // Encode floats to shorts
  this->ts.eulerX = ENCODE_SHORT(this->eulerXFloat);
  this->ts.eulerY = ENCODE_SHORT(this->eulerYFloat);
  this->ts.eulerZ = ENCODE_SHORT(this->eulerZFloat);

  // 2. Serialize to bytes
  makeByteTankStatus(this->buffer, TANKSTATUS_PACKET_LENGTH, &this->ts);
  return this->buffer;
}

void TankStatusClass::BuildFromBytes(unsigned char *incoming_buffer) {
  // 1. Parse bytes into the C struct
  memcpy(this->buffer, incoming_buffer, TANKSTATUS_PACKET_LENGTH);
  readByteTankStatus(this->buffer, TANKSTATUS_PACKET_LENGTH, &this->ts);

  // 2. Sync the C struct UP to wrapper variables
  this->driveLeft = this->ts.driveLeft;
  this->driveRight = this->ts.driveRight;
  this->changeFlag = this->ts.changeFlag;

  // Decode shorts back to floats
  this->eulerXFloat = DECODE_SHORT(this->ts.eulerX);
  this->eulerYFloat = DECODE_SHORT(this->ts.eulerY);
  this->eulerZFloat = DECODE_SHORT(this->ts.eulerZ);
}
