#include <TankStatus/TankStatus.h>
#include <string.h> // Required for memcpy

/**
 * Serializes the TankStatus struct into a byte buffer for USART streaming.
 */
void constructTankStatus(struct TankStatus *self) {
  self->driveLeft = 0;
  self->driveRight = 0;
  self->eulerX = 0;
  self->eulerY = 0;
  self->eulerZ = 90;
  self->changeFlag = 0;
  // self->isLocked = 0;
}

void makeByteTankStatus(unsigned char *buffer, int byteLength,
                        struct TankStatus *ts) {
  // Safety check: ensure buffer is large enough
  if (byteLength < TANKSTATUS_PACKET_LENGTH || buffer == NULL || ts == NULL) {
    return;
  }

  int offset = 0;

  memcpy(buffer + offset, (const void *)&ts->driveLeft, sizeof(ts->driveLeft));
  offset += sizeof(ts->driveLeft);

  memcpy(buffer + offset, (const void *)&ts->driveRight,
         sizeof(ts->driveRight));
  offset += sizeof(ts->driveRight);

  // Padding REMOVED. Packing shoulder-to-shoulder!

  memcpy(buffer + offset, (const void *)&ts->eulerX, sizeof(ts->eulerX));
  offset += sizeof(ts->eulerX);

  memcpy(buffer + offset, (const void *)&ts->eulerY, sizeof(ts->eulerY));
  offset += sizeof(ts->eulerY);

  memcpy(buffer + offset, (const void *)&ts->eulerZ, sizeof(ts->eulerZ));
}

void readByteTankStatus(unsigned char *buffer, int byteLength,
                        struct TankStatus *ts) {
  // Safety check
  if (byteLength < TANKSTATUS_PACKET_LENGTH || buffer == NULL || ts == NULL) {
    return;
  }

  int offset = 0;

  memcpy((void *)&ts->driveLeft, buffer + offset, sizeof(ts->driveLeft));
  offset += sizeof(ts->driveLeft);

  memcpy((void *)&ts->driveRight, buffer + offset, sizeof(ts->driveRight));
  offset += sizeof(ts->driveRight);

  // Padding REMOVED.

  memcpy((void *)&ts->eulerX, buffer + offset, sizeof(ts->eulerX));
  offset += sizeof(ts->eulerX);

  memcpy((void *)&ts->eulerY, buffer + offset, sizeof(ts->eulerY));
  offset += sizeof(ts->eulerY);

  memcpy((void *)&ts->eulerZ, buffer + offset, sizeof(ts->eulerZ));
}
/*
float getEulerfloat(short data) {
  // data should be a byte FF where first is the char, second half is the dec
  // float dec = data & 0xFF >> 8; // decimal portion
  float dec = (data & 0xFF) / 256.0f;
  float dataOut = data >> 8; // integer portion
  return dataOut + dec;
}

int makeEulerInt(float data);
*/
