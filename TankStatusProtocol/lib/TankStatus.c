#include "TankStatus.h"
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
// note: memcopy ignores the locked boolean/byte
void makeByteTankStatus(unsigned char *buffer, int byteLength,
                        struct TankStatus *ts) {
  // Safety check: ensure buffer is large enough
  if (byteLength < TANKSTATUS_PACKET_LENGTH || buffer == NULL || ts == NULL) {
    return;
  }

  int offset = 0;

  // Copying field by field ensures we control the exact byte order
  memcpy(buffer + offset, (const void *)&ts->driveLeft, sizeof(ts->driveLeft));
  offset += sizeof(ts->driveLeft);

  memcpy(buffer + offset, (const void *)&ts->driveRight,
         sizeof(ts->driveRight));
  offset += sizeof(ts->driveRight);

  // Skip 2 bytes of padding if your hardware requires 4-byte alignment for
  // floats This aligns with your #define TANKSTATUS_PACKET_LENGTH 16
  offset += 2;

  memcpy(buffer + offset, (const void *)&ts->eulerX, sizeof(ts->eulerX));
  offset += sizeof(ts->eulerX);

  memcpy(buffer + offset, (const void *)&ts->eulerY, sizeof(ts->eulerY));
  offset += sizeof(ts->eulerY);

  memcpy(buffer + offset, (const void *)&ts->eulerZ, sizeof(ts->eulerZ));
}

/**
 * Deserializes a byte sequence from a buffer back into a TankStatus struct.
 */
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

  // Skip the 2 bytes of padding used to reach the 16-byte alignment
  offset += 2;

  memcpy((void *)&ts->eulerX, buffer + offset, sizeof(ts->eulerX));
  offset += sizeof(ts->eulerX);

  memcpy((void *)&ts->eulerY, buffer + offset, sizeof(ts->eulerY));
  offset += sizeof(ts->eulerY);

  memcpy((void *)&ts->eulerZ, buffer + offset, sizeof(ts->eulerZ));
}
