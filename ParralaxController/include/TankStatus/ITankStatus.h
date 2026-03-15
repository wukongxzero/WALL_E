#pragma once

// todo: rename file as from ITankStatus to tankStatus
struct TankStatus {
  volatile int driveLeft;
  volatile int driveRight;

  volatile int eulerX;
  volatile int eulerY;
  volatile int eulerZ;
};

void makeByteTankStatus(unsigned char *buffer, int byteLength,
                        struct TankStatus *ts);

void readByteTankeStatus(unsigned char *buffer, int byteLength,
                         struct TankStatus *ts);
