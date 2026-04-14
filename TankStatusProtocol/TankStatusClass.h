
extern "C" {
#include "lib/TankStatus.h"
}

class TankStatusClass {
private:
  unsigned char buffer[TANKSTATUS_PACKET_LENGTH];

protected:
  struct TankStatus ts;

public:
  float eulerXFloat = 0;
  float eulerYFloat = 0;
  float eulerZFloat = 0;

  // wrapping for python
  volatile unsigned char *driveLeft;
  volatile unsigned char *driveRight;
  volatile short *eulerX;
  volatile short *eulerY;
  volatile short *eulerZ;
  volatile bool *changeFlag;
  int packetLength = TANKSTATUS_PACKET_LENGTH;

  TankStatusClass();
  ~TankStatusClass();
  unsigned char *MakeIntoBytes();
  void BuildFromBytes(unsigned char *buffer);
};
