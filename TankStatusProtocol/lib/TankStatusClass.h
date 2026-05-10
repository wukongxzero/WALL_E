#include "TankStatus.h"
extern "C" {
#include "TankStatus.h"
}

class TankStatusClass {
private:
  unsigned char buffer[TANKSTATUS_PACKET_LENGTH];

protected:
  struct TankStatus ts;

public:
  // Normal values, NOT pointers!
  unsigned char driveLeft;
  unsigned char driveRight;
  short eulerX;
  short eulerY;
  short eulerZ;
  bool changeFlag;

  float eulerXFloat;
  float eulerYFloat;
  float eulerZFloat;

  int packetLength;

  TankStatusClass();
  ~TankStatusClass();
  
  unsigned char *MakeIntoBytes();
  void BuildFromBytes(unsigned char *incoming_buffer);
};
