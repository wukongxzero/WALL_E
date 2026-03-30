#ifndef ASYNC_GRAPHIC_h
#define ASYNC_GRAPHIC_h

#include "Graphics/RotatePixelArt.h"
#include <TankStatus/TankStatus.h>
#include <wchar.h>
#define ANIMATION_DRIVE_SCALAR 10
//_localSubscriber here are inspired by a flywheel pattern...but its not really
// that

// assign a CoreMapping to spoof inheritance while also saving data
// basically two mappings can also share the same pointer to the original
// TODO:Potentially can use core map as element in render queue
// NOTE: core mapping's purpose should serve as a way to share all original data
// not changing like rotation data
struct CoreMapping {
  struct TankStatus *_localSubscriber;
  unsigned int originalAddress;
  unsigned char isStationary; // is a bool
  unsigned int originalSize;
};
struct AngleMapping {
  struct CoreMapping *overlapMap;
  struct SparsePointSprite sprite;
  float lastAngle;
};
// these two structs are functionally the same...but the volatile data being
// read is diffrent
struct DriveLeftMapping {
  struct CoreMapping *overlapMap;
  struct SparsePointSprite sprite;
  unsigned char lastAngleSpeedL;
};
struct DriveRightMapping {
  struct CoreMapping *overlapMap;
  struct SparsePointSprite sprite;
  unsigned char lastAngleSpeedR;
};
struct GroupMapping {
  struct DriveLeftMapping *leftCtrl;
  struct DriveRightMapping *rightCtrl;
  struct AngleMapping *angleCtrl;
};

void constructCoreMap(struct CoreMapping *self, unsigned int srcAddr,
                      int srcSize, struct TankStatus *tsSub);
void constructAngleMapping(struct AngleMapping *self,
                           struct CoreMapping *spriteMap);
void constructDriveLeftMapping(struct DriveLeftMapping *self,
                               struct CoreMapping *spriteMap);
void constructDriveRightMapping(struct DriveRightMapping *self,
                                struct CoreMapping *spriteMap);

void startRenderAngleSubscribe(void *arg);
void startRenderDriveLeftSubscribe(void *arg);
void startRenderDriveRightSubscribe(void *arg);
void AsyncStartTankStatusRenderMap(void *arg);

#endif
