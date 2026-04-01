#ifndef SIMULATION_SCREEN
#include "Graphics/RotatePixelArt.h"
#include "Propellor/ST7796S.h"
#include <Graphics/AsyncTankStatusGraphics.h>
#include <SPIRendering.h>
#include <TankStatus/TankStatus.h>
#include <propeller.h>
#include <simpletools.h>
#define SCREEN_SCALAR 2
#define abs(x) ((x) < 0 ? -(x) : (x))
// NOTE:the repetitive part of this could easily be solved with a c++
// template...not sure how inline functions work
//
//
void constructCoreMap(struct CoreMapping *self, unsigned int srcAddr,
                      int srcSize, struct TankStatus *tsSub) {
  // self->original = src;
  self->originalAddress = srcAddr;
  self->originalSize = srcSize;
  self->isStationary = 1;
  self->_localSubscriber = tsSub;
}

void constructAngleMapping(struct AngleMapping *self,
                           struct CoreMapping *spriteMap,
                           struct SparseElement *stack) {
  self->overlapMap = spriteMap;
  // extractSparseMatrix(&self->sprite, self->overlapMap->original,

  self->sprite.vertexes = stack;
  extractEEPROMSparseMatrix(&self->sprite, spriteMap->originalAddress, 16,
                            spriteMap->originalSize);

  self->lastAngle = 0;
}
void constructDriveLeftMapping(struct DriveLeftMapping *self,
                               struct CoreMapping *spriteMap,
                               struct SparseElement *stack) {
  self->overlapMap = spriteMap;

  self->sprite.vertexes = stack;
  // extractLargeSparseMatrix(&self->sprite, self->overlapMap->original);
  extractEEPROMSparseMatrix(&self->sprite, self->overlapMap->originalAddress,
                            32, self->overlapMap->originalSize);

  // extractSparseMatrix(&self->sprite, self->overlapMap->original, 1024);
  self->lastAngleSpeedL = 0;
}
void constructDriveRightMapping(struct DriveRightMapping *self,
                                struct CoreMapping *spriteMap,
                                struct SparseElement *stack) {
  self->overlapMap = spriteMap;
  // extractSparseMatrix(&self->sprite, self->overlapMap->original,
  //                     self->overlapMap->originalSize);

  self->sprite.vertexes = stack;
  //  extractSparseMatrix(&self->sprite, self->overlapMap->original, 1024);
  extractEEPROMSparseMatrix(&self->sprite, self->overlapMap->originalAddress,
                            32, self->overlapMap->originalSize);
  //  extractLargeSparseMatrix(&self->sprite, self->overlapMap->original);
  self->lastAngleSpeedR = 0;
}

void constructStaticSpriteNoMap(struct StaticNoMapping *self,
                                struct CoreMapping *spriteMap,
                                struct SparseElement *stack) {
  self->overlapMap = spriteMap;

  self->sprite.vertexes = stack;
  extractEEPROMSparseMatrix(&self->sprite, self->overlapMap->originalAddress,
                            16, self->overlapMap->originalSize);
}
void startRenderAngleSubscribe(void *arg) {

  struct AngleMapping *angleMap = (struct AngleMapping *)arg;
  if (angleMap->lastAngle != angleMap->overlapMap->_localSubscriber->eulerY) {

    rotateSparsePointSpriteRenderNColor(
        &angleMap->sprite, (int)angleMap->overlapMap->_localSubscriber->eulerY,
        RGB565(0, 0, 0), SCREEN_SCALAR * 8);

    angleMap->lastAngle = angleMap->overlapMap->_localSubscriber->eulerY;
    int heatMap = angleMap->overlapMap->_localSubscriber->eulerY;
    int invHeatMap = 255 - heatMap;
    // TODO:need a rotation rate
    rotateSparsePointSpriteRenderNColor(
        &angleMap->sprite, (int)angleMap->overlapMap->_localSubscriber->eulerY,
        RGB565(255, invHeatMap, invHeatMap), SCREEN_SCALAR * 4);

    angleMap->overlapMap->isStationary = 0;
  }
}
void startRenderDriveLeftSubscribe(void *arg) {

  struct DriveLeftMapping *driveLeftMap = (struct DriveLeftMapping *)arg;
  if (driveLeftMap->lastAngleSpeedL !=
      driveLeftMap->overlapMap->_localSubscriber->driveLeft) {

    clearSparsePointSpriteRenderNColor(
        &driveLeftMap->sprite, driveLeftMap->lastAngleSpeedL, SCREEN_SCALAR);
    driveLeftMap->lastAngleSpeedL =
        driveLeftMap->overlapMap->_localSubscriber->driveLeft;

    int heatMap = driveLeftMap->overlapMap->_localSubscriber->driveLeft;
    int invHeatMap = 255 - heatMap;
    // TODO:need a rotation rate
    rotateSparsePointSpriteRenderNColor(&driveLeftMap->sprite, heatMap,
                                        RGB565(invHeatMap, 0, heatMap),
                                        SCREEN_SCALAR);

    driveLeftMap->overlapMap->isStationary = 0;
  }
}
void startRenderDriveRightSubscribe(void *arg) {
  struct DriveRightMapping *driveRightMap = (struct DriveRightMapping *)arg;
  if (driveRightMap->lastAngleSpeedR !=
      driveRightMap->overlapMap->_localSubscriber->driveRight) {
    clearSparsePointSpriteRenderNColor(
        &driveRightMap->sprite, driveRightMap->lastAngleSpeedR, SCREEN_SCALAR);
    driveRightMap->lastAngleSpeedR =
        driveRightMap->overlapMap->_localSubscriber->driveRight;

    int heatMap = driveRightMap->overlapMap->_localSubscriber->driveRight;
    int invHeatMap = 255 - heatMap;
    // TODO:need a rotation rate
    rotateSparsePointSpriteRenderNColor(&driveRightMap->sprite, heatMap,
                                        RGB565(invHeatMap, 0, heatMap),
                                        SCREEN_SCALAR);

    driveRightMap->overlapMap->isStationary = 0;
  }
}

void AsyncStartTankStatusRenderMap(void *arg) {
  struct GroupMapping *group = (struct GroupMapping *)arg;

  tft_init(0, 4, 3, 2, 1);
  tft_fillScreen(RGB565(0, 0, 0));
  // Add to Render Cog (before the while loop):
  rotateSparsePointSpriteRenderNColor(&group->angleCtrl->sprite, 0,
                                      RGB565(0, 0, 255), SCREEN_SCALAR * 4);

  rotateSparsePointSpriteRenderNColor(&group->rightCtrl->sprite, 0,
                                      RGB565(255, 255, 255), SCREEN_SCALAR);
  rotateSparsePointSpriteRenderNColor(&group->leftCtrl->sprite, 0,
                                      RGB565(255, 255, 255), SCREEN_SCALAR);
  rotateSparsePointSpriteRenderEmbeddedColor(&group->tankBase->sprite, 180, 15);

  waitcnt(CNT + CLKFREQ);

  while (1) {

    // print("heartbeat");
    // waitcnt(CNT + CLKFREQ / 10);

    rotateSparsePointSpriteRenderEmbeddedColor(&group->tankBase->sprite, 180,
                                               15);
    startRenderDriveLeftSubscribe(group->leftCtrl);
    startRenderDriveRightSubscribe(group->rightCtrl);
    startRenderAngleSubscribe(group->angleCtrl);
  }
}
#endif
