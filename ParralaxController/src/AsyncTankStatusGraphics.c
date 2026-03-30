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
                           struct CoreMapping *spriteMap) {
  self->overlapMap = spriteMap;
  // extractSparseMatrix(&self->sprite, self->overlapMap->original,
  //                    self->overlapMap->originalSize);
  extractEEPROMSparseMatrix(&self->sprite, spriteMap->originalAddress,
                            spriteMap->originalSize);

  self->lastAngle = 0;
}
void constructDriveLeftMapping(struct DriveLeftMapping *self,
                               struct CoreMapping *spriteMap) {
  self->overlapMap = spriteMap;
  // extractLargeSparseMatrix(&self->sprite, self->overlapMap->original);
  extractEEPROMSparseMatrix(&self->sprite, self->overlapMap->originalAddress,
                            self->overlapMap->originalSize);

  // extractSparseMatrix(&self->sprite, self->overlapMap->original, 1024);
  self->lastAngleSpeedL = 0;
}
void constructDriveRightMapping(struct DriveRightMapping *self,
                                struct CoreMapping *spriteMap) {
  self->overlapMap = spriteMap;
  // extractSparseMatrix(&self->sprite, self->overlapMap->original,
  //                     self->overlapMap->originalSize);
  //  extractSparseMatrix(&self->sprite, self->overlapMap->original, 1024);
  extractEEPROMSparseMatrix(&self->sprite, self->overlapMap->originalAddress,
                            self->overlapMap->originalSize);
  //  extractLargeSparseMatrix(&self->sprite, self->overlapMap->original);
  self->lastAngleSpeedR = 0;
}

void startRenderAngleSubscribe(void *arg) {

  struct AngleMapping *angleMap = (struct AngleMapping *)arg;
  if (angleMap->lastAngle != angleMap->overlapMap->_localSubscriber->eulerY) {
    angleMap->lastAngle = angleMap->overlapMap->_localSubscriber->eulerY;

    ClearSparseSprite(&(angleMap->sprite), angleMap->sprite.screenLocationX,
                      angleMap->sprite.screenLocationY, SCREEN_SCALAR);
    // rotateSpriteHardModify(&balanceBeamAngle, 10);
    reverseRotationSparsePointSprite(&(angleMap->sprite));
    rotateSparsePointSprite(&angleMap->sprite,
                            angleMap->overlapMap->_localSubscriber->eulerY);

    int heatMap = angleMap->overlapMap->_localSubscriber->eulerY;
    int invHeatMap = 255 - heatMap;
    renderSparsePointSpriteColor(
        &angleMap->sprite, angleMap->sprite.screenLocationX,
        angleMap->sprite.screenLocationY, SCREEN_SCALAR,
        RGB565(255, invHeatMap, invHeatMap));
    angleMap->overlapMap->isStationary = 0;
  } else {
    if ((abs(angleMap->sprite.angleDegrees % 360) < 4) ==
        !angleMap->overlapMap->isStationary) {
      ClearSparseSprite(&angleMap->sprite, angleMap->sprite.screenLocationX,
                        angleMap->sprite.screenLocationY, SCREEN_SCALAR);
      //      extractSparseMatrix(&angleMap->sprite,
      //      angleMap->overlapMap->original,
      //                          angleMap->overlapMap->originalSize);
      // extractEEPROMSparseMatrix(&angleMap->sprite,
      //                          angleMap->overlapMap->originalAddress,
      //                          angleMap->overlapMap->originalSize);

      renderSparseSprite(&angleMap->sprite, angleMap->sprite.screenLocationX,
                         angleMap->sprite.screenLocationY, SCREEN_SCALAR);
      angleMap->overlapMap->isStationary = 1;
    }
  }
}
void startRenderDriveLeftSubscribe(void *arg) {

  struct DriveLeftMapping *driveLeftMap = (struct DriveLeftMapping *)arg;
  if (driveLeftMap->lastAngleSpeedL !=
      driveLeftMap->overlapMap->_localSubscriber->driveLeft) {
    driveLeftMap->lastAngleSpeedL =
        driveLeftMap->overlapMap->_localSubscriber->driveLeft;

    ClearSparseSprite(&(driveLeftMap->sprite),
                      driveLeftMap->sprite.screenLocationX,
                      driveLeftMap->sprite.screenLocationY, SCREEN_SCALAR);
    // rotateSpriteHardModify(&balanceBeamAngle, 10);
    reverseRotationSparsePointSprite(&(driveLeftMap->sprite));

    rotateSparsePointSprite(
        &driveLeftMap->sprite,
        driveLeftMap->overlapMap->_localSubscriber->driveLeft /
            ANIMATION_DRIVE_SCALAR);

    int heatMap = driveLeftMap->overlapMap->_localSubscriber->driveLeft;
    int invHeatMap = 255 - heatMap;
    renderSparsePointSpriteColor(&driveLeftMap->sprite,
                                 driveLeftMap->sprite.screenLocationX,
                                 driveLeftMap->sprite.screenLocationY,
                                 SCREEN_SCALAR, RGB565(heatMap, 0, invHeatMap));
    driveLeftMap->overlapMap->isStationary = 0;
  } else {
    if ((abs(driveLeftMap->sprite.angleDegrees % 360) < 4) ==
        !driveLeftMap->overlapMap->isStationary) {
      ClearSparseSprite(&driveLeftMap->sprite,
                        driveLeftMap->sprite.screenLocationX,
                        driveLeftMap->sprite.screenLocationY, SCREEN_SCALAR);
      //      extractSparseMatrix(&driveLeftMap->sprite,
      //                         driveLeftMap->overlapMap->original,
      //                         driveLeftMap->overlapMap->originalSize);
      // extractEEPROMSparseMatrix(&driveLeftMap->sprite,
      //                          driveLeftMap->overlapMap->originalAddress,
      //                          driveLeftMap->overlapMap->originalSize);

      renderSparseSprite(&driveLeftMap->sprite,
                         driveLeftMap->sprite.screenLocationX,
                         driveLeftMap->sprite.screenLocationY, SCREEN_SCALAR);
      driveLeftMap->overlapMap->isStationary = 1;
    }
  }
}
void startRenderDriveRightSubscribe(void *arg) {
  struct DriveRightMapping *driveRightMap = (struct DriveRightMapping *)arg;
  if (driveRightMap->lastAngleSpeedR !=
      driveRightMap->overlapMap->_localSubscriber->driveRight) {
    driveRightMap->lastAngleSpeedR =
        driveRightMap->overlapMap->_localSubscriber->driveRight;

    ClearSparseSprite(&(driveRightMap->sprite),
                      driveRightMap->sprite.screenLocationX,
                      driveRightMap->sprite.screenLocationY, SCREEN_SCALAR);
    // rotateSpriteHardModify(&balanceBeamAngle, 10);
    reverseRotationSparsePointSprite(&(driveRightMap->sprite));

    rotateSparsePointSprite(
        &driveRightMap->sprite,
        driveRightMap->overlapMap->_localSubscriber->driveRight /
            ANIMATION_DRIVE_SCALAR);

    int heatMap = driveRightMap->overlapMap->_localSubscriber->driveRight;
    int invHeatMap = 255 - heatMap;
    renderSparsePointSpriteColor(&driveRightMap->sprite,
                                 driveRightMap->sprite.screenLocationX,
                                 driveRightMap->sprite.screenLocationY,
                                 SCREEN_SCALAR, RGB565(heatMap, 0, invHeatMap));
    driveRightMap->overlapMap->isStationary = 0;
  } else {
    if ((abs(driveRightMap->sprite.angleDegrees % 360) < 4) ==
        !driveRightMap->overlapMap->isStationary) {
      ClearSparseSprite(&driveRightMap->sprite,
                        driveRightMap->sprite.screenLocationX,
                        driveRightMap->sprite.screenLocationY, SCREEN_SCALAR);
      // extractSparseMatrix(&driveRightMap->sprite,
      //                     driveRightMap->overlapMap->original,
      //                     driveRightMap->overlapMap->originalSize);
      // extractEEPROMSparseMatrix(&driveRightMap->sprite,
      //                           driveRightMap->overlapMap->originalAddress,
      //                           driveRightMap->overlapMap->originalSize);

      renderSparseSprite(&driveRightMap->sprite,
                         driveRightMap->sprite.screenLocationX,
                         driveRightMap->sprite.screenLocationY, SCREEN_SCALAR);
      driveRightMap->overlapMap->isStationary = 1;
    }
  }
}

void AsyncStartTankStatusRenderMap(void *arg) {
  struct GroupMapping *group = (struct GroupMapping *)arg;

  tft_init(0, 4, 3, 2, 1);
  tft_fillScreen(RGB565(0, 0, 0));
  // Add to Render Cog (before the while loop):
  print("Subscriber RAM Address: %p\n",
        group->rightCtrl->overlapMap->_localSubscriber);

  renderSparsePointSpriteColor(
      &group->angleCtrl->sprite, group->angleCtrl->sprite.screenLocationX,
      group->angleCtrl->sprite.screenLocationY, 2, RGB565(255, 255, 255));
  renderSparsePointSpriteColor(
      &group->leftCtrl->sprite, group->leftCtrl->sprite.screenLocationX,
      group->leftCtrl->sprite.screenLocationY, 2, RGB565(0, 0, 255));
  renderSparsePointSpriteColor(
      &group->rightCtrl->sprite, group->rightCtrl->sprite.screenLocationX,
      group->rightCtrl->sprite.screenLocationY, 2, RGB565(0, 0, 255));

  while (1) {
    while (lockset(spriteLock))
      ;
    startRenderDriveLeftSubscribe(group->leftCtrl);
    startRenderDriveRightSubscribe(group->rightCtrl);
    startRenderAngleSubscribe(group->angleCtrl);
    print("cogTankStatus%u\n",
          group->rightCtrl->overlapMap->_localSubscriber->driveRight);
    lockclr(spriteLock);
    waitcnt(CNT + CLKFREQ / 100);
  }
}
