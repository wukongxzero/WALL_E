#include "Graphics/AGraphicsObject.h"
#include "Graphics/PixelDataFrame.h"

#ifdef SIMULATION_SCREEN

#include <Graphics/NcursesScreenImpl.h>
#endif

// update hard position
void moveGraphicAbs(struct GraphicsObject *self, unsigned short newX,
                    unsigned short newY) {
  self->boundingBox._x = newX;
  self->boundingBox._y = newY;
}

// struct GraphicsObject *self, signed short x, signed short y
//  update x and y to move by the passed values, signed allows you to subtract
void moveGraphicByVal(struct GraphicsObject *self, signed short byX,
                      signed short byY) {
  self->boundingBox._x += byX;
  self->boundingBox._y += byY;
}
void update(struct GraphicsObject *self) {}

// animated
void updateAnimate(struct GraphicsObjectSprite *self) {

  if (self->spriteAnimation->isAnimated) {
    self->spriteAnimation->frameCurrent++;
    self->spriteAnimation->frameCurrent %= self->spriteAnimation->frameCount;
  }

#ifdef SIMULATION_SCREEN
  // renderFrame(
  //     &self->spriteAnimation->frames[self->spriteAnimation->frameCurrent],
  //     self->graphicsObject->boundingBox._x,
  //     self->graphicsObject->boundingBox._y);
#endif
}

void cycleAnimation(struct PixelDataRGB_8bit_Animation *self) {}

void attachGraphicsObjectToAnimatedSprite(struct GraphicsObjectSprite *self,
                                          struct GraphicsObject *boundingBox) {
  self->graphicsObject = boundingBox;
}
void moveByAnimatedSprite(struct GraphicsObjectSprite *self, signed short byX,
                          signed short byY) {
  // protect memory core segment issue
  if (!self->graphicsObject)
    return;

  self->graphicsObject->boundingBox._x += byX;
  self->graphicsObject->boundingBox._y += byY;
}

void constructGraphicsObject(struct GraphicsObject *self, unsigned short loc_x,
                             unsigned short loc_y, unsigned short width_x,
                             unsigned short height_y) {
  // bounding box on stack
  struct BoundingBox bb;
  bb._x = loc_x;
  bb._y = loc_y;
  bb._width = width_x;
  bb._height = height_y;
  self->boundingBox = bb;
}

void constructGraphicsSprite(struct GraphicsObjectSprite *self,
                             struct GraphicsObject *selfBoundingBox) {
  constructGraphicsObject(selfBoundingBox, 0, 0, 0, 0);
  self->graphicsObject = selfBoundingBox;
}

void constructGraphicsObjectWOrigin(struct GraphicsObjectSprite *self,
                                    struct GraphicsObject *selfBoundingBox,
                                    unsigned short loc_x, unsigned short loc_y,
                                    unsigned width_x, unsigned height_y) {
  constructGraphicsObject(selfBoundingBox, loc_x, loc_y, width_x, height_y);
  self->graphicsObject = selfBoundingBox;
}
