// #pragma once
// #include "PixelDataFrame.h"
// class AGraphicsObject {
// private:
//   int _x = 0;
//   int _y = 0;
//   int _width = 0;
//   int _height = 0;
//   PixelDataFrame *_sprite;
//
// public:
//   AGraphicsObject(int originX, int originY);
//   void move(int x, int y);
//   void setSprite(PixelDataFrame *updateSprite);
// };
#ifndef GRAPHICS_OBJECT
#define GRAPHICS_OBJECT
#include "PixelDataFrame.h"

struct BoundingBox {
  unsigned short _x;
  unsigned short _y;
  // TODO: change to signed short to support pixel flips
  unsigned short _width;
  unsigned short _height;
};

// not a c++ object lol
struct GraphicsObject {
  struct BoundingBox boundingBox;
  unsigned char _id;
  // defualt sprite
  struct PixelDataRGB_8bit *defaultSprite;
};

// difference is this one animates
struct GraphicsObjectSprite {
  struct GraphicsObject *graphicsObject;
  struct PixelDataRGB_8bit_Animation *spriteAnimation;
};

// update hard position
void moveGraphicAbs(struct GraphicsObject *self, unsigned short newX,
                    unsigned short newY);

// struct GraphicsObject *self, signed short x, signed short y
//  update x and y to move by the passed values, signed allows you to subtract
void moveGraphicByVal(struct GraphicsObject *self, signed short byX,
                      signed short byY);

// set self to all pixels 0 so it can clean itself up on render
void clearObject(struct GraphicsObject *self);
// allow animation frames to cycle, cant do it if bool denies ability to be
// animated

void update(struct GraphicsObject *self);
void updateAnimate(struct GraphicsObjectSprite *self);
void attachGraphicsObjectToAnimatedSprite(struct GraphicsObjectSprite *self,
                                          struct GraphicsObject *boundingBox);
void moveByAnimatedSprite(struct GraphicsObjectSprite *self, signed short byX,
                          signed short byY);

void constructGraphicsObject(struct GraphicsObject *self, unsigned short loc_x,
                             unsigned short loc_y, unsigned short width_x,
                             unsigned short height_y);

void constructGraphicsSprite(struct GraphicsObjectSprite *self,
                             struct GraphicsObject *selfBoundingBox);

void constructGraphicsSpriteWOrigin(struct GraphicsObjectSprite *self,
                                    struct GraphicsObject *selfBoundingBox,
                                    unsigned short loc_x, unsigned short loc_y,
                                    unsigned width_x, unsigned height_y);

#endif
