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

// not a c++ object lol
struct GraphicsObject {
  unsigned char _id;
  unsigned short _x;
  unsigned short _y;
  unsigned short _width;
  unsigned short _height;

  struct PixelDataRGB_8bit *frame;
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

#endif
