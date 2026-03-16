#pragma once
#include "PixelDataFrame.h"
class AGraphicsObject {
private:
  int _x = 0;
  int _y = 0;
  int _width = 0;
  int _height = 0;
  PixelDataFrame *_sprite;

public:
  AGraphicsObject(int originX, int originY);
  void move(int x, int y);
  void setSprite(PixelDataFrame *updateSprite);
};
