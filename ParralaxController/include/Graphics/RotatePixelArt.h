
#ifndef ROTATE_PIXEL_ART
#define ROTATE_PIXEL_ART

#include <Graphics/PixelDataFrame.h>
#define CENTER_GRAPHIC_PIXEL_MAX 8

struct SparseElement {
  unsigned short _row;
  unsigned short _col;
  unsigned char _value;
};

// using a sparse matrix
struct RotatingSprite {
  struct PixelDataRGB_8bit *rotateSpace;
  unsigned short elementCount;
  struct SparseElement centerGraphic[CENTER_GRAPHIC_PIXEL_MAX];
  int angleDegrees;
};

// TODO: repeat this constructor with dynamic arrays and malloc, for now build
// elements on stack
//
void extractForegroundFromSprite(
    struct RotatingSprite *self,
    unsigned char sourceGrid[CENTER_GRAPHIC_PIXEL_MAX]
                            [CENTER_GRAPHIC_PIXEL_MAX],
    int maxCapacity);

void constructSparseRotationElement(struct SparseElement *self,
                                    unsigned short row, unsigned short col,
                                    unsigned char pixelValue);

void constructGraphicsSpriteWOrigin(struct RotatingSprite *self,
                                    struct PixelDataRGB_8bit *space);

void rotateSprite(struct RotatingSprite *self, signed int angle);

void updateRotateSprite(struct RotatingSprite *self);

#endif
