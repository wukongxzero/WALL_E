
#ifndef ROTATE_PIXEL_ART
#define ROTATE_PIXEL_ART

#include <Graphics/PixelDataFrame.h>
#include <wchar.h>

#define CENTER_GRAPHIC_PIXEL_MAX 8
// Note.
#define SPARSE_PIXEL_MAX 288
#define PI 3.14159265359
#define PIXEL_DATA_FRAME_MIDPOINT 7

// Scale: 1 = 1x1 pixels, 10 = 10x10 blocks on screen, etc.
// void renderSparseSprite(struct RotatingSprite *self, int screenX, int
// screenY,
//                        int scale);
// Helper to map your 8-bit pixel value to RGB565
// unsigned short get565Color(unsigned char value);

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

// TODO:refactor as a transformation struct like SparsePointTrasnformation
// potentially isolate sprite tranbsformation as seperate structure
struct SparsePointSprite {
  int angleDegrees;
  unsigned short elementCount;
  struct SparseElement *vertexes;

  int centerRotatePointX;
  int centerRotatePointY;

  // TODO: at some point refactor with unsigned short
  int screenLocationX;
  int screenLocationY;
};

// TODO: repeat this constructor with dynamic arrays and malloc, for now build
// elements on stack
//
void extractForegroundFromSprite(struct RotatingSprite *self,
                                 unsigned char sourceGrid[16][16],
                                 int maxCapacity);

void constructSparseRotationElement(struct SparseElement *self,
                                    unsigned short row, unsigned short col,
                                    unsigned char pixelValue);

void constructGraphicsSpriteRotatableWOrigin(struct RotatingSprite *self,
                                             struct PixelDataRGB_8bit *space);

void rotateSprite(struct RotatingSprite *self, signed int angle);
void rotateSpriteHardModify(struct RotatingSprite *self, signed int angle);

void updateRotateSprite(struct RotatingSprite *self);
//----------------sparse point struct, goal to support rotations in screen space
void constructSparseMatrixSprite(struct SparsePointSprite *self);
void extractSparseMatrix(struct SparsePointSprite *self,
                         unsigned char *sourceGrid, int spriteFlatLength);
void extractLargeSparseMatrix(struct SparsePointSprite *self,
                              unsigned char sourceGrid[1024]);
void extractEEPROMSparseMatrix(struct SparsePointSprite *self,
                               unsigned int eepromAddr, int spriteWidth,
                               unsigned short spriteFlatLength);

void rotateSparsePointSprite(struct SparsePointSprite *self, int angle);
void rotateSparsePointSpriteRenderNColor(struct SparsePointSprite *self,
                                         int angle, int color, short scale);
void rotateSparsePointSpriteRenderEmbeddedColor(struct SparsePointSprite *self,
                                                int angle, short scale);

void clearSparsePointSpriteRenderNColor(struct SparsePointSprite *self,
                                        int angle, short scale);
void reverseRotationSparsePointSprite(struct SparsePointSprite *self);

#endif
