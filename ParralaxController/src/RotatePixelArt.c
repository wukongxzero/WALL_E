#include <Graphics/PixelDataFrame.h>
#include <Graphics/RotatePixelArt.h>
#include <math.h>
#include <string.h>

void extractForegroundFromSprite(struct RotatingSprite *self,
                                 unsigned char sourceGrid[16][16],
                                 int maxCapacity) {
  self->elementCount = 0;

  for (int r = 0; r < CENTER_GRAPHIC_PIXEL_MAX; r++) {
    for (int c = 0; c < CENTER_GRAPHIC_PIXEL_MAX; c++) {

      // Check if the pixel is NOT the background
      if (sourceGrid[r][c] != _) {

        // Ensure we don't overflow the provided sparse array
        if (self->elementCount < maxCapacity) {
          self->centerGraphic[self->elementCount]._row = r;
          self->centerGraphic[self->elementCount]._col = c;
          self->centerGraphic[self->elementCount]._value =
              (unsigned char)sourceGrid[r][c];
          self->elementCount++;
        }
      }
    }
  }
}

// TODO: repeat this constructor with dynamic arrays and malloc, for now build
// elements on stack

void constructGraphicsSpriteRotatableWOrigin(struct RotatingSprite *self,
                                             struct PixelDataRGB_8bit *space) {
  self->rotateSpace = space;
  self->angleDegrees = 0;
  self->elementCount = 0;
}

void rotateSprite(struct RotatingSprite *self, signed int angle) {
  memset(self->rotateSpace->frame, (unsigned char)_,
         sizeof(self->rotateSpace->frame));
  angle %= 360;

  float radiansCos = cos(angle * (PI / 180.0));
  float radiansSin = sin(angle * (PI / 180.0));

  for (int i = 0; i < self->elementCount; i++) {

    self->rotateSpace
        ->frame[self->centerGraphic[i]._row + PIXEL_DATA_FRAME_MIDPOINT]
               [self->centerGraphic[i]._col + PIXEL_DATA_FRAME_MIDPOINT] =
        (unsigned char)_;
    double cx = self->centerGraphic[i]._col - 7.5 / 2;
    double cy = self->centerGraphic[i]._row - 7.5 / 2;

    // 2. Apply Z-axis rotation matrix
    double rotX = (cx * radiansCos) - (cy * radiansSin);
    double rotY = (cx * radiansSin) + (cy * radiansCos);

    int newCol = (int)round((rotX + 7.5 / 2) + 4);
    int newRow = (int)round((rotY + 7.5 / 2) + 4);

    self->rotateSpace->frame[newCol][newRow] =
        (unsigned char)self->centerGraphic[i]._value;
  }
}

void rotateSpriteHardModify(struct RotatingSprite *self, signed int angle) {
  angle %= 360;
  self->angleDegrees = angle;

  float radiansCos = cos(angle * (PI / 180.0));
  float radiansSin = sin(angle * (PI / 180.0));

  for (int i = 0; i < self->elementCount; i++) {

    double cx = self->centerGraphic[i]._col;
    double cy = self->centerGraphic[i]._row;

    // 2. Apply Z-axis rotation matrix
    double rotX = (cx * radiansCos) - (cy * radiansSin);
    double rotY = (cx * radiansSin) + (cy * radiansCos);

    self->centerGraphic[i]._row = (int)round((rotX));
    self->centerGraphic[i]._col = (int)round((rotY));
  }
}

void updateRotateSprite(struct RotatingSprite *self);
//----------------sparse point struct, goal to support rotations in screen space
static void autoFindCenterPoint(struct SparsePointSprite *self) {

  unsigned int x = 0;
  unsigned int y = 0;
  for (int i = 0; i < self->elementCount; i++) {
    x += self->vertexes[i]._row;
    y += self->vertexes[i]._col;
  }
  self->centerRotatePointX = x / self->elementCount;
  self->centerRotatePointY = y / self->elementCount;
}

void constructSparseMatrixSprite(struct SparsePointSprite *self) {}
void extractSparseMatrix(struct SparsePointSprite *self,
                         unsigned char sourceGrid[16][16]) {
  self->elementCount = 0;
  unsigned int x = 0;
  unsigned int y = 0;

  for (int r = 0; r < CENTER_GRAPHIC_PIXEL_MAX; r++) {
    for (int c = 0; c < CENTER_GRAPHIC_PIXEL_MAX; c++) {

      // Check if the pixel is NOT the background
      if (sourceGrid[r][c] != _) {

        // Ensure we don't overflow the provided sparse array
        if (self->elementCount < SPARSE_PIXEL_MAX) {

          x += r;
          y += c;
          self->vertexes[self->elementCount]._row = r;
          self->vertexes[self->elementCount]._col = c;
          self->vertexes[self->elementCount]._value =
              (unsigned char)sourceGrid[r][c];
          self->elementCount++;
        }
      }
    }
  }

  self->centerRotatePointY =
      (x + (self->elementCount / 2)) / self->elementCount;
  self->centerRotatePointX =
      (y + (self->elementCount / 2)) / self->elementCount;
}
// void rotateSparsePointSprite(struct SparsePointSprite *self, int angle) {
//   angle %= 360;
//
//   float radiansCos = cos(angle * (PI / 180.0));
//   float radiansSin = sin(angle * (PI / 180.0));
//
//   for (int i = 0; i < self->elementCount; i++) {
//
//     double cx = self->vertexes[i]._col - self->centerRotatePointY;
//     double cy = self->vertexes[i]._row - self->centerRotatePointX;
//
//     // 2. Apply Z-axis rotation matrix
//     double rotX = (cx * radiansCos) - (cy * radiansSin);
//     double rotY = (cx * radiansSin) + (cy * radiansCos);
//
//     self->vertexes[i]._row = (int)round((rotX + self->centerRotatePointX));
//     self->vertexes[i]._col = (int)round((rotY + self->centerRotatePointY));
//   }
//   autoFindCenterPoint(self);
// }
//
void rotateSparsePointSprite(struct SparsePointSprite *self, int angle) {
  float rad = angle * (PI / 180.0f);
  float s = sin(rad);
  float c = cos(rad);

  for (int i = 0; i < self->elementCount; i++) {
    // 1. Translate to Origin (relative to the center of the beam)
    double x =
        (double)self->vertexes[i]._col - (double)self->centerRotatePointX;
    double y =
        (double)self->vertexes[i]._row - (double)self->centerRotatePointY;

    // 2. Rotate around (0,0)
    double rotX = (x * c) - (y * s);
    double rotY = (x * s) + (y * c);

    // 3. Translate back to Screen Space
    self->vertexes[i]._col =
        (short)round(rotX + (double)self->centerRotatePointX);
    self->vertexes[i]._row =
        (short)round(rotY + (double)self->centerRotatePointY);
  }
  // DO NOT call autoFindCenterPoint(self) here if you want it to stay in one
  // place!
}
