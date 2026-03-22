#include <Graphics/PixelDataFrame.h>
#include <Graphics/RotatePixelArt.h>
#include <math.h>
#include <string.h>
#define PI 3.14159265359
#define PIXEL_DATA_FRAME_MIDPOINT 7.5

void extractForegroundFromSprite(
    struct RotatingSprite *self,
    unsigned char sourceGrid[CENTER_GRAPHIC_PIXEL_MAX]
                            [CENTER_GRAPHIC_PIXEL_MAX],
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

void constructGraphicsSpriteWOrigin(struct RotatingSprite *self,
                                    struct PixelDataRGB_8bit *space) {
  self->rotateSpace = space;
  self->angleDegrees = 0;
  self->elementCount = 0;
}

void rotateSprite(struct RotatingSprite *self, signed int angle) {
  memset(self->rotateSpace->frame, _, sizeof(self->rotateSpace->frame));
  angle %= 360;

  float radiansCos = cos(angle * (PI / 180.0));
  float radiansSin = sin(angle * (PI / 180.0));

  for (int i = 0; i < self->elementCount; i++) {
    // 1. Shift coordinates so the center of the 16x16 grid is at (0,0)
    float cx = self->centerGraphic[i]._col - PIXEL_DATA_FRAME_MIDPOINT;
    float cy = self->centerGraphic[i]._row - PIXEL_DATA_FRAME_MIDPOINT;

    // 2. Apply Z-axis rotation matrix
    float rotX = (cx * radiansCos) - (cy * radiansSin);
    float rotY = (cx * radiansSin) + (cy * radiansCos);

    int newCol = (int)round(rotX + PIXEL_DATA_FRAME_MIDPOINT);
    int newRow = (int)round(rotY + PIXEL_DATA_FRAME_MIDPOINT);
    self->rotateSpace->frame[newCol][newRow] = self->centerGraphic[i]._value;
  }
}

void updateRotateSprite(struct RotatingSprite *self);
