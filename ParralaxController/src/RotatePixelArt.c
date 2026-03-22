#include <Graphics/PixelDataFrame.h>
#include <Graphics/RotatePixelArt.h>
#include <math.h>
#include <string.h>
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

void updateRotateSprite(struct RotatingSprite *self);
