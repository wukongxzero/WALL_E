
#include <Graphics/PixelDataFrame.h>
#include <Graphics/RotatePixelArt.h>
#include <math.h>
#include <string.h>

#define SNAP_MULTIPLE(a, b) ((a) - ((a) % (b)))

// =========================================================================
// 1. HARDWARE-INDEPENDENT FUNCTIONS (Available to both Sim and Hardware)
// =========================================================================

void extractForegroundFromSprite(struct RotatingSprite *self,
                                 unsigned char sourceGrid[16][16],
                                 int maxCapacity) {
  self->elementCount = 0;
  for (int r = 0; r < CENTER_GRAPHIC_PIXEL_MAX; r++) {
    for (int c = 0; c < CENTER_GRAPHIC_PIXEL_MAX; c++) {
      if (sourceGrid[r][c] != _) {
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
    double rotX = (cx * radiansCos) - (cy * radiansSin);
    double rotY = (cx * radiansSin) + (cy * radiansCos);
    self->centerGraphic[i]._row = (int)round((rotX));
    self->centerGraphic[i]._col = (int)round((rotY));
  }
}

void extractSparseMatrix(struct SparsePointSprite *self,
                         unsigned char *sourceGrid, int spriteFlatLength) {
  self->elementCount = 0;
  unsigned int x = 0;
  unsigned int y = 0;

  for (int r = 0; r < spriteFlatLength; r++) {
    if (sourceGrid[r] != _) {
      if (self->elementCount < spriteFlatLength) {
        x += r % 32;
        y += 0 + ((int)(r / 32));
        self->vertexes[self->elementCount]._row = r % 32;
        self->vertexes[self->elementCount]._col = 0 + ((int)(r / 32));
        self->vertexes[self->elementCount]._value =
            (unsigned char)sourceGrid[r];
        self->elementCount++;
      }
    }
  }
  self->centerRotatePointY =
      (x + (self->elementCount / 2)) / self->elementCount;
  self->centerRotatePointX =
      (y + (self->elementCount / 2)) / self->elementCount;
  self->screenLocationX = self->centerRotatePointX * 2;
  self->screenLocationY = self->centerRotatePointY * 2;
}

void extractLargeSparseMatrix(struct SparsePointSprite *self,
                              unsigned char *sourceGrid) {
  self->elementCount = 0;
  unsigned int x = 0;
  unsigned int y = 0;

  for (int r = 0; r < 32 * 32; r++) {
    if (sourceGrid[r] != _) {
      if (self->elementCount < 1024) {
        x += r % 32;
        y += 0 + ((int)(r / 32));
        self->vertexes[self->elementCount]._row = r % 32;
        self->vertexes[self->elementCount]._col = 0 + ((int)(r / 32));
        self->vertexes[self->elementCount]._value =
            (unsigned char)sourceGrid[r];
        self->elementCount++;
      }
    }
  }
  self->centerRotatePointY =
      (x + (self->elementCount / 2)) / self->elementCount;
  self->centerRotatePointX =
      (y + (self->elementCount / 2)) / self->elementCount;
}

void rotateSparsePointSprite(struct SparsePointSprite *self, int angle) {
  self->angleDegrees = SNAP_MULTIPLE(angle % 360, 10);
  float rad = self->angleDegrees * (PI / 180.0f);
  float s = sin(rad);
  float c = cos(rad);

  for (int i = 0; i < self->elementCount; i++) {
    double x =
        (double)self->vertexes[i]._col - (double)self->centerRotatePointX;
    double y =
        (double)self->vertexes[i]._row - (double)self->centerRotatePointY;

    double rotX = (x * c) - (y * s);
    double rotY = (x * s) + (y * c);

    self->vertexes[i]._col =
        (short)round(rotX + (double)self->centerRotatePointX);
    self->vertexes[i]._row =
        (short)round(rotY + (double)self->centerRotatePointY);
  }
}

void reverseRotationSparsePointSprite(struct SparsePointSprite *self) {
  rotateSparsePointSprite(self, -self->angleDegrees);
}

// =========================================================================
// 2. HARDWARE-DEPENDENT RENDERING (Propellor / ST7796S ONLY)
// =========================================================================
#ifndef SIMULATION_SCREEN

#include "Propellor/ST7796S.h"
#include <SPIRendering.h>

void extractEEPROMSparseMatrix(struct SparsePointSprite *self,
                               unsigned int eepromAddr, int spriteWidth,
                               int spriteFlatLength) {
  self->elementCount = 0;
  unsigned int x = 0;
  unsigned int y = 0;
  unsigned char charBuffer;

  for (int r = 0; r < spriteFlatLength; r++) {
    ee_getStr(&charBuffer, 1, eepromAddr + r);
    int pixelCount = spriteWidth;
    if (charBuffer != _) {
      if (self->elementCount < spriteFlatLength) {
        x += r % pixelCount;
        y += 0 + ((int)(r / pixelCount));
        self->vertexes[self->elementCount]._row = r % pixelCount;
        self->vertexes[self->elementCount]._col = 0 + ((int)(r / pixelCount));
        self->vertexes[self->elementCount]._value = charBuffer;
        self->elementCount++;
      }
    }
  }
  self->centerRotatePointY =
      (x + (self->elementCount / 2)) / self->elementCount;
  self->centerRotatePointX =
      (y + (self->elementCount / 2)) / self->elementCount;
}

void rotateSparsePointSpriteRenderNColor(struct SparsePointSprite *self,
                                         int angle, int color, short scale) {
  self->angleDegrees = angle;
  float rad = self->angleDegrees * (PI / 180.0f);
  float s = sinf(rad);
  float c = cosf(rad);

  for (int i = 0; i < self->elementCount; i++) {
    double x =
        (double)self->vertexes[i]._col - (double)self->centerRotatePointX;
    double y =
        (double)self->vertexes[i]._row - (double)self->centerRotatePointY;

    double rotX = (x * c) - (y * s);
    double rotY = (x * s) + (y * c);

    int finalX = self->screenLocationX + (int)roundf(rotX * scale) +
                 self->centerRotatePointX;
    int finalY = self->screenLocationY + (int)roundf(rotY * scale) +
                 self->centerRotatePointY;

    tft_fillRect(finalX - (scale / 2), finalY - (scale / 2), scale, scale,
                 color);
  }
}

void rotateSparsePointSpriteRenderEmbeddedColor(struct SparsePointSprite *self,
                                                int angle, short scale) {
  self->angleDegrees = angle;
  float rad = self->angleDegrees * (PI / 180.0f);
  float s = sinf(rad);
  float c = cosf(rad);

  for (int i = 0; i < self->elementCount; i++) {
    double x =
        (double)self->vertexes[i]._col - (double)self->centerRotatePointX;
    double y =
        (double)self->vertexes[i]._row - (double)self->centerRotatePointY;

    double rotX = (x * c) - (y * s);
    double rotY = (x * s) + (y * c);

    int finalX = self->screenLocationX + (int)roundf(rotX * scale) +
                 self->centerRotatePointX;
    int finalY = self->screenLocationY + (int)roundf(rotY * scale) +
                 self->centerRotatePointY;

    switch (self->vertexes[i]._value) {
    case 219:
      tft_fillRect(finalX - (scale / 2), finalY - (scale / 2), scale, scale,
                   RGB565(200, 200, 200));
      break;
    case 210:
      tft_fillRect(finalX - (scale / 2), finalY - (scale / 2), scale, scale,
                   RGB565(255, 165, 0));
      break;
    case 200:
      tft_fillRect(finalX - (scale / 2), finalY - (scale / 2), scale, scale,
                   RGB565(50, 50, 50));
      break;
    }
  }
}

void clearSparsePointSpriteRenderNColor(struct SparsePointSprite *self,
                                        int angle, short scale) {
  tft_fillRect(self->screenLocationX + 25 - scale * 50 / 2,
               self->screenLocationY + 30 - scale * 50 / 2, scale * 50,
               scale * 50, RGB565(0, 0, 0));
}

#endif // SIMULATION_SCREEN
