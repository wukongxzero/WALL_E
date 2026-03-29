#include "Graphics/RotatePixelArt.h"
#ifndef SIMULATION_SCREEN
#include <Propellor/ST7796S.h>
#include <math.h>
void rotateAndRenderSparse(struct SparsePointSprite *self, int screenX,
                           int screenY, int angle, int scale, int offsetX,
                           int offsetY) {
  float rad = angle * (PI / 180.0f);
  float s = sin(rad);
  float c = cos(rad);

  // The pivot point for an 8x8 grid is 3.5 (the middle of 0-7)
  float pivot = 3.5f;

  for (int i = 0; i < self->elementCount; i++) {
    struct SparseElement *el = &self->vertexes[i];

    // 1. Translate point so pivot is at (0,0)
    float x = (float)el->_col - pivot;
    float y = (float)el->_row - pivot;

    // 2. Rotate
    float rotX = (x * c) - (y * s);
    float rotY = (x * s) + (y * c);

    // 3. Translate back to screen coordinates and scale
    // We add 0.5f before casting to int to perform rounding
    int drawX = screenX + (int)((rotX + pivot) * scale + 0.5f);
    int drawY = screenY + (int)((rotY + pivot) * scale + 0.5f);

    // 4. Render using your high-speed 16-bit block write
    tft_fillRect(drawX + offsetX, drawY + offsetY, scale, scale,
                 (RGB565(255, 255, 255)));
  }
}

void renderSparseSprite(struct SparsePointSprite *self, int screenX,
                        int screenY, int scale) {
  for (int i = 0; i < self->elementCount; i++) {
    struct SparseElement *el = &self->vertexes[i];

    // 1. Calculate the scaled position on the LCD
    // We use the coordinates stored in the sparse element
    int posX = screenX + (el->_col * scale);
    int posY = screenY + (el->_row * scale);

    // 2. Convert the 8-bit sprite value to RGB565
    // unsigned short color = get565Color(el->_value);

    // 3. Use the high-speed 16-bit fill feature
    tft_fillRect(posX, posY, scale, scale, RGB565(255, 255, 255));
  }
}

void renderSparsePointSprite(struct SparsePointSprite *self, int screenX,
                             int screenY, int scale) {
  for (int i = 0; i < self->elementCount; i++) {
    struct SparseElement *el = &self->vertexes[i];

    // 1. Calculate the scaled position on the LCD
    // We use the coordinates stored in the sparse element
    int posX = screenX + (el->_col * scale);
    int posY = screenY + (el->_row * scale);

    // 2. Convert the 8-bit sprite value to RGB565
    // unsigned short color = get565Color(el->_value);

    // 3. Use the high-speed 16-bit fill feature
    tft_fillRect(posX, posY, scale, scale, RGB565(255, 255, 255));
  }
}

void renderSparsePointSpriteColor(struct SparsePointSprite *self, int screenX,
                                  int screenY, int scale, int color) {
  for (int i = 0; i < self->elementCount; i++) {
    struct SparseElement *el = &self->vertexes[i];

    // 1. Calculate the scaled position on the LCD
    // We use the coordinates stored in the sparse element
    int posX = screenX + (el->_col * scale);
    int posY = screenY + (el->_row * scale);

    // 2. Convert the 8-bit sprite value to RGB565
    // unsigned short color = get565Color(el->_value);

    // 3. Use the high-speed 16-bit fill feature
    tft_fillRect(posX, posY, scale, scale, color);
  }
}

void ClearSparseSprite(struct RotatingSprite *self, int screenX, int screenY,
                       int scale) {
  for (int i = 0; i < self->elementCount; i++) {
    struct SparseElement *el = &self->centerGraphic[i];

    // 1. Calculate the scaled position on the LCD
    // We use the coordinates stored in the sparse element
    int posX = screenX + (el->_col * scale);
    int posY = screenY + (el->_row * scale);

    // 2. Convert the 8-bit sprite value to RGB565
    // unsigned short color = get565Color(el->_value);

    // 3. Use the high-speed 16-bit fill feature
    tft_fillRect(posX, posY, scale, scale, RGB565(0, 0, 0));
  }
}
#endif
