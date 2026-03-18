// #pragma once

#ifndef PIXELDATA
#define PIXELDATA

#include <locale.h> // Required for UTF-8 support
#define SPRITE_MAX_SIZE 16
#define MAX_FRAME_COUNT                                                        \
  2 // will replace later with dynamic heap memory allocation

#define _ 32  // Background
#define X 219 // Outline/Color

struct PixelDataRGB_8bit {
  unsigned short copySize; // = SPRITE_MAX_SIZE * SPRITE_MAX_SIZE;
  unsigned char frame[SPRITE_MAX_SIZE][SPRITE_MAX_SIZE];
};

struct PixelDataRGB_8bit_Animation {
  int frameCount;
  int frameCurrent;
  unsigned int copySize;    // SPRITE_MAX_SIZE * SPRITE_MAX_SIZE;
  unsigned char isAnimated; // boolean not supported
  struct PixelDataRGB_8bit frames[MAX_FRAME_COUNT];
};

void constructSprite(struct PixelDataRGB_8bit *self,
                     unsigned char frame[SPRITE_MAX_SIZE][SPRITE_MAX_SIZE]);

void constructSpriteAnimation(struct PixelDataRGB_8bit_Animation *self,
                              struct PixelDataRGB_8bit frames[MAX_FRAME_COUNT]);

void deconstructSprite(struct PixelDataRGB_8bit *spriteToFree);
void deconstructSpriteAnimation(
    struct PixelDataRGB_8bit_Animation *spriteToFree);
void cycleAnimation(struct PixelDataRGB_8bit_Animation *self);

// example array sprite

#endif
