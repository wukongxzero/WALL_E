// #pragma once

#ifndef PIXELDATA
#define PIXELDATA

#include <locale.h> // Required for UTF-8 support
#define SPRITE_MAX_SIZE 16

#define _ 32  // Background
#define X 219 // Outline/Color

// Note: this is not how graphics work, use 8_bit double array
//  sprite are 32 pix x 32
struct PixelDataFrame {
  unsigned char red[SPRITE_MAX_SIZE][SPRITE_MAX_SIZE];
  unsigned char green[SPRITE_MAX_SIZE][SPRITE_MAX_SIZE];
  unsigned char blue[SPRITE_MAX_SIZE][SPRITE_MAX_SIZE];
};

struct PixelDataRGB_8bit {
  // support animated sprites
  int frameCount;
  unsigned char isAnimnated; // boolean not supported
  char frame[SPRITE_MAX_SIZE][SPRITE_MAX_SIZE];
};
// example array sprite
#ifdef INCLUDE_SAMPLES

unsigned char samplePixelSprite1[SPRITE_MAX_SIZE][SPRITE_MAX_SIZE] = {
    {_, _, _, _, _, _, _, _, _, _, _, _, _, _, _, _},
    {_, _, _, _, _, _, _, _, _, _, _, _, _, _, _, _},
    {_, _, _, _, _, _, _, _, _, _, _, _, _, _, _, _},
    {_, _, X, X, X, X, X, X, X, X, X, _, _, _, _, _},
    {_, _, X, X, X, X, X, X, X, X, X, _, _, _, _, _},
    {_, _, X, X, X, X, X, X, X, X, X, _, _, _, _, _},
    {_, _, _, _, _, _, _, X, X, X, _, _, _, _, _, _},
    {_, _, _, _, _, _, _, X, X, X, _, _, _, _, _, _},
    {_, _, _, X, X, X, X, X, X, X, X, X, X, _, _, _},
    {_, _, _, X, _, _, _, _, _, _, _, _, X, _, _, _},
    {_, _, _, X, _, _, _, _, _, _, _, _, X, _, _, _},
    {X, X, X, X, X, X, X, X, X, X, X, X, X, X, X, X},
    {X, X, _, _, X, _, _, _, _, _, _, X, _, _, X, X},
    {_, X, X, X, X, X, _, _, _, _, X, X, X, X, X, _},
    {_, _, X, _, X, _, _, _, _, _, _, X, X, X, _, _},
    {_, _, _, X, X, X, X, X, X, X, X, X, X, _, _, _}};

unsigned char samplePixelSprite2[SPRITE_MAX_SIZE][SPRITE_MAX_SIZE] = {
    {_, _, _, _, _, _, _, _, _, _, _, _, _, _, _, _},
    {_, _, _, _, _, _, _, _, _, _, _, _, _, _, _, _},
    {_, _, _, _, _, _, _, _, _, _, _, _, _, _, _, _},
    {_, _, _, _, _, _, _, _, _, _, _, _, _, _, _, _},
    {_, _, X, X, X, X, X, X, X, X, X, _, _, _, _, _},
    {_, _, X, X, X, X, X, X, X, X, X, _, _, _, _, _},
    {_, _, X, X, X, X, X, X, X, X, X, _, _, _, _, _},
    {_, _, _, _, _, _, _, X, X, X, _, _, _, _, _, _},
    {_, _, _, X, X, X, X, X, X, X, X, X, X, _, _, _},
    {_, _, _, X, _, _, _, _, _, _, _, _, X, _, _, _},
    {_, _, _, X, _, _, _, _, _, _, _, _, X, _, _, _},
    {X, X, X, X, X, X, X, X, X, X, X, X, X, X, X, X},
    {X, X, _, X, _, X, _, _, _, _, X, _, X, _, X, X},
    {_, X, X, _, X, _, _, _, _, _, _, X, _, X, X, _},
    {_, _, X, X, _, X, _, _, _, _, X, _, X, X, _, _},
    {_, _, _, X, X, X, X, X, X, X, X, X, X, _, _, _}};

#endif
#endif
