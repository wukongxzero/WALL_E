#pragma once
#define SPRITE_MAX_SIZE 16

#define _ 0   // Background
#define X 254 // Outline/Color

// sprite are 32 pix x 32
struct PixelDataFrame {
public:
  unsigned char red[SPRITE_MAX_SIZE][SPRITE_MAX_SIZE];
  unsigned char green[SPRITE_MAX_SIZE][SPRITE_MAX_SIZE];
  unsigned char blue[SPRITE_MAX_SIZE][SPRITE_MAX_SIZE];
};

struct PixelDataGrayScale {
public:
  unsigned char gray[SPRITE_MAX_SIZE][SPRITE_MAX_SIZE];
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

PixelDataGrayScale.gray = samplePixelSprite1;
#endif
