#include <Graphics/PixelDataFrame.h>
#include <stdlib.h>
#include <string.h>

void constructSprite(struct PixelDataRGB_8bit *self,
                     unsigned char frame[SPRITE_MAX_SIZE][SPRITE_MAX_SIZE]) {
  self->copySize = SPRITE_MAX_SIZE * SPRITE_MAX_SIZE;
  memcpy(self->frame, frame, SPRITE_MAX_SIZE * SPRITE_MAX_SIZE);
}

void constructSpriteAnimation(
    struct PixelDataRGB_8bit_Animation *self,
    struct PixelDataRGB_8bit frames[MAX_FRAME_COUNT]) {

  self->frameCount = MAX_FRAME_COUNT;
  self->frameCurrent = 0;
  self->copySize = MAX_FRAME_COUNT * SPRITE_MAX_SIZE * SPRITE_MAX_SIZE;
  self->isAnimated = 0;

  // TODO: replace with for loop
  constructSprite(&self->frames[0], frames[0].frame);
  constructSprite(&self->frames[1], frames[1].frame);
}

void deconstructSprite(struct PixelDataRGB_8bit *spriteToFree) {
  free(spriteToFree);
}

void deconstructSpriteAnimation(
    struct PixelDataRGB_8bit_Animation *spriteToFree) {
  free(spriteToFree);
}
