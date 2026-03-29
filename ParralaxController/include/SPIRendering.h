#ifndef RENDER_SPI_H
#define RENDER_SPI_H
#include <Graphics/RotatePixelArt.h>
#include <Propellor/ST7796S.h>
void rotateAndRenderSparse(struct SparsePointSprite *self, int screenX,
                           int screenY, int angle, int scale, int offsetX,
                           int offsetY);
void renderSparseSprite(struct SparsePointSprite *self, int screenX,
                        int screenY, int scale);

void renderSparsePointSpriteColor(struct SparsePointSprite *self, int screenX,
                                  int screenY, int scale, int color);

void ClearSparseSprite(struct SparsePointSprite *self, int screenX, int screenY,
                       int scale);
#endif
