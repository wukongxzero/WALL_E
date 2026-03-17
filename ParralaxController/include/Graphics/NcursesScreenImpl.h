// define SIMULATION_SCREEN if running on terminal
#define SIMULATION_SCREEN

#ifdef SIMULATION_SCREEN
// #include "Graphics/ARendederSubscriber.h"
#include <Graphics/AGraphicsObject.h>
#include <Graphics/PixelDataFrame.h>
#include <ncurses.h>

void startCurse();
void endCurse();
void draw_rectangle(int y1, int x1, int y2, int x2);
void clearRenderedFrame(int x, int y);
void renderFrame(struct PixelDataRGB_8bit *frame, int offsetX, int offsetY);

#endif
