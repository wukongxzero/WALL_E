// define SIMULATION_SCREEN if running on terminal

#ifdef SIMULATION_SCREEN

#ifndef SIMULATION_SCREEN_H
#define SIMULATION_SCREEN_H

#include "TankStatus/ATankStatusPublisher.h"
// #include "Graphics/ARendederSubscriber.h"
#include <Graphics/AGraphicsObject.h>
#include <Graphics/PixelDataFrame.h>
#include <ncurses.h>
#include <pthread.h> // For threading support
//
////declare this structure
struct AsyncInputManager {
  struct TankStatusPublisher *tankPub;
  volatile float virtJoystickX;
  volatile float virtJoyStickY;
};

extern struct AsyncInputManager virtualJoystickOne;

void startCurse();
void endCurse();
void draw_rectangle(int y1, int x1, int y2, int x2);
void clearRenderedFrame(int x, int y);
void renderFrame(struct PixelDataRGB_8bit *frame, int offsetX, int offsetY);
void renderFrameAnimation(struct PixelDataRGB_8bit_Animation *animation,
                          int offsetX, int offsetY);

void playLoadingAnimation(struct PixelDataRGB_8bit *sampleSprite1,
                          unsigned char sprite1[16][16],
                          unsigned char sprite2[16][16]);

void constructVirtualJoystick(struct TankStatusPublisher *pub);
void *async_notify_worker_virtualJoystick(void *arg);
#endif
#endif
