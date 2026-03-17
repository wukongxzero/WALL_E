#include <stdio.h>

#define SIMULATION_SCREEN

#ifndef SIMULATION_SCREEN
#include "simpletools.h"
#include <Graphics/ARendederSubscriber.h>
#include <ITankStatus.h>
#include <Propellor/SPI.h>
#include <propeller.h>

#include <TankStatus/ABytePublisher.h>

int main(void) {
  for (;;) {
  }
  return 0;
}
#else
#define INCLUDE_SAMPLES
#include <Graphics/NcursesScreenImpl.h>
#include <Graphics/PixelDataFrame.h>
#include <ncurses.h>
#include <stdlib.h> // for malloc
#include <string.h>

int main(void) {
  startCurse();
  //  init_pair(1, COLOR_GREEN, COLOR_WHITE);
  // start_color();

  int height = 20; // Representing your Y-axis
  int width = 60;  // Representing your X-axis
  int start_y = 2; // Offset from top
  int start_x = 5; // Offset from left
  draw_rectangle(0, 0, 33, 43);
  refresh();

  struct PixelDataRGB_8bit tank;

  playLoadingAnimation(&tank, &samplePixelSprite1, &samplePixelSprite2);
  char ch;
  while ((ch = getch()) != 'q') { // Press 'q' to quit
    switch (ch) {
    case KEY_UP:
      // Move tank up
      break;
    case KEY_DOWN:
      // Move tank down
      break;
    case KEY_LEFT:
      // Move tank left
      break;
    case KEY_RIGHT:
      // Move tank right
      break;
    case ' ':
      // Spacebar: Fire!
      break;
    case ERR:
      // No key was pressed (only happens if nodelay or timeout is set)
      break;
    }
  }

  renderFrame(&tank, 0, 10);

  refresh();

  // free(tank);
  // Draw a box around the window
  // 0, 0 tells ncurses to use default border characters
  // Refresh to show the box

  endCurse();
}

#endif
