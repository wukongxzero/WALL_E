#include "Graphics/AGraphicsObject.h"
#include <stdio.h>

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
#include <Graphics/NcursesScreenImpl.h>
#include <Graphics/PixelDataFrame.h>
#include <Graphics/pixelArtSampleData.h>
#include <ncurses.h>
#include <stdlib.h> // for malloc
#include <string.h>

int main(void) {
  startCurse();
  //  init_pair(1, COLOR_GREEN, COLOR_WHITE);
  // start_color();

  draw_rectangle(0, 0, 33, 43);
  refresh();

  // setup sprite images(non moving)
  struct PixelDataRGB_8bit tankImage1;
  struct PixelDataRGB_8bit tankImage2;
  constructSprite(&tankImage1, samplePixelSprite1);
  constructSprite(&tankImage2, samplePixelSprite2);

  // setup sprite with boundingBox
  struct GraphicsObject tankSpriteControl;
  constructGraphicsObject(&tankSpriteControl, 10, 10, 16, 16);
  tankSpriteControl.defaultSprite = &tankImage2;

  renderFrame(&tankImage1, 10, 10);
  refresh();
  // setup animation
  struct PixelDataRGB_8bit_Animation tankAnimation;
  struct PixelDataRGB_8bit tankImageSequence[2] = {tankImage1, tankImage2};
  constructSpriteAnimation(&tankAnimation, tankImageSequence);

  // set up animated sprite
  struct GraphicsObjectSprite animatedTankSpriteObject;
  constructGraphicsSprite(&animatedTankSpriteObject, &tankSpriteControl);
  animatedTankSpriteObject.spriteAnimation = &tankAnimation;
  animatedTankSpriteObject.spriteAnimation->isAnimated = 1;

  moveGraphicAbs(&tankSpriteControl, 10, 10);

  // gameloop
  // playLoadingAnimation(&tankImage1, samplePixelSprite1, samplePixelSprite2);
  for (int i = 0; i <= 42; i++) {
    moveGraphicByVal(&tankSpriteControl, 1, 0);
    update(&tankSpriteControl);

    renderFrame(tankSpriteControl.defaultSprite,
                tankSpriteControl.boundingBox._x,
                tankSpriteControl.boundingBox._y);
    napms(50); // Pause for 1000ms (1 second)
    clearRenderedFrame(tankSpriteControl.boundingBox._x,
                       tankSpriteControl.boundingBox._y);
  }

  for (int i = 42; i >= 0; i--) {

    moveGraphicByVal(&tankSpriteControl, -1, 0);
    updateAnimate(&animatedTankSpriteObject);
    renderFrameAnimation(animatedTankSpriteObject.spriteAnimation,
                         tankSpriteControl.boundingBox._x,
                         tankSpriteControl.boundingBox._y);
    //   fix I just found, becuase sprite is a pointer i should not pass this
    //   an object, pass the .sprite as itslef
    //   renderFrameAnimation(tankAnimatedObject.spriteAnimation,
    //                       tankObject.boundingBox._x,
    //                       tankObject.boundingBox._y);
    napms(100); // Pause for 1000ms (1 second)
    clearRenderedFrame(tankSpriteControl.boundingBox._x,
                       tankSpriteControl.boundingBox._y);
  }
  // free(tank);
  // free(&tankObject);

  char ch;

  animatedTankSpriteObject.spriteAnimation->isAnimated = false;
  mvprintw(42, 32, "Button Press");
  refresh();
  nodelay(stdscr, TRUE);
  while (1) { // Press 'q' to quit
              //
    int ch = getch();
    switch (ch) {
    case ERR:
      // Shows the actual number (e.g., 259)
      updateAnimate(&animatedTankSpriteObject);
      renderFrameAnimation(animatedTankSpriteObject.spriteAnimation,
                           tankSpriteControl.boundingBox._x,
                           tankSpriteControl.boundingBox._y);

      refresh();
    case 'w':
      mvprintw(42 + 1, 32, "Key Code: %d  ", ch);
      // moveGraphicByVal(&tankSpriteControl, 0, 1);

      break;
    }
    /*
refresh();
switch (ch) {
case KEY_UP:
// Move tank up
mvprintw(10, 10, "KEYUP");
moveGraphicByVal(&tankObject, 1, 0);
break;
case KEY_DOWN:
// Move tank down
moveGraphicByVal(&tankObject, -1, 0);
break;
case KEY_LEFT:
// Move tank left
moveGraphicByVal(&tankObject, 0, 1);
break;
case KEY_RIGHT:
// Move tank right
moveGraphicByVal(&tankObject, 0, -1);
break;
case ' ':
// Spacebar: Fire!
break;
case ERR:
// No key was pressed (only happens if nodelay or timeout is set)
break;
}

update(&tankObject);

*/
  }

  // renderFrame(&tank, 0, 10);

  refresh();

  // free(tank);
  // Draw a box around the window
  // 0, 0 tells ncurses to use default border characters
  // Refresh to show the box

  endCurse();
}

#endif
