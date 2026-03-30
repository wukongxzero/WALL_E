#include "Graphics/AGraphicsObject.h"
#include "Graphics/RotatePixelArt.h"
#include "JoyStickPublisher.h"
#include "Propellor/PrintStatusSubscriber.h"
#include "TankStatus/ATankStatusPublisher.h"
#include "TankStatus/TankStatus.h"
#include <Graphics/RotatePixelArt.h>
#include <stdio.h>

// todo:place in macros
#define HC05_RX 11
#define HC05_TX 10
#define HC05_EN 12
#define HC05_BAUD 9600 // 115200
// HC05_DATA_BAUD
// define where eeprom sprites are stored
#define BIT32_SPRITE_SIZE 1024
#define BIT16_SPRITE_SIZE 256

#define GEAR_LOCATION 32768
#define BEAM_LOCATION GEAR_LOCATION + BIT32_SPRITE_SIZE
#define TANK_ANIMATION_F1_LOCATION BEAM_LOCATION + BIT16_SPRITE_SIZE
#define TANK_ANIMATION_F2_LOCATION                                             \
  TANK_ANIMATION_F1_LOCATION + BIT16_SPRITE_SIZE

#ifndef SIMULATION_SCREEN
#include "simpletools.h"
#include <Graphics/ARendederSubscriber.h>
#include <Graphics/AsyncTankStatusGraphics.h>
#include <JoyStickPublisher.h>
#include <Propellor/PrintStatusSubscriber.h>
#include <Propellor/SPI.h>
#include <Propellor/ST7796S.h>
#include <Propellor/hc05.h>
#include <SPIRendering.h>
#include <TankStatus/ABytePublisher.h>
#include <TankStatus/TankStatus.h>
#include <TankStatusBTAdopt.h>
#include <propeller.h>

#include <math.h>
// NOTE: not using the heap allows better preformance
//
struct JoyStickPublisher wheelDriver;
struct TankStatusPublisher tsPublisher; // legacy for config 1

struct TankStatusPublisher tsPublisherJoystick;
struct TankStatusPublisher tsPublisherhc05;

struct TankStatus printTankStatusVar; // legacy...originally used to print but
                                      // then I used to test joystick mapping
struct TankStatus displayTankStatus;  // act as a subscriber for tft
struct TankStatus
    btLocalStatus; // act as a subscriber for the bluetooth...the publisher
                   // structure status is in built into the tank status

struct CoreMapping graphicsCoreBeam;
struct CoreMapping graphicsCoreWheelLeft;
struct CoreMapping graphicsCoreWheelRight;
struct CoreMapping graphicsCoreBody;

//
// unsigned int cogStackBTRead[128];
unsigned int cogDisplay[800];
volatile int spriteLock;

TankStatusBTAdopter hc05ToTankStatus;

struct GroupMapping gm;
struct DriveLeftMapping wlLeft;
struct DriveRightMapping wlright;
struct AngleMapping beamMiddle;
struct StaticNoMapping tankBody;

// NOTE:so cool thing is apparently I can share allocated memory for same
// sprited
struct SparseElement wheelLeftStack[288];
// struct SparseElement wheelRightStack[288];
struct SparseElement beamStack[32];
struct SparseElement tankStack[72];
// int config1();

int main(void) {
  spriteLock = locknew();
  // return config1(); //exists for code refrence without switching branches
  // test_adc_heartbeat();
  // initialize hardware peripherals(adc tftscreen  spi hc05 usart )
  // //TODO: declare pins with macro...
  adc_init(21, 20, 19, 18);
  waitcnt(CNT + CLKFREQ / 2); // Wait 0.5 seconds for ADC to stabilize
  // tft_init(0, 4, 3, 2, 1);
  // tft_fillScreen(RGB565(0, 0, 0));

  hc05_t btModule;
  hc05_init(&btModule, HC05_RX, HC05_TX, HC05_EN, HC05_DATA_BAUD);
  //  printTankStatus(
  //       &displayTankStatus); // should have update tank status based on cog
  //  publishing/updating  this subscriber

  constructTankStatus(&displayTankStatus);
  displayTankStatus.eulerY = 0;
  constructTankStatusPublisher(&tsPublisher);
  constructJoystick(&wheelDriver, 0, 2, &tsPublisher);

  constructTankStatusBTAdopter(&hc05ToTankStatus, &btLocalStatus, &tsPublisher,
                               &btModule);

  constructCoreMap(&graphicsCoreBeam, BEAM_LOCATION, BIT16_SPRITE_SIZE,
                   &displayTankStatus);
  constructCoreMap(&graphicsCoreBody, TANK_ANIMATION_F1_LOCATION,
                   BIT16_SPRITE_SIZE, &displayTankStatus);

  constructCoreMap(&graphicsCoreWheelLeft, GEAR_LOCATION, BIT32_SPRITE_SIZE,
                   &displayTankStatus);
  constructCoreMap(&graphicsCoreWheelRight, GEAR_LOCATION, BIT32_SPRITE_SIZE,
                   &displayTankStatus);

  constructAngleMapping(&beamMiddle, &graphicsCoreBeam, beamStack);
  // print("beam%i\n", beamMiddle.sprite.elementCount);
  constructDriveLeftMapping(&wlLeft, &graphicsCoreWheelLeft, wheelLeftStack);
  // print("gearSprite%i\n", wlLeft.sprite.elementCount);
  constructDriveRightMapping(&wlright, &graphicsCoreWheelRight, wheelLeftStack);
  // print("gearSprite%i\n", wlright.sprite.elementCount);
  constructStaticSpriteNoMap(&tankBody, &graphicsCoreBody, tankStack);
  ///* Program size is 15880 bytes

  gm.leftCtrl = &wlLeft;
  gm.rightCtrl = &wlright;
  gm.angleCtrl = &beamMiddle;
  gm.tankBase = &tankBody;

  gm.leftCtrl->sprite.screenLocationY = 30;
  gm.leftCtrl->sprite.screenLocationX = 20;

  gm.rightCtrl->sprite.screenLocationY = TFT_HEIGHT - 70;
  gm.rightCtrl->sprite.screenLocationX = 20; // TFT_WIDTH - 100;

  gm.angleCtrl->sprite.screenLocationY = TFT_HEIGHT / 2 - 15;
  gm.angleCtrl->sprite.screenLocationX = TFT_WIDTH / 2;

  gm.tankBase->sprite.screenLocationY = TFT_HEIGHT / 2 - 20;
  gm.tankBase->sprite.screenLocationX = 60;

  // print("Subscriber TankStatus for display : time 1");
  // subscribe(hc05ToTankStatus.publisher,
  //          &displayTankStatus); // in this configuration the bluetooth recv
  calibrateCenter(&wheelDriver);

  subscribe(&tsPublisher, &displayTankStatus);
  subscribe(wheelDriver.publisher, &tsPublisher._localStatus);

  //  readTankStatusBT(&hc05ToTankStatus); // should also notify publisher
  // int cog_id = cogstart(readTankStatusBT, (void *)&hc05ToTankStatus,

  int cog_id_2 = cogstart(AsyncStartTankStatusRenderMap, (void *)&gm,
                          cogDisplay, sizeof(cogDisplay));
  pause(1000);
  // start display status
  // add delay between start cog and cog 0 print
  while (1) {

    waitcnt(CNT + CLKFREQ);
    readJoystick(&wheelDriver);

    while (lockset(spriteLock))
      ;

    wheelDriver.publisher->_localStatus.eulerY += 1;
    notify(wheelDriver.publisher);

    lockclr(spriteLock);

    // print("e%u\n", round(displayTankStatus.driveRight));
    // print("e%u\n", round(displayTankStatus.driveLeft));

    //    print("%u\n", wheelDriver.publisher->_localStatus.driveRight);

    // Send the new positions to the render Cog
  }

  // printTankStatus(
  //    &displayTankStatus); // should have update tank status based on cog
  // publishing/updating  this subscriber
  //

  // TODO: need to map balance beam to euler Y
  return 0;
}

#else
#include <Graphics/NcursesScreenImpl.h>
#include <Graphics/PixelDataFrame.h>
#include <Graphics/pixelArtSampleData.h>
#include <ncurses.h>
#include <stdlib.h> // for malloc
#include <string.h>
// must define

int configSimulation();
int main(void) {
  configSimulation();
  return 0;
}

#endif

#ifndef SIMULATION_SCREEN

#else

int configSimulation() {
#ifdef PROPELLOR rdlong atomicLong;
#endif
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

  // set up rotating sprite
  struct GraphicsObject rotationTransformHandler;
  struct RotatingSprite balanceBeamAngle;
  struct PixelDataRGB_8bit transparentSpriteContainer;
  constructGraphicsObject(&rotationTransformHandler, 0, 0, 0, 0);
  constructSprite(&transparentSpriteContainer, samplePixelSprite1);
  // construct must come first
  constructGraphicsSpriteRotatableWOrigin(&balanceBeamAngle,
                                          &transparentSpriteContainer);
  extractForegroundFromSprite(&balanceBeamAngle, sampleRotateBeam, 16);

  moveGraphicAbs(&tankSpriteControl, 10, 10);
  // gameloop
  // playLoadingAnimation(&tankImage1, samplePixelSprite1,
  // samplePixelSprite2);
  /*
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
    //   fix I just found, becuase sprite is a pointer i should not pass
    //   this an object, pass the .sprite as itslef
    //   renderFrameAnimation(tankAnimatedObject.spriteAnimation,
    //                       tankObject.boundingBox._x,
    //                       tankObject.boundingBox._y);
    napms(100); // Pause for 1000ms (1 second)
    clearRenderedFrame(tankSpriteControl.boundingBox._x,
                       tankSpriteControl.boundingBox._y);
  }
  napms(1000);

  // rotate visual test
  clearRenderedFrame(rotationTransformHandler.boundingBox._x,
                     rotationTransformHandler.boundingBox._y);
  rotateSprite(&balanceBeamAngle, 0);
  moveGraphicAbs(&rotationTransformHandler, 20, 15);
  clearRenderedFrame(rotationTransformHandler.boundingBox._x,
                     rotationTransformHandler.boundingBox._y);
  renderFrame(rotationTransformHandler.defaultSprite,
              rotationTransformHandler.boundingBox._x,
              rotationTransformHandler.boundingBox._y);

  for (int i = 0; i <= 360; i++) {
    rotateSprite(&balanceBeamAngle, i);
    clearRenderedFrame(rotationTransformHandler.boundingBox._x,
                       rotationTransformHandler.boundingBox._y);
    renderFrame(balanceBeamAngle.rotateSpace,
                rotationTransformHandler.boundingBox._x,
                rotationTransformHandler.boundingBox._y);

    napms(10);
  }
  */

  for (int i = 0; i <= 180; i++) {
    clearRenderedFrame(rotationTransformHandler.boundingBox._x,
                       rotationTransformHandler.boundingBox._y);
    clearRenderedFrame(tankSpriteControl.boundingBox._x,
                       tankSpriteControl.boundingBox._y);

    rotateSprite(&balanceBeamAngle, 90 + i);
    updateAnimate(&animatedTankSpriteObject);
    moveGraphicAbs(&rotationTransformHandler, 10, 2);

    renderFrameAnimation(animatedTankSpriteObject.spriteAnimation,
                         tankSpriteControl.boundingBox._x,
                         tankSpriteControl.boundingBox._y);
    renderFrame(balanceBeamAngle.rotateSpace,
                rotationTransformHandler.boundingBox._x,
                rotationTransformHandler.boundingBox._y);
    napms(10);
  }

  int angleToTurn = 90;

  printw("ASCII Value Inspector\n");
  printw("---------------------\n");
  printw("Press any key to see its value. Press 'q' to quit.\n\n");
  refresh();
  cbreak();             // 1. Disable line buffering (no "Enter" needed)
  noecho();             // 2. Don't print the character back to the screen
  keypad(stdscr, TRUE); // 3. Enable Arrow keys and Function keys
                        //
  struct TankStatusPublisher virtJoyStickStatus;
  constructVirtualJoystick(&virtJoyStickStatus);
  struct TankStatus mainTankStatus;
  constructTankStatus(&mainTankStatus);
  subscribe(virtualJoystickOne.tankPub, &mainTankStatus);

  int ch;
  for (int i = 0; i < 3; i++) {
    erase();
    while ((ch = getch()) != 'q') {
      // Clear the previous line and print the new values
      clearRenderedFrame(tankSpriteControl.boundingBox._x,
                         tankSpriteControl.boundingBox._y);

      if (ch == 'd') {
        // moveGraphicByVal(&tankSpriteControl, 1, 0);
        // moveGraphicByVal(&rotationTransformHandler, 1, 0);
        virtualJoystickOne.virtJoystickX += 1;
      }
      if (ch == 'a') {
        // moveGraphicByVal(&tankSpriteControl, -1, 0);

        // moveGraphicByVal(&rotationTransformHandler, -1, 0);
        virtualJoystickOne.virtJoystickX -= 1;
      }

      if (ch == 'w') {
        // moveGraphicByVal(&tankSpriteControl, 0, -1);
        // moveGraphicByVal(&rotationTransformHandler, 0, -1);
        virtualJoystickOne.virtJoyStickY -= 1;
      }
      if (ch == 's') {
        // moveGraphicByVal(&rotationTransformHandler, 0, 1);
        // moveGraphicAbsveGraphicByVal(&tankSpriteControl, 0, 1);
        virtualJoystickOne.virtJoyStickY += 1;
      }
      if (ch == 'r') {
        angleToTurn += 5;
        virtualJoystickOne.tankPub->_localStatus.eulerY = angleToTurn;
      }

      if (ch == 'e') {
        angleToTurn -= 5;
        virtualJoystickOne.tankPub->_localStatus.eulerY = angleToTurn;
      }

      virtualJoystickOne.tankPub->_localStatus.driveLeft =
          virtualJoystickOne.virtJoystickX;

      virtualJoystickOne.tankPub->_localStatus.driveRight =
          virtualJoystickOne.virtJoyStickY;

      virtualJoystickOne.tankPub->_localStatus.eulerY = angleToTurn;

      notify(virtualJoystickOne.tankPub);

      rotateSprite(&balanceBeamAngle, mainTankStatus.eulerY);
      updateAnimate(&animatedTankSpriteObject);
      moveGraphicAbs(&rotationTransformHandler, mainTankStatus.driveLeft + 10,
                     mainTankStatus.driveRight + 4);
      moveGraphicAbs(animatedTankSpriteObject.graphicsObject,
                     mainTankStatus.driveLeft + 10,
                     mainTankStatus.driveRight + 10);
      renderFrame(tankSpriteControl.defaultSprite,
                  tankSpriteControl.boundingBox._x,
                  tankSpriteControl.boundingBox._y);

      renderFrame(balanceBeamAngle.rotateSpace,
                  rotationTransformHandler.boundingBox._x,
                  rotationTransformHandler.boundingBox._y);

      clrtoeol();

      // %d prints the decimal (ASCII) value
      // %c prints the actual character representation
      // %x prints the Hexadecimal value
      // printw("Key: '%c' | Decimal: %d | Hex: 0x%X \n",
      //        (ch >= 32 && ch <= 126) ? ch : '?', ch, ch);
      // printw("leftDrive '%i' | rightDrive '%i' | eulerY '%f'\n",
      //        virtualJoystickOne.tankPub->_localStatus.driveRight,
      //        virtualJoystickOne.tankPub->_localStatus.driveLeft,
      //        virtualJoystickOne.tankPub->_localStatus.eulerY);
      // printw("leftDrive '%i' | rightDrive '%i' | eulerY '%f'\n",
      //        mainTankStatus.driveRight, mainTankStatus.driveLeft,
      //        mainTankStatus.eulerY);
      // printw("subscriberCount:%i",
      // virtualJoystickOne.tankPub->subscriberCount);

      refresh();
    }
  }

  // int result = pthread_create(&thread_id, NULL, async_notify_worker, &pub);
  //
  //
  pthread_t thread_id;
  // int result =
  //    pthread_create(&thread_id, NULL, async_notify_worker_virtualJoystick,
  //                   &virtualJoystickOne);

  renderFrameAnimation(animatedTankSpriteObject.spriteAnimation,
                       tankSpriteControl.boundingBox._x,
                       tankSpriteControl.boundingBox._y);

  pthread_join(thread_id, NULL);
  // TODO:test button on seperate pthread.h
  /*
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
  }
  */
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

  // renderFrame(&tank, 0, 10);

  refresh();

  // free(tank);
  // Draw a box around the window
  // 0, 0 tells ncurses to use default border characters
  // Refresh to show the box

  endCurse();
}

#endif
