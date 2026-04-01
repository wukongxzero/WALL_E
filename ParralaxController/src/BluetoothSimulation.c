#include <stdio.h>
#ifdef BLUETOOTH
#include "TankStatus/ATankStatusPublisher.h"
#include "TankStatus/TankStatus.h"
#include <Graphics/NcursesScreenImpl.h>
#include <Graphics/PixelDataFrame.h>
#include <Graphics/RotatePixelArt.h>
#include <Graphics/pixelArtSampleData.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
#include <ncurses.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdlib.h> // for malloc
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

// Assuming these are your custom graphics headers
// #include "Graphics/ARendederSubscriber.h" ...

int bitBackCounter = 0;
unsigned char tankStatusBufferIn[16];

int main(int argc, char **argv) {
  // ==========================================
  // 1. BLUETOOTH SETUP
  // ==========================================
  if (argc < 2) {
    fprintf(stderr, "Usage: %s <HC-06 MAC ADDRESS>\n", argv[0]);
    exit(1);
  }

  struct sockaddr_rc addr = {0};
  int s, status;
  char dest[18];
  char buf[1024];

  strncpy(dest, argv[1], 18);

  s = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);
  addr.rc_family = AF_BLUETOOTH;
  addr.rc_channel = (uint8_t)1;
  str2ba(dest, &addr.rc_bdaddr);

  printf("Connecting to %s...\n", dest);
  status = connect(s, (struct sockaddr *)&addr, sizeof(addr));

  if (status != 0) {
    perror("Connect failed");
    close(s);
    return 1;
  }
  printf("Connected! Launching NCurses Simulation in 2 seconds...\n");
  sleep(2); // Give user a moment to see connection success

  // ==========================================
  // 2. NCURSES & SPRITE SETUP
  // ==========================================
  startCurse();
  draw_rectangle(0, 0, 33, 43);
  short angleMem = 0;

  // Setup Sprites
  struct PixelDataRGB_8bit tankImage1;
  struct PixelDataRGB_8bit tankImage2;
  constructSprite(&tankImage1, samplePixelSprite1);
  constructSprite(&tankImage2, samplePixelSprite2);

  struct GraphicsObject tankSpriteControl;
  constructGraphicsObject(&tankSpriteControl, 10, 10, 16, 16);
  tankSpriteControl.defaultSprite = &tankImage2;

  struct PixelDataRGB_8bit_Animation tankAnimation;
  struct PixelDataRGB_8bit tankImageSequence[2] = {tankImage1, tankImage2};
  constructSpriteAnimation(&tankAnimation, tankImageSequence);

  struct GraphicsObjectSprite animatedTankSpriteObject;
  constructGraphicsSprite(&animatedTankSpriteObject, &tankSpriteControl);
  animatedTankSpriteObject.spriteAnimation = &tankAnimation;
  animatedTankSpriteObject.spriteAnimation->isAnimated = 1;

  struct GraphicsObject rotationTransformHandler;
  struct RotatingSprite balanceBeamAngle;
  struct PixelDataRGB_8bit transparentSpriteContainer;
  constructGraphicsObject(&rotationTransformHandler, 0, 0, 0, 0);
  constructSprite(&transparentSpriteContainer, samplePixelSprite1);
  constructGraphicsSpriteRotatableWOrigin(&balanceBeamAngle,
                                          &transparentSpriteContainer);
  extractForegroundFromSprite(&balanceBeamAngle, sampleRotateBeam, 16);

  // Setup Virtual Joystick and Data Binding
  struct TankStatusPublisher virtJoyStickStatus;
  constructVirtualJoystick(&virtJoyStickStatus);

  struct TankStatus mainTankStatus;
  constructTankStatus(&mainTankStatus);
  // Default values

  mainTankStatus.driveLeft = 127;
  mainTankStatus.driveRight = 127;
  mainTankStatus.eulerY = 0;

  // NCurses specific configurations
  cbreak();              // Disable line buffering
  noecho();              // Don't print characters back to the screen
  keypad(stdscr, TRUE);  // Enable Arrow keys
  nodelay(stdscr, TRUE); // CRITICAL: Makes getch() non-blocking!

  int ch;
  erase();
  // ==========================================
  // 3. MAIN NON-BLOCKING GAME LOOP
  // ==========================================
  while ((ch = getch()) != 'q') {

    // --- A. Process Incoming Bluetooth Data ---
    int bytesReceived = recv(s, buf, sizeof(buf) - 1, MSG_DONTWAIT);

    if (bytesReceived > 0) {
      for (int i = 0; i < bytesReceived; i++) {
        tankStatusBufferIn[bitBackCounter] = buf[i];
        bitBackCounter++;

        if (bitBackCounter == TANKSTATUS_PACKET_LENGTH) {
          readByteTankStatus(tankStatusBufferIn, TANKSTATUS_PACKET_LENGTH,
                             &mainTankStatus);
          bitBackCounter = 0;
        }
      }
    } else if (bytesReceived == 0) {
      break; // Connection dropped
    }

    // --- B. Optional Keyboard Overrides ---
    if (ch != ERR) {
      if (ch == 'w')
        mainTankStatus.driveLeft += 1;
      if (ch == 's')
        mainTankStatus.driveLeft -= 1;
      if (ch == 'e') {
        angleMem += 10;
        mainTankStatus.eulerY = (short)angleMem % 360;
      }
      if (ch == 'r') {
        angleMem -= 10;
        mainTankStatus.eulerY = (short)angleMem % 360;
      }
    } // --- C. Clear Old Frames ---
    //
    clearRenderedFrame(tankSpriteControl.boundingBox._x,
                       tankSpriteControl.boundingBox._y);
    clearRenderedFrame(rotationTransformHandler.boundingBox._x,
                       rotationTransformHandler.boundingBox._y);

    // --- D. Update Physics & Graphics States ---

    // CRITICAL FIX: Protect against Bluetooth misalignment garbage data!
    // This forces the angle to stay between -360 and 360 even if the float is
    // corrupted.
    int safeAngle = (int)mainTankStatus.eulerY;
    safeAngle = safeAngle % 360;

    rotateSprite(&balanceBeamAngle, safeAngle);
    updateAnimate(&animatedTankSpriteObject);

    // Scale the 0-255 joystick values so they fit on the screen
    int screenX = 30 + ((mainTankStatus.driveLeft - 127) / 4);
    int screenY = 20 + ((mainTankStatus.driveRight - 127) / 4);

    // Clamp coordinates so it NEVER flies off screen
    if (screenX < 0)
      screenX = 0;
    if (screenY < 0)
      screenY = 0;
    if (screenX > 70)
      screenX = 70;
    if (screenY > 30)
      screenY = 30;

    moveGraphicAbs(&rotationTransformHandler, screenX, screenY - 6);
    moveGraphicAbs(animatedTankSpriteObject.graphicsObject, screenX, screenY);

    // --- E. Render New Frames ---
    renderFrameAnimation(animatedTankSpriteObject.spriteAnimation,
                         tankSpriteControl.boundingBox._x,
                         tankSpriteControl.boundingBox._y);

    renderFrame(balanceBeamAngle.rotateSpace,
                rotationTransformHandler.boundingBox._x,
                rotationTransformHandler.boundingBox._y);

    // --- F. Stream Telemetry Back via Bluetooth ---
    static int frameCounter = 0;
    frameCounter++;

    if (frameCounter % 5 == 0) {
      unsigned char txBuffer[TANKSTATUS_PACKET_LENGTH];
      makeByteTankStatus(txBuffer, TANKSTATUS_PACKET_LENGTH, &mainTankStatus);
      send(s, txBuffer, TANKSTATUS_PACKET_LENGTH, MSG_DONTWAIT);
    }

    // --- G. Debug Info Display ---
    // CRITICAL FIX: Moved to Row 2, Column 40 so it's ALWAYS visible on screen
    mvprintw(2, 40, "BT Data -> L:%3u | R:%3u | Ang:%7.1f  ",
             mainTankStatus.driveLeft, mainTankStatus.driveRight,
             mainTankStatus.eulerY);

    refresh();
    napms(10);
  }

  // ==========================================
  // 4. CLEANUP
  // ==========================================
  endCurse();
  close(s);
  printf("Exited Simulation and Closed Bluetooth Socket.\n");

  return 0;
}
#endif
