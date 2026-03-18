#include <Graphics/AGraphicsObject.h>
#include <Graphics/NcursesScreenImpl.h>
#include <locale.h>
#include <ncurses.h>
#include <stdio.h>
#include <string.h>

void startCurse() {
  setlocale(LC_ALL, "");
  initscr();
  noecho();
  nocbreak();
  keypad(stdscr, TRUE);
}

void endCurse() {
  refresh();
  getch();
  endwin();
}

void draw_rectangle(int y1, int x1, int y2, int x2) {
  x1 *= 2;
  x2 *= 2;
  mvhline(y1, x1, 0, x2 - x1);   /* Top line */
  mvhline(y2, x1, 0, x2 - x1);   /* Bottom line */
  mvvline(y1, x1, 0, y2 - y1);   /* Left line */
  mvvline(y1, x2, 0, y2 - y1);   /* Right line */
  mvaddch(y1, x1, ACS_ULCORNER); /* Upper left corner */
  mvaddch(y1, x2, ACS_URCORNER); /* Upper right corner */
  mvaddch(y2, x1, ACS_LLCORNER); /* Lower left corner */
  mvaddch(y2, x2, ACS_LRCORNER); /* Lower right corner */
}

void clearRenderedFrame(int offsetX, int offsetY) {
  for (int y = -4; y <= 16; y++) {
    for (int x = -4; x <= 16; x++) {
      mvaddch(y + offsetY, (x * 2) + offsetX, _);
    }
  }
  refresh();
}

void renderFrame(struct PixelDataRGB_8bit *image, int offsetX, int offsetY) {

  for (int y = 0; y < 16; y++) {
    for (int x = 0; x < 16; x++) {
      /*
      if (image->frame[y][x] == X) { // X is 219
        // Use the ncurses solid block macro
        // We use two chars because terminal cells are tall/skinny
        mvaddch(y + offsetY, (x * 2) + offsetX, X);
        mvaddch(y + offsetY, (x * 2) + offsetX + 1, X);
      } else {
        // Background/Empty space
        mvaddch(y + offsetY, (x * 2) + offsetX, _);
        mvaddch(y + offsetY, (x * 2) + offsetX + 1, _);
      }
      */
      mvaddch(y + offsetY, (x * 2) + offsetX, image->frame[y][x]);
    }
  }
  refresh();
}
void renderFrameAnimation(struct PixelDataRGB_8bit_Animation *animation,
                          int offsetX, int offsetY) {

  renderFrame(&(animation->frames[animation->frameCurrent]), offsetX, offsetY);
}

void playLoadingAnimation(struct PixelDataRGB_8bit *sampleSprite1,
                          unsigned char sprite1[16][16],
                          unsigned char sprite2[16][16]) {
  // memcpy(sampleSprite1->frame, sprite1, 16 * 16);

  constructSprite(sampleSprite1, sprite1);
  for (int i = 0; i <= 100; i++) {
    renderFrame(sampleSprite1, 100 - i, 10);
    if (i % 2) {
      constructSprite(sampleSprite1, sprite1);
      // memcpy(sampleSprite1->frame, sprite1, 16 * 16);
    } else {

      constructSprite(sampleSprite1, sprite2);
      // memcpy(sampleSprite1->frame, sprite2, 16 * 16);
    }

    napms(10); // Pause for 1000ms (1 second)
    clearRenderedFrame(100 - i, 10);
    refresh();
  }
}
