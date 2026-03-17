#include <Graphics/AGraphicsObject.h>
#include <Graphics/NcursesScreenImpl.h>
#include <locale.h>
#include <ncurses.h>
#include <stdio.h>

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
  for (int y = 0; y < 16; y++) {
    for (int x = 0; x < 16; x++) {
      mvaddch(y + offsetY, (x * 2) + offsetX, _);

      //
      //
      // if (image->frame[y][x] == X) {
      //  // Use 'attron' with a color pair to draw a colored block
      //  mvwaddstr(stdscr, y + 2, (x * 2) + 2, " ");
      //  mvprintw(x + 20, y + 20,
      //           "X"); // Prints at Row 10, Col 20

      //} else {
      //  mvwaddstr(stdscr, y + 2, (x * 2) + 2, " ");
      //  mvprintw(x + 20, y + 20,
      //           "~X"); // Prints at Row 10, Col 20
      //}
    }
  }
  refresh();
}

void renderFrame(struct PixelDataRGB_8bit *image, int offsetX, int offsetY) {

  for (int y = 0; y < 16; y++) {
    for (int x = 0; x < 16; x++) {
      mvaddch(y + offsetY, (x * 2) + offsetX, image->frame[y][x]);

      //
      //
      // if (image->frame[y][x] == X) {
      //  // Use 'attron' with a color pair to draw a colored block
      //  mvwaddstr(stdscr, y + 2, (x * 2) + 2, " ");
      //  mvprintw(x + 20, y + 20,
      //           "X"); // Prints at Row 10, Col 20

      //} else {
      //  mvwaddstr(stdscr, y + 2, (x * 2) + 2, " ");
      //  mvprintw(x + 20, y + 20,
      //           "~X"); // Prints at Row 10, Col 20
      //}
    }
  }
  refresh();
}
