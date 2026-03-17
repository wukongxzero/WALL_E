#include <Graphics/NcursesScreenImpl.h>
#include <ncurses.h>

void draw_rectangle(int y1, int x1, int y2, int x2) {
  mvhline(y1, x1, 0, x2 - x1);   /* Top line */
  mvhline(y2, x1, 0, x2 - x1);   /* Bottom line */
  mvvline(y1, x1, 0, y2 - y1);   /* Left line */
  mvvline(y1, x2, 0, y2 - y1);   /* Right line */
  mvaddch(y1, x1, ACS_ULCORNER); /* Upper left corner */
  mvaddch(y1, x2, ACS_URCORNER); /* Upper right corner */
  mvaddch(y2, x1, ACS_LLCORNER); /* Lower left corner */
  mvaddch(y2, x2, ACS_LRCORNER); /* Lower right corner */
}
