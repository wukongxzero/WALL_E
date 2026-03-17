<<<<<<< Updated upstream
#include <Graphics/ARendederSubscriber.h>
#include <Propellor/SPI.h>
#include <TankStatus/ABytePublisher.h>
#include <propeller.h>
#include <simpletools.h>
=======

#include <TankStatus/ABytePublisher.h>
#include <TankStatus/IOutImpl.h>
#include <TankStatus/ITankStatus.h>

>>>>>>> Stashed changes
#include <stdio.h>

#define SIMULATION_SCREEN

#ifndef SIMULATION_SCREEN
#include "simpletools.h"
#include <propeller.h>
int main(void) {
  for (;;) {
  }
  return 0;
}
#else
#include <Graphics/NcursesScreenImpl.h>
#include <ncurses.h>
int main(void) {
  initscr();
  noecho();
  nocbreak();
  keypad(stdscr, TRUE);

  draw_rectangle(5, 10, 15, 40);
  mvwprintw(stdscr, 10, 10, "hello curse");
  refresh();
  getch();
  endwin();
}

#endif
