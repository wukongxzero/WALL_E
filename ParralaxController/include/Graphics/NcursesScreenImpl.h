#pragma once
// define SIMULATION_SCREEN if running on terminal
#define SIMULATION_SCREEN

#ifdef SIMULATION_SCREEN
#include "ARendederSubcriber.h"
#include <ncurses.h>

void draw_rectangle(int y1, int x1, int y2, int x2);
#endif
