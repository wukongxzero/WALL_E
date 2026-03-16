#pragma once
// define SIMULATION_SCREEN if running on terminal
#define SIMULATION_SCREEN

#ifdef SIMULATION_SCREEN
#include "IRendererScreenImpl.h"
#include <ncurses.h>

// this is the impl part of a bridge pattern while acting as an adopter pattern
// for ncurses

class NcursesScreenImpl : public IRenderScreenImpl {
public:
  void update(AGraphicsObject &obj);
};

#endif
