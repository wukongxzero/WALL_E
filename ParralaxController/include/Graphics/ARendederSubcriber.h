#pragma once
#include "AGraphicsObject.h"
#include "ARendederSubcriber.h"
#include "IRendererScreenImpl.h"

class ARendererSub {
public:
  IRenderScreenImpl renderImpl;
  AGraphicsObject *graphicsList[];
};
