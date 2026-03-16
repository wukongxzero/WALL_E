#pragma once
#include "AGraphicsObject.h"

class IRenderScreenImpl {
public:
  virtual void update(AGraphicsObject &obj);
};
