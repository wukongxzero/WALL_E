#pragma once
#include "../Propellor/SPI.h"
#include "AGraphicsObject.h"
#include "IRendererScreenImpl.h"

class SpiScreenImpl : public IRenderScreenImpl {
private:
  TankBot::SPI _spi;

public:
  void update(AGraphicsObject &obj);
};
