// #pragma once
// class ARendererSub {
// public:
//   IRenderScreenImpl renderImpl;
//   AGraphicsObject *graphicsList[];
// };
//
#ifndef RENDER_TANKSTATUS
#define RENDER_TANKSTATUS
#define MAX_RENDER_OBJECT 16

#include <Graphics/AGraphicsObject.h>
#include <Propellor/SPI.h>
#include <TankStatus/TankStatus.h>

// I guess this structure can update the graphics subscribers so kindof pub/sub
// but i would think of this as just a list manager that grabs postions and
// updates them via the SPI renderer
struct GraphicsTankStatusAdopterSubscriber {
  // use this to subscribe to a structure with a TankStatus publisher
  struct TankStatus *syncTank;
  struct GraphicsObject *objectOrder[MAX_RENDER_OBJECT];

  unsigned char objectCount;
};

void move(struct GraphicsTankStatusAdopterSubscriber *self, unsigned char _id,
          unsigned short x, unsigned short y);

void moveBy(struct GraphicsTankStatusAdopterSubscriber *self, unsigned char _id,
            signed short byX, signed short byY);

// update graphics animations/movements and render them to spiscreenpointer
#ifndef SIMULATION_SCREEN
void update(struct GraphicsTankStatusAdopterSubscriber *self,
            struct TankBotSPI spiScreenInterface);

#else
void updateCurses(GraphicsTankStatusAdopterSubscriber *self);
#endif

#endif
