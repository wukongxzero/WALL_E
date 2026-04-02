#ifndef SD_CARD_IO
#define SD_CARD_IO
#include <TankStatus/TankStatus.h>
#define SD_DO 22  // MISO
#define SD_CLK 23 // SCLK
#define SD_DI 24  // MOSI
#define SD_CS 25  // CS

void initSDCardLogger();
void asyncLogTankStatusToCSV(void *arg);

#endif
