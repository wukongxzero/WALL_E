// NOTE: so my sample data takes up about 1kb for the gear and maybe .5 kb for
// the beam ... meaning I've been getting a stack overflow The only way around
// this would be to have a eeprom array data
#include "pixelArtSampleData.h"
#include "simpletools.h"

#define START_LOCATION 32768

int main() {
  ee_putStr(&gear32x32_slim, 1024, START_LOCATION);
  ee_putStr(&beam_16x16, 256, START_LOCATION + 1024);
  ee_putStr(&samplePixelSprite1, 256, START_LOCATION + 1024 + 256);
  ee_putStr(&samplePixelSprite2, 256, START_LOCATION + 1024 + 256 + 256);
  print("flashed");

  unsigned char beam[256];
  ee_getStr(beam, 256, START_LOCATION + 1024);
  for (int i = 0; i < 256; i++) {
    if (i % 16 == 0) {
      print("\n");
    }
    print("%u", beam[i]);
  }

  unsigned char animTank[256];
  ee_getStr(animTank, 256, START_LOCATION + 1024 + 256);
  for (int i = 0; i < 256; i++) {
    if (i % 16 == 0) {
      print("\n");
    }
    print("%u", animTank[i]);
  }

  unsigned char animTank2[256];
  ee_getStr(animTank2, 256, START_LOCATION + 1024 + 256 + 256);
  for (int i = 0; i < 256; i++) {
    if (i % 16 == 0) {
      print("\n");
    }
    print("%u", animTank2[i]);
  }
  // TODO: add tank sprite in
  // ee_putStr(&samplePixelSprite1, 256, START_LOCATION + 1024 + 256);
}
