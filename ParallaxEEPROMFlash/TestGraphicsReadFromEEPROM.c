// NOTE: so my sample data takes up about 1kb for the gear and maybe .5 kb for
// the beam ... meaning I've been getting a stack overflow The only way around
// this would be to have a eeprom array data
#include "pixelArtSampleData.h"
#include "simpletools.h"

#define START_LOCATION 32768
#define BAUD 115200
unsigned char buffer[1024];
unsigned char beam[256];
unsigned char anim[256];
unsigned char anim2[256];

int main() {
  print("trying to read");

  ee_getStr(buffer, 1024, START_LOCATION);
  ee_getStr(beam, 256, START_LOCATION + 1024);
  ee_getStr(anim, 256, START_LOCATION + 1024 + 256);
  ee_getStr(anim2, 256, START_LOCATION + 1024 + 256 + 256);
  // TODO: add tank sprite in
  // ee_putStr(&samplePixelSprite1, 256, START_LOCATION + 1024 + 256);
  //
  print("startforloop");
  for (int i = 0; i < 1024; i++) {
    if (i % 32 == 0) {
      print("\n");
    }
    print("%u ", buffer[i]);
  }

  print("\ntestBeam\n");
  for (int i = 0; i < 256; i++) {
    if (i % 16 == 0) {
      print("\n");
    }
    print("%u ", beam[i]);
  }

  print("\nanim\n");
  for (int i = 0; i < 256; i++) {
    if (i % 16 == 0) {
      print("\n");
    }
    print("%u ", anim[i]);
  }

  print("\ntestBeam\n");
  for (int i = 0; i < 256; i++) {
    if (i % 16 == 0) {
      print("\n");
    }
    print("%u ", anim2[i]);
  }
}
