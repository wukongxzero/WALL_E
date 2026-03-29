// NOTE: so my sample data takes up about 1kb for the gear and maybe .5 kb for
// the beam ... meaning I've been getting a stack overflow The only way around
// this would be to have a eeprom array data
#include "pixelArtSampleData.h"
#include "simpletools.h"
#include <stdlib.h>

#define START_LOCATION 32768
#define BAUD 115200
// unsigned char buffer[1024];
// unsigned char beam[256];
//  unsigned char anim[256];
//  unsigned char anim2[256];
unsigned char buffer;

int main() {
  print("trying to read");
  //**************GEAR*********************************
  // unsigned char *buffer = malloc(1024);
  // ee_getStr(buffer, 1024, START_LOCATION);
  print("startforloop");
  for (int i = 0; i < 1024; i++) {
    if (i % 32 == 0) {
      print("\n");
    }
    ee_getStr(&buffer, 1, START_LOCATION + i);
    print("%c", buffer);
    // print("%u", buffer[i]);
  }
  //  free(buffer);
  //**************GEAR*********************************
  //**************BEAM*********************************
  // unsigned char *beam = malloc(256);

  print("\ntestBeam\n");
  for (int i = 0; i < 256; i++) {
    if (i % 16 == 0) {
      print("\n");
    }
    ee_getStr(&buffer, 1, START_LOCATION + 1024 + i);
    print("%c", buffer);
  }
  // free(beam);
  //**************BEAM*********************************
  //**************ANIM*********************************
  //
  print("\ntestBeam\n");
  for (int i = 0; i < 256; i++) {
    if (i % 16 == 0) {
      print("\n");
    }
    ee_getStr(&buffer, 1, START_LOCATION + 1024 + 256 + i);
    print("%c", buffer);
  }
  //**************ANIM*********************************
  //**************ANIM2*********************************
  print("\ntestBeam\n");
  for (int i = 0; i < 256; i++) {
    if (i % 16 == 0) {
      print("\n");
    }
    ee_getStr(&buffer, 1, START_LOCATION + 1024 + 256 + 256 + i);
    print("%c", buffer);
  }
  //**************ANIM2*********************************
}
