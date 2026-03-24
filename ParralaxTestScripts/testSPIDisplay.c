#include "ST7796S.h"
#include <stdio.h>

int main() {
  // Initialize pins (CS, SCK, MOSI, DC, RST)
  tft_init(0, 4, 3, 2, 1);

  printf("TFT Initialized. Clearing screen...\n");
  tft_fillScreen(RGB565(0, 0, 0)); // Black

  // Render a simple pattern
  for (int x = 0; x < 100; x++) {
    for (int y = 0; y < 100; y++) {
      tft_drawPixel(x + 50, y + 50, RGB565(255, 0, 0)); // Red square
    }
  }

  while (1) {
    // Main loop
  }
  return 0;
}
