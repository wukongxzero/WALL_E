#include "ST7796S.h"
#include <stdio.h>

int main() {
  // Initialize pins (CS, SCK, MOSI, DC, RST)
  tft_init(0, 4, 3, 2, 1);

  printf("TFT Initialized. Clearing screen...\n");
  tft_fillScreen(RGB565(0, 0, 0)); // Black
                                   //// Write some text
  // tft_writeString(10, 10, "PROPELLER TANK CTRL", RGB565(255, 255, 255),
  //                 RGB565(0, 0, 40), 2);

  tft_writeString(10, 50, "System Status: ONLINE", RGB565(0, 255, 0),
                  RGB565(0, 0, 40), 1);

  // Render a simple pattern
  for (int x = 0; x < 100; x += 10) {
    for (int y = 0; y < 100; y += 10) {
      //      tft_drawPixel(x + 50, y + 50, RGB565(255, 0, 0)); // Red square
      tft_drawConvolutedPixel(x + 50, y + 50, x + 60, y + 60,
                              RGB565(150, 150, 150));
    }
  }

  tft_fillRect(200, 100, 50, 50, RGB565(255, 150, 150));

  // Render a simple pattern
  for (int x = 0; x < 100; x += 10) {
    for (int y = 0; y < 100; y += 10) {
      //      tft_drawPixel(x + 50, y + 50, RGB565(255, 0, 0)); // Red square
      tft_fillRect(x + 50, y + 50, 10, 10, RGB565(150, 150, 150));
    }
  }

  for (int x = 0; x < 100; x += 10) {
    for (int y = 0; y < 100; y += 10) {
      //      tft_drawPixel(x + 50, y + 50, RGB565(255, 0, 0)); // Red square
      tft_fillRect(x + 50, y + 50, 10, 10, RGB565(0, 0, 0));
    }
  }

  tft_fillRect(200, 100, 50, 50, RGB565(0, 0, 0));

  while (1) {
    // Main loop
  }
  return 0;
}
