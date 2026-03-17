#include "simpletools.h"

// Define Pins
#define TFT_SCK 0
#define TFT_SDI 1
#define TFT_CS 2
#define TFT_DC 3
#define TFT_RST 4

// Colors (RGB565 format)
#define _ 0x0000 // Black
#define X 0xFFFF // White
#define SCALE 20 // 16 * 20 = 320px (Perfect fit for the short side!)

unsigned short pixel_art[16][16] = {
    {_, _, _, X, X, X, X, _, _, X, X, X, X, _, _, _},
    {_, _, X, _, _, _, _, X, X, _, _, _, _, X, _, _},
    {_, X, _, _, _, _, _, _, _, _, _, _, _, _, X, _},
    {X, _, _, _, _, _, _, _, _, _, _, _, _, _, _, X},
    {X, _, _, _, _, _, _, _, _, _, _, _, _, _, _, X},
    {_, X, _, _, _, _, _, _, _, _, _, _, _, _, X, _},
    {_, _, X, _, _, _, _, _, _, _, _, _, _, X, _, _},
    {_, _, _, X, _, _, _, _, _, _, _, _, X, _, _, _},
    {_, _, _, _, X, _, _, _, _, _, _, X, _, _, _, _},
    {_, _, _, _, _, X, _, _, _, _, X, _, _, _, _, _},
    {_, _, _, _, _, _, X, _, _, X, _, _, _, _, _, _},
    {_, _, _, _, _, _, _, X, X, _, _, _, _, _, _, _},
    {_, _, _, _, _, _, _, _, _, _, _, _, _, _, _, _},
    {_, _, _, _, _, _, _, _, _, _, _, _, _, _, _, _},
    {_, _, _, _, _, _, _, _, _, _, _, _, _, _, _, _},
    {_, _, _, _, _, _, _, _, _, _, _, _, _, _, _, _}};

// Send 8 bits to the screen
void tft_write(unsigned char byte) {
  shift_out(TFT_SDI, TFT_SCK, MSBFIRST, 8, byte);
}

void tft_cmd(unsigned char cmd) {
  low(TFT_DC);
  low(TFT_CS);
  tft_write(cmd);
  high(TFT_CS);
}

void tft_data(unsigned char data) {
  high(TFT_DC);
  low(TFT_CS);
  tft_write(data);
  high(TFT_CS);
}

// Set the "drawing window"
void set_window(int x0, int y0, int x1, int y1) {
  tft_cmd(0x2A); // Column Address Set
  tft_data(x0 >> 8);
  tft_data(x0 & 0xFF);
  tft_data(x1 >> 8);
  tft_data(x1 & 0xFF);

  tft_cmd(0x2B); // Row Address Set
  tft_data(y0 >> 8);
  tft_data(y0 & 0xFF);
  tft_data(y1 >> 8);
  tft_data(y1 & 0xFF);

  tft_cmd(0x2C); // Memory Write
}

void tft_init() {
  low(TFT_RST);
  pause(100);
  high(TFT_RST);
  pause(100);
  tft_cmd(0x11); // Sleep Out
  pause(120);
  tft_cmd(0x36);
  tft_data(0x48); // Orientation
  tft_cmd(0x3A);
  tft_data(0x55); // 16-bit Color
  tft_cmd(0x29);  // Display ON
}

int main() {
  tft_init();

  // Loop through the 16x16 array
  for (int r = 0; r < 16; r++) {
    for (int c = 0; c < 16; c++) {
      unsigned short color = pixel_art[r][c];

      // Draw a scaled block for each array element
      set_window(c * SCALE, r * SCALE, (c * SCALE) + SCALE - 1,
                 (r * SCALE) + SCALE - 1);

      high(TFT_DC);
      low(TFT_CS);
      for (int i = 0; i < (SCALE * SCALE); i++) {
        tft_write(color >> 8);
        tft_write(color & 0xFF);
      }
      high(TFT_CS);
    }
  }
}
