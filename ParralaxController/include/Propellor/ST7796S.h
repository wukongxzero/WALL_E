#ifndef ST7796S_H
#define ST7796S_H

#include <propeller.h>

// Screen Dimensions
#define TFT_WIDTH 320
#define TFT_HEIGHT 480

// Command Definitions
#define ST7796S_SOFT_RESET 0x01
#define ST7796S_SLEEP_OUT 0x11
#define ST7796S_DISPLAY_ON 0x29
#define ST7796S_CASET 0x2A  // Column Address Set
#define ST7796S_PASET 0x2B  // Page Address Set
#define ST7796S_RAMWR 0x2C  // Memory Write
#define ST7796S_MADCTL 0x36 // Memory Access Control
#define ST7796S_COLMOD 0x3A // Interface Pixel Format

// Color Utility: Convert RGB888 to RGB565
#define RGB565(r, g, b) (((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3))

void tft_init(int cs, int sck, int mosi, int dc, int rst);
void tft_drawPixel(int x, int y, unsigned short color);
void tft_drawConvolutedPixel(int x, int y, int x2, int y2,
                             unsigned short color);

void tft_fillScreen(unsigned short color);
void tft_drawChar(int x, int y, char c, unsigned short color, unsigned short bg,
                  int size);
void tft_writeString(int x, int y, const char *str, unsigned short color,
                     unsigned short bg, int size);

void tft_fillRect(int x, int y, int w, int h, unsigned short color);
void tft_fillScreenByBlock(unsigned short color);
#endif
