
#ifndef SIMULATION_SCREEN
#include <Propellor/ST7796S.h>
#include <Propellor/font.h>

static int _CS, _SCK, _MOSI, _DC, _RST;

// Optimized 16-bit bit-bang for FlexSpin
static inline void spi_write16(unsigned short data) {

  __attribute__((optimize("-O3")));
  for (int i = 0; i < 16; i++) {
    if (data & 0x8000)
      OUTA |= (1 << _MOSI);
    else
      OUTA &= ~(1 << _MOSI);

    OUTA |= (1 << _SCK); // Rising edge
    data <<= 1;
    OUTA &= ~(1 << _SCK); // Falling edge
  }
}

// Fast SPI Bit-bang (8-bit)
static void spi_write8(unsigned char data) {
  __attribute__((optimize("-O3")));
  for (int i = 0; i < 8; i++) {
    if (data & 0x80)
      OUTA |= (1 << _MOSI);
    else
      OUTA &= ~(1 << _MOSI);

    OUTA |= (1 << _SCK); // Clock High
    data <<= 1;
    OUTA &= ~(1 << _SCK); // Clock Low
  }
}

// Write Command to LCD
void writeCmd(unsigned char cmd) {
  OUTA &= ~(1 << _DC); // DC Low for Command
  OUTA &= ~(1 << _CS); // CS Low
  spi_write8(cmd);
  OUTA |= (1 << _CS); // CS High
}

// Write Data to LCD
void writeData(unsigned char data) {
  OUTA |= (1 << _DC);  // DC High for Data
  OUTA &= ~(1 << _CS); // CS Low
  spi_write8(data);
  OUTA |= (1 << _CS); // CS High
}

void setAddrWindow(int x0, int y0, int x1, int y1) {
  writeCmd(ST7796S_CASET);
  writeData(x0 >> 8);
  writeData(x0 & 0xFF);
  writeData(x1 >> 8);
  writeData(x1 & 0xFF);

  writeCmd(ST7796S_PASET);
  writeData(y0 >> 8);
  writeData(y0 & 0xFF);
  writeData(y1 >> 8);
  writeData(y1 & 0xFF);

  writeCmd(ST7796S_RAMWR);
}

void tft_init(int cs, int sck, int mosi, int dc, int rst) {
  _CS = cs;
  _SCK = sck;
  _MOSI = mosi;
  _DC = dc;
  _RST = rst;

  DIRA |= (1 << _CS) | (1 << _SCK) | (1 << _MOSI) | (1 << _DC) | (1 << _RST);
  OUTA |= (1 << _CS) | (1 << _RST);

  // Hardware Reset
  OUTA &= ~(1 << _RST);
  waitcnt(CNT + CLKFREQ / 10);
  OUTA |= (1 << _RST);
  waitcnt(CNT + CLKFREQ / 10);

  writeCmd(ST7796S_SOFT_RESET);
  waitcnt(CNT + CLKFREQ / 5);

  writeCmd(ST7796S_SLEEP_OUT);
  waitcnt(CNT + CLKFREQ / 5);

  writeCmd(ST7796S_COLMOD);
  writeData(0x55); // 16-bit color

  writeCmd(ST7796S_MADCTL);
  writeData(0x48); // Rotation/Order

  writeCmd(ST7796S_DISPLAY_ON);
  waitcnt(CNT + CLKFREQ / 5);
}

void tft_drawPixel(int x, int y, unsigned short color) {
  if (x < 0 || x >= TFT_WIDTH || y < 0 || y >= TFT_HEIGHT)
    return;
  setAddrWindow(x, y, x, y);
  OUTA |= (1 << _DC);
  OUTA &= ~(1 << _CS);
  spi_write8(color >> 8);
  spi_write8(color & 0xFF);
  OUTA |= (1 << _CS);
}

void tft_drawConvolutedPixel(int x, int y, int x2, int y2,
                             unsigned short color) {
  if (x < 0 || x >= TFT_WIDTH || y < 0 || y >= TFT_HEIGHT)
    return;
  setAddrWindow(x, y, x2, y2);
  OUTA |= (1 << _DC);
  OUTA &= ~(1 << _CS);
  spi_write8(color >> 8);
  spi_write8(color & 0xFF);
  OUTA |= (1 << _CS);
}

void tft_fillScreen(unsigned short color) {
  setAddrWindow(0, 0, TFT_WIDTH - 1, TFT_HEIGHT - 1);
  unsigned char hi = color >> 8, lo = color & 0xFF;
  OUTA |= (1 << _DC);
  OUTA &= ~(1 << _CS);
  for (int i = 0; i < (TFT_WIDTH * TFT_HEIGHT); i++) {
    spi_write8(hi);
    spi_write8(lo);
  }
  OUTA |= (1 << _CS);
}

// Note:not actually by block
void tft_fillScreenByBlock(unsigned short color) {
  setAddrWindow(0, 0, TFT_WIDTH - 1, TFT_HEIGHT - 1);
  unsigned char hi = color >> 8, lo = color & 0xFF;
  OUTA |= (1 << _DC);
  OUTA &= ~(1 << _CS);
  for (int i = 0; i < (TFT_HEIGHT * TFT_HEIGHT); i += 1) {
    setAddrWindow(i, i, i + 8 - 1, i + 8 - 1);
    spi_write16(hi);
    spi_write16(lo);
  }

  OUTA |= (1 << _CS);
}

/**
 * Draws a single character
 * @param x, y: Top left coordinates
 * @param c: The character
 * @param color: Text color (RGB565)
 * @param bg: Background color (RGB565)
 * @param size: Scaling factor (1 = 5x7, 2 = 10x14, etc)
 */
void tft_drawChar(int x, int y, char c, unsigned short color, unsigned short bg,
                  int size) {
  if (c < 32 || c > 126)
    c = '?'; // Basic bounds check

  int font_idx = (c - 32) * 5;
  unsigned char hi = color >> 8, lo = color & 0xFF;
  unsigned char bg_hi = bg >> 8, bg_lo = bg & 0xFF;

  // Set a window for the character to speed up SPI transfer
  // A 5x7 font with a 1px gap is 6x8 pixels.
  setAddrWindow(x, y, x + (6 * size) - 1, y + (8 * size) - 1);

  OUTA |= (1 << _DC);
  OUTA &= ~(1 << _CS);

  // We loop through 8 rows and 6 columns (5 for font + 1 for spacing)
  for (int row = 0; row < 8; row++) {
    for (int r_scale = 0; r_scale < size; r_scale++) {
      for (int col = 0; col < 6; col++) {
        // Determine if this pixel is "on"
        int pixel_on = 0;
        if (col < 5) {
          unsigned char line = font5x7[font_idx + col];
          if (line & (1 << row))
            pixel_on = 1;
        }

        for (int c_scale = 0; c_scale < size; c_scale++) {
          if (pixel_on) {
            spi_write8(hi);
            spi_write8(lo);
          } else {
            spi_write8(bg_hi);
            spi_write8(bg_lo);
          }
        }
      }
    }
  }
  OUTA |= (1 << _CS);
}

void tft_writeString(int x, int y, const char *str, unsigned short color,
                     unsigned short bg, int size) {
  int curX = x;
  while (*str) {
    tft_drawChar(curX, y, *str++, color, bg, size);
    curX += (6 * size); // Move cursor to next char position

    // Simple wrap-around logic
    if (curX > (TFT_WIDTH - (6 * size))) {
      curX = x;
      y += (8 * size);
    }
  }
}
void tft_fillRect(int x, int y, int w, int h, unsigned short color) {
  // 1. Define the boundaries of the "chunk"
  setAddrWindow(x, y, x + w - 1, y + h - 1);

  // 2. Prepare for Data
  OUTA |= (1 << _DC);
  OUTA &= ~(1 << _CS);

  // 3. Blast the color data
  int totalPixels = w * h;
  for (int i = 0; i < totalPixels; i++) {
    spi_write16(color);
  }

  OUTA |= (1 << _CS);
}
#endif
