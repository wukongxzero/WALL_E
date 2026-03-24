#include "ST7796S.h"

static int _CS, _SCK, _MOSI, _DC, _RST;

// Fast SPI Bit-bang (8-bit)
static void spi_write8(unsigned char data) {
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
