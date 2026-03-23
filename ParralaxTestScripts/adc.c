#include <propeller.h>
#include <stdio.h>

// 1. Declare the macros for the Activity Board ADC Pins
#define ADC_CS 21
#define ADC_SCLK 20
#define ADC_DOUT 19 // DO (Data Out from ADC to Propeller)
#define ADC_DIN 18  // DI (Data In from Propeller to ADC)

// Global variables to store pin states for the standard functions
static int _cs, _scl, _do, _di;

/**
 * From Scratch: adc_init
 * Sets up the pins
 */
void adc_init(int csPin, int sclPin, int doPin, int diPin) {
  _cs = csPin;
  _scl = sclPin;
  _do = doPin;
  _di = diPin;

  // Initial Directions
  DIRA |= (1 << _cs) | (1 << _scl) | (1 << _di);
  DIRA &= ~(1 << _do);

  // Initial States
  OUTA |= (1 << _cs);   // CS High (Idle)
  OUTA &= ~(1 << _scl); // Clock Low
}

/**
 * From Scratch: adc_in
 * Matches the 12-bit SPI sequence for the ADC124S021
 */

int adc_in(int channel) {
  int result = 0;

  // We run the SPI frame TWICE to flush the pipeline.
  for (int pass = 0; pass < 2; pass++) {
    result = 0;
    OUTA &= ~(1 << _cs); // Start Communication

    for (int i = 15; i >= 0; i--) {
      OUTA |= (1 << _scl); // Pulse Clock High (Rising Edge)

      // The chip expects the channel address on clock pulses 2, 3, and 4
      // i=14 is clock 2, i=13 is clock 3, i=12 is clock 4
      if (i >= 12 && i <= 14) {
        int bit_index = i - 12; // Maps i=14->Bit 2, i=13->Bit 1, i=12->Bit 0
        if (channel & (1 << bit_index))
          OUTA |= (1 << _di);
        else
          OUTA &= ~(1 << _di);
      } else {
        OUTA &= ~(1 << _di);
      }

      OUTA &= ~(1 << _scl); // Pulse Clock Low (Falling Edge)

      // Read the DO pin on the falling edge (Cycles 12 down to 1)
      if (i >= 1 && i <= 12) {
        result <<= 1;
        if (INA & (1 << _do))
          result |= 1;
      }
    }

    OUTA |= (1 << _cs); // End Communication

    // Tiny micro-delay between the two passes
    __asm__("nop");
  }

  return result & 0x0FFF; // Return the 12-bit value from the 2nd pass
}
/**
 * From Scratch: adc_volts
 */
float adc_volts(int channel) { return (float)adc_in(channel) * 5.0f / 4096.0f; }

/**
 * Diagnostic: Heartbeat Test
 */
void test_adc_heartbeat() {
  DIRA |= (1 << ADC_CS) | (1 << ADC_SCLK) | (1 << ADC_DIN);
  DIRA &= ~(1 << ADC_DOUT);

  OUTA |= (1 << ADC_CS);
  OUTA &= ~(1 << ADC_SCLK);

  printf("Starting Heartbeat Test...\n");
  waitcnt(CNT + CLKFREQ);

  OUTA &= ~(1 << ADC_CS);

  OUTA |= (1 << ADC_DIN);
  for (int i = 0; i < 2; i++) {
    OUTA |= (1 << ADC_SCLK);
    waitcnt(CNT + (CLKFREQ / 100000));
    OUTA &= ~(1 << ADC_SCLK);
  }

  if (!(INA & (1 << ADC_DOUT))) {
    printf("SUCCESS: ADC Chip is responding! (Null bit detected)\n");
  } else {
    printf(
        "FAILURE: ADC Chip is silent. Check Power Switch (Position 1 or 2).\n");
  }

  OUTA |= (1 << ADC_CS);
}
