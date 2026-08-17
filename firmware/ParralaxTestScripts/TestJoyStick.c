// #include <abvolts.h>     // Activity Board ADC library
#include "adc.c"
#include <propeller.h>
#include <simpletools.h> // Basic Propeller tools
#include <stdio.h>
//  FlexSpin looks for this to set the serial speed
//  Default is usually 115200
#define _BAUD 115200
//

int main() {
  test_adc_heartbeat();

  // Standard Activity Board Pins
  adc_init(21, 20, 19, 18);

  while (1) {
    int rawX = adc_in(0); // Joystick X
    int rawY = adc_in(2); // Joystick Y

    // Use (double) cast for printf compatibility with --printf=float
    printf("X: %1.2fV (%4d) | Y: %1.2fV (%4d)\r", (double)adc_volts(0), rawX,
           (double)adc_volts(1), rawY);

    waitcnt(CNT + CLKFREQ / 10);
  }
  return 0;
}

// GEMENI NOTES on ADC hardware
// What you are experiencing is called Pipeline Delay (sometimes called
// "Ghosting" or "Crosstalk").

// When you tell the ADC124S021 chip "Read Channel 1," it doesn't give you
// Channel 1 right away. It gives you the result of the previous conversion
// (which was Channel 0), and it cues up Channel 1 for the next time you ask.
