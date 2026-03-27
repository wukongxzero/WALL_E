#include "include/TankStatus.h"
#include <hc05.h>
#include <propeller.h>

#include <simpletools.h> // Basic Propeller tools
void hc05_tx_hex(hc05_t *bt, unsigned char b) {
  const char hex_chars[] = "0123456789ABCDEF";
  hc05_tx(bt, hex_chars[b >> 4]);   // Send first digit
  hc05_tx(bt, hex_chars[b & 0x0F]); // Send second digit
  hc05_tx(bt, ' ');                 // Add a space for readability
}

// Example Pins for Parallax Propeller
#define HC05_RX 11
#define HC05_TX 10
#define HC05_EN 12
#define START_BIT_SIZE 3
#define TANKSTATUS_USART_PACKAGE_SIZE 16 + START_BIT_SIZE
struct TankStatus localUsartTankStatus;
unsigned char tsUsartBuffer[TANKSTATUS_USART_PACKAGE_SIZE] = {};
int totalStatus = 0;

int main() {
  hc05_t bt;
  constructTankStatus(&localUsartTankStatus);
  print("bt declare\n");

  // 1. Initialize variables (RX, TX, EN, Data Baud)
  hc05_init(&bt, HC05_RX, HC05_TX, HC05_EN, 9600);
  print("hc init");

  // 2. Start the background reader Cog
  if (hc05_start_rx_cog(&bt) == -1) {
    while (1)
      ; // Halt if out of cogs
  }

#ifdef PROGRAM_AT_MODE

  print("config mode\n");
  // --- Configuration Phase ---
  // 3. Switch to AT Mode (EN goes HIGH, Baud shifts to 38400)
  hc05_set_at_mode(&bt, 1);
  print("print command");

  // 4. Send some AT commands
  hc05_send_command(&bt, "NAME=WALLE_PROP"); // Sets name
  hc05_send_command(&bt, "PSWD=\"1234\"");   // Sets PIN

  // Flush the RX buffer of the "OK\r\n" responses so they don't
  // clutter our main loop later
  waitcnt(CNT + CLKFREQ); // wait 1 second
  while (hc05_rx_check(&bt) != -1)
    ;

  // --- Data Phase ---
  // 5. Back to normal data mode (EN goes LOW, Baud shifts to 9600)
  hc05_set_at_mode(&bt, 0);

#endif

  // buffer count
  print("mode revert\n");
  // hc05_print(&bt, "WALL-E Configured and Ready!\r\n");

  while (1) {
    // Fast, non-blocking check of the circular buffer
    int incoming = hc05_rx_check(&bt);

    if (incoming != -1) {

      unsigned char c = (unsigned char)incoming;
      tsUsartBuffer[totalStatus] = c;

      // print("bufferRead%X bitsatus %i| saved ts buffer %c <-should be same "
      //       "num\n",
      //     c, totalStatus, tsUsartBuffer[totalStatus]);

      print("bufferRead %02X | status %i | saved ts buffer %02X <-should be "
            "same num\n",
            c, totalStatus, tsUsartBuffer[totalStatus]);

      totalStatus++;
    }
    if (totalStatus == 16) {
      // send ts back
      // readByteTankStatus(tsUsartBuffer, 16, &localUsartTankStatus);

      // hc05_print(&bt, "start");
      for (int i = 0; i < 16; i++) {

        hc05_tx(&bt, tsUsartBuffer[i]);
        print("[%u]", tsUsartBuffer[i]);
        // hc05_print(&bt, &tsUsartBuffer[i]);
      }

      // hc05_print(&bt, "end");
    }
    totalStatus %= 16; // reset after 16 chars

    // Do your heavy matrix math or screen updates here!
    // The RX cog is safely catching data in the background.
  }

  return 0;
}
