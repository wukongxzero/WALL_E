#include <hc05.h>
#include <propeller.h>

#include <simpletools.h> // Basic Propeller tools

// Example Pins for Parallax Propeller
#define HC05_RX 11
#define HC05_TX 10
#define HC05_EN 12

int main() {
  hc05_t bt;
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
  print("mode revert\n");
  hc05_print(&bt, "WALL-E Configured and Ready!\r\n");

  while (1) {
    // Fast, non-blocking check of the circular buffer
    int incoming = hc05_rx_check(&bt);

    if (incoming != -1) {
      char c = (char)incoming;

      hc05_tx(&bt, c); // Echo back

      if (c == 'F')
        hc05_print(&bt, " -> Forward\r\n");
      if (c == 'S')
        hc05_print(&bt, " -> Stopping\r\n");
    }

    // Do your heavy matrix math or screen updates here!
    // The RX cog is safely catching data in the background.
  }

  return 0;
}
