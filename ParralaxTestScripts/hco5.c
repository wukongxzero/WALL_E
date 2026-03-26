#include <hc05.h>
#include <simpletools.h>

// --- Background RX Cog Loop ---
static char usart_rx_blocking(hc05_t *bt) {
  while (INA & (1 << bt->rx_pin))
    ; // Wait for start bit

  // Use the volatile baud_ticks (adapts if we switch to AT mode)
  unsigned int ticks = bt->baud_ticks;
  unsigned int t = CNT + (ticks >> 1) + ticks;
  char b = 0;

  for (int i = 0; i < 8; i++) {
    waitcnt(t);
    if (INA & (1 << bt->rx_pin))
      b |= (1 << i);
    t += ticks;
  }
  waitcnt(t); // Wait out stop bit
  return b;
}

static void rx_cog_loop(void *arg) {
  hc05_t *bt = (hc05_t *)arg;
  print("block exec\n");
  while (1) {
    char c = usart_rx_blocking(bt);
    int next_head = (bt->head + 1) % RX_BUFFER_SIZE;
    if (next_head != bt->tail) {
      bt->rx_buffer[bt->head] = c;
      bt->head = next_head;
    }
  }
}

// --- Public Functions ---

void hc05_init(hc05_t *bt, int rx, int tx, int en, int data_baud) {

  bt->rx_pin = rx;
  bt->tx_pin = tx;
  bt->en_pin = en;
  bt->baud_ticks = CLKFREQ / data_baud;
  bt->head = 0;
  bt->tail = 0;
  bt->cog_id = -1;

  // Setup TX and RX pins
  DIRA |= (1 << tx);
  OUTA |= (1 << tx);
  DIRA &= ~(1 << rx);

  DIRA |= (1 << en);
  OUTA &= ~(1 << en);
}

int hc05_start_rx_cog(hc05_t *bt) {
  bt->cog_id =
      cogstart(rx_cog_loop, (void *)bt, bt->cog_stack, sizeof(bt->cog_stack));
  return bt->cog_id;
}

void hc05_tx(hc05_t *bt, char c) {
  unsigned int tx_data = (c << 1) | 0x200;
  unsigned int ticks = bt->baud_ticks;
  unsigned int t = CNT;
  for (int i = 0; i < 10; i++) {
    if (tx_data & 1)
      OUTA |= (1 << bt->tx_pin);
    else
      OUTA &= ~(1 << bt->tx_pin);
    tx_data >>= 1;
    t += ticks;
    waitcnt(t);
  }
}

void hc05_print(hc05_t *bt, const char *str) {
  while (*str)
    hc05_tx(bt, *str++);
}

int hc05_rx_check(hc05_t *bt) {
  if (bt->head == bt->tail)
    return -1;
  char c = bt->rx_buffer[bt->tail];
  bt->tail = (bt->tail + 1) % RX_BUFFER_SIZE;
  return (int)c;
}

// --- HC-05 Specific Functions ---

void hc05_set_at_mode(hc05_t *bt, int enable) {
  if (bt->en_pin == -1)
    return;

  if (enable) {
    OUTA |= (1 << bt->en_pin);               // Pull EN High
    bt->baud_ticks = CLKFREQ / HC05_AT_BAUD; // Switch to 38400
  } else {
    OUTA &= ~(1 << bt->en_pin);                // Pull EN Low
    bt->baud_ticks = CLKFREQ / HC05_DATA_BAUD; // Back to 9600
  }

  // Give the HC-05 hardware a tiny moment to process the pin change
  waitcnt(CNT + (CLKFREQ / 10));
}

void hc05_send_command(hc05_t *bt, const char *cmd) {
  hc05_print(bt, "AT+");
  hc05_print(bt, cmd);
  hc05_print(bt, "\r\n"); // HC-05 strictly requires CRLF!
}
