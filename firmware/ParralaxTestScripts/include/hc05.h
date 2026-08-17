#ifndef HC05_USART_BG_H
#define HC05_USART_BG_H

#include <propeller.h>

#define RX_BUFFER_SIZE 128
#define HC05_AT_BAUD 38400
#define HC05_DATA_BAUD 9600

typedef struct {
  int rx_pin;
  int tx_pin;
  int en_pin; // New: Controls AT Mode

  // Volatile so the background RX cog sees baud rate changes instantly
  volatile unsigned int baud_ticks;

  // --- Multicore FIFO ---
  volatile char rx_buffer[RX_BUFFER_SIZE];
  volatile int head;
  volatile int tail;

  unsigned int cog_stack[128];
  int cog_id;
} hc05_t;

// Core setup
void hc05_init(hc05_t *bt, int rx, int tx, int en, int data_baud);
int hc05_start_rx_cog(hc05_t *bt);

// TX/RX
void hc05_tx(hc05_t *bt, char c);
void hc05_print(hc05_t *bt, const char *str);
int hc05_rx_check(hc05_t *bt);

// New: HC-05 Specific Controls
void hc05_set_at_mode(hc05_t *bt, int enable);
void hc05_send_command(hc05_t *bt, const char *cmd);

#endif
