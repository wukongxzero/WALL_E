#pragma once
// These define's must be placed at the beginning before #include
// "TimerInterrupt.h" _TIMERINTERRUPT_LOGLEVEL_ from 0 to 4 Don't define
// _TIMERINTERRUPT_LOGLEVEL_ > 0. Only for special ISR debugging only. Can hang
// the system.
#define TIMER_INTERRUPT_DEBUG 0
#define _TIMERINTERRUPT_LOGLEVEL_ 0
#define USE_TIMER_1 false
#if (defined(__AVR_ATmega644__) || defined(__AVR_ATmega644A__) ||              \
     defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644PA__) ||            \
     defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_NANO) ||                  \
     defined(ARDUINO_AVR_MINI) || defined(ARDUINO_AVR_ETHERNET) ||             \
     defined(ARDUINO_AVR_FIO) || defined(ARDUINO_AVR_BT) ||                    \
     defined(ARDUINO_AVR_LILYPAD) || defined(ARDUINO_AVR_PRO) ||               \
     defined(ARDUINO_AVR_NG) || defined(ARDUINO_AVR_UNO_WIFI_DEV_ED) ||        \
     defined(ARDUINO_AVR_DUEMILANOVE) || defined(ARDUINO_AVR_FEATHER328P) ||   \
     defined(ARDUINO_AVR_METRO) || defined(ARDUINO_AVR_PROTRINKET5) ||         \
     defined(ARDUINO_AVR_PROTRINKET3) ||                                       \
     defined(ARDUINO_AVR_PROTRINKET5FTDI) ||                                   \
     defined(ARDUINO_AVR_PROTRINKET3FTDI))
#define USE_TIMER_2 true
#warning Using Timer1

#define USE_TIMER_5 true
#warning Using Timer5
#endif

#define TIMER5_INTERVAL_MS 1000
#define TIMER_INTERVAL_MS 2000
/// Init timer ITimer1
