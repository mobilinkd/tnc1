// Copyright 2014 Mobilinkd LLC <info@mobilinkd.com>
// All rights reserved.

#ifndef CONFIG_H_
#define CONFIG_H_

#include <avr/io.h>

// Not actually referenced
// #define HC_05_RX DDRD, PORTD, PIND0
// #define HC_05_TX DDRD, PORTD, PIND1
// Note: no hw flow control used

#define HC_05_COMMAND_DDR DDRD
#define HC_05_COMMAND_PORT PORTD
#define HC_05_COMMAND_PIN PIN5

#define HC_05_RESET_DDR DDRC
#define HC_05_RESET_PORT PORTC
#define HC_05_RESET_PIN PIN5

#define HC_05_STATUS_DDR DDRD
#define HC_05_STATUS_PORT PORTD
#define HC_05_STATUS_INPORT PIND
#define HC_05_STATUS_PIN PIN7

#define HC_05_POWER_DDR DDRD
#define HC_05_POWER_PORT PORTD
#define HC_05_POWER_PIN PIN4

// INT1 pin
#define BUTTON_DDR DDRD
#define BUTTON_PORT PORTD
#define BUTTON_INPORT PIND
#define BUTTON_PIN PIN3

// INT0 pin
#define VIN_DETECT_DDR DDRD
#define VIN_DETECT_PORT PORTD
#define VIN_DETECT_INPORT PIND
#define VIN_DETECT_PIN PIN2

// Both PTT pins must be on the same port.
#define PTT_DDR DDRB
#define PTT_PORT PORTB

// Multiplexed PTT (2K2 resistor on Mic+)
#define PTT_PIN_M PIN1

// Simplex PTT (Kenwood, MiniDIN-6 data port)
#define PTT_PIN_S PIN2

#define AUDIO_OUT_DDR DDRD
#define AUDIO_OUT_PORT PORTD
#define AUDIO_OUT_PIN PIN6

#define AUDIO_IN_DDR DDRC
#define AUDIO_IN_PORT PORTC
#define AUDIO_IN_PIN PIN0

#define AUDIO_OFFSET_DDR DDRC
#define AUDIO_OFFSET_PORT PORTC
#define AUDIO_OFFSET_PIN PIN1

#define BATTERY_LEVEL_DDR DDRC
#define BATTERY_LEVEL_PORT PORTC
#define BATTERY_LEVEL_PIN PIN2

#define BATTERY_DIVIDER_DDR DDRC
#define BATTERY_DIVIDER_PORT PORTC
#define BATTERY_DIVIDER_PIN PIN3

#endif // CONFIG_H_
