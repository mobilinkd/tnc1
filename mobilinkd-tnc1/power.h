// Copyright 2014 Mobilinkd LLC <info@mobilinkd.com>
// All rights reserved.


#ifndef POWER_H_
#define POWER_H_

#include "config.h"

#include <drv/ser.h>

extern Serial ser;
extern volatile uint8_t wdt_location;

extern const char PREFIX[];
extern const char ENDL[];

void power_on(void);
void power_on_message(int hc_status);

void enable_power_off(void);
void request_power_off(void);
int power_off_requested(void);
void power_off(void);

#define POWER_ON_VIN_ON 1
#define POWER_OFF_VIN_OFF 2

void set_power_config(uint8_t config);
uint8_t get_power_config(void);

int usb_vin_available(void);

#endif // POWER_H_
