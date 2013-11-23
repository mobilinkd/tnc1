// Copyright 2013 Mobilinkd LLC <info@mobilinkd.com>
// All rights reserved.

#ifndef BATTERY_H_
#define BATTERY_H_

/**
 * Check the battery level.  The value returned represents the voltage
 * in 100uV units.  For example, a return value of 37200 represents
 * 3.72 volts.
 *
 * @note While result returned is in 100uV units, the resolution is 6.4mV.
 *
 * @return An integer representing the battery voltage in 100uV units.
 */
uint16_t check_battery(void);

#endif // BATTERY_H_
