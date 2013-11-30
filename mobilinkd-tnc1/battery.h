// Copyright 2013 Mobilinkd LLC <info@mobilinkd.com>
// All rights reserved.

#ifndef BATTERY_H_
#define BATTERY_H_

/**
 * Check the battery level.  The value returned represents the voltage
 * in milliVolts.  For example, a return value of 3720 represents 3720mV
 * or 3.72 Volts.
 *
 * @note While result returned is in mV units, the resolution is at best
 *  6.4mV and the accuracy is much less.
 *
 * @return An integer representing the battery voltage in milliVolts.
 */
uint16_t check_battery(void);

#endif // BATTERY_H_
