// Copyright 2014 Robert C. Riggs <rob@pangalactic.org>
// All rights reserved.


#ifndef MOBILINKD_EEPROM_H_
#define MOBILINKD_EEPROM_H_

#include <cfg/compiler.h>
#include <avr/eeprom.h>

#include <stdlib.h>

typedef struct Params
{
    uint8_t txdelay;             ///< How long in 10mS units to wait for TX to settle before starting data
    uint8_t persist;             ///< Likelyhood of taking the channel when its not busy
    uint8_t slot;                ///< How long in 10mS units to wait between sampling the channel to see if free
    uint8_t txtail;              ///< How long in 10mS units to wait after the data before keying off the transmitter
    uint8_t duplex;              ///< Ignore current channel activity - just key up
    uint8_t output_volume;       ///< output volume (0-255)
    uint8_t input_volume;        ///< input attenuation
    uint8_t squelch;             ///< input squelch level (0-255)
    uint8_t options;             ///< boolean options
    uint8_t chksum;              ///< Validity check of params data
} Params;

// Boolean options.
#define KISS_OPTION_CONN_TRACK      0x01
#define KISS_OPTION_VERBOSE         0x02
#define KISS_OPTION_VIN_POWER_ON    0x04
#define KISS_OPTION_VIN_POWER_OFF   0x08
#define KISS_OPTION_PTT_SIMPLEX     0x10
#define KISS_OPTION_BT_POWER_OFF    0x20

extern Params EEMEM eeparams;

#define MOBILINKD_EEPROM_SAVE(p) do { eeprom_write_block ((const void *) &p, (void *) &eeparams, sizeof (eeparams)); } while (0)

#define MOBILINKD_EEPROM_LOAD(p) do { eeprom_read_block ((void *) &p, (const void *) &eeparams, sizeof (eeparams)); } while (0)

void eeprom_overwrite_byte(unsigned int addr, char value);

uint32_t get_bootloader_value(void);

/**
 * Check whether to enter the bootloader based on a 32-bit EEPROM value.
 * Enter the bootloader if the number of set bits is odd.  Fully erased
 * there are 32-set bits.  The program resets the highest rank set bit
 * first, and the bootloader resets the next highest order bit.  Each
 * bootloader entry requires two bits be reset.
 *
 * It is the program's, not the bootloader's responsibility to erase the
 * values when 0 is reached.
 *
 * @return 1 if an odd number of bits is set, otherwise 0.
 */
void set_enter_bootloader(void);

#endif // MOBILINKD_EEPROM_H_
