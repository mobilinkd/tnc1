// Copyright 2014 Robert C. Riggs <rob@pangalactic.org>
// All rights reserved.


#ifndef MOBILINKD_EEPROM_H_
#define MOBILINKD_EEPROM_H_

#include <avr/eeprom.h>

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

extern Params EEMEM eeparams;

#define MOBILINKD_EEPROM_SAVE(p) do { eeprom_write_block ((const void *) &p, (void *) &eeparams, sizeof (eeparams)); } while (0)

#define MOBILINKD_EEPROM_LOAD(p) do { eeprom_read_block ((void *) &p, (const void *) &eeparams, sizeof (eeparams)); } while (0)


#endif // MOBILINKD_EEPROM_H_
