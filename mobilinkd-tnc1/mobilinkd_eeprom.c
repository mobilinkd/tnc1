// Copyright 2014 Robert C. Riggs <rob@pangalactic.org>
// All rights reserved.

#include "mobilinkd_eeprom.h"

#include <cfg/macros.h>
#include <avr/eeprom.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include <stdlib.h>
#include <stdint.h>

Params EEMEM eeparams;

/*
 * The EEPROM is 1KB in size.  We are going to used the 4 bytes at address
 * 0x3F8 (end - 8) to store whether we wish to enter the bootloader on reboot.
 * There are a number reasons for the TNC to reboot:
 *
 * 1. Initial power on after manufacturing and hardware maintenance.
 * 2. Power on after the battery is completely drained and recharged.
 * 3. Power on with battery removed.  (This may be a common operating mode.)
 * 4. Watch-dog timeout -- error in programming or due to environment.
 * 5. Brown-out detection -- power glitch or battery level too low.
 * 6. Reset line pulled low.
 * 7. Firmware upload.
 *
 * However, during normal operation, the TNC never reboots.  It goes to
 * sleep during power off and wakes on power on.
 *
 * We do not want to enter the bootloader unless we fully intend to make
 * use of it.  To that end, there are two ways to enter the bootloader:
 *
 * 1. Hold the power button down during (true) power on or reset.
 * 2. Write to a special location in EEPROM that is read by the bootloader.
 *
 * With an uninitialized EEPROM, these values will all be 0xFF.  Each time
 * the application wishes to enter the bootloader, it erases the first set
 * bit that it encounters, starting with the lowest address and highest
 * bit and ending at the highest address and lowest bit.  If all of the
 * values are reset {0, 0, 0, 0}, the value is set to {7F, FF, FF, FF}.
 *
 * The reason for doing this is that this only updates the 4 bytes once
 * every 16 boot attempts (2 bits are used each time).  This reduces
 * wear on the EEPROM cells.
 */

#define ENTER_EEPROM_ADDR ((uint8_t *)0x3F8)

// Parts taken from Atmega's AVR103 App Note.
void eeprom_overwrite_byte(unsigned int addr, char value)
{
    do {} while( EECR & (1<<EEPE) ); // Wait for completion of previous write.

    cli();

    EEAR = addr;        // Set EEPROM address register.
    EECR = (1<<EERE);   // Start EEPROM read operation.

    EEDR = value;       // Set EEPROM data register.
    EECR = (1<<EEMPE) | // Set Master Write Enable bit...
           (1<<EEPM1);  // ...and Write-only mode.
    EECR |= (1<<EEPE);  // Start Write-only operation.

    sei();
}

uint32_t get_bootloader_value(void)
{
    return eeprom_read_dword((uint32_t*) ENTER_EEPROM_ADDR);
}

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
void set_enter_bootloader(void)
{
    uint8_t enter_eeprom_index = 4;
    uint8_t enter_eeprom_value = 0;
    uint8_t enter_eeprom_count = 0;

    for (uint8_t i = 0; i != 4; i++)
    {
        enter_eeprom_value = eeprom_read_byte(ENTER_EEPROM_ADDR + i);
        if (enter_eeprom_value != 0)
        {
            enter_eeprom_index = i;
            break;
        }
    }

    // No bits available, start with 31 set.
    if (enter_eeprom_index == 4)
    {
        eeprom_write_byte((uint8_t*) ENTER_EEPROM_ADDR, 0x7f);
        for (uint8_t i = 1; i != 4; i++)
        {
            eeprom_write_byte((uint8_t*) ENTER_EEPROM_ADDR + i, 0xff);
        }
        return;
    }

    uint8_t c = enter_eeprom_value;

    // Find the highest set bit.
    while (c & 1)
    {
        enter_eeprom_count += 1;
        c >>= 1;
    }

    if (enter_eeprom_count & 1) return; // Already have an odd number set.

    enter_eeprom_value &= ~BV(enter_eeprom_count - 1);
    eeprom_overwrite_byte(
        (int) ENTER_EEPROM_ADDR + enter_eeprom_index, enter_eeprom_value);
}
