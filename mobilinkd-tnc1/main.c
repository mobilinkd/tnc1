/**
 * \file
 * <!--
 * This file is part of BeRTOS.
 *
 * Bertos is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * As a special exception, you may use this file as part of a free software
 * library without restriction.  Specifically, if other files instantiate
 * templates or use macros or inline functions from this file, or you compile
 * this file and link it with other files to produce an executable, this
 * file does not by itself cause the resulting executable to be covered by
 * the GNU General Public License.  This exception does not however
 * invalidate any other reasons why the executable file might be covered by
 * the GNU General Public License.
 *
 * Copyright 2012 Robin Gilks <g8ecj@gilks.org>
 * Portions copyright 2013 Mobilinkd LLC <mobilinkd@gmail.com>
 * -->
 *
 * \author Robin Gilks <g8ecj@gilks.org>
 *
 * \brief Arduino KISS TNC radio demo.
 *
 * This example shows how to read and write KISS data to the KISS module(s)
 * It uses the following modules:
 * afsk
 * kiss
 * ser
 *
 * You will see how to implement the KISS protocol, init the afsk de/modulator and
 * how to process messages using kiss module and how the p-persist algorithm works.
 */

#include <cpu/irq.h>
#include <cfg/debug.h>
#include <cfg/log.h>

#include <net/afsk.h>
#include <net/kiss.h>

#include <drv/ser.h>
#include <drv/timer.h>

#include <stdio.h>
#include <alloca.h>
#include <string.h>

#include <avr/eeprom.h>
#include <avr/io.h>
#include <avr/wdt.h>

#include "buildrev.h"
#include "hc-05.h"
#include "battery.h"
#include "power.h"
#include "mobilinkd_error.h"
#include "mobilinkd_util.h"

static Afsk afsk;
Serial ser;
static KissCtx kiss;

#define ADC_CH 0

uint8_t mcusr_mirror __attribute__ ((section (".noinit")));

void get_mcusr(void) __attribute__((naked))
__attribute__((section(".init3")));
void get_mcusr(void)
{
    mcusr_mirror = MCUSR;
    MCUSR = 0;
    wdt_disable();
}

/**
 * This is a new version of the Mobilinkd TNC.
 *
 * Key differences are:
 * Use the Atmega 328P "Power Down" mode to power down the TNC,
 * eliminating the need for the power control circuitry.
 *
 * This provides push-button on/off, as well as 5V detection and power
 * control based on whether USB power is available.
 *
 * The push button can be used by the bootloader to determine whether
 * to run.  This means that non-intentional reboots will not cause the
 * firmware to be erased.
 *
 * In order to do this, the HC-05 Command and Reset pins had to be
 * moved because they occupy the only pins that can wake the TNC from
 * Standby mode.  PD2 (INT0) and PD3 (INT1) are used for 5V and and
 * power button respectively.
 */

/**
 * Initialize the module.  Initialize the following subsystems:
 *
 * - kernel debugging
 * - timer
 * - serial UART0
 * - Bluetooth module
 * - AFSK driver
 * - KISS driver
 *
 * The AFSK module should be initialized after the startup banner is
 * printed to reduce the chance of a buffer overrun.
 */
static void init(void)
{
    IRQ_ENABLE;
    kdbg_init();
    timer_init();

    power_on();

    int hc_status = init_hc05(&ser.fd);

    wdt_location = 0;

    mobilinkd_set_error(MOBILINKD_ERROR_WATCHDOG_TIMEOUT);

    afsk_init(&afsk, ADC_CH, 0);

    wdt_location = 1;

    kiss_init(&kiss, &afsk.fd, &ser.fd);
    wdt_location = 2;

    if (kiss.params.options & KISS_OPTION_VIN_POWER_ON)
    {
        set_power_config(get_power_config() | POWER_ON_VIN_ON);
    }
    if (kiss.params.options & KISS_OPTION_VIN_POWER_OFF)
    {
        set_power_config(get_power_config() | POWER_OFF_VIN_OFF);
    }

    if (kiss.params.options & KISS_OPTION_PTT_SIMPLEX)
    {
        afsk_ptt_set(&afsk, AFSK_PTT_MODE_SIMPLEX);
    }
    else
    {
        afsk_ptt_set(&afsk, AFSK_PTT_MODE_MULTIPLEX);
    }

    power_on_message(hc_status);
    enable_power_off();

    wdt_enable(WDTO_4S);
}

/**
 * Main loop
 * Initialises all objects then polls the serial and ax25 data sources for data
 *
 */
int main(void)
{
    init();

    while (1)
    {
        if (power_off_requested()) power_off();

        if (mobilinkd_get_error() != MOBILINKD_ERROR_WATCHDOG_TIMEOUT)
        {
            if (kiss_get_verbosity(&kiss))
            {
                afsk_rx_bottom_half(&afsk);
                kfile_print_P(&ser.fd, PREFIX);
                kfile_print_P(&ser.fd, mobilinkd_strerror(mobilinkd_get_error()));
                kfile_print_P(&ser.fd, ENDL);
            }
            mobilinkd_set_error(MOBILINKD_ERROR_WATCHDOG_TIMEOUT);
        }
        wdt_location = 3;
        afsk_rx_bottom_half(&afsk);
        wdt_location = 4;
        kiss_poll_modem(&kiss);
        wdt_location = 5;
        kiss_poll_serial(&kiss);
        wdt_location = 6;
        afsk_tx_bottom_half(&afsk);
        wdt_reset();
    }
    return 0;
}
