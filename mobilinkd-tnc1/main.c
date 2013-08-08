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

static Afsk afsk;
static Serial ser;
static KissCtx kiss;

#define ADC_CH 0

uint8_t mcusr_mirror __attribute__ ((section (".noinit")));
uint8_t wdt_location __attribute__ ((section (".noinit")));

void get_mcusr(void) __attribute__((naked))
__attribute__((section(".init3")));
void get_mcusr(void)
{
	mcusr_mirror = MCUSR;
	MCUSR = 0;
	wdt_disable();
}

/**
 * Module initialse
 *
 */
static void init(void)
{

	IRQ_ENABLE;
	kdbg_init();
	timer_init();

	/* Initialize serial port */
	ser_init(&ser, SER_UART0);
	ser_setbaudrate(&ser, 38400L);

	int hc_status = init_hc05(&ser.fd);

	// Init afsk demodulator
	afsk_init(&afsk, ADC_CH, 0);

	// Init KISS context
	kiss_init(&kiss, &afsk.fd, &ser.fd);

	// Announce
	kfile_print(&ser.fd, "\r\n== BeRTOS AVR/Mobilinkd TNC1\r\n");
	kfile_printf(&ser.fd, "== Version 1.2, Build %d\r\n", VERS_BUILD);
	if (mcusr_mirror & 8)
	{
		kfile_printf(&ser.fd, "== WDT (loc = %02x)\r\n", wdt_location);
	}
	else
	{
		wdt_location = 0;
	}

	// if (hc_status != 0)
	kfile_printf(&ser.fd, "== HC-05 = %02x\r\n", hc_status);

	kfile_print(&ser.fd, "== Starting.\r\n");

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
		wdt_location = 1;
		// Use KISS module call to check AFSK and serial objects for incoming data
		kiss_poll_modem(&kiss);
		wdt_location = 129;
		kiss_poll_serial(&kiss);
		afsk_tx_bottom_half(&afsk);
		wdt_reset();
	}
	return 0;
}
