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
 * Copyright 2013 Mobilinkd LLC <mobilinkd@gmail.com>
 *
 * -->
 *
 * \author Rob Riggs <rob.wx9o@gmail.com>
 *
 * \brief HC-05 Bluetooth module initialization.
 *
 */

#include <drv/ser.h>
#include <drv/timer.h>

#include <avr/io.h>
#include <avr/eeprom.h>

#include <string.h>
#include <alloca.h>

#include "hc-05.h"

static EEMEM uint16_t bt_initialized = 0;

#define BT_INIT_MAGIC 0xc0a1

const uint8_t BT_RESET_PIN = 3;
const uint8_t BT_COMMAND_PIN = 2;

#define BT_NAME "Mobilinkd TNC1"

static bool starts_with(KFile* ser, const char* compare, mtime_t timeout_ms)
{
	const ticks_t started = timer_clock();

	size_t len = strlen(compare);

	char* buffer = (char*) alloca(len);
	size_t pos = 0;

	while (timer_clock() - started < ms_to_ticks(timeout_ms))
	{
		int c = kfile_getc(ser);

		if (c == EOF)
			continue;

		if (c == '\n')
		{
			if (pos == 0)
				continue; // ignore blank line.

			if (pos != len)
				return false;

			for (size_t i = 0; i != len; i++)
			{
				if (compare[i] != buffer[i])
					return false;
			}

			return true;
		}
		else if (pos != len)
		{
			buffer[pos++] = c;
		}
		// Consume and ignore the rest...
	}
	return false;
}

static bool ends_with(KFile* ser, const char* compare, mtime_t timeout_ms)
{
	const ticks_t started = timer_clock();

	size_t len = strlen(compare);

	char* buffer = (char*) alloca(len);
	memset(buffer, 0, len);
	uint8_t pos = 0;

	while (timer_clock() - started < ms_to_ticks(timeout_ms))
	{
		int c = kfile_getc(ser);

		if (c == EOF)
			continue;
		if (c == '\r')
			continue;

		if (c == '\n')
		{
			for (size_t i = 0, j = pos; i != len; i++)
			{
				if (compare[i] != buffer[j++])
					return false;
				if (j == len)
					j = 0;
			}

			return true;
		}
		else
		{
			buffer[pos++] = c;
			if (pos == len)
				pos = 0;
		}
	}
	return false;
}

static void hc05_reset(void)
{
	DDRD |= BV(BT_RESET_PIN);       // Set BT reset pin to output mode.
	PORTD &= ~BV(BT_RESET_PIN);     // Bring it low to reset the BT module.
	timer_delay(200L);
	PORTD |= BV(BT_RESET_PIN);      // Bring it back high for normal use.
	DDRD &= ~BV(BT_RESET_PIN);      // Set BT reset pin to input mode.
	timer_delay(800L);
}

void hc05_command_mode()
{
	// Bring PD2 HIGH to enter command mode.
	PORTD |= BV(BT_COMMAND_PIN);
}

void hc05_normal_mode()
{
	// Bring PD2 LOW to exit command mode.
	PORTD &= ~BV(BT_COMMAND_PIN);
}

static bool hc05_soft_reset(KFile* ser)
{
	// Do a soft reset here.
	kfile_print(ser, "AT+RESET\r\n");
	kfile_flush(ser);
	return starts_with(ser, "OK", 1000L);
}

int init_hc05(KFile* ser)
{
	DDRD |= BV(BT_COMMAND_PIN);

	int result = 0;

	if (eeprom_read_word(&bt_initialized) == BT_INIT_MAGIC)
		return result;

	timer_delay(800L);
	hc05_command_mode();
	timer_delay(1000L);
	hc05_reset();

	kfile_print(ser, "AT\r\n");
	kfile_flush(ser);
	starts_with(ser, "OK", 1000L); // ignore the first one.

	kfile_print(ser, "AT\r\n");
	kfile_flush(ser);
	bool cmd_ok = starts_with(ser, "OK", 1000L);

#if 1
	if (!cmd_ok)
	{
		// Something wrong here.
		hc05_normal_mode();
		hc05_reset();
		result = 1;
		return result;
	}
#endif

	kfile_print(ser, "AT+NAME?\r\n");
	kfile_flush(ser);
	bool name_ok = ends_with(ser, BT_NAME, 1000);
	if (!starts_with(ser, "OK", 1000L))
		result |= 2;

	if (!name_ok)   // Need to initialize the HC-05 module.
	{
		result |= 4;
		kfile_print(ser, "AT+NAME=");
		kfile_print(ser, BT_NAME);
		kfile_print(ser, "\r\n");
		kfile_flush(ser);
		if (!starts_with(ser, "OK", 1000L))
			result |= 8;

		kfile_print(ser, "AT+UART=38400,0,0\r\n");
		kfile_flush(ser);
		if (!starts_with(ser, "OK", 1000L))
			result |= 16;

		kfile_print(ser, "AT+RESET\r\n");
		kfile_flush(ser);
		if (!starts_with(ser, "OK", 1000L))
			result |= 32;

	}

	timer_delay(1000L);
	hc05_normal_mode();

	if (!hc05_soft_reset(ser))
		result |= 64;

	eeprom_write_word((uint16_t*) &bt_initialized, BT_INIT_MAGIC);

	return result;
}
