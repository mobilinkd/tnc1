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
 *
 * -->
 *
 * \author Robin Gilks <g8ecj@gilks.org>
 *
 * \brief KISS module
 *
 * You will see how to implement the KISS protocol, init the afsk de/modulator and
 * how to process messages using ax25 module and how the p-persist algorithm works.
 *
 * $WIZ$ module_name = "kiss"
 * $WIZ$ module_configuration = "bertos/cfg/cfg_kiss.h"
 * $WIZ$ module_depends = "timer", "kfile" "crc_ccitt"
 * $WIZ$ module_hw = "bertos/hw/hw_kiss.h"
 */


#include "cfg/cfg_kiss.h"

#include <cfg/compiler.h>

#include <io/kfile.h>




typedef struct Params
{
	uint8_t txdelay;             ///< How long in 10mS units to wait for TX to settle before starting data
	uint8_t persist;             ///< Likelyhood of taking the channel when its not busy
	uint8_t slot;                ///< How long in 10mS units to wait between sampling the channel to see if free
	uint8_t txtail;              ///< How long in 10mS units to wait after the data before keying off the transmitter
	uint8_t duplex;              ///< Ignore current channel activity - just key up
	uint8_t output_volume;       ///< output volume (0-255)
	uint8_t input_volume;        ///< input attenuation (future)
	uint8_t squelch;             ///< input squelch level (0-255)
	uint8_t chksum;              ///< Validity check of params data
} Params;


#define HW_CMD_BUFFER_SIZE 16


typedef struct KissCtx
{
	uint8_t rx_buf[CONFIG_KISS_FRAME_BUF_LEN];   ///< Buffer of decoded data prior to transmission to serial
	uint16_t rx_pos;                             ///< Offset in buffer of next octet
//	uint8_t tx_buf[CONFIG_KISS_FRAME_BUF_LEN];   ///< buffer of decoded KISS data prior to transmission to the modem
	uint16_t tx_pos;                             ///< Offset in buffer of next octet
	KFile *modem;                                ///< I/f to the afsk modem
	KFile *serial;                               ///< I/f to the serial port
	uint8_t command;                             ///< KISS command byte
	uint8_t state;                               ///< what data we are expecting next
	uint8_t prev_state;                          ///< what state to return to after transpose
	ticks_t last_tick;                           ///< timestamp of last byte into tx_buf
	ticks_t p_tick;                              ///< p-persistence timestamp.
    ticks_t tx_wait_tick;                        ///< timestamp started waiting to tx.
    uint8_t can_tx_now;
    uint8_t hw_cmd_buffer[HW_CMD_BUFFER_SIZE];   ///< data for hw commands.
    uint8_t hw_cmd_len;                          ///< number of bytes in hw_cmd_buffer.
    Params params;                               ///< Operational KISS Parameters that control transmission
} KissCtx;



void kiss_init (KissCtx * k, KFile * channel, KFile * serial);
void kiss_poll_serial (KissCtx * k);
void kiss_poll_modem (KissCtx * k);

