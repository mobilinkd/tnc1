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
 * Copyright 2012 Robin Gilks
 *
 * -->
 *
 * \brief HDLC handler.
 *
 * \author Robin Gilks <g8ecj@gilks.org>
 * $WIZ$ module_name = "hdlc"
 * $WIZ$ module_depends = "kfile"
 */

#ifndef NET_HDLC_H
#define NET_HDLC_H

#include <stdbool.h>
#include <struct/fifobuf.h>


/**
 * HDLC (High-Level Data Link Control) context.
 * Maybe to be moved in a separate HDLC module one day.
 */
typedef struct Hdlc
{
	uint8_t ones_count;          ///< how many non-stuffed 1's we have
	uint8_t flag_count;          ///< working copy of number of flags for txhead or txtail
	uint8_t TXhead;              ///< number of flags for TXhead
	uint8_t TXtail;              ///< number of flags for TXtail
	uint8_t this_byte;           ///< The current bit being worked on.
	uint8_t bit_idx;             ///< Current bit position.
	uint8_t last_bit;            ///< value of last bit
	uint16_t crc;                ///< running CRC
	uint16_t saved_crc;          ///< CRC saved for output
	uint8_t state;               ///< state: in frame, seen flag, waiting for data or flag, processing crc1, crc2, closing flag
	uint8_t error;               ///< last error
} Hdlc;

#define STATE_IDLE         1

// state values for RX
#define  RX_WAIT_FLAG      STATE_IDLE
#define  RX_WAIT_DATA      2
#define  RX_IN_FRAME       3

// state values for TX
#define  TX_COMPLETE       STATE_IDLE
#define  TX_HEAD           2
#define  TX_IN_FRAME       3
#define  TX_CRC_LO         4
#define  TX_CRC_HI         5
#define  TX_TAIL           6

#define HDLC_ERROR_NONE    0
#define HDLC_ERROR_OVERRUN 1
#define HDLC_ERROR_CRC     2
#define HDLC_ERROR_ABORT   3
#define HDLC_PKT_AVAILABLE 4

// this is an odd value because we process the hdlc data as a stream and we
// don't see the closing flag until we have already put 7 bits of it into the CRC
#define HDLC_GOOD_CRC   0x4f85


int hdlc_flush(Hdlc* hdlc, FIFOBuffer* fifo);
int hdlc_decode (Hdlc * hdlc, bool bit, FIFOBuffer * fifo);
int hdlc_encode (Hdlc * hdlc, FIFOBuffer * fifo);
void hdlc_head (Hdlc * hdlc, uint8_t txhead);
void hdlc_tail (Hdlc * hdlc, uint8_t txtail);
void hdlc_init (Hdlc * hdlc);

#endif
