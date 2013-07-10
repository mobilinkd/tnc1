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
 * Does NRZI encode/decode, bit stuffing and stripping, CRC generation and checking.
 * Handles aborts but leaves short packet handling to upper layers
 *
 * \author Robin Gilks <g8ecj@gilks.org>
 */

#include "hdlc.h"
#include "afsk.h"

#include <cfg/macros.h>
#include "cfg/cfg_afsk.h"

#include <cpu/irq.h>

/** Store a bit into the CRC
 * \param hdlc HDLC context.
 * \param lsb_int  bit to be stored.
 */
static void hdlccrcBit(Hdlc * hdlc, bool lsb_int)
{
	uint16_t last_bit;

	last_bit = hdlc->crc ^ lsb_int; // XOR lsb of CRC with the latest bit
	hdlc->crc >>= 1;                // Shift 16-bit CRC one bit to the right

	if (last_bit & 0x0001)          // If XOR result from above has lsb set
		hdlc->crc ^= 0x8408;         // mix in the polynomial

}

/** Store a bit into the current byte. Calculate CRC on the bit
 * When byte is full, and we are in a frame, put it in the fifo
 * \param hdlc HDLC context.
 * \param bit  bit to be stored.
 * \param fifo FIFO buffer used to push characters to.
 */
static int hdlc_store_bit(Hdlc * hdlc, bool bit, FIFOBuffer * fifo)
{

	hdlc->this_byte >>= 1;
	hdlc->this_byte |= bit ? 0x80 : 0;
	hdlc->bit_idx++;
	// hdlccrcBit (hdlc, bit);

	if (hdlc->bit_idx >= 8)
	{
		hdlc->bit_idx = 0;

		// if last octet was a flag then this is real data
		if (hdlc->state == RX_WAIT_DATA)
			hdlc->state = RX_IN_FRAME;

		// only pass data up to app if in a frame
		if (hdlc->state == RX_IN_FRAME)
		{
			// filled a byte, add to message in the queue
			if (fifo_isfull(fifo))
			{
				return HDLC_ERROR_OVERRUN;
			}
			fifo_push(fifo, hdlc->this_byte);
		}
	}
	return HDLC_ERROR_NONE;
}

int hdlc_flush(Hdlc* hdlc, FIFOBuffer* fifo)
{
	// lost carrier; clear any queued data.
	hdlc->state = RX_WAIT_FLAG;
	hdlc->bit_idx = 0;
	fifo_flush(fifo);
	hdlc->ones_count = 0;
	return HDLC_ERROR_ABORT;
}

/**
 * High-Level Data Link Control decoding function.
 * Parse bitstream in order to find characters.
 *
 * \param hdlc HDLC context.
 * \param bit  current bit to be parsed.
 * \param fifo FIFO buffer used to push characters.
 *
 * \return int current status
 */
int hdlc_decode(Hdlc * hdlc, bool bit, FIFOBuffer * fifo)
{
	int ret = HDLC_ERROR_NONE;
	bool this_bit;

	// if bit has changed from last bit then its a zero, else its a one
	if ((bit ^ hdlc->last_bit) & 0x01)
		this_bit = 0;
	else
		this_bit = 1;

	hdlc->last_bit = bit;

	if (this_bit)
	{
		// got a '1', don't try and count too many of them!!
		if (hdlc->ones_count < 8)
			hdlc->ones_count++;

	}
	else
	{
		// got a '0' - see if it closes a bunch of '1's
		// check for abort - mega number of no-transitions
		if (hdlc->ones_count > 6)
		{
			// clear stuff, junk data queued so far
			hdlc->state = RX_WAIT_FLAG;
			// hdlc->crc = 0xffff;
			hdlc->bit_idx = 0;
			fifo_flush(fifo);
			hdlc->ones_count = 0;
			return HDLC_ERROR_ABORT;
		}
		else if (hdlc->ones_count == 6)
		{
			hdlc->ones_count = 0;
			// flag - if in frame then its the end of the frame so check CRC
			if (hdlc->state == RX_IN_FRAME)
			{
				ret = HDLC_PKT_AVAILABLE;
			}
			// not in a frame, just another flag!!
			else
			{
				fifo_flush(fifo);
				ret = HDLC_ERROR_NONE;
			}

			hdlc->state = RX_WAIT_DATA;
			// its (maybe) an opening flag
			// clear CRC and bit counter ready for frame
			// hdlc->crc = 0xffff;
			hdlc->bit_idx = 0;
			hdlc->ones_count = 0;
			return ret;
		}
		else if (hdlc->ones_count == 5)
		{
			// was bit stuffing
			// just clear count and throw bit away - its not part of the CRC
			hdlc->ones_count = 0;
			return ret;
		}
		else
			// its a real '0' so store it (and add to CRC)
			hdlc->ones_count = 0;
	}
	ret = hdlc_store_bit(hdlc, this_bit, fifo);

	return ret;
}

/**
 * High-Level Data Link Control encoding function.
 * Parse fifo in order to find bits to transmit
 * does head, tail pre/post-amble flags and CRC generation
 * and output on TX underrun. Also does NRZI encoding
 *
 * \param hdlc HDLC context.
 * \param fifo FIFO buffer used to pull characters.
 *
 * \return 0/1 data to transmit
 * \return -1 no data available
 */
int hdlc_encode(Hdlc * hdlc, FIFOBuffer * fifo)
{
	int data;

	// see if bit stuffing
	if ((hdlc->ones_count == 5)
		&& ((hdlc->state != TX_HEAD) && (hdlc->state != TX_TAIL)))
	{
		data = 0;
		hdlc->ones_count = 0;
	}
	else
	{
		// not bit stuffing so the data comes from the current output byte
		// see if output of byte complete yet
		if (hdlc->bit_idx >= 8)
		{
			// run out of data - see where we top it up from
			hdlc->bit_idx = 0;
			// these states reflect the actions we are doing (if more than 1 byte involved) or just done
			switch (hdlc->state)
			{
			case TX_HEAD:
				// see if done opening flags, can only get here if data is available so...
				if (--hdlc->flag_count == 0)
				{
					hdlc->this_byte = fifo_pop(fifo);
					hdlc->crc = 0xffff;
					hdlc->bit_idx = 0;
					hdlc->ones_count = 0;
					hdlc->state = TX_IN_FRAME;
				}
				else
				{
					hdlc->this_byte = 0x7e;
				}
				break;
			case TX_IN_FRAME:
				hdlc->this_byte = fifo_pop(fifo);
				// if no more data in fifo then start the CRC
				if (fifo_isempty(fifo))
				{
					hdlc->state = TX_CRC_LO;
				}
				break;
			case TX_CRC_LO:
				hdlc->saved_crc = hdlc->crc;
				hdlc->this_byte = (hdlc->saved_crc & 0xff) ^ 0xff;
				hdlc->state = TX_CRC_HI;
				break;
			case TX_CRC_HI:
				// finished with low byte of CRC, output high byte of CRC
				hdlc->this_byte = (hdlc->saved_crc >> 8) ^ 0xff;
				hdlc->state = TX_TAIL;
				hdlc->flag_count = hdlc->TXtail + 1;    // Always one flag before tail starts.
				break;
			case TX_TAIL:
				// done both CRC bytes, do trailing flags
				if (!fifo_isempty(fifo))
				{
					// Next frame is ready; send back to back.
					hdlc->state = TX_HEAD;
					hdlc->flag_count = 1;
				}
				else
				{
					// send a few flags to make the trailer
					if (--hdlc->flag_count == 0)
						hdlc->state = TX_COMPLETE;
				}
				hdlc->this_byte = 0x7e;
				break;
			case TX_COMPLETE:
				if (!fifo_isempty(fifo))
				{
					// New frame is ready.  Start head.
					hdlc->state = TX_HEAD;
					hdlc->flag_count = hdlc->TXhead;
					hdlc->this_byte = 0x7e;
				}
				else
				{
				    hdlc->bit_idx = 8;
                    return -1;
				}
			}
		}

		data = hdlc->this_byte & 1;
		if (data)
			hdlc->ones_count++;
		hdlc->this_byte >>= 1;
		hdlc->bit_idx++;
		// a bit stuffed zero doesn't update the CRC but all other bits do!
		hdlccrcBit(hdlc, data);
	}

	// NRZI coding - to send a 1 we return the same value as last time
	//               to send a 0 we toggle the value
	if (data == 0)
	{
		hdlc->last_bit ^= 0x01;
		hdlc->ones_count = 0;
	}

	return hdlc->last_bit & 0x01;
}

/**
 * Sets head timing by defining the number of flags to output
 * Has to be done here as this is the only module that knows about flags!!
 *
 * \param hdlc HDLC context.
 * \param txhead preamble
 *
 */
void hdlc_head(Hdlc * hdlc, uint8_t txhead)
{
	hdlc->TXhead = txhead;
}

/**
 * Sets tail timing by defining the number of flags to output
 * Has to be done here as this is the only module that knows about flags!!
 *
 * \param hdlc HDLC context.
 * \param txtail postamble
 *
 */
void hdlc_tail(Hdlc * hdlc, uint8_t txtail)
{
	hdlc->TXtail = txtail;
}

/**
 * Initialise
 * Sets initial state of state-machine
 *
 * \param hdlc HDLC context.
 *
 */
void hdlc_init(Hdlc * hdlc)
{

	hdlc->state = STATE_IDLE;
	hdlc->ones_count = 0;
	hdlc->bit_idx = 0;
	hdlc->error = HDLC_ERROR_NONE;

}

