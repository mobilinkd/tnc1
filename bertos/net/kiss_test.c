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
 * Copyright 2009 Develer S.r.l. (http://www.develer.com/)
 *
 * -->
 *
 * \brief KISS test.
 *
 * $test$: cp bertos/cfg/cfg_kiss.h $cfgdir/
 *
 * \author Robin Gilks <g8ecj@gilks.org>
 */

#include "kiss.h"
#include "hdlc.h"

#include <string.h>

#include <struct/kfile_mem.h>

#include <cfg/debug.h>
#include <cfg/kfile_debug.h>
#include <cfg/test.h>


static KissCtx kiss;

KFileMem mem;						  // used to send data
KFileMem mem1;						  // used to receive data

uint8_t kiss_packet_raw[] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
	180, 181, 182, 183, 184, 185, 185, 187, 188, 189,
	190, 191, 192, 193, 194, 195, 196, 197, 198, 199,
	210, 211, 212, 213, 214, 215, 216, 217, 218, 219,
	220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 15, 248      /* includes CRC */
};

uint8_t kiss_packet_kiss[] = { 192, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
	180, 181, 182, 183, 184, 185, 185, 187, 188, 189,
	190, 191, 219, 220, 193, 194, 195, 196, 197, 198, 199,
	210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 221,
	220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 192
};

uint8_t kiss_packet_rx[100];


static int
test_bodge (UNUSED_ARG (KFile *, fd))
{
	return HDLC_PKT_AVAILABLE;
}

int
kiss_testSetup (void)
{
	kdbg_init ();
	// what I send into the kiss module via the modem interface
	kfilemem_init (&mem, kiss_packet_raw, sizeof (kiss_packet_raw));
	// actually get stuff back into kiss_packet_rx
	kfilemem_init (&mem1, kiss_packet_rx, sizeof (kiss_packet_rx));
	mem.fd.error = test_bodge;	  // kfilemem has no error hook!! Naughty naughty kfilemem
	kiss_init (&kiss, &mem.fd, &mem1.fd);
	return 0;
}

int
kiss_testTearDown (void)
{
	return 0;
}

int
kiss_testRun (void)
{

	kiss_poll_modem (&kiss);
	ASSERT (memcmp (kiss_packet_rx, kiss_packet_kiss, sizeof (kiss_packet_kiss)) == 0);
	kprintf ("modem to serial KISS data flow OK\n");

	// operate data flow from serial to modem
	kfilemem_init (&mem, kiss_packet_kiss, sizeof (kiss_packet_kiss));
	kfilemem_init (&mem1, kiss_packet_rx, sizeof (kiss_packet_rx));
	kiss_init (&kiss, &mem1.fd, &mem.fd);
	kiss_poll_serial (&kiss);
	ASSERT (memcmp (kiss_packet_rx, kiss_packet_raw, sizeof (kiss_packet_raw - 2)) == 0);  // ignore CRC in check
	kprintf ("serial to modem binary data flow OK\n");
	return 0;
}

TEST_MAIN (kiss);
