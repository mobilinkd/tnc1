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
 * \brief KISS protocol handler
 *
 * This module converts KISS encoded data from one KFile object and passes it to another as binary
 * It also passes data the other way
 * It implements KISS commands 1-6
 * It uses the standard p-persist algorithms for keying of a radio
 *
 */

#include "kiss.h"
#include "hw/hw_kiss.h"
#include "cfg/cfg_kiss.h"

#include "hc-05.h"

#define LOG_LEVEL   KISS_LOG_LEVEL
#define LOG_FORMAT  KISS_LOG_FORMAT
#include <cfg/debug.h>
#include <cfg/log.h>

#include <net/afsk.h>

#include <drv/ser.h>
#include <drv/timer.h>
#include <avr/wdt.h>

#include <algo/crc_ccitt.h>

#include <stdlib.h>
#include <string.h>

// KISS/SLIP framing characters
#define FEND     192       /** frame end       */
#define FESC     219       /** frame escape    */
#define TFEND    220       /** transposed fend */
#define TFESC    221       /** transposed fesc */

// KISS commands
#define TXDELAY  1
#define PERSIST  2
#define SLOT     3
#define TXTAIL   4
#define DUPLEX   5
#define HARDWARE 6

// what states I can be in
#define WAIT_FOR_FEND      1
#define WAIT_FOR_COMMAND   2
#define WAIT_FOR_PARAMETER 3
#define WAIT_FOR_TRANSPOSE 4
#define WAIT_FOR_DATA      5
#define WAIT_FOR_TRANSMIT  6

extern uint8_t wdt_location;

/**
 * Load kiss parameters from eeprom
 * Check a basic crc and if in error then load a set of defaults
 *
 * \param k kiss context
 *
 */
static void load_params(KissCtx * k)
{

    KISS_EEPROM_LOAD ();

    if (~(k->params.chksum)
        != k->params.txdelay + k->params.persist + k->params.slot
            + k->params.txtail + k->params.duplex + k->params.hware)
    {
        // load sensible defaults if backing store has a bad checksum
        k->params.txdelay = 50;
        k->params.persist = 64;
        k->params.slot = 10;
        k->params.txtail = 1;
        k->params.duplex = 0;
        k->params.hware = 0;
    }
}

/**
 * Save kiss parameters to eeprom
 * Calculate and store a basic CRC with the data
 *
 * \param k kiss context
 *
 */
static void save_params(KissCtx * k)
{
    k->params.chksum = ~(k->params.txdelay + k->params.persist + k->params.slot
        + k->params.txtail + k->params.duplex + k->params.hware);

    KISS_EEPROM_SAVE ();

}

/**
 * Decode a KISS command
 * KISS protocol defines 6 commands so we try and make use of them
 *
 * \param b parameter value
 * \param k kiss context
 *
 * Uses previously stored command value
 */
static void kiss_decode_command(KissCtx * k, uint8_t b)
{
    switch (k->command)
    {
    case TXDELAY:
        k->params.txdelay = b;
        afsk_head(k->modem, b);
        break;
    case PERSIST:
        k->params.persist = b;
        break;
    case SLOT:
        k->params.slot = b;
        break;
    case TXTAIL:
        k->params.txtail = b;
        afsk_tail(k->modem, b);
        break;
    case DUPLEX:
        k->params.duplex = b;
        break;
    case HARDWARE:
        LOG_INFO("Hardware command not supported");
        break;
    }

    save_params(k);
}

///< Transmit algorithm
///<   if full_duplex
///<     > keyup
///<     PTT
///<     start TXdelay timer
///<   else
///<     start slot timer
///<   fi
///<   if slot expires
///<     if no DCD && random < persist
///<        > keyup
///<        PTT
///<        start TXdelay timer
///<     else
///<        stir random
///<        start slot timer
///<     fi
///<   fi

/**
 * Check KISS parameters to see if/when we can transmit
 *
 * The algorithm above is not to spec.  That said, the spec says nothing
 * about what "full duplex" means in this context. Aside from that, here
 * is what it says about carrier, p-persistence and slot timing.
 *
 * 1. Wait indefinitely for the carrier to clear.
 * 2. If rand(0..1) < persistence
 * 3.    transmit
 * 4. Otherwise
 * 5.    wait slot * 10ms
 * 6.    go to 1.
 *
 * @param k KISS context
 * @return true if we may transmit, otherwise false.
 */
static bool can_transmit(KissCtx* k)
{
    wdt_location = 134;

    // Assume we have the TX channel all to ourselves.  Ignore RX carrier.
    if (k->params.duplex)
    {
        LOG_INFO("TX permitted\r\n");
        return true;
    }

    wdt_location = 135;
    // Waiting to TX?
    if (k->p_tick != 0)
    {
        if (timer_clock() - k->p_tick < ms_to_ticks(k->params.slot * 10))
        {
            return false;
        }
        k->p_tick = 0;
    }

    Afsk* afsk = AFSK_CAST(k->modem);

    // see if the channel is busy
    if (carrier_present(afsk))
    {
        wdt_location = 136;
        uint16_t i = rand();
        if ((i & 0xff) < 4)
        {
            LOG_INFO("TX channel busy\r\n"); // Otherwise really noisy...
        }
        return false;
    }
    // the channel is clear - see if persist allows us to TX
    else
    {
        wdt_location = 137;

        uint16_t i = rand();
        // make an 8 bit random number from a 16 bit one
        if (((i >> 8) ^ (i & 0xff)) < k->params.persist)
        {
            LOG_INFO("TX permitted\r\n");
            return true;
        }
        else
        {
            wdt_location = 138;

            k->p_tick = timer_clock();
            if (k->p_tick == 0)
                k->p_tick++;    // It could happen...

            LOG_INFO("TX persist backoff\r\n");
            return false;
        }
    }
}

static void kiss_tx_to_modem(KissCtx* k, int c)
{
    wdt_location = 139;

    k->tx_pos++;
    kfile_putc(c, k->modem);
}

static void kiss_flush_modem(KissCtx* k)
{
    wdt_location = 140;
    kfile_flush(k->modem);	  // wait for transmitter to finish
    LOG_INFO("TX [%d] complete\r\n", k->tx_pos);
    k->tx_pos = 0;
}

/**
 * Receive function
 * Encode the raw ax25 data as a KISS protocol stream
 *
 * \param k KISS context
 *
 */
static void kiss_tx_to_serial(KissCtx * k)
{
    /// function used by KISS poll to process completed rx'd packets
    /// here, we're just pumping out the KISS data to the serial object

    uint16_t len;
    uint8_t c;

    kfile_putc(FEND, k->serial);
    kfile_putc(0, k->serial);       /// channel 0, data command

    for (len = 0; len < k->rx_pos; len++)
    {
        c = k->rx_buf[len];
        switch (c)
        {
        case FEND:
            kfile_putc(FESC, k->serial);
            kfile_putc(TFEND, k->serial);
            break;
        case FESC:
            kfile_putc(FESC, k->serial);
            kfile_putc(TFESC, k->serial);
            break;
        default:
            kfile_putc(c, k->serial);
        }
    }

    kfile_putc(FEND, k->serial);
}

/**
 * Read incoming binary data from the modem
 * Encode into SLIP encoded data prefixed by a KISS data command and add to KISS object's buffer
 * Pass up to the serial port if HDLC CRC is OK
 *
 * \param k kiss context
 *
 */
void kiss_poll_modem(KissCtx * k)
{
    int c;

    // get octets from modem
    while ((c = kfile_getc(k->modem)) != EOF)
    {
        if (k->rx_pos < CONFIG_KISS_FRAME_BUF_LEN)
        {
            k->rx_buf[k->rx_pos++] = c;
        }
        // otherwise drop it.
    }

    switch (kfile_error(k->modem))
    {
    case HDLC_PKT_AVAILABLE:
        if (k->rx_pos >= KISS_MIN_FRAME_LEN)
        {
            // Check CRC here.
            uint16_t crc = CRC_CCITT_INIT_VAL;
            const uint8_t* end = k->rx_buf + k->rx_pos;
            for (uint8_t* p = k->rx_buf; p != end; p++)
            {
                crc = updcrc_ccitt(*p, crc);
            }
            if (crc != 0xF0B8)  // AX.25 magic CRC
            {
                LOG_INFO("Invalid CRC: %04hX\r\n", crc);
            }
            else
            {
                k->rx_pos -= 2;            // drop the CRC octets
                kiss_tx_to_serial(k);
            }
        }
        else
        {
            LOG_INFO("Short frame\r\n");
        }
        kfile_clearerr(k->modem);
        k->rx_pos = 0;
        break;
    case HDLC_ERROR_CRC:
        LOG_INFO("CRC error\r\n");
        kfile_clearerr(k->modem);
        k->rx_pos = 0;
        break;
    case HDLC_ERROR_OVERRUN:
        if (k->rx_pos >= KISS_MIN_FRAME_LEN)
            LOG_INFO("Buffer overrun\r\n");
        kfile_clearerr(k->modem);
        k->rx_pos = 0;
        break;
    case HDLC_ERROR_ABORT:
        if (k->rx_pos >= KISS_MIN_FRAME_LEN)
            LOG_INFO("Data abort\r\n");
        kfile_clearerr(k->modem);
        k->rx_pos = 0;
        break;
//    default: // ignore other states
    }
}

static void reboot(void)
{
    wdt_disable();
    wdt_enable(WDTO_15MS);
    while (1)
    {
    }
}

/**
 * Read incoming KISS data from serial port
 * Decode the SLIP encoded data and add to KISS object's buffer
 * TX to radio if appropriate
 *
 * \param k kiss context
 *
 */
void kiss_poll_serial(KissCtx * k)
{
    int c;

    /*
     * We use the serial device to buffer the TX data in the serial
     * RX buffer.
     *
     * KISS Persistence requires that we wait indefinitely for the carrier
     * to clear, and that we obey the p-persistence value and slot time.
     * This can lead to multi-second delays in sending.  While this is
     * happening, we may be dropping data from the serial port.
     *
     * Check if it is OK to transmit as soon as the KISS data symbol
     * arrives.  Do not read any data from the serial port until
     */

    wdt_location = 130;
    if (k->state == WAIT_FOR_TRANSMIT)
    {
        if (can_transmit(k))
        {
            k->state = WAIT_FOR_DATA;	// Send it.
        }
        else
        {
            return;
        }
    }

    wdt_location = 131;
    while ((c = kfile_getc(k->serial)) != EOF)
    {
        wdt_reset();
        wdt_location = 132;
        k->last_tick = timer_clock();

        switch (k->state)
        // see what we are looking for
        {
        case WAIT_FOR_FEND:
            if (c == FEND)
            {
                k->state = WAIT_FOR_COMMAND;
                k->tx_pos = 0;
            }
            break;
        case WAIT_FOR_COMMAND:
            if (c == FEND)			  // may get two FEND in a row!!
                break;
            if ((c & 0xf0) != 0)	  // we only support channel 0
            {
                LOG_INFO("Only KISS channel 0 supported\r\n");
                k->state = WAIT_FOR_FEND;
            }
            else if ((c & 0x0f) != 0)
            {
                k->state = WAIT_FOR_PARAMETER;
                k->command = c & 0x0f;
            }
            else
            {
                k->command = c & 0x0f;
                if (k->can_tx_now)
                {
                    k->state = WAIT_FOR_DATA;
                }
                else
                {
                    srand(timer_clock());			// Noise.
                    k->state = WAIT_FOR_TRANSMIT;	// command == data
                    k->tx_wait_tick = timer_clock();
                    return;
                }
            }
            break;
        case WAIT_FOR_PARAMETER:
            kiss_decode_command(k, c);
            k->last_tick = timer_clock();
            k->state = WAIT_FOR_FEND;
            break;
        case WAIT_FOR_TRANSPOSE:
            switch (c)
            // the default is to put whatever character we got in the buffer
            {
            case TFEND:
                c = FEND;
                break;
            case TFESC:
                c = FESC;
                break;
            }

            kiss_tx_to_modem(k, c);
            k->last_tick = timer_clock();
            k->state = WAIT_FOR_DATA;
            break;
        case WAIT_FOR_DATA:
            if (c == FESC)
            {
                k->state = WAIT_FOR_TRANSPOSE;
            }
            else if (c == FEND)
            {
                wdt_location = 151;
                if (k->tx_pos) kiss_flush_modem(k);
                k->last_tick = timer_clock();
                k->state = WAIT_FOR_COMMAND;
                k->can_tx_now = 1;
            }
            else
            {
                kiss_tx_to_modem(k, c);
                k->last_tick = timer_clock();
            }
            break;
        }
    }

    k->can_tx_now = 0;

    wdt_location = 133;

    // sanity checks
    // no serial input in last 2s?
    if (k->state != WAIT_FOR_FEND
        && timer_clock() - k->last_tick > ms_to_ticks(2000L))
    {
        if (k->tx_pos)
        {
            kfile_printf(k->serial, "== RX Timeout (%d)\r\n", k->state);
            wdt_location = 152;
            kiss_flush_modem(k);
        }
        k->state = WAIT_FOR_FEND;
    }
}

/**
 * Initialise the KISS context
 * Check KISS parameters to see if/when we can transmit
 *
 * \param k kiss context
 * \param channel KFile object to the modem
 * \param serial KFile object to the serial (UART) port
 *
 */
void kiss_init(KissCtx * k, KFile * channel, KFile * serial)
{
    /// Initialize KISS TX object
    ASSERT(k);
    ASSERT(channel);
    ASSERT(serial);

    memset(k, 0, sizeof(*k));
    k->modem = channel;
    k->serial = serial;
    k->state = WAIT_FOR_FEND;

    // get KISS parameters from EEPROM
    load_params(k);
    // pass head and tail timing values to modem
    afsk_head(k->modem, k->params.txdelay);
    afsk_tail(k->modem, k->params.txtail);

}

