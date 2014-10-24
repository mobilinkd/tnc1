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
 * Portions Copyright 2013 Mobilinkd LLC <rob@mobilinkd.com>
 *
 * -->
 *
 * \author Robin Gilks <g8ecj@gilks.org>
 * \author Mobilinkd LLC <rob@mobilinkd.com>
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
#include "cfg/cfg_kiss.h"
#include "mobilinkd_eeprom.h"
#include "mobilinkd_version.h"
#include "mobilinkd_error.h"

#include "hc-05.h"
#include "battery.h"

#define LOG_LEVEL   KISS_LOG_LEVEL
#define LOG_FORMAT  KISS_LOG_FORMAT
#include <cfg/debug.h>
#include <cfg/log.h>

#include <net/afsk.h>

#include <drv/ser.h>
#include <drv/timer.h>
#include <avr/wdt.h>
#include <avr/pgmspace.h>
#include <io/kfile.h>

#include <algo/crc_ccitt.h>

#include <stdlib.h>
#include <string.h>

// KISS/SLIP framing characters
#define FEND     192       /** frame end       */
#define FESC     219       /** frame escape    */
#define TFEND    220       /** transposed fend */
#define TFESC    221       /** transposed fesc */
#define ESCAPE    27       /** bootloader!     */

// KISS commands
#define TXDELAY  1
#define PERSIST  2
#define SLOT     3
#define TXTAIL   4
#define DUPLEX   5
#define HARDWARE 6

// what states I can be in
#define BT_NOT_CONNECTED   0
#define WAIT_FOR_FEND      1
#define WAIT_FOR_COMMAND   2
#define WAIT_FOR_PARAMETER 3
#define WAIT_FOR_TRANSPOSE 4
#define WAIT_FOR_DATA      5
#define WAIT_FOR_TRANSMIT  6
#define STREAM_VOLUME      7

// HW commands (should be in kiss HW module?)
#define SAVE                    0   // Save settings to EEPROM.
#define SET_OUTPUT_VOLUME       1
#define SET_INPUT_ATTEN         2
#define SET_SQUELCH_LEVEL       3
#define POLL_INPUT_VOLUME       4
#define STREAM_INPUT_VOLUME     5
#define GET_BATTERY_LEVEL       6
#define SEND_MARK               7
#define SEND_SPACE              8
#define SEND_BOTH               9
#define STOP_TX                10
#define RESET                  11
#define GET_OUTPUT_VOLUME      12
#define GET_INPUT_ATTEN        13
#define GET_SQUELCH_LEVEL      14

#define SET_VERBOSITY          16
#define GET_VERBOSITY          17

#define GET_TXDELAY            33
#define GET_PERSIST            34
#define GET_TIMESLOT           35
#define GET_TXTAIL             36
#define GET_DUPLEX             37

#define GET_FIRMWARE_VERSION   40
#define GET_HARDWARE_VERSION   41

#define SET_BLUETOOTH_NAME     65
#define GET_BLUETOOTH_NAME     66
#define SET_BLUETOOTH_PIN      67   // Danger Will Robinson.
#define GET_BLUETOOTH_PIN      68
#define SET_BT_CONN_TRACK      69   // Bluetooth connection tracking
#define GET_BT_CONN_TRACK      70   // Bluetooth connection tracking
#define SET_BT_MAJOR_CLASS     71   // Bluetooth Major Class
#define GET_BT_MAJOR_CLASS     72   // Bluetooth Major Class

#define SET_USB_POWER_ON       73   // Power on when USB power available
#define GET_USB_POWER_ON       74
#define SET_USB_POWER_OFF      75   // Power off when USB power unavailable
#define GET_USB_POWER_OFF      76
#define SET_BT_POWER_OFF       77   // Power off after n seconds w/o BT conn
#define GET_BT_POWER_OFF       78

#define SET_PTT_CHANNEL        79   // Which PTT line to use (currently 0 or 1,
#define GET_PTT_CHANNEL        80   // multiplex or simplex)

#define GET_CAPABILITIES      126   // Send all capabilities.
#define GET_ALL_VALUES        127   // Send all settings & versions.


extern uint8_t wdt_location;

static uint8_t checksum(KissCtx* k)
{
    return ~(k->params.txdelay + k->params.persist + k->params.slot
        + k->params.txtail + k->params.duplex + k->params.output_volume
        + k->params.input_volume + k->params.squelch + k->params.options);
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
    k->params.chksum = checksum(k);

    MOBILINKD_EEPROM_SAVE((k->params));
}


/**
 * Load kiss parameters from eeprom
 * Check a basic crc and if in error then load a set of defaults
 *
 * \param k kiss context
 *
 */
static void load_params(KissCtx * k)
{

    MOBILINKD_EEPROM_LOAD((k->params));

    if (k->params.chksum != checksum(k))
    {
        // load sensible defaults if backing store has a bad checksum
        k->params.txdelay = 50;
        k->params.persist = 64;
        k->params.slot = 10;
        k->params.txtail = 1;
        k->params.duplex = 0;
        k->params.output_volume = 128;
        k->params.input_volume = 2;
        k->params.squelch = 2;
        k->params.options = KISS_OPTION_VIN_POWER_ON | KISS_OPTION_VIN_POWER_OFF | KISS_OPTION_PTT_SIMPLEX;
        save_params(k);
    }
}

INLINE void kiss_tx_buffer_to_serial(
    KissCtx* k, const uint8_t* buf, uint16_t len)
{
    Afsk* afsk = AFSK_CAST(k->modem);

    for (uint16_t i = 0; i < len; i++)
    {
        afsk_rx_bottom_half(afsk);
        uint8_t c = buf[i];
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
}

static void kiss_tx_to_serial(
    KissCtx * k, uint8_t type, const uint8_t* buf, uint16_t len)
{
    kfile_putc(FEND, k->serial);
    kfile_putc(type, k->serial);

    kiss_tx_buffer_to_serial(k, buf, len);

    kfile_putc(FEND, k->serial);
}

INLINE void kiss_tx_buffer_to_serial_P(
    KissCtx* k, const uint8_t* buf, uint16_t len)
{
    Afsk* afsk = AFSK_CAST(k->modem);

    for (uint16_t i = 0; i < len; i++)
    {
        afsk_rx_bottom_half(afsk);
        uint8_t c = pgm_read_byte(buf + i);
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
}

static void kiss_tx_to_serial_P(
    KissCtx * k, uint8_t type, const uint8_t* buf, uint16_t len)
{
    kfile_putc(FEND, k->serial);
    kfile_putc(type, k->serial);

    kiss_tx_buffer_to_serial_P(k, buf, len);

    kfile_putc(FEND, k->serial);
}

INLINE void kiss_change_state(KissCtx* k, uint8_t state)
{
    LOG_INFO("kiss_change_state %d -> %d\r\n", (int) k->state, (int) state);
    k->state = state;
}

static void send_input_atten(KissCtx* k)
{
    uint8_t buf[2];
    buf[0] = GET_INPUT_ATTEN;
    buf[1] = get_input_atten(k->modem);
    kiss_tx_to_serial(k, HARDWARE, buf, 2);
    kfile_flush(k->serial);
}

static void send_input_volume(KissCtx* k)
{
    uint8_t buf[2];
    buf[0] = POLL_INPUT_VOLUME;
    buf[1] = get_input_volume(k->modem);
    kiss_tx_to_serial(k, HARDWARE, buf, 2);
    kfile_flush(k->serial);
}

static void send_output_volume(KissCtx* k)
{
    uint8_t buf[2];
    buf[0] = GET_OUTPUT_VOLUME;
    buf[1] = k->params.output_volume;
    kiss_tx_to_serial(k, HARDWARE, buf, 2);
    kfile_flush(k->serial);
}

static void send_squelch_level(KissCtx* k)
{
    uint8_t buf[2];
    buf[0] = GET_SQUELCH_LEVEL;
    buf[1] = k->params.squelch;
    kiss_tx_to_serial(k, HARDWARE, buf, 2);
    kfile_flush(k->serial);
}

static void send_txdelay(KissCtx* k)
{
    uint8_t buf[2];
    buf[0] = GET_TXDELAY;
    buf[1] = k->params.txdelay;
    kiss_tx_to_serial(k, HARDWARE, buf, 2);
    kfile_flush(k->serial);
}

static void send_persist(KissCtx* k)
{
    uint8_t buf[2];
    buf[0] = GET_PERSIST;
    buf[1] = k->params.persist;
    kiss_tx_to_serial(k, HARDWARE, buf, 2);
    kfile_flush(k->serial);
}

static void send_timeslot(KissCtx* k)
{
    uint8_t buf[2];
    buf[0] = GET_TIMESLOT;
    buf[1] = k->params.slot;
    kiss_tx_to_serial(k, HARDWARE, buf, 2);
    kfile_flush(k->serial);
}

static void send_txtail(KissCtx* k)
{
    uint8_t buf[2];
    buf[0] = GET_TXTAIL;
    buf[1] = k->params.txtail;
    kiss_tx_to_serial(k, HARDWARE, buf, 2);
    kfile_flush(k->serial);
}

static void send_duplex(KissCtx* k)
{
    uint8_t buf[2];
    buf[0] = GET_DUPLEX;
    buf[1] = k->params.duplex;
    kiss_tx_to_serial(k, HARDWARE, buf, 2);
    kfile_flush(k->serial);
}

static void send_firmware_version(KissCtx* k)
{
    kiss_tx_to_serial_P(k, HARDWARE, (const uint8_t*)firmware_version,
        strlen_P(firmware_version));
    kfile_flush(k->serial);
}

static void send_hardware_version(KissCtx* k)
{
    kiss_tx_to_serial_P(k, HARDWARE, (const uint8_t*)hardware_version,
        strlen_P(hardware_version));
    kfile_flush(k->serial);
}

static void send_battery_level(KissCtx* k)
{
    uint16_t level = check_battery();
    uint8_t buf[3];
    buf[0] = GET_BATTERY_LEVEL;
    buf[1] = (uint8_t)(level >> 8);
    buf[2] = (uint8_t)(level & 0xFF);
    kiss_tx_to_serial(k, HARDWARE, buf, 3);
    kfile_flush(k->serial);
}

static void send_verbosity(KissCtx* k)
{
    uint8_t buf[2];
    buf[0] = GET_VERBOSITY;
    buf[1] = kiss_get_verbosity(k);
    kiss_tx_to_serial(k, HARDWARE, buf, 2);
    kfile_flush(k->serial);
}

static void send_bt_conn_track(KissCtx* k)
{
    uint8_t buf[2];
    buf[0] = GET_BT_CONN_TRACK;
    buf[1] = kiss_get_conn_track(k);
    kiss_tx_to_serial(k, HARDWARE, buf, 2);
    kfile_flush(k->serial);
}

static void send_capabilities(KissCtx* k)
{
    uint8_t buf[3];
    buf[0] = GET_CAPABILITIES;
    buf[1] = CAP_DCD | CAP_BATTERY_LEVEL | CAP_FIRMWARE_VERSION | \
        CAP_INPUT_ATTEN | CAP_SQUELCH;
    // TNC is connected.  If detected, connection tracking is possible.
    buf[1] |= hc05_connected() ? CAP_BT_CONN_TRACK : 0;
    buf[2] = (CAP_VERBOSE_ERROR) >> 8;
    kiss_tx_to_serial(k, HARDWARE, buf, 2);
    kfile_flush(k->serial);
}

static void send_all_values(KissCtx* k)
{
    send_hardware_version(k);
    send_firmware_version(k);
    send_txdelay(k);
    send_persist(k);
    send_timeslot(k);
    send_duplex(k);
    send_output_volume(k);
    send_squelch_level(k);
    send_input_atten(k);
    send_battery_level(k);
    send_verbosity(k);
    send_capabilities(k);
    // TNC is connected.  If detected, connection tracking is possible.
    if (hc05_connected()) send_bt_conn_track(k);
}

static void kiss_decode_hw_command(KissCtx * k)
{
    // PARAMS are saved by the caller.

    uint8_t cmd = k->tx_buf[0];
    uint8_t value = k->tx_buf[1];

    LOG_INFO("kiss_decode_hw_command (%d)\r\n", (int) cmd);

    switch (cmd)
    {
    case SET_OUTPUT_VOLUME:
        k->params.output_volume = value;
        save_params(k);
        set_output_volume(k->modem, value);
        LOG_INFO("SET_OUTPUT_VOLUME (%d)\r\n", (int) value);
        break;
    case SET_INPUT_ATTEN:
        k->params.input_volume = value;
        save_params(k);
        set_input_atten(k->modem, value);
        LOG_INFO("SET_INPUT_VOLUME (%d)\r\n", (int) value);
        break;
    case SET_SQUELCH_LEVEL:
        k->params.squelch = value;
        save_params(k);
        set_squelch_level(k->modem, value);
        break;
    case POLL_INPUT_VOLUME:
        afsk_test_tx_end(k->modem);
        send_input_volume(k);
        break;
    case STREAM_INPUT_VOLUME:
        afsk_test_tx_end(k->modem);
        send_input_volume(k);
        kiss_change_state(k, STREAM_VOLUME);
        k->last_tick = timer_clock();
        break;
    case SEND_MARK:
        afsk_test_tx_start(k->modem, HDLC_TEST_MARK);
        break;
    case SEND_SPACE:
        afsk_test_tx_start(k->modem, HDLC_TEST_SPACE);
        break;
    case SEND_BOTH:
        afsk_test_tx_start(k->modem, HDLC_TEST_BOTH);
        break;
    case STOP_TX:
        afsk_test_tx_end(k->modem);
        break;
    case GET_OUTPUT_VOLUME:
        afsk_test_tx_end(k->modem);
        send_output_volume(k);
        break;
    case GET_INPUT_ATTEN:
        afsk_test_tx_end(k->modem);
        send_input_atten(k);
        break;
    case GET_SQUELCH_LEVEL:
        afsk_test_tx_end(k->modem);
        send_squelch_level(k);
        break;
    case GET_TXDELAY:
        afsk_test_tx_end(k->modem);
        send_txdelay(k);
        break;
    case GET_PERSIST:
        afsk_test_tx_end(k->modem);
        send_persist(k);
        break;
    case GET_TIMESLOT:
        afsk_test_tx_end(k->modem);
        send_timeslot(k);
        break;
    case GET_TXTAIL:
        afsk_test_tx_end(k->modem);
        send_txtail(k);
        break;
    case GET_DUPLEX:
        afsk_test_tx_end(k->modem);
        send_duplex(k);
        break;
    case GET_FIRMWARE_VERSION:
        afsk_test_tx_end(k->modem);
        send_firmware_version(k);
        break;
    case GET_HARDWARE_VERSION:
        afsk_test_tx_end(k->modem);
        send_hardware_version(k);
        break;
    case GET_BATTERY_LEVEL:
        afsk_test_tx_end(k->modem);
        send_battery_level(k);
        break;
    case SET_VERBOSITY:
        kiss_set_verbosity(k, value);
        save_params(k);
        break;
    case GET_VERBOSITY:
        send_verbosity(k);
        break;
    case SET_BT_CONN_TRACK:
        if (hc05_connected())
        {
            kiss_set_conn_track(k, value);
            save_params(k);
        }
        break;
    case GET_BT_CONN_TRACK:
        if (hc05_connected())
        {
            send_bt_conn_track(k);
        }
        break;
    case GET_CAPABILITIES:
        afsk_test_tx_end(k->modem);
        send_capabilities(k);
        break;
    case GET_ALL_VALUES:
        afsk_test_tx_end(k->modem);
        send_all_values(k);
        break;
    }
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
static void kiss_decode_command(KissCtx * k)
{
    uint8_t b = k->tx_buf[0];

    LOG_INFO("kiss_decode_command (%d)\r\n", (int)b);

    switch (k->command)
    {
    case TXDELAY:
        k->params.txdelay = b;
        save_params(k);
        afsk_head(k->modem, b);
        break;
    case PERSIST:
        k->params.persist = b;
        save_params(k);
        break;
    case SLOT:
        k->params.slot = b;
        save_params(k);
        break;
    case TXTAIL:
        k->params.txtail = b;
        save_params(k);
        afsk_tail(k->modem, b);
        break;
    case DUPLEX:
        k->params.duplex = b;
        save_params(k);
        break;
    case HARDWARE:
        kiss_decode_hw_command(k);
        break;
    }

    k->tx_pos = 0;
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

INLINE void kiss_flush_modem(KissCtx* k)
{
    wdt_location = 140;
    kfile_flush(k->modem);	  // wait for transmitter to finish
    LOG_INFO("TX [%d] complete\r\n", k->tx_pos);
    k->tx_pos = 0;
}

/**
 * Should the packet be sent via Bluetooth connection.  Return true unless
 * connection tracking is enabled and there is no Bluetooth connection
 * established.
 *
 * @param k is the KISS context.
 */
INLINE bool kiss_can_send(KissCtx* k)
{
    return (!kiss_get_conn_track(k) || hc05_connected());
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

    if (k->state == STREAM_VOLUME || (kfile_error(k->modem) & 16)) return;

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
                mobilinkd_set_error(MOBILINKD_ERROR_RX_CRC);
            }
            else
            {
                k->rx_pos -= 2;            // drop the CRC octets
                if (kiss_can_send(k))
                    kiss_tx_to_serial(k, 0, k->rx_buf, k->rx_pos);
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
        mobilinkd_set_error(MOBILINKD_ERROR_RX_CRC);
        kfile_clearerr(k->modem);
        k->rx_pos = 0;
        break;
    case HDLC_ERROR_OVERRUN:
        if (k->rx_pos >= KISS_MIN_FRAME_LEN)
        {
            mobilinkd_set_error(MOBILINKD_ERROR_RX_BUFFER_OVERFLOW);
            LOG_INFO("Buffer overrun\r\n");
        }
        kfile_clearerr(k->modem);
        k->rx_pos = 0;
        break;
    case HDLC_ERROR_ABORT:
        if (k->rx_pos >= KISS_MIN_FRAME_LEN)
        {
            mobilinkd_set_error(MOBILINKD_ERROR_RX_ABORT);
            LOG_INFO("Data abort\r\n");
        }
        kfile_clearerr(k->modem);
        k->rx_pos = 0;
        break;
//    default: // ignore other states
    }
}

static void __attribute__ ((noreturn)) reboot(void)
{
    cli();
    wdt_disable();
    wdt_enable(WDTO_15MS);
    while (1) {}
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
     * If we are in WAIT_FOR_TRANSMIT, there is a complete packet
     * ready to send.
     *
     * KISS Persistence requires that we wait indefinitely for the carrier
     * to clear, and that we obey the p-persistence value and slot time.
     * This can lead to multi-second delays in sending.  While this is
     * happening, we may be dropping data from the serial port.
     *
     * We attempted to check if it is OK to transmit as soon as the first
     * KISS data symbol arrives, using the serial buffer as the TX buffer.
     * It seems older, slower Android devices will not TX all of the data
     * fast enough, stuttering the BT serial data, which would lead to
     * corrupted packets.  So that's not feasible.
     */

    wdt_location = 130;
    if (k->state == WAIT_FOR_TRANSMIT)
    {
        if (can_transmit(k))
        {
            kfile_write(k->modem, k->tx_buf, k->tx_pos);
            kiss_flush_modem(k);
            k->tx_pos = 0;
            kiss_change_state(k, WAIT_FOR_DATA);
        }
        else
        {
            return;
        }
    }

    wdt_location = 131;

    // Check for serial errors.  Happens during packet ops because of
    // the small buffer sizes.
    if ((c = kfile_error(k->serial)) != 0)
    {
        if (c & (SERRF_RXFIFOOVERRUN | SERRF_RXSROVERRUN))
            mobilinkd_set_error(MOBILINKD_ERROR_SERIAL_RX_OVERRUN);
        else
            mobilinkd_set_error(MOBILINKD_ERROR_SERIAL_RX_ERROR);

        kfile_clearerr(k->serial);
        k->tx_pos = 0;
        kiss_change_state(k, WAIT_FOR_FEND);
        return;
    }

    while ((c = kfile_getc(k->serial)) != EOF)
    {
        wdt_reset();
        wdt_location = 132;
        k->last_tick = timer_clock();

        switch (k->state)
        // see what we are looking for
        {
        case STREAM_VOLUME:
        case WAIT_FOR_FEND:
            if (c == FEND)
            {
                kiss_change_state(k, WAIT_FOR_COMMAND);
                k->tx_pos = 0;
                k->escape_count = 0;
                break;
            }
            // Bootloader require <esc>S<esc>S<esc>S<esc>S sequence.
            else if ((c == ESCAPE) && ((k->escape_count & 1) == 0))
            {
                k->escape_count++;
                break;
            }
            else if ((c == 'S') && ((k->escape_count & 1) == 1))
            {
                k->escape_count++;
                kfile_putc('?', k->serial);
                kfile_flush(k->serial);
                if (k->escape_count == 8) reboot();
                break;
            }
            else
            {
                k->escape_count = 0;
            }
            break;
        case WAIT_FOR_COMMAND:
            if (c == FEND)			  // may get two FEND in a row!!
                break;
            if ((c & 0xf0) != 0)	  // we only support channel 0
            {
                LOG_INFO("Only KISS channel 0 supported\r\n");
                kiss_change_state(k, WAIT_FOR_FEND);
            }
            else
            {
                k->command = c & 0x0f;
                ticks_t t = timer_clock();
                srand(t);			// Noise.
                k->tx_wait_tick = t;
                kiss_change_state(k, WAIT_FOR_DATA);	// command == data
            }
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
            k->tx_buf[k->tx_pos++] = c;
            k->last_tick = timer_clock();
            kiss_change_state(k, WAIT_FOR_DATA);
            break;
        case WAIT_FOR_DATA:
            if (c == FESC)
            {
                kiss_change_state(k, WAIT_FOR_TRANSPOSE);
            }
            else if (c == FEND || k->tx_pos == CONFIG_KISS_FRAME_BUF_LEN)
            {
                if (k->command != 0)
                {
                    kiss_change_state(k, WAIT_FOR_COMMAND);
                    kiss_decode_command(k);
                }
                else
                {
                    wdt_location = 151;
                    if (k->tx_pos >= KISS_MIN_FRAME_LEN)
                    {
                        kiss_change_state(k, WAIT_FOR_TRANSMIT);
                        k->last_tick = timer_clock();
                        return;
                    }
                    else
                    {
                        k->tx_pos = 0;
                        kiss_change_state(k, WAIT_FOR_COMMAND);
                    }
                }
            }
            else
            {
                k->tx_buf[k->tx_pos++] = c;
                k->last_tick = timer_clock();
            }
            break;
        }
    }

    wdt_location = 133;

    if (k->state == STREAM_VOLUME
        && timer_clock() - k->last_tick > ms_to_ticks(100L))
    {
        send_input_volume(k);
        k->last_tick = timer_clock();
        return;
    }


    // sanity checks
    // no serial input in last 2s?
    if (k->state != WAIT_FOR_FEND
        && timer_clock() - k->last_tick > ms_to_ticks(2000L))
    {
        if (k->tx_pos != 0)
        {
            mobilinkd_set_error(MOBILINKD_ERROR_SERIAL_RX_TIMEOUT);
            LOG_ERR("== RX Timeout (%d)\r\n", k->state);
            k->tx_pos = 0;
        }
        kiss_change_state(k, WAIT_FOR_FEND);
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
    set_output_volume(k->modem, k->params.output_volume);
    set_input_atten(k->modem, k->params.input_volume);
    set_squelch_level(k->modem, k->params.squelch);
}

