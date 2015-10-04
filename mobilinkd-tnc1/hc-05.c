// Copyright 2013 Mobilinkd LLC <info@mobilinkd.com>
// All rights reserved.

#include <drv/ser.h>
#include <drv/timer.h>

#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>

#include <string.h>
#include <alloca.h>

#include "hc-05.h"
#include "mobilinkd_util.h"
#include "config.h"

static EEMEM uint16_t bt_initialized;

#define BT_INIT_MAGIC 0xc0a2

const char BT_NAME[] PROGMEM = "Mobilinkd TNC2";
const char NEWLINE[] PROGMEM = "\r\n";

const char OK_rsp[] PROGMEM = "OK";

const char AT_cmd[] PROGMEM = "AT\r\n";
const char RESET_cmd[] PROGMEM = "AT+RESET\r\n";
const char POLAR_cmd[] PROGMEM = "AT+POLAR=1,0\r\n";
const char STATE_qry[] PROGMEM = "AT+STATE?\r\n";
const char NAME_qry[] PROGMEM = "AT+NAME?\r\n";
const char NAME_cmd[] PROGMEM = "AT+NAME=";
const char UART_cmd[] PROGMEM = "AT+UART=38400,0,0\r\n";
const char IPSCAN_cmd[] PROGMEM = "AT+IPSCAN=1024,512,1024,512\r\n";

static bool starts_with_P(KFile* ser, const char* compare, mtime_t timeout_ms)
{
    const ticks_t started = timer_clock();

    size_t len = strlen_P(compare);

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
                if (pgm_read_byte(compare + i) != buffer[i])
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

static bool ends_with_P(KFile* ser, const char* compare, mtime_t timeout_ms)
{
    const ticks_t started = timer_clock();

    size_t len = strlen_P(compare);

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
                if (pgm_read_byte(compare + i) != buffer[j++])
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

void hc05_power_on(void)
{
    HC_05_RESET_PORT |= BV(HC_05_RESET_PIN);    // Bring it back high for normal use.
    HC_05_RESET_DDR &= ~BV(HC_05_RESET_PIN);    // Set BT reset pin to input mode.
    HC_05_POWER_DDR |= BV(HC_05_POWER_PIN);     // Set power pin as output.
    HC_05_POWER_PORT |= BV(HC_05_POWER_PIN);    // Set it high to power on.
    timer_delay(500);
}

void hc05_power_off(void)
{
    HC_05_RESET_DDR |= BV(HC_05_RESET_PIN);     // Set BT reset pin to output mode.
    HC_05_RESET_PORT &= ~BV(HC_05_RESET_PIN);   // Bring it low to reset the BT module.
    timer_delay(5);

    HC_05_POWER_DDR |= BV(HC_05_POWER_PIN);     // Set power pin as output.
    HC_05_POWER_PORT &= ~BV(HC_05_POWER_PIN);   // Set power pin low to power down.
    timer_delay(5);

    // HC_05_POWER_DDR &= ~BV(HC_05_POWER_PIN);    // Set power pin input (high impedance).
    // HC_05_RESET_DDR &= ~BV(HC_05_RESET_PIN);    // Set BT reset pin to input mode.
}

INLINE void hc05_reset(void)
{
    HC_05_RESET_DDR |= BV(HC_05_RESET_PIN);     // Set BT reset pin to output mode.
    HC_05_RESET_PORT &= ~BV(HC_05_RESET_PIN);   // Bring it low to reset the BT module.
    timer_delay(200L);
    HC_05_RESET_PORT |= BV(HC_05_RESET_PIN);    // Bring it back high for normal use.
    HC_05_RESET_DDR &= ~BV(HC_05_RESET_PIN);    // Set BT reset pin to input mode.
    timer_delay(800L);
}

INLINE void hc05_command_mode(void)
{
    // Bring PD2 HIGH to enter command mode.
    HC_05_COMMAND_PORT |= BV(HC_05_COMMAND_PIN);
}

INLINE void hc05_normal_mode(void)
{
    // Bring PD2 LOW to exit command mode.
    HC_05_COMMAND_PORT &= ~BV(HC_05_COMMAND_PIN);
}

bool hc05_connected()
{
    return (HC_05_STATUS_PORT & BV(HC_05_STATUS_PIN)) ? false : true;
}

INLINE bool hc05_soft_reset(KFile* ser)
{
    // Do a soft reset here.
    kfile_print_P(ser, RESET_cmd);
    kfile_flush(ser);
    return starts_with_P(ser, OK_rsp, 1000L);
}

/**
 * Adjust the polarity of the PI09 pin so that it goes LOW when a
 * connection is established.  And set PORTB5 on the AVR to INPUT
 * with the pull-up resistor engaged.
 *
 * In this configuration, we can track the connection status.  And
 * we know that the TNC1 has been modified for connection tracking
 * because that should be the only way that this pin goes low.
 *
 * If the connection between PI09 and PORTB5 has not been made, then
 * PORTB5 should always be HIGH, even when we can safely assume that
 * a Bluetooth connection does exist.
 *
 * When in the TNC configuration mode (e.g. GET_ALL_VALUES was sent)
 * the Bluetooth connection must be established.  (There is a race
 * condition here but it can be safely ignored.)  If GET_ALL_VALUES
 * is sent and PORTB5 is low, we assume that the TNC can do
 * connection tracking and CAP_BT_CONN_TRACK is returned as a
 * capability and the GET_BT_CONN_TRACK state is returned via
 * GET_ALL_VALUES.
 *
 * The configuration programs can safely assume that when a
 * GET_BT_CONN_TRACK value is returned via GET_ALL_VALUES that
 * connection tracking is possible.
 */
INLINE bool hc05_adjust_polarity(KFile* ser)
{
    // Set PB5 to input with pull-up engaged.
    HC_05_STATUS_DDR &= ~BV(HC_05_STATUS_PIN);
    HC_05_STATUS_PORT |= BV(HC_05_STATUS_PIN);

    kfile_print_P(ser, POLAR_cmd);
    kfile_flush(ser);
    return starts_with_P(ser, OK_rsp, 1000L);
}

uint8_t init_hc05(KFile* ser)
{
    HC_05_COMMAND_DDR |= BV(HC_05_COMMAND_PIN);

    uint8_t result = 0;

    if (eeprom_read_word(&bt_initialized) == BT_INIT_MAGIC)
        return result;

    timer_delay(800L);
    hc05_command_mode();
    timer_delay(1000L);
    hc05_reset();

    kfile_print_P(ser, AT_cmd);
    kfile_flush(ser);
    starts_with_P(ser, OK_rsp, 1000L); // ignore the first one.

    kfile_print_P(ser, AT_cmd);
    kfile_flush(ser);
    bool cmd_ok = starts_with_P(ser, OK_rsp, 1000L);

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

    if (!hc05_adjust_polarity(ser)) result |= 1;

    kfile_print_P(ser, NAME_qry);
    kfile_flush(ser);
    bool name_ok = ends_with_P(ser, BT_NAME, 1000);
    if (!starts_with_P(ser, OK_rsp, 1000L))
        result |= 2;

    if (!name_ok)   // Need to initialize the HC-05 module.
    {
        result |= 4;
        kfile_print_P(ser, NAME_cmd);
        kfile_print_P(ser, BT_NAME);
        kfile_print_P(ser, NEWLINE);
        kfile_flush(ser);
        if (!starts_with_P(ser, OK_rsp, 1000L))
            result |= 8;

        kfile_print_P(ser, UART_cmd);
        kfile_flush(ser);
        if (!starts_with_P(ser, OK_rsp, 1000L))
            result |= 16;

        kfile_print_P(ser, RESET_cmd);
        kfile_flush(ser);
        if (!starts_with_P(ser, OK_rsp, 1000L))
            result |= 32;

    }

    timer_delay(1000L);
    hc05_normal_mode();

    hc05_reset();   // Hard reset needed to activate POLAR cmd.

    eeprom_write_word((uint16_t*) &bt_initialized, BT_INIT_MAGIC);

    return result;
}
