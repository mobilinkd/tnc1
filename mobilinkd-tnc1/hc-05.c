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

static EEMEM uint16_t bt_initialized = 0;

#define BT_INIT_MAGIC 0xc0a1

const uint8_t BT_RESET_PIN = 3;
const uint8_t BT_COMMAND_PIN = 2;

const char BT_NAME[] PROGMEM = "Mobilinkd TNC1";
const char NEWLINE[] PROGMEM = "\r\n";

const char OK_rsp[] PROGMEM = "OK";
const char CONNECTED_rsp[] PROGMEM = "+STATE:CONNECTED";

const char AT_cmd[] PROGMEM = "AT\r\n";
const char RESET_cmd[] PROGMEM = "AT+RESET\r\n";
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

INLINE void hc05_reset(void)
{
    DDRD |= BV(BT_RESET_PIN);       // Set BT reset pin to output mode.
    PORTD &= ~BV(BT_RESET_PIN);     // Bring it low to reset the BT module.
    timer_delay(200L);
    PORTD |= BV(BT_RESET_PIN);      // Bring it back high for normal use.
    DDRD &= ~BV(BT_RESET_PIN);      // Set BT reset pin to input mode.
    timer_delay(800L);
}

INLINE void hc05_command_mode(void)
{
    // Bring PD2 HIGH to enter command mode.
    PORTD |= BV(BT_COMMAND_PIN);
}

INLINE void hc05_normal_mode(void)
{
    // Bring PD2 LOW to exit command mode.
    PORTD &= ~BV(BT_COMMAND_PIN);
}

INLINE bool hc05_soft_reset(KFile* ser)
{
    // Do a soft reset here.
    kfile_print_P(ser, RESET_cmd);
    kfile_flush(ser);
    return starts_with_P(ser, OK_rsp, 1000L);
}

uint8_t init_hc05(KFile* ser)
{
    DDRD |= BV(BT_COMMAND_PIN);

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

    if (!hc05_soft_reset(ser))
        result |= 64;

    eeprom_write_word((uint16_t*) &bt_initialized, BT_INIT_MAGIC);

    return result;
}
