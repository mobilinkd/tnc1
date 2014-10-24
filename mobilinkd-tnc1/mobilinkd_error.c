// Copyright 2013 Mobilinkd LLC <info@mobilinkd.com>
// All rights reserved.

#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include "mobilinkd_error.h"

volatile uint8_t mobilinkd_error_code __attribute__ ((section (".noinit")));

static void __attribute__ ((noreturn)) reboot(void)
{
    cli();
    wdt_disable();
    wdt_enable(WDTO_15MS);
    while (1) {}
}

void  __attribute__ ((noreturn)) mobilinkd_abort(uint8_t error_code)
{
    mobilinkd_set_error(error_code);
    reboot();
}

const char MOBILINKD_ERROR_NONE_STR[] PROGMEM = "No error";
const char MOBILINKD_ERROR_AFSK_ADC_OVERFLOW_STR[] PROGMEM = "AFSK ADC buffer overflow";
const char MOBILINKD_ERROR_WATCHDOG_TIMEOUT_STR[] PROGMEM = "WDT timed out";
const char MOBILINKD_ERROR_RX_BUFFER_OVERFLOW_STR[] PROGMEM = "RX buffer overflow";
const char MOBILINKD_ERROR_RX_CRC_STR[] PROGMEM = "Packet CRC error";
const char MOBILINKD_ERROR_RX_ABORT_STR[] PROGMEM = "RX packet abort";
const char MOBILINKD_ERROR_SERIAL_RX_TIMEOUT_STR[] PROGMEM = "Serial receive timeout";
const char MOBILINKD_ERROR_SERIAL_RX_OVERRUN_STR[] PROGMEM = "Serial buffer overrun";
const char MOBILINKD_ERROR_SERIAL_RX_ERROR_STR[] PROGMEM = "Unknown error";
const char MOBILINKD_ERROR_UNKNOWN_STR[] PROGMEM = "Unknown error";

const char* mobilinkd_strerror(uint8_t error_code)
{
    switch (error_code)
    {
    case MOBILINKD_ERROR_NONE:
        return MOBILINKD_ERROR_NONE_STR;
    case MOBILINKD_ERROR_AFSK_ADC_OVERFLOW:
        return MOBILINKD_ERROR_AFSK_ADC_OVERFLOW_STR;
    case MOBILINKD_ERROR_WATCHDOG_TIMEOUT:
        return MOBILINKD_ERROR_WATCHDOG_TIMEOUT_STR;
    case MOBILINKD_ERROR_RX_BUFFER_OVERFLOW:
        return MOBILINKD_ERROR_RX_BUFFER_OVERFLOW_STR;
    case MOBILINKD_ERROR_RX_CRC:
        return MOBILINKD_ERROR_RX_CRC_STR;
    case MOBILINKD_ERROR_RX_ABORT:
        return MOBILINKD_ERROR_RX_ABORT_STR;
    case MOBILINKD_ERROR_SERIAL_RX_TIMEOUT:
        return MOBILINKD_ERROR_SERIAL_RX_TIMEOUT_STR;
    case MOBILINKD_ERROR_SERIAL_RX_OVERRUN:
        return MOBILINKD_ERROR_SERIAL_RX_OVERRUN_STR;
    case MOBILINKD_ERROR_SERIAL_RX_ERROR:
        return MOBILINKD_ERROR_SERIAL_RX_ERROR_STR;
    default:
        return MOBILINKD_ERROR_UNKNOWN_STR;
    }
}
