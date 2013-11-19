// Copyright 2013 Rob Riggs <rob@pangalactic.org>
// All rights reserved.

#include <avr/wdt.h>
#include <avr/interrupt.h>

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

void mobilinkd_set_error(uint8_t error_code)
{
    mobilinkd_error_code = error_code;
}

uint8_t mobilinkd_get_error()
{
    return mobilinkd_error_code;
}

const char* mobilinkd_strerror(uint8_t error_code)
{
    switch (error_code)
    {
    case MOBILINKD_ERROR_NONE:
        return "Normal startup";
    case MOBILINKD_ERROR_AFSK_ADC_OVERFLOW:
        return "AFSK ADC buffer overflow";
    case MOBILINKD_ERROR_WATCHDOG_TIMEOUT:
        return "WDT timed out";
    default:
        return "Unknown error";
    }
}
