// Copyright 2013 Mobilinkd LLC <info@mobilinkd.com>
// All rights reserved.

#ifndef MOBILINKD_ERROR_H_
#define MOBILINKD_ERROR_H_

#include <cfg/compiler.h>

extern volatile uint8_t mobilinkd_error_code;

void mobilinkd_abort(uint8_t error_code);
const char* mobilinkd_strerror(uint8_t error_code);

#define MOBILINKD_ERROR_NONE 0
#define MOBILINKD_ERROR_AFSK_ADC_OVERFLOW 1
#define MOBILINKD_ERROR_WATCHDOG_TIMEOUT 2
#define MOBILINKD_ERROR_RX_BUFFER_OVERFLOW 3
#define MOBILINKD_ERROR_RX_CRC 4
#define MOBILINKD_ERROR_RX_ABORT 5
#define MOBILINKD_ERROR_SERIAL_RX_TIMEOUT 6

INLINE void mobilinkd_set_error(uint8_t error_code)
{
    mobilinkd_error_code = error_code;
}

INLINE uint8_t mobilinkd_get_error(void)
{
    return mobilinkd_error_code;
}

#endif // MOBILINKD_ERROR_H_
