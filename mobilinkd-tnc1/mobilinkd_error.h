// Copyright 2013 Mobilinkd LLC <info@mobilinkd.com>
// All rights reserved.

#ifndef MOBILINKD_ERROR_H_
#define MOBILINKD_ERROR_H_

void mobilinkd_abort(uint8_t error_code);
void mobilinkd_set_error(uint8_t error_code);
uint8_t mobilinkd_get_error(void);
const char* mobilinkd_strerror(uint8_t error_code);

#define MOBILINKD_ERROR_NONE 0
#define MOBILINKD_ERROR_AFSK_ADC_OVERFLOW 1
#define MOBILINKD_ERROR_WATCHDOG_TIMEOUT 2

#endif // MOBILINKD_ERROR_H_
