// Copyright 2013 Mobilinkd LLC <info@mobilinkd.com>
// All rights reserved.

#ifndef MOBILINKD_HC_05_H_
#define MOBILINKD_HC_05_H_

#include <cfg/debug.h>
#include <io/kfile.h>

uint8_t init_hc05(KFile* ser);

bool hc05_connected(void);
void hc05_power_on(void);
void hc05_power_off(void);

#endif // MOBILINKD_HC_05_H_
