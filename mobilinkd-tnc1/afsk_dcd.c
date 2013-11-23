// Copyright 2013 Mobilinkd LLC <info@mobilinkd.com>
// All rights reserved.

#include "afsk_dcd.h"

#include <string.h>

void AfskDcd_init(AfskDcd* this)
{
    memset(this, 0, sizeof(AfskDcd));
    this->short_limit = DEFAULT_SHORT_LIMIT;
    this->long_limit = DEFAULT_LONG_LIMIT;
    this->decay_count = DEFAULT_DECAY_COUNT;
}

/**
 * Check the number of bits set in the dcd_buffer.  If it is greater
 * than the
 */
INLINE uint8_t AfskDcd_check(const AfskDcd* this)
{
    uint8_t count = 0;
    uint8_t tmp = this->dcd_buffer;

    while (tmp)
    {
        if (tmp & 0x80) ++count;
        tmp <<= 1;
    }

    return count > this->long_limit ? 0 : 1;
}

void AfskDcd_detect(AfskDcd* this, int8_t adc, int8_t avg)
{
    this->sample_counter += 1;
    this->sample_buffer <<= 1;
    this->sample_buffer |= adc > avg;

    if (this->sample_counter != 31) return;
    this->sample_counter = 0;

    // Use shifted XOR to detect 0 crossing.
    uint32_t zero_crossing = (this->sample_buffer << 1);
    zero_crossing ^= this->sample_buffer;

    uint8_t count = 1;
    uint8_t error = 0;
    for (size_t i = 1; i < (sizeof(zero_crossing) * 8); ++i)
    {
        if (zero_crossing & 0x80000000)
        {
            if ((count == 0) || (count > 3)) ++error;
            count = 0;
        }
        else
        {
            ++count;
        }
        zero_crossing <<= 1;
    }
    if (count > 4) ++error;

    this->dcd_buffer <<= 1;
    if ((error > this->short_limit) || (this->sample_buffer == 0))
    {
        this->dcd_buffer |= 1;
    }

    if (AfskDcd_check(this))
    {
        this->dcd_decay = this->decay_count;
    }
    else if (this->dcd_decay != 0)
    {
        this->dcd_decay -= 1;
    }

    this->sample_buffer = 0;
}
