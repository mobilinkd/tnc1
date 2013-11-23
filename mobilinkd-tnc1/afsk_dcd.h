// Copyright 2013 Mobilinkd LLC <info@mobilinkd.com>
// All rights reserved.

#ifndef AFSK_DCD_H_
#define AFSK_DCD_H_

#include <cfg/compiler.h>

#include <stdint.h>

#define DEFAULT_SHORT_LIMIT     2           // More than this and DCD is off.
#define DEFAULT_LONG_LIMIT      6           // More than this and DCD is off.
#define DEFAULT_DECAY_COUNT     12          // About 50ms.

/**
 * AFSK data carrier detection.
 *
 * A good carrier will go through a zero-crossing event every 2 to 4
 * bits (1200Hz - 2200Hz).  There will be noise.  However, lack of carrier
 * is easily detected because the number of zero-crossing events outside
 * the valid range goes up dramatically.  (In other words, when the carrier
 * is present, the signal to noise ratio (SNR) goes up.)
 *
 * This measures the noise and records an error for every noise event.
 * This is done over a 4ms (32-bit) range (the short range).  Then the
 * error rate over a 32ms period (the long range) determines whether DCD
 * is present.  If DCD is lost (the error rate is too high) over this
 * 32ms range, a DCD decay counter starts. If DCD is not re-acquired in
 * this period, then DCD is dropped.
 *
 * The number of errors over the short period and long period, as well
 * as the decay count are tunable.  However, the defaults are pretty
 * sensible and seem to work well.
 *
 * Each sample is stored in a one-bit buffer.  Every 31 bits, the error
 * rate is checked and stored in the 32ms dcd_buffer.  If the error rate
 * is within the limits, the DCD decay counter is set and DCD is on;
 * otherwise the decay counter is decremented until it reaches 0, at which
 * point DCD is off.
 */
typedef struct AfskDcd
{
    uint8_t dcd_buffer;
    uint8_t dcd_decay;
    uint8_t sample_counter;
    uint8_t short_limit;
    uint8_t long_limit;
    uint8_t decay_count;
    uint32_t sample_buffer;
} AfskDcd;                      // 10 bytes.

/**
 * Initialize the AfskDcd object.
 */
void AfskDcd_init(AfskDcd* this);

/**
 * Return true if a carrier is detected, otherwise false.
 */
INLINE uint8_t AfskDcd_dcd(const AfskDcd* this)
{
    return this->dcd_decay != 0;
}

/**
 * Use the sample to detect the carrier.
 */
void AfskDcd_detect(AfskDcd* this, int8_t adc, int8_t avg);

INLINE void AfskDcd_set_short_limit(AfskDcd* this, uint8_t value)
{
    this->short_limit = value;
}

INLINE void AfskDcd_set_long_limit(AfskDcd* this, uint8_t value)
{
    this->long_limit = value;
}

INLINE void AfskDcd_set_delay_count(AfskDcd* this, uint8_t value)
{
    this->decay_count = value;
}

#endif // AFSK_DCD_H_
