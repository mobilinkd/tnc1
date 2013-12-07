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
 * Copyright 2008 Develer S.r.l. (http://www.develer.com/)
 *
 * -->
 *
 * \brief AFSK1200 modem.
 *
 * \author Francesco Sacchi <batt@develer.com>
 */

#include "afsk.h"
#include <net/ax25.h>

#include "cfg/cfg_afsk.h"
#include "hw/hw_afsk.h"

#include <drv/timer.h>
#include <avr/wdt.h>

#include <cfg/module.h>

#define LOG_LEVEL   AFSK_LOG_LEVEL
#define LOG_FORMAT  AFSK_LOG_FORMAT
#include <cfg/log.h>

#include <cpu/power.h>
#include <cpu/pgm.h>
#include <struct/fifobuf.h>

#include <string.h> /* memset */

#define PHASE_BIT    8
#define PHASE_INC    1

#define PHASE_MAX    (SAMPLEPERBIT * PHASE_BIT)
#define PHASE_THRES  (PHASE_MAX / 2) // - PHASE_BIT / 2)

// Modulator constants
#define MARK_FREQ  1200
#define MARK_INC   (uint16_t)(DIV_ROUND(SIN_LEN * (uint32_t)MARK_FREQ, CONFIG_AFSK_DAC_SAMPLERATE))

#define SPACE_FREQ 2200
#define SPACE_INC  (uint16_t)(DIV_ROUND(SIN_LEN * (uint32_t)SPACE_FREQ, CONFIG_AFSK_DAC_SAMPLERATE))

#define DAC_SAMPLEPERBIT (CONFIG_AFSK_DAC_SAMPLERATE / BITRATE)

/**
 * Sine table for the first quarter of wave.
 * The rest of the wave is computed from this first quarter.
 * This table is used to generate the modulated data.
 */
static const uint8_t PROGMEM sin_table[] =
{
    0,   1,   3,   4,   6,   7,   9,  10,  12,  14,  15,  17,  18,  20,  21,  23,
   24,  26,  27,  29,  30,  32,  34,  35,  37,  38,  39,  41,  42,  44,  45,  47,
   48,  50,  51,  53,  54,  55,  57,  58,  60,  61,  62,  64,  65,  66,  68,  69,
   70,  72,  73,  74,  75,  77,  78,  79,  80,  82,  83,  84,  85,  86,  87,  89,
   90,  91,  92,  93,  94,  95,  96,  97,  98,  99, 100, 101, 102, 103, 104, 105,
  106, 106, 107, 108, 109, 110, 110, 111, 112, 113, 113, 114, 115, 115, 116, 117,
  117, 118, 118, 119, 120, 120, 121, 121, 122, 122, 122, 123, 123, 124, 124, 124,
  125, 125, 125, 125, 126, 126, 126, 126, 126, 127, 127, 127, 127, 127, 127, 127
};

#define SIN_LEN 512 ///< Full wave length

STATIC_ASSERT(sizeof(sin_table) == SIN_LEN / 4);

/**
 * Given the index, this function computes the correct sine sample
 * based only on the first quarter of wave.
 */
INLINE uint8_t sin_sample(uint16_t idx, uint8_t volume)
{
    const uint16_t SIN_TABLE_END = sizeof(sin_table) - 1;

    const uint16_t phase = (idx / sizeof(sin_table)) & 0x03;
    const uint16_t index = idx & SIN_TABLE_END;
    uint8_t sample;
    if (phase & 1)
    {
        sample = pgm_read8(&sin_table[SIN_TABLE_END - index]);
    }
    else
    {
        sample = pgm_read8(&sin_table[index]);
        __asm__ __volatile__ (
            "nop" "\n\t");
    }

    uint16_t tmp = (sample * volume);
    sample = (tmp >> 8);

    return phase & 2 ? 127 - sample : 128 + sample;
}


#define BIT_DIFFER(bitline1, bitline2) (((bitline1) ^ (bitline2)) & 0x01)
#define EDGE_FOUND(bitline)            BIT_DIFFER((bitline), (bitline) >> 1)

INLINE void capture(Afsk* af, int8_t adc)
{
    int8_t min = 127;
    int8_t max = -128;
    int16_t avg = 0;

    for (size_t i = 0; i < DC_FILTER_SIZE; ++i)
    {
        const int8_t tmp = af->dc_filter.buffer[i];
        min = tmp < min ? tmp : min;
        max = tmp > max ? tmp : max;
        avg += tmp;
    }

    avg >>= DC_FILTER_SHIFT;

    af->input_volume = max - min;

    afsk_adc_isr(af, af->dc_filter.buffer[af->dc_filter.pos]);

    AfskDcd_detect(&af->afskDcd, adc, avg);
    if (AfskDcd_dcd(&af->afskDcd))
        carrier_on(af);
    else
        carrier_off(af);

    af->dc_filter.buffer[af->dc_filter.pos++] = adc - avg;
    if (af->dc_filter.pos == DC_FILTER_SIZE) af->dc_filter.pos = 0;
}

// Never called from ISR.
void afsk_rx_bottom_half(Afsk* af)
{
    while (!fifo_isempty_locked(&af->adc_fifo))
    {
        capture(af, (int8_t) fifo_pop_locked(&af->adc_fifo));
    }
}


/**
 * ADC ISR callback.
 * This function has to be called by the ADC ISR when a sample of the configured
 * channel is available.
 * \param af Afsk context to operate on.
 * \param curr_sample current sample from the ADC.
 */
void afsk_adc_isr(Afsk *af, int8_t curr_sample)
{
	uint8_t this_bit = 0;

	if (af->status & 16) return;        // Test/configuration mode.

	if (!carrier_present(af))
	{
	    af->status = hdlc_flush(&af->rx_hdlc, &af->rx_fifo);
	    return;
	}

	/*
	 * Frequency discriminator and LP IIR filter.
	 * This filter is designed to work
	 * at the given sample rate and bit rate.
	 */
	STATIC_ASSERT(SAMPLERATE == 9600);
	STATIC_ASSERT(BITRATE == 1200);

	/*
	 * Frequency discrimination is achieved by simply multiplying
	 * the sample with a delayed sample of (samples per bit) / 2.
	 * Then the signal is lowpass filtered with a first order,
	 * 600 Hz filter. The filter implementation is selectable
	 * through the CONFIG_AFSK_FILTER config variable.
	 */

	af->iir_x[0] = af->iir_x[1];

	#if (CONFIG_AFSK_FILTER == AFSK_BUTTERWORTH)
		af->iir_x[1] = ((int8_t)fifo_pop(&af->delay_fifo) * curr_sample) >> 2;
		//af->iir_x[1] = ((int8_t)fifo_pop(&af->delay_fifo) * curr_sample) / 6.027339492;
	#elif (CONFIG_AFSK_FILTER == AFSK_CHEBYSHEV)
		af->iir_x[1] = ((int8_t)fifo_pop(&af->delay_fifo) * curr_sample) >> 2;
		//af->iir_x[1] = ((int8_t)fifo_pop(&af->delay_fifo) * curr_sample) / 3.558147322;
	#else
		#error Filter type not found!
	#endif

	af->iir_y[0] = af->iir_y[1];

	#if CONFIG_AFSK_FILTER == AFSK_BUTTERWORTH
		/*
		 * This strange sum + shift is an optimization for af->iir_y[0] * 0.668.
		 * iir * 0.668 ~= (iir * 21) / 32 =
		 * = (iir * 16) / 32 + (iir * 4) / 32 + iir / 32 =
		 * = iir / 2 + iir / 8 + iir / 32 =
		 * = iir >> 1 + iir >> 3 + iir >> 5
		 */
		af->iir_y[1] = af->iir_x[0] + af->iir_x[1] + (af->iir_y[0] >> 1) + (af->iir_y[0] >> 3) + (af->iir_y[0] >> 5);
		//af->iir_y[1] = af->iir_x[0] + af->iir_x[1] + af->iir_y[0] * 0.6681786379;
	#elif CONFIG_AFSK_FILTER == AFSK_CHEBYSHEV
		/*
		 * This should be (af->iir_y[0] * 0.438) but
		 * (af->iir_y[0] >> 1) is a faster approximation :-)
		 */
		af->iir_y[1] = af->iir_x[0] + af->iir_x[1] + (af->iir_y[0] >> 1);
		//af->iir_y[1] = af->iir_x[0] + af->iir_x[1] + af->iir_y[0] * 0.4379097269;
	#endif

	/* Save this sampled bit in a delay line */
	af->sampled_bits <<= 1;
	af->sampled_bits |= (af->iir_y[1] > 0) ? 1 : 0;

	/* Store current ADC sample in the af->delay_fifo */
	fifo_push(&af->delay_fifo, curr_sample);

	/* If there is an edge, adjust phase sampling */
	if (EDGE_FOUND(af->sampled_bits))
	{
		if (af->curr_phase < PHASE_THRES)
			af->curr_phase += PHASE_INC;
		else
			af->curr_phase -= PHASE_INC;
	}
	af->curr_phase += PHASE_BIT;

	/* sample the bit */
	if (af->curr_phase >= PHASE_MAX)
	{
		af->curr_phase %= PHASE_MAX;

		/*
		 * Determine bit value by reading the last 3 sampled bits.
		 * If the number of ones is two or greater, the bit value is a 1,
		 * otherwise is a 0.
		 * This algorithm presumes that there are 8 samples per bit.
		 */
		STATIC_ASSERT(SAMPLEPERBIT == 8);
		uint8_t bits = af->sampled_bits & 0x07;
		if (bits == 0x07			  // 111, 3 bits set to 1
			 || bits == 0x06		  // 110, 2 bits
			 || bits == 0x05		  // 101, 2 bits
			 || bits == 0x03		  // 011, 2 bits
			)
			this_bit = 1;

		af->status = hdlc_decode (&af->rx_hdlc, this_bit, &af->rx_fifo);
	}
}

// Never called from ISR.
void afsk_tx_bottom_half(Afsk* af)
{
    int8_t bit;

    while (!fifo_isfull_locked(&af->tx_bit_fifo))
    {
        bit = hdlc_encode(&af->tx_hdlc, &af->tx_fifo);
        if (bit == -1) break;
        fifo_push_locked(&af->tx_bit_fifo, bit);
    }
}

// Only called in ISR context.
INLINE int8_t get_next_bit(Afsk *af)
{
    if (LIKELY((af->status & 16) == 0))
    {
        return fifo_isempty(&af->tx_bit_fifo) ?
            -1 : (int8_t) fifo_pop(&af->tx_bit_fifo);
    }

    switch(af->status)
    {
    case HDLC_TEST_SPACE:
        return 0;
    case HDLC_TEST_MARK:
        return 1;
    default: // case HDLC_TEST_BOTH:
        af->sampled_bits ^= 1;
        return af->sampled_bits;
    }
}

/**
 * DAC ISR callback.
 * This function has to be called by the DAC ISR when a sample of the configured
 * channel has been converted out.
 *
 * \param af Afsk context to operate on.
 *
 * \return The next DAC output sample.
 */
uint8_t afsk_dac_isr(Afsk *af)
{
	/* Check if we are at a start of a sample cycle */
	if (af->sample_count == 0)
	{
		/* We have just finished transmitting a bit, get a new one. */
		/* note that hdlc module does all the NRZI as well as bit stuffing etc */
		af->curr_out = get_next_bit(af);
		af->sample_count = DAC_SAMPLEPERBIT;
		switch (af->curr_out)
		{
		case -1:
			AFSK_DAC_IRQ_STOP (af->dac_ch);
			af->sending = false;
			AFSK_STROBE_OFF ();
			return 0;
		case 1:
			af->phase_inc = MARK_INC;
			break;
		default:
			af->phase_inc = SPACE_INC;
			break;
		}
    }

	/* Get new sample and put it out on the DAC */
	af->phase_acc += af->phase_inc;
	af->phase_acc &= (SIN_LEN - 1);

	af->sample_count--;
	return sin_sample(af->phase_acc, af->output_volume);
}


static size_t afsk_read(KFile *fd, void *_buf, size_t size)
{
	Afsk *af = AFSK_CAST(fd);
	uint8_t *buf = (uint8_t *)_buf;

	afsk_rx_bottom_half(af);

	#if CONFIG_AFSK_RXTIMEOUT == 0
	while (size-- && !fifo_isempty_locked(&af->rx_fifo))
	#else
	while (size--)
	#endif
	{
		#if CONFIG_AFSK_RXTIMEOUT != -1
		ticks_t start = timer_clock();
		#endif

		while (fifo_isempty_locked(&af->rx_fifo))
		{
		    afsk_rx_bottom_half(af);
			cpu_relax();
			#if CONFIG_AFSK_RXTIMEOUT != -1
			if (timer_clock() - start > ms_to_ticks(CONFIG_AFSK_RXTIMEOUT))
				return buf - (uint8_t *)_buf;
			#endif
		}

		*buf++ = fifo_pop_locked(&af->rx_fifo);
	}

	return buf - (uint8_t *)_buf;
}

static size_t afsk_write(KFile *fd, const void *_buf, size_t size)
{
	Afsk *af = AFSK_CAST(fd);
	const uint8_t *buf = (const uint8_t *)_buf;

	while (size--)
	{
		while (fifo_isfull_locked(&af->tx_fifo))
		{
		    afsk_tx_bottom_half(af);
            cpu_relax();
		}

		fifo_push_locked(&af->tx_fifo, *buf++);
        afsk_tx_bottom_half(af);

        if (!af->sending)
        {
            af->phase_inc = MARK_INC;
            af->phase_acc = 0;
            af->sending = true;
            AFSK_DAC_IRQ_START (af);
            AFSK_STROBE_ON();
        }
	}

	return buf - (const uint8_t *)_buf;
}

static int afsk_flush(KFile *fd)
{
	Afsk *af = AFSK_CAST(fd);
	while (af->sending)
	{
        afsk_tx_bottom_half(af);
        cpu_relax();
	}
	return 0;
}

static int afsk_error(KFile *fd)
{
	Afsk *af = AFSK_CAST(fd);
	int err;

	ATOMIC(err = af->status);
	return err;
}

static void afsk_clearerr (KFile * fd)
{
	Afsk *af = AFSK_CAST (fd);
	ATOMIC (af->status = 0);
}

/**
 * Sets head timings by defining the number of flags to output
 * Has to be done here as this is the only module that interfaces directly to hdlc!!
 *
 * \param fd caste afsk context.
 * \param c value
 *
 */
void afsk_head (KFile * fd, int c)
{
	Afsk *af = AFSK_CAST (fd);

	hdlc_head (&af->tx_hdlc, c);
}

/**
 * Sets tail timings by defining the number of flags to output
 * Has to be done here as this is the only module that interfaces directly to hdlc!!
 *
 * \param fd caste afsk context.
 * \param c value
 *
 */
void afsk_tail (KFile * fd, int c)
{
	Afsk *af = AFSK_CAST (fd);

	hdlc_tail (&af->tx_hdlc, c);
}

void afsk_test_tx_start(KFile *fd, int8_t hdlc_status)
{
    Afsk *af = AFSK_CAST(fd);

    ATOMIC(
        af->status = hdlc_status;
        af->sampled_bits = 0;       // reuse this since RX disabled while testing.
        afsk_tx_bottom_half(af);

        if (!af->sending)
        {
            af->sample_count = 0;
            af->phase_acc = 0;
            af->sending = true;
            AFSK_DAC_IRQ_START (af);
            AFSK_STROBE_ON();
        }
    );
}

void afsk_test_tx_end(KFile *fd)
{
    Afsk *af = AFSK_CAST(fd);
    af->status = HDLC_ERROR_NONE;
    af->sampled_bits = 0;
}

INLINE void DCFilter_init(DCFilter* dc_filter)
{
    dc_filter->pos = 0;
    memset(dc_filter->buffer, 0, sizeof(dc_filter->buffer));
}

/**
 * Initialize an AFSK1200 modem.
 * \param af Afsk context to operate on.
 * \param adc_ch  ADC channel used by the demodulator.
 * \param dac_ch  DAC channel used by the modulator.
 */
void afsk_init(Afsk *af, int adc_ch, int dac_ch)
{
	#if CONFIG_AFSK_RXTIMEOUT != -1
	MOD_CHECK(timer);
	#endif

	memset(af, 0, sizeof(*af));
	af->adc_ch = adc_ch;
	af->dac_ch = dac_ch;

	fifo_init(&af->delay_fifo, (uint8_t *)af->delay_buf, sizeof(af->delay_buf));
	fifo_init(&af->rx_fifo, af->rx_buf, sizeof(af->rx_buf));
    fifo_init(&af->adc_fifo, (uint8_t *)af->adc_buffer, sizeof(af->adc_buffer));
    DCFilter_init(&af->dc_filter);

	/* Fill sample FIFO with 0 */
	for (int i = 0; i < SAMPLEPERBIT / 2; i++)
		fifo_push(&af->delay_fifo, 0);

	fifo_init(&af->tx_fifo, af->tx_buf, sizeof(af->tx_buf));
    fifo_init(&af->tx_bit_fifo, af->tx_bit_buf, sizeof(af->tx_bit_buf));

	AFSK_ADC_INIT(adc_ch, af);
	AFSK_DAC_INIT(dac_ch, af);
	AFSK_STROBE_INIT();
	LOG_INFO("MARK_INC %d, SPACE_INC %d\n", MARK_INC, SPACE_INC);

	hdlc_init (&af->rx_hdlc);
	hdlc_init (&af->tx_hdlc);
	// set initial defaults for timings
	hdlc_head (&af->tx_hdlc, DIV_ROUND (CONFIG_AFSK_PREAMBLE_LEN * BITRATE, 8000));
	hdlc_tail (&af->tx_hdlc, DIV_ROUND (CONFIG_AFSK_TRAILER_LEN * BITRATE, 8000));
	DB (af->fd._type = KFT_AFSK);
	af->fd.write = afsk_write;
	af->fd.read = afsk_read;
	af->fd.flush = afsk_flush;
	af->fd.error = afsk_error;
	af->fd.clearerr = afsk_clearerr;
	af->phase_inc = MARK_INC;

	AfskDcd_init(&af->afskDcd);
}
