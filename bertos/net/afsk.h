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
 *
 * $WIZ$ module_name = "afsk"
 * $WIZ$ module_configuration = "bertos/cfg/cfg_afsk.h"
 * $WIZ$ module_depends = "timer", "kfile", "hdlc"
 * $WIZ$ module_hw = "bertos/hw/hw_afsk.h"
 */

#ifndef NET_AFSK_H
#define NET_AFSK_H

#include "cfg/cfg_afsk.h"
#include "hw/hw_afsk.h"
#include "hdlc.h"

#include <cfg/compiler.h>

#include <io/kfile.h>

#include <struct/fifobuf.h>



/**
 * ADC sample rate.
 * The demodulator filters are designed to work at this frequency.
 * If you need to change this remember to update afsk_adc_isr().
 */
#define SAMPLERATE 9600

/**
 * Bitrate of the received/transmitted data.
 * The demodulator filters and decoderes are designed to work at this frequency.
 * If you need to change this remember to update afsk_adc_isr().
 */
#define BITRATE    1200

#define SAMPLEPERBIT (SAMPLERATE / BITRATE)

/**
 * RX FIFO buffer full error.
 */
#define AFSK_RXFIFO_OVERRUN BV(0)

/**
 * AFSK1200 modem context.
 */
typedef struct Afsk
{
	/** Base "class" */
	KFile fd;

	/** ADC channel to be used by the demodulator */
	int adc_ch;

	/** DAC channel to be used by the modulator */
	int dac_ch;

	/** Current sample of bit for output data. */
	uint8_t sample_count;

	/** Current character to be modulated */
	int8_t curr_out;

	/**
	 * DDS phase accumulator for generating modulated data.
	 */
	uint16_t phase_acc;

	/** Current phase increment for current modulated bit */
	uint16_t phase_inc;

	/** Delay line used to delay samples by (SAMPLEPERBIT / 2) */
	FIFOBuffer delay_fifo;

	/**
	 * Buffer for delay FIFO.
	 * The 1 is added because the FIFO macros need
	 * 1 byte more to handle a buffer (SAMPLEPERBIT / 2) bytes long.
	 */
	int8_t delay_buf[SAMPLEPERBIT / 2 + 1];

	/** FIFO for received data */
	FIFOBuffer rx_fifo;

	/** FIFO rx buffer */
	uint8_t rx_buf[CONFIG_AFSK_RX_BUFLEN];

	/** FIFO for transmitted data */
	FIFOBuffer tx_fifo;

	/** FIFO tx buffer */
	uint8_t tx_buf[CONFIG_AFSK_TX_BUFLEN];

	FIFOBuffer tx_bit_fifo;
    uint8_t tx_bit_buf[16];

	/** IIR filter X cells, used to filter sampled data by the demodulator */
	int16_t iir_x[2];

	/** IIR filter Y cells, used to filter sampled data by the demodulator */
	int16_t iir_y[2];

	/**
	 * Bits sampled by the demodulator are here.
	 * Since ADC samplerate is higher than the bitrate, the bits here are
	 * SAMPLEPERBIT times the bitrate.
	 */
	uint8_t sampled_bits;

	/**
	 * Current phase, needed to know when the bitstream at ADC speed
	 * should be sampled.
	 */
	int8_t curr_phase;

		/** True while modem sends data */
	volatile bool sending;

	/**
	 * AFSK modem status.
	 * If 0 all is ok, otherwise errors are present.
	 */
	volatile int status;

		/** Hdlc context */
	Hdlc tx_hdlc;
	Hdlc rx_hdlc;

	volatile uint8_t carrier;
	volatile uint8_t input_volume;
	uint8_t input_volume_gain;
	uint8_t output_volume;
	uint8_t squelch_level;

} Afsk;

#define KFT_AFSK MAKE_ID('A', 'F', 'S', 'K')

INLINE Afsk *AFSK_CAST(KFile *fd)
{
  ASSERT(fd->_type == KFT_AFSK);
  return (Afsk *)fd;
}

INLINE void carrier_on(Afsk* af)
{
    af->carrier = 1;
    AFSK_STROBE_ON();
}

INLINE void carrier_off(Afsk* af)
{
    af->carrier = 0;
    AFSK_STROBE_OFF();
}

INLINE uint8_t carrier_present(const Afsk* af) { return af->carrier; }

/**
 * Set the output volume.  The output volume is a range from 0-255, with
 * 255 being the highest.
 *
 * @note Setting a very low output volume may result in poor audio
 *  quality due to the reduction in the accuracy of the waveform.
 */
INLINE void set_output_volume(KFile* fd, uint8_t volume)
{
    Afsk* afsk = AFSK_CAST(fd);

    afsk->output_volume = volume;
    AFSK_SET_OUTPUT_VOLUME(afsk);
}

INLINE void set_input_volume(KFile* fd, uint8_t level)
{
    Afsk* afsk = AFSK_CAST(fd);
    afsk->input_volume_gain = level;
}

INLINE uint8_t get_input_volume(KFile* fd)
{
    Afsk* afsk = AFSK_CAST(fd);
    return afsk->input_volume;
}

INLINE void set_squelch_level(KFile* fd, uint8_t level)
{
    Afsk* afsk = AFSK_CAST(fd);
    afsk->squelch_level = level;
}

/// Can be called multiple times before tx_end().
void afsk_test_tx_start(KFile *fd, int8_t hdlc_status);
void afsk_test_tx_end(KFile *fd);


void afsk_head (KFile * fd, int c);
void afsk_tail (KFile * fd, int c);

void afsk_adc_isr (Afsk * af, int8_t sample);
uint8_t afsk_dac_isr (Afsk * af);
void afsk_set_timings (Afsk * af, uint8_t txhead, uint8_t txtail);
void afsk_init (Afsk * af, int adc_ch, int dac_ch);

void afsk_tx_bottom_half(Afsk* af);

#define HEAD   1
#define TAIL   4


/**
 * \name Afsk filter types.
 * $WIZ$ afsk_filter_list = "AFSK_BUTTERWORTH", "AFSK_CHEBYSHEV"
 * \{
 */
#define AFSK_BUTTERWORTH  0
#define AFSK_CHEBYSHEV    1
/* \} */

int afsk_testSetup(void);
int afsk_testRun(void);
int afsk_testTearDown(void);

#endif /* NET_AFSK_H */
