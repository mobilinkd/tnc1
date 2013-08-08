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
 * Copyright 2010 Develer S.r.l. (http://www.develer.com/)
 * All Rights Reserved.
 * -->
 *
 * \brief AFSK modem hardware-specific definitions.
 *
 *
 * \author Francesco Sacchi <batt@develer.com>
 */


#include "hw_afsk.h"

#include <net/afsk.h>
#include <cpu/irq.h>

#include <avr/io.h>
#include <avr/interrupt.h>

#include <string.h>

#define DC_FILTER_SHIFT 5
#define DC_FILTER_SIZE 32

typedef struct DCFilter
{
    uint8_t buffer[32];
    uint8_t pos;
    uint8_t tail;
    uint8_t count;
} DCFilter;

static DCFilter dc_filter;

INLINE void capture(Afsk* af, int16_t adc)
{
    int16_t avg = 0;
    uint8_t min = 255;
    uint8_t max = 0;

    for (size_t i = 0; i < DC_FILTER_SIZE; ++i)
    {
        const uint8_t tmp = dc_filter.buffer[i];
        avg += tmp;
        min = tmp < min ? tmp : min;
        max = tmp > max ? tmp : max;
    }

    avg >>= DC_FILTER_SHIFT;

    if ((max - min) >= af->squelch_level)
    {
        carrier_on(af);
    }
    else
    {
        carrier_off(af);
    }

    af->input_volume = max - min;

    afsk_adc_isr(af, dc_filter.buffer[dc_filter.pos] - avg);

    dc_filter.buffer[dc_filter.pos++] = (adc >> 2);
    if (dc_filter.pos == DC_FILTER_SIZE) dc_filter.pos = 0;
}

/*
 * Here we are using only one modem. If you need to receive
 * from multiple modems, you need to define an array of contexts.
 */
static Afsk *ctx;

void hw_afsk_adcInit(int ch, Afsk *_ctx)
{
	ctx = _ctx;
	ASSERT(ch <= 5);

	dc_filter.pos = 0;
	dc_filter.tail = DC_FILTER_SIZE;
	dc_filter.count = 0;
	memset(dc_filter.buffer, 0x80, DC_FILTER_SIZE);

	AFSK_STROBE_INIT();
	AFSK_STROBE_OFF();
	/* Set prescaler to clk/8 (2 MHz), CTC, top = ICR1 */
	TCCR1A = 0;
	TCCR1B = BV(CS11) | BV(WGM13) | BV(WGM12);
	/* Set max value to obtain a 9600Hz freq */
	ICR1 = ((CPU_FREQ / 8) / 9600) - 1;

	/* Set reference to AVCC (5V), select CH */
	ADMUX = BV(REFS0) | ch;

	DDRC &= ~BV(ch);
	PORTC &= ~BV(ch);
	DIDR0 |= BV(ch);

	/* Set autotrigger on Timer1 Input capture flag */
	ADCSRB = BV(ADTS2) | BV(ADTS1) | BV(ADTS0);
	/* Enable ADC, autotrigger, 1MHz, IRQ enabled */
	/* We are using the ADC a bit out of specifications otherwise it's not fast enough for our
	 * purposes */
	ADCSRA = BV(ADEN) | BV(ADSC) | BV(ADATE) | BV(ADIE) | BV(ADPS2);
}

bool hw_afsk_dac_isr;

/*
 * This is how you declare an ISR.
 */
DECLARE_ISR(ADC_vect)
{
	TIFR1 = BV(ICF1);
	capture(ctx, ADC);
}

#if CONFIG_AFSK_PWM_TX == 1
DECLARE_ISR(TIMER0_OVF_vect)
{
    OCR0A = afsk_dac_isr(ctx);             // uses timer 0 on port D bit 5
}
#else
DECLARE_ISR(TIMER2_COMPA_vect)
{
    // TCNT2 = DAC_TIMER_VALUE;
    PORTD = ((PORTD & 0x03) | (afsk_dac_isr(ctx) & 0xF0));
}
#endif
