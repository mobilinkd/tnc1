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
#include "mobilinkd_error.h"

#include <net/afsk.h>
#include <cpu/irq.h>

#include <avr/io.h>
#include <avr/interrupt.h>

#include <string.h>

/**
 * Takes a normalized ADC (-512/+511) value and computes the attenuated
 * value, clamps the input within [-128/+127] and returns the result.
 */
INLINE int8_t input_attenuation(Afsk* af, int16_t adc)
{
    int16_t result = (adc >> af->input_volume_gain);

    if (result < -128) result = -128;
    else if (result > 127) result = 127;

    return result;
}

/*
 * Here we are using only one modem. If you need to receive
 * from multiple modems, you need to define an array of contexts.
 */
static Afsk *ctx;

/**
 * Initialize the ADC.  The ADC runs at its slowest rate (125KHz) to
 * ensure best resolution possible.
 *
 * - The AVCC (3.3V) voltage reference is used.
 * - The prescaler is set to 125KHz, and set to free-running mode, providing
 *   almost exactly 9600 conversions per second.
 * - The signal source is set to the given ADC channel.
 * - The ADC channel is set to input with digital input disabled.
 * - The ADC interrupt is enabled.
 *
 * @param ch
 * @param _ctx
 */
void hw_afsk_adcInit(int ch, Afsk *_ctx)
{
	ctx = _ctx;
	ASSERT(ch <= 5);

	AFSK_STROBE_INIT();
	AFSK_STROBE_OFF();

    /* Set reference to AVCC (3.3V) and select ADC channel. */
    ADMUX = BV(REFS0) | ch;

    DDRC &= ~BV(ch);
    PORTC &= ~BV(ch);
    DIDR0 |= BV(ch);

    // Set prescaler to 128 so we have a 125KHz clock source.
    // This provides almost exactly 9600 conversions a second.
    ADCSRA = (BV(ADPS2) | BV(ADPS1) | BV(ADPS0));

    // Put the ADC into free-running mode.
    ADCSRB &= ~(BV(ADTS2) | BV(ADTS1) | BV(ADTS0));

    // Set signal source to free running, start the ADC, start
    // conversion and enable interrupt.
    ADCSRA |= (BV(ADATE) | BV(ADEN) | BV(ADSC) | BV(ADIE));
}

bool hw_afsk_dac_isr;

/**
 * The ADC interrupt.  This is called 9600 times a second while input
 * capture is enabled.  The data is pulled from the ADC, attenuated,
 * and the attenuated value is placed on the ADC FIFO.  This will then
 * be processed by the bottom-half handler outside the interrupt
 * context.
 *
 * If the ADC FIFO overflows, the error is recorded and the TNC is reset.
 */
DECLARE_ISR(ADC_vect)
{
    if (fifo_isfull(&ctx->adc_fifo))
    {
        return; // Happens during init and during serial IO.
        // mobilinkd_abort(MOBILINKD_ERROR_AFSK_ADC_OVERFLOW);
    }

    fifo_push(&ctx->adc_fifo, (uint8_t) input_attenuation(ctx, ADC - 512));
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
