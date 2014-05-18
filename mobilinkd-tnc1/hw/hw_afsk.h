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
 * Copyright 2006 Develer S.r.l. (http://www.develer.com/)
 * All Rights Reserved.
 * -->
 *
 * \brief AFSK modem hardware-specific definitions.
 *
 *
 * \author Francesco Sacchi <batt@develer.com>
 */

#ifndef HW_AFSK_H
#define HW_AFSK_H

#include "cfg/cfg_arch.h"
#include <cfg/cfg_afsk.h>
#include <cfg/macros.h>

#include <avr/io.h>

struct Afsk;
void hw_afsk_adcInit(int ch, struct Afsk *_ctx);
void hw_afsk_dacInit(int ch, struct Afsk *_ctx);

/**
 * Initialize the specified channel of the ADC for AFSK needs.
 * The adc should be configured to have a continuos stream of convertions.
 * For every convertion there must be an ISR that read the sample
 * and call afsk_adc_isr(), passing the context and the sample.
 *
 * \param ch channel to be used for AFSK demodulation.
 * \param ctx AFSK context (\see Afsk). This parameter must be saved and
 *            passed back to afsk_adc_isr() for every convertion.
 */

/*
 * This macro will be called for AFSK initialization. We could implement everything here as a macro,
 * but since initialization is rather complicated we decided to split its own function. Such function
 * is defined in hw_afsk.c.
 * Remember: since this .c file is not created by the wizard, you must add it to your_project_name.mk.
 * If you create the file using BeRTOS SDK, it will be added for you.
 */
#define AFSK_ADC_INIT(ch, ctx) hw_afsk_adcInit(ch, ctx)

/*
 * Activate strobe pin. We use it for debugging purposes. If you don't use it, simply
 * leave empty the following macros
 */
#define AFSK_STROBE_INIT() do {} while (0)

/*
 * Set the pin high. This macro is called at the beginning of the interrupt routine
 */
#define AFSK_STROBE_ON()   do {} while (0)

/*
 * Set the pin low. This macro is called at the end of the interrupt routine
 */
#define AFSK_STROBE_OFF()  do {} while (0)

#define DAC_TIMER_VALUE (DIV_ROUND((CPU_FREQ / 8), CONFIG_AFSK_DAC_SAMPLERATE))

/**
 * Initialize the specified channel of the DAC for AFSK needs.
 * The DAC has to be configured in order to call an ISR for every sample sent.
 * The DAC doesn't have to start the IRQ immediatly but have to wait
 * the AFSK driver to call AFSK_DAC_IRQ_START().
 * The ISR must then call afsk_dac_isr() passing the AFSK context.
 * \param ch DAC channel to be used for AFSK modulation.
 * \param ctx AFSK context (\see Afsk).  This parameter must be saved and
 *             passed back to afsk_dac_isr() for every convertion.
 */
#if CONFIG_AFSK_PWM_TX == 1
// If using a PWM output then use mode 3 fast (asymmetric) mode running as fast as possible
// on port D bit 3 (Arduino D3) thus keeping PTT on the original port B bit 3 (Arduino D11).
#define AFSK_DAC_INIT(ch, ctx)\
	do { \
		(void)ch, (void)ctx;\
			TCCR0A = BV(COM0A1) | BV(WGM01) | BV(WGM00); \
			TCCR0B = BV(CS00); \
			DDRB |= BV(2);\
			TIMSK0 = BV(TOIE0); \
			DDRD &= ~BV(4); PORTD &= ~BV(4); \
			DDRD &= ~BV(5); PORTD &= ~BV(5); \
            DDRD |= BV(6); \
			DDRD &= ~BV(7); PORTD &= ~BV(7); \
		} while (0)
#else
#define AFSK_DAC_INIT(ch, ctx)\
    do { \
        TCCR2A = BV(WGM21); \
        TCCR2B = BV(CS11); \
        ASSR &= ~BV(AS2); \
        OCR2A = DAC_TIMER_VALUE; \
        TIMSK2 = BV(OCIE2A); \
        (void)ch, (void)ctx; \
        DDRD |= 0xF0; \
        DDRB |= BV(2); \
    } while (0)
#endif

#if CONFIG_AFSK_PWM_TX == 0
/**
 * Start DAC convertions on channel \a ch.
 * \param ch DAC channel.
 */
#define AFSK_DAC_IRQ_START(af)\
    do { \
        extern bool hw_afsk_dac_isr; \
        PORTB |= BV(2); \
        hw_afsk_dac_isr = true; \
        TCCR1B = 0; \
        TCNT2 = DAC_TIMER_VALUE; \
        TCCR2B = BV(CS11); \
    } while (0)
#else
/**
 * Start DAC conversions.
 *
 * PWM is on D5.
 * PTT is on B2.
 *
 * Turn on PTT.
 * Turn off ADC interrupt.
 * Turn on PWM interrupt. Timer uses no prescaling.
 */
#define AFSK_DAC_IRQ_START(af)\
    do { \
        extern bool hw_afsk_dac_isr; \
        PORTB |= BV(2); \
        hw_afsk_dac_isr = true; \
        ADCSRA &= ~(BV(ADATE) | BV(ADEN) | BV(ADSC) | BV(ADIE)); \
        TCCR0B = BV(CS00); \
    } while (0)
#endif

#if CONFIG_AFSK_PWM_TX == 0
/**
 * Stop DAC conversions on channel \a ch.
 * \param ch DAC channel.
 */
#define AFSK_DAC_IRQ_STOP(ch)\
    do { \
        (void)ch; \
        extern bool hw_afsk_dac_isr; \
        PORTB &= ~BV(2); PORTD &= 3; \
        hw_afsk_dac_isr = false; \
        TCCR1B = BV(CS11) | BV(WGM13) | BV(WGM12); \
        TCCR2B = 0; \
    } while (0)
#else // PWM output
/**
 * Stop DAC conversions on \a ch.
 *
 * Turn off PTT.
 * Turn off DAC interrupt.
 * Turn on ADC interrupt.
 *
 */
#define AFSK_DAC_IRQ_STOP(ch)\
    do { \
        (void)ch; \
        extern bool hw_afsk_dac_isr; \
        PORTB &= ~BV(2); \
        hw_afsk_dac_isr = false; \
        TCCR0B = 0; \
        ADCSRA |= (BV(ADATE) | BV(ADEN) | BV(ADSC) | BV(ADIE)); \
    } while (0)
#endif

#define AFSK_SET_OUTPUT_VOLUME(af) do { (void)af; } while (0)

#endif /* HW_AFSK_H */
