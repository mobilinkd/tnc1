// Copyright 2013 Mobilinkd LLC <info@mobilinkd.com>
// All rights reserved.

#include <cfg/macros.h>

#include <cpu/irq.h>

#include <avr/interrupt.h>
#include <avr/io.h>

#include "battery.h"

uint16_t check_battery()
{
    uint32_t adc = 0;

    // Disable interrupts.
    cli();

    // Disable digital input
    DDRC &= ~BV(1);
    PORTC &= ~BV(1);
    DIDR0 |= BV(1);

    // Save current ADC state.
    uint8_t adcsra = ADCSRA;
    uint8_t adcsrb = ADCSRB;
    uint8_t admux = ADMUX;
    // Set prescaler to 128 and enable ADC.
    ADCSRA = (BV(ADPS2) | BV(ADPS1) | BV(ADPS0) | BV(ADEN));
    // Put the ADC into free-running mode.
    ADCSRB &= ~(BV(ADTS2) | BV(ADTS1) | BV(ADTS0));
    // Set ADC to check ADC1
    // Set result type to ADLAR
    // Set reference to AVCC (3.3V)
    ADMUX = (BV(REFS0) | BV(ADLAR) | 1);
    // Enable interrupts.
    sei();
    // Read ADC until it stabilizes.
    uint32_t prev = 0;
    uint8_t count = 0;
    do {
        prev = adc;
        // Start ADC one-shot.
        ADCSRA |= BV(ADSC);
        // Busy wait for result and ignore the first result.
        while (ADCSRA & BV(ADSC));
        adc = ADC;
    } while ((prev != adc) && (++count != 10));
    // Disable interrupts.
    cli();
    // Restore ADC state.
    ADMUX = admux;
    ADCSRA = adcsra;
    ADCSRB = adcsrb;
    // Enable interrupts.
    sei();

    adc *= 100;
    uint16_t result = adc / 976;
    return result;
}
