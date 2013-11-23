// Copyright 2013 Mobilinkd LLC <info@mobilinkd.com>
// All rights reserved.

#include <cfg/macros.h>

#include <cpu/irq.h>

#include <avr/interrupt.h>
#include <avr/io.h>

#include "battery.h"

uint16_t check_battery()
{
    uint16_t result = 0;

    // Disable interrupts.
    cli();
    // Save current ADC state.
    uint8_t ADCSRA_save = ADCSRA;
    uint8_t ADCSRB_save = ADCSRB;
    uint8_t ADMUX_save = ADMUX;
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
    // Start ADC one-shot.
    ADCSRA |= BV(ADSC);
    // Busy wait for result.
    while (ADCSRA & BV(ADSC));
    result = ADC;
    // Disable interrupts.
    cli();
    // Restore ADC state.
    ADMUX = ADMUX_save;
    ADCSRA = ADCSRA_save;
    ADCSRB = ADCSRB_save;
    // Enable interrupts.
    sei();

    return result;
}
