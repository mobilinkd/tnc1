// Copyright 2013 Mobilinkd LLC <info@mobilinkd.com>
// All rights reserved.

#include "config.h"

#include <cfg/macros.h>

#include <cpu/irq.h>

#include <avr/interrupt.h>
#include <avr/io.h>

#include "battery.h"

uint16_t check_battery()
{
    uint32_t adc = 0;

    // Enable battery divider
    BATTERY_DIVIDER_DDR |= BV(BATTERY_DIVIDER_PIN);   // output mode
    BATTERY_DIVIDER_PORT &= ~BV(BATTERY_DIVIDER_PIN); // low

    // Disable interrupts.
    cli();

    // Disable digital input
    BATTERY_LEVEL_DDR &= ~BV(BATTERY_LEVEL_PIN);
    BATTERY_LEVEL_PORT &= ~BV(BATTERY_LEVEL_PIN);
    DIDR0 |= BV(BATTERY_LEVEL_PIN);

    // Save current ADC state.
    uint8_t adcsra = ADCSRA;
    uint8_t adcsrb = ADCSRB;
    uint8_t admux = ADMUX;
    // Set prescaler to 128 and enable ADC.
    ADCSRA = (BV(ADPS2) | BV(ADPS1) | BV(ADPS0) | BV(ADEN));
    // Put the ADC into free-running mode.
    ADCSRB &= ~(BV(ADTS2) | BV(ADTS1) | BV(ADTS0));
    // Set ADC to check battery pin
    // Set result type to ADLAR
    // Set reference to AVCC (3.3V)
    ADMUX = (BV(REFS0) | BV(ADLAR) | BATTERY_LEVEL_PIN);
    // Enable interrupts.
    sei();
    // Read ADC until it stabilizes.
    uint32_t prev = 66635;
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

    // Disable battery divider
    BATTERY_DIVIDER_DDR &= ~BV(BATTERY_DIVIDER_PIN);  // input mode; high impedance

    adc *= 100;
    uint16_t result = adc / 976;
    return result;
}
