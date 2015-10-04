// Copyright 2014 Mobilinkd LLC <info@mobilinkd.com>
// All rights reserved.

#include "power.h"
#include "config.h"
#include "hc-05.h"
#include "mobilinkd_error.h"
#include "mobilinkd_version.h"
#include "mobilinkd_util.h"
#include "mobilinkd_eeprom.h"
#include "battery.h"

#include <drv/ser.h>
#include <drv/timer.h>

#include <cpu/irq.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/pgmspace.h>

#define POWER_OFF 0
#define POWER_ON 1

static uint8_t power_config = 0;
static uint8_t power_status = POWER_ON;     // Can only be set when powered on.
static uint8_t power_request = POWER_ON;   // Initially, we should power off.

static void __attribute__ ((noreturn)) reboot(void)
{
    cli();
    sleep_disable();
    wdt_disable();
    wdt_enable(WDTO_15MS);
    while (1) {}
}

/**
 * VIN Detect interrupt service routine.
 *
 * If POWER_ON_VIN_ON (power on when VIN is present), this is configured
 * immediately before we go to sleep to trigger when it goes HIGH.
 *
 * If POWER_OFF_VIN_OFF (power off when VIN is removed), this is
 * configured in the power_on() routine to trigger on the falling edge.
 *
 * In this way TURN_OFF will not be triggered when POWER_OFF and TURN_ON
 * will not be triggered when POWER_ON.
 *
 * And the interrupt is not used when inappropriate for the settings.
 *
 * @param INT0_vect
 */
DECLARE_ISR(INT0_vect)
{
    EIMSK &= ~BV(INT0);
    if (power_status == POWER_ON)
    {
        request_power_off();
    }
}

// BUTTON Detect
DECLARE_ISR(INT1_vect)
{
    EIMSK &= ~BV(INT1);
    if (power_status == POWER_ON)
    {
        request_power_off();
    }
}

void power_on(void)
{
    ser_init(&ser, SER_UART0);
    ser_setbaudrate(&ser, 38400L);

    hc05_power_on();

    AUDIO_OFFSET_DDR |= BV(AUDIO_OFFSET_PIN);
    AUDIO_OFFSET_PORT |= BV(AUDIO_OFFSET_PIN);
}

void request_power_off(void)
{
    power_request = POWER_OFF;
}

bool power_off_requested(void)
{
    return power_request == POWER_OFF;
}

void enable_power_off(void)
{
    BUTTON_DDR &= ~BV(BUTTON_PIN);      // Input.

    cli();  // We are powering on.  Do not disturb our preparation.

    power_status = POWER_ON;

    // Button interrupt is set to falling edge.
    EICRA = BV(ISC11);
    EIMSK = BV(INT1);

    if (power_config & POWER_OFF_VIN_OFF)
    {
        // Rising edge for this one.
        EICRA |= (BV(ISC01) | BV(ISC00));
        EIMSK |= BV(INT0);
    }

    sei();  // Allow interrupts to power down.
}

void power_off(void)
{
    power_status = POWER_OFF;
    power_request = POWER_ON;

    wdt_disable();

    hc05_power_off();

    cli();  // We are powering off.  Do not disturb our preparation.

    // Remove the offset voltage for ADC
    AUDIO_OFFSET_PORT &= ~BV(AUDIO_OFFSET_PIN);
    AUDIO_OFFSET_DDR &= ~BV(AUDIO_OFFSET_PIN);

    // disable ADC
    uint8_t ADCSRA_save = ADCSRA;
    ADCSRA = 0;

    PRR = 0xFF;  // turn off various modules

    // Button interrupt is set to low level.
    EICRA = BV(ISC11);
    EIMSK = BV(INT1);

    if (power_config & POWER_ON_VIN_ON)
    {
        // VIN interrupt set to low level (inverted).
        // EICRA |= BV(ISC00);
        EIMSK |= BV(INT0);
    }

    wdt_disable();

    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();

    // turn off brown-out enable in software
    MCUCR = BV(BODS) | BV(BODSE);   // turn on brown-out enable select
    MCUCR = BV(BODS);   // this must be done within 4 clock cycles of above
    sei();              // Allow interrupts to wake from power down.
    sleep_cpu();

    sleep_disable();

    power_status = POWER_ON;

    EIMSK = 0;

    PRR = 0;
    ADCSRA = ADCSRA_save;

    power_on();
    power_on_message(0);
    enable_power_off();

    wdt_enable(WDTO_4S);
}

void set_power_config(uint8_t config)
{
    power_config = config;
}

uint8_t get_power_config(void)
{
    return power_config;
}

const char firmware_version[] PROGMEM = "\050" MOBILINKD_VERSION_STR;
const char hardware_version[] PROGMEM = "\051" "2.0";
const char PREFIX[] PROGMEM = "== ";
const char ENDL[] PROGMEM = "\r\n";

volatile uint8_t wdt_location __attribute__ ((section (".noinit")));

void power_on_message(int hc_status)
{
    // Announce
    kfile_print_P(&ser.fd, PSTR("\r\n== BeRTOS AVR/Mobilinkd TNC2\r\n"));

    kfile_print_P(&ser.fd, PSTR("== Version "));
    kfile_print_P(&ser.fd, firmware_version + 1);
    kfile_print_P(&ser.fd, ENDL);

    uint16_t voltage = check_battery();

    kfile_printf(&ser.fd, "== Voltage: %umV\r\n", voltage);

    if (usb_vin_available())
    {
        kfile_printf(&ser.fd, PSTR("== USB power detected\r\n"));
    }

#ifdef DEBUG
    kfile_printf(&ser.fd, "== Bootloader flags: %08lx\r\n", get_bootloader_value());
    kfile_printf(&ser.fd, "== WDT (loc = %02x)\r\n", wdt_location);
    kfile_print_P(&ser.fd, PREFIX);
    kfile_print_P(&ser.fd, mobilinkd_strerror(mobilinkd_get_error()));
    kfile_print_P(&ser.fd, ENDL);
#endif

    if (hc_status != 0)
        kfile_printf(&ser.fd, "== HC-05 = %02x\r\n", hc_status);
    kfile_print_P(&ser.fd, PSTR("== Starting.\r\n"));
}

bool usb_vin_available(void)
{
    VIN_DETECT_DDR &= ~BV(VIN_DETECT_PIN);
    return ((VIN_DETECT_PORT & BV(VIN_DETECT_PIN)) != 0);
}
