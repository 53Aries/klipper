// Support for bit-banging commands to CS1237 ADC chip
//
// Copyright (C) 2024 Gareth Farrington <gareth@waves.ky>
// Copyright (C) 2025 53Aries
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "autoconf.h" // CONFIG_MACH_AVR
#include "board/gpio.h" // gpio_out_write
#include "board/irq.h" // irq_poll
#include "board/misc.h" // timer_read_time
#include "basecmd.h" // oid_alloc
#include "command.h" // DECL_COMMAND
#include "sched.h" // sched_add_timer
#include "sensor_bulk.h" // sensor_bulk_report
#include "load_cell_probe.h" // load_cell_probe_report_sample
#include <stdbool.h>
#include <stdint.h>

struct cs1237_adc {
    struct timer timer;
    uint8_t config_byte;    // configuration register value
    uint8_t flags;
    uint32_t rest_ticks;
    uint32_t last_error;
    struct gpio_in dout; // pin used to receive data from the cs1237
    struct gpio_out sclk; // pin used to generate clock for the cs1237
    struct sensor_bulk sb;
    struct load_cell_probe *lce;
};

enum {
    CS_PENDING = 1<<0, CS_OVERFLOW = 1<<1, CS_CONFIGURED = 1<<2,
};

#define BYTES_PER_SAMPLE 4
#define SAMPLE_ERROR_DESYNC 1L << 31
#define SAMPLE_ERROR_READ_TOO_LONG 1L << 30

static struct task_wake wake_cs1237;


/****************************************************************
 * Low-level bit-banging
 ****************************************************************/

#define MIN_PULSE_TIME nsecs_to_ticks(200)

static uint32_t
nsecs_to_ticks(uint32_t ns)
{
    return timer_from_us(ns * 1000) / 1000000;
}

// Pause for 200ns
static void
cs1237_delay_noirq(void)
{
    if (CONFIG_MACH_AVR) {
        // Optimize avr, as calculating time takes longer than needed delay
        asm("nop\n    nop");
        return;
    }
    uint32_t end = timer_read_time() + MIN_PULSE_TIME;
    while (timer_is_before(timer_read_time(), end))
        ;
}

// Pause for a minimum of 200ns
static void
cs1237_delay(void)
{
    if (CONFIG_MACH_AVR)
        // Optimize avr, as calculating time takes longer than needed delay
        return;
    uint32_t end = timer_read_time() + MIN_PULSE_TIME;
    while (timer_is_before(timer_read_time(), end))
        irq_poll();
}

// Read 'num_bits' from the sensor
static uint32_t
cs1237_raw_read(struct gpio_in dout, struct gpio_out sclk, int num_bits)
{
    uint32_t bits_read = 0;
    while (num_bits--) {
        irq_disable();
        gpio_out_toggle_noirq(sclk);
        cs1237_delay_noirq();
        gpio_out_toggle_noirq(sclk);
        uint_fast8_t bit = gpio_in_read(dout);
        irq_enable();
        cs1237_delay();
        bits_read = (bits_read << 1) | bit;
    }
    return bits_read;
}

// Write configuration to CS1237
// The CS1237 requires a specific write sequence:
// 1. Pull SCLK high for at least 100us
// 2. Send 36 additional pulses (29-36th pulse)
// 3. Wait for DOUT to go low (ACK)
// 4. Send 7 more pulses while shifting config bits
static void
cs1237_write_config(struct cs1237_adc *cs1237, uint8_t config)
{
    // Enter write mode: 29-36 pulses after data read
    // We provide 36 pulses total
    int i;
    for (i = 0; i < 36; i++) {
        irq_disable();
        gpio_out_toggle_noirq(cs1237->sclk);
        cs1237_delay_noirq();
        gpio_out_toggle_noirq(cs1237->sclk);
        irq_enable();
        cs1237_delay();
    }
    
    // Wait for DOUT to go low (ACK from chip)
    uint32_t timeout = timer_read_time() + timer_from_us(1000);
    while (gpio_in_read(cs1237->dout)) {
        if (!timer_is_before(timer_read_time(), timeout))
            return; // timeout
        irq_poll();
    }
    
    // Send 7 config bits (MSB first)
    // Note: The CS1237 echoes back the config bits on DOUT
    for (i = 6; i >= 0; i--) {
        irq_disable();
        gpio_out_toggle_noirq(cs1237->sclk);
        cs1237_delay_noirq();
        gpio_out_toggle_noirq(cs1237->sclk);
        irq_enable();
        cs1237_delay();
    }
    
    // One more pulse to complete write
    irq_disable();
    gpio_out_toggle_noirq(cs1237->sclk);
    cs1237_delay_noirq();
    gpio_out_toggle_noirq(cs1237->sclk);
    irq_enable();
}


/****************************************************************
 * CS1237 Sensor Support
 ****************************************************************/

// Check if data is ready
static uint_fast8_t
cs1237_is_data_ready(struct cs1237_adc *cs1237)
{
    return !gpio_in_read(cs1237->dout);
}

// Event handler that wakes wake_cs1237() periodically
static uint_fast8_t
cs1237_event(struct timer *timer)
{
    struct cs1237_adc *cs1237 = container_of(timer, struct cs1237_adc, timer);
    uint32_t rest_ticks = cs1237->rest_ticks;
    uint8_t flags = cs1237->flags;
    if (flags & CS_PENDING) {
        cs1237->sb.possible_overflows++;
        cs1237->flags = CS_PENDING | CS_OVERFLOW;
        rest_ticks *= 4;
    } else if (cs1237_is_data_ready(cs1237)) {
        // New sample pending
        cs1237->flags = CS_PENDING;
        sched_wake_task(&wake_cs1237);
        rest_ticks *= 8;
    }
    cs1237->timer.waketime += rest_ticks;
    return SF_RESCHEDULE;
}

static void
add_sample(struct cs1237_adc *cs1237, uint8_t oid, uint32_t counts,
                uint8_t force_flush) {
    // Add measurement to buffer
    cs1237->sb.data[cs1237->sb.data_count] = counts;
    cs1237->sb.data[cs1237->sb.data_count + 1] = counts >> 8;
    cs1237->sb.data[cs1237->sb.data_count + 2] = counts >> 16;
    cs1237->sb.data[cs1237->sb.data_count + 3] = counts >> 24;
    cs1237->sb.data_count += BYTES_PER_SAMPLE;

    if (cs1237->sb.data_count + BYTES_PER_SAMPLE > ARRAY_SIZE(cs1237->sb.data)
        || force_flush)
        sensor_bulk_report(&cs1237->sb, oid);
}

// cs1237 ADC query
static void
cs1237_read_adc(struct cs1237_adc *cs1237, uint8_t oid)
{
    // Configure chip on first read
    if (!(cs1237->flags & CS_CONFIGURED)) {
        cs1237_write_config(cs1237, cs1237->config_byte);
        cs1237->flags |= CS_CONFIGURED;
    }

    // Read 24 bits from sensor
    uint32_t adc = cs1237_raw_read(cs1237->dout, cs1237->sclk, 24);
    
    // One additional clock pulse to prepare for next conversion
    irq_disable();
    gpio_out_toggle_noirq(cs1237->sclk);
    cs1237_delay_noirq();
    gpio_out_toggle_noirq(cs1237->sclk);
    irq_enable();

    // Clear pending flag (and note if an overflow occurred)
    irq_disable();
    uint8_t flags = cs1237->flags;
    cs1237->flags &= CS_CONFIGURED; // Keep configured flag
    irq_enable();

    // Convert to signed 24-bit value
    uint32_t counts = adc;
    if (counts & 0x800000)
        counts |= 0xFF000000;

    // Check for errors
    if (flags & CS_OVERFLOW) {
        // Transfer took too long
        cs1237->last_error = SAMPLE_ERROR_READ_TOO_LONG;
    }

    // forever send errors until reset
    if (cs1237->last_error != 0) {
        counts = cs1237->last_error;
    }

    // probe is optional, report if enabled
    if (cs1237->last_error == 0 && cs1237->lce) {
        load_cell_probe_report_sample(cs1237->lce, counts);
    }

    // Add measurement to buffer
    add_sample(cs1237, oid, counts, false);
}

// Create a cs1237 sensor
void
command_config_cs1237(uint32_t *args)
{
    struct cs1237_adc *cs1237 = oid_alloc(args[0]
                , command_config_cs1237, sizeof(*cs1237));
    cs1237->timer.func = cs1237_event;
    cs1237->config_byte = args[1];
    cs1237->dout = gpio_in_setup(args[2], 1);
    cs1237->sclk = gpio_out_setup(args[3], 0);
}
DECL_COMMAND(command_config_cs1237, "config_cs1237 oid=%c config=%c"
             " dout_pin=%u sclk_pin=%u");

void
cs1237_attach_load_cell_probe(uint32_t *args) {
    uint8_t oid = args[0];
    struct cs1237_adc *cs1237 = oid_lookup(oid, command_config_cs1237);
    cs1237->lce = load_cell_probe_oid_lookup(args[1]);
}
DECL_COMMAND(cs1237_attach_load_cell_probe, "cs1237_attach_load_cell_probe oid=%c"
    " load_cell_probe_oid=%c");

// start/stop capturing ADC data
void
command_query_cs1237(uint32_t *args)
{
    uint8_t oid = args[0];
    struct cs1237_adc *cs1237 = oid_lookup(oid, command_config_cs1237);
    sched_del_timer(&cs1237->timer);
    cs1237->flags = 0;
    cs1237->last_error = 0;
    cs1237->rest_ticks = args[1];
    if (!cs1237->rest_ticks) {
        // End measurements
        return;
    }
    // Start new measurements
    sensor_bulk_reset(&cs1237->sb);
    irq_disable();
    cs1237->timer.waketime = timer_read_time() + cs1237->rest_ticks;
    sched_add_timer(&cs1237->timer);
    irq_enable();
}
DECL_COMMAND(command_query_cs1237, "query_cs1237 oid=%c rest_ticks=%u");

void
command_query_cs1237_status(const uint32_t *args)
{
    uint8_t oid = args[0];
    struct cs1237_adc *cs1237 = oid_lookup(oid, command_config_cs1237);
    irq_disable();
    const uint32_t start_t = timer_read_time();
    uint8_t is_data_ready = cs1237_is_data_ready(cs1237);
    irq_enable();
    uint8_t pending_bytes = is_data_ready ? BYTES_PER_SAMPLE : 0;
    sensor_bulk_status(&cs1237->sb, oid, start_t, 0, pending_bytes);
}
DECL_COMMAND(command_query_cs1237_status, "query_cs1237_status oid=%c");

// Background task that performs measurements
void
cs1237_capture_task(void)
{
    if (!sched_check_wake(&wake_cs1237))
        return;
    uint8_t oid;
    struct cs1237_adc *cs1237;
    foreach_oid(oid, cs1237, command_config_cs1237) {
        if (cs1237->flags & CS_PENDING)
            cs1237_read_adc(cs1237, oid);
    }
}
DECL_TASK(cs1237_capture_task);
