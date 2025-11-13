// Support for bit-banging CS1237 ADC chip for load-cell usage
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "autoconf.h"
#include "board/gpio.h"      // gpio_out_write
#include "board/irq.h"       // irq_poll
#include "board/misc.h"      // timer_read_time
#include "basecmd.h"         // oid_alloc
#include "command.h"         // DECL_COMMAND
#include "sched.h"           // sched_add_timer
#include "sensor_bulk.h"     // sensor_bulk_report
#include "load_cell_probe.h" // load_cell_probe_report_sample
#include <stdbool.h>
#include <stdint.h>

struct cs1237_adc {
    struct timer timer;
    uint8_t flags;
    uint32_t rest_ticks;
    uint32_t last_error;
    struct gpio_in dout;   // CS1237 DOUT pin
    struct gpio_out sclk;  // CS1237 SCLK pin (PD_SCK)
    struct sensor_bulk sb;
    struct load_cell_probe *lce;
};

enum {
    CS_PENDING = 1<<0, CS_OVERFLOW = 1<<1,
};

#define BYTES_PER_SAMPLE 4
#define SAMPLE_ERROR_READ_TOO_LONG (1UL << 30)

static struct task_wake wake_cs1237;

// Minimum delay ~200ns between SCK edges
#define MIN_PULSE_TICKS nsecs_to_ticks(200)

static inline uint32_t nsecs_to_ticks(uint32_t ns)
{
    return timer_from_us(ns * 1000) / 1000000;
}

static inline void cs1237_delay_noirq(void)
{
#if CONFIG_MACH_AVR
    asm("nop\n\tnop");
#else
    uint32_t end = timer_read_time() + MIN_PULSE_TICKS;
    while (timer_is_before(timer_read_time(), end))
        ;
#endif
}

static inline void cs1237_delay(void)
{
#if CONFIG_MACH_AVR
    return;
#else
    uint32_t end = timer_read_time() + MIN_PULSE_TICKS;
    while (timer_is_before(timer_read_time(), end))
        irq_poll();
#endif
}

// Generate one full clock pulse on SCLK
static inline void cs1237_clock_pulse(struct gpio_out sclk)
{
    irq_disable();
    gpio_out_toggle_noirq(sclk);
    cs1237_delay_noirq();
    gpio_out_toggle_noirq(sclk);
    irq_enable();
    cs1237_delay();
}

// Read 24 data bits MSB-first; optionally add one trailing pulse to latch next conversion
static uint32_t
cs1237_read24(struct gpio_in dout, struct gpio_out sclk)
{
    uint32_t v = 0;
    for (int i = 0; i < 24; i++) {
        irq_disable();
        gpio_out_toggle_noirq(sclk);
        cs1237_delay_noirq();
        gpio_out_toggle_noirq(sclk);
        uint_fast8_t bit = gpio_in_read(dout);
        irq_enable();
        cs1237_delay();
        v = (v << 1) | bit;
    }
    // A single extra pulse advances the CS1237 to the next conversion
    cs1237_clock_pulse(sclk);
    return v;
}

// Data ready is indicated by DOUT low
static inline uint_fast8_t cs1237_is_data_ready(struct cs1237_adc *cs)
{
    return !gpio_in_read(cs->dout);
}

// Periodic event used to check for new data and overflow conditions
static uint_fast8_t cs1237_event(struct timer *timer)
{
    struct cs1237_adc *cs = container_of(timer, struct cs1237_adc, timer);
    uint32_t rest_ticks = cs->rest_ticks;
    if (cs->flags & CS_PENDING) {
        cs->sb.possible_overflows++;
        cs->flags = CS_PENDING | CS_OVERFLOW;
        rest_ticks *= 4;
    } else if (cs1237_is_data_ready(cs)) {
        cs->flags = CS_PENDING;
        sched_wake_task(&wake_cs1237);
        rest_ticks *= 8;
    }
    cs->timer.waketime += rest_ticks;
    return SF_RESCHEDULE;
}

static void add_sample(struct cs1237_adc *cs, uint8_t oid,
                       uint32_t counts, uint8_t force_flush)
{
    cs->sb.data[cs->sb.data_count] = counts;
    cs->sb.data[cs->sb.data_count + 1] = counts >> 8;
    cs->sb.data[cs->sb.data_count + 2] = counts >> 16;
    cs->sb.data[cs->sb.data_count + 3] = counts >> 24;
    cs->sb.data_count += BYTES_PER_SAMPLE;
    if (cs->sb.data_count + BYTES_PER_SAMPLE > ARRAY_SIZE(cs->sb.data)
        || force_flush)
        sensor_bulk_report(&cs->sb, oid);
}

static void cs1237_read_adc(struct cs1237_adc *cs, uint8_t oid)
{
    uint32_t adc = cs1237_read24(cs->dout, cs->sclk);

    // Clear pending flag and capture overflow state
    irq_disable();
    uint8_t flags = cs->flags;
    cs->flags = 0;
    irq_enable();

    // Sign-extend 24-bit two's complement to 32-bit
    if (adc & 0x800000)
        adc |= 0xFF000000;

    if (flags & CS_OVERFLOW)
        cs->last_error = SAMPLE_ERROR_READ_TOO_LONG;

    uint32_t counts = cs->last_error ? cs->last_error : adc;

    if (cs->last_error == 0 && cs->lce)
        load_cell_probe_report_sample(cs->lce, counts);

    add_sample(cs, oid, counts, false);
}

// config_cs1237: Set up pins and initial state
void command_config_cs1237(uint32_t *args)
{
    struct cs1237_adc *cs = oid_alloc(args[0], command_config_cs1237,
                                      sizeof(*cs));
    cs->timer.func = cs1237_event;
    cs->flags = 0;
    cs->last_error = 0;
    cs->dout = gpio_in_setup(args[1], 1);
    cs->sclk = gpio_out_setup(args[2], 0);
    // Hold SCLK low when active; bring high to enter power-down
    gpio_out_write(cs->sclk, 1);
}
DECL_COMMAND(command_config_cs1237,
             "config_cs1237 oid=%c dout_pin=%u sclk_pin=%u");

void cs1237_attach_load_cell_probe(uint32_t *args)
{
    uint8_t oid = args[0];
    struct cs1237_adc *cs = oid_lookup(oid, command_config_cs1237);
    cs->lce = load_cell_probe_oid_lookup(args[1]);
}
DECL_COMMAND(cs1237_attach_load_cell_probe,
    "cs1237_attach_load_cell_probe oid=%c load_cell_probe_oid=%c");

// Start/stop streaming measurements
void command_query_cs1237(uint32_t *args)
{
    uint8_t oid = args[0];
    struct cs1237_adc *cs = oid_lookup(oid, command_config_cs1237);
    sched_del_timer(&cs->timer);
    cs->flags = 0;
    cs->last_error = 0;
    cs->rest_ticks = args[1];
    if (!cs->rest_ticks) {
        // Stop measurements and power down
        gpio_out_write(cs->sclk, 1);
        return;
    }
    // Start measurements: wake by driving SCLK low
    gpio_out_write(cs->sclk, 0);
    sensor_bulk_reset(&cs->sb);
    irq_disable();
    cs->timer.waketime = timer_read_time() + cs->rest_ticks;
    sched_add_timer(&cs->timer);
    irq_enable();
}
DECL_COMMAND(command_query_cs1237, "query_cs1237 oid=%c rest_ticks=%u");

void command_query_cs1237_status(const uint32_t *args)
{
    uint8_t oid = args[0];
    struct cs1237_adc *cs = oid_lookup(oid, command_config_cs1237);
    irq_disable();
    const uint32_t start_t = timer_read_time();
    uint8_t ready = cs1237_is_data_ready(cs);
    irq_enable();
    uint8_t pending = ready ? BYTES_PER_SAMPLE : 0;
    sensor_bulk_status(&cs->sb, oid, start_t, 0, pending);
}
DECL_COMMAND(command_query_cs1237_status, "query_cs1237_status oid=%c");

// Background capture task
void cs1237_capture_task(void)
{
    if (!sched_check_wake(&wake_cs1237))
        return;
    uint8_t oid;
    struct cs1237_adc *cs;
    foreach_oid(oid, cs, command_config_cs1237) {
        if (cs->flags)
            cs1237_read_adc(cs, oid);
    }
}
DECL_TASK(cs1237_capture_task);
