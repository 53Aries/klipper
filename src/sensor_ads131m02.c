// Support for ADS131M02 ADC Chip
//
// Copyright (C) 2025
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "board/irq.h" // irq_disable
#include "board/gpio.h" // gpio_out_write
#include "board/misc.h" // timer_read_time
#include "basecmd.h" // oid_alloc
#include "command.h" // DECL_COMMAND
#include "sched.h" // sched_add_timer
#include "sensor_bulk.h" // sensor_bulk_report
#include "load_cell_probe.h" // load_cell_probe_report_sample
#include "spicmds.h" // spidev_transfer
#include <stdint.h>

struct ads131m02_adc {
    struct timer timer;
    uint32_t rest_ticks;
    struct gpio_in data_ready;
    struct spidev_s *spi;
    uint8_t pending_flag, data_count;
    uint8_t channel_select; // 0 or 1
    struct sensor_bulk sb;
    struct load_cell_probe *lce;
};

// Flag types
enum {
    FLAG_PENDING = 1 << 0
};

#define BYTES_PER_SAMPLE 4

static struct task_wake wake_ads131m02;

/****************************************************************
 * ADS131M02 Sensor Support
 ****************************************************************/

static inline uint8_t
ads131m02_is_data_ready(struct ads131m02_adc *adc) {
    return gpio_in_read(adc->data_ready) == 0;
}

// Event handler that wakes wake_ads131m02() periodically
static uint_fast8_t
ads131m02_event(struct timer *timer)
{
    struct ads131m02_adc *adc = container_of(timer, struct ads131m02_adc,
                                              timer);
    uint32_t rest_ticks = adc->rest_ticks;
    if (adc->pending_flag) {
        adc->sb.possible_overflows++;
        rest_ticks *= 4;
    } else if (ads131m02_is_data_ready(adc)) {
        adc->pending_flag = 1;
        sched_wake_task(&wake_ads131m02);
        rest_ticks *= 8;
    }
    adc->timer.waketime += rest_ticks;
    return SF_RESCHEDULE;
}

// Add a measurement to the buffer
static void
add_sample(struct ads131m02_adc *adc, uint8_t oid, uint_fast32_t counts)
{
    adc->sb.data[adc->sb.data_count] = counts;
    adc->sb.data[adc->sb.data_count + 1] = counts >> 8;
    adc->sb.data[adc->sb.data_count + 2] = counts >> 16;
    adc->sb.data[adc->sb.data_count + 3] = counts >> 24;
    adc->sb.data_count += BYTES_PER_SAMPLE;

    if ((adc->sb.data_count + BYTES_PER_SAMPLE) > ARRAY_SIZE(adc->sb.data)) {
        sensor_bulk_report(&adc->sb, oid);
    }
}

// ADS131M02 ADC query
static void
ads131m02_read_adc(struct ads131m02_adc *adc, uint8_t oid)
{
    // Typical communication frame at 24-bit word length:
    // 3-byte STATUS + 3-byte CH0 + 3-byte CH1 + 3-byte CRC.
    // Clock out with NULL command bytes on DIN.
    uint8_t rx[12] = {0};
    spidev_transfer(adc->spi, 1, sizeof(rx), rx);
    adc->pending_flag = 0;
    barrier();

    // Select channel bytes (skip 3-byte status)
    uint8_t offset = 3 + (adc->channel_select ? 3 : 0);
    uint32_t counts = ((uint32_t)rx[offset] << 16)
                    | ((uint32_t)rx[offset + 1] << 8)
                    | ((uint32_t)rx[offset + 2]);
    // Sign-extend 24-bit two's complement to 32-bit
    if (counts & 0x800000)
        counts |= 0xFF000000;

    if (adc->lce) {
        load_cell_probe_report_sample(adc->lce, counts);
    }
    add_sample(adc, oid, counts);
}

// Create an ads131m02 sensor
void
command_config_ads131m02(uint32_t *args)
{
    struct ads131m02_adc *adc = oid_alloc(args[0]
                , command_config_ads131m02, sizeof(*adc));
    adc->timer.func = ads131m02_event;
    adc->pending_flag = 0;
    adc->spi = spidev_oid_lookup(args[1]);
    // ADS131M02 DRDY# is active-low; enable internal pull-up to avoid floating
    adc->data_ready = gpio_in_setup(args[2], 1);
    uint8_t ch = args[3];
    adc->channel_select = (ch > 0) ? 1 : 0;
}
DECL_COMMAND(command_config_ads131m02, "config_ads131m02 oid=%c spi_oid=%c data_ready_pin=%u channel=%c");

void
ads131m02_attach_load_cell_probe(uint32_t *args) {
    uint8_t oid = args[0];
    struct ads131m02_adc *adc = oid_lookup(oid, command_config_ads131m02);
    adc->lce = load_cell_probe_oid_lookup(args[1]);
}
DECL_COMMAND(ads131m02_attach_load_cell_probe,
    "ads131m02_attach_load_cell_probe oid=%c load_cell_probe_oid=%c");

// start/stop capturing ADC data
void
command_query_ads131m02(uint32_t *args)
{
    uint8_t oid = args[0];
    struct ads131m02_adc *adc = oid_lookup(oid, command_config_ads131m02);
    sched_del_timer(&adc->timer);
    adc->pending_flag = 0;
    adc->rest_ticks = args[1];
    if (!adc->rest_ticks) {
        // End measurements
        return;
    }
    // Start new measurements
    sensor_bulk_reset(&adc->sb);
    irq_disable();
    adc->timer.waketime = timer_read_time() + adc->rest_ticks;
    sched_add_timer(&adc->timer);
    irq_enable();
}
DECL_COMMAND(command_query_ads131m02, "query_ads131m02 oid=%c rest_ticks=%u");

void
command_query_ads131m02_status(const uint32_t *args)
{
    uint8_t oid = args[0];
    struct ads131m02_adc *adc = oid_lookup(oid, command_config_ads131m02);
    irq_disable();
    const uint32_t start_t = timer_read_time();
    uint8_t is_data_ready = ads131m02_is_data_ready(adc);
    irq_enable();
    uint8_t pending_bytes = is_data_ready ? BYTES_PER_SAMPLE : 0;
    sensor_bulk_status(&adc->sb, oid, start_t, 0, pending_bytes);
}
DECL_COMMAND(command_query_ads131m02_status, "query_ads131m02_status oid=%c");

// Background task that performs measurements
void
ads131m02_capture_task(void)
{
    if (!sched_check_wake(&wake_ads131m02))
        return;
    uint8_t oid;
    struct ads131m02_adc *adc;
    foreach_oid(oid, adc, command_config_ads131m02) {
        if (adc->pending_flag)
            ads131m02_read_adc(adc, oid);
    }
}
DECL_TASK(ads131m02_capture_task);
