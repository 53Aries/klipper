# CS1237 Support
#
# Copyright (C) 2024 Gareth Farrington <gareth@waves.ky>
# Copyright (C) 2025 53Aries
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
from . import bulk_sensor

#
# Constants
#
UPDATE_INTERVAL = 0.10
SAMPLE_ERROR_DESYNC = -0x80000000
SAMPLE_ERROR_LONG_READ = 0x40000000

# CS1237 Configuration Register Format (8-bit: B7-B0)
# Per datasheet Section 2.6.7.2:
# B7: Reserved (must be 0)
# B6: REF output switch (1=on, 0=off)
# B5-B4: SPEED_SEL (00=10Hz, 01=40Hz, 10=640Hz, 11=1280Hz)
# B3-B2: PGA_SEL (00=1, 01=2, 10=64, 11=128)
# B1-B0: CH_SEL (00=Channel A, 01=Chip retention, 10=Temp, 11=Internal short)

CONFIG_SPEED_10HZ = 0x00
CONFIG_SPEED_40HZ = 0x01
CONFIG_SPEED_640HZ = 0x02
CONFIG_SPEED_1280HZ = 0x03

CONFIG_GAIN_1 = 0x00
CONFIG_GAIN_2 = 0x01
CONFIG_GAIN_64 = 0x02
CONFIG_GAIN_128 = 0x03

CONFIG_CHANNEL_A = 0x00
CONFIG_CHANNEL_TEMP = 0x02

CONFIG_REF_ON = 0x01  # REF output on (default)


class CS1237:
    def __init__(self, config):
        self.printer = printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.last_error_count = 0
        self.consecutive_fails = 0
        self.sensor_type = "cs1237"
        
        # Chip options
        dout_pin_name = config.get('dout_pin')
        sclk_pin_name = config.get('sclk_pin')
        ppins = printer.lookup_object('pins')
        dout_ppin = ppins.lookup_pin(dout_pin_name)
        sclk_ppin = ppins.lookup_pin(sclk_pin_name)
        self.mcu = mcu = dout_ppin['chip']
        self.oid = mcu.create_oid()
        
        if sclk_ppin['chip'] is not mcu:
            raise config.error("%s config error: All pins must be "
                               "connected to the same MCU" % (self.name,))
        self.dout_pin = dout_ppin['pin']
        self.sclk_pin = sclk_ppin['pin']
        
        # Sample rate choices: 10, 40, 640, 1280 Hz
        sps_options = {'10': 10, '40': 40, '640': 640, '1280': 1280}
        self.sps = config.getchoice('sample_rate', sps_options, default='640')
        
        # Map sample rate to config value
        sps_map = {10: CONFIG_SPEED_10HZ, 40: CONFIG_SPEED_40HZ,
                   640: CONFIG_SPEED_640HZ, 1280: CONFIG_SPEED_1280HZ}
        speed_config = sps_map[self.sps]
        
        # Gain/channel choices
        gain_options = {'1': CONFIG_GAIN_1, '2': CONFIG_GAIN_2,
                        '64': CONFIG_GAIN_64, '128': CONFIG_GAIN_128}
        gain_config = config.getchoice('gain', gain_options, default='128')
        
        # Channel selection (normally use channel A for load cells)
        channel_options = {'A': CONFIG_CHANNEL_A, 'TEMP': CONFIG_CHANNEL_TEMP}
        channel_config = config.getchoice('channel', channel_options,
                                          default='A')
        
        # Build 8-bit config byte: B7=0, B6=1 (REF on), B5-B4=speed, B3-B2=gain, B1-B0=channel
        self.config_byte = ((0 << 7) | (CONFIG_REF_ON << 6) | (speed_config << 4) 
                            | (gain_config << 2) | channel_config)
        
        ## Bulk Sensor Setup
        self.bulk_queue = bulk_sensor.BulkDataQueue(mcu, oid=self.oid)
        # Clock tracking
        chip_smooth = self.sps * UPDATE_INTERVAL * 2
        self.ffreader = bulk_sensor.FixedFreqReader(mcu, chip_smooth, "<i")
        # Process messages in batches
        self.batch_bulk = bulk_sensor.BatchBulkHelper(
            self.printer, self._process_batch, self._start_measurements,
            self._finish_measurements, UPDATE_INTERVAL)
        
        # Command Configuration
        self.query_cs1237_cmd = None
        self.attach_probe_cmd = None
        mcu.add_config_cmd(
            "config_cs1237 oid=%d config=%d dout_pin=%s sclk_pin=%s"
            % (self.oid, self.config_byte, self.dout_pin, self.sclk_pin))
        mcu.add_config_cmd("query_cs1237 oid=%d rest_ticks=0"
                           % (self.oid,), on_restart=True)
        
        mcu.register_config_callback(self._build_config)

    def _build_config(self):
        self.query_cs1237_cmd = self.mcu.lookup_command(
            "query_cs1237 oid=%c rest_ticks=%u")
        self.attach_probe_cmd = self.mcu.lookup_command(
            "cs1237_attach_load_cell_probe oid=%c load_cell_probe_oid=%c")
        self.ffreader.setup_query_command("query_cs1237_status oid=%c",
                                          oid=self.oid,
                                          cq=self.mcu.alloc_command_queue())

    def get_mcu(self):
        return self.mcu

    def get_samples_per_second(self):
        return self.sps

    # returns a tuple of the minimum and maximum value of the sensor, used to
    # detect if a data value is saturated
    def get_range(self):
        return -0x800000, 0x7FFFFF

    # add_client interface, direct pass through to bulk_sensor API
    def add_client(self, callback):
        self.batch_bulk.add_client(callback)

    def attach_load_cell_probe(self, load_cell_probe_oid):
        self.attach_probe_cmd.send([self.oid, load_cell_probe_oid])

    # Measurement decoding
    def _convert_samples(self, samples):
        adc_factor = 1. / (1 << 23)
        count = 0
        for ptime, val in samples:
            if val == SAMPLE_ERROR_DESYNC or val == SAMPLE_ERROR_LONG_READ:
                self.last_error_count += 1
                break  # additional errors are duplicates
            samples[count] = (round(ptime, 6), val, round(val * adc_factor, 9))
            count += 1
        del samples[count:]

    # Start, stop, and process message batches
    def _start_measurements(self):
        self.consecutive_fails = 0
        self.last_error_count = 0
        # Start bulk reading
        rest_ticks = self.mcu.seconds_to_clock(1. / (10. * self.sps))
        self.query_cs1237_cmd.send([self.oid, rest_ticks])
        logging.info("%s starting '%s' measurements",
                     self.sensor_type, self.name)
        # Initialize clock tracking
        self.ffreader.note_start()

    def _finish_measurements(self):
        # don't use serial connection after shutdown
        if self.printer.is_shutdown():
            return
        # Halt bulk reading
        self.query_cs1237_cmd.send_wait_ack([self.oid, 0])
        self.ffreader.note_end()
        logging.info("%s finished '%s' measurements",
                     self.sensor_type, self.name)

    def _process_batch(self, eventtime):
        prev_overflows = self.ffreader.get_last_overflows()
        prev_error_count = self.last_error_count
        samples = self.ffreader.pull_samples()
        self._convert_samples(samples)
        overflows = self.ffreader.get_last_overflows() - prev_overflows
        errors = self.last_error_count - prev_error_count
        if errors > 0:
            logging.error("%s: Forced sensor restart due to error", self.name)
            self._finish_measurements()
            self._start_measurements()
        elif overflows > 0:
            self.consecutive_fails += 1
            if self.consecutive_fails > 4:
                logging.error("%s: Forced sensor restart due to overflows",
                              self.name)
                self._finish_measurements()
                self._start_measurements()
        else:
            self.consecutive_fails = 0
        return {'data': samples, 'errors': self.last_error_count,
                'overflows': self.ffreader.get_last_overflows()}


def load_config_prefix(config):
    return CS1237(config)


CS1237_SENSOR_TYPE = {"cs1237": CS1237}
