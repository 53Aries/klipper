import logging
from . import bulk_sensor

# Match HX71x style BulkSensor integration, but with CS1237 command names
UPDATE_INTERVAL = 0.10
SAMPLE_ERROR_LONG_READ = 0x40000000

class CS1237:
    def __init__(self, config):
        self.printer = printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        # Pins
        dout_pin_name = config.get('dout_pin')
        sclk_pin_name = config.get('sclk_pin')
        ppins = printer.lookup_object('pins')
        dout_ppin = ppins.lookup_pin(dout_pin_name)
        sclk_ppin = ppins.lookup_pin(sclk_pin_name)
        self.mcu = mcu = dout_ppin['chip']
        if sclk_ppin['chip'] is not mcu:
            raise config.error("%s config error: All pins must be on the same MCU" % (self.name,))
        self.oid = mcu.create_oid()
        self.dout_pin = dout_ppin['pin']
        self.sclk_pin = sclk_ppin['pin']
        # Assume 320sps default for probing; this just tunes host buffering
        self.sps = config.getint('sample_rate', default=320, minval=10, maxval=1280)
        # Bulk reader setup
        self.bulk_queue = bulk_sensor.BulkDataQueue(mcu, oid=self.oid)
        chip_smooth = self.sps * UPDATE_INTERVAL * 2
        self.ffreader = bulk_sensor.FixedFreqReader(mcu, chip_smooth, "<i")
        self.batch_bulk = bulk_sensor.BatchBulkHelper(
            self.printer, self._process_batch, self._start_measurements,
            self._finish_measurements, UPDATE_INTERVAL)
        # Commands
        self.query_cmd = None
        self.attach_probe_cmd = None
        mcu.add_config_cmd(
            "config_cs1237 oid=%d dout_pin=%s sclk_pin=%s" % (self.oid, self.dout_pin, self.sclk_pin))
        mcu.add_config_cmd("query_cs1237 oid=%d rest_ticks=0" % (self.oid,), on_restart=True)
        mcu.register_config_callback(self._build_config)

        self.last_error_count = 0
        self.consecutive_fails = 0

    def _build_config(self):
        self.query_cmd = self.mcu.lookup_command("query_cs1237 oid=%c rest_ticks=%u")
        self.attach_probe_cmd = self.mcu.lookup_command(
            "cs1237_attach_load_cell_probe oid=%c load_cell_probe_oid=%c")
        self.ffreader.setup_query_command("query_cs1237_status oid=%c",
                                          oid=self.oid,
                                          cq=self.mcu.alloc_command_queue())

    def get_mcu(self):
        return self.mcu

    def get_samples_per_second(self):
        return self.sps

    def get_range(self):
        # 24-bit signed
        return -0x800000, 0x7FFFFF

    def add_client(self, callback):
        self.batch_bulk.add_client(callback)

    def attach_load_cell_probe(self, load_cell_probe_oid):
        self.attach_probe_cmd.send([self.oid, load_cell_probe_oid])

    def _convert_samples(self, samples):
        adc_factor = 1. / (1 << 23)
        count = 0
        for ptime, val in samples:
            if val == SAMPLE_ERROR_LONG_READ:
                self.last_error_count += 1
                break
            samples[count] = (round(ptime, 6), val, round(val * adc_factor, 9))
            count += 1
        del samples[count:]

    def _start_measurements(self):
        self.consecutive_fails = 0
        self.last_error_count = 0
        rest_ticks = self.mcu.seconds_to_clock(1. / (10. * self.sps))
        self.query_cmd.send([self.oid, rest_ticks])
        logging.info("cs1237 starting '%s' measurements", self.name)
        self.ffreader.note_start()

    def _finish_measurements(self):
        if self.printer.is_shutdown():
            return
        self.query_cmd.send_wait_ack([self.oid, 0])
        self.ffreader.note_end()
        logging.info("cs1237 finished '%s' measurements", self.name)

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
                logging.error("%s: Forced sensor restart due to overflows", self.name)
                self._finish_measurements()
                self._start_measurements()
        else:
            self.consecutive_fails = 0
        return {'data': samples, 'errors': self.last_error_count,
                'overflows': self.ffreader.get_last_overflows()}

CS1237_SENSOR_TYPE = {"cs1237": CS1237}