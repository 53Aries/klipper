# Report pressure delta (Pa) as a temperature-like sensor so UI can graph it
#
# This virtual sensor reads delta from a [pressure_fan] and reports it via the
# standard heaters/sensor interface as a 'temperature'. Mainsail will show it on
# the Temperatures panel (units will display as °C, but numeric value is Pa).
#
# Example config:
# [temperature_sensor delta_pa]
# sensor_type: PRESSURE_DELTA
# pressure_fan: exhaust
# mode: window    # window|raw
# scale: 1.0      # multiply (e.g., 0.1 to show deci-Pa)
# offset: 0.0     # add offset
# report_time: 0.8
#
import logging

REPORT_TIME = 0.8

class PressureDeltaSensor:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.reactor = self.printer.get_reactor()
        self.mcu = self.printer.lookup_object('mcu')
        self._callback = lambda *args, **kwargs: None

        self.pfan_name = config.get('pressure_fan')
        self.mode = config.getchoice('mode', {'window':'window','raw':'raw'}, default='window')
        self.scale = config.getfloat('scale', 1.0)
        self.offset = config.getfloat('offset', 0.0)
        self.report_time = config.getfloat('report_time', REPORT_TIME, above=0.1)

        self.temp = 0.0
        self.min_temp = -999999.0
        self.max_temp = 999999.0
        self.sample_timer = None
        self.pfan = None

        if self.printer.get_start_args().get('debugoutput') is not None:
            return
        self.printer.register_event_handler('klippy:ready', self._on_ready)

    def _on_ready(self):
        # Lookup referenced pressure_fan
        try:
            self.pfan = self.printer.lookup_object('pressure_fan ' + self.pfan_name)
        except Exception:
            self.pfan = None
            logging.error("PRESSURE_DELTA %s: pressure_fan '%s' not found", self.name, self.pfan_name)
        self.sample_timer = self.reactor.register_timer(self._sample)
        self.reactor.update_timer(self.sample_timer, self.reactor.NOW)

    # heaters sensor API
    def setup_minmax(self, min_temp, max_temp):
        self.min_temp = min_temp
        self.max_temp = max_temp
    def setup_callback(self, cb):
        self._callback = cb
    def get_report_time_delta(self):
        return self.report_time

    def _sample(self, eventtime):
        try:
            if self.pfan is not None:
                if self.mode == 'window':
                    avg, n = self.pfan.get_window_delta()
                    val = avg if avg is not None else None
                else:
                    _p, _b, delta = self.pfan.get_pressure()
                    val = delta
                if val is not None:
                    self.temp = self.scale * float(val) + self.offset
            # Emit callback in MCU time base for graph alignment
            read_time = self.mcu.estimated_print_time(self.reactor.monotonic())
            self._callback(read_time, self.temp)
        except Exception:
            logging.exception("PRESSURE_DELTA %s: sample error", self.name)
        return eventtime + self.report_time

    def get_status(self, eventtime):
        return {
            'temperature': self.temp,
            'measured_min_temp': self.min_temp,
            'measured_max_temp': self.max_temp,
        }


def load_config(config):
    # Register sensor
    pheaters = config.get_printer().load_object(config, 'heaters')
    pheaters.add_sensor_factory('PRESSURE_DELTA', PressureDeltaSensor)
