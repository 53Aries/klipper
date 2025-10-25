# Control an exhaust fan using pressure readings (e.g. BMP280/BME280) to maintain slight vacuum
#
# Copyright (C) 2025  53Aries
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
from . import fan

# Defaults
REPORT_TIME = 0.8  # seconds, aligns with BME280 default report interval
DEFAULT_TARGET_DELTA_PA = 15.0  # target vacuum relative to baseline in Pascals
PID_PARAM_BASE = 255.0


DEFAULT_MIN_TEMP = -40.0
DEFAULT_MAX_TEMP = 85.0


class PressureFan:
    def __init__(self, config):
        # Basics
        self.name = config.get_name().split()[1]
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()

        # Fan output (standard fan config keys apply: pin, cycle_time, etc.)
        self.fan = fan.Fan(config, default_shutdown_speed=0.0)
        # Sensor setup (reuse Klipper heaters sensor pattern)
        # Expect sensor_type and its settings (e.g., BME280 + I2C) within this section
        pheaters = self.printer.load_object(config, 'heaters')
        self.min_temp = config.getfloat('min_temp', DEFAULT_MIN_TEMP)
        self.max_temp = config.getfloat('max_temp', DEFAULT_MAX_TEMP, above=self.min_temp)
        self.sensor_obj = pheaters.setup_sensor(config)
        self.sensor_obj.setup_minmax(self.min_temp, self.max_temp)
        # Register a no-op callback to keep periodic scheduling similar to other sensors
        self.sensor_obj.setup_callback(self._sensor_callback)
        pheaters.register_sensor(config, self)

        # Control limits
        self.max_speed_conf = config.getfloat('max_speed', 1.0, above=0.0, maxval=1.0)
        self.min_speed_conf = config.getfloat('min_speed', 0.0, minval=0.0, maxval=1.0)
        self.max_speed = self.max_speed_conf
        self.min_speed = self.min_speed_conf

        # Target and baseline
        self.target_delta_conf = config.getfloat('target_delta', DEFAULT_TARGET_DELTA_PA)
        self.target_delta = self.target_delta_conf
        self.baseline_pressure = config.getfloat('baseline_pressure', None)
        self.auto_set_baseline = config.getboolean('auto_set_baseline', False)

        # Control algorithm
        algos = {'watermark': ControlBangBang, 'pid': ControlPID}
        algo = config.getchoice('control', algos, default='pid')
        self.control = algo(self, config)

        # Timing
        self.sample_period = REPORT_TIME
        self.next_speed_time = 0.0
        self.last_speed_value = -1.0

        # GCODES
        gcode = self.printer.lookup_object('gcode')
        gcode.register_mux_command(
            'SET_PRESSURE_FAN_TARGET', 'PRESSURE_FAN', self.name,
            self.cmd_SET_PRESSURE_FAN_TARGET,
            desc=self.cmd_SET_PRESSURE_FAN_TARGET_help)
        gcode.register_mux_command(
            'SET_PRESSURE_BASELINE', 'PRESSURE_FAN', self.name,
            self.cmd_SET_PRESSURE_BASELINE,
            desc=self.cmd_SET_PRESSURE_BASELINE_help)
        gcode.register_mux_command(
            'QUERY_PRESSURE_FAN', 'PRESSURE_FAN', self.name,
            self.cmd_QUERY_PRESSURE_FAN,
            desc=self.cmd_QUERY_PRESSURE_FAN_help)

        # Connect/ready hooks
        self.printer.register_event_handler('klippy:connect', self._on_connect)
        self.printer.register_event_handler('klippy:ready', self._on_ready)

    # Event handlers
    def _on_connect(self):
        # Use sensor report period if available
        try:
            self.sample_period = float(self.sensor_obj.get_report_time_delta())
        except Exception:
            self.sample_period = REPORT_TIME

    def _on_ready(self):
        # Optionally set baseline at ready
        if self.baseline_pressure is None and self.auto_set_baseline:
            self.reactor.register_timer(self._deferred_baseline, self.reactor.NOW + 1.0)
        # Start control loop
        self.reactor.register_timer(self._control_timer, self.reactor.NOW + self.sample_period)

    def _deferred_baseline(self, eventtime):
        p = self._read_pressure()
        if p is not None:
            self.baseline_pressure = p
            logging.info("pressure_fan %s: Baseline set to %.2f Pa", self.name, p)
        return self.reactor.NEVER

    def _sensor_callback(self, read_time, temp):
        # We rely on our own timer loop; callback retained for completeness
        pass

    # Core operations
    def _read_pressure(self):
        try:
            # BME280/BMP280 extras expose .pressure in hPa; convert to Pa
            p_hpa = getattr(self.sensor_obj, 'pressure', None)
            if p_hpa is None:
                return None
            return float(p_hpa) * 100.0
        except Exception:
            logging.exception("pressure_fan %s: Failed reading pressure", self.name)
            return None

    def get_current_delta(self):
        p = self._read_pressure()
        if p is None or self.baseline_pressure is None:
            return None, None, None
        delta = self.baseline_pressure - p  # positive if chamber is under vacuum
        return p, self.baseline_pressure, delta

    def set_pf_speed(self, read_time, value):
        # Bound
        if value <= 0.0:
            value = 0.0
        elif value < self.min_speed:
            value = self.min_speed
        if ((read_time < self.next_speed_time or self.last_speed_value < 0.0)
                and abs(value - self.last_speed_value) < 0.05):
            return
        speed_time = read_time + self.sample_period
        self.next_speed_time = speed_time + 0.6
        self.last_speed_value = value
        self.fan.set_speed(value, speed_time)

    # Timer
    def _control_timer(self, eventtime):
        # Read pressure and invoke control
        p = self._read_pressure()
        if p is None:
            return eventtime + self.sample_period
        # Determine baseline
        if self.baseline_pressure is None:
            # No baseline yet; do nothing until operator sets it
            return eventtime + self.sample_period
        # Update control
        self.control.pressure_callback(eventtime, p)
        return eventtime + self.sample_period

    # External getters used by controllers
    def get_limits(self):
        return self.min_speed, self.max_speed
    def get_target_delta(self):
        return self.target_delta
    def get_pressure(self):
        # returns (current_pressure_Pa, baseline_Pa, current_delta_Pa)
        return self.get_current_delta()

    # GCode commands
    cmd_SET_PRESSURE_FAN_TARGET_help = (
        "Set target vacuum (delta Pa) and optional fan limits. "
        "Example: SET_PRESSURE_FAN_TARGET PRESSURE_FAN=%s TARGET_DELTA=20 MIN_SPEED=0 MAX_SPEED=1"
    )
    def cmd_SET_PRESSURE_FAN_TARGET(self, gcmd):
        target = gcmd.get_float('TARGET_DELTA', self.target_delta_conf)
        self.target_delta = target
        min_speed = gcmd.get_float('MIN_SPEED', self.min_speed)
        max_speed = gcmd.get_float('MAX_SPEED', self.max_speed)
        if min_speed > max_speed:
            raise self.printer.command_error(
                "Requested min speed (%.2f) is greater than max speed (%.2f)" % (min_speed, max_speed))
        self.min_speed = min_speed
        self.max_speed = max_speed

    cmd_SET_PRESSURE_BASELINE_help = (
        "Set or capture baseline pressure (absolute Pa). "
        "Call with BASELINE=<Pa> or omit to capture current sensor value."
    )
    def cmd_SET_PRESSURE_BASELINE(self, gcmd):
        if gcmd.get('BASELINE', None) is None:
            p = self._read_pressure()
            if p is None:
                raise self.printer.command_error("No pressure reading available to capture baseline")
            self.baseline_pressure = p
        else:
            self.baseline_pressure = gcmd.get_float('BASELINE')

    cmd_QUERY_PRESSURE_FAN_help = "Report current pressure, baseline, delta, target, and fan speed"
    def cmd_QUERY_PRESSURE_FAN(self, gcmd):
        p, base, delta = self.get_current_delta()
        speed = max(0.0, self.last_speed_value)
        if p is None:
            gcmd.respond_info("pressure_fan %s: No reading yet (baseline=%s)" % (self.name, base))
            return
        gcmd.respond_info(
            "pressure_fan %s: P=%.2f Pa, baseline=%.2f Pa, delta=%.2f Pa, target=%.2f Pa, speed=%.2f"
            % (self.name, p, base, delta, self.target_delta, speed))

    # Status
    def get_status(self, eventtime):
        p, base, delta = self.get_current_delta()
        return {
            'pressure': None if p is None else round(p, 2),
            'baseline': base,
            'delta': None if delta is None else round(delta, 2),
            'target_delta': self.target_delta,
            'fan': self.fan.get_status(eventtime),
        }


######################################################################
# Control algorithms
######################################################################

class ControlBangBang:
    def __init__(self, pressure_fan, config):
        self.pfan = pressure_fan
        self.max_delta = config.getfloat('max_delta', 3.0, above=0.0)
        self.active = False

    def pressure_callback(self, read_time, pressure_pa):
        _, _, delta = self.pfan.get_pressure()
        if delta is None:
            return
        target = self.pfan.get_target_delta()
        # Switch based on band around target
        if self.active and delta >= target + self.max_delta:
            # Too much vacuum -> stop/reduce fan
            self.active = False
        elif (not self.active) and delta <= target - self.max_delta:
            # Not enough vacuum -> run fan
            self.active = True
        speed = self.pfan.max_speed if self.active else 0.0
        self.pfan.set_pf_speed(read_time, speed)


class ControlPID:
    def __init__(self, pressure_fan, config):
        self.pfan = pressure_fan
        self.Kp = config.getfloat('pid_Kp', 10.0) / PID_PARAM_BASE
        self.Ki = config.getfloat('pid_Ki', 0.0) / PID_PARAM_BASE
        self.Kd = config.getfloat('pid_Kd', 0.0) / PID_PARAM_BASE
        self.min_deriv_time = config.getfloat('pid_deriv_time', 2.0, above=0.0)
        self.integ_max = self.pfan.max_speed / self.Ki if self.Ki else 0.0
        self.prev_err = 0.0
        self.prev_time = 0.0
        self.prev_deriv = 0.0
        self.prev_integ = 0.0

    def pressure_callback(self, read_time, pressure_pa):
        _, _, delta = self.pfan.get_pressure()
        if delta is None:
            return
        target = self.pfan.get_target_delta()
        err = target - delta  # positive => increase vacuum (more fan)
        dt = read_time - self.prev_time
        # Derivative
        derr = err - self.prev_err
        if dt >= self.min_deriv_time:
            deriv = derr / dt
        else:
            deriv = (self.prev_deriv * (self.min_deriv_time - dt) + derr) / max(self.min_deriv_time, 1e-6)
        # Integral
        integ = self.prev_integ + err * max(dt, 0.0)
        if self.Ki:
            integ = max(0.0, min(self.integ_max, integ))
        # Output (direct-acting: more error -> more speed)
        co = self.Kp * err + self.Ki * integ + self.Kd * deriv
        min_spd, max_spd = self.pfan.get_limits()
        bounded = max(0.0, min(max_spd, co))
        self.pfan.set_pf_speed(read_time, max(min_spd, bounded))
        # Save state
        self.prev_err = err
        self.prev_time = read_time
        self.prev_deriv = deriv
        if co == bounded:
            self.prev_integ = integ


def load_config_prefix(config):
    return PressureFan(config)
