# Control an exhaust fan using pressure readings (e.g. BMP280/BME280) to maintain slight vacuum
#
# Copyright (C) 2025  53Aries
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
from collections import deque
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
        self.section = config.get_name()
        self.name = self.section.split()[1]
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
        # Default target 0.0 so it stays off until the user sets a target
        self.target_delta_conf = config.getfloat('target_delta', 0.0)
        self.target_delta = self.target_delta_conf
        self.baseline_pressure = config.getfloat('baseline_pressure', None)
        self.auto_set_baseline = config.getboolean('auto_set_baseline', False)
        # Control algorithm
        algos = {'watermark': ControlBangBang, 'pid': ControlPID, 'step': ControlStepwise}
        algo = config.getchoice('control', algos, default='pid')
        self.control = algo(self, config)
        # Timing
        self.sample_period = REPORT_TIME
        self.next_speed_time = 0.0
        self.last_speed_value = -1.0
        # Slow-window sampling for simple, robust control
        self.sample_window_sec = config.getfloat('sample_window_sec', 60.0, above=5.0)
        self.drop_outliers = config.getint('drop_outliers', 1, minval=0)
        self._delta_samples = deque()  # (eventtime, delta)
        # Startup behavior: optionally start at full speed when a positive target is set
        self.start_full_speed = config.getboolean('start_full_speed', True)
        # Optional control deadband to ignore tiny errors without adding lag
        self.control_deadband = config.getfloat('control_deadband', 0.0, minval=0.0)
        # (filtering controls removed for simplicity)

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
        # (old filter/PID autotune commands removed)
        # Runtime control deadband adjustment
        gcode.register_mux_command(
            'SET_PRESSURE_DEADBAND', 'PRESSURE_FAN', self.name,
            self.cmd_SET_PRESSURE_DEADBAND,
            desc=self.cmd_SET_PRESSURE_DEADBAND_help)

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
        if self.target_delta <= 0.0:
            value = 0.0
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
        # Maintain window of raw delta samples for robust averaging
        raw_delta = self.baseline_pressure - p
        self._append_delta_sample(eventtime, raw_delta)
        # Update control
        # Convert reactor time to the fan MCU's print_time domain
        fan_mcu = self.fan.get_mcu()
        read_time = fan_mcu.estimated_print_time(eventtime)
        self.control.pressure_callback(read_time, p)
        return eventtime + self.sample_period

    # Sampling helpers for robust window average
    def _append_delta_sample(self, eventtime, delta):
        try:
            self._delta_samples.append((float(eventtime), float(delta)))
            # prune old
            cutoff = float(eventtime) - float(self.sample_window_sec)
            while self._delta_samples and self._delta_samples[0][0] < cutoff:
                self._delta_samples.popleft()
        except Exception:
            pass

    def get_window_delta(self):
        """Return (avg_delta, count) over the window after dropping outliers.
        Returns (None, 0) if not enough samples.
        """
        now = self.reactor.monotonic()
        cutoff = now - float(self.sample_window_sec)
        # prune
        while self._delta_samples and self._delta_samples[0][0] < cutoff:
            self._delta_samples.popleft()
        vals = [d for (t, d) in self._delta_samples if t >= cutoff]
        n = len(vals)
        k = min(self.drop_outliers, n // 3)  # avoid dropping all
        min_needed = max(3, 2 * k + 3)
        if n < min_needed:
            return None, n
        vals.sort()
        trimmed = vals[k:n - k] if k > 0 else vals
        if not trimmed:
            return None, n
        avg = sum(trimmed) / float(len(trimmed))
        return avg, n

    # External getters used by controllers
    def get_limits(self):
        return self.min_speed, self.max_speed
    def get_target_delta(self):
        return self.target_delta
    def get_pressure(self):
        # returns (current_pressure_Pa, baseline_Pa, raw_delta_Pa)
        return self.get_current_delta()
    def alter_target_delta(self, value):
        self.target_delta = value
    def set_control(self, control):
        old = self.control
        self.control = control
        return old

    # GCode commands
    cmd_SET_PRESSURE_FAN_TARGET_help = (
        "Set target vacuum (delta Pa) and optional fan limits. "
        "Example: SET_PRESSURE_FAN_TARGET PRESSURE_FAN=%s TARGET_DELTA=20 MIN_SPEED=0 MAX_SPEED=1"
    )
    def cmd_SET_PRESSURE_FAN_TARGET(self, gcmd):
        target = gcmd.get_float('TARGET_DELTA', self.target_delta_conf)
        prev_target = self.target_delta
        self.target_delta = target
        min_speed = gcmd.get_float('MIN_SPEED', self.min_speed)
        max_speed = gcmd.get_float('MAX_SPEED', self.max_speed)
        if min_speed > max_speed:
            raise self.printer.command_error(
                "Requested min speed (%.2f) is greater than max speed (%.2f)" % (min_speed, max_speed))
        self.min_speed = min_speed
        self.max_speed = max_speed
        # If we just enabled control (target moved from <=0 to >0), start at full speed
        if self.start_full_speed and prev_target <= 0.0 and self.target_delta > 0.0:
            try:
                eventtime = self.reactor.monotonic()
                read_time = self.fan.get_mcu().estimated_print_time(eventtime)
                self.set_pf_speed(read_time, self.max_speed)
            except Exception:
                logging.exception("pressure_fan %s: failed to start at full speed", self.name)

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

    cmd_QUERY_PRESSURE_FAN_help = "Report current pressure, baseline, delta, windowed average, target, and fan speed"
    def cmd_QUERY_PRESSURE_FAN(self, gcmd):
        # Raw values for visibility
        p, base, raw_delta = self.get_current_delta()
        # Windowed average value
        win_avg, win_n = self.get_window_delta()
        speed = max(0.0, self.last_speed_value)
        if p is None:
            gcmd.respond_info("pressure_fan %s: No reading yet (baseline=%s)" % (self.name, base))
            return
        gcmd.respond_info(
            "pressure_fan %s: P=%.2f Pa, baseline=%.2f Pa, delta=%.2f Pa%s, target=%.2f Pa, speed=%.2f"
            % (
                self.name, p, base, raw_delta,
                (", window=%.2f Pa (n=%d)" % (win_avg, win_n)) if win_avg is not None else "",
                self.target_delta, speed,
            )
        )

    # Old SET_PRESSURE_FILTER removed for simplicity

    # Old PRESSURE_PID_CALIBRATE removed for simplicity

    # Control deadband runtime setter
    cmd_SET_PRESSURE_DEADBAND_help = (
        "Set a control deadband in Pascals; errors within ±DEADBAND are ignored by PID. "
        "Example: SET_PRESSURE_DEADBAND PRESSURE_FAN=%s DEADBAND=0.5" % ('%s')
    )
    def cmd_SET_PRESSURE_DEADBAND(self, gcmd):
        db = gcmd.get_float('DEADBAND', None)
        if db is not None:
            self.control_deadband = max(0.0, float(db))
        gcmd.respond_info("pressure_fan %s: control_deadband=%.3f Pa" % (self.name, self.control_deadband))

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
        # (old filter calibration removed)


######################################################################
# Control algorithms
######################################################################

class ControlStepwise:
    """Simple slow step controller.
    Once per decision_period, compute a robust window-averaged delta and
    nudge the fan speed up or down by step_speed if outside a band around
    the target. Keeps behavior responsive enough without complex filters.
    """
    def __init__(self, pressure_fan, config):
        self.pfan = pressure_fan
        self.band_pa = config.getfloat('band_pa', 0.5, above=0.0)
        self.step_speed = config.getfloat('step_speed', 0.05, above=0.0)
        self.decision_period = config.getfloat('decision_period', 60.0, above=1.0)
        self._last_decision = 0.0

    def pressure_callback(self, read_time, pressure_pa):
        # Only decide at the requested cadence and once we have enough samples
        now = self.pfan.reactor.monotonic()
        if (now - self._last_decision) < self.decision_period:
            return
        avg, n = self.pfan.get_window_delta()
        if avg is None:
            return
        target = self.pfan.get_target_delta()
        err = target - avg
        # Hold within band (and when target is 0 the fan logic will shut it off)
        if abs(err) <= self.band_pa:
            self._last_decision = now
            return
        cur = max(0.0, self.pfan.last_speed_value)
        new = cur + (self.step_speed if err > 0.0 else -self.step_speed)
        min_spd, max_spd = self.pfan.get_limits()
        new = max(0.0, min(max_spd, new))
        self.pfan.set_pf_speed(read_time, max(min_spd, new))
        self._last_decision = now

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
        # Apply deadband: ignore tiny errors to avoid dithering
        db = getattr(self.pfan, 'control_deadband', 0.0)
        if db > 0.0:
            if abs(err) <= db:
                err = 0.0
            else:
                err = err - db if err > 0.0 else err + db
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


######################################################################
# (PID autotune removed)


def load_config_prefix(config):
    return PressureFan(config)

