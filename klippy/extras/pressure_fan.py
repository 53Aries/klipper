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
        # PID autotune command
        gcode.register_mux_command(
            'PRESSURE_PID_CALIBRATE', 'PRESSURE_FAN', self.name,
            self.cmd_PRESSURE_PID_CALIBRATE,
            desc=self.cmd_PRESSURE_PID_CALIBRATE_help)

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
        # Update control
        # Convert reactor time to the fan MCU's print_time domain
        fan_mcu = self.fan.get_mcu()
        read_time = fan_mcu.estimated_print_time(eventtime)
        self.control.pressure_callback(read_time, p)
        return eventtime + self.sample_period

    # External getters used by controllers
    def get_limits(self):
        return self.min_speed, self.max_speed
    def get_target_delta(self):
        return self.target_delta
    def get_pressure(self):
        # returns (current_pressure_Pa, baseline_Pa, current_delta_Pa)
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

    # PID Autotune G-Code
    cmd_PRESSURE_PID_CALIBRATE_help = (
        "Run PID calibration for a pressure_fan. Optional args: "
        "BAND_PA=<Pa> MIN_CYCLES=<n> MAX_CYCLES=<n> MAX_TIME=<sec> "
        "SETTLE_SEC=<sec> FILTER_ALPHA=<0..1> PEAK_DEADBAND=<Pa>"
    )
    def cmd_PRESSURE_PID_CALIBRATE(self, gcmd):
        target = gcmd.get_float('TARGET_DELTA')
        write_file = gcmd.get_int('WRITE_FILE', 0)
        band_pa = gcmd.get_float('BAND_PA', TUNE_DELTA_PA)
        min_cycles = gcmd.get_int('MIN_CYCLES', 12)
        max_cycles = gcmd.get_int('MAX_CYCLES', 24)
        max_time = gcmd.get_float('MAX_TIME', 600.0)
        settle_sec = gcmd.get_float('SETTLE_SEC', 2.0)
        filter_alpha = gcmd.get_float('FILTER_ALPHA', 0.3)
        peak_deadband = gcmd.get_float('PEAK_DEADBAND', 0.3)
        # Install autotune controller
        calibrate = ControlAutoTune(self, target,
                                    band_pa=band_pa,
                                    min_cycles=min_cycles,
                                    max_cycles=max_cycles,
                                    max_time=max_time,
                                    settle_sec=settle_sec,
                                    filter_alpha=filter_alpha,
                                    peak_deadband=peak_deadband)
        old_control = self.set_control(calibrate)
        # Seed targets and start
        self.alter_target_delta(target)
        # Block until autotune completes
        reactor = self.printer.get_reactor()
        eventtime = reactor.monotonic()
        try:
            while calibrate.check_busy() and not self.printer.is_shutdown():
                eventtime = reactor.pause(eventtime + 1.0)
        finally:
            self.set_control(old_control)
        if write_file:
            calibrate.write_file('/tmp/pressure_tune.txt')
        cycles, elapsed, reason = calibrate.get_summary()
        Kp, Ki, Kd = calibrate.calc_final_pid()
        if Kp == 0.0 and Ki == 0.0 and Kd == 0.0:
            msg = ("PRESSURE_PID_CALIBRATE finished (%s, cycles=%d, elapsed=%.1fs) "
                   "but insufficient data collected. Try increasing MAX_TIME or MIN_CYCLES, "
                   "or adjust BAND_PA/PEAK_DEADBAND/SETTLE_SEC for clearer oscillations.")
            gcmd.respond_info(msg % (reason or 'stopped', cycles, elapsed))
            logging.info("Pressure PID Autotune finished without sufficient data: reason=%s cycles=%d elapsed=%.1fs",
                         reason, cycles, elapsed)
            return
        logging.info("Pressure PID Autotune final: Kp=%.3f Ki=%.3f Kd=%.3f (cycles=%d elapsed=%.1fs)",
                     Kp, Ki, Kd, cycles, elapsed)
        gcmd.respond_info(
            ("PID parameters: pid_Kp=%.3f pid_Ki=%.3f pid_Kd=%.3f\n"
             "Collected cycles: %d, elapsed: %.1fs. Use SAVE_CONFIG to persist these in your config.")
            % (Kp, Ki, Kd, cycles, elapsed))
        # Store results for SAVE_CONFIG
        configfile = self.printer.lookup_object('configfile')
        configfile.set(self.section, 'control', 'pid')
        configfile.set(self.section, 'pid_Kp', "%.3f" % (Kp,))
        configfile.set(self.section, 'pid_Ki', "%.3f" % (Ki,))
        configfile.set(self.section, 'pid_Kd', "%.3f" % (Kd,))

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


######################################################################
# PID Autotune (relay/oscillation method)
######################################################################

TUNE_DELTA_PA = 2.0

class ControlAutoTune:
    def __init__(self, pressure_fan, target_delta,
                 band_pa=TUNE_DELTA_PA, min_cycles=12, max_cycles=24,
                 max_time=600.0, settle_sec=2.0, filter_alpha=0.3,
                 peak_deadband=0.3):
        self.pfan = pressure_fan
        self.calibrate_delta = target_delta
        # Relay/band and timing configuration
        self.band_pa = max(0.1, float(band_pa))
        self.min_cycles = int(max(2, min_cycles))
        self.max_cycles = int(max(self.min_cycles, max_cycles))
        self.max_time = float(max_time)
        self.settle_sec = max(0.0, float(settle_sec))
        self.alpha = max(0.0, min(1.0, float(filter_alpha)))
        self.peak_deadband = max(0.0, float(peak_deadband))

        # State
        self.heating = False  # fan on => increasing vacuum
        self.last_switch_time = 0.0
        self.delta_filt = None
        self.peak = 0.0
        self.peak_time = 0.0
        self.peaks = []
        self.last_speed = -1.0
        self.start_time = 0.0
        self.done = False
        self.done_reason = ""

    def _maybe_init(self, read_time, delta):
        if self.start_time == 0.0:
            self.start_time = read_time
            self.last_switch_time = read_time
        if self.delta_filt is None:
            self.delta_filt = delta

    def pressure_callback(self, read_time, pressure_pa):
        # Measure current delta
        _, _, delta = self.pfan.get_pressure()
        if delta is None:
            return
        self._maybe_init(read_time, delta)
        # Exponential moving average to reduce jitter
        self.delta_filt = self.alpha * delta + (1.0 - self.alpha) * self.delta_filt

        # Symmetric relay thresholds around target
        upper = self.calibrate_delta + (self.band_pa * 0.5)
        lower = self.calibrate_delta - (self.band_pa * 0.5)

        # Only allow state changes after settle_sec
        can_switch = (read_time - self.last_switch_time) >= self.settle_sec
        if self.heating and can_switch and self.delta_filt >= upper:
            self.heating = False
            self.last_switch_time = read_time
            self._check_peaks()
        elif (not self.heating) and can_switch and self.delta_filt <= lower:
            self.heating = True
            self.last_switch_time = read_time
            self._check_peaks()

        # Track peaks with deadband to avoid noise triggering
        if self.heating:
            # Increasing vacuum -> track max delta
            if (self.delta_filt - self.peak) > self.peak_deadband or self.peak == 0.0:
                self.peak = self.delta_filt
                self.peak_time = read_time
        else:
            # Decreasing vacuum -> track min delta
            if (self.peak - self.delta_filt) > self.peak_deadband or self.peak == 0.0:
                self.peak = self.delta_filt
                self.peak_time = read_time

        # Command fan directly
        speed = self.pfan.max_speed if self.heating else 0.0
        if speed != self.last_speed:
            self.last_speed = speed
            self.pfan.set_pf_speed(read_time, speed)

    def _check_peaks(self):
        # Record last peak and reset tracker
        self.peaks.append((self.peak, self.peak_time))
        # Reset peak tracker
        self.peak = 0.0
        if len(self.peaks) >= 4:
            self._calc_pid(len(self.peaks) - 1)

    def _calc_pid(self, pos):
        # Use Astrom-Hagglund method on delta oscillation
        temp_diff = self.peaks[pos][0] - self.peaks[pos-1][0]
        time_diff = self.peaks[pos][1] - self.peaks[pos-2][1]
        amplitude = 0.5 * abs(temp_diff)
        if amplitude <= 0 or time_diff <= 0:
            return 0.0, 0.0, 0.0
        # Effective gain: oscillation driven by 0..max_speed
        Ku = 4. * self.pfan.max_speed / (3.1415926535 * amplitude)
        Tu = time_diff
        Ti = 0.5 * Tu
        Td = 0.125 * Tu
        Kp = 0.6 * Ku * PID_PARAM_BASE
        Ki = Kp / Ti
        Kd = Kp * Td
        logging.info("Pressure Autotune: raw=%f Ku=%f Tu=%f  Kp=%f Ki=%f Kd=%f",
                     temp_diff, Ku, Tu, Kp, Ki, Kd)
        return Kp, Ki, Kd

    def calc_final_pid(self):
        if len(self.peaks) < 2 * self.min_cycles:
            return 0.0, 0.0, 0.0
        cycle_times = [(self.peaks[pos][1] - self.peaks[pos-2][1], pos)
                       for pos in range(4, len(self.peaks))]
        if not cycle_times:
            return 0.0, 0.0, 0.0
        midpoint_pos = sorted(cycle_times)[len(cycle_times)//2][1]
        return self._calc_pid(midpoint_pos)

    def check_busy(self):
        # Continue until we have enough peaks, but also honor max_cycles and max_time
        if self.start_time and (self.peaks and len(self.peaks) >= 2 * self.max_cycles):
            self.done = True
            self.done_reason = "reached max cycles"
            return False
        now_cycles = len(self.peaks) // 2
        if now_cycles < self.min_cycles:
            # Not enough data yet
            return True
        # Enough cycles, but keep going if under max_cycles and under max_time to stabilize
        if self.start_time:
            elapsed = self.peaks[-1][1] - self.start_time
            if elapsed >= self.max_time:
                self.done = True
                self.done_reason = "reached max time"
                return False
        # If we have at least min_cycles, allow finish once next peak arrives
        if now_cycles < self.max_cycles:
            return True
        self.done = True
        self.done_reason = "completed cycles"
        return False

    def get_summary(self):
        cycles = len(self.peaks) // 2
        elapsed = 0.0
        if self.start_time and self.peaks:
            elapsed = max(0.0, self.peaks[-1][1] - self.start_time)
        return cycles, elapsed, (self.done_reason or "")

    # Optional logging
    def write_file(self, filename):
        try:
            with open(filename, 'w') as f:
                for p, t in self.peaks:
                    f.write(f"{t:.3f} {p:.3f}\n")
        except Exception:
            logging.exception("Pressure Autotune: failed writing file")

    # Control API compatibility shim
    def get_status(self, eventtime):
        return {}

    # Not used, but keep signature similar to other controls
    def get_temp(self, eventtime):
        _, _, delta = self.pfan.get_pressure()
        return delta or 0.0, self.calibrate_delta

    # Unused by autotune
    def set_tf_speed(self, *args, **kwargs):
        pass

    def temperature_callback(self, *args, **kwargs):
        pass

    # PID constants not used here


def load_config_prefix(config):
    return PressureFan(config)

