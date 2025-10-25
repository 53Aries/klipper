# Pressure-based exhaust fan (pressure_fan)

Control an exhaust fan to maintain a slight negative pressure (vacuum) in an enclosure using a BMx280 (BMP280/BME280/BMP388) pressure sensor.

This module reads absolute pressure from a BMx280 sensor, captures a baseline at rest, and controls a fan so the delta (baseline − current) tracks a target in Pascals. It offers three control modes; the recommended simple default is a stepwise controller that uses a robust 60s windowed average.

## Requirements
- A supported Bosch BMx280 series pressure sensor wired via I2C.
- The BMx280 sensor configured in the same section using Klipper's standard sensor keys.
- A fan output (any PWM-capable fan pin).

Tip: For stable behavior with slow, low-noise decisions, prefer high pressure oversampling and a small or disabled chip IIR filter:
- bme280_oversample_pressure: 5  (16×)
- bme280_iir_filter: 0            (off)

## Example configuration

```
[pressure_fan exhaust]
# Fan wiring
# Option A: Use an existing fan so UI shows a slider/percent (recommended)
fan: exhaust                 # references [fan_generic exhaust] below
# Notes:
# - Works with [fan], [fan_generic], [heater_fan], [controller_fan], [temperature_fan]
# - The name after the section must match (e.g., [fan_generic exhaust] → fan: exhaust)
# Option B: control the pin directly in this section (omit 'fan:' and set pin)
# pin: <your_fan_pin>
# Optional PWM tuning
# cycle_time: 0.010
# hardware_pwm: False
off_below: 0.20         # many fans won't spin below ~20%
kick_start_time: 0.5    # give a brief full-speed kick to start rotation
start_full_speed: true  # when a positive target is set, start at 100% then step down

# Control mode (step|pid|watermark). Recommended: step
control: step

# Target vacuum (Pascals, positive values)
# 0.0 keeps fan off until set at runtime via gcode
target_delta: 10.0

# Fan speed bounds while under control
min_speed: 0.0
max_speed: 1.0

# Stepwise controller parameters
# Decision cadence and robust averaging window
sample_window_sec: 60.0
drop_outliers: 1           # drop min/max before averaging
decision_period: 60.0      # adjust at most once per minute
band_pa: 0.5               # hold zone around target
step_speed: 0.05           # speed nudge per decision

# Baseline handling
# Set at runtime (recommended) or auto-capture at ready
# auto_set_baseline: true

# BMx280 sensor configured inline via standard sensor interface
sensor_type: bme280
# Common I2C options (adjust per your MCU/board)
# i2c_mcu: mcu
# i2c_bus: i2c1a
# i2c_address: 0x76

# Recommended chip filtering/oversampling
bme280_oversample_pressure: 5   # 16×
bme280_iir_filter: 0            # off

# Example [fan_generic] so Mainsail shows the fan slider
[fan_generic exhaust]
pin: <your_fan_pin>
min_speed: 0.0
max_speed: 1.0
off_below: 0.20
kick_start_time: 0.5
```

Notes:
- Place the sensor in the enclosure and ensure it’s not directly in the exhaust jet.
- Capture a baseline when the enclosure is at rest: `SET_PRESSURE_BASELINE PRESSURE_FAN=exhaust`.
- Then set a target vacuum (for example 10 Pa): `SET_PRESSURE_FAN_TARGET PRESSURE_FAN=exhaust TARGET_DELTA=10`.

Troubleshooting first run:
- The step controller adjusts slowly (default: ±0.05 every 60s) and many fans won’t start at 5% duty. To verify wiring quickly, try:
  - `SET_PRESSURE_FAN_TARGET PRESSURE_FAN=exhaust TARGET_DELTA=20 MIN_SPEED=0.25`
  - Or temporarily reduce `decision_period` and/or increase `step_speed` in the config, then restart.
- Ensure a baseline is set (auto or via `SET_PRESSURE_BASELINE`). If baseline isn’t set, the controller won’t act.

## G-code commands

- SET_PRESSURE_FAN_TARGET PRESSURE_FAN=<name> TARGET_DELTA=<Pa> [MIN_SPEED=<0..1>] [MAX_SPEED=<0..1>]
  - Sets the target vacuum and optional speed limits.

- SET_PRESSURE_BASELINE PRESSURE_FAN=<name> [BASELINE=<Pa>]
  - Capture the current absolute pressure as baseline when BASELINE is omitted, or set an explicit absolute pressure.

- SET_PRESSURE_DEADBAND PRESSURE_FAN=<name> DEADBAND=<Pa>
  - For PID mode only: ignore errors within ±DEADBAND to prevent dithering.

- QUERY_PRESSURE_FAN PRESSURE_FAN=<name>
  - Reports the current absolute pressure, baseline, raw delta, optional 60s windowed average, target, and fan speed.

## Control modes

- step (recommended): Once per `decision_period`, compute a trimmed mean of the last `sample_window_sec` of deltas (drop `drop_outliers` from each end after sorting). If outside ±`band_pa`, nudge fan by `step_speed`. Simple and robust.
- pid: Classic PID on delta error. Supports `pid_Kp`, `pid_Ki`, `pid_Kd` (scaled by 255 internally), optional `pid_deriv_time`, and `control_deadband`.
- watermark: Bang-bang around a band defined by `max_delta`.

## Best practices
- Use higher oversampling and minimal IIR on the sensor; let the long window average handle noise.
- Start with a small `band_pa` (0.5–1.0 Pa) and small `step_speed` (0.03–0.07) for gentle adjustments.
- If the enclosure is very leaky or fan is coarse, increase `decision_period` and `step_speed` slightly.

## Show delta on the Temperatures panel (optional)

You can graph the current pressure delta (Pa) directly on the Temperatures panel by adding a virtual temperature sensor that reports the delta value (displayed in °C but numerically equal to Pa):

```
[temperature_sensor delta_pa]
sensor_type: PRESSURE_DELTA
pressure_fan: exhaust       # name of your [pressure_fan]
mode: window                # window|raw
scale: 1.0                  # multiply (e.g., 0.1 to show deci-Pa)
offset: 0.0                 # add offset if desired
report_time: 0.8
```

After a restart, “delta_pa” will appear in the Temperatures list; the number is in Pa (units shown by the UI will be °C).

