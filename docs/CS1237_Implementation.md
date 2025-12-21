# CS1237 ADC Driver for Klipper - Implementation Summary

This document summarizes the CS1237 ADC driver implementation for Klipper's load_cell_probe system.

## Files Created

### 1. Python Driver: `klippy/extras/cs1237.py`
- Main Python interface for the CS1237 ADC chip
- Implements bulk sensor framework integration
- Supports configurable sample rates: 10, 40, 640, 1280 Hz
- Supports configurable gains: 1, 2, 64, 128
- Channel selection: A (load cell) or TEMP (temperature sensor)
- Automatic error detection and recovery
- Load cell probe attachment support

### 2. MCU Driver: `src/sensor_cs1237.c`
- Low-level C driver for MCU communication
- Implements bit-banging protocol for CS1237
- 24-bit ADC data reading
- Configuration register programming
- Timer-based sampling with overflow detection
- Integration with sensor_bulk framework
- Load cell probe sample reporting

### 3. Build System Updates

#### `src/Makefile`
- Added: `src-$(CONFIG_WANT_CS1237) += sensor_cs1237.c`

#### `src/Kconfig`
Added four configuration sections:
- `config WANT_CS1237` - Main configuration flag
- Updated `NEED_SENSOR_BULK` dependencies
- Updated `WANT_LOAD_CELL_PROBE` dependencies
- Added menu entry for optional CS1237 support

### 4. Integration Updates

#### `klippy/extras/load_cell.py`
- Added `cs1237` import
- Added CS1237_SENSOR_TYPE to sensor options

#### `klippy/extras/load_cell_probe.py`
- Added `cs1237` import
- Added CS1237_SENSOR_TYPE to probe sensor options

### 5. Configuration Examples: `config/sample-cs1237.cfg`
Comprehensive examples including:
- Basic load cell configuration
- Load cell probe configuration
- High-speed configuration
- High-precision configuration
- Temperature monitoring configuration
- Detailed notes and troubleshooting guide

## CS1237 Specifications

### Sample Rates
- 10 Hz: Very stable, high precision
- 40 Hz: Balanced performance
- 640 Hz: Fast updates
- 1280 Hz: Maximum speed

### Gain Settings
- 1x: Large signals, temperature
- 2x: Moderate signals
- 64x: Good sensitivity
- 128x: Maximum sensitivity (typical for load cells)

### Channels
- A: Primary load cell input
- TEMP: Internal temperature sensor

## Protocol Details

The CS1237 uses a 2-wire synchronous serial interface:
- **DOUT**: Data output from ADC
- **SCLK**: Serial clock (controlled by MCU)

### Data Reading Sequence
1. Wait for DOUT to go low (data ready)
2. Clock out 24 data bits (MSB first)
3. Provide 1 additional clock pulse
4. Data is sign-extended to 32-bit integer

### Configuration Sequence
1. Send 36 clock pulses after data read
2. Wait for DOUT low (acknowledgment)
3. Send 7 configuration bits
4. Send 1 final clock pulse

## Key Features

### Error Detection
- Desynchronization detection
- Overflow detection (late reads)
- Automatic sensor restart on errors

### Integration
- Compatible with Klipper's bulk_sensor framework
- Full load_cell_probe support
- Automatic MCU time synchronization
- Batched data reporting

### Performance
- Low latency sampling
- Efficient bit-banging implementation
- AVR-optimized timing
- Minimal CPU overhead

## Usage Example

```ini
[load_cell_probe]
sensor_type: cs1237
dout_pin: ^PB7
sclk_pin: PB6
sample_rate: 640
gain: 128
channel: A
trigger_force: 75
force_safety_limit: 2000
```

## Calibration Procedure

1. Configure sensor in printer.cfg
2. Restart Klipper
3. Run: `LOAD_CELL_DIAGNOSTIC_MODE_ON`
4. Run: `LOAD_CELL_TARE` (with no load)
5. Apply known weight
6. Run: `LOAD_CELL_CALIBRATE GRAMS=<weight>`
7. Run: `SAVE_CONFIG`

## Comparison with Similar Chips

| Feature | CS1237 | HX711 | ADS1220 |
|---------|--------|-------|---------|
| Interface | 2-wire | 2-wire | SPI |
| Max Rate | 1280 Hz | 80 Hz | 2000 Hz |
| Gains | 1,2,64,128 | 32,64,128 | 1-128 |
| Bits | 24 | 24 | 24 |
| Channels | 2 | 3 | 4 |
| Temp Sensor | Yes | No | Yes |

## Testing Recommendations

1. **Wiring Test**: Verify DOUT/SCLK connections
2. **Basic Operation**: Check data stream with QUERY_LOAD_CELL
3. **Rate Testing**: Try different sample rates
4. **Gain Testing**: Verify proper gain scaling
5. **Load Testing**: Apply known weights
6. **Probe Testing**: Run test taps
7. **Long-term Stability**: Monitor for drift

## Troubleshooting

### No Data / Communication Errors
- Check pin assignments
- Verify MCU matches on both pins
- Check pullup on dout_pin (use ^)
- Verify CS1237 power supply

### Unstable Readings
- Lower sample_rate
- Check for electrical noise
- Verify proper grounding
- Check load cell wiring

### Saturated Readings
- Decrease gain
- Check load cell capacity
- Verify no mechanical binding

### Drift
- Enable drift filter
- Check temperature stability
- Verify load cell mounting

## License
GNU GPLv3 (same as Klipper)

## Credits
Based on the HX711/HX717 and ADS1220 implementations by Gareth Farrington
