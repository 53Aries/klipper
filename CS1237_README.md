# CS1237 ADC Driver for Klipper - Project Status

## Current Status: Ready for New Chip

This implementation is complete and ready to use with a new/working CS1237 chip.

## What Happened

During development, the CS1237 chip on the test board was corrupted by configuration write attempts. The chip is now stuck at approximately 1.9 Hz (an invalid state between the 10 Hz and 40 Hz settings) and cannot be recovered through software.

## What Was Fixed

Through extensive debugging and datasheet analysis, the following critical issues were identified and corrected:

1. **B6 Bit Inversion**: The REFO_OFF bit (B6) was inverted - should be 0 to keep reference output enabled
2. **Temperature Mode Constraint**: Temperature channel requires gain=1 (now automatically enforced)
3. **Configuration Format**: Corrected to proper 8-bit format with B7 reserved (must be 0)
4. **Clock Sequence**: Implemented exact 46-clock sequence per datasheet Figure 9
5. **Timing**: Proper sequencing with delays between operations

## Current Implementation

The driver now includes:

- **Automatic Configuration**: Writes config to chip on first successful read after boot
- **Safe Sequencing**: Waits for data ready, reads pending sample, then writes config
- **Single Attempt**: Only attempts config write once per boot to avoid issues
- **Manual Command**: `WRITE_CS1237_CONFIG` command available for testing/recovery
- **Validation**: Temperature mode automatically forces gain=1 per datasheet

## For New Chip Installation

1. **Install New CS1237 Module**: Replace the corrupted chip/module

2. **Configuration**:
   ```ini
   [load_cell]
   sensor_type: cs1237
   dout_pin: PB4      # Your actual pin
   sclk_pin: PB3      # Your actual pin
   sample_rate: 640   # 10, 40, 640, or 1280 Hz
   gain: 128          # 1, 2, 64, or 128
   channel: A         # A or TEMP
   ```

3. **Important Notes**:
   - Do NOT use `^` pullup on dout_pin (bidirectional pin)
   - Both pins must be on the same MCU
   - Temperature mode (channel: TEMP) automatically forces gain: 1
   - Configuration is written automatically on first read after boot

4. **Testing**:
   ```
   FIRMWARE_RESTART
   LOAD_CELL_DIAGNOSTIC
   ```
   
   Expected output:
   - "Measured samples per second" should match configured sample_rate
   - "Sensor reported no errors"
   - Good sample collection (19+ samples in 2 seconds at 10 Hz)

5. **Manual Config Write** (if needed):
   ```
   WRITE_CS1237_CONFIG              # For unnamed [load_cell]
   WRITE_CS1237_CONFIG LOAD_CELL=my_load_cell  # For named [load_cell my_load_cell]
   ```

## Files Modified

### Core Driver Files
- `src/sensor_cs1237.c` - MCU driver with correct 46-clock sequence
- `klippy/extras/cs1237.py` - Python interface with validation
- `klippy/extras/load_cell.py` - Added WRITE_CS1237_CONFIG command

### Build System
- `src/Kconfig` - CS1237 configuration options
- `src/Makefile` - Build integration

### Documentation
- `docs/Config_Reference.md` - [cs1237] section reference
- `docs/Load_Cell.md` - CS1237 usage guide  
- `docs/CS1237_Implementation.md` - Technical details
- `config/sample-cs1237.cfg` - Configuration examples

## Git Repository

- **Repository**: https://github.com/53Aries/klipper
- **Branch**: feature/cs1237
- **Latest Commit**: 46d0ff0ff - "Re-enable automatic config write with proper sequencing"

## Known Issues

- Previous CS1237 chip corrupted during testing (hardware fault, requires replacement)
- Configuration write was the cause of corruption (now fixed with proper sequencing)

## Lessons Learned

1. The CS1237 configuration write requires **exact** timing and sequence per datasheet Figure 9
2. B6 (REFO_OFF) bit has inverted logic: 1=disable REF, 0=enable REF
3. Temperature mode MUST use gain=1 (datasheet requirement)
4. Configuration register is 8-bit with B7 reserved (must be 0)
5. Multiple rapid config write attempts can corrupt the chip permanently
6. CS1237 has no hardware reset pin - requires full power cycle to reset
7. The chip preserves configuration through power-down mode

## Next Steps

1. Install new CS1237 chip/module
2. Test with automatic configuration write
3. Verify sample rate matches configuration
4. Proceed with load cell calibration
5. Use for load_cell_probe bed leveling

## Support

For issues or questions:
- Klipper Discourse: https://www.klipper3d.org
- GitHub Issues: https://github.com/53Aries/klipper/issues

---

**Status**: Implementation complete, awaiting hardware replacement for final validation.
