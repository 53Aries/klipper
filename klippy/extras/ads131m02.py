# ADS131M02 Support
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
from . import bulk_sensor, bus


# Constants
BYTES_PER_SAMPLE = 4
UPDATE_INTERVAL = 0.10


class ADS131M02:
    def __init__(self, config):
        self.printer = printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.last_error_count = 0
        self.consecutive_fails = 0
        # Safe gating: allow defining the section without touching MCU
        self.enabled = config.getboolean('enabled', default=True)
        # Minimal config surface
        self.channel = config.getint('channel', 0, minval=0, maxval=1)
        self.sps = config.getint('sps', 500, minval=10, maxval=20000)
        # SPI/Pin Setup only if enabled
        if not self.enabled:
            # Create minimal placeholders to avoid later attribute errors
            self.spi = None
            self.mcu = printer.lookup_object('mcu')
            self.oid = self.mcu.create_oid()
            self.data_ready_pin = None
        else:
            # Default speed: 1 MHz (ADS131M0x supports higher; increase as needed)
            self.spi = bus.MCU_SPI_from_config(config, 1, default_speed=1000000)
            self.mcu = mcu = self.spi.get_mcu()
            self.oid = mcu.create_oid()
            # Data Ready (DRDY) Pin
            drdy_pin = config.get('data_ready_pin')
            ppins = printer.lookup_object('pins')
            drdy_ppin = ppins.lookup_pin(drdy_pin)
            self.data_ready_pin = drdy_ppin['pin']
            drdy_pin_mcu = drdy_ppin['chip']
            if drdy_pin_mcu != self.mcu:
                raise config.error("ADS131M02 config error: SPI communication and"
                                   " data_ready_pin must be on the same MCU")

        # Bulk Sensor Setup
        self.bulk_queue = bulk_sensor.BulkDataQueue(self.mcu, oid=self.oid)
        chip_smooth = max(100, self.sps) * UPDATE_INTERVAL
        self.ffreader = bulk_sensor.FixedFreqReader(mcu, chip_smooth, "<i")
        self.batch_bulk = bulk_sensor.BatchBulkHelper(
            self.printer, self._process_batch, self._start_measurements,
            self._finish_measurements, UPDATE_INTERVAL)

        # Command Configuration
        self.attach_probe_cmd = None
        if self.enabled:
            mcu.add_config_cmd(
                "config_ads131m02 oid=%d spi_oid=%d data_ready_pin=%s channel=%d"
                % (self.oid, self.spi.get_oid(), self.data_ready_pin, self.channel))
            # Avoid any on-restart commands to keep MCU startup clean
            mcu.add_config_cmd("query_ads131m02 oid=%d rest_ticks=0"
                               % (self.oid,))
            mcu.register_config_callback(self._build_config)
            self.query_ads131m02_cmd = None
        else:
            # Register config callback minimally (no MCU commands when disabled)
            mcu.register_config_callback(self._build_config)
            self.query_ads131m02_cmd = None

    def _build_config(self):
        cq = self.spi.get_command_queue()
        self.query_ads131m02_cmd = self.mcu.lookup_command(
            "query_ads131m02 oid=%c rest_ticks=%u", cq=cq)
        self.attach_probe_cmd = self.mcu.lookup_command(
            "ads131m02_attach_load_cell_probe oid=%c load_cell_probe_oid=%c")
        self.ffreader.setup_query_command("query_ads131m02_status oid=%c",
                                          oid=self.oid, cq=cq)

    def get_mcu(self):
        return self.mcu

    def get_samples_per_second(self):
        return self.sps

    def get_range(self):
        # 24-bit signed range
        return -0x800000, 0x7FFFFF

    def add_client(self, callback):
        self.batch_bulk.add_client(callback)

    def attach_load_cell_probe(self, load_cell_probe_oid):
        self.attach_probe_cmd.send([self.oid, load_cell_probe_oid])

    # Measurement decoding
    def _convert_samples(self, samples):
        adc_factor = 1. / (1 << 23)
        count = 0
        for ptime, val in samples:
            samples[count] = (round(ptime, 6), val, round(val * adc_factor, 9))
            count += 1
        del samples[count:]

    # Start, stop, and process message batches
    def _start_measurements(self):
        if not self.enabled:
            logging.info("ADS131M02 '%s' is disabled; skipping start", self.name)
            return
        self.last_error_count = 0
        self.consecutive_fails = 0
        # Initialize chip with fixed MODE and OSR from SPS
        self.reset_chip()
        self.setup_chip()
        rest_ticks = self.mcu.seconds_to_clock(1. / (10. * max(1, self.sps)))
        self.query_ads131m02_cmd.send([self.oid, rest_ticks])
        logging.info("ADS131M02 starting '%s' measurements", self.name)
        self.ffreader.note_start()

    def _finish_measurements(self):
        if self.printer.is_shutdown() or not self.enabled:
            return
        self.query_ads131m02_cmd.send_wait_ack([self.oid, 0])
        self.ffreader.note_end()
        logging.info("ADS131M02 finished '%s' measurements", self.name)

    def _process_batch(self, eventtime):
        samples = self.ffreader.pull_samples()
        self._convert_samples(samples)
        return {'data': samples, 'errors': self.last_error_count,
                'overflows': self.ffreader.get_last_overflows()}

    # --- Chip initialization helpers (optional) ---
    # No raw init groups; driver uses fixed init

    # --- Low-level SPI helpers for 24-bit word commands ---
    def _w24(self, val):
        val &= 0xFFFFFF
        return [ (val >> 16) & 0xFF, (val >> 8) & 0xFF, val & 0xFF ]

    def _send_words24(self, words):
        payload = []
        for w in words:
            payload.extend(self._w24(w))
        if payload:
            self.spi.spi_send(payload)

    def _transfer_words24(self, words):
        payload = []
        for w in words:
            payload.extend(self._w24(w))
        if not payload:
            return []
        params = self.spi.spi_transfer(payload)
        resp = params.get('response', [])
        # Pack every 3 bytes into a 24-bit word
        words24 = []
        for i in range(0, len(resp), 3):
            if i + 2 >= len(resp):
                break
            w = (resp[i] << 16) | (resp[i+1] << 8) | resp[i+2]
            words24.append(w)
        return words24
    # No hex override helpers in minimal driver

    # --- Command builders per ADS131M02 ---
    # 16-bit opcodes placed in 24-bit word by left-shifting 8 bits.
    def _cmd_word(self, cmd16):
        return (cmd16 & 0xFFFF) << 8

    def _wreg_word(self, addr, count):
        # WREG (011 aaaaaa nnnnnnnn) => use 5 address bits
        cmd16 = (0b011 << 13) | ((addr & 0x1F) << 8) | ((count - 1) & 0xFF)
        return self._cmd_word(cmd16)

    def _rreg_word(self, addr, count):
        cmd16 = (0b101 << 13) | ((addr & 0x1F) << 8) | ((count - 1) & 0xFF)
        return self._cmd_word(cmd16)

    def _rreg(self, addr, count):
        # Issue RREG and clock out data with NULL words; take last 'count' words
        words = [self._rreg_word(addr, count)] + [0] * (count + 1)
        resp = self._transfer_words24(words)
        if len(resp) < (count + 2):
            return []
        data_words = resp[-count:]
        # Convert 24-bit word to 16-bit register value (bits 23:8)
        regs = [ (w >> 8) & 0xFFFF for w in data_words ]
        return regs

    def _verify_reg(self, addr, expected, mask=None):
        vals = self._rreg(addr, 1)
        if not vals:
            raise self.printer.command_error(
                f"ADS131M02 {self.name}: no response reading reg 0x{addr:02x}")
        actual = vals[0] & 0xFFFF
        if mask is None:
            ok = (actual == (expected & 0xFFFF))
        else:
            ok = ((actual & mask) == (expected & mask))
        if not ok:
            raise self.printer.command_error(
                "Failed to set ADS131M02 register [0x%x] to 0x%04x: got 0x%04x"
                % (addr, expected & 0xFFFF, actual))

    def reset_chip(self):
        # RESET command (0011h) must complete a full 4-word frame
        RESET = 0x0011
        self._send_words24([self._cmd_word(RESET), 0, 0, 0])

    def setup_chip(self):
        # Fixed MODE: WLENGTH=24-bit, defaults for DRDY
        mode_val = (0b01 << 8)
        self._send_words24([self._wreg_word(0x02, 1), (mode_val & 0xFFFF) << 8, 0, 0])
        # CLOCK: choose OSR nearest to requested sps
        osr_choices = [128, 256, 512, 1024, 2048, 4096, 8192, 16256]
        target = max(128, min(16256, self.sps))
        nearest = min(osr_choices, key=lambda x: abs(x - target))
        osr_code = {128:0,256:1,512:2,1024:3,2048:4,4096:5,8192:6,16256:7}[nearest]
        clock_val = (osr_code & 0x7) << 2
        self._send_words24([self._wreg_word(0x03, 1), (clock_val & 0xFFFF) << 8, 0, 0])
        # Wake from standby and begin conversions (0033h)
        WAKEUP = 0x0033
        self._send_words24([self._cmd_word(WAKEUP), 0, 0, 0])


ADS131M02_SENSOR_TYPE = {"ads131m02": ADS131M02}

# Klipper module entry point for the [ads131m02] config section
def load_config(config):
    return ADS131M02(config)
