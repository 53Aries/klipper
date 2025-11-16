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
        # Config
        # Channel selection: 0 or 1 (default CH1 differential bridge)
        self.channel = config.getint('channel', 1, minval=0, maxval=1)
        # Sampling rate (used for timing/smoothing; device ODR left at defaults)
        self.sps = config.getint('sample_rate', 2000, minval=1, maxval=64000)
        # PGA Gain (matches ads1220 style options)
        self.gain_options = {'1': 0, '2': 1, '4': 2, '8': 3, '16': 4, '32': 5, '64': 6, '128': 7}
        self.gain = config.getchoice('gain', self.gain_options, default='128')
        # ADS1220-like options (optional): MODE(WLENGTH/CRC/DRDY), CLOCK(OSR)
        self.enable_ads1220_style = config.getboolean('enable_ads1220_style', default=False)
        # WLENGTH[1:0] at MODE bits 9:8: 00=16, 01=24(def), 10=32(LSB zero), 11=32(sign extend)
        self.word_len_options = {'16': 0b00, '24': 0b01, '32z': 0b10, '32s': 0b11, '32': 0b11}
        self.word_length = None
        try:
            wl = config.get('word_length', default="").strip()
            if wl:
                self.word_length = self.word_len_options[str(int(wl))]
        except Exception:
            logging.warning("ADS131M02 '%s': invalid word_length ignored", self.name)
            self.word_length = None
        self.input_crc = config.getboolean('input_crc', default=False)
        self.crc_type_options = {'16': 0, '32': 1}
        self.crc_type = None
        try:
            ct = config.get('crc_type', default="").strip()
            if ct:
                self.crc_type = self.crc_type_options[str(int(ct))]
        except Exception:
            logging.warning("ADS131M02 '%s': invalid crc_type ignored", self.name)
            self.crc_type = None
        # DRDY options (MODE bits 3:2,1,0)
        self.drdy_sel_options = {'lagging': 0b00, 'or': 0b01, 'leading': 0b10, 'leading2': 0b11}
        self.drdy_sel = None
        try:
            dsel = config.get('drdy_sel', default="").strip().lower()
            if dsel:
                self.drdy_sel = self.drdy_sel_options[dsel]
        except Exception:
            logging.warning("ADS131M02 '%s': invalid drdy_sel ignored", self.name)
            self.drdy_sel = None
        self.drdy_fmt_options = {'level': 0, 'pulse': 1}
        self.drdy_fmt = None
        try:
            dfmt = config.get('drdy_fmt', default="").strip().lower()
            if dfmt:
                self.drdy_fmt = self.drdy_fmt_options[dfmt]
        except Exception:
            logging.warning("ADS131M02 '%s': invalid drdy_fmt ignored", self.name)
            self.drdy_fmt = None
        self.drdy_hiz = None
        try:
            # allow true/false strings
            dh = config.get('drdy_hiz', default="").strip().lower()
            if dh in ('true','false','1','0','yes','no'):
                self.drdy_hiz = 1 if dh in ('true','1','yes') else 0
        except Exception:
            logging.warning("ADS131M02 '%s': invalid drdy_hiz ignored", self.name)
            self.drdy_hiz = None
        # OSR mapping – CLOCK OSR[2:0] at bits 4:2
        self.osr_map = {'128': 0, '256': 1, '512': 2, '1024': 3, '2048': 4, '4096': 5, '8192': 6, '16256': 7}
        self.osr_code = None
        try:
            osr = config.get('osr', default="").strip()
            if osr:
                self.osr_code = self.osr_map[str(int(osr))]
        except Exception:
            logging.warning("ADS131M02 '%s': invalid osr ignored", self.name)
            self.osr_code = None
        # Optional register init similar to ADS1220
        # If you set enable_register_init: true, you must also provide
        # a valid init sequence for your setup (see reset_chip/setup_chip).
        self.enable_register_init = config.getboolean('enable_register_init',
                                                      default=False)
        # Optional explicit register values (16-bit) for WREG writes.
        # Values are left-shifted into a 24-bit word as per device framing.
        # If provided, they are written after RESET and before WAKEUP.
        self.mode_reg = self._parse_hex_opt(config.get('mode_reg', default=""))
        self.clock_reg = self._parse_hex_opt(config.get('clock_reg', default=""))
        self.ch1_cfg_reg = self._parse_hex_opt(config.get('ch1_cfg_reg', default=""))
        # Optional raw init sequence: semicolon or pipe separated groups of
        # comma-separated hex bytes, e.g. "0x65,0x00|0x40,0x01,0x02,0x03"
        self.init_sequence = config.get('init_sequence', default="").strip()
        # SPI Setup
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
        chip_smooth = self.sps * UPDATE_INTERVAL * 2
        self.ffreader = bulk_sensor.FixedFreqReader(mcu, chip_smooth, "<i")
        self.batch_bulk = bulk_sensor.BatchBulkHelper(
            self.printer, self._process_batch, self._start_measurements,
            self._finish_measurements, UPDATE_INTERVAL)

        # Command Configuration
        self.attach_probe_cmd = None
        mcu.add_config_cmd(
            "config_ads131m02 oid=%d spi_oid=%d data_ready_pin=%s channel=%d"
            % (self.oid, self.spi.get_oid(), self.data_ready_pin, self.channel))
        mcu.add_config_cmd("query_ads131m02 oid=%d rest_ticks=0"
                           % (self.oid,), on_restart=True)
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
        self.last_error_count = 0
        self.consecutive_fails = 0
        # Initialize chip (if requested) then start bulk reading
        if self.enable_register_init:
            self.reset_chip()
            self.setup_chip()
        rest_ticks = self.mcu.seconds_to_clock(1. / (10. * self.sps))
        self.query_ads131m02_cmd.send([self.oid, rest_ticks])
        logging.info("ADS131M02 starting '%s' measurements", self.name)
        self.ffreader.note_start()

    def _finish_measurements(self):
        if self.printer.is_shutdown():
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
    def _parse_init_groups(self):
        if not self.init_sequence:
            return []
        groups = []
        for grp in self.init_sequence.replace(";", "|").split("|"):
            grp = grp.strip()
            if not grp:
                continue
            bytestr = [b.strip() for b in grp.split(',') if b.strip()]
            try:
                groups.append([int(x, 0) for x in bytestr])
            except Exception:
                raise self.printer.config_error(
                    f"{self.name}: invalid init_sequence group: '{grp}'")
        return groups

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
    def _parse_hex_opt(self, s):
        s = (s or '').strip()
        if not s:
            return None
        try:
            # Accept forms like "0x1234" or "4660"
            return int(s, 0) & 0xFFFF
        except Exception:
            logging.warning("ADS131M02 '%s': invalid hex '%s' ignored", self.name, s)
            return None

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
        # Prefer user-provided init sequence if present
        if self.init_sequence:
            for group in self._parse_init_groups():
                self.spi.spi_send(group)
            return
        # RESET command (0011h) must complete a full 4-word frame
        RESET = 0x0011
        self._send_words24([self._cmd_word(RESET), 0, 0, 0])

    def setup_chip(self):
        if self.init_sequence:
            return
        # Optional explicit register programming before generic gain and wake
        writes = []
        verifies = []
        # ADS1220-style mapped registers (if enabled and not overridden)
        if self.enable_ads1220_style and self.mode_reg is None:
            mode_val = 0
            # Warning: MCU reader expects 24-bit words; ignore non-24 WLENGTH
            if self.word_length is not None:
                if self.word_length != 0b01:
                    logging.warning("ADS131M02 '%s': only 24-bit word_length is supported by MCU reader; ignoring WLENGTH!=24", self.name)
                else:
                    mode_val |= (self.word_length & 0x3) << 8  # WLENGTH[1:0] bits 9:8
            if self.input_crc:
                mode_val |= (1 << 12)  # RX_CRC_EN bit12
                if self.crc_type is not None:
                    mode_val |= ((self.crc_type & 0x1) << 11)  # CRC_TYPE bit11
            if self.drdy_sel is not None:
                mode_val |= (self.drdy_sel & 0x3) << 2  # DRDY_SEL[1:0] bits3:2
            if self.drdy_hiz is not None:
                mode_val |= (1 << 1) if self.drdy_hiz else 0  # DRDY_HIZ bit1
            if self.drdy_fmt is not None:
                mode_val |= (self.drdy_fmt & 0x1)  # DRDY_FMT bit0
            writes.extend([self._wreg_word(0x02, 1), (mode_val & 0xFFFF) << 8, 0, 0])
            # Verify only the bits we explicitly set
            mode_mask = 0
            mode_mask |= (0x3 << 8) if (self.word_length == 0b01) else 0
            mode_mask |= (1 << 12) if self.input_crc else 0
            mode_mask |= (1 << 11) if (self.input_crc and self.crc_type is not None) else 0
            mode_mask |= (0x3 << 2) if self.drdy_sel is not None else 0
            mode_mask |= (1 << 1) if self.drdy_hiz is not None else 0
            mode_mask |= (1 << 0) if self.drdy_fmt is not None else 0
            if mode_mask:
                verifies.append((0x02, mode_val, mode_mask))
        if self.enable_ads1220_style and self.clock_reg is None and self.osr_code is not None:
            clock_val = (self.osr_code & 0x7) << 2  # OSR[2:0] bits4:2
            writes.extend([self._wreg_word(0x03, 1), (clock_val & 0xFFFF) << 8, 0, 0])
            verifies.append((0x03, clock_val, (0x7 << 2)))
        if self.mode_reg is not None:
            writes.extend([self._wreg_word(0x02, 1), (self.mode_reg & 0xFFFF) << 8, 0, 0])
            verifies.append((0x02, self.mode_reg & 0xFFFF, None))
        if self.clock_reg is not None:
            writes.extend([self._wreg_word(0x03, 1), (self.clock_reg & 0xFFFF) << 8, 0, 0])
            verifies.append((0x03, self.clock_reg & 0xFFFF, None))
        if self.ch1_cfg_reg is not None:
            writes.extend([self._wreg_word(0x0E, 1), (self.ch1_cfg_reg & 0xFFFF) << 8, 0, 0])
            verifies.append((0x0E, self.ch1_cfg_reg & 0xFFFF, None))
        if writes:
            self._send_words24(writes)
            # Readback verify
            for addr, exp, mask in verifies:
                self._verify_reg(addr, exp, mask)
        # Program CH1 gain via GAIN (addr=0x04): PGAGAIN1[2:0] at bits 6:4
        gain_code = self.gain_options[str(self.gain)]
        gain_val = (gain_code & 0x7) << 4
        # WREG for GAIN1 with one data word, pad to 4-word frame
        self._send_words24([self._wreg_word(0x04, 1), (gain_val & 0xFFFF) << 8, 0, 0])
        # Verify gain bits (mask 0x0070)
        self._verify_reg(0x04, gain_val, 0x0070)
        # Wake from standby and begin conversions (0033h)
        WAKEUP = 0x0033
        self._send_words24([self._cmd_word(WAKEUP), 0, 0, 0])


ADS131M02_SENSOR_TYPE = {"ads131m02": ADS131M02}
