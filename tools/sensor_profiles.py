"""The eight proposed sensors as configuration data (handoff section 2).

STATUS: authored from recalled datasheet/protocol knowledge. They are NOT verified against
the real datasheets or hardware yet - check every register, timing, polarity and scale before
the demo (see docs in STATUS.md). The firmware has no per-sensor code: each profile below is
just a ConfigApply payload built from generic ops, decode fields and RPN calc steps.

Configuration A: DHT22 + GY-511 + LD2450 + RC522
Configuration B: HC-SR04 + BME280 + MH-Z19 + MAX6675
Pins follow the design notes (GPIO6 DHT22; GPIO4/5 HC-SR04; I2C 8/9; UART1 17/18, UART2 15/16;
SPI 12/11/13, CS 14 RC522 / CS 10 MAX6675). Sensor ids are fixed so tests can refer to them.
"""
from profile_lib import Registers, add_calc, add_decode, add_init_op, add_op, load_pb

ID = {"gy511": 1, "dht22": 2, "ld2450": 3, "rc522": 4,
      "bme280": 5, "hcsr04": 6, "mhz19": 7, "max6675": 8}


def dht22(pb, sid=ID["dht22"]):
    """Humidity/temperature, 40-bit frame after a >=1 ms start pulse. Temperature is
    sign-magnitude; checksum is the sum of the first four bytes."""
    p = pb.SensorProfile(sensor_id=sid, bus=pb.BUS_GPIO, period_ms=2000, enabled=True, output_count=2)
    p.gpio.data_pin = 6
    r = Registers(["humidity_pct", "temp_c"])
    p.output_kinds.extend([pb.VALUE_HUMIDITY_PCT, pb.VALUE_TEMPERATURE_C])
    add_op(p, type=pb.OP_GPIO_PULSE, pin=6, level=False, delay_us=2000)
    add_op(p, type=pb.OP_GPIO_BIT_FRAME, pin=6, read_len=5, timeout_us=200, threshold_us=50)
    c = p.checks.add()
    c.kind, c.start, c.length, c.check_offset = pb.CHECK_SUM8, 0, 4, 4
    add_decode(p, r, "humidity_pct", src_offset=0, width=2, big_endian=True, scale=0.1)
    add_decode(p, r, "temp_c", src_offset=2, width=2, big_endian=True, sign=pb.SIGN_MAGNITUDE, scale=0.1)
    return p


def hcsr04(pb, sid=ID["hcsr04"]):
    """Ultrasonic distance: 10 us trigger on GPIO4, echo width measured on GPIO5 (level-shift ECHO!)."""
    p = pb.SensorProfile(sensor_id=sid, bus=pb.BUS_GPIO, period_ms=100, enabled=True, output_count=1)
    p.gpio.data_pin, p.gpio.aux_pin = 4, 5
    r = Registers(["distance_cm"])
    p.output_kinds.append(pb.VALUE_DISTANCE_CM)
    add_op(p, type=pb.OP_GPIO_PULSE, pin=4, level=True, delay_us=10)
    add_op(p, type=pb.OP_GPIO_PULSE_WIDTH, pin=5, level=True, timeout_us=30000)
    add_decode(p, r, "distance_cm", src_offset=0, width=4, big_endian=True, scale=0.01715)  # us * 0.0343 / 2
    return p


def gy511(pb, sid=ID["gy511"]):
    """LSM303DLHC accelerometer (0x19): 12-bit left-justified, +-2 g => 1 mg/LSB."""
    p = pb.SensorProfile(sensor_id=sid, bus=pb.BUS_I2C, period_ms=20, enabled=True, output_count=3)  # 50 Hz = motion model rate
    p.i2c.sda, p.i2c.scl, p.i2c.addr, p.i2c.clock_hz = 8, 9, 0x19, 400000
    r = Registers(["ax_g", "ay_g", "az_g"])
    p.output_kinds.extend([pb.VALUE_ACCEL_X_G, pb.VALUE_ACCEL_Y_G, pb.VALUE_ACCEL_Z_G])
    add_init_op(p, type=pb.OP_WRITE, tx=bytes([0x20, 0x57]))              # CTRL_REG1_A: 100 Hz, XYZ on
    add_op(p, type=pb.OP_WRITE_READ, tx=bytes([0xA8]), read_len=6)       # OUT_X_L_A with auto-increment
    for i, n in enumerate(["ax_g", "ay_g", "az_g"]):
        add_decode(p, r, n, src_offset=2 * i, width=2, shift=4, value_bits=12,
                   sign=pb.SIGN_TWOS_COMPLEMENT, scale=0.001)
    return p


def bme280(pb, sid=ID["bme280"]):
    """BME280 (0x76) with Bosch's floating-point compensation, entirely as config.
    Calibration is read from the device by init_ops and kept in registers."""
    p = pb.SensorProfile(sensor_id=sid, bus=pb.BUS_I2C, period_ms=2000, enabled=True, output_count=3)  # = environment model spacing
    p.i2c.sda, p.i2c.scl, p.i2c.addr, p.i2c.clock_hz = 8, 9, 0x76, 400000
    r = Registers(["temp_c", "pressure_pa", "humidity_pct"])
    p.output_kinds.extend([pb.VALUE_TEMPERATURE_C, pb.VALUE_PRESSURE_PA, pb.VALUE_HUMIDITY_PCT])

    add_init_op(p, type=pb.OP_WRITE, tx=bytes([0xF2, 0x01]))              # ctrl_hum: x1 (before ctrl_meas)
    add_init_op(p, type=pb.OP_WRITE, tx=bytes([0xF4, 0x27]))              # ctrl_meas: T x1, P x1, normal
    add_init_op(p, type=pb.OP_WRITE_READ, tx=bytes([0x88]), read_len=24)  # T1..T3, P1..P9 -> init[0..23]
    add_init_op(p, type=pb.OP_WRITE_READ, tx=bytes([0xA1]), read_len=1)   # H1            -> init[24]
    add_init_op(p, type=pb.OP_WRITE_READ, tx=bytes([0xE1]), read_len=7)   # H2..H6        -> init[25..31]
    add_op(p, type=pb.OP_WRITE_READ, tx=bytes([0xF7]), read_len=8)        # press[0..2] temp[3..5] hum[6..7]

    def init(name, off, width=2, **kw):
        add_decode(p, r, name, source=pb.SRC_INIT, src_offset=off, width=width, scale=1.0, **kw)

    s16 = dict(sign=pb.SIGN_TWOS_COMPLEMENT)
    init("T1", 0)
    init("T2", 2, **s16); init("T3", 4, **s16)
    init("P1", 6)
    for i, n in enumerate(["P2", "P3", "P4", "P5", "P6", "P7", "P8", "P9"]):
        init(n, 8 + 2 * i, **s16)
    init("H1", 24, 1)
    init("H2", 25, **s16)
    init("H3", 27, 1)
    init("H4a", 28, 1, **s16)                      # H4 = int8(0xE4)*16 + (0xE5 & 0x0F)
    init("H4b", 29, 1, value_bits=4)
    init("H5a", 30, 1, **s16)                      # H5 = int8(0xE6)*16 + (0xE5 >> 4)
    init("H5b", 29, 1, shift=4, value_bits=4)
    init("H6", 31, 1, **s16)

    def cycle(name, off, **kw):
        add_decode(p, r, name, src_offset=off, width=3, big_endian=True, shift=4, value_bits=20, scale=1.0)
    cycle("adcP", 0)
    cycle("adcT", 3)
    add_decode(p, r, "adcH", src_offset=6, width=2, big_endian=True, scale=1.0)

    for n in ("tfine", "v1", "v2"):
        r.alloc(n)
    c = lambda dest, expr: add_calc(pb, p, r, dest, expr)
    # temperature
    c("v1", "(adcT / 16384 - T1 / 1024) * T2")
    c("v2", "adcT / 131072 - T1 / 8192")
    c("v2", "v2 * v2 * T3")
    c("tfine", "v1 + v2")
    c("temp_c", "tfine / 5120")
    # pressure
    c("v1", "tfine / 2 - 64000")
    c("v2", "v1 * v1 * P6 / 32768")
    c("v2", "v2 + v1 * P5 * 2")
    c("v2", "v2 / 4 + P4 * 65536")
    c("v1", "(P3 * v1 * v1 / 524288 + P2 * v1) / 524288")
    c("v1", "(1 + v1 / 32768) * P1")
    c("pressure_pa", "(1048576 - adcP - v2 / 4096) * 6250 / v1")
    c("v1", "P9 * pressure_pa * pressure_pa / 2147483648")
    c("v2", "pressure_pa * P8 / 32768")
    c("pressure_pa", "pressure_pa + (v1 + v2 + P7) / 16")
    # humidity
    c("H4a", "H4a * 16 + H4b")
    c("H5a", "H5a * 16 + H5b")
    c("humidity_pct", "tfine - 76800")
    c("humidity_pct", "(adcH - (H4a * 64 + H5a / 16384 * humidity_pct)) * "
                      "(H2 / 65536 * (1 + H6 / 67108864 * humidity_pct * (1 + H3 / 67108864 * humidity_pct)))")
    c("humidity_pct", "humidity_pct * (1 - H1 * humidity_pct / 524288)")
    c("humidity_pct", "max(0, min(100, humidity_pct))")
    return p


def ld2450(pb, sid=ID["ld2450"]):
    """HLK-LD2450 radar, 30-byte frame AA FF 03 00 ... 55 CC. First target X/Y/speed use the
    'top bit set = positive' sign convention. UART sync discards partial frames."""
    p = pb.SensorProfile(sensor_id=sid, bus=pb.BUS_UART, period_ms=100, enabled=True, output_count=3)
    p.uart.port, p.uart.tx, p.uart.rx, p.uart.baud = 1, 17, 18, 256000
    r = Registers(["x_mm", "y_mm", "speed_cm_s"])
    # y_mm (distance straight ahead of the radar) is the "radar distance" the anomaly model can use
    p.output_kinds.extend([pb.VALUE_OTHER, pb.VALUE_RADAR_DISTANCE_MM, pb.VALUE_OTHER])
    add_op(p, type=pb.OP_UART_SYNC_READ, tx=bytes([0xAA, 0xFF, 0x03, 0x00]), read_len=30,
           tail=bytes([0x55, 0xCC]), timeout_us=200000)
    for i, n in enumerate(["x_mm", "y_mm", "speed_cm_s"]):
        add_decode(p, r, n, src_offset=4 + 2 * i, width=2, sign=pb.SIGN_MAGNITUDE_SET_IS_POSITIVE, scale=1.0)
    return p


def mhz19(pb, sid=ID["mhz19"]):
    """MH-Z19 CO2: command 0x86 -> 9-byte reply, CO2 ppm = bytes 2..3, checksum = -(sum 1..7)."""
    p = pb.SensorProfile(sensor_id=sid, bus=pb.BUS_UART, period_ms=2000, enabled=True, output_count=1)
    p.uart.port, p.uart.tx, p.uart.rx, p.uart.baud = 2, 15, 16, 9600
    r = Registers(["co2_ppm"])
    p.output_kinds.append(pb.VALUE_CO2_PPM)
    add_op(p, type=pb.OP_WRITE_READ, tx=bytes([0xFF, 0x01, 0x86, 0, 0, 0, 0, 0, 0x79]),
           read_len=9, timeout_us=200000)
    c = p.checks.add()
    c.kind, c.start, c.length, c.check_offset = pb.CHECK_SUM8_NEG, 1, 7, 8
    add_decode(p, r, "co2_ppm", src_offset=2, width=2, big_endian=True, scale=1.0)
    return p


def rc522(pb, sid=ID["rc522"]):
    """MFRC522 card detection on SPI (CS14). Outputs [card_present, atqa]:
    card_present 1 when a card answered the REQA request, else 0; atqa the card's 2-byte answer
    (e.g. 4 = MIFARE Classic 1K, 0x44 = NTAG/Ultralight), 0 when no card.
    Register address byte: write = (reg<<1)&0x7E, read = that | 0x80. Per cycle: antenna on, wait for
    the card to power up, REQA (0x26, 7-bit frame) via the Transceive command, read the FIFO, antenna
    off (so the card is reset before the next poll). 11 operations, about 7 ms blocking per poll.
    Hardware (node2, clone chip version 0xB2): with the default receiver gain no card was ever seen; with RFCfgReg = 0x70
    a card held on the reader answered with a 2-byte FIFO in every poll."""
    p = pb.SensorProfile(sensor_id=sid, bus=pb.BUS_SPI, period_ms=200, enabled=True, output_count=2)
    p.spi.sclk, p.spi.mosi, p.spi.miso, p.spi.cs, p.spi.clock_hz, p.spi.mode = 12, 11, 13, 14, 1000000, 0
    r = Registers(["card_present", "atqa"])
    p.output_kinds.extend([pb.VALUE_OTHER, pb.VALUE_OTHER])
    # init: wake, 100 % ASK modulation, CRC preset (+ maximum receiver gain)
    # No SoftReset command: on the clone chip tested (version 0xB2) it never completed and left the chip
    # stuck (CommandReg 0x1F, no command ever ran). CommandReg = 0 only clears power-down / receiver-off.
    add_init_op(p, type=pb.OP_WRITE, tx=bytes([0x02, 0x00]))           # CommandReg = Idle, wake from power-down
    add_init_op(p, type=pb.OP_DELAY, delay_us=10000)
    add_init_op(p, type=pb.OP_WRITE, tx=bytes([0x2A, 0x40]))           # TxASKReg = 100 % ASK
    add_init_op(p, type=pb.OP_WRITE, tx=bytes([0x22, 0x3D]))           # ModeReg = CRC preset 0x6363
    add_init_op(p, type=pb.OP_WRITE, tx=bytes([0x4C, 0x70]))           # RFCfgReg = receiver gain 48 dB: without it the clone chip (0xB2) never saw a card
    # cycle: antenna on (the card powers up from the field), REQA, read the answer, antenna off again.
    # The antenna stays off between polls, which resets any card in the field so each poll sees a fresh card.
    add_op(p, type=pb.OP_WRITE, tx=bytes([0x28, 0x83]))                # TxControlReg: antenna on
    add_op(p, type=pb.OP_DELAY, delay_us=6000)                         # card start-up (spec: > 5 ms)
    add_op(p, type=pb.OP_WRITE, tx=bytes([0x02, 0x00]))                # CommandReg = Idle
    add_op(p, type=pb.OP_WRITE, tx=bytes([0x14, 0x80]))                # FIFOLevelReg: flush
    add_op(p, type=pb.OP_WRITE, tx=bytes([0x12, 0x26]))                # FIFODataReg <- REQA
    add_op(p, type=pb.OP_WRITE, tx=bytes([0x02, 0x0C]))                # CommandReg = Transceive
    add_op(p, type=pb.OP_WRITE, tx=bytes([0x1A, 0x87]))                # BitFramingReg: StartSend, 7 bits
    add_op(p, type=pb.OP_DELAY, delay_us=1000)                         # wait for ATQA (arrives < 1 ms)
    add_op(p, type=pb.OP_WRITE_READ, tx=bytes([0x94, 0x00]), read_len=2)         # FIFOLevelReg -> raw[1]
    add_op(p, type=pb.OP_WRITE_READ, tx=bytes([0x92, 0x92, 0x00]), read_len=3)   # FIFO -> raw[3..4]
    add_op(p, type=pb.OP_WRITE, tx=bytes([0x28, 0x80]))                # antenna off
    add_decode(p, r, "level", src_offset=1, width=1, scale=1.0)         # FIFO level: 2 = a card answered
    add_decode(p, r, "atqa", src_offset=3, width=2, scale=1.0)          # little-endian
    add_calc(pb, p, r, "card_present", "max(0, 1 - abs(level - 2))")   # 1 only for exactly 2 bytes
    add_calc(pb, p, r, "atqa", "atqa * card_present")                  # 0 when no card (FIFO is then garbage)
    return p


def rc522_version(pb, sid=ID["rc522"]):
    """MFRC522 on SPI (CS14): reads VersionReg (0x37 -> address byte 0xEE); expect 0x91/0x92."""
    p = pb.SensorProfile(sensor_id=sid, bus=pb.BUS_SPI, period_ms=200, enabled=True, output_count=1)
    p.spi.sclk, p.spi.mosi, p.spi.miso, p.spi.cs, p.spi.clock_hz, p.spi.mode = 12, 11, 13, 14, 1000000, 0
    r = Registers(["version"])
    p.output_kinds.append(pb.VALUE_OTHER)
    add_op(p, type=pb.OP_WRITE_READ, tx=bytes([0xEE, 0x00]), read_len=2)
    add_decode(p, r, "version", src_offset=1, width=1, scale=1.0)
    return p


def max6675(pb, sid=ID["max6675"]):
    """MAX6675 (CS10): 16-bit word, temperature = bits 14..3 at 0.25 C/LSB. D2 (bit 2) set
    means open thermocouple - not checked yet."""
    p = pb.SensorProfile(sensor_id=sid, bus=pb.BUS_SPI, period_ms=250, enabled=True, output_count=1)
    p.spi.sclk, p.spi.mosi, p.spi.miso, p.spi.cs, p.spi.clock_hz, p.spi.mode = 12, 11, 13, 10, 1000000, 0
    r = Registers(["temp_c"])
    p.output_kinds.append(pb.VALUE_TEMPERATURE_C)
    add_op(p, type=pb.OP_READ, read_len=2)
    add_decode(p, r, "temp_c", src_offset=0, width=2, big_endian=True, shift=3, value_bits=12, scale=0.25)
    return p


ALL = {"dht22": dht22, "hcsr04": hcsr04, "gy511": gy511, "bme280": bme280,
       "ld2450": ld2450, "mhz19": mhz19, "rc522": rc522, "rc522_version": rc522_version, "max6675": max6675}


def config_a(pb, generation):
    m = pb.ConfigApply(generation=generation)
    m.profiles.extend([dht22(pb), gy511(pb), ld2450(pb), rc522(pb)])
    return m


def config_b(pb, generation):
    m = pb.ConfigApply(generation=generation)
    m.profiles.extend([hcsr04(pb), bme280(pb), mhz19(pb), max6675(pb)])
    return m


if __name__ == "__main__":
    # python tools/sensor_profiles.py <A|B|dht22|gy511|ld2450|rc522|bme280|hcsr04|mhz19|max6675> [generation] [--hex | --out FILE]
    #   --hex   prints the ConfigApply as hex (paste after "CFG " on the serial console)
    #   --out   writes the binary ConfigApply to a file (publish it with node_client.py config FILE)
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("what", help="A, B, or one sensor name: " + ", ".join(ALL))
    ap.add_argument("generation", nargs="?", type=int, default=1)
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--hex", action="store_true")
    g.add_argument("--out")
    a = ap.parse_args()
    pb = load_pb()
    key = a.what.lower()
    if key == "a":
        msg = config_a(pb, a.generation)
    elif key == "b":
        msg = config_b(pb, a.generation)
    elif key in ALL:
        msg = pb.ConfigApply(generation=a.generation)
        msg.profiles.append(ALL[key](pb))
    else:
        raise SystemExit(f"unknown '{a.what}'")
    data = msg.SerializeToString()
    if a.out:
        open(a.out, "wb").write(data)
        print(f"wrote {len(data)} bytes to {a.out}")
    elif a.hex:
        print(data.hex())
    else:
        raise SystemExit("choose --hex or --out FILE (a raw binary on stdout gets corrupted by PowerShell redirection)")