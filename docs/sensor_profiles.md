# Sensor profiles

The firmware has **no per-sensor code**. A sensor is a `SensorProfile` (bus parameters, `init_ops`, per-cycle `ops`,
`decode` fields, optional `calc` RPN steps, `checks`, `output_kinds`). The eight proposed sensors are authored in
`tools/sensor_profiles.py` (readable Python, compiled to Protobuf; `python tools/sensor_profiles.py A 1` writes
configuration A). Configuration A = DHT22 + GY-511 + LD2450 + RC522; B = HC-SR04 + BME280 + MH-Z19 + MAX6675.

> **Status: written from datasheet knowledge and host-tested against simulated devices; seven of the eight profiles were then
> observed working on real boards (GY-511, DHT22, BME280, MAX6675, MH-Z19B, LD2450, RC522). The HC-SR04 profile is team-reported
> only. A line-by-line check of every register, timing, polarity and scale against each datasheet has not been done, so verify
> them (ideally with a logic analyzer) before relying on any number.**

| id | Sensor | Bus / pins (design notes) | Period | Outputs (kind) | Decoding notes | Open risks |
|---|---|---|---|---|---|---|
| 1 | GY-511 (LSM303DLHC accel, 0x19) | I2C SDA8 SCL9, 400 kHz | 20 ms | ax, ay, az in g (`ACCEL_X/Y/Z`) | 12-bit left-justified two's complement, 1 mg/LSB (+-2 g); init writes CTRL_REG1_A=0x57 | 50 Hz needs 20 ms I2C reads while other buses share the task; verify sensitivity/orientation |
| 2 | DHT22 | GPIO6 | 2 s | humidity %, temperature C | 40-bit frame, sign-magnitude temperature, checksum = sum of 4 bytes; start pulse 2 ms low | bit capture is bit-banged with interrupts off (~5 ms); RMT would be more robust |
| 3 | LD2450 radar | UART1 TX17 RX18, 256000 baud | 100 ms | x mm, y mm, speed cm/s (target 1, `OTHER`) | sync on `AA FF 03 00`, tail `55 CC`, "top bit set = positive" sign convention | convention and frame layout unverified; only the first target is decoded |
| 4 | RC522 | SPI SCLK12 MOSI11 MISO13 CS14, 1 MHz | 200 ms | card_present (0/1), ATQA (`OTHER`) | antenna on, REQA, read FIFO level and answer, antenna off; **init writes RFCfgReg = 0x70 (receiver gain)**: the clone chip (VersionReg 0xB2) never saw a card without it | detects that a card is present, not whose it is; `rc522_version()` in `sensor_profiles.py` reads only the version register |
| 5 | BME280 (0x76) | I2C SDA8 SCL9 | 2 s | temperature C, pressure Pa, humidity % | calibration read by `init_ops`; Bosch floating-point compensation as 21 RPN steps (163 tokens) | host check: 25.08 C / 100653.27 Pa on Bosch's example; humidity only vs my transcription |
| 6 | HC-SR04 | GPIO TRIG4 ECHO5 | 100 ms | distance cm | 10 us trigger, echo width -> cm = us*0.01715 | **ECHO is 5 V: level-shift or divide before the ESP32-S3 pin**; blocking up to 30 ms |
| 7 | MH-Z19 | UART2 TX15 RX16, 9600 baud | 2 s | CO2 ppm | command 0x86, 9-byte reply, checksum = -(sum bytes 1..7) | warm-up time and calibration behaviour not modelled; **needs 5 V, TX/RX crossed** (a wrong wiring only shows as READ_TIMEOUT); wired to UART1 17/18 on this board (`config_shorthand.WIRING`), the profile default is UART2 15/16 |
| 8 | MAX6675 | SPI CS10 (shared SCLK/MISO) | 250 ms | temperature C | 16-bit word, bits 14..3 at 0.25 C | open-thermocouple bit (D2) is not checked |

Notes
- Pin numbers come from the design notes; `cm_pin_allowed` refuses 0, 3, 19, 20, 26-37, 43-46 (flash, octal PSRAM, USB,
  console, strapping) and pins shared between two running profiles. Check against your actual board.
- One profile is active per bus type at a time; replacing a bus is one `ConfigApply`; other buses keep running.
- The ML selector requires a model's sample period to match the sensor period (GY-511 20 ms for the motion model,
  DHT22/BME280 2000 ms for the environment model).
- Known limits of the decode language: no conditionals, no integer-wraparound arithmetic, no multi-frame state.
