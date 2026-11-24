# Step-by-step guide: set up, MQTT, wiring, bring-up, demo and data collection

This is your to-do document. It is ordered: follow the phases in sequence and tick the boxes.
Everything the software can do has been checked on the PC and in an emulator; **nothing has been run on real hardware yet,
so this guide is also how you produce that evidence.** Things I could not verify myself are marked **[unverified]**.

Contents
1. What you need
2. Phase 1 - check the PC
3. Phase 2 - set up the MQTT broker
4. Phase 3 - wire the sensors to the ESP32-S3
5. Phase 4 - configure, build and flash
6. Phase 5 - bring up each sensor over the serial console (no Wi-Fi needed)
7. Phase 6 - connect the node to Wi-Fi and MQTT
8. Phase 7 - the demo over MQTT (configs, models, rules, bridge)
9. Phase 8 - collect your own ML data and retrain
10. Phase 9 - measurements and evidence to keep
11. Troubleshooting
12. Cheat sheets (commands, topics, pins)

---

## 1. What you need

**Hardware**
- [ ] ESP32-S3-N16R8 dev board (e.g. DevKitC-1 style, two USB-C ports) and a USB cable (data cable, not charge-only)
- [ ] Sensors: GY-511 (LSM303DLHC), BME280, DHT22 (module or bare), HC-SR04 (or HC-SR04P), LD2450, MH-Z19(B), RC522, MAX6675 + K-type thermocouple
- [ ] Breadboard(s), jumper wires, a multimeter
- [ ] For the HC-SR04 ECHO line: 1 kOhm and 2 kOhm resistors (a voltage divider), or a level shifter
- [ ] If your DHT22 is a bare 4-pin part (not a 3-pin module): one 4.7-10 kOhm resistor (pull-up)
- [ ] A second ESP32-S3 board **only** if you want to demo the node-to-node bridge

**PC** (already set up in this folder)
- [ ] Windows, PowerShell, Python venv in `.venv`, ESP-IDF 6.1 at `C:\esp\v6.1` with tools in `C:\Espressif`
- [ ] A Wi-Fi network that allows two devices to talk to each other (see Phase 2, "Network pitfalls")

---

## 2. Phase 1 - check the PC

Open PowerShell in the project folder and run:

```powershell
# 1. everything that can be tested without hardware (takes a few minutes)
.venv\Scripts\python.exe tools\run_all_tests.py

# 2. ESP-IDF works
. .\tools\idf_env.ps1
idf.py --version            # expect: ESP-IDF v6.1
```

- [ ] All suites print `0 failures` / `ALL HOST SUITES PASS`.
- [ ] `idf.py --version` prints `ESP-IDF v6.1`.


---

## 3. Phase 2 - set up the MQTT broker

MQTT needs a **broker** (a small server) on the laptop. The ESP32 and your Python tools both connect to it.
I have not installed Mosquitto here, so these steps follow its normal Windows install **[unverified]**.

### 3.1 Install Mosquitto
1. Download the Windows installer from https://mosquitto.org/download/ (64-bit) and install with the defaults
   (it installs to `C:\Program Files\mosquitto`).
2. Mosquitto 2.x only accepts connections from the same PC unless you configure it. Create the file
   `C:\Program Files\mosquitto\aai16.conf` (run your editor as Administrator) with exactly:

```
listener 1883
allow_anonymous true
```

   (`allow_anonymous true` is acceptable only on a private lab network. This build uses plain, unencrypted MQTT.)

### 3.2 Allow it through the Windows firewall (PowerShell **as Administrator**)
```powershell
New-NetFirewallRule -DisplayName "Mosquitto MQTT 1883" -Direction Inbound -Protocol TCP -LocalPort 1883 -Action Allow -Profile Any
```

### 3.3 Start the broker (keep this window open while you work)
```powershell
& "C:\Program Files\mosquitto\mosquitto.exe" -c "C:\Program Files\mosquitto\aai16.conf" -v
```
You should see `Opening ipv4 listen socket on port 1883`.

### 3.4 Find your laptop's IP address
```powershell
ipconfig
```
Use the **IPv4 Address of the Wi-Fi adapter** (for example `192.168.1.23`). This is what the ESP32 will connect to:
`mqtt://192.168.1.23:1883`.

### 3.5 Test the broker from the laptop
In a second PowerShell window:
```powershell
& "C:\Program Files\mosquitto\mosquitto_sub.exe" -h localhost -t "test/#" -v
```
In a third window:
```powershell
& "C:\Program Files\mosquitto\mosquitto_pub.exe" -h localhost -t test/hello -m "hi"
```
- [ ] The subscriber window prints `test/hello hi`. The broker window logs the connections.

### 3.5b Test it from the project's own client
```powershell
.venv\Scripts\python.exe tools\node_client.py --broker 127.0.0.1 --node node1 listen --seconds 5
```
It should run for 5 seconds without an error (nothing is published yet, so it prints nothing).

### 3.6 Network pitfalls (the usual reasons "it does not connect")
- The ESP32-S3 supports **2.4 GHz Wi-Fi only** (not 5 GHz). Use a 2.4 GHz network.
- University and public networks (eduroam, guest Wi-Fi) often block devices from talking to each other ("client
  isolation"). If the ESP32 cannot reach the laptop even with the right IP, use a **phone hotspot** or a home router:
  connect the laptop and the ESP32 to the same hotspot.
- If the Windows network is "Public", the firewall rule above uses `-Profile Any`, which covers it; switching the
  network to "Private" also helps.
- The laptop's IP can change; if the node stops connecting later, re-run `ipconfig` and update the node (Phase 6).

### 3.7 What the node publishes and listens to
All payloads are binary Protobuf, so `mosquitto_sub` shows unreadable bytes. Use `tools\node_client.py listen` to read them.

| Direction | Topic (node id `node1`) |
|---|---|
| to node | `aai16/node1/in/config`, `rules`, `bridge`, `model/chunk`, `model/control`, `command` |
| from node | `aai16/node1/out/config_ack`, `rules_ack`, `bridge_ack`, `model_chunk_ack`, `model_status`, `inference`, `heartbeat`, `notification`, `reading/<sensor id>` |

---

## 4. Phase 3 - wire the sensors to the ESP32-S3

### 4.1 Golden rules (read before you connect anything)
1. **Power off (unplug USB) while changing wires.**
2. **The ESP32-S3 GPIO pins are 3.3 V only. They are NOT 5 V tolerant.** Never connect a 5 V signal directly.
3. Connect **all grounds together** (every sensor GND to a GND pin of the ESP32).
4. Some sensors need 3.3 V and some need 5 V for their power pin:

| Sensor | Power pin to use | Why |
|---|---|---|
| GY-511, BME280, RC522, MAX6675, DHT22 | **3V3** | RC522 must NOT get 5 V; the others work at 3.3 V |
| LD2450, MH-Z19(B), HC-SR04 | **5V** (the board's 5V/VBUS pin, available when powered by USB) | these modules need 5 V supply; their serial/logic outputs are 3.3 V (LD2450, MH-Z19) except the HC-SR04 ECHO (see below) |

5. Pin labels on cheap modules vary. **Check the printed labels on YOUR module** against the tables below.
6. The numbers are **GPIO numbers** (GPIO4 = the pin labelled `4` on a DevKitC-1). Do not confuse them with board position.
7. Do not use GPIO 0, 3, 19, 20, 26-37, 43-46: they are reserved (flash, PSRAM, USB, console, boot straps). The firmware also refuses them.

### 4.2 The complete pin map

| Bus | ESP32-S3 pin | Used for |
|---|---|---|
| I2C | GPIO8 = SDA, GPIO9 = SCL | GY-511 **and** BME280 (different addresses, can be wired together) |
| UART1 | GPIO17 = ESP TX, GPIO18 = ESP RX | LD2450 |
| UART2 | GPIO15 = ESP TX, GPIO16 = ESP RX | MH-Z19(B) |
| SPI | GPIO12 = SCLK, GPIO11 = MOSI, GPIO13 = MISO | RC522 **and** MAX6675 (shared) |
| SPI chip selects | GPIO14 = RC522 CS, GPIO10 = MAX6675 CS | one each |
| GPIO | GPIO6 | DHT22 data |
| GPIO | GPIO4 = TRIG, GPIO5 = ECHO | HC-SR04 |

Only **one sensor per bus is active at a time** (that is the demo: configuration A uses DHT22, GY-511, LD2450, RC522;
configuration B uses HC-SR04, BME280, MH-Z19, MAX6675). You can keep all eight wired and switch by software.

### 4.3 Sensor by sensor

**GY-511 (accelerometer, I2C address 0x19)**

| GY-511 pin | Connect to |
|---|---|
| VCC | 3V3 |
| GND | GND |
| SDA | GPIO8 |
| SCL | GPIO9 |
| (DRDY, INT1, INT2, etc.) | leave unconnected |

**BME280 (I2C address 0x76)** - same two wires as the GY-511

| BME280 pin | Connect to |
|---|---|
| VCC / VIN | 3V3 |
| GND | GND |
| SDA | GPIO8 |
| SCL | GPIO9 |
| SDO | **GND** (this selects address 0x76; if SDO goes to 3V3 the address is 0x77 and the profile will not find it) |
| CSB | 3V3 (selects I2C mode) if the module exposes it |

Note: many cheap boards sold as "BME280" are really **BMP280** (no humidity). If humidity looks wrong, check the chip marking.

**DHT22 / AM2302 (temperature + humidity, GPIO6)**

| DHT22 pin | Connect to |
|---|---|
| VCC (+) | 3V3 |
| DATA (OUT) | GPIO6 |
| GND (-) | GND |
| (bare 4-pin part only) | a 4.7-10 kOhm resistor between DATA and 3V3. A 3-pin module already has it. |

**HC-SR04 (distance, TRIG GPIO4, ECHO GPIO5)** - **ECHO is a 5 V signal: protect the pin**

| HC-SR04 pin | Connect to |
|---|---|
| VCC | 5V |
| GND | GND |
| TRIG | GPIO4 (3.3 V is enough to trigger it) |
| ECHO | **through a voltage divider** to GPIO5: ECHO - 1 kOhm - (GPIO5 node) - 2 kOhm - GND |

The divider turns 5 V into about 3.3 V (5 x 2/3). If you have the 3.3 V-capable **HC-SR04P**, you can power it from 3V3 and
skip the divider.

**LD2450 (24 GHz radar, UART1)**

| LD2450 pin | Connect to |
|---|---|
| 5V | 5V |
| GND | GND |
| TX | GPIO18 (ESP RX) |
| RX | GPIO17 (ESP TX) |

TX and RX cross over: the sensor's TX goes to the ESP's RX. Baud rate is 256000 (set by the profile).

**MH-Z19B (CO2, UART2)**

| MH-Z19B pin | Connect to |
|---|---|
| Vin (5 V) | 5V |
| GND | GND |
| Tx | GPIO16 (ESP RX) |
| Rx | GPIO15 (ESP TX) |
| (PWM, HD, AOT) | leave unconnected |

Baud 9600. The sensor needs about 3 minutes of warm-up after power-on before readings are meaningful.

**RC522 (RFID, SPI, chip select GPIO14)** - **3.3 V only**

| RC522 pin | Connect to |
|---|---|
| 3.3V | 3V3 (**never 5 V**) |
| GND | GND |
| RST | 3V3 (keeps it out of reset) |
| SDA (this is the chip-select) | GPIO14 |
| SCK | GPIO12 |
| MOSI | GPIO11 |
| MISO | GPIO13 |
| IRQ | leave unconnected |

**MAX6675 (thermocouple amplifier, SPI, chip select GPIO10)** - read-only, no MOSI wire

| MAX6675 pin | Connect to |
|---|---|
| VCC | 3V3 |
| GND | GND |
| SCK | GPIO12 (shared with the RC522) |
| SO | GPIO13 (shared MISO) |
| CS | GPIO10 |
| T+ / T- | the K-type thermocouple (red = negative on standard K-type: follow the marking on the module) |

### 4.4 One-page wiring overview

```
                      ESP32-S3 (3.3 V logic)
   3V3 ---------------------------------------+---> GY-511 VCC, BME280 VCC, DHT22 VCC,
                                              |     RC522 3.3V (+RST), MAX6675 VCC
   5V  ---------------------------------------+---> LD2450 5V, MH-Z19B Vin, HC-SR04 VCC
   GND ---------------------------------------+---> ALL sensor grounds

   GPIO8  (SDA) ------- GY-511 SDA ------- BME280 SDA          (I2C)
   GPIO9  (SCL) ------- GY-511 SCL ------- BME280 SCL
   GPIO17 (TX)  ------- LD2450 RX      GPIO18 (RX) ------- LD2450 TX         (UART1)
   GPIO15 (TX)  ------- MH-Z19B Rx     GPIO16 (RX) ------- MH-Z19B Tx        (UART2)
   GPIO12 (SCLK) ----- RC522 SCK ----- MAX6675 SCK                            (SPI, shared)
   GPIO11 (MOSI) ----- RC522 MOSI
   GPIO13 (MISO) ----- RC522 MISO ---- MAX6675 SO
   GPIO14 (CS)   ----- RC522 SDA(CS)           GPIO10 (CS) ---- MAX6675 CS
   GPIO6         ----- DHT22 DATA (+ pull-up to 3V3 if bare part)
   GPIO4         ----- HC-SR04 TRIG
   GPIO5  <--[1k]--+-- HC-SR04 ECHO          (+ 2k from the GPIO5 node to GND)
                   |
                  [2k]
                   |
                  GND
```

### 4.5 Check before you power on
- [ ] No 5 V wire touches any GPIO pin (only the HC-SR04 ECHO, through the 1k/2k divider).
- [ ] RC522 power is on **3V3**, not 5V.
- [ ] Every sensor has a GND connection to the ESP32.
- [ ] LD2450 / MH-Z19B: TX goes to RX and RX goes to TX.
- [ ] BME280 SDO is on GND (address 0x76).
- [ ] Use the multimeter in continuity mode to check there is no short between 3V3 and GND.

**Tip:** if you are unsure, wire and test **one sensor at a time** (Phase 5 is designed for that).

---

## 5. Phase 4 - configure, build and flash

### 5.1 Plug the board in
A DevKitC-1 has two USB-C ports. Use the one labelled **UART** (it goes through a USB-to-serial chip). The firmware's
console and command line are on that port. The other port ("USB") is the chip's native USB, and the console does not
appear there with the default settings.

Find the COM port:
```powershell
Get-CimInstance Win32_SerialPort | Select-Object Name, DeviceID
```
(or Device Manager > Ports). Example: `COM5`.

### 5.2 Enter your settings (node id, Wi-Fi, broker)
```powershell
. .\tools\idf_env.ps1
idf.py set-target esp32s3
idf.py menuconfig
```
In the menu choose **AAI16 node** and set:
- `Node id` (e.g. `node1`; a second board needs a different id)
- `Wi-Fi SSID` and `Wi-Fi password` (leave the SSID empty for the serial-only bring-up in Phase 5)
- `MQTT broker URI` = `mqtt://<your laptop IP>:1883`

Press `S`/Enter to save, `Q` to quit. (These values are stored in the local, untracked `sdkconfig` file.)

### 5.3 Build and flash
```powershell
idf.py -p COM5 flash monitor
```
Replace `COM5` with your port. The first build takes several minutes. If flashing says "failed to connect", hold the
**BOOT** button, tap **RESET**, release BOOT and run the command again.

Leave the monitor open. To leave it later press `Ctrl+]`.

### 5.4 What a good start looks like
You should see lines like these (numbers differ):
```
I (xxx) esp_psram: Found 8MB PSRAM device
I (xxx) aai16: NVS ready: ESP_OK
I (xxx) aai16: node 'node1' booted: config generation 0, model none
W (xxx) aai16: no Wi-Fi SSID configured ... serial console only      <- only when the SSID is empty
```
- [ ] Board boots without `Guru Meditation` or reboot loops.
- [ ] In the monitor, type `HELP` + Enter: it lists the commands.

**Serial commands**

| Command | What it does |
|---|---|
| `SHOW` | state: generation, active sensor ids, rules, model |
| `WATCH <id>` / `WATCH ALL` / `WATCH OFF` | print readings (at most one line per sensor per second) |
| `CFG <hex>` | apply a configuration (hex-encoded Protobuf, see below) |
| `STOP` / `RUN` | pause / resume all sensor reading |
| `CLEAR` | stop every sensor and remove rules/bridge settings |
| `REBOOT` | restart the node |

---

## 6. Phase 5 - bring up each sensor over the serial console (no Wi-Fi needed)

Do this one sensor at a time. It proves each wire and each sensor profile on its own.

### 6.1 The method
1. On the laptop generate the configuration for one sensor as hex:
   ```powershell
   .venv\Scripts\python.exe tools\sensor_profiles.py gy511 1 --hex
   ```
   The last number (`1`) is the **generation** and must be **higher every time** you send a configuration.
   Check the current value with `SHOW` (`cfg_gen=...`) and use that + 1.
2. Copy the printed hex, then in the monitor type `CFG ` followed by the hex and press Enter. Expect `OK submitted`.
3. Type `SHOW`: the sensor id appears in `sensors=[...]`.
4. Type `WATCH <id>` (the id is in the table below). One line per second appears.
5. A sensor on a *different* bus can be added next (generation + 1) without disturbing the first.
   A sensor on the *same* bus replaces the previous one.

Reading lines look like this: `READ id=1 ok [0.012, -0.034, 1.004]`.
On failure: `READ id=1 bus_error` (or `timeout`, `bad_frame`). Typing `WATCH OFF` stops the output.

### 6.2 Order and expected results

| # | Sensor | Command to generate the hex | Watch | What a healthy reading looks like | If it fails |
|---|---|---|---|---|---|
| 1 | GY-511 | `... sensor_profiles.py gy511 <gen> --hex` | `WATCH 1` | `[ax, ay, az]` in g. At rest the three values together have a length of about 1.0 g (for example `[0.02, -0.01, 1.00]`). Tilt the board and the numbers move. | `bus_error`: SDA/SCL swapped, no power, or wrong address. A length far from 1.0 means the scale is wrong. |
| 2 | BME280 | `... bme280 <gen> --hex` | `WATCH 5` | `[temp C, pressure Pa, humidity %]`: room temperature, pressure roughly 95000-105000, humidity 20-80. | `bus_error`: SDO not on GND (address 0x77) or wiring. Odd humidity: it may be a BMP280. |
| 3 | DHT22 | `... dht22 <gen> --hex` | `WATCH 2` | `[humidity %, temp C]`, new value every 2 s. Breathe on it: both rise. | `timeout`: no pull-up / DATA not on GPIO6 / no power. `bad_frame`: noise or marginal timing. |
| 4 | HC-SR04 | `... hcsr04 <gen> --hex` | `WATCH 6` | `[distance cm]`, e.g. 30 with a hand 30 cm away. | `timeout`: ECHO divider wrong, no 5 V on VCC, or nothing in range. Note this replaces the DHT22 (same GPIO bus). |
| 5 | LD2450 | `... ld2450 <gen> --hex` | `WATCH 3` | `[x mm, y mm, speed cm/s]`; zeros when nothing is detected, numbers appear when you walk in front of it. | `timeout`/`bad_frame`: TX/RX swapped, no 5 V, wrong baud. |
| 6 | MH-Z19B | `... mhz19 <gen> --hex` | `WATCH 7` | `[CO2 ppm]`, roughly 400-1500 in a room (after about 3 minutes of warm-up). Breathing near it raises it. | `timeout`: TX/RX swapped or no 5 V. `bad_frame`: checksum failure. |
| 7 | RC522 | `... rc522 <gen> --hex` | `WATCH 4` | `[card_present, atqa]`: `[0, 0]` with no card, `[1, 4]` (MIFARE Classic 1K) or `[1, 68]` (NTAG) when a card is held on the reader. The older chip-version check is `rc522_version` (`[146]` or `[145]`). | Always `[0, 0]` or errors: CS/SCK/MOSI/MISO wrong, 5 V used, or card not close (< 3 cm). Card detection is NOT hardware-verified. |
| 8 | MAX6675 | `... max6675 <gen> --hex` | `WATCH 8` | `[temp C]`, around room temperature with the thermocouple attached; warms when you hold the tip. | `0.00` or `1023.75`: not wired / open circuit (this is not detected as an error yet). |

(The `...` stands for `.venv\Scripts\python.exe tools`.)

**Record for each sensor:** a line of the reading and whether it is plausible. Compare the numbers with a reference if you
have one (a thermometer, a ruler, a phone barometer). This is the evidence that the sensor profiles are correct. Any
number that is wrong tells you which profile needs fixing; send me the line and I will adjust `tools/sensor_profiles.py`.

### 6.3 Both alternatives per bus
After all eight work individually you can show the swap:
- GPIO bus: DHT22 (id 2) <-> HC-SR04 (id 6)
- I2C bus: GY-511 (id 1) <-> BME280 (id 5)
- UART: LD2450 (id 3) uses UART1 and MH-Z19B (id 7) uses UART2, but the config treats them as the one "UART" bus: the newer replaces the older.
- SPI bus: RC522 (id 4) <-> MAX6675 (id 8)

Whole sets: `sensor_profiles.py A <gen> --hex` (config A is about 650 hex characters) and `B` (about 3400).

---

## 7. Phase 6 - connect the node to Wi-Fi and MQTT

1. Start the broker (Phase 2, step 3.3) and keep the window open.
2. `idf.py menuconfig` > **AAI16 node**: set the Wi-Fi SSID, password and `mqtt://<laptop IP>:1883`.
3. Rebuild and flash: `idf.py -p COM5 flash monitor`.
4. In the monitor, look for:
   ```
   I (xxx) plat_net: got IP
   I (xxx) plat_net: MQTT connected
   ```
   and in the broker window a new client connecting.
5. From the laptop:
   ```powershell
   .venv\Scripts\python.exe tools\node_client.py --broker <laptop IP> --node node1 listen --seconds 30
   ```
   After at most 10 seconds you should see a `heartbeat` message (`node_id: "node1"`...).
- [ ] The heartbeat arrives. If not, see Troubleshooting ("MQTT").

---

## 8. Phase 7 - the demo over MQTT

> The full FlexiCargo demonstration, scene by scene with ready-made MQTTX messages, is in
> [demo_guide.md](demo_guide.md). This section shows the same features with the command-line client.

Replace `<ip>` with your laptop's IP. Run `listen` in a **second window** to watch the replies:
```powershell
.venv\Scripts\python.exe tools\node_client.py --broker <ip> --node node1 listen --seconds 600
```

Generation numbers: each configuration must use a number **higher** than the last one the node accepted (`SHOW` tells you).

### 8.1 Configuration A, swap one sensor, configuration B
```powershell
$py = ".venv\Scripts\python.exe"
& $py tools\node_client.py --broker <ip> --node node1 config A --gen 10      # expect ADMITTED then APPLIED
# swap only the I2C sensor GY-511 -> BME280 (the other buses keep running):
& $py tools\sensor_profiles.py bme280 11 --out bme.bin
& $py tools\node_client.py --broker <ip> --node node1 config bme.bin
& $py tools\node_client.py --broker <ip> --node node1 config B --gen 12
```
- [ ] `listen` shows `config_ack` ADMITTED and APPLIED for each, and `reading/<id>` messages from the new sensors.
- [ ] An old generation is refused with a reason: ... config A --gen 5 answers REASON_STALE_GENERATION and nothing changes.

### 8.2 Upload and activate the Motion model (Model A), choose its inputs
```powershell
& $py tools\node_client.py --broker <ip> --node node1 config A --gen 20          # GY-511 (id 1) must be active
& $py tools\node_client.py --broker <ip> --node node1 upload ml\artifacts\motion\motion.amdl --kind motion --transfer 1
& $py tools\node_client.py --broker <ip> --node node1 activate --kind motion --transfer 1 --gen 1
& $py tools\node_client.py --broker <ip> --node node1 inputs --kind motion --gen 2  1:0:0 1:1:1 1:2:2
```
(`1:0:0` means sensor 1, value 0, goes into model input slot 0, and so on.)
- [ ] After about 1.3 seconds (64 samples at 50 Hz) `listen` shows `inference` with a `predicted_class`
      (0 STATIONARY, 1 MOVING, 2 VIBRATING). Shake the board: it should change.
- [ ] **Wrong input test:** `... inputs --kind motion --gen 3  2:1:0 1:1:1 1:2:2` (a DHT22 temperature into an
      accelerometer slot) is **refused** with `VALUE_KIND_MISMATCH` and the model keeps running.
- [ ] **Corrupt upload test:** copy `motion.amdl`, flip one byte in the middle (for example with a hex editor), upload it as
      transfer 5: the node answers `DIGEST_MISMATCH` and the old model keeps running.

### 8.3 Swap to the Environment model (Model B) and switch its sensor
```powershell
& $py tools\node_client.py --broker <ip> --node node1 upload ml\artifacts\environment\environment.amdl --kind environment --transfer 2
& $py tools\node_client.py --broker <ip> --node node1 activate --kind environment --transfer 2 --gen 4
& $py tools\node_client.py --broker <ip> --node node1 inputs --kind environment --gen 5  2:1:0 2:0:1     # DHT22: temp -> slot 0, humidity -> slot 1
```
The first inference needs 20 samples at 2 seconds = about 40 s. Then switch the sensor **without changing the model**:
```powershell
& $py tools\node_client.py --broker <ip> --node node1 config B --gen 30          # DHT22 stops; model reports SUSPENDED (sensor gone)
& $py tools\node_client.py --broker <ip> --node node1 inputs --kind environment --gen 6  5:0:0 5:2:1   # BME280: temp -> slot 0, humidity -> slot 1
```
- [ ] `model_status` shows SUSPENDED/`SENSOR_NOT_ACTIVE` after the swap, ACTIVE after the new mapping, and inferences resume.

### 8.4 Rules (optional)
```powershell
& $py tools\make_example_rules.py rules --out rules.bin --gen 1 --sensor 2 --index 1 --above 28 --text "too warm"
& $py tools\node_client.py --broker <ip> --node node1 rules rules.bin
```
When DHT22 temperature (value index 1) exceeds 28 C, a `notification` is published (once per second at most).
For a GPIO action use `--pulse-pin 21 --pulse-ms 500` (pin 21 must be free; wire an LED + resistor to it). Pins used by running sensors are refused.

### 8.5 Two nodes: the bridge (optional, needs a second board with node id `node2`)
On node1 make node2's DHT22 appear as virtual sensor 130:
```powershell
& $py tools\make_example_rules.py bridge --out bridge.bin --gen 1 --remote-node node2 --remote-sensor 2 --local 130 --kinds humidity_pct,temperature_c
& $py tools\node_client.py --broker <ip> --node node1 bridge bridge.bin
```
Node2 must be running its DHT22 (config with a DHT22 profile). Node1 then treats sensor 130 like a local sensor
(rules and ML mappings can use it). Virtual sensors are never re-published, so there are no loops.

### 8.6 Reboot test (the one thing the emulator cannot prove)
1. Apply a configuration, a model + mapping, and a rule.
2. In the serial console type `REBOOT` (or press the RESET button).
3. After it restarts, type `SHOW`.
- [ ] Sensors, rules and the model come back, and readings/inferences continue by themselves. **Tell me the result either way.**

---

## 9. Phase 8 - collect your own ML data and retrain

Both models currently come from **synthetic** data. The goal here is a model trained on your recordings.

### 9.1 Collect
Make sure the node is on MQTT (Phase 6) and the right sensor is configured. The tool saves one CSV per recording:

```powershell
# MOTION model (GY-511, sensor id 1, 50 Hz): one label per file, several separate sessions per class
& $py tools\collect_readings.py --broker <ip> --node node1 --sensor 1 --label STATIONARY --seconds 120 --out data\motion\stationary_01.csv
& $py tools\collect_readings.py --broker <ip> --node node1 --sensor 1 --label MOVING     --seconds 120 --out data\motion\moving_01.csv
& $py tools\collect_readings.py --broker <ip> --node node1 --sensor 1 --label VIBRATING  --seconds 120 --out data\motion\vibrating_01.csv

# ENVIRONMENT model: DHT22 (sensor id 2; humidity = value 0, temperature = value 1)
& $py tools\collect_readings.py --broker <ip> --node node1 --sensor 2 --label STABLE   --seconds 900 --out data\environment\stable_dht_01.csv
& $py tools\collect_readings.py --broker <ip> --node node1 --sensor 2 --label CHANGING --seconds 900 --out data\environment\changing_dht_01.csv
# and the BME280 (sensor id 5; temperature = value 0, humidity = value 2)
& $py tools\collect_readings.py --broker <ip> --node node1 --sensor 5 --label STABLE   --columns temperature=0,humidity=2 --seconds 900 --out data\environment\stable_bme_01.csv
```

How to record good data (these are suggestions, not tested thresholds):
- **Motion:** STATIONARY = board resting on a table; MOVING = carried, tilted, slow movements; VIBRATING = shaken quickly
  or placed on a vibrating machine. Change the orientation and the person/surface between sessions. Aim for at least
  **5 separate recordings per class** (the trainer splits by whole recording, so it needs several), each a couple of minutes.
- **Environment:** a window is 40 s (20 samples every 2 s). STABLE = undisturbed room, sensor not touched.
  CHANGING = something that moves temperature by about 1 C or humidity by about 4 % within a window (breathe on the
  sensor, hold a warm mug nearby, a cold pack, a hair dryer from a distance). The synthetic labels used these thresholds
  (stable: under 0.15 C and 0.6 % per window; changing: at least 0.8 C or 4 %). Keep **your labelling consistent** and write
  down what you used. Record **both** DHT22 and BME280 so the model sees both noise levels.
- Keep the sensor periods as in the profiles (GY-511 20 ms, DHT22/BME280 2000 ms); the loader rejects other spacings.
- One label per file. Never mix labels in a recording.

### 9.2 Retrain
```powershell
& $py ml\train_motion.py      --data data\motion      --out ml\artifacts_real\motion      --version 2
& $py ml\train_environment.py --data data\environment --out ml\artifacts_real\environment --version 2
```
Read the output and `ml\artifacts_real\<task>\report.json`:
- `data_provenance` should say **REAL recordings** and `split_mode` should be `by_session`. If it says
  "optimistic", you had fewer than 3 recordings for some class: record more.
- `int8_test_accuracy` and the confusion matrix are your honest numbers. If they are low, the data or labels need work.
  A warning is printed under 80 %.
- This uses a new output folder so your synthetic models stay available.

### 9.3 Load the new model on the node
Same as Phase 7.2/7.3 but with the new files and higher numbers:
```powershell
& $py tools\node_client.py --broker <ip> --node node1 upload ml\artifacts_real\motion\motion.amdl --kind motion --transfer 11
& $py tools\node_client.py --broker <ip> --node node1 activate --kind motion --transfer 11 --gen 40
& $py tools\node_client.py --broker <ip> --node node1 inputs --kind motion --gen 41  1:0:0 1:1:1 1:2:2
```
(Generation numbers just have to be larger than the last ones the node accepted.)
- [ ] Check `inference` outputs on fresh movements against what you did.

---

## 10. Phase 9 - measurements and evidence to keep

These fill the open rows of `docs/requirements_traceability.md` and the benchmark plan in `docs/project_status.md`.
Keep the raw output (copy from the monitor / `listen`) and note the date and settings.

| What | How |
|---|---|
| Boot + `SHOW` after reboot | screenshot / copy of the monitor |
| Each sensor's reading | the `READ ...` lines from Phase 5 + a reference value |
| Achieved sampling rate | `listen` output: `timestamp_us` differences of `reading/<id>` over a few minutes |
| Inference time | `inference_us` field in `inference` messages; also the load line `arena used X of Y B` in the monitor at activation |
| Model swap time | time from starting `upload` to the `model_status` ACTIVE (the client prints the upload time) |
| Reference vs optimized kernels | build `build_ref` (see README) and compare `inference_us` and the class scores for the same input |
| Wi-Fi loss | unplug the router during an upload, restart it, resend: the node resumes or restarts the transfer and keeps the old model |
| Heartbeat | `free_heap`, counters, uptime over a long run |

When something does not behave as described here, send me the **serial log** (and the exact command): that is enough for me to fix it.

---

## 10b. The anomaly model (Model C: any sensor, any combination)

`ml/ANOMALY_SPEC.md` has the full design. In short: 8 role slots (motion, temperature, humidity, pressure, CO2,
distance, radar distance, equipment temperature), 10 value channels, an autoencoder trained only on normal data.
A slot counts only while its sensor has delivered at least 5 readings in the last 6 s (the "presence mask"), so any
subset of sensors works. Score = mean reconstruction error over the present slots; ANOMALY when score > threshold.
The most unusual slot is the one with the largest per-slot error.

```bash
cd ml && ..\.venv\Scripts\python.exe train_anomaly.py          # writes ml/artifacts/anomaly/anomaly.amdl
python tools/node_client.py --broker <ip> --node node1 upload ml/artifacts/anomaly/anomaly.amdl --kind anomaly --transfer 30 --chunk 1024
python tools/node_client.py --broker <ip> --node node1 activate --kind anomaly --transfer 30 --gen 30
# inputs are sensor:value_index:channel ; channels 0-2 accel x/y/z, 3 temp, 4 humidity, 5 pressure, 6 CO2,
# 7 distance, 8 radar distance (mm), 9 equipment temp. Map any subset (motion needs all three axes).
python tools/node_client.py --broker <ip> --node node1 inputs --kind anomaly --gen 31 5:0:3 5:2:4 5:1:5 130:1:8 8:0:9
```

Sensor 130 is node2's LD2450 arriving through the node-to-node bridge (value 1 = y_mm, kind RADAR_DISTANCE_MM).
Host tests: `tests/anomaly_host/run.py` (C code vs the Python reference, 103 checks).

Retraining on your own data (version 2 on node1 was trained this way):

```bash
# 12 min of NORMAL conditions (nobody in the radar's view, nothing touching/breathing on the sensors)
python tools/record_slot_data.py --broker <ip> --seconds 720 --out data/anomaly/normal_01.csv --source node1:5 --source node1:8 --source node1:2
cd ml && ..\.venv\Scripts\python.exe train_anomaly.py --out artifacts\anomaly_real --version 2 --real ..\data\anomaly\normal_01.csv
# test file with deliberate disturbances, scored exactly like the node scores it:
python ml/eval_real_anomaly.py ml/artifacts/anomaly_real data/anomaly/test_02.csv
```

Real recordings replace 60 % of the training rows for the slots that were recorded (temperature, humidity,
pressure, equipment); motion, CO2, distance and radar stay synthetic. The threshold is the real validation p99.7 x 1.5.

---

## 10c. Dormant sensors: a card tap wakes a sensor (rule actions SENSOR_START / SENSOR_STOP)

A sensor can be stored **dormant**: its full profile is kept (and survives a reboot) but it does not run and uses no
bus time. A rule can start it, optionally for a limited time. Short form: `{"sensor": "gy511", "dormant": true}`
(full form: `"enabled": false, "dormant": true`). A dormant profile reserves its pins and shares its bus slot, so a
bus holds either a running or a dormant sensor.

```json
{"generation": "auto", "sensors": [{"sensor": "rc522"}, {"sensor": "gy511", "dormant": true}]}
```

Rule (publish to `aai16/<node>/json_in/rules`; example file `rules_tap_wakes_gy511.json`): RC522 `card_present > 0.5`,
edge triggered, action `RACT_SENSOR_START` with `sensor_id` 1 and `duration_ms` 60000 (0 = until stopped).
`RACT_SENSOR_STOP` stops a sensor a rule woke. Behaviour (host-tested, `tests/dormant_host`, 1058 checks):
- a new tap while awake re-arms the timer; the sensor never starts by itself after a reboot;
- a rule can only start dormant sensors and only stop sensors a rule woke (configured sensors are never touched);
- a config for that bus always wins (dormant -> running, or removed), CLEAR removes dormant profiles too;
- notifications "sensor N started (timed)" / "stopped (timer)" are published on the notification topic.

RC522 clone chips (VersionReg 0xB2) need the receiver gain raised (RFCfgReg = 0x70, now part of the profile),
otherwise no card is ever seen. A card held on the reader for the whole time re-arms the timer on every poll that
briefly loses it; tap and remove it.

---

## 11. Troubleshooting

| Symptom | Likely cause and what to do |
|---|---|
| `idf.py` says port busy / cannot open | another monitor is open: close it (`Ctrl+]`), or unplug/replug |
| Flash says "failed to connect" | hold BOOT, tap RESET, release BOOT, retry; try the other USB port/cable |
| No text in the monitor | wrong USB port (use the UART one), wrong COM port, or the board is in download mode: tap RESET |
| Boot loop / `Guru Meditation` | send me the full log; check nothing shorts 3V3 and GND |
| `CFG` says `ERR ...` | the hex was cut off or contains spaces; paste it as one line right after `CFG ` |
| Config is refused, `STALE_GENERATION` | use a generation number larger than `SHOW`'s `cfg_gen`; `CLEAR` also raises it by 1 |
| `FORBIDDEN_PIN` | you edited a pin to a reserved one (0, 3, 19, 20, 26-37, 43-46) |
| `READ ... bus_error` (I2C) | SDA/SCL swapped, no power, wrong address (BME280 SDO), missing common GND |
| `READ ... timeout` (UART) | TX/RX not crossed, sensor not powered with 5 V, wrong baud |
| `READ ... timeout` (DHT22) | pull-up missing (bare part), DATA not on GPIO6, no power |
| `READ ... timeout` (HC-SR04) | ECHO divider wrong, VCC not 5 V, target out of range |
| `READ ... bad_frame` | noise or bad contact; for the MH-Z19 a checksum error; try shorter wires |
| RC522 reads `0` or `255` | check 3.3 V (not 5 V), CS on GPIO14, RST to 3V3, and the SPI wires |
| MAX6675 shows `0.00` / `1023.75` | thermocouple or MISO/CS wiring; open-circuit detection is not implemented |
| Wi-Fi never connects | wrong SSID/password, a 5 GHz-only network, or weak signal; look for `Wi-Fi disconnected, retrying` |
| `got IP` but no `MQTT connected` | wrong broker IP, broker not running, firewall rule missing, or client isolation on the network |
| `MQTT connected` but `listen` is silent | different node id, wrong broker in `--broker`, or you are looking at the wrong topic; heartbeat comes every 10 s |
| Upload stalls | `listen` for `model_chunk_ack` status; `OUT_OF_ORDER` is handled automatically; `STORAGE_ERROR` means the flash write failed (send me the log) |
| `activate` answers `RUNTIME_REJECTED` | the TFLite runtime refused the model (shape/quantization differ from its contract): send the monitor log lines starting `model rejected:` |
| `inputs` answers `PERIOD_MISMATCH` | the sensor period differs from the model: GY-511 must be 20 ms, DHT22/BME280 2000 ms |
| Inference never starts | the first result needs a full window (about 1.3 s for motion, 40 s for environment) and valid readings the whole time |
| Everything forgotten after reboot | tell me: persistence is implemented but this is exactly what has not been shown on hardware yet |

---

## 12. Cheat sheets

**Serial console:** `HELP`, `SHOW`, `WATCH <id|ALL|OFF>`, `RUN`, `STOP`, `CLEAR`, `REBOOT`, `CFG <hex>`, `RULES <hex>`, `BRIDGE <hex>`

**Sensor ids:** 1 GY-511 - 2 DHT22 - 3 LD2450 - 4 RC522 - 5 BME280 - 6 HC-SR04 - 7 MH-Z19 - 8 MAX6675 (virtual bridged sensors: 128-255)

**Value order per sensor:** GY-511 `[ax, ay, az]` - BME280 `[temp, pressure, humidity]` - DHT22 `[humidity, temp]` - HC-SR04 `[cm]` -
LD2450 `[x, y, speed]` - MH-Z19 `[ppm]` - RC522 `[card_present, atqa]` - MAX6675 `[temp]`

**Client:** `tools\node_client.py --broker <ip> --node <id>` then one of
`config A|B|<file> --gen N` - `rules <file>` - `bridge <file>` - `command run|stop|clear` -
`upload <file.amdl> --kind motion|environment --transfer N` - `activate --kind K --transfer N --gen G` -
`inputs --kind K --gen G sensor:value:slot ...` - `listen --seconds N`

**Model numbers:** motion classes 0 STATIONARY, 1 MOVING, 2 VIBRATING - environment 0 STABLE, 1 CHANGING

**Useful numbers:** motion window 64 samples at 50 Hz - environment window 20 samples at 0.5 Hz - max config size 28 KB -
upload chunk 256-1024 bytes (default 512)

**Files to look at:** `docs/project_status.md` (what is verified), `docs/requirements_traceability.md` (the acceptance table to fill in),
`docs/sensor_profiles.md`, `docs/ml_model_contracts.md`, `docs/mqtt_protobuf_protocol.md`, `docs/testing.md`.

---

## What is not verified (so check it while you follow this guide)

- The wiring tables and expected readings come from datasheet knowledge and the code's profiles. Seven of the eight sensors were
  observed working on boards with these pins; the HC-SR04 was not re-observed. The pin labels on your modules may differ.
  Two findings from the boards: the MH-Z19B needs 5 V and TX/RX crossed (a wrong wiring only shows up as READ_TIMEOUT), and the
  RC522 clone chip needs the receiver-gain write that is part of the profile.
- Mosquitto installation steps (they were followed on a Windows laptop; your steps may differ).
- Not yet measured: the HC-SR04 timing, the ESP-NN speed-up against the reference build, long (hour-scale) runs, and a model
  upload while a fast sensor (GY-511, 20 ms) is running.
