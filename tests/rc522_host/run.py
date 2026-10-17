"""RC522/SPI host tests using explicit SDK, register and RF response doubles.

Requires gcc. No hardware, serial, network, credentials or production injection.
These validate framing/control flow; they do not emulate analogue RF behavior.
"""
import os
from pathlib import Path
import shutil
import subprocess

test = Path(__file__).resolve().parent
repo = test.parents[1]
output = repo / "build" / "rc522_host"
output.mkdir(parents=True, exist_ok=True)
compiler = shutil.which("gcc")
if not compiler:
    raise SystemExit("Host gcc not found")
for name in ["esp_err.h", "esp_log.h", "esp_timer.h", "freertos/FreeRTOS.h",
             "freertos/task.h", "driver/gpio.h", "driver/spi_master.h"]:
    header = output / name
    header.parent.mkdir(parents=True, exist_ok=True)
    header.write_text('#include "platform.h"\n')
flags = [compiler, "-std=c11", "-Wall", "-Wextra", "-Werror", "-Wno-unused-parameter",
         "-I", str(test), "-I", str(output), "-I", str(repo / "main")]
for component in ["spi_bus_manager", "rc522_sensor", "sensor_status", "mqtt_manager", "ld2450_sensor"]:
    flags += ["-I", str(repo / "components" / component / "include")]
for name in ["test_spi", "test_rc522", "test_reader"]:
    binary = output / (name + (".exe" if os.name == "nt" else ""))
    sources = [str(test / (name + ".c")), str(repo / "tests/sensor_host/platform.c")]
    if name == "test_reader":
        sources += [str(repo / "components/sensor_status/sensor_status.c")]
    subprocess.run(flags + sources + ["-o", str(binary)], check=True)
    scenarios = (["idle", "held", "remove", "change", "spi_init", "task", "init", "reads",
                  "resume", "overrun", "mqtt_offline", "mqtt_full", "uid4", "uid7",
                  "held_error"] if name == "test_reader" else [None])
    for scenario in scenarios:
        subprocess.run([str(binary)] + ([scenario] if scenario else []), check=True, timeout=15)
