"""Boot the REAL firmware image in Espressif's QEMU (ESP32-S3) and drive its serial console.

This is emulation, not a board: there are no sensors on the emulated buses (transactions fail or time out),
no Wi-Fi and no broker. It proves that the actual ESP32-S3 binary boots, accepts and refuses configuration
through the same validator as MQTT, runs the sensor scheduler without hanging or crashing, and writes the
committed configuration to NVS.

KNOWN LIMITATION OF THIS QEMU BUILD: flash contents are not kept across a restart. A control experiment with
stock ESP-IDF code only (tests/qemu_nvs_probe) loses its NVS counter on every esp_restart(), so reboot
persistence CANNOT be demonstrated here; it needs a real board. This script therefore asserts the NVS *write*
(from the firmware's own log) and the REBOOT command, but not restoration after reboot.

    1. build:  . .\\tools\\idf_env.ps1 ; idf.py build
       image:  python -m esptool --chip esp32s3 merge-bin -o build\\qemu_flash.bin --flash-mode dio --flash-size 16MB
               --fill-flash-size 16MB 0x0 build\\bootloader\\bootloader.bin 0x8000 build\\partition_table\\partition-table.bin
               0x20000 build\\aai16_edge_node.bin
    2. run:    .venv\\Scripts\\python.exe tools\\qemu_session.py
Needs tools/qemu (unpacked from C:\\Espressif\\dist\\qemu-xtensa-softmmu-*.tar.xz).
"""
import pathlib
import shutil
import socket
import subprocess
import sys
import threading
import time

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "tools"))
import sensor_profiles as sp  # noqa: E402
from profile_lib import load_pb  # noqa: E402

QEMU = ROOT / "tools" / "qemu" / "qemu" / "bin" / "qemu-system-xtensa.exe"
BUILD = ROOT / "build"


def _free_port():
    """An unused TCP port (serial = N, monitor = N + 1000 is checked free too): never reconnects to a stale emulator."""
    for _ in range(50):
        with socket.socket() as a, socket.socket() as b:
            a.bind(("127.0.0.1", 0))
            port = a.getsockname()[1]
            try:
                b.bind(("127.0.0.1", port + 1000))
            except OSError:
                continue
            return port
    raise RuntimeError("no free ports")


class Session:
    def __init__(self, flash):
        port = _free_port()
        self.mon_port = port + 1000
        self.lines, self.buf, self.lock = [], b"", threading.Lock()
        args = [str(QEMU), "-machine", "esp32s3", "-nographic", "-m", "8M",
                "-drive", f"file={flash},if=mtd,format=raw,cache=writethrough",
                "-global", "driver=ssi_psram,property=is_octal,value=true",
                "-serial", f"tcp:127.0.0.1:{port},server,wait",   # wait: QEMU holds the guest until we connect, so no boot output is lost
                "-monitor", f"tcp:127.0.0.1:{self.mon_port},server,nowait"]
        self.proc = subprocess.Popen(args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        for _ in range(100):
            try:
                self.sock = socket.create_connection(("127.0.0.1", port), timeout=1)
                break
            except OSError:
                time.sleep(0.1)
        else:
            raise RuntimeError("could not connect to the QEMU serial port")
        self.sock.settimeout(0.2)
        threading.Thread(target=self._reader, daemon=True).start()

    def _reader(self):
        while self.proc.poll() is None:
            try:
                d = self.sock.recv(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            if not d:
                break
            with self.lock:
                self.buf += d
                *done, self.buf = self.buf.split(b"\n")
                self.lines += [x.decode("utf-8", "replace").rstrip("\r") for x in done]

    def wait_for(self, text, timeout=30):
        end = time.time() + timeout
        while time.time() < end:
            with self.lock:
                if any(text in l for l in self.lines):
                    return True
            time.sleep(0.1)
        return False

    def cmd(self, line, settle=1.2):
        with self.lock:
            start = len(self.lines)
        self.sock.sendall(line.encode() + b"\n")
        time.sleep(settle)
        with self.lock:
            return [l for l in self.lines[start:] if l.strip()]

    def reboot(self, timeout=30):
        """Restart the firmware the way a real node restarts: its own REBOOT command (esp_restart)."""
        with self.lock:
            self.lines.clear()
            self.buf = b""
        self.sock.sendall(b"REBOOT\n")
        return self.wait_for("booted: config generation", timeout)

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass
        try:
            with socket.create_connection(("127.0.0.1", self.mon_port), timeout=2) as m:
                time.sleep(0.2)
                m.sendall(b"quit\n")
                time.sleep(0.3)
        except OSError:
            pass
        try:
            self.proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self.proc.kill()
            self.proc.wait()


def main():
    if not QEMU.exists() or not (BUILD / "qemu_flash.bin").exists():
        sys.exit("build the firmware and create build/qemu_flash.bin first (see the module docstring)")
    pb = load_pb()
    flash = BUILD / "qemu_run.bin"
    shutil.copy(BUILD / "qemu_flash.bin", flash)

    good = sp.config_a(pb, 1).SerializeToString().hex()
    bad_msg = pb.ConfigApply(generation=2)
    p = sp.gy511(pb)
    p.i2c.sda = 26  # SPI flash pin: must be refused
    bad_msg.profiles.append(p)
    bad = bad_msg.SerializeToString().hex()

    failures = 0

    def check(name, ok, detail=""):
        nonlocal failures
        failures += not ok
        print(f"  {'ok  ' if ok else 'FAIL'} {name}" + (f"   [{detail}]" if detail else ""))

    def panicked(s):
        return any("Guru Meditation" in l or "abort()" in l or "assert failed" in l for l in s.lines)

    print("== boot and console")
    s = Session(flash)
    try:
        check("firmware boots and the node starts", s.wait_for("node 'node1' booted"), "app_main ran on the emulated ESP32-S3")
        check("8 MB PSRAM found, NVS ready", s.wait_for("Found 8MB PSRAM") and s.wait_for("NVS ready: ESP_OK"))
        time.sleep(0.5)
        r = s.cmd("HELP");                       check("HELP", any("CFG" in l and "REBOOT" in l for l in r))
        r = s.cmd("SHOW");                       check("SHOW: nothing configured yet", any("cfg_gen=0" in l and "sensors=[]" in l for l in r))
        s.cmd("CFG " + bad);                     r = s.cmd("SHOW")
        check("config with a forbidden pin (GPIO26) is refused, state unchanged", any("cfg_gen=0" in l and "sensors=[]" in l for l in r))
        r = s.cmd("CFG " + good, settle=2.5);    check("config A accepted over the console", any("OK submitted" in l for l in r))
        check("committed configuration written to NVS", any("nvs wrote cfg" in l for l in s.lines), next((l[l.find('nvs'):] for l in s.lines if "nvs wrote cfg" in l), ""))
        r = s.cmd("SHOW")
        check("SHOW: generation 1, sensors 2,1,3,4 (GPIO, I2C, UART, SPI)", any("cfg_gen=1" in l and "sensors=[2,1,3,4]" in l for l in r))
        r = s.cmd("WATCH ALL", settle=3.5)
        reads = [l for l in s.lines if "READ id=" in l]
        check("WATCH ALL prints a line per sensor (here: errors, since nothing is wired in the emulator)", len(reads) >= 2 and any("bus_error" in l or "timeout" in l for l in reads), (reads[0][reads[0].find("READ"):] if reads else "no READ lines"))
        s.cmd("WATCH OFF")
        s.cmd("STOP"); r = s.cmd("SHOW");        check("STOP", any("STOPPED" in l for l in r))
        s.cmd("RUN");  r = s.cmd("SHOW");        check("RUN", any("RUNNING" in l for l in r))
        r = s.cmd("FLY");                        check("unknown command rejected", any("ERR" in l for l in r))
        r = s.cmd("CFG zz");                     check("malformed hex rejected", any("ERR" in l for l in r))
        time.sleep(4.0)
        r = s.cmd("SHOW", settle=1.5);           check("still responsive after 4 s of the scheduler running all four (sensorless) buses", any("node=node1" in l for l in r))
        check("no panic or reset while running", not panicked(s) and not any("Rebooting" in l for l in s.lines))
        r = s.cmd("CLEAR", settle=2.5);          r = s.cmd("SHOW")
        check("CLEAR stops all buses (generation 2, no sensors)", any("cfg_gen=2" in l and "sensors=[]" in l for l in r))
        check("node state (rules / bridge / ML mapping / model generation) written to NVS", any("nvs wrote state" in l for l in s.lines))
        check("REBOOT restarts the firmware, which comes back up", s.reboot() and s.wait_for("NVS ready"))
        r = s.cmd("SHOW")
        check("node answers after the restart", any("node=node1" in l for l in r))
    finally:
        s.close()

    print("\nNot asserted (emulator limitation, see module docstring): configuration restored after a reboot.")
    print(f"{'ALL QEMU CHECKS PASSED' if not failures else str(failures) + ' QEMU CHECK(S) FAILED'}")
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
