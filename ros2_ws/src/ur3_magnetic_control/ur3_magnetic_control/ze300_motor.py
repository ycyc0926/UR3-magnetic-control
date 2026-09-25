"""ZE300 V3.03 custom RS485 motor control (115200 8N1 by default)."""

import argparse
import fcntl
import json
import math
import os
from pathlib import Path
import select
import struct
import sys
import termios
import time
import tty


DEFAULT_PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
COUNTS_PER_TURN = 16384
DEFAULT_ORIGIN_FILE = Path("/home/yc/UR3/config/motor_origin.json")


def crc16(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0xA001 if crc & 1 else 0)
    return crc


def telemetry(payload):
    if len(payload) != 22:
        raise OSError(f"ZE300 telemetry length {len(payload)}, expected 22")
    single, counts, speed, current, voltage, bus_current, temp, mode, enabled, fault = (
        struct.unpack("<HiiiHHBBBB", payload)
    )
    return {
        "single_turn_deg": single * 360 / COUNTS_PER_TURN,
        "position_counts": counts,
        "position_deg": counts * 360 / COUNTS_PER_TURN,
        "speed_rpm": speed / 100,
        "q_current_a": current / 1000,
        "bus_voltage_v": voltage / 100,
        "bus_current_a": bus_current / 100,
        "temperature_c": temp,
        "run_state": mode,
        "enabled": enabled != 0,
        "fault_code": fault,
    }


def position_counts(degrees):
    if not math.isfinite(degrees):
        raise ValueError("motor position must be finite")
    counts = round(degrees * COUNTS_PER_TURN / 360)
    if not -(2 ** 31) <= counts < 2 ** 31:
        raise ValueError("motor position exceeds the protocol's signed 32-bit range")
    return counts


def single_turn_counts(degrees):
    return round(degrees * COUNTS_PER_TURN / 360) % COUNTS_PER_TURN


def save_origin(motor, path=DEFAULT_ORIGIN_FILE):
    status = motor.status()
    if abs(status["speed_rpm"]) > 0.5:
        raise ValueError("stop the motor before saving its physical origin")
    origin = {
        "schema": "ze300-magnet-origin/v1",
        "single_turn_counts": single_turn_counts(status["single_turn_deg"]),
        "reference": "magnet N points along tool0 +X",
    }
    path = Path(path)
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(origin, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    temporary.replace(path)
    return origin


def restore_origin(motor, path=DEFAULT_ORIGIN_FILE, timeout=60.0):
    origin = json.loads(Path(path).read_text(encoding="utf-8"))
    target = origin.get("single_turn_counts")
    if (origin.get("schema") != "ze300-magnet-origin/v1"
            or type(target) is not int or not 0 <= target < COUNTS_PER_TURN):
        raise ValueError("invalid saved ZE300 magnet origin")
    current = motor.status()
    if abs(current["speed_rpm"]) > 0.5:
        motor.speed(0)
        current = motor.status()
    delta = (target - single_turn_counts(current["single_turn_deg"])
             + COUNTS_PER_TURN // 2) % COUNTS_PER_TURN - COUNTS_PER_TURN // 2
    if abs(delta) <= position_counts(0.5):
        delta = 0
    else:
        motor.relative(delta * 360 / COUNTS_PER_TURN)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        status = motor.status()
        error = (target - single_turn_counts(status["single_turn_deg"])
                 + COUNTS_PER_TURN // 2) % COUNTS_PER_TURN - COUNTS_PER_TURN // 2
        if abs(error) <= position_counts(0.5) and abs(status["speed_rpm"]) <= 0.5:
            if abs(status["position_deg"]) > 0.5:
                motor.set_origin()
                status = motor.status()
            return {"commanded_delta_deg": delta * 360 / COUNTS_PER_TURN, **status}
        time.sleep(0.1)
    raise TimeoutError("ZE300 did not reach the saved physical origin")


def watch_origin(port=DEFAULT_PORT, address=1, baud=115200, path=DEFAULT_ORIGIN_FILE):
    connected = False
    while True:
        if Path(path).is_file():
            replied = False
            try:
                with ZE300Motor(port, address, baud, lock_timeout=0) as motor:
                    motor.status()
                    replied = True
                    if not connected:
                        result = restore_origin(motor, path)
                        print(f"ZE300 origin restored: {result['commanded_delta_deg']:.2f} deg", flush=True)
                connected = True
            except BlockingIOError:
                # Another local control command owns the serial port.
                connected = True
            except (OSError, ValueError) as error:
                if connected or replied:
                    print(f"ZE300 unavailable or origin restore failed: {error}", flush=True)
                connected = replied
        time.sleep(1)


class ZE300Motor:
    def __init__(self, port=DEFAULT_PORT, address=1, baud=115200, timeout=1.0, lock_timeout=2.0):
        if not 1 <= address <= 254:
            raise ValueError("ZE300 address must be 1..254")
        baud_flag = getattr(termios, f"B{baud}", None)
        if baud_flag is None:
            raise ValueError(f"unsupported serial baud rate: {baud}")
        self.fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        self.address = address
        self.timeout = timeout
        self.sequence = 0
        try:
            deadline = time.monotonic() + lock_timeout
            while True:
                try:
                    fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
                    break
                except BlockingIOError:
                    if time.monotonic() >= deadline:
                        raise
                    time.sleep(0.05)
            tty.setraw(self.fd)
            settings = termios.tcgetattr(self.fd)
            settings[4] = baud_flag
            settings[5] = baud_flag
            termios.tcsetattr(self.fd, termios.TCSANOW, settings)
        except BaseException:
            self.close()
            raise

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()

    def close(self):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None

    def _read_exact(self, length, deadline):
        data = bytearray()
        while len(data) < length:
            remaining = deadline - time.monotonic()
            if remaining <= 0 or not select.select([self.fd], [], [], remaining)[0]:
                raise TimeoutError("ZE300 did not reply; check motor power, port and address")
            part = os.read(self.fd, length - len(data))
            if part:
                data.extend(part)
        return bytes(data)

    def _command(self, command, payload=b""):
        if self.fd is None:
            raise OSError("ZE300 serial port is closed")
        sequence = self.sequence
        self.sequence = (sequence + 1) & 0xFF
        frame = bytes((0xAE, sequence, self.address, command, len(payload))) + payload
        frame += struct.pack("<H", crc16(frame))
        termios.tcflush(self.fd, termios.TCIFLUSH)
        deadline = time.monotonic() + self.timeout
        sent = 0
        while sent < len(frame):
            remaining = deadline - time.monotonic()
            if remaining <= 0 or not select.select([], [self.fd], [], remaining)[1]:
                raise TimeoutError("ZE300 serial write timed out")
            sent += os.write(self.fd, frame[sent:])
        while self._read_exact(1, deadline) != b"\xAC":
            pass
        header = b"\xAC" + self._read_exact(4, deadline)
        reply = header + self._read_exact(header[4] + 2, deadline)
        if (header[1], header[2], header[3]) != (sequence, self.address, command):
            raise OSError("ZE300 reply sequence, address or command does not match")
        if crc16(reply[:-2]) != struct.unpack("<H", reply[-2:])[0]:
            raise OSError("ZE300 reply CRC is invalid")
        if command == 0x1D:
            if header[4] != 2:
                raise OSError("ZE300 origin reply must contain two bytes")
            return struct.unpack("<H", reply[5:-2])[0]
        return telemetry(reply[5:-2])

    def status(self):
        return self._command(0x0B)

    def speed(self, rpm, acceleration_rpm_s=0):
        if not math.isfinite(rpm) or not math.isfinite(acceleration_rpm_s):
            raise ValueError("motor speed and acceleration must be finite")
        speed = round(rpm * 100)
        acceleration = round(acceleration_rpm_s * 100)
        if not -(2 ** 31) <= speed < 2 ** 31 or not 0 <= acceleration < 2 ** 32:
            raise ValueError("motor speed or acceleration exceeds protocol range")
        return self._command(0x21, struct.pack("<iI", speed, acceleration))

    def absolute(self, degrees):
        return self._command(0x22, struct.pack("<i", position_counts(degrees)))

    def relative(self, degrees):
        return self._command(0x23, struct.pack("<i", position_counts(degrees)))

    def home(self):
        return self._command(0x24)

    def set_origin(self):
        return self._command(0x1D)

    def wait_for_home(self, timeout=60.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            result = self.status()
            if abs(result["position_deg"]) <= 0.5 and abs(result["speed_rpm"]) <= 0.5:
                return result
            time.sleep(0.1)
        raise TimeoutError("ZE300 did not reach the origin within 60 seconds")

    def off(self):
        return self._command(0x2F)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--address", type=int, default=1)
    parser.add_argument("--baud", type=int, default=115200)
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("status", help="read position, speed and motor status")
    speed = commands.add_parser("speed", help="set continuous speed in rpm")
    speed.add_argument("rpm", type=float)
    speed.add_argument("--acceleration-rpm-s", type=float, default=0)
    for name in ("absolute", "relative"):
        command = commands.add_parser(name, help=f"move to {name} position in degrees")
        command.add_argument("degrees", type=float)
    home = commands.add_parser("home", help="return to the configured origin by the shortest rotation")
    home.add_argument("--wait", action="store_true", help="wait until the origin is reached")
    commands.add_parser("save-origin", help="remember the current magnet orientation across power cycles")
    commands.add_parser("restore-origin", help="return to the saved physical origin using single-turn angle")
    commands.add_parser("watch-origin", help="restore origin automatically when the motor reconnects")
    commands.add_parser("off", help="disable motor output (freewheel)")
    args = parser.parse_args(argv)
    if args.command == "watch-origin":
        watch_origin(args.port, args.address, args.baud)
        return
    try:
        with ZE300Motor(args.port, args.address, args.baud) as motor:
            if args.command == "status":
                result = motor.status()
            elif args.command == "speed":
                result = motor.speed(args.rpm, args.acceleration_rpm_s)
            elif args.command == "absolute":
                result = motor.absolute(args.degrees)
            elif args.command == "relative":
                result = motor.relative(args.degrees)
            elif args.command == "home":
                result = motor.home()
                if args.wait:
                    result = motor.wait_for_home()
            elif args.command == "save-origin":
                result = save_origin(motor)
            elif args.command == "restore-origin":
                result = restore_origin(motor)
            else:
                result = motor.off()
        print(json.dumps(result, ensure_ascii=False))
    except (OSError, ValueError) as error:
        raise SystemExit(str(error)) from error


if __name__ == "__main__":
    main(sys.argv[1:])
