import os
import select
import struct
import threading
import json

import pytest

from ur3_magnetic_control import ze300_motor
from ur3_magnetic_control.ze300_motor import ZE300Motor, crc16, position_counts


def test_protocol_over_pty():
    assert crc16(bytes.fromhex("AE 00 01 0B 00")) == 0x289B
    assert position_counts(360) == 16384
    master, slave = os.openpty()
    port = os.ttyname(slave)
    errors = []
    requests = []

    def read_exact(length):
        data = b""
        while len(data) < length:
            assert select.select([master], [], [], 2)[0], "timed out waiting for host"
            data += os.read(master, length - len(data))
        return data

    def device():
        try:
            payload = struct.pack("<HiiiHHBBBB", 0, 16384, 1000, 0, 2426, 0, 30, 3, 1, 0)
            for expected in (0x0B, 0x21, 0x22, 0x23, 0x24, 0x1D, 0x2F):
                header = read_exact(5)
                frame = header + read_exact(header[4] + 2)
                assert header[0] == 0xAE and header[2:4] == bytes((1, expected))
                assert crc16(frame[:-2]) == struct.unpack("<H", frame[-2:])[0]
                requests.append(frame)
                body = struct.pack("<H", 42) if expected == 0x1D else payload
                reply = bytes((0xAC, header[1], 1, expected, len(body))) + body
                os.write(master, reply + struct.pack("<H", crc16(reply)))
        except BaseException as error:
            errors.append(error)

    worker = threading.Thread(target=device)
    worker.start()
    try:
        with ZE300Motor(port) as motor:
            assert motor.status()["position_deg"] == 360
            assert motor.speed(10)["speed_rpm"] == 10
            motor.absolute(360)
            motor.relative(-90)
            motor.home()
            assert motor.set_origin() == 42
            motor.off()
    finally:
        worker.join(timeout=3)
        os.close(master)
        os.close(slave)
    assert not worker.is_alive()
    assert not errors, errors
    assert requests[1][5:13] == struct.pack("<iI", 1000, 0)
    assert requests[2][5:9] == struct.pack("<i", 16384)
    assert requests[3][5:9] == struct.pack("<i", -4096)
    assert requests[4][4] == 0


def test_home_waits_for_position_and_speed():
    statuses = iter([
        {"position_deg": 90, "speed_rpm": 1},
        {"position_deg": 0.1, "speed_rpm": 1},
        {"position_deg": 0.1, "speed_rpm": 0.1},
    ])
    motor = object.__new__(ZE300Motor)
    motor.status = lambda: next(statuses)
    assert motor.wait_for_home(timeout=2)["speed_rpm"] == 0.1


def test_saved_origin_uses_shortest_single_turn_move(tmp_path):
    class Motor:
        angle = 359
        commands = []
        position = 0

        def status(self):
            return {"single_turn_deg": self.angle, "position_deg": self.position, "speed_rpm": 0}

        def relative(self, angle):
            self.commands.append(angle)
            self.angle = (self.angle + angle) % 360
            self.position += angle

        def set_origin(self):
            self.position = 0

    motor = Motor()
    motor.angle = 0
    path = tmp_path / "origin.json"
    ze300_motor.save_origin(motor, path)
    assert json.loads(path.read_text())["single_turn_counts"] == 0
    motor.angle = 359
    motor.position = 359
    result = ze300_motor.restore_origin(motor, path)
    assert 0 < motor.commands[0] < 2
    assert abs(result["single_turn_deg"]) < 0.5
    assert result["position_deg"] == 0


def test_watch_restores_after_reconnection(tmp_path, monkeypatch):
    path = tmp_path / "origin.json"
    path.write_text("{}")
    connection = iter((True, False, True))
    restores = []

    class Motor:
        def __init__(self, *_, **__):
            self.available = next(connection)

        def __enter__(self):
            return self

        def __exit__(self, *_):
            pass

        def status(self):
            if not self.available:
                raise TimeoutError("motor off")

    def stop_after_three_polls(_):
        if len(restores) == 2:
            raise KeyboardInterrupt

    monkeypatch.setattr(ze300_motor, "ZE300Motor", Motor)
    monkeypatch.setattr(ze300_motor, "restore_origin", lambda *_: restores.append(True) or {"commanded_delta_deg": 0})
    monkeypatch.setattr(ze300_motor.time, "sleep", stop_after_three_polls)
    with pytest.raises(KeyboardInterrupt):
        ze300_motor.watch_origin(path=path)
    assert len(restores) == 2


def test_serial_port_is_exclusive():
    master, slave = os.openpty()
    port = os.ttyname(slave)
    try:
        with ZE300Motor(port):
            with pytest.raises(BlockingIOError):
                ZE300Motor(port, lock_timeout=0)
    finally:
        os.close(master)
        os.close(slave)
