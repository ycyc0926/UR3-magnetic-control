"""Read-only RTDE telemetry including program, safety, TCP and payload state.

Only output negotiation/start/pause packets exist. Never sets an input recipe.
"""
import json
import math
import socket
import struct
import time

import ur3_realtime_monitor as rtde
from record_rtde_outputs import OutputRecorder

FIELDS = [('timestamp', 'DOUBLE'), ('actual_q', 'VECTOR6D'), ('actual_qd', 'VECTOR6D'),
          ('actual_TCP_pose', 'VECTOR6D'), ('actual_TCP_speed', 'VECTOR6D'),
          ('tcp_offset', 'VECTOR6D'), ('payload', 'DOUBLE'), ('payload_cog', 'VECTOR3D'),
          ('speed_scaling', 'DOUBLE'), ('target_speed_fraction', 'DOUBLE'),
          ('runtime_state', 'UINT32'), ('robot_mode', 'INT32'), ('safety_mode', 'INT32')]
FORMATS = {'DOUBLE': 'd', 'VECTOR6D': '6d', 'VECTOR3D': '3d', 'UINT32': 'I', 'INT32': 'i'}


def negotiate(sock):
    rtde.send_packet(sock, rtde.RTDE_REQUEST_PROTOCOL_VERSION, struct.pack('>H', 2))
    if rtde.receive_command(sock, rtde.RTDE_REQUEST_PROTOCOL_VERSION) != b'\x01':
        raise RuntimeError('RTDE v2 rejected')
    rtde.send_packet(sock, rtde.RTDE_GET_URCONTROL_VERSION)
    version = '.'.join(map(str, struct.unpack('>IIII', rtde.receive_command(sock, rtde.RTDE_GET_URCONTROL_VERSION))))
    rtde.send_packet(sock, rtde.RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS,
                     struct.pack('>d', 125.)+','.join(name for name, kind in FIELDS).encode())
    reply = rtde.receive_command(sock, rtde.RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS)
    if not reply or reply[1:].decode().split(',') != [kind for name, kind in FIELDS]:
        raise RuntimeError('Unexpected output recipe')
    recipe = reply[0]
    rtde.send_packet(sock, rtde.RTDE_CONTROL_PACKAGE_START)
    if rtde.receive_command(sock, rtde.RTDE_CONTROL_PACKAGE_START) != b'\x01':
        raise RuntimeError('RTDE output synchronization rejected')
    return recipe, version


def parse(payload, recipe):
    expected_size = 1+sum(struct.calcsize('>'+FORMATS[kind]) for name, kind in FIELDS)
    if len(payload) != expected_size or payload[0] != recipe:
        raise RuntimeError('Invalid output packet size/recipe')
    result = {}; cursor = 1
    for name, kind in FIELDS:
        fmt = '>'+FORMATS[kind]
        values = struct.unpack_from(fmt, payload, cursor); cursor += struct.calcsize(fmt)
        if not all(math.isfinite(value) for value in values):
            raise RuntimeError('Nonfinite RTDE output: '+name)
        result[name] = values[0] if len(values) == 1 else values
    return result


def read_status():
    with socket.create_connection((rtde.ROBOT_IP, 30004), timeout=3.) as sock:
        recipe, version = negotiate(sock)
        try: return parse(rtde.receive_command(sock, rtde.RTDE_DATA_PACKAGE), recipe), version
        finally: rtde.send_packet(sock, rtde.RTDE_CONTROL_PACKAGE_PAUSE)


class StatusRecorder(OutputRecorder):
    def _record(self):
        try:
            with self.path.open('x', encoding='utf-8') as stream:
                self.phase = 'connecting'
                with socket.create_connection((rtde.ROBOT_IP, 30004), timeout=3.) as sock:
                    self.phase = 'negotiating status outputs'
                    recipe, self.version = negotiate(sock)
                    sock.settimeout(.25); self.phase = 'streaming status outputs'
                    try:
                        while not self.ending.is_set():
                            state = parse(rtde.receive_command(sock, rtde.RTDE_DATA_PACKAGE), recipe)
                            entry = {'host_monotonic_s': time.monotonic(), 'host_time_ns': time.time_ns(), 'state': state}
                            stream.write(json.dumps(entry, allow_nan=False)+'\n'); stream.flush()
                            self.latest = entry; self.count += 1; self.first.set()
                    finally: rtde.send_packet(sock, rtde.RTDE_CONTROL_PACKAGE_PAUSE)
        except Exception as error:
            self.error = f'{self.phase}: {error}'; self.first.set()
