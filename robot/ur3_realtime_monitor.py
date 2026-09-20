#!/usr/bin/env python3
"""Read UR controller joint and TCP state through the read-only RTDE output stream."""

from __future__ import annotations

import argparse
import math
import socket
import struct
import time


ROBOT_IP = "192.168.56.101"
RTDE_PORT = 30004

RTDE_REQUEST_PROTOCOL_VERSION = 86
RTDE_GET_URCONTROL_VERSION = 118
RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS = 79
RTDE_CONTROL_PACKAGE_START = 83
RTDE_CONTROL_PACKAGE_PAUSE = 80
RTDE_DATA_PACKAGE = 85

OUTPUT_NAMES = (
    "timestamp",
    "actual_q",
    "actual_qd",
    "actual_TCP_pose",
    "actual_TCP_speed",
)
EXPECTED_TYPES = ("DOUBLE", "VECTOR6D", "VECTOR6D", "VECTOR6D", "VECTOR6D")


def receive_exact(sock: socket.socket, size: int) -> bytes:
    chunks: list[bytes] = []
    remaining = size
    while remaining:
        chunk = sock.recv(remaining)
        if not chunk:
            raise ConnectionError("RTDE connection closed by the controller")
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def receive_packet(sock: socket.socket) -> tuple[int, bytes]:
    header = receive_exact(sock, 3)
    size, command = struct.unpack(">HB", header)
    if size < 3:
        raise RuntimeError(f"Invalid RTDE packet size: {size}")
    return command, receive_exact(sock, size - 3)


def send_packet(sock: socket.socket, command: int, payload: bytes = b"") -> None:
    sock.sendall(struct.pack(">HB", len(payload) + 3, command) + payload)


def receive_command(sock: socket.socket, expected_command: int) -> bytes:
    while True:
        command, payload = receive_packet(sock)
        if command == expected_command:
            return payload


def negotiate(sock: socket.socket, frequency_hz: float) -> tuple[int, str]:
    send_packet(sock, RTDE_REQUEST_PROTOCOL_VERSION, struct.pack(">H", 2))
    reply = receive_command(sock, RTDE_REQUEST_PROTOCOL_VERSION)
    if not reply or not reply[0]:
        raise RuntimeError("Controller rejected RTDE protocol version 2")

    send_packet(sock, RTDE_GET_URCONTROL_VERSION)
    version_payload = receive_command(sock, RTDE_GET_URCONTROL_VERSION)
    if len(version_payload) != 16:
        raise RuntimeError("Unexpected controller-version reply")
    version = ".".join(str(value) for value in struct.unpack(">IIII", version_payload))

    names = ",".join(OUTPUT_NAMES).encode("ascii")
    setup_payload = struct.pack(">d", frequency_hz) + names
    send_packet(sock, RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS, setup_payload)
    reply = receive_command(sock, RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS)
    if not reply:
        raise RuntimeError("Controller returned an empty RTDE output recipe")
    recipe_id = reply[0]
    received_types = tuple(reply[1:].decode("ascii").split(","))
    if received_types != EXPECTED_TYPES:
        raise RuntimeError(
            f"Unexpected RTDE types: {received_types}; expected {EXPECTED_TYPES}"
        )

    send_packet(sock, RTDE_CONTROL_PACKAGE_START)
    reply = receive_command(sock, RTDE_CONTROL_PACKAGE_START)
    if not reply or not reply[0]:
        raise RuntimeError("Controller refused to start RTDE synchronization")
    return recipe_id, version


def parse_state(payload: bytes, recipe_id: int) -> dict[str, tuple[float, ...] | float]:
    if len(payload) != 1 + 25 * 8:
        raise RuntimeError(f"Unexpected RTDE data payload size: {len(payload)}")
    values = struct.unpack(">B25d", payload)
    if values[0] != recipe_id:
        raise RuntimeError(f"Unexpected RTDE recipe id: {values[0]}")
    data = values[1:]
    return {
        "timestamp": data[0],
        "actual_q": data[1:7],
        "actual_qd": data[7:13],
        "actual_TCP_pose": data[13:19],
        "actual_TCP_speed": data[19:25],
    }


def format_state(state: dict[str, tuple[float, ...] | float]) -> str:
    joint_rad = state["actual_q"]
    tcp_pose = state["actual_TCP_pose"]
    tcp_speed = state["actual_TCP_speed"]
    assert isinstance(joint_rad, tuple)
    assert isinstance(tcp_pose, tuple)
    assert isinstance(tcp_speed, tuple)

    joint_deg = tuple(math.degrees(value) for value in joint_rad)
    xyz_mm = tuple(value * 1000.0 for value in tcp_pose[:3])
    linear_speed_mm_s = math.sqrt(sum(value * value for value in tcp_speed[:3])) * 1000.0
    return (
        "关节角[deg] = [" + ", ".join(f"{value:8.3f}" for value in joint_deg) + "]\n"
        "TCP位置[mm] = [" + ", ".join(f"{value:8.3f}" for value in xyz_mm) + "]\n"
        "TCP姿态向量[rad] = ["
        + ", ".join(f"{value:8.5f}" for value in tcp_pose[3:])
        + f"]\nTCP线速度 = {linear_speed_mm_s:.3f} mm/s"
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="只读显示 UR3 六个关节角和 TCP 位姿，不发送运动指令。"
    )
    parser.add_argument("--robot-ip", default=ROBOT_IP)
    parser.add_argument("--duration", type=float, default=5.0, help="监控秒数；0 表示持续运行")
    parser.add_argument("--stream-hz", type=float, default=125.0, help="RTDE 请求频率")
    parser.add_argument("--display-hz", type=float, default=5.0, help="终端显示频率")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.duration < 0.0:
        raise ValueError("duration 不能小于 0")
    if not 1.0 <= args.stream_hz <= 125.0:
        raise ValueError("CB3 的 stream-hz 必须在 1 到 125 Hz 之间")
    if not 0.5 <= args.display_hz <= args.stream_hz:
        raise ValueError("display-hz 必须在 0.5 Hz 与 stream-hz 之间")

    with socket.create_connection((args.robot_ip, RTDE_PORT), timeout=3.0) as sock:
        sock.settimeout(3.0)
        recipe_id, version = negotiate(sock, args.stream_hz)
        print(f"已连接 {args.robot_ip}:{RTDE_PORT}，控制器版本 {version}")
        print("只读监控：不会向机械臂发送运动命令。按 Ctrl+C 停止。\n")

        started = time.monotonic()
        next_display = started
        packet_count = 0
        last_state = None
        try:
            while args.duration == 0.0 or time.monotonic() - started < args.duration:
                command, payload = receive_packet(sock)
                if command != RTDE_DATA_PACKAGE:
                    continue
                last_state = parse_state(payload, recipe_id)
                packet_count += 1
                now = time.monotonic()
                if now >= next_display:
                    print(format_state(last_state), end="\n\n")
                    next_display = now + 1.0 / args.display_hz
        except KeyboardInterrupt:
            pass
        finally:
            try:
                send_packet(sock, RTDE_CONTROL_PACKAGE_PAUSE)
            except OSError:
                pass

        elapsed = max(time.monotonic() - started, 1e-9)
        print(f"收到 {packet_count} 帧，实测 RTDE 速率 {packet_count / elapsed:.1f} Hz")
        if last_state is None:
            raise RuntimeError("没有收到机器人状态数据")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (ConnectionError, OSError, RuntimeError, ValueError) as exc:
        print(f"ERROR: {exc}")
        raise SystemExit(1)
