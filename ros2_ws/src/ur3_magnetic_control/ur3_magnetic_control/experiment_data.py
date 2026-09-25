"""Keep the camera observations for one arm experiment beside its trajectory."""

import csv
import json
import math
from pathlib import Path
from threading import Event, Thread
import time

from scipy.spatial.transform import Rotation

from .ze300_motor import COUNTS_PER_TURN


DEFAULT_H_SESSIONS = Path("/home/yc/UR3/camera/tracking_sessions")


class MotorRecorder:
    """Sample the encoder without blocking robot joint-state callbacks."""

    def __init__(self, motor, path, initial_status=None):
        self.motor = motor
        self.samples = []
        self.error = None
        self.stop_event = Event()
        self.stream = Path(path).open("x", encoding="utf-8", buffering=1)
        if initial_status is not None:
            self._save(initial_status)
        self.thread = Thread(target=self._run, daemon=True)
        self.thread.start()

    def _save(self, status, stamp_ns=None):
        sample = {"host_time_ns": time.time_ns() if stamp_ns is None else stamp_ns, **status}
        self.samples.append(sample)
        self.stream.write(json.dumps(sample, ensure_ascii=False) + "\n")

    def _run(self):
        try:
            while not self.stop_event.is_set():
                start_ns = time.time_ns()
                status = self.motor.status()
                self._save(status, (start_ns + time.time_ns()) // 2)
                self.stop_event.wait(0.05)
        except (OSError, ValueError) as error:
            self.error = str(error)

    def stop(self):
        self.stop_event.set()
        self.thread.join()
        self.stream.close()


def magnet_pose(tool_quaternion_xyzw, position_counts, phase_sign,
                encoder_turns_per_magnet_turn, zero_angle_rad=0.0):
    """Magnet +X is the N pole at encoder zero; +Z is the shaft."""
    angle = (phase_sign * position_counts * 360.0 / (COUNTS_PER_TURN * encoder_turns_per_magnet_turn)
             + math.degrees(zero_angle_rad))
    rotation = Rotation.from_quat(tool_quaternion_xyzw) * Rotation.from_euler("z", angle, degrees=True)
    return {
        "magnet_phase_deg": angle,
        "magnet_quaternion_world_xyzw": rotation.as_quat().tolist(),
        "magnet_north_world": rotation.apply([1.0, 0.0, 0.0]).tolist(),
    }


def add_magnet_pose(path, motor_samples, phase_sign, encoder_turns_per_magnet_turn,
                    zero_angle_rad=0.0):
    """Merge encoder samples into one robot trajectory file after execution."""
    if not path.is_file():
        return 0
    samples = sorted(motor_samples, key=lambda sample: sample["host_time_ns"])
    result_path = path.with_suffix(".tmp")
    count = 0
    cursor = 0
    try:
        with path.open(encoding="utf-8") as incoming, result_path.open("x", encoding="utf-8") as outgoing:
            for line in incoming:
                row = json.loads(line)
                for old_field in (
                    "magnet_orientation_status", "magnet_phase_deg",
                    "magnet_quaternion_world_xyzw", "magnet_north_world",
                ):
                    row.pop(old_field, None)
                stamp = row["host_time_ns"]
                while cursor + 1 < len(samples) and samples[cursor + 1]["host_time_ns"] <= stamp:
                    cursor += 1
                nearby = samples[cursor:cursor + 2]
                if nearby and min(abs(sample["host_time_ns"] - stamp) for sample in nearby) <= 250_000_000:
                    first = nearby[0]
                    last = nearby[-1]
                    closest = min(nearby, key=lambda sample: abs(sample["host_time_ns"] - stamp))
                    if (len(nearby) == 2 and first["host_time_ns"] < stamp < last["host_time_ns"]
                            and max(stamp - first["host_time_ns"], last["host_time_ns"] - stamp) <= 250_000_000):
                        fraction = (stamp - first["host_time_ns"]) / (last["host_time_ns"] - first["host_time_ns"])
                        counts = first["position_counts"] + fraction * (last["position_counts"] - first["position_counts"])
                    else:
                        counts = closest["position_counts"]
                    row["motor_position_counts_at_robot_sample"] = counts
                    row["motor_nearest_sample_time_error_ms"] = abs(closest["host_time_ns"] - stamp) / 1e6
                    row["motor_reported_position_deg"] = counts * 360.0 / COUNTS_PER_TURN
                    if phase_sign is not None and encoder_turns_per_magnet_turn is not None:
                        row.update(magnet_pose(
                            row["tool_quaternion_world_xyzw"], counts,
                            phase_sign, encoder_turns_per_magnet_turn, zero_angle_rad,
                        ))
                    else:
                        row["magnet_orientation_status"] = "encoder_to_magnet_rotation_uncalibrated"
                        row["magnet_rotation_axis_world"] = row["motor_axis_world"]
                else:
                    row["magnet_orientation_status"] = "motor_sample_unavailable"
                outgoing.write(json.dumps(row, ensure_ascii=False, allow_nan=False) + "\n")
                count += 1
        result_path.replace(path)
    finally:
        result_path.unlink(missing_ok=True)
    return count


def copy_h_positions(output, start_ns, end_ns, sessions=DEFAULT_H_SESSIONS,
                     source_session=None):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    if source_session is None:
        statuses = list(Path(sessions).glob("*/status.json"))
        status = max(statuses, key=lambda path: path.stat().st_mtime_ns) if statuses else None
        source_session = status.parent if status and status.stat().st_mtime_ns >= end_ns - 5_000_000_000 else None
    source = Path(source_session) / "positions.csv" if source_session else None
    summary = {
        "source_session": str(source_session) if source_session else None,
        "start_host_time_ns": start_ns,
        "end_host_time_ns": end_ns,
        "rows": 0,
        "detected_rows": 0,
        "time_alignment": "camera frame timestamp versus robot host clock; no hardware trigger",
    }
    if source is not None and source.is_file():
        with source.open(newline="", encoding="utf-8") as incoming, (output / "positions.csv").open(
            "x", newline="", encoding="utf-8"
        ) as outgoing:
            reader = csv.reader(incoming)
            writer = csv.writer(outgoing)
            header = next(reader)
            if not header or header[0] != "host_frame_time_ns":
                raise ValueError(f"invalid H trajectory header: {source}")
            writer.writerow(header)
            for row in reader:
                if len(row) != len(header):
                    continue
                try:
                    stamp = int(row[0])
                except ValueError:
                    continue
                if start_ns <= stamp <= end_ns:
                    writer.writerow(row)
                    summary["rows"] += 1
                    summary["detected_rows"] += row[1] == "1"
    (output / "metadata.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8"
    )
    return summary
