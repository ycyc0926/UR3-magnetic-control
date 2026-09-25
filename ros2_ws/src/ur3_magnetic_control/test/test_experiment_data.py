import csv
import json
from pathlib import Path
import tempfile

from ur3_magnetic_control.experiment_data import add_magnet_pose, copy_h_positions


def test_copies_only_frames_during_experiment():
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        session = root / "tracking_sessions" / "session"
        session.mkdir(parents=True)
        with (session / "positions.csv").open("w", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(["host_frame_time_ns", "detected", "world_x_mm"])
            writer.writerows([(100, 1, 1), (200, 0, ""), (300, 1, 3), (400, 1, 4)])
        output = root / "experiment" / "h_robot"
        result = copy_h_positions(output, 150, 350, source_session=session)
        assert result["rows"] == 2
        assert result["detected_rows"] == 1
        with (output / "positions.csv").open(newline="") as stream:
            rows = list(csv.DictReader(stream))
        assert [row["host_frame_time_ns"] for row in rows] == ["200", "300"]
        assert json.loads((output / "metadata.json").read_text())["rows"] == 2
        (session / "status.json").write_text("{}")
        automatic = copy_h_positions(
            root / "automatic" / "h_robot", 150, 350,
            sessions=root / "tracking_sessions",
        )
        assert automatic["rows"] == 2


def test_magnet_pose_interpolates_encoder_without_claiming_unknown_magnet_angle():
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / "actual_magnet_path.jsonl"
        path.write_text(json.dumps({
            "host_time_ns": 200_000_000,
            "tool_quaternion_world_xyzw": [0, 0, 0, 1],
            "magnet_world_mm": [1, 2, 3],
            "motor_axis_world": [0, 0, 1],
        }) + "\n", encoding="utf-8")
        samples = [
            {"host_time_ns": 100_000_000, "position_counts": 0},
            {"host_time_ns": 300_000_000, "position_counts": 8192},
        ]
        assert add_magnet_pose(path, samples, None, None) == 1
        row = json.loads(path.read_text())
        assert row["motor_position_counts_at_robot_sample"] == 4096
        assert row["magnet_orientation_status"] == "encoder_to_magnet_rotation_uncalibrated"
        assert row["magnet_rotation_axis_world"] == [0, 0, 1]
        assert "magnet_north_world" not in row
        assert row["magnet_world_mm"] == [1, 2, 3]
        assert add_magnet_pose(path, samples, 1, 1) == 1
        row = json.loads(path.read_text())
        assert abs(row["magnet_north_world"][1] - 1) < 1e-12
        assert "magnet_orientation_status" not in row
        assert add_magnet_pose(path, samples, -1, 1) == 1
        row = json.loads(path.read_text())
        assert abs(row["magnet_north_world"][1] + 1) < 1e-12
