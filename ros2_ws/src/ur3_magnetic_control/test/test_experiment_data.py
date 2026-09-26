import json
from pathlib import Path
import subprocess
import sys
import tempfile

from ur3_magnetic_control.experiment_data import active_experiment, current_experiment, add_magnet_pose


def test_active_experiment_exists_only_while_producer_holds_lock():
    with tempfile.TemporaryDirectory() as directory:
        experiment = Path(directory)/'run'
        experiment.mkdir()
        context_file = Path(directory)/'active'
        assert current_experiment(context_file) is None
        try:
            with active_experiment(experiment, context_file):
                assert current_experiment(context_file) == experiment
                try:
                    with active_experiment(experiment, context_file):
                        raise AssertionError('Competing experiment was allowed')
                except RuntimeError:
                    pass
                assert current_experiment(context_file) == experiment
                raise ValueError('Simulated experiment failure')
        except ValueError:
            pass
        assert current_experiment(context_file) is None
        assert context_file.read_text() == ''
        # A hard exit leaves the path behind, but the OS releases its lock.
        subprocess.run([sys.executable, '-c',
                        'import os, sys; from pathlib import Path; '
                        'from ur3_magnetic_control.experiment_data import active_experiment; '
                        'context = active_experiment(sys.argv[1], Path(sys.argv[2])); '
                        'context.__enter__(); os._exit(0)',
                        str(experiment), str(context_file)], check=True)
        assert context_file.read_text() == str(experiment)
        assert current_experiment(context_file) is None
        with active_experiment(experiment, context_file):
            assert current_experiment(context_file) == experiment


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
