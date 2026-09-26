"""Offline tests for reusable magnet-centre path generation and plotting."""

import json
import math
from contextlib import ExitStack
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np
from ament_index_python.packages import get_package_share_directory
from moveit_msgs.msg import RobotState, RobotTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from scipy.spatial.transform import Rotation
import xacro
import yaml

from ur3_magnetic_control.acrylic_ceiling_guard import load_guard_configuration
from ur3_magnetic_control.magnet_trajectory import (
    JOINT_NAMES,
    MAX_CARTESIAN_STEP_M,
    MagnetPathGeometry,
    MagnetTrajectoryNode,
    build_circle_targets,
    build_point_targets,
    build_square_targets,
    horizontal_tool_z_target_rotation,
    hold_wrist3_zero,
    concatenate_joint_trajectories,
    joint_state_sample_time,
    load_waypoint_targets,
    main,
    ordered_waypoint_matches,
    orientation_path_distance,
    parallel_axis_target_rotation,
    parse_arguments,
    plot_trajectory,
    point_polyline_distance,
    requested_targets,
    requested_route_from_plan,
    resolve_target_height,
    robot_state_after_trajectory,
    sample_pose_guard_path,
    subdivide_route,
    table_parallel_target_rotation,
    trace_from_jsonl,
    validate_route,
    wrist3_zero_trajectory,
)
from ur3_magnetic_control.cartesian_line_move import (
    CartesianLineMove,
    duration_seconds,
    normalize_speed_scaling,
    set_duration,
)


ROOT = Path(__file__).resolve().parents[4]


class TargetGenerationTests(unittest.TestCase):
    def test_recording_switches_and_reverse_speed(self):
        for flags, no_record, no_camera in [([], False, False), (["--no"], True, True),
                                          (["--no", "camera"], False, True),
                                          (["--no-record"], True, True)]:
            parsed = parse_arguments(["point", "--target-z-mm", "400", *flags,
                                      "--motor-rpm", "-10"])
            self.assertEqual((parsed.no_record, parsed.no_camera), (no_record, no_camera))
            self.assertEqual(parsed.motor_rpm, -10)
        with self.assertRaises(SystemExit):
            parse_arguments(["point", "--target-z-mm", "400", "--no", "--output", "/tmp/run"])

    def test_point_uses_absolute_table_world_millimetres(self):
        points, metadata = build_point_targets([125.0, 30.0, 450.0])
        np.testing.assert_allclose(points, [[0.125, 0.030, 0.450]])
        self.assertEqual(metadata["shape"], "point")

    def test_xy_target_keeps_current_height_and_defaults_to_five_mm_s(self):
        parsed = parse_arguments(["point", "--target-xy-mm", "88", "148"])
        self.assertEqual(parsed.speed_mm_s, 5.0)
        targets, metadata = requested_targets(parsed)
        np.testing.assert_allclose(resolve_target_height(targets, 0.4),
                                   [[0.088, 0.148, 0.4]])
        self.assertEqual(metadata["height"], "current_magnet_z")
        for speed in (-0.1, 20.1):
            with self.subTest(speed=speed), self.assertRaisesRegex(SystemExit, r"\[0, 20\]"):
                main(["point", "--target-xy-mm", "88", "148",
                      "--speed-mm-s", str(speed)])
        self.assertIsNone(main(["point", "--target-xy-mm", "88", "148",
                                "--speed-mm-s", "0", "--execute"]))

    def test_z_target_keeps_current_xy(self):
        parsed = parse_arguments(["point", "--target-z-mm", "350", "--no-record"])
        self.assertTrue(parsed.no_record)
        targets, metadata = requested_targets(parsed)
        np.testing.assert_allclose(resolve_target_height(targets, [0.123, 0.146, 0.4]),
                                   [[0.123, 0.146, 0.35]])
        self.assertEqual(metadata["xy"], "current_magnet_xy")

    def test_wrist3_zero_moves_only_last_joint(self):
        path = wrist3_zero_trajectory([1, 2, 3, 4, 5, math.radians(10)])
        np.testing.assert_allclose(path.points[0].positions, [1, 2, 3, 4, 5, math.radians(10)])
        np.testing.assert_allclose(path.points[-1].positions, [1, 2, 3, 4, 5, 0])
        self.assertGreater(duration_seconds(path.points[-1].time_from_start), 3.7)

    def test_hold_wrist3_zero_keeps_other_joints_and_derivatives(self):
        path = wrist3_zero_trajectory([1, 2, 3, 4, 5, math.radians(10)])
        hold_wrist3_zero(path)
        for point in path.points:
            self.assertEqual(point.positions[-1], 0.0)
            self.assertEqual(point.velocities[-1], 0.0)
            self.assertEqual(point.accelerations[-1], 0.0)
            self.assertEqual(list(point.positions[:5]), [1, 2, 3, 4, 5])

    def test_square_is_closed_and_has_requested_geometry(self):
        points, metadata = build_square_targets(
            [100.0, 200.0, 300.0], 20.0, rotation_deg=31.0
        )
        values = np.asarray(points)
        self.assertEqual(values.shape, (5, 3))
        np.testing.assert_allclose(values[0], values[-1], atol=1.0e-15)
        np.testing.assert_allclose(values[:4].mean(axis=0), [0.1, 0.2, 0.3])
        np.testing.assert_allclose(
            np.linalg.norm(np.diff(values, axis=0), axis=1), 0.020
        )
        self.assertEqual(metadata["rotation_deg"], 31.0)

    def test_square_uses_current_height_and_levels_tool_z_before_motion(self):
        parsed = parse_arguments([
            "square", "--center-xy-mm", "138", "198", "--size-mm", "100",
        ])
        self.assertEqual((parsed.motor_axis_parallel, parsed.axis_alignment_phase),
                         ("tool-z-table", "before"))
        self.assertEqual(parsed.table_yaw_deg, 0.0)
        self.assertEqual(parse_arguments([
            "square", "--center-xy-mm", "138", "198", "--size-mm", "100",
            "--table-yaw-deg", "180",
        ]).table_yaw_deg, 180.0)
        targets, _ = requested_targets(parsed)
        np.testing.assert_allclose(
            resolve_target_height(targets, 0.4),
            [[0.088, 0.148, 0.4], [0.188, 0.148, 0.4],
             [0.188, 0.248, 0.4], [0.088, 0.248, 0.4],
             [0.088, 0.148, 0.4]],
        )
        horizontal = parse_arguments([
            "square", "--center-xy-mm", "138", "198", "--size-mm", "100",
            "--tool-z-parallel-table", "--tool-z-heading-deg", "55",
            "--axis-roll-deg", "125",
        ])
        self.assertEqual(horizontal.motor_axis_parallel, "tool-z-table")
        self.assertEqual(horizontal.tool_z_heading_deg, 55.0)
        self.assertEqual(horizontal.motor_axis_roll_deg, 125.0)

    def test_clockwise_square_reverses_signed_area(self):
        counterclockwise, _ = build_square_targets([0, 0, 100], 10)
        clockwise, _ = build_square_targets([0, 0, 100], 10, clockwise=True)

        def signed_area(points):
            points = np.asarray(points)
            return 0.5 * np.sum(
                points[:-1, 0] * points[1:, 1]
                - points[1:, 0] * points[:-1, 1]
            )

        self.assertGreater(signed_area(counterclockwise), 0.0)
        self.assertLess(signed_area(clockwise), 0.0)

    def test_circle_is_closed_dense_and_at_requested_radius(self):
        points, metadata = build_circle_targets([20, 30, 400], 10)
        values = np.asarray(points)
        center = np.asarray([0.020, 0.030, 0.400])
        np.testing.assert_allclose(values[0], values[-1], atol=1.0e-15)
        np.testing.assert_allclose(
            np.linalg.norm(values[:, :2] - center[:2], axis=1), 0.010
        )
        self.assertLessEqual(
            np.max(np.linalg.norm(np.diff(values, axis=0), axis=1)),
            MAX_CARTESIAN_STEP_M,
        )
        self.assertEqual(metadata["samples"], len(points) - 1)

    def test_circle_rejects_too_few_samples(self):
        minimum = math.ceil(2.0 * math.pi * 0.020 / MAX_CARTESIAN_STEP_M)
        with self.assertRaisesRegex(ValueError, "circle samples"):
            build_circle_targets([0, 0, 100], 20, samples=minimum - 1)


class MotorAxisOrientationTests(unittest.TestCase):
    def test_tool_z_can_point_horizontally(self):
        start = Rotation.from_euler("xyz", [20, -12, 33], degrees=True).as_matrix()
        aligned = horizontal_tool_z_target_rotation(start, 55, 125)
        axis = aligned["rotation_world_tool"][:, 2]
        np.testing.assert_allclose(axis, [math.cos(math.radians(55)),
                                         math.sin(math.radians(55)), 0], atol=1e-12)
        with self.assertRaisesRegex(ValueError, "heading and axis roll"):
            horizontal_tool_z_target_rotation(start, 181, 0)

    def test_level_tool_plane_also_levels_motor_axis(self):
        start = Rotation.from_euler("xyz", [71, -28, 13], degrees=True).as_matrix()
        aligned = table_parallel_target_rotation(start, [0, -1, 0], yaw_deg=180)
        result = aligned["rotation_world_tool"]
        self.assertAlmostEqual(abs(result[2, 2]), 1.0, places=12)
        self.assertAlmostEqual((result @ [0, -1, 0])[2], 0.0, places=12)
        self.assertEqual(aligned["table_yaw_deg"], 180.0)
        with self.assertRaisesRegex(ValueError, "table yaw"):
            table_parallel_target_rotation(start, [0, -1, 0], yaw_deg=181)

    def test_nearest_y_direction_uses_minimum_rotation(self):
        # tool -Y initially points mostly toward world +Z and somewhat -Y,
        # matching the sign-selection case used by the live robot.
        current_axis = np.asarray([0.3, -0.2, math.sqrt(0.87)])
        current_axis /= np.linalg.norm(current_axis)
        start = parallel_axis_target_rotation(
            np.eye(3),
            current_axis,
            "y",
            "nearest",
        )
        self.assertEqual(start["selected_direction"], "negative")
        np.testing.assert_allclose(start["target_axis_world"], [0, -1, 0])
        np.testing.assert_allclose(
            start["rotation_world_tool"] @ current_axis,
            [0, -1, 0],
            atol=1.0e-12,
        )
        self.assertAlmostEqual(
            start["rotation_angle_rad"],
            math.acos(float(np.dot(current_axis, [0, -1, 0]))),
        )

    def test_pose_guard_rotates_at_final_magnet_centre(self):
        target_rotation = np.asarray([
            [0.0, -1.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
        ])
        centres, quaternions = sample_pose_guard_path(
            [[0, 0, 0.3], [0.010, 0, 0.3]],
            np.eye(3),
            target_rotation,
            "after",
        )
        final_centre_count = np.count_nonzero(
            np.linalg.norm(centres - [0.010, 0, 0.3], axis=1) < 1.0e-12
        )
        self.assertGreater(final_centre_count, 100)
        np.testing.assert_allclose(centres[-1], [0.010, 0, 0.3])
        np.testing.assert_allclose(
            np.abs(quaternions[-1]),
            np.abs(Rotation.from_matrix(target_rotation).as_quat()),
            atol=1.0e-12,
        )

    def test_orientation_path_distance_rejects_off_path_twist(self):
        centres, quaternions = sample_pose_guard_path(
            [[0, 0, 0.3], [0.010, 0, 0.3]],
            np.eye(3),
        )
        error = orientation_path_distance(
            [0.005, 0, 0.3],
            Rotation.from_euler("x", 3.0, degrees=True).as_quat(),
            centres,
            quaternions,
            0.002,
        )
        self.assertAlmostEqual(math.degrees(error), 3.0, places=10)


class WaypointAndRouteTests(unittest.TestCase):
    def test_loads_yaml_json_and_csv_waypoints(self):
        expected = np.asarray([[0.001, 0.002, 0.003], [0.004, 0.005, 0.006]])
        with tempfile.TemporaryDirectory(prefix="magnet_waypoints_") as directory:
            root = Path(directory)
            yaml_path = root / "points.yaml"
            yaml_path.write_text(
                "frame: table_world\nwaypoints_mm:\n"
                "  - [1, 2, 3]\n  - [4, 5, 6]\n",
                encoding="utf-8",
            )
            json_path = root / "points.json"
            json_path.write_text(json.dumps([[1, 2, 3], [4, 5, 6]]))
            csv_path = root / "points.csv"
            csv_path.write_text(
                "x_mm,y_mm,z_mm\n1,2,3\n4,5,6\n", encoding="utf-8"
            )
            for path in (yaml_path, json_path, csv_path):
                with self.subTest(path=path.name):
                    points, metadata = load_waypoint_targets(path)
                    np.testing.assert_allclose(points, expected)
                    self.assertEqual(metadata["count"], 2)

    def test_wrong_waypoint_frame_is_rejected(self):
        with tempfile.TemporaryDirectory(prefix="magnet_waypoints_") as directory:
            path = Path(directory) / "points.json"
            path.write_text(
                json.dumps({"frame": "base", "waypoints_mm": [[1, 2, 3]]})
            )
            with self.assertRaisesRegex(ValueError, "table_world"):
                load_waypoint_targets(path)

    def test_route_includes_approach_and_enforces_limits(self):
        route, segments, total = validate_route(
            [0.0, 0.0, 0.1], [[0.01, 0.0, 0.1], [0.01, 0.02, 0.1]]
        )
        self.assertEqual(len(route), 3)
        np.testing.assert_allclose(segments, [0.01, 0.02])
        self.assertAlmostEqual(total, 0.03)
        for distance in (0.350, 0.400):
            _, segments, total = validate_route([0, 0, 0.1], [[distance, 0, 0.1]])
            self.assertEqual(segments, [distance])
            self.assertEqual(total, distance)
        with self.assertRaisesRegex(ValueError, "400 mm"):
            validate_route([0, 0, 0.1], [[0.401, 0, 0.1]])

    def test_point_to_polyline_distance(self):
        distance = point_polyline_distance(
            [0.5, 0.25, 0.0], [[0, 0, 0], [1, 0, 0]]
        )
        self.assertAlmostEqual(distance, 0.25)

    def test_dense_polyline_distance_is_fast_enough_for_live_monitoring(self):
        vertices = np.column_stack((
            np.linspace(0.0, 1.0, 20_000),
            np.zeros(20_000),
            np.zeros(20_000),
        ))
        started = time.perf_counter()
        for _ in range(5):
            self.assertAlmostEqual(
                point_polyline_distance([0.5, 0.002, 0.0], vertices),
                0.002,
                places=9,
            )
        self.assertLess(time.perf_counter() - started, 0.25)

    def test_polyline_distance_handles_repeated_vertices(self):
        self.assertAlmostEqual(
            point_polyline_distance(
                [0.5, 0.25, 0.0],
                [[0, 0, 0], [0, 0, 0], [1, 0, 0]],
            ),
            0.25,
        )

    def test_waypoints_are_matched_in_path_order(self):
        matches = ordered_waypoint_matches(
            [[0, 0, 0], [1, 0, 0], [0, 0, 0]],
            [[0, 0, 0], [0.5, 0, 0], [1, 0, 0],
             [0.5, 0, 0], [0, 0, 0]],
        )
        self.assertEqual(
            [match["sample_index"] for match in matches], [0, 2, 4]
        )
        self.assertEqual(max(match["error_m"] for match in matches), 0.0)

    def test_route_is_subdivided_without_changing_endpoint(self):
        chunks = subdivide_route(
            [[0.0, 0.0, 0.0], [0.060, 0.0, 0.0]], 0.025
        )
        np.testing.assert_allclose(
            chunks,
            [[0.020, 0.0, 0.0],
             [0.040, 0.0, 0.0],
             [0.060, 0.0, 0.0]],
        )
        with self.assertRaisesRegex(ValueError, "planning chunk"):
            subdivide_route([[0, 0, 0], [0.01, 0, 0]], 0.001)

    def test_segment_trajectories_are_joined_with_increasing_time(self):
        segments = []
        for start, finish, duration in ((0.0, 1.0, 1.0), (1.0, 2.0, 2.0)):
            trajectory = JointTrajectory(joint_names=["joint"])
            left = JointTrajectoryPoint(positions=[start])
            right = JointTrajectoryPoint(positions=[finish])
            set_duration(left.time_from_start, 0.0)
            set_duration(right.time_from_start, duration)
            trajectory.points = [left, right]
            segments.append(trajectory)
        combined = concatenate_joint_trajectories(segments)
        self.assertEqual(
            [point.positions[0] for point in combined.points],
            [0.0, 1.0, 2.0],
        )
        self.assertEqual(
            [duration_seconds(point.time_from_start) for point in combined.points],
            [0.0, 1.0, 3.0],
        )

    def test_robot_state_advances_to_segment_endpoint(self):
        state = RobotState()
        state.joint_state = JointState(
            name=["other", "joint"], position=[3.0, 0.0]
        )
        trajectory = JointTrajectory(joint_names=["joint"])
        trajectory.points = [JointTrajectoryPoint(positions=[2.0])]
        advanced = robot_state_after_trajectory(state, trajectory)
        np.testing.assert_allclose(advanced.joint_state.position, [3.0, 2.0])
        np.testing.assert_allclose(state.joint_state.position, [3.0, 0.0])


class SavedTraceTests(unittest.TestCase):
    def test_reads_jsonl_time_and_plots_saved_plan_and_measurement(self):
        trace = [
            {"time_s": 0.0, "magnet_world_mm": [10.0, 20.0, 30.0]},
            {"time_s": 1.0, "magnet_world_mm": [11.0, 22.0, 33.0]},
        ]
        actual = [
            {"monotonic_s": 8.0, "magnet_world_mm": [10.0, 20.0, 30.0]},
            {"monotonic_s": 8.5, "magnet_world_mm": [10.5, 21.0, 31.5]},
        ]
        with tempfile.TemporaryDirectory(prefix="magnet_trace_") as directory:
            root = Path(directory)
            (root / "plan.json").write_text(
                json.dumps({
                    "planned_trace": trace,
                    "planning": {
                        "route_world_m": [[0.010, 0.020, 0.030],
                                          [0.011, 0.022, 0.033]]
                    },
                }),
                encoding="utf-8",
            )
            jsonl = root / "actual_magnet_path.jsonl"
            jsonl.write_text(
                "".join(json.dumps(value) + "\n" for value in actual),
                encoding="utf-8",
            )
            loaded = trace_from_jsonl(jsonl)
            self.assertEqual([value["time_s"] for value in loaded], [0.0, 0.5])
            np.testing.assert_allclose(
                requested_route_from_plan(root / "plan.json"),
                [[10.0, 20.0, 30.0], [11.0, 22.0, 33.0]],
            )
            output = plot_trajectory(root, root / "magnet_path.png")
            self.assertEqual(output, root / "magnet_path.png")
            self.assertGreater(output.stat().st_size, 1000)


class ExecutionGateTests(unittest.TestCase):
    def test_recording_modes_in_planning_execution_and_failure(self):
        from ur3_magnetic_control import magnet_trajectory as module
        from ur3_magnetic_control.experiment_data import active_experiment, current_experiment

        for flags in ([], ["--no"], ["--no-record"], ["--no", "camera"]):
            for outcome in ("plan", "execute", "failure", "home_failure", "stop_failure"):
                with self.subTest(flags=flags, outcome=outcome), tempfile.TemporaryDirectory() as directory, ExitStack() as stack:
                    output = Path(directory) / "run"
                    no_record = flags in (["--no"], ["--no-record"])
                    context_file = Path(directory) / "active"
                    context = stack.enter_context(patch.object(
                        module, "active_experiment",
                        side_effect=lambda path: active_experiment(path, context_file)))
                    measured = {"magnet_world_mm": [146., 146., 400.],
                                "motor_axis_world": [0., 0., 1.],
                                "tool_normal_world": [0., 0., 1.],
                                "tool_quaternion_world_xyzw": [0., 0., 0., 1.]}
                    node = Mock()
                    node.latest_joint_state = JointState(name=list(JOINT_NAMES),
                                                        position=[0.] * 6, velocity=[0.] * 6)
                    node.joint_state_received_at = time.monotonic()
                    node.geometry.inspect.return_value = measured
                    # A stopped External Control program may not have published
                    # its first running-state message yet. Planning must still work.
                    node.robot_program_running = None
                    node.speed_scaling_percent = 0.
                    node.wait_for_fresh_state.side_effect = lambda timeout=5., require_execution_state=False: MagnetTrajectoryNode.wait_for_fresh_state(
                        node, timeout=.02, require_execution_state=require_execution_state)
                    node.tool_orientation_reference_world = None
                    node.table_parallel_start_world = None
                    node.target_motor_axis_world = node.target_tool_z_world = None
                    node.actual_stream = None
                    node.execution_goal_sent = outcome != "plan"
                    node.dashboard_stop_message = None
                    node.ceiling_guard.tool_data = {"execution_allowed": True}
                    node.verify_start.return_value = 0.
                    node.plan_magnet_targets.return_value = {
                        "solution": RobotTrajectory(), "start_state": RobotState(),
                        "planned_trace": [{"time_s": 0., **measured}],
                        "targets_world_m": [[.146, .146, .4]],
                        "start_magnet_world_m": [.146, .146, .4],
                        "fraction": 1., "total_length_m": 0., "duration_s": 1.,
                        "motor_axis_alignment": None,
                    }
                    node.start_actual_recording.side_effect = lambda path: MagnetTrajectoryNode.start_actual_recording(node, path)
                    node.stop_actual_recording.side_effect = lambda: MagnetTrajectoryNode.stop_actual_recording(node)
                    node.monitor_execution_state.side_effect = lambda state: MagnetTrajectoryNode.monitor_execution_state(node, state)
                    if outcome == "failure":
                        node.execute.side_effect = RuntimeError("simulated execution failure")
                    def create_node():
                        self.assertEqual(current_experiment(context_file),
                                         output if outcome != "plan" and not no_record else None)
                        return node
                    stack.enter_context(patch.object(module, "MagnetTrajectoryNode", side_effect=create_node))
                    startup = stack.enter_context(patch.object(module, "prepare_motion_stack"))
                    external = stack.enter_context(patch.object(module, "ensure_external_control"))
                    def connect_external(_node):
                        _node.robot_program_running = True
                        _node.speed_scaling_percent = 100.
                    external.side_effect = connect_external
                    order = Mock()
                    order.attach_mock(startup, "prepare")
                    order.attach_mock(node.plan_magnet_targets, "plan")
                    order.attach_mock(external, "play")
                    order.attach_mock(node.execute, "execute")
                    ros = stack.enter_context(patch.object(module, "rclpy"))
                    ros.spin_once.side_effect = lambda *a, **k: setattr(node, "joint_state_received_at", time.monotonic())
                    motor = stack.enter_context(patch.object(module, "ZE300Motor")).return_value
                    order.attach_mock(motor.speed, "motor")
                    motor.status.return_value = motor.speed.return_value = {"position_counts": 0, "speed_rpm": -10}
                    homed_status = {"commanded_delta_deg": 1., "position_counts": 0,
                                    "position_deg": 0., "single_turn_deg": 0.,
                                    "speed_rpm": 0., "enabled": True, "fault_code": 0}
                    def return_motor_to_origin(_motor):
                        self.assertEqual(_motor.speed.call_args.args, (0,))
                        if outcome == "home_failure":
                            raise TimeoutError("simulated homing timeout")
                        _motor.status.return_value = homed_status
                        return homed_status
                    home = stack.enter_context(patch.object(module, "restore_origin", side_effect=return_motor_to_origin))
                    order.attach_mock(home, "home")
                    if outcome == "stop_failure":
                        def fail_stop(rpm):
                            if rpm == 0:
                                raise OSError("simulated motor stop failure")
                            return motor.speed.return_value
                        motor.speed.side_effect = fail_stop
                    recorder = stack.enter_context(patch.object(module, "MotorRecorder", wraps=module.MotorRecorder))
                    writer = stack.enter_context(patch.object(module, "write_json_exclusive", wraps=module.write_json_exclusive))
                    plot = stack.enter_context(patch.object(module, "plot_trajectory"))
                    destination = stack.enter_context(patch.object(module, "default_output_directory", return_value=output))
                    arguments = ["point", "--target-mm", "146", "146", "400", "--motor-rpm", "-10", *flags]
                    if outcome != "plan":
                        arguments.append("--execute")
                    if outcome in ("failure", "home_failure", "stop_failure"):
                        with self.assertRaises(SystemExit):
                            main(arguments)
                        motor.speed.assert_any_call(0)
                        self.assertEqual(str(node.get_logger.return_value.error.call_args.args[0]),
                                         {"failure": "simulated execution failure",
                                          "home_failure": "simulated homing timeout",
                                          "stop_failure": "simulated motor stop failure"}[outcome])
                    else:
                        main(arguments)
                    self.assertEqual(context.call_count, int(outcome != "plan" and not no_record))
                    self.assertIsNone(current_experiment(context_file))
                    self.assertFalse((output / "h_robot").exists())
                    startup.assert_called_once_with(node, ROOT, no_record=no_record)
                    if outcome != "plan":
                        external.assert_called_once_with(node)
                        self.assertEqual([item[0] for item in order.mock_calls[:4]],
                                         ["prepare", "plan", "play", "motor"])
                        motor.speed.assert_any_call(-10.)
                        node.execute.assert_called_once()
                        node.start_actual_recording.assert_called_once_with(None if no_record else output / "actual_magnet_path.jsonl")
                    else:
                        external.assert_not_called()
                        motor.speed.assert_not_called()
                    if outcome in ("execute", "home_failure"):
                        home.assert_called_once_with(motor)
                        self.assertEqual([item[0] for item in order.mock_calls[:7]],
                                         ["prepare", "plan", "play", "motor", "execute", "motor", "home"])
                    else:
                        home.assert_not_called()
                    if no_record:
                        self.assertEqual(list(Path(directory).iterdir()), [])
                        destination.assert_not_called()
                        writer.assert_not_called()
                        plot.assert_not_called()
                        recorder.assert_not_called()
                        ros.init.assert_called_once_with(args=["--ros-args", "--disable-external-lib-logs"])
                        if outcome == "execute":
                            self.assertEqual(node.actual_samples.maxlen, 1)
                    else:
                        self.assertTrue((output / "plan.json").is_file())
                        if outcome != "plan":
                            self.assertTrue((output / "motor_samples.jsonl").is_file())
                            self.assertTrue((output / ("execution.json" if outcome == "execute" else "failure.json")).is_file())
                        if outcome == "execute":
                            execution = json.loads((output / "execution.json").read_text())
                            self.assertEqual(execution["motor_return_to_origin"], homed_status)
                    self.assertFalse((output / "h_robot").exists())

    def test_startup_or_planning_failure_never_starts_motion(self):
        from ur3_magnetic_control import magnet_trajectory as module

        for stage in ("startup", "planning"):
            with self.subTest(stage=stage), ExitStack() as stack:
                node = Mock()
                node.latest_joint_state = JointState(position=[0.] * 6, velocity=[0.] * 6)
                node.plan_magnet_targets.side_effect = RuntimeError("planning failed")
                stack.enter_context(patch.object(module, "MagnetTrajectoryNode", return_value=node))
                stack.enter_context(patch.object(module, "rclpy"))
                startup = stack.enter_context(patch.object(module, "prepare_motion_stack"))
                external = stack.enter_context(patch.object(module, "ensure_external_control"))
                motor = stack.enter_context(patch.object(module, "ZE300Motor"))
                if stage == "startup":
                    startup.side_effect = RuntimeError("startup failed")
                with self.assertRaises(SystemExit):
                    main(["point", "--target-mm", "146", "146", "400",
                          "--motor-rpm", "10", "--no", "--execute"])
                motor.assert_not_called()
                node.execute.assert_not_called()
                external.assert_not_called()

    def test_live_monitor_records_motion_and_propagates_boundary_failure(self):
        checked = {"magnet_world_mm": [100.0, 20.0, 300.0]}
        geometry = SimpleNamespace(inspect=lambda names, positions: checked)
        message = JointState(name=list(JOINT_NAMES), position=[100.0] * 5 + [0.0])
        runner = SimpleNamespace(
            robot_program_running=True,
            joint_state_received_at=time.monotonic(),
            latest_joint_state=message,
            last_logged_state_time=None,
            geometry=geometry,
            actual_samples=[],
            actual_stream=None,
            recording_error=None,
            get_logger=lambda: SimpleNamespace(warning=lambda message: None),
            table_parallel_start_world=None,
            table_parallel_mode=None,
            table_parallel_ready=False,
            tool_orientation_reference_world=None,
        )
        MagnetTrajectoryNode.monitor_execution_state(runner, RobotState())
        self.assertEqual(runner.actual_samples[0]["magnet_world_mm"], checked["magnet_world_mm"])
        self.assertNotIn("magnet_speed_mm_s", runner.actual_samples[0])
        runner.actual_stream = SimpleNamespace(
            write=lambda value: (_ for _ in ()).throw(OSError("disk full")),
            close=lambda: None,
        )
        runner.last_logged_state_time = None
        MagnetTrajectoryNode.monitor_execution_state(runner, RobotState())
        self.assertEqual(runner.recording_error, "disk full")
        self.assertIsNone(runner.actual_stream)
        runner.table_parallel_start_world = np.asarray([0.1, 0.02, 0.3])
        checked.update(magnet_world_mm=[103.0, 20.0, 300.0],
                       tool_normal_world=[0.0, 1.0, 0.0])
        runner.last_logged_state_time = None
        with self.assertRaisesRegex(RuntimeError, "tool0 axis differs"):
            MagnetTrajectoryNode.monitor_execution_state(runner, RobotState())
        runner.table_parallel_mode = "tool-z-table"
        checked["tool_normal_world"] = [0.0, 0.0, 1.0]
        runner.last_logged_state_time = None
        with self.assertRaisesRegex(RuntimeError, "tool0 axis differs"):
            MagnetTrajectoryNode.monitor_execution_state(runner, RobotState())
        runner.table_parallel_start_world = None
        geometry.inspect = lambda names, positions: (_ for _ in ()).throw(
            RuntimeError("ceiling clearance failed")
        )
        runner.last_logged_state_time = None
        with self.assertRaisesRegex(RuntimeError, "ceiling clearance failed"):
            MagnetTrajectoryNode.monitor_execution_state(runner, RobotState())

    def test_dashboard_stop_response_is_verified(self):
        response = SimpleNamespace(success=True, message="Stopped")
        future = SimpleNamespace(done=lambda: True, result=lambda: response)
        client = SimpleNamespace(
            wait_for_service=lambda timeout_sec: True,
            call_async=lambda request: future,
        )
        runner = SimpleNamespace(
            dashboard_stop_client=client,
            dashboard_stop_attempted=False,
            dashboard_stop_succeeded=False,
            dashboard_stop_message=None,
        )
        with patch(
            "ur3_magnetic_control.cartesian_line_move.rclpy.spin_until_future_complete"
        ):
            message = CartesianLineMove.stop_robot_program(runner)
        self.assertEqual(message, "Stopped")
        self.assertTrue(runner.dashboard_stop_attempted)
        self.assertTrue(runner.dashboard_stop_succeeded)

    def test_execution_guard_stops_program_before_cancelling_action(self):
        events = []

        class Future:
            def __init__(self, result=None, done=False):
                self._result = result
                self._done = done

            def result(self):
                return self._result

            def done(self):
                return self._done

        class GoalHandle:
            accepted = True

            @staticmethod
            def get_result_async():
                return Future(done=False)

            @staticmethod
            def cancel_goal_async():
                events.append("cancel")
                return Future(done=True)

        action_client = SimpleNamespace(
            wait_for_server=lambda timeout_sec: True,
            send_goal_async=lambda goal: Future(GoalHandle(), done=True),
        )
        runner = SimpleNamespace(
            execution_goal_sent=False,
            robot_program_running=True,
            speed_scaling_percent=100.0,
            execute_client=action_client,
            dashboard_stop_client=SimpleNamespace(
                wait_for_service=lambda timeout_sec: True
            ),
            joint_state_received_at=time.monotonic(),
            execution_validator=lambda state: (_ for _ in ()).throw(
                RuntimeError("guard trip")
            ),
            current_robot_state=lambda: RobotState(),
            verify_start=lambda trajectory, tolerance=0.002: 0.0,
            stop_robot_program=lambda timeout=3.0: (
                events.append("dashboard_stop") or "Stopped"
            ),
            get_logger=lambda: SimpleNamespace(
                error=lambda message: None,
                fatal=lambda message: None,
            ),
        )
        trajectory = RobotTrajectory()
        endpoint = JointTrajectoryPoint()
        set_duration(endpoint.time_from_start, 1.0)
        trajectory.joint_trajectory.points = [endpoint]
        with patch(
            "ur3_magnetic_control.cartesian_line_move.rclpy.spin_until_future_complete"
        ), patch(
            "ur3_magnetic_control.cartesian_line_move.rclpy.spin_once"
        ), self.assertRaisesRegex(RuntimeError, "guard trip"):
            CartesianLineMove.execute(runner, trajectory)
        self.assertEqual(events, ["dashboard_stop", "cancel"])


class CurrentGeometryRegressionTests(unittest.TestCase):
    def test_known_calibrated_state_maps_to_expected_magnet_and_clearances(self):
        description = xacro.process_file(
            str(
                Path(get_package_share_directory("ur_description"))
                / "urdf/ur.urdf.xacro"
            ),
            mappings={
                "name": "ur",
                "ur_type": "ur3",
                "use_fake_hardware": "true",
                "kinematics_params": str(ROOT / "config/ur3_calibration.yaml"),
            },
        ).toxml()
        tool = yaml.safe_load(
            (ROOT / "config/magnet_tool_geometry_draft.yaml").read_text()
        )["provisional_whole_tool_envelope"]
        guard = SimpleNamespace(
            configuration=load_guard_configuration(),
            tool_lower=np.asarray(tool["min_xyz_m"], dtype=float),
            tool_upper=np.asarray(tool["max_xyz_m"], dtype=float),
        )
        geometry = MagnetPathGeometry(description, guard)
        joints = [
            -1.491387192402975,
            -1.9344733397113245,
            -1.4286039511310022,
            0.3999532461166382,
            3.0628597736358643,
            1.7700637578964233,
        ]
        result = geometry.inspect(JOINT_NAMES, joints)
        self.assertEqual(len(result["tool0_world_mm"]), 3)
        np.testing.assert_allclose(
            result["magnet_world_mm"],
            [186.435347, 37.991101, 434.475648],
            atol=1.0e-5,
        )
        required_mm = {
            "ceiling": 7.0,
            "left": 12.0,
            "right": 12.0,
            "table": 12.0,
        }
        for boundary, required in required_mm.items():
            self.assertGreaterEqual(result["gaps_mm"][boundary], required)
        table_limit = geometry.limits["table"]
        geometry.limits["table"] = result["gaps_mm"]["table"] / 1000.0 - 0.001
        geometry.inspect(JOINT_NAMES, joints)
        geometry.limits["table"] = table_limit
        for boundary in required_mm:
            original = geometry.limits[boundary]
            geometry.limits[boundary] = result["gaps_mm"][boundary] / 1000.0 + 0.001
            with self.assertRaisesRegex(RuntimeError, f"{boundary} clearance failed"):
                geometry.inspect(JOINT_NAMES, joints)
            geometry.limits[boundary] = original


class ControllerInterpolationTests(unittest.TestCase):
    def test_each_spline_segment_is_collision_checked_at_interior_samples(self):
        checked = []
        guard = SimpleNamespace(
            validate_state=lambda state, label: checked.append(
                (list(state.joint_state.position), label)
            )
        )
        runner = SimpleNamespace(ceiling_guard=guard)
        start = RobotState()
        start.joint_state = JointState(name=["joint"], position=[0.0])
        trajectory = JointTrajectory(joint_names=["joint"])
        left = JointTrajectoryPoint(positions=[0.0])
        right = JointTrajectoryPoint(positions=[1.0])
        set_duration(left.time_from_start, 0.0)
        set_duration(right.time_from_start, 2.0)
        trajectory.points = [left, right]
        count = MagnetTrajectoryNode.validate_controller_interpolation(
            runner, trajectory, start
        )
        self.assertEqual(count, 3)
        np.testing.assert_allclose(
            [sample[0][0] for sample in checked], [0.25, 0.5, 0.75]
        )
        self.assertTrue(all("controller spline" in item[1] for item in checked))

    def test_speed_scaling_accepts_driver_factor_and_legacy_percent(self):
        self.assertEqual(normalize_speed_scaling(0.08), (0.08, 8.0))
        self.assertEqual(normalize_speed_scaling(8.0), (0.08, 8.0))
        for value in (0.0, float("nan"), 101.0):
            with self.subTest(value=value), self.assertRaises(ValueError):
                normalize_speed_scaling(value)

    def test_joint_state_derivatives_use_driver_stamp_not_callback_arrival(self):
        message = JointState()
        message.header.stamp.sec = 123
        message.header.stamp.nanosec = 456_000_000
        self.assertAlmostEqual(
            joint_state_sample_time(message, fallback_monotonic_s=7.0),
            123.456,
        )
        message.header.stamp.sec = 0
        message.header.stamp.nanosec = 0
        self.assertEqual(
            joint_state_sample_time(message, fallback_monotonic_s=7.0),
            7.0,
        )

if __name__ == "__main__":
    unittest.main()
