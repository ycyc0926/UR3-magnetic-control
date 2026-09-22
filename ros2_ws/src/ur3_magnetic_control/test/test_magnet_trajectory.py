"""Offline tests for reusable magnet-centre path generation and plotting."""

import json
import math
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

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
    EXECUTION_TOKEN,
    JOINT_NAMES,
    MAX_CARTESIAN_STEP_M,
    MagnetPathGeometry,
    MagnetTrajectoryNode,
    build_trajectory_trace,
    build_circle_targets,
    build_point_targets,
    build_square_targets,
    concatenate_joint_trajectories,
    joint_state_sample_time,
    load_waypoint_targets,
    main,
    ordered_waypoint_matches,
    orientation_path_distance,
    parallel_axis_target_rotation,
    plot_trajectory,
    point_polyline_distance,
    requested_route_from_plan,
    robot_state_after_trajectory,
    sample_pose_guard_path,
    subdivide_route,
    trace_from_jsonl,
    validate_route,
)
from ur3_magnetic_control.cartesian_line_move import (
    CartesianLineMove,
    duration_seconds,
    normalize_speed_scaling,
    set_duration,
)


ROOT = Path(__file__).resolve().parents[4]


class TargetGenerationTests(unittest.TestCase):
    def test_point_uses_absolute_table_world_millimetres(self):
        points, metadata = build_point_targets([125.0, 30.0, 450.0])
        np.testing.assert_allclose(points, [[0.125, 0.030, 0.450]])
        self.assertEqual(metadata["shape"], "point")

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
        with self.assertRaisesRegex(ValueError, "300 mm"):
            validate_route([0, 0, 0.1], [[0.301, 0, 0.1]])

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
    def test_execution_requires_fresh_onsite_confirmations_before_ros_init(self):
        with self.assertRaisesRegex(SystemExit, "fresh onsite confirmations"):
            main([
                "point",
                "--target-mm", "1", "2", "300",
                "--execute",
                "--confirmation-token", EXECUTION_TOKEN,
                "--accept-provisional-tool-envelope",
                "--motor-stopped",
            ])

    def test_execution_requires_provisional_envelope_acknowledgement(self):
        with self.assertRaisesRegex(SystemExit, "provisional"):
            main([
                "point",
                "--target-mm", "1", "2", "300",
                "--execute",
                "--confirmation-token", EXECUTION_TOKEN,
                "--motor-stopped",
                "--onsite-clearance-confirmed",
                "--sole-operator-confirmed",
                "--external-control-only-confirmed",
            ])

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
        system = yaml.safe_load((ROOT / "config/ur3_system.yaml").read_text())
        geometry = MagnetPathGeometry(description, guard, system)
        joints = [
            -1.491387192402975,
            -1.9344733397113245,
            -1.4286039511310022,
            0.3999532461166382,
            3.0628597736358643,
            1.7700637578964233,
        ]
        result = geometry.inspect(JOINT_NAMES, joints)
        np.testing.assert_allclose(
            result["magnet_world_mm"],
            [243.505538, 99.444644, 432.830871],
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

    def test_workspace_ingress_only_accepts_preexisting_bounded_violation(self):
        geometry = MagnetPathGeometry.__new__(MagnetPathGeometry)
        geometry.workspace_lower = np.asarray([-1.0, -1.0, -1.0])
        geometry.workspace_upper = np.asarray([1.0, 1.0, 1.0])
        geometry.workspace_ingress_limit = np.zeros(6)
        with self.assertRaisesRegex(RuntimeError, "workspace-ingress"):
            geometry.configure_workspace_ingress([0.0, 0.0, 1.2], False)
        violation = geometry.configure_workspace_ingress(
            [0.0, 0.0, 1.2], True
        )
        np.testing.assert_allclose(violation, [0, 0, 0, 0, 0, 0.2])

    def test_workspace_ingress_cannot_leave_again(self):
        class FakeGeometry:
            @staticmethod
            def inspect(_names, positions, allow_workspace_ingress=False):
                value = float(positions[0])
                violation = max(-value, 0.0)
                return {
                    "magnet_world_mm": [value * 1000.0, 0.0, 0.0],
                    "tool_quaternion_world_xyzw": [0.0, 0.0, 0.0, 1.0],
                    "workspace_outside": violation > 0.0,
                    "workspace_violation_mm": [violation * 1000.0, 0, 0, 0, 0, 0],
                }

        trajectory = JointTrajectory(joint_names=["joint"])
        points = []
        for timestamp, position in enumerate((-1.0, 1.0, -1.0)):
            point = JointTrajectoryPoint(positions=[position])
            set_duration(point.time_from_start, float(timestamp))
            points.append(point)
        trajectory.points = points
        with self.assertRaisesRegex(RuntimeError, "left the workspace"):
            build_trajectory_trace(
                trajectory,
                FakeGeometry(),
                allow_workspace_ingress=True,
            )


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
