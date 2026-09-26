"""Offline startup checks; no driver or robot program is started by these tests."""

from contextlib import ExitStack
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

from ur3_magnetic_control import motion_startup as startup


class MotionStartupTests(unittest.TestCase):
    def test_service_reuse_and_missing_components(self):
        for driver, moveit, names, expected, fail in (
            (True, True, [], [], False),
            (False, False, [], ["driver", "moveit"], False),
            (True, False, [], ["moveit"], False),
            (False, True, [], ["driver"], False),
            (False, False, ["controller_manager", "move_group"], [], False),
            (True, True, ["controller_manager"] * 2, [], True),
        ):
            with self.subTest(driver=driver, moveit=moveit, names=names), ExitStack() as stack:
                node = Mock()
                controllers = node.create_client.return_value
                controllers.wait_for_service.return_value = driver
                node.cartesian_client.wait_for_service.return_value = moveit
                node.get_node_names.return_value = names
                node.call.return_value.controller = [NS(name=name, state="active")
                                                     for name in startup.STATE_CONTROLLERS]
                start = stack.enter_context(patch.object(startup, "start_component"))
                stack.enter_context(patch.object(startup.rclpy, "spin_once"))
                if fail:
                    with self.assertRaisesRegex(RuntimeError, "Multiple"):
                        startup.prepare_motion_stack(node, Path("/project"), no_record=True)
                else:
                    startup.prepare_motion_stack(node, Path("/project"), no_record=True)
                    node.wait_for_fresh_state.assert_called_once()
                self.assertEqual([call.args for call in start.call_args_list],
                                 [(name, Path("/project"), True) for name in expected])
                node.destroy_client.assert_called_once_with(controllers)
        node = Mock()
        node.get_node_names.return_value = []
        with self.assertRaisesRegex(RuntimeError, "timed out"):
            startup.prepare_motion_stack(node, Path("/project"), timeout=0)

    def test_external_control_gates_and_failed_play_cleanup(self):
        for scenario in ("stopped", "playing", "wrong_program", "safety_stop", "power_off",
                         "paused", "play_rejected", "timeout", "playing_timeout", "stop_failed",
                         "controller_inactive", "zero_speed"):
            with self.subTest(scenario=scenario), ExitStack() as stack:
                replies = {
                    "get_loaded_program": NS(success=True, program_name=startup.EXTERNAL_PROGRAM),
                    "get_safety_mode": NS(success=True, safety_mode=NS(mode=startup.SafetyMode.NORMAL)),
                    "get_robot_mode": NS(success=True, robot_mode=NS(mode=startup.RobotMode.RUNNING)),
                    "program_state": NS(success=True, state=NS(state="STOPPED")),
                    "play": NS(success=True),
                    "list_controllers": NS(controller=[NS(name="scaled_joint_trajectory_controller", state="active")]),
                }
                node = Mock(robot_program_running=True, speed_scaling_percent=100.)
                node.create_client.side_effect = lambda service, name: name.rsplit("/", 1)[-1]
                node.call.side_effect = lambda client, request, **kw: replies[client]
                if scenario.startswith("playing"):
                    replies["program_state"].state.state = "PLAYING"
                if scenario == "wrong_program":
                    replies["get_loaded_program"].program_name = "/programs/other.urp"
                if scenario == "safety_stop":
                    replies["get_safety_mode"].safety_mode.mode = startup.SafetyMode.PROTECTIVE_STOP
                if scenario == "power_off":
                    replies["get_robot_mode"].robot_mode.mode = startup.RobotMode.POWER_OFF
                if scenario == "paused":
                    replies["program_state"].state.state = "PAUSED"
                if scenario == "play_rejected":
                    replies["play"] = NS(success=False, message="Play refused")
                if scenario in ("timeout", "playing_timeout", "stop_failed"):
                    node.robot_program_running = False
                if scenario == "stop_failed":
                    node.stop_robot_program.side_effect = RuntimeError("Stop failed")
                if scenario == "controller_inactive":
                    replies["list_controllers"].controller[0].state = "inactive"
                if scenario == "zero_speed":
                    node.speed_scaling_percent = 0.
                clock = [0.]
                stack.enter_context(patch.object(startup.time, "monotonic", side_effect=lambda: clock[0]))
                stack.enter_context(patch.object(startup.rclpy, "spin_once",
                                               side_effect=lambda *a, **k: clock.__setitem__(0, clock[0] + .1)))
                succeeds = scenario in ("stopped", "playing")
                if succeeds:
                    startup.ensure_external_control(node, timeout=.2)
                else:
                    with self.assertRaises(RuntimeError):
                        startup.ensure_external_control(node, timeout=.2)
                calls = [call.args[0] for call in node.call.call_args_list]
                played = scenario in ("stopped", "play_rejected", "timeout", "stop_failed",
                                      "controller_inactive", "zero_speed")
                self.assertEqual(calls.count("play"), int(played))
                self.assertEqual(node.stop_robot_program.call_count, int(played and not succeeds))
                self.assertEqual(node.destroy_client.call_count, node.create_client.call_count)

    def test_native_service_command_and_no_record_launch(self):
        with patch.object(startup.subprocess, "run") as run:
            run.return_value = NS(stdout="active", returncode=0)
            startup.start_component("driver", Path("/project"), no_record=True)
            self.assertEqual(run.call_count, 1)
            run.reset_mock()
            run.side_effect = [NS(stdout="inactive"), NS(returncode=0)]
            startup.start_component("moveit", Path("/project"), no_record=True)
            command = run.call_args.args[0]
            self.assertIn("--unit=ur3-motion-moveit", command)
            self.assertIn("--property=Restart=no", command)
            self.assertIn("--property=StandardOutput=null", command)
            self.assertIn("--property=StandardError=null", command)
            self.assertEqual(command[-4:], ["/project", "moveit", "/project", "--no-record"])

        # Real launch and real rclpy logging in an isolated ROS domain, using only a
        # short-lived test node. Also exercise the installed spawner's ROS-arg parser.
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "robot").mkdir()
            (root / "robot/motion_stack.launch.py").write_text('''
import sys
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
def check(context):
    assert context.launch_configurations['start_driver'] == 'true'
    assert context.launch_configurations['start_moveit'] == 'false'
    return [Node(executable=sys.executable, arguments=['-c',
        "import rclpy; rclpy.init(); n=rclpy.create_node('startup_logging_check'); "
        "n.get_logger().info('STARTUP_LOG_CHECK'); n.destroy_node(); rclpy.shutdown()"], output='both'),
        Node(package='controller_manager', executable='spawner', arguments=['--help'])]
def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=check)])
''')
            env = dict(os.environ, ROS_LOG_DIR=str(root / "logs"), ROS_DOMAIN_ID="232",
                       ROS_LOCALHOST_ONLY="1", PYTHONDONTWRITEBYTECODE="1")
            command = [sys.executable, "-m", "ur3_magnetic_control.motion_startup", "driver", str(root)]
            for no_record in (True, False):
                result = subprocess.run(command + (["--no-record"] if no_record else []),
                                        env=env, capture_output=True, text=True, timeout=15)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                self.assertIn("STARTUP_LOG_CHECK", result.stdout + result.stderr)
                self.assertNotIn("process has died", result.stdout + result.stderr)
                self.assertEqual(bool(list(root.rglob("*.log"))), not no_record)
                self.assertEqual((root / "logs").exists(), not no_record)


if __name__ == "__main__":
    unittest.main()
