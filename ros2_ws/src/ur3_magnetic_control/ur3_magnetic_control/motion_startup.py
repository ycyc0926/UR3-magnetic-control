"""Prepare the existing motion stack; only explicit execution may press Play."""

from pathlib import Path
import subprocess
import time

import rclpy
from controller_manager_msgs.srv import ListControllers
from std_srvs.srv import Trigger
from ur_dashboard_msgs.msg import RobotMode, SafetyMode
from ur_dashboard_msgs.srv import GetLoadedProgram, GetProgramState, GetRobotMode, GetSafetyMode


EXTERNAL_PROGRAM = "/programs/external_control.urp"
STATE_CONTROLLERS = {"joint_state_broadcaster", "io_and_status_controller",
                     "speed_scaling_state_broadcaster"}


def start_component(component, root, no_record=False):
    """A transient user service survives this command and cannot start twice."""
    unit = f"ur3-motion-{component}"
    active = subprocess.run(["systemctl", "--user", "is-active", unit],
                            capture_output=True, text=True, timeout=5)
    if active.stdout.strip() in ("active", "activating"):
        return
    command = ["systemd-run", "--user", f"--unit={unit}", "--collect",
               "--service-type=exec", "--property=Restart=no"]
    if no_record:
        command += ["--property=StandardOutput=null", "--property=StandardError=null"]
    command += ["/bin/bash", "-c",
                'set -e; source "$1/ros2_env.sh"; shift; '
                'exec /usr/bin/python3 -m ur3_magnetic_control.motion_startup "$@"',
                "ur3-motion", str(root), component, str(root)]
    if no_record:
        command.append("--no-record")
    result = subprocess.run(command, capture_output=True, text=True, timeout=10)
    if result.returncode:
        raise RuntimeError(f"Cannot start {unit}: {result.stderr.strip()}")


def prepare_motion_stack(node, root, no_record=False, timeout=45.0):
    controllers = node.create_client(ListControllers, "/controller_manager/list_controllers")
    try:
        # Give DDS discovery time to find manually launched services before starting any.
        driver_ready = controllers.wait_for_service(timeout_sec=1.0)
        moveit_ready = node.cartesian_client.wait_for_service(timeout_sec=1.0)
        names = node.get_node_names()
        for component, ready, marker in (("driver", driver_ready, "controller_manager"),
                                          ("moveit", moveit_ready, "move_group")):
            if names.count(marker) > 1:
                raise RuntimeError(f"Multiple {marker} nodes found; close duplicate motion stacks")
            if not ready and marker not in names:
                print(f"STARTUP: starting {component}", flush=True)
                start_component(component, root, no_record)
        deadline = time.monotonic() + timeout
        required = [controllers, node.cartesian_client, node.fk_client,
                    node.ceiling_guard.apply_client, node.ceiling_guard.validity_client,
                    node.ceiling_guard.description_client, node.dashboard_stop_client]
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if all(client.service_is_ready() for client in required) and node.execute_client.server_is_ready():
                response = node.call(controllers, ListControllers.Request(), timeout=2.0)
                active = {item.name for item in response.controller if item.state == "active"}
                if STATE_CONTROLLERS <= active:
                    node.wait_for_fresh_state(timeout=5.0)
                    print("STARTUP: driver and calibrated MoveIt ready", flush=True)
                    return
        raise RuntimeError("Motion startup timed out; inspect ur3-motion-driver / ur3-motion-moveit services")
    finally:
        node.destroy_client(controllers)


def ensure_external_control(node, timeout=10.0):
    """Play only the known, stopped program; never recover stops or release brakes."""
    clients = []
    play_attempted = False

    def call(service, name):
        client = node.create_client(service, "/dashboard_client/" + name)
        clients.append(client)
        reply = node.call(client, service.Request(), timeout=3.0)
        if not reply.success:
            raise RuntimeError(f"Dashboard {name} failed: {reply.answer if hasattr(reply, 'answer') else reply.message}")
        return reply

    try:
        program = call(GetLoadedProgram, "get_loaded_program").program_name
        if program != EXTERNAL_PROGRAM:
            raise RuntimeError(f"Load {EXTERNAL_PROGRAM} on the pendant first; loaded program is {program!r}")
        if call(GetSafetyMode, "get_safety_mode").safety_mode.mode not in (SafetyMode.NORMAL, SafetyMode.REDUCED):
            raise RuntimeError("Robot safety stop is active; resolve it on the pendant")
        if call(GetRobotMode, "get_robot_mode").robot_mode.mode != RobotMode.RUNNING:
            raise RuntimeError("Robot is not powered and brake-released; prepare it on the pendant")
        state = call(GetProgramState, "program_state").state.state
        if state not in ("STOPPED", "PLAYING"):
            raise RuntimeError(f"External Control is {state}; automatic resume refused")
        if state == "STOPPED":
            play_attempted = True  # A timed-out Play request may still have reached the robot.
            call(Trigger, "play")
            print("STARTUP: External Control Play accepted; waiting for connection", flush=True)
        controllers = node.create_client(ListControllers, "/controller_manager/list_controllers")
        clients.append(controllers)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.robot_program_running is True:
                response = node.call(controllers, ListControllers.Request(), timeout=2.0)
                if any(item.name == "scaled_joint_trajectory_controller" and item.state == "active"
                       for item in response.controller):
                    node.wait_for_fresh_state(timeout=2.0, require_execution_state=True)
                    if node.robot_program_running is True and node.speed_scaling_percent > 0:
                        print("STARTUP: External Control connected", flush=True)
                        return
        raise RuntimeError("External Control connection/controller/speed scaling timed out; no automatic Play retry")
    except BaseException as error:
        if play_attempted:
            try:
                node.stop_robot_program()
            except Exception as stop_error:
                raise RuntimeError(f"{error}; stopping External Control also failed: {stop_error}") from error
        raise
    finally:
        for client in clients:
            node.destroy_client(client)


def launch_component(component, root, no_record=False):
    """Service entry point: reuse the repository launch file and its calibration."""
    import logging
    import os
    import shlex
    import sys
    import launch.logging
    from launch import LaunchDescription, LaunchService
    from launch.actions import IncludeLaunchDescription, SetLaunchConfiguration
    from launch.launch_description_sources import PythonLaunchDescriptionSource

    actions = []
    if no_record:
        # Suppress launch and child-node file logs, as well as service stdout/stderr.
        launch.logging.launch_config.log_dir = str(root)
        launch.logging.launch_config.log_handler_factory = lambda *a, **k: logging.NullHandler()
        os.environ["OVERRIDE_LAUNCH_PROCESS_OUTPUT"] = "screen"
        prefix = [sys.executable, "-c",
                  "import os,sys; os.execvp(sys.argv[1], sys.argv[1:]+"
                  "['--ros-args','--disable-external-lib-logs'])"]
        actions.append(SetLaunchConfiguration("launch-prefix", shlex.join(prefix)))
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(Path(root) / "robot/motion_stack.launch.py")),
        launch_arguments={"start_driver": str(component == "driver").lower(),
                          "start_moveit": str(component == "moveit").lower()}.items()))
    service = LaunchService(noninteractive=True)
    service.include_launch_description(LaunchDescription(actions))
    return service.run()


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("component", choices=("driver", "moveit"))
    parser.add_argument("root", type=Path)
    parser.add_argument("--no-record", action="store_true")
    args = parser.parse_args()
    raise SystemExit(launch_component(args.component, args.root, args.no_record))
