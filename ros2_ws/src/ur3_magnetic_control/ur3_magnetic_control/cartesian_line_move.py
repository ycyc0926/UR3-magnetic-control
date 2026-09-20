import argparse
import copy
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetCartesianPath, GetPositionFK
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64

from .acrylic_ceiling_guard import AcrylicCeilingGuard


EXECUTION_TOKEN = "I_ACCEPT_REAL_ROBOT_MOTION"


def duration_seconds(duration):
    return float(duration.sec) + float(duration.nanosec) * 1.0e-9


def set_duration(duration, seconds):
    seconds = max(0.0, float(seconds))
    whole = int(math.floor(seconds))
    nanos = int(round((seconds - whole) * 1.0e9))
    if nanos >= 1_000_000_000:
        whole += 1
        nanos -= 1_000_000_000
    duration.sec = whole
    duration.nanosec = nanos


class CartesianLineMove(Node):
    def __init__(self):
        super().__init__("cartesian_line_move")
        self.latest_joint_state = None
        self.joint_state_received_at = None
        self.execution_validator = None
        self.robot_program_running = None
        self.speed_scaling_percent = None
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            JointState, "/joint_states", self.on_joint_state, state_qos
        )
        self.create_subscription(
            Bool,
            "/io_and_status_controller/robot_program_running",
            self.on_program_running,
            state_qos,
        )
        self.create_subscription(
            Float64,
            "/speed_scaling_state_broadcaster/speed_scaling",
            self.on_speed_scaling,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            ),
        )
        self.fk_client = self.create_client(GetPositionFK, "/compute_fk")
        self.cartesian_client = self.create_client(
            GetCartesianPath, "/compute_cartesian_path"
        )
        self.execute_client = ActionClient(
            self, ExecuteTrajectory, "/execute_trajectory"
        )
        self.ceiling_guard = AcrylicCeilingGuard(self)

    def on_joint_state(self, message):
        self.latest_joint_state = message
        self.joint_state_received_at = time.monotonic()

    def on_program_running(self, message):
        self.robot_program_running = bool(message.data)

    def on_speed_scaling(self, message):
        self.speed_scaling_percent = float(message.data)

    def wait_for_state(self, timeout=5.0, require_execution_state=False):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.latest_joint_state is not None and (
                not require_execution_state
                or (
                    self.robot_program_running is not None
                    and self.speed_scaling_percent is not None
                )
            ):
                return
        raise RuntimeError(
            "timed out waiting for robot state "
            f"(require_execution_state={require_execution_state}, "
            f"joint_state={self.latest_joint_state is not None}, "
            f"program_running={self.robot_program_running}, "
            f"speed_scaling={self.speed_scaling_percent})"
        )

    def call(self, client, request, timeout=10.0):
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f"service unavailable: {client.srv_name}")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done() or future.result() is None:
            raise RuntimeError(f"service call failed: {client.srv_name}")
        return future.result()

    def current_robot_state(self):
        state = RobotState()
        state.joint_state = copy.deepcopy(self.latest_joint_state)
        state.joint_state.velocity = []
        state.joint_state.effort = []
        state.is_diff = False
        return state

    def unwrap_and_check_joint_limits(self, trajectory, start_state):
        limits = {
            "shoulder_pan_joint": (-2.0 * math.pi + 0.15, 2.0 * math.pi - 0.15),
            "shoulder_lift_joint": (-2.0 * math.pi + 0.15, 2.0 * math.pi - 0.15),
            "elbow_joint": (-math.pi + 0.15, math.pi - 0.15),
            "wrist_1_joint": (-2.0 * math.pi + 0.15, 2.0 * math.pi - 0.15),
            "wrist_2_joint": (-2.0 * math.pi + 0.15, 2.0 * math.pi - 0.15),
            "wrist_3_joint": (-2.0 * math.pi + 0.15, 2.0 * math.pi - 0.15),
        }
        current = dict(
            zip(start_state.joint_state.name, start_state.joint_state.position)
        )
        previous = [current[name] for name in trajectory.joint_names]
        for point_index, point in enumerate(trajectory.points):
            adjusted = []
            for joint_index, (name, raw_value) in enumerate(
                zip(trajectory.joint_names, point.positions)
            ):
                reference = previous[joint_index]
                value = raw_value + 2.0 * math.pi * round(
                    (reference - raw_value) / (2.0 * math.pi)
                )
                lower, upper = limits[name]
                if value < lower or value > upper:
                    raise RuntimeError(
                        f"unwrapped path exceeds guarded limit at point "
                        f"{point_index}: {name}={value:.5f} rad, "
                        f"allowed=[{lower:.5f}, {upper:.5f}]"
                    )
                adjusted.append(value)
            point.positions = adjusted
            previous = adjusted

    def plan(self, axis, distance_m, speed_m_s):
        start_state = self.current_robot_state()
        self.ceiling_guard.apply(start_state)
        self.ceiling_guard.validate_state(start_state, "current robot state")
        fk_request = GetPositionFK.Request()
        fk_request.header.frame_id = "base"
        fk_request.fk_link_names = ["tool0"]
        fk_request.robot_state = start_state
        fk_response = self.call(self.fk_client, fk_request)
        if fk_response.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f"FK failed, code={fk_response.error_code.val}")

        start_pose = fk_response.pose_stamped[0].pose
        target_pose = copy.deepcopy(start_pose)
        direction = 1.0 if axis[0] == "+" else -1.0
        world_axis_in_base = self.ceiling_guard.world_axes_in_base[axis[-1]]
        target_pose.position.x += direction * distance_m * world_axis_in_base[0]
        target_pose.position.y += direction * distance_m * world_axis_in_base[1]
        target_pose.position.z += direction * distance_m * world_axis_in_base[2]

        request = GetCartesianPath.Request()
        request.header.frame_id = "base"
        request.start_state = self.ceiling_guard.decorate_state(start_state)
        request.group_name = "ur_manipulator"
        request.link_name = "tool0"
        request.waypoints = [target_pose]
        request.max_step = 0.002
        request.jump_threshold = 2.0
        request.prismatic_jump_threshold = 0.0
        request.revolute_jump_threshold = 0.10
        request.avoid_collisions = True
        request.max_velocity_scaling_factor = 0.05
        request.max_acceleration_scaling_factor = 0.05

        response = self.call(self.cartesian_client, request, timeout=20.0)
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(
                f"Cartesian planning failed, code={response.error_code.val}"
            )
        if response.fraction < 0.999:
            raise RuntimeError(
                f"Cartesian path incomplete: {response.fraction * 100.0:.1f}%"
            )

        trajectory = response.solution.joint_trajectory
        if len(trajectory.points) < 2:
            raise RuntimeError("Cartesian planner returned too few trajectory points")
        self.unwrap_and_check_joint_limits(trajectory, start_state)
        self.validate_trajectory_states(trajectory, start_state)
        original_duration = duration_seconds(trajectory.points[-1].time_from_start)
        if original_duration <= 0.0:
            raise RuntimeError("Cartesian trajectory has no time parameterization")

        requested_duration = distance_m / speed_m_s
        time_scale = max(1.0, requested_duration / original_duration)
        for point in trajectory.points:
            set_duration(
                point.time_from_start,
                duration_seconds(point.time_from_start) * time_scale,
            )
            if point.velocities:
                point.velocities = [value / time_scale for value in point.velocities]
            if point.accelerations:
                point.accelerations = [
                    value / (time_scale * time_scale) for value in point.accelerations
                ]

        final_duration = duration_seconds(trajectory.points[-1].time_from_start)
        return response.solution, start_pose, target_pose, response.fraction, final_duration

    def validate_trajectory_states(self, trajectory, start_state):
        names = list(trajectory.joint_names)
        name_to_index = {
            name: index for index, name in enumerate(start_state.joint_state.name)
        }
        for name in names:
            if name not in name_to_index:
                raise RuntimeError(f"trajectory contains unknown joint: {name}")
        for point_index, point in enumerate(trajectory.points):
            state = copy.deepcopy(start_state)
            positions = list(state.joint_state.position)
            for name, value in zip(names, point.positions):
                positions[name_to_index[name]] = value
            state.joint_state.position = positions
            state.joint_state.velocity = []
            state.joint_state.effort = []
            self.ceiling_guard.validate_state(
                state, f"trajectory point {point_index}/{len(trajectory.points) - 1}"
            )

    def verify_start(self, trajectory, tolerance=0.01):
        if self.joint_state_received_at is None or time.monotonic() - self.joint_state_received_at > 0.2:
            raise RuntimeError('joint state is stale')
        current = dict(zip(self.latest_joint_state.name, self.latest_joint_state.position))
        first = trajectory.joint_trajectory.points[0].positions
        planned = dict(zip(trajectory.joint_trajectory.joint_names, first))
        differences = [abs(current[name] - value) for name, value in planned.items()]
        maximum = max(differences)
        if maximum > tolerance:
            raise RuntimeError(
                f"robot moved after planning; start mismatch={maximum:.6f} rad"
            )
        return maximum

    def execute(self, trajectory):
        self.verify_start(trajectory, tolerance=0.002)
        if not self.robot_program_running:
            raise RuntimeError("External Control program is not running")
        if self.speed_scaling_percent is None or self.speed_scaling_percent <= 0:
            raise RuntimeError('Robot speed scaling is zero or unavailable')
        if not self.execute_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("MoveIt execute trajectory action is unavailable")
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        goal_future = self.execute_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, goal_future, timeout_sec=5.0)
        goal_handle = goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("trajectory execution goal was rejected")
        result_future = goal_handle.get_result_async()
        planned_duration = duration_seconds(
            trajectory.joint_trajectory.points[-1].time_from_start
        )
        scale = max(0.01, min(1.0, self.speed_scaling_percent / 100.0))
        execution_timeout = max(60.0, planned_duration / scale * 1.5 + 10.0)
        print(
            f"SPEED_SCALING={self.speed_scaling_percent:.1f}% "
            f"EXECUTION_TIMEOUT={execution_timeout:.1f}s"
        )
        deadline = time.monotonic() + execution_timeout
        try:
            while not result_future.done() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.01)
                if time.monotonic() - self.joint_state_received_at > 0.2:
                    raise RuntimeError('Lost fresh joint feedback during execution')
                if self.execution_validator is not None:
                    self.execution_validator(self.current_robot_state())
        except BaseException:
            cancel = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel, timeout_sec=2.0)
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=5.0)
            raise
        if not result_future.done() or result_future.result() is None:
            goal_handle.cancel_goal_async()
            raise RuntimeError("trajectory execution timed out")
        result = result_future.result().result
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(
                f"trajectory execution failed, code={result.error_code.val}"
            )


def parse_arguments(arguments):
    parser = argparse.ArgumentParser(
        description="Plan a guarded Cartesian line for the real UR3."
    )
    parser.add_argument("--axis", choices=["+x", "-x", "+y", "-y", "+z", "-z"], required=True)
    parser.add_argument("--distance-mm", type=float, required=True)
    parser.add_argument("--speed-mm-s", type=float, required=True)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--confirmation-token", default="")
    return parser.parse_args(arguments)


def format_position(pose: Pose):
    return (
        f"({pose.position.x * 1000.0:.2f}, "
        f"{pose.position.y * 1000.0:.2f}, "
        f"{pose.position.z * 1000.0:.2f}) mm"
    )


def main(args=None):
    parsed = parse_arguments(sys.argv[1:] if args is None else args)
    if parsed.distance_mm <= 0.0 or parsed.distance_mm > 300.0:
        raise SystemExit("distance must be in (0, 300] mm")
    if parsed.speed_mm_s <= 0.0 or parsed.speed_mm_s > 25.0:
        raise SystemExit("speed must be in (0, 25] mm/s")
    if parsed.execute and parsed.confirmation_token != EXECUTION_TOKEN:
        raise SystemExit("execution refused: confirmation token is missing or invalid")

    rclpy.init()
    node = CartesianLineMove()
    try:
        node.wait_for_state(require_execution_state=parsed.execute)
        if parsed.execute and not node.robot_program_running:
            raise RuntimeError("External Control program is not running")
        trajectory, start, target, fraction, duration = node.plan(
            parsed.axis,
            parsed.distance_mm / 1000.0,
            parsed.speed_mm_s / 1000.0,
        )
        mismatch = node.verify_start(trajectory)
        print(f"PLAN_OK axis={parsed.axis} fraction={fraction:.6f}")
        print(f"START_TOOL0={format_position(start)}")
        print(f"TARGET_TOOL0={format_position(target)}")
        print(
            f"POINTS={len(trajectory.joint_trajectory.points)} "
            f"DURATION={duration:.3f}s START_MISMATCH={mismatch:.6f}rad"
        )
        final_positions = trajectory.joint_trajectory.points[-1].positions
        print(
            "FINAL_JOINTS_RAD="
            + ", ".join(
                f"{name}:{value:.5f}"
                for name, value in zip(
                    trajectory.joint_trajectory.joint_names, final_positions
                )
            )
        )
        if not parsed.execute:
            print("PLAN_ONLY: no command sent to the robot")
            return
        print("EXECUTION_START")
        node.execute(trajectory)
        node.wait_for_state(timeout=2.0, require_execution_state=True)
        print("EXECUTION_SUCCESS")
        print(f"FINAL_MAX_ABS_JOINT_VELOCITY={max(abs(v) for v in node.latest_joint_state.velocity):.6f}rad/s")
    except Exception as error:
        node.get_logger().error(str(error))
        raise SystemExit(1) from error
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
