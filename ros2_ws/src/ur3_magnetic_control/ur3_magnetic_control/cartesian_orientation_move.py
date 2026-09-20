import argparse
import copy
import math
import sys

import rclpy
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetCartesianPath, GetPositionFK

from .cartesian_line_move import (
    EXECUTION_TOKEN,
    CartesianLineMove,
    duration_seconds,
    set_duration,
)


def quaternion_multiply(left, right):
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def base_frame_delta_quaternion(roll, pitch, yaw):
    qx = (math.sin(roll / 2.0), 0.0, 0.0, math.cos(roll / 2.0))
    qy = (0.0, math.sin(pitch / 2.0), 0.0, math.cos(pitch / 2.0))
    qz = (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))
    return quaternion_multiply(qz, quaternion_multiply(qy, qx))


def axis_angle_quaternion(axis, angle):
    norm = math.sqrt(sum(value * value for value in axis))
    if norm <= 0.0:
        raise ValueError("rotation axis must be nonzero")
    scale = math.sin(angle / 2.0) / norm
    return (
        axis[0] * scale,
        axis[1] * scale,
        axis[2] * scale,
        math.cos(angle / 2.0),
    )


class CartesianOrientationMove(CartesianLineMove):
    def plan_orientation(self, roll, pitch, yaw, angular_speed):
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
        current = (
            start_pose.orientation.x,
            start_pose.orientation.y,
            start_pose.orientation.z,
            start_pose.orientation.w,
        )
        if abs(roll) > 1.0e-12 or abs(pitch) > 1.0e-12:
            raise RuntimeError(
                "roll/pitch are disabled: the motor axis must remain parallel to the table"
            )
        delta = axis_angle_quaternion(
            self.ceiling_guard.world_axes_in_base["z"], yaw
        )
        target = quaternion_multiply(delta, current)
        norm = math.sqrt(sum(value * value for value in target))
        (
            target_pose.orientation.x,
            target_pose.orientation.y,
            target_pose.orientation.z,
            target_pose.orientation.w,
        ) = tuple(value / norm for value in target)

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
        request.max_velocity_scaling_factor = 0.03
        request.max_acceleration_scaling_factor = 0.03

        response = self.call(self.cartesian_client, request, timeout=20.0)
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(
                f"Cartesian orientation planning failed, code={response.error_code.val}"
            )
        if response.fraction < 0.999:
            raise RuntimeError(
                f"Cartesian orientation path incomplete: {response.fraction * 100.0:.1f}%"
            )
        trajectory = response.solution.joint_trajectory
        if len(trajectory.points) < 2:
            raise RuntimeError("orientation planner returned too few trajectory points")
        self.unwrap_and_check_joint_limits(trajectory, start_state)
        self.validate_trajectory_states(trajectory, start_state)

        original_duration = duration_seconds(trajectory.points[-1].time_from_start)
        angle = math.sqrt(roll * roll + pitch * pitch + yaw * yaw)
        requested_duration = angle / angular_speed
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
        return response.solution, response.fraction, duration_seconds(
            trajectory.points[-1].time_from_start
        )


def parse_arguments(arguments):
    parser = argparse.ArgumentParser(
        description="Plan a guarded small base-frame orientation change for the real UR3."
    )
    parser.add_argument("--roll-deg", type=float, default=0.0)
    parser.add_argument("--pitch-deg", type=float, default=0.0)
    parser.add_argument("--yaw-deg", type=float, default=0.0)
    parser.add_argument("--angular-speed-deg-s", type=float, default=5.0)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--confirmation-token", default="")
    return parser.parse_args(arguments)


def main(args=None):
    parsed = parse_arguments(sys.argv[1:] if args is None else args)
    values_deg = (parsed.roll_deg, parsed.pitch_deg, parsed.yaw_deg)
    total_deg = math.sqrt(sum(value * value for value in values_deg))
    if abs(parsed.roll_deg) > 1.0e-9 or abs(parsed.pitch_deg) > 1.0e-9:
        raise SystemExit(
            "roll/pitch motion is disabled; only world-Z yaw preserves the horizontal motor axis"
        )
    if total_deg <= 0.0 or total_deg > 25.0:
        raise SystemExit("world-Z yaw change must be nonzero and <=25 deg")
    if not 0.0 < parsed.angular_speed_deg_s <= 10.0:
        raise SystemExit("angular speed must be in (0, 10] deg/s")
    if parsed.execute and parsed.confirmation_token != EXECUTION_TOKEN:
        raise SystemExit("execution refused: confirmation token is missing or invalid")

    rclpy.init()
    node = CartesianOrientationMove()
    try:
        node.wait_for_state(require_execution_state=parsed.execute)
        if parsed.execute and not node.robot_program_running:
            raise RuntimeError("External Control program is not running")
        trajectory, fraction, duration = node.plan_orientation(
            *(math.radians(value) for value in values_deg),
            math.radians(parsed.angular_speed_deg_s),
        )
        mismatch = node.verify_start(trajectory)
        print(
            f"PLAN_OK base_rpy_deg=({parsed.roll_deg:.3f}, "
            f"{parsed.pitch_deg:.3f}, {parsed.yaw_deg:.3f}) fraction={fraction:.6f}"
        )
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
        print(
            "FINAL_MAX_ABS_JOINT_VELOCITY="
            f"{max(abs(value) for value in node.latest_joint_state.velocity):.6f}rad/s"
        )
    except Exception as error:
        node.get_logger().error(str(error))
        raise SystemExit(1) from error
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
