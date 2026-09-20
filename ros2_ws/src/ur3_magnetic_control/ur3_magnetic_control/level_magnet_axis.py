"""Level the motor shaft by rotating the end pose at a fixed flange position."""

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
from .cartesian_orientation_move import axis_angle_quaternion, quaternion_multiply


def quaternion_to_matrix(quaternion):
    x, y, z, w = quaternion
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]


def matrix_vector(matrix, vector):
    return [sum(row[index] * vector[index] for index in range(3)) for row in matrix]


def vector_dot(left, right):
    return sum(a * b for a, b in zip(left, right))


def vector_cross(left, right):
    return [
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    ]


def vector_norm(vector):
    return math.sqrt(vector_dot(vector, vector))


def normalized(vector):
    norm = vector_norm(vector)
    if norm <= 0.0:
        raise RuntimeError("cannot normalize a zero vector")
    return [value / norm for value in vector]


class LevelMagnetAxis(CartesianLineMove):
    def plan_level(self, angular_speed, start_state=None):
        if start_state is None:
            start_state = self.current_robot_state()
        self.ceiling_guard.apply(start_state)
        self.ceiling_guard.validate_state(start_state, "current robot state")

        fk_request = GetPositionFK.Request()
        fk_request.header.frame_id = "base"
        fk_request.fk_link_names = ["tool0"]
        fk_request.robot_state = self.ceiling_guard.decorate_state(start_state)
        fk_response = self.call(self.fk_client, fk_request)
        if fk_response.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f"FK failed, code={fk_response.error_code.val}")

        start_pose = fk_response.pose_stamped[0].pose
        current_quaternion = (
            start_pose.orientation.x,
            start_pose.orientation.y,
            start_pose.orientation.z,
            start_pose.orientation.w,
        )
        current_rotation = quaternion_to_matrix(current_quaternion)
        configuration = self.ceiling_guard.configuration
        shaft_tool = configuration["motor_axis_tool_vector"]
        shaft_base = normalized(matrix_vector(current_rotation, shaft_tool))
        table_normal_base = normalized(
            self.ceiling_guard.world_axes_in_base["z"]
        )
        vertical_component = vector_dot(shaft_base, table_normal_base)
        vertical_component = max(-1.0, min(1.0, vertical_component))
        elevation = math.asin(vertical_component)
        if abs(math.degrees(elevation)) > 5.0:
            raise RuntimeError(
                f"motor shaft elevation {math.degrees(elevation):.3f} deg exceeds "
                "the guarded 5 deg automatic-leveling limit"
            )

        target_shaft = normalized(
            [
                shaft_base[index] - vertical_component * table_normal_base[index]
                for index in range(3)
            ]
        )
        rotation_axis = vector_cross(shaft_base, target_shaft)
        sine = vector_norm(rotation_axis)
        cosine = max(-1.0, min(1.0, vector_dot(shaft_base, target_shaft)))
        correction_angle = math.atan2(sine, cosine)
        if correction_angle < math.radians(0.05):
            raise RuntimeError(
                f"motor shaft is already level within 0.05 deg "
                f"(elevation={math.degrees(elevation):.4f} deg)"
            )
        rotation_axis = normalized(rotation_axis)
        delta = axis_angle_quaternion(rotation_axis, correction_angle)
        target_quaternion = quaternion_multiply(delta, current_quaternion)
        target_quaternion = normalized(target_quaternion)
        target_rotation = quaternion_to_matrix(target_quaternion)

        # Keep the flange/tool0 origin fixed and change only the end orientation.
        # The sphere centre therefore moves with the physical motor/shaft assembly.
        offset_tool = configuration["magnet_tcp_xyz_m"]
        current_offset_base = matrix_vector(current_rotation, offset_tool)
        target_offset_base = matrix_vector(target_rotation, offset_tool)
        start_sphere_center_base = [
            start_pose.position.x + current_offset_base[0],
            start_pose.position.y + current_offset_base[1],
            start_pose.position.z + current_offset_base[2],
        ]
        target_pose = copy.deepcopy(start_pose)
        target_sphere_center_base = [
            start_pose.position.x + target_offset_base[0],
            start_pose.position.y + target_offset_base[1],
            start_pose.position.z + target_offset_base[2],
        ]
        (
            target_pose.orientation.x,
            target_pose.orientation.y,
            target_pose.orientation.z,
            target_pose.orientation.w,
        ) = target_quaternion

        request = GetCartesianPath.Request()
        request.header.frame_id = "base"
        request.start_state = self.ceiling_guard.decorate_state(start_state)
        request.group_name = "ur_manipulator"
        request.link_name = "tool0"
        request.waypoints = [target_pose]
        request.max_step = 0.001
        request.jump_threshold = 2.0
        request.prismatic_jump_threshold = 0.0
        request.revolute_jump_threshold = 0.05
        request.avoid_collisions = True
        request.max_velocity_scaling_factor = 0.02
        request.max_acceleration_scaling_factor = 0.02

        response = self.call(self.cartesian_client, request, timeout=20.0)
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(
                f"motor leveling planning failed, code={response.error_code.val}"
            )
        if response.fraction < 0.999:
            raise RuntimeError(
                f"motor leveling path incomplete: {response.fraction * 100.0:.1f}%"
            )
        trajectory = response.solution.joint_trajectory
        if len(trajectory.points) < 2:
            raise RuntimeError("motor leveling planner returned too few points")
        self.unwrap_and_check_joint_limits(trajectory, start_state)
        self.validate_trajectory_states(trajectory, start_state)

        original_duration = duration_seconds(trajectory.points[-1].time_from_start)
        if original_duration <= 0.0:
            raise RuntimeError('Invalid trajectory duration')
        requested_duration = correction_angle / angular_speed
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
        return (
            response.solution,
            math.degrees(elevation),
            math.degrees(correction_angle),
            start_sphere_center_base,
            target_sphere_center_base,
            duration_seconds(trajectory.points[-1].time_from_start),
        )


def parse_arguments(arguments):
    parser = argparse.ArgumentParser(
        description="Level the guarded UR3 magnet shaft to the table plane."
    )
    parser.add_argument("--angular-speed-deg-s", type=float, default=1.0)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--confirmation-token", default="")
    return parser.parse_args(arguments)


def main(args=None):
    parsed = parse_arguments(sys.argv[1:] if args is None else args)
    if not 0.1 <= parsed.angular_speed_deg_s <= 2.0:
        raise SystemExit("angular speed must be in [0.1, 2.0] deg/s")
    if parsed.execute and parsed.confirmation_token != EXECUTION_TOKEN:
        raise SystemExit("execution refused: confirmation token is missing or invalid")

    rclpy.init()
    node = LevelMagnetAxis()
    try:
        if parsed.execute and not node.ceiling_guard.configuration['motor_axis_verified']:
            raise RuntimeError('Execution blocked: physical motor shaft direction has not been confirmed')
        node.wait_for_state(require_execution_state=parsed.execute)
        if parsed.execute and not node.robot_program_running:
            raise RuntimeError("External Control program is not running")
        (
            trajectory,
            elevation,
            correction,
            start_sphere_center,
            target_sphere_center,
            duration,
        ) = node.plan_level(math.radians(parsed.angular_speed_deg_s))
        mismatch = node.verify_start(trajectory)
        print(node.ceiling_guard.status_text())
        print(
            f"LEVEL_PLAN_OK shaft_start_elevation={elevation:.4f}deg "
            f"correction={correction:.4f}deg target_elevation=0.0000deg"
        )
        print(
            "SPHERE_CENTER_BASE_START_M="
            + ",".join(f"{value:.6f}" for value in start_sphere_center)
        )
        print(
            "SPHERE_CENTER_BASE_TARGET_M="
            + ",".join(f"{value:.6f}" for value in target_sphere_center)
        )
        print(
            f"POINTS={len(trajectory.joint_trajectory.points)} "
            f"DURATION={duration:.3f}s START_MISMATCH={mismatch:.6f}rad"
        )
        if not parsed.execute:
            print("PLAN_ONLY: no command sent to the robot")
            return
        print("EXECUTION_START")
        node.execute(trajectory)
        node.wait_for_state(timeout=2.0, require_execution_state=True)
        print("EXECUTION_SUCCESS")
    except Exception as error:
        node.get_logger().error(str(error))
        raise SystemExit(1) from error
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
