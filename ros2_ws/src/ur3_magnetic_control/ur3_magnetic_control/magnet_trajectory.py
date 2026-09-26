#!/usr/bin/env python3
"""Reusable guarded Cartesian paths expressed at the magnet centre.

The command accepts absolute magnet-centre coordinates in ``table_world`` and
keeps the current tool orientation fixed.  It can generate a point move, a
closed square, a closed circle, or a path loaded from YAML/JSON/CSV.  Planning
is the default; --execute sends motion to the robot.

An optional final motor-axis constraint can rotate the tool about the magnet
centre and align the verified shaft direction with a selected ``table_world``
axis.  The default alignment order translates first and rotates at the final
magnet centre; this keeps the requested point fixed during the orientation
change.

This is an application-level guard, not a certified safety function.  The
current whole-tool envelope is deliberately conservative but still marked as
provisional because cables and some bracket details have not been measured.
"""

from __future__ import annotations

import argparse
from collections import deque
from contextlib import ExitStack
import copy
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import (
    AttachedCollisionObject,
    CollisionObject,
    MoveItErrorCodes,
    PlanningScene,
    RobotTrajectory,
)
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath, GetPositionFK
from rcl_interfaces.srv import GetParameters
from scipy.spatial.transform import Rotation
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import yaml

from .acrylic_ceiling_guard import AcrylicCeilingGuard, DEFAULT_PROJECT_ROOT
from .acrylic_geometry import modeled_boundary_gaps_m
from .cartesian_line_move import (
    CartesianLineMove,
    duration_seconds,
    set_duration,
)
from .ceiling_geometry import CeilingGeometry, sample_trajectory
from .clearance_policy import load_clearance_limits_m
from .experiment_data import MotorRecorder, active_experiment, add_magnet_pose
from .motion_startup import ensure_external_control, prepare_motion_stack
from .ze300_motor import DEFAULT_PORT as DEFAULT_MOTOR_PORT, ZE300Motor, position_counts, restore_origin


JOINT_NAMES = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)
TOOL_OBJECT_ID = "provisional_complete_magnet_tool"
TABLE_OBJECT_ID = "table_clearance_forbidden"
PATH_SCHEMA = "ur3_magnet_centre_path/v1"
MAX_CARTESIAN_STEP_M = 0.002
MAX_SEGMENT_M = 0.400
MAX_TOTAL_PATH_M = 1.000
MAX_TARGETS = 600
MAX_DURATION_S = 300.0
# Independently planned Cartesian chunks come to rest at every chunk boundary.
MAX_SEGMENTED_DURATION_S = 420.0
PLANNED_JOINT_SPEED_LIMIT_RAD_S = math.radians(5.0)
WRIST3_ZERO_PLAN_TOLERANCE_RAD = math.radians(0.5)
WRIST3_ZERO_LIVE_TOLERANCE_RAD = math.radians(1.0)
PLANNED_JOINT_ACCELERATION_LIMIT_RAD_S2 = 5.0
PLANNED_TOOL_ANGULAR_SPEED_LIMIT_RAD_S = math.radians(5.0)
PLANNED_PATH_TOLERANCE_M = 0.00075
FINAL_POSITION_TOLERANCE_M = 0.00075
FINAL_AXIS_TOLERANCE_RAD = math.radians(0.25)
PLANNED_ORIENTATION_PATH_TOLERANCE_RAD = math.radians(0.50)
WRIST3_FIXED_TOOL_ORIENTATION_TOLERANCE_RAD = math.radians(10.0)
PLANNED_TABLE_TILT_TOLERANCE_RAD = math.radians(0.5)
LIVE_TABLE_TILT_TOLERANCE_RAD = math.radians(2.0)
ORIENTATION_WAYPOINT_STEP_RAD = math.radians(1.0)
POSE_GUARD_POSITION_STEP_M = 0.001
POSE_GUARD_ORIENTATION_STEP_RAD = math.radians(0.25)
TRACE_INTERVAL_S = 0.02
COLLISION_SUBDIVISIONS_PER_SEGMENT = 4
MIN_PLANNING_CHUNK_M = 0.002
MAX_PLANNING_CHUNK_M = 0.050


def project_root() -> Path:
    return Path(os.environ.get("UR3_PROJECT_ROOT", DEFAULT_PROJECT_ROOT))


def finite_point(values, label="point") -> np.ndarray:
    point = np.asarray(values, dtype=float)
    if point.shape != (3,) or not np.isfinite(point).all():
        raise ValueError(f"{label} must contain exactly three finite values")
    return point


def homogeneous_point(transform, point) -> np.ndarray:
    transform = np.asarray(transform, dtype=float)
    point = finite_point(point)
    if transform.shape != (4, 4) or not np.isfinite(transform).all():
        raise ValueError("transform must be a finite 4x4 matrix")
    return (transform @ np.r_[point, 1.0])[:3]


def quaternion_matrix(pose: Pose) -> np.ndarray:
    quaternion = [
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    ]
    if not np.isfinite(quaternion).all() or np.linalg.norm(quaternion) < 1.0e-9:
        raise ValueError("invalid tool orientation")
    return Rotation.from_quat(quaternion).as_matrix()


def normalized_vector(value, label="vector") -> np.ndarray:
    vector = finite_point(value, label)
    norm = float(np.linalg.norm(vector))
    if norm < 1.0e-12:
        raise ValueError(f"{label} must be nonzero")
    return vector / norm


def rotation_between_vectors(source, target) -> Rotation:
    """Return the deterministic minimum-angle rotation from source to target."""
    source = normalized_vector(source, "source vector")
    target = normalized_vector(target, "target vector")
    cosine = float(np.clip(np.dot(source, target), -1.0, 1.0))
    cross = np.cross(source, target)
    sine = float(np.linalg.norm(cross))
    if sine > 1.0e-12:
        return Rotation.from_rotvec(cross / sine * math.atan2(sine, cosine))
    if cosine > 0.0:
        return Rotation.identity()
    # Antiparallel vectors admit infinitely many pi rotations.  Pick the
    # least-aligned Cartesian basis deterministically and make a perpendicular
    # axis from it, avoiding numerical instability near 180 degrees.
    basis = np.eye(3)[int(np.argmin(np.abs(source)))]
    axis = normalized_vector(np.cross(source, basis), "antiparallel axis")
    return Rotation.from_rotvec(axis * math.pi)


def parallel_axis_target_rotation(
    start_rotation_world,
    motor_axis_tool,
    world_axis,
    direction="nearest",
    roll_deg=0.0,
):
    """Choose and construct a minimum-turn tool rotation for an axis constraint."""
    start = Rotation.from_matrix(np.asarray(start_rotation_world, dtype=float))
    motor_axis_tool = normalized_vector(motor_axis_tool, "motor axis in tool")
    if world_axis not in ("x", "y", "z"):
        raise ValueError("world axis must be x, y, or z")
    if direction not in ("nearest", "positive", "negative"):
        raise ValueError("axis direction must be nearest, positive, or negative")
    basis = np.eye(3)[("x", "y", "z").index(world_axis)]
    current_axis = normalized_vector(
        start.as_matrix() @ motor_axis_tool,
        "current motor axis",
    )
    if direction == "positive":
        target_axis = basis
        selected_direction = "positive"
    elif direction == "negative":
        target_axis = -basis
        selected_direction = "negative"
    elif float(np.dot(current_axis, basis)) >= 0.0:
        target_axis = basis
        selected_direction = "positive"
    else:
        target_axis = -basis
        selected_direction = "negative"
    delta = rotation_between_vectors(current_axis, target_axis)
    target = delta * start
    roll_deg = float(roll_deg)
    if not math.isfinite(roll_deg) or not -180.0 <= roll_deg <= 180.0:
        raise ValueError("motor-axis roll must be within [-180, 180] degrees")
    if abs(roll_deg) > 1.0e-12:
        target = (
            Rotation.from_rotvec(target_axis * math.radians(roll_deg))
            * target
        )
    angle = float((start.inv() * target).magnitude())
    final_axis = target.as_matrix() @ motor_axis_tool
    if np.linalg.norm(final_axis - target_axis) > 1.0e-9:
        raise RuntimeError("failed to construct the requested motor-axis rotation")
    return {
        "rotation_world_tool": target.as_matrix(),
        "current_axis_world": current_axis,
        "target_axis_world": target_axis.copy(),
        "selected_direction": selected_direction,
        "roll_deg": roll_deg,
        "rotation_angle_rad": angle,
    }


def table_parallel_target_rotation(start_rotation_world, motor_axis_tool, yaw_deg=0.0):
    """Keep tool0's XY plane level; its tool-frame motor axis then stays level."""
    start = Rotation.from_matrix(np.asarray(start_rotation_world, dtype=float))
    motor_axis_tool = normalized_vector(motor_axis_tool, "motor axis in tool")
    if abs(motor_axis_tool[2]) > 1.0e-6:
        raise ValueError("motor axis must lie in the tool0 XY plane")
    normal = start.as_matrix()[:, 2]
    target_normal = np.asarray([0.0, 0.0, 1.0 if normal[2] >= 0.0 else -1.0])
    target = rotation_between_vectors(normal, target_normal) * start
    yaw_deg = float(yaw_deg)
    if not math.isfinite(yaw_deg) or not -180.0 <= yaw_deg <= 180.0:
        raise ValueError("table yaw must be within [-180, 180] degrees")
    if yaw_deg:
        target = Rotation.from_rotvec([0.0, 0.0, math.radians(yaw_deg)]) * target
    return {
        "rotation_world_tool": target.as_matrix(),
        "current_axis_world": start.as_matrix() @ motor_axis_tool,
        "target_axis_world": target.as_matrix() @ motor_axis_tool,
        "selected_direction": "positive" if target_normal[2] > 0 else "negative",
        "roll_deg": 0.0,
        "table_yaw_deg": yaw_deg,
        "rotation_angle_rad": float((start.inv() * target).magnitude()),
    }


def table_tilt_rad(checked):
    return math.acos(float(np.clip(abs(checked["tool_normal_world"][2]), 0.0, 1.0)))


def horizontal_tool_z_target_rotation(start_rotation_world, heading_deg, roll_deg):
    """Point tool0 +Z along a horizontal table heading."""
    heading_deg, roll_deg = float(heading_deg), float(roll_deg)
    if not all(math.isfinite(value) and -180.0 <= value <= 180.0
               for value in (heading_deg, roll_deg)):
        raise ValueError("tool Z heading and axis roll must be within [-180, 180] degrees")
    start = Rotation.from_matrix(np.asarray(start_rotation_world, dtype=float))
    current_axis = start.as_matrix()[:, 2]
    heading = math.radians(heading_deg)
    target_axis = np.asarray([math.cos(heading), math.sin(heading), 0.0])
    target = rotation_between_vectors(current_axis, target_axis) * start
    if roll_deg:
        target = Rotation.from_rotvec(target_axis * math.radians(roll_deg)) * target
    return {
        "rotation_world_tool": target.as_matrix(),
        "current_axis_world": current_axis,
        "target_axis_world": target_axis,
        "selected_direction": "heading",
        "roll_deg": roll_deg,
        "tool_z_heading_deg": heading_deg,
        "rotation_angle_rad": float((start.inv() * target).magnitude()),
    }


def horizontal_tool_z_error_rad(checked):
    return math.asin(float(np.clip(abs(checked["tool_normal_world"][2]), 0.0, 1.0)))


def interpolate_rotations(start_rotation, target_rotation, maximum_step_rad):
    """Return a geodesic rotation sequence including start and target."""
    maximum_step_rad = float(maximum_step_rad)
    if not math.isfinite(maximum_step_rad) or maximum_step_rad <= 0.0:
        raise ValueError("orientation step must be finite and positive")
    start = Rotation.from_matrix(np.asarray(start_rotation, dtype=float))
    target = Rotation.from_matrix(np.asarray(target_rotation, dtype=float))
    relative = start.inv() * target
    rotation_vector = relative.as_rotvec()
    angle = float(np.linalg.norm(rotation_vector))
    count = max(1, int(math.ceil(angle / maximum_step_rad)))
    return [
        (start * Rotation.from_rotvec(rotation_vector * (index / count))).as_matrix()
        for index in range(count + 1)
    ]


def sample_pose_guard_path(
    route,
    start_rotation_world,
    target_rotation_world=None,
    alignment_phase="after",
):
    """Densely sample allowed (magnet-centre, tool-orientation) pairs."""
    route = [finite_point(point, f"pose route point {index}")
             for index, point in enumerate(route)]
    if len(route) < 2:
        raise ValueError("pose guard route must contain at least two points")
    start_rotation_world = np.asarray(start_rotation_world, dtype=float)
    if alignment_phase not in ("before", "after"):
        raise ValueError("alignment phase must be before or after")

    translation_rotation = (
        start_rotation_world
        if target_rotation_world is None or alignment_phase == "after"
        else np.asarray(target_rotation_world, dtype=float)
    )
    translation_centres = [route[0]]
    for left, right in zip(route, route[1:]):
        distance = float(np.linalg.norm(right - left))
        count = max(1, int(math.ceil(distance / POSE_GUARD_POSITION_STEP_M)))
        translation_centres.extend(
            left + (right - left) * (index / count)
            for index in range(1, count + 1)
        )
    translation_rotations = [translation_rotation] * len(translation_centres)

    if target_rotation_world is None:
        centres = translation_centres
        rotations = translation_rotations
    else:
        rotation_path = interpolate_rotations(
            start_rotation_world,
            target_rotation_world,
            POSE_GUARD_ORIENTATION_STEP_RAD,
        )
        rotation_centre = route[0] if alignment_phase == "before" else route[-1]
        rotation_centres = [rotation_centre] * len(rotation_path)
        if alignment_phase == "before":
            centres = [*rotation_centres, *translation_centres[1:]]
            rotations = [*rotation_path, *translation_rotations[1:]]
        else:
            centres = [*translation_centres, *rotation_centres[1:]]
            rotations = [*translation_rotations, *rotation_path[1:]]
    quaternions = np.asarray(
        [Rotation.from_matrix(rotation).as_quat() for rotation in rotations],
        dtype=float,
    )
    return np.asarray(centres, dtype=float), quaternions


def orientation_path_distance(
    magnet_world,
    quaternion_world_tool,
    path_centres,
    path_quaternions,
    position_radius_m,
):
    """Smallest orientation error among allowed poses near a magnet position."""
    point = finite_point(magnet_world, "magnet centre")
    quaternion = np.asarray(quaternion_world_tool, dtype=float)
    centres = np.asarray(path_centres, dtype=float)
    quaternions = np.asarray(path_quaternions, dtype=float)
    if (
        quaternion.shape != (4,)
        or centres.ndim != 2
        or centres.shape[1:] != (3,)
        or quaternions.shape != (len(centres), 4)
        or not np.isfinite([*quaternion, *centres.ravel(), *quaternions.ravel()]).all()
    ):
        raise ValueError("invalid pose path")
    quaternion_norm = float(np.linalg.norm(quaternion))
    quaternion_norms = np.linalg.norm(quaternions, axis=1)
    if quaternion_norm < 1.0e-12 or np.any(quaternion_norms < 1.0e-12):
        raise ValueError("pose path contains an invalid quaternion")
    distances = np.linalg.norm(centres - point, axis=1)
    candidates = distances <= float(position_radius_m)
    if not np.any(candidates):
        candidates[int(np.argmin(distances))] = True
    normalized = quaternions[candidates] / quaternion_norms[candidates, None]
    dots = np.abs(normalized @ (quaternion / quaternion_norm))
    angles = 2.0 * np.arccos(np.clip(dots, 0.0, 1.0))
    return float(np.min(angles))


def build_point_targets(target_mm) -> tuple[list[np.ndarray], dict]:
    target = finite_point(target_mm, "target") / 1000.0
    return [target], {"shape": "point", "target_world_mm": list(map(float, target_mm))}


def resolve_target_height(targets_world, current_world):
    current = np.asarray(current_world, dtype=float)
    resolved = []
    for index, target in enumerate(targets_world):
        if len(target) == 2:
            target = [*target, float(current[2] if current.shape == (3,) else current)]
        elif len(target) == 1 and current.shape == (3,):
            target = [current[0], current[1], target[0]]
        resolved.append(finite_point(target, f"target {index}"))
    return resolved


def build_square_targets(center_mm, size_mm, rotation_deg=0.0, clockwise=False):
    xy_only = len(center_mm) == 2
    center = finite_point([*center_mm, 0.0] if xy_only else center_mm,
                          "square center") / 1000.0
    size = float(size_mm) / 1000.0
    angle = float(rotation_deg)
    if not math.isfinite(size) or not 0.001 <= size <= 0.300:
        raise ValueError("square size must be within [1, 300] mm")
    if not math.isfinite(angle):
        raise ValueError("square rotation must be finite")
    half = size / 2.0
    if clockwise:
        xy = [(-half, -half), (-half, half), (half, half), (half, -half)]
    else:
        xy = [(-half, -half), (half, -half), (half, half), (-half, half)]
    rotation = Rotation.from_euler("z", angle, degrees=True).as_matrix()[:2, :2]
    points = []
    for value in [*xy, xy[0]]:
        offset = rotation @ np.asarray(value)
        points.append(center + [offset[0], offset[1], 0.0])
    if xy_only:
        points = [point[:2] for point in points]
    return points, {
        "shape": "square",
        "center_world_xy_mm" if xy_only else "center_world_mm": list(map(float, center_mm)),
        **({"height": "current_magnet_z"} if xy_only else {}),
        "size_mm": float(size_mm),
        "rotation_deg": angle,
        "clockwise": bool(clockwise),
    }


def build_circle_targets(
    center_mm,
    radius_mm,
    samples=None,
    clockwise=False,
    start_angle_deg=0.0,
):
    center = finite_point(center_mm, "circle center") / 1000.0
    radius = float(radius_mm) / 1000.0
    start = float(start_angle_deg)
    if not math.isfinite(radius) or not 0.0005 <= radius <= 0.150:
        raise ValueError("circle radius must be within [0.5, 150] mm")
    if not math.isfinite(start):
        raise ValueError("circle start angle must be finite")
    minimum = max(24, int(math.ceil(2.0 * math.pi * radius / MAX_CARTESIAN_STEP_M)))
    count = minimum if samples is None else int(samples)
    if count < minimum or count > MAX_TARGETS - 1:
        raise ValueError(
            f"circle samples must be within [{minimum}, {MAX_TARGETS - 1}] "
            "so every chord is no longer than 2 mm"
        )
    direction = -1.0 if clockwise else 1.0
    angles = np.linspace(
        math.radians(start), math.radians(start) + direction * 2.0 * math.pi,
        count + 1,
    )
    points = [
        center + [radius * math.cos(angle), radius * math.sin(angle), 0.0]
        for angle in angles
    ]
    return points, {
        "shape": "circle",
        "center_world_mm": list(map(float, center_mm)),
        "radius_mm": float(radius_mm),
        "samples": count,
        "clockwise": bool(clockwise),
        "start_angle_deg": start,
    }


def load_waypoint_targets(path: Path):
    path = Path(path)
    suffix = path.suffix.lower()
    if suffix in (".yaml", ".yml", ".json"):
        with path.open(encoding="utf-8") as stream:
            data = json.load(stream) if suffix == ".json" else yaml.safe_load(stream)
        if isinstance(data, dict):
            if data.get("frame", "table_world") != "table_world":
                raise ValueError("waypoint file frame must be table_world")
            values = data.get("waypoints_mm")
        else:
            values = data
    elif suffix == ".csv":
        with path.open(newline="", encoding="utf-8") as stream:
            rows = list(csv.DictReader(stream))
        required = {"x_mm", "y_mm", "z_mm"}
        if not rows or not required.issubset(rows[0]):
            raise ValueError("CSV requires x_mm,y_mm,z_mm columns")
        values = [[row["x_mm"], row["y_mm"], row["z_mm"]] for row in rows]
    else:
        raise ValueError("waypoint file must be YAML, JSON, or CSV")
    if not isinstance(values, list) or not values:
        raise ValueError("waypoint file contains no waypoints")
    points = [finite_point(value, f"waypoint {index}") / 1000.0
              for index, value in enumerate(values)]
    if len(points) > MAX_TARGETS:
        raise ValueError(f"at most {MAX_TARGETS} waypoints are allowed")
    return points, {
        "shape": "waypoints",
        "source": str(path.resolve()),
        "count": len(points),
    }


def route_length(points) -> float:
    points = [finite_point(point) for point in points]
    return float(sum(np.linalg.norm(right - left) for left, right in zip(points, points[1:])))


def validate_route(start_world, targets_world):
    start = finite_point(start_world, "current magnet center")
    targets = [finite_point(value, f"target {index}")
               for index, value in enumerate(targets_world)]
    if not targets or len(targets) > MAX_TARGETS:
        raise ValueError(f"target count must be within [1, {MAX_TARGETS}]")
    route = [start, *targets]
    segments = [float(np.linalg.norm(right - left))
                for left, right in zip(route, route[1:])]
    if any(length > MAX_SEGMENT_M for length in segments):
        raise ValueError(f"a path segment exceeds the {MAX_SEGMENT_M * 1000:.0f} mm limit")
    total = sum(segments)
    if total <= 1.0e-6 or total > MAX_TOTAL_PATH_M:
        raise ValueError("total path length must be within (1, 1000] mm")
    return route, segments, total


def subdivide_route(route, maximum_chunk_m):
    """Return ordered polyline targets no farther apart than ``maximum_chunk_m``."""
    maximum_chunk_m = float(maximum_chunk_m)
    if (
        not math.isfinite(maximum_chunk_m)
        or not MIN_PLANNING_CHUNK_M <= maximum_chunk_m <= MAX_PLANNING_CHUNK_M
    ):
        raise ValueError(
            "planning chunk must be within "
            f"[{MIN_PLANNING_CHUNK_M * 1000.0:.0f}, "
            f"{MAX_PLANNING_CHUNK_M * 1000.0:.0f}] mm"
        )
    points = [finite_point(point, f"route point {index}")
              for index, point in enumerate(route)]
    if len(points) < 2:
        raise ValueError("route must contain at least two points")
    result = []
    for left, right in zip(points, points[1:]):
        distance = float(np.linalg.norm(right - left))
        count = max(1, int(math.ceil(distance / maximum_chunk_m)))
        result.extend(
            left + (right - left) * (index / count)
            for index in range(1, count + 1)
        )
    if len(result) > 2000:
        raise ValueError("segmented route contains more than 2000 planning chunks")
    return result


def robot_state_after_trajectory(start_state, trajectory):
    """Copy ``start_state`` and replace joints with a trajectory endpoint."""
    if not trajectory.points:
        raise ValueError("trajectory contains no points")
    result = copy.deepcopy(start_state)
    final = dict(zip(trajectory.joint_names, trajectory.points[-1].positions))
    if any(name not in result.joint_state.name for name in final):
        raise ValueError("trajectory endpoint contains an unknown joint")
    result.joint_state.position = [
        final.get(name, old)
        for name, old in zip(
            result.joint_state.name, result.joint_state.position
        )
    ]
    result.joint_state.velocity = []
    result.joint_state.effort = []
    return result


def concatenate_joint_trajectories(trajectories):
    """Join independently time-parameterized, stationary-ended segments."""
    trajectories = list(trajectories)
    if not trajectories:
        raise ValueError("no trajectories to concatenate")
    combined = JointTrajectory()
    combined.header = copy.deepcopy(trajectories[0].header)
    combined.joint_names = list(trajectories[0].joint_names)
    elapsed = 0.0
    previous = None
    for segment_index, source_trajectory in enumerate(trajectories):
        trajectory = copy.deepcopy(source_trajectory)
        if list(trajectory.joint_names) != list(combined.joint_names):
            raise RuntimeError("segmented trajectories use different joint orders")
        if len(trajectory.points) < 2:
            raise RuntimeError(
                f"planning chunk {segment_index} returned too few points"
            )
        segment_start = trajectory.points[0]
        segment_duration = duration_seconds(
            trajectory.points[-1].time_from_start
        )
        if segment_duration <= 0.0:
            raise RuntimeError(
                f"planning chunk {segment_index} has no time parameterization"
            )
        # Every independently parameterized chunk is a deliberate stop point.
        # Normalize endpoint derivatives before joining so the controller sees
        # an unambiguous stationary boundary rather than two acceleration
        # values at the same position/time.  The modified splines are audited
        # again after concatenation.
        for endpoint in (trajectory.points[0], trajectory.points[-1]):
            if endpoint.velocities:
                maximum_velocity = float(np.max(np.abs(endpoint.velocities)))
                if maximum_velocity > 1.0e-4:
                    raise RuntimeError(
                        f"planning chunk {segment_index} endpoint is not "
                        f"stationary: {maximum_velocity:.6f} rad/s"
                    )
                endpoint.velocities = [0.0] * len(endpoint.velocities)
            if endpoint.accelerations:
                endpoint.accelerations = [0.0] * len(endpoint.accelerations)
        if previous is not None:
            mismatch = float(np.max(np.abs(
                np.asarray(previous.positions, dtype=float)
                - np.asarray(segment_start.positions, dtype=float)
            )))
            if mismatch > 1.0e-4:
                raise RuntimeError(
                    f"planning chunk {segment_index} start mismatch is "
                    f"{mismatch:.6f} rad"
                )
            for field in ("velocities", "accelerations"):
                left = np.asarray(getattr(previous, field), dtype=float)
                right = np.asarray(getattr(segment_start, field), dtype=float)
                if left.size and right.size and (
                    left.shape != right.shape
                    or float(np.max(np.abs(left - right))) > 1.0e-4
                ):
                    raise RuntimeError(
                        f"planning chunk {segment_index} has a discontinuous "
                        f"{field} boundary"
                    )
        for point_index, point in enumerate(trajectory.points):
            if segment_index and point_index == 0:
                continue
            appended = copy.deepcopy(point)
            set_duration(
                appended.time_from_start,
                elapsed + duration_seconds(point.time_from_start),
            )
            combined.points.append(appended)
        elapsed += segment_duration
        previous = combined.points[-1]
    return combined


def point_polyline_distance(point, polyline) -> float:
    point = finite_point(point)
    vertices = np.asarray(polyline, dtype=float)
    if vertices.ndim != 2 or vertices.shape[1] != 3 or len(vertices) < 1:
        raise ValueError("polyline must contain at least one 3-D point")
    if len(vertices) == 1:
        return float(np.linalg.norm(point - vertices[0]))
    left = vertices[:-1]
    delta = vertices[1:] - left
    denominator = np.einsum("ij,ij->i", delta, delta)
    numerator = np.einsum("ij,ij->i", point - left, delta)
    fraction = np.divide(
        numerator,
        denominator,
        out=np.zeros_like(numerator),
        where=denominator > 0.0,
    )
    fraction = np.clip(fraction, 0.0, 1.0)
    projection = left + fraction[:, None] * delta
    return float(np.min(np.linalg.norm(point - projection, axis=1)))


def ordered_waypoint_matches(waypoints, samples):
    """Match each waypoint to a nondecreasing sample index."""
    waypoints = np.asarray(waypoints, dtype=float)
    samples = np.asarray(samples, dtype=float)
    if (
        waypoints.ndim != 2
        or samples.ndim != 2
        or waypoints.shape[1:] != (3,)
        or samples.shape[1:] != (3,)
        or not len(waypoints)
        or not len(samples)
        or not np.isfinite(waypoints).all()
        or not np.isfinite(samples).all()
    ):
        raise ValueError("waypoints and samples must be finite N-by-3 arrays")
    cursor = 0
    matches = []
    for waypoint in waypoints:
        distances = np.linalg.norm(samples[cursor:] - waypoint, axis=1)
        relative_index = int(np.argmin(distances))
        cursor += relative_index
        matches.append({
            "sample_index": cursor,
            "error_m": float(distances[relative_index]),
        })
    return matches


def json_number(value):
    value = float(value)
    return value if math.isfinite(value) else None


def joint_state_sample_time(message, fallback_monotonic_s):
    """Use the driver's acquisition stamp for derivatives, not callback time."""
    stamp = message.header.stamp
    timestamp = float(stamp.sec) + float(stamp.nanosec) * 1.0e-9
    if math.isfinite(timestamp) and timestamp > 0.0:
        return timestamp
    fallback = float(fallback_monotonic_s)
    if not math.isfinite(fallback):
        raise ValueError("joint-state fallback time must be finite")
    return fallback


def write_json_exclusive(path: Path, value):
    with Path(path).open("x", encoding="utf-8") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)


def config_hashes(root: Path):
    names = (
        "ur3_system.yaml",
        "ur3_calibration.yaml",
        "table_world_calibration.yaml",
        "acrylic_side_boundaries.yaml",
        "magnet_tool_geometry_draft.yaml",
    )
    return {
        name: hashlib.sha256((root / "config" / name).read_bytes()).hexdigest()
        for name in names
    }


class FullToolGuard(AcrylicCeilingGuard):
    """MoveIt scene containing the table and the provisional complete tool."""

    def __init__(self, node):
        super().__init__(node)
        root = project_root()
        with (root / "config" / "magnet_tool_geometry_draft.yaml").open(
            encoding="utf-8"
        ) as stream:
            self.tool_data = yaml.safe_load(stream)
        envelope = self.tool_data["provisional_whole_tool_envelope"]
        self.tool_lower = finite_point(envelope["min_xyz_m"], "tool lower bound")
        self.tool_upper = finite_point(envelope["max_xyz_m"], "tool upper bound")
        if np.any(self.tool_upper <= self.tool_lower):
            raise ValueError("invalid complete-tool envelope")
        tcp = np.asarray(self.configuration["magnet_tcp_xyz_m"], dtype=float)
        radius = float(self.configuration["magnet_swept_radius_m"])
        length = float(self.configuration["magnet_swept_length_m"])
        axis = np.asarray(self.configuration["motor_axis_tool_vector"], dtype=float)
        half = radius * np.sqrt(np.maximum(0.0, 1.0 - axis**2)) + length / 2 * np.abs(axis)
        if np.any(tcp - half < self.tool_lower) or np.any(tcp + half > self.tool_upper):
            raise ValueError("complete-tool envelope does not contain the magnetic cylinder")
        self.provisional = (
            self.tool_data.get("status") != "complete_verified"
            or envelope.get("verified_encloses_all_rigid_parts") is not True
            or envelope.get("cable_geometry_included") is not True
        )
        self.robot_description = None

    def _attached_tool_envelope(self):
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = (self.tool_upper - self.tool_lower).tolist()
        pose = Pose()
        center = (self.tool_upper + self.tool_lower) / 2.0
        pose.position.x, pose.position.y, pose.position.z = map(float, center)
        pose.orientation.w = 1.0

        collision = CollisionObject()
        collision.header.frame_id = "tool0"
        collision.id = TOOL_OBJECT_ID
        collision.primitives = [primitive]
        collision.primitive_poses = [pose]
        collision.operation = CollisionObject.ADD

        attached = AttachedCollisionObject()
        attached.link_name = "tool0"
        attached.object = collision
        attached.touch_links = ["tool0", "flange", "wrist_3_link"]
        return attached

    def _forbidden_table_object(self):
        height = 1.0
        top = self.configuration["clearance_limits_m"]["table"]
        return self._world_box(
            TABLE_OBJECT_ID,
            [0.0, 0.0, top - height / 2.0],
            [2.0, 2.0, height],
        )

    def apply(self, robot_state):
        parameters = GetParameters.Request()
        parameters.names = ["robot_description"]
        response = self.node.call(self.description_client, parameters, timeout=30.0)
        self.robot_description = response.values[0].string_value
        if self.configuration["kinematics_hash"] not in self.robot_description:
            raise RuntimeError(
                "MoveIt model is not calibrated for this UR3; use "
                "robot/calibrated_moveit.launch.py"
            )
        request = ApplyPlanningScene.Request()
        request.scene = PlanningScene()
        request.scene.is_diff = True
        request.scene.world.collision_objects = [
            self._forbidden_ceiling_object(),
            *self._forbidden_side_objects(),
            self._forbidden_table_object(),
        ]
        request.scene.robot_state = copy.deepcopy(robot_state)
        request.scene.robot_state.is_diff = True
        request.scene.robot_state.attached_collision_objects = [
            self._attached_tool_envelope()
        ]
        applied = self.node.call(self.apply_client, request, timeout=10.0)
        if not applied.success:
            raise RuntimeError("MoveIt rejected the complete guarded planning scene")

    def decorate_state(self, robot_state):
        guarded = copy.deepcopy(robot_state)
        guarded.is_diff = True
        guarded.attached_collision_objects = [self._attached_tool_envelope()]
        return guarded


class MagnetPathGeometry:
    """Independent world-plane and complete-tool checks for joint states."""

    def __init__(self, robot_description, guard: FullToolGuard):
        if not robot_description:
            raise ValueError("robot_description is unavailable")
        configuration = guard.configuration
        self.model = CeilingGeometry(
            robot_description, configuration["T_base_from_world"]
        )
        self.table = configuration["table_geometry"]
        with (project_root() / "config" / "acrylic_side_boundaries.yaml").open(
            encoding="utf-8"
        ) as stream:
            self.sides = yaml.safe_load(stream)
        self.offset = np.asarray(configuration["magnet_tcp_xyz_m"], dtype=float)
        self.motor_axis_tool = normalized_vector(
            configuration["motor_axis_tool_vector"],
            "motor axis in tool",
        )
        self.radius = float(configuration["magnet_swept_radius_m"])
        self.magnet_length = float(configuration["magnet_swept_length_m"])
        self.magnet_axis_tool = np.asarray(configuration["motor_axis_tool_vector"], dtype=float)
        self.tool_corners = np.asarray(
            [[x, y, z]
             for x in (guard.tool_lower[0], guard.tool_upper[0])
             for y in (guard.tool_lower[1], guard.tool_upper[1])
             for z in (guard.tool_lower[2], guard.tool_upper[2])],
            dtype=float,
        )
        self.limits = load_clearance_limits_m(project_root())

    def inspect(self, names, positions):
        positions = np.asarray(positions, dtype=float)
        if len(names) != len(positions) or not np.isfinite(positions).all():
            raise RuntimeError("invalid joint state for geometry inspection")
        joints = dict(zip(names, positions))
        if any(name not in joints for name in JOINT_NAMES):
            raise RuntimeError("joint state does not contain the six UR joints")
        transforms = self.model.transforms(joints)
        tool_base = transforms["tool0"]
        tool_world = self.model.world_from_base @ tool_base
        rotation_world_tool = tool_world[:3, :3]
        magnet_base = tool_base[:3, 3] + tool_base[:3, :3] @ self.offset
        magnet_world = tool_world[:3, 3] + rotation_world_tool @ self.offset
        motor_axis_world = normalized_vector(
            rotation_world_tool @ self.motor_axis_tool,
            "motor axis in world",
        )
        quaternion_world_tool = Rotation.from_matrix(
            rotation_world_tool
        ).as_quat()

        bounds = self.model.bounds(
            joints, self.offset, self.radius, self.magnet_length, self.magnet_axis_tool
        )
        tool_points = (
            self.tool_corners @ tool_world[:3, :3].T + tool_world[:3, 3]
        )
        bounds[TOOL_OBJECT_ID] = (tool_points.min(axis=0), tool_points.max(axis=0))
        minima = {name: (math.inf, None) for name in self.limits}
        for geometry_name, (lower, upper) in bounds.items():
            gaps = modeled_boundary_gaps_m(self.table, self.sides, lower, upper)
            for boundary, gap in gaps.items():
                if gap < minima[boundary][0]:
                    minima[boundary] = (float(gap), geometry_name)

        for boundary, (gap, geometry_name) in minima.items():
            required = self.limits[boundary]
            if gap + 1.0e-9 < required:
                raise RuntimeError(
                    f"{boundary} clearance failed at {geometry_name}: "
                    f"gap={gap * 1000.0:.3f} mm, "
                    f"required={required * 1000.0:.3f} mm"
                )

        return {
            "tool0_world_mm": (tool_world[:3, 3] * 1000.0).tolist(),
            "tool0_base_mm": (tool_base[:3, 3] * 1000.0).tolist(),
            "magnet_world_mm": (magnet_world * 1000.0).tolist(),
            "magnet_base_mm": (magnet_base * 1000.0).tolist(),
            "tool_quaternion_world_xyzw": quaternion_world_tool.tolist(),
            "tool_normal_world": rotation_world_tool[:, 2].tolist(),
            "motor_axis_world": motor_axis_world.tolist(),
            "gaps_mm": {
                key: json_number(value[0] * 1000.0) for key, value in minima.items()
            },
            "limiting_geometry": {key: value[1] for key, value in minima.items()},
        }


def scale_joint_trajectory(trajectory, factor):
    factor = float(factor)
    if not math.isfinite(factor) or factor <= 0.0:
        raise ValueError("trajectory time scale must be finite and positive")
    for point in trajectory.points:
        set_duration(
            point.time_from_start,
            duration_seconds(point.time_from_start) * factor,
        )
        if point.velocities:
            point.velocities = [value / factor for value in point.velocities]
        if point.accelerations:
            point.accelerations = [value / (factor * factor)
                                   for value in point.accelerations]


def maximum_trajectory_acceleration(trajectory):
    maximum = 0.0
    for point in trajectory.points:
        if point.accelerations:
            maximum = max(
                maximum,
                float(np.max(np.abs(point.accelerations))),
            )
    return maximum


def build_trajectory_trace(trajectory, geometry: MagnetPathGeometry):
    trace = []
    previous_time = None
    previous_q = None
    previous_magnet = None
    previous_rotation = None
    maximum_joint_speed = 0.0
    maximum_magnet_speed = 0.0
    maximum_tool_angular_speed = 0.0
    for timestamp, positions in sample_trajectory(trajectory, TRACE_INTERVAL_S):
        checked = geometry.inspect(trajectory.joint_names, positions)
        magnet = np.asarray(checked["magnet_world_mm"], dtype=float) / 1000.0
        rotation = Rotation.from_quat(
            checked["tool_quaternion_world_xyzw"]
        )
        joint_speed = magnet_speed = tool_angular_speed = 0.0
        if previous_time is not None:
            elapsed = timestamp - previous_time
            if elapsed <= 0.0:
                raise RuntimeError("trajectory sampling produced non-increasing time")
            joint_speed = float(np.max(np.abs(positions - previous_q)) / elapsed)
            magnet_speed = float(np.linalg.norm(magnet - previous_magnet) / elapsed)
            tool_angular_speed = float(
                (previous_rotation.inv() * rotation).magnitude() / elapsed
            )
            maximum_joint_speed = max(maximum_joint_speed, joint_speed)
            maximum_magnet_speed = max(maximum_magnet_speed, magnet_speed)
            maximum_tool_angular_speed = max(
                maximum_tool_angular_speed, tool_angular_speed
            )
        trace.append({
            "time_s": float(timestamp),
            "q_rad": positions.tolist(),
            **checked,
            "joint_speed_deg_s": math.degrees(joint_speed),
            "magnet_speed_mm_s": magnet_speed * 1000.0,
            "tool_angular_speed_deg_s": math.degrees(tool_angular_speed),
        })
        previous_time = timestamp
        previous_q = positions
        previous_magnet = magnet
        previous_rotation = rotation
    return (
        trace,
        maximum_joint_speed,
        maximum_magnet_speed,
        maximum_tool_angular_speed,
    )


class MagnetTrajectoryNode(CartesianLineMove):
    def __init__(self):
        super().__init__(
            node_name="magnet_trajectory",
            guard_factory=FullToolGuard,
        )
        self.geometry = None
        self.target_motor_axis_world = None
        self.target_tool_z_world = None
        self.table_parallel_start_world = None
        self.table_parallel_mode = None
        self.table_parallel_ready = False
        self.tool_orientation_reference_world = None
        self.actual_stream = None
        self.actual_samples = []
        self.last_logged_state_time = None

    def wait_for_fresh_state(self, timeout=5.0, require_execution_state=False):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            state_is_fresh = (
                self.joint_state_received_at is not None
                and time.monotonic() - self.joint_state_received_at <= 0.2
            )
            execution_state_is_ready = (
                not require_execution_state
                or (
                    self.robot_program_running is not None
                    and self.speed_scaling_percent is not None
                )
            )
            if state_is_fresh and execution_state_is_ready:
                return
        joint_age = (None if self.joint_state_received_at is None
                     else round(time.monotonic() - self.joint_state_received_at, 3))
        raise RuntimeError(
            "timed out waiting for fresh robot state "
            f"(require_execution_state={require_execution_state}, "
            f"joint_age_s={joint_age}, program_running={self.robot_program_running}, "
            f"speed_scaling={self.speed_scaling_percent})"
        )

    def validate_controller_interpolation(self, trajectory, start_state):
        """Collision-check interior points of every controller spline segment."""
        names = list(trajectory.joint_names)
        state_indices = {
            name: index for index, name in enumerate(start_state.joint_state.name)
        }
        if any(name not in state_indices for name in names):
            raise RuntimeError("controller trajectory contains an unknown joint")
        sample_count = 0
        for segment_index, (left, right) in enumerate(
            zip(trajectory.points, trajectory.points[1:])
        ):
            duration = (
                duration_seconds(right.time_from_start)
                - duration_seconds(left.time_from_start)
            )
            if duration <= 0.0:
                raise RuntimeError("controller trajectory times are not increasing")
            segment = JointTrajectory()
            segment.joint_names = names
            segment.points = [left, right]
            samples = list(
                sample_trajectory(
                    segment,
                    interval_s=duration / COLLISION_SUBDIVISIONS_PER_SEGMENT,
                )
            )
            for _, positions in samples[1:-1]:
                state = copy.deepcopy(start_state)
                values = list(state.joint_state.position)
                for name, value in zip(names, positions):
                    values[state_indices[name]] = float(value)
                state.joint_state.position = values
                state.joint_state.velocity = []
                state.joint_state.effort = []
                self.ceiling_guard.validate_state(
                    state,
                    f"controller spline {segment_index} interior {sample_count}",
                )
                sample_count += 1
        return sample_count

    def current_tool_pose(self, start_state):
        request = GetPositionFK.Request()
        request.header.frame_id = "base"
        request.fk_link_names = ["tool0"]
        request.robot_state = start_state
        response = self.call(self.fk_client, request)
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f"FK failed, code={response.error_code.val}")
        return response.pose_stamped[0].pose

    def plan_magnet_targets(
        self,
        targets_world,
        speed_m_s,
        planning_chunk_m=None,
        motor_axis_parallel=None,
        motor_axis_direction="nearest",
        motor_axis_roll_deg=0.0,
        table_yaw_deg=0.0,
        tool_z_heading_deg=0.0,
        axis_alignment_phase="after",
    ):
        speed_m_s = float(speed_m_s)
        if not math.isfinite(speed_m_s) or not 0.0 < speed_m_s <= 0.020:
            raise ValueError("moving magnet speed must be within (0, 20] mm/s")
        if motor_axis_parallel is not None and motor_axis_parallel not in (
            "x", "y", "z", "table", "tool-z-table"
        ):
            raise ValueError("axis parallel target must be x, y, z, table, or tool-z-table")
        if motor_axis_direction not in ("nearest", "positive", "negative"):
            raise ValueError(
                "motor-axis direction must be nearest, positive, or negative"
            )
        if motor_axis_parallel is None and motor_axis_direction != "nearest":
            raise ValueError(
                "motor-axis direction requires --motor-axis-parallel"
            )
        if axis_alignment_phase not in ("before", "after"):
            raise ValueError("axis alignment phase must be before or after")
        if motor_axis_parallel == "table" and (
            motor_axis_direction != "nearest" or axis_alignment_phase != "before"
            or motor_axis_roll_deg != 0.0
        ):
            raise ValueError("table-parallel tool requires nearest direction, zero roll and before alignment")
        if motor_axis_parallel != "table" and table_yaw_deg != 0.0:
            raise ValueError("table yaw requires table-parallel tool")
        if motor_axis_parallel == "tool-z-table" and (
            motor_axis_direction != "nearest" or axis_alignment_phase != "before"
        ):
            raise ValueError("horizontal tool Z requires nearest direction and before alignment")
        if motor_axis_parallel != "tool-z-table" and tool_z_heading_deg != 0.0:
            raise ValueError("tool Z heading requires horizontal tool Z mode")
        start_state = self.current_robot_state()
        self.ceiling_guard.apply(start_state)
        self.ceiling_guard.validate_state(start_state, "current robot state")
        wrist3 = dict(zip(start_state.joint_state.name, start_state.joint_state.position))["wrist_3_joint"]
        if abs(wrist3) > WRIST3_ZERO_PLAN_TOLERANCE_RAD:
            raise RuntimeError(
                f"wrist_3_joint must be 0 deg before magnet motion; current={math.degrees(wrist3):.3f} deg"
            )
        start_pose = self.current_tool_pose(start_state)
        rotation_base_tool = quaternion_matrix(start_pose)
        offset = np.asarray(
            self.ceiling_guard.configuration["magnet_tcp_xyz_m"], dtype=float
        )
        start_tool_base = np.asarray(
            [start_pose.position.x, start_pose.position.y, start_pose.position.z]
        )
        start_magnet_base = start_tool_base + rotation_base_tool @ offset
        base_from_world = np.asarray(
            self.ceiling_guard.configuration["T_base_from_world"], dtype=float
        )
        world_from_base = np.linalg.inv(base_from_world)
        start_magnet_world = homogeneous_point(world_from_base, start_magnet_base)
        targets_world = resolve_target_height(targets_world, start_magnet_world)
        if (len(targets_world) > 1
                and np.linalg.norm(targets_world[0] - start_magnet_world) < 0.0005):
            targets_world = targets_world[1:]
        start_rotation_world = world_from_base[:3, :3] @ rotation_base_tool
        self.tool_orientation_reference_world = (
            Rotation.from_matrix(start_rotation_world)
            if motor_axis_parallel is None else None
        )
        self.table_parallel_start_world = (
            start_magnet_world.copy()
            if motor_axis_parallel in ("table", "tool-z-table") else None
        )
        self.table_parallel_mode = motor_axis_parallel
        self.table_parallel_ready = False
        route, segment_lengths, total_length = validate_route(
            start_magnet_world, targets_world
        )

        axis_alignment = None
        target_rotation_world = None
        if motor_axis_parallel is not None:
            if (motor_axis_parallel != "tool-z-table"
                    and not self.ceiling_guard.configuration["motor_axis_verified"]):
                raise RuntimeError(
                    "motor-axis alignment is blocked because the physical "
                    "shaft direction has not been verified"
                )
            if motor_axis_parallel == "tool-z-table":
                axis_alignment = horizontal_tool_z_target_rotation(
                    start_rotation_world, tool_z_heading_deg, motor_axis_roll_deg
                )
            elif motor_axis_parallel == "table":
                axis_alignment = table_parallel_target_rotation(
                    start_rotation_world,
                    self.ceiling_guard.configuration["motor_axis_tool_vector"],
                    table_yaw_deg,
                )
            else:
                axis_alignment = parallel_axis_target_rotation(
                    start_rotation_world,
                    self.ceiling_guard.configuration["motor_axis_tool_vector"],
                    motor_axis_parallel,
                    motor_axis_direction,
                    motor_axis_roll_deg,
                )
            target_rotation_world = axis_alignment["rotation_world_tool"]
            self.target_tool_z_world = (
                axis_alignment["target_axis_world"].copy()
                if motor_axis_parallel == "tool-z-table" else None
            )
            self.target_motor_axis_world = (
                axis_alignment["target_axis_world"].copy()
                if motor_axis_parallel != "tool-z-table" else None
            )
        else:
            self.target_motor_axis_world = None
            self.target_tool_z_world = None

        def pose_from_step(step):
            magnet_world = step["magnet_world"]
            rotation_world = step["rotation_world_tool"]
            magnet_base = homogeneous_point(base_from_world, magnet_world)
            rotation_base = base_from_world[:3, :3] @ rotation_world
            tool_base = magnet_base - rotation_base @ offset
            pose = copy.deepcopy(start_pose)
            if not np.isfinite(tool_base).all():
                raise RuntimeError("a target produced a non-finite tool pose")
            pose.position.x, pose.position.y, pose.position.z = map(float, tool_base)
            quaternion = Rotation.from_matrix(rotation_base).as_quat()
            (
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ) = map(float, quaternion)
            return pose

        translation_centres = (
            targets_world
            if planning_chunk_m is None
            else subdivide_route(route, planning_chunk_m)
        )
        translation_rotation = (
            target_rotation_world
            if target_rotation_world is not None
            and axis_alignment_phase == "before"
            else start_rotation_world
        )
        translation_steps = [
            {
                "phase": "translation",
                "magnet_world": centre,
                "rotation_world_tool": translation_rotation,
            }
            for centre in translation_centres
        ]
        orientation_steps = []
        if target_rotation_world is not None and axis_alignment[
            "rotation_angle_rad"
        ] > math.radians(0.05):
            rotation_centre = (
                route[0] if axis_alignment_phase == "before" else route[-1]
            )
            orientation_steps = [
                {
                    "phase": "axis_alignment",
                    "magnet_world": rotation_centre,
                    "rotation_world_tool": rotation,
                }
                for rotation in interpolate_rotations(
                    start_rotation_world,
                    target_rotation_world,
                    ORIENTATION_WAYPOINT_STEP_RAD,
                )[1:]
            ]
        ordered_steps = (
            [*orientation_steps, *translation_steps]
            if axis_alignment_phase == "before"
            else [*translation_steps, *orientation_steps]
        )
        if not ordered_steps:
            raise RuntimeError("requested pose path contains no motion targets")

        def request_cartesian(segment_state, waypoint_poses):
            # Short segmented paths deliberately stop at each boundary.  Ask
            # MoveIt for a reasonably conditioned time parameterization, then
            # uniformly slow the joined trajectory below the application-level
            # magnet and joint speed limits before any execution is possible.
            parameterization_scale = 0.25 if planning_chunk_m is not None else 0.05
            request = GetCartesianPath.Request()
            request.header.frame_id = "base"
            request.start_state = self.ceiling_guard.decorate_state(segment_state)
            request.group_name = "ur_manipulator"
            request.link_name = "tool0"
            request.waypoints = waypoint_poses
            request.max_step = MAX_CARTESIAN_STEP_M
            request.jump_threshold = 2.0
            request.prismatic_jump_threshold = 0.0
            request.revolute_jump_threshold = 0.10
            request.avoid_collisions = True
            request.max_velocity_scaling_factor = parameterization_scale
            request.max_acceleration_scaling_factor = parameterization_scale
            return self.call(self.cartesian_client, request, timeout=60.0)

        if planning_chunk_m is None:
            planning_groups = [("combined", ordered_steps)]
        else:
            planning_groups = []
            if axis_alignment_phase == "before" and orientation_steps:
                planning_groups.append(("axis_alignment", orientation_steps))
            planning_groups.extend(
                ("translation", [step]) for step in translation_steps
            )
            if axis_alignment_phase == "after" and orientation_steps:
                planning_groups.append(("axis_alignment", orientation_steps))

        planning_chunks = []
        segment_state = copy.deepcopy(start_state)
        segment_trajectories = []
        fractions = []
        single_response = None
        for chunk_index, (phase, steps) in enumerate(planning_groups):
            waypoint_poses = [pose_from_step(step) for step in steps]
            response = request_cartesian(segment_state, waypoint_poses)
            single_response = response
            fraction_value = float(response.fraction)
            fractions.append(fraction_value)
            endpoint = steps[-1]
            if planning_chunk_m is not None:
                endpoint_axis = (
                    endpoint["rotation_world_tool"]
                    @ normalized_vector(
                        self.ceiling_guard.configuration[
                            "motor_axis_tool_vector"
                        ],
                        "motor axis in tool",
                    )
                )
                planning_chunks.append({
                    "index": chunk_index,
                    "phase": phase,
                    "waypoint_count": len(steps),
                    "target_world_mm": (
                        endpoint["magnet_world"] * 1000.0
                    ).tolist(),
                    "target_motor_axis_world": endpoint_axis.tolist(),
                    "fraction": fraction_value,
                    "trajectory_points": len(
                        response.solution.joint_trajectory.points
                    ),
                })
            label = (
                "Cartesian planning"
                if planning_chunk_m is None
                else f"Cartesian planning chunk {chunk_index} ({phase})"
            )
            if response.error_code.val != MoveItErrorCodes.SUCCESS:
                raise RuntimeError(
                    f"{label} failed, code={response.error_code.val}"
                )
            if fraction_value < 0.999999:
                raise RuntimeError(
                    f"{label} incomplete: {fraction_value * 100.0:.3f}%"
                )
            segment = response.solution.joint_trajectory
            if len(segment.points) < 2:
                raise RuntimeError(f"{label} returned too few trajectory points")
            self.unwrap_and_check_joint_limits(segment, segment_state)
            hold_wrist3_zero(segment)
            self.validate_trajectory_states(segment, segment_state)
            segment_trajectories.append(segment)
            segment_state = robot_state_after_trajectory(segment_state, segment)

        if planning_chunk_m is None:
            solution = single_response.solution
        else:
            solution = RobotTrajectory()
            solution.joint_trajectory = concatenate_joint_trajectories(
                segment_trajectories
            )
        trajectory = solution.joint_trajectory
        wrist3_index = trajectory.joint_names.index("wrist_3_joint")
        wrist3_error = max(abs(point.positions[wrist3_index]) for point in trajectory.points)
        if wrist3_error > WRIST3_ZERO_PLAN_TOLERANCE_RAD:
            raise RuntimeError(
                f"planned wrist_3_joint departed 0 deg by {math.degrees(wrist3_error):.3f} deg"
            )
        fraction = min(fractions)

        self.geometry = MagnetPathGeometry(
            self.ceiling_guard.robot_description,
            self.ceiling_guard,
        )
        self.geometry.inspect(
            start_state.joint_state.name,
            start_state.joint_state.position,
        )

        original_duration = duration_seconds(trajectory.points[-1].time_from_start)
        if original_duration <= 0.0:
            raise RuntimeError("Cartesian trajectory has no time parameterization")
        orientation_angle = (
            0.0
            if axis_alignment is None
            else axis_alignment["rotation_angle_rad"]
        )
        requested_duration = (
            total_length / speed_m_s
            + orientation_angle / PLANNED_TOOL_ANGULAR_SPEED_LIMIT_RAD_S
        )
        duration_limit_s = (
            MAX_SEGMENTED_DURATION_S
            if planning_chunk_m is not None
            else MAX_DURATION_S
        )
        minimum_time_scale = 0.5 if planning_chunk_m is not None else 1.0
        initial_time_scale = max(
            minimum_time_scale,
            requested_duration / original_duration,
        )
        scale_joint_trajectory(trajectory, initial_time_scale)
        scaled_duration = duration_seconds(trajectory.points[-1].time_from_start)
        if scaled_duration > duration_limit_s:
            raise RuntimeError(
                "initially scaled planned duration "
                f"{scaled_duration:.1f}s exceeds {duration_limit_s:.0f}s "
                f"(original={original_duration:.1f}s, "
                f"requested={requested_duration:.1f}s, "
                f"scale={initial_time_scale:.3f})"
            )
        (
            trace,
            maximum_joint_speed,
            maximum_magnet_speed,
            maximum_tool_angular_speed,
        ) = build_trajectory_trace(
            trajectory,
            self.geometry,
        )
        for _ in range(4):
            additional_scale = max(
                1.0,
                maximum_joint_speed / PLANNED_JOINT_SPEED_LIMIT_RAD_S,
                maximum_magnet_speed / speed_m_s,
                maximum_tool_angular_speed
                / PLANNED_TOOL_ANGULAR_SPEED_LIMIT_RAD_S,
            )
            if additional_scale <= 1.0000001:
                break
            # Sampling positions shift slightly as timestamps are rescaled.
            additional_scale *= 1.02
            duration_before_speed_limits = scaled_duration
            joint_speed_before_limits = maximum_joint_speed
            magnet_speed_before_limits = maximum_magnet_speed
            angular_speed_before_limits = maximum_tool_angular_speed
            scale_joint_trajectory(trajectory, additional_scale)
            scaled_duration = duration_seconds(
                trajectory.points[-1].time_from_start
            )
            if scaled_duration > duration_limit_s:
                raise RuntimeError(
                    "speed-limited planned duration "
                    f"{scaled_duration:.1f}s exceeds {duration_limit_s:.0f}s "
                    f"(before={duration_before_speed_limits:.1f}s, "
                    f"joint_peak={math.degrees(joint_speed_before_limits):.3f}deg/s, "
                    f"magnet_peak={magnet_speed_before_limits * 1000.0:.3f}mm/s, "
                    f"tool_angular_peak={math.degrees(angular_speed_before_limits):.3f}deg/s, "
                    f"scale={additional_scale:.3f})"
                )
            (
                trace,
                maximum_joint_speed,
                maximum_magnet_speed,
                maximum_tool_angular_speed,
            ) = build_trajectory_trace(
                trajectory,
                self.geometry,
            )
        if maximum_joint_speed > PLANNED_JOINT_SPEED_LIMIT_RAD_S + 1.0e-9:
            raise RuntimeError(
                "planned joint speed remains above the application limit: "
                f"{math.degrees(maximum_joint_speed):.6f} deg/s > "
                f"{math.degrees(PLANNED_JOINT_SPEED_LIMIT_RAD_S):.6f} deg/s"
            )
        if maximum_magnet_speed > speed_m_s + 1.0e-9:
            raise RuntimeError(
                "planned magnet speed remains above the requested limit: "
                f"{maximum_magnet_speed * 1000.0:.6f} mm/s > "
                f"{speed_m_s * 1000.0:.6f} mm/s"
            )
        if (
            maximum_tool_angular_speed
            > PLANNED_TOOL_ANGULAR_SPEED_LIMIT_RAD_S + 1.0e-9
        ):
            raise RuntimeError(
                "planned tool angular speed remains above the application "
                f"limit: {math.degrees(maximum_tool_angular_speed):.6f} deg/s > "
                f"{math.degrees(PLANNED_TOOL_ANGULAR_SPEED_LIMIT_RAD_S):.6f} deg/s"
            )
        if motor_axis_parallel in ("table", "tool-z-table"):
            for entry in trace:
                magnet = np.asarray(entry["magnet_world_mm"]) / 1000.0
                error = (horizontal_tool_z_error_rad(entry)
                         if motor_axis_parallel == "tool-z-table"
                         else table_tilt_rad(entry))
                if (np.linalg.norm(magnet - start_magnet_world) > 0.001
                        and error > PLANNED_TABLE_TILT_TOLERANCE_RAD):
                    raise RuntimeError("planned tool axis is not parallel to the table")
        final_duration = duration_seconds(trajectory.points[-1].time_from_start)
        if final_duration > duration_limit_s:
            raise RuntimeError(
                f"planned duration {final_duration:.1f}s exceeds "
                f"{duration_limit_s:.0f}s"
            )
        maximum_joint_acceleration = maximum_trajectory_acceleration(trajectory)
        if (
            maximum_joint_acceleration
            > PLANNED_JOINT_ACCELERATION_LIMIT_RAD_S2 + 1.0e-6
        ):
            raise RuntimeError(
                "planned joint acceleration exceeds MoveIt limit: "
                f"{maximum_joint_acceleration:.3f} rad/s^2 > "
                f"{PLANNED_JOINT_ACCELERATION_LIMIT_RAD_S2:.3f} rad/s^2"
            )
        final_world = np.asarray(trace[-1]["magnet_world_mm"]) / 1000.0
        final_error = float(np.linalg.norm(final_world - targets_world[-1]))
        if final_error > 0.00025:
            raise RuntimeError(
                f"planned final magnet error is {final_error * 1000.0:.3f} mm"
            )
        planned_polyline = np.asarray(
            [entry["magnet_world_mm"] for entry in trace], dtype=float
        ) / 1000.0
        intended_polyline = np.asarray(route, dtype=float)
        maximum_path_error = max(
            point_polyline_distance(point, intended_polyline)
            for point in planned_polyline
        )
        ordered_matches = ordered_waypoint_matches(
            intended_polyline, planned_polyline
        )
        maximum_waypoint_miss = max(
            match["error_m"] for match in ordered_matches
        )
        if max(maximum_path_error, maximum_waypoint_miss) > PLANNED_PATH_TOLERANCE_M:
            raise RuntimeError(
                "time-parameterized magnet path does not follow the requested "
                f"polyline within {PLANNED_PATH_TOLERANCE_M * 1000.0:.2f} mm: "
                f"path error={maximum_path_error * 1000.0:.3f} mm, "
                f"waypoint miss={maximum_waypoint_miss * 1000.0:.3f} mm"
            )
        pose_centres, pose_quaternions = sample_pose_guard_path(
            route,
            start_rotation_world,
            target_rotation_world,
            axis_alignment_phase,
        )
        orientation_path_errors = [
            orientation_path_distance(
                np.asarray(entry["magnet_world_mm"], dtype=float) / 1000.0,
                entry["tool_quaternion_world_xyzw"],
                pose_centres,
                pose_quaternions,
                PLANNED_PATH_TOLERANCE_M + POSE_GUARD_POSITION_STEP_M,
            )
            for entry in trace
        ]
        maximum_orientation_path_error = max(orientation_path_errors)
        orientation_tolerance = (
            WRIST3_FIXED_TOOL_ORIENTATION_TOLERANCE_RAD
            if motor_axis_parallel is None else PLANNED_ORIENTATION_PATH_TOLERANCE_RAD
        )
        if maximum_orientation_path_error > orientation_tolerance:
            raise RuntimeError(
                "time-parameterized tool orientation departed the requested "
                "pose path: "
                f"{math.degrees(maximum_orientation_path_error):.3f} deg > "
                f"{math.degrees(orientation_tolerance):.3f} deg"
            )

        final_motor_axis = normalized_vector(
            trace[-1]["motor_axis_world"], "planned final motor axis"
        )
        final_axis_error = None
        alignment_report = None
        if axis_alignment is not None:
            target_motor_axis = axis_alignment["target_axis_world"]
            planned_axis = (
                normalized_vector(trace[-1]["tool_normal_world"], "planned tool Z")
                if motor_axis_parallel == "tool-z-table" else final_motor_axis
            )
            final_axis_error = math.acos(float(np.clip(
                np.dot(planned_axis, target_motor_axis), -1.0, 1.0
            )))
            if final_axis_error > FINAL_AXIS_TOLERANCE_RAD:
                raise RuntimeError(
                    "planned final motor-axis error is "
                    f"{math.degrees(final_axis_error):.3f} deg"
                )
            alignment_report = {
                "world_axis": motor_axis_parallel,
                "requested_direction": motor_axis_direction,
                "selected_direction": axis_alignment["selected_direction"],
                "roll_about_target_axis_deg": axis_alignment["roll_deg"],
                "table_yaw_deg": axis_alignment.get("table_yaw_deg"),
                "tool_z_heading_deg": axis_alignment.get("tool_z_heading_deg"),
                "phase": axis_alignment_phase,
                "current_axis_world": axis_alignment[
                    "current_axis_world"
                ].tolist(),
                "target_axis_world": target_motor_axis.tolist(),
                "minimum_rotation_deg": math.degrees(
                    axis_alignment["rotation_angle_rad"]
                ),
                "orientation_waypoint_count": len(orientation_steps),
                "planned_final_axis_world": planned_axis.tolist(),
                "planned_final_axis_error_deg": math.degrees(final_axis_error),
            }
        collision_samples = self.validate_controller_interpolation(
            trajectory, start_state
        )
        minimum_planned_clearance = {
            boundary: min(entry["gaps_mm"][boundary] for entry in trace)
            for boundary in ("ceiling", "left", "right", "table")
        }
        return {
            "solution": solution,
            "start_state": start_state,
            "start_magnet_world_m": start_magnet_world.tolist(),
            "targets_world_m": [point.tolist() for point in targets_world],
            "route_world_m": [point.tolist() for point in route],
            "segment_lengths_m": segment_lengths,
            "total_length_m": total_length,
            "fraction": fraction,
            "planning_chunk_mm": (
                None if planning_chunk_m is None else planning_chunk_m * 1000.0
            ),
            "planning_chunks": planning_chunks,
            "duration_s": final_duration,
            "duration_limit_s": duration_limit_s,
            "requested_translation_duration_s": total_length / speed_m_s,
            "requested_orientation_duration_s": (
                orientation_angle / PLANNED_TOOL_ANGULAR_SPEED_LIMIT_RAD_S
            ),
            "initial_time_scale": initial_time_scale,
            "maximum_planned_joint_speed_deg_s": math.degrees(maximum_joint_speed),
            "maximum_planned_joint_acceleration_rad_s2": maximum_joint_acceleration,
            "maximum_planned_magnet_speed_mm_s": maximum_magnet_speed * 1000.0,
            "maximum_planned_tool_angular_speed_deg_s": math.degrees(
                maximum_tool_angular_speed
            ),
            "motor_axis_alignment": alignment_report,
            "planned_trace": trace,
            "maximum_requested_path_error_mm": maximum_path_error * 1000.0,
            "maximum_requested_waypoint_miss_mm": maximum_waypoint_miss * 1000.0,
            "maximum_orientation_path_error_deg": math.degrees(
                maximum_orientation_path_error
            ),
            "ordered_waypoint_sample_indices": [
                match["sample_index"] for match in ordered_matches
            ],
            "controller_interpolation_collision_samples": collision_samples,
            "minimum_planned_clearance_mm": minimum_planned_clearance,
            "final_error_mm": final_error * 1000.0,
        }

    def start_actual_recording(self, path: Path | None):
        self.actual_stream = Path(path).open("x", encoding="utf-8", buffering=1) if path is not None else None
        self.recording_error = None
        # No-file mode keeps only the latest measurement for safety/final checks.
        self.actual_samples = [] if path is not None else deque(maxlen=1)
        self.last_logged_state_time = None
        self.table_parallel_ready = False
        self.execution_validator = self.monitor_execution_state

    def stop_actual_recording(self):
        self.execution_validator = None
        if self.actual_stream is not None:
            self.actual_stream.close()
            self.actual_stream = None

    def monitor_execution_state(self, _robot_state):
        if self.robot_program_running is False:
            raise RuntimeError("External Control program stopped during execution")
        received = self.joint_state_received_at
        if received is None:
            return
        message = self.latest_joint_state
        sample_time = joint_state_sample_time(message, received)
        if (
            self.last_logged_state_time is not None
            and sample_time <= self.last_logged_state_time
        ):
            return
        mapping = dict(zip(message.name, message.position))
        wrist3_error = abs(mapping["wrist_3_joint"])
        if wrist3_error > WRIST3_ZERO_LIVE_TOLERANCE_RAD:
            raise RuntimeError(
                f"wrist_3_joint departed 0 deg by {math.degrees(wrist3_error):.3f} deg"
            )
        positions = np.asarray([mapping[name] for name in JOINT_NAMES], dtype=float)
        checked = self.geometry.inspect(JOINT_NAMES, positions)
        if self.tool_orientation_reference_world is not None:
            current_rotation = Rotation.from_quat(checked["tool_quaternion_world_xyzw"])
            deviation = (self.tool_orientation_reference_world.inv() * current_rotation).magnitude()
            if deviation > WRIST3_FIXED_TOOL_ORIENTATION_TOLERANCE_RAD:
                raise RuntimeError(
                    f"tool orientation departed the start by {math.degrees(deviation):.3f} deg"
                )
        if self.table_parallel_start_world is not None:
            tilt = (horizontal_tool_z_error_rad(checked)
                    if self.table_parallel_mode == "tool-z-table"
                    else table_tilt_rad(checked))
            magnet = np.asarray(checked["magnet_world_mm"]) / 1000.0
            if tilt <= math.radians(1.0):
                self.table_parallel_ready = True
            elif (self.table_parallel_ready
                  or np.linalg.norm(magnet - self.table_parallel_start_world) > 0.001):
                if tilt > LIVE_TABLE_TILT_TOLERANCE_RAD:
                    raise RuntimeError(
                        f"tool0 axis differs from table parallel by {math.degrees(tilt):.2f} degrees"
                    )
        entry = {
            "host_time_ns": time.time_ns(),
            "monotonic_s": received,
            "joint_state_stamp_s": sample_time,
            "q_rad": positions.tolist(),
            **checked,
        }
        self.actual_samples.append(entry)
        if self.actual_stream is not None:
            try:
                self.actual_stream.write(json.dumps(entry, ensure_ascii=False, allow_nan=False) + "\n")
            except OSError as error:
                self.recording_error = str(error)
                self.get_logger().warning(f"trajectory recording stopped: {error}")
                stream, self.actual_stream = self.actual_stream, None
                try:
                    stream.close()
                except OSError:
                    pass
        self.last_logged_state_time = sample_time


def trace_from_plan(path: Path):
    with Path(path).open(encoding="utf-8") as stream:
        data = json.load(stream)
    trace = data.get("planned_trace") or data.get("planning", {}).get("planned_trace")
    if not isinstance(trace, list) or not trace:
        raise ValueError(f"no planned_trace in {path}")
    return trace


def requested_route_from_plan(path: Path):
    with Path(path).open(encoding="utf-8") as stream:
        data = json.load(stream)
    route = data.get("planning", {}).get("route_world_m")
    if route is None:
        return None
    points = np.asarray(route, dtype=float)
    if (
        points.ndim != 2
        or points.shape[1] != 3
        or len(points) < 2
        or not np.isfinite(points).all()
    ):
        raise ValueError(f"invalid requested route in {path}")
    return points * 1000.0


def trace_from_jsonl(path: Path):
    values = []
    with Path(path).open(encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            value = json.loads(line)
            if "magnet_world_mm" not in value:
                raise ValueError(f"line {line_number} has no magnet_world_mm")
            values.append(value)
    if not values:
        raise ValueError(f"no trajectory samples in {path}")
    first = values[0].get(
        "joint_state_stamp_s", values[0].get("monotonic_s", 0.0)
    )
    for index, value in enumerate(values):
        timestamp = value.get(
            "joint_state_stamp_s", value.get("monotonic_s", float(index))
        )
        value.setdefault("time_s", timestamp - first)
    return values


def plot_trajectory(input_path: Path, output_path: Path):
    input_path = Path(input_path)
    planned = actual = requested = None
    if input_path.is_dir():
        plan_path = input_path / "plan.json"
        actual_path = input_path / "actual_magnet_path.jsonl"
        if plan_path.exists():
            planned = trace_from_plan(plan_path)
            requested = requested_route_from_plan(plan_path)
        if actual_path.exists() and actual_path.stat().st_size:
            actual = trace_from_jsonl(actual_path)
    elif input_path.suffix.lower() == ".jsonl":
        actual = trace_from_jsonl(input_path)
    else:
        planned = trace_from_plan(input_path)
        requested = requested_route_from_plan(input_path)
    if planned is None and actual is None:
        raise ValueError("no planned or measured magnet-centre trace found")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figure, (axis_xy, axis_time) = plt.subplots(1, 2, figsize=(12, 5))
    if requested is not None:
        axis_xy.plot(
            requested[:, 0],
            requested[:, 1],
            ":",
            color="black",
            linewidth=1.2,
            label="requested",
        )
        axis_xy.scatter(
            requested[1:, 0],
            requested[1:, 1],
            marker=".",
            color="black",
            s=18,
        )
    for label, trace, style in (
        ("planned", planned, "--"),
        ("measured", actual, "-"),
    ):
        if trace is None:
            continue
        points = np.asarray([entry["magnet_world_mm"] for entry in trace], dtype=float)
        times = np.asarray([entry.get("time_s", index * TRACE_INTERVAL_S)
                            for index, entry in enumerate(trace)], dtype=float)
        times -= times[0]
        axis_xy.plot(points[:, 0], points[:, 1], style, label=label)
        axis_time.plot(times, points[:, 0], style, label=f"X {label}")
        axis_time.plot(times, points[:, 1], style, label=f"Y {label}")
        axis_time.plot(times, points[:, 2], style, label=f"Z {label}")
        axis_xy.scatter(points[0, 0], points[0, 1], marker="o", s=30)
        axis_xy.scatter(points[-1, 0], points[-1, 1], marker="x", s=40)
    axis_xy.set_title("Magnet-centre path in table_world")
    axis_xy.set_xlabel("X [mm]")
    axis_xy.set_ylabel("Y [mm]")
    axis_xy.axis("equal")
    axis_xy.grid(True)
    axis_xy.legend()
    axis_time.set_title("Magnet-centre coordinates")
    axis_time.set_xlabel("Time [s]")
    axis_time.set_ylabel("Position [mm]")
    axis_time.grid(True)
    axis_time.legend(fontsize="small", ncol=2)
    figure.tight_layout()
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output_path, dpi=160)
    plt.close(figure)
    return output_path


def add_motion_arguments(parser):
    parser.add_argument("--speed-mm-s", type=float, default=5.0)
    motor_command = parser.add_mutually_exclusive_group()
    motor_command.add_argument("--motor-rpm", type=float,
                               help="rotate in rpm (negative=reverse); stop and return to saved origin after success")
    motor_command.add_argument("--motor-position-deg", type=float,
                               help="command ZE300 absolute position in degrees")
    motor_command.add_argument("--motor-relative-deg", type=float,
                               help="command ZE300 relative motion in degrees")
    parser.add_argument("--motor-port", default=DEFAULT_MOTOR_PORT)
    parser.add_argument("--motor-address", type=int, default=1)
    parser.add_argument("--motor-baud", type=int, default=115200)
    axis_group = parser.add_mutually_exclusive_group()
    axis_group.add_argument(
        "--motor-axis-parallel",
        choices=("x", "y", "z", "table"),
        help=(
            "align the motor shaft to a table_world axis, or keep the tool0 "
            "XY plane parallel to the table"
        ),
    )
    axis_group.add_argument(
        "--tool-z-parallel-table",
        dest="motor_axis_parallel",
        action="store_const",
        const="tool-z-table",
        help="keep tool0 +Z parallel to the table during translation",
    )
    parser.add_argument(
        "--motor-axis-direction",
        choices=("nearest", "positive", "negative"),
        default="nearest",
        help=(
            "choose +axis, -axis, or the direction requiring the smallest "
            "rotation (default: nearest)"
        ),
    )
    parser.add_argument(
        "--motor-axis-roll-deg", "--axis-roll-deg",
        type=float,
        default=0.0,
        help=(
            "remaining roll about the selected target axis after alignment; "
            "useful for choosing a reachable wrist posture (default: 0)"
        ),
    )
    parser.add_argument(
        "--table-yaw-deg",
        type=float,
        default=0.0,
        help="rotate around the table normal after leveling tool0 (default: 0)",
    )
    parser.add_argument(
        "--tool-z-heading-deg", type=float, default=0.0,
        help="horizontal tool0 +Z heading in table_world XY, measured from +X",
    )
    parser.add_argument(
        "--axis-alignment-phase",
        choices=("before", "after"),
        default="after",
        help=(
            "rotate about the magnet centre before or after the requested "
            "position path (default: after)"
        ),
    )
    parser.add_argument(
        "--planning-chunk-mm",
        type=float,
        help=(
            "plan the requested polyline as consecutive short Cartesian "
            "segments; useful near a kinematic branch boundary"
        ),
    )
    parser.add_argument("--output", type=Path)
    parser.add_argument("--execute", action="store_true",
                        help="automatically prepare motion services and External Control, then execute")
    recording = parser.add_mutually_exclusive_group()
    recording.add_argument("--no", nargs="?", const="all", choices=("all", "camera"),
                           help="write no data; '--no camera' is retained for compatibility (camera files require web recording)")
    recording.add_argument("--no-record", action="store_true",
                           help="alias for --no: write no data files")


def wrist3_zero_trajectory(joint_positions):
    start = np.asarray(joint_positions, dtype=float)
    if start.shape != (len(JOINT_NAMES),) or not np.isfinite(start).all():
        raise ValueError("six finite joint positions are required")
    target = start.copy()
    target[JOINT_NAMES.index("wrist_3_joint")] = 0.0
    duration = max(2.0, abs(start[-1]) / PLANNED_JOINT_SPEED_LIMIT_RAD_S * 2.0)
    trajectory = JointTrajectory()
    trajectory.header.frame_id = "base"
    trajectory.joint_names = list(JOINT_NAMES)
    for positions, time_s in ((start, 0.0), (target, duration)):
        point = JointTrajectoryPoint()
        point.positions = positions.tolist()
        point.velocities = [0.0] * len(JOINT_NAMES)
        point.accelerations = [0.0] * len(JOINT_NAMES)
        set_duration(point.time_from_start, time_s)
        trajectory.points.append(point)
    return trajectory


def hold_wrist3_zero(trajectory):
    index = list(trajectory.joint_names).index("wrist_3_joint")
    for point in trajectory.points:
        point.positions[index] = 0.0
        if point.velocities:
            point.velocities[index] = 0.0
        if point.accelerations:
            point.accelerations[index] = 0.0


def run_wrist3_zero(execute):
    rclpy.init()
    node = MagnetTrajectoryNode()
    try:
        prepare_motion_stack(node, project_root(), no_record=True)
        node.wait_for_fresh_state()
        state = node.current_robot_state()
        node.ceiling_guard.apply(state)
        node.ceiling_guard.validate_state(state, "current robot state")
        joints = dict(zip(state.joint_state.name, state.joint_state.position))
        trajectory = wrist3_zero_trajectory([joints[name] for name in JOINT_NAMES])
        angle_deg = math.degrees(joints["wrist_3_joint"])
        if abs(angle_deg) <= math.degrees(WRIST3_ZERO_PLAN_TOLERANCE_RAD):
            print(f"WRIST3_ALREADY_ZERO current={angle_deg:.3f}deg")
            return
        node.geometry = MagnetPathGeometry(
            node.ceiling_guard.robot_description, node.ceiling_guard
        )
        node.validate_trajectory_states(trajectory, state)
        node.validate_controller_interpolation(trajectory, state)
        trace, joint_speed, magnet_speed, angular_speed = build_trajectory_trace(
            trajectory, node.geometry
        )
        if (joint_speed > PLANNED_JOINT_SPEED_LIMIT_RAD_S
                or angular_speed > PLANNED_TOOL_ANGULAR_SPEED_LIMIT_RAD_S
                or magnet_speed > 0.0001):
            raise RuntimeError("wrist3-zero path exceeds speed or magnet-centre hold limit")
        print(f"WRIST3_ZERO_PLAN current={angle_deg:.3f}deg duration={trace[-1]['time_s']:.2f}s")
        if not execute:
            return
        def monitor(_state):
            message = node.latest_joint_state
            actual = dict(zip(message.name, message.position))
            node.geometry.inspect(JOINT_NAMES, [actual[name] for name in JOINT_NAMES])
        node.execution_validator = monitor
        solution = RobotTrajectory()
        solution.joint_trajectory = trajectory
        ensure_external_control(node)
        node.wait_for_fresh_state(timeout=2.0, require_execution_state=True)
        node.execute(solution)
        node.wait_for_fresh_state(require_execution_state=True)
        actual = dict(zip(node.latest_joint_state.name, node.latest_joint_state.position))
        final_deg = math.degrees(actual["wrist_3_joint"])
        if abs(final_deg) > math.degrees(WRIST3_ZERO_PLAN_TOLERANCE_RAD):
            raise RuntimeError(f"wrist_3_joint ended at {final_deg:.3f}deg, expected 0deg")
        print(f"WRIST3_ZERO_SUCCESS actual={final_deg:.3f}deg")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def parse_arguments(arguments):
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    point = subparsers.add_parser("point", help="move magnet centre to one point")
    point_target = point.add_mutually_exclusive_group(required=True)
    point_target.add_argument("--target-mm", nargs=3, type=float,
                              metavar=("X", "Y", "Z"))
    point_target.add_argument("--target-xy-mm", nargs=2, type=float,
                              metavar=("X", "Y"),
                              help="keep the current magnet-centre Z height")
    point_target.add_argument("--target-z-mm", type=float, metavar="Z",
                              help="keep the current magnet-centre X and Y")
    add_motion_arguments(point)

    square = subparsers.add_parser("square", help="trace a closed XY square")
    square_center = square.add_mutually_exclusive_group(required=True)
    square_center.add_argument("--center-mm", nargs=3, type=float,
                               metavar=("X", "Y", "Z"))
    square_center.add_argument("--center-xy-mm", nargs=2, type=float,
                               metavar=("X", "Y"),
                               help="keep the current magnet-centre Z height")
    square.add_argument("--size-mm", type=float, required=True)
    square.add_argument("--rotation-deg", type=float, default=0.0)
    square.add_argument("--clockwise", action="store_true")
    add_motion_arguments(square)
    square.set_defaults(motor_axis_parallel="tool-z-table", axis_alignment_phase="before")

    circle = subparsers.add_parser("circle", help="trace a closed XY circle")
    circle.add_argument("--center-mm", nargs=3, type=float, required=True,
                        metavar=("X", "Y", "Z"))
    circle.add_argument("--radius-mm", type=float, required=True)
    circle.add_argument("--samples", type=int)
    circle.add_argument("--start-angle-deg", type=float, default=0.0)
    circle.add_argument("--clockwise", action="store_true")
    add_motion_arguments(circle)

    waypoints = subparsers.add_parser("waypoints", help="load YAML/JSON/CSV points")
    waypoints.add_argument("--file", type=Path, required=True)
    add_motion_arguments(waypoints)

    plot = subparsers.add_parser("plot", help="plot a saved plan or measured JSONL")
    plot.add_argument("--input", type=Path, required=True)
    plot.add_argument("--output", type=Path)
    wrist3 = subparsers.add_parser("wrist3-zero", help="rotate only wrist 3 to 0 degrees; no data recorded")
    wrist3.add_argument("--execute", action="store_true")
    parsed = parser.parse_args(arguments)
    if hasattr(parsed, "no_record"):
        parsed.no_record = parsed.no_record or parsed.no == "all"
        parsed.no_camera = parsed.no_record or parsed.no == "camera"
        if parsed.no_record and parsed.output is not None:
            parser.error("--no / --no-record cannot be combined with --output")
    return parsed


def requested_targets(arguments):
    if arguments.command == "point":
        if arguments.target_z_mm is not None:
            if not math.isfinite(arguments.target_z_mm):
                raise ValueError("target Z must be finite")
            return [np.asarray([arguments.target_z_mm / 1000.0])], {
                "shape": "point", "target_world_z_mm": arguments.target_z_mm,
                "xy": "current_magnet_xy",
            }
        if arguments.target_xy_mm is not None:
            xy = np.asarray(arguments.target_xy_mm, dtype=float)
            if not np.isfinite(xy).all():
                raise ValueError("target XY must be finite")
            return [xy / 1000.0], {
                "shape": "point", "target_world_xy_mm": xy.tolist(),
                "height": "current_magnet_z",
            }
        return build_point_targets(arguments.target_mm)
    if arguments.command == "square":
        return build_square_targets(
            arguments.center_xy_mm if arguments.center_xy_mm is not None
            else arguments.center_mm,
            arguments.size_mm,
            arguments.rotation_deg,
            arguments.clockwise,
        )
    if arguments.command == "circle":
        return build_circle_targets(
            arguments.center_mm,
            arguments.radius_mm,
            arguments.samples,
            arguments.clockwise,
            arguments.start_angle_deg,
        )
    if arguments.command == "waypoints":
        return load_waypoint_targets(arguments.file)
    raise ValueError(f"unsupported motion command: {arguments.command}")


def default_output_directory(execute):
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    return project_root() / ("experiments" if execute else "plans") / stamp


def trajectory_points_for_report(trajectory):
    return [
        {
            "time_s": duration_seconds(point.time_from_start),
            "positions_rad": list(point.positions),
            "velocities_rad_s": list(point.velocities),
            "accelerations_rad_s2": list(point.accelerations),
        }
        for point in trajectory.points
    ]


def main(args=None):
    parsed = parse_arguments(sys.argv[1:] if args is None else args)
    if parsed.command == "plot":
        output = parsed.output
        if output is None:
            output = (parsed.input if parsed.input.is_dir() else parsed.input.parent) / "magnet_path.png"
        result = plot_trajectory(parsed.input, output)
        print(result)
        return
    if parsed.command == "wrist3-zero":
        run_wrist3_zero(parsed.execute)
        return

    if not math.isfinite(parsed.speed_mm_s) or not 0.0 <= parsed.speed_mm_s <= 20.0:
        raise SystemExit("speed must be within [0, 20] mm/s")
    if parsed.speed_mm_s == 0.0:
        if any(value is not None for value in (
            parsed.motor_rpm, parsed.motor_position_deg, parsed.motor_relative_deg
        )):
            raise SystemExit("0 mm/s holds the arm; use ze300_motor for motor-only commands")
        print("ROBOT_HOLD: 0 mm/s, no movement command sent")
        return
    if parsed.motor_rpm is not None and (
        not math.isfinite(parsed.motor_rpm)
        or not -(2 ** 31) <= round(parsed.motor_rpm * 100) < 2 ** 31
    ):
        raise SystemExit("motor rpm exceeds the ZE300 protocol range")
    for degrees in (parsed.motor_position_deg, parsed.motor_relative_deg):
        if degrees is not None:
            try:
                position_counts(degrees)
            except ValueError as error:
                raise SystemExit(str(error)) from error
    if parsed.planning_chunk_mm is not None and (
        not math.isfinite(parsed.planning_chunk_mm)
        or not MIN_PLANNING_CHUNK_M * 1000.0
        <= parsed.planning_chunk_mm
        <= MAX_PLANNING_CHUNK_M * 1000.0
    ):
        raise SystemExit(
            "planning chunk must be within "
            f"[{MIN_PLANNING_CHUNK_M * 1000.0:.0f}, "
            f"{MAX_PLANNING_CHUNK_M * 1000.0:.0f}] mm"
        )
    if (
        parsed.motor_axis_parallel is None
        and (
            parsed.motor_axis_direction != "nearest"
            or abs(parsed.motor_axis_roll_deg) > 1.0e-12
        )
    ):
        raise SystemExit(
            "motor-axis direction/roll requires --motor-axis-parallel"
        )
    if (
        not math.isfinite(parsed.motor_axis_roll_deg)
        or not -180.0 <= parsed.motor_axis_roll_deg <= 180.0
    ):
        raise SystemExit("motor-axis roll must be within [-180, 180] degrees")
    if parsed.motor_axis_parallel == "table" and (
        parsed.motor_axis_direction != "nearest"
        or parsed.axis_alignment_phase != "before"
        or parsed.motor_axis_roll_deg != 0.0
    ):
        raise SystemExit("table-parallel tool requires nearest direction, zero roll and before alignment")
    if not math.isfinite(parsed.table_yaw_deg) or not -180.0 <= parsed.table_yaw_deg <= 180.0:
        raise SystemExit("table yaw must be within [-180, 180] degrees")
    if parsed.motor_axis_parallel != "table" and parsed.table_yaw_deg != 0.0:
        raise SystemExit("table yaw requires table-parallel tool")
    if parsed.motor_axis_parallel == "tool-z-table" and (
        parsed.motor_axis_direction != "nearest"
        or parsed.axis_alignment_phase != "before"
    ):
        raise SystemExit("horizontal tool Z requires nearest direction and before alignment")
    if not math.isfinite(parsed.tool_z_heading_deg) or not -180.0 <= parsed.tool_z_heading_deg <= 180.0:
        raise SystemExit("tool Z heading must be within [-180, 180] degrees")
    if parsed.motor_axis_parallel != "tool-z-table" and parsed.tool_z_heading_deg != 0.0:
        raise SystemExit("tool Z heading requires --tool-z-parallel-table")
    try:
        targets, shape = requested_targets(parsed)
    except (OSError, ValueError, KeyError, TypeError) as error:
        raise SystemExit(str(error)) from error

    output = None
    if not parsed.no_record:
        output = (parsed.output or default_output_directory(parsed.execute)).resolve()
        try:
            output.mkdir(parents=True, exist_ok=False)
        except FileExistsError as error:
            raise SystemExit(f"refusing to overwrite existing output directory: {output}") from error

    root = project_root()
    with (root / "config" / "ur3_system.yaml").open(encoding="utf-8") as stream:
        motor_calibration = yaml.safe_load(stream)["calibration"]
    phase_sign = motor_calibration.get("motor_phase_sign")
    encoder_turns_per_magnet_turn = motor_calibration.get("motor_encoder_turns_per_magnet_turn")
    zero_angle_rad = motor_calibration.get("motor_phase_zero_rad")
    if phase_sign not in (None, -1, 1):
        raise SystemExit("calibration.motor_phase_sign must be -1, 1, or null")
    if encoder_turns_per_magnet_turn is not None and (
        not isinstance(encoder_turns_per_magnet_turn, (int, float))
        or not math.isfinite(encoder_turns_per_magnet_turn)
        or encoder_turns_per_magnet_turn <= 0
    ):
        raise SystemExit("calibration.motor_encoder_turns_per_magnet_turn must be positive or null")
    if zero_angle_rad is None or not math.isfinite(zero_angle_rad):
        raise SystemExit("calibration.motor_phase_zero_rad must be a finite number")
    report = {
        "schema": PATH_SCHEMA,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "command": parsed.command,
        "shape": shape,
        "frame": "table_world",
        "units": {"position": "mm", "time": "s", "joint": "rad"},
        "requested_speed_mm_s": float(parsed.speed_mm_s),
        "requested_motor_axis_parallel": parsed.motor_axis_parallel,
        "requested_motor_axis_direction": parsed.motor_axis_direction,
        "requested_motor_axis_roll_deg": float(parsed.motor_axis_roll_deg),
        "requested_table_yaw_deg": float(parsed.table_yaw_deg),
        "requested_tool_z_heading_deg": float(parsed.tool_z_heading_deg),
        "requested_axis_alignment_phase": parsed.axis_alignment_phase,
        "execute_requested": bool(parsed.execute),
        "requested_motor_rpm": parsed.motor_rpm,
        "requested_motor_position_deg": parsed.motor_position_deg,
        "requested_motor_relative_deg": parsed.motor_relative_deg,
        "controls_motor": bool(parsed.execute and any(value is not None for value in (
            parsed.motor_rpm, parsed.motor_position_deg, parsed.motor_relative_deg
        ))),
        "motor_command_sent": False,
        "planning_chunk_requested_mm": parsed.planning_chunk_mm,
        "motion_sent": False,
        "execution_allowed_by_file": False,
        "tool_geometry_provisional": True,
        "config_sha256": config_hashes(root),
        "global_clearance_mm": {
            key: value * 1000.0 for key, value in load_clearance_limits_m(root).items()
        },
        "limitations": [
            "application-level guard; not a certified safety function",
            "complete rigid tool envelope is provisional",
            "cables, platform posts and loose obstacles are not completely modeled",
            "PolyScope safety planes and payload settings are external to this report",
        ],
    }

    rclpy.init(args=["--ros-args", "--disable-external-lib-logs"] if parsed.no_record else None)
    node = None
    motor = None
    motor_recorder = None
    experiment_start_ns = None
    actual_path = output / "actual_magnet_path.jsonl" if output is not None else None
    camera_context = ExitStack()
    try:
        if parsed.execute and output is not None:
            camera_context.enter_context(active_experiment(output))
        node = MagnetTrajectoryNode()
        prepare_motion_stack(node, root, no_record=parsed.no_record)
        # Running-state feedback may first appear after External Control is played.
        # Planning needs fresh joints; execution feedback is checked after Play.
        node.wait_for_fresh_state(timeout=5.0)
        velocities = list(node.latest_joint_state.velocity)
        if velocities and (
            len(velocities) != len(node.latest_joint_state.position)
            or not np.isfinite(velocities).all()
        ):
            raise RuntimeError("robot reported an invalid joint velocity vector")
        if parsed.execute and not velocities:
            raise RuntimeError("joint velocity feedback is required for execution")
        if velocities and max(abs(value) for value in velocities) > 0.002:
            raise RuntimeError("robot must be stationary before planning")
        planning = node.plan_magnet_targets(
            targets,
            parsed.speed_mm_s / 1000.0,
            planning_chunk_m=(
                None
                if parsed.planning_chunk_mm is None
                else parsed.planning_chunk_mm / 1000.0
            ),
            motor_axis_parallel=parsed.motor_axis_parallel,
            motor_axis_direction=parsed.motor_axis_direction,
            motor_axis_roll_deg=parsed.motor_axis_roll_deg,
            table_yaw_deg=parsed.table_yaw_deg,
            tool_z_heading_deg=parsed.tool_z_heading_deg,
            axis_alignment_phase=parsed.axis_alignment_phase,
        )
        targets = [np.asarray(point) for point in planning["targets_world_m"]]
        trajectory = planning.pop("solution")
        planning.pop("start_state")
        planned_trace = planning.pop("planned_trace")
        report["execution_allowed_by_file"] = (
            node.ceiling_guard.tool_data.get("execution_allowed") is True
        )
        report["tool_geometry_provisional"] = bool(
            node.ceiling_guard.provisional
        )
        report.update(
            planning=planning,
            joint_names=list(trajectory.joint_trajectory.joint_names),
            trajectory_points=trajectory_points_for_report(
                trajectory.joint_trajectory
            ),
        )
        # Keep the trace at top level so the plotting command can consume a
        # plan without knowing how the rest of the report is organized.
        report["planned_trace"] = planned_trace
        node.wait_for_fresh_state(timeout=2.0)
        report["start_mismatch_rad"] = node.verify_start(trajectory)
        if output is not None:
            write_json_exclusive(output / "plan.json", report)
            plot_trajectory(output / "plan.json", output / "magnet_path.png")
        print(
            "PLAN_OK "
            f"shape={parsed.command} fraction={planning['fraction']:.6f} "
            f"length={planning['total_length_m'] * 1000.0:.3f}mm "
            f"duration={planning['duration_s']:.3f}s "
            f"samples={len(planned_trace)}"
        )
        print(
            "START_MAGNET_WORLD_MM="
            + ",".join(f"{value * 1000.0:.3f}" for value in planning["start_magnet_world_m"])
        )
        print(
            "FINAL_TARGET_WORLD_MM="
            + ",".join(f"{value * 1000.0:.3f}" for value in planning["targets_world_m"][-1])
        )
        if planning["motor_axis_alignment"] is not None:
            alignment = planning["motor_axis_alignment"]
            print(
                ("FINAL_TOOL_Z_WORLD=" if alignment["world_axis"] == "tool-z-table"
                 else "FINAL_MOTOR_AXIS_WORLD=")
                + ",".join(
                    f"{value:.6f}"
                    for value in alignment["planned_final_axis_world"]
                )
            )
            axis_label = (
                "tool0_z_parallel_table" if alignment["world_axis"] == "tool-z-table"
                else "tool0_plane_parallel_table" if alignment["world_axis"] == "table"
                else f"table_world_{alignment['selected_direction']}"
                     f"{alignment['world_axis'].upper()}"
            )
            yaw_label = (
                f"yaw={alignment['table_yaw_deg']:.3f}deg "
                if alignment["world_axis"] == "table" else ""
            )
            heading_label = (
                f"heading={alignment['tool_z_heading_deg']:.3f}deg "
                if alignment["world_axis"] == "tool-z-table" else ""
            )
            print(
                f"AXIS_ALIGNMENT={axis_label} "
                f"rotation={alignment['minimum_rotation_deg']:.3f}deg "
                f"roll={alignment['roll_about_target_axis_deg']:.3f}deg "
                f"{yaw_label}"
                f"{heading_label}"
                f"phase={alignment['phase']} "
                f"error={alignment['planned_final_axis_error_deg']:.4f}deg"
            )
        if not parsed.execute:
            print("PLAN_ONLY: no command sent to the robot")
            if not parsed.no_record:
                print(output / "plan.json")
                print(output / "magnet_path.png")
            return

        if config_hashes(root) != report["config_sha256"]:
            raise RuntimeError("configuration files changed after planning")
        ensure_external_control(node)
        node.wait_for_fresh_state(timeout=2.0, require_execution_state=True)
        node.verify_start(trajectory)
        live_velocities = list(node.latest_joint_state.velocity)
        if (
            not live_velocities
            or not np.isfinite(live_velocities).all()
            or max(abs(value) for value in live_velocities) > 0.002
        ):
            raise RuntimeError("robot must be stationary immediately before execution")
        experiment_start_ns = time.time_ns()
        if report["controls_motor"]:
            motor = ZE300Motor(parsed.motor_port, parsed.motor_address, parsed.motor_baud)
            report["motor_status_before"] = motor.status()
            report["motor_command_attempted"] = True
            if parsed.motor_rpm is not None:
                report["motor_start_reply"] = motor.speed(parsed.motor_rpm)
                motor_command_text = f"speed={parsed.motor_rpm:.2f}rpm"
            elif parsed.motor_position_deg is not None:
                report["motor_start_reply"] = motor.absolute(parsed.motor_position_deg)
                motor_command_text = f"absolute={parsed.motor_position_deg:.2f}deg"
            else:
                report["motor_start_reply"] = motor.relative(parsed.motor_relative_deg)
                motor_command_text = f"relative={parsed.motor_relative_deg:.2f}deg"
            report["motor_command_sent"] = True
            print(f"MOTOR_COMMAND_OK {motor_command_text}")
        elif not parsed.no_record:
            try:
                motor = ZE300Motor(parsed.motor_port, parsed.motor_address, parsed.motor_baud)
                report["motor_start_reply"] = motor.status()
            except OSError as motor_error:
                report["motor_recording_error"] = str(motor_error)
                if motor is not None:
                    motor.close()
                    motor = None
        if motor is not None and output is not None:
            motor_recorder = MotorRecorder(
                motor, output / "motor_samples.jsonl", report["motor_start_reply"]
            )
        node.start_actual_recording(actual_path)
        report["execution_attempted"] = True
        try:
            node.execute(trajectory)
            node.wait_for_fresh_state(timeout=2.0, require_execution_state=True)
            node.monitor_execution_state(node.current_robot_state())
        finally:
            report["motion_sent"] = bool(node.execution_goal_sent)
            report["dashboard_stop_attempted"] = bool(
                node.dashboard_stop_attempted
            )
            report["dashboard_stop_succeeded"] = bool(
                node.dashboard_stop_succeeded
            )
            report["dashboard_stop_message"] = node.dashboard_stop_message
            node.stop_actual_recording()
            if motor_recorder is not None:
                motor_recorder.stop()
        if report["controls_motor"]:
            # The recorder must release the serial port before sending commands.
            motor.speed(0)
        if not node.actual_samples:
            raise RuntimeError("execution completed without measured trajectory samples")
        final_measured = np.asarray(
            node.actual_samples[-1]["magnet_world_mm"], dtype=float
        ) / 1000.0
        final_error = float(np.linalg.norm(final_measured - targets[-1]))
        if final_error > FINAL_POSITION_TOLERANCE_M:
            raise RuntimeError(
                f"measured final magnet error is {final_error * 1000.0:.3f} mm"
            )
        final_measured_axis = normalized_vector(
            node.actual_samples[-1]["motor_axis_world"],
            "measured final motor axis",
        )
        final_axis_error = None
        constrained_axis = (
            normalized_vector(node.actual_samples[-1]["tool_normal_world"], "measured tool Z")
            if node.target_tool_z_world is not None else final_measured_axis
        )
        target_axis = (node.target_tool_z_world if node.target_tool_z_world is not None
                       else node.target_motor_axis_world)
        if target_axis is not None:
            final_axis_error = math.acos(float(np.clip(
                np.dot(constrained_axis, target_axis),
                -1.0,
                1.0,
            )))
            if final_axis_error > FINAL_AXIS_TOLERANCE_RAD:
                raise RuntimeError(
                    "measured final axis error is "
                    f"{math.degrees(final_axis_error):.3f} deg"
                )
        execution = {
            "completed": True,
            "start_host_time_ns": experiment_start_ns,
            "end_host_time_ns": time.time_ns(),
            "motion_sent": bool(node.execution_goal_sent),
            "dashboard_stop_attempted": bool(node.dashboard_stop_attempted),
            "dashboard_stop_succeeded": bool(node.dashboard_stop_succeeded),
            "dashboard_stop_message": node.dashboard_stop_message,
            "sample_count": len(node.actual_samples),
            "recording_error": node.recording_error,
            "motor_recording_error": (
                motor_recorder.error if motor_recorder is not None
                else report.get("motor_recording_error")
            ),
            "magnet_orientation_calibrated": (
                phase_sign is not None and encoder_turns_per_magnet_turn is not None
            ),
            "motor_phase_sign": phase_sign,
            "motor_encoder_turns_per_magnet_turn": encoder_turns_per_magnet_turn,
            "magnet_north_angle_at_encoder_zero_rad": zero_angle_rad,
            "final_magnet_world_mm": (final_measured * 1000.0).tolist(),
            "final_error_mm": final_error * 1000.0,
            "final_motor_axis_world": final_measured_axis.tolist(),
            "final_tool_z_world": node.actual_samples[-1]["tool_normal_world"],
            "final_motor_axis_error_deg": (
                None
                if final_axis_error is None or node.target_tool_z_world is not None
                else math.degrees(final_axis_error)
            ),
            "final_tool_z_error_deg": (
                math.degrees(final_axis_error)
                if node.target_tool_z_world is not None else None
            ),
        }
        if motor is not None:
            execution["motor_status_before"] = report.get("motor_status_before")
            execution["motor_target_rpm"] = parsed.motor_rpm
            execution["motor_target_position_deg"] = parsed.motor_position_deg
            execution["motor_target_relative_deg"] = parsed.motor_relative_deg
            if report["controls_motor"]:
                print("MOTOR_RETURNING_TO_ORIGIN: waiting for stop, then shortest rotation", flush=True)
                execution["motor_return_to_origin"] = restore_origin(motor)
                print("MOTOR_HOME_OK " + json.dumps(execution["motor_return_to_origin"], ensure_ascii=False))
            try:
                execution["motor_status"] = motor.status()
            except OSError as status_error:
                execution["motor_status_error"] = str(status_error)
            if report["controls_motor"]:
                execution["motor_output_left_enabled"] = bool(execution["motor_return_to_origin"]["enabled"])
        if output is not None:
            try:
                execution["magnet_trajectory_samples_annotated"] = add_magnet_pose(
                    actual_path,
                    motor_recorder.samples if motor_recorder is not None else [],
                    phase_sign,
                    encoder_turns_per_magnet_turn,
                    zero_angle_rad,
                )
            except (OSError, ValueError, KeyError) as pose_error:
                execution["magnet_pose_error"] = str(pose_error)
            write_json_exclusive(output / "execution.json", execution)
            plot_trajectory(
                output / "plan.json" if node.recording_error else output,
                output / "magnet_path.png",
            )
        print("EXECUTION_SUCCESS " + json.dumps(execution, ensure_ascii=False))
        if parsed.no_record:
            print("NO_RECORD: no data files written")
        else:
            print(actual_path)
            print(output / "magnet_path.png")
    except BaseException as error:
        if motor_recorder is not None:
            motor_recorder.stop()
        if motor is not None and report.get("motor_command_attempted"):
            try:
                motor.speed(0)
                report["motor_stopped_after_failure"] = True
            except (OSError, ValueError) as stop_error:
                report["motor_stop_error"] = str(stop_error)
        if node is not None:
            node.stop_actual_recording()
            report["motion_sent"] = bool(node.execution_goal_sent)
        if actual_path is not None and actual_path.exists():
            try:
                add_magnet_pose(
                    actual_path,
                    motor_recorder.samples if motor_recorder is not None else [],
                    phase_sign,
                    encoder_turns_per_magnet_turn,
                    zero_angle_rad,
                )
            except (OSError, ValueError, KeyError) as pose_error:
                report["magnet_pose_error"] = str(pose_error)
        failure = {
            "failed_utc": datetime.now(timezone.utc).isoformat(),
            "motion_sent": bool(report.get("motion_sent")),
            "error": str(error),
            "start_host_time_ns": experiment_start_ns,
            "end_host_time_ns": time.time_ns(),
            "motor_command_sent": report["motor_command_sent"],
            "motor_stopped_after_failure": report.get("motor_stopped_after_failure"),
            "motor_stop_error": report.get("motor_stop_error"),
            "motor_recording_error": (
                motor_recorder.error if motor_recorder is not None
                else report.get("motor_recording_error")
            ),
            "magnet_pose_error": report.get("magnet_pose_error"),
        }
        if node is not None:
            failure.update(
                dashboard_stop_attempted=bool(node.dashboard_stop_attempted),
                dashboard_stop_succeeded=bool(node.dashboard_stop_succeeded),
                dashboard_stop_message=node.dashboard_stop_message,
            )
        if output is not None:
            failure_path = output / "failure.json"
            if not failure_path.exists():
                write_json_exclusive(failure_path, failure)
        if actual_path is not None and actual_path.exists() and actual_path.stat().st_size:
            try:
                plot_trajectory(output, output / "magnet_path.png")
            except Exception:
                pass
        if node is not None:
            node.get_logger().error(str(error))
        else:
            print(str(error), file=sys.stderr)
        raise SystemExit(1) from error
    finally:
        camera_context.close()
        if motor is not None:
            motor.close()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
