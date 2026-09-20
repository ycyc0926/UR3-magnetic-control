#!/usr/bin/env python3
"""Solve FLIR intrinsics and fixed-camera/UR-base extrinsics from samples.

The script prints a YAML-ready result.  It deliberately does not overwrite a
configuration file so a calibration can be inspected before it is accepted.
"""

import argparse

import cv2
import numpy as np
import yaml
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


def matrix4(rotation, translation):
    result = np.eye(4)
    result[:3, :3] = rotation
    result[:3, 3] = translation
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--samples", default="/home/yc/UR3/camera/calibration_samples/samples.yaml"
    )
    parser.add_argument(
        "--world", default="/home/yc/UR3/config/table_world_calibration.yaml"
    )
    arguments = parser.parse_args()
    with open(arguments.samples, encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    with open(arguments.world, encoding="utf-8") as stream:
        world = yaml.safe_load(stream)

    samples = data["samples"]
    width, height = data["camera"]["image_size_px"]
    edge = float(data["marker"]["outer_edge_m"])
    object_points = np.asarray(
        [
            [-edge / 2.0, edge / 2.0, 0.0],
            [edge / 2.0, edge / 2.0, 0.0],
            [edge / 2.0, -edge / 2.0, 0.0],
            [-edge / 2.0, -edge / 2.0, 0.0],
        ],
        dtype=np.float64,
    )
    image_points = [
        np.asarray(sample["marker_corners_px"], dtype=np.float64)
        for sample in samples
    ]
    flags = (
        cv2.CALIB_ZERO_TANGENT_DIST
        | cv2.CALIB_FIX_K3
        | cv2.CALIB_FIX_K4
        | cv2.CALIB_FIX_K5
        | cv2.CALIB_FIX_K6
    )
    intrinsic_rms, camera_matrix, distortion, _, _ = cv2.calibrateCamera(
        [object_points.astype(np.float32) for _ in samples],
        [points.astype(np.float32) for points in image_points],
        (width, height),
        None,
        None,
        flags=flags,
    )

    rotations_base_from_tcp = [
        Rotation.from_rotvec(sample["tcp_pose_base_m_rad"][3:]).as_matrix()
        for sample in samples
    ]
    translations_base_from_tcp = [
        np.asarray(sample["tcp_pose_base_m_rad"][:3], dtype=np.float64)
        for sample in samples
    ]

    # Stable initialization measured from this physical camera arrangement.
    rotation_camera_from_base_initial = np.asarray(
        [
            [-0.9997, -0.0215, -0.0079],
            [-0.0215, 0.99976, -0.0033],
            [0.00797, -0.00314, -0.99996],
        ]
    )
    rotation_camera_from_base_initial = Rotation.from_matrix(
        rotation_camera_from_base_initial
    ).as_matrix()
    parameters_initial = np.r_[
        Rotation.from_matrix(rotation_camera_from_base_initial).as_rotvec(),
        [0.085, 0.415, 0.833],
        np.radians([0.86, -0.37, 91.14]),
        [0.0008, 0.0, 0.00085],
    ]

    def reprojection_residuals(parameters):
        rotation_camera_from_base = Rotation.from_rotvec(
            parameters[:3]
        ).as_matrix()
        translation_camera_from_base = parameters[3:6]
        rotation_tcp_from_marker = Rotation.from_euler(
            "xyz", parameters[6:9]
        ).as_matrix()
        translation_tcp_from_marker = parameters[9:12]
        residuals = []
        for index, observed in enumerate(image_points):
            points_tcp = (
                rotation_tcp_from_marker @ object_points.T
            ).T + translation_tcp_from_marker
            points_base = (
                rotations_base_from_tcp[index] @ points_tcp.T
            ).T + translations_base_from_tcp[index]
            points_camera = (
                rotation_camera_from_base @ points_base.T
            ).T + translation_camera_from_base
            projected, _ = cv2.projectPoints(
                points_camera,
                np.zeros(3),
                np.zeros(3),
                camera_matrix,
                distortion,
            )
            residuals.extend((projected.reshape(-1, 2) - observed).ravel())
        return np.asarray(residuals)

    lower = np.r_[
        [-10.0] * 3,
        [-5.0] * 3,
        np.radians([-10.0, -10.0, -180.0]),
        [-0.02] * 3,
    ]
    upper = np.r_[
        [10.0] * 3,
        [5.0] * 3,
        np.radians([10.0, 10.0, 180.0]),
        [0.02] * 3,
    ]
    solution = least_squares(
        reprojection_residuals,
        parameters_initial,
        bounds=(lower, upper),
        loss="soft_l1",
        f_scale=1.0,
        max_nfev=5000,
    )
    if not solution.success:
        raise RuntimeError(solution.message)

    rotation_camera_from_base = Rotation.from_rotvec(solution.x[:3]).as_matrix()
    translation_camera_from_base = solution.x[3:6]
    transform_camera_from_base = matrix4(
        rotation_camera_from_base, translation_camera_from_base
    )
    transform_base_from_camera = np.linalg.inv(transform_camera_from_base)
    transform_tcp_from_marker = matrix4(
        Rotation.from_euler("xyz", solution.x[6:9]).as_matrix(), solution.x[9:12]
    )
    transform_base_from_world = np.asarray(world["T_base_from_world"], dtype=float)
    transform_camera_from_world = (
        transform_camera_from_base @ transform_base_from_world
    )
    transform_world_from_camera = np.linalg.inv(transform_camera_from_world)

    errors = reprojection_residuals(solution.x).reshape(len(samples), 4, 2)
    pixel_norms = np.linalg.norm(errors, axis=2)
    acrylic_z = float(world["fixed_work_surface"]["acrylic_top_world_z_m"])
    plane_matrix = np.column_stack(
        [
            transform_camera_from_world[:3, 0],
            transform_camera_from_world[:3, 1],
            transform_camera_from_world[:3, 2] * acrylic_z
            + transform_camera_from_world[:3, 3],
        ]
    )
    homography = camera_matrix @ plane_matrix
    homography /= homography[2, 2]
    inverse_homography = np.linalg.inv(homography)
    inverse_homography /= inverse_homography[2, 2]

    output = {
        "camera_matrix": camera_matrix.tolist(),
        "distortion_coefficients": distortion.ravel().tolist(),
        "T_camera_from_base": transform_camera_from_base.tolist(),
        "T_base_from_camera": transform_base_from_camera.tolist(),
        "T_camera_from_world": transform_camera_from_world.tolist(),
        "T_world_from_camera": transform_world_from_camera.tolist(),
        "T_tcp_from_marker": transform_tcp_from_marker.tolist(),
        "acrylic_top_world_z_m": acrylic_z,
        "H_undistorted_pixel_from_world_xy_at_acrylic": homography.tolist(),
        "H_world_xy_from_undistorted_pixel_at_acrylic": inverse_homography.tolist(),
        "quality": {
            "sample_count": len(samples),
            "intrinsic_rms_px": float(intrinsic_rms),
            "robot_camera_rms_px": float(np.sqrt(np.mean(pixel_norms**2))),
            "robot_camera_max_px": float(np.max(pixel_norms)),
        },
    }
    print(yaml.safe_dump(output, sort_keys=False))


if __name__ == "__main__":
    main()
