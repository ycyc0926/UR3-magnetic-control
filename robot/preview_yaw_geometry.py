#!/usr/bin/env python3
"""Read-only geometric yaw diagnostic, never an executable robot trajectory.

Only RTDE output fields are negotiated. No motion/IO inputs, ROS publishers,
driver processes or execution API exist here. A successful diagnostic does NOT
validate self-collision, unknown bracket/rod geometry, supports, or dynamics.
"""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import socket

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory

from ur3_magnetic_control.ceiling_geometry import CeilingGeometry
from ur3_magnetic_control.clearance_policy import clearance_limits_mm
import ur3_realtime_monitor as rtde


ROOT = Path(__file__).resolve().parents[1]
JOINTS = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
          'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']


def box_corners(lower, upper):
    low, high = np.asarray(lower, float), np.asarray(upper, float)
    if low.shape != (3,) or high.shape != (3,) or not np.isfinite([low, high]).all() or np.any(high <= low):
        raise ValueError('Invalid box bounds')
    return np.asarray([[x, y, z] for x in [low[0], high[0]]
                       for y in [low[1], high[1]] for z in [low[2], high[2]]])


def fixed_sphere_target(start, offset, yaw):
    target = start.copy()
    sphere = start[:3, 3] + start[:3, :3] @ offset
    target[:3, :3] = Rotation.from_rotvec([0., 0., yaw]).as_matrix() @ start[:3, :3]
    target[:3, 3] = sphere - target[:3, :3] @ offset
    return target


def read_state():
    with socket.create_connection((rtde.ROBOT_IP, 30004), timeout=3) as sock:
        recipe, version = rtde.negotiate(sock, 10)
        state = rtde.parse_state(rtde.receive_command(sock, rtde.RTDE_DATA_PACKAGE), recipe)
        rtde.send_packet(sock, rtde.RTDE_CONTROL_PACKAGE_PAUSE)  # This output stream only.
    if max(abs(v) for v in state['actual_qd']) > .002 or np.linalg.norm(state['actual_TCP_speed']) > .002:
        raise RuntimeError('Robot not stationary; diagnostic stopped')
    return state, version


def run():
    limits_mm = clearance_limits_mm(ROOT)
    table = yaml.safe_load((ROOT/'config/table_world_calibration.yaml').read_text())
    geometry = yaml.safe_load((ROOT/'config/magnet_tool_geometry_draft.yaml').read_text())
    state, version = read_state()
    calibration_path = ROOT/'config/ur3_calibration.yaml'
    calibration = yaml.safe_load(calibration_path.read_text())
    if calibration['kinematics']['hash'] != 'calib_18089309548208516197':
        raise RuntimeError('Unexpected measured robot calibration')
    xml = xacro.process_file(str(Path(get_package_share_directory('ur_description'))/'urdf/ur.urdf.xacro'),
        mappings={'name': 'ur', 'ur_type': 'ur3', 'use_fake_hardware': 'true',
                  'kinematics_params': str(calibration_path)}).toxml()
    # Fake-hardware xacro omits the hardware calibration hash from the XML.
    # Validate the explicitly selected YAML above, then independently check FK.
    model = CeilingGeometry(xml, table['T_base_from_world'])
    offset = np.asarray(geometry['previously_confirmed']['magnet_center_xyz_m'])
    radius = geometry['previously_confirmed']['magnet_radius_m']
    motor = geometry['conditional_motor_body']
    vertices = box_corners(motor['min_xyz_m'], motor['max_xyz_m'])
    envelope = geometry['provisional_whole_tool_envelope']
    envelope_vertices = box_corners(envelope['min_xyz_m'], envelope['max_xyz_m'])
    bottom = table['fixed_work_surface']['acrylic_bottom_world_z_m']
    q0 = np.asarray(state['actual_q'])

    def fk(q):
        return model.world_from_base @ model.transforms(dict(zip(JOINTS, q)))['tool0']

    start = fk(q0)
    sphere = start[:3, 3] + start[:3, :3] @ offset
    flange_base = model.transforms(dict(zip(JOINTS, q0)))['tool0']
    p_error = np.linalg.norm(flange_base[:3, 3]+flange_base[:3, :3]@offset-state['actual_TCP_pose'][:3])
    r_error = (Rotation.from_matrix(flange_base[:3, :3]).inv()
               * Rotation.from_rotvec(state['actual_TCP_pose'][3:])).magnitude()
    if p_error > .00025 or r_error > np.deg2rad(.05):
        raise RuntimeError('FK/controller mismatch; do not use this model')

    def heights(q):
        result = model.heights(dict(zip(JOINTS, q)), offset, radius)
        pose = fk(q)
        result['conditional_motor_body'] = float(np.max(vertices @ pose[2, :3]) + pose[2, 3])
        result['provisional_whole_tool_envelope'] = float(np.max(envelope_vertices @ pose[2, :3]) + pose[2, 3])
        return result

    initial = heights(q0)
    report = {'utc': datetime.now(timezone.utc).isoformat(), 'mode': 'read_only_geometric_diagnostic',
        'execution_allowed': False, 'is_executable_trajectory': False, 'controller_version': version,
        'snapshot': state, 'sphere_world_mm': (sphere*1000).tolist(),
        'shaft_world_unit': (start[:3, :3] @ np.array([0., -1., 0.])).tolist(),
        'fk_position_error_mm': float(p_error*1000), 'fk_rotation_error_deg': float(np.rad2deg(r_error)),
        'ceiling_world_mm': bottom*1000, 'clearance_limits_mm': limits_mm,
        'initial_top_world_mm': {k: v*1000 for k, v in initial.items()},
        'geometry': geometry, 'unvalidated': ['self_collision', 'full_tool_geometry', 'platform_support_collision',
            'controller_spline_interpolation', 'joint_speeds_accelerations_and_torques', 'onsite_clearance'],
        'cases': []}

    # Continuation on the CURRENT joint branch only. Failure is not proof that
    # every possible IK branch is unreachable. Never switch branches abruptly.
    for sign in [1, -1]:
        q = q0.copy()
        case = {'requested_world_yaw_deg': sign*90, 'completed': False, 'sample_count': 1,
                'max_joint_step_deg': 0., 'max_linear_interpolation_sphere_drift_mm': 0.,
                'joint_names': JOINTS, 'minimum_gap_mm': (bottom-max(initial.values()))*1000,
                'limiting_link': max(initial, key=initial.get), 'first_height_violation': None,
                'interpolation_checked': 'linear only; five substeps per 0.5-degree yaw segment'}
        achieved = 0.
        for angle in np.arange(.5, 90.01, .5)*sign:
            desired = fixed_sphere_target(start, offset, np.deg2rad(angle))
            def residual(values):
                pose = fk(values)
                return np.r_[pose[:3, 3]-desired[:3, 3],
                    .1*Rotation.from_matrix(desired[:3, :3].T@pose[:3, :3]).as_rotvec()]
            solution = least_squares(residual, q,
                bounds=([-2*np.pi, -2*np.pi, -np.pi, -2*np.pi, -2*np.pi, -2*np.pi],
                        [2*np.pi, 2*np.pi, np.pi, 2*np.pi, 2*np.pi, 2*np.pi]),
                xtol=1e-11, ftol=1e-11, gtol=1e-11, max_nfev=150)
            error = residual(solution.x)
            if np.linalg.norm(error[:3]) > .000001 or np.linalg.norm(error[3:])/.1 > .00001:
                case.update(stopped_at_yaw_deg=float(angle), reason='numerical_IK_tolerance_not_met')
                break
            step = float(np.max(np.abs(np.rad2deg(solution.x-q))))
            if step > 5.:
                case.update(stopped_at_yaw_deg=float(angle), reason='large_joint_change_near_branch_singularity',
                            rejected_joint_step_deg=step)
                break
            case['max_joint_step_deg'] = max(case['max_joint_step_deg'], step)
            for fraction in np.linspace(0., 1., 6)[1:]:
                sample = q+(solution.x-q)*fraction
                tops = heights(sample)
                link = max(tops, key=tops.get)
                gap = (bottom-tops[link])*1000
                if gap < case['minimum_gap_mm']:
                    case.update(minimum_gap_mm=gap, limiting_link=link)
                if gap < limits_mm['ceiling'] and case['first_height_violation'] is None:
                    case['first_height_violation'] = {'near_yaw_deg': float(angle), 'gap_mm': gap, 'link': link}
                pose = fk(sample)
                drift = np.linalg.norm(pose[:3, 3]+pose[:3, :3]@offset-sphere)*1000
                case['max_linear_interpolation_sphere_drift_mm'] = max(case['max_linear_interpolation_sphere_drift_mm'], float(drift))
                case['sample_count'] += 1
            q = solution.x
            achieved = float(angle)
        else:
            case.update(completed=True, reason='IK_and_partial_geometry_height_scan_complete_NOT_execution_approval')
        case['achieved_yaw_deg'] = achieved
        case['joint_change_deg'] = np.rad2deg(q-q0).tolist()
        report['cases'].append(case)
        print(json.dumps(case, ensure_ascii=False), flush=True)
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', type=Path, required=True, help='NEW diagnostic JSON, never a controller input')
    args = parser.parse_args()
    if args.report.exists():raise SystemExit('Refusing to overwrite an existing diagnostic')
    result = run()
    args.report.parent.mkdir(parents=True, exist_ok=True)
    with args.report.open('x', encoding='utf-8') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2)
    print(args.report)
