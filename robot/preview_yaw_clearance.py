#!/usr/bin/env python3
"""Read-only +Y shaft yaw audit. NO motion/IO/execution interface exists.

Combines calibrated FK/IK, surface bounds and in-memory MoveIt/FCL checks.
The result remains non-executable even when every sampled state passes.
"""
import argparse
import csv
from datetime import datetime, timezone
import io
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory

from preview_yaw_geometry import ROOT, JOINTS, box_corners, fixed_sphere_target, read_state
from ur3_magnetic_control.ceiling_geometry import CeilingGeometry
from ur3_magnetic_control.clearance_policy import clearance_limits_mm


def surface_bounds(pose, kind, shape):
    if kind in ('mesh', 'box'):
        points = shape @ pose[:3, :3].T + pose[:3, 3]
        return points.min(axis=0), points.max(axis=0)
    if kind == 'sphere':
        half = np.full(3, shape)
    elif kind == 'cylinder':
        radius, length = shape
        half = radius*np.linalg.norm(pose[:3, :2], axis=1) + np.abs(pose[:3, 2])*length/2
    else:
        raise ValueError('Unsupported surface')
    return pose[:3, 3]-half, pose[:3, 3]+half


def yaw_to_positive_y(axis):
    axis = np.asarray(axis, float)
    if axis.shape != (3,) or not np.isfinite(axis).all() or np.linalg.norm(axis[:2]) < 1e-6:
        raise ValueError('Shaft has no valid horizontal direction')
    angle = (90.-np.degrees(np.arctan2(axis[1], axis[0]))+180.) % 360.-180.
    if not 0. < angle < 120.:
        raise ValueError('Requested CCW diagnostic only supports 0 to 120 degrees')
    return float(angle)


def audit(output, snapshot=None):
    binary = ROOT/'robot/offline_collision/build/check_self_collision'
    if not binary.is_file():
        raise RuntimeError('Build the offline collision checker before running this audit')
    # Exclusive directory; all generated files remain offline diagnostic data.
    output.mkdir(parents=True, exist_ok=False)
    limits_mm = clearance_limits_mm(ROOT)
    table = yaml.safe_load((ROOT/'config/table_world_calibration.yaml').read_text())
    side = yaml.safe_load((ROOT/'config/acrylic_side_boundaries.yaml').read_text())
    geometry = yaml.safe_load((ROOT/'config/magnet_tool_geometry_draft.yaml').read_text())
    if (side['frame'] != 'table_world' or side['user_confirmed']['photo_right'] != '+X'
            or not side['user_confirmed']['distances_are_to_inner_faces']):
        raise RuntimeError('Unconfirmed side panel convention')
    left, right = side['left_inner_x_m'], side['right_inner_x_m']
    if not np.isfinite([left, right]).all() or not left < right:
        raise RuntimeError('Invalid panel coordinates')
    if snapshot:
        prior = json.loads(snapshot.read_text())
        state, version = prior['snapshot'], prior['controller_version']
    else:
        state, version = read_state()
    calibration = ROOT/'config/ur3_calibration.yaml'
    if yaml.safe_load(calibration.read_text())['kinematics']['hash'] != 'calib_18089309548208516197':
        raise RuntimeError('Unexpected robot calibration')
    xml = xacro.process_file(str(Path(get_package_share_directory('ur_description'))/'urdf/ur.urdf.xacro'),
        mappings={'name': 'ur', 'ur_type': 'ur3', 'use_fake_hardware': 'true',
                  'kinematics_params': str(calibration)}).toxml()
    srdf = xacro.process_file(str(Path(get_package_share_directory('ur_moveit_config'))/'srdf/ur.srdf.xacro'),
        mappings={'name': 'ur', 'prefix': ''}).toxml()
    urdf_path, srdf_path = output/'diagnostic.urdf', output/'diagnostic.srdf'
    urdf_path.write_text(xml)
    srdf_path.write_text(srdf)
    model = CeilingGeometry(xml, table['T_base_from_world'])
    q0 = np.asarray(state['actual_q'])
    offset = np.asarray(geometry['previously_confirmed']['magnet_center_xyz_m'])
    radius = geometry['previously_confirmed']['magnet_radius_m']
    envelope = geometry['provisional_whole_tool_envelope']
    corners = box_corners(envelope['min_xyz_m'], envelope['max_xyz_m'])
    bottom = table['fixed_work_surface']['acrylic_bottom_world_z_m']

    def fk(q):
        return model.world_from_base @ model.transforms(dict(zip(JOINTS, q)))['tool0']

    start = fk(q0)
    axis = start[:3, :3] @ np.array([0., -1., 0.])
    angle = yaw_to_positive_y(axis)
    sphere = start[:3, 3] + start[:3, :3]@offset
    flange_base = model.transforms(dict(zip(JOINTS, q0)))['tool0']
    position_error = np.linalg.norm(flange_base[:3, 3]+flange_base[:3, :3]@offset-state['actual_TCP_pose'][:3])
    rotation_error = (Rotation.from_matrix(flange_base[:3, :3]).inv()
                      * Rotation.from_rotvec(state['actual_TCP_pose'][3:])).magnitude()
    if position_error > .00025 or rotation_error > np.deg2rad(.05):
        raise RuntimeError('FK/controller mismatch')
    data = {'mode': 'offline_clearance_audit', 'execution_allowed': False, 'is_executable_trajectory': False,
        'utc': datetime.now(timezone.utc).isoformat(), 'controller_version': version, 'snapshot': state,
        'snapshot_source': str(snapshot) if snapshot else 'fresh_RTDE_outputs_only',
        'side_boundaries': side, 'tool_geometry': geometry, 'joint_names': JOINTS,
        'requested_yaw_deg': angle, 'sphere_world_mm': (sphere*1000).tolist(),
        'fk_position_error_mm': float(position_error*1000), 'fk_rotation_error_deg': float(np.degrees(rotation_error)),
        'srdf_disabled_pairs': [dict(item.attrib) for item in ET.fromstring(srdf).findall('disable_collisions')],
        'tool_touch_links': ['tool0', 'flange', 'wrist_3_link'],
        'interpolation': 'five linear-joint substeps per <=0.5 degree yaw; not controller time interpolation',
        'unvalidated': ['actual_full_tool_envelope', 'aluminium_posts_and_loose_objects', 'cables',
                        'controller_time_interpolation', 'dynamic_limits_and_stopping',
                        'calibration_and_physical_clearance_uncertainty'],
        'clearance_limits_mm': limits_mm,
        'samples': [], 'minimum_gaps': {}, 'max_sphere_drift_mm': 0., 'max_joint_step_deg': 0.}
    minima = {name: {'gap_mm': float('inf')} for name in ['left', 'right', 'ceiling', 'table']}
    expected_fk = []

    def add(q, yaw):
        transforms = model.transforms(dict(zip(JOINTS, q)))
        shapes = []
        for name, origin, kind, shape in model.surfaces:
            pose = model.world_from_base @ transforms[name] @ origin
            low, high = surface_bounds(pose, kind, shape)
            shapes.append((name, low, high))
        pose = model.world_from_base @ transforms['tool0']
        low, high = surface_bounds(pose, 'box', corners)
        shapes.append(('unverified_tool_envelope', low, high))
        current_sphere = pose[:3, 3]+pose[:3, :3]@offset
        shapes.append(('magnet_sphere', current_sphere-radius, current_sphere+radius))
        for name, low, high in shapes:
            values = {'left': low[0]-left, 'right': right-high[0], 'ceiling': bottom-high[2], 'table': low[2]}
            for boundary, gap in values.items():
                if gap*1000 < minima[boundary]['gap_mm']:
                    minima[boundary] = {'gap_mm': float(gap*1000), 'link': name, 'near_yaw_deg': float(yaw)}
        data['max_sphere_drift_mm'] = max(data['max_sphere_drift_mm'], float(np.linalg.norm(current_sphere-sphere)*1000))
        data['samples'].append({'yaw_deg': float(yaw), 'q_rad': q.tolist()})
        expected_fk.append(transforms['tool0'])

    add(q0, 0.)
    q, achieved = q0.copy(), 0.
    for yaw in np.r_[np.arange(.5, angle, .5), angle]:
        desired = fixed_sphere_target(start, offset, np.deg2rad(yaw))
        def residual(values):
            pose = fk(values)
            return np.r_[pose[:3, 3]-desired[:3, 3],
                         .1*Rotation.from_matrix(desired[:3, :3].T@pose[:3, :3]).as_rotvec()]
        solution = least_squares(residual, q,
            bounds=([-2*np.pi, -2*np.pi, -np.pi, -2*np.pi, -2*np.pi, -2*np.pi],
                    [2*np.pi, 2*np.pi, np.pi, 2*np.pi, 2*np.pi, 2*np.pi]),
            xtol=1e-11, ftol=1e-11, gtol=1e-11, max_nfev=150)
        error = residual(solution.x)
        step = float(np.max(np.abs(np.degrees(solution.x-q))))
        if np.linalg.norm(error[:3]) > 1e-6 or np.linalg.norm(error[3:])/.1 > 1e-5 or step > 5.:
            data['rejection'] = {'yaw_deg': float(yaw), 'reason': 'IK_error_or_joint_jump', 'step_deg': step}
            break
        data['max_joint_step_deg'] = max(data['max_joint_step_deg'], step)
        for fraction in np.linspace(0., 1., 6)[1:]:
            add(q+(solution.x-q)*fraction, achieved+(yaw-achieved)*fraction)
        q, achieved = solution.x, float(yaw)
    data['achieved_yaw_deg'] = achieved
    data['ik_complete'] = abs(achieved-angle) < 1e-8
    data['joint_change_deg'] = np.degrees(q-q0).tolist()
    data['minimum_gaps'] = minima
    data['sampled_plane_margins_pass'] = all(
        minima[key]['gap_mm'] >= value for key, value in limits_mm.items()
    )
    input_path = output/'samples_NOT_EXECUTABLE.json'
    input_path.write_text(json.dumps(data, ensure_ascii=False, indent=2))
    completed = subprocess.run([str(binary), str(urdf_path), str(srdf_path), str(input_path)],
        capture_output=True, text=True, timeout=120, check=True)
    (output/'self_collision.csv').write_text(completed.stdout)
    (output/'self_collision_diagnostic.log').write_text(completed.stderr)
    rows = list(csv.DictReader(io.StringIO(completed.stdout)))
    if len(rows) != len(data['samples']):
        raise RuntimeError('Collision checker row count mismatch')
    results = {'samples_checked': len(rows), 'bare_collision_samples': 0, 'with_tool_collision_samples': 0,
               'joint_bound_violation_samples': 0, 'first_collision': None,
               'max_cross_checker_fk_position_error_mm': 0., 'max_cross_checker_fk_angle_error_deg': 0.}
    for index, (row, expected) in enumerate(zip(rows, expected_fk)):
        if int(row['index']) != index:
            raise RuntimeError('Collision checker index mismatch')
        results['bare_collision_samples'] += int(row['bare_collision'])
        results['with_tool_collision_samples'] += int(row['tool_collision'])
        results['joint_bound_violation_samples'] += 1-int(row['joint_bounds_ok'])
        if row['tool_collision'] == '1' and results['first_collision'] is None:
            results['first_collision'] = {'index': index, 'yaw_deg': data['samples'][index]['yaw_deg'],
                                          'bare_pairs': row['bare_pairs'], 'tool_pairs': row['tool_pairs']}
        p = np.asarray([float(row[key]) for key in ['fx', 'fy', 'fz']])
        r = Rotation.from_quat([float(row[key]) for key in ['qx', 'qy', 'qz', 'qw']])
        pos_error = np.linalg.norm(p-expected[:3, 3])*1000
        angle_error = np.degrees((r.inv()*Rotation.from_matrix(expected[:3, :3])).magnitude())
        results['max_cross_checker_fk_position_error_mm'] = max(results['max_cross_checker_fk_position_error_mm'], float(pos_error))
        results['max_cross_checker_fk_angle_error_deg'] = max(results['max_cross_checker_fk_angle_error_deg'], float(angle_error))
    results['fk_models_match'] = (results['max_cross_checker_fk_position_error_mm'] < 1e-5
                                  and results['max_cross_checker_fk_angle_error_deg'] < 1e-5)
    data['self_collision'] = results
    # This flag describes only modeled, discrete geometry; never execution.
    data['sampled_geometry_pass'] = (data['ik_complete'] and data['sampled_plane_margins_pass']
        and results['fk_models_match'] and not results['bare_collision_samples']
        and not results['with_tool_collision_samples'] and not results['joint_bound_violation_samples'])
    del data['samples']
    (output/'summary.json').write_text(json.dumps(data, ensure_ascii=False, indent=2))
    print(json.dumps({key: data[key] for key in ['execution_allowed', 'requested_yaw_deg', 'ik_complete',
        'minimum_gaps', 'self_collision', 'sampled_geometry_pass', 'joint_change_deg']}, ensure_ascii=False, indent=2))
    print(output/'summary.json')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True, help='NEW diagnostic directory')
    parser.add_argument('--snapshot', type=Path, help='Existing report; omit for one read-only RTDE snapshot')
    args = parser.parse_args()
    audit(args.output.resolve(), args.snapshot)
