#!/usr/bin/env python3
"""Prepare and audit a 5-degree timing DRAFT. Never executes or starts ROS nodes.

Uses the installed joint_trajectory_controller library to evaluate splines.
Unverified physical geometry and live execution/stopping remain blockers.
"""
import argparse
import csv
from datetime import datetime, timezone
import io
import json
from pathlib import Path
import subprocess

import numpy as np
from scipy.spatial.transform import Rotation
from scipy.interpolate import CubicSpline
from scipy.optimize import brentq
import yaml

from preview_yaw_geometry import ROOT, JOINTS, box_corners
from preview_yaw_clearance import audit, surface_bounds
from ur3_magnetic_control.clearance_policy import clearance_limits_mm
from ur3_magnetic_control.ceiling_geometry import CeilingGeometry

JOINT_SPEED_LIMIT = np.deg2rad(1.)
JOINT_ACCEL_LIMIT = np.deg2rad(2.)
SAMPLE_PERIOD = .008


def quintic_coefficients(left, right):
    duration = right['time_s']-left['time_s']
    if duration <= 0:
        raise ValueError('Nonpositive segment duration')
    c0 = np.asarray(left['q_rad'])
    c1 = np.asarray(left['velocity_rad_s'])*duration
    c2 = np.asarray(left['acceleration_rad_s2'])*duration**2/2
    d = np.asarray(right['q_rad'])-c0-c1-c2
    v = np.asarray(right['velocity_rad_s'])*duration-c1-2*c2
    a = np.asarray(right['acceleration_rad_s2'])*duration**2-2*c2
    return np.asarray([c0, c1, c2, 10*d-4*v+a/2, -15*d+7*v-a, 6*d-3*v+a/2]), duration


def quintic(left, right, time):
    coefficients, duration = quintic_coefficients(left, right)
    u = np.clip((time-left['time_s'])/duration, 0., 1.)
    q = sum(coefficients[k]*u**k for k in range(6))
    v = sum(k*coefficients[k]*u**(k-1) for k in range(1, 6))/duration
    a = sum(k*(k-1)*coefficients[k]*u**(k-2) for k in range(2, 6))/duration**2
    return q, v, a


def derivative_peaks(left, right):
    coefficients, duration = quintic_coefficients(left, right)
    peaks = np.zeros(2)
    for joint in range(6):
        polynomial = np.polynomial.Polynomial(coefficients[:, joint])
        for order in [1, 2]:
            derivative = polynomial.deriv(order)
            candidates = [0., 1.]
            candidates += [float(root.real) for root in derivative.deriv().roots()
                           if abs(root.imag) < 1e-9 and 0 < root.real < 1]
            peaks[order-1] = max(peaks[order-1], float(np.max(np.abs(derivative(np.asarray(candidates)))))/duration**order)
    return peaks


def smooth_full_turn_points(nodes, duration=45.):
    angles = np.asarray([node['yaw_deg'] for node in nodes])
    joints = np.asarray([node['q_rad'] for node in nodes])
    if (len(angles) < 3 or angles[0] != 0. or np.any(np.diff(angles) <= 0)
            or np.max(np.diff(angles)) > .500001 or not 0 < angles[-1] < 120.
            or joints.shape != (len(angles), 6) or not np.isfinite(joints).all()
            or not np.isfinite(duration) or duration < 30.):
        raise ValueError('Invalid full-turn timing input')
    curve = CubicSpline(angles, joints, axis=0)
    def progress(u):return 10*u**3-15*u**4+6*u**5
    points = []
    for index, angle in enumerate(angles):
        u = 0. if index == 0 else (1. if index == len(angles)-1 else brentq(lambda value: progress(value)-angle/angles[-1], 0., 1.))
        rate = angles[-1]*(30*u**2-60*u**3+30*u**4)/duration
        accel = angles[-1]*(60*u-180*u**2+120*u**3)/duration**2
        points.append({'time_s': round(1.+duration*u, 9), 'q_rad': joints[index].tolist(),
            'yaw_node_deg': float(angle), 'velocity_rad_s': (curve(angle, 1)*rate).tolist(),
            'acceleration_rad_s2': (curve(angle, 2)*rate**2+curve(angle, 1)*accel).tolist()})
    return [dict(points[0], time_s=0.)]+points+[dict(points[-1], time_s=duration+3.)]


def pilot_points(nodes):
    if not nodes or abs(nodes[0]['yaw_deg']) > 1e-9:
        raise ValueError('Nodes must start at zero yaw')
    selected = [node for node in nodes if node['yaw_deg'] <= 5.+1e-8]
    if abs(selected[-1]['yaw_deg']-5.) > 1e-8:
        raise ValueError('Need an exact 5-degree endpoint')
    points = []
    def add(q, yaw, time):
        values = np.asarray(q, float)
        if values.shape != (6,) or not np.isfinite(values).all():
            raise ValueError('Invalid joint values')
        points.append({'time_s': time, 'q_rad': values.tolist(), 'yaw_node_deg': float(yaw),
                       'velocity_rad_s': [0.]*6, 'acceleration_rad_s2': [0.]*6})
    add(selected[0]['q_rad'], 0., 0.)
    add(selected[0]['q_rad'], 0., 1.)
    for before, after in zip(selected, selected[1:]):
        delta_yaw = after['yaw_deg']-before['yaw_deg']
        if not 0 < delta_yaw <= .500001:
            raise ValueError('Yaw nodes not sufficiently dense/ordered')
        delta = np.max(np.abs(np.asarray(after['q_rad'])-before['q_rad']))
        # Zero endpoint velocities/accelerations: exact maxima of the quintic.
        duration = max(2., 1.875*delta/JOINT_SPEED_LIMIT,
                       np.sqrt((10/np.sqrt(3))*delta/JOINT_ACCEL_LIMIT))
        duration = float(np.ceil(duration*1000)/1000)
        add(after['q_rad'], after['yaw_deg'], points[-1]['time_s']+duration)
    add(selected[-1]['q_rad'], 5., points[-1]['time_s']+2.)
    return points


def run(output, full_turn=False):
    build = ROOT/'robot/offline_collision/build'
    sampler, checker = build/'sample_timed_trajectory', build/'check_self_collision'
    if not sampler.is_file() or not checker.is_file():
        raise RuntimeError('Build both offline utilities first')
    output.mkdir(parents=True, exist_ok=False)
    limits_mm = clearance_limits_mm(ROOT)
    source = output/'fresh_geometry'
    audit(source)  # Fresh read-only RTDE snapshot plus full +Y geometry audit.
    summary = json.loads((source/'summary.json').read_text())
    if not summary['sampled_geometry_pass']:
        raise RuntimeError('Source geometry audit did not pass')
    samples = json.loads((source/'samples_NOT_EXECUTABLE.json').read_text())['samples']
    nodes = samples[::5]
    points = smooth_full_turn_points(nodes) if full_turn else pilot_points(nodes)
    target_yaw = summary['requested_yaw_deg'] if full_turn else 5.
    speed_limit = np.deg2rad(8.) if full_turn else JOINT_SPEED_LIMIT
    accel_limit = np.deg2rad(10.) if full_turn else JOINT_ACCEL_LIMIT
    draft = {'execution_allowed': False, 'mode': 'timed_full_turn_DRAFT' if full_turn else 'timed_pilot_DRAFT_NOT_AUTHORIZED',
        'joint_names': JOINTS, 'target_relative_world_yaw_deg': target_yaw, 'sample_period_s': SAMPLE_PERIOD,
        'joint_speed_limit_rad_s': float(speed_limit), 'joint_accel_limit_rad_s2': float(accel_limit),
        'points': points, 'end_behavior': 'stop at world +Y' if full_turn else 'stop at 5 degrees; never auto-chain later stages',
        'future_checkpoints_relative_to_original_deg': [5., 15., 30., 45., 60., 75., 90., summary['requested_yaw_deg']],
        'conditions': ['live_start_revalidation', 'actual_geometry_and_clearance_verified',
                       'live_controller_configuration_verified', 'separate_bounded_execution_authorization',
                       'feedback_loss_and_cancel_stopping_behavior_verified']}
    draft_path = output/'timed_pilot_DRAFT_NOT_AUTHORIZED.json'
    draft_path.write_text(json.dumps(draft, ensure_ascii=False, indent=2))
    result = subprocess.run([str(sampler), str(draft_path)], capture_output=True, text=True, check=True, timeout=60)
    (output/'controller_interpolation.csv').write_text(result.stdout)
    rows = list(csv.DictReader(io.StringIO(result.stdout)))
    if len(rows) < 2:
        raise RuntimeError('Missing controller samples')
    times = np.asarray([float(row['time_s']) for row in rows])
    if abs(times[0]) > 1e-9 or abs(times[-1]-points[-1]['time_s']) > 1e-9 or np.any(np.diff(times) <= 0):
        raise RuntimeError('Controller sampler time coverage invalid')
    if np.max(np.diff(times)) > SAMPLE_PERIOD+2e-9:
        raise RuntimeError('Controller sampler is too sparse')
    q = np.asarray([[float(row[f'q{i}']) for i in range(6)] for row in rows])
    velocity = np.asarray([[float(row[f'v{i}']) for i in range(6)] for row in rows])
    accel = np.asarray([[float(row[f'a{i}']) for i in range(6)] for row in rows])
    if not np.isfinite(np.r_[q.ravel(), velocity.ravel(), accel.ravel()]).all():
        raise RuntimeError('Invalid interpolation values')
    errors = np.zeros(3)
    segment = 0
    for index, time in enumerate(times):
        while segment+1 < len(points)-1 and time > points[segment+1]['time_s']:
            segment += 1
        expected = quintic(points[segment], points[segment+1], time)
        for column, (actual, wanted) in enumerate(zip([q[index], velocity[index], accel[index]], expected)):
            errors[column] = max(errors[column], np.max(np.abs(actual-wanted)))
    if np.max(errors) > 1e-8:
        raise RuntimeError('Installed controller disagrees with quintic audit')
    analytic_speed, analytic_accel = 0., 0.
    for left, right in zip(points, points[1:]):
        speed, acceleration = derivative_peaks(left, right)
        analytic_speed = max(analytic_speed, speed)
        analytic_accel = max(analytic_accel, acceleration)
    if analytic_speed > speed_limit+1e-12 or analytic_accel > accel_limit+1e-12:
        raise RuntimeError('Analytic limits exceeded')
    table = yaml.safe_load((ROOT/'config/table_world_calibration.yaml').read_text())
    model = CeilingGeometry((source/'diagnostic.urdf').read_text(), table['T_base_from_world'])
    side, geometry = summary['side_boundaries'], summary['tool_geometry']
    box = geometry['provisional_whole_tool_envelope']
    corners = box_corners(box['min_xyz_m'], box['max_xyz_m'])
    offset = np.asarray(geometry['previously_confirmed']['magnet_center_xyz_m'])
    radius = geometry['previously_confirmed']['magnet_radius_m']
    bottom = table['fixed_work_surface']['acrylic_bottom_world_z_m']
    minima = {name: {'gap_mm': float('inf')} for name in ['left', 'right', 'ceiling', 'table']}
    drift, max_twist, max_sphere_speed, max_flange_speed, max_axis_elevation = 0., 0., 0., 0., 0.
    sphere_start = None
    expected_fk = []
    for index, (joints, speeds) in enumerate(zip(q, velocity)):
        transforms = model.transforms(dict(zip(JOINTS, joints)))
        expected_fk.append(transforms['tool0'])
        tool = model.world_from_base @ transforms['tool0']
        sphere = tool[:3, 3]+tool[:3, :3]@offset
        if sphere_start is None:sphere_start = sphere.copy()
        drift = max(drift, float(np.linalg.norm(sphere-sphere_start)))
        shapes = []
        for name, origin, kind, shape in model.surfaces:
            low, high = surface_bounds(model.world_from_base@transforms[name]@origin, kind, shape)
            shapes.append((name, low, high))
        low, high = surface_bounds(tool, 'box', corners)
        shapes += [('unverified_tool_envelope', low, high), ('magnet_sphere', sphere-radius, sphere+radius)]
        for name, low, high in shapes:
            values = {'left': low[0]-side['left_inner_x_m'], 'right': side['right_inner_x_m']-high[0],
                      'ceiling': bottom-high[2], 'table': low[2]}
            for boundary, gap in values.items():
                if gap*1000 < minima[boundary]['gap_mm']:
                    minima[boundary] = {'gap_mm': float(gap*1000), 'link': name, 'time_s': float(times[index])}
        angular, linear_sphere, linear_flange = np.zeros(3), np.zeros(3), np.zeros(3)
        for name, kind, parent, child, fixed, axis in model.joints:
            if name not in JOINTS:continue
            joint_pose = model.world_from_base @ transforms[parent] @ fixed
            axis_world = joint_pose[:3, :3] @ (axis/np.linalg.norm(axis))
            rate = speeds[JOINTS.index(name)]
            angular += axis_world*rate
            linear_sphere += np.cross(axis_world, sphere-joint_pose[:3, 3])*rate
            linear_flange += np.cross(axis_world, tool[:3, 3]-joint_pose[:3, 3])*rate
        max_twist = max(max_twist, float(np.linalg.norm(angular)))
        max_sphere_speed = max(max_sphere_speed, float(np.linalg.norm(linear_sphere)))
        max_flange_speed = max(max_flange_speed, float(np.linalg.norm(linear_flange)))
        axis = tool[:3, :3]@np.array([0., -1., 0.])
        max_axis_elevation = max(max_axis_elevation, abs(float(np.arctan2(axis[2], np.linalg.norm(axis[:2])))))
    collision_input = {'execution_allowed': False, 'joint_names': JOINTS, 'tool_geometry': geometry,
                       'samples': [{'q_rad': values.tolist()} for values in q]}
    collision_path = output/'interpolated_states_NOT_AUTHORIZED.json'
    collision_path.write_text(json.dumps(collision_input))
    collision_result = subprocess.run([str(checker), str(source/'diagnostic.urdf'), str(source/'diagnostic.srdf'),
        str(collision_path)], capture_output=True, text=True, check=True, timeout=60)
    (output/'timed_self_collision.csv').write_text(collision_result.stdout)
    (output/'timed_self_collision.log').write_text(collision_result.stderr)
    collisions = list(csv.DictReader(io.StringIO(collision_result.stdout)))
    if len(collisions) != len(q):raise RuntimeError('Collision sample count mismatch')
    collision_count, bad_bounds, max_fk_p, max_fk_r = 0, 0, 0., 0.
    for index, (row, expected) in enumerate(zip(collisions, expected_fk)):
        if int(row['index']) != index:raise RuntimeError('Collision sequence mismatch')
        collision_count += int(row['tool_collision'])
        bad_bounds += 1-int(row['joint_bounds_ok'])
        p = np.asarray([float(row[key]) for key in ['fx', 'fy', 'fz']])
        r = Rotation.from_quat([float(row[key]) for key in ['qx', 'qy', 'qz', 'qw']])
        max_fk_p = max(max_fk_p, float(np.linalg.norm(p-expected[:3, 3])))
        max_fk_r = max(max_fk_r, float((r.inv()*Rotation.from_matrix(expected[:3, :3])).magnitude()))
    first = model.world_from_base@expected_fk[0]
    last = model.world_from_base@expected_fk[-1]
    turn = Rotation.from_matrix(last[:3, :3]@first[:3, :3].T).as_rotvec()
    turn_ok = abs(np.degrees(turn[2])-target_yaw) < .001 and np.linalg.norm(turn[:2]) < 1e-5
    checks_pass = (all(minima[key]['gap_mm'] >= value for key, value in limits_mm.items()) and collision_count == 0
        and bad_bounds == 0 and max_fk_p < 1e-8 and max_fk_r < 1e-8
        and drift < .0001 and np.degrees(max_twist) < (5. if full_turn else 1.) and np.degrees(max_axis_elevation) < .1 and turn_ok)
    report = {'utc': datetime.now(timezone.utc).isoformat(), 'execution_allowed': False, 'motion_sent': False,
        'motor_stop_and_area_cleared': 'user_confirmed; not electronically sensed',
        'pilot_yaw_deg': target_yaw, 'full_turn': full_turn, 'total_nominal_time_s': points[-1]['time_s'],
        'motion_time_s': points[-2]['time_s']-points[1]['time_s'], 'initial_hold_s': 1., 'final_hold_s': 2.,
        'future_stages_auto_execute': False, 'samples': len(q), 'sample_period_s': SAMPLE_PERIOD,
        'installed_controller_interpolator_tested': True, 'active_controller_configuration_verified': False,
        'quintic_max_abs_errors_q_v_a': errors.tolist(),
        'analytic_max_joint_speed_deg_s': float(np.degrees(analytic_speed)),
        'analytic_max_joint_accel_deg_s2': float(np.degrees(analytic_accel)),
        'sampled_max_tool_angular_speed_deg_s': float(np.degrees(max_twist)),
        'sampled_max_sphere_speed_mm_s': max_sphere_speed*1000,
        'sampled_max_flange_speed_mm_s': max_flange_speed*1000,
        'max_sphere_drift_mm': drift*1000, 'max_axis_elevation_deg': float(np.degrees(max_axis_elevation)),
        'clearance_limits_mm': limits_mm,
        'minimum_gaps': minima, 'self_collision_samples': collision_count, 'joint_bound_violations': bad_bounds,
        'checks_pass_for_draft': bool(checks_pass), 'joint_change_deg': np.degrees(q[-1]-q[0]).tolist(),
        'end_turn_rotvec_deg': np.degrees(turn).tolist(),
        'remaining_blockers': ['physical_tool_envelope_and_posts_not_fully_verified',
            'physical_ceiling_clearance_and_calibration_error', 'active_controller_and_live_start_validation',
            'feedback_monitor_and_cancellation_stopping_path', 'explicit_pilot_execution_confirmation']}
    (output/'pilot_audit.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
    print('PILOT_AUDIT '+json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True, help='New offline draft directory')
    parser.add_argument('--full-turn', action='store_true', help='Audit the complete +Y turn, 45 s motion; NEVER execute')
    args = parser.parse_args()
    run(args.output.resolve(), args.full_turn)
