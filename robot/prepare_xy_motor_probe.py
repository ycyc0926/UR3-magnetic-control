#!/usr/bin/env python3
"""Offline fixed-pose 3 mm -X probe for one supervised manual-motor trial.

The recorded H observation is never replaced with the commanded ball target.

No ROS node, network connection, publisher, action client or execution interface.
All outputs remain execution_allowed=false. Geometry is a provisional model.
"""
import argparse
import copy
import csv
from datetime import datetime, timezone
import hashlib
import io
import json
from pathlib import Path
import subprocess

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory

from preview_yaw_geometry import ROOT, JOINTS, box_corners
from preview_yaw_clearance import surface_bounds
from prepare_lower_yaw import timed_phase
from prepare_yaw_pilot import derivative_peaks, quintic
from ur3_magnetic_control.clearance_policy import (
    BOUNDARIES,
    clearance_limits_mm,
    load_clearance_limits_m,
)
from ur3_magnetic_control.ceiling_geometry import CeilingGeometry
from manual_motor_probe_guard import MEASURED_TCP_SPEED_LIMIT_MM_S, MEASURED_JOINT_SPEED_LIMIT_DEG_S

CONFIGS = ['table_world_calibration.yaml', 'acrylic_side_boundaries.yaml',
           'magnet_tool_geometry_draft.yaml', 'ur3_calibration.yaml',
           'camera_robot_calibration.yaml', 'ur3_system.yaml']
SPEED_LIMIT_DEG_S = 1.0
ACCEL_LIMIT_DEG_S2 = 2.0


def validate_snapshot(data):
    """Check validity at capture time; this never declares an old sample live."""
    state, vision = data['snapshot'], data['vision']
    captured = datetime.fromisoformat(data['captured_at'])
    stamp = datetime.fromisoformat(vision['utc'])
    if captured.tzinfo is None or stamp.tzinfo is None:
        raise ValueError('Timezone required')
    age = (captured-stamp).total_seconds()
    if not 0 <= age <= 1.5 or vision.get('detected') is not True or vision.get('stream_stale') is not False:
        raise ValueError('Invalid or stale vision at snapshot capture')
    if vision.get('tracking_reason') != 'tracked' or vision.get('consecutive_detections', 0) < 20:
        raise ValueError('Target has not been tracked continuously')
    for key in ['actual_q', 'actual_qd', 'actual_TCP_pose', 'actual_TCP_speed', 'tcp_offset']:
        value = np.asarray(state[key], float)
        if value.shape != (6,) or not np.isfinite(value).all():
            raise ValueError('Invalid '+key)
    target = np.asarray(vision['world_xy_mm'], float)/1000.
    if target.shape != (2,) or not np.isfinite(target).all():
        raise ValueError('Invalid H position')
    if (np.max(np.abs(state['actual_qd'])) > .0001 or
            np.max(np.abs(state['actual_TCP_speed'])) > .0001):
        raise ValueError('Snapshot robot was moving')
    if state['safety_mode'] != 1 or state['robot_mode'] != 7 or state['runtime_state'] != 1:
        raise ValueError('Capture requires NORMAL safety and stopped program')
    if not np.allclose(state['tcp_offset'], [0, -.062, .041, 0, 0, 0], atol=1e-8, rtol=0):
        raise ValueError('Wrong active TCP')
    if not np.isfinite(state['payload']) or abs(state['payload']-.32) > 1e-6:
        raise ValueError('Unexpected payload')
    return np.asarray(state['actual_q'], float), target


def xy_target(pose, sphere, target_xy):
    target_xy = np.asarray(target_xy, float)
    if target_xy.shape != (2,) or not np.isfinite(target_xy).all():
        raise ValueError('Invalid target')
    delta = target_xy-sphere[:2]
    if not .0005 <= np.linalg.norm(delta) <= .030:
        raise ValueError('XY diagnostic requires a 0.5--30 mm displacement')
    target = pose.copy()
    target[:2, 3] += delta
    return target


class XYGeometry:
    def __init__(self, xml, table, side, tool, q0, target_xy):
        self.table, self.side, self.tool = table, side, tool
        self.model = CeilingGeometry(xml, table['T_base_from_world'])
        self.offset = np.asarray(tool['previously_confirmed']['magnet_center_xyz_m'])
        self.radius = float(tool['previously_confirmed']['magnet_radius_m'])
        envelope = tool['provisional_whole_tool_envelope']
        self.corners = box_corners(envelope['min_xyz_m'], envelope['max_xyz_m'])
        self.padded_corners = box_corners(np.asarray(envelope['min_xyz_m'])-.005,
                                         np.asarray(envelope['max_xyz_m'])+.005)
        self.start_q = np.asarray(q0)
        self.start_pose = self.pose(q0)
        self.start_sphere = self.sphere(self.start_pose)
        self.target_pose = xy_target(self.start_pose, self.start_sphere, target_xy)
        self.target_sphere = self.sphere(self.target_pose)
        self.delta = self.target_sphere-self.start_sphere
        self.distance = np.linalg.norm(self.delta)
        self.direction = self.delta/self.distance
        self.gap_limits = load_clearance_limits_m(ROOT)

    def pose(self, q):
        return self.model.world_from_base @ self.model.transforms(dict(zip(JOINTS, q)))['tool0']

    def sphere(self, pose):
        return pose[:3, 3]+pose[:3, :3] @ self.offset

    def bounds(self, q, padded=True):
        transforms = self.model.transforms(dict(zip(JOINTS, q)))
        shapes = [(name, *surface_bounds(self.model.world_from_base @ transforms[name] @ origin, kind, shape))
                  for name, origin, kind, shape in self.model.surfaces]
        pose = self.model.world_from_base @ transforms['tool0']
        sphere = self.sphere(pose)
        shapes += [('tool', *surface_bounds(pose, 'box', self.padded_corners if padded else self.corners)),
                   ('sphere', sphere-self.radius, sphere+self.radius)]
        gaps = dict.fromkeys(BOUNDARIES, float('inf'))
        for name, low, high in shapes:
            values = {'ceiling': self.table['fixed_work_surface']['acrylic_bottom_world_z_m']-high[2],
                      'left': low[0]-self.side['left_inner_x_m'],
                      'right': self.side['right_inner_x_m']-high[0], 'table': low[2]}
            for key, value in values.items():
                gaps[key] = min(gaps[key], float(value))
        return gaps, pose

    def inspect(self, q, tight=False):
        q = np.asarray(q, float)
        if q.shape != (6,) or not np.isfinite(q).all():
            raise ValueError('Invalid joints')
        gaps, pose = self.bounds(q)
        if any(gaps[key] < limit for key, limit in self.gap_limits.items()):
            raise ValueError('Padded model clearance failed: '+str(gaps))
        sphere = self.sphere(pose)
        offset = sphere-self.start_sphere
        progress = float(offset @ self.direction)
        cross_track = float(np.linalg.norm(offset-progress*self.direction))
        angle = float(np.degrees((Rotation.from_matrix(self.start_pose[:3, :3]).inv()
                                 * Rotation.from_matrix(pose[:3, :3])).magnitude()))
        linear_tolerance = .00005 if tight else .0007
        angle_tolerance = .01 if tight else .15
        if (cross_track > linear_tolerance or abs(offset[2]) > linear_tolerance or
                not -linear_tolerance <= progress <= self.distance+linear_tolerance or angle > angle_tolerance):
            raise ValueError('XY corridor, height or fixed-pose constraint failed')
        return {'gaps_mm': {k: v*1000 for k, v in gaps.items()},
                'sphere_world_mm': (sphere*1000).tolist(), 'progress_mm': progress*1000,
                'cross_track_mm': cross_track*1000, 'height_change_mm': float(offset[2]*1000),
                'orientation_change_deg': angle}


def check_collision(binary, urdf, srdf, geometry, qs, output, expected=None):
    data = {'execution_allowed': False, 'joint_names': JOINTS, 'tool_geometry': geometry,
            'samples': [{'q_rad': np.asarray(q).tolist()} for q in qs]}
    path = output.with_suffix('.json')
    path.write_text(json.dumps(data, allow_nan=False))
    result = subprocess.run([str(binary), str(urdf), str(srdf), str(path)],
                            capture_output=True, text=True, check=True, timeout=60)
    output.write_text(result.stdout)
    output.with_suffix('.log').write_text(result.stderr)
    rows = list(csv.DictReader(io.StringIO(result.stdout)))
    if 'FCL positive control passed' not in result.stderr or len(rows) != len(qs):
        raise ValueError('Incomplete FCL check or missing positive control')
    maximum_fk_error = 0.
    for i, row in enumerate(rows):
        if int(row['index']) != i or row['bare_collision'] != '0' or row['tool_collision'] != '0' or row['joint_bounds_ok'] != '1':
            raise ValueError('FCL collision, ordering or joint-bound failure at sample '+str(i))
        if expected is not None:
            pose = expected[i]
            p_error = np.linalg.norm(np.array([float(row[k]) for k in ['fx', 'fy', 'fz']])-pose[:3, 3])
            r_error = (Rotation.from_quat([float(row[k]) for k in ['qx', 'qy', 'qz', 'qw']]).inv()
                       * Rotation.from_matrix(pose[:3, :3])).magnitude()
            maximum_fk_error = max(maximum_fk_error, float(p_error), float(r_error))
    if maximum_fk_error > 1e-8:
        raise ValueError('Independent FK implementations disagree')
    return {'samples': len(rows), 'maximum_fk_error': maximum_fk_error, 'positive_control_passed': True}


def plan(snapshot_path, output, duration=5.):
    if not np.isfinite(duration) or duration != 5.:
        raise ValueError('This scope is exactly 5 seconds of motion')
    data = json.loads(snapshot_path.read_text())
    q0, observed_h_xy = validate_snapshot(data)
    table, side, tool, calibration = [yaml.safe_load((ROOT/'config'/name).read_text()) for name in CONFIGS[:4]]
    if calibration['kinematics']['hash'] != 'calib_18089309548208516197':
        raise ValueError('Wrong robot calibration')
    if (side['frame'] != 'table_world' or side['user_confirmed']['distances_are_to_inner_faces'] is not True
            or side['execution_allowed'] is not False or tool['execution_allowed'] is not False
            or tool['status'] != 'incomplete_draft'):
        raise ValueError('Unexpected geometry conventions or authorization metadata')
    output.mkdir(parents=True, exist_ok=False)
    report = {'utc': datetime.now(timezone.utc).isoformat(), 'mode': 'offline_fixed_pose_XY_manual_motor_probe',
              'execution_allowed': False, 'motion_sent': False, 'checks_pass_for_draft': False,
              'source_snapshot': str(snapshot_path.resolve()), 'snapshot_sha256': hashlib.sha256(snapshot_path.read_bytes()).hexdigest(),
              'config_sha256': {name: hashlib.sha256((ROOT/'config'/name).read_bytes()).hexdigest() for name in CONFIGS},
              'source_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              'snapshot_data': data, 'table_geometry': table, 'side_geometry': side, 'tool_geometry': tool,
              'remaining_blockers': ['specific_motion_not_authorized', 'actual_tool_envelope_supports_cables_and_people_not_verified',
                                     'physical_stopping_distance_not_measured', 'camera_world_mapping_not_independently_verified',
                                     'motor_probe_executor_not_yet_reviewed', 'actual_TCP_speed_excursions_require_guard', 'manual_motor_has_no_automatic_stop', 'H_response_to_rotating_translating_magnet_unknown',
                                     'current_axis_tilt_differs_from_previous_successful_experiment']}
    try:
        xml = xacro.process_file(str(Path(get_package_share_directory('ur_description'))/'urdf/ur.urdf.xacro'),
            mappings={'name': 'ur', 'ur_type': 'ur3', 'use_fake_hardware': 'true',
                      'kinematics_params': str(ROOT/'config/ur3_calibration.yaml')}).toxml()
        srdf_xml = xacro.process_file(str(Path(get_package_share_directory('ur_moveit_config'))/'srdf/ur.srdf.xacro'),
                                     mappings={'name': 'ur', 'prefix': ''}).toxml()
        urdf, srdf = output/'diagnostic.urdf', output/'diagnostic.srdf'
        urdf.write_text(xml); srdf.write_text(srdf_xml)
        model = CeilingGeometry(xml, table['T_base_from_world'])
        start_pose = model.world_from_base @ model.transforms(dict(zip(JOINTS, q0)))['tool0']
        offset = np.asarray(tool['previously_confirmed']['magnet_center_xyz_m'])
        sphere = start_pose[:3, 3]+start_pose[:3, :3]@offset
        target_xy = sphere[:2]+np.array([-.003, 0.])
        if np.linalg.norm(observed_h_xy-sphere[:2]) > .0005:
            raise ValueError('Probe must start with H aligned within existing mapping tolerance')
        report.update(observed_h_start_world_mm=(observed_h_xy*1000).tolist(),
                      commanded_ball_delta_world_mm=[-3., 0., 0.],
                      manual_motor_max_duration_s=6., measured_tcp_abort_mm_s=MEASURED_TCP_SPEED_LIMIT_MM_S,
                      measured_joint_abort_deg_s=MEASURED_JOINT_SPEED_LIMIT_DEG_S, world_frame='table_world')
        geometry = XYGeometry(xml, table, side, tool, q0, target_xy)
        base_pose = geometry.model.transforms(dict(zip(JOINTS, q0)))['tool0']
        p_error = np.linalg.norm(base_pose[:3, 3]+base_pose[:3, :3]@geometry.offset-data['snapshot']['actual_TCP_pose'][:3])
        r_error = (Rotation.from_matrix(base_pose[:3, :3]).inv()*Rotation.from_rotvec(data['snapshot']['actual_TCP_pose'][3:])).magnitude()
        if p_error > .00025 or r_error > np.deg2rad(.05):
            raise ValueError('Controller active TCP and calibrated FK disagree')
        geometry.inspect(q0, tight=True)
        knots = [q0]
        max_ik_step = 0.
        for fraction in np.linspace(0, 1, max(3, int(np.ceil(geometry.distance/.0005))+1))[1:]:
            target = geometry.start_pose.copy()
            target[:3, 3] += fraction*geometry.delta
            def residual(q):
                pose = geometry.pose(q)
                return np.r_[pose[:3, 3]-target[:3, 3], .1*Rotation.from_matrix(target[:3, :3].T@pose[:3, :3]).as_rotvec()]
            fit = least_squares(residual, knots[-1],
                bounds=([-2*np.pi, -2*np.pi, -np.pi, -2*np.pi, -2*np.pi, -2*np.pi],
                        [2*np.pi, 2*np.pi, np.pi, 2*np.pi, 2*np.pi, 2*np.pi]),
                xtol=1e-11, ftol=1e-11, gtol=1e-11, max_nfev=150)
            error = residual(fit.x)
            step = np.max(np.abs(fit.x-knots[-1]))
            if np.linalg.norm(error[:3]) > 1e-6 or np.linalg.norm(error[3:]) > 1e-6 or step > np.deg2rad(2):
                raise ValueError('IK or joint continuity failed')
            geometry.inspect(fit.x, tight=True)
            max_ik_step = max(max_ik_step, float(step)); knots.append(fit.x)
        phase = timed_phase(knots, .25, duration)
        points = [dict(phase[0], time_s=0.)]+phase+[dict(phase[-1], time_s=duration+.5)]
        peaks = np.max([derivative_peaks(a, b) for a, b in zip(points, points[1:])], axis=0)
        if peaks[0] > np.deg2rad(SPEED_LIMIT_DEG_S) or peaks[1] > np.deg2rad(ACCEL_LIMIT_DEG_S2):
            raise ValueError('Analytic joint speed/acceleration limit failed; do not relax thresholds')
        draft = {'execution_allowed': False, 'joint_names': JOINTS, 'sample_period_s': .008, 'points': points,
                 'scope': 'one fixed-pose -X 3 mm probe with supervised manual motor; no square or automatic motor control',
                 'phases': {'move': [.25, duration+.25], 'final_hold': [duration+.25, duration+.5]}}
        path = output/'timed_path_NOT_AUTHORIZED.json'
        path.write_text(json.dumps(draft, indent=2, allow_nan=False))
        binaries = ROOT/'robot/offline_collision/build'
        sampled = subprocess.run([str(binaries/'sample_timed_trajectory'), str(path)],
                                 capture_output=True, text=True, check=True, timeout=60)
        (output/'controller_interpolation.csv').write_text(sampled.stdout)
        rows = list(csv.DictReader(io.StringIO(sampled.stdout)))
        ts = np.asarray([float(row['time_s']) for row in rows])
        qs = np.asarray([[float(row[f'q{i}']) for i in range(6)] for row in rows])
        vs = np.asarray([[float(row[f'v{i}']) for i in range(6)] for row in rows])
        accs = np.asarray([[float(row[f'a{i}']) for i in range(6)] for row in rows])
        if (len(rows) < 2 or abs(ts[0]) > 1e-9 or abs(ts[-1]-points[-1]['time_s']) > 1e-9
                or np.any(np.diff(ts) <= 0) or np.max(np.diff(ts)) > .008000002
                or not np.isfinite(np.r_[qs.ravel(), vs.ravel(), accs.ravel()]).all()):
            raise ValueError('Incomplete or invalid controller interpolation')
        minima = dict.fromkeys(BOUNDARIES, float('inf'))
        max_error = 0.; max_height = 0.; max_angle = 0.; max_cross = 0.
        segment = 0; last_stop = -1.; stopped_states = []; spheres = []; expected_fk = []
        stop_minima = dict.fromkeys(BOUNDARIES, float('inf'))
        for t, q, v, a in zip(ts, qs, vs, accs):
            while segment+1 < len(points)-1 and t > points[segment+1]['time_s']:
                segment += 1
            expected = quintic(points[segment], points[segment+1], t)
            max_error = max(max_error, *(float(np.max(np.abs(x-y))) for x, y in zip(expected, [q, v, a])))
            checked = geometry.inspect(q, tight=True)
            spheres.append(checked['sphere_world_mm'])
            expected_fk.append(geometry.model.transforms(dict(zip(JOINTS, q)))['tool0'])
            max_height = max(max_height, abs(checked['height_change_mm']))
            max_angle = max(max_angle, checked['orientation_change_deg'])
            max_cross = max(max_cross, checked['cross_track_mm'])
            for key, value in checked['gaps_mm'].items(): minima[key] = min(minima[key], value)
            if t-last_stop < .1: continue
            last_stop = t
            for latency in [0., .1, .2]:
                for deceleration in [1., 4.]:
                    for fraction in [0., .5, 1.]:
                        braking_time = np.abs(v)/deceleration*fraction
                        stop_q = q+v*latency+v*braking_time-np.sign(v)*deceleration*braking_time**2/2
                        stop_checked = geometry.inspect(stop_q)
                        for key, value in stop_checked['gaps_mm'].items(): stop_minima[key] = min(stop_minima[key], value)
                        stopped_states.append(stop_q)
        tcp_speed = np.linalg.norm(np.diff(np.asarray(spheres), axis=0)/np.diff(ts)[:, None], axis=1)
        if max_error > 1e-8 or np.max(tcp_speed) > 2.0:
            raise ValueError('Interpolation cross-check or Cartesian speed limit failed')
        padded = copy.deepcopy(tool)
        box = padded['provisional_whole_tool_envelope']
        box['min_xyz_m'] = (np.asarray(box['min_xyz_m'])-.005).tolist()
        box['max_xyz_m'] = (np.asarray(box['max_xyz_m'])+.005).tolist()
        print('Checking nominal and stopping samples in offline FCL', flush=True)
        fcl = check_collision(binaries/'check_self_collision', urdf, srdf, padded, list(qs)+stopped_states,
                              output/'padded_self_collision.csv',
                              expected_fk+[geometry.model.transforms(dict(zip(JOINTS, q)))['tool0'] for q in stopped_states])
        report.update(checks_pass_for_draft=True, sphere_start_world_mm=(geometry.start_sphere*1000).tolist(),
                      sphere_target_world_mm=(geometry.target_sphere*1000).tolist(), displacement_world_mm=(geometry.delta*1000).tolist(),
                      distance_mm=geometry.distance*1000, motion_duration_s=duration, total_duration_s=points[-1]['time_s'],
                      initial_fk_position_error_mm=p_error*1000, initial_fk_rotation_error_deg=float(np.degrees(r_error)),
                      max_joint_speed_deg_s=float(np.degrees(peaks[0])), max_joint_acceleration_deg_s2=float(np.degrees(peaks[1])),
                      max_tcp_speed_mm_s=float(np.max(tcp_speed)), max_ik_step_deg=float(np.degrees(max_ik_step)),
                      max_height_change_mm=max_height, max_orientation_change_deg=max_angle, max_cross_track_mm=max_cross,
                      padded_min_gaps_mm=minima, padded_stop_min_gaps_mm=stop_minima, samples=len(rows),
                      clearance_limits_mm=clearance_limits_mm(ROOT),
                      stop_sensitivity_samples=len(stopped_states), controller_interpolation_max_error=max_error, fcl=fcl,
                      stopping_model={'latencies_s': [0, .1, .2], 'decelerations_rad_s2': [1, 4], 'physically_validated': False},
                      draft_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                      artifacts_sha256={p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                                        for p in output.iterdir() if p.is_file()},
                      helper_sha256={str(p.resolve()): hashlib.sha256(p.read_bytes()).hexdigest() for p in
                                     [ROOT/'robot'/name for name in ['preview_yaw_geometry.py', 'preview_yaw_clearance.py',
                                                                   'prepare_lower_yaw.py', 'prepare_yaw_pilot.py']]
                                     +[ROOT/'ros2_ws/src/ur3_magnetic_control/ur3_magnetic_control'/name
                                       for name in ['ceiling_geometry.py', 'clearance_policy.py']]})
    except Exception as error:
        report['error'] = str(error)
        raise
    finally:
        (output/'audit.json').write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False))
    print(json.dumps({k: v for k, v in report.items() if k not in ['snapshot_data', 'table_geometry', 'side_geometry', 'tool_geometry']}, ensure_ascii=False, indent=2))
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--snapshot', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--duration', type=float, default=5.)
    args = parser.parse_args()
    plan(args.snapshot, args.output, args.duration)
