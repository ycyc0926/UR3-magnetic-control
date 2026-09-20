#!/usr/bin/env python3
"""Offline audit of the user's exact -X 100 mm / 10 mm/s straight line.

Reads saved observations only. No ROS or hardware clients. A one-second smooth
speed ramp at each end surrounds nine seconds of constant 10 mm/s translation.
"""
import copy
import csv
from datetime import datetime, timezone
import hashlib
import io
import itertools
import json
from pathlib import Path
import subprocess

import numpy as np
from scipy.interpolate import CubicSpline
from scipy.spatial.transform import Rotation
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory

from prepare_xy_alignment import XYGeometry, validate_snapshot, check_collision, CONFIGS
from preview_yaw_geometry import ROOT, JOINTS, box_corners
from prepare_yaw_pilot import derivative_peaks, quintic
from ur3_magnetic_control.clearance_policy import clearance_limits_mm, load_clearance_limits_m
from ur3_magnetic_control.ceiling_geometry import CeilingGeometry
from manual_motor_probe_guard import MEASURED_TCP_SPEED_LIMIT_MM_S


class UniformXGeometry(XYGeometry):
    def __init__(self, xml, table, side, tool, q0):
        self.table, self.side, self.tool = table, side, tool
        self.model = CeilingGeometry(xml, table['T_base_from_world'])
        self.offset = np.asarray(tool['previously_confirmed']['magnet_center_xyz_m'])
        self.radius = float(tool['previously_confirmed']['magnet_radius_m'])
        e = tool['provisional_whole_tool_envelope']
        self.corners = box_corners(e['min_xyz_m'], e['max_xyz_m'])
        self.padded_corners = box_corners(np.array(e['min_xyz_m'])-.005, np.array(e['max_xyz_m'])+.005)
        self.start_q = np.asarray(q0)
        self.start_pose = self.pose(q0)
        self.start_sphere = self.sphere(self.start_pose)
        self.delta = np.array([-.100, 0., 0.])
        self.distance = .100
        self.direction = np.array([-1., 0., 0.])
        self.target_pose = self.start_pose.copy()
        self.target_pose[:3, 3] += self.delta
        self.target_sphere = self.sphere(self.target_pose)
        self.gap_limits = load_clearance_limits_m(ROOT)


def profile(t):
    """Positive traveled distance (mm), speed (mm/s), acceleration (mm/s²)."""
    if not 0 <= t <= 11:
        raise ValueError('Time outside the single reviewed segment')
    if t < 1:
        return 10*(t**3-.5*t**4), 10*(3*t*t-2*t**3), 60*t*(1-t)
    if t <= 10:
        return 5+10*(t-1), 10., 0.
    s, v, a = profile(11-t)
    return 100-s, v, -a


def plan(folder, joint_tracking_margin_deg_s=.5):
    if not .1 <= joint_tracking_margin_deg_s <= .5:
        raise ValueError('Joint velocity tracking margin outside this scope')
    limits_m = load_clearance_limits_m(ROOT)
    limits_mm = clearance_limits_mm(ROOT)
    data = json.loads((folder/'snapshot.json').read_text())
    q0, h = validate_snapshot(data)
    coarse = json.loads((folder/'requested_path_preliminary.json').read_text())
    if coarse['requested_world_delta_mm'] != [-100, 0, 0] or coarse['geometric_failure_count']:
        raise ValueError('Requested-path preliminary check failed')
    for name, expected in coarse['config_sha256'].items():
        if hashlib.sha256((ROOT/'config'/name).read_bytes()).hexdigest() != expected:
            raise ValueError('Configuration changed')
    qs = np.asarray(coarse['q_knots'])
    if qs.shape != (201, 6) or not np.array_equal(qs[0], q0):
        raise ValueError('Coarse path does not start at saved real observation')
    table, side, tool = [yaml.safe_load((ROOT/'config'/n).read_text()) for n in CONFIGS[:3]]
    xml = (folder/'diagnostic.urdf').read_text()
    g = UniformXGeometry(xml, table, side, tool, q0)
    srdf = xacro.process_file(str(Path(get_package_share_directory('ur_moveit_config'))/'srdf/ur.srdf.xacro'),
                             mappings={'name': 'ur', 'prefix': ''}).toxml()
    (folder/'diagnostic.srdf').write_text(srdf)
    base = g.model.transforms(dict(zip(JOINTS, q0)))['tool0']
    pe = np.linalg.norm(base[:3, 3]+base[:3, :3]@g.offset-data['snapshot']['actual_TCP_pose'][:3])
    re = (Rotation.from_matrix(base[:3, :3]).inv()*Rotation.from_rotvec(data['snapshot']['actual_TCP_pose'][3:])).magnitude()
    if pe > .00025 or re > np.deg2rad(.05):
        raise ValueError('Live TCP and calibrated FK mismatch')
    # Preserve the observed H location. User explicitly requested direct motion
    # from here instead of an additional alignment. The stale-H guard stays .5 mm.
    curve = CubicSpline(np.linspace(0, 100, 201), qs, axis=0)
    points = []
    for t in np.linspace(0, 11, 111):
        s, v, a = profile(float(t))
        points.append({'time_s': round(.25+t, 9), 'q_rad': curve(s).tolist(),
                       'velocity_rad_s': (curve(s, 1)*v).tolist(),
                       'acceleration_rad_s2': (curve(s, 2)*v*v+curve(s, 1)*a).tolist()})
    points = [dict(points[0], time_s=0.)]+points+[dict(points[-1], time_s=11.5)]
    peaks = np.max([derivative_peaks(a, b) for a, b in zip(points, points[1:])], axis=0)
    if np.degrees(peaks[0]) > 3.5 or np.degrees(peaks[1]) > 6.5:
        raise ValueError('Proposed straight-line dynamics outside reviewed scope')
    draft = {'execution_allowed': False, 'joint_names': JOINTS, 'sample_period_s': .008,
             'points': points, 'scope': 'single -X 100 mm; 10 mm/s cruise; fixed Z/pose; manual motor',
             'phases': {'ramp_up': [.25, 1.25], 'cruise': [1.25, 10.25],
                        'ramp_down': [10.25, 11.25], 'final_hold': [11.25, 11.5]}}
    path = folder/'timed_path_NOT_AUTHORIZED.json'
    path.write_text(json.dumps(draft, indent=2, allow_nan=False))
    binaries = ROOT/'robot/offline_collision/build'
    result = subprocess.run([str(binaries/'sample_timed_trajectory'), str(path)],
                            capture_output=True, text=True, check=True, timeout=60)
    (folder/'controller_interpolation.csv').write_text(result.stdout)
    rows = list(csv.DictReader(io.StringIO(result.stdout)))
    ts = np.array([float(r['time_s']) for r in rows])
    q = np.array([[float(r[f'q{i}']) for i in range(6)] for r in rows])
    v = np.array([[float(r[f'v{i}']) for i in range(6)] for r in rows])
    acc = np.array([[float(r[f'a{i}']) for i in range(6)] for r in rows])
    if (len(ts) < 1400 or abs(ts[0]) > 1e-9 or abs(ts[-1]-11.5) > 1e-9
            or np.any(np.diff(ts) <= 0) or max(np.diff(ts)) > .008000002
            or not np.isfinite(np.r_[q.ravel(), v.ravel(), acc.ravel()]).all()):
        raise ValueError('Invalid real-controller interpolation')
    minima = dict.fromkeys(('ceiling', 'left', 'right', 'table'), float('inf'))
    stop_minima = minima.copy(); stop_qs = []; spheres = []; expected = []
    error = 0.; segment = 0; last_stop = -1.
    for t, joint, vel, acceleration in zip(ts, q, v, acc):
        while segment+1 < len(points)-1 and t > points[segment+1]['time_s']:
            segment += 1
        value = quintic(points[segment], points[segment+1], t)
        error = max(error, *(float(np.max(np.abs(x-y))) for x, y in zip(value, (joint, vel, acceleration))))
        checked = g.inspect(joint, tight=True)
        spheres.append(checked['sphere_world_mm'])
        expected.append(g.model.transforms(dict(zip(JOINTS, joint)))['tool0'])
        for key, val in checked['gaps_mm'].items(): minima[key] = min(minima[key], val)
        if t-last_stop >= .1:
            last_stop = t
            for delay, decel, fraction in itertools.product((0., .1, .2), (1., 4.), (0., .5, 1.)):
                bt = abs(vel)/decel*fraction
                stop = joint+vel*delay+vel*bt-np.sign(vel)*decel*bt*bt/2
                bounds, _ = g.bounds(stop)
                if any(bounds[k] < lim for k, lim in limits_m.items()):
                    raise ValueError('Nominal stop clearance failed')
                for key, val in bounds.items(): stop_minima[key] = min(stop_minima[key], val*1000)
                stop_qs.append(stop)
    sphere = np.array(spheres)
    speed = np.linalg.norm(np.diff(sphere, axis=0)/np.diff(ts)[:, None], axis=1)
    cruise = (ts[:-1] >= 1.25)&(ts[1:] <= 10.25)
    if error > 1e-8 or max(speed) > 10.05 or max(abs(speed[cruise]-10)) > .01:
        raise ValueError('Controller interpolation is not the reviewed uniform-speed path')
    padded = copy.deepcopy(tool); box = padded['provisional_whole_tool_envelope']
    box['min_xyz_m'] = (np.array(box['min_xyz_m'])-.005).tolist()
    box['max_xyz_m'] = (np.array(box['max_xyz_m'])+.005).tolist()
    fcl = check_collision(binaries/'check_self_collision', folder/'diagnostic.urdf', folder/'diagnostic.srdf',
        padded, list(q)+stop_qs, folder/'padded_self_collision.csv',
        expected+[g.model.transforms(dict(zip(JOINTS, x)))['tool0'] for x in stop_qs])
    # Path-specific measured-speed gate: an 80 ms position-difference velocity
    # must stay within a reviewed margin of the controller's desired velocity.
    # The 200 ms stop latency below already covers the causal filter window.
    # There is no fixed 5 deg/s gate in this motion scope.
    margin = np.deg2rad(joint_tracking_margin_deg_s)
    corners = []; stop_bounds = minima.copy(); reviewed_speed_bounds = np.zeros(6)
    stride = max(1, round(.2/np.median(np.diff(ts))))
    indices = sorted(set(range(0, len(q), stride)) | {len(q)-1})
    for index in indices:
        joint, desired_velocity = q[index], v[index]
        for signs in itertools.product((-1, 1), repeat=6):
            signs = np.asarray(signs)
            monitored_velocity = desired_velocity+margin*signs
            reviewed_speed_bounds = np.maximum(reviewed_speed_bounds, np.abs(monitored_velocity))
            displacement = (.0015*signs + monitored_velocity*.2
                            +np.sign(monitored_velocity)*monitored_velocity**2/2)
            s = joint+displacement; bounds, _ = g.bounds(s)
            for key, val in bounds.items(): stop_bounds[key] = min(stop_bounds[key], val*1000)
            corners.append(s)
    stop_fcl = check_collision(binaries/'check_self_collision', folder/'diagnostic.urdf', folder/'diagnostic.srdf',
        padded, corners, folder/'measured_stop_fcl.csv',
        [g.model.transforms(dict(zip(JOINTS, x)))['tool0'] for x in corners])
    stop_passed = all(stop_bounds[k] >= lim for k, lim in limits_mm.items())
    stop_review = {'model_stop_sensitivity_passed': stop_passed, 'execution_allowed': False,
                  'joint_speed_monitor_mode': 'windowed_desired_velocity_tracking',
                  'joint_velocity_tracking_margin_deg_s': joint_tracking_margin_deg_s,
                  'velocity_window_s': .08,
                  'reviewed_absolute_speed_bounds_deg_s': np.degrees(reviewed_speed_bounds).tolist(),
                  'latency_s': .2, 'deceleration_rad_s2': 1.,
                  'tracking_error_rad': .0015, 'physically_validated': False,
                  'min_gaps_mm': stop_bounds, 'fcl': stop_fcl,
                  'clearance_limits_mm': limits_mm}
    (folder/'STOP_REVIEW.json').write_text(json.dumps(stop_review, indent=2))
    if not stop_passed:
        raise ValueError(f'Requested {joint_tracking_margin_deg_s:g} deg/s tracking-margin stopping sensitivity clearance failed')
    report = {'utc': datetime.now(timezone.utc).isoformat(), 'mode': 'offline_uniform_world_negative_x_100mm',
              'execution_allowed': False, 'motion_sent': False, 'checks_pass_for_draft': True,
              'snapshot_data': data, 'source_snapshot': str((folder/'snapshot.json').resolve()),
              'snapshot_sha256': hashlib.sha256((folder/'snapshot.json').read_bytes()).hexdigest(),
              'table_geometry': table, 'side_geometry': side, 'tool_geometry': tool,
              'sphere_start_world_mm': (g.start_sphere*1000).tolist(),
              'sphere_target_world_mm': (g.target_sphere*1000).tolist(),
              'observed_h_start_world_mm': (h*1000).tolist(), 'displacement_world_mm': [-100., 0., 0.],
              'motion_duration_s': 11., 'total_duration_s': 11.5, 'cruise_speed_mm_s': 10.,
              'manual_motor_max_duration_s': 12., 'measured_tcp_abort_mm_s': MEASURED_TCP_SPEED_LIMIT_MM_S,
              'joint_speed_monitor_mode': 'windowed_desired_velocity_tracking',
              'joint_velocity_tracking_margin_deg_s': joint_tracking_margin_deg_s,
              'velocity_window_s': .08, 'tcp_speed_monitor_mode': 'windowed_pose_difference',
              'reviewed_absolute_speed_bounds_deg_s': np.degrees(reviewed_speed_bounds).tolist(),
              'max_joint_speed_deg_s': float(np.degrees(peaks[0])),
              'max_joint_acceleration_deg_s2': float(np.degrees(peaks[1])),
              'max_tcp_speed_mm_s': float(max(speed)), 'max_cruise_speed_error_mm_s': float(max(abs(speed[cruise]-10))),
              'clearance_limits_mm': limits_mm,
              'padded_min_gaps_mm': minima, 'padded_stop_min_gaps_mm': stop_minima,
              'samples': len(q), 'stop_sensitivity_samples': len(stop_qs), 'fcl': fcl,
              'source_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              'config_sha256': coarse['config_sha256'],
              'artifacts_sha256': {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in folder.iterdir()
                                   if p.is_file() and p.name not in ('audit.json', 'planning.log')},
              'helper_sha256': {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in
                 [ROOT/'robot'/n for n in ('prepare_xy_alignment.py', 'prepare_yaw_pilot.py', 'preview_yaw_geometry.py', 'preview_yaw_clearance.py', 'manual_motor_probe_guard.py')]
                 +[ROOT/'ros2_ws/src/ur3_magnetic_control/ur3_magnetic_control'/n
                   for n in ('ceiling_geometry.py', 'clearance_policy.py')]}}
    (folder/'audit.json').write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False))
    return report


if __name__ == '__main__':
    import argparse
    p = argparse.ArgumentParser(); p.add_argument('folder', type=Path)
    p.add_argument('--joint-tracking-margin-deg-s', type=float, default=.5)
    args = p.parse_args()
    a = plan(args.folder, args.joint_tracking_margin_deg_s)
    print(json.dumps({k: a[k] for k in ('checks_pass_for_draft', 'max_tcp_speed_mm_s',
        'max_cruise_speed_error_mm_s', 'max_joint_speed_deg_s', 'max_joint_acceleration_deg_s2',
        'padded_min_gaps_mm', 'samples', 'fcl')}, ensure_ascii=False))
