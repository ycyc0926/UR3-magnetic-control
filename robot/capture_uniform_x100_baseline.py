#!/usr/bin/env python3
"""Capture one stopped state and build coarse IK for a fixed-pose world -X line.

RTDE, Dashboard and vision are read only. This file has no controller, action,
program-playback or robot-input interfaces.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory

from prepare_uniform_x100 import UniformXGeometry
from prepare_xy_alignment import CONFIGS, validate_snapshot
from preview_yaw_geometry import ROOT, JOINTS
from rtde_status_outputs import read_status
from run_checked_xy_alignment import dashboard, vision_request


def capture(output):
    if output.exists():
        raise ValueError('Output already exists')
    state, version = read_status()
    vision = vision_request()
    data = {
        'captured_at': datetime.now(timezone.utc).isoformat(),
        'snapshot': state,
        'vision': vision,
        'controller_version': version,
        'dashboard': {key: dashboard(key) for key in
                      ('robotmode', 'safetystatus', 'running', 'programState', 'get loaded program')},
        'motion_sent': False,
        'execution_allowed': False,
    }
    q0, h = validate_snapshot(data)
    if data['dashboard'] != {
            'robotmode': 'Robotmode: RUNNING', 'safetystatus': 'Safetystatus: NORMAL',
            'running': 'Program running: false', 'programState': 'STOPPED external_control.urp',
            'get loaded program': 'Loaded program: /programs/external_control.urp'}:
        raise ValueError('Dashboard is not in the expected stopped state')
    table, side, tool = [yaml.safe_load((ROOT/'config'/name).read_text()) for name in CONFIGS[:3]]
    xml = xacro.process_file(
        str(Path(get_package_share_directory('ur_description'))/'urdf/ur.urdf.xacro'),
        mappings={'name': 'ur', 'ur_type': 'ur3', 'use_fake_hardware': 'true',
                  'kinematics_params': str(ROOT/'config/ur3_calibration.yaml')}).toxml()
    geometry = UniformXGeometry(xml, table, side, tool, q0)
    offset_mm = h*1000-geometry.start_sphere[:2]*1000
    if np.linalg.norm(offset_mm) > 3.:
        raise ValueError('H center is more than 3 mm from the ball center')
    knots = [q0]
    minima = dict.fromkeys(('ceiling', 'left', 'right', 'table'), float('inf'))
    max_step = 0.; failures = []
    for index, progress_mm in enumerate(np.linspace(0, 100, 201)):
        if index:
            target = geometry.start_pose.copy()
            target[:3, 3] += geometry.direction*(progress_mm/1000)
            def residual(q):
                pose = geometry.pose(q)
                return np.r_[pose[:3, 3]-target[:3, 3],
                             .1*Rotation.from_matrix(target[:3, :3].T@pose[:3, :3]).as_rotvec()]
            fit = least_squares(residual, knots[-1],
                bounds=([-2*np.pi, -2*np.pi, -np.pi, -2*np.pi, -2*np.pi, -2*np.pi],
                        [2*np.pi, 2*np.pi, np.pi, 2*np.pi, 2*np.pi, 2*np.pi]),
                xtol=1e-11, ftol=1e-11, gtol=1e-11, max_nfev=150)
            error = residual(fit.x); step = np.max(np.abs(fit.x-knots[-1]))
            if np.linalg.norm(error[:3]) > 1e-6 or np.linalg.norm(error[3:]) > 1e-6 or step > np.deg2rad(2):
                failures.append({'progress_mm': float(progress_mm), 'error': error.tolist(),
                                 'joint_step_deg': float(np.degrees(step))})
                break
            knots.append(fit.x); max_step = max(max_step, float(step))
        checked = geometry.inspect(knots[-1], tight=True)
        for key, value in checked['gaps_mm'].items():
            minima[key] = min(minima[key], value)
    if failures or len(knots) != 201:
        raise ValueError('Coarse fixed-pose IK failed: '+str(failures))
    output.mkdir()
    (output/'snapshot.json').write_text(json.dumps(data, indent=2, allow_nan=False)+'\n')
    (output/'diagnostic.urdf').write_text(xml)
    preliminary = {
        'mode': 'coarse_world_negative_x_100mm', 'execution_allowed': False,
        'motion_sent': False, 'requested_world_delta_mm': [-100, 0, 0],
        'requested_cruise_mm_s': 10, 'q_knots': np.asarray(knots).tolist(),
        'geometric_failure_count': 0, 'max_ik_step_deg': float(np.degrees(max_step)),
        'sphere_start_world_mm': (geometry.start_sphere*1000).tolist(),
        'sphere_target_world_mm': (geometry.target_sphere*1000).tolist(),
        'observed_h_start_world_mm': (h*1000).tolist(), 'h_minus_ball_xy_mm': offset_mm.tolist(),
        'nominal_min_gaps_mm': minima,
        'config_sha256': {name: hashlib.sha256((ROOT/'config'/name).read_bytes()).hexdigest()
                          for name in CONFIGS},
    }
    (output/'requested_path_preliminary.json').write_text(json.dumps(preliminary, indent=2, allow_nan=False)+'\n')
    return preliminary


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    print(json.dumps(capture(args.output), indent=2))
