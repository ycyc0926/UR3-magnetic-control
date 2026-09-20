#!/usr/bin/env python3
"""Read-only planning: lower sphere 20 mm, stop, yaw toward world +Y, stop.

RTDE outputs only. This module never creates a ROS node, sends a trajectory,
changes robot configuration, or promotes unknown physical checks to verified.
"""
import argparse
import csv
from datetime import datetime, timezone
import io
import json
from pathlib import Path
import subprocess

import numpy as np
from scipy.interpolate import CubicSpline
from scipy.optimize import brentq, least_squares
from scipy.spatial.transform import Rotation
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory

from preview_yaw_geometry import ROOT, JOINTS, box_corners, fixed_sphere_target, read_state
from preview_yaw_clearance import surface_bounds, yaw_to_positive_y
from prepare_yaw_pilot import derivative_peaks, quintic
from ur3_magnetic_control.clearance_policy import clearance_limits_mm
from ur3_magnetic_control.ceiling_geometry import CeilingGeometry


def timed_phase(joints, start, duration):
    q = np.asarray(joints, dtype=float)
    if (q.ndim != 2 or q.shape[1] != 6 or len(q) < 3 or not np.isfinite(q).all()
            or not np.isfinite([start, duration]).all() or start < 0 or duration <= 0):
        raise ValueError('Invalid phase')
    parameter = np.linspace(0., 1., len(q))
    curve = CubicSpline(parameter, q, axis=0)
    def progress(u):return 10*u**3-15*u**4+6*u**5
    result = []
    for index, value in enumerate(parameter):
        u = 0. if index == 0 else (1. if index == len(q)-1 else brentq(lambda t:progress(t)-value, 0., 1.))
        rate = 30*u**2*(1-u)**2/duration
        acceleration = (60*u-180*u*u+120*u**3)/duration**2
        result.append({'time_s': round(start+duration*u, 9), 'q_rad': q[index].tolist(),
                       'velocity_rad_s': (curve(value, 1)*rate).tolist(),
                       'acceleration_rad_s2': (curve(value, 2)*rate**2+curve(value, 1)*acceleration).tolist()})
    return result


def plan(output, lower_mm=20., diagnostic_only=False, lower_seconds=10.):
    allowed = 1. <= lower_mm <= 3. if diagnostic_only else 10. <= lower_mm <= 30.
    if not np.isfinite(lower_mm) or not allowed:
        raise ValueError('A short diagnostic allows only 1--3 mm; full plans allow 10--30 mm')
    if not np.isfinite(lower_seconds) or not 10. <= lower_seconds <= 30.:
        raise ValueError('Descent duration must be 10--30 seconds')
    limits_mm = clearance_limits_mm(ROOT)
    output.mkdir(parents=True, exist_ok=False)
    table = yaml.safe_load((ROOT/'config/table_world_calibration.yaml').read_text())
    side = yaml.safe_load((ROOT/'config/acrylic_side_boundaries.yaml').read_text())
    geometry = yaml.safe_load((ROOT/'config/magnet_tool_geometry_draft.yaml').read_text())
    calibration = ROOT/'config/ur3_calibration.yaml'
    if yaml.safe_load(calibration.read_text())['kinematics']['hash'] != 'calib_18089309548208516197':
        raise RuntimeError('Wrong robot calibration')
    state, version = read_state()
    xml = xacro.process_file(str(Path(get_package_share_directory('ur_description'))/'urdf/ur.urdf.xacro'),
        mappings={'name':'ur','ur_type':'ur3','use_fake_hardware':'true','kinematics_params':str(calibration)}).toxml()
    srdf = xacro.process_file(str(Path(get_package_share_directory('ur_moveit_config'))/'srdf/ur.srdf.xacro'),
        mappings={'name':'ur','prefix':''}).toxml()
    (output/'diagnostic.urdf').write_text(xml)
    (output/'diagnostic.srdf').write_text(srdf)
    model = CeilingGeometry(xml, table['T_base_from_world'])
    offset = np.asarray(geometry['previously_confirmed']['magnet_center_xyz_m'])
    radius = geometry['previously_confirmed']['magnet_radius_m']
    box = geometry['provisional_whole_tool_envelope']
    corners = box_corners(box['min_xyz_m'], box['max_xyz_m'])
    q0 = np.asarray(state['actual_q'])
    def fk(q):return model.world_from_base@model.transforms(dict(zip(JOINTS,q)))['tool0']
    first = fk(q0)
    base_pose = model.transforms(dict(zip(JOINTS,q0)))['tool0']
    error_p = np.linalg.norm(base_pose[:3,3]+base_pose[:3,:3]@offset-state['actual_TCP_pose'][:3])
    error_r = (Rotation.from_matrix(base_pose[:3,:3]).inv()*Rotation.from_rotvec(state['actual_TCP_pose'][3:])).magnitude()
    if error_p > .00025 or error_r > np.deg2rad(.05):raise RuntimeError('Active TCP or calibration mismatch')
    sphere0 = first[:3,3]+first[:3,:3]@offset
    initial_axis = first[:3,:3]@np.array([0.,-1.,0.])
    initial_heading = float(np.degrees(np.arctan2(initial_axis[1],initial_axis[0])))
    target_yaw = 0. if diagnostic_only else yaw_to_positive_y(initial_axis)
    maximum_step = 0.
    def solve(target, previous):
        nonlocal maximum_step
        def residual(q):
            pose = fk(q)
            return np.r_[pose[:3,3]-target[:3,3],.1*Rotation.from_matrix(target[:3,:3].T@pose[:3,:3]).as_rotvec()]
        fit = least_squares(residual, previous,
            bounds=([-2*np.pi,-2*np.pi,-np.pi,-2*np.pi,-2*np.pi,-2*np.pi],
                    [2*np.pi,2*np.pi,np.pi,2*np.pi,2*np.pi,2*np.pi]),
            xtol=1e-11,ftol=1e-11,gtol=1e-11,max_nfev=150)
        residual_value = residual(fit.x)
        step = float(np.max(np.abs(fit.x-previous)))
        if np.linalg.norm(residual_value[:3]) > 1e-6 or np.linalg.norm(residual_value[3:]) > 1e-6 or step > np.deg2rad(2.):
            raise RuntimeError('IK or joint continuity failure')
        maximum_step = max(maximum_step,step)
        return fit.x
    lower = [q0]
    lowered = first.copy()
    lowered[2,3] -= lower_mm/1000.
    for depth in np.linspace(0.,lower_mm/1000.,int(np.ceil(lower_mm/.5))+1)[1:]:
        pose = first.copy();pose[2,3] -= depth
        lower.append(solve(pose,lower[-1]))
    yaw = [lower[-1]]
    for angle in np.linspace(0.,target_yaw,int(np.ceil(target_yaw/.5))+1)[1:]:
        yaw.append(solve(fixed_sphere_target(lowered,offset,np.deg2rad(angle)),yaw[-1]))
    descent = timed_phase(lower,1.,lower_seconds)
    descent_end = 1.+lower_seconds
    turn_start = descent_end+2.
    if diagnostic_only:
        points = [dict(descent[0],time_s=0.)]+descent+[dict(descent[-1],time_s=turn_start)]
        phases = {'lower':[1.,descent_end],'final_hold':[descent_end,turn_start]}
    else:
        turn = timed_phase(yaw,turn_start,45.)
        points = [dict(descent[0],time_s=0.)]+descent+turn+[dict(turn[-1],time_s=turn_start+47.)]
        phases = {'lower':[1.,descent_end],'hold':[descent_end,turn_start],
                  'yaw':[turn_start,turn_start+45.],'final_hold':[turn_start+45.,turn_start+47.]}
    draft = {'execution_allowed':False,'joint_names':JOINTS,'sample_period_s':.008,'points':points,
             'lower_mm':lower_mm,'target_world_heading_deg':initial_heading if diagnostic_only else 90.,
             'target_relative_yaw_deg':target_yaw,'diagnostic_only':diagnostic_only,'phases':phases,
             'automatic_return_to_original_height':False}
    draft_path = output/'timed_path_NOT_AUTHORIZED.json'
    draft_path.write_text(json.dumps(draft,indent=2))
    build = ROOT/'robot/offline_collision/build'
    sampled = subprocess.run([str(build/'sample_timed_trajectory'),str(draft_path)],capture_output=True,text=True,check=True,timeout=60)
    (output/'controller_interpolation.csv').write_text(sampled.stdout)
    rows = list(csv.DictReader(io.StringIO(sampled.stdout)))
    times = np.asarray([float(row['time_s']) for row in rows])
    qs = np.asarray([[float(row[f'q{i}']) for i in range(6)] for row in rows])
    vs = np.asarray([[float(row[f'v{i}']) for i in range(6)] for row in rows])
    accs = np.asarray([[float(row[f'a{i}']) for i in range(6)] for row in rows])
    if (len(rows) < 2 or abs(times[0]) > 1e-9 or abs(times[-1]-points[-1]['time_s']) > 1e-9
            or np.any(np.diff(times) <= 0) or np.max(np.diff(times)) > .008000002
            or not np.isfinite(np.r_[qs.ravel(),vs.ravel(),accs.ravel()]).all()):
        raise RuntimeError('Incomplete controller interpolation')
    max_interpolation_error = 0.
    segment = 0
    for i,t in enumerate(times):
        while segment+1 < len(points)-1 and t > points[segment+1]['time_s']:segment += 1
        expected = quintic(points[segment],points[segment+1],t)
        max_interpolation_error = max(max_interpolation_error,*(float(np.max(np.abs(a-b))) for a,b in zip(expected,[qs[i],vs[i],accs[i]])))
    peaks = np.max([derivative_peaks(a,b) for a,b in zip(points,points[1:])],axis=0)
    speed_limit,accel_limit = ((.2,.06) if diagnostic_only else (8.,10.))
    if max_interpolation_error > 1e-8 or peaks[0] > np.deg2rad(speed_limit) or peaks[1] > np.deg2rad(accel_limit):
        raise RuntimeError('Controller interpolation or timing limit failed')
    minima = {phase:{k:float('inf') for k in ['ceiling','left','right','table']}
              for phase in (['lower'] if diagnostic_only else ['lower','yaw'])}
    baseline_tops = None
    max_rise = 0.;max_xy_drift = 0.;max_yaw_drift = 0.;max_elevation = 0.
    expected_fk = []
    below_sphere = sphere0-np.array([0.,0.,lower_mm/1000.])
    for t,q in zip(times,qs):
        phase = 'lower' if diagnostic_only or t < turn_start else 'yaw'
        transforms = model.transforms(dict(zip(JOINTS,q)))
        expected_fk.append(transforms['tool0'])
        pose = model.world_from_base@transforms['tool0']
        sphere = pose[:3,3]+pose[:3,:3]@offset
        max_xy_drift = max(max_xy_drift,float(np.linalg.norm(sphere[:2]-sphere0[:2])))
        if phase == 'yaw':max_yaw_drift = max(max_yaw_drift,float(np.linalg.norm(sphere-below_sphere)))
        axis = pose[:3,:3]@np.array([0.,-1.,0.])
        max_elevation = max(max_elevation,float(abs(np.degrees(np.arctan2(axis[2],np.linalg.norm(axis[:2]))))))
        shapes = [(name,*surface_bounds(model.world_from_base@transforms[name]@origin,kind,shape))
                  for name,origin,kind,shape in model.surfaces]
        shapes += [('tool_envelope',*surface_bounds(pose,'box',corners)),('sphere',sphere-radius,sphere+radius)]
        tops = {name:float(high[2]) for name,low,high in shapes}
        if baseline_tops is None:baseline_tops = tops
        if phase == 'lower':max_rise = max(max_rise,max(tops[name]-baseline_tops[name] for name in tops))
        for name,low,high in shapes:
            gaps = {'ceiling':table['fixed_work_surface']['acrylic_bottom_world_z_m']-high[2],
                    'left':low[0]-side['left_inner_x_m'],'right':side['right_inner_x_m']-high[0],'table':low[2]}
            for boundary,value in gaps.items():minima[phase][boundary] = min(minima[phase][boundary],float(value*1000))
    collision_data = {'execution_allowed':False,'joint_names':JOINTS,'tool_geometry':geometry,
                      'samples':[{'q_rad':q.tolist()} for q in qs]}
    collision_path = output/'states_NOT_AUTHORIZED.json'
    collision_path.write_text(json.dumps(collision_data))
    collision = subprocess.run([str(build/'check_self_collision'),str(output/'diagnostic.urdf'),str(output/'diagnostic.srdf'),str(collision_path)],capture_output=True,text=True,check=True,timeout=60)
    (output/'self_collision.csv').write_text(collision.stdout)
    (output/'self_collision.log').write_text(collision.stderr)
    contacts = list(csv.DictReader(io.StringIO(collision.stdout)))
    if len(contacts) != len(rows):raise RuntimeError('Incomplete FCL check')
    collision_count = 0;bad_bounds = 0;fk_error = 0.
    for i,(row,expected) in enumerate(zip(contacts,expected_fk)):
        if int(row['index']) != i:raise RuntimeError('FCL order mismatch')
        collision_count += int(row['tool_collision']);bad_bounds += 1-int(row['joint_bounds_ok'])
        fk_error = max(fk_error,float(np.linalg.norm(np.array([float(row[k]) for k in ['fx','fy','fz']])-expected[:3,3])),
                       float((Rotation.from_quat([float(row[k]) for k in ['qx','qy','qz','qw']]).inv()*Rotation.from_matrix(expected[:3,:3])).magnitude()))
    final = fk(qs[-1]);shaft = final[:3,:3]@np.array([0.,-1.,0.])
    heading = float(np.degrees(np.arctan2(shaft[1],shaft[0])))
    yaw_clear = diagnostic_only or all(minima['yaw'][key] >= value for key, value in limits_mm.items())
    passed = (all(minima['lower'][key] >= value for key, value in limits_mm.items())
              and yaw_clear and max_rise <= .00002
              and max_xy_drift < .0001 and max_yaw_drift < .0001 and max_elevation < .1
              and abs(heading-(initial_heading if diagnostic_only else 90.)) < .001
              and np.linalg.norm(final[:3,3]+final[:3,:3]@offset-below_sphere)<1e-6
              and collision_count == 0 and bad_bounds == 0 and fk_error < 1e-8)
    report = {'utc':datetime.now(timezone.utc).isoformat(),'mode':'read_only_short_retreat_audit' if diagnostic_only else 'read_only_lower_then_yaw_audit',
              'execution_allowed':False,'motion_sent':False,'checks_pass_for_draft':bool(passed),
              'snapshot':state,'controller_version':version,'table_geometry':table,'side_geometry':side,'tool_geometry':geometry,
              'lower_mm':lower_mm,'diagnostic_only':diagnostic_only,'target_yaw_deg':target_yaw,
              'duration_s':points[-1]['time_s'],'samples':len(rows),
              'sphere_start_world_mm':(sphere0*1000).tolist(),'sphere_final_world_mm':(below_sphere*1000).tolist(),
              'clearance_limits_mm':limits_mm,
              'min_gaps_by_phase_mm':minima,'maximum_surface_rise_during_lowering_mm':max_rise*1000,
              'max_xy_drift_mm':max_xy_drift*1000,'max_sphere_drift_during_yaw_mm':max_yaw_drift*1000,
              'max_axis_elevation_deg':max_elevation,'final_shaft_heading_deg':heading,
              'max_joint_speed_deg_s':float(np.degrees(peaks[0])),'max_joint_acceleration_deg_s2':float(np.degrees(peaks[1])),
              'max_ik_step_deg':float(np.degrees(maximum_step)),'self_collision_samples':collision_count,'joint_bound_violations':bad_bounds,
              'controller_interpolation_max_error':max_interpolation_error,'cross_checker_fk_error':fk_error,
              'remaining_blockers':['actual_tool_envelope_and_fixed_obstacles_not_verified',
                                    'physical_plane_clearance_uncertainty','stopping_path_not_validated',
                                    'live_monitor_not_adapted_to_lower_then_yaw']}
    (output/'audit.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
    print(json.dumps({k:v for k,v in report.items() if k not in ['table_geometry','side_geometry','tool_geometry','snapshot']},ensure_ascii=False,indent=2),flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--lower-mm',type=float,default=20.)
    parser.add_argument('--diagnostic-only',action='store_true',help='Plan only a 1--3 mm descent, no yaw points')
    parser.add_argument('--lower-seconds',type=float,default=10.)
    args = parser.parse_args()
    plan(args.output,args.lower_mm,args.diagnostic_only,args.lower_seconds)
