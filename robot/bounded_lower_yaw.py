#!/usr/bin/env python3
"""One supervised stage of a checked downward retreat / horizontal yaw.

Default is read-only preflight. Never chains stages, changes TCP/safety or
starts the magnetic motor. User no-contact/clear-area reports are physical
assessments, NOT electronically sensed or safety-rated guarantees.
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
import time

import numpy as np
from scipy.spatial.transform import Rotation
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from trajectory_msgs.msg import JointTrajectoryPoint
from rcl_interfaces.srv import GetParameters

from preview_yaw_geometry import ROOT, JOINTS, box_corners, read_state
from preview_yaw_clearance import surface_bounds
from run_checked_yaw import dashboard, seconds
from ur3_magnetic_control.clearance_policy import clearance_limits_mm, load_clearance_limits_m
from ur3_magnetic_control.ceiling_geometry import CeilingGeometry
from record_rtde_outputs import OutputRecorder


# Measured-velocity abort thresholds, not requested trajectory speeds or UR
# safety settings. The user requested 10 deg/s for the lowering monitor.
# Planned speed/acceleration bounds and all geometric/tracking checks remain.
MEASURED_JOINT_SPEED_LIMIT_DEG_S = {'lower': 10., 'yaw': 9.}


class StageGeometry:
    def __init__(self, folder, stage):
        if stage not in ('lower','yaw'):raise ValueError('Unknown stage')
        self.stage = stage
        self.folder = folder
        self.audit = json.loads((folder/'audit.json').read_text())
        self.draft = json.loads((folder/'timed_path_NOT_AUTHORIZED.json').read_text())
        self.diagnostic_only = self.audit.get('diagnostic_only',False)
        self.lower_m = float(self.audit['lower_mm'])/1000.
        if (not self.audit['checks_pass_for_draft'] or
                (self.diagnostic_only and (stage != 'lower' or not .001 <= self.lower_m <= .003)) or
                (not self.diagnostic_only and not .010 <= self.lower_m <= .030)):
            raise RuntimeError('Requires a checked short diagnostic or a checked 10--30 mm retreat plan')
        if self.draft.get('diagnostic_only',False) != self.diagnostic_only or self.draft['lower_mm'] != self.audit['lower_mm']:
            raise RuntimeError('Audit and trajectory scope disagree')
        self.table,self.side,self.tool = [self.audit[k] for k in ('table_geometry','side_geometry','tool_geometry')]
        self.gap_limits = load_clearance_limits_m(ROOT)
        if self.audit.get('clearance_limits_mm') != clearance_limits_mm(ROOT):
            raise RuntimeError('Audit does not use the current global clearance policy')
        for data,filename in [(self.table,'table_world_calibration.yaml'),(self.side,'acrylic_side_boundaries.yaml'),
                              (self.tool,'magnet_tool_geometry_draft.yaml')]:
            if data != yaml.safe_load((ROOT/'config'/filename).read_text()):
                raise RuntimeError('Configuration changed; replan required')
        self.model = CeilingGeometry((folder/'diagnostic.urdf').read_text(),self.table['T_base_from_world'])
        self.offset = np.asarray(self.tool['previously_confirmed']['magnet_center_xyz_m'])
        box = self.tool['provisional_whole_tool_envelope']
        self.corners = box_corners(box['min_xyz_m'],box['max_xyz_m'])
        self.radius = self.tool['previously_confirmed']['magnet_radius_m']
        self.bottom = self.table['fixed_work_surface']['acrylic_bottom_world_z_m']
        self.original_q = np.asarray(self.audit['snapshot']['actual_q'])
        self.original_pose = self.pose(self.original_q)
        self.original_sphere = self.sphere(self.original_pose)
        self.original_tops = self.bounds(self.original_q)[1]
        self.target_sphere = self.original_sphere-np.array([0.,0.,self.lower_m])
        source_points = self.draft['points']
        turn_start = self.draft['phases'].get('yaw',[source_points[-1]['time_s']])[0]
        self.points = ([p for p in source_points if p['time_s'] <= turn_start] if stage == 'lower'
                       else [dict(p,time_s=round(p['time_s']-turn_start,9)) for p in source_points if p['time_s'] >= turn_start])
        self.yaw_scope_review_passed = False
        self.start_q = np.asarray(self.points[0]['q_rad'])
        self.end_q = np.asarray(self.points[-1]['q_rad'])
        self.path_tolerance = .0015 if stage == 'lower' else .006
        if self.draft['joint_names'] != JOINTS or self.points[0]['time_s'] != 0.:
            raise RuntimeError('Invalid stage points')
        for point in [self.points[0],self.points[-1]]:
            if np.max(np.abs(point['velocity_rad_s'])) > 1e-10 or np.max(np.abs(point['acceleration_rad_s2'])) > 1e-10:
                raise RuntimeError('Stage must start and end at rest')

    def pose(self,q):return self.model.world_from_base@self.model.transforms(dict(zip(JOINTS,q)))['tool0']
    def sphere(self,pose):return pose[:3,3]+pose[:3,:3]@self.offset

    def bounds(self,q):
        transforms = self.model.transforms(dict(zip(JOINTS,q)))
        shapes = [(name,*surface_bounds(self.model.world_from_base@transforms[name]@origin,kind,shape))
                  for name,origin,kind,shape in self.model.surfaces]
        pose = self.model.world_from_base@transforms['tool0']
        sphere = self.sphere(pose)
        shapes += [('tool_envelope',*surface_bounds(pose,'box',self.corners)),('sphere',sphere-self.radius,sphere+self.radius)]
        gaps = dict.fromkeys(['ceiling','left','right','table'],float('inf'))
        tops = {}
        for name,low,high in shapes:
            tops[name] = float(high[2])
            values = {'ceiling':self.bottom-high[2],'left':low[0]-self.side['left_inner_x_m'],
                      'right':self.side['right_inner_x_m']-high[0],'table':low[2]}
            for key,value in values.items():gaps[key] = min(gaps[key],float(value))
        return gaps,tops,pose

    def inspect(self,q):
        if np.asarray(q).shape != (6,) or not np.isfinite(q).all():raise RuntimeError('Invalid joints')
        gaps,tops,pose = self.bounds(q)
        if any(gaps[key] < limit for key, limit in self.gap_limits.items()):
            raise RuntimeError('Live geometric clearance violation: '+str(gaps))
        sphere = self.sphere(pose)
        delta = sphere-self.original_sphere
        axis = pose[:3,:3]@np.array([0.,-1.,0.])
        elevation = float(np.degrees(np.arctan2(axis[2],np.linalg.norm(axis[:2]))))
        heading = float(np.degrees(np.arctan2(axis[1],axis[0])))
        if abs(elevation) > .3 or np.linalg.norm(delta[:2]) > .0007:
            raise RuntimeError('Shaft tilt or horizontal drift exceeds bound')
        if self.stage == 'lower':
            angle = (Rotation.from_matrix(self.original_pose[:3,:3]).inv()*Rotation.from_matrix(pose[:3,:3])).magnitude()
            rise = max(tops[name]-self.original_tops[name] for name in tops)
            if angle > np.deg2rad(.2) or rise > .00025 or not -self.lower_m-.0007 <= delta[2] <= .00025:
                raise RuntimeError('Retreat changed orientation, raised a surface or left downward corridor')
        elif np.linalg.norm(sphere-self.target_sphere) > .0007:
            raise RuntimeError('Yaw moved sphere away from lowered center')
        return {'gaps_mm':{k:v*1000 for k,v in gaps.items()},'sphere_world_mm':(sphere*1000).tolist(),
                'heading_deg':heading,'elevation_deg':elevation,'drop_mm':float(-delta[2]*1000)}

    def offline_checks(self,report_dir):
        sampler = ROOT/'robot/offline_collision/build/sample_timed_trajectory'
        original = self.folder/'timed_path_NOT_AUTHORIZED.json'
        sampled = subprocess.run([str(sampler),str(original)],capture_output=True,text=True,check=True,timeout=60)
        if sampled.stdout != (self.folder/'controller_interpolation.csv').read_text():
            raise RuntimeError('Original audited trajectory was altered')
        subset = {'joint_names':JOINTS,'sample_period_s':.008,'points':self.points,'execution_allowed':False}
        path = report_dir/'stage_checked_input.json';path.write_text(json.dumps(subset))
        result = subprocess.run([str(sampler),str(path)],capture_output=True,text=True,check=True,timeout=60)
        (report_dir/'stage_interpolation.csv').write_text(result.stdout)
        rows = list(csv.DictReader(io.StringIO(result.stdout)))
        states = [];stop_states = [];min_stop = 1.;max_stop_rise = 0.;last_stop = -1.
        for row in rows:
            t = float(row['time_s'])
            q = np.array([float(row[f'q{i}']) for i in range(6)])
            v = np.array([float(row[f'v{i}']) for i in range(6)])
            a = np.array([float(row[f'a{i}']) for i in range(6)])
            self.inspect(q)
            planned_limit=.2 if self.diagnostic_only else (1.1 if self.stage == 'lower' else 8.)
            if np.max(np.abs(v)) > np.deg2rad(planned_limit):
                raise RuntimeError('Stage speed exceeds bound')
            if np.max(np.abs(a)) > np.deg2rad(10.):raise RuntimeError('Stage acceleration exceeds bound')
            states.append({'q_rad':q.tolist()})
            if t-last_stop < .1:continue
            last_stop = t
            # Sensitivity test, not a certified stopping-distance guarantee:
            # extrapolation up to 200 ms then independent-joint deceleration.
            # Installed URCap script uses stopj(4.0); include slower 1.0 too.
            for latency in [0.,.1,.2]:
                for deceleration in [1.,4.]:
                    for fraction in [0.,.5,1.]:
                        duration = np.abs(v)/deceleration*fraction
                        stopped = q+v*latency+v*duration-np.sign(v)*deceleration*duration**2/2
                        gaps,tops,_ = self.bounds(stopped)
                        min_stop = min(min_stop,gaps['ceiling'])
                        max_stop_rise = max(max_stop_rise,max(tops[name]-self.original_tops[name] for name in tops))
                        if any(gaps[key] < limit for key, limit in self.gap_limits.items()):
                            raise RuntimeError('Stop sensitivity clearance failed')
                        stop_states.append({'q_rad':stopped.tolist()})
        if self.stage == 'lower' and max_stop_rise > .00002:raise RuntimeError('Retreat stop sensitivity raises a surface')
        # Check the complete stage and the stop states against the same FCL model.
        collision = {'joint_names':JOINTS,'tool_geometry':self.tool,'samples':states+stop_states,'execution_allowed':False}
        collision_path = report_dir/'collision_input.json';collision_path.write_text(json.dumps(collision))
        checked = subprocess.run([str(ROOT/'robot/offline_collision/build/check_self_collision'),
            str(self.folder/'diagnostic.urdf'),str(self.folder/'diagnostic.srdf'),str(collision_path)],
            capture_output=True,text=True,check=True,timeout=60)
        (report_dir/'collision.csv').write_text(checked.stdout)
        contacts = list(csv.DictReader(io.StringIO(checked.stdout)))
        if len(contacts) != len(collision['samples']) or any(r['tool_collision'] != '0' or r['joint_bounds_ok'] != '1' for r in contacts):
            raise RuntimeError('Stage/stop FCL or joint limit failure')
        review = None
        if self.stage == 'yaw':
            # Keep the original estimated-tool metadata unchanged. Independently
            # check a larger box and the *same* nominal/stop samples. The basis
            # is the user's motor/bracket dimensions and photographs, not an
            # assertion that an exact physical tool model was measured.
            enlarged = copy.deepcopy(collision)
            envelope = enlarged['tool_geometry']['provisional_whole_tool_envelope']
            envelope['min_xyz_m'] = (np.asarray(envelope['min_xyz_m'])-.005).tolist()
            envelope['max_xyz_m'] = (np.asarray(envelope['max_xyz_m'])+.005).tolist()
            padded_corners=box_corners(envelope['min_xyz_m'],envelope['max_xyz_m'])
            minimum_padded_gaps=dict.fromkeys(['ceiling','left','right','table'],float('inf'))
            for sample in enlarged['samples']:
                pose=self.pose(sample['q_rad'])
                low,high=surface_bounds(pose,'box',padded_corners)
                gaps={'ceiling':self.bottom-high[2],'left':low[0]-self.side['left_inner_x_m'],
                      'right':self.side['right_inner_x_m']-high[0],'table':low[2]}
                for key,value in gaps.items():minimum_padded_gaps[key]=min(minimum_padded_gaps[key],float(value))
            if any(minimum_padded_gaps[key] < limit for key, limit in self.gap_limits.items()):
                raise RuntimeError('Enlarged tool envelope clearance failed')
            padded_input=report_dir/'enlarged_tool_collision_input.json'
            padded_input.write_text(json.dumps(enlarged))
            enlarged_check=subprocess.run([str(ROOT/'robot/offline_collision/build/check_self_collision'),
                str(self.folder/'diagnostic.urdf'),str(self.folder/'diagnostic.srdf'),str(padded_input)],
                capture_output=True,text=True,check=True,timeout=60)
            (report_dir/'enlarged_tool_collision.csv').write_text(enlarged_check.stdout)
            padded_rows=list(csv.DictReader(io.StringIO(enlarged_check.stdout)))
            if len(padded_rows)!=len(enlarged['samples']) or any(r['tool_collision']!='0' or r['joint_bounds_ok']!='1' for r in padded_rows):
                raise RuntimeError('Enlarged tool FCL check failed')
            # Rotating strictly about world Z preserves the height of every
            # rigid tool point, independently of its exact shape. Starting
            # from a successfully verified retreat gives additional clearance.
            nominal_yaw_gap=self.audit['min_gaps_by_phase_mm']['yaw']['ceiling']
            if (nominal_yaw_gap < self.gap_limits['ceiling'] * 1000.0
                    or self.audit['max_sphere_drift_during_yaw_mm']>.1):
                raise RuntimeError('Insufficient lowered yaw margin')
            self.yaw_scope_review_passed=True
            review={'passed':True,'tool_padding_per_face_mm':5.,
                    'minimum_padded_tool_gaps_mm':{k:v*1000 for k,v in minimum_padded_gaps.items()},
                    'height_basis':'fixed sphere, horizontal world-Z yaw after downward retreat',
                    'physical_basis':'user-reported motor/bracket dimensions, photos, clear area/cables and initial no-contact',
                    'exact_tool_geometry_physically_measured':False,'not_safety_rated':True}
        return {'stage_samples':len(states),'stop_sensitivity_samples':len(stop_states),'yaw_scope_review':review,
                'clearance_limits_mm':clearance_limits_mm(ROOT),
                'minimum_stop_model_ceiling_gap_mm':min_stop*1000,
                'maximum_stop_model_surface_rise_mm':max_stop_rise*1000,
                'source_sha256':hashlib.sha256(original.read_bytes()).hexdigest(),
                'physical_assumptions':['user confirms initial highest point is not touching acrylic',
                    'user confirms motor stopped, work area and cables clear',
                    'fixed platform has previously measured single height',
                    'tool box remains an estimate based on supplied dimensions/photos'],
                'not_safety_rated':True}


class StageRunner(Node):
    def __init__(self,guard,report,recorder=None):
        super().__init__('supervised_lower_or_yaw')
        self.guard,self.report = guard,report
        self.recorder = recorder
        self.q=self.qd=self.scale=None
        self.joint_header_stamp=None
        self.received=self.scale_received=self.feedback_received=0.
        self.feedback_error=0.;self.program=False;self.active=False;self.goal_handle=None
        self.latest_action_feedback=None
        self.samples=[]
        self.create_subscription(JointState,'/joint_states',self.joints,1)
        self.create_subscription(Float64,'/speed_scaling_state_broadcaster/speed_scaling',self.scaling,1)
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool,'/io_and_status_controller/robot_program_running',lambda m:setattr(self,'program',m.data),qos)
        self.action=ActionClient(self,FollowJointTrajectory,'/scaled_joint_trajectory_controller/follow_joint_trajectory')

    def joints(self,m):
        p=dict(zip(m.name,m.position));v=dict(zip(m.name,m.velocity))
        if not all(n in p and n in v for n in JOINTS):return
        self.q=np.array([p[n] for n in JOINTS]);self.qd=np.array([v[n] for n in JOINTS]);self.received=time.monotonic()
        self.joint_header_stamp={'sec':m.header.stamp.sec,'nanosec':m.header.stamp.nanosec}
    def scaling(self,m):self.scale=float(m.data);self.scale_received=time.monotonic()
    def feedback(self,m):
        v=m.feedback.error.positions
        self.feedback_error=max(abs(x) for x in v) if len(v)==6 and np.isfinite(v).all() else float('inf')
        self.feedback_received=time.monotonic()
        self.latest_action_feedback={key:{field:list(getattr(getattr(m.feedback,key),field))
            for field in ['positions','velocities','accelerations']} for key in ['desired','actual','error']}
    def wait(self,future,timeout=3.):
        end=time.monotonic()+timeout
        while not future.done() and time.monotonic()<end:rclpy.spin_once(self,timeout_sec=.005)
        if not future.done() or future.result() is None:raise RuntimeError('ROS request timeout')
        return future.result()
    def fresh(self):
        initial=self.received;end=time.monotonic()+1.
        while self.received==initial and time.monotonic()<end:rclpy.spin_once(self,timeout_sec=.005)
        return self.live()
    def live(self):
        now=time.monotonic()
        if self.q is None or now-self.received>.12 or now-self.scale_received>.2:raise RuntimeError('Feedback stale')
        if not self.program or self.scale is None or not 0.<self.scale<=100.1:raise RuntimeError('Program/speed scaling not ready')
        if self.qd is None or np.asarray(self.qd).shape!=(6,) or not np.isfinite(self.qd).all():
            raise RuntimeError('Invalid measured joint velocity: '+str(self.qd))
        limit=MEASURED_JOINT_SPEED_LIMIT_DEG_S[self.guard.stage]
        if np.max(np.abs(self.qd))>np.deg2rad(limit):
            index=int(np.argmax(np.abs(self.qd)))
            raise RuntimeError(f'Measured {JOINTS[index]} speed {np.degrees(self.qd[index]):.6f} deg/s exceeds {limit:.3f} deg/s limit')
        if self.active and now-self.sent_at>.5:
            if now-self.feedback_received>.25 or self.feedback_error>self.guard.path_tolerance:
                raise RuntimeError('Action feedback stale or tracking error exceeds limit')
            recorder=getattr(self,'recorder',None)
            if recorder is not None:
                recorder.check_fresh(.12)
        return self.guard.inspect(self.q)
    def preflight(self):
        deadline=time.monotonic()+5.
        while time.monotonic()<deadline:
            rclpy.spin_once(self,timeout_sec=.01)
            if self.q is not None and self.scale is not None and self.program:break
        self.fresh()
        if np.max(np.abs(self.q-self.guard.start_q))>.0005 or np.max(np.abs(self.qd))>.002:
            raise RuntimeError('Start mismatch or not stationary; no automatic repositioning')
        before,version=read_state()
        if np.max(np.abs(np.asarray(before['actual_q'])-self.guard.start_q))>.0005:raise RuntimeError('RTDE start mismatch')
        pose=self.guard.model.transforms(dict(zip(JOINTS,before['actual_q'])))['tool0']
        position_error=np.linalg.norm(pose[:3,3]+pose[:3,:3]@self.guard.offset-before['actual_TCP_pose'][:3])
        rotation_error=(Rotation.from_matrix(pose[:3,:3]).inv()*Rotation.from_rotvec(before['actual_TCP_pose'][3:])).magnitude()
        if position_error>.00025 or rotation_error>np.deg2rad(.05):raise RuntimeError('Active TCP/model mismatch')
        for name,key,wanted in [('/scaled_joint_trajectory_controller','interpolation_method','splines'),('/controller_manager','update_rate',125)]:
            client=self.create_client(GetParameters,name+'/get_parameters')
            if not client.wait_for_service(timeout_sec=2.):raise RuntimeError('Controller service unavailable')
            req=GetParameters.Request();req.names=[key]
            value=self.wait(client.call_async(req)).values[0]
            if (value.string_value if isinstance(wanted,str) else value.integer_value)!=wanted:raise RuntimeError('Controller settings differ')
        status={k:dashboard(k) for k in ['robotmode','safetystatus','running']}
        if status != {'robotmode':'Robotmode: RUNNING','safetystatus':'Safetystatus: NORMAL','running':'Program running: true'}:
            raise RuntimeError('Robot not ready: '+str(status))
        self.report.update(before=before,preflight=self.fresh(),dashboard=status,speed_scaling_percent=self.scale)
        print('PREFLIGHT '+json.dumps(self.report,ensure_ascii=False),flush=True)
    def stop(self):
        if self.goal_handle:
            try:self.goal_handle.cancel_goal_async()
            except Exception:pass
        try:self.report['stop_reply']=dashboard('stop')
        except Exception as error:self.report['stop_error']=str(error)
    def execute(self):
        if self.guard.stage=='yaw' and not self.guard.yaw_scope_review_passed:
            raise RuntimeError('Yaw requires successful enlarged-envelope and stopping review')
        if not self.action.wait_for_server(timeout_sec=2.):raise RuntimeError('Controller action unavailable')
        self.fresh()
        if np.max(np.abs(self.q-self.guard.start_q))>.0005 or np.max(np.abs(self.qd))>.002:raise RuntimeError('Start changed')
        goal=FollowJointTrajectory.Goal();goal.trajectory.joint_names=JOINTS
        for item in self.guard.points:
            p=JointTrajectoryPoint(positions=item['q_rad'],velocities=item['velocity_rad_s'],accelerations=item['acceleration_rad_s2'])
            p.time_from_start=seconds(item['time_s']);goal.trajectory.points.append(p)
        for name in JOINTS:
            goal.path_tolerance.append(JointTolerance(name=name,position=self.guard.path_tolerance))
            goal.goal_tolerance.append(JointTolerance(name=name,position=.001,velocity=.002))
        goal.goal_time_tolerance=seconds(3.)
        self.sent_at=time.monotonic();self.report['motion_sent']=True;self.active=True
        try:
            self.goal_handle=self.wait(self.action.send_goal_async(goal,feedback_callback=self.feedback))
            if not self.goal_handle.accepted:raise RuntimeError('Stage rejected')
            future=self.goal_handle.get_result_async();last_seen=0.;last_log=0.
            deadline=time.monotonic()+max(30.,self.guard.points[-1]['time_s']*1.5/(self.scale/100.)+10.)
            while not future.done():
                rclpy.spin_once(self,timeout_sec=.002)
                if time.monotonic()>deadline:raise RuntimeError('Execution deadline exceeded')
                checked=self.live()
                if self.received!=last_seen:
                    last_seen=self.received
                    self.samples.append({'elapsed_s':time.monotonic()-self.sent_at,'q_rad':self.q.tolist(),
                        'qd_rad_s':self.qd.tolist(),'joint_header_stamp':self.joint_header_stamp,
                        'action_feedback':self.latest_action_feedback,**checked})
                    if time.monotonic()-last_log>2.:
                        print('MOVING '+json.dumps(self.samples[-1]),flush=True);last_log=time.monotonic()
            result=future.result()
            if result is None or result.status!=4 or result.result.error_code!=0:raise RuntimeError('Stage failed: '+str(result))
            self.active=False
            until=time.monotonic()+1.
            while time.monotonic()<until:rclpy.spin_once(self,timeout_sec=.005);self.live()
            after,_=read_state();checked=self.guard.inspect(np.asarray(after['actual_q']))
            world=self.guard.model.world_from_base
            delta=world[:3,:3]@(np.asarray(after['actual_TCP_pose'][:3])-self.report['before']['actual_TCP_pose'][:3])
            if np.max(np.abs(np.asarray(after['actual_q'])-self.guard.end_q))>.001:raise RuntimeError('Final joint target mismatch')
            if self.guard.stage=='lower':
                if np.linalg.norm(delta-np.array([0.,0.,-self.guard.lower_m]))>.0003:
                    raise RuntimeError('Measured descent differs from requested distance')
            else:
                if abs(checked['heading_deg']-90.)>.15 or np.linalg.norm(delta)>.0005:raise RuntimeError('Final yaw or sphere position mismatch')
            self.report.update(completed=True,after=after,final=checked,measured_world_displacement_mm=(delta*1000).tolist())
            print('VERIFIED '+json.dumps(self.report,ensure_ascii=False),flush=True)
        except BaseException as error:
            # Persist the triggering sample, not just the last sample which
            # passed the monitor. Do not loosen a limit following an abort.
            self.report.update(completed=False,error=str(error),failure_feedback={
                'elapsed_s':time.monotonic()-self.sent_at,
                'q_rad':None if self.q is None else self.q.tolist(),
                'qd_rad_s':None if self.qd is None else self.qd.tolist(),
                'joint_header_stamp':self.joint_header_stamp,
                'joint_sample_age_s':time.monotonic()-self.received,
                'action_feedback':self.latest_action_feedback})
            if self.recorder is not None:self.report['failure_direct_rtde']=self.recorder.latest
            self.stop();raise


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--audit-directory',type=Path,required=True)
    parser.add_argument('--stage',choices=['lower','yaw'],required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--execute',action='store_true')
    parser.add_argument('--onsite-ready',action='store_true')
    parser.add_argument('--initial-no-contact',action='store_true')
    args=parser.parse_args()
    if args.execute and not(args.onsite_ready and args.initial_no_contact):raise SystemExit('User readiness/no-contact confirmation required')
    args.output.mkdir(parents=True,exist_ok=False)
    guard=StageGeometry(args.audit_directory.resolve(),args.stage)
    report={'utc':datetime.now(timezone.utc).isoformat(),'stage':args.stage,'motion_sent':False,
            'initial_no_contact_user_confirmed':args.initial_no_contact,'onsite_ready_user_confirmed':args.onsite_ready,
            'no_automatic_next_stage':True,'no_motor_commands':True,
            'measured_joint_speed_limit_deg_s':MEASURED_JOINT_SPEED_LIMIT_DEG_S[args.stage]}
    report['offline_checks']=guard.offline_checks(args.output)
    rclpy.init();node=None;recorder=None
    try:
        node=StageRunner(guard,report);node.preflight()
        if args.execute:
            recorder=OutputRecorder(args.output/'direct_rtde_outputs.jsonl')
            recorder.start();node.recorder=recorder;node.fresh();node.execute()
    finally:
        if recorder:
            recorder.close();report['direct_rtde_recording']=recorder.summary()
        if node:report['live_samples']=node.samples;node.destroy_node()
        (args.output/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
        rclpy.shutdown()
