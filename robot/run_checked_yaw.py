#!/usr/bin/env python3
"""Run one explicitly authorized, offline-audited +Y yaw. Default: preflight only.

No URScript, motor I/O, safety unlocking, payload/TCP changes or automatic retry.
Human clearance assessment is required; geometric checks are not safety-rated.
"""
import argparse
import csv
from datetime import datetime, timezone
import io
import json
from pathlib import Path
import socket
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
from builtin_interfaces.msg import Duration
from rcl_interfaces.srv import GetParameters

from preview_yaw_geometry import ROOT, JOINTS, box_corners, read_state
from preview_yaw_clearance import surface_bounds
from ur3_magnetic_control.clearance_policy import clearance_limits_mm, load_clearance_limits_m
from ur3_magnetic_control.ceiling_geometry import CeilingGeometry


def dashboard(command):
    if command not in ['robotmode', 'safetystatus', 'running', 'stop']:
        raise ValueError('Dashboard command outside scope')
    with socket.create_connection(('192.168.56.101', 29999), timeout=2) as sock:
        stream = sock.makefile('rwb', buffering=0)
        stream.readline()
        stream.write((command+'\n').encode())
        return stream.readline().decode().strip()


def seconds(value):
    ns = int(round(value*1e9))
    return Duration(sec=ns//1000000000, nanosec=ns % 1000000000)


def unresolved_physical_checks(audit, tool):
    checks = [item for item in audit.get('remaining_blockers', [])
              if item in ('physical_tool_envelope_and_posts_not_fully_verified',
                          'physical_ceiling_clearance_and_calibration_error',
                          'feedback_monitor_and_cancellation_stopping_path')]
    if not tool['provisional_whole_tool_envelope'].get('verified_encloses_all_rigid_parts', False):
        checks.append('whole_tool_envelope_not_physically_verified')
    return checks


class GeometryGuard:
    def __init__(self, folder):
        self.source = json.loads((folder/'fresh_geometry/summary.json').read_text())
        self.gap_limits = load_clearance_limits_m(ROOT)
        if self.source.get('clearance_limits_mm') != clearance_limits_mm(ROOT):
            raise RuntimeError('Geometry audit does not use the current global clearance policy')
        self.table = yaml.safe_load((ROOT/'config/table_world_calibration.yaml').read_text())
        self.model = CeilingGeometry((folder/'fresh_geometry/diagnostic.urdf').read_text(), self.table['T_base_from_world'])
        self.side = self.source['side_boundaries']
        self.tool = self.source['tool_geometry']
        if self.side != yaml.safe_load((ROOT/'config/acrylic_side_boundaries.yaml').read_text()):
            raise RuntimeError('Side geometry changed after audit')
        if self.tool != yaml.safe_load((ROOT/'config/magnet_tool_geometry_draft.yaml').read_text()):
            raise RuntimeError('Tool geometry changed after audit')
        box = self.tool['provisional_whole_tool_envelope']
        self.corners = box_corners(box['min_xyz_m'], box['max_xyz_m'])
        self.offset = np.asarray(self.tool['previously_confirmed']['magnet_center_xyz_m'])
        self.radius = self.tool['previously_confirmed']['magnet_radius_m']
        self.bottom = self.table['fixed_work_surface']['acrylic_bottom_world_z_m']
        self.start_q = np.asarray(self.source['snapshot']['actual_q'])
        pose = self.fk(self.start_q)
        self.sphere_start = pose[:3, 3]+pose[:3, :3]@self.offset

    def fk(self, q):
        return self.model.world_from_base @ self.model.transforms(dict(zip(JOINTS, q)))['tool0']

    def inspect(self, q):
        if np.asarray(q).shape != (6,) or not np.isfinite(q).all():
            raise RuntimeError('Invalid joints')
        transforms = self.model.transforms(dict(zip(JOINTS, q)))
        shapes = []
        for name, origin, kind, shape in self.model.surfaces:
            low, high = surface_bounds(self.model.world_from_base@transforms[name]@origin, kind, shape)
            shapes.append((name, low, high))
        tool = self.model.world_from_base@transforms['tool0']
        low, high = surface_bounds(tool, 'box', self.corners)
        sphere = tool[:3, 3]+tool[:3, :3]@self.offset
        shapes += [('tool_envelope', low, high), ('sphere', sphere-self.radius, sphere+self.radius)]
        gaps = {'ceiling': float('inf'), 'left': float('inf'), 'right': float('inf'), 'table': float('inf')}
        for name, low, high in shapes:
            candidates = {'ceiling': self.bottom-high[2], 'left': low[0]-self.side['left_inner_x_m'],
                          'right': self.side['right_inner_x_m']-high[0], 'table': low[2]}
            for boundary, gap in candidates.items():
                gaps[boundary] = min(gaps[boundary], float(gap))
                if gap < self.gap_limits[boundary]:
                    raise RuntimeError(
                        f'{name}: {boundary} gap below global policy '
                        f'({gap*1000:.3f} mm < {self.gap_limits[boundary]*1000:.3f} mm)'
                    )
        axis = tool[:3, :3]@np.array([0., -1., 0.])
        elevation = np.degrees(np.arctan2(axis[2], np.linalg.norm(axis[:2])))
        drift = np.linalg.norm(sphere-self.sphere_start)
        if abs(elevation) > .3 or drift > .001:
            raise RuntimeError(f'Tool path deviation: elevation={elevation:.3f} deg, sphere drift={drift*1000:.3f} mm')
        return {'minimum_gaps_mm': {key:value*1000 for key,value in gaps.items()},
                'shaft_heading_deg': float(np.degrees(np.arctan2(axis[1], axis[0]))),
                'shaft_elevation_deg': float(elevation), 'sphere_drift_mm': float(drift*1000)}


class Runner(Node):
    def __init__(self, folder):
        super().__init__('bounded_world_y_yaw')
        self.folder = folder
        self.draft = json.loads((folder/'timed_pilot_DRAFT_NOT_AUTHORIZED.json').read_text())
        self.audit = json.loads((folder/'pilot_audit.json').read_text())
        if not self.audit['checks_pass_for_draft'] or not self.audit.get('full_turn'):
            raise RuntimeError('A successful full-turn audit is required')
        if not 0 < self.draft['target_relative_world_yaw_deg'] <= 95. or self.draft['joint_names'] != JOINTS:
            raise RuntimeError('Trajectory outside this operation')
        if self.audit['analytic_max_joint_speed_deg_s'] > 8 or self.audit['sampled_max_tool_angular_speed_deg_s'] > 5:
            raise RuntimeError('Speed outside bounded operation')
        self.guard = GeometryGuard(folder)
        self.q = self.qd = None
        self.received = self.scale_received = self.feedback_received = 0.
        self.scale = None
        self.program = False
        self.feedback_error = 0.
        self.goal_handle = None
        self.samples = []
        self.goal_sent = False
        self.monitor_action_feedback = False
        self.report = {'motion_sent': False, 'utc': datetime.now(timezone.utc).isoformat(),
                       'source_audit': str(folder), 'onsite_clearance_and_motor_stop': 'user reported',
                       'hardware_safety_is_not_bypassed': True}
        self.report['unresolved_physical_checks'] = unresolved_physical_checks(self.audit, self.guard.tool)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(JointState, '/joint_states', self.on_joints, 1)
        self.create_subscription(Bool, '/io_and_status_controller/robot_program_running',
                                 lambda value: setattr(self, 'program', value.data), qos)
        self.create_subscription(Float64, '/speed_scaling_state_broadcaster/speed_scaling', self.on_scale, 1)
        self.action = ActionClient(self, FollowJointTrajectory, '/scaled_joint_trajectory_controller/follow_joint_trajectory')

    def on_joints(self, message):
        mapping = dict(zip(message.name, message.position))
        rates = dict(zip(message.name, message.velocity))
        if not all(name in mapping and name in rates for name in JOINTS):return
        self.q = np.asarray([mapping[name] for name in JOINTS])
        self.qd = np.asarray([rates[name] for name in JOINTS])
        self.received = time.monotonic()

    def on_scale(self, message):
        self.scale, self.scale_received = float(message.data), time.monotonic()

    def on_feedback(self, message):
        values = message.feedback.error.positions
        self.feedback_error = max(abs(value) for value in values) if values else float('inf')
        self.feedback_received = time.monotonic()

    def wait(self, future, timeout=5.):
        deadline = time.monotonic()+timeout
        while not future.done() and time.monotonic() < deadline:rclpy.spin_once(self, timeout_sec=.01)
        if not future.done() or future.result() is None:raise RuntimeError('ROS request timeout')
        return future.result()

    def check_live(self):
        now = time.monotonic()
        if self.q is None or now-self.received > .12:raise RuntimeError('Joint feedback stale')
        if self.scale is None or now-self.scale_received > .2:raise RuntimeError('Speed scaling stale')
        if not self.program:raise RuntimeError('External Control not running')
        if not 0 < self.scale <= 100.1:raise RuntimeError('Speed scaling zero/invalid')
        if self.qd is None or np.asarray(self.qd).shape != (6,) or not np.isfinite(self.qd).all():
            raise RuntimeError('Invalid joint velocity feedback')
        if np.max(np.abs(self.qd)) > np.deg2rad(9.):raise RuntimeError('Measured joint speed above 9 deg/s')
        if self.monitor_action_feedback and now-self.sent_at > .5:
            if now-self.feedback_received > .25:raise RuntimeError('Action feedback stale')
            if self.feedback_error > .003:raise RuntimeError('Joint tracking error exceeds 0.003 rad')
        return self.guard.inspect(self.q)

    def preflight(self):
        # Re-sample the exact draft with the installed controller and compare
        # every byte to the audited artifact. Any edited timing/point fails.
        sampler = ROOT/'robot/offline_collision/build/sample_timed_trajectory'
        result = subprocess.run([str(sampler), str(self.folder/'timed_pilot_DRAFT_NOT_AUTHORIZED.json')],
            capture_output=True, text=True, check=True, timeout=60)
        if result.stdout != (self.folder/'controller_interpolation.csv').read_text():
            raise RuntimeError('Trajectory changed since audit')
        deadline = time.monotonic()+5.
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.05)
            if self.q is not None and self.scale is not None and self.program:break
        before = self.check_live()
        if np.max(np.abs(self.q-self.guard.start_q)) > .0005 or np.max(np.abs(self.qd)) > .002:
            raise RuntimeError('Start changed or robot not stationary; replan required')
        actual, version = read_state()
        flange = self.guard.model.transforms(dict(zip(JOINTS, actual['actual_q'])))['tool0']
        error = np.linalg.norm(flange[:3, 3]+flange[:3, :3]@self.guard.offset-actual['actual_TCP_pose'][:3])
        angle = (Rotation.from_matrix(flange[:3, :3]).inv()*Rotation.from_rotvec(actual['actual_TCP_pose'][3:])).magnitude()
        if error > .00025 or angle > np.deg2rad(.05):raise RuntimeError('Active TCP/model mismatch')
        for node, key, expected in [('/scaled_joint_trajectory_controller', 'interpolation_method', 'splines'),
                                    ('/controller_manager', 'update_rate', 125)]:
            client = self.create_client(GetParameters, node+'/get_parameters')
            if not client.wait_for_service(timeout_sec=3):raise RuntimeError('Controller parameter service unavailable')
            request = GetParameters.Request();request.names = [key]
            value = self.wait(client.call_async(request)).values[0]
            received = value.string_value if isinstance(expected, str) else value.integer_value
            if received != expected:raise RuntimeError('Unexpected controller '+key)
        status = {key:dashboard(key) for key in ['robotmode', 'safetystatus', 'running']}
        if (status['robotmode'] != 'Robotmode: RUNNING' or status['safetystatus'] != 'Safetystatus: NORMAL'
                or status['running'] != 'Program running: true'):
            raise RuntimeError('Controller status not ready: '+str(status))
        self.report.update(before=actual, preflight=before, dashboard=status,
                           speed_scaling_percent=self.scale, fk_error_mm=float(error*1000))
        print('PREFLIGHT '+json.dumps(self.report, ensure_ascii=False), flush=True)

    def stop_on_failure(self):
        if self.goal_handle is not None:
            try:self.goal_handle.cancel_goal_async()
            except Exception:pass
        # Only stops this UR program; never unlocks safety or resumes motion.
        try:self.report['stop_reply'] = dashboard('stop')
        except Exception as error:self.report['stop_failure'] = str(error)

    def run(self, execute):
        self.preflight()
        if not execute:return
        # External Control being played and general area clearance do not
        # resolve unverified physical geometry, calibration or stopping margin.
        # Never promote a diagnostic draft solely via an operator-ready flag.
        if self.report['unresolved_physical_checks']:
            raise RuntimeError('Execution held: '+', '.join(self.report['unresolved_physical_checks']))
        if not self.action.wait_for_server(timeout_sec=3):raise RuntimeError('Trajectory action unavailable')
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        for item in self.draft['points']:
            point = JointTrajectoryPoint()
            point.positions=item['q_rad'];point.velocities=item['velocity_rad_s'];point.accelerations=item['acceleration_rad_s2']
            point.time_from_start=seconds(item['time_s'])
            goal.trajectory.points.append(point)
        for name in JOINTS:
            goal.path_tolerance.append(JointTolerance(name=name, position=.003))
            goal.goal_tolerance.append(JointTolerance(name=name, position=.0015, velocity=.002))
        goal.goal_time_tolerance=seconds(3.)
        # No automatic retries: ambiguous sends are stopped, never resent.
        # Service/dashboard reads may take time. Drain new feedback before
        # checking again; never authorize motion from the preflight snapshot.
        deadline = time.monotonic()+1.
        previous_received = self.received
        while self.received == previous_received and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.01)
        self.check_live()
        if np.max(np.abs(self.q-self.guard.start_q)) > .0005 or np.max(np.abs(self.qd)) > .002:
            raise RuntimeError('Start changed immediately before execution')
        self.goal_sent = True;self.sent_at=time.monotonic();self.report['motion_sent']=True
        self.monitor_action_feedback = True
        try:
            future = self.action.send_goal_async(goal, feedback_callback=self.on_feedback)
            self.goal_handle = self.wait(future)
            if not self.goal_handle.accepted:raise RuntimeError('Trajectory rejected')
            finished = self.goal_handle.get_result_async()
            deadline=time.monotonic()+max(75., 1.5*self.draft['points'][-1]['time_s']/(self.scale/100.)+10.)
            logged = 0.;validated_at=0.
            while not finished.done():
                rclpy.spin_once(self, timeout_sec=.002)
                if time.monotonic()>deadline:raise RuntimeError('Execution deadline exceeded')
                if self.received != validated_at:
                    checked=self.check_live();validated_at=self.received
                    self.samples.append({'elapsed_s': time.monotonic()-self.sent_at, 'q_rad': self.q.tolist(), **checked})
                    if time.monotonic()-logged >= 3.:
                        print('MOVING '+json.dumps(self.samples[-1]), flush=True);logged=time.monotonic()
                elif time.monotonic()-self.received > .12:raise RuntimeError('Joint stream stopped')
            result=finished.result()
            if result is None or result.status != 4 or result.result.error_code != 0:
                raise RuntimeError('Trajectory unsuccessful: '+str(result))
            self.monitor_action_feedback = False
            settled=time.monotonic()+2.
            while time.monotonic()<settled:rclpy.spin_once(self, timeout_sec=.01);self.check_live()
            after, _=read_state()
            info=self.guard.inspect(np.asarray(after['actual_q']))
            measured_axis=np.asarray(self.guard.table['T_world_from_base'])[:3,:3]@Rotation.from_rotvec(after['actual_TCP_pose'][3:]).as_matrix()@np.array([0.,-1.,0.])
            heading=float(np.degrees(np.arctan2(measured_axis[1],measured_axis[0])))
            tcp_drift=float(np.linalg.norm(np.asarray(after['actual_TCP_pose'][:3])-self.report['before']['actual_TCP_pose'][:3])*1000)
            if abs(heading-90.)>.2 or tcp_drift>.5:raise RuntimeError('Final TCP/axis verification failed')
            self.report.update(completed=True, after=after, final=info, measured_shaft_heading_deg=heading,
                               measured_tcp_drift_mm=tcp_drift)
            print('VERIFIED '+json.dumps(self.report, ensure_ascii=False), flush=True)
        except BaseException as error:
            self.report.update(completed=False, error=str(error));self.stop_on_failure();raise


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--audit-directory',type=Path,required=True)
    parser.add_argument('--execute',action='store_true')
    parser.add_argument('--onsite-ready',action='store_true',help='Operator confirmed stopped motor, clear area, external-only program and available pendant')
    parser.add_argument('--report',type=Path,required=True)
    args=parser.parse_args()
    if args.report.exists():raise SystemExit('Refusing to overwrite execution record')
    if args.execute and not args.onsite_ready:raise SystemExit('Onsite readiness confirmation required')
    rclpy.init();node=None
    try:
        node=Runner(args.audit_directory.resolve());node.run(args.execute)
    finally:
        if node:
            node.report['live_samples']=node.samples
            with args.report.open('x') as stream:json.dump(node.report,stream,ensure_ascii=False,indent=2)
            node.destroy_node()
        rclpy.shutdown()
