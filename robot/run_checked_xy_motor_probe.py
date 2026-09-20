#!/usr/bin/env python3
"""One supervised -X 3 mm probe with manually switched motor; default is read-only.

Explicit --execute plus fresh onsite confirmations permit ONE reviewed motion.
No motor commands, automatic target updates, retries or square playback exist.
Execution activates the one controller, plays External Control, and stops the
program on success or failure. A local monitor is not a safety-rated system.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import socket
import threading
import time
from urllib.request import Request, build_opener, ProxyHandler

import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Bool
from action_msgs.msg import GoalStatusArray
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from controller_manager_msgs.srv import ListControllers, SwitchController
from rcl_interfaces.srv import GetParameters
from trajectory_msgs.msg import JointTrajectoryPoint

from prepare_xy_motor_probe import ROOT, JOINTS, XYGeometry, validate_snapshot
from manual_motor_probe_guard import check_probe_vision, check_tcp_speed, MEASURED_TCP_SPEED_LIMIT_MM_S, MEASURED_JOINT_SPEED_LIMIT_DEG_S
from explicit_probe_trigger import ExplicitProbeTrigger
from preview_yaw_geometry import read_state
from rtde_status_outputs import StatusRecorder, read_status
from ur3_magnetic_control.clearance_policy import clearance_limits_mm

MEASURED_SPEED_LIMIT_DEG_S = MEASURED_JOINT_SPEED_LIMIT_DEG_S
PATH_TOLERANCE_RAD = .0015
VISION_TARGET_TOLERANCE_MM = .5
VISION_MOTION_ABORT_MM = 1.0


def dashboard(command):
    if command not in ('robotmode', 'safetystatus', 'running', 'programState', 'get loaded program', 'play', 'stop'):
        raise ValueError('Unsupported Dashboard command')
    with socket.create_connection(('192.168.56.101', 29999), timeout=2.) as sock:
        stream = sock.makefile('rwb', buffering=0); stream.readline()
        stream.write((command+'\n').encode())
        return stream.readline().decode().strip()


def vision_request(endpoint='/status', body=None):
    request = Request('http://127.0.0.1:8767'+endpoint,
                      data=None if body is None else json.dumps(body).encode(),
                      headers={} if body is None else {'Content-Type': 'application/json'})
    with build_opener(ProxyHandler({})).open(request, timeout=.5) as response:
        return json.load(response)


def valid_vision(value, target_mm, tolerance_mm):
    age = (datetime.now(timezone.utc)-datetime.fromisoformat(value['utc'])).total_seconds()
    point = np.asarray(value['world_xy_mm'], float)
    if (value.get('detected') is not True or value.get('stream_stale') is not False or not 0 <= age <= .3
            or value.get('tracking_reason') != 'tracked' or point.shape != (2,) or not np.isfinite(point).all()):
        raise RuntimeError('H feedback invalid or stale')
    if np.linalg.norm(point-np.asarray(target_mm)) > tolerance_mm:
        raise RuntimeError('H moved beyond the reviewed fixed target; no automatic chase')
    return point


class VisionReader:
    def __init__(self):
        self.value = None; self.received = 0.; self.error = None
        self.ending = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True)

    def run(self):
        while not self.ending.is_set():
            try:
                value = vision_request()
                self.value = value; self.received = time.monotonic(); self.error = None
            except Exception as error:
                self.error = str(error)
            self.ending.wait(.05)

    def check(self, target, tolerance):
        if self.error is not None or self.value is None or time.monotonic()-self.received > .3:
            raise RuntimeError('Vision reader unavailable or stale: '+str(self.error))
        return valid_vision(self.value, target, tolerance)


class DashboardReader:
    """Read status off the ROS monitoring thread; never replays a program."""
    def __init__(self):
        self.value = None; self.received = 0.; self.error = None
        self.ending = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True)

    def run(self):
        while not self.ending.is_set():
            try:
                value = {key: dashboard(key) for key in ('running', 'programState')}
                self.value = value; self.received = time.monotonic(); self.error = None
            except Exception as error: self.error = str(error)
            self.ending.wait(.1)

    def ready(self):
        return (self.error is None and self.value is not None and time.monotonic()-self.received <= .5
                and self.value == {'running': 'Program running: true', 'programState': 'PLAYING external_control.urp'})


class StableReadiness:
    def __init__(self):
        self.since = None

    def observe(self, now, ros_ready, direct_state, dashboard_ready):
        good = (ros_ready and dashboard_ready and direct_state.get('runtime_state') == 2
                and direct_state.get('robot_mode') == 7 and direct_state.get('safety_mode') == 1
                and 0 < direct_state.get('speed_scaling', 0) <= 1)
        if not good: self.since = None; return False
        if self.since is None: self.since = now
        return now-self.since >= .5


def load_review(folder):
    active_clearance_limits = clearance_limits_mm(ROOT)
    seal = json.loads((folder/'EXECUTION_REVIEW.json').read_text())
    if seal.get('software_review_passed') is not True or seal.get('execution_allowed') is not False:
        raise ValueError('A sealed software review is required; this is not execution permission')
    for name, expected in seal['sha256'].items():
        if hashlib.sha256(Path(name).read_bytes()).hexdigest() != expected:
            raise ValueError('Reviewed probe software/artifact changed: '+name)
    stop = json.loads((folder/'STOP_REVIEW.json').read_text())
    if (stop.get('model_stop_sensitivity_passed') is not True
            or stop.get('clearance_limits_mm') != active_clearance_limits):
        raise ValueError('Measured-speed stopping review required')
    report = json.loads((folder/'audit.json').read_text())
    if report.get('checks_pass_for_draft') is not True or report.get('execution_allowed') is not False:
        raise ValueError('Requires an offline passing, unapproved XY draft')
    if report.get('mode') != 'offline_fixed_pose_XY_manual_motor_probe':
        raise ValueError('Wrong motion scope')
    if report.get('clearance_limits_mm') != active_clearance_limits:
        raise ValueError('Offline review does not use the current global clearance policy')
    for name, expected in report['config_sha256'].items():
        if hashlib.sha256((ROOT/'config'/name).read_bytes()).hexdigest() != expected:
            raise ValueError('Configuration changed: '+name)
    for name, expected in report['artifacts_sha256'].items():
        if Path(name).name != name or hashlib.sha256((folder/name).read_bytes()).hexdigest() != expected:
            raise ValueError('Audit artifact changed: '+name)
    for name, expected in report['helper_sha256'].items():
        if hashlib.sha256(Path(name).read_bytes()).hexdigest() != expected:
            raise ValueError('Geometry/planning helper changed: '+name)
    if hashlib.sha256((ROOT/'robot/prepare_xy_motor_probe.py').read_bytes()).hexdigest() != report['source_sha256']:
        raise ValueError('Planner changed after audit')
    data = report['snapshot_data']
    if hashlib.sha256(Path(report['source_snapshot']).read_bytes()).hexdigest() != report['snapshot_sha256']:
        raise ValueError('Source snapshot changed')
    if json.loads(Path(report['source_snapshot']).read_text()) != data:
        raise ValueError('Audit snapshot mismatch')
    q0, observed_h = validate_snapshot(data)
    target = np.asarray(report['sphere_target_world_mm'][:2])/1000.
    if not np.allclose(report['displacement_world_mm'], [-3, 0, 0], atol=1e-6, rtol=0):
        raise ValueError('Wrong reviewed displacement')
    if report['motion_duration_s'] != 5. or report['total_duration_s'] != 5.5:
        raise ValueError('Wrong reviewed timing')
    geometry = XYGeometry((folder/'diagnostic.urdf').read_text(), report['table_geometry'], report['side_geometry'],
                          report['tool_geometry'], q0, target)
    draft = json.loads((folder/'timed_path_NOT_AUTHORIZED.json').read_text())
    if draft['joint_names'] != JOINTS or draft['execution_allowed'] is not False:
        raise ValueError('Unexpected draft metadata')
    return report, draft, geometry


class AlignmentRunner(Node):
    def __init__(self, audit, draft, geometry, report):
        super().__init__('supervised_single_xy_motor_probe')
        self.audit, self.draft, self.geometry, self.report = audit, draft, geometry, report
        self.q = self.qd = self.scale = None
        self.joint_received = self.scale_received = self.feedback_received = 0.
        self.feedback_error = float('inf'); self.program = False; self.statuses = []
        self.feedback_data = None; self.goal = None; self.recorder = None; self.vision = None
        self.active = False; self.goal_id = None
        self.dashboard_reader = None
        self.motor_gate = None
        self.probe_started = False
        self.create_subscription(JointState, '/joint_states', self.joints, 1)
        self.create_subscription(Float64, '/speed_scaling_state_broadcaster/speed_scaling', self.scaling, 1)
        retained = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, '/io_and_status_controller/robot_program_running', lambda m: setattr(self, 'program', m.data), retained)
        self.create_subscription(GoalStatusArray, '/scaled_joint_trajectory_controller/follow_joint_trajectory/_action/status', self.action_status, retained)
        self.action = ActionClient(self, FollowJointTrajectory, '/scaled_joint_trajectory_controller/follow_joint_trajectory')

    def joints(self, message):
        p = dict(zip(message.name, message.position)); v = dict(zip(message.name, message.velocity))
        if not all(name in p and name in v for name in JOINTS): return
        self.q = np.array([p[name] for name in JOINTS]); self.qd = np.array([v[name] for name in JOINTS])
        self.joint_received = time.monotonic()

    def scaling(self, message):
        self.scale = float(message.data); self.scale_received = time.monotonic()

    def refresh(self, running=False):
        previous = self.joint_received
        deadline = time.monotonic()+.3
        while self.joint_received == previous and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.005)
        return self.live(running=running)

    def action_status(self, message):
        self.statuses = [(list(s.goal_info.goal_id.uuid), s.status) for s in message.status_list]

    def feedback(self, message):
        error = message.feedback.error.positions
        self.feedback_error = max(map(abs, error)) if len(error) == 6 and np.isfinite(error).all() else float('inf')
        self.feedback_received = time.monotonic()
        self.feedback_data = {key: {field: list(getattr(getattr(message.feedback, key), field))
                              for field in ['positions', 'velocities', 'accelerations']}
                              for key in ['desired', 'actual', 'error']}

    def wait(self, future, seconds=3.):
        end = time.monotonic()+seconds
        while not future.done() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=.005)
            if self.active: self.live(running=True)
        if not future.done() or future.result() is None: raise RuntimeError('ROS request timeout')
        return future.result()

    def service(self, kind, name, request):
        client = self.create_client(kind, name)
        if not client.wait_for_service(timeout_sec=3.): raise RuntimeError('Service unavailable: '+name)
        try: return self.wait(client.call_async(request), 5.)
        finally: self.destroy_client(client)

    def live(self, running=False):
        now = time.monotonic()
        if (self.q is None or self.qd is None or now-self.joint_received > .12
                or not np.isfinite(self.q).all() or not np.isfinite(self.qd).all()):
            raise RuntimeError('Joint feedback invalid or stale')
        if np.max(np.abs(self.qd)) > np.deg2rad(MEASURED_SPEED_LIMIT_DEG_S):
            raise RuntimeError(f'Measured joint speed exceeded {MEASURED_SPEED_LIMIT_DEG_S:g} degrees/s')
        if running and (not self.program or self.scale is None or now-self.scale_received > .2
                        or not np.isfinite(self.scale) or not 0 < self.scale <= 100.):
            raise RuntimeError('Program or effective speed scaling invalid')
        active_goals = [(goal_id, status) for goal_id, status in self.statuses if status in (1, 2, 3)]
        pending_own_acceptance = self.active and self.goal_id is None
        if ((pending_own_acceptance and len(active_goals) > 1) or
                (not pending_own_acceptance and any(goal_id != self.goal_id for goal_id, status in active_goals))):
            raise RuntimeError('Unknown active action goal')
        checked = self.geometry.inspect(self.q)
        if self.recorder:
            self.recorder.check_fresh(.12)
            direct = self.recorder.latest['state']
            if direct['safety_mode'] != 1 or direct['robot_mode'] != 7:
                raise RuntimeError('Direct RTDE safety or robot mode changed')
            if np.max(np.abs(np.asarray(direct['actual_qd']))) > np.deg2rad(MEASURED_SPEED_LIMIT_DEG_S):
                raise RuntimeError('Independent RTDE measured speed exceeded limit')
            self.geometry.inspect(np.asarray(direct['actual_q']))
            check_tcp_speed(direct)
            if running and (direct['runtime_state'] != 2 or not 0 < direct['speed_scaling'] <= 1):
                raise RuntimeError('Direct RTDE runtime/scaling invalid')
        if running and self.dashboard_reader is not None and not self.dashboard_reader.ready():
            raise RuntimeError('Direct Dashboard running state invalid or stale')
        if self.vision:
            if self.probe_started:
                self.vision.check(self.audit['observed_h_start_world_mm'], 16.)
                check_probe_vision(self.vision.value, self.audit['observed_h_start_world_mm'])
            else:
                self.vision.check(self.audit['observed_h_start_world_mm'], VISION_TARGET_TOLERANCE_MM)
        if self.vision and 'recording_directory' in self.report:
            recording = self.vision.value.get('recording', {})
            if not recording.get('active') or recording.get('error'):
                raise RuntimeError('Video recording ended or failed during probe')
        if self.motor_gate and self.probe_started:
            self.motor_gate.check_during(time.time_ns())
        if self.active and now-self.sent_at > .5:
            if now-self.feedback_received > .25 or self.feedback_error > PATH_TOLERANCE_RAD:
                raise RuntimeError('Action feedback stale or tracking error exceeded limit')
        return checked

    def preflight(self):
        end = time.monotonic()+3.
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=.01)
        self.live()
        if np.max(np.abs(self.q-self.geometry.start_q)) > .0005 or np.max(np.abs(self.qd)) > .0001:
            raise RuntimeError('Start pose changed; no automatic repositioning')
        controllers = self.service(ListControllers, '/controller_manager/list_controllers', ListControllers.Request()).controller
        selected = [c for c in controllers if c.name == 'scaled_joint_trajectory_controller']
        if len(selected) != 1 or selected[0].state != 'inactive':
            raise RuntimeError('Initial motion controller must be inactive')
        for controller in controllers:
            if controller.state == 'active' and any(i.startswith(name+'/') for i in controller.claimed_interfaces for name in JOINTS):
                raise RuntimeError('Another joint command controller is active')
        for name, key, wanted in [('/scaled_joint_trajectory_controller', 'interpolation_method', 'splines'),
                                  ('/controller_manager', 'update_rate', 125)]:
            request = GetParameters.Request(); request.names = [key]
            value = self.service(GetParameters, name+'/get_parameters', request).values[0]
            if (value.string_value if isinstance(wanted, str) else value.integer_value) != wanted:
                raise RuntimeError('Controller configuration changed')
        status = {command: dashboard(command) for command in ['robotmode', 'safetystatus', 'running', 'programState', 'get loaded program']}
        if (status['robotmode'] != 'Robotmode: RUNNING' or status['safetystatus'] != 'Safetystatus: NORMAL'
                or status['running'] != 'Program running: false' or status['programState'] != 'STOPPED external_control.urp'
                or status['get loaded program'] != 'Loaded program: /programs/external_control.urp'):
            raise RuntimeError('Unexpected direct controller status: '+str(status))
        state, _ = read_status()
        if (state['runtime_state'] != 1 or state['safety_mode'] != 1 or state['robot_mode'] != 7
                or np.max(np.abs(state['actual_qd'])) > .0001 or np.max(np.abs(state['actual_TCP_speed'])) > .0001
                or not np.allclose(state['tcp_offset'], [0, -.062, .041, 0, 0, 0], atol=1e-8, rtol=0)
                or abs(state['payload']-.32) > 1e-6):
            raise RuntimeError('Direct stopped state, TCP or payload check failed')
        if np.max(np.abs(np.asarray(state['actual_q'])-self.geometry.start_q)) > .0005:
            raise RuntimeError('Direct RTDE start mismatch')
        base = self.geometry.model.transforms(dict(zip(JOINTS, state['actual_q'])))['tool0']
        error = np.linalg.norm(base[:3, 3]+base[:3, :3]@self.geometry.offset-state['actual_TCP_pose'][:3])
        angle = (Rotation.from_matrix(base[:3, :3]).inv()*Rotation.from_rotvec(state['actual_TCP_pose'][3:])).magnitude()
        if error > .00025 or angle > np.deg2rad(.05): raise RuntimeError('Active TCP and model mismatch')
        vision = vision_request()
        valid_vision(vision, self.audit['observed_h_start_world_mm'], VISION_TARGET_TOLERANCE_MM)
        if vision.get('recording', {}).get('active'): raise RuntimeError('Existing recording must not be interrupted')
        self.report.update(preflight_dashboard=status, before=state, before_vision=vision,
                           start_check=self.geometry.inspect(state['actual_q']))
        print('READ_ONLY_PREFLIGHT_PASSED '+json.dumps(self.report, ensure_ascii=False), flush=True)

    def abort(self):
        if self.goal:
            try: self.goal.cancel_goal_async()
            except Exception as error: self.report['cancel_error'] = str(error)
        try:
            self.report['stop_reply'] = dashboard('stop')
        except Exception as error:
            self.report['stop_error'] = str(error)
            print('STOP NOT CONFIRMED: onsite operator must use the agreed physical stop.', flush=True)

    def stopped_check(self):
        states = []
        for _ in range(3):
            state, _ = read_state()
            if (np.max(np.abs(state['actual_qd'])) > .0001 or
                    np.max(np.abs(state['actual_TCP_speed'])) > .0001):
                raise RuntimeError('Post-stop velocity was not zero')
            states.append(state)
        status = {key: dashboard(key) for key in ['running', 'programState', 'safetystatus']}
        if status != {'running': 'Program running: false', 'programState': 'STOPPED external_control.urp',
                      'safetystatus': 'Safetystatus: NORMAL'}:
            raise RuntimeError('Final stopped/safety state not confirmed: '+str(status))
        self.report.update(after=states[-1], stopped_samples=states, after_dashboard=status)

    def execute(self, folder):
        attempted_play = False; controller_activated = False; recording_owned = False
        try:
            self.recorder = StatusRecorder(folder/'direct_rtde_outputs.jsonl')
            self.recorder.start()
            self.vision = VisionReader(); self.vision.thread.start()
            end = time.monotonic()+2.
            while self.vision.value is None and time.monotonic() < end:
                rclpy.spin_once(self, timeout_sec=.01)
            self.vision.check(self.audit['observed_h_start_world_mm'], VISION_TARGET_TOLERANCE_MM)
            self.refresh()
            vision_request('/record/start', {'duration_s': 120})
            recording_owned = True
            end = time.monotonic()+7.
            while time.monotonic() < end:
                rclpy.spin_once(self, timeout_sec=.01); self.live()
                recording = self.vision.value.get('recording', {})
                if recording.get('active') and recording.get('frames', 0) >= 100:
                    self.report['recording_directory'] = recording['directory']; break
            else: raise RuntimeError('Recording did not start receiving frames')
            request = SwitchController.Request()
            request.activate_controllers = ['scaled_joint_trajectory_controller']
            request.strictness = SwitchController.Request.STRICT
            request.activate_asap = True; request.timeout.sec = 3
            if not self.service(SwitchController, '/controller_manager/switch_controller', request).ok:
                raise RuntimeError('Controller activation failed')
            controller_activated = True
            # Activation creates a fresh action server; no prior action is replayed.
            if not self.action.wait_for_server(timeout_sec=2.): raise RuntimeError('Action server unavailable')
            for _ in range(20): rclpy.spin_once(self, timeout_sec=.01)
            self.live()
            if np.max(np.abs(self.q-self.geometry.start_q)) > .0005:
                raise RuntimeError('Start changed before playback')
            attempted_play = True
            self.report['play_reply'] = dashboard('play')
            if self.report['play_reply'] != 'Starting program': raise RuntimeError('External Control playback failed')
            self.dashboard_reader = DashboardReader(); self.dashboard_reader.thread.start()
            readiness = StableReadiness(); self.report['startup_samples'] = []; last_sample = -1.
            end = time.monotonic()+8.
            while time.monotonic() < end:
                rclpy.spin_once(self, timeout_sec=.005); self.live()
                if np.max(np.abs(self.q-self.geometry.start_q)) > .0005 or np.max(np.abs(self.qd)) > .0001:
                    raise RuntimeError('Unexpected motion while enabling External Control')
                now = time.monotonic()
                ros_ready = (self.program and self.scale is not None and 0 < self.scale <= 100
                             and now-self.scale_received < .2)
                direct = self.recorder.latest['state']
                direct_dashboard_ready = self.dashboard_reader.ready()
                if now-last_sample >= .05:
                    last_sample = now
                    self.report['startup_samples'].append({'host_time_ns': time.time_ns(), 'ros_program': self.program,
                        'ros_scale': self.scale, 'rtde_runtime_state': direct['runtime_state'],
                        'rtde_speed_scaling': direct['speed_scaling'], 'dashboard': self.dashboard_reader.value,
                        'dashboard_error': self.dashboard_reader.error})
                if readiness.observe(now, ros_ready, direct, direct_dashboard_ready): break
            else: raise RuntimeError('ROS, RTDE and Dashboard did not agree continuously within 8 seconds')
            self.refresh(running=True)
            self.vision.check(self.audit['observed_h_start_world_mm'], VISION_TARGET_TOLERANCE_MM)
            self.motor_gate = ExplicitProbeTrigger(timeout_s=90.)
            self.report['motor_ready_utc'] = datetime.now(timezone.utc).isoformat()
            print('READY_FOR_ONE_MANUAL_MOTOR_TRIAL: ARM HOLDING, VIDEO VERIFIED. Explicit arm control panel below camera page http://127.0.0.1:8767/#arm-control ; no trajectory sent yet.', flush=True)
            # Only the clearly labeled, one-shot arm-control page can start the goal.
            # The original vision-only UI remains vision-only.
            end = time.monotonic()+90.
            while time.monotonic() < end:
                rclpy.spin_once(self, timeout_sec=.005); self.live(running=True)
                if np.max(np.abs(self.q-self.geometry.start_q)) > .0005 or np.max(np.abs(self.qd)) > .0001:
                    raise RuntimeError('Unexpected arm motion while awaiting operator')
                if self.motor_gate.ready(time.time_ns()):
                    self.probe_started = True
                    self.report['explicit_arm_start_request'] = self.motor_gate.start_event
                    break
            else: raise RuntimeError('No explicit arm start within 90 s; no retry')
            self.refresh(running=True)
            goal = FollowJointTrajectory.Goal(); goal.trajectory.joint_names = JOINTS
            for item in self.draft['points']:
                point = JointTrajectoryPoint(positions=item['q_rad'], velocities=item['velocity_rad_s'],
                                             accelerations=item['acceleration_rad_s2'])
                nanoseconds = round(item['time_s']*1e9)
                point.time_from_start.sec = nanoseconds//1000000000
                point.time_from_start.nanosec = nanoseconds%1000000000
                goal.trajectory.points.append(point)
            for name in JOINTS:
                goal.path_tolerance.append(JointTolerance(name=name, position=PATH_TOLERANCE_RAD))
                goal.goal_tolerance.append(JointTolerance(name=name, position=.001, velocity=.002))
            goal.goal_time_tolerance.sec = 3
            self.sent_at = time.monotonic(); self.active = True
            self.report['motion_sent'] = True
            self.goal = self.wait(self.action.send_goal_async(goal, feedback_callback=self.feedback))
            if not self.goal.accepted: raise RuntimeError('XY trajectory was rejected')
            self.goal_id = list(self.goal.goal_id.uuid)
            future = self.goal.get_result_async()
            deadline = time.monotonic()+6.0
            last_seen = 0.; last_print = 0.
            with (folder/'live_samples.jsonl').open('x') as stream:
                while not future.done():
                    rclpy.spin_once(self, timeout_sec=.002)
                    checked = self.live(running=True)
                    if time.monotonic() > deadline: raise RuntimeError('Bounded execution deadline exceeded')
                    if self.joint_received != last_seen:
                        last_seen = self.joint_received
                        entry = {'host_time_ns': time.time_ns(), 'elapsed_s': time.monotonic()-self.sent_at,
                                 'q_rad': self.q.tolist(), 'qd_rad_s': self.qd.tolist(),
                                 'action_feedback': self.feedback_data, 'vision': self.vision.value, **checked}
                        stream.write(json.dumps(entry, allow_nan=False)+'\n')
                        if time.monotonic()-last_print > .5:
                            last_print = time.monotonic()
                            print('XY_MOTOR_PROBE '+json.dumps({'elapsed_s': entry['elapsed_s'], **checked}), flush=True)
            result = future.result()
            if result is None or result.status != 4 or result.result.error_code != 0:
                raise RuntimeError('XY trajectory did not succeed: '+str(result))
            self.active = False
            print('STOP MOTOR NOW: arm goal finished; no further trial.', flush=True)
            end = time.monotonic()+.05
            while time.monotonic() < end:
                rclpy.spin_once(self, timeout_sec=.005); self.live(running=True)
            end_q = np.asarray(self.draft['points'][-1]['q_rad'])
            if np.max(np.abs(self.q-end_q)) > .001 or np.max(np.abs(self.qd)) > .0001:
                raise RuntimeError('Final joint target or zero speed not reached')
            self.report['stop_reply'] = dashboard('stop')
            self.stopped_check()
            after = self.report['after']
            world = self.geometry.model.world_from_base
            delta = world[:3, :3]@(np.asarray(after['actual_TCP_pose'][:3])-self.report['before']['actual_TCP_pose'][:3])
            rotation = (Rotation.from_rotvec(self.report['before']['actual_TCP_pose'][3:]).inv()
                        * Rotation.from_rotvec(after['actual_TCP_pose'][3:])).magnitude()
            if np.linalg.norm(delta-self.geometry.delta) > .0003 or rotation > np.deg2rad(.15):
                raise RuntimeError('Direct TCP displacement/orientation verification failed')
            self.report.update(completed=True, measured_world_displacement_mm=(delta*1000).tolist(),
                               measured_orientation_change_deg=float(np.degrees(rotation)))
        except BaseException as error:
            self.report.update(completed=False, error=str(error), failure_feedback={
                'q_rad': None if self.q is None else self.q.tolist(),
                'qd_rad_s': None if self.qd is None else self.qd.tolist(),
                'action_feedback': self.feedback_data,
                'direct_rtde': None if self.recorder is None else self.recorder.latest})
            if attempted_play:
                self.abort()
                try: self.stopped_check()
                except Exception as verification: self.report['stop_verification_error'] = str(verification)
            raise
        finally:
            print('KEEP MOTOR PHYSICALLY STOPPED. Software cannot stop motor.', flush=True)
            self.active = False
            if controller_activated and 'stop_error' not in self.report:
                try:
                    controllers = self.service(ListControllers, '/controller_manager/list_controllers', ListControllers.Request()).controller
                    selected = next(c for c in controllers if c.name == 'scaled_joint_trajectory_controller')
                    if selected.state == 'inactive':
                        self.report['controller_deactivated'] = True
                        self.report['controller_already_inactive'] = True
                    elif selected.state == 'active':
                        request = SwitchController.Request()
                        request.deactivate_controllers = ['scaled_joint_trajectory_controller']
                        request.strictness = SwitchController.Request.STRICT; request.timeout.sec = 3
                        self.report['controller_deactivated'] = self.service(SwitchController, '/controller_manager/switch_controller', request).ok
                    else: raise RuntimeError('Unexpected final controller state: '+selected.state)
                except Exception as error: self.report['deactivate_error'] = str(error)
            if recording_owned:
                try:
                    # Preserve a short stationary observation tail; this does not stop the motor.
                    end = time.monotonic()+1.
                    while time.monotonic() < end: rclpy.spin_once(self, timeout_sec=.02)
                    vision_request('/record/stop', {})
                    end = time.monotonic()+2.
                    while time.monotonic() < end:
                        rclpy.spin_once(self, timeout_sec=.02)
                        if (self.vision.value and not self.vision.value['recording']['active'] and
                                self.vision.value['recording']['directory'] == self.report.get('recording_directory')):
                            self.report['recording_saved'] = self.vision.value['recording']; break
                    else: raise RuntimeError('Recording stop acknowledgment not observed')
                except Exception as error: self.report['recording_stop_error'] = str(error)
            if self.vision:
                self.vision.ending.set(); self.vision.thread.join(1.)
                self.report['final_vision'] = self.vision.value
            if self.dashboard_reader:
                self.dashboard_reader.ending.set(); self.dashboard_reader.thread.join(1.)
            if self.motor_gate:
                self.motor_gate.close()
            if self.recorder:
                self.recorder.close(); self.report['rtde_recording'] = self.recorder.summary()
            print('FINAL '+json.dumps(self.report, ensure_ascii=False), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--audit-directory', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--motor-stopped', action='store_true')
    parser.add_argument('--manual-motor-six-seconds-confirmed', action='store_true')
    parser.add_argument('--onsite-clearance-confirmed', action='store_true')
    parser.add_argument('--sole-operator-confirmed', action='store_true')
    parser.add_argument('--external-control-only-confirmed', action='store_true')
    args = parser.parse_args()
    if args.execute and not all([args.manual_motor_six_seconds_confirmed, args.motor_stopped, args.onsite_clearance_confirmed,
                                 args.sole_operator_confirmed, args.external_control_only_confirmed]):
        raise SystemExit('Fresh onsite confirmations are required for execution')
    audit, draft, geometry = load_review(args.audit_directory)
    args.output.mkdir(parents=True, exist_ok=False)
    report = {'utc': datetime.now(timezone.utc).isoformat(), 'motion_sent': False, 'completed': False,
              'read_only_preflight': not args.execute, 'audit_directory': str(args.audit_directory.resolve()),
              'executor_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              'executor_dependency_sha256': {name: hashlib.sha256((ROOT/'robot'/name).read_bytes()).hexdigest()
                  for name in ['rtde_status_outputs.py', 'record_rtde_outputs.py', 'ur3_realtime_monitor.py', 'manual_motor_probe_guard.py', 'explicit_probe_trigger.py']},
              'motor_stopped_user_confirmed': args.motor_stopped,
              'manual_motor_six_seconds_user_confirmed': args.manual_motor_six_seconds_confirmed,
              'measured_tcp_abort_mm_s': MEASURED_TCP_SPEED_LIMIT_MM_S,
              'onsite_clearance_user_confirmed': args.onsite_clearance_confirmed,
              'sole_operator_user_confirmed': args.sole_operator_confirmed,
              'external_control_only_user_confirmed': args.external_control_only_confirmed,
              'no_motor_commands': True, 'no_automatic_next_stage': True,
              'measured_speed_limit_deg_s': MEASURED_SPEED_LIMIT_DEG_S}
    rclpy.init(); node = None
    try:
        node = AlignmentRunner(audit, draft, geometry, report)
        node.preflight()
        report['preflight_passed'] = True
        if args.execute: node.execute(args.output)
    except BaseException as error:
        report['error'] = str(error)
        raise
    finally:
        if node: node.destroy_node()
        rclpy.shutdown()
        (args.output/'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
