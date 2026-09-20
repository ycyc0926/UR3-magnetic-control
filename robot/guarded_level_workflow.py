#!/usr/bin/env python3
"""Read/plan by default; explicit execution of a bounded retreat + leveling.

Every state, including an initial retreat, uses the single global clearance
policy. The physical acrylic plane is never changed. Model checks are not
certified safety functions and do not model cables or the full tool shape.
"""
import argparse
import copy
import json
import math
from pathlib import Path
import socket
import time

import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rcl_interfaces.srv import GetParameters
from ur3_magnetic_control.ceiling_geometry import CeilingGeometry, sample_trajectory
from ur3_magnetic_control.level_magnet_axis import LevelMagnetAxis
from ur3_magnetic_control.cartesian_line_move import EXECUTION_TOKEN
import ur3_realtime_monitor as rtde


JOINTS = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
          'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']


def read_robot():
    with socket.create_connection((rtde.ROBOT_IP, 30004), timeout=3) as sock:
        rid, _ = rtde.negotiate(sock, 125)
        state = rtde.parse_state(rtde.receive_command(sock, rtde.RTDE_DATA_PACKAGE), rid)
        rtde.send_packet(sock, rtde.RTDE_CONTROL_PACKAGE_PAUSE)
    return state


def state_joints(state):
    return dict(zip(state.joint_state.name, state.joint_state.position))


def final_state(start, trajectory):
    state = copy.deepcopy(start)
    values = state_joints(state)
    values.update(zip(trajectory.joint_trajectory.joint_names, trajectory.joint_trajectory.points[-1].positions))
    state.joint_state.position = [values[name] for name in state.joint_state.name]
    state.joint_state.velocity = []
    state.joint_state.effort = []
    return state


class Workflow:
    def __init__(self, node):
        self.node = node
        self.config = node.ceiling_guard.configuration
        self.normal_clearance = self.config['clearance_limits_m']['ceiling']
        client = node.create_client(GetParameters, '/move_group/get_parameters')
        req = GetParameters.Request(); req.names = ['robot_description']
        xml = node.call(client, req).values[0].string_value
        if 'calib_18089309548208516197' not in xml:
            raise RuntimeError('MoveIt does not have the measured UR3 calibration')
        self.model = CeilingGeometry(xml, self.config['T_base_from_world'])
        self.bottom = self.config['underside_world_z_m']
        self.normal = np.asarray(self.config['world_axes_in_base']['z'])
        self.normal /= np.linalg.norm(self.normal)
        self.report = {'mode': 'plan_only', 'physical_ceiling_world_m': self.bottom,
                       'motor_axis_tool': self.config['motor_axis_tool_vector']}

    def heights(self, state):
        return self.model.heights(state_joints(state), self.config['magnet_tcp_xyz_m'],
                                  self.config['magnet_sphere_radius_m'])

    def check_feedback(self):
        actual = read_robot()
        if max(abs(v) for v in actual['actual_qd']) > .002:
            raise RuntimeError('Robot is not stationary')
        pose = self.model.transforms(dict(zip(JOINTS, actual['actual_q'])))['tool0']
        tcp = pose[:3, 3] + pose[:3, :3] @ np.asarray(self.config['magnet_tcp_xyz_m'])
        position_error = np.linalg.norm(tcp - actual['actual_TCP_pose'][:3])
        angle_error = (Rotation.from_matrix(pose[:3, :3]).inv()
                       * Rotation.from_rotvec(actual['actual_TCP_pose'][3:])).magnitude()
        if position_error > .00025 or angle_error > math.radians(.05):
            raise RuntimeError(f'FK/controller mismatch: {position_error*1000:.3f} mm, {math.degrees(angle_error):.4f} deg')
        self.report['fk_position_error_mm'] = position_error*1000
        self.report['fk_angle_error_deg'] = math.degrees(angle_error)
        self.node.wait_for_state()
        ros = state_joints(self.node.current_robot_state())
        if max(abs(ros[n]-v) for n, v in zip(JOINTS, actual['actual_q'])) > .002:
            raise RuntimeError('ROS state and controller disagree')
        return actual

    def audit(self, trajectory, start, retreat=False):
        initial = self.heights(start)
        baseline = state_joints(start)
        initial_pose = self.model.transforms(baseline)['tool0']
        peaks = initial.copy()
        count = 0
        max_flange_drift = 0.0
        previous_pose = None
        previous_time = None
        max_angular_speed = 0.0
        for stamp, q in sample_trajectory(trajectory.joint_trajectory, interval_s=.01):
            state = copy.deepcopy(start)
            values = baseline.copy()
            values.update(zip(trajectory.joint_trajectory.joint_names, q))
            state.joint_state.position = [values[name] for name in state.joint_state.name]
            heights = self.heights(state)
            if self.bottom-max(heights.values()) < self.normal_clearance:
                raise RuntimeError(
                    f'Interpolated trajectory clearance below '
                    f'{self.normal_clearance*1000:.1f} mm'
                )
            if retreat and any(heights[name] > initial[name]+.00002 for name in initial):
                raise RuntimeError('Clearance retreat would raise a link surface')
            if max(abs(values[name]-baseline[name]) for name in JOINTS) > math.radians(5):
                raise RuntimeError('A joint changes by more than 5 degrees')
            pose = self.model.transforms(values)['tool0']
            drift = np.linalg.norm(pose[:3, 3]-initial_pose[:3, 3])
            max_flange_drift = max(max_flange_drift, float(drift))
            if not retreat and drift > .00015:
                raise RuntimeError('Flange translation exceeds 0.15 mm during rotation')
            if retreat:
                delta = pose[:3, 3]-initial_pose[:3, 3]
                if delta @ self.normal > .00002 or np.linalg.norm(delta-self.normal*(delta@self.normal)) > .00015:
                    raise RuntimeError('Retreat deviates from downward table normal')
            if previous_pose is not None:
                speed = (Rotation.from_matrix(previous_pose[:3, :3]).inv()
                         * Rotation.from_matrix(pose[:3, :3])).magnitude() / (stamp-previous_time)
                max_angular_speed = max(max_angular_speed, speed)
            previous_pose, previous_time = pose, stamp
            # Also check self-collision and the current scene at all interpolated states.
            self.node.ceiling_guard.validate_state(state, f'dense sample {count}')
            for name, height in heights.items():
                peaks[name] = max(peaks[name], height)
            count += 1
        if max_angular_speed > math.radians(1.0):
            raise RuntimeError('Angular speed exceeds 1 deg/s')
        end = final_state(start, trajectory)
        end_pose = self.model.transforms(state_joints(end))['tool0']
        shaft = end_pose[:3, :3] @ np.asarray(self.config['motor_axis_tool_vector'])
        elevation = math.degrees(math.asin(np.clip(shaft @ self.normal, -1, 1)))
        if not retreat and abs(elevation) > .03:
            raise RuntimeError('End motor shaft is not level within 0.03 degrees in model')
        return {'samples': count, 'min_clearance_mm': (self.bottom-max(peaks.values()))*1000,
                'per_link_min_clearance_mm': {k:(self.bottom-v)*1000 for k,v in peaks.items()},
                'flange_max_translation_mm': max_flange_drift*1000,
                'max_angular_speed_deg_s': math.degrees(max_angular_speed),
                'end_shaft_elevation_deg': elevation}

    def live_check(self):
        def check(state):
            gap = self.bottom - max(self.heights(state).values())
            if gap < self.normal_clearance:
                raise RuntimeError(f'Live model clearance {gap*1000:.3f} mm below bound')
        return check

    def run(self, execute):
        self.check_feedback()
        start = self.node.current_robot_state()
        self.report['before'] = read_robot()
        gap = self.bottom-max(self.heights(start).values())
        self.report['start_min_clearance_mm'] = gap*1000
        retreat = None
        planned_start = start
        try:
            if gap < self.normal_clearance + .0005:
                if gap < self.normal_clearance:
                    raise RuntimeError('Start clearance violates the global policy; automatic retreat refused')
                retreat = self.node.plan('-z', .002, .0005)[0]
                self.report['retreat'] = self.audit(retreat, start, retreat=True)
                planned_start = final_state(start, retreat)
            level = self.node.plan_level(math.radians(.5), start_state=planned_start)[0]
            self.report['level'] = self.audit(level, planned_start)
            print(json.dumps(self.report, ensure_ascii=False, indent=2), flush=True)
            if not execute:
                return
            if not self.config['motor_axis_verified']:
                raise RuntimeError('Motor axis not physically confirmed')
            self.node.wait_for_state(require_execution_state=True)
            self.check_feedback()
            self.report['mode'] = 'execution_started'
            if retreat is not None:
                self.node.execution_validator = self.live_check()
                self.node.wait_for_state(require_execution_state=True)
                print('EXECUTING 2 mm downward retreat under the global clearance policy', flush=True)
                self.node.execute(retreat)
                self.check_feedback()
                # Replan leveling from newly measured position after the retreat.
                planned_start = self.node.current_robot_state()
                level = self.node.plan_level(math.radians(.5), start_state=planned_start)[0]
                self.report['level_actual_start'] = self.audit(level, planned_start)
            self.node.execution_validator = self.live_check()
            self.node.wait_for_state(require_execution_state=True)
            print('EXECUTING flange-centred leveling, peak angular speed <=1 deg/s', flush=True)
            self.node.execute(level)
            actual = self.check_feedback()
            self.report['after'] = actual
            pose = Rotation.from_rotvec(actual['actual_TCP_pose'][3:])
            shaft = pose.apply(self.config['motor_axis_tool_vector'])
            self.report['measured_shaft_elevation_deg'] = math.degrees(math.asin(np.clip(shaft@self.normal, -1, 1)))
            self.report['after_min_clearance_mm'] = (self.bottom-max(self.heights(self.node.current_robot_state()).values()))*1000
            if abs(self.report['measured_shaft_elevation_deg']) > .1:
                raise RuntimeError('Post-motion shaft elevation exceeds 0.1 deg')
            self.report['mode'] = 'executed_and_verified'
            print('LEVELING_VERIFIED '+json.dumps(self.report, ensure_ascii=False), flush=True)
        finally:
            self.node.execution_validator = None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--confirmation-token', default='')
    parser.add_argument('--report', type=Path)
    args = parser.parse_args()
    if args.execute and args.confirmation_token != EXECUTION_TOKEN:
        raise SystemExit('Explicit execution confirmation required')
    rclpy.init(); node = LevelMagnetAxis(); flow = None
    try:
        node.wait_for_state(require_execution_state=args.execute)
        flow = Workflow(node)
        flow.run(args.execute)
    finally:
        if flow and args.report:
            # Exclusive creation preserves previous experiment records.
            with args.report.open('x', encoding='utf-8') as out:
                json.dump(flow.report, out, ensure_ascii=False, indent=2)
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
