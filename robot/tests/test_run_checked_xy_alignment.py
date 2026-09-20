"""Offline monitor and authorization tests. All hardware calls are mocked."""
from datetime import datetime, timezone, timedelta
import json
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, patch

import numpy as np
from run_checked_xy_alignment import (AlignmentRunner, PositionVelocityWindow, valid_vision,
                                      valid_uniform_vision, VisionReader, StableReadiness, main)


def vision():
    return {'utc': datetime.now(timezone.utc).isoformat(), 'detected': True,
            'stream_stale': False, 'tracking_reason': 'tracked', 'world_xy_mm': [200., 100.]}


def runner():
    return SimpleNamespace(q=np.zeros(6), qd=np.zeros(6), joint_received=time.monotonic(),
        scale=100., scale_received=time.monotonic(), program=True, statuses=[], goal_id=None,
        geometry=SimpleNamespace(inspect=lambda q: {'valid': True}), recorder=None, vision=None,
        dashboard_reader=None,
        active=False, feedback_received=time.monotonic(), feedback_error=0., sent_at=time.monotonic())


class MonitorTests(unittest.TestCase):
    def test_user_requested_five_degree_joint_limit(self):
        node = runner()
        node.qd = np.deg2rad([5., -5., 0., 0., 0., 0.])
        AlignmentRunner.live(node, running=True)
        node.qd[0] = np.deg2rad(5.01)
        with self.assertRaises(RuntimeError):
            AlignmentRunner.live(node, running=True)

    def test_fresh_stationary_feedback_passes(self):
        self.assertTrue(AlignmentRunner.live(runner(), running=True)['valid'])

    def test_feedback_loss_nonfinite_or_overspeed_aborts(self):
        for field, value in [('joint_received', time.monotonic()-1), ('qd', np.full(6, np.nan)),
                             ('q', np.full(6, np.inf)), ('qd', np.full(6, np.deg2rad(5.01)))]:
            with self.subTest(field=field):
                node = runner(); setattr(node, field, value)
                with self.assertRaises(RuntimeError): AlignmentRunner.live(node, running=True)

    def test_program_and_scaling_faults_abort(self):
        for field, value in [('program', False), ('scale', 0.), ('scale', -1.),
                             ('scale', float('nan')), ('scale_received', time.monotonic()-1)]:
            with self.subTest(field=field, value=value):
                node = runner(); setattr(node, field, value)
                with self.assertRaises(RuntimeError): AlignmentRunner.live(node, running=True)

    def test_unknown_active_goal_aborts(self):
        node = runner(); node.statuses = [([1]*16, 2)]
        with self.assertRaises(RuntimeError): AlignmentRunner.live(node)
        node.goal_id = [1]*16
        AlignmentRunner.live(node)
        node.statuses.append(([2]*16, 1))
        with self.assertRaises(RuntimeError): AlignmentRunner.live(node)

    def test_two_goals_during_own_acceptance_abort(self):
        node = runner(); node.active = True; node.statuses = [([1]*16, 1)]
        AlignmentRunner.live(node)
        node.statuses.append(([2]*16, 2))
        with self.assertRaises(RuntimeError): AlignmentRunner.live(node)

    def test_stale_action_or_path_error_aborts(self):
        for field, value in [('feedback_received', time.monotonic()-1), ('feedback_error', .002)]:
            node = runner(); node.active = True; node.sent_at -= 1; setattr(node, field, value)
            with self.assertRaises(RuntimeError): AlignmentRunner.live(node)

    def test_independent_rtde_overspeed_aborts(self):
        node = runner()
        node.recorder = SimpleNamespace(check_fresh=lambda age: None,
            latest={'state': {'actual_q': [0.]*6, 'actual_qd': [np.deg2rad(5.01)]*6,
                              'safety_mode': 1, 'robot_mode': 7, 'runtime_state': 2, 'speed_scaling': 1.}})
        with self.assertRaises(RuntimeError): AlignmentRunner.live(node)

    def test_direct_tcp_norm_aborts_even_with_small_joint_speeds(self):
        node = runner()
        state = {'actual_q': [0.]*6, 'actual_qd': [0.]*6,
                 'actual_TCP_speed': [.001, 0, 0, 0, 0, 0],
                 'safety_mode': 1, 'robot_mode': 7, 'runtime_state': 2, 'speed_scaling': 1.}
        node.recorder = SimpleNamespace(check_fresh=lambda age: None, latest={'state': state})
        AlignmentRunner.live(node, running=True)
        state['actual_TCP_speed'] = [.0038505426, 0, 0, 0, 0, 0]
        AlignmentRunner.live(node, running=True)
        for speed in [[.015, .015, 0, 0, 0, 0], [.02001, 0, 0, 0, 0, 0], [float('nan')]*6, [0]*3]:
            state['actual_TCP_speed'] = speed
            with self.subTest(speed=speed), self.assertRaises(RuntimeError):
                AlignmentRunner.live(node, running=True)

    def test_uniform_motion_uses_windowed_tracking_not_raw_velocity_spikes(self):
        node = runner(); node.uniform = True; node.velocity_tracking_margin = np.deg2rad(.5)
        node.active = True
        node.feedback_data = {'desired': {'velocities': [np.deg2rad(3.)]*6}}
        node.qd[:] = np.deg2rad(40.)  # ignored: CB3's derivative output is quantized/spiky
        node.joint_velocity_window = PositionVelocityWindow()
        node.direct_joint_velocity_window = PositionVelocityWindow()
        node.direct_tcp_velocity_window = PositionVelocityWindow()
        q1 = np.full(6, np.deg2rad(3.4)*.08)
        node.joint_velocity_window.add(0., np.zeros(6)); node.joint_velocity_window.add(.08, q1)
        node.direct_joint_velocity_window.add(0., np.zeros(6))
        node.direct_tcp_velocity_window.add(0., np.zeros(3))
        direct = {'actual_q': q1, 'actual_qd': [np.deg2rad(40.)]*6,
                  'actual_TCP_pose': [.0008, 0, 0, 0, 0, 0],
                  'actual_TCP_speed': [.100, 0, 0, 0, 0, 0], 'safety_mode': 1,
                  'robot_mode': 7, 'runtime_state': 2, 'speed_scaling': 1.}
        node.recorder = SimpleNamespace(check_fresh=lambda age: None,
            latest={'host_monotonic_s': .08, 'state': direct})
        AlignmentRunner.live(node, running=True)
        q2 = q1+np.full(6, np.deg2rad(3.4)*.08); q2[0] = q1[0]+np.deg2rad(3.51)*.08
        node.q = q2; node.joint_velocity_window.add(.16, q2)
        direct['actual_q'] = q2; node.recorder.latest['host_monotonic_s'] = .16
        direct['actual_TCP_pose'][0] += .0008
        with self.assertRaises(RuntimeError): AlignmentRunner.live(node, running=True)

    def test_position_velocity_window_rejects_short_period_spikes(self):
        window = PositionVelocityWindow(.08)
        window.add(0., [0.]); window.add(.008, [.01]); window.add(.08, [.0008])
        self.assertAlmostEqual(window.velocity()[0], .01)

    def test_abort_still_stops_program_if_cancel_fails(self):
        node = SimpleNamespace(goal=MagicMock(), report={})
        node.goal.cancel_goal_async.side_effect = RuntimeError('lost ROS connection')
        with patch('run_checked_xy_alignment.dashboard', return_value='Stopped') as dashboard:
            AlignmentRunner.abort(node)
            dashboard.assert_called_once_with('stop')
        self.assertEqual(node.report['stop_reply'], 'Stopped')


class VisionTests(unittest.TestCase):
    def test_fresh_target_passes_but_moved_target_fails(self):
        valid_vision(vision(), [200., 100.], .5)
        with self.assertRaises(RuntimeError): valid_vision(vision(), [202., 100.], 1.)

    def test_lost_stale_or_invalid_target_fails(self):
        for key, value in [('detected', False), ('stream_stale', True), ('tracking_reason', 'locked_out'),
                           ('utc', (datetime.now(timezone.utc)-timedelta(seconds=1)).isoformat()),
                           ('world_xy_mm', [float('nan'), 100.])]:
            data = vision(); data[key] = value
            with self.subTest(key=key):
                    with self.assertRaises(RuntimeError): valid_vision(data, [200, 100], 1.)

    def test_uniform_vision_accepts_negative_x_progress_and_rejects_wrong_corridor(self):
        value = vision(); value.update(world_xy_mm=[242., 100.], pixel_full=[1700., 1650.])
        valid_uniform_vision(value, [242., 100.], False)
        value['world_xy_mm'] = [150., 105.]
        valid_uniform_vision(value, [242., 100.], True)
        for xy in ([248., 100.], [120., 100.], [180., 111.]):
            value['world_xy_mm'] = xy
            with self.subTest(xy=xy), self.assertRaises(RuntimeError):
                valid_uniform_vision(value, [242., 100.], True)

    def test_http_timeout_cannot_reuse_old_valid_sample(self):
        reader = VisionReader(); reader.value = vision(); reader.received = time.monotonic()
        reader.error = 'timed out'
        with self.assertRaises(RuntimeError): reader.check([200, 100], 1.)


class AuthorizationTests(unittest.TestCase):
    def test_execute_requires_all_explicit_confirmations(self):
        with patch('sys.argv', ['run', '--audit-directory', 'unused', '--output', 'unused', '--execute']), \
                patch('run_checked_xy_alignment.load_review') as load:
            with self.assertRaises(SystemExit): main()
            load.assert_not_called()

    def test_default_preflight_cannot_execute(self):
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp)/'preflight'
            with patch('sys.argv', ['run', '--audit-directory', 'unused', '--output', str(output)]), \
                    patch('run_checked_xy_alignment.load_review', return_value=({}, {}, object())), \
                    patch('run_checked_xy_alignment.AlignmentRunner') as node_class, \
                    patch('run_checked_xy_alignment.rclpy.init'), patch('run_checked_xy_alignment.rclpy.shutdown'):
                main()
                node_class.return_value.preflight.assert_called_once()
                node_class.return_value.execute.assert_not_called()
            report = json.loads((output/'report.json').read_text())
            self.assertFalse(report['motion_sent'])
            self.assertTrue(report['read_only_preflight'])


class StartupReadinessTests(unittest.TestCase):
    def state(self, runtime=2):
        return {'runtime_state': runtime, 'robot_mode': 7, 'safety_mode': 1, 'speed_scaling': 1.}

    def test_reverse_connection_and_positive_ros_scaling_are_insufficient(self):
        gate = StableReadiness()
        self.assertFalse(gate.observe(0., True, self.state(), False))
        self.assertFalse(gate.observe(10., True, self.state(), False))

    def test_pause_or_stop_cannot_be_treated_as_ready(self):
        for runtime in [0, 1, 3, 4, 5]:
            gate = StableReadiness()
            self.assertFalse(gate.observe(0., True, self.state(runtime), True))
            self.assertFalse(gate.observe(10., True, self.state(runtime), True))

    def test_startup_lag_waits_for_half_second_of_agreement(self):
        gate = StableReadiness()
        self.assertFalse(gate.observe(0., True, self.state(), False))
        self.assertFalse(gate.observe(.1, True, self.state(), True))
        self.assertFalse(gate.observe(.4, True, self.state(), True))
        self.assertTrue(gate.observe(.7, True, self.state(), True))

    def test_any_disagreement_resets_stability_window(self):
        gate = StableReadiness()
        self.assertFalse(gate.observe(0., True, self.state(), True))
        self.assertFalse(gate.observe(.4, False, self.state(), True))
        self.assertFalse(gate.observe(.6, True, self.state(), True))
        self.assertFalse(gate.observe(1., True, self.state(), True))
        self.assertTrue(gate.observe(1.2, True, self.state(), True))


if __name__ == '__main__':
    unittest.main()
