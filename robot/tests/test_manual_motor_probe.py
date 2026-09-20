"""Offline failure tests; no real ROS services or hardware connections."""
from pathlib import Path
import json
import tempfile
import unittest
from unittest.mock import patch

from manual_motor_probe_guard import MotorMarkerGate, check_probe_vision, check_tcp_speed, MEASURED_TCP_SPEED_LIMIT_MM_S
from run_checked_xy_motor_probe import main
from run_checked_xy_motor_probe import AlignmentRunner
from test_run_checked_xy_alignment import runner, vision
from types import SimpleNamespace
import numpy as np
from explicit_probe_trigger import ExplicitProbeTrigger
import threading
import time
from urllib.request import Request, build_opener, ProxyHandler
from urllib.error import HTTPError


class GuardTests(unittest.TestCase):
    def test_live_guard_allows_reviewed_h_motion_and_rejects_measured_tcp_overspeed(self):
        node = runner()
        node.report = {}
        node.probe_started = True
        node.motor_gate = SimpleNamespace(check_during=lambda now: None)
        node.audit = {'observed_h_start_world_mm': [200., 100.]}
        v = vision(); v.update(world_xy_mm=[188., 100.], pixel_full=[1200., 1600.])
        node.vision = SimpleNamespace(value=v, check=lambda target, tolerance: None)
        state = {'actual_q': [0.]*6, 'actual_qd': [0.]*6,
                 'actual_TCP_speed': [.001, 0, 0, 0, 0, 0],
                 'safety_mode': 1, 'robot_mode': 7, 'runtime_state': 2, 'speed_scaling': 1.}
        node.recorder = SimpleNamespace(check_fresh=lambda age: None, latest={'state': state})
        AlignmentRunner.live(node, running=True)
        state['actual_TCP_speed'][0] = .0201
        with self.assertRaises(RuntimeError): AlignmentRunner.live(node, running=True)
        state['actual_TCP_speed'][0] = 0.
        v['world_xy_mm'] = [203., 100.]
        with self.assertRaises(RuntimeError): AlignmentRunner.live(node, running=True)

    def test_tcp_speed_is_measured_norm_not_per_axis(self):
        self.assertEqual(MEASURED_TCP_SPEED_LIMIT_MM_S, 20.)
        for speed in [.001, .0038505426, .020, -.020]:
            check_tcp_speed({'actual_TCP_speed': [speed, 0, 0, 0, 0, 0]})
        for speed in [[.015, .015, 0, 0, 0, 0], [.02001, 0, 0, 0, 0, 0],
                      [float('nan')]*6, [0]*3]:
            with self.subTest(speed=speed), self.assertRaises(RuntimeError):
                check_tcp_speed({'actual_TCP_speed': speed})

    def test_h_progress_allowed_but_wrong_direction_sideways_and_boundary_fail(self):
        def value(xy, pixel=(1318, 1684)):
            return {'world_xy_mm': xy, 'pixel_full': pixel}
        check_probe_vision(value([188, 100]), [200, 100])
        for v in [value([201.01, 100]), value([200, 103.6]), value([184, 100]),
                  value([200, 100], (1318, 1830)), value([float('nan'), 100])]:
            with self.subTest(v=v), self.assertRaises(RuntimeError):
                check_probe_vision(v, [200, 100])

    def test_gate_rejects_old_or_repeated_markers_and_missing_source(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d)/'events'; armed = 10_000_000_000
            def event(kind, stamp, **extra):
                return dict(event=kind, wall_time_ns=stamp,
                            source='operator_click_not_hardware_feedback', **extra)
            old = event('motor_started', armed-1)
            current = event('motor_started', armed+100_000_000)
            path.write_text(json.dumps(old)+'\n')
            gate = MotorMarkerGate(path, armed)
            self.assertFalse(gate.ready(armed))
            path.write_text(json.dumps(old)+'\n'+json.dumps(current)+'\n')
            self.assertTrue(gate.ready(armed+200_000_000))
            gate.check_during(armed+1_000_000_000)
            with self.assertRaises(RuntimeError): gate.check_during(armed+6_200_000_000)
            for extra in [event('motor_started', armed+300_000_000),
                          event('motor_stopped', armed+300_000_000),
                          event('target_reset', armed+300_000_000)]:
                path.write_text(json.dumps(current)+'\n'+json.dumps(extra)+'\n')
                with self.assertRaises(RuntimeError): gate.check_during(armed+400_000_000)
                with self.assertRaises(RuntimeError): gate.ready(armed+400_000_000)
            del current['source']; path.write_text(json.dumps(current)+'\n')
            with self.assertRaises(RuntimeError): MotorMarkerGate(path, armed).ready(armed+200_000_000)

    def test_stale_future_and_partial_markers_never_trigger_a_goal(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d)/'events'; armed = 10_000_000_000
            event = dict(event='motor_started', wall_time_ns=armed+100_000_000,
                         source='operator_click_not_hardware_feedback')
            path.write_text(json.dumps(event)+'\n')
            for now in [armed, armed+700_000_000]:
                with self.assertRaises(RuntimeError): MotorMarkerGate(path, armed).ready(now)
            path.write_text('{"event":')
            self.assertFalse(MotorMarkerGate(path, armed).ready(armed+100_000_000))


class AuthorizationTests(unittest.TestCase):
    def test_execute_requires_manual_motor_stop_plan_confirmation(self):
        argv = ['run', '--audit-directory', 'unused', '--output', 'unused', '--execute',
                '--motor-stopped', '--onsite-clearance-confirmed', '--sole-operator-confirmed',
                '--external-control-only-confirmed']
        with patch('sys.argv', argv), patch('run_checked_xy_motor_probe.load_review') as load:
            with self.assertRaises(SystemExit): main()
            load.assert_not_called()

    def test_default_never_activates_or_executes(self):
        with tempfile.TemporaryDirectory() as d:
            output = Path(d)/'read_only'
            with patch('sys.argv', ['run', '--audit-directory', 'unused', '--output', str(output)]), \
                    patch('run_checked_xy_motor_probe.load_review', return_value=({}, {}, object())), \
                    patch('run_checked_xy_motor_probe.AlignmentRunner') as node, \
                    patch('run_checked_xy_motor_probe.rclpy.init'), \
                    patch('run_checked_xy_motor_probe.rclpy.shutdown'):
                main()
                node.return_value.preflight.assert_called_once()
                node.return_value.execute.assert_not_called()


class ExplicitControlTests(unittest.TestCase):
    def gate(self):
        gate = object.__new__(ExplicitProbeTrigger)
        gate.token = 'test-token'; gate.deadline = time.monotonic()+2
        gate.start_event = None; gate.abort_requested = False; gate.lock = threading.Lock()
        return gate

    def test_single_explicit_request_only_and_abort_after_deadline(self):
        gate = self.gate(); mono = time.monotonic(); wall = time.time_ns()
        self.assertFalse(gate.accept('/start', 'wrong', mono, wall))
        self.assertFalse(gate.ready(wall))
        self.assertTrue(gate.accept('/start', gate.token, mono, wall))
        self.assertFalse(gate.accept('/start', gate.token, mono, wall))
        self.assertTrue(gate.ready(wall+100_000_000))
        gate.check_during(wall+5_900_000_000)
        with self.assertRaises(RuntimeError): gate.check_during(wall+6_100_000_000)
        self.assertTrue(gate.accept('/abort', gate.token, gate.deadline+1, wall))
        with self.assertRaises(RuntimeError): gate.check_during(wall)

    def test_expired_start_and_future_or_stale_requests_rejected(self):
        gate = self.gate(); mono = time.monotonic(); wall = time.time_ns()
        self.assertFalse(gate.accept('/start', gate.token, gate.deadline+1, wall))
        self.assertTrue(gate.accept('/start', gate.token, mono, wall))
        for now in [wall-1, wall+500_000_001]:
            with self.assertRaises(RuntimeError): gate.ready(now)

    def test_http_get_never_arms_motion_and_cross_origin_post_is_rejected(self):
        gate = ExplicitProbeTrigger(port=0)
        try:
            url = 'http://127.0.0.1:'+str(gate.server.server_address[1])
            op = build_opener(ProxyHandler({}))
            with op.open(url, timeout=1) as response:
                self.assertIn('真实机械臂'.encode(), response.read())
            with op.open(url+'/status', timeout=1) as response:
                status = json.load(response)
            self.assertTrue(status['available'])
            self.assertTrue(status['can_start'])
            self.assertFalse(status['started'])
            self.assertIsNone(gate.start_event)
            def request(origin, token):
                return Request(url+'/start', data=json.dumps({'token': token}).encode(),
                               headers={'Origin': origin, 'Content-Type': 'application/json'})
            for origin, token, expected in [('http://untrusted.invalid', gate.token, 403),
                                            (url, 'wrong', 409)]:
                with self.assertRaises(HTTPError) as e: op.open(request(origin, token), timeout=1)
                self.assertEqual(e.exception.code, expected)
            with op.open(request(url, gate.token), timeout=1) as response:
                self.assertEqual(response.status, 202)
            self.assertIsNotNone(gate.start_event)
            with op.open(url+'/status', timeout=1) as response:
                self.assertFalse(json.load(response)['can_start'])
            gate.deadline = time.monotonic()-1
            with op.open(url+'/status', timeout=1) as response:
                status = json.load(response)
            self.assertFalse(status['available'])
            self.assertEqual(status['seconds_remaining'], 0)
        finally:
            gate.close()


if __name__ == '__main__':
    unittest.main()
