"""Offline tests only: no node, socket, controller or robot is started."""
import time
from types import SimpleNamespace
import unittest

import numpy as np
import yaml

from run_checked_yaw import GeometryGuard, ROOT, Runner, seconds, unresolved_physical_checks


class MonitorTests(unittest.TestCase):
    def state(self):
        now = time.monotonic()
        return SimpleNamespace(q=np.zeros(6), qd=np.zeros(6), received=now,
            scale=100., scale_received=now, program=True,
            monitor_action_feedback=False, sent_at=now-2.,
            feedback_received=now, feedback_error=0.,
            guard=SimpleNamespace(inspect=lambda q: {'valid': True}))

    def test_duration_normalizes_nanoseconds(self):
        value = seconds(1.9999999996)
        self.assertEqual((value.sec, value.nanosec), (2, 0))

    def test_fresh_stationary_feedback(self):
        self.assertEqual(Runner.check_live(self.state()), {'valid': True})

    def test_bad_live_feedback_is_rejected(self):
        for key, value in [('received', 0.), ('scale_received', 0.),
                           ('program', False), ('scale', 0.),
                           ('qd', np.full(6, np.nan)),
                           ('qd', np.full(6, np.deg2rad(10.)))]:
            with self.subTest(key=key):
                state = self.state()
                setattr(state, key, value)
                with self.assertRaises(RuntimeError):Runner.check_live(state)

    def test_action_feedback_required_only_while_active(self):
        state = self.state()
        state.feedback_received = 0.
        Runner.check_live(state)
        state.monitor_action_feedback = True
        with self.assertRaisesRegex(RuntimeError, 'Action feedback stale'):
            Runner.check_live(state)

    def test_tracking_error_stops_active_motion(self):
        state = self.state()
        state.monitor_action_feedback = True
        state.feedback_error = .004
        with self.assertRaisesRegex(RuntimeError, 'tracking error'):
            Runner.check_live(state)


class GeometryTests(unittest.TestCase):
    def test_historical_sub_five_mm_audit_is_rejected(self):
        with self.assertRaisesRegex(RuntimeError, 'current global clearance policy'):
            GeometryGuard(ROOT/'robot/tests/fixtures/yaw_full_smooth_45s_20260919_checked')

    def test_invalid_joint_input_rejected(self):
        guard = object.__new__(GeometryGuard)
        for q in [np.zeros(5), np.full(6, np.nan)]:
            with self.assertRaisesRegex(RuntimeError, 'Invalid joints'):
                guard.inspect(q)

    def test_live_execution_cannot_bypass_unverified_physical_geometry(self):
        tool = yaml.safe_load((ROOT/'config/magnet_tool_geometry_draft.yaml').read_text())
        blockers = unresolved_physical_checks({}, tool)
        self.assertIn('whole_tool_envelope_not_physically_verified', blockers)

    def test_nominal_geometry_cannot_clear_stopping_and_calibration_checks(self):
        blockers = unresolved_physical_checks(
            {'remaining_blockers': ['physical_ceiling_clearance_and_calibration_error',
                                    'feedback_monitor_and_cancellation_stopping_path']},
            {'provisional_whole_tool_envelope': {'verified_encloses_all_rigid_parts': True}})
        self.assertEqual(len(blockers), 2)


if __name__ == '__main__':unittest.main()
