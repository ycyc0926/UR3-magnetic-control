"""No sockets, ROS nodes, hardware or motor commands in these tests."""
import time
from types import SimpleNamespace
import unittest
import numpy as np
from bounded_lower_yaw import StageGeometry, StageRunner, ROOT, MEASURED_JOINT_SPEED_LIMIT_DEG_S


class StageTests(unittest.TestCase):
    def setUp(self):
        self.folder=ROOT/'robot/tests/fixtures/lower20_then_yaw_20260919'
        self.guard=SimpleNamespace(stage='lower',start_q=np.zeros(6),path_tolerance=.0015,
            yaw_scope_review_passed=True,inspect=lambda q:{'gaps_mm':{'ceiling':6.}})

    def test_historical_stage_audit_is_invalidated_by_global_policy(self):
        with self.assertRaisesRegex(RuntimeError,'current global clearance policy'):
            StageGeometry(self.folder,'lower')

    def state(self):
        now=time.monotonic()
        return SimpleNamespace(q=self.guard.start_q,qd=np.zeros(6),scale=100.,program=True,
            received=now,scale_received=now,feedback_received=now,feedback_error=0.,
            active=False,sent_at=now-1.,guard=self.guard)

    def test_stale_feedback_program_off_and_speed_abort(self):
        for key,value in [('received',0.),('scale_received',0.),('program',False),
                          ('scale',0.),('qd',np.full(6,np.nan)),('qd',np.full(6,np.deg2rad(10.1)))]:
            state=self.state();setattr(state,key,value)
            with self.subTest(key=key):
                with self.assertRaises(RuntimeError):StageRunner.live(state)

    def test_tracking_error_and_missing_action_feedback_abort(self):
        for key,value in [('feedback_received',0.),('feedback_error',.002)]:
            state=self.state();state.active=True;setattr(state,key,value)
            with self.assertRaises(RuntimeError):StageRunner.live(state)

    def test_speed_error_preserves_joint_and_numeric_trigger(self):
        state=self.state();state.qd[3]=np.deg2rad(10.25)
        with self.assertRaisesRegex(RuntimeError,'wrist_1_joint speed 10.250000 deg/s exceeds 10.000'):
            StageRunner.live(state)

    def test_lower_speed_threshold_checks_both_signs_and_every_joint(self):
        self.assertEqual(MEASURED_JOINT_SPEED_LIMIT_DEG_S['lower'],10.)
        for joint in range(6):
            for sign in [-1.,1.]:
                with self.subTest(joint=joint,sign=sign):
                    state=self.state();state.qd[joint]=np.deg2rad(sign*10.)
                    StageRunner.live(state)
                    state.qd[joint]=np.deg2rad(sign*10.001)
                    with self.assertRaisesRegex(RuntimeError,'exceeds 10.000'):
                        StageRunner.live(state)

    def test_yaw_speed_threshold_is_unchanged(self):
        self.assertEqual(MEASURED_JOINT_SPEED_LIMIT_DEG_S['yaw'],9.)
        for sign in [-1.,1.]:
            state=self.state();state.guard=SimpleNamespace(stage='yaw',path_tolerance=.006,
                inspect=lambda q:{'gaps_mm':{'ceiling':6.}})
            state.qd[3]=np.deg2rad(sign*9.)
            StageRunner.live(state)
            state.qd[3]=np.deg2rad(sign*9.001)
            with self.assertRaisesRegex(RuntimeError,'exceeds 9.000'):
                StageRunner.live(state)

    def test_successful_goal_no_longer_requires_action_feedback(self):
        state=self.state();state.feedback_received=0.
        StageRunner.live(state)

    def test_loss_of_independent_recorder_aborts(self):
        def fail(age):raise RuntimeError('Direct RTDE recorder stale')
        state=self.state();state.active=True
        state.recorder=SimpleNamespace(check_fresh=fail)
        with self.assertRaisesRegex(RuntimeError,'recorder stale'):StageRunner.live(state)

    def test_yaw_without_scope_review_never_reaches_action_client(self):
        state=SimpleNamespace(guard=SimpleNamespace(stage='yaw',yaw_scope_review_passed=False))
        with self.assertRaisesRegex(RuntimeError,'requires successful enlarged-envelope'):
            StageRunner.execute(state)


if __name__=='__main__':unittest.main()
