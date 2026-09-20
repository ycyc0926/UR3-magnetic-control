"""Offline failure cases for the new XY diagnostic; no hardware connections."""
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
from scipy.spatial.transform import Rotation

from prepare_xy_alignment import validate_snapshot, xy_target, XYGeometry, check_collision
from ur3_magnetic_control.clearance_policy import load_clearance_limits_m


def sample():
    return {'captured_at': '2026-09-19T10:55:13.400000+00:00',
            'snapshot': {'actual_q': [0.]*6, 'actual_qd': [0.]*6,
                         'actual_TCP_pose': [.1, -.3, .4, 0, 1.5, 0], 'actual_TCP_speed': [0.]*6,
                         'tcp_offset': [0, -.062, .041, 0, 0, 0], 'payload': .32,
                         'safety_mode': 1, 'robot_mode': 7, 'runtime_state': 1},
            'vision': {'utc': '2026-09-19T10:55:13.360000+00:00', 'detected': True, 'stream_stale': False,
                       'tracking_reason': 'tracked', 'consecutive_detections': 40,
                       'world_xy_mm': [200., 100.]}}


class SnapshotTests(unittest.TestCase):
    def test_target_units_and_complete_state(self):
        q, target = validate_snapshot(sample())
        self.assertEqual(q.shape, (6,))
        np.testing.assert_allclose(target, [.2, .1])

    def test_stale_lost_or_unstable_vision_rejected(self):
        for key, value in [('utc', '2026-09-19T10:55:10+00:00'),
                           ('utc', '2026-09-19T10:55:14+00:00'),
                           ('detected', False), ('stream_stale', True),
                           ('tracking_reason', 'locked_out'), ('consecutive_detections', 5),
                           ('world_xy_mm', [float('nan'), 1])]:
            with self.subTest(key=key, value=value):
                data = sample(); data['vision'][key] = value
                with self.assertRaises(ValueError): validate_snapshot(data)

    def test_moving_robot_wrong_tcp_payload_or_safety_rejected(self):
        for key, value in [('actual_qd', [.01]*6), ('actual_TCP_speed', [.001]*6),
                           ('tcp_offset', [0, 0, .1, 0, 0, 0]), ('payload', .5),
                           ('safety_mode', 3), ('runtime_state', 2), ('robot_mode', 5),
                           ('actual_q', [float('nan')]*6), ('actual_TCP_pose', [1, 2, 3])]:
            with self.subTest(key=key):
                data = sample(); data['snapshot'][key] = value
                with self.assertRaises(ValueError): validate_snapshot(data)

    def test_naive_timestamp_rejected(self):
        data = sample(); data['captured_at'] = '2026-09-19T10:55:13.400000'
        with self.assertRaises(ValueError): validate_snapshot(data)


class CorridorTests(unittest.TestCase):
    def test_xy_target_preserves_tool_pose_and_sphere_height(self):
        pose = np.eye(4); pose[:3, :3] = Rotation.from_euler('xyz', [.3, 1.2, .8]).as_matrix()
        pose[:3, 3] = [.2, .1, .4]
        offset = np.array([0, -.062, .041])
        sphere = pose[:3, 3]+pose[:3, :3]@offset
        desired = sphere[:2]+[-.016, .003]
        result = xy_target(pose, sphere, desired)
        result_sphere = result[:3, 3]+result[:3, :3]@offset
        np.testing.assert_allclose(result_sphere, np.r_[desired, sphere[2]])
        np.testing.assert_array_equal(result[:3, :3], pose[:3, :3])

    def test_unbounded_zero_and_nonfinite_target_rejected(self):
        for target in [[0, 0], [.031, 0], [float('nan'), 0], [1]]:
            with self.subTest(target=target):
                with self.assertRaises(ValueError): xy_target(np.eye(4), np.zeros(3), target)

    def guard(self, displacement, rotation_deg=0., gap_override=None):
        guard = object.__new__(XYGeometry)
        guard.offset = np.zeros(3)
        guard.start_pose = np.eye(4)
        guard.start_sphere = np.zeros(3)
        guard.direction = np.array([1., 0., 0.])
        guard.distance = .02
        guard.gap_limits = load_clearance_limits_m()
        pose = np.eye(4); pose[:3, 3] = displacement
        pose[:3, :3] = Rotation.from_euler('z', rotation_deg, degrees=True).as_matrix()
        gaps = {'ceiling': .03, 'left': .2, 'right': .2, 'table': .02}
        gaps.update(gap_override or {})
        guard.bounds = lambda q: (gaps, pose)
        return guard

    def test_intended_translation_is_allowed_but_off_path_motion_is_rejected(self):
        result = self.guard([.016, 0, 0]).inspect(np.zeros(6), tight=True)
        self.assertAlmostEqual(result['progress_mm'], 16.)
        for point in [[.01, .0008, 0], [.01, 0, .0008], [-.001, 0, 0], [.021, 0, 0]]:
            with self.subTest(point=point):
                with self.assertRaises(ValueError): self.guard(point).inspect(np.zeros(6))

    def test_orientation_or_any_plane_violation_rejected(self):
        with self.assertRaises(ValueError): self.guard([.01, 0, 0], .2).inspect(np.zeros(6))
        for boundary in ['ceiling', 'left', 'right', 'table']:
            with self.subTest(boundary=boundary):
                with self.assertRaises(ValueError):
                    self.guard([.01, 0, 0], gap_override={boundary: .001}).inspect(np.zeros(6))


class CollisionOutputTests(unittest.TestCase):
    def test_missing_or_positive_collision_results_cannot_pass(self):
        header = 'index,bare_collision,tool_collision,joint_bounds_ok,bare_pairs,tool_pairs,fx,fy,fz,qx,qy,qz,qw\n'
        for stdout, stderr in [(header, 'FCL positive control passed'),
                               (header+'0,0,1,1,,,,,,,,,\n', 'FCL positive control passed'),
                               (header+'0,0,0,0,,,,,,,,,\n', 'FCL positive control passed'),
                               (header+'0,0,0,1,,,,,,,,,\n', '')]:
            with self.subTest(stdout=stdout, stderr=stderr), tempfile.TemporaryDirectory() as tmp:
                result = type('Result', (), {'stdout': stdout, 'stderr': stderr})()
                with patch('prepare_xy_alignment.subprocess.run', return_value=result):
                    with self.assertRaises(ValueError):
                        check_collision(Path('offline'), Path('model'), Path('semantic'), {},
                                        [np.zeros(6)], Path(tmp)/'collision.csv')


if __name__ == '__main__':
    unittest.main()
