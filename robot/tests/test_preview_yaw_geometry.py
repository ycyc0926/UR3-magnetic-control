"""Offline helper tests: no robot connection, ROS nodes, or device access."""
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from preview_yaw_geometry import box_corners, fixed_sphere_target


class YawGeometryTests(unittest.TestCase):
    def test_motor_dimensions_follow_axis_convention(self):
        vertices = box_corners([-.0215, .004, .0195], [.0215, .046, .0625])
        self.assertEqual(vertices.shape, (8, 3))
        np.testing.assert_allclose(np.ptp(vertices, axis=0), [.043, .042, .043])
        np.testing.assert_allclose(np.mean(vertices, axis=0), [0., .025, .041], atol=1e-15)

    def test_invalid_box_fails(self):
        for low, high in [([0, 0, 0], [1, 0, 1]), ([0, 0, 0], [1, float('nan'), 1]),
                          ([0, 0], [1, 1, 1])]:
            with self.assertRaises(ValueError):box_corners(low, high)

    def test_pure_world_yaw_fixes_sphere_and_all_rigid_tool_heights(self):
        start = np.eye(4)
        start[:3, :3] = Rotation.from_euler('xyz', [.2, -1.2, .6]).as_matrix()
        start[:3, 3] = [.15, .05, .45]
        offset = np.array([0., -.062, .041])
        points = box_corners([-.06, -.09, -.01], [.06, .05, .08])
        sphere = start[:3, 3] + start[:3, :3]@offset
        original_z = points@start[2, :3] + start[2, 3]
        for degrees in np.linspace(-90, 90, 25):
            target = fixed_sphere_target(start, offset, np.deg2rad(degrees))
            np.testing.assert_allclose(target[:3, 3]+target[:3, :3]@offset, sphere, atol=1e-12)
            np.testing.assert_allclose(points@target[2, :3]+target[2, 3], original_z, atol=1e-12)

    def test_yaw_direction_and_shaft_elevation(self):
        start = np.eye(4)
        start[:3, :3] = np.array([[0., -1., 0.], [0., 0., 1.], [-1., 0., 0.]])
        target = fixed_sphere_target(start, np.array([0., -.062, .041]), np.pi/2)
        np.testing.assert_allclose(target[:3, :3]@np.array([0., -1., 0.]), [0., 1., 0.], atol=1e-12)


if __name__ == '__main__':unittest.main()
