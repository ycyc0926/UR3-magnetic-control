"""Geometry-only tests; imports do not contact hardware."""
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from preview_yaw_clearance import surface_bounds, yaw_to_positive_y
from preview_yaw_geometry import box_corners


class ClearanceTests(unittest.TestCase):
    def test_box_bounds_include_rotation(self):
        pose = np.eye(4)
        pose[:3, :3] = Rotation.from_euler('z', 90, degrees=True).as_matrix()
        pose[:3, 3] = [1, 2, 3]
        low, high = surface_bounds(pose, 'box', box_corners([-1, -2, -3], [1, 2, 3]))
        np.testing.assert_allclose(low, [-1, 1, 0])
        np.testing.assert_allclose(high, [3, 3, 6])

    def test_sphere(self):
        low, high = surface_bounds(np.eye(4), 'sphere', .01)
        np.testing.assert_allclose(low, [-.01]*3)
        np.testing.assert_allclose(high, [.01]*3)

    def test_cylinder(self):
        low, high = surface_bounds(np.eye(4), 'cylinder', (.02, .1))
        np.testing.assert_allclose(low, [-.02, -.02, -.05])
        np.testing.assert_allclose(high, [.02, .02, .05])

    def test_yaw_target(self):
        self.assertAlmostEqual(yaw_to_positive_y([1, 0, 0]), 90.)
        direction = Rotation.from_euler('z', -1.7, degrees=True).apply([1, 0, 0])
        self.assertAlmostEqual(yaw_to_positive_y(direction), 91.7)

    def test_vertical_or_wrong_direction_rejected(self):
        for axis in [[0, 0, 1], [0, 1, 0], [-1, 0, 0], [float('nan'), 0, 0]]:
            with self.assertRaises(ValueError):yaw_to_positive_y(axis)


if __name__ == '__main__':unittest.main()
