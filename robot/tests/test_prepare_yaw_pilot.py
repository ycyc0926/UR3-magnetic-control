import unittest
import numpy as np
from prepare_yaw_pilot import pilot_points, quintic, derivative_peaks, smooth_full_turn_points, JOINT_SPEED_LIMIT, JOINT_ACCEL_LIMIT


class PilotTests(unittest.TestCase):
    def nodes(self, scale=1.):
        return [{'yaw_deg': value, 'q_rad': [float(np.deg2rad(value)*scale)]*6}
                for value in np.arange(0., 5.01, .5)]

    def test_pilot_stops_at_five_degrees(self):
        points = pilot_points(self.nodes())
        self.assertEqual(points[-1]['yaw_node_deg'], 5.)
        self.assertEqual(points[-1]['time_s'], 23.)
        for point in points:
            self.assertEqual(point['velocity_rad_s'], [0.]*6)
            self.assertEqual(point['acceleration_rad_s2'], [0.]*6)

    def test_quintic_boundary_conditions(self):
        points = pilot_points(self.nodes())
        a, b = points[1:3]
        for point in [a, b]:
            q, v, acc = quintic(a, b, point['time_s'])
            np.testing.assert_allclose(q, point['q_rad'], atol=1e-12)
            np.testing.assert_allclose(v, 0., atol=1e-12)
            np.testing.assert_allclose(acc, 0., atol=1e-12)

    def test_large_joint_change_is_slowed(self):
        points = pilot_points(self.nodes(scale=10.))
        self.assertGreater(points[-1]['time_s'], 23.)
        for a, b in zip(points, points[1:]):
            for time in np.linspace(a['time_s'], b['time_s'], 501):
                q, v, acc = quintic(a, b, time)
                self.assertLessEqual(np.max(np.abs(v)), JOINT_SPEED_LIMIT+1e-12)
                self.assertLessEqual(np.max(np.abs(acc)), JOINT_ACCEL_LIMIT+1e-12)

    def test_missing_endpoint_rejected(self):
        with self.assertRaises(ValueError):pilot_points(self.nodes()[:-1])

    def test_sparse_or_invalid_nodes_rejected(self):
        with self.assertRaises(ValueError):pilot_points(self.nodes()[::2])
        nodes = self.nodes();nodes[3]['q_rad'][0] = float('nan')
        with self.assertRaises(ValueError):pilot_points(nodes)

    def test_nonzero_derivative_endpoints(self):
        a, b = pilot_points(self.nodes())[1:3]
        a = dict(a, velocity_rad_s=[.003]*6, acceleration_rad_s2=[-.001]*6)
        b = dict(b, velocity_rad_s=[.002]*6, acceleration_rad_s2=[.004]*6)
        for point in [a, b]:
            values = quintic(a, b, point['time_s'])
            for actual, key in zip(values, ['q_rad', 'velocity_rad_s', 'acceleration_rad_s2']):
                np.testing.assert_allclose(actual, point[key], atol=1e-12)

    def test_derivative_peaks(self):
        a, b = pilot_points(self.nodes())[1:3]
        velocity, acceleration = derivative_peaks(a, b)
        delta = np.deg2rad(.5)
        self.assertAlmostEqual(velocity, 1.875*delta/2)
        self.assertAlmostEqual(acceleration, 10/np.sqrt(3)*delta/4)

    def test_smooth_full_turn_endpoints_and_duration(self):
        nodes = [{'yaw_deg': float(value), 'q_rad': [float(np.deg2rad(value))]*6}
                 for value in np.arange(0., 90.01, .5)]
        points = smooth_full_turn_points(nodes)
        self.assertEqual(points[-1]['time_s'], 48.)
        for point in [points[0], points[-1]]:
            np.testing.assert_allclose(point['velocity_rad_s'], 0., atol=1e-12)
            np.testing.assert_allclose(point['acceleration_rad_s2'], 0., atol=1e-12)


if __name__ == '__main__':unittest.main()
