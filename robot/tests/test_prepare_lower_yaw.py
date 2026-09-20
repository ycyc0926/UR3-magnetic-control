import unittest
import numpy as np
from prepare_lower_yaw import timed_phase
from prepare_yaw_pilot import quintic


class LowerYawTests(unittest.TestCase):
    def test_endpoints_stop_and_times_are_ordered(self):
        q = np.linspace(np.zeros(6),np.full(6,.02),41)
        points = timed_phase(q,1.,10.)
        self.assertEqual(points[0]['time_s'],1.)
        self.assertEqual(points[-1]['time_s'],11.)
        self.assertTrue(np.all(np.diff([p['time_s'] for p in points]) > 0))
        for point in [points[0],points[-1]]:
            np.testing.assert_allclose(point['velocity_rad_s'],0.,atol=1e-12)
            np.testing.assert_allclose(point['acceleration_rad_s2'],0.,atol=1e-12)

    def test_two_phases_have_a_stationary_gap(self):
        q = np.linspace(np.zeros(6),np.full(6,.02),41)
        descent = timed_phase(q,1.,10.)
        rotation = timed_phase(q+.02,13.,45.)
        for t in [11.,12.,13.]:
            actual,velocity,acceleration = quintic(descent[-1],rotation[0],t)
            np.testing.assert_allclose(actual,q[-1],atol=1e-12)
            np.testing.assert_allclose(velocity,0.,atol=1e-12)
            np.testing.assert_allclose(acceleration,0.,atol=1e-12)

    def test_bad_inputs_fail(self):
        for q,start,duration in [(np.zeros((2,6)),1.,10.),(np.full((3,6),np.nan),1.,10.),
                                (np.zeros((3,6)),1.,0.),(np.zeros((3,6)),-1.,10.)]:
            with self.assertRaises(ValueError):timed_phase(q,start,duration)


if __name__ == '__main__':unittest.main()
