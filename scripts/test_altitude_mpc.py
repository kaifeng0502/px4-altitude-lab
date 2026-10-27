"""Numerical behavior checks, independent of PX4 and the graphical simulator."""
import unittest
import numpy as np
from altitude_mpc import AltitudeMPC, MPCConfig


class MPCBehavior(unittest.TestCase):
    def test_hover_equilibrium(self):
        result = AltitudeMPC().solve(2., 0., 2.)
        self.assertAlmostEqual(result['acceleration_m_s2'], 0., places=6)

    def test_step_converges_and_respects_bounds(self):
        controller = AltitudeMPC()
        h, v, previous = 2., 0., 0.
        peak_h = h
        braking_while_rising = False
        for _ in range(100):
            result = controller.solve(h, v, 4.)
            a = result['acceleration_m_s2']
            self.assertLessEqual(abs(a), 2.0001)
            self.assertLessEqual(abs(a-previous), .5001)
            self.assertLessEqual(np.max(np.abs(np.array(result['predicted_states'])[1:, 1])-np.array(result['predicted_speed_slack_m_s'])), 1.5001)
            braking_while_rising |= a < -.05 and v > .1
            h, v = h + .1*v + .005*a, v + .1*a
            peak_h = max(peak_h, h)
            previous = a
        self.assertLess(abs(h-4.), .005)
        self.assertLess(abs(v), .005)
        self.assertLess(peak_h, 4.05)
        self.assertTrue(braking_while_rising)

    def test_calibrated_hover_and_complete_step(self):
        c = MPCConfig(acceleration_gain=1.55, acceleration_bias=.04)
        controller = AltitudeMPC(c)
        controller.previous_u = -c.acceleration_bias/c.acceleration_gain
        equilibrium = controller.solve(2., 0., 2.)
        self.assertAlmostEqual(equilibrium['acceleration_m_s2'], -.04/1.55, places=5)
        h, v = 2., 0.
        peak = h
        for _ in range(100):
            result = controller.solve(h, v, 4.)
            command = result['acceleration_m_s2']
            actual_a = c.acceleration_gain*command+c.acceleration_bias
            h, v = h+.1*v+.005*actual_a, v+.1*actual_a
            peak = max(peak, h)
        self.assertLess(abs(h-4.), .005)
        self.assertLess(abs(v), .005)
        self.assertLess(peak, 4.05)
        with self.assertRaises(ValueError):
            AltitudeMPC(MPCConfig(acceleration_gain=0.))

    def test_downward_step_and_invalid_state(self):
        controller = AltitudeMPC()
        self.assertLess(controller.solve(4., 0., 2.)['acceleration_m_s2'], 0.)
        with self.assertRaises(ValueError):
            controller.solve(float('nan'), 0., 2.)
        # Excess measured speed is recoverable through explicit soft speed bounds.
        recovery = AltitudeMPC().solve(2., 2., 4.)
        self.assertLess(recovery['acceleration_m_s2'], 0.)
        self.assertGreater(recovery['max_speed_slack_m_s'], .1)
        self.assertLessEqual(abs(recovery['acceleration_m_s2']), .5001)


if __name__ == '__main__':
    unittest.main()
