import unittest
from flight_quality import ground_quality,HeightBiasCompensator


class FlightQualityBehavior(unittest.TestCase):
    def test_waits_for_real_state_convergence(self):
        times=[i*.02 for i in range(501)]
        self.assertTrue(ground_quality([(t,.3,.001) for t in times],10.)['ready'])
        self.assertFalse(ground_quality([(t,.3,.1) for t in times],10.)['ready'])
        self.assertFalse(ground_quality([(t,.03*t,0.) for t in times],10.)['ready'])

    def test_slow_bias_correction_is_bounded_gated_and_reset(self):
        c=HeightBiasCompensator()
        for k in range(200):c.update(3.95,0.,4.,k*.1)
        self.assertGreater(c.correction,0.)
        self.assertLessEqual(c.correction,.12)
        value=c.correction
        c.update(3.,1.,4.,20.)
        self.assertEqual(c.correction,value)
        c.update(3.95,0.,4.,20.1,saturated=True)
        self.assertEqual(c.correction,value)
        c.update(4.,0.,2.,20.2)
        self.assertEqual(c.correction,0.)


if __name__=='__main__':unittest.main()
