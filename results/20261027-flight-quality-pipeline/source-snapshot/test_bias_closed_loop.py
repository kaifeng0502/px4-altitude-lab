"""Check that bounded outer integration removes a persistent model disturbance."""
import unittest
from altitude_mpc import AltitudeMPC,MPCConfig
from flight_quality import HeightBiasCompensator


class BiasClosedLoopBehavior(unittest.TestCase):
    def test_reduces_residual_error_without_changing_the_plant(self):
        def fly(integral):
            config=MPCConfig(acceleration_gain=1.55,acceleration_bias=.04,max_velocity=1.,
                             min_acceleration=-.8,max_acceleration=.8,max_jerk=2.)
            mpc=AltitudeMPC(config);correction=HeightBiasCompensator() if integral else None
            h,v,u=2.,0.,0.
            for k in range(300):
                target=4.
                c=correction.update(h,v,target,k*.1,abs(u)>.78) if correction else 0.
                u=mpc.solve(h,v,target+c)['acceleration_m_s2']
                # Identical unmodeled acceleration disturbance in both trials.
                actual=1.55*u+.04+.12
                h,v=h+.1*v+.005*actual,v+.1*actual
                self.assertLessEqual(abs(u),.8001)
            return abs(h-4.)
        plain=fly(False);compensated=fly(True)
        self.assertGreater(plain,.01)
        self.assertLess(compensated,plain*.4)


if __name__=='__main__':unittest.main()
