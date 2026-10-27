import unittest
from control_clock import ControlClock


class ClockBehavior(unittest.TestCase):
    def test_jitter_does_not_accumulate_phase_drift(self):
        clock=ControlClock(.1)
        deadlines=[]
        for t in [10.,10.02,10.08,10.102,10.198,10.22,10.298,10.304]:
            event=clock.poll(t)
            if event:deadlines.append(event['scheduled_boot_s'])
        for actual,expected in zip(deadlines,[10.,10.1,10.2,10.3]):
            self.assertAlmostEqual(actual,expected)
        self.assertEqual(len(deadlines),4)

    def test_nearby_sample_is_used_without_waiting_for_next_frame(self):
        clock=ControlClock(.1,early_tolerance=.004)
        clock.poll(0.)
        self.assertIsNone(clock.poll(.08))
        early=clock.poll(.096)
        self.assertAlmostEqual(early['deadline_phase_error_sim_ms'],-4.)
        self.assertIsNone(clock.poll(.104))
        self.assertAlmostEqual(clock.poll(.204)['deadline_phase_error_sim_ms'],4.)
        self.assertAlmostEqual(clock.poll(.296)['scheduled_boot_s'],.3)

    def test_overrun_skips_ticks_without_catchup_burst(self):
        clock=ControlClock(.1)
        clock.poll(0.)
        event=clock.poll(.36)
        self.assertEqual(event['skipped_control_ticks'],2)
        self.assertAlmostEqual(event['scheduled_boot_s'],.3)
        self.assertIsNone(clock.poll(.36))
        self.assertAlmostEqual(clock.poll(.401)['scheduled_boot_s'],.4)
        with self.assertRaises(RuntimeError):clock.poll(.2)


if __name__=='__main__':unittest.main()
