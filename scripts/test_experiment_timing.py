"""Exercise the telemetry-to-command ordering without launching PX4 or sockets."""
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
from run_experiment import Experiment


class FreshFeedbackBehavior(unittest.TestCase):
    def test_receive_first_and_send_acceleration_without_duplicate_solve(self):
        events=[]
        message=SimpleNamespace(time_boot_ms=1000,x=0.,y=0.,z=-1.4,vz=-.2,
            get_srcSystem=lambda:5,get_type=lambda:'LOCAL_POSITION_NED')
        queued=[message]
        def receive(**kwargs):
            if queued:
                events.append('receive');return queued.pop()
            return None
        def solve(h,v,target):
            events.append('solve')
            self.assertAlmostEqual(h,1.4);self.assertAlmostEqual(v,.2)
            return {'acceleration_m_s2':.2,'solve_ms':.1,'iterations':1,
                    'constraint_violation':0.,'max_speed_slack_m_s':0.}
        packet=Mock(side_effect=lambda *args:events.append('send'))
        link=SimpleNamespace(recv_match=receive,mav=SimpleNamespace(
            heartbeat_send=Mock(),set_position_target_local_ned_send=packet))
        mpc=SimpleNamespace(config=SimpleNamespace(dt=.1),solve=Mock(side_effect=solve))
        with tempfile.TemporaryDirectory() as temp:
            experiment=Experiment(link,SimpleNamespace(poll=lambda:None),Path(temp),'mpc',mpc,'scheduled')
            experiment.target=(0.,0.,-2.,0.)
            experiment.mpc_active=True
            try:
                experiment.pump(.006)
                experiment.pump(.006)
                self.assertLess(events.index('receive'),events.index('solve'))
                self.assertLess(events.index('solve'),events.index('send'))
                self.assertEqual(mpc.solve.call_count,1)
                args=packet.call_args_list[0].args
                self.assertEqual(args[0],1000)
                self.assertEqual(args[4],2300)  # z/vz ignored, az active
                self.assertAlmostEqual(args[13],-.2)  # NED sign conversion
                experiment.controller_file.flush()
                self.assertIn('state_receive_age_wall_ms',(Path(temp)/'controller.csv').read_text())
            finally:
                for file in [experiment.csv_file,experiment.text_file,experiment.controller_file,experiment.prediction_file]:file.close()


if __name__=='__main__':unittest.main()
