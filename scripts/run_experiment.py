#!/usr/bin/env python3
"""Launch an owned PX4 SITL instance, fly a 2 m -> 4 m step, land, save logs.

Run in the Linux VM using the project's virtual environment. This script has
no remote vehicle option: it starts its own SITL and uses loopback UDP only.
"""
import argparse
import csv
import fcntl
import datetime as dt
import json
import math
import os
from pathlib import Path
import signal
import statistics
import subprocess
import time
from collections import deque

from pymavlink import mavutil
from control_clock import ControlClock
from flight_quality import ground_quality, HeightBiasCompensator

ROOT = Path(__file__).resolve().parents[1]
INSTANCE = 4
SYSTEM = INSTANCE + 1


class Experiment:
    def __init__(self, connection, process, out, controller="pid", mpc=None, timing_mode="scheduled", ground_init="settled", bias_compensation="off"):
        self.link, self.process, self.out = connection, process, out
        self.messages = {}
        self.acks = {}
        self.events = []
        self.boot_ms = 0
        self.target = None
        self.ground_z = 0.0
        self.positions = deque(maxlen=2000)
        self.vertical_history = deque(maxlen=2000)
        self.ground_init = ground_init
        self.bias_compensator = HeightBiasCompensator() if bias_compensation == "integral" else None
        self.last_heartbeat = 0
        self.last_setpoint = 0
        self.timing_mode = timing_mode
        self.control_clock = ControlClock(mpc.config.dt, early_tolerance=.004) if mpc is not None else None
        self.last_decision_send_wall = None
        self.timing_origin_wall = time.monotonic()
        self.controller_name = controller
        self.mpc = None
        self.mpc_active = False
        self.mpc_last_boot_s = None
        self.mpc_acceleration = 0.0
        self.mpc_records = 0
        self.mpc_max_solve_ms = 0.0
        self.mpc_max_interval_s = 0.0
        self.prediction_file = None
        self.controller_file = None
        if controller == 'mpc':
            self.mpc = mpc
            self.prediction_file = (out / 'mpc-predictions.jsonl').open('w')
            self.controller_file = (out / 'controller.csv').open('w', newline='')
            self.controller_csv = csv.writer(self.controller_file)
            self.controller_csv.writerow(['boot_s', 'target_height_m', 'height_m', 'up_velocity_m_s',
                'up_acceleration_command_m_s2', 'solve_ms', 'iterations', 'constraint_violation',
                'update_interval_sim_s', 'max_predicted_speed_slack_m_s',
                'scheduled_boot_s', 'deadline_lateness_sim_ms', 'skipped_control_ticks',
                'state_receive_age_wall_ms', 'solve_start_wall_s', 'command_send_wall_s',
                'command_send_spacing_wall_ms', 'deadline_phase_error_sim_ms',
                'height_reference_correction_m', 'mpc_effective_reference_m', 'bias_integrator_active'])
        self.csv_file = (out / 'telemetry.csv').open('w', newline='')
        self.csv = csv.writer(self.csv_file)
        self.csv.writerow(['boot_s', 'ground_relative_height_m', 'up_velocity_m_s',
                           'target_height_m', 'ground_reference_ned_z_m'])
        self.text_file = (out / 'status_text.txt').open('w')

    def event(self, name, **data):
        record = {'event': name, 'boot_s': self.boot_ms / 1000,
                  'utc': dt.datetime.now(dt.timezone.utc).isoformat(), **data}
        self.events.append(record)
        (self.out / 'events.json').write_text(json.dumps(self.events, indent=2))
        print(json.dumps(record), flush=True)

    def pump(self, seconds=0.05):
        if self.timing_mode == 'legacy':
            return self._pump_legacy(seconds)
        end = time.monotonic()+seconds
        while time.monotonic()<end:
            if self.process.poll() is not None:
                raise RuntimeError('Owned PX4 simulation exited; inspect px4-console.log')
            # Drain queued packets before choosing a state or deciding to solve.
            self._receive_messages()
            now=time.monotonic()
            if now-self.last_heartbeat>1:
                self.link.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,0,0,mavutil.mavlink.MAV_STATE_ACTIVE)
                self.last_heartbeat=now
            decision=None
            if self.target is not None and self.mpc_active:
                if now-self.last_position_wall>.5:
                    raise RuntimeError('Stale position feedback during MPC')
                boot_s=self.boot_ms/1000
                decision=self.control_clock.poll(boot_s)
                if decision is not None:
                    interval=boot_s-self.mpc_last_boot_s if self.mpc_last_boot_s is not None else 0.
                    if interval>.35:
                        raise RuntimeError(f'MPC update gap too long: {interval:.3f} simulation seconds')
                    pos=self.messages['LOCAL_POSITION_NED']
                    height=self.ground_z-pos.z
                    target=self.ground_z-self.target[2]
                    solve_start=time.monotonic()
                    received_age=(solve_start-self.last_position_wall)*1000
                    correction=0.
                    if self.bias_compensator is not None:
                        config=self.mpc.config
                        saturated=(self.mpc_acceleration>=config.max_acceleration-.02 or self.mpc_acceleration<=config.min_acceleration+.02)
                        correction=self.bias_compensator.update(height,-pos.vz,target,boot_s,saturated)
                    effective_reference=target+correction
                    result=self.mpc.solve(height,-pos.vz,effective_reference)
                    self.mpc_acceleration=result['acceleration_m_s2']
                    self.mpc_last_boot_s=boot_s
                    self.mpc_records+=1
                    self.mpc_max_solve_ms=max(self.mpc_max_solve_ms,result['solve_ms'])
                    self.mpc_max_interval_s=max(self.mpc_max_interval_s,interval)
            # Send a new optimized command immediately. Repeated setpoints keep
            # Offboard alive independently of the simulation-time QP schedule.
            if self.target is not None and (decision is not None or time.monotonic()-self.last_setpoint>=.05):
                x,y,z,yaw=self.target
                mask=8|16|32|64|128|256|2048
                az=0.
                if self.mpc_active:
                    mask=4|8|16|32|64|128|2048
                    az=-self.mpc_acceleration
                self.link.mav.set_position_target_local_ned_send(self.boot_ms,SYSTEM,1,
                    mavutil.mavlink.MAV_FRAME_LOCAL_NED,mask,x,y,z,0,0,0,0,0,az,yaw,0)
                sent=time.monotonic()
                self.last_setpoint=sent
                if decision is not None:
                    spacing=(sent-self.last_decision_send_wall)*1000 if self.last_decision_send_wall is not None else 0.
                    self.last_decision_send_wall=sent
                    # Age is LOCAL time since this process received the state;
                    # it does not include estimator or network age before receipt.
                    timing={**decision,'state_receive_age_wall_ms':received_age,
                            'solve_start_wall_s':solve_start-self.timing_origin_wall,
                            'command_send_wall_s':sent-self.timing_origin_wall,
                            'command_send_spacing_wall_ms':spacing,
                            'height_reference_correction_m':correction,
                            'mpc_effective_reference_m':effective_reference,
                            'bias_integrator_active':self.bias_compensator.active if self.bias_compensator else False}
                    self.controller_csv.writerow([boot_s,target,height,-pos.vz,
                        self.mpc_acceleration,result['solve_ms'],result['iterations'],
                        result['constraint_violation'],interval,result['max_speed_slack_m_s'],
                        *[timing[k] for k in ['scheduled_boot_s','deadline_lateness_sim_ms',
                        'skipped_control_ticks','state_receive_age_wall_ms','solve_start_wall_s',
                        'command_send_wall_s','command_send_spacing_wall_ms','deadline_phase_error_sim_ms',
                        'height_reference_correction_m','mpc_effective_reference_m','bias_integrator_active']]])
                    # Log after sending so disk writes cannot delay this command.
                    self.prediction_file.write(json.dumps({'boot_s':boot_s,'target_height_m':target,
                                                           **timing,**result})+'\n')
                    self.controller_file.flush()
                    self.prediction_file.flush()
            time.sleep(.002)

    def _pump_legacy(self, seconds=0.05):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if self.process.poll() is not None:
                raise RuntimeError('Owned PX4 simulation exited; inspect px4-console.log')
            now = time.monotonic()
            if now - self.last_heartbeat > 1:
                self.link.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0,
                    mavutil.mavlink.MAV_STATE_ACTIVE)
                self.last_heartbeat = now
            if self.target is not None and now - self.last_setpoint >= 0.05:
                x, y, z, yaw = self.target
                az = 0.0
                # Position-only baseline, including MPC takeoff/landing setup.
                mask = 8 | 16 | 32 | 64 | 128 | 256 | 2048
                if self.mpc_active:
                    pos = self.messages['LOCAL_POSITION_NED']
                    if now - self.last_position_wall > 0.5:
                        raise RuntimeError('Stale position feedback during MPC')
                    boot_s = self.boot_ms / 1000
                    interval = boot_s - self.mpc_last_boot_s if self.mpc_last_boot_s is not None else 0.
                    if self.mpc_last_boot_s is None or interval >= self.mpc.config.dt - .01:
                        if interval > .35:
                            raise RuntimeError(f'MPC update gap too long: {interval:.3f} simulation seconds')
                        target_h = self.ground_z - z
                        result = self.mpc.solve(self.ground_z-pos.z, -pos.vz, target_h)
                        self.mpc_acceleration = result['acceleration_m_s2']
                        self.controller_csv.writerow([boot_s, target_h, self.ground_z-pos.z, -pos.vz,
                            self.mpc_acceleration, result['solve_ms'], result['iterations'],
                            result['constraint_violation'], interval, result['max_speed_slack_m_s'], *(['']*11)])
                        self.prediction_file.write(json.dumps({'boot_s': boot_s, 'target_height_m': target_h,
                                                               **result}) + '\n')
                        self.controller_file.flush()
                        self.prediction_file.flush()
                        self.mpc_last_boot_s = boot_s
                        self.mpc_records += 1
                        self.mpc_max_solve_ms = max(self.mpc_max_solve_ms, result['solve_ms'])
                        self.mpc_max_interval_s = max(self.mpc_max_interval_s, interval)
                    # x/y position + z acceleration; z position AND z velocity are
                    # ignored, so PX4's vertical P/PI loops cannot add feedback.
                    # MAVLink LOCAL_NED is downward-positive: a_z = -a_up.
                    mask = 4 | 8 | 16 | 32 | 64 | 128 | 2048
                    az = -self.mpc_acceleration
                self.link.mav.set_position_target_local_ned_send(
                    self.boot_ms, SYSTEM, 1, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                    mask, x, y, z, 0, 0, 0, 0, 0, az, yaw, 0)
                self.last_setpoint = now
            self._receive_messages()
            time.sleep(0.005)

    def _receive_messages(self):
        while True:
            msg = self.link.recv_match(blocking=False)
            if msg is None:
                break
            if msg.get_srcSystem() != SYSTEM:
                continue
            kind = msg.get_type()
            self.messages[kind] = msg
            if kind == 'LOCAL_POSITION_NED':
                self.boot_ms = msg.time_boot_ms
                self.last_position_wall = time.monotonic()
                self.positions.append((msg.time_boot_ms / 1000, msg.z))
                self.vertical_history.append((msg.time_boot_ms/1000,msg.z,msg.vz))
                self.csv.writerow([msg.time_boot_ms / 1000, self.ground_z - msg.z, -msg.vz,
                                   self.ground_z - self.target[2] if self.target else '', self.ground_z])
            elif kind == 'COMMAND_ACK':
                self.acks[msg.command] = msg.result
            elif kind == 'STATUSTEXT':
                self.text_file.write(f'{self.boot_ms / 1000:.3f} {msg.severity} {msg.text}\n')
                self.text_file.flush()
                print(f'PX4: {msg.text}', flush=True)

    def command(self, command, params=(), timeout=4):
        self.acks.pop(command, None)
        values = list(params) + [0] * (7 - len(params))
        self.link.mav.command_long_send(SYSTEM, 1, command, 0, *values)
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.pump()
            if command in self.acks:
                result = self.acks.pop(command)
                if result != mavutil.mavlink.MAV_RESULT_IN_PROGRESS:
                    return result
        return None

    def observe_sim_seconds(self, seconds):
        """Keep the observation window fixed in simulation time, even on a slow VM."""
        finish_ms = self.boot_ms + seconds * 1000
        deadline = time.monotonic() + max(60, seconds * 4)
        while self.boot_ms < finish_ms:
            if time.monotonic() > deadline:
                raise TimeoutError('Simulation clock or local-position telemetry stalled')
            self.pump()
            hb = self.messages['HEARTBEAT']
            if not (hb.base_mode & 128) or ((hb.custom_mode >> 16) & 255) != 6:
                raise RuntimeError('Unexpected arming or flight-mode change during observation')

    def wait_for(self, predicate, timeout, label):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.pump()
            if predicate():
                return
        raise TimeoutError(label)

    def run(self):
        self.wait_for(lambda: 'HEARTBEAT' in self.messages, 40, 'No PX4 heartbeat')
        heartbeat = self.messages['HEARTBEAT']
        if heartbeat.autopilot != mavutil.mavlink.MAV_AUTOPILOT_PX4:
            raise RuntimeError('Unexpected autopilot')
        for message_id, hz in [(32, 50 if self.mpc is not None and self.timing_mode == 'scheduled' else 20), (245, 5), (30, 10)]:
            self.command(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                         [message_id, 1e6 / hz])
        self.wait_for(lambda: 'LOCAL_POSITION_NED' in self.messages, 40, 'No local position')
        # Allow estimator alignment and preflight checks; stream a stationary target.
        pos = self.messages['LOCAL_POSITION_NED']
        self.target = (pos.x, pos.y, pos.z, 0.0)
        self.event('preflight')
        if self.ground_init == 'legacy':
            self.pump(15)
        else:
            begin=self.boot_ms/1000;deadline=time.monotonic()+190
            stable_since=None;last_report=-100.
            while True:
                self.pump()
                now=self.boot_ms/1000
                if self.messages['HEARTBEAT'].base_mode & 128:
                    raise RuntimeError('Vehicle unexpectedly armed during ground alignment')
                quality=ground_quality(self.vertical_history,now)
                ready=now-begin>=40. and quality['ready']
                stable_since=(stable_since if stable_since is not None else now) if ready else None
                if now-begin-last_report>=15:
                    self.event('ground_alignment_wait',elapsed_sim_s=now-begin,quality=quality)
                    last_report=now-begin
                if stable_since is not None and now-stable_since>=3.:
                    self.event('ground_alignment_complete',elapsed_sim_s=now-begin,quality=quality)
                    break
                if now-begin>150 or time.monotonic()>deadline:
                    raise TimeoutError('Ground height/velocity estimate did not settle; no takeoff')
        # The EKF's local z=0 need not coincide with the ground. Calibrate the
        # reference before arming; keep this reference fixed throughout the flight.
        ground_samples = [z for t, z in self.positions if t >= self.boot_ms / 1000 - 3]
        if len(ground_samples) < 20 or not all(math.isfinite(z) for z in ground_samples):
            raise RuntimeError('Insufficient valid samples for the ground datum')
        self.ground_z = statistics.median(ground_samples)
        pos = self.messages['LOCAL_POSITION_NED']
        self.target = (pos.x, pos.y, self.ground_z - 2, 0.0)
        self.event('ground_reference', ned_z_m=self.ground_z, sample_count=len(ground_samples))
        result = self.command(mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                              [mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 6, 0])
        if result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            raise RuntimeError(f'Offboard mode rejected: {result}')
        self.wait_for(lambda: ((self.messages['HEARTBEAT'].custom_mode >> 16) & 255) == 6,
                      5, 'Offboard mode not confirmed')
        for attempt in range(5):
            result = self.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, [1])
            if result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                break
            self.pump(5)
        else:
            raise RuntimeError(f'Arming rejected: {result}; no checks were bypassed')
        self.wait_for(lambda: bool(self.messages['HEARTBEAT'].base_mode & 128), 5,
                      'Arming not confirmed')
        self.event('takeoff', target_height_m=2.0)
        stable_since = None
        end = time.monotonic() + 50
        while time.monotonic() < end:
            self.pump()
            pos = self.messages['LOCAL_POSITION_NED']
            stable = abs(self.ground_z - pos.z - 2) < 0.10 and abs(pos.vz) < 0.10
            stable_since = (stable_since or time.monotonic()) if stable else None
            if stable_since and time.monotonic() - stable_since > 3:
                break
        else:
            raise TimeoutError('Could not establish 2 m hover')
        self.event('hover_stable', target_height_m=2.0)
        if self.mpc is not None:
            self.mpc_active = True
            self.event('controller_switch', controller='mpc', target_height_m=2.0,
                       config=self.mpc.metadata())
        # Equal pre-step observation time for both controllers.
        self.observe_sim_seconds(5)
        if self.mpc is not None:
            pos = self.messages['LOCAL_POSITION_NED']
            if abs(self.ground_z-pos.z-2) > .15 or abs(pos.vz) > .15:
                raise RuntimeError('MPC did not retain the initial 2 m hover')
        self.observe_sim_seconds(5)
        self.target = (self.target[0], self.target[1], self.ground_z - 4.0, self.target[3])
        self.event('height_step', target_height_m=4.0)
        self.observe_sim_seconds(30)
        pos = self.messages['LOCAL_POSITION_NED']
        # Flight completion and tracking quality are separate outcomes. Preserve
        # a noisy or unsettled response rather than selecting a favorable sample.
        self.event('step_observation_complete', final_height_m=self.ground_z - pos.z,
                   final_up_velocity_m_s=-pos.vz, observation_sim_s=30)
        self.land()

    def land(self):
        self.mpc_active = False
        result = self.command(mavutil.mavlink.MAV_CMD_NAV_LAND)
        if result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            raise RuntimeError(f'Landing rejected: {result}')
        self.target = None
        self.event('landing')
        self.wait_for(lambda: ('EXTENDED_SYS_STATE' in self.messages and
                      self.messages['EXTENDED_SYS_STATE'].landed_state ==
                      mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND), 50, 'Landing timeout')
        self.wait_for(lambda: not (self.messages['HEARTBEAT'].base_mode & 128),
                      15, 'Auto-disarm timeout')
        self.event('landed_disarmed')
        self.pump(2)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--label', default='baseline')
    parser.add_argument('--controller', choices=['pid', 'mpc'], default='pid')
    parser.add_argument('--mpc-model', choices=['nominal', 'calibrated'], default=None)
    parser.add_argument('--timing-mode', choices=['legacy', 'scheduled'], default='scheduled')
    parser.add_argument('--ground-init',choices=['legacy','settled'],default=None)
    parser.add_argument('--motion-profile',choices=['standard','gentle'],default='gentle')
    parser.add_argument('--bias-compensation',choices=['off','integral'],default='off')
    parser.add_argument('--simulator', choices=['sih', 'gazebo'], default='sih')
    args = parser.parse_args()
    args.ground_init=args.ground_init or ('settled' if args.simulator=='gazebo' else 'legacy')
    if args.bias_compensation=='integral' and args.controller!='mpc':
        parser.error('Integral reference compensation requires MPC')
    if args.bias_compensation=='integral' and args.timing_mode!='scheduled':
        parser.error('Integral reference compensation requires scheduled timing')
    args.mpc_model = args.mpc_model or ('calibrated' if args.simulator == 'gazebo' else 'nominal')
    if args.controller == 'mpc' and args.mpc_model == 'calibrated' and args.simulator != 'gazebo':
        parser.error('This calibrated model is specific to Gazebo X500; use --mpc-model nominal for SIH')
    if not all(c.isalnum() or c in '-_' for c in args.label):
        parser.error('label must contain only letters, numbers, hyphens or underscores')
    (ROOT / 'results').mkdir(exist_ok=True)
    instance_lock = (ROOT / 'results/px4-instance4.lock').open('w')
    try:
        fcntl.flock(instance_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        raise SystemExit('Another project experiment is already using PX4 instance 4')
    # Validate dependencies and QP construction before starting the owned PX4.
    mpc = None
    if args.controller == 'mpc':
        from altitude_mpc import AltitudeMPC, MPCConfig
        config = MPCConfig()
        calibration = None
        if args.mpc_model == 'calibrated':
            calibration_path = ROOT / 'config/calibrated-altitude-model.json'
            calibration = json.loads(calibration_path.read_text())
            config = MPCConfig(acceleration_gain=calibration['acceleration_gain'],
                               acceleration_bias=calibration['acceleration_bias'])
        if args.motion_profile=='gentle':
            from dataclasses import replace
            config=replace(config,max_velocity=1.,min_acceleration=-.8,max_acceleration=.8,max_jerk=2.)
        mpc = AltitudeMPC(config)
    source = ROOT / 'vendor/PX4-Autopilot'
    build = source / 'build/px4_sitl_default'
    binary = build / 'bin/px4'
    if not binary.is_file():
        raise SystemExit('Build PX4 first: make px4_sitl_default')
    stamp = dt.datetime.now().strftime('%Y%m%d-%H%M%S')
    out = ROOT / 'results' / f'{stamp}-{args.label}'
    runtime = out / 'runtime'
    runtime.mkdir(parents=True)
    model = 'gz_x500' if args.simulator == 'gazebo' else 'sihsim_quadx'
    metadata = {'px4_commit': subprocess.check_output(['git', '-C', str(source),
                'rev-parse', 'HEAD'], text=True).strip(), 'model': model,
                'simulator': args.simulator, 'controller': args.controller,
                'instance': INSTANCE, 'label': args.label,
                'simulation': True, 'status': 'started',
                'mpc_model': args.mpc_model if args.controller == 'mpc' else None,
                'ground_init':args.ground_init,'motion_profile':args.motion_profile,
                'bias_compensation':args.bias_compensation,
                'bias_compensator_config':{'gain_per_s':.15,'max_reference_correction_m':.12,'error_gate_m':.15,'velocity_gate_m_s':.15} if args.bias_compensation=='integral' else None,
                'timing_mode': args.timing_mode,
                'timing_version': 2 if args.timing_mode == 'scheduled' else 0,
                'control_clock_early_tolerance_sim_ms': 4 if args.timing_mode == 'scheduled' else None,
                'local_position_stream_hz': 50 if args.controller == 'mpc' and args.timing_mode == 'scheduled' else 20,
                'initial_height_m': 2, 'final_height_m': 4, 'observation_sim_s': 30}
    if args.controller == 'mpc' and calibration is not None:
        (out / 'calibration-model.json').write_text(json.dumps(calibration, indent=2)+'\n')
    (out / 'metadata.json').write_text(json.dumps(metadata, indent=2))
    env = dict(os.environ, PX4_SIM_MODEL=model,
               PX4_SIMULATOR='gz' if args.simulator == 'gazebo' else 'sihsim',
               PX4_PARAM_COM_RC_IN_MODE='4', PX4_PARAM_SDLOG_MODE='2')
    if args.simulator == 'gazebo':
        env.update(PX4_GZ_STANDALONE='1', PX4_GZ_WORLD='default')
    log = (out / 'px4-console.log').open('w')
    process = subprocess.Popen([str(binary), '-d', '-i', str(INSTANCE),
                                '-w', str(runtime), str(build / 'etc')],
                               env=env, stdout=log, stderr=subprocess.STDOUT,
                               start_new_session=True)
    link = mavutil.mavlink_connection(f'udpin:127.0.0.1:{14540 + INSTANCE}',
                                     source_system=250, source_component=190)
    experiment = Experiment(link, process, out, args.controller, mpc, args.timing_mode, args.ground_init, args.bias_compensation)
    print(f'OUTPUT={out}', flush=True)
    try:
        experiment.run()
        metadata['status'] = 'completed'
    except BaseException as exc:
        metadata['status'] = 'failed'
        metadata['error'] = str(exc)
        experiment.event('error', message=str(exc))
        if process.poll() is None and 'HEARTBEAT' in experiment.messages:
            try:
                experiment.land()
            except Exception:
                pass
        raise
    finally:
        experiment.csv_file.close()
        experiment.text_file.close()
        for file in (experiment.controller_file, experiment.prediction_file):
            if file is not None:
                file.close()
        if experiment.mpc is not None:
            metadata['mpc'] = experiment.mpc.metadata()
            metadata['mpc'].update(solve_count=experiment.mpc_records,
                max_solve_ms=experiment.mpc_max_solve_ms,
                max_update_interval_sim_s=experiment.mpc_max_interval_s,
                activation='after PX4-controlled takeoff and stable 2 m hover')
        link.close()
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=12)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5)
        log.close()
        metadata['ulog_files'] = [str(p.relative_to(out)) for p in out.rglob('*.ulg')]
        metadata['ground_reference_ned_z_m'] = experiment.ground_z
        (out / 'metadata.json').write_text(json.dumps(metadata, indent=2))
        print(f'RESULT={metadata["status"]} {out}', flush=True)


if __name__ == '__main__':
    main()
