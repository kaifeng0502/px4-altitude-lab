#!/usr/bin/env python3
"""Analyze the internal cascade signals from the actual PX4 ULog."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from pyulog import ULog


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('run', type=Path)
    args = parser.parse_args()
    run = args.run.resolve()
    meta = json.loads((run / 'metadata.json').read_text())
    if meta['status'] != 'completed':
        raise SystemExit('This run did not complete; do not report it as a successful experiment.')
    logs = sorted(run.rglob('*.ulg'))
    if len(logs) != 1:
        raise SystemExit(f'Expected one ULog, found {len(logs)}')
    ulog = ULog(str(logs[0]))

    def frame(topic):
        dataset = ulog.get_dataset(topic)
        df = pd.DataFrame(dataset.data)
        df['boot_s'] = df.timestamp / 1e6
        df.to_csv(run / f'{topic}.csv', index=False)
        return df

    pos = frame('vehicle_local_position')
    truth = frame('vehicle_local_position_groundtruth')
    sp = frame('vehicle_local_position_setpoint')
    thrust = frame('vehicle_thrust_setpoint')
    status = frame('vehicle_status')
    land = frame('vehicle_land_detected')
    is_mpc = meta.get('controller', 'pid') == 'mpc'
    controller = pd.read_csv(run / 'controller.csv') if is_mpc else None
    events = json.loads((run / 'events.json').read_text())
    datum = next(e for e in events if e['event'] == 'ground_reference')
    ground_z = float(datum['ned_z_m'])
    truth_ground = truth[(truth.boot_s >= datum['boot_s'] - 3) & (truth.boot_s <= datum['boot_s'])]
    truth_ground_z = float(truth_ground.z.median())
    command_step = next(e['boot_s'] for e in events if e['event'] == 'height_step')
    landing = next(e['boot_s'] for e in events if e['event'] == 'landing')
    if is_mpc:
        candidates = controller[controller.target_height_m == meta['final_height_m']]
        if candidates.empty:
            raise RuntimeError('No MPC decision for the 4 m reference')
        step = float(candidates.boot_s.iloc[0])
        trajectory = frame('trajectory_setpoint')
        measured = trajectory[(trajectory.boot_s >= step + .3) & (trajectory.boot_s < landing - .3)]
        internal = sp[(sp.boot_s >= step + .3) & (sp.boot_s < landing - .3)]
        if len(measured) < 20 or len(internal) < 20:
            raise RuntimeError('Not enough internal samples to verify acceleration-only control')
        for df, z, vz, az in [(measured, 'position[2]', 'velocity[2]', 'acceleration[2]'),
                              (internal, 'z', 'vz', 'acceleration[2]')]:
            if not (df[z].isna().all() and df[vz].isna().all() and np.isfinite(df[az]).all()):
                raise RuntimeError('MPC was not exclusively using vertical acceleration: hidden P/PI feedback possible')
    else:
        candidates = sp[(sp.boot_s >= command_step - .2) & (sp.z < ground_z - 3.9)]
        if candidates.empty:
            raise RuntimeError('No actual internal 4 m setpoint found in ULog')
        step = float(candidates.boot_s.iloc[0])
    controlled = status[(status.boot_s >= step) & (status.boot_s < landing - .2)]
    if controlled.empty or not (controlled.nav_state == 14).all():
        raise RuntimeError('PX4 was not continuously in Offboard during the measured step')
    if not (controlled.arming_state == 2).all():
        raise RuntimeError('PX4 was not continuously armed during the measured step')
    if not bool(land.landed.iloc[-1]) or int(status.arming_state.iloc[-1]) != 1:
        raise RuntimeError('ULog does not confirm final landing and disarming')
    for df in [pos, truth, sp, thrust, status, land] + ([controller] if is_mpc else []):
        df['t'] = df.boot_s - step
    pos['height'] = ground_z - pos.z
    sp['height'] = ground_z - sp.z
    truth['height'] = truth_ground_z - truth.z
    before = pos[(pos.t >= -2) & (pos.t < 0)]
    after = pos[(pos.t >= 0) & (pos.boot_s < landing - .1)]
    if len(before) < 10 or len(after) < 100:
        raise RuntimeError('Insufficient hover/step samples')
    baseline = float(before.height.mean())
    command_initial = float(meta['initial_height_m'])
    amplitude = float(meta['final_height_m']) - command_initial

    def crossing(level):
        reached = after[after.height >= level]
        return float(reached.t.iloc[0]) if len(reached) else None

    t10, t90 = crossing(command_initial + .1 * amplitude), crossing(command_initial + .9 * amplitude)
    error = np.abs(after.height.to_numpy() - 4)
    outside = np.flatnonzero(error > .05)
    settling_index = int(outside[-1]) + 1 if len(outside) else 0
    settling = (float(after.t.iloc[settling_index])
                if settling_index < len(after) else None)
    # Require at least 2 s remaining in band; do not label a final isolated sample settled.
    if settling is not None and after.t.iloc[-1] - settling < 2:
        settling = None
    tail = after[after.t >= after.t.iloc[-1] - 2]
    truth_tail = truth[(truth.boot_s >= tail.boot_s.iloc[0]) & (truth.boot_s < landing - .1)]
    metrics = {
        'step_boot_s': step, 'baseline_height_m': baseline, 'target_height_m': 4.0,
        'commanded_step_amplitude_m': amplitude,
        'rise_time_definition': 'first upward crossings of 2.2 m and 3.8 m in estimated height',
        'rise_time_10_90_s': t90 - t10 if t10 is not None and t90 is not None else None,
        'overshoot_m': max(0.0, float(after.height.max() - 4)),
        'overshoot_percent_of_step': max(0.0, float(after.height.max() - 4)) / amplitude * 100,
        'settling_time_within_5cm_s': settling,
        'final_mean_height_m': float(tail.height.mean()),
        'final_mean_error_m': float(4 - tail.height.mean()),
        'final_mean_true_height_m': float(truth_tail.height.mean()),
        'final_mean_estimate_minus_truth_m': float(tail.height.mean() - truth_tail.height.mean()),
        'ground_reference_ned_z_m': ground_z,
        'truth_ground_reference_ned_z_m': truth_ground_z,
        'max_abs_estimate_minus_truth_m': float(np.max(np.abs(after.height.to_numpy()-np.interp(after.boot_s, truth.boot_s, truth.height)))),
        'peak_up_velocity_m_s': float((-after.vz).max()),
        'observation_window_s': float(after.t.iloc[-1]),
        'ulog_dropout_count': len(ulog.dropouts),
        'ulog_dropout_total_ms': sum(d.duration for d in ulog.dropouts),
        'landed_event_confirmed': any(e['event'] == 'landed_disarmed' for e in events),
    }
    metrics['controller'] = meta.get('controller', 'pid')
    metrics['timing_mode'] = meta.get('timing_mode', 'legacy')
    metrics['ground_init'] = meta.get('ground_init','legacy')
    metrics['motion_profile'] = meta.get('motion_profile','standard')
    metrics['bias_compensation'] = meta.get('bias_compensation','off')
    metrics['final_mean_physical_error_m'] = float(4-truth_tail.height.mean())
    if is_mpc:
        decisions = controller[(controller.t >= 0) & (controller.boot_s < landing)]
        metrics['mpc_acceleration_only_verified_in_ulog'] = True
        metrics['mpc_max_solve_ms'] = float(decisions.solve_ms.max())
        metrics['mpc_p95_solve_ms'] = float(decisions.solve_ms.quantile(.95))
        metrics['mpc_max_update_interval_sim_s'] = float(decisions.update_interval_sim_s.max())
        metrics['mpc_max_predicted_speed_slack_m_s'] = float(decisions.max_predicted_speed_slack_m_s.max()) if 'max_predicted_speed_slack_m_s' in decisions else 0.
        metrics['mpc_max_constraint_violation'] = float(decisions.constraint_violation.max())
        metrics['mpc_max_abs_commanded_acceleration_m_s2'] = float(decisions.up_acceleration_command_m_s2.abs().max())
        metrics['mpc_measured_speed_limit_excess_m_s'] = max(0., float(after.vz.abs().max())-meta['mpc']['max_velocity'])
        metrics['mpc_limits_apply_to'] = 'predictions and commands, not guaranteed actual vehicle motion'
    (run / 'metrics.json').write_text(json.dumps(metrics, indent=2))
    params = {k: v for k, v in ulog.initial_parameters.items()
              if k.startswith(('MPC_', 'MC_', 'SIH_', 'COM_RC', 'SDLOG_'))}
    (run / 'parameters.json').write_text(json.dumps(params, indent=2))
    # Unified 20 Hz table for convenient inspection; raw per-topic CSVs are also retained.
    window = float(meta.get('observation_sim_s', 30))
    t = np.arange(-4, min(window, float(after.t.iloc[-1])), .05)
    combined = pd.DataFrame({'time_from_step_s': t})
    for name, df, field, sign in [
        ('height_m', pos, 'height', 1), ('height_target_m', sp, 'height', 1),
        ('true_height_m', truth, 'height', 1),
        ('up_velocity_m_s', pos, 'vz', -1), ('up_velocity_target_m_s', sp, 'vz', -1),
        ('up_acceleration_m_s2', pos, 'az', -1),
        ('up_acceleration_target_m_s2', sp, 'acceleration[2]', -1),
        ('up_thrust_normalized_ned', sp, 'thrust[2]', -1),
        ('thrust_normalized_body_z_negative', thrust, 'xyz[2]', -1)]:
        valid = np.isfinite(df[field])
        combined[name] = (np.interp(t, df.t[valid], sign * df[field][valid])
                          if valid.any() else np.full(len(t), np.nan))
    if is_mpc:
        # Height is the external optimizer's reference; PX4 intentionally has no
        # vertical position/velocity setpoint. Keep these missing signals NaN.
        index = np.maximum(0, np.searchsorted(controller.t.to_numpy(), t, side='right')-1)
        combined['height_target_m'] = controller.target_height_m.to_numpy()[index]
        combined['mpc_acceleration_command_m_s2'] = controller.up_acceleration_command_m_s2.to_numpy()[index]
        if 'mpc_effective_reference_m' in controller and controller.mpc_effective_reference_m.notna().any():
            combined['mpc_effective_reference_m'] = controller.mpc_effective_reference_m.to_numpy()[index]
            combined['height_reference_correction_m'] = controller.height_reference_correction_m.to_numpy()[index]
    combined['height_tracking_error_m'] = combined.height_target_m-combined.height_m
    combined['height_estimation_error_m'] = combined.height_m-combined.true_height_m
    combined['height_physical_error_m'] = combined.height_target_m-combined.true_height_m
    combined.to_csv(run / 'response.csv', index=False)

    plt.rcParams.update({'font.size': 11, 'axes.spines.top': False,
                         'axes.spines.right': False, 'figure.facecolor': '#f8fafc',
                         'axes.facecolor': '#ffffff'})
    fig, axes = plt.subplots(4, 1, figsize=(10, 12), sharex=True, layout='constrained')
    blue, orange = '#1769aa', '#e88423'
    axes[0].plot(t, combined.height_target_m, '--', color=orange, label='Target')
    if 'mpc_effective_reference_m' in combined and meta.get('bias_compensation')=='integral':
        axes[0].plot(t, combined.mpc_effective_reference_m, ':', color='#7c3aed', label='MPC reference after bounded integral')
    axes[0].plot(t, combined.height_m, color=blue, label='Estimated height')
    simulator = 'Gazebo X500' if meta['model'] == 'gz_x500' else 'SIH Quadrotor X'
    axes[0].plot(t, combined.true_height_m, color='#00897b', alpha=.85, label='Simulation ground truth')
    axes[0].axhspan(3.95, 4.05, color=blue, alpha=.08)
    axes[0].set_ylabel('Height above initial ground (m)')
    axes[0].set_title('1  MPC height tracking: 2 m to 4 m' if is_mpc else '1  Position loop: 2 m to 4 m', loc='left', fontweight='bold')
    if not is_mpc:
        axes[1].plot(t, combined.up_velocity_target_m_s, '--', color=orange, label='Internal target')
    else:
        axes[1].axhline(meta['mpc']['max_velocity'], ls=':', color=orange, label='MPC soft speed limit')
        axes[1].axhline(-meta['mpc']['max_velocity'], ls=':', color=orange)
    axes[1].plot(t, combined.up_velocity_m_s, color=blue, label='Estimated velocity')
    axes[1].set_ylabel('Upward velocity (m/s)')
    axes[1].set_title('2  Measured velocity (vertical PI bypassed)' if is_mpc else '2  Velocity loop', loc='left', fontweight='bold')
    axes[2].plot(t, combined.up_acceleration_target_m_s2, '--', color=orange, label='Internal target')
    if is_mpc:
        axes[2].step(t, combined.mpc_acceleration_command_m_s2, where='post', color='#00897b', ls=':', label='External MPC command')
    axes[2].plot(t, combined.up_acceleration_m_s2, color=blue, alpha=.8, label='Estimated acceleration')
    axes[2].set_ylabel('Upward acceleration (m/s²)')
    axes[2].set_title('3  Acceleration demand', loc='left', fontweight='bold')
    axes[3].plot(t, combined.up_thrust_normalized_ned, color=blue, label='Upward thrust command (NED)')
    hover_thrust = float(combined.loc[t < 0, 'up_thrust_normalized_ned'].mean())
    axes[3].axhline(hover_thrust, ls=':', color='#64748b', label='Pre-step hover mean')
    axes[3].set_ylabel('Normalized thrust')
    axes[3].set_title('4  Thrust demand (unitless, not newtons)', loc='left', fontweight='bold')
    for ax in axes:
        ax.axvline(0, color='#64748b', lw=1, alpha=.6)
        ax.grid(alpha=.18)
        ax.legend(loc='best', frameon=False, fontsize=9)
        ax.set_xlim(-4, window)
    axes[-1].set_xlabel('Time from first MPC 4 m decision (s)' if is_mpc else 'Time from internal height setpoint step (s)')
    control_label = 'MPC acceleration control' if is_mpc else 'PX4 P/PI baseline'
    fig.suptitle(f'PX4 altitude-step experiment · {control_label}\nv1.17.0 · {simulator} · upward-positive coordinates',
                 fontsize=16, fontweight='bold')
    fig.savefig(run / 'altitude-response.png', dpi=180)
    fig.savefig(run / 'altitude-response.pdf')
    print(json.dumps(metrics, indent=2))
    print(f'CHART={run / "altitude-response.png"}')


if __name__ == '__main__':
    main()
