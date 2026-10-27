#!/usr/bin/env python3
"""Compare completed PX4 P/PI and 1-D MPC runs on the same simulator/version."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('baseline', type=Path)
    parser.add_argument('mpc', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    runs = [args.baseline.resolve(), args.mpc.resolve()]
    meta = [json.loads((p/'metadata.json').read_text()) for p in runs]
    metrics = [json.loads((p/'metrics.json').read_text()) for p in runs]
    if [m.get('controller', 'pid') for m in meta] != ['pid', 'mpc']:
        raise SystemExit('Expected baseline pid run followed by mpc run')
    if any(m['status'] != 'completed' for m in meta):
        raise SystemExit('Both runs must be completed')
    if meta[0].get('ground_init','legacy')!=meta[1].get('ground_init','legacy'):
        raise SystemExit('Ground initialization differs between controller trials')
    for key in ['model', 'simulator', 'px4_commit', 'initial_height_m', 'final_height_m', 'observation_sim_s']:
        if meta[0][key] != meta[1][key]:
            raise SystemExit(f'Incomparable runs: {key} differs')
    parameters = [json.loads((p/'parameters.json').read_text()) for p in runs]
    if parameters[0] != parameters[1]:
        differing = sorted(k for k in parameters[0].keys() | parameters[1].keys()
                           if parameters[0].get(k) != parameters[1].get(k))
        raise SystemExit(f'PX4 parameters differ: {differing}')
    if any(m.get('max_abs_estimate_minus_truth_m', float('inf')) > .5 for m in metrics):
        raise SystemExit('Estimator/ground-truth mismatch exceeds 0.5 m; investigate before comparing')
    if not metrics[1].get('mpc_acceleration_only_verified_in_ulog'):
        raise SystemExit('MPC acceleration-only interface has not been verified')
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=True)
    data = [pd.read_csv(p/'response.csv') for p in runs]
    labels = ['PX4 P/PI baseline', '1-D linear MPC']
    colors = ['#1769aa', '#e88423']
    plt.rcParams.update({'font.size': 10, 'axes.spines.top': False, 'axes.spines.right': False})
    fig, axes = plt.subplots(4, 1, figsize=(10, 11), sharex=True, layout='constrained')
    fields = [('height_m', 'Estimated height (m)'), ('up_velocity_m_s', 'Upward velocity (m/s)'),
              ('up_acceleration_target_m_s2', 'Acceleration demand (m/s²)'),
              ('up_thrust_normalized_ned', 'Normalized thrust')]
    for df, label, color in zip(data, labels, colors):
        for ax, (field, ylabel) in zip(axes, fields):
            ax.plot(df.time_from_step_s, df[field], color=color, label=label)
            ax.set_ylabel(ylabel)
    axes[0].plot([-4, 0, 0, 30], [2, 2, 4, 4], '--', color='#475569', label='Height reference')
    axes[0].axhspan(3.95, 4.05, color='#64748b', alpha=.12)
    axes[1].axhline(meta[1]['mpc']['max_velocity'], ls=':', color=colors[1], label='MPC soft speed limit')
    axes[1].axhline(-meta[1]['mpc']['max_velocity'], ls=':', color=colors[1])
    for ax in axes:
        ax.axvline(0, color='#64748b', lw=1, alpha=.5)
        ax.grid(alpha=.2)
        ax.legend(fontsize=9, frameon=False)
        ax.set_xlim(-4, 30)
    axes[-1].set_xlabel('Simulation time from each controller reference step (s)')
    fig.suptitle('2 m → 4 m: PX4 cascade versus constrained MPC\nSame PX4/model and exported control parameters; different constraints', fontsize=14)
    fig.savefig(out/'pid-vs-mpc.png', dpi=170)
    fig.savefig(out/'pid-vs-mpc.pdf')

    # Show what the optimizer predicts at several real decisions, rather than
    # inventing a simulated forecast disconnected from the executed flight.
    step = metrics[1]['step_boot_s']
    predictions = [json.loads(line) for line in (runs[1]/'mpc-predictions.jsonl').read_text().splitlines()]
    predictions = [p for p in predictions if p['target_height_m'] == 4.]
    fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True, layout='constrained')
    df = data[1]
    axes[0].plot(df.time_from_step_s, df.height_m, color='#111827', lw=2, label='EKF estimated height')
    axes[1].plot(df.time_from_step_s, df.up_velocity_m_s, color='#111827', lw=2, label='EKF estimated velocity')
    for offset in [0., .5, 1., 2.]:
        p = min(predictions, key=lambda p: abs(p['boot_s']-step-offset))
        states = np.array(p['predicted_states'])
        future_t = p['boot_s']-step + np.arange(len(states))*meta[1]['mpc']['dt']
        for ax, column in zip(axes, [0, 1]):
            ax.plot(future_t, states[:, column], '--', label=f"Prediction at {p['boot_s']-step:.2f} s")
    axes[0].axhline(4., color='#64748b', ls=':')
    axes[0].set_ylabel('Height (m)')
    axes[1].set_ylabel('Upward velocity (m/s)')
    axes[-1].set_xlabel('Simulation time from MPC reference step (s)')
    for ax in axes:
        ax.grid(alpha=.2)
        ax.legend(fontsize=8, frameon=False)
        ax.set_xlim(-.3, 7)
    fig.suptitle('Receding horizon: actual optimizer forecasts versus EKF feedback')
    fig.savefig(out/'mpc-predictions.png', dpi=170)
    fig.savefig(out/'mpc-predictions.pdf')
    summary = {'baseline_run': str(runs[0]), 'mpc_run': str(runs[1]),
               'same_exported_control_parameters_verified': True, 'baseline': metrics[0], 'mpc': metrics[1],
               'comparison_scope': 'one trial per controller, different control laws and constraints; not a general superiority claim'}
    (out/'comparison.json').write_text(json.dumps(summary, indent=2)+'\n')
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
