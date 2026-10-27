#!/usr/bin/env python3
"""Fit a causal static acceleration map on one run; validate on a different run.

Uses ULog-exported internal acceleration demand and EKF acceleration, not
simulation ground truth. The fit is a local closed-loop least-squares model,
not an unbiased physical identification or proof of delay absence.
"""
import argparse
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def load(run):
    p = pd.read_csv(run/'vehicle_local_position.csv')
    u = pd.read_csv(run/'vehicle_local_position_setpoint.csv')
    events = json.loads((run/'events.json').read_text())
    step = next(e['boot_s'] for e in events if e['event'] == 'height_step')
    return p, u, step


def demand(u, t):
    index = np.clip(np.searchsorted(u.boot_s, t, side='right')-1, 0, len(u)-1)
    return -u['acceleration[2]'].to_numpy()[index]


def rmse(y, p):
    return float(np.sqrt(np.mean((y-p)**2)))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('training', type=Path)
    parser.add_argument('validation', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--model-output', type=Path, required=True)
    args = parser.parse_args()
    if args.training.resolve() == args.validation.resolve():
        raise SystemExit('Training and validation runs must differ')
    metadata=[json.loads((run/'metadata.json').read_text()) for run in [args.training,args.validation]]
    if any(m['status']!='completed' for m in metadata):
        raise SystemExit('Use completed training and validation flights')
    for key in ['model','simulator','px4_commit']:
        if metadata[0][key]!=metadata[1][key]:
            raise SystemExit(f'Training and validation plants differ: {key}')
    p, u, step = load(args.training)
    t = np.arange(step, step+6., .02)
    a = np.interp(t, p.boot_s, -p.az)
    command = demand(u, t)
    gain, bias = np.linalg.lstsq(np.c_[command, np.ones(len(t))], a, rcond=None)[0]
    if not .5 < gain < 3. or abs(bias) > .5:
        raise SystemExit('Fit outside expected local-model range; inspect data')
    diagnostics = []
    for delay in np.arange(0., .201, .01):
        delayed = demand(u, t-delay)
        k, b = np.linalg.lstsq(np.c_[delayed, np.ones(len(t))], a, rcond=None)[0]
        diagnostics.append({'delay_s': float(delay), 'gain': float(k), 'bias': float(b),
                            'training_acceleration_rmse': rmse(a, k*delayed+b)})
    vp, vu, vs = load(args.validation)
    vt = np.arange(vs+.3, vs+6., .02)
    va = np.interp(vt, vp.boot_s, -vp.az)
    vc = demand(vu, vt)
    # Exclude demands outside the calibration range and near thrust saturation.
    # Use the SAME mask for both models, fixed before scoring.
    thrust = np.interp(vt, vu.boot_s, -vu['thrust[2]'])
    mask = (np.abs(vc) < 2.) & (thrust < .98) & ((np.abs(vc) > .2) | (np.abs(va) > .3))
    if mask.sum() < 30:
        raise SystemExit('Insufficient dynamic holdout samples')
    def velocity_errors(pos, inp, start, stop):
        # Independent 0.2 s windows, integrate the logged command with 0.004 s
        # substeps and compare delta velocity; each window starts at its own
        # measured state. This is NOT a full-flight open-loop rollout.
        errors = [[], []]
        for begin in np.arange(start, stop-.2, .2):
            ts = begin + np.arange(50)*.004
            cmd = demand(inp, ts)
            if np.abs(cmd).max() > 2.:
                continue
            actual = float(np.interp(begin+.2, pos.boot_s, -pos.vz)-np.interp(begin, pos.boot_s, -pos.vz))
            for values, predicted in zip(errors, [cmd, gain*cmd+bias]):
                values.append(float(predicted.sum()*.004-actual))
        return {'window_s': .2, 'count': len(errors[0]),
                'nominal_delta_velocity_rmse_m_s': rmse(np.array(errors[0]), 0),
                'calibrated_delta_velocity_rmse_m_s': rmse(np.array(errors[1]), 0)}
    report = {'training_run': args.training.name, 'validation_run': args.validation.name,
              'training_window_from_step_s': [0, 6], 'training_sample_count': len(t),
              'model_order': 2, 'model': metadata[0]['model'], 'simulator': metadata[0]['simulator'],
              'px4_commit': metadata[0]['px4_commit'], 'acceleration_gain': float(gain), 'acceleration_bias': float(bias),
              'equation': 'h_dot=v; v_dot=K*u+b',
              'training_nominal_acceleration_rmse': rmse(a, command),
              'training_calibrated_acceleration_rmse': rmse(a, gain*command+bias),
              'validation_dynamic_samples': int(mask.sum()),
              'validation_nominal_acceleration_rmse': rmse(va[mask], vc[mask]),
              'validation_calibrated_acceleration_rmse': rmse(va[mask], gain*vc[mask]+bias),
              'validation_velocity_windows': velocity_errors(vp, vu, vs+.3, vs+6.),
              'nonnegative_delay_scan': diagnostics,
              'limitations': 'Local closed-loop EKF fit; no instrumental-variable correction; measurement alignment may mask physical delay; no truth in feedback',
              'data_sha256': {f'{run.name}/{name}': hashlib.sha256((run/name).read_bytes()).hexdigest()
                              for run in [args.training, args.validation]
                              for name in ['vehicle_local_position.csv', 'vehicle_local_position_setpoint.csv']}}
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output/'identification.json').write_text(json.dumps(report, indent=2)+'\n')
    args.model_output.parent.mkdir(parents=True, exist_ok=True)
    args.model_output.write_text(json.dumps({'acceleration_gain': float(gain), 'acceleration_bias': float(bias),
        'source': report}, indent=2)+'\n')
    fig, axes = plt.subplots(2, 1, figsize=(10, 7), layout='constrained')
    for ax, ts, y, cmd, title in [(axes[0], t-step, a, command, 'Training: original MPC flight'),
                                  (axes[1], vt[mask]-vs, va[mask], vc[mask], 'Holdout: separate P/PI flight, within calibration range')]:
        ax.plot(ts, y, color='#111827', label='EKF acceleration')
        ax.plot(ts, cmd, '--', color='#1769aa', label='Nominal prediction: a = u')
        ax.plot(ts, gain*cmd+bias, color='#e88423', label=f'Calibrated: a = {gain:.3f}u {bias:+.3f}')
        ax.set_title(title, loc='left');ax.set_ylabel('Upward acceleration (m/s²)')
        ax.set_xlabel('Time from height reference step (s)');ax.grid(alpha=.2);ax.legend(fontsize=9)
    fig.suptitle('Calibrating the two-state altitude model (no added dynamic state)')
    fig.savefig(args.output/'model-identification.png', dpi=170)
    fig.savefig(args.output/'model-identification.pdf')
    print(json.dumps({k:v for k,v in report.items() if k not in ['nonnegative_delay_scan','data_sha256']}, indent=2))


if __name__ == '__main__':
    main()
