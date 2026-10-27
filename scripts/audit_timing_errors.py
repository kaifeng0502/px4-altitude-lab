#!/usr/bin/env python3
"""Separate tracking/estimation errors and audit decision timing on saved runs.

Truth is exclusively a post-flight diagnostic. Historical files cannot reveal
network latency or the exact remote age of a MAVLink state at reception.
"""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def audit(run):
    meta=json.loads((run/'metadata.json').read_text())
    if meta['status']!='completed':
        raise ValueError(f'Run did not complete: {run}')
    metrics=json.loads((run/'metrics.json').read_text())
    df=pd.read_csv(run/'response.csv')
    df=df[df.time_from_step_s>=0].copy()
    df['tracking_error_m']=df.height_target_m-df.height_m
    df['estimation_error_m']=df.height_m-df.true_height_m
    df['physical_error_m']=df.height_target_m-df.true_height_m
    tail=df[df.time_from_step_s>=df.time_from_step_s.max()-2]
    ctrl=pd.read_csv(run/'controller.csv')
    ctrl=ctrl[(ctrl.boot_s>=metrics['step_boot_s'])&(ctrl.boot_s<=metrics['step_boot_s']+30)]
    dt=ctrl.update_interval_sim_s.to_numpy()
    dt=dt[dt>0]
    pos=pd.read_csv(run/'vehicle_local_position.csv')
    pos=pos[(pos.boot_s>=metrics['step_boot_s'])&(pos.boot_s<=metrics['step_boot_s']+30)]
    late=pos[pos.boot_s>=pos.boot_s.max()-5]
    height_slope=-float(np.polyfit(late.boot_s,late.z,1)[0])
    age=(pos.timestamp-pos.timestamp_sample)/1000
    summary={'run':run.name,'controller':meta.get('controller'), 'model':meta.get('mpc_model',meta['mpc']['type']),
        'timing_mode':meta.get('timing_mode','legacy'), 'timing_version':meta.get('timing_version',1 if meta.get('timing_mode')=='scheduled' else 0), 'decision_count':len(ctrl),
        'interval_mean_ms':float(dt.mean()*1000),'interval_p95_ms':float(np.quantile(dt,.95)*1000),
        'interval_max_ms':float(dt.max()*1000),'interval_over_120ms_count':int((dt>.120001).sum()),
        'solve_p95_ms':float(ctrl.solve_ms.quantile(.95)),'solve_max_ms':float(ctrl.solve_ms.max()),
        'ulog_state_publication_minus_sample_median_ms':float(age.median()),
        'last_2s_mean_tracking_error_m':float(tail.tracking_error_m.mean()),
        'last_2s_mean_estimation_error_m':float(tail.estimation_error_m.mean()),
        'last_2s_mean_physical_error_m':float(tail.physical_error_m.mean()),
        'error_identity_max_residual_m':float(np.abs(df.physical_error_m-df.tracking_error_m-df.estimation_error_m).max()),
        'last_5s_mean_ekf_up_velocity_m_s':float(-late.vz.mean()),
        'last_5s_ekf_height_slope_m_s':height_slope,
        'metrics':metrics}
    for name in ['deadline_phase_error_sim_ms','deadline_lateness_sim_ms','state_receive_age_wall_ms','command_send_spacing_wall_ms']:
        if name in ctrl and ctrl[name].notna().any():
            summary[name]={'median':float(ctrl[name].median()),'p95':float(ctrl[name].quantile(.95)),'max':float(ctrl[name].max())}
    if all(k in ctrl and ctrl[k].notna().any() for k in ['state_receive_age_wall_ms','command_send_wall_s','solve_start_wall_s']):
        local_send_age=ctrl.state_receive_age_wall_ms+(ctrl.command_send_wall_s-ctrl.solve_start_wall_s)*1000
        summary['state_local_age_at_send_wall_ms']={'median':float(local_send_age.median()),
            'p95':float(local_send_age.quantile(.95)),'max':float(local_send_age.max())}
    reset_changes=pos.z_reset_counter.diff().fillna(0)!=0
    summary['height_frame_resets_during_step']=int(reset_changes.sum())
    if 'skipped_control_ticks' in ctrl and ctrl.skipped_control_ticks.notna().any():
        summary['skipped_control_ticks']=int(ctrl.skipped_control_ticks.sum())
    telemetry=pd.read_csv(run/'telemetry.csv')
    telemetry=telemetry[(telemetry.boot_s>=metrics['step_boot_s'])&(telemetry.boot_s<=metrics['step_boot_s']+30)]
    feedback_dt=np.diff(telemetry.boot_s)*1000
    feedback_dt=feedback_dt[feedback_dt>0]
    summary['feedback_sample_interval_sim_ms']={'median':float(np.median(feedback_dt)),
        'p95':float(np.quantile(feedback_dt,.95)),'max':float(feedback_dt.max())}
    return summary,df,ctrl


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('runs',type=Path,nargs='+')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    values=[audit(run) for run in args.runs]
    report={'definitions':{'tracking':'target - EKF','estimation':'EKF - truth','physical':'target - truth = tracking + estimation'},
        'truth_usage':'offline diagnostics only; each height uses its fixed preflight ground datum',
        'timing_limitations':'Historical boot_s intervals are simulation-time decision gaps, not measured network latency. ULog timestamp-sample is estimator publication age, not MAVLink transport age. New receive-age fields are local wall age since receipt only.',
        'runs':[x[0] for x in values]}
    (args.output/'audit.json').write_text(json.dumps(report,indent=2)+'\n')
    plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False})
    n=len(values);cols=2 if n>1 else 1;rows=(n+cols-1)//cols
    fig,axes=plt.subplots(rows,cols,figsize=(12,3.2*rows),squeeze=False,layout='constrained')
    for ax,(summary,df,ctrl) in zip(axes.flat,values):
        shown=df[df.time_from_step_s>=5]
        t=shown.time_from_step_s
        ax.plot(t,shown.tracking_error_m,color='#1769aa',label='Target − EKF')
        ax.plot(t,shown.estimation_error_m,color='#e88423',label='EKF − truth')
        ax.plot(t,shown.physical_error_m,'--',color='#111827',label='Target − truth')
        ax.axhspan(-.05,.05,color='#64748b',alpha=.1);ax.axhline(0,color='#64748b',lw=.7)
        ax.set_title(summary['run'].replace('20261027-',''),fontsize=10,loc='left')
        ax.set_xlim(5,30);ax.set_xlabel('Time from step (s)');ax.set_ylabel('Signed error (m)')
        ax.grid(alpha=.2);ax.legend(fontsize=8,frameon=False)
        df.to_csv(args.output/(summary['run']+'-errors.csv'),index=False)
    bound=max(.06,max(float(df.loc[df.time_from_step_s>=5,['tracking_error_m','estimation_error_m','physical_error_m']].abs().max().max()) for _,df,_ in values)*1.1)
    for ax in list(axes.flat)[:n]:ax.set_ylim(-bound,bound)
    for ax in list(axes.flat)[n:]:ax.set_visible(False)
    fig.suptitle('Late response: distinguish control tracking from state-estimation error\nPositive total error means the vehicle is below its target')
    fig.savefig(args.output/'error-decomposition.png',dpi=170)
    fig.savefig(args.output/'error-decomposition.pdf')
    fig,axes=plt.subplots(2,1,figsize=(11,7),layout='constrained')
    for summary,df,ctrl in values:
        t=ctrl.boot_s-summary['metrics']['step_boot_s']
        label=summary['run'].replace('20261027-','')
        axes[0].plot(t,ctrl.update_interval_sim_s*1000,lw=.8,alpha=.7,label=label)
        axes[1].plot(t,ctrl.solve_ms,lw=.8,alpha=.7,label=label)
    axes[0].axhline(100,color='#111827',ls=':',label='Model step: 100 ms')
    axes[0].set_ylabel('Decision interval (sim ms)');axes[1].set_ylabel('QP duration (wall ms)')
    for ax in axes:ax.grid(alpha=.2);ax.set_xlim(0,30);ax.legend(fontsize=7,frameon=False)
    axes[1].set_xlabel('Simulation time from reference step (s)')
    fig.suptitle('Decision timing and solve time use different clocks')
    fig.savefig(args.output/'timing-audit.png',dpi=170)
    fig.savefig(args.output/'timing-audit.pdf')
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
