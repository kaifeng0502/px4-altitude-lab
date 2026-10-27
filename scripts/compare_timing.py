#!/usr/bin/env python3
"""Compare timing changes on the same frozen calibrated MPC and PX4 plant."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from audit_timing_errors import audit


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--legacy',type=Path,nargs='+',required=True)
    parser.add_argument('--scheduled',type=Path,nargs='+',required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    runs=args.legacy+args.scheduled
    meta=[json.loads((run/'metadata.json').read_text()) for run in runs]
    reference=meta[0]
    keys=['dt','horizon','height_weight','velocity_weight','acceleration_weight',
          'delta_acceleration_weight','velocity_slack_weight','max_velocity',
          'min_acceleration','max_acceleration','max_jerk','acceleration_gain','acceleration_bias']
    params=[json.loads((run/'parameters.json').read_text()) for run in runs]
    values=[]
    for run,m,parameters in zip(runs,meta,params):
        if m['status']!='completed' or m.get('controller')!='mpc' or m.get('mpc_model')!='calibrated':
            raise SystemExit('Use completed calibrated MPC flights')
        expected='legacy' if run in args.legacy else 'scheduled'
        if m.get('timing_mode','legacy')!=expected:
            raise SystemExit('Run has the wrong timing mode')
        for key in ['model','simulator','px4_commit','initial_height_m','final_height_m','observation_sim_s']:
            if m[key]!=reference[key]:raise SystemExit(f'Plant/protocol mismatch: {key}')
        if (m.get('ground_init','legacy'),m.get('bias_compensation','off'))!=(reference.get('ground_init','legacy'),reference.get('bias_compensation','off')):
            raise SystemExit('Ground initialization or compensation changed; isolate timing first')
        if any(m['mpc'][key]!=reference['mpc'][key] for key in keys):
            raise SystemExit('MPC model or tuning changed')
        if parameters!=params[0]:raise SystemExit('Exported PX4 parameters changed')
        value=audit(run)
        if not value[0]['metrics'].get('mpc_acceleration_only_verified_in_ulog'):
            raise SystemExit('Vertical P/PI bypass not verified')
        values.append(value)
    args.output.mkdir(parents=True,exist_ok=True)
    report={'same_model_weights_constraints_and_exported_px4_parameters':True,
            'changed_factors':['receive-before-send ordering','phase-locked simulation-time QP deadlines',
                '50 Hz instead of 20 Hz position telemetry','immediate sending after a solve','post-send disk logging','explicit 4 ms early-sample tolerance in scheduled version 2'],
            'unchanged_factors':{k:reference['mpc'][k] for k in keys},
            'scope':'Small-sample software timing comparison; no guarantee of hard realtime or absolute height accuracy',
            'observer_decision':'Not added in this iteration: truth/estimator disagreement is separate from acceleration disturbance; measure and isolate before adding integral/observer states',
            'runs':[value[0] for value in values]}
    (args.output/'comparison.json').write_text(json.dumps(report,indent=2)+'\n')
    labels=[]
    counts={'legacy':0,'scheduled':0}
    for summary,df,ctrl in values:
        mode=summary['timing_mode'];counts[mode]+=1
        labels.append(f'{mode.capitalize()} v{summary["timing_version"]} · trial {counts[mode]}')
    colors=['#d97706']*len(args.legacy)+[['#1769aa','#00897b','#7c3aed'][i%3] for i in range(len(args.scheduled))]
    fig,axes=plt.subplots(2,1,figsize=(11,7),layout='constrained')
    for (summary,df,ctrl),label,color in zip(values,labels,colors):
        dt=np.sort(ctrl.update_interval_sim_s.to_numpy()*1000)
        dt=dt[dt>0]
        axes[0].step(dt,np.arange(1,len(dt)+1)/len(dt),where='post',label=label,color=color)
        axes[1].plot(ctrl.boot_s-summary['metrics']['step_boot_s'],ctrl.update_interval_sim_s*1000,
                     label=label,color=color,lw=.8,alpha=.8)
    axes[0].axvline(100,color='#64748b',ls=':');axes[1].axhline(100,color='#64748b',ls=':')
    axes[0].set_xlabel('Decision interval (simulation ms)');axes[0].set_ylabel('Cumulative fraction')
    axes[1].set_xlabel('Simulation time from MPC reference step (s)');axes[1].set_ylabel('Decision interval (sim ms)')
    for ax in axes:ax.grid(alpha=.2);ax.legend(frameon=False)
    fig.suptitle('Timing change with the same calibrated MPC model and tuning')
    fig.savefig(args.output/'timing-comparison.png',dpi=170);fig.savefig(args.output/'timing-comparison.pdf')
    fig,axes=plt.subplots(3,1,figsize=(11,9),sharex=True,layout='constrained')
    for (summary,df,ctrl),label,color in zip(values,labels,colors):
        shown=df[df.time_from_step_s>=5]
        for ax,column in zip(axes,['tracking_error_m','estimation_error_m','physical_error_m']):
            ax.plot(shown.time_from_step_s,shown[column],label=label,color=color)
    for ax,title in zip(axes,['Tracking: target − EKF','Estimation: EKF − truth','Physical: target − truth']):
        ax.set_title(title,loc='left');ax.set_ylabel('Signed error (m)');ax.grid(alpha=.2)
        ax.axhspan(-.05,.05,color='#64748b',alpha=.1);ax.axhline(0,color='#64748b',lw=.6)
        ax.set_xlim(5,30);ax.legend(fontsize=9,frameon=False)
    axes[-1].set_xlabel('Simulation time from reference step (s)')
    fig.suptitle('A better control clock does not automatically fix state estimation\nGround truth is used only after flight')
    fig.savefig(args.output/'timing-error-separation.png',dpi=170);fig.savefig(args.output/'timing-error-separation.pdf')
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
