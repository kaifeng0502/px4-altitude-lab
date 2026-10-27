#!/usr/bin/env python3
"""Report all sequential flight-quality trials using physical ground truth offline."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from pyulog import ULog


def evaluate(run):
    meta=json.loads((run/'metadata.json').read_text());metrics=json.loads((run/'metrics.json').read_text())
    if meta['status']!='completed' or not metrics.get('mpc_acceleration_only_verified_in_ulog'):
        raise ValueError(f'Incomplete/unverified run: {run}')
    events=json.loads((run/'events.json').read_text())
    landing=next(e['boot_s'] for e in events if e['event']=='landing')
    ground=next(e for e in events if e['event']=='ground_reference')
    alignment=next((e for e in events if e['event']=='ground_alignment_complete'),None)
    true=pd.read_csv(run/'vehicle_local_position_groundtruth.csv')
    true['t']=true.boot_s-metrics['step_boot_s']
    true['height']=metrics['truth_ground_reference_ned_z_m']-true.z
    after=true[(true.t>=0)&(true.boot_s<landing-.1)]
    error=np.abs(after.height.to_numpy()-4.)
    outside=np.flatnonzero(error>.05);index=int(outside[-1])+1 if len(outside) else 0
    settling=float(after.t.iloc[index]) if index<len(after) else None
    if settling is not None and after.t.iloc[-1]-settling<2.:settling=None
    tail=after[after.t>=after.t.iloc[-1]-2.]
    physical={'final_mean_height_m':float(tail.height.mean()),'final_mean_error_m':float(4.-tail.height.mean()),
              'overshoot_m':max(0.,float(after.height.max()-4.)),
              'settling_time_5cm_s':settling,'peak_abs_velocity_m_s':float(after.vz.abs().max()),
              'integrated_absolute_error_m_s':float(np.trapezoid(error,after.t))}
    pos=pd.read_csv(run/'vehicle_local_position.csv')
    pre=pos[(pos.boot_s>=ground['boot_s']-3)&(pos.boot_s<=ground['boot_s'])]
    pre_quality={'mean_estimated_ned_velocity_m_s':float(pre.vz.mean()),
                 'alignment_elapsed_sim_s':alignment['elapsed_sim_s'] if alignment else 15.,
                 'alignment_quality':alignment.get('quality') if alignment else None}
    ulog=ULog(str(next(run.rglob('*.ulg'))))
    params={k:v for k,v in ulog.initial_parameters.items() if k.startswith('EKF2_')}
    (run/'estimator-parameters.json').write_text(json.dumps(params,indent=2)+'\n')
    ctrl=pd.read_csv(run/'controller.csv')
    selected=ctrl[(ctrl.boot_s>=metrics['step_boot_s'])&(ctrl.boot_s<landing)]
    correction=selected.height_reference_correction_m if 'height_reference_correction_m' in selected else pd.Series([0.])
    record={'run':run.name,'ground_init':meta.get('ground_init','legacy'),
            'motion_profile':meta.get('motion_profile','standard'),'bias_compensation':meta.get('bias_compensation','off'),
            'mpc_config':meta['mpc'],'ground_quality':pre_quality,'physical':physical,'estimated_metrics':metrics,
            'max_abs_reference_correction_m':float(correction.abs().max()) if correction.notna().any() else 0.}
    return record,meta,params,pd.read_csv(run/'response.csv')


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--baseline',type=Path,required=True)
    parser.add_argument('--height',type=Path,required=True)
    parser.add_argument('--gentle',type=Path,nargs='+',required=True)
    parser.add_argument('--compensated',type=Path,nargs='+',required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    runs=[args.baseline,args.height]+args.gentle+args.compensated
    values=[evaluate(run) for run in runs]
    for record,meta,params,df in values:
        if params!=values[0][2]:raise SystemExit('EKF parameters changed; do not attribute this to ground initialization')
        for key in ['model','px4_commit','timing_mode','timing_version']:
            if meta.get(key)!=values[0][1].get(key):raise SystemExit(f'Experiment setup differs: {key}')
    args.output.mkdir(parents=True,exist_ok=True)
    report={'trials':[v[0] for v in values],'ekf_parameters_unchanged':True,
            'truth_usage':'post-flight analysis only; never read by ground gate, MPC or bias correction',
            'scope':'Sequential small-sample ablation; sensor noise and initialization differ; all planned completed trials retained'}
    for name,paths in [('gentle_off',args.gentle),('gentle_integral',args.compensated)]:
        records=[v[0] for p,v in zip(runs,values) if p in paths]
        report[name]={'count':len(records),
            'mean_abs_final_true_error_m':float(np.mean([abs(x['physical']['final_mean_error_m']) for x in records])),
            'mean_true_overshoot_m':float(np.mean([x['physical']['overshoot_m'] for x in records])),
            'mean_peak_abs_true_velocity_m_s':float(np.mean([x['physical']['peak_abs_velocity_m_s'] for x in records])),
            'mean_abs_final_estimated_error_m':float(np.mean([abs(x['estimated_metrics']['final_mean_error_m']) for x in records])),
            'true_settled_trials':sum(x['physical']['settling_time_5cm_s'] is not None for x in records)}
    report['selected_defaults']={'ground_init':'settled','motion_profile':'gentle','bias_compensation':'off',
        'reason':'Bounded reference integration was implemented and flown twice, but did not consistently improve physical endpoint error; retain as an option only'}
    (args.output/'comparison.json').write_text(json.dumps(report,indent=2)+'\n')
    plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False})
    fig,axes=plt.subplots(3,1,figsize=(11,10),sharex=True,layout='constrained')
    counts={}
    colors={'baseline':'#d97706','height':'#7c3aed','gentle':'#1769aa','integral':'#00897b'}
    for path,(record,meta,params,df) in zip(runs,values):
        stage='baseline' if path==args.baseline else 'height' if path==args.height else 'gentle' if path in args.gentle else 'integral'
        counts[stage]=counts.get(stage,0)+1
        labels={'baseline':'Original','height':'Settled ground','gentle':'Settled + gentle','integral':'Settled + gentle + integral'}
        label=labels[stage]+f' · trial {counts[stage]}'
        style='-' if counts[stage]==1 else '--'
        for ax,field in zip(axes,['true_height_m','up_velocity_m_s','height_m']):
            ax.plot(df.time_from_step_s,df[field],color=colors[stage],ls=style,alpha=.85,label=label)
    axes[0].plot([-2,0,0,30],[2,2,4,4],':',color='#111827',label='Desired height')
    axes[0].axhspan(3.95,4.05,color='#64748b',alpha=.12)
    axes[0].set_ylabel('Physical height (m)');axes[1].set_ylabel('EKF upward velocity (m/s)');axes[2].set_ylabel('EKF height (m)')
    for ax in axes:ax.grid(alpha=.2);ax.set_xlim(-2,30);ax.legend(fontsize=7,frameon=False,ncol=2)
    axes[-1].set_xlabel('Simulation time from MPC reference step (s)')
    fig.suptitle('Sequential improvements: ground alignment, gentler motion, bounded reference integral\nGround truth appears only in this post-flight comparison')
    fig.savefig(args.output/'flight-quality-comparison.png',dpi=170);fig.savefig(args.output/'flight-quality-comparison.pdf')
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
