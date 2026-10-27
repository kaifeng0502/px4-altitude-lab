#!/usr/bin/env python3
"""Compare fresh nominal/calibrated MPC flights without retuning the controller."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('nominal',type=Path)
    parser.add_argument('calibrated',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--nominal-repeat',type=Path)
    parser.add_argument('--calibrated-repeat',type=Path)
    args=parser.parse_args()
    runs=[args.nominal.resolve(),args.calibrated.resolve()]
    meta=[json.loads((p/'metadata.json').read_text()) for p in runs]
    metrics=[json.loads((p/'metrics.json').read_text()) for p in runs]
    for m in meta:
        if m['status']!='completed' or m.get('controller')!='mpc':
            raise SystemExit('Both runs must be completed MPC flights')
    if meta[0].get('mpc_model')!='nominal' or meta[1].get('mpc_model')!='calibrated':
        raise SystemExit('Expected fresh nominal run, then calibrated run')
    for key in ['model','simulator','px4_commit','initial_height_m','final_height_m','observation_sim_s']:
        if meta[0][key]!=meta[1][key]:
            raise SystemExit(f'Run conditions differ: {key}')
    if any(m.get('timing_mode','legacy')!=meta[0].get('timing_mode','legacy') or m.get('local_position_stream_hz',20)!=meta[0].get('local_position_stream_hz',20) for m in meta):
        raise SystemExit('Timing/telemetry settings differ; isolate model changes first')
    def timing_version(m):
        return m.get('timing_version',1 if m.get('timing_mode')=='scheduled' else 0)
    if any(timing_version(m)!=timing_version(meta[0]) for m in meta):
        raise SystemExit('Timing algorithm versions differ')
    if any((m.get('ground_init','legacy'),m.get('bias_compensation','off'))!=(meta[0].get('ground_init','legacy'),meta[0].get('bias_compensation','off')) for m in meta):
        raise SystemExit('Ground initialization/reference compensation differ')
    tuning=['dt','horizon','height_weight','velocity_weight','acceleration_weight',
            'delta_acceleration_weight','velocity_slack_weight','max_velocity',
            'min_acceleration','max_acceleration','max_jerk']
    for key in tuning:
        if meta[0]['mpc'][key]!=meta[1]['mpc'][key]:
            raise SystemExit(f'Controller was retuned: {key}')
    params=[json.loads((p/'parameters.json').read_text()) for p in runs]
    if params[0]!=params[1]:
        raise SystemExit('Exported PX4 parameters differ')
    if not all(m.get('mpc_acceleration_only_verified_in_ulog') for m in metrics):
        raise SystemExit('Acceleration-only interface was not verified in both logs')
    if any(m['max_abs_estimate_minus_truth_m']>.5 for m in metrics):
        raise SystemExit('Excessive estimator/truth disagreement; investigate before comparing')
    repeats=[]
    for index, repeat in enumerate([args.nominal_repeat,args.calibrated_repeat]):
        if repeat is None:
            continue
        repeat=repeat.resolve()
        mm=json.loads((repeat/'metadata.json').read_text())
        mt=json.loads((repeat/'metrics.json').read_text())
        pp=json.loads((repeat/'parameters.json').read_text())
        if mm.get('timing_mode','legacy')!=meta[index].get('timing_mode','legacy') or mm.get('local_position_stream_hz',20)!=meta[index].get('local_position_stream_hz',20):
            raise SystemExit('Repeat timing/telemetry settings differ')
        if (mm.get('ground_init','legacy'),mm.get('bias_compensation','off'))!=(meta[index].get('ground_init','legacy'),meta[index].get('bias_compensation','off')):
            raise SystemExit('Repeat ground initialization/reference compensation differ')
        if timing_version(mm)!=timing_version(meta[index]):
            raise SystemExit('Repeat timing algorithm version differs')
        if mm['status']!='completed' or mm.get('mpc_model')!=meta[index]['mpc_model']:
            raise SystemExit('Repeat has wrong status/model')
        for key in ['model','simulator','px4_commit','initial_height_m','final_height_m','observation_sim_s']:
            if mm[key]!=meta[index][key]:
                raise SystemExit(f'Repeat conditions differ: {key}')
        if any(mm['mpc'][key]!=meta[index]['mpc'][key] for key in tuning+['acceleration_gain','acceleration_bias']):
            raise SystemExit('Repeat model or tuning differs')
        if pp!=params[index] or not mt.get('mpc_acceleration_only_verified_in_ulog'):
            raise SystemExit('Repeat PX4 parameters/interface not verified')
        if mt['max_abs_estimate_minus_truth_m']>.5:
            raise SystemExit('Repeat estimator/truth disagreement too large')
        repeats.append((index,repeat,mm,mt))
    args.output.mkdir(parents=True,exist_ok=True)
    data=[pd.read_csv(p/'response.csv') for p in runs]
    labels=['Original: a = u','Calibrated: a = K·u + b']
    colors=['#d97706','#1769aa']
    plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False})
    fig,axes=plt.subplots(4,1,figsize=(10,11),sharex=True,layout='constrained')
    for df,label,color in zip(data,labels,colors):
        for ax,field,ylabel in zip(axes,
             ['height_m','up_velocity_m_s','up_acceleration_target_m_s2','up_thrust_normalized_ned'],
             ['EKF height (m)','Upward velocity (m/s)','Acceleration command (m/s²)','Normalized thrust']):
            ax.plot(df.time_from_step_s,df[field],color=color,label=label)
            ax.set_ylabel(ylabel)
    for index,repeat,mm,mt in repeats:
        df=pd.read_csv(repeat/'response.csv')
        for ax,field in zip(axes,['height_m','up_velocity_m_s','up_acceleration_target_m_s2','up_thrust_normalized_ned']):
            ax.plot(df.time_from_step_s,df[field],color=colors[index],ls='--',alpha=.6,label=labels[index]+' (repeat)')
    axes[0].plot([-3,0,0,30],[2,2,4,4],'--',color='#475569',label='Height reference')
    axes[0].axhspan(3.95,4.05,color='#64748b',alpha=.12)
    axes[1].axhline(meta[0]['mpc']['max_velocity'],ls=':',color='#64748b',label='Soft speed limit')
    axes[1].axhline(-meta[0]['mpc']['max_velocity'],ls=':',color='#64748b')
    for ax in axes:
        ax.axvline(0,color='#64748b',alpha=.4);ax.grid(alpha=.2)
        ax.legend(fontsize=9,frameon=False);ax.set_xlim(-3,30)
    axes[-1].set_xlabel('Simulation time from first MPC 4 m decision (s)')
    fig.suptitle('Model calibration only: original versus calibrated two-state MPC\nFresh Gazebo worlds; same weights, horizon and input constraints',fontsize=14)
    fig.savefig(args.output/'mpc-model-comparison.png',dpi=170)
    fig.savefig(args.output/'mpc-model-comparison.pdf')

    fig,axes=plt.subplots(2,1,figsize=(11,7),sharex=True,layout='constrained')
    for ax,run,m,metric,label,color,df in zip(axes,runs,meta,metrics,labels,colors,data):
        ax.plot(df.time_from_step_s,df.height_m,color='#111827',lw=2,label='EKF feedback')
        ps=[json.loads(line) for line in (run/'mpc-predictions.jsonl').read_text().splitlines()]
        ps=[p for p in ps if p['target_height_m']==4.]
        for offset in [0.,1.]:
            p=min(ps,key=lambda p:abs(p['boot_s']-metric['step_boot_s']-offset))
            x=np.array(p['predicted_states'])
            t=p['boot_s']-metric['step_boot_s']+np.arange(len(x))*m['mpc']['dt']
            ax.plot(t,x[:,0],'--',label=f"Forecast at {p['boot_s']-metric['step_boot_s']:.2f} s")
        ax.axhline(4,color='#64748b',ls=':');ax.set_title(label,loc='left')
        ax.set_ylabel('Height (m)');ax.grid(alpha=.2);ax.legend(fontsize=9,frameon=False);ax.set_xlim(-.3,7)
    axes[-1].set_xlabel('Simulation time from MPC reference step (s)')
    fig.suptitle('Actual optimizer forecasts before and after model calibration')
    fig.savefig(args.output/'mpc-model-forecasts.png',dpi=170)
    fig.savefig(args.output/'mpc-model-forecasts.pdf')
    fig,axes=plt.subplots(2,1,figsize=(10,6),sharex=True,layout='constrained')
    overview=[(index,run,1) for index,run in enumerate(runs)]+[(index,run,2) for index,run,mm,mt in repeats]
    for index,run,trial in overview:
        df=pd.read_csv(run/'response.csv')
        label=('Original' if index==0 else 'Calibrated')+f' · trial {trial}'
        for ax,field in zip(axes,['height_m','up_velocity_m_s']):
            ax.plot(df.time_from_step_s,df[field],color=colors[index],
                    ls='-' if trial==1 else '--',alpha=1. if trial==1 else .65,label=label)
    axes[0].plot([-1,0,0,8],[2,2,4,4],':',color='#475569',label='Reference')
    axes[0].axhspan(3.95,4.05,color='#64748b',alpha=.12)
    axes[1].axhline(meta[0]['mpc']['max_velocity'],ls=':',color='#64748b',label='Soft speed limit')
    axes[0].set_ylabel('EKF height (m)');axes[1].set_ylabel('Upward velocity (m/s)')
    for ax in axes:
        ax.set_xlim(-1,8);ax.grid(alpha=.2);ax.legend(fontsize=8,frameon=False)
    axes[-1].set_xlabel('Simulation time from MPC reference step (s)')
    fig.suptitle('Two-state model calibration: early response in all four flights')
    fig.savefig(args.output/'mpc-model-overview.png',dpi=170)
    fig.savefig(args.output/'mpc-model-overview.pdf')

    # Score fixed K,b on each NEW flight. Refit neither model on these runs.
    gain=meta[1]['mpc']['acceleration_gain'];bias=meta[1]['mpc']['acceleration_bias']
    holdout=[]
    for run,metric in zip(runs,metrics):
        p=pd.read_csv(run/'vehicle_local_position.csv')
        u=pd.read_csv(run/'vehicle_local_position_setpoint.csv')
        t=np.arange(metric['step_boot_s'],metric['step_boot_s']+6,.02)
        index=np.clip(np.searchsorted(u.boot_s,t,side='right')-1,0,len(u)-1)
        command=-u['acceleration[2]'].to_numpy()[index]
        measured=np.interp(t,p.boot_s,-p.az)
        holdout.append({'run':run.name,'sample_count':len(t),
            'nominal_acceleration_rmse':float(np.sqrt(np.mean((measured-command)**2))),
            'calibrated_acceleration_rmse':float(np.sqrt(np.mean((measured-gain*command-bias)**2)))})
    report={'nominal_run':str(runs[0]),'calibrated_run':str(runs[1]),
            'same_exported_px4_parameters':True,'same_controller_tuning':{k:meta[0]['mpc'][k] for k in tuning},
            'calibrated_gain':gain,'calibrated_bias':bias,
            'nominal':metrics[0],'calibrated':metrics[1],'new_flight_holdout_prediction_errors':holdout,
            'scope':'One fresh flight per model; local EKF-based calibration; no general superiority claim'}
    report['all_trials']=[{'model':m['mpc_model'],'run':run.name,'metrics':metric}
        for run,m,metric in zip(runs,meta,metrics)]+[
        {'model':mm['mpc_model'],'run':repeat.name,'metrics':mt} for index,repeat,mm,mt in repeats]
    report['scope']='All scheduled fresh trials are retained; small-sample comparison; local EKF fit'
    report['summary']={}
    for name in ['nominal','calibrated']:
        samples=[x['metrics'] for x in report['all_trials'] if x['model']==name]
        report['summary'][name]={'count':len(samples),
            'mean_rise_time_s':float(np.mean([x['rise_time_10_90_s'] for x in samples])),
            'mean_overshoot_m':float(np.mean([x['overshoot_m'] for x in samples])),
            'mean_peak_velocity_m_s':float(np.mean([x['peak_up_velocity_m_s'] for x in samples])),
            'mean_absolute_final_estimated_error_m':float(np.mean([abs(x['final_mean_error_m']) for x in samples])),
            'settled_trials':sum(x['settling_time_within_5cm_s'] is not None for x in samples)}
    (args.output/'comparison.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__=='__main__':
    main()
