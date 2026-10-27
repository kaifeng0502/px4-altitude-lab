#!/usr/bin/env python3
"""Render the final saved flight comparison with Chinese labels; no simulation."""
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
import pandas as pd

ROOT=Path(__file__).resolve().parents[1]
REPORT=ROOT/'results/20261027-flight-quality-pipeline/comparison.json'
OUT=ROOT/'deliverables'

def main():
    font_manager.findfont('Noto Sans CJK JP',fallback_to_default=False)
    plt.rcParams.update({'font.family':'Noto Sans CJK JP','font.size':10,
        'axes.unicode_minus':False,'axes.spines.top':False,'axes.spines.right':False})
    report=json.loads(REPORT.read_text())
    fig,axes=plt.subplots(3,1,figsize=(11,10),sharex=True,layout='constrained')
    counts={}
    for i,record in enumerate(report['trials']):
        stage='baseline' if i==0 else 'height' if i==1 else 'integral' if record['bias_compensation']=='integral' else 'gentle'
        names={'baseline':'旧流程对照','height':'稳定地面基准','gentle':'稳定基准＋温和动作','integral':'稳定基准＋温和动作＋尾差补偿'}
        colors={'baseline':'#d97706','height':'#7c3aed','gentle':'#1769aa','integral':'#00897b'}
        counts[stage]=counts.get(stage,0)+1
        label=f'{names[stage]}（第{counts[stage]}次）'
        df=pd.read_csv(ROOT/'results'/record['run']/'response.csv')
        for ax,field in zip(axes,['true_height_m','up_velocity_m_s','height_m']):
            ax.plot(df.time_from_step_s,df[field],color=colors[stage],
                    ls='-' if counts[stage]==1 else '--',lw=1.7,alpha=.9,label=label)
    axes[0].plot([-2,0,0,30],[2,2,4,4],':',color='#111827',label='目标高度：2 米 → 4 米')
    axes[0].axhspan(3.95,4.05,color='#64748b',alpha=.12)
    for ax,title,ylabel in zip(axes,['① 仿真真实高度：判断飞得准不准','② 飞控估计的向上速度：判断动作快慢','③ 飞控估计高度：对照真实高度看估计偏差'],['真实高度（米）','估计向上速度（米/秒）','估计高度（米）']):
        ax.set_title(title,loc='left',fontweight='bold');ax.set_ylabel(ylabel)
        ax.set_xlim(-2,30);ax.grid(alpha=.2);ax.legend(fontsize=8,frameon=False,ncol=2)
    axes[-1].set_xlabel('从 MPC 收到 4 米目标起算的仿真时间（秒）')
    fig.suptitle('PX4 一维高度 MPC：六次顺序优化试验\n真值仅用于飞行结束后的分析；同色实线、虚线表示同方案的两次试验',fontsize=14)
    OUT.mkdir(exist_ok=True)
    fig.savefig(OUT/'最终效果对比.png',dpi=170)
    fig.savefig(OUT/'最终效果对比.pdf')
    print(OUT/'最终效果对比.png')

if __name__=='__main__':main()
