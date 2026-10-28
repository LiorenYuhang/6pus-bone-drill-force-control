"""Render recorded numerical simulations only; no experimental hardware data."""
import argparse,csv
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def rows(p):
    with p.open(newline='') as f: return list(csv.DictReader(f))


def main():
    a=argparse.ArgumentParser(); a.add_argument('directory',type=Path); root=a.parse_args().directory
    for case in ('nominal','dt_020','delay_2','noise','tight_limit'):
        fig,axs=plt.subplots(3,1,figsize=(9,8),sharex=True,constrained_layout=True)
        for kind in ('PID_BASELINE','DS_SCTDE_ORACLE','DS_SCTDE_MODEL_ESTIMATE'):
            data=rows(root/f'{case}_{kind}.csv')
            for ax,key in zip(axs,('eta_error_rad_rms','q_error_m_rms','sync_error_m_rms')):
                ax.plot([float(d['time_s']) for d in data],[float(d[key]) for d in data],label=kind)
        for ax,label in zip(axs,('attitude RMS (rad)','stroke RMS (m)','sync RMS (m)')):
            ax.grid(alpha=.3);ax.set_ylabel(label)
        axs[0].legend(fontsize=8);axs[-1].set_xlabel('time (s)')
        fig.suptitle(f'SIMULATION ONLY: {case}; truncated lines hit modeled stop')
        output=root/f'{case}_comparison.png'
        if output.exists(): raise SystemExit('refusing to overwrite')
        fig.savefig(output,dpi=130);plt.close(fig)
    data=rows(root/'nominal_DS_SCTDE_MODEL_ESTIMATE.csv')
    fig,axs=plt.subplots(2,1,figsize=(9,7),sharex=True,constrained_layout=True)
    for i in range(1,7):
        t=[float(d['time_s']) for d in data]
        axs[0].plot(t,[float(d[f'tau_cmd_{i}_nm']) for d in data],label=f'command {i}')
        axs[0].plot(t,[float(d[f'tau_applied_{i}_nm']) for d in data],linestyle='--',alpha=.5)
    for i in range(1,4):
        axs[1].plot(t,[float(d[f'estimated_task_wrench_{i}_nm']) for d in data],label=f'estimated {i}')
        axs[1].plot(t,[float(d[f'observed_task_wrench_{i}_nm']) for d in data],linestyle='--',alpha=.7,label=f'plant {i}')
    axs[0].set_ylabel('motor torque (N m)');axs[1].set_ylabel('task wrench (N m)');axs[1].set_xlabel('time (s)')
    for ax in axs: ax.grid(alpha=.3); ax.legend(ncol=3,fontsize=7)
    fig.suptitle('SIMULATION ONLY: model estimator (dashed motor curves: applied)')
    output=root/'nominal_model_wrench_and_torque.png'
    if output.exists(): raise SystemExit('refusing to overwrite')
    fig.savefig(output,dpi=130);plt.close(fig)


if __name__=='__main__': main()
