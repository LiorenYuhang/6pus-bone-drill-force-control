"""Plot the archived simulation CSVs. All curves are simulated, never measurements."""
import argparse
import csv
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def read(path):
    with path.open(newline='') as f: return list(csv.DictReader(f))


def main():
    ap=argparse.ArgumentParser(); ap.add_argument('directory',type=Path); args=ap.parse_args(); root=args.directory
    for sc in ['nominal','delay_2','noise','tight_limit','command_proxy']:
        fig,axs=plt.subplots(3,1,figsize=(9,8),sharex=True,constrained_layout=True)
        for kind in ['PID_BASELINE','DS_SCTDE']:
            rows=read(root/f'{sc}_{kind}.csv'); t=np.array([float(x['time_s']) for x in rows])
            for ax,col in zip(axs,['eta_error_rad_rms','q_error_m_rms','sync_error_m_rms']):
                ax.plot(t,[float(x[col]) for x in rows],label=kind)
        for ax,label in zip(axs,['attitude error (rad RMS)','axis error (m RMS)','sync error (m RMS)']):
            ax.set_ylabel(label); ax.grid(alpha=.3)
        axs[0].legend(); axs[-1].set_xlabel('simulation time (s)')
        fig.suptitle(f'SIMULATION ONLY: {sc} (truncated curves ended at simulated limit)')
        fig.savefig(root/f'{sc}_comparison.png',dpi=130); plt.close(fig)
    rows=read(root/'nominal_DS_SCTDE.csv'); t=[float(x['time_s']) for x in rows]
    fig,axs=plt.subplots(2,1,figsize=(9,7),sharex=True,constrained_layout=True)
    for i in range(1,7): axs[0].plot(t,[float(x[f'tau_cmd_{i}_nm']) for x in rows],label=f'axis {i}')
    for i in range(1,4):
        selected=[x for x in rows if x[f'tde_estimate_{i}_nm']]
        axs[1].plot([float(x['time_s']) for x in selected],[float(x[f'tde_estimate_{i}_nm']) for x in selected],label=f'H estimate {i}')
        axs[1].plot(t,[float(x[f'true_lumped_{i}_nm']) for x in rows],linestyle='--',alpha=.7,label=f'H plant {i}')
    axs[0].set_ylabel('command (N m)'); axs[1].set_ylabel('task H (N m)'); axs[1].set_xlabel('simulation time (s)')
    for ax in axs: ax.grid(alpha=.3); ax.legend(ncol=3,fontsize=7)
    fig.suptitle('SIMULATION ONLY: nominal motor commands and TDE'); fig.savefig(root/'nominal_tde_torque.png',dpi=130); plt.close(fig)


if __name__=='__main__': main()
