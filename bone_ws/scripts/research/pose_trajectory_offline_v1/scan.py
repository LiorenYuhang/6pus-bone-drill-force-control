#!/usr/bin/env python3
"""Offline IK/reference-shaper study. No ROS node, hardware transport or control output."""
import argparse
import csv
import json
import math
from pathlib import Path
import numpy as np
from pus6_kinematics.analytical_6pus import Analytical6PusKinematics
from pus6_control.joint_trajectory_controller import StatefulJerkLimitedShaper

SCENARIOS={'roll':(1,0,0),'pitch':(0,1,0),'yaw':(0,0,1),'roll_pitch':(1,1,0),
           'roll_yaw':(1,0,1),'pitch_yaw':(0,1,1),'roll_pitch_yaw':(1,1,1)}

def evaluate(amp_deg=.25,period_s=5.,cycles=2,dt=.02):
    if not all(math.isfinite(v) and v>0 for v in (amp_deg,period_s,dt)) or not isinstance(cycles,int) or cycles<1:
        raise ValueError('invalid offline parameters')
    samples=round(period_s/dt)
    if samples<2 or abs(samples*dt-period_s)>1e-9: raise ValueError('period must be an integer number of samples')
    kin=Analytical6PusKinematics()
    home=np.array(kin.inverse_kinematics([0.,0.,kin.nominal_height,0.,0.,0.]))
    results={}
    for name,mask in SCENARIOS.items():
        shaper=StatefulJerkLimitedShaper(initial_positions=home.tolist())
        rows=[]
        for i in range(cycles*samples+1):
            t=i*dt; angle=math.radians(amp_deg)*math.sin(2*math.pi*t/period_s)
            rpy=[angle*axis for axis in mask]
            raw=np.array(kin.inverse_kinematics([0.,0.,kin.nominal_height,*rpy]))
            shaped=np.array(shaper.update(raw.tolist(),dt)) if i else home.copy()
            row={'sample':i,'t_s':t,'roll_ref_rad':rpy[0],'pitch_ref_rad':rpy[1],'yaw_ref_rad':rpy[2]}
            for j in range(6):
                row[f'ik_q{j+1}_m']=float(raw[j]); row[f'shaped_q{j+1}_m']=float(shaped[j])
                row[f'ik_delta_q{j+1}_steps']=float((raw[j]-home[j])*kin.steps_per_meter)
            rows.append(row)
        raw_delta=np.array([[r[f'ik_delta_q{j+1}_steps'] for j in range(6)] for r in rows])
        shaped=np.array([[r[f'shaped_q{j+1}_m'] for j in range(6)] for r in rows])
        raw=np.array([[r[f'ik_q{j+1}_m'] for j in range(6)] for r in rows])
        results[name]={'rows':rows,'max_abs_ik_delta_steps':np.max(abs(raw_delta),axis=0).tolist(),
                       'max_ik_vs_shaped_m':np.max(abs(raw-shaped),axis=0).tolist(),
                       'last_shaped_minus_home_m':(shaped[-1]-home).tolist()}
    return results

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--amplitude-deg',type=float,default=.25);p.add_argument('--period-s',type=float,default=5.)
    p.add_argument('--cycles',type=int,default=2); p.add_argument('--dt',type=float,default=.02); a=p.parse_args()
    results=evaluate(a.amplitude_deg,a.period_s,a.cycles,a.dt)
    a.output.mkdir(parents=True,exist_ok=False)
    summary={'classification':'EVALUATED_OFFLINE','real_motion':'NOT_EXECUTED','hardware_write_count':0,
             'note':'IK and shaper only; not actuator tracking, measured pose, safety approval or physical repeatability',
             'amplitude_deg':a.amplitude_deg,'period_s':a.period_s,'cycles':a.cycles,'dt_s':a.dt,'scenarios':{}}
    for name,result in results.items():
        with (a.output/f'{name}.csv').open('x',newline='') as f:
            writer=csv.DictWriter(f,fieldnames=result['rows'][0]);writer.writeheader();writer.writerows(result['rows'])
        summary['scenarios'][name]={key:value for key,value in result.items() if key!='rows'}
    (a.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(len(SCENARIOS),1,figsize=(10,14),sharex=True,layout='constrained')
    for ax,(name,result) in zip(axes,results.items()):
        rows=result['rows']
        for j in range(6): ax.plot([r['t_s'] for r in rows],[r[f'ik_delta_q{j+1}_steps'] for r in rows],label=f'ID{j+1}',linewidth=.8)
        ax.set_ylabel(name+'\nsteps'); ax.grid(True,alpha=.25)
    axes[0].legend(ncol=6);axes[-1].set_xlabel('offline model time (s)')
    fig.suptitle('OFFLINE IK references only; no measured motion')
    fig.savefig(a.output/'offline_ik_references.png',dpi=140);plt.close(fig)
    print(json.dumps({'output':str(a.output),'classification':summary['classification'],'real_motion':'NOT_EXECUTED','hardware_write_count':0}))
if __name__=='__main__': main()
