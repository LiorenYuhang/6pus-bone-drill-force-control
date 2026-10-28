"""Identical prescribed synthetic signals; diagnostic comparison, not plant simulation."""
import json
import math
from pathlib import Path
import sys
import numpy as np
from ds_sctde import Config, DSSCTDE
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'joint_torque_pid'))
from controller import Config as PIDConfig, JointTorquePID


def run():
    j=np.vstack((np.eye(3),np.eye(3)))  # illustrative, not 6PUS geometry
    c=DSSCTDE(Config(np.eye(3),[2.]*3,[3.]*3,[1.]*6,[2.]*6,[3.]*6,[4.]*6,[.5]*6,
                     [.01]*6,[100.]*6,[10000.]*6,[1.]*6,.001,.1,.2,10000.))
    p=JointTorquePID(PIDConfig((1.,)*6,(0.,)*6,(0.,)*6,(100.,)*6,(10000.,)*6,(1.,)*6,.001,.1,.2,.02))
    rows=[]; start=1_000_000_000; dt=.02
    for i in range(101):
        t=start+int(i*dt*1e9); sec=i*dt
        desired=.001*math.sin(sec); desired_dot=.001*math.cos(sec); desired_acc=-.001*math.sin(sec)
        # Equal prescribed disturbance for both controllers; no plant dynamics.
        actual=.0001*math.sin(2*sec); actual_dot=.0002*math.cos(2*sec); actual_acc=-.0004*math.sin(2*sec)
        eta=np.array([actual,0,0]); ed=np.array([desired,0,0]); vel=np.array([actual_dot,0,0]); vd=np.array([desired_dot,0,0])
        a=np.array([actual_acc,0,0]); ad=np.array([desired_acc,0,0])
        q=j@eta; qd=j@ed
        s=dict(timestamp_ns=t,fault_codes=[0]*6,eta_rad=eta.tolist(),eta_d_rad=ed.tolist(),
            eta_dot_rad_s=vel.tolist(),eta_dot_d_rad_s=vd.tolist(),eta_ddot_rad_s2=a.tolist(),eta_ddot_d_rad_s2=ad.tolist(),
            q_m=q.tolist(),q_d_m=qd.tolist(),q_dot_m_s=(j@vel).tolist(),q_dot_d_m_s=(j@vd).tolist(),
            q_ddot_d_m_s2=(j@ad).tolist(),friction_motor_nm=[0.]*6,jacobian_m_per_rad=j.tolist(),
            applied_task_wrench_prev_nm=[0.]*3,applied_wrench_timestamp_ns=t-int(dt*1e9))
        ds=c.update(s,t); pid=p.update(qd,q,[t]*6,t,[0]*6)
        if ds is not None: rows.append((ds,pid))
    x=np.array(rows)
    return {'kind':'prescribed synthetic signals, not dynamics or performance', 'steps':len(rows), 'dt_s':dt,
            'pid_output_rms_nm':float(np.sqrt(np.mean(x[:,1,:]**2))),
            'ds_output_rms_nm':float(np.sqrt(np.mean(x[:,0,:]**2))), 'hardware_write_count':0}


if __name__=='__main__': print(json.dumps(run(),indent=2))
