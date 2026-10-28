"""Nonlinear nominal geometry with synthetic dynamics: research simulation only."""
from dataclasses import dataclass
from collections import deque
from pathlib import Path
import argparse
import csv
import json
import math
import sys
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / 'ds_sctde'))
sys.path.insert(0, str(HERE.parent / 'joint_torque_pid'))
from ds_sctde import Config as DSConfig, DSSCTDE, ControllerFault
from controller import Config as PIDConfig, JointTorquePID, Fault
from kinematics_map import attitude_jacobian
from pus6_kinematics.analytical_6pus import Analytical6PusKinematics


@dataclass(frozen=True)
class Scenario:
    name: str
    dt: float = .005
    delay_cycles: int = 0
    noise_acc: float = 0.0
    noise_vel: float = 0.0
    noise_angle: float = 0.0
    noise_position: float = 0.0
    noise_joint_vel: float = 0.0
    torque_limit: float = .008
    disturbance_scale: float = 1.0
    source: str = 'simulation_oracle'
    duration: float = 4.0
    seed: int = 1028
    equal_axes: bool = False
    stiffness_scale: float = 1.0
    motor_lag_scale: float = 1.0
    map_scale: float = 1.0
    observer_stiffness_scale: float = 0.8
    quantize: bool = True
    max_substep: float = .0005
    pid_kd: float = .005


CASES = [Scenario('nominal',dt=.01,duration=2.), Scenario('dt_020',dt=.02,duration=2.),
         Scenario('pid_original_d',dt=.01,duration=2.,pid_kd=.04),
         Scenario('integrator_half',dt=.01,duration=2.,max_substep=.00025),
         Scenario('delay_2',dt=.01,duration=2.,delay_cycles=2),
         Scenario('noise',dt=.01,duration=2.,noise_angle=.0001,noise_position=.0000005,noise_acc=.04,noise_vel=.0003),
         Scenario('disturbance_3x',dt=.01,duration=2.,disturbance_scale=3.),
         Scenario('stiffness_low',dt=.01,duration=2.,stiffness_scale=.7),
         Scenario('stiffness_high',dt=.01,duration=2.,stiffness_scale=1.3),
         Scenario('motor_lag_2x',dt=.01,duration=2.,motor_lag_scale=2.),
         Scenario('mapping_error',dt=.01,duration=2.,map_scale=1.1),
         Scenario('tight_limit',dt=.01,duration=2.,torque_limit=.00004)]


def geometry():
    kin=Analytical6PusKinematics()
    return attitude_jacobian(kin,[0.,0.,kin.nominal_height,0.,0.,0.])


KIN=Analytical6PusKinematics()


def nonlinear_geometry(eta, kin=None):
    """IK from existing model; analytic J derived from its slider constraint."""
    eta=np.asarray(eta,dtype=float)
    kin=kin or KIN
    pose=[0.,0.,kin.nominal_height,*eta]
    q=np.asarray(kin.inverse_kinematics(pose))
    a=np.asarray(kin.base_anchors); b=np.asarray(kin.platform_anchors_body)
    r=np.asarray(kin._rotation_matrix_zyx(*eta)); pb=b@r.T
    dx=pb[:,0]-a[:,0]; dy=pb[:,1]-a[:,1]
    root=np.sqrt(kin.strut_length**2-dx**2-dy**2)
    if np.any(root < 1e-8): raise ValueError('singular slider geometry')
    js=[]
    for method in (kin._drot_dalpha,kin._drot_dbeta,kin._drot_dgamma):
        dp=b@np.asarray(method(*eta)).T
        js.append(dp[:,2]+(dx*dp[:,0]+dy*dp[:,1])/root)
    return q,np.column_stack(js)


class CausalDifference:
    """Causal finite difference with first-order low-pass, not hardware qualified."""
    def __init__(self,tau=.03): self.previous=None; self.vel=None; self.acc=None; self.tau=tau
    def update(self,t,eta,q):
        x=np.r_[eta,q]
        if self.previous is None:
            self.previous=(t,x.copy()); return None
        dt=t-self.previous[0]
        if dt<=0 or dt>.1: raise ValueError('invalid observer interval')
        v=(x-self.previous[1])/dt; alpha=dt/(self.tau+dt)
        if self.vel is None:
            self.vel=v; self.previous=(t,x.copy()); return None
        vn=self.vel+alpha*(v-self.vel)
        a=(vn[:3]-self.vel[:3])/dt
        self.acc=a if self.acc is None else self.acc+alpha*(a-self.acc)
        self.vel=vn; self.previous=(t,x.copy())
        return vn[:3],vn[3:],self.acc.copy()


def modeled_wrench(eta,velocity,q,qvelocity,k_model,b_model,map_scale=1.):
    """Experimental spring/damper estimator, requires unverified parameters."""
    qgeom,j=nonlinear_geometry(eta); j=j*map_scale
    f=np.asarray(k_model)*(np.asarray(q)-qgeom)+np.asarray(b_model)*(np.asarray(qvelocity)-j@np.asarray(velocity))
    return j.T@f


def disturbance(t, scale):
    return scale*np.array([.0015*(t>=1.), .001*math.sin(2*math.pi*.5*t), -.0006*(t>=2.)])


def trajectory(t, j):
    # Starts at rest; all derivatives are analytically matched to the target.
    amp=np.array([.02,-.012,.012]); omega=2*math.pi*.25
    e=amp*(1-math.cos(omega*t)); v=amp*omega*math.sin(omega*t); a=amp*omega**2*math.cos(omega*t)
    q,jd=nonlinear_geometry(e)
    eps=1e-4
    _,jp=nonlinear_geometry(e+eps*v); _,jm=nonlinear_geometry(e-eps*v)
    return e,v,a, q,jd@v,jd@a+((jp-jm)/(2*eps))@v


class Plant:
    """M eta_dd=J^T f - damping - disturbance; m q_dd=tau/n-f-friction.

    f=K(q-J eta)+B(qdot-J etadot); six unequal masses, damping and
    motor time constants. q is deviation from 0.05 m. All parameters synthetic.
    """
    def __init__(self,j,equal_axes=False,stiffness_scale=1.,motor_lag_scale=1.):
        self.j=j; self.eta=np.zeros(3); self.vel=np.zeros(3)
        self.q=np.zeros(6); self.qvel=np.zeros(6); self.motor=np.zeros(6)
        self.mass=np.array([.025,.03,.02]); self.axis_mass=np.array([.20,.23,.19,.25,.21,.24])
        self.spring=np.array([700.,780.,740.,820.,760.,800.]); self.damp=np.array([4.,4.5,4.2,4.8,4.1,4.6])
        self.axis_drag=np.array([1.,1.1,1.2,.9,1.0,1.3]); self.rot_drag=np.array([.005,.006,.004])
        self.motor_tau=np.array([.008,.009,.011,.010,.012,.009]); self.n=np.array([.001]*6)
        self.spring *= stiffness_scale; self.motor_tau *= motor_lag_scale
        if equal_axes:
            for name in ('axis_mass','spring','damp','axis_drag','motor_tau'):
                value=getattr(self,name); setattr(self,name,np.full(6,float(np.mean(value))))

    def state(self,t,scale):
        qkin,j=nonlinear_geometry(self.eta)
        f=self.spring*(.05+self.q-qkin)+self.damp*(self.qvel-j@self.vel)
        w=j.T@f
        accel=(w-self.rot_drag*self.vel-disturbance(t,scale))/self.mass
        return f,w,accel

    def step(self,cmd,t,h,scale):
        f,_,acc=self.state(t,scale)
        self.motor += (cmd-self.motor)*min(h/self.motor_tau.min(),1.) * (self.motor_tau.min()/self.motor_tau)
        # smooth Coulomb approximation plus viscous drag
        qacc=(self.motor/self.n-f-self.axis_drag*self.qvel-.02*np.tanh(self.qvel/.001))/self.axis_mass
        self.vel += h*acc; self.eta += h*self.vel
        self.qvel += h*qacc; self.q += h*self.qvel


def make_controller(kind,j,sc):
    if kind=='PID_BASELINE':
        return JointTorquePID(PIDConfig((1.5,)*6,(0.,)*6,(sc.pid_kd,)*6,(sc.torque_limit,)*6,(.2,)*6,(1.,)*6,.0005,.05,.2,.01))
    # Nominal task inertia is deliberately not the plant inertia; controller
    # has no access to spring, friction, disturbance or actual axis masses.
    return DSSCTDE(DSConfig(np.diag([.020,.024,.016]),[20.]*3,[8.]*3,[2.]*6,[3.]*6,
        [.15]*6,[.04]*6,[.00020]*6,[.001]*6,[sc.torque_limit]*6,[.2]*6,
        [.01]*6,.0005,.05,.2,10000.,allow_unverified_proxy=sc.source=='command_proxy_unverified'))


def simulate(sc, kind):
    if kind not in ('PID_BASELINE','DS_SCTDE_ORACLE','DS_SCTDE_MODEL_ESTIMATE'): raise ValueError('unknown controller')
    j=geometry(); p=Plant(j,sc.equal_axes,sc.stiffness_scale,sc.motor_lag_scale); ctrl=make_controller(kind,j,sc); rng=np.random.default_rng(sc.seed)
    estimator=CausalDifference(); observer_k=np.array([700.,780.,740.,820.,760.,800.])*sc.observer_stiffness_scale
    observer_b=np.array([4.,4.5,4.2,4.8,4.1,4.6])
    delay=deque(); prev_obs=None; held=np.zeros(6); rows=[]; fault=''; saturations=0; limit_hits=0
    total=int(round(sc.duration/sc.dt)); sub=max(1,int(math.ceil(sc.dt/sc.max_substep))); h=sc.dt/sub
    for k in range(total+1):
        t=k*sc.dt; now_ns=1_000_000_000+int(round(t*1e9))
        _,w,acc=p.state(t,sc.disturbance_scale)
        # Noise is measurement error, not plant state. acceleration is an idealized
        # direct sensor; prior w is an oracle, NOT ordinarily available on 6PUS.
        obs={'time':t,'ns':now_ns,'eta':p.eta.copy()+rng.normal(0,sc.noise_angle,3),'vel':p.vel.copy()+rng.normal(0,sc.noise_vel,3),
             'acc':acc+rng.normal(0,sc.noise_acc,3),'q':.05+p.q.copy()+rng.normal(0,sc.noise_position,6),
             'qvel':p.qvel.copy()+rng.normal(0,sc.noise_joint_vel,6),
             'w':w.copy(),'true_acc':acc.copy(),'previous_cmd':held.copy()}
        if sc.quantize: obs['q']=np.round(obs['q']/.000025)*.000025
        delay.append(obs)
        if len(delay)<=sc.delay_cycles:
            for z in range(sub): p.step(held,t+z*h,h,sc.disturbance_scale)
            continue
        obs=delay.popleft(); eta_d,vel_d,acc_d,qd,qvd,qad=trajectory(obs['time'],j)
        if kind=='DS_SCTDE_MODEL_ESTIMATE':
            estimates=estimator.update(obs['time'],obs['eta'],obs['q'])
            if estimates is None:
                if k<total:
                    for z in range(sub): p.step(held,t+z*h,h,sc.disturbance_scale)
                continue
            obs['vel'],obs['qvel'],obs['acc']=estimates
            obs['w_model']=modeled_wrench(obs['eta'],obs['vel'],obs['q'],obs['qvel'],observer_k,observer_b,sc.map_scale)
        estimated=None
        try:
            if kind=='PID_BASELINE':
                out=ctrl.update(qd,obs['q'],[obs['ns']]*6,now_ns,[0]*6)
                primed=ctrl.state['primed']; raw=np.array(out)
                saturated=not primed and not np.allclose(raw, np.array(ctrl.state['p_nm'])+np.array(ctrl.state['i_nm'])+np.array(ctrl.state['d_nm']),rtol=0,atol=1e-12)
            else:
                if prev_obs is None: prev_obs=obs
                prev_w=prev_obs['w'] if kind=='DS_SCTDE_ORACLE' else prev_obs['w_model']
                source='simulation_oracle' if kind=='DS_SCTDE_ORACLE' else 'model_estimate'
                _,j_control=nonlinear_geometry(obs['eta']); j_control=j_control*sc.map_scale
                s={'timestamp_ns':obs['ns'],'fault_codes':[0]*6,'eta_rad':obs['eta'], 'eta_d_rad':eta_d,
                   'eta_dot_rad_s':obs['vel'],'eta_dot_d_rad_s':vel_d,'eta_ddot_rad_s2':obs['acc'],
                   'eta_ddot_d_rad_s2':acc_d,'q_m':obs['q'],'q_d_m':qd,'q_dot_m_s':obs['qvel'],
                   'q_dot_d_m_s':qvd,'q_ddot_d_m_s2':qad,'friction_motor_nm':[0.]*6,
                   'jacobian_m_per_rad':j_control,'applied_wrench_timestamp_ns':prev_obs['ns'],
                   'applied_task_wrench_prev_nm':prev_w,'applied_wrench_source':source}
                out=ctrl.update(s,now_ns); primed=out is None
                raw=np.zeros(6) if primed else np.asarray(out)
                saturated=False if primed else ctrl.state['saturated']
                if not primed: estimated=np.asarray(ctrl.state['tde_wrench_nm'])
                prev_obs=obs
            held=raw
        except (Fault,ControllerFault,ValueError,OverflowError) as exc:
            fault=str(exc); held=np.zeros(6); break
        saturations+=int(saturated)
        actual_ref=trajectory(t,j)
        mbar=np.diag([.020,.024,.016])
        true_h=w-mbar@acc
        observed_true_h=obs['w']-mbar@obs['true_acc']
        er=actual_ref[0]-p.eta; eq=actual_ref[3]-(.05+p.q); sync=eq-np.roll(eq,-1)
        row={'time_s':t,'observed_time_s':obs['time'],'saturated':int(saturated),
             'eta_error_rad_rms':float(np.sqrt(np.mean(er**2))), 'q_error_m_rms':float(np.sqrt(np.mean(eq**2))),
             'sync_error_m_rms':float(np.sqrt(np.mean(sync**2))),
             'tde_error_nm_rms':float(np.sqrt(np.mean((estimated-true_h)**2))) if estimated is not None else '',
             'wrench_est_error_nm_rms':float(np.sqrt(np.mean((obs['w_model']-obs['w'])**2))) if kind=='DS_SCTDE_MODEL_ESTIMATE' else '',
             'tde_error_observed_time_nm_rms':float(np.sqrt(np.mean((estimated-observed_true_h)**2))) if estimated is not None else ''}
        for i in range(3):
            row[f'eta_error_{i+1}_rad']=er[i]
            row[f'eta_target_{i+1}_rad']=actual_ref[0][i]
            row[f'eta_actual_{i+1}_rad']=p.eta[i]
            row[f'disturbance_{i+1}_nm']=disturbance(t,sc.disturbance_scale)[i]
            row[f'tde_estimate_{i+1}_nm']=estimated[i] if estimated is not None else ''
            row[f'true_lumped_{i+1}_nm']=true_h[i]
            row[f'true_lumped_observed_{i+1}_nm']=observed_true_h[i]
            row[f'observed_task_wrench_{i+1}_nm']=obs['w'][i]
            row[f'estimated_task_wrench_{i+1}_nm']=obs['w_model'][i] if kind=='DS_SCTDE_MODEL_ESTIMATE' else ''
            row[f'current_task_wrench_{i+1}_nm']=w[i]
        for i in range(6):
            row[f'q_error_{i+1}_m']=eq[i]; row[f'tau_cmd_{i+1}_nm']=held[i]; row[f'tau_applied_{i+1}_nm']=p.motor[i]
            row[f'q_target_{i+1}_m']=actual_ref[3][i]; row[f'q_actual_{i+1}_m']=.05+p.q[i]
            row[f'sync_error_{i+1}_m']=sync[i]
        rows.append(row)
        if np.max(np.abs(p.q))>.01 or not all(np.isfinite(x).all() for x in (p.eta,p.q,p.vel,p.qvel)):
            limit_hits+=1; fault='simulated stroke or numeric limit'; break
        if k<total:
            for z in range(sub): p.step(held,t+z*h,h,sc.disturbance_scale)
    def rms(key):
        vals=[float(r[key]) for r in rows if r[key]!='']
        return float(np.sqrt(np.mean(np.square(vals)))) if vals else None
    return rows, {'scenario':sc.name,'algorithm':kind,'samples':len(rows),'duration_completed_s':rows[-1]['time_s'] if rows else 0.,
        'eta_rms_rad':rms('eta_error_rad_rms'),'q_rms_m':rms('q_error_m_rms'),'sync_rms_m':rms('sync_error_m_rms'),
        'tde_rms_nm':rms('tde_error_nm_rms'), 'wrench_est_rms_nm':rms('wrench_est_error_nm_rms'),
        'peak_torque_nm':max((abs(r[f'tau_cmd_{i}_nm']) for r in rows for i in range(1,7)),default=0.),
        'saturated_samples':saturations,'stroke_limit_events':limit_hits,'fault':fault,'hardware_write_count':0}


def main():
    parser=argparse.ArgumentParser(); parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists() and any(args.output.iterdir()):
        raise SystemExit('output directory must be empty; choose a new path')
    args.output.mkdir(parents=True,exist_ok=True)
    summaries=[]
    for sc in CASES:
        for kind in ('PID_BASELINE','DS_SCTDE_ORACLE','DS_SCTDE_MODEL_ESTIMATE'):
            rows,summary=simulate(sc,kind); summaries.append(summary)
            path=args.output/f'{sc.name}_{kind}.csv'
            if rows:
                with path.open('w',newline='') as f:
                    writer=csv.DictWriter(f,fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
            print(sc.name,kind,summary,flush=True)
    with (args.output/'summary.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(summaries[0])); writer.writeheader(); writer.writerows(summaries)
    j=geometry(); plant=Plant(j)
    (args.output/'conditions.json').write_text(json.dumps({'scenarios':[vars(x) for x in CASES],
        'plant':'nominal configured nonlinear IK with synthetic compliant dynamics; no hardware validation',
        'jacobian_m_per_rad':j.tolist(),
        'plant_parameters':{name:getattr(plant,name).tolist() for name in
            ('mass','axis_mass','spring','damp','axis_drag','rot_drag','motor_tau','n')},
        'controller_parameters':{'ds_nominal_inertia':[.020,.024,.016],'ds_kp':[20.]*3,'ds_kd':[8.]*3,
            'ds_cross':[2.]*6,'ds_p':[3.]*6,'ds_kc':[.15]*6,'ds_k':[.04]*6,'ds_motor_inertia':[.00020]*6,
            'pid_kp':[1.5]*6,'pid_ki':[0.]*6,'pid_kd':[.005]*6,'pid_derivative_filter_s':.01,
            'motor_slew_nm_s':.2},
        'integrator':'semi-implicit Euler, scenario.max_substep seconds', 'hardware_write_count':0},indent=2))


if __name__=='__main__': main()
