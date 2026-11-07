"""Six measured slider positions -> six signed incremental native forces.

SI throughout: m, m/s, N. No UART, Tare, mode change or friction jump command.
Optional experimental caps are distinct from the final actuator output boundary.
"""
from dataclasses import dataclass
from collections import deque
import numpy as np
from .active_stabilization import finite_vector
from .rate_limiter import PositionReferenceRateLimiter
from pus6_motor_driver.force_units import NOMINAL_N_PER_G


@dataclass(frozen=True)
class JointForcePIDConfig:
    joint_kp_n_m: tuple=(20000.,)*6
    joint_ki_n_m_s: tuple=(10000.,)*6
    joint_kd_n_s_m: tuple=(1000.,)*6
    joint_integral_limit_n: tuple | None=(1.,)*6
    joint_increment_limit_n: tuple | None=(2.,)*6
    joint_min_position_m: tuple=(.03,)*6  # normal raw 200, not storage/physical zero
    joint_max_position_m: tuple=(.07,)*6  # normal raw 1800; intersect device limits
    joint_max_reference_speed_m_s: float | None=.017  # None: direct latest IK reference
    joint_derivative_filter_tau_s: float=.04
    joint_position_deadband_m: float=.0000125  # half encoder step
    joint_use_latest_feedback: bool=False
    joint_active_braking: bool=False
    joint_continuous_velocity: bool=False  # opt-in for dual IMU; legacy motion paths unchanged
    joint_integrate_while_moving: bool=False
    joint_integral_from_measured_error: bool=False
    joint_velocity_window_s: float=.08
    joint_velocity_noise_span_m: float=.0000375
    joint_prediction_horizon_s: float=.02
    joint_brake_horizon_cap_s: float=.10
    joint_brake_rate_g_s: tuple=(5000.,)*6
    joint_max_tracking_error_m: float | None=.00015  # None: report error without a tracking-only exit

    def __post_init__(self):
        for name,value in vars(self).items():
            if name in ('joint_max_reference_speed_m_s','joint_integral_limit_n',
                        'joint_increment_limit_n','joint_max_tracking_error_m') and value is None:
                continue
            if isinstance(value,(tuple,list,np.ndarray)):
                v=finite_vector(value,6)
                if name not in ('joint_min_position_m','joint_max_position_m') and (v<0).any():raise ValueError(name)
                object.__setattr__(self,name,tuple(v))
            elif not np.isfinite(value) or value<0:raise ValueError(name)
        if any(a>=b for a,b in zip(self.joint_min_position_m,self.joint_max_position_m)):raise ValueError('position bounds')
        if ((self.joint_max_reference_speed_m_s is not None and self.joint_max_reference_speed_m_s<=0)
                or (self.joint_max_tracking_error_m is not None and self.joint_max_tracking_error_m<=0)):
            raise ValueError('positive speed/tracking bound required')
        if self.joint_increment_limit_n is not None and min(self.joint_increment_limit_n)<=0:
            raise ValueError('positive increment limits required')
        if self.joint_velocity_window_s<=0 or min(self.joint_brake_rate_g_s)<=0:raise ValueError('positive estimation window/braking rates required')


class JointPositionForcePID:
    def __init__(self,config=None,*,clock,force_slew_compensation=True):
        self.config=config or JointForcePIDConfig();self.clock=clock
        self.force_slew_compensation=bool(force_slew_compensation)
        self.reference=(None if self.config.joint_max_reference_speed_m_s is None else
            PositionReferenceRateLimiter([.05]*6,self.config.joint_max_reference_speed_m_s,clock=clock))
        self.initialized=False

    def reset(self,measured_m,*,sample_ns,now_ns,position_stamps_ns=None):
        q=self.validate_position(measured_m)
        if self.reference is not None:self.reference.reset(q,now_ns=now_ns)
        self.previous_q=q.copy();self.previous_sample_ns=sample_ns
        self.integral=np.zeros(6);self.previous_integral=self.integral.copy();self.velocity=np.zeros(6)
        stamps=[sample_ns]*6 if position_stamps_ns is None else list(position_stamps_ns)
        self.position_history=[deque([(t,float(x))]) for t,x in zip(stamps,q)]
        self.position_history_previous_ns=list(stamps)
        self.last_applied=np.zeros(6);self.braking_axes=[False]*6
        self.last_error=np.zeros(6);self.last_requested=np.zeros(6);self.initialized=True

    def validate_position(self,q):
        q=finite_vector(q,6);c=self.config
        if (q<np.array(c.joint_min_position_m)-1e-12).any() or (q>np.array(c.joint_max_position_m)+1e-12).any():
            raise ValueError('slider target/feedback outside mechanical normal bounds')
        return q

    def update(self,target_m,measured_m,*,sample_ns,now_ns,position_stamps_ns=None):
        if not self.initialized:raise ValueError('joint PID not initialized')
        target=self.validate_position(target_m);q=self.validate_position(measured_m);c=self.config
        dt=(sample_ns-self.previous_sample_ns)*1e-9
        if dt<=0:raise ValueError('joint PID requires new synchronized feedback')
        ref=target.copy() if self.reference is None else self.reference.update(target,now_ns=now_ns)
        e=ref-q
        if c.joint_max_tracking_error_m is not None and (abs(e)>c.joint_max_tracking_error_m+1e-12).any():
            raise ValueError('joint reference tracking error exceeds configured bound')
        measured_effective=np.where(abs(e)<=c.joint_position_deadband_m+1e-12,0.,e)
        ages=np.zeros(6);raw_velocity=self.velocity.copy();filter_alpha=np.ones(6)
        if c.joint_use_latest_feedback:
            stamps=[sample_ns]*6 if position_stamps_ns is None else list(position_stamps_ns)
            if len(stamps)!=6:raise ValueError('six position source times required')
            estimates=[]
            for i,(t,x) in enumerate(zip(stamps,q)):
                if not 0<=now_ns-t<=200_000_000:raise ValueError('position source timestamp invalid/stale')
                h=self.position_history[i]
                if t<h[-1][0]:raise ValueError('decreasing axis source time')
                if t==h[-1][0] and abs(x-h[-1][1])>1e-12:raise ValueError('changed position with duplicate source time')
                if t>h[-1][0]:h.append((t,float(x)))
                while len(h)>2 and t-h[0][0]>int(c.joint_velocity_window_s*1e9):h.popleft()
                a=np.array(h,dtype=float);tx=(a[:,0]-a[-1,0])*1e-9;y=a[:,1]
                v=0.
                if len(a)>2 and (c.joint_continuous_velocity or np.ptp(y)>c.joint_velocity_noise_span_m):
                    centered=tx-tx.mean();den=centered@centered
                    if den>0:v=float(centered@(y-y.mean())/den)
                estimates.append(v);ages[i]=(now_ns-t)*1e-9
                if c.joint_continuous_velocity:
                    # A repeated source sample must not advance the filter.
                    axis_dt=(t-self.position_history_previous_ns[i])*1e-9
                    tau=c.joint_derivative_filter_tau_s
                    filter_alpha[i]=(-np.expm1(-axis_dt/tau) if tau>0 else float(axis_dt>0))
                    self.position_history_previous_ns[i]=t
            raw_velocity=np.array(estimates)
            if c.joint_continuous_velocity:
                # Continuous regression including sub-threshold encoder spans,
                # followed by exact first-order filtering at each axis RX time.
                self.velocity+=filter_alpha*(raw_velocity-self.velocity)
            else:
                self.velocity=raw_velocity.copy()
        else:
            measured_velocity=(q-self.previous_q)/dt
            raw_velocity=measured_velocity.copy()
            alpha=dt/(c.joint_derivative_filter_tau_s+dt)
            filter_alpha[:]=alpha
            self.velocity+=alpha*(measured_velocity-self.velocity)
        horizon=ages+c.joint_prediction_horizon_s
        if c.joint_active_braking and self.force_slew_compensation:
            # Account for the time required to remove the actually dispatched
            # increment. This is a slew horizon, not a mass/deceleration model.
            horizon+=np.minimum(abs(self.last_applied)/(np.array(c.joint_brake_rate_g_s)*NOMINAL_N_PER_G),c.joint_brake_horizon_cap_s)
        predicted=q+self.velocity*horizon
        control_error=ref-predicted if c.joint_active_braking else e
        effective=np.where(abs(control_error)<=c.joint_position_deadband_m+1e-12,0.,control_error)
        p=np.array(c.joint_kp_n_m)*effective
        d=-np.array(c.joint_kd_n_s_m)*self.velocity  # no reference-step derivative kick
        self.previous_integral=self.integral.copy()
        cap_i=np.full(6,np.inf) if c.joint_integral_limit_n is None else np.array(c.joint_integral_limit_n)
        # Prediction remains a P/braking input. It must not manufacture a
        # persistent load-trim error inside the measured encoder deadband.
        integral_error=measured_effective if c.joint_integral_from_measured_error else effective
        candidate=np.clip(self.integral+np.array(c.joint_ki_n_m_s)*integral_error*dt,-cap_i,cap_i)
        if c.joint_active_braking and not c.joint_integrate_while_moving:
            # Accumulate stiction/load trim only when stationary. Retain the
            # support already learned while moving; D provides the braking.
            candidate=np.where(abs(self.velocity)>0,self.integral,candidate)
        cap=np.full(6,np.inf) if c.joint_increment_limit_n is None else np.array(c.joint_increment_limit_n)
        u=p+candidate+d;delta_i=candidate-self.integral
        blocked=((u>cap+1e-10)&(delta_i>0))|((u<-cap-1e-10)&(delta_i<0))
        self.integral=np.where(blocked,self.integral,candidate)
        requested=p+self.integral+d;output=np.clip(requested,-cap,cap)
        self.previous_q=q.copy();self.previous_sample_ns=sample_ns
        self.last_error=integral_error;self.last_requested=requested
        # Fast slew only for release/reversal or force opposing measured motion.
        self.braking_axes=(c.joint_active_braking & (
            (output*self.last_applied<0) |
            ((abs(output)<abs(self.last_applied))&(output*self.last_applied>=0)) |
            (output*self.velocity<0))).tolist()
        return output,dict(joint_target_m=target.tolist(),joint_reference_m=ref.tolist(),joint_measured_m=q.tolist(),
            joint_error_m=e.tolist(),joint_velocity_m_s=self.velocity.tolist(),joint_p_n=p.tolist(),
            joint_i_n=self.integral.tolist(),joint_d_n=d.tolist(),joint_requested_delta_force_n=requested.tolist(),
            joint_delta_force_n=output.tolist(),joint_output_limited=bool((abs(requested)>cap).any()),joint_dt_s=dt,
            joint_prediction_m=predicted.tolist(),joint_control_error_m=control_error.tolist(),
            joint_effective_error_m=effective.tolist(),joint_integral_error_m=integral_error.tolist(),
            joint_prediction_horizon_s=horizon.tolist(),
            joint_feedback_age_s=ages.tolist(),joint_braking_axes=self.braking_axes,
            joint_velocity_raw_m_s=raw_velocity.tolist(),joint_velocity_filter_alpha=filter_alpha.tolist(),
            joint_output_limited_axes=(abs(requested)>cap+1e-10).tolist(),
            joint_integral_blocked_axes=blocked.tolist(),
            joint_pd_opposing_axes=(p*d<0).tolist(),
            joint_pd_cancellation_n=np.where(p*d<0,np.minimum(abs(p),abs(d)),0.).tolist())

    def track_applied(self,applied_delta_n):
        self.last_applied=finite_vector(applied_delta_n,6).copy()
        difference=self.last_requested-finite_vector(applied_delta_n,6)
        # Native g quantization is not saturation; tolerate half a nominal g.
        blocked=(difference*self.last_error>0)&(abs(difference)>.005)
        self.integral=np.where(blocked,self.previous_integral,self.integral)
        return blocked.tolist()
