"""Position dual-IMU attitude PID -> FK pose PID -> Mode 3 wrench increments.

The retained Position controller produces an angular residual, not Nm. Its
SO(3) composed reference is tracked using actual FK and a simple SI-output PID.
No weighted IMU/FK fusion, virtual mass, or impedance model is used.
"""
from collections import deque
from dataclasses import dataclass
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from .position_attitude_math import relative_quat, apply_z90_transform, quat_to_rot_matrix, compute_so3_platform_error, compose_so3_pid_residual
from .rate_limiter import PhysicalTimeRateLimiter
from .position_attitude_pid import M2PidGains, M2PidLimits, M2TaskSpaceAttitudePid

C_IMU_TO_MECHANICAL = np.array([[0.,-1.,0.],[1.,0.,0.],[0.,0.,1.]])


def finite_vector(value, size):
    value=np.asarray(value,dtype=float)
    if value.shape!=(size,) or not np.isfinite(value).all():
        raise ValueError(f'{size} finite components required')
    return value


def valid_quaternion(value):
    q=finite_vector(value,4)
    if np.linalg.norm(q)<1e-9:raise ValueError('invalid zero quaternion')
    return q/np.linalg.norm(q)


class ImuHistory:
    """Small host-receipt history for sampling at the existing joint lag time.

    Does not claim sensor hardware clock synchronization or extrapolate samples.
    """
    def __init__(self):
        self.samples=deque(maxlen=200)  # orientation receipt times
        self.gyros=deque(maxlen=200)   # gyro receipt times, never refreshed by q

    @property
    def latest_complete_ns(self):
        return min(self.samples[-1][0],self.gyros[-1][0]) if self.samples and self.gyros else 0

    def push(self, stamp, quaternion=None, gyro=None):
        # H30 may publish partial Imu fields (covariance[0] == -1). Retain
        # independent receipt times rather than stamping cached gyro as new.
        q=valid_quaternion(quaternion) if quaternion is not None else None
        g=finite_vector(gyro,3) if gyro is not None else None
        if q is not None:
            if self.samples and stamp<=self.samples[-1][0]:raise ValueError('IMU receive time rollback')
            self.samples.append((stamp,q))
        if g is not None:
            if self.gyros and stamp<=self.gyros[-1][0]:raise ValueError('gyro receive time rollback')
            self.gyros.append((stamp,g))

    def query(self, stamp, max_gap_ns):
        def bracket(history):
            for left,right in zip(history,list(history)[1:]):
                if left[0]<=stamp<=right[0]:
                    if right[0]-left[0]>max_gap_ns:raise ValueError('IMU interpolation gap')
                    return left,right,(stamp-left[0])/(right[0]-left[0])
            raise ValueError('IMU does not bracket synchronized joint timestamp')
        left,right,alpha=bracket(self.samples)
        q=Slerp([0.,1.],Rotation.from_quat([left[1],right[1]]))([alpha])[0].as_quat()
        left,right,alpha=bracket(self.gyros)
        return q,left[1]+alpha*(right[1]-left[1])


@dataclass(frozen=True)
class StabilizationConfig:
    # Position final retained Classical P/PD: angular residual, rad/rad and s.
    attitude_kp: tuple=(.05,.05,.025)
    attitude_ki: tuple=(0.,0.,0.)
    attitude_kd: tuple=(0.,.0015,.0015)
    attitude_residual_limit_rad: tuple=tuple(np.radians([.5,.5,.5]))
    attitude_integral_limit_rad_s: tuple=tuple(np.radians([2.,2.,4.]))
    attitude_min_dt_s: float=.001
    attitude_max_dt_s: float=.1
    attitude_derivative_enabled: bool=True
    attitude_derivative_lpf_cutoff_hz: float=5.
    # New actuator-interface PID: [N/m]*3, [Nm/rad]*3 (P); I=P/s, D=P*s.
    # Software example values, NOT force-actuated hardware tuning.
    pose_kp: tuple=(10.,10.,10.,.2,.2,.2)
    pose_ki: tuple=(0.,)*6
    pose_kd: tuple=(0.,)*6
    pose_integral_limit_n_nm: tuple=(1.,1.,1.,.1,.1,.1)
    pose_derivative_filter_tau_s: float=.05
    force_limit_n: tuple=(1.,1.,1.)
    moment_limit_nm: tuple=(.1,.1,.1)
    wrench_rate: tuple=(1.,1.,1.,.1,.1,.1)  # N/s then Nm/s

    def __post_init__(self):
        for name,value in vars(self).items():
            if isinstance(value,(tuple,list,np.ndarray)):
                size=6 if name.startswith('pose_') or name=='wrench_rate' else 3
                values=finite_vector(value,size)
                positive=('limit_rad' in name or name in ('wrench_rate','force_limit_n','moment_limit_nm'))
                if (values<0).any() or (positive and (values<=0).any()):
                    raise ValueError(f'invalid {name}')
                object.__setattr__(self,name,tuple(values))
            elif name!='attitude_derivative_enabled':
                if not np.isfinite(value) or value<0:raise ValueError(f'invalid {name}')
        if self.attitude_min_dt_s<=0 or self.attitude_max_dt_s<self.attitude_min_dt_s:
            raise ValueError('invalid attitude PID dt bounds')


class PoseWrenchPID:
    """Pose error [m,rad] -> incremental wrench [N,Nm], no reference-force FF.

    Conditional integration handles task clipping; downstream saturation/slew
    feedback can also undo this sample's integration. Derivative is filtered.
    """
    def __init__(self,config):
        self.config=config
        self.reset()

    def reset(self):
        self.integral=np.zeros(6)
        self.previous_error=None
        self.derivative=np.zeros(6)
        self.previous_integral=self.integral.copy()

    def update(self,error,dt):
        error=finite_vector(error,6)
        if not np.isfinite(dt) or dt<=0:raise ValueError('invalid pose PID interval')
        c=self.config
        self.previous_integral=self.integral.copy()
        raw_d=np.zeros(6) if self.previous_error is None else (error-self.previous_error)/dt
        alpha=dt/(c.pose_derivative_filter_tau_s+dt)
        self.derivative+=alpha*(raw_d-self.derivative)
        p=np.array(c.pose_kp)*error
        d=np.array(c.pose_kd)*self.derivative
        i_cap=np.array(c.pose_integral_limit_n_nm)
        candidate=np.clip(self.integral+np.array(c.pose_ki)*error*dt,-i_cap,i_cap)
        caps=np.r_[c.force_limit_n,c.moment_limit_nm]
        candidate_output=p+candidate+d
        pushing=((candidate_output>caps)&(candidate>self.integral))|((candidate_output<-caps)&(candidate<self.integral))
        self.integral=np.where(pushing,self.integral,candidate)
        self.previous_error=error.copy()
        requested=p+self.integral+d
        return requested,np.clip(requested,-caps,caps)

    def track_applied(self,requested,applied):
        # All vectors are task-space SI quantities, never raw motor g.
        blocked=(np.asarray(requested)-np.asarray(applied))*self.previous_error>1e-12
        self.integral=np.where(blocked,self.previous_integral,self.integral)


class ActiveStabilizer:
    def __init__(self,config=None,*,clock):
        self.config=config or StabilizationConfig()
        self.clock=clock
        self.reference=None
        c=self.config
        self.attitude_pid=M2TaskSpaceAttitudePid(
            M2PidGains(np.array(c.attitude_kp),np.array(c.attitude_ki),np.array(c.attitude_kd)),
            M2PidLimits(np.array(c.attitude_residual_limit_rad),np.array(c.attitude_integral_limit_rad_s),
                c.attitude_min_dt_s,c.attitude_max_dt_s,c.attitude_derivative_enabled,c.attitude_derivative_lpf_cutoff_hz))
        self.pose_pid=PoseWrenchPID(c)
        self.limiter=PhysicalTimeRateLimiter(c.wrench_rate,[0.]*6,clock=clock)

    def reset(self):
        self.reference=None
        self.attitude_pid.reset()
        self.pose_pid.reset()

    def capture(self,pose,base_q,platform_q,stamp_ns):
        pose=finite_vector(pose,6)
        base_q=valid_quaternion(base_q)
        platform_q=valid_quaternion(platform_q)
        self.reference=(pose.copy(),base_q.copy(),platform_q.copy())
        self.last_sample_ns=stamp_ns
        self.attitude_pid.reset()
        self.pose_pid.reset()
        self.limiter.reset([0.]*6,now_ns=self.clock())

    def target_pose(self,pose,base_q,platform_q,base_gyro,platform_gyro,*,sample_ns):
        if self.reference is None:raise ValueError('attitude reference not captured')
        pose=finite_vector(pose,6)
        base_q=valid_quaternion(base_q)
        platform_q=valid_quaternion(platform_q)
        finite_vector(base_gyro,3)
        finite_vector(platform_gyro,3)
        dt=(sample_ns-self.last_sample_ns)*1e-9
        if dt<=0:raise ValueError('nonpositive synchronized sample interval')
        p0,qb0,qp0=self.reference
        r0=Rotation.from_euler('xyz',p0[3:]).as_matrix()
        rb=quat_to_rot_matrix(apply_z90_transform(relative_quat(qb0,base_q)))
        r_rel_p=quat_to_rot_matrix(apply_z90_transform(relative_quat(qp0,platform_q)))
        rp=r0@r_rel_p
        rfk=Rotation.from_euler('xyz',pose[3:]).as_matrix()
        rff=rb.T@r0
        _,e_world=compute_so3_platform_error(r0,rp)
        self.previous_attitude_integral=self.attitude_pid.integral.copy()
        residual_world=self.attitude_pid.update(e_world,dt)
        # Exact Position M2 SO(3) composition, generalized to captured R0.
        _,_,r_command,_=compose_so3_pid_residual(rb,rff,residual_world)
        target=np.r_[p0[:3],Rotation.from_matrix(r_command).as_euler('xyz')]
        self.last_sample_ns=sample_ns
        diagnostics=dict(reference_rotation_base=rff.tolist(),command_rotation_base=r_command.tolist(),
            base_rel_rpy_deg=Rotation.from_matrix(rb).as_euler('xyz',degrees=True).tolist(),
            platform_rel_rpy_deg=Rotation.from_matrix(r_rel_p).as_euler('xyz',degrees=True).tolist(),
            platform_error_world_rad=e_world.tolist(),attitude_residual_world_rad=residual_world.tolist(),
            attitude_pid_p_rad=self.attitude_pid.last_p_term.tolist(),
            attitude_pid_i_rad=self.attitude_pid.last_i_term.tolist(),attitude_pid_d_rad=self.attitude_pid.last_d_term.tolist(),
            sample_dt_s=dt,desired_pose_m_rad=target.tolist())
        return target,diagnostics

    def update(self,pose,base_q,platform_q,base_gyro,platform_gyro,*,sample_ns,now_ns):
        target,diagnostics=self.target_pose(pose,base_q,platform_q,base_gyro,platform_gyro,sample_ns=sample_ns)
        dt=diagnostics['sample_dt_s'];p0=self.reference[0]
        rff=np.array(diagnostics['reference_rotation_base']);r_command=np.array(diagnostics['command_rotation_base'])
        e_world=np.array(diagnostics['platform_error_world_rad']);residual_world=np.array(diagnostics['attitude_residual_world_rad'])
        rfk=Rotation.from_euler('xyz',pose[3:]).as_matrix()
        _,e_fk=compute_so3_platform_error(r_command,rfk)
        error=np.r_[p0[:3]-pose[:3],e_fk]
        requested,bounded=self.pose_pid.update(error,dt)
        command=self.limiter.update(bounded,now_ns=now_ns)
        self.last_requested=requested.copy()
        self.pose_pid.track_applied(requested,command)
        if not np.allclose(requested,command,atol=1e-12,rtol=0):
            self.attitude_pid.integral=self.previous_attitude_integral.copy()
        self.last_sample_ns=sample_ns
        return command,dict(reference_rotation_base=rff.tolist(),command_rotation_base=r_command.tolist(),
            base_rel_rpy_deg=diagnostics['base_rel_rpy_deg'],platform_rel_rpy_deg=diagnostics['platform_rel_rpy_deg'],
            platform_error_world_rad=e_world.tolist(),attitude_residual_world_rad=residual_world.tolist(),
            attitude_pid_p_rad=self.attitude_pid.last_p_term.tolist(),
            attitude_pid_i_rad=self.attitude_pid.last_i_term.tolist(),
            attitude_pid_d_rad=self.attitude_pid.last_d_term.tolist(),
            pose_error_m_rad=error.tolist(),pose_pid_integral_n_nm=self.pose_pid.integral.tolist(),
            requested_wrench_n_nm=requested.tolist(),commanded_wrench_n_nm=command.tolist(),
            task_limited=not np.allclose(requested,command,atol=1e-12,rtol=0),sample_dt_s=dt)

    def track_actuator_output(self,applied_wrench_n_nm,limited):
        """Observe existing N/g drive caps/slew, not measured-vs-command lag.

        Sensor response error is diagnostic; it must not masquerade as saturation.
        """
        if limited and self.pose_pid.previous_error is not None:
            self.pose_pid.track_applied(self.last_requested,finite_vector(applied_wrench_n_nm,6))
            self.attitude_pid.integral=self.previous_attitude_integral.copy()
