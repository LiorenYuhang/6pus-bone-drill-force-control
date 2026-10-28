"""Research-only dual IMU -> J11 SO(3) PID -> 6PUS IK -> shaped lengths."""
from pathlib import Path
import math
import sys
import numpy as np
from scipy.spatial.transform import Rotation

SCRIPTS=Path(__file__).resolve().parents[3] / 'src' / 'h30_imu' / 'scripts'
sys.path.insert(0,str(SCRIPTS))
from j11_base_feedforward_runner import relative_quat, apply_z90_transform, quat_to_rot_matrix
from j11_m2_pid_residual_runner import (M2TaskSpaceAttitudePid, M2PidGains, M2PidLimits,
    compute_so3_platform_error, compose_so3_pid_residual)
from pus6_kinematics.analytical_6pus import Analytical6PusKinematics
from pus6_control.joint_trajectory_controller import StatefulJerkLimitedShaper


def quat(q):
    q=np.asarray(q,dtype=float)
    if q.shape != (4,) or not np.all(np.isfinite(q)) or np.linalg.norm(q)<1e-8:
        raise ValueError('invalid IMU quaternion')
    return tuple(q/np.linalg.norm(q))


class ImuToLengths:
    def __init__(self,base_reference,platform_reference,kp,ki,kd,residual_limit_rad,
                 integral_limit_rad_s,shaper, max_dt_s=.1):
        self.base0=quat(base_reference); self.platform0=quat(platform_reference)
        arrays=[np.asarray(v,dtype=float) for v in (kp,ki,kd,residual_limit_rad,integral_limit_rad_s)]
        if any(v.shape!=(3,) or not np.all(np.isfinite(v)) or np.any(v<0) for v in arrays):
            raise ValueError('invalid 3-axis PID parameters')
        if not math.isfinite(max_dt_s) or max_dt_s<=.001: raise ValueError('invalid dt')
        self.max_dt=max_dt_s
        self.pid=M2TaskSpaceAttitudePid(M2PidGains(*arrays[:3]), M2PidLimits(
            residual_limit_rad=arrays[3],integral_limit_rad_s=arrays[4],max_dt_s=max_dt_s))
        self.ik=Analytical6PusKinematics()
        self.home=self.ik.inverse_kinematics([0,0,self.ik.nominal_height,0,0,0])
        # Initialize at model HOME; these are software references, not measured positions.
        self.shaper=StatefulJerkLimitedShaper(initial_positions=self.home,**shaper)
        self.previous_ns=None
    def step(self,base_q,platform_q,now_ns):
        base_q=quat(base_q); platform_q=quat(platform_q)
        if self.previous_ns is None:
            self.previous_ns=now_ns; return None
        dt=(now_ns-self.previous_ns)*1e-9
        if not .001<=dt<=self.max_dt: raise ValueError('invalid IMU control interval')
        rb=quat_to_rot_matrix(apply_z90_transform(relative_quat(self.base0,base_q)))
        rp=quat_to_rot_matrix(apply_z90_transform(relative_quat(self.platform0,platform_q)))
        _,error=compute_so3_platform_error(np.eye(3),rp)
        residual=self.pid.update(error,dt)
        _,_,command,_=compose_so3_pid_residual(rb,rb.T,residual)
        rpy=Rotation.from_matrix(command).as_euler('xyz')
        pose=[0.,0.,self.ik.nominal_height,*map(float,rpy)]
        target=self.ik.inverse_kinematics(pose)
        shaped=self.shaper.update(target,dt)
        self.previous_ns=now_ns
        return {'pose_m_rad':pose,'error_world_rad':error.tolist(),'residual_world_rad':residual.tolist(),
                'ik_lengths_m':list(target),'desired_lengths_m':shaped,'dt_s':dt}
