"""Position J14 SO(3) feedforward + active J13 LADRC, angular output only.

Reuse Position quaternion/SO(3) helpers and the unchanged pure LADRC module.
The two IMUs provide relative orientation measurements, not a torque estimate.
"""
from collections import deque
from pathlib import Path
import threading

import numpy as np
from scipy.spatial.transform import Rotation, Slerp
import yaml

from .active_stabilization import finite_vector, valid_quaternion
from .position_attitude_math import (relative_quat, apply_z90_transform,
    quat_to_rot_matrix, compute_so3_platform_error, compose_so3_pid_residual)
from .position_attitude_pid import M2PidGains, M2PidLimits, M2TaskSpaceAttitudePid
from .position_ladrc import ThreeAxisLadrc

AXES = ('roll', 'pitch', 'yaw')


def load_attitude_parameters(path=None):
    if path is None:
        from ament_index_python.packages import get_package_share_directory
        path = Path(get_package_share_directory('pus6_force_control'))/'config/mode3_dual_imu.yaml'
    values = yaml.safe_load(Path(path).read_text())['attitude']
    if not isinstance(values, dict):
        raise ValueError('attitude parameter dictionary required')
    return values


class DualQuaternionBuffer:
    """Align q-only samples at newest common host receipt time, no extrapolation.

    Header time is retained for diagnostics; H30 headers are host ROS stamps,
    not hardware synchronized sensor clocks. Gyro-only messages never renew q.
    """
    def __init__(self, timeout_s=.1):
        self.timeout_ns = int(timeout_s*1e9)
        self.history = {name: deque(maxlen=200) for name in ('base', 'platform')}
        self.lock = threading.Lock()
        self.error = None

    def push(self, name, q, rx_ns, header_ns=None):
        q = valid_quaternion(q)
        with self.lock:
            h = self.history[name]
            if h and rx_ns <= h[-1][0]:
                raise ValueError('IMU orientation receive time rollback')
            h.append((int(rx_ns), q, header_ns))

    def snapshot(self, now_ns):
        with self.lock:
            if self.error:
                raise ValueError(self.error)
            histories = {k: list(v) for k, v in self.history.items()}
        for name, h in histories.items():
            if len(h) < 2:
                raise ValueError(name+' IMU needs orientation history')
            if not 0 <= now_ns-h[-1][0] <= self.timeout_ns:
                raise ValueError(name+' IMU orientation stale')
        stamp = min(h[-1][0] for h in histories.values())
        if now_ns-stamp > self.timeout_ns:
            raise ValueError('common IMU timestamp stale')
        result = dict(sample_ns=stamp, now_ns=int(now_ns), time_basis='host_monotonic_orientation_receipt')
        for name, h in histories.items():
            for left, right in zip(h, h[1:]):
                if left[0] <= stamp <= right[0]:
                    gap = right[0]-left[0]
                    if gap > self.timeout_ns:
                        raise ValueError(name+' IMU interpolation gap')
                    alpha = (stamp-left[0])/gap
                    q = Slerp([0., 1.], Rotation.from_quat([left[1], right[1]]))([alpha])[0].as_quat()
                    result[name+'_q_xyzw'] = q.tolist()
                    result[name+'_timing'] = dict(latest_rx_ns=h[-1][0], latest_header_ns=h[-1][2],
                        bracket_rx_ns=[left[0], right[0]], bracket_header_ns=[left[2], right[2]],
                        interpolation_alpha=alpha, latest_age_s=(now_ns-h[-1][0])*1e-9)
                    break
            else:
                raise ValueError(name+' IMU does not bracket common timestamp')
        return result


class PositionLadrcAttitude:
    """Same active residual choice as Position controller_mode=3axis_active.

    Generalize the HOME identity target to captured FK R0, as Force's retained
    Classical adapter does. A disabled channel holds its captured Euler value.
    Observer prior input is the previous dispatched angular residual command,
    exactly as Position; it is not asserted to be a measured actuator input.
    """
    def __init__(self, parameters, axes=AXES):
        self.parameters = p = dict(parameters)
        if not axes or any(a not in AXES for a in axes) or len(set(axes)) != len(axes):
            raise ValueError('select unique roll/pitch/yaw channels')
        self.axes = tuple(axes)
        self.mask = np.array([a in axes for a in AXES], dtype=float)
        v = lambda prefix: np.array([p[prefix+'_'+a] for a in AXES], dtype=float)
        # Parameter suffixes in the retained YAML carry their original units.
        self.classical = M2TaskSpaceAttitudePid(M2PidGains(v('kp'), v('ki'), v('kd')),
            M2PidLimits(np.deg2rad([p['max_residual_'+a+'_deg'] for a in AXES]),
                np.deg2rad([p['max_integral_'+a+'_deg_s'] for a in AXES]), p['min_dt_s'], p['max_dt_s'],
                p['derivative_enabled'], p['derivative_lpf_cutoff_hz']))
        self.ladrc = ThreeAxisLadrc(**{key: p['ladrc_'+key] for key in
            [prefix+'_'+a for prefix in ('b0', 'omega_c', 'omega_o') for a in AXES]},
            **{'u_limit_'+a+'_deg': p['ladrc_max_residual_'+a+'_deg'] for a in AXES},
            nominal_dt=1./p['control_rate_hz'], min_dt=p['min_dt_s'], max_dt=p['max_dt_s'])
        self.reference = None

    def capture(self, pose, sample):
        self.pose0 = finite_vector(pose, 6).copy()
        self.r0 = Rotation.from_euler('xyz', self.pose0[3:]).as_matrix()
        self.qb0 = valid_quaternion(sample['base_q_xyzw'])
        self.qp0 = valid_quaternion(sample['platform_q_xyzw'])
        self.last_ns = sample['sample_ns']
        self.previous_residual = np.zeros(3)
        self.ladrc.reset(); self.classical.reset()
        self.reference = dict(pose_m_rad=self.pose0.tolist(), sample=sample)
        self.last_target = self.pose0.copy()
        self.diagnostics = dict(status='captured', imu=sample)

    def update(self, sample):
        if self.reference is None:
            raise ValueError('capture dual IMU reference first')
        dt = (sample['sample_ns']-self.last_ns)*1e-9
        if dt < 0:
            raise ValueError('common IMU timestamp rollback')
        if dt < self.parameters['min_dt_s']:
            self.diagnostics = dict(self.diagnostics, imu=sample, observer_updated=False)
            return self.last_target.copy(), self.diagnostics
        if dt > self.parameters['max_dt_s']:
            raise ValueError('dual IMU sample interval exceeds retained 100 ms limit')
        rb_full = quat_to_rot_matrix(apply_z90_transform(relative_quat(self.qb0, sample['base_q_xyzw'])))
        rb = Rotation.from_rotvec(Rotation.from_matrix(rb_full).as_rotvec()*self.mask).as_matrix()
        relp = quat_to_rot_matrix(apply_z90_transform(relative_quat(self.qp0, sample['platform_q_xyzw'])))
        rp = self.r0@relp
        _, error = compute_so3_platform_error(self.r0, rp)
        error *= self.mask
        prior = self.previous_residual.copy()
        residual, raw, z1, z2, saturated = self.ladrc.step(error, prior, dt)
        residual *= self.mask
        classical = self.classical.update(error, dt)
        if not np.isfinite(np.r_[residual, raw, z1, z2]).all() or any(c.last_anomaly_reason for c in self.ladrc.channels):
            raise ValueError('LADRC observer anomaly')
        ff = rb.T@self.r0
        _, _, command, _ = compose_so3_pid_residual(rb, ff, residual)
        target = np.r_[self.pose0[:3], Rotation.from_matrix(command).as_euler('xyz')]
        target[3:] = np.where(self.mask, target[3:], self.pose0[3:])
        self.previous_residual = residual.copy()
        self.last_ns = sample['sample_ns']; self.last_target = target.copy()
        self.diagnostics = dict(status='active', observer_updated=True, imu=sample, dt_s=dt,
            base_rotation_mechanical=rb.tolist(), platform_relative_rotation_mechanical=relp.tolist(),
            relative_base_platform_rotation=(rb_full.T@rp).tolist(), platform_error_world_rad=error.tolist(),
            feedforward_rotation_base=ff.tolist(), command_rotation_base=Rotation.from_euler('xyz', target[3:]).as_matrix().tolist(),
            prior_dispatched_residual_rad=prior.tolist(), ladrc_residual_rad=residual.tolist(), ladrc_raw_rad=raw.tolist(),
            classical_shadow_residual_rad=classical.tolist(), leso_z1_rad=z1.tolist(), leso_z2_rad_s=z2.tolist(),
            ladrc_saturated_axes=saturated, axes=list(self.axes), desired_pose_m_rad=target.tolist())
        return target, self.diagnostics
