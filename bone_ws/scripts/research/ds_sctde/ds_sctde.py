"""6PUS 3-angle constrained-task DS-SCTDE research model; never drives hardware.

Input angles are local ZYX Euler coordinates, not a globally valid SO(3) error.
Task wrench is conjugate to Euler rates; motor output is N m. See research report.
"""
from dataclasses import dataclass
import numpy as np


def arr(x, shape, name):
    a = np.asarray(x, dtype=float)
    if a.shape != shape or not np.isfinite(a).all():
        raise ValueError(f'{name} must be finite with shape {shape}')
    return a


@dataclass(frozen=True)
class Config:
    mass: object             # 3x3 nominal Euler-coordinate inertia, kg m^2
    kp: object               # 3, 1/s^2
    kd: object               # 3, 1/s
    cross: object            # 6, 1/s
    p: object                # 6, 1/s
    kc: object               # 6, N m / m
    k: object                # 6, N m s / m
    motor_inertia: object    # 6, N m s^2 / m, motor-coordinate reflected coefficient
    transmission: object     # 6, signed dx/dtheta, m/rad (ideal power mapping)
    limit: object            # 6, N m
    slew: object             # 6, N m/s
    integral_limit: object   # 6, m s, numerical bound for integral(c)
    min_dt: float
    max_dt: float
    max_age: float
    max_condition: float

    def __post_init__(self):
        m = arr(self.mass, (3, 3), 'mass')
        if not np.allclose(m, m.T) or np.min(np.linalg.eigvalsh(m)) <= 0:
            raise ValueError('mass must be symmetric positive definite')
        object.__setattr__(self, 'mass', m.copy())
        for key, n in [('kp',3), ('kd',3), ('cross',6), ('p',6), ('kc',6), ('k',6), ('motor_inertia',6), ('transmission',6), ('limit',6), ('slew',6), ('integral_limit',6)]:
            a = arr(getattr(self, key), (n,), key)
            if key == 'transmission':
                if np.any(a == 0): raise ValueError('transmission must be nonzero')
            elif key == 'motor_inertia':
                if np.any(a < 0): raise ValueError('motor inertia cannot be negative')
            elif np.any(a <= 0):
                raise ValueError(f'{key} must be positive')
            object.__setattr__(self, key, a.copy())
        if not all(np.isfinite(x) for x in (self.min_dt, self.max_dt, self.max_age, self.max_condition)) or not (0 < self.min_dt <= self.max_dt and self.max_age > 0 and self.max_condition > 1):
            raise ValueError('invalid timing or condition limits')


class ControllerFault(RuntimeError):
    pass


class DSSCTDE:
    def __init__(self, cfg):
        self.cfg = cfg
        self.reset()

    def reset(self):
        self.fault = None
        self.previous_ns = None
        self.previous_acceleration = None
        self.integral_c = np.zeros(6)
        self.output = np.zeros(6)
        self.state = None

    def trip(self, reason):
        self.fault = str(reason)
        self.output = np.zeros(6)  # computational state only; not a hardware stop
        raise ControllerFault(self.fault)

    def update(self, s, now_ns):
        if self.fault: raise ControllerFault(self.fault)
        try:
            return self._update(s, now_ns)
        except (KeyError, ValueError, TypeError, OverflowError, np.linalg.LinAlgError) as exc:
            self.trip(exc)

    def _update(self, s, now_ns):
        cfg = self.cfg
        t = s['timestamp_ns']
        if not isinstance(t, int) or not isinstance(now_ns, int) or t <= 0 or t > now_ns or (now_ns-t)*1e-9 > cfg.max_age:
            raise ValueError('invalid or stale timestamp')
        if any(s['fault_codes']) or len(s['fault_codes']) != 6:
            raise ValueError('motor feedback fault')
        eta = arr(s['eta_rad'], (3,), 'eta_rad')
        eta_d = arr(s['eta_d_rad'], (3,), 'eta_d_rad')
        eta_dot = arr(s['eta_dot_rad_s'], (3,), 'eta_dot_rad_s')
        eta_dot_d = arr(s['eta_dot_d_rad_s'], (3,), 'eta_dot_d_rad_s')
        eta_ddot = arr(s['eta_ddot_rad_s2'], (3,), 'eta_ddot_rad_s2')
        eta_ddot_d = arr(s['eta_ddot_d_rad_s2'], (3,), 'eta_ddot_d_rad_s2')
        q = arr(s['q_m'], (6,), 'q_m'); qd = arr(s['q_d_m'], (6,), 'q_d_m')
        qdot = arr(s['q_dot_m_s'], (6,), 'q_dot_m_s'); qdotd = arr(s['q_dot_d_m_s'], (6,), 'q_dot_d_m_s')
        qddotd = arr(s['q_ddot_d_m_s2'], (6,), 'q_ddot_d_m_s2')
        friction = arr(s['friction_motor_nm'], (6,), 'friction_motor_nm')
        j = arr(s['jacobian_m_per_rad'], (6,3), 'jacobian_m_per_rad')
        sv = np.linalg.svd(j, compute_uv=False)
        if sv[-1] <= 1e-12 or sv[0]/sv[-1] > cfg.max_condition:
            raise ValueError('task Jacobian rank or conditioning')
        if self.previous_ns is None:
            self.previous_ns, self.previous_acceleration = t, eta_ddot.copy()
            self.state = {'primed': True, 'tau_nm': [0.]*6}
            return None
        dt = (t-self.previous_ns)*1e-9
        if not cfg.min_dt <= dt <= cfg.max_dt: raise ValueError('invalid sample interval')
        if s['applied_wrench_timestamp_ns'] != self.previous_ns:
            raise ValueError('applied wrench must correspond to previous sample')
        applied = arr(s['applied_task_wrench_prev_nm'], (3,), 'applied_task_wrench_prev_nm')
        h = applied - cfg.mass @ self.previous_acceleration
        task = cfg.mass @ (eta_ddot_d + cfg.kp*(eta_d-eta) + cfg.kd*(eta_dot_d-eta_dot)) + h
        force = np.linalg.pinv(j.T) @ task
        e = qd-q; edot = qdotd-qdot
        # Six-axis cyclic extension of paper eqs. (21)-(24): e_s[i]=e[i]-e[i+1].
        es = e-np.roll(e,-1); c = es-np.roll(es,1)
        es_dot = edot-np.roll(edot,-1); c_dot = es_dot-np.roll(es_dot,1)
        integ = np.clip(self.integral_c + c*dt, -cfg.integral_limit, cfg.integral_limit)
        def evaluate(integral):
            ec = e + cfg.cross*integral
            ec_dot = edot + cfg.cross*c
            sigma = ec_dot + cfg.p*ec
            qddot_r = qddotd + cfg.cross*c_dot + cfg.p*ec_dot
            # Coefficients in motor-torque units; direction follows signed dx/dtheta.
            sync = np.sign(cfg.transmission)*(cfg.motor_inertia*qddot_r + cfg.kc*c + cfg.k*sigma) + friction
            return ec, sigma, qddot_r, sync, sync + cfg.transmission*force
        ec, sigma, qddot_r, sync, raw = evaluate(integ)
        lo = np.maximum(-cfg.limit, self.output-cfg.slew*dt)
        hi = np.minimum(cfg.limit, self.output+cfg.slew*dt)
        # Freeze all cross-coupling integrators during saturation; no hardware claim.
        if np.any((raw < lo) | (raw > hi)):
            integ = self.integral_c.copy()
            ec, sigma, qddot_r, sync, raw = evaluate(integ)
        out = np.clip(raw, lo, hi)
        if not all(np.isfinite(x).all() for x in (h, task, force, ec, sigma, qddot_r, sync, raw, out)):
            raise ValueError('nonfinite computed state')
        self.integral_c, self.output = integ, out
        self.previous_ns, self.previous_acceleration = t, eta_ddot.copy()
        self.state = {'primed': False, 'e_q_m': e.tolist(), 'e_task_rad': (eta_d-eta).tolist(),
            'synchronization_error_m': es.tolist(), 'cross_deviation_m': c.tolist(),
            'ec_m': ec.tolist(), 'sigma_m_s': sigma.tolist(), 'reference_acceleration_m_s2': qddot_r.tolist(),
            'tde_wrench_nm': h.tolist(), 'task_wrench_nm': task.tolist(), 'actuator_force_n': force.tolist(),
            'sync_motor_nm': sync.tolist(), 'tau_raw_nm': raw.tolist(), 'tau_nm': out.tolist(),
            'saturated': bool(np.any(out != raw)), 'dt_s': dt, 'algorithm': 'DS_SCTDE'}
        return tuple(out)
