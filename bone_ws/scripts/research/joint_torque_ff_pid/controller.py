"""Offline feedforward-summing prototype, not a Simulink reconstruction.

The caller must supply six independently established motor torque feedforward
values (N m) and their timestamps. No torque source or hardware backend exists
here. Do not use for real actuation.
"""
from dataclasses import dataclass
from math import isfinite
from numbers import Integral

N = 6


def vector(value, name):
    if len(value) != N or not all(isfinite(float(x)) for x in value):
        raise ValueError(f'{name} must contain six finite values')
    return tuple(float(x) for x in value)


@dataclass(frozen=True)
class Config:
    kp: tuple  # N m / m
    ki: tuple  # N m / (m s)
    kd: tuple  # N m s / m
    torque_limit: tuple  # N m, symmetric
    slew_limit: tuple  # N m / s
    direction: tuple  # motor torque sign for positive extension error
    min_dt: float
    max_dt: float
    max_age: float
    derivative_tau: float  # seconds; first-order filter

    def __post_init__(self):
        for name in ('kp', 'ki', 'kd', 'torque_limit', 'slew_limit', 'direction'):
            object.__setattr__(self, name, vector(getattr(self, name), name))
        if any(x < 0 for name in ('kp', 'ki', 'kd') for x in getattr(self, name)):
            raise ValueError('gains must be nonnegative')
        if any(x <= 0 for name in ('torque_limit', 'slew_limit') for x in getattr(self, name)):
            raise ValueError('limits must be positive')
        if any(x not in (-1, 1) for x in self.direction):
            raise ValueError('direction must be +1 or -1')
        if not all(isfinite(x) for x in (self.min_dt, self.max_dt, self.max_age, self.derivative_tau)) or not (0 < self.min_dt <= self.max_dt and self.max_age > 0 and self.derivative_tau >= 0):
            raise ValueError('invalid timing configuration')


class Fault(RuntimeError):
    pass


class JointTorqueFeedforwardPID:
    def __init__(self, config):
        self.config = config
        self.fault = None
        self.previous_error = None
        self.previous_t = None
        self.integral = [0.0] * N  # signed, in error * seconds
        self.derivative = [0.0] * N
        self.output = [0.0] * N
        self.state = None

    def trip(self, reason):
        self.fault = str(reason)
        self.output = [0.0] * N  # software state only; does not stop a drive
        raise Fault(self.fault)

    def update(self, desired, actual, feedback_ns, now_ns, faults, feedforward_nm, feedforward_ns):
        if self.fault:
            raise Fault(self.fault)
        try:
            desired = vector(desired, 'desired')
            actual = vector(actual, 'actual')
            if len(feedback_ns) != N or len(faults) != N or any(faults):
                raise ValueError('invalid feedback or motor fault')
            if not isinstance(now_ns, Integral) or any(not isinstance(t, Integral) or t <= 0 or t > now_ns or (now_ns - int(t)) * 1e-9 > self.config.max_age for t in feedback_ns):
                raise ValueError('stale, future or invalid feedback timestamp')
            feedback_ns = tuple(int(t) for t in feedback_ns)
            feedforward_nm = vector(feedforward_nm, 'feedforward_nm')
            if len(feedforward_ns) != N or any(not isinstance(t, Integral) or t <= 0 or t > now_ns or (now_ns - int(t)) * 1e-9 > self.config.max_age for t in feedforward_ns):
                raise ValueError('stale, future or invalid feedforward timestamp')
            errors = [desired[i] - actual[i] for i in range(N)]
            if not all(isfinite(x) for x in errors):
                raise ValueError('nonfinite error')
            if self.previous_t is None:
                self.previous_t = tuple(feedback_ns)
                self.previous_error = errors
                self.state = {'error_m': errors, 'p_nm': [0.0]*N, 'i_nm': [0.0]*N, 'd_nm': [0.0]*N, 'tau_nm': [0.0]*N, 'feedforward_nm': list(feedforward_nm), 'primed': True}
                return tuple(self.output)
            dts = [(feedback_ns[i] - self.previous_t[i]) * 1e-9 for i in range(N)]
            if any(not self.config.min_dt <= dt <= self.config.max_dt for dt in dts):
                raise ValueError('invalid feedback interval')
            p, iterm, dterm, output = [], [], [], []
            next_integral, next_derivative = [], []
            for i, dt in enumerate(dts):
                cfg = self.config
                alpha = dt / (cfg.derivative_tau + dt)
                derivative = self.derivative[i] + alpha * ((errors[i] - self.previous_error[i]) / dt - self.derivative[i])
                integ = self.integral[i] + errors[i] * dt
                sign = cfg.direction[i]
                p_i, d_i = sign * cfg.kp[i] * errors[i], sign * cfg.kd[i] * derivative
                raw = feedforward_nm[i] + p_i + sign * cfg.ki[i] * integ + d_i
                lo = max(-cfg.torque_limit[i], self.output[i] - cfg.slew_limit[i] * dt)
                hi = min(cfg.torque_limit[i], self.output[i] + cfg.slew_limit[i] * dt)
                # Conditional integration also accounts for slew saturation.
                if (raw > hi and sign * errors[i] > 0) or (raw < lo and sign * errors[i] < 0):
                    integ = self.integral[i]
                    raw = feedforward_nm[i] + p_i + sign * cfg.ki[i] * integ + d_i
                out = max(lo, min(hi, raw))
                if not all(isfinite(x) for x in (derivative, integ, p_i, d_i, raw, out)):
                    raise ValueError('nonfinite controller state')
                next_integral.append(integ); next_derivative.append(derivative)
                p.append(p_i); iterm.append(sign * cfg.ki[i] * integ); dterm.append(d_i); output.append(out)
            self.integral, self.derivative, self.output = next_integral, next_derivative, output
            self.previous_error, self.previous_t = errors, tuple(feedback_ns)
            self.state = {'error_m': errors, 'p_nm': p, 'i_nm': iterm, 'd_nm': dterm, 'tau_nm': output[:], 'feedforward_nm': list(feedforward_nm), 'primed': False}
            return tuple(output)
        except (ValueError, TypeError, OverflowError) as exc:
            self.trip(exc)
