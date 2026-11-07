"""First-order physical-time limiter ported from Torque SixAxisPositionDrive.command.

The formula is old + clip(target-old, -rate*dt, rate*dt). No acceleration,
jerk, nominal-period substitution or third-order shaper is involved.
"""
import time

import numpy as np


class PhysicalTimeRateLimiter:
    """Scalar or vector limiter; rate has units of value/second.

    Seed from an explicit initial value and monotonic timestamp. The first update
    is limited too. Repeated or decreasing timestamps fail without changing state.
    Keep the floating state across calls, before protocol quantization (Torque).
    """

    def __init__(self, rate_per_s, initial_value=0.0, *, clock=time.monotonic_ns):
        self.clock = clock
        initial = np.asarray(initial_value, dtype=float)
        self.rate = np.broadcast_to(np.asarray(rate_per_s, dtype=float), initial.shape).copy()
        if not np.isfinite(self.rate).all() or (self.rate <= 0).any():
            raise ValueError("rate must be positive and finite")
        self.reset(initial)

    def reset(self, value, *, now_ns=None):
        value = np.asarray(value, dtype=float)
        if value.shape != self.rate.shape or not np.isfinite(value).all():
            raise ValueError("initial value must match rate shape and be finite")
        now = self.clock() if now_ns is None else now_ns
        if not isinstance(now, int) or now < 0:
            raise ValueError("timestamp must be nonnegative monotonic nanoseconds")
        self.value = value.copy()
        self.last_ns = now

    def update(self, target, *, now_ns=None):
        target = np.asarray(target, dtype=float)
        if target.shape != self.value.shape or not np.isfinite(target).all():
            raise ValueError("target must match initial shape and be finite")
        now = self.clock() if now_ns is None else now_ns
        if not isinstance(now, int) or now <= self.last_ns:
            raise ValueError("nonpositive command interval")
        dt = (now - self.last_ns) * 1e-9
        self.value = self.value + np.clip(target - self.value, -self.rate * dt, self.rate * dt)
        self.last_ns = now
        return float(self.value) if self.value.ndim == 0 else self.value.copy()


class PositionReferenceRateLimiter(PhysicalTimeRateLimiter):
    """Position in m; max_speed_m_s retains Torque's 0.017 m/s setting."""

    def __init__(self, initial_positions_m, max_speed_m_s=0.017, *, clock=time.monotonic_ns):
        super().__init__(max_speed_m_s, initial_positions_m, clock=clock)


class ForceReferenceRateLimiter(PhysicalTimeRateLimiter):
    """Force in N; max_rate_n_s must be configured independently of position."""

    def __init__(self, max_rate_n_s, initial_force_n=0.0, *, clock=time.monotonic_ns):
        super().__init__(max_rate_n_s, initial_force_n, clock=clock)
