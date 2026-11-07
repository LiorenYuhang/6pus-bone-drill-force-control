"""Z force PID in pus6_base; numerical interface only, no hardware access.

Input/output are task generalized force in N. This does not estimate external
contact force. Derivative acts on measured force to avoid reference-step kick.
"""
from dataclasses import dataclass
import math
import time

from .rate_limiter import ForceReferenceRateLimiter


@dataclass(frozen=True)
class ForcePIDConfig:
    kp: float = 1.0                 # N/N
    ki: float = 0.0                 # 1/s
    kd: float = 0.0                 # s
    output_min_n: float = -10.0     # total task force output bounds
    output_max_n: float = 10.0
    integral_limit_n: float = 5.0   # bound on integral contribution, N
    derivative_filter_tau_s: float = 0.05
    reference_rate_n_s: float = 1.0 # example, not hardware tuning
    output_mode: str = "reference_plus_pid"  # or pid_only

    def __post_init__(self):
        for name, value in vars(self).items():
            if name != "output_mode" and not math.isfinite(value):
                raise ValueError(f"{name} must be finite")
        if min(self.kp, self.ki, self.kd, self.integral_limit_n, self.derivative_filter_tau_s) < 0:
            raise ValueError("gains, integral bound and filter time must be nonnegative")
        if self.reference_rate_n_s <= 0 or self.output_min_n >= self.output_max_n:
            raise ValueError("positive reference rate and ordered output bounds required")
        if self.output_mode not in ("reference_plus_pid", "pid_only"):
            raise ValueError("output_mode must be reference_plus_pid or pid_only")


@dataclass(frozen=True)
class ForcePIDResult:
    reference_n: float
    measured_n: float
    error_n: float
    proportional_n: float
    integral_n: float
    derivative_n: float
    output_n: float
    saturated: bool
    dt_s: float


class TaskSpaceForcePID:
    def __init__(self, config=None, *, clock=time.monotonic_ns):
        self.config = config or ForcePIDConfig()
        self.clock = clock
        self.reference = ForceReferenceRateLimiter(self.config.reference_rate_n_s, clock=clock)
        self.reset(now_ns=self.reference.last_ns)

    def reset(self, *, initial_reference_n=0.0, measured_force_n=None, now_ns=None):
        if measured_force_n is not None and not math.isfinite(measured_force_n):
            raise ValueError("measured force must be finite")
        self.reference.reset(initial_reference_n, now_ns=now_ns)
        self.integral_n = 0.0
        self.filtered_derivative_n_s = 0.0
        self.previous_measured_n = measured_force_n

    def update(self, target_force_n, measured_force_n, *, now_ns=None):
        if not math.isfinite(measured_force_n):
            raise ValueError("measured force must be finite")
        c = self.config
        now = self.clock() if now_ns is None else now_ns
        dt = (now - self.reference.last_ns) * 1e-9
        ref = self.reference.update(target_force_n, now_ns=now)
        error = ref - measured_force_n
        raw_d = (0.0 if self.previous_measured_n is None
                 else -(measured_force_n - self.previous_measured_n) / dt)
        alpha = dt / (c.derivative_filter_tau_s + dt)
        self.filtered_derivative_n_s += alpha * (raw_d - self.filtered_derivative_n_s)
        p = c.kp * error
        d = c.kd * self.filtered_derivative_n_s
        ff = ref if c.output_mode == "reference_plus_pid" else 0.0
        candidate_i = max(-c.integral_limit_n, min(c.integral_limit_n,
                                                self.integral_n + c.ki * error * dt))
        candidate_u = ff + p + candidate_i + d
        # Conditional integration: reject changes pushing farther into saturation.
        delta_i = candidate_i - self.integral_n
        if not ((candidate_u > c.output_max_n and delta_i > 0)
                or (candidate_u < c.output_min_n and delta_i < 0)):
            self.integral_n = candidate_i
        raw_u = ff + p + self.integral_n + d
        output = max(c.output_min_n, min(c.output_max_n, raw_u))
        self.previous_measured_n = measured_force_n
        return ForcePIDResult(ref, measured_force_n, error, p, self.integral_n, d,
                              output, output != raw_u, dt)
