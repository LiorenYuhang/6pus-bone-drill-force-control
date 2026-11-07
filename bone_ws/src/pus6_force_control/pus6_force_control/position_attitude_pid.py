"""Position J11-M2 attitude residual PID, copied without algorithm changes.

Source: h30_imu/scripts/j11_m2_pid_residual_runner.py in Position.
Output is an angular residual in rad, never a force or a moment. This local
copy has no dependency on the Position checkout or its ROS runner.
"""
from dataclasses import dataclass, field
import math
from typing import List, Optional, Sequence
import numpy as np

@dataclass
class M2PidGains:
    kp: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float64))
    ki: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float64))
    kd: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float64))


@dataclass
class M2PidLimits:
    residual_limit_rad: np.ndarray = field(
        default_factory=lambda: np.array(
            [math.radians(1.0), math.radians(1.0), math.radians(2.0)], dtype=np.float64
        )
    )
    integral_limit_rad_s: np.ndarray = field(
        default_factory=lambda: np.array(
            [math.radians(2.0), math.radians(2.0), math.radians(4.0)], dtype=np.float64
        )
    )
    min_dt_s: float = 0.001
    max_dt_s: float = 0.100
    derivative_enabled: bool = True
    derivative_lpf_cutoff_hz: float = 10.0  # Low-pass filter cutoff frequency in Hz; <= 0 disables LPF


class M2TaskSpaceAttitudePid:
    def __init__(
        self,
        gains: Optional[M2PidGains] = None,
        limits: Optional[M2PidLimits] = None,
    ) -> None:
        self.gains = gains if gains is not None else M2PidGains()
        self.limits = limits if limits is not None else M2PidLimits()
        self.enabled: bool = True
        self.integral: np.ndarray = np.zeros(3, dtype=np.float64)
        self.last_error: np.ndarray = np.zeros(3, dtype=np.float64)
        self.last_cmd: np.ndarray = np.zeros(3, dtype=np.float64)
        self.has_prev_error: bool = False
        self.saturated_axes: List[bool] = [False, False, False]
        self.last_p_term: np.ndarray = np.zeros(3, dtype=np.float64)
        self.last_i_term: np.ndarray = np.zeros(3, dtype=np.float64)
        self.last_d_term: np.ndarray = np.zeros(3, dtype=np.float64)
        self.last_raw_derivative: np.ndarray = np.zeros(3, dtype=np.float64)
        self.last_filtered_derivative: np.ndarray = np.zeros(3, dtype=np.float64)
        self.derivative_initialized: bool = False

    def reset(self) -> None:
        self.integral = np.zeros(3, dtype=np.float64)
        self.last_error = np.zeros(3, dtype=np.float64)
        self.last_cmd = np.zeros(3, dtype=np.float64)
        self.has_prev_error = False
        self.saturated_axes = [False, False, False]
        self.last_p_term = np.zeros(3, dtype=np.float64)
        self.last_i_term = np.zeros(3, dtype=np.float64)
        self.last_d_term = np.zeros(3, dtype=np.float64)
        self.last_raw_derivative = np.zeros(3, dtype=np.float64)
        self.last_filtered_derivative = np.zeros(3, dtype=np.float64)
        self.derivative_initialized = False

    def enable(self) -> None:
        self.enabled = True

    def disable(self) -> None:
        self.enabled = False
        self.reset()

    def update(
        self,
        e_R_W_rad: Sequence[float],
        dt_s: float,
        omega_W_rad_s: Optional[Sequence[float]] = None,
    ) -> np.ndarray:
        """Computes PID corrective residual delta_phi_W [rad] in World frame.

        Mathematical Structure:
          e_R_W = Log(R_WP_des @ R_WP_meas^T)^vee
          e_dot_R_W_raw = (e_R_W[k] - e_R_W[k-1]) / dt
          e_dot_R_W = LPF(e_dot_R_W_raw)
          delta_phi_W = Kp * e_R_W + Kd * e_dot_R_W + Ki * integral
        """
        if not self.enabled:
            return np.zeros(3, dtype=np.float64)

        e = np.asarray(e_R_W_rad, dtype=np.float64)
        if len(e) != 3 or not np.all(np.isfinite(e)):
            return self.last_cmd

        if not math.isfinite(dt_s) or dt_s < self.limits.min_dt_s:
            return self.last_cmd

        effective_dt = min(dt_s, self.limits.max_dt_s)

        # 1. Proportional term
        p_term = self.gains.kp * e

        # 2. Integral term with anti-windup clamping (strictly 0 when Ki = 0)
        integral_cand = self.integral + e * effective_dt
        for i in range(3):
            if self.saturated_axes[i] and (e[i] * self.last_cmd[i] > 0.0):
                integral_cand[i] = self.integral[i]
        self.integral = np.clip(
            integral_cand,
            -self.limits.integral_limit_rad_s,
            self.limits.integral_limit_rad_s,
        )
        i_term = self.gains.ki * self.integral

        # 3. Derivative term with low-pass filtering
        if self.limits.derivative_enabled:
            if self.has_prev_error:
                raw_deriv = (e - self.last_error) / effective_dt
                if self.limits.derivative_lpf_cutoff_hz > 0.0:
                    fc = self.limits.derivative_lpf_cutoff_hz
                    tau = 1.0 / (2.0 * math.pi * fc)
                    alpha = tau / (tau + effective_dt)
                    if not self.derivative_initialized:
                        filt_deriv = raw_deriv.copy()
                        self.derivative_initialized = True
                    else:
                        filt_deriv = alpha * self.last_filtered_derivative + (1.0 - alpha) * raw_deriv
                else:
                    filt_deriv = raw_deriv.copy()
            else:
                raw_deriv = np.zeros(3, dtype=np.float64)
                filt_deriv = np.zeros(3, dtype=np.float64)
            d_term = self.gains.kd * filt_deriv
        else:
            raw_deriv = np.zeros(3, dtype=np.float64)
            filt_deriv = np.zeros(3, dtype=np.float64)
            d_term = np.zeros(3, dtype=np.float64)

        raw_output = p_term + i_term + d_term
        cmd = np.clip(raw_output, -self.limits.residual_limit_rad, self.limits.residual_limit_rad)

        for i in range(3):
            self.saturated_axes[i] = bool(abs(raw_output[i]) > self.limits.residual_limit_rad[i])

        self.last_error = e.copy()
        self.last_cmd = cmd.copy()
        self.has_prev_error = True
        self.last_p_term = p_term
        self.last_i_term = i_term
        self.last_d_term = d_term
        self.last_raw_derivative = raw_deriv
        self.last_filtered_derivative = filt_deriv

        return cmd

