#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Stage J13-2A1: First-Order Linear Active Disturbance Rejection Control (LADRC)
Core Module with Second-Order Linear Extended State Observer (LESO).

Mathematical Specification:
  Single-Axis Nominal Plant:
    e_dot = f - b0 * u
    where:
      e: World-frame SO(3) tracking error component (rad)
      u: World-frame attitude residual command (rad)
      f: Equivalent lumped disturbance (rad/s)
      b0 > 0: High-frequency input gain parameter (s^-1)

  Continuous Second-Order LESO:
    z1_dot = z2 - b0 * u_actual + beta1 * (y - z1)
    z2_dot = beta2 * (y - z1)
    where:
      y = e: measured tracking error (rad)
      beta1 = 2 * omega_o
      beta2 = omega_o^2
      z1 ≈ e (rad)
      z2 ≈ f (rad/s)

  Exact Discrete LESO (ZOH Closed-Form Discretization):
    z_k = Phi * z_{k-1} + Gamma_u * u_actual_{k-1} + Gamma_y * y_k
    where:
      Ts = dt
      w = omega_o * Ts
      E = exp(-w)
      Phi = E * [[1 - w, Ts], [-omega_o^2 * Ts, 1 + w]]
      Gamma_u = [-b0 * Ts * E,  b0 * (1 - E * (1 + w))]^T
      Gamma_y = [1 - E * (1 - w),  E * omega_o^2 * Ts]^T

  LADRC Control Law (Shadow Mode):
    u_raw = (omega_c * z1 + z2) / b0
    u_shadow = clip(u_raw, -u_limit, +u_limit)

Strict Safety Contract:
  - This module is pure Python and contains NO ROS dependency.
  - In SHADOW MODE, observer updates ALWAYS use the actual physical plant input
    (u_plant_actual_{k-1}), NEVER the hypothetical shadow output (u_shadow).
  - Both discrete observer eigenvalues are strictly lambda = exp(-omega_o * Ts) in (0, 1).
"""

from __future__ import annotations

import math
from typing import Any, Dict, List, Optional, Sequence, Tuple
import numpy as np


class FirstOrderLadrcChannel:
    """Single-axis SISO First-Order LADRC with Exact Discrete Second-Order LESO."""

    def __init__(
        self,
        b0: float,
        omega_c: float,
        omega_o: float,
        u_limit: float = math.radians(0.5),
        nominal_dt: float = 0.020,
        min_dt: float = 0.001,
        max_dt: float = 0.100,
    ) -> None:
        """Initializes single-axis LADRC channel.

        Args:
            b0: Nominal plant input gain parameter (s^-1), must be > 0.
            omega_c: Controller bandwidth (rad/s), must be > 0.
            omega_o: Observer bandwidth (rad/s), must be > 0.
            u_limit: Output saturation limit (rad), must be > 0.
            nominal_dt: Nominal sample time (s).
            min_dt: Minimum allowable dt (s).
            max_dt: Maximum allowable dt (s).

        Raises:
            ValueError: If parameters violate positivity constraints.
        """
        if b0 <= 0.0 or not math.isfinite(b0):
            raise ValueError(f"b0 must be positive and finite, got {b0}")
        if omega_c <= 0.0 or not math.isfinite(omega_c):
            raise ValueError(f"omega_c must be positive and finite, got {omega_c}")
        if omega_o <= 0.0 or not math.isfinite(omega_o):
            raise ValueError(f"omega_o must be positive and finite, got {omega_o}")
        if u_limit <= 0.0 or not math.isfinite(u_limit):
            raise ValueError(f"u_limit must be positive and finite, got {u_limit}")

        self.b0: float = float(b0)
        self.omega_c: float = float(omega_c)
        self.omega_o: float = float(omega_o)
        self.u_limit: float = float(u_limit)
        self.nominal_dt: float = float(nominal_dt)
        self.min_dt: float = float(min_dt)
        self.max_dt: float = float(max_dt)

        # Internal observer states
        self.z1: float = 0.0
        self.z2: float = 0.0
        self.previous_u_actual: float = 0.0
        self.initialized: bool = False

        # Anomaly / fault tracking
        self.anomaly_count: int = 0
        self.last_anomaly_reason: str = ""

    def reset(
        self,
        z1_init: float = 0.0,
        z2_init: float = 0.0,
        previous_u_actual: float = 0.0,
    ) -> None:
        """Resets observer states and establishes warm/cold start condition.

        Args:
            z1_init: Initial state estimate z1 (typically current measured error y0).
            z2_init: Initial disturbance estimate z2 (typically 0.0).
            previous_u_actual: Last actual control applied to the plant.
        """
        if not math.isfinite(z1_init):
            z1_init = 0.0
        if not math.isfinite(z2_init):
            z2_init = 0.0
        if not math.isfinite(previous_u_actual):
            previous_u_actual = 0.0

        self.z1 = float(z1_init)
        self.z2 = float(z2_init)
        self.previous_u_actual = float(previous_u_actual)
        self.initialized = True
        self.last_anomaly_reason = ""

    def compute_discrete_matrices(
        self, dt: float
    ) -> Tuple[float, float, float, float, float, float, float, float]:
        """Computes exact ZOH discretization coefficients for sample time dt.

        Returns:
            Tuple of (phi11, phi12, phi21, phi22, gamma_u1, gamma_u2, gamma_y1, gamma_y2)
        """
        Ts = dt
        w = self.omega_o * Ts
        E = math.exp(-w)

        phi11 = E * (1.0 - w)
        phi12 = E * Ts
        phi21 = -E * (self.omega_o**2) * Ts
        phi22 = E * (1.0 + w)

        gamma_u1 = -self.b0 * Ts * E
        gamma_u2 = self.b0 * (1.0 - E * (1.0 + w))

        gamma_y1 = 1.0 - E * (1.0 - w)
        gamma_y2 = E * (self.omega_o**2) * Ts

        return phi11, phi12, phi21, phi22, gamma_u1, gamma_u2, gamma_y1, gamma_y2

    def observer_update(
        self,
        measurement: float,
        actual_previous_input: float,
        dt: float,
    ) -> Tuple[float, float]:
        """Executes exact discrete LESO update step using plant feedback and prior control.

        Args:
            measurement: Measured error y_k = e_k (rad).
            actual_previous_input: Actual control applied to the plant at k-1 (rad).
            dt: Sample time interval (s).

        Returns:
            Tuple of updated (z1_k, z2_k).
        """
        # Validate and sanitize dt
        effective_dt = dt
        if not math.isfinite(dt) or dt < self.min_dt or dt > self.max_dt:
            self.anomaly_count += 1
            self.last_anomaly_reason = f"Invalid dt={dt}, clamping to nominal {self.nominal_dt}"
            effective_dt = self.nominal_dt

        # Validate inputs
        if not math.isfinite(measurement):
            self.anomaly_count += 1
            self.last_anomaly_reason = f"Non-finite measurement: {measurement}"
            return self.z1, self.z2

        if not math.isfinite(actual_previous_input):
            self.anomaly_count += 1
            self.last_anomaly_reason = f"Non-finite actual_previous_input: {actual_previous_input}"
            actual_previous_input = self.previous_u_actual

        y_k = float(measurement)
        u_prev = float(actual_previous_input)

        # Compute exact discrete state transition matrices
        (
            phi11,
            phi12,
            phi21,
            phi22,
            gu1,
            gu2,
            gy1,
            gy2,
        ) = self.compute_discrete_matrices(effective_dt)

        # Discrete observer update
        z1_next = phi11 * self.z1 + phi12 * self.z2 + gu1 * u_prev + gy1 * y_k
        z2_next = phi21 * self.z1 + phi22 * self.z2 + gu2 * u_prev + gy2 * y_k

        # Numerical sanity check
        if not (math.isfinite(z1_next) and math.isfinite(z2_next)):
            self.anomaly_count += 1
            self.last_anomaly_reason = f"Non-finite state computed: z1={z1_next}, z2={z2_next}"
            return self.z1, self.z2

        self.z1 = z1_next
        self.z2 = z2_next
        self.previous_u_actual = u_prev
        self.initialized = True

        return self.z1, self.z2

    def compute_shadow_command(self) -> Tuple[float, float, bool]:
        """Computes LADRC residual command from current observer states.

        Returns:
            Tuple of (u_shadow, u_raw, is_saturated).
        """
        if not (math.isfinite(self.z1) and math.isfinite(self.z2)):
            return 0.0, 0.0, False

        u_raw = (self.omega_c * self.z1 + self.z2) / self.b0
        u_shadow = max(-self.u_limit, min(self.u_limit, u_raw))
        is_saturated = abs(u_raw) > self.u_limit

        return u_shadow, u_raw, is_saturated

    def step(
        self,
        measurement: float,
        actual_previous_input: float,
        dt: float,
    ) -> Tuple[float, float, float, float, bool]:
        """Combined observer update and shadow command calculation.

        Args:
            measurement: Measured tracking error y_k (rad).
            actual_previous_input: Actual control applied at tick k-1 (rad).
            dt: Elapsed time since previous update (s).

        Returns:
            Tuple of (u_shadow, u_raw, z1, z2, is_saturated).
        """
        z1, z2 = self.observer_update(measurement, actual_previous_input, dt)
        u_shadow, u_raw, is_saturated = self.compute_shadow_command()
        return u_shadow, u_raw, z1, z2, is_saturated


class ThreeAxisLadrc:
    """3-Axis container managing independent Roll, Pitch, Yaw LADRC channels."""

    def __init__(
        self,
        b0_roll: float = 20.0,
        b0_pitch: float = 10.0,
        b0_yaw: float = 6.0,
        omega_c_roll: float = 0.8,
        omega_c_pitch: float = 1.0,
        omega_c_yaw: float = 0.25,
        omega_o_roll: float = 2.5,
        omega_o_pitch: float = 4.0,
        omega_o_yaw: float = 1.2,
        u_limit_roll_deg: float = 0.5,
        u_limit_pitch_deg: float = 0.5,
        u_limit_yaw_deg: float = 0.5,
        nominal_dt: float = 0.020,
        min_dt: float = 0.001,
        max_dt: float = 0.100,
    ) -> None:
        """Initializes 3-axis independent LADRC channels."""
        self.channels: List[FirstOrderLadrcChannel] = [
            FirstOrderLadrcChannel(
                b0=b0_roll,
                omega_c=omega_c_roll,
                omega_o=omega_o_roll,
                u_limit=math.radians(u_limit_roll_deg),
                nominal_dt=nominal_dt,
                min_dt=min_dt,
                max_dt=max_dt,
            ),
            FirstOrderLadrcChannel(
                b0=b0_pitch,
                omega_c=omega_c_pitch,
                omega_o=omega_o_pitch,
                u_limit=math.radians(u_limit_pitch_deg),
                nominal_dt=nominal_dt,
                min_dt=min_dt,
                max_dt=max_dt,
            ),
            FirstOrderLadrcChannel(
                b0=b0_yaw,
                omega_c=omega_c_yaw,
                omega_o=omega_o_yaw,
                u_limit=math.radians(u_limit_yaw_deg),
                nominal_dt=nominal_dt,
                min_dt=min_dt,
                max_dt=max_dt,
            ),
        ]

    @property
    def b0(self) -> np.ndarray:
        return np.array([ch.b0 for ch in self.channels], dtype=np.float64)

    @property
    def omega_c(self) -> np.ndarray:
        return np.array([ch.omega_c for ch in self.channels], dtype=np.float64)

    @property
    def omega_o(self) -> np.ndarray:
        return np.array([ch.omega_o for ch in self.channels], dtype=np.float64)

    @property
    def u_limit(self) -> np.ndarray:
        return np.array([ch.u_limit for ch in self.channels], dtype=np.float64)

    @property
    def z1(self) -> np.ndarray:
        return np.array([ch.z1 for ch in self.channels], dtype=np.float64)

    @property
    def z2(self) -> np.ndarray:
        return np.array([ch.z2 for ch in self.channels], dtype=np.float64)

    def reset(
        self,
        z1_init: Optional[Sequence[float]] = None,
        z2_init: Optional[Sequence[float]] = None,
        previous_u_actual: Optional[Sequence[float]] = None,
    ) -> None:
        """Resets all 3 channels."""
        z1 = [0.0, 0.0, 0.0] if z1_init is None else list(z1_init)
        z2 = [0.0, 0.0, 0.0] if z2_init is None else list(z2_init)
        u_prev = [0.0, 0.0, 0.0] if previous_u_actual is None else list(previous_u_actual)

        for i in range(3):
            self.channels[i].reset(
                z1_init=z1[i],
                z2_init=z2[i],
                previous_u_actual=u_prev[i],
            )

    def observer_update(
        self,
        e_R_W_rad: Sequence[float],
        actual_previous_input_rad: Sequence[float],
        dt: float,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Updates observer states for all 3 axes."""
        z1_out = np.zeros(3, dtype=np.float64)
        z2_out = np.zeros(3, dtype=np.float64)

        for i in range(3):
            z1_i, z2_i = self.channels[i].observer_update(
                measurement=e_R_W_rad[i],
                actual_previous_input=actual_previous_input_rad[i],
                dt=dt,
            )
            z1_out[i] = z1_i
            z2_out[i] = z2_i

        return z1_out, z2_out

    def compute_shadow_command(self) -> Tuple[np.ndarray, np.ndarray, List[bool]]:
        """Computes shadow command for all 3 axes."""
        u_shadow = np.zeros(3, dtype=np.float64)
        u_raw = np.zeros(3, dtype=np.float64)
        sat = [False, False, False]

        for i in range(3):
            u_s, u_r, is_sat = self.channels[i].compute_shadow_command()
            u_shadow[i] = u_s
            u_raw[i] = u_r
            sat[i] = is_sat

        return u_shadow, u_raw, sat

    def step(
        self,
        e_R_W_rad: Sequence[float],
        actual_previous_input_rad: Sequence[float],
        dt: float,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, List[bool]]:
        """Executes full step across all 3 axes in causal tick order.

        Returns:
            Tuple of (u_shadow, u_raw, z1, z2, saturated_axes).
        """
        z1, z2 = self.observer_update(e_R_W_rad, actual_previous_input_rad, dt)
        u_shadow, u_raw, sat = self.compute_shadow_command()
        return u_shadow, u_raw, z1, z2, sat
