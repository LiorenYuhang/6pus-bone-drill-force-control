r"""Six-Axis Wrench Allocation and Kinematic Jacobian for 6PUS Parallel Manipulator.

Physical Formulation:
  Actuator Coordinates:
    q_i: Prismatic slider position along vertical Z-axis guide rails, SI unit: meters (m).
    \dot{q}: Slider velocity vector (m/s).
    F_slider: Vertical driving forces exerted by electric cylinders (N).
    Actuator native force data unit is g (gram), confirmed by manufacturer sales.
    Nominal SI scale for numerical software mapping: ~101.9716 g/N.

  Task-Space Coordinates:
    p = [x, y, z]^T: Platform origin position in base frame, SI unit: meters (m).
    \eta = [\alpha, \beta, \gamma]^T: Z-Y-X Euler angles (roll, pitch, yaw) matching inver.m, SI unit: radians (rad).
    \xi = [\dot{p}^T, \omega_B^T]^T: Spatial twist (linear velocity m/s, angular velocity rad/s in base frame).
    \dot{X}_\eta = [\dot{p}^T, \dot{\eta}^T]^T: Generalized Euler coordinate velocity.
    w_spatial = [F^T, M_B^T]^T: Task wrench (force in N, moment about platform origin in N*m).

  Kinematic and Dynamic Dual Relationships:
    \dot{q} = J_spatial \xi = J_\eta \dot{X}_\eta
    \omega_B = T(\eta) \dot{\eta}
    J_\eta = J_spatial \begin{bmatrix} I_3 & 0 \\ 0 & T(\eta) \end{bmatrix}
    w_spatial = J_spatial^T F_slider
    w_spatial^T \xi = F_slider^T \dot{q}  (Principle of Virtual Work)

Features:
- Closed-form analytical spatial Jacobian and finite-difference Euler Jacobian.
- Exact geometric mapping T(eta) between Euler rates and base-frame angular velocity.
- Scale-normalized condition number \kappa(\tilde{J}) using platform characteristic length L_char.
- Full strut geometry analysis (strut lengths, unit vectors u_i, tilt angles \theta_i, u_{i,z}).
- Direct linear solve (np.linalg.solve) avoiding explicit matrix inversion.
- StaticTareBaseline management with configuration validity checks.
- Dual conversion: Task Wrench -> 6-Axis Actuator Forces, and 6-Axis Forces -> Reconstructed Task Wrench.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Optional, Sequence, Tuple, Union
import numpy as np

from pus6_kinematics.analytical_6pus import (
    Analytical6PusKinematics, DEFAULT_PLATFORM_RADIUS_M,
    DEFAULT_BASE_RADIUS_M, DEFAULT_STRUT_LENGTH_M,
)


from pus6_motor_driver.force_units import NOMINAL_N_PER_G, NOMINAL_G_PER_N, g_to_n, n_to_g


def euler_rates_to_omega_matrix_zyx(alpha: float, beta: float, gamma: float) -> np.ndarray:
    r"""Computes 3x3 transformation matrix T(eta) such that \omega_B = T(\eta) \dot{\eta}.

    For Z-Y-X Euler angle rotation R = Rz(gamma) * Ry(beta) * Rx(alpha):
      \omega_B = \dot{\gamma} \hat{z} + Rz(\gamma) \dot{\beta} \hat{y} + Rz(\gamma) Ry(\beta) \dot{\alpha} \hat{x}
    """
    ca, sa = math.cos(alpha), math.sin(alpha)
    cb, sb = math.cos(beta), math.sin(beta)
    cg, sg = math.cos(gamma), math.sin(gamma)
    return np.array([
        [cb * cg, -sg, 0.0],
        [cb * sg,  cg, 0.0],
        [-sb,     0.0, 1.0],
    ], dtype=float)


@dataclass(frozen=True)
class StrutGeometryInfo:
    """Detailed geometric parameters of the 6PUS limbs at a specific pose."""
    strut_lengths_m: Tuple[float, ...]
    unit_vectors: Tuple[Tuple[float, float, float], ...]  # u_i from slider to platform
    z_projections: Tuple[float, ...]                      # u_{i,z}
    tilt_angles_deg: Tuple[float, ...]                    # arccos(u_{i,z}) in degrees


@dataclass(frozen=True)
class StaticTareBaseline:
    """Reference static assembly preload and gravity tare baseline.

    IMPORTANT: This baseline is NOT a global rigid-body gravity model.
    It represents the combined effect of platform weight and assembly preload
    measured at the specific nominal operating configuration.
    Valid only within a local neighborhood of reference_pose.
    Beyond this neighborhood, platform weight redistributes across limbs and tare
    must be recalibrated.
    """
    reference_pose: Tuple[float, ...] = (0.0, 0.0, 0.132612268, 0.0, 0.0, 0.0)
    baseline_forces_g: Tuple[float, ...] = (0.0,) * 6
    timestamp: str = "UNCONFIGURED"
    source_experiment: str = "UNCONFIGURED: supply a measured local tare explicitly"
    valid_translation_radius_m: float = 0.003   # +/- 3 mm
    valid_rotation_radius_rad: float = 0.0349   # +/- 2 degrees (~0.0349 rad)

    def is_valid_at_pose(self, pose: Sequence[float]) -> bool:
        p = np.asarray(pose, dtype=float)
        p_ref = np.asarray(self.reference_pose, dtype=float)
        trans_err = float(np.linalg.norm(p[:3] - p_ref[:3]))
        rot_err = float(np.linalg.norm(p[3:6] - p_ref[3:6]))
        return trans_err <= self.valid_translation_radius_m and rot_err <= self.valid_rotation_radius_rad


@dataclass(frozen=True)
class WrenchAllocationResult:
    """Result container for six-axis wrench allocation."""
    task_wrench_ref: np.ndarray      # [Fx, Fy, Fz, Mx, My, Mz] (N, N*m)
    actuator_forces_n: np.ndarray    # Six-axis net slider forces (N)
    actuator_forces_g: np.ndarray    # Six-axis net forces in manufacturer grams (g)
    dispatched_forces_g: np.ndarray  # Net force + tare preload (g)
    jacobian_6x6: np.ndarray         # Spatial Jacobian mapping task twist to slider velocity
    condition_number: float          # Raw kappa(J^T) = sigma_max / sigma_min
    normalized_condition_number: float  # Scale-normalized kappa(J_tilde^T) using L_char
    min_singular_value: float        # Minimum singular value of raw J^T
    is_singular: bool                # Based on dimensionless, characteristic-length-scaled J
    strut_geometry: Optional[StrutGeometryInfo] = None
    tare_warning: Optional[str] = None


class SixAxisWrenchAllocator:
    """Computes Jacobian, solves wrench allocation, and reconstructs task wrenches."""

    def __init__(
        self,
        kinematics: Optional[Analytical6PusKinematics] = None,
        *,
        cond_max_threshold: float = 1.0e4,
        min_singular_threshold: float = 1.0e-5,
        newtons_per_gram: float = NOMINAL_N_PER_G,
        char_length_m: float = DEFAULT_PLATFORM_RADIUS_M,
    ) -> None:
        for name, value in (
            ("cond_max_threshold", cond_max_threshold),
            ("min_singular_threshold", min_singular_threshold),
            ("newtons_per_gram", newtons_per_gram),
            ("char_length_m", char_length_m),
        ):
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be positive and finite")
        self.kin = kinematics or Analytical6PusKinematics()
        self.cond_max_threshold = cond_max_threshold
        self.min_singular_threshold = min_singular_threshold
        self.newtons_per_gram = newtons_per_gram
        self.grams_per_newton = 1.0 / newtons_per_gram
        self.char_length_m = char_length_m

    def compute_strut_geometry(self, pose: Sequence[float]) -> StrutGeometryInfo:
        """Computes geometric limb lengths, 3D unit vectors, Z-projections, and tilt angles."""
        p = np.asarray(pose, dtype=float)
        if p.shape != (6,) or not np.isfinite(p).all():
            raise ValueError("Pose must contain six finite values (m, rad)")
        x, y, z, alpha, beta, gamma = p
        R = np.asarray(self.kin._rotation_matrix_zyx(alpha, beta, gamma), dtype=float)
        q = self.kin.inverse_kinematics(pose)

        lengths = []
        unit_vecs = []
        z_projs = []
        tilts = []

        for i in range(6):
            a_i = np.asarray(self.kin.base_anchors[i], dtype=float)
            b_i = np.asarray(self.kin.platform_anchors_body[i], dtype=float)
            p_i = np.array([x, y, z], dtype=float) + R @ b_i
            s_i = np.array([a_i[0], a_i[1], q[i]], dtype=float)
            l_i = p_i - s_i
            dist = float(np.linalg.norm(l_i))
            u_i = l_i / dist
            uz = float(u_i[2])

            lengths.append(dist)
            unit_vecs.append((float(u_i[0]), float(u_i[1]), uz))
            z_projs.append(uz)
            tilts.append(float(math.degrees(math.acos(max(-1.0, min(1.0, uz))))))

        return StrutGeometryInfo(
            strut_lengths_m=tuple(lengths),
            unit_vectors=tuple(unit_vecs),
            z_projections=tuple(z_projs),
            tilt_angles_deg=tuple(tilts),
        )

    def compute_analytical_spatial_jacobian(self, pose: Sequence[float]) -> np.ndarray:
        r"""Compute closed-form 6x6 spatial Jacobian J_spatial such that \dot{q} = J_spatial * \xi.

        Row i is given analytically by:
          J_spatial[i, :] = [ u_{i,x}/u_{i,z},  u_{i,y}/u_{i,z},  1.0,  ( (R b_i) x u_i ) / u_{i,z} ]
        """
        p = np.asarray(pose, dtype=float)
        if p.shape != (6,) or not np.isfinite(p).all():
            raise ValueError("Pose must contain six finite values (m, rad)")
        x, y, z, alpha, beta, gamma = p
        R = np.asarray(self.kin._rotation_matrix_zyx(alpha, beta, gamma), dtype=float)
        q = self.kin.inverse_kinematics(pose)

        J_sp = np.zeros((6, 6), dtype=float)
        for i in range(6):
            a_i = np.asarray(self.kin.base_anchors[i], dtype=float)
            b_i = np.asarray(self.kin.platform_anchors_body[i], dtype=float)
            p_i = np.array([x, y, z], dtype=float) + R @ b_i
            s_i = np.array([a_i[0], a_i[1], q[i]], dtype=float)
            l_i = p_i - s_i
            dist = float(np.linalg.norm(l_i))
            u_i = l_i / dist
            uz = float(u_i[2])
            if abs(uz) < 1e-6:
                raise ValueError(f"Limb {i+1} strut is perpendicular to vertical guide rail")

            r_i = R @ b_i
            r_cross_u = np.cross(r_i, u_i)

            J_sp[i, 0:3] = u_i / uz
            J_sp[i, 3:6] = r_cross_u / uz

        return J_sp

    def compute_jacobian(self, pose: Sequence[float], eps: float = 1.0e-6) -> np.ndarray:
        r"""Compute full 6x6 task-space Jacobian J_xi = dq / dx_task at pose via central differences.

        For backward compatibility, this returns the generalized Euler Jacobian J_eta.
        At zero orientation (\alpha=\beta=\gamma=0), J_eta is mathematically identical to J_spatial.
        """
        p = np.asarray(pose, dtype=float)
        if p.shape != (6,) or not np.isfinite(p).all():
            raise ValueError(f"Pose must be 6 finite floats, got {pose}")

        cols = []
        for j in range(6):
            p_plus = p.copy()
            p_minus = p.copy()
            p_plus[j] += eps
            p_minus[j] -= eps
            q_plus = np.asarray(self.kin.inverse_kinematics(p_plus), dtype=float)
            q_minus = np.asarray(self.kin.inverse_kinematics(p_minus), dtype=float)
            cols.append((q_plus - q_minus) / (2.0 * eps))

        J = np.column_stack(cols)
        if not np.isfinite(J).all():
            raise ValueError("Jacobian calculation yielded non-finite entries")
        return J

    def check_conditioning(
        self,
        J: np.ndarray,
        char_length_m: Optional[float] = None,
    ) -> Tuple[float, float, bool]:
        """Inspects singular values and condition number of J^T.

        Args:
            J: 6x6 Jacobian matrix.
            char_length_m: Optional characteristic length for dimensional scaling.
                           If None, returns raw condition number (backward-compatible).

        Returns:
            (cond_number, min_sigma, is_singular)
        """
        if char_length_m is not None:
            D = np.diag([1.0, 1.0, 1.0, 1.0 / char_length_m, 1.0 / char_length_m, 1.0 / char_length_m])
            J_eval = J @ D
        else:
            J_eval = J

        s = np.linalg.svd(J_eval.T, compute_uv=False)
        sigma_max = float(s[0])
        sigma_min = float(s[-1])
        cond = sigma_max / sigma_min if sigma_min > 1e-15 else float("inf")
        is_singular = cond > self.cond_max_threshold or sigma_min < self.min_singular_threshold
        return cond, sigma_min, is_singular

    def check_normalized_conditioning(
        self,
        J: np.ndarray,
        char_length_m: Optional[float] = None,
    ) -> Tuple[float, float, bool]:
        """Inspects scale-normalized condition number using platform characteristic length."""
        L = char_length_m if char_length_m is not None else self.char_length_m
        return self.check_conditioning(J, char_length_m=L)

    def allocate(
        self,
        task_wrench: Sequence[float],
        pose: Sequence[float],
        tare_preload_g: Optional[Union[Sequence[float], StaticTareBaseline]] = None,
    ) -> WrenchAllocationResult:
        r"""Solves J^T * F_slider = w_task without explicit matrix inversion.

        Args:
            task_wrench: [Fx, Fy, Fz, Mx, My, Mz] in SI units (N, N*m).
            pose: platform pose [x, y, z, roll, pitch, yaw] (m, rad).
            tare_preload_g: static assembly preload & gravity baseline in grams or StaticTareBaseline.

        Returns:
            WrenchAllocationResult with solved forces, Jacobian, and conditioning.
        """
        w = np.asarray(task_wrench, dtype=float)
        if w.shape != (6,) or not np.isfinite(w).all():
            raise ValueError(f"task_wrench must have 6 finite elements, got {task_wrench}")

        # Use base-frame spatial wrench, not Euler generalized moments.
        J = self.compute_analytical_spatial_jacobian(pose)
        cond_raw, min_sigma, _ = self.check_conditioning(J, char_length_m=None)
        cond_norm, min_sigma_norm, is_singular = self.check_normalized_conditioning(J)

        if is_singular:
            raise ValueError(
                f"Mechanism configuration is near-singular: normalized_cond={cond_norm:.1f} "
                f"(max {self.cond_max_threshold}), normalized_min_sigma={min_sigma_norm:.2e}; "
                f"raw_cond={cond_raw:.1f} (diagnostic only)"
            )

        # Solve J^T * F = w via linear system solve (LU decomposition with pivoting)
        F_n = np.linalg.solve(J.T, w)

        # Convert to manufacturer grams
        F_g = np.asarray(n_to_g(F_n, scale_n_per_g=self.newtons_per_gram))

        tare_arr = None
        tare_warning = None
        if tare_preload_g is not None:
            if isinstance(tare_preload_g, StaticTareBaseline):
                if not tare_preload_g.is_valid_at_pose(pose):
                    raise ValueError("Static tare is invalid at the supplied pose")
                tare_arr = np.asarray(tare_preload_g.baseline_forces_g, dtype=float)
            else:
                tare_arr = np.asarray(tare_preload_g, dtype=float)

            if tare_arr.shape != (6,) or not np.isfinite(tare_arr).all():
                raise ValueError("tare_preload_g must have 6 finite elements")
            dispatched_g = F_g + tare_arr
        else:
            dispatched_g = F_g.copy()

        strut_info = self.compute_strut_geometry(pose)

        return WrenchAllocationResult(
            task_wrench_ref=w,
            actuator_forces_n=F_n,
            actuator_forces_g=F_g,
            dispatched_forces_g=dispatched_g,
            jacobian_6x6=J,
            condition_number=cond_raw,
            normalized_condition_number=cond_norm,
            min_singular_value=min_sigma,
            is_singular=is_singular,
            strut_geometry=strut_info,
            tare_warning=tare_warning,
        )

    def reconstruct_task_wrench(
        self,
        actual_forces_g: Sequence[float],
        pose: Sequence[float],
        tare_preload_g: Optional[Union[Sequence[float], StaticTareBaseline]] = None,
    ) -> np.ndarray:
        """Reconstructs platform task-space wrench w = J^T * F_net from sensor readings.

        Args:
            actual_forces_g: six-axis sensor forces in grams (from 0x2C).
            pose: current platform pose [x, y, z, roll, pitch, yaw].
            tare_preload_g: static assembly preload & gravity baseline in grams or StaticTareBaseline.

        Returns:
            reconstructed task wrench [Fx, Fy, Fz, Mx, My, Mz] (N, N*m).
        """
        f_raw = np.asarray(actual_forces_g, dtype=float)
        if f_raw.shape != (6,) or not np.isfinite(f_raw).all():
            raise ValueError("actual_forces_g must have 6 finite elements")

        if tare_preload_g is not None:
            if isinstance(tare_preload_g, StaticTareBaseline):
                tare = np.asarray(tare_preload_g.baseline_forces_g, dtype=float)
            else:
                tare = np.asarray(tare_preload_g, dtype=float)
            if tare.shape != (6,) or not np.isfinite(tare).all():
                raise ValueError("tare_preload_g must have 6 finite elements")
            if isinstance(tare_preload_g, StaticTareBaseline) and not tare_preload_g.is_valid_at_pose(pose):
                raise ValueError("Static tare is invalid at the supplied pose")
            f_net_g = f_raw - tare
        else:
            f_net_g = f_raw

        force_n = g_to_n(f_net_g, scale_n_per_g=self.newtons_per_gram)
        return self.reconstruct_task_wrench_n(force_n, pose)

    def reconstruct_task_wrench_n(self, force_n, pose):
        """SI-only virtual work: actuator N -> base-frame wrench [N, Nm]."""
        force_n = np.asarray(force_n, dtype=float)
        if force_n.shape != (6,) or not np.isfinite(force_n).all():
            raise ValueError('force_n must have six finite elements')
        return self.compute_analytical_spatial_jacobian(pose).T @ force_n

    def verify_virtual_work(
        self,
        task_wrench: Sequence[float],
        task_velocity: Sequence[float],
        pose: Sequence[float],
    ) -> dict:
        r"""Validates principle of virtual work: w^T \xi == F^T \dot{q}."""
        w = np.asarray(task_wrench, dtype=float)
        xi = np.asarray(task_velocity, dtype=float)
        alloc = self.allocate(w, pose)
        J = alloc.jacobian_6x6
        q_dot = J @ xi

        work_task = float(np.dot(w, xi))
        work_actuator = float(np.dot(alloc.actuator_forces_n, q_dot))
        err = abs(work_task - work_actuator)

        return {
            "work_task_power_W": work_task,
            "work_actuator_power_W": work_actuator,
            "absolute_error_W": err,
            "is_energy_conserving": err < 1.0e-9,
        }
