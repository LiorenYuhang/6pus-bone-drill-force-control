"""Pure math reused from Position j11_m2_pid_residual_runner.py.

Relative quaternion, mechanical Rz(+90) conjugation and world SO(3) error.
No angle-residual PID gains, ROS runner, IK target publisher or hardware code.
Callers validate input quaternions before these original math helpers.
"""
import math
from typing import Sequence, Tuple
import numpy as np
from scipy.spatial.transform import Rotation as R

def normalize_quat(q: Sequence[float]) -> Tuple[float, float, float, float]:
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return (x / n, y / n, z / n, w / n)

def quat_conjugate(q: Sequence[float]) -> Tuple[float, float, float, float]:
    return (-q[0], -q[1], -q[2], q[3])

def quat_multiply(q1: Sequence[float], q2: Sequence[float]) -> Tuple[float, float, float, float]:
    """Hamilton quaternion multiplication q1 ⊗ q2 in [x, y, z, w] convention."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )

def relative_quat(q0: Sequence[float], q: Sequence[float]) -> Tuple[float, float, float, float]:
    """q_rel = q0^* ⊗ q"""
    return normalize_quat(quat_multiply(quat_conjugate(normalize_quat(q0)), normalize_quat(q)))

def apply_z90_transform(q_rel: Sequence[float]) -> Tuple[float, float, float, float]:
    """Applies R_mech = Rz(+90) @ R(q_rel) @ Rz(-90)"""
    qz_pos = (0.0, 0.0, math.sin(math.pi / 4.0), math.cos(math.pi / 4.0))
    qz_neg = (0.0, 0.0, -math.sin(math.pi / 4.0), math.cos(math.pi / 4.0))
    return normalize_quat(quat_multiply(quat_multiply(qz_pos, q_rel), qz_neg))

def quat_to_rot_matrix(q: Sequence[float]) -> np.ndarray:
    x, y, z, w = normalize_quat(q)
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ], dtype=np.float64)

def compute_so3_platform_error(
    R_WP_des: np.ndarray,
    R_WP_meas: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    """Computes frame-consistent SO(3) attitude tracking error.

    E_W = R_WP_des @ R_WP_meas.T
    e_R_W = Log(E_W)^vee   (Lie algebra error vector in World frame [rad])

    Returns:
      (E_W_mat, e_R_W_rad)
    """
    E_W_mat = R_WP_des @ R_WP_meas.T
    rot_err = R.from_matrix(E_W_mat)
    e_R_W_rad = rot_err.as_rotvec()
    return E_W_mat, e_R_W_rad


def rot_matrix_to_quat_xyzw(R_mat: np.ndarray) -> Tuple[float, float, float, float]:
    tr = R_mat[0, 0] + R_mat[1, 1] + R_mat[2, 2]
    if tr > 0:
        S = math.sqrt(tr + 1.0) * 2.0
        w = 0.25 * S
        x = (R_mat[2, 1] - R_mat[1, 2]) / S
        y = (R_mat[0, 2] - R_mat[2, 0]) / S
        z = (R_mat[1, 0] - R_mat[0, 1]) / S
    elif (R_mat[0, 0] > R_mat[1, 1]) and (R_mat[0, 0] > R_mat[2, 2]):
        S = math.sqrt(1.0 + R_mat[0, 0] - R_mat[1, 1] - R_mat[2, 2]) * 2.0
        w = (R_mat[2, 1] - R_mat[1, 2]) / S
        x = 0.25 * S
        y = (R_mat[0, 1] + R_mat[1, 0]) / S
        z = (R_mat[0, 2] + R_mat[2, 0]) / S
    elif R_mat[1, 1] > R_mat[2, 2]:
        S = math.sqrt(1.0 + R_mat[1, 1] - R_mat[0, 0] - R_mat[2, 2]) * 2.0
        w = (R_mat[0, 2] - R_mat[2, 0]) / S
        x = (R_mat[0, 1] + R_mat[1, 0]) / S
        y = 0.25 * S
        z = (R_mat[1, 2] + R_mat[2, 1]) / S
    else:
        S = math.sqrt(1.0 + R_mat[2, 2] - R_mat[0, 0] - R_mat[1, 1]) * 2.0
        w = (R_mat[1, 0] - R_mat[0, 1]) / S
        x = (R_mat[0, 2] + R_mat[2, 0]) / S
        y = (R_mat[1, 2] + R_mat[2, 1]) / S
        z = 0.25 * S
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    return (x / norm, y / norm, z / norm, w / norm)



def compose_so3_pid_residual(
    R_WB: np.ndarray,
    R_BP_FF: np.ndarray,
    delta_phi_W_rad: Sequence[float],
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, Tuple[float, float, float, float]]:
    """Authoritative SO(3) residual composition law for Dual-IMU platform.

    Derivation:
      1. World residual rotation: Delta_R_PID_W = Exp([delta_phi_W]_x)
      2. Transform to Base mechanical frame:
         Delta_R_PID_B = R_WB.T @ Delta_R_PID_W @ R_WB
      3. Compose command:
         R_BP_cmd = Delta_R_PID_B @ R_BP_FF
         (Note: exactly equivalent to R_BP_FF @ Delta_R_PID_W)

    Returns:
      (Delta_R_PID_W, Delta_R_PID_B, R_BP_cmd, q_cmd_xyzw)
    """
    phi_W = np.asarray(delta_phi_W_rad, dtype=np.float64)
    Delta_R_PID_W = R.from_rotvec(phi_W).as_matrix()
    Delta_R_PID_B = R_WB.T @ Delta_R_PID_W @ R_WB
    R_BP_cmd = Delta_R_PID_B @ R_BP_FF
    q_cmd_xyzw = rot_matrix_to_quat_xyzw(R_BP_cmd)
    return Delta_R_PID_W, Delta_R_PID_B, R_BP_cmd, q_cmd_xyzw

