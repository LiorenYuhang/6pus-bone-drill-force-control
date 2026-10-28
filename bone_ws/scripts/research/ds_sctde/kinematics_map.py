"""Local fixed-translation Euler-angle Jacobian using existing 6PUS IK."""
import numpy as np


def attitude_jacobian(kinematics, pose, epsilon=1e-6):
    pose = np.asarray(pose, dtype=float)
    if pose.shape != (6,) or not np.isfinite(pose).all() or not np.isfinite(epsilon) or epsilon <= 0:
        raise ValueError('pose or finite-difference step invalid')
    columns = []
    for j in range(3, 6):
        a = pose.copy(); b = pose.copy()
        a[j] += epsilon; b[j] -= epsilon
        columns.append((np.asarray(kinematics.inverse_kinematics(a)) - np.asarray(kinematics.inverse_kinematics(b))) / (2*epsilon))
    result = np.column_stack(columns)
    if not np.isfinite(result).all(): raise ValueError('invalid Jacobian')
    return result
