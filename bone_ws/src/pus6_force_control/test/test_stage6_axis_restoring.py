"""Targeted tests for coordinate transforms, SO(3) sign conventions, Jacobian allocation, and hold baseline."""
import math
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from pus6_kinematics.analytical_6pus import Analytical6PusKinematics
from pus6_motor_driver.force_units import NOMINAL_N_PER_G, NOMINAL_G_PER_N, g_to_n, n_to_g
from pus6_force_control.wrench_allocation import SixAxisWrenchAllocator
from pus6_force_control.active_stabilization import ActiveStabilizer, StabilizationConfig
from pus6_force_control.position_attitude_math import (
    relative_quat, apply_z90_transform, quat_to_rot_matrix,
    compute_so3_platform_error, compose_so3_pid_residual
)
from pus6_force_control.force_pipeline import TaskSpaceForcePipeline

C_Z90 = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
I_QUAT = np.array([0.0, 0.0, 0.0, 1.0])


def test_imu_z90_coordinate_transformation_mapping():
    """Verify R_mech = Rz(+90) @ R_imu @ Rz(-90) maps IMU frame axes to mechanism axes."""
    # When IMU pitches around sensor X (+X_imu):
    # R_mech = Rz(+90) @ RotX @ Rz(-90) = RotY(+angle)
    # This verifies X_imu maps to Y_mech.
    q_x_imu = Rotation.from_euler('x', 5.0, degrees=True).as_quat()
    q_mech = apply_z90_transform(q_x_imu)
    rpy_mech = Rotation.from_quat(q_mech).as_euler('xyz', degrees=True)
    assert rpy_mech[1] == pytest.approx(5.0, abs=1e-5)
    assert rpy_mech[0] == pytest.approx(0.0, abs=1e-5)
    assert rpy_mech[2] == pytest.approx(0.0, abs=1e-5)

    # When IMU rolls around sensor Y (+Y_imu):
    # R_mech = Rz(+90) @ RotY @ Rz(-90) = RotX(-angle)
    # This verifies Y_imu maps to -X_mech.
    q_y_imu = Rotation.from_euler('y', 5.0, degrees=True).as_quat()
    q_mech_y = apply_z90_transform(q_y_imu)
    rpy_mech_y = Rotation.from_quat(q_mech_y).as_euler('xyz', degrees=True)
    assert rpy_mech_y[0] == pytest.approx(-5.0, abs=1e-5)
    assert rpy_mech_y[1] == pytest.approx(0.0, abs=1e-5)
    assert rpy_mech_y[2] == pytest.approx(0.0, abs=1e-5)

    # Z axis is preserved
    q_z_imu = Rotation.from_euler('z', 5.0, degrees=True).as_quat()
    q_mech_z = apply_z90_transform(q_z_imu)
    rpy_mech_z = Rotation.from_quat(q_mech_z).as_euler('xyz', degrees=True)
    assert rpy_mech_z[2] == pytest.approx(5.0, abs=1e-5)


@pytest.mark.parametrize('axis,name', [(0, 'Roll'), (1, 'Pitch'), (2, 'Yaw')])
def test_so3_platform_error_restoring_signs(axis, name):
    """Verify that a positive perturbation produces a negative error vector that restores equilibrium."""
    delta_angle = 0.02  # ~1.15 deg
    R_des = np.eye(3)
    rotvec = np.zeros(3)
    rotvec[axis] = delta_angle
    R_meas = Rotation.from_rotvec(rotvec).as_matrix()

    _, e_R_W_rad = compute_so3_platform_error(R_des, R_meas)
    # Desired is 0, measured is +delta. Tracking error e = Des - Meas must be strictly negative!
    assert e_R_W_rad[axis] < 0
    assert e_R_W_rad[axis] == pytest.approx(-delta_angle, rel=1e-3)


def test_base_feedforward_compensates_base_tilt():
    """Verify Base tilt generates opposite relative reference R_BP_FF to maintain level platform."""
    now = [1_000_000_000]
    stab = ActiveStabilizer(clock=lambda: now[0])
    p0 = np.array([0.0, 0.0, 0.133, 0.0, 0.0, 0.0])
    stab.capture(p0, I_QUAT, I_QUAT, now[0])

    # Base pitches +2 deg around mechanical Y (nose up)
    R_base_mech = Rotation.from_euler('y', 2.0, degrees=True).as_matrix()
    R_base_imu = Rotation.from_euler('z', -90, degrees=True).as_matrix() @ R_base_mech @ Rotation.from_euler('z', 90, degrees=True).as_matrix()
    q_base = Rotation.from_matrix(R_base_imu).as_quat()

    now[0] += 20_000_000
    target_pose, diag = stab.target_pose(p0, q_base, I_QUAT, [0, 0, 0], [0, 0, 0], sample_ns=now[0])
    # Platform IMU stayed level (q_plat = I).
    # Commanded relative pitch between Base and Platform must be -2 deg (pitch down relative to base)
    cmd_rpy = Rotation.from_matrix(diag['command_rotation_base']).as_euler('xyz', degrees=True)
    assert cmd_rpy[1] == pytest.approx(-2.0, abs=1e-3)


def test_jacobian_physical_allocation_and_virtual_work():
    """Verify 6PUS Jacobian satisfies virtual work and maps moments to correct limb forces."""
    kin = Analytical6PusKinematics()
    alloc = SixAxisWrenchAllocator(kin)
    pose = [0.0, 0.0, kin.nominal_height, 0.0, 0.0, 0.0]

    # 1. Virtual work principle: w^T * xi == F^T * q_dot
    vw = alloc.verify_virtual_work([1.0, 2.0, 3.0, 0.1, 0.2, 0.3], [0.01, -0.02, 0.03, 0.1, -0.1, 0.05], pose)
    assert vw['is_energy_conserving']
    assert vw['absolute_error_W'] < 1e-9

    # 2. Pure Roll Moment (+Mx = +1 Nm): pushes +Y side up, pulls -Y side down
    res_mx = alloc.allocate([0.0, 0.0, 0.0, 1.0, 0.0, 0.0], pose)
    f_mx = res_mx.actuator_forces_n
    # Actuators 1, 6 have y = +0.0238 m; Actuators 3, 4 have y = -0.0238 m
    assert f_mx[0] > 0 and f_mx[5] > 0  # +Y side pushed UP
    assert f_mx[2] < 0 and f_mx[3] < 0  # -Y side pulled DOWN

    # 3. Pure Pitch Moment (+My = +1 Nm): right hand rule about +Y means -X side pushed UP, +X side pulled DOWN
    res_my = alloc.allocate([0.0, 0.0, 0.0, 0.0, 1.0, 0.0], pose)
    f_my = res_my.actuator_forces_n
    # Actuators 1, 2 have x < 0; Actuators 5, 6 have x > 0
    assert f_my[0] > 0 and f_my[1] > 0  # -X side pushed UP
    assert f_my[4] < 0 and f_my[5] < 0  # +X side pulled DOWN


def test_unit_conversion_nominal_n_per_g():
    """Verify N to g and g to N conversions preserve signs and scale accurately."""
    # 10 N -> ~1019.716 g
    g_val = n_to_g(10.0)
    assert g_val == pytest.approx(10.0 / NOMINAL_N_PER_G)
    # -10 N -> negative g
    g_neg = n_to_g(-10.0)
    assert g_neg == pytest.approx(-10.0 / NOMINAL_N_PER_G)
    # Round trip
    assert g_to_n(g_val) == pytest.approx(10.0)
    assert g_to_n(g_neg) == pytest.approx(-10.0)


def test_holding_force_baseline_invariance():
    """Verify holding force baseline is latched once and unaffected by subsequent force readings."""
    now = [1_000_000_000]
    pipe = TaskSpaceForcePipeline(clock=lambda: now[0])
    hold_init = [100.0, 120.0, 95.0, 110.0, 105.0, 115.0]
    pipe.initialize_hold_forces_g(hold_init)

    assert pipe.hold_forces_g == tuple(hold_init)

    # Advance time and call with massive sensor feedback disturbance
    now[0] += 20_000_000
    noisy_feedback = [500.0, -300.0, 1200.0, 0.0, -800.0, 450.0]
    pose = [0.0, 0.0, pipe.allocator.kin.nominal_height, 0.0, 0.0, 0.0]
    wrench = [0.0, 0.0, 0.0, 0.05, 0.0, 0.0]

    cycle = pipe.command_increment_wrench(wrench, noisy_feedback, pose, now_ns=now[0])
    # Hold forces baseline must remain 100% identical!
    assert pipe.hold_forces_g == tuple(hold_init)
    # Second cycle with zero wrench
    now[0] += 20_000_000
    cycle2 = pipe.command_increment_wrench([0.0]*6, noisy_feedback, pose, now_ns=now[0])
    assert pipe.hold_forces_g == tuple(hold_init)
