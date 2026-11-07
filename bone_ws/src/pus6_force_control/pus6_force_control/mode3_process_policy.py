"""Three-process policy only; legacy motion/dual-IMU entry points stay unchanged.

50 N is the LAF50-024D catalog maximum push/pull force, NOT a documented
firmware force-clamp register. 0x27 is signed g; 0x20 is overcurrent in mA;
0x21/0x22 are drive-output maxima with a documented 0..1000 scale.
"""
from .six_axis_force_drive import ForceLimitsConfig


RATED_AXIAL_FORCE_N = 50.
PID_OVERRIDES = dict(joint_integral_limit_n=None, joint_increment_limit_n=None,
                     joint_max_tracking_error_m=None)


def force_limits():
    return ForceLimitsConfig(min_force_n=(-RATED_AXIAL_FORCE_N,)*6,
        max_force_n=(RATED_AXIAL_FORCE_N,)*6, max_rate_g_s=None, brake_rate_g_s=(5000.,)*6)


def policy_summary():
    return dict(pid_integral_limit_n=None, pid_increment_limit_n=None, tracking_error_exit_m=None,
        absolute_target_force_limit_n=RATED_AXIAL_FORCE_N,
        force_boundary_source='LAF50-024D catalog maximum push/pull force; host enforced',
        firmware_50n_force_clamp_verified=False, normal_reference_speed_limit_m_s=None,
        normal_force_rate_limit_g_s=None, exceptional_recovery_rate_g_s=5000.)
