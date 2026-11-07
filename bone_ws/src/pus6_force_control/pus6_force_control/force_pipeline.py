"""Offline feedback -> spatial wrench -> Z PID -> native force frames.

The caller supplies the CURRENT pose or six measured slider positions for FK.
No nominal pose is substituted for missing feedback. This stage always builds
commands in dry_run; it neither opens UART nor selects motor modes.
"""
from dataclasses import dataclass
import time

import numpy as np

from .six_axis_force_drive import SixAxisForceDrive
from .task_space_force_pid import TaskSpaceForcePID
from .wrench_allocation import SixAxisWrenchAllocator
from pus6_motor_driver.force_units import NOMINAL_N_PER_G, g_to_n, n_to_g


def vector6(value, name):
    out = np.asarray(value, dtype=float)
    if out.shape != (6,) or not np.isfinite(out).all():
        raise ValueError(f"{name} must contain six finite values")
    return out


def raw_steps_to_slider_m(raw_steps, geometry):
    """Existing OPERATING_HOME mapping, distinct from raw steps times m/step."""
    steps = vector6(raw_steps, 'raw_steps')
    return (np.asarray(geometry.operating_home_stroke_m)
            + (steps - geometry.operating_home_raw_steps) * geometry.meters_per_step)


@dataclass(frozen=True)
class ForceCompensation:
    """Calibration interfaces; no historical tare is enabled by default.

    direction maps increasing positive motor axial force into +q (slider up).
    Sensor zero is removed from feedback ONLY. The task/static support bias is
    removed from feedback and added to command ONCE. Static support already
    includes gravity: use either actuator_bias_n OR gravity_wrench, never both.
    Forces here are slider drive forces, not inclined strut tensions.
    """
    sensor_zero_g: tuple = (0.0,) * 6
    direction: tuple = (1.0,) * 6
    newtons_per_g: tuple = (NOMINAL_N_PER_G,) * 6
    actuator_bias_n: tuple = (0.0,) * 6

    def __post_init__(self):
        for name in vars(self):
            value = vector6(getattr(self, name), name)
            object.__setattr__(self, name, tuple(value))
        if any(x not in (-1.0, 1.0) for x in self.direction):
            raise ValueError("force directions must be +1 or -1")
        if min(self.newtons_per_g) <= 0:
            raise ValueError("force scales must be positive")

    def feedback_n(self, force_g_raw):
        return np.asarray(g_to_n(vector6(force_g_raw, "force_g_raw") - self.sensor_zero_g,
                                 self.direction, self.newtons_per_g))

    def command_g(self, forces_n):
        # Native 0x27 and 0x2C polarity/scale must be confirmed together before use.
        # Feedback electronic zero is not an additional physical preload command.
        return np.asarray(n_to_g(vector6(forces_n, "forces_n"), self.direction, self.newtons_per_g))

    def command_n(self, forces_g):
        return np.asarray(g_to_n(vector6(forces_g, "forces_g"), self.direction, self.newtons_per_g))


@dataclass(frozen=True)
class ForceCycleResult:
    pose: np.ndarray
    measured_wrench: np.ndarray  # wrench_n_nm: [Fx,Fy,Fz,Mx,My,Mz]
    commanded_wrench: np.ndarray  # N, Nm
    applied_wrench: np.ndarray  # predicted after drive limits, NOT measured force
    pid: object
    allocation: object
    drive_result: dict
    actuator_limited: bool


class TaskSpaceForcePipeline:
    def __init__(self, pid_config=None, *, drive=None, allocator=None,
                 compensation=None, clock=time.monotonic_ns):
        self.allocator = allocator or SixAxisWrenchAllocator()
        self.compensation = compensation or ForceCompensation()
        self.drive = drive or SixAxisForceDrive(bus=None, clock=clock)
        if self.drive.hardware_write_enabled:
            raise ValueError("stage-1 pipeline requires hardware_write_enabled=False")
        self.drive.direction = self.compensation.direction
        self.drive.newtons_per_g = self.compensation.newtons_per_g
        self.pid = TaskSpaceForcePID(pid_config, clock=clock)
        self.clock = clock
        self.hold_forces_g = None  # explicit per-experiment reference; never Tare
        # Anchor the actuator slew limiter at zero for a bounded first command.
        if self.drive.last_commanded_forces_g is None:
            self.drive.last_commanded_forces_g = [0.0] * 6
            self.drive.last_command_time_ns = self.pid.reference.last_ns

    def initialize_hold_forces_g(self, force_g_raw, *, now_ns=None):
        """Capture current native feedback ONCE, without any hardware command.

        The UART owner must use exactly this same vector in its Mode 3 frame.
        An active reference cannot be silently recaptured by PID/fault reset.
        """
        if self.hold_forces_g is not None:
            raise RuntimeError("hold reference already captured")
        if any(self.compensation.actuator_bias_n) or any(self.compensation.sensor_zero_g):
            raise ValueError("hold reference replaces software zero/static bias; do not subtract twice")
        hold = self.drive.anchor_hold_forces_g(force_g_raw, now_ns=now_ns)
        self.hold_forces_g = tuple(hold)
        self.pid.reset(now_ns=self.drive.last_command_time_ns)
        return self.hold_forces_g

    @property
    def hold_forces_n(self):
        return (np.zeros(6) if self.hold_forces_g is None
                else self.compensation.command_n(self.hold_forces_g))

    def relative_feedback_n(self, force_g_raw):
        if self.hold_forces_g is None:
            return self.compensation.feedback_n(force_g_raw)
        return self.compensation.command_n(vector6(force_g_raw, "force_g_raw") - self.hold_forces_g)

    def update(self, target_z_n, force_g_raw, pose, *, gravity_wrench=None, now_ns=None):
        """All wrenches: pus6_base, moments about platform origin, [N,N*m].

        target_z_n is net task force after the selected static/gravity removal.
        In dynamics it also contains inertial/friction effects. External contact
        force requires a load/dynamics model and the correct action/reaction sign.
        """
        pose = vector6(pose, "pose")
        c = self.compensation
        bias = np.array(c.actuator_bias_n)
        gravity = np.zeros(6) if gravity_wrench is None else vector6(gravity_wrench, "gravity_wrench")
        if self.hold_forces_g is not None and (np.any(bias) or np.any(gravity)):
            raise ValueError("hold reference already includes current support; no extra bias/gravity")
        if np.any(bias) and np.any(gravity):
            raise ValueError("static actuator bias already includes gravity; choose one compensation")
        # Validate geometry/conditioning before advancing PID state.
        support = self.allocator.allocate(gravity, pose)
        J = support.jacobian_6x6
        force_n = self.relative_feedback_n(force_g_raw)
        wrench_n_nm = J.T @ (force_n - bias) - gravity
        old_integral = self.pid.integral_n
        now = self.clock() if now_ns is None else now_ns
        pid = self.pid.update(target_z_n, float(wrench_n_nm[2]), now_ns=now)
        wrench = np.zeros(6)
        wrench[2] = pid.output_n
        allocation = self.allocator.allocate(wrench + gravity, pose)
        forces_n = self.hold_forces_n + allocation.actuator_forces_n + bias
        # Explicit True even if an external caller attempts to change drive settings.
        dispatch = self.drive.command_forces_n(forces_n, dry_run=True, now_ns=now)
        encoded_g = [frame["target_force_g"] for frame in dispatch["frames"]]
        applied = J.T @ (c.command_n(encoded_g) - self.hold_forces_n - bias) - gravity
        limited = (not np.allclose(forces_n, dispatch["limited_forces_n"], atol=1e-10, rtol=0)
                   or not np.allclose(dispatch["target_forces_g"], dispatch["dispatched_forces_g"], atol=1e-10, rtol=0))
        # Downstream force caps/slew limits must not wind up the task integrator.
        if limited and (pid.output_n - applied[2]) * pid.error_n > 0:
            self.pid.integral_n = old_integral
        return ForceCycleResult(pose.copy(), wrench_n_nm, wrench, applied, pid,
                                allocation, dispatch, limited)

    def command_increment_wrench(self, wrench_n_nm, force_g_raw, pose, *, now_ns=None):
        """Generalized SI increment -> existing allocator/drive; never selects modes.

        Active stabilization supplies pose-PID moments in Nm and captured-position
        hold forces in N. No Z PID, electronic zero or extra gravity is applied here.
        """
        if self.hold_forces_g is None:
            raise ValueError("Mode 3 current-force hold must be initialized")
        pose=vector6(pose,"pose");wrench=vector6(wrench_n_nm,"wrench_n_nm")
        allocation=self.allocator.allocate(wrench,pose)
        j=allocation.jacobian_6x6
        measured=j.T@self.relative_feedback_n(force_g_raw)
        absolute_n=self.hold_forces_n+allocation.actuator_forces_n
        dispatch=self.drive.command_forces_n(absolute_n,dry_run=True,now_ns=now_ns)
        encoded=[row["target_force_g"] for row in dispatch["frames"]]
        applied=j.T@(self.compensation.command_n(encoded)-self.hold_forces_n)
        limited=(not np.allclose(absolute_n,dispatch["limited_forces_n"],atol=1e-10,rtol=0)
                 or not np.allclose(dispatch["target_forces_g"],dispatch["dispatched_forces_g"],atol=1e-10,rtol=0))
        return ForceCycleResult(pose.copy(),measured,wrench.copy(),applied,None,allocation,dispatch,limited)

    def command_actuator_increment(self,delta_force_n,force_g_raw,pose,*,now_ns=None,braking_axes=None):
        """Joint outer PID -> existing hold, SI caps, native g slew and encoding.

        J is for measured/predicted wrench monitoring; no inverse allocation is
        applied to an already actuator-space force vector.
        """
        if self.hold_forces_g is None:raise ValueError('holding reference required')
        delta=vector6(delta_force_n,'delta_force_n');j=None;allocation=None
        if pose is not None:
            pose=vector6(pose,'pose');j=self.allocator.compute_analytical_spatial_jacobian(pose)
            allocation=self.allocator.allocate(j.T@delta,pose)
        absolute=self.hold_forces_n+delta
        dispatch=self.drive.command_forces_n(absolute,dry_run=True,now_ns=now_ns,braking_axes=braking_axes)
        encoded=[r['target_force_g'] for r in dispatch['frames']]
        applied_delta=self.compensation.command_n(encoded)-self.hold_forces_n
        limited=(not np.allclose(absolute,dispatch['limited_forces_n'],atol=1e-10,rtol=0)
            or not np.allclose(dispatch['target_forces_g'],dispatch['dispatched_forces_g'],atol=1e-10,rtol=0))
        return ForceCycleResult(None if pose is None else pose.copy(),None if j is None else j.T@self.relative_feedback_n(force_g_raw),
            None if j is None else j.T@delta,None if j is None else j.T@applied_delta,None,allocation,dispatch,limited)

    def update_from_snapshot(self, target_z_n, snapshot, *, initial_pose=None,
                             gravity_wrench=None, now_ns=None):
        """Use real status positions for FK, retaining ID1..6 ordering.

        Snapshot is the existing SixAxisForceDrive.read_feedback() dictionary.
        Host receipt times are not device sample times; alignment is caller-owned.
        """
        if not snapshot["all_healthy"] or any(snapshot["error_codes"]):
            raise ValueError("motor fault in feedback")
        cfg = self.allocator.kin.geometry_config
        sliders = raw_steps_to_slider_m(snapshot["position_steps"], cfg)
        pose = self.allocator.kin.forward_kinematics(sliders, initial_guess=initial_pose)
        return self.update(target_z_n, snapshot["force_g_raw"], pose,
                           gravity_wrench=gravity_wrench, now_ns=now_ns)
