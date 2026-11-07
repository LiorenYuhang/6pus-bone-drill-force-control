import numpy as np
import pytest

from pus6_force_control.rate_limiter import (
    PositionReferenceRateLimiter, ForceReferenceRateLimiter,
)
from pus6_force_control.task_space_force_pid import ForcePIDConfig, TaskSpaceForcePID
from pus6_force_control.force_pipeline import ForceCompensation, TaskSpaceForcePipeline
from pus6_force_control.six_axis_force_drive import (
    SixAxisForceDrive, ForceLimitsConfig, decode_u16_to_signed_i16,
)
from pus6_force_control.wrench_allocation import (
    SixAxisWrenchAllocator, StaticTareBaseline, euler_rates_to_omega_matrix_zyx,
)
from pus6_motor_driver.fake_bus import FakeBus
from pus6_motor_driver.protocol import parse_frame


@pytest.fixture
def timeline():
    return [1_000_000_000]


def test_physical_dt_and_units(timeline):
    clock = lambda: timeline[0]
    pos = PositionReferenceRateLimiter([0.05] * 6, clock=clock)
    force = ForceReferenceRateLimiter(10.0, clock=clock)
    for dt in (0.02, 0.007, 0.093):
        old_p = pos.value.copy()
        old_f = float(force.value)
        timeline[0] += round(dt * 1e9)
        assert pos.update([0.075] * 6) == pytest.approx(old_p + 0.017 * dt)
        assert force.update(100.0) == pytest.approx(old_f + 10.0 * dt)
    # A repeated or reversed clock cannot relax the rate limit or alter state.
    old = float(force.value)
    for stamp in (timeline[0], timeline[0] - 1):
        with pytest.raises(ValueError, match="nonpositive"):
            force.update(0, now_ns=stamp)
        assert float(force.value) == old
    timeline[0] += 1_000_000_000
    assert force.update(-100) == pytest.approx(old - 10.0)


def test_pid_feedback_sign_feedforward_and_reset(timeline):
    pid = TaskSpaceForcePID(ForcePIDConfig(kp=2, reference_rate_n_s=100), clock=lambda: timeline[0])
    timeline[0] += 20_000_000
    result = pid.update(1.0, 0.5)
    assert result.reference_n == 1.0
    assert result.error_n == 0.5
    assert result.output_n == 2.0  # reference + kp*error
    pid.reset(measured_force_n=0.5)
    timeline[0] += 10_000_000
    result = pid.update(0, 0.5)
    assert result.output_n == -1.0
    with pytest.raises(ValueError):
        pid.update(0, float("nan"))
    with pytest.raises(ValueError):
        pid.update(float("inf"), 0, now_ns=timeline[0] + 1)


def test_integral_saturation_recovery(timeline):
    cfg = ForcePIDConfig(kp=1, ki=2, output_mode="pid_only", output_min_n=-2,
                         output_max_n=2, reference_rate_n_s=1000, integral_limit_n=0.5)
    pid = TaskSpaceForcePID(cfg, clock=lambda: timeline[0])
    for _ in range(100):
        timeline[0] += 10_000_000
        result = pid.update(10.0, 0.0)
        assert result.output_n == 2.0
        assert pid.integral_n == 0.0
    timeline[0] += 20_000_000
    assert pid.update(-1.0, 0.0).output_n < 0
    for _ in range(100):
        timeline[0] += 10_000_000
        pid.update(0.1, 0.0)
    assert abs(pid.integral_n) <= 0.5


def test_derivative_filter_and_no_reference_kick(timeline):
    cfg = ForcePIDConfig(kp=0, kd=1, output_mode="pid_only", reference_rate_n_s=1000,
                         derivative_filter_tau_s=0.1, output_min_n=-100, output_max_n=100)
    pid = TaskSpaceForcePID(cfg, clock=lambda: timeline[0])
    pid.reset(measured_force_n=0.0)
    timeline[0] += 100_000_000
    assert pid.update(20, 0).derivative_n == 0
    timeline[0] += 100_000_000
    assert pid.update(20, 1).derivative_n == pytest.approx(-5)
    timeline[0] += 100_000_000
    assert pid.update(20, 1).derivative_n == pytest.approx(-2.5)


def test_spatial_mapping_at_nonzero_orientation():
    a = SixAxisWrenchAllocator()
    pose = [0.003, -0.002, a.kin.nominal_height + 0.004, 0.08, -0.06, 0.07]
    spatial = a.compute_analytical_spatial_jacobian(pose)
    transform = np.eye(6)
    transform[3:, 3:] = euler_rates_to_omega_matrix_zyx(*pose[3:])
    assert np.allclose(a.compute_jacobian(pose), spatial @ transform, atol=1e-7)
    wrench = [1, -2, 5, 0.07, -0.08, 0.09]
    allocated = a.allocate(wrench, pose)
    assert np.allclose(allocated.jacobian_6x6, spatial)
    assert np.allclose(a.reconstruct_task_wrench(allocated.dispatched_forces_g, pose), wrench)
    twist = np.array([0.02, -0.01, 0.03, 0.1, -0.2, 0.3])
    assert np.dot(wrench, twist) == pytest.approx(allocated.actuator_forces_n @ (spatial @ twist))
    tare = StaticTareBaseline(reference_pose=tuple(pose), baseline_forces_g=(10,) * 6)
    allocated = a.allocate(wrench, pose, tare)
    assert np.allclose(a.reconstruct_task_wrench(allocated.dispatched_forces_g, pose, tare), wrench)
    with pytest.raises(ValueError, match="tare"):
        a.allocate(wrench, [0, 0, a.kin.nominal_height + 0.02, 0, 0, 0], tare)
    with pytest.raises(ValueError, match="finite"):
        a.allocate(wrench, [float("nan")] * 6)


def test_bias_and_gravity_are_counted_once(timeline):
    comp = ForceCompensation(sensor_zero_g=(20,) * 6, actuator_bias_n=(1,) * 6)
    pipe = TaskSpaceForcePipeline(ForcePIDConfig(kp=0), compensation=comp, clock=lambda: timeline[0])
    pose = [0, 0, pipe.allocator.kin.nominal_height, 0, 0, 0]
    timeline[0] += 1_000_000_000
    result = pipe.update(0, [20 + 1 / 0.00980665] * 6, pose)
    assert np.allclose(result.measured_wrench, 0, atol=1e-9)
    assert result.drive_result["dispatched_forces_g"] == pytest.approx([1 / 0.00980665] * 6)
    # Sensor zero is not added back to native target force.
    with pytest.raises(ValueError, match="choose one"):
        pipe.update(0, [0] * 6, pose, gravity_wrench=[0, 0, 6, 0, 0, 0])
    other = TaskSpaceForcePipeline(ForcePIDConfig(kp=0), clock=lambda: timeline[0])
    timeline[0] += 1_000_000_000
    result = other.update(0, [1 / 0.00980665] * 6, pose, gravity_wrench=[0, 0, 6, 0, 0, 0])
    assert np.allclose(result.measured_wrench, 0, atol=1e-9)
    assert result.drive_result["dispatched_forces_g"] == pytest.approx([1 / 0.00980665] * 6)


def test_dry_pipeline_uses_actual_fk_and_builds_native_frames(timeline):
    bus = FakeBus(read_only=True)
    bus.positions = {i: 1010 for i in range(1, 7)}
    drive = SixAxisForceDrive(bus, clock=lambda: timeline[0])
    pipe = TaskSpaceForcePipeline(drive=drive, clock=lambda: timeline[0])
    snapshot = drive.read_feedback()
    timeline[0] += 20_000_000
    result = pipe.update_from_snapshot(1, snapshot)
    assert result.pose[2] == pytest.approx(pipe.allocator.kin.nominal_height + 0.00025)
    assert result.pid.reference_n == pytest.approx(0.02)
    assert result.drive_result["dry_run"]
    assert len(result.drive_result["frames"]) == 6
    for row in result.drive_result["frames"]:
        frame = parse_frame(bytes.fromhex(row["frame_hex"]))
        assert frame.register_addr == 0x27
    assert not bus.write_audit
    # Build Mode 3 frame only: no mode selection is executed.
    frame = parse_frame(drive.build_mode3_entry_frame(3, 20))
    assert frame.register_addr == 0x25
    assert int.from_bytes(frame.data[:2], "little") == 3
    assert not bus.write_audit
    snapshot["error_codes"][0] = 1
    with pytest.raises(ValueError, match="fault"):
        pipe.update_from_snapshot(0, snapshot)


def test_actuator_slew_saturation_prevents_integral_windup(timeline):
    drive = SixAxisForceDrive(None, ForceLimitsConfig(max_rate_g_s=(1.0,) * 6), clock=lambda: timeline[0])
    pipe = TaskSpaceForcePipeline(ForcePIDConfig(kp=1, ki=2, reference_rate_n_s=100),
                                 drive=drive, clock=lambda: timeline[0])
    pose = [0, 0, pipe.allocator.kin.nominal_height, 0, 0, 0]
    for _ in range(20):
        timeline[0] += 20_000_000
        result = pipe.update(1, [0] * 6, pose)
        assert result.actuator_limited
        assert pipe.pid.integral_n == 0
    # First command is also limited from seeded zero, not an unlimited jump.
    assert result.drive_result["dispatched_forces_g"] == pytest.approx([0.4] * 6)
    assert np.allclose(result.applied_wrench, 0)  # encoded targets still round to zero g


def test_negative_encoding_calibration_and_no_write(timeline):
    comp = ForceCompensation(direction=(-1,) * 6, newtons_per_g=(0.01,) * 6)
    assert comp.feedback_n([-100] * 6) == pytest.approx([1] * 6)
    assert comp.command_g([1] * 6) == pytest.approx([-100] * 6)
    cfg = ForceLimitsConfig(min_force_n=(-9.80665,) * 6, allow_negative_forces=True)
    bus = FakeBus(read_only=True)
    drive = SixAxisForceDrive(bus, cfg, clock=lambda: timeline[0])
    result = drive.command_forces([-100] * 6)
    raw = parse_frame(bytes.fromhex(result["frames"][0]["frame_hex"]))
    assert decode_u16_to_signed_i16(int.from_bytes(raw.data, "little")) == -100
    assert bus.write_audit == []
    with pytest.raises(RuntimeError):
        timeline[0] += 1
        drive.command_forces([0] * 6, dry_run=False)
    assert bus.write_audit == []


def test_signed_nonzero_actual_force_status_to_wrench(timeline):
    forces = [-120, 230, -340, 450, -560, 670]

    class ForceStatusBus(FakeBus):
        def request(self, frame, **kwargs):
            response = bytearray(super().request(frame, **kwargs))
            mid = parse_frame(frame).motor_id
            # 0x30 status contains the signed actual-force field for 0x2C.
            response[13:15] = forces[mid - 1].to_bytes(2, "little", signed=True)
            response[-1] = sum(response[2:-1]) & 0xFF
            return bytes(response)

    bus = ForceStatusBus(read_only=True)
    bus.positions = {axis: 1000 for axis in range(1, 7)}
    drive = SixAxisForceDrive(bus, clock=lambda: timeline[0])
    pipeline = TaskSpaceForcePipeline(drive=drive, clock=lambda: timeline[0])
    snapshot = drive.read_feedback()
    assert snapshot["force_g_raw"] == forces
    timeline[0] += 20_000_000
    result = pipeline.update_from_snapshot(0, snapshot)
    J = pipeline.allocator.compute_analytical_spatial_jacobian(result.pose)
    assert result.measured_wrench == pytest.approx(J.T @ (np.array(forces) * 0.00980665))
    assert result.pid.measured_n == pytest.approx(sum(forces) * 0.00980665)
    assert result.pid.output_n < 0  # negative feedback for positive measured Z force
    assert not bus.write_audit


def test_offline_first_order_force_plant_tracking(timeline):
    """Checks PID feedback on a synthetic plant; says nothing about hardware."""
    pid = TaskSpaceForcePID(ForcePIDConfig(kp=0.3, ki=0.6, reference_rate_n_s=2),
                           clock=lambda: timeline[0])
    measured = 0.0
    for _ in range(1000):
        timeline[0] += 10_000_000
        result = pid.update(1, measured)
        measured += 0.01 * (result.output_n - measured) / 0.1
    assert measured == pytest.approx(1.0, abs=0.002)


@pytest.mark.parametrize("overrides", [dict(reference_rate_n_s=0), dict(kp=float("nan")),
                                     dict(output_mode="position"), dict(derivative_filter_tau_s=-1)])
def test_invalid_pid_config(overrides):
    with pytest.raises(ValueError):
        ForcePIDConfig(**overrides)
