"""Stage 3C.2 signed hold + SI increments, offline only (no serial)."""
import numpy as np
import pytest
from pus6_force_control.force_pipeline import TaskSpaceForcePipeline, ForceCompensation
from pus6_force_control.task_space_force_pid import ForcePIDConfig
from pus6_motor_driver.force_units import g_to_n

HOLD = [-17, -187, -66, -30, 20, 0]


def pipeline(**kwargs):
    p = TaskSpaceForcePipeline(ForcePIDConfig(kp=1, reference_rate_n_s=1),
                              clock=lambda: 1_000_000_000, **kwargs)
    p.initialize_hold_forces_g(HOLD)
    return p, [0, 0, p.allocator.kin.nominal_height, 0, 0, 0]


def test_capture_copies_signed_feedback_and_zero_increment_is_native_hold():
    p, pose = pipeline()
    assert p.hold_forces_g == tuple(HOLD)
    assert p.relative_feedback_n(HOLD) == pytest.approx([0]*6)
    result = p.update(0, HOLD, pose, now_ns=1_020_000_000)
    assert result.pid.output_n == 0
    assert result.measured_wrench == pytest.approx([0]*6)
    assert [f['target_force_g'] for f in result.drive_result['frames']] == HOLD
    assert result.applied_wrench == pytest.approx([0]*6)
    assert p.drive.last_commanded_forces_g == pytest.approx(HOLD)
    with pytest.raises(RuntimeError, match='already captured'):
        p.initialize_hold_forces_g([0]*6)


@pytest.mark.parametrize('sign', [1, -1])
def test_pid_delta_allocation_then_hold_addition_and_native_encoding(sign):
    p, pose = pipeline()
    t = 2_000_000_000
    r = p.update(sign, HOLD, pose, now_ns=t)
    assert r.pid.output_n == 2*sign  # unchanged reference-plus-P implementation
    expected_delta = p.allocator.allocate([0,0,2*sign,0,0,0],pose).actuator_forces_n
    assert r.allocation.actuator_forces_n == pytest.approx(expected_delta)
    assert r.drive_result['requested_forces_n'] == pytest.approx(np.array(g_to_n(HOLD))+expected_delta)
    written = [f['target_force_g'] for f in r.drive_result['frames']]
    assert np.asarray(g_to_n(written))-g_to_n(HOLD) == pytest.approx(expected_delta,abs=.004904)
    assert np.sign(r.applied_wrench[2]) == sign


def test_relative_wrench_removes_only_initial_hold_and_retains_sign():
    p, pose = pipeline()
    delta_g=[10,-20,30,-40,50,-60]
    raw=np.asarray(HOLD)+delta_g
    r=p.update(0,raw,pose,now_ns=1_020_000_000)
    j=p.allocator.compute_analytical_spatial_jacobian(pose)
    assert r.measured_wrench == pytest.approx(j.T@np.array(g_to_n(delta_g)))
    assert p.hold_forces_g == tuple(HOLD)


def test_limit_applies_to_absolute_hold_plus_delta_and_slew_starts_at_hold():
    p, pose = pipeline()
    r=p.update(10,HOLD,pose,now_ns=1_020_000_000)
    assert np.max(np.abs(np.array(r.drive_result['dispatched_forces_g'])-HOLD))<=10+1e-9
    with pytest.raises(ValueError, match='absolute'):
        TaskSpaceForcePipeline().initialize_hold_forces_g([1501]*6)
    # Near-boundary hold + negative delta must saturate the TOTAL, not delta alone.
    p=TaskSpaceForcePipeline(clock=lambda:1_000_000_000)
    p.initialize_hold_forces_g([-1499]*6)
    r=p.update(-10,[-1499]*6,pose,now_ns=20_000_000_000)
    assert min(r.drive_result['requested_forces_n']) < -14.709975
    assert min(r.drive_result['limited_forces_n']) == pytest.approx(-14.709975)
    assert r.actuator_limited


def test_cannot_double_subtract_software_zero_or_support():
    for c in (ForceCompensation(sensor_zero_g=(1,)*6),ForceCompensation(actuator_bias_n=(1,)*6)):
        with pytest.raises(ValueError,match='subtract twice'):
            TaskSpaceForcePipeline(compensation=c).initialize_hold_forces_g(HOLD)
    p,pose=pipeline()
    with pytest.raises(ValueError,match='no extra'):
        p.update(0,HOLD,pose,gravity_wrench=[0,0,1,0,0,0],now_ns=1_020_000_000)


def test_node_reset_keeps_hold_and_pid_initialization_does_not_zero_motor_target():
    import rclpy
    from pus6_force_control.force_control_node import TaskSpaceForceControlNode
    rclpy.init();node=TaskSpaceForceControlNode('test_stage3c2_hold')
    try:
        node.timer.cancel();node.initialize_hold_forces_g(HOLD)
        node.pipeline.pid.integral_n=2
        node._reset_control_state(node.clock())
        assert node.pipeline.pid.integral_n==0
        assert node.pipeline.hold_forces_g==tuple(HOLD)
        assert node.drive.last_commanded_forces_g==HOLD
        assert node.last_axial_command_n==pytest.approx(g_to_n(HOLD))
    finally:node.destroy_node();rclpy.shutdown()


def test_hold_only_profile_has_no_force_steps():
    from pus6_force_control.stage3c_experiment import response_stages
    assert response_stages({'hold_only':True,'graded_force_response':True})==[('zero_native',0.,2.5)]


def test_real_snapshot_status_updates_existing_scheduler_without_reset_or_timeout_change(tmp_path):
    import time
    from pus6_motor_driver.j14_process_uart import ServoCore
    from pus6_motor_driver.fake_bus import MockServoBus
    from pus6_motor_driver.motor import Motor
    from pus6_force_control.stage3c_experiment import ingest_snapshot_feedback
    bus=MockServoBus();bus.modes={i:0 for i in range(1,7)}
    core=ServoCore(dict(epoch=1,output_dir=str(tmp_path),hardware_mode='mock',
        maintenance_only=True,control_mode='force',force_fault_recovery='position_reanchor'),bus=bus)
    try:
        core.initialize()
        threshold=core.feedback.hard_stop_feedback_age_ns
        base=time.monotonic_ns()
        for seq in range(1,7):
            # More than 1s overall, but actual snapshots continue every 0.2s.
            stamp=base+seq*200_000_000
            core.feedback._clock_ns=lambda:stamp
            status={i:Motor(bus,i).read_status() for i in range(1,7)}
            rx={i:stamp-1_000_000+i for i in range(1,7)}
            ingest_snapshot_feedback(core,status,rx,stamp-20_000_000,seq)
            assert core.rx_ns==list(rx.values())
            assert not core.feedback.stopped
        assert core.feedback.hard_stop_feedback_age_ns==threshold
        assert not bus.write_audit
    finally:core.rec._dumped=True
