"""Direct references, bounded SO(3) targets and captured fault state. No UART."""
from dataclasses import replace
from functools import partial
import json
from pathlib import Path
import sys

import numpy as np
import pytest
from scipy.spatial.transform import Rotation
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'pus6_control'))
from pus6_force_control.mode3_dual_imu import DualImuPlan, SyntheticImuSource
from pus6_force_control.dual_imu_replay import replay_history
from pus6_force_control.joint_position_force_pid import JointPositionForcePID
from pus6_force_control.z_position_motion import ZPlan, MotionConfig, pid_config, slider_to_steps, run
from pus6_motor_driver.fake_bus import MockServoBus

ROOT = Path(__file__).resolve().parents[4]
HISTORY = ROOT/'results/mode3_dual_imu/run_20261106_153217_939698'


@pytest.fixture
def plan():
    parameters = yaml.safe_load((ROOT/'bone_ws/src/pus6_force_control/config/mode3_dual_imu.yaml').read_text())['attitude']
    return DualImuPlan([995, 993, 991, 988, 993, 997], source=SyntheticImuSource(), parameters=parameters, axes=('roll', 'pitch'))


def test_direct_ik_reference_has_no_limiter_but_retains_tracking_guard_and_pid(plan):
    config = pid_config(plan)
    assert config.joint_max_reference_speed_m_s is None
    assert pid_config(ZPlan([1000]*6)).joint_max_reference_speed_m_s == .0002
    ns = 1_000_000_000
    pid = JointPositionForcePID(config, clock=lambda: ns)
    pid.reset(plan.initial_q, sample_ns=ns, now_ns=ns, position_stamps_ns=[ns]*6)
    assert pid.reference is None
    target = plan.initial_q+np.array([.001, -.001, .0005, -.0005, .0015, -.0015])
    ns += 20_000_000
    _, diag = pid.update(target, plan.initial_q, sample_ns=ns, now_ns=ns, position_stamps_ns=[ns]*6)
    assert diag['joint_reference_m'] == pytest.approx(target, abs=0)
    assert diag['joint_error_m'] == pytest.approx(target-plan.initial_q)
    assert max(abs(np.array(diag['joint_delta_force_n']))) <= 4
    ns += 20_000_000
    with pytest.raises(ValueError, match='tracking error'):
        pid.update(plan.initial_q+.002501, plan.initial_q, sample_ns=ns, now_ns=ns, position_stamps_ns=[ns]*6)
    with pytest.raises(ValueError):
        replace(config, joint_max_reference_speed_m_s=0.)
    with pytest.raises(ValueError):
        replace(config, joint_max_reference_speed_m_s=float('inf'))


def test_continuous_spatial_target_constraint_exact_ik_and_fixed_xyz_yaw(plan):
    previous = None
    bounded_end = None
    for degrees in np.linspace(0, 12, 241):
        raw = plan.initial_pose.copy(); raw[3:5] += np.deg2rad([-degrees, -.65*degrees])
        pose, q, diag = plan.constrain_pose(raw)
        steps = slider_to_steps(q, plan.geometry)
        assert (steps >= plan.envelope_min_steps).all() and (steps <= plan.envelope_max_steps).all()
        assert pose[:3] == pytest.approx(plan.initial_pose[:3], abs=0)
        assert pose[5] == plan.initial_pose[5]
        expected = plan.initial_q+np.asarray(plan.kinematics.differential_inverse_kinematics(pose, plan.initial_pose))
        assert q == pytest.approx(expected, abs=1e-12)
        if previous is not None:
            assert np.max(np.abs(pose[3:]-previous[3:])) < np.deg2rad(.051)
        previous = pose
        if degrees == 0: assert not diag['saturated']
        bounded_end = (pose, q, diag)
    assert not bounded_end[2]['saturated']
    assert bounded_end[2]['rotation_scale'] == 1
    # Projection has no elapsed-time state or hidden slew; repeated calls exact.
    again = plan.constrain_pose(raw)
    assert again[0] == pytest.approx(bounded_end[0], abs=0)
    with pytest.raises(ValueError, match='IK_TARGET_INVALID'):
        plan.constrain_pose([float('nan')]*6)


def test_target_saturation_and_actual_workspace_and_mechanical_reasons_separate(plan):
    raw = plan.initial_pose.copy(); raw[3] += np.deg2rad(70)
    with pytest.raises(RuntimeError, match='IK_TARGET_'):
        plan.constrain_pose(raw)
    assert not plan.target_request['target_reachable']
    assert plan.target_request['action'] == 'reject_before_force_write'
    assert plan.target_request['bounded_pose_m_rad'] is None
    with pytest.raises(RuntimeError, match='ACTUAL_POSITION_MECHANICAL_BOUND'):
        plan.check_positions(plan.envelope_max_steps+1)
    with pytest.raises(RuntimeError, match='ACTUAL_POSITION_MECHANICAL_BOUND'):
        plan.check_positions([1801]*6)
    start = slider_to_steps(plan.initial_q, plan.geometry)
    narrow = DualImuPlan(start, source=SyntheticImuSource(), parameters=plan.controller.parameters,
        axes=('roll', 'pitch'), device_limits=[(s-2, s+2) for s in start])
    assert narrow.target_min_steps == pytest.approx(start-2)
    with pytest.raises(RuntimeError,match='IK_TARGET_'): narrow.constrain_pose(raw)


def test_historical_last_target_extension_no_longer_hits_tiny_start_envelope(plan):
    row = json.loads((HISTORY/'tracking.jsonl').read_text().splitlines()[-1])
    raw = np.asarray(row['target_pose_m_rad']); raw[3:5] = plan.initial_pose[3:5]+1.03*(raw[3:5]-plan.initial_pose[3:5])
    pose, q, diag = plan.constrain_pose(raw)
    assert diag['target_reachable'] and not diag['saturated'] and diag['rotation_scale'] == 1
    plan.check_positions(row['actual_positions_steps'])
    assert np.max(slider_to_steps(q, plan.geometry)-plan.target_max_steps) <= 0


@pytest.mark.parametrize('problem', ['imu', 'tracking', 'actual', 'ik'])
def test_fault_snapshot_preserves_pending_and_last_complete_frames_before_recovery(tmp_path, plan, problem):
    class Source(SyntheticImuSource):
        count = 0
        def activate(self, elapsed):
            self.count += 1
            if problem == 'imu' and self.count == 3:
                raise ValueError('platform IMU orientation stale')
            super().activate(elapsed)
    source = Source(duration_s=.15, amplitude_deg=.005)
    class Plan(DualImuPlan):
        active_count = 0
        def target(self, phase, elapsed_s):
            result = super().target(phase, elapsed_s)
            if phase == 'stabilize':
                self.active_count += 1
                if problem == 'ik' and self.active_count == 3:
                    pose = self.initial_pose.copy(); pose[3:6] += np.deg2rad([60,20,20])
                    return self.constrain_pose(pose)[:2]
                if problem == 'tracking' and self.active_count == 3:
                    pose = self.initial_pose.copy(); pose[3] += .2
                    pose, q, d = self.constrain_pose(pose)
                    self.control_diagnostics['target_constraint'] = d
                    return pose, q
            return result
    bus = MockServoBus(initial_positions=[1000]*6, initial_modes=[0]*6)
    original_write = bus.write
    def write(frame):
        from pus6_motor_driver.protocol import parse_frame
        parsed = parse_frame(frame)
        if problem == 'actual' and source.count >= 3 and parsed.register_addr == 0x27:
            bus.positions[2] = 1801
        if problem == 'tracking' and source.count == 2 and parsed.register_addr == 0x27:
            # Delivered by actual Mock status responses before the next PID
            # update. A +100-step target then has >100-step actual error.
            bus.positions = {i: 998 for i in range(1, 7)}
        return original_write(frame)
    bus.write = write
    config = dict(epoch=1, hardware_mode='mock', output_dir=str(tmp_path), report_filename='dual_imu_report.json',
        execute=True, dry_run=False, hardware_write_enabled=True, is_hardware_qualified=True,
        control_mode='force', maintenance_only=True, force_fault_recovery='position_reanchor', max_force_handover_displacement_steps=6)
    factory = partial(Plan, source=source, parameters=plan.controller.parameters, duration_s=.15)
    report = run(config, MotionConfig(hold_seconds=.02, settle_seconds=.02, settle_timeout_s=.5), bus=bus, plan_factory=factory)
    if problem == 'tracking' and report['outcome'] != 'fault':
        pytest.fail('tracking fault injection must reach existing error guard')
    assert report['outcome'] == 'fault'
    assert report['hardware_write_count'] == 0 and report['all_mode0_exit_confirmed']
    snapshot = json.loads((tmp_path/'fault_snapshot.json').read_text())
    assert snapshot['fault_timestamp_ns'] == report['fault_timestamp_ns']
    assert snapshot['last_complete_frame']['pid_output_g'] is not None
    assert snapshot['last_complete_frame']['attitude']['imu'] is not None
    assert len(snapshot['latest_feedback']['actual_positions_steps']) == 6
    assert snapshot['serial_closed'] and snapshot['all_mode0_exit_confirmed']
    if problem == 'imu': assert snapshot['pending_cycle']['stage'] == 'target_generation'
    if problem == 'ik':
        assert snapshot['pending_cycle']['stage'] == 'target_generation'
        assert snapshot['pending_cycle']['pid'] is None
        assert snapshot['target_request']['target_reachable'] is False
        assert snapshot['target_request']['bounded_pose_m_rad'] is None
        assert report['workspace_result']['unreachable_targets'] == 1
        assert not report['workspace_result']['full_target_motion_completed']
        assert len([w for w in bus.write_audit if w['register_address']==0x27]) == report['control_cycles']*6
    if problem == 'actual': assert 'ACTUAL_POSITION_MECHANICAL_BOUND' in snapshot['error']


def test_historical_replay_never_opens_serial_and_reference_matches_latest_target(tmp_path, monkeypatch):
    import serial
    monkeypatch.setattr(serial, 'Serial', lambda *a, **kw: pytest.fail('offline replay opened serial'))
    report = replay_history(HISTORY, tmp_path/'replay')
    assert report['hardware_write_count'] == 0
    assert report['replayed_frames'] == report['input_frames'] == 419
    assert report['fault'] is None
    assert report['max_reference_minus_bounded_ik_m'] == 0
    assert max(report['per_axis_peak_reference_speed_mm_s']) > 2
    assert report['pid']['joint_max_reference_speed_m_s'] is None
    assert report['force_policy']['normal_max_rate_g_s'] is None
    assert report['force_policy']['pid_slew_delay_compensation'] is False
    assert report['max_normal_force_shaping_error_g'] == 0
    assert report['comparison']['optimized']['peak_d_n'][0] < report['comparison']['baseline']['peak_d_n'][0]
    assert max(report['target_force_max_g']) <= 1500
