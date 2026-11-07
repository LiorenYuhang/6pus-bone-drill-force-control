"""Trajectory and shared native-force runner tests, no real serial access."""
from dataclasses import asdict
from functools import partial
from pathlib import Path
import json
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'pus6_control'))
from pus6_force_control.single_axis_motion import SingleAxisPlan, YawHoldContinuationPlan, EXPERIMENT_AXES, VERIFIED_PEAK_SPEED_M_S, analyze_run, run_suite
from pus6_force_control.z_position_motion import MotionConfig, pid_config, run, slider_to_steps
from pus6_motor_driver.fake_bus import MockServoBus


@pytest.mark.parametrize('axis', EXPERIMENT_AXES)
def test_full_positive_negative_return_ik_and_speed(axis):
    initial = [995, 993, 993, 992, 993, 995]
    amplitude = .005 if axis in ('x', 'y') else np.deg2rad(2.)
    plan = SingleAxisPlan(initial, axis=axis, amplitude=amplitude)
    assert plan.amplitude == amplitude
    assert plan.peak_speed_m_s <= VERIFIED_PEAK_SPEED_M_S
    other = [i for i in range(6) if i != plan.axis_index]
    for phase, duration in plan.phase_schedule:
        if duration is None:
            continue
        q_values = []
        time_values = np.linspace(0., duration, 2001)
        for t in time_values:
            pose, q = plan.target(phase, t)
            assert pose[other] == pytest.approx(plan.initial_pose[other])
            assert q == pytest.approx(plan.initial_q+np.asarray(plan.kinematics.differential_inverse_kinematics(pose, plan.initial_pose)))
            plan.check_positions(slider_to_steps(q, plan.geometry))
            q_values.append(q)
        assert abs(np.gradient(q_values, time_values, axis=0)).max() <= VERIFIED_PEAK_SPEED_M_S
    assert plan.target('positive_hold', 1.)[0][plan.axis_index] == pytest.approx(plan.initial_pose[plan.axis_index]+amplitude)
    assert plan.target('negative_hold', 1.)[0][plan.axis_index] == pytest.approx(plan.initial_pose[plan.axis_index]-amplitude)
    assert slider_to_steps(plan.target('final_hold', 1.)[1], plan.geometry) == pytest.approx(initial)
    # Explicitly preserve the complete successful hardware PID configuration.
    folder = Path(__file__).resolve().parents[4]/'results/z_position_motion/20261106_133122_655194'
    if (folder/'z_position_report.json').exists():
        baseline = json.loads((folder/'z_position_report.json').read_text())
        assert json.loads(json.dumps(asdict(pid_config(plan)))) == baseline['pid']


def test_amplitude_reduction_preserves_device_stroke_and_initial_feedback():
    plan = SingleAxisPlan([1780]*6, axis='x', amplitude=.005, device_limits=[(200, 1800)]*6)
    assert 0 < plan.amplitude < .005
    assert plan.path_min_steps.min() >= 200
    assert plan.path_max_steps.max() <= 1800+1e-7
    assert slider_to_steps(plan.initial_q, plan.geometry) == pytest.approx([1780]*6)
    with pytest.raises(ValueError, match='initial actual position'):
        SingleAxisPlan([1900]*6, axis='x', amplitude=.005)


def test_shared_runner_all_phases_analysis_and_mode0(tmp_path):
    config = dict(epoch=1, hardware_mode='mock', output_dir=str(tmp_path), execute=True, dry_run=False,
        hardware_write_enabled=True, is_hardware_qualified=True, control_mode='force', maintenance_only=True,
        force_fault_recovery='position_reanchor', max_force_handover_displacement_steps=6)
    settings = MotionConfig(move_seconds=.05, hold_seconds=.02, settle_seconds=.02, settle_timeout_s=.5)
    # Existing static bus, sub-step pose request: flow validation only.
    bus = MockServoBus(initial_positions=[1000]*6, initial_modes=[0]*6)
    report = run(config, settings, bus=bus, plan_factory=partial(SingleAxisPlan, axis='x', amplitude=1e-7))
    assert report['outcome'] == 'motion_complete'
    assert report['all_mode0_exit_confirmed']
    assert report['hardware_write_count'] == 0
    assert [p['phase'] for p in report['stages']] == ['initial_hold', 'positive_move', 'positive_hold',
        'negative_move', 'negative_hold', 'return_to_start', 'final_hold']
    result = analyze_run(tmp_path)
    assert result['axis'] == 'x' and result['all_mode0_exit_confirmed']
    assert result['mode_loss_rows'] == result['fault_rows'] == result['force_readback_mismatches'] == 0
    assert result['cross_motion_peak_mm_deg'] == pytest.approx([0]*6)
    assert result['mode0_fk_converged']
    assert result['mode0_return_error_mm_deg'] == pytest.approx([0]*6)


def test_suite_reads_each_result_before_next_axis_and_stops_on_failure(tmp_path, monkeypatch):
    import pus6_force_control.single_axis_motion as module
    events = []
    def fake_run(config, *a, **kw):
        axis = Path(config['output_dir']).name
        events.append(('run', axis))
        Path(config['output_dir']).mkdir()
        (Path(config['output_dir'])/'z_position_report.json').write_text(json.dumps(dict(
            source='MOCK', outcome='motion_complete' if axis == 'x' else 'fault',
            plan=dict(axis=axis), all_mode0_exit_confirmed=True)))
    original = module.analyze_run
    def analyze(path):
        events.append(('analyze', Path(path).name))
        return original(path)
    monkeypatch.setattr(module, 'run', fake_run)
    monkeypatch.setattr(module, 'analyze_run', analyze)
    result = run_suite(tmp_path, {})
    assert len(result) == 2
    assert events == [('run', 'x'), ('analyze', 'x'), ('run', 'y'), ('analyze', 'y')]
    assert not (tmp_path/'roll').exists()


def test_yaw_timeout_continuation_starts_measured_and_returns_original_origin():
    original = SingleAxisPlan([997, 994, 992, 989, 994, 997], axis='yaw', amplitude=np.deg2rad(2.))
    reference = dict(plan=original.summary(), all_mode0_exit_confirmed=True,
                     error='RuntimeError: negative_hold did not settle/hold within timeout')
    actual = [993, 1000, 989, 993, 989, 1003]
    plan = YawHoldContinuationPlan(actual, MotionConfig(settle_timeout_s=40.), reference_report=reference)
    assert slider_to_steps(plan.target('initial_hold', 0)[1], plan.geometry) == pytest.approx(actual)
    assert plan.target('negative_hold', 0)[1] == pytest.approx(original.target('negative_hold', 0)[1])
    assert slider_to_steps(plan.target('return_to_start', 60)[1], plan.geometry) == pytest.approx([997, 994, 992, 989, 994, 997])
    assert plan.target('final_hold', 0)[0] == pytest.approx(original.initial_pose)
    assert pid_config(plan) == pid_config(original)
    assert [phase for phase, _ in plan.phase_schedule] == ['initial_hold', 'negative_hold', 'return_to_start', 'final_hold']
    with pytest.raises(ValueError, match='confirmed Mode0'):
        YawHoldContinuationPlan(actual, reference_report=dict(reference, all_mode0_exit_confirmed=False))
