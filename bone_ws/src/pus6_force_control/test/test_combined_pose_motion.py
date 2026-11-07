"""Combined planning and existing runner integration; no real serial access."""
from dataclasses import asdict
from functools import partial
import json
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'pus6_control'))
from pus6_force_control.combined_pose_motion import CombinedPosePlan, CombinedTrajectoryConfig, main, read_only_precheck
from pus6_force_control.combined_pose_analysis import analyze_combined_run, correlation, stats
from pus6_force_control.z_position_motion import MotionConfig, pid_config, run, slider_to_steps
from pus6_force_control.single_axis_motion import VERIFIED_PEAK_SPEED_M_S
from pus6_motor_driver.fake_bus import MockServoBus


@pytest.fixture(scope='module')
def plan():
    return CombinedPosePlan([1004, 1012, 999, 1007, 1018, 995])


def test_closed_measured_start_all_six_requested_amplitudes_and_smooth_endpoints(plan):
    assert slider_to_steps(plan.at_fraction(0)[1], plan.geometry) == pytest.approx([1004, 1012, 999, 1007, 1018, 995])
    assert np.array_equal(plan.at_fraction(1)[0], plan.initial_pose)
    assert np.array_equal(plan.at_fraction(1)[1], plan.initial_q)
    amplitudes = np.asarray(plan.trajectory.amplitudes_si)
    assert plan.amplitude_scale == 1
    assert plan.sampled_pose_delta_min == pytest.approx(-amplitudes, rel=1e-6)
    assert plan.sampled_pose_delta_max == pytest.approx(amplitudes, rel=1e-6)
    assert len(set(plan.trajectory.harmonics)) == 6
    for endpoint, direction in [(0, 1), (1, -1)]:
        h = 1e-5
        p0 = plan.at_fraction(endpoint)[0]
        p1 = plan.at_fraction(endpoint+direction*h)[0]
        p2 = plan.at_fraction(endpoint+direction*2*h)[0]
        assert np.max(np.abs((p1-p0)/h)) < 1e-9
        assert np.max(np.abs((p2-2*p1+p0)/h**2)) < 1e-5
    assert np.all(np.abs(plan.at_fraction(.371)[0]-plan.initial_pose)>1e-6)


def test_combined_ik_and_independently_sampled_speed_with_unchanged_limit(plan):
    samples = np.linspace(0, 1, 7001)
    q = []
    for u in samples:
        pose, joint = plan.at_fraction(u)
        expected = plan.initial_q+np.asarray(plan.kinematics.differential_inverse_kinematics(pose, plan.initial_pose))
        assert joint == pytest.approx(expected, abs=1e-12)
        plan.check_positions(slider_to_steps(joint, plan.geometry))
        q.append(joint)
    speed = np.abs(np.gradient(q, samples*plan.period_s, axis=0)).max()
    assert speed <= VERIFIED_PEAK_SPEED_M_S
    assert plan.period_s >= 180
    assert plan.peak_speed_m_s <= VERIFIED_PEAK_SPEED_M_S/1.019


def test_mechanical_boundary_scales_amplitude_not_protection():
    bounded = CombinedPosePlan([1000]*6, device_limits=[(970, 1030)]*6)
    assert bounded.amplitude_scale < 1
    assert np.all(bounded.path_min_steps>=970) and np.all(bounded.path_max_steps<=1030)
    assert bounded.settings.max_current_ma == 500
    with pytest.raises(ValueError, match='initial position'):
        CombinedPosePlan([1000]*6, device_limits=[(1001, 1800)]*6)


def test_pid_configuration_exactly_matches_successful_100s_z_recording(plan):
    recorded = Path(__file__).resolve().parents[4]/'results/z_position_motion/20261106_133122_655194/z_position_report.json'
    expected = json.loads(recorded.read_text())['pid']
    assert json.loads(json.dumps(asdict(pid_config(plan)))) == expected


def test_offline_entry_and_cli_permission_exclusions(monkeypatch, capsys):
    import pus6_force_control.z_position_motion as base
    def forbidden(*a, **kw):
        pytest.fail('offline entry opened hardware')
    monkeypatch.setattr(base, 'ServoCore', forbidden)
    assert main([]) == 0
    assert json.loads(capsys.readouterr().out)['hardware_write_count'] == 0
    for args in (['--hardware'], ['--hardware', '--read-only'], ['--period-seconds', 'nan']):
        with pytest.raises(SystemExit) as exc:
            main(args)
        assert exc.value.code == 2


def test_readonly_owned_port_never_opens_uart(tmp_path, monkeypatch):
    import pus6_force_control.combined_pose_motion as module
    import serial
    monkeypatch.setattr(module.subprocess, 'run', lambda *a, **kw: type('R', (), dict(returncode=0))())
    monkeypatch.setattr(serial, 'Serial', lambda *a, **kw: pytest.fail('owned port was opened'))
    result = read_only_precheck(tmp_path/'owned')
    assert not result['ready'] and result['hardware_write_count'] == 0
    assert 'owned' in result['error']


def test_numeric_summary_and_undefined_correlations():
    values = np.array([[1., -2.], [3., -4.]])
    result = stats(values)
    assert result['rmse'] == pytest.approx([np.sqrt(5), np.sqrt(10)])
    assert result['max_abs'] == [3., 4.]
    assert stats([]) is None
    a = np.tile(np.arange(4)[:, None], (1, 6)).astype(float)
    a[:, 5] = 0
    c = correlation(a, -a)
    assert c[0][0] == pytest.approx(-1)
    assert c[0][5] is None and c[5][0] is None


def test_existing_runner_combined_phases_mode0_and_offline_analysis(tmp_path):
    # Static transport, sub-encoder amplitudes: sequence test, not a plant model.
    bus = MockServoBus(initial_positions=[1000]*6, initial_modes=[0]*6)
    config = dict(epoch=1, hardware_mode='mock', output_dir=str(tmp_path), report_filename='combined_report.json',
        execute=True, dry_run=False, hardware_write_enabled=True, is_hardware_qualified=True,
        control_mode='force', maintenance_only=True, force_fault_recovery='position_reanchor',
        max_force_handover_displacement_steps=6)
    trajectory = CombinedTrajectoryConfig(amplitudes_si=(1e-9,)*6, requested_period_s=.05)
    settings = MotionConfig(hold_seconds=.02, settle_seconds=.02, settle_timeout_s=.5)
    report = run(config, settings, bus=bus, plan_factory=partial(CombinedPosePlan, trajectory=trajectory))
    assert report['outcome'] == 'motion_complete'
    assert report['hardware_write_count'] == 0
    assert report['all_mode3_confirmed'] and report['all_mode0_exit_confirmed'] and report['serial_closed']
    assert [s['phase'] for s in report['stages']] == ['initial_hold', 'combined_motion', 'final_hold']
    assert list(bus.modes.values()) == [0]*6
    summary = analyze_combined_run(tmp_path)
    assert summary['source'] == 'MOCK'
    assert summary['mode0_return_error_mm_deg'] == pytest.approx([0]*6, abs=1e-6)
    assert summary['pid_increment_clipped_rows_per_axis'] == [0]*6
    for file in ('summary.json', 'summary.md', 'pose_tracking.png', 'joint_tracking.png', 'force_pid_current.png', 'error_correlations.png'):
        assert (tmp_path/file).stat().st_size > 0
    # A real early fault may have no trajectory rows/plan. Still summarize truthfully.
    fault = tmp_path/'early_fault'; fault.mkdir()
    (fault/'combined_report.json').write_text(json.dumps(dict(source='MOCK', outcome='fault', error='test before plan')))
    result = analyze_combined_run(fault)
    assert result['pose_error_motion_mm_deg'] is None
    assert result['mode0_return_error_mm_deg'] is None
    assert result['fk_valid_rows'] == 0
