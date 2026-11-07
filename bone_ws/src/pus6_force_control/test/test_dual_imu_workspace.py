"""Three-axis workspace boundaries and measured-run analysis; no hardware."""
import json
from pathlib import Path

import numpy as np
import pytest

from pus6_force_control.mode3_dual_imu import DualImuPlan, SyntheticImuSource
from pus6_force_control.dual_imu_attitude import load_attitude_parameters
from pus6_force_control.dual_imu_workspace_validation import compensation_pose
from pus6_force_control.dual_imu_analysis import analyze_run
from pus6_force_control.z_position_motion import slider_to_steps

ROOT = Path(__file__).resolve().parents[4]


def make_plan(device_limits=None):
    return DualImuPlan([1032,1085,1069,954,910,931], source=SyntheticImuSource(),
        parameters=load_attitude_parameters(), axes=('roll','pitch','yaw'), device_limits=device_limits)


def test_absolute_workspace_and_exact_three_axis_targets_have_no_hidden_scale():
    p = make_plan()
    assert p.target_min_steps == pytest.approx([200]*6)
    assert p.target_max_steps == pytest.approx([1800]*6)
    assert p.envelope_min_steps == pytest.approx([200]*6)
    assert p.envelope_max_steps == pytest.approx([1800]*6)
    # Concurrent large R/P/Y, both signs; no single-axis experiment.
    for degrees in ((20,20,20),(-20,20,-20),(20,-20,-20),(-20,-20,20),(32,5,8),(5,5,45)):
        raw = compensation_pose(p,degrees)
        pose,q,d = p.constrain_pose(raw)
        assert pose == pytest.approx(raw,abs=0)
        assert d['target_reachable'] and not d['ik_saturated'] and d['rotation_scale'] == 1
        assert max(abs(slider_to_steps(q,p.geometry)-slider_to_steps(p.initial_q,p.geometry)))>100
        assert min(d['mechanical_remaining_lower_steps']) >= 0
        assert min(d['mechanical_remaining_upper_steps']) >= 0
    assert p.joint_reference_speed_m_s is None and p.force_limits().max_rate_g_s is None


def test_unreachable_target_fault_is_recorded_and_distinct_from_actual_guard():
    p = make_plan()
    raw = compensation_pose(p,(-50,10,10))
    with pytest.raises(RuntimeError,match='IK_TARGET_WORKSPACE_BOUND'):
        p.constrain_pose(raw)
    d = p.target_request
    assert d['saturated'] and not d['target_reachable'] and d['bounded_pose_m_rad'] is None
    assert d['action'] == 'reject_before_force_write'
    assert not p.workspace_result('motion_complete')['full_target_motion_completed']
    p.check_positions([1699]*6)  # actual can use independent overshoot space
    assert p.control_diagnostics['actual_workspace']['mechanical_remaining_upper_steps'] == [101]*6
    p.check_positions([1701]*6)  # no duplicate conservative inner workspace
    with pytest.raises(RuntimeError,match='ACTUAL_POSITION_MECHANICAL_BOUND'):p.check_positions([1801]*6)
    # The previous fault at M2=1084 is normal travel, not a new larger +/-box.
    p.check_positions([1032,1084,1069,954,910,931])


def test_device_limits_remain_authoritative_and_start_must_fit():
    p = make_plan([(250,1750)]*6)
    assert p.target_min_steps == pytest.approx([250]*6)
    assert p.target_max_steps == pytest.approx([1750]*6)
    with pytest.raises(RuntimeError,match='MECHANICAL'):p.check_positions([1751]*6)
    narrow = make_plan([(900,1100)]*6)
    assert narrow.target_min_steps == pytest.approx([900]*6)


def test_analysis_uses_measured_world_error_and_preserves_fault(tmp_path,monkeypatch):
    import serial
    monkeypatch.setattr(serial,'Serial',lambda *a,**kw:pytest.fail('analysis opened serial'))
    history = ROOT/'results/mode3_dual_imu/run_20261106_161531_093605'
    s = analyze_run(history,tmp_path/'analysis')
    assert s['analysis_hardware_write_count'] == 0 and s['hardware_write_count'] == 5718
    assert s['outcome'] == 'fault' and 'ACTUAL_POSITION_ENVELOPE_BOUND' in s['error']
    assert s['all_mode0_exit_confirmed'] and s['serial_closed']
    assert not s['full_unscaled_target_completed']
    assert s['base_peak_abs_deg'][1] == pytest.approx(12.292519,abs=1e-5)
    assert s['world_error_max_abs_deg'][1] == pytest.approx(6.597910,abs=1e-5)
    assert s['actual_max_steps'][1] == 1084  # includes the fault feedback not in tracking
    assert s['actual_remaining_upper_steps'][1] == 716
    assert s['stability_verdict'] == 'NOT_ESTABLISHED'
    assert (tmp_path/'analysis/force.png').exists()
    assert json.loads((tmp_path/'analysis/analysis.json').read_text())['fault_timestamp_ns']==111402529236985
    with pytest.raises(FileExistsError):analyze_run(history,tmp_path/'analysis')
