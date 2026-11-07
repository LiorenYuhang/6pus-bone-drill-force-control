"""Continuous measured velocity, filtered D and conditional integration. No UART."""
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from pus6_force_control.mode3_dual_imu import DualImuPlan, DualImuMotionConfig, SyntheticImuSource
from pus6_force_control.dual_imu_attitude import load_attitude_parameters
from pus6_force_control.joint_position_force_pid import JointPositionForcePID
from pus6_force_control.z_position_motion import MotionConfig, ZPlan, pid_config
from pus6_force_control.dual_imu_replay import replay_history
from pus6_motor_driver.safety import MAX_CURRENT_MA

ROOT = Path(__file__).resolve().parents[4]


def config():
    return pid_config(DualImuPlan([1000]*6,source=SyntheticImuSource(),parameters=load_attitude_parameters()))


def controller():
    p=JointPositionForcePID(config(),clock=lambda:0,force_slew_compensation=False)
    p.reset(np.full(6,.05),sample_ns=1_000_000_000,now_ns=1_000_000_000)
    return p


def test_one_step_motion_is_not_cut_off_and_filtered_velocity_decays_at_rest():
    p=controller(); target=np.full(6,.051)
    _,d=p.update(target,np.full(6,.050025),sample_ns=1_020_000_000,now_ns=1_020_000_000)
    _,d=p.update(target,np.full(6,.050025),sample_ns=1_040_000_000,now_ns=1_040_000_000)
    # Only one encoder step in the window: old 1.5-step span gate forced zero.
    assert min(d['joint_velocity_raw_m_s'])>0
    assert min(d['joint_velocity_m_s'])>0
    previous=None
    for k in range(3,12):
        ns=1_000_000_000+k*20_000_000
        _,d=p.update(target,np.full(6,.050025),sample_ns=ns,now_ns=ns)
        if k>=7:
            velocity=np.array(d['joint_velocity_m_s'])
            assert (velocity>0).all()
            if previous is not None: assert (velocity<previous).all()
            previous=velocity
    assert np.array(d['joint_d_n']) == pytest.approx(-1000*np.array(d['joint_velocity_m_s']))


def test_duplicate_axis_feedback_does_not_advance_velocity_filter():
    p=controller()
    p.update(np.full(6,.051),np.full(6,.050025),sample_ns=1_020_000_000,now_ns=1_020_000_000)
    p.update(np.full(6,.051),np.full(6,.05005),sample_ns=1_040_000_000,now_ns=1_040_000_000)
    before=p.velocity[0]
    q=np.full(6,.050075);q[0]=.05005
    _,d=p.update(np.full(6,.051),q,sample_ns=1_060_000_000,now_ns=1_060_000_000,
        position_stamps_ns=[1_040_000_000]+[1_060_000_000]*5)
    assert d['joint_velocity_filter_alpha'][0]==0
    assert p.velocity[0]==before
    with pytest.raises(ValueError,match='stale'):
        p.update(np.full(6,.051),q,sample_ns=1_080_000_000,now_ns=1_300_000_000,
            position_stamps_ns=[1_060_000_000]*6)


def test_reference_step_is_direct_and_has_no_derivative_kick_or_force_ramp():
    p=controller();target=np.full(6,.051)
    u,d=p.update(target,np.full(6,.05),sample_ns=1_020_000_000,now_ns=1_020_000_000)
    assert d['joint_reference_m']==pytest.approx(target,abs=0)
    assert d['joint_d_n']==pytest.approx([0]*6)
    assert u==pytest.approx([4]*6)
    assert all(d['joint_integral_blocked_axes'])
    assert p.integral==pytest.approx([0]*6)
    with pytest.raises(ValueError,match='tracking error'):
        p.update(np.full(6,.052501),np.full(6,.05),sample_ns=1_040_000_000,now_ns=1_040_000_000)


def test_integral_unwinds_while_moving_and_does_not_wind_up_against_applied_cap():
    p=controller();p.integral[:]=2
    for k in range(1,5):
        q=np.full(6,.05+k*.000025);ns=1_000_000_000+k*20_000_000
        u,d=p.update(q-.0001,q,sample_ns=ns,now_ns=ns)
        p.track_applied(u)
    assert min(p.velocity)>0
    assert max(p.integral)<2  # moving no longer freezes the trim/unwind path
    p=controller()
    u,d=p.update(np.full(6,.0501),np.full(6,.05),sample_ns=1_020_000_000,now_ns=1_020_000_000)
    assert min(p.integral)>0
    p.track_applied(np.full(6,.1))
    assert p.integral==pytest.approx([0]*6)  # downstream applied force anti-windup


def test_shared_safety_baseline_and_legacy_motion_defaults():
    assert DualImuMotionConfig().max_current_ma==MAX_CURRENT_MA==1500
    assert MotionConfig().max_current_ma==500
    p=DualImuPlan([1000]*6,source=SyntheticImuSource(),parameters=load_attitude_parameters())
    assert p.target_min_steps==pytest.approx(p.lower_steps,abs=0)
    assert p.target_max_steps==pytest.approx(p.upper_steps,abs=0)
    assert p.envelope_min_steps==pytest.approx(p.lower_steps,abs=0)
    old=pid_config(ZPlan([1000]*6));new=pid_config(p)
    assert old.joint_kd_n_s_m==(5000.,)*6 and not old.joint_continuous_velocity
    assert new.joint_kp_n_m==old.joint_kp_n_m and new.joint_ki_n_m_s==old.joint_ki_n_m_s
    assert new.joint_kd_n_s_m==(1000.,)*6
    assert new.joint_max_tracking_error_m==old.joint_max_tracking_error_m==.0025
    assert p.force_limits().max_rate_g_s is None and p.joint_reference_speed_m_s is None


def test_last_experiment_replay_smoother_d_and_force_but_same_recorded_guard(tmp_path,monkeypatch):
    import serial
    monkeypatch.setattr(serial,'Serial',lambda *a,**kw:pytest.fail('replay opened hardware'))
    r=replay_history(ROOT/'results/mode3_dual_imu/run_20261106_175818_903202',tmp_path/'replay')
    assert r['hardware_write_count']==0 and r['completed_input_frames']==515 and r['input_frames']==516
    assert r['replayed_frames']==515 and r['fault']['recorded_fault_frame']
    assert 'tracking error' in r['fault']['error']
    assert r['max_reference_minus_bounded_ik_m']==r['max_normal_force_shaping_error_g']==0
    old=r['comparison']['baseline'];new=r['comparison']['optimized']
    assert old['output_clipped_frames']==132
    assert max(new['peak_d_n'])<5 and max(old['peak_d_n'])>29
    assert sum(old['force_jumps_at_least_800g'])==21 and sum(new['force_jumps_at_least_800g'])==0
    assert sum(new['target_force_total_variation_g'])<sum(old['target_force_total_variation_g'])/2
    assert new['output_clipped_frames']<old['output_clipped_frames']
