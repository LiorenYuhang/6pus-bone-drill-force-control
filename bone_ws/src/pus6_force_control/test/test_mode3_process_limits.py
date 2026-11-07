"""Only software/Mock: native-capacity policy and retained final anti-windup."""
from dataclasses import asdict, replace
import time
import numpy as np
import pytest

from pus6_force_control.joint_position_force_pid import JointPositionForcePID, JointForcePIDConfig
from pus6_force_control.mode3_process_policy import PID_OVERRIDES, force_limits
from pus6_force_control.six_axis_force_drive import SixAxisForceDrive
from pus6_motor_driver.force_units import g_to_n


def test_native_caps_optional_and_legacy_config_unchanged():
    legacy=JointForcePIDConfig(joint_integral_limit_n=(2.5,)*6,joint_increment_limit_n=(4.,)*6,
        joint_max_tracking_error_m=.0025)
    native=replace(legacy,**PID_OVERRIDES)
    assert native.joint_integral_limit_n is None and native.joint_increment_limit_n is None
    assert native.joint_max_tracking_error_m is None
    assert legacy.joint_increment_limit_n==(4.,)*6 and legacy.joint_integral_limit_n==(2.5,)*6
    assert native.joint_kp_n_m==legacy.joint_kp_n_m and native.joint_kd_n_s_m==legacy.joint_kd_n_s_m


def test_integral_above_old_cap_and_tracking_above_100_steps_not_a_fault():
    config=JointForcePIDConfig(joint_kp_n_m=(8000.,)*6,joint_ki_n_m_s=(6000.,)*6,
        joint_max_reference_speed_m_s=None,**PID_OVERRIDES)
    pid=JointPositionForcePID(config,clock=lambda:0,force_slew_compensation=False)
    pid.reset([.05]*6,sample_ns=1_000_000_000,now_ns=1_000_000_000)
    for i in range(1,61):
        now=1_000_000_000+i*20_000_000
        delta,diag=pid.update([.051]*6,[.05]*6,sample_ns=now,now_ns=now)
        pid.track_applied(delta)
    assert pid.integral[0]==pytest.approx(7.2) and delta[0]>4
    delta,diag=pid.update([.053]*6,[.05]*6,sample_ns=now+20_000_000,now_ns=now+20_000_000)
    assert diag['joint_error_m'][0]>.0025 and not diag['joint_output_limited']
    with pytest.raises(ValueError,match='mechanical'):
        pid.update([.071]*6,[.05]*6,sample_ns=now+40_000_000,now_ns=now+40_000_000)


def test_manufacturer_rating_cap_encoding_and_final_antiwindup():
    drive=SixAxisForceDrive(None,force_limits(),clock=lambda:0)
    forces=drive.command_forces_n([-100,100,0,0,0,0],dry_run=True,now_ns=1)
    assert [r['target_force_g'] for r in forces['frames']][:2]==[-5098,5098]
    assert max(abs(x) for x in g_to_n([r['target_force_g'] for r in forces['frames']]))<=50
    config=JointForcePIDConfig(joint_kp_n_m=(8000.,)*6,joint_ki_n_m_s=(6000.,)*6,
        joint_max_reference_speed_m_s=None,**PID_OVERRIDES)
    pid=JointPositionForcePID(config,clock=lambda:0,force_slew_compensation=False)
    pid.reset([.05]*6,sample_ns=1_000_000_000,now_ns=1_000_000_000)
    delta,_=pid.update([.06]*6,[.05]*6,sample_ns=1_020_000_000,now_ns=1_020_000_000)
    assert delta[0]>50
    assert all(pid.track_applied(g_to_n([5098]*6)))
    assert np.array_equal(pid.integral,np.zeros(6))  # final saturation still rejects windup


def test_recorded_minus_12n_request_reaches_native_force_frame():
    drive=SixAxisForceDrive(None,force_limits(),clock=lambda:0)
    requested=-12.428744525172478
    command=drive.command_forces_n([g_to_n(-222)+requested,0,0,0,0,0],dry_run=True,now_ns=1)
    assert command['frames'][0]['target_force_g']==-1489
    assert command['limited_forces_n'][0]==pytest.approx(g_to_n(-222)+requested)
    assert drive.limits.max_rate_g_s is None


def test_child_accepts_hold_and_force_feedback_above_old_1500g(tmp_path):
    from pus6_force_control.mode3_process_child import JointPositionServoCore
    from pus6_motor_driver.fake_bus import MockServoBus
    bus=MockServoBus(initial_positions=[1000]*6,initial_modes=[0]*6)
    bus.actual_forces={i:2000 for i in range(1,7)}
    config=dict(epoch=1,hardware_mode='mock',output_dir=str(tmp_path),execute=True,dry_run=False,
        hardware_write_enabled=True,is_hardware_qualified=True,control_mode='force',maintenance_only=True,
        force_fault_recovery='position_reanchor',max_force_handover_displacement_steps=6,max_current_ma=1500)
    core=JointPositionServoCore(config,bus)
    try:
        ready=core.initialize()
        assert ready['hold_forces_g']==[2000]*6
        config=JointForcePIDConfig(joint_kp_n_m=(8000.,)*6,joint_ki_n_m_s=(6000.,)*6,
            joint_max_reference_speed_m_s=None,**PID_OVERRIDES)
        core._enter(dict(pid_config=asdict(config)))
        core._read_positions()
        assert core.pid is not None and core.config['max_force_n']==[50.]*6
        now=time.monotonic_ns()
        core.command.latest=dict(seq=1,publish_ns=now,target_m=[.053,.05,.05,.05,.05,.05],
            target_steps=[1120,1000,1000,1000,1000,1000],target_pose_m_rad=[0]*6,
            phase='stabilize',phase_elapsed_s=1,attitude=dict(imu=dict(sample_ns=now)))
        row=core.cycle(now)['motion']
        assert row['target_force_g'][0]>1500 and row['position_error_steps'][0]>100
        assert not core.fault and core.normal_cycles==1
    finally:core.stop('normal_exit');core.close()
    assert all(r['register_address'] not in (0x20,0x21,0x22) for r in bus.write_audit)
    import json
    report=json.loads((tmp_path/'motion_child_report.json').read_text())
    assert report['all_mode0_exit_confirmed'] and report['hardware_write_count']==0
