"""Offline tests of source-time velocity and explicitly configured braking slew."""
import numpy as np
import pytest
from dataclasses import replace
from pus6_force_control.joint_position_force_pid import JointForcePIDConfig,JointPositionForcePID
from pus6_force_control.six_axis_force_drive import SixAxisForceDrive,ForceLimitsConfig


def config(**kw):
    return replace(JointForcePIDConfig(),joint_use_latest_feedback=True,joint_active_braking=True,
        joint_kp_n_m=(8000.,)*6,joint_ki_n_m_s=(0.,)*6,joint_kd_n_s_m=(5000.,)*6,
        joint_max_tracking_error_m=.0025,**kw)


def test_one_step_jitter_is_not_reported_as_motion_and_real_motion_brakes_early():
    now=[1_000_000_000];p=JointPositionForcePID(config(),clock=lambda:now[0]);q=np.full(6,.05)
    p.reset(q,sample_ns=now[0],now_ns=now[0])
    for i in range(20):
        now[0]+=20_000_000
        u,d=p.update(q+8*.000025,q+(i%2)*.000025,sample_ns=now[0],now_ns=now[0])
        assert d['joint_velocity_m_s']==pytest.approx([0]*6)
        p.track_applied(u)
    q+=.000025
    for i in range(5):
        now[0]+=20_000_000;q+=.000025
        u,d=p.update(np.full(6,.0502),q,sample_ns=now[0],now_ns=now[0]);p.track_applied(u)
    assert min(d['joint_velocity_m_s'])>0
    assert max(u)<0 # target not yet passed, but velocity asks for braking
    assert all(d['joint_braking_axes'])


def test_braking_slew_is_explicit_bounded_and_absolute_limits_still_apply():
    now=[1_000_000_000]
    drive=SixAxisForceDrive(None,ForceLimitsConfig(brake_rate_g_s=(5000.,)*6),clock=lambda:now[0])
    drive.anchor_hold_forces_g([-100]*6,now_ns=now[0]);now[0]+=400_000_000
    drive.command_forces([100]*6,dry_run=True,now_ns=now[0])
    now[0]+=20_000_000
    r=drive.command_forces([-300]*6,dry_run=True,now_ns=now[0],braking_axes=[True,False,True,False,True,False])
    assert r['dispatched_forces_g']==pytest.approx([0,90,0,90,0,90])
    now[0]+=20_000_000;r=drive.command_forces([-300]*6,dry_run=True,now_ns=now[0],braking_axes=[True]*6)
    assert r['dispatched_forces_g'][0]==pytest.approx(-100) # crosses holding -100 in 40ms, not 400ms
    now[0]+=10_000_000_000
    r=drive.command_forces_n([100]*6,dry_run=True,now_ns=now[0],braking_axes=[True]*6)
    assert max(r['limited_forces_n'])<=14.709975
    assert max(abs(v) for v in r['dispatched_forces_g'])<=1500
    drive=SixAxisForceDrive(None,clock=lambda:now[0]);drive.anchor_hold_forces_g([0]*6,now_ns=now[0]);now[0]+=20_000_000
    with pytest.raises(ValueError,match='explicitly'):drive.command_forces([100]*6,dry_run=True,now_ns=now[0],braking_axes=[True]*6)


def test_one_encoder_step_deadband_does_not_integrate_float_roundoff():
    now=[1_000_000_000];cfg=config(joint_position_deadband_m=.000025)
    cfg=replace(cfg,joint_ki_n_m_s=(6000.,)*6)
    p=JointPositionForcePID(cfg,clock=lambda:now[0]);q=np.full(6,.05)
    p.reset(q,sample_ns=now[0],now_ns=now[0])
    for k in range(40):
        now[0]+=20_000_000;u,d=p.update(q+.000025,q,sample_ns=now[0],now_ns=now[0]);p.track_applied(u)
        assert u==pytest.approx([0]*6,abs=1e-10)
    assert p.integral==pytest.approx([0]*6)


def test_explicit_terminal_trace_allows_valid_anchor_and_pause_but_never_tare(tmp_path):
    from pus6_force_control.stage5_experiment import Mode3OnlyTrace
    from pus6_motor_driver.protocol import pack_write_register_cmd
    class Port:
        def write(self, data):return len(data)
    trace=Mode3OnlyTrace(Port(),tmp_path)
    with pytest.raises(RuntimeError):trace.write(pack_write_register_cmd(1,0x25,[0,0,0,0,1000]))
    trace.enable_terminal_recovery()
    trace.write(pack_write_register_cmd(1,0x25,[0,0,0,0,1000]))
    trace.write(pack_write_register_cmd(1,0x1A,[1]))
    for reg,words in [(9,[1]),(0x29,[1000]),(0x25,[0,0,0,0,2004])]:
        with pytest.raises(RuntimeError):trace.write(pack_write_register_cmd(1,reg,words))
