"""Fixed startup dwell and measured-error integration; no real hardware."""
from functools import partial
import json

import numpy as np
import pytest

from pus6_force_control.dual_imu_attitude import load_attitude_parameters
from pus6_force_control.joint_position_force_pid import JointPositionForcePID
from pus6_force_control.mode3_dual_imu import DualImuPlan, DualImuMotionConfig, SyntheticImuSource
from pus6_force_control.z_position_motion import MotionConfig, ZPlan, pid_config, run
from pus6_motor_driver.fake_bus import MockServoBus
from pus6_motor_driver.protocol import parse_frame


def make_pid():
    plan=DualImuPlan([1000]*6,source=SyntheticImuSource(),parameters=load_attitude_parameters())
    p=JointPositionForcePID(pid_config(plan),clock=lambda:0,force_slew_compensation=False)
    p.reset(np.full(6,.05),sample_ns=1_000_000_000,now_ns=1_000_000_000)
    return p


@pytest.mark.parametrize('error_steps',[-1,0,1])
def test_prediction_crossing_deadband_does_not_integrate(error_steps):
    p=make_pid();p.integral[:]=.3
    # Nonzero retained velocity moves prediction outside the one-step deadband
    # even though the real encoder error is at/inside it.
    p.velocity[:]=-.004 if error_steps>=0 else .004
    q=np.full(6,.05);target=q+error_steps*.000025
    _,d=p.update(target,q,sample_ns=1_020_000_000,now_ns=1_020_000_000)
    assert min(abs(np.array(d['joint_control_error_m'])))>.000025
    assert min(abs(np.array(d['joint_p_n'])))>0
    assert min(abs(np.array(d['joint_d_n'])))>0
    assert d['joint_integral_error_m']==pytest.approx([0]*6,abs=0)
    assert p.integral==pytest.approx([.3]*6)
    # Downstream anti-windup uses the same measured integral input.
    p.track_applied(np.zeros(6))
    assert p.integral==pytest.approx([.3]*6)


def test_integral_uses_measured_error_even_when_prediction_reverses_sign():
    p=make_pid();p.velocity[:]=.005
    q=np.full(6,.05);target=q+.00005
    _,d=p.update(target,q,sample_ns=1_020_000_000,now_ns=1_020_000_000)
    assert max(d['joint_control_error_m'])<0
    assert d['joint_integral_error_m']==pytest.approx([.00005]*6)
    assert p.integral==pytest.approx([6000*.00005*.02]*6)
    assert p.last_error==pytest.approx([.00005]*6)


def test_measured_error_integration_retains_caps_and_downstream_antiwindup():
    p=make_pid();p.integral[:]=2.499
    u,d=p.update(np.full(6,.05005),np.full(6,.05),sample_ns=1_020_000_000,now_ns=1_020_000_000)
    assert p.integral==pytest.approx([2.5]*6)
    p.track_applied(np.zeros(6))
    assert p.integral==pytest.approx([2.499]*6)
    # No shared experiment behavior or numerical gain/limit changes.
    old=pid_config(ZPlan([1000]*6));new=p.config
    assert not old.joint_integral_from_measured_error and new.joint_integral_from_measured_error
    assert new.joint_kp_n_m==old.joint_kp_n_m==(8000.,)*6
    assert new.joint_ki_n_m_s==old.joint_ki_n_m_s==(6000.,)*6
    assert new.joint_kd_n_s_m==(1000.,)*6
    assert new.joint_integral_limit_n==(2.5,)*6 and new.joint_increment_limit_n==(4.,)*6
    assert new.joint_max_tracking_error_m==.0025
    assert new.joint_max_reference_speed_m_s is None


def test_runtime_enters_stabilize_after_one_second_without_settling_and_carries_state(tmp_path,monkeypatch):
    import serial
    monkeypatch.setattr(serial,'Serial',lambda *a,**kw:pytest.fail('test opened serial'))
    class Source(SyntheticImuSource):
        active=0
        def activate(self,elapsed):
            self.active+=1
            if self.active==2:raise ValueError('injected IMU stale after first active frame')
            super().activate(elapsed)
    source=Source(duration_s=60,amplitude_deg=.005)
    bus=MockServoBus(initial_positions=[1000]*6,initial_modes=[0]*6)
    original_write=bus.write
    def write(frame):
        parsed=parse_frame(frame)
        if parsed.register_addr==0x27:
            # The Mode3 atomic handover does not use this normal force frame.
            # Five steps never satisfy the old two-step convergence criterion.
            bus.positions[1]=1005
        return original_write(frame)
    bus.write=write
    config=dict(epoch=1,hardware_mode='mock',output_dir=str(tmp_path),report_filename='dual_imu_report.json',
        execute=True,dry_run=False,hardware_write_enabled=True,is_hardware_qualified=True,
        control_mode='force',maintenance_only=True,force_fault_recovery='position_reanchor',
        max_force_handover_displacement_steps=6)
    factory=partial(DualImuPlan,source=source,parameters=load_attitude_parameters(),duration_s=60)
    report=run(config,DualImuMotionConfig(hold_seconds=.02,settle_seconds=.02,settle_timeout_s=.5),bus=bus,plan_factory=factory)
    rows=[json.loads(line) for line in (tmp_path/'tracking.jsonl').read_text().splitlines()]
    holds=[r for r in rows if r['phase']=='initial_hold'];active=[r for r in rows if r['phase']=='stabilize']
    assert holds[-1]['phase_elapsed_s']>=1.
    assert len(active)==1 and source.active==2
    assert all(abs(r['position_error_steps'][0])>2 for r in holds)
    assert report['all_mode3_confirmed'] and report['all_mode0_exit_confirmed']
    assert report['hardware_write_count']==0 and report['serial_closed']
    assert 'injected IMU stale' in report['error'] and 'settle/hold' not in report['error']
    previous=holds[-1]['pid'];first=active[0]['pid']
    assert abs(previous['joint_i_n'][0])>.1
    expected=np.array(previous['joint_i_n'])+6000*np.array(first['joint_integral_error_m'])*first['joint_dt_s']
    assert first['joint_i_n']==pytest.approx(expected)
    assert first['joint_dt_s']<.1
    assert active[0]['attitude']['observer_updated']
