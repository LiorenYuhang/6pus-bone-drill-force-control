"""Normal force steps are unshaped; exceptional recovery has explicit 5000 g/s."""
from functools import partial
import json
from pathlib import Path
import sys

import numpy as np
import pytest
from scipy.spatial.transform import Rotation
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'pus6_control'))
from pus6_force_control.dual_imu_force_policy import dual_imu_force_limits
from pus6_force_control.mode3_dual_imu import DualImuPlan, DualImuMotionConfig, SyntheticImuSource
from pus6_force_control.six_axis_force_drive import SixAxisForceDrive, ForceLimitsConfig
from pus6_force_control.joint_position_force_pid import JointPositionForcePID
from pus6_force_control.z_position_motion import run, MotionConfig, pid_config, ZPlan
from pus6_motor_driver.fake_bus import MockServoBus
from pus6_motor_driver.force_units import g_to_n

ROOT = Path(__file__).resolve().parents[4]


@pytest.mark.parametrize('braking', [False, True])
def test_normal_force_full_step_reversal_and_absolute_cap_ignore_all_slew(braking):
    now = 1_000_000_000
    drive = SixAxisForceDrive(None, dual_imu_force_limits(), clock=lambda: now)
    drive.anchor_hold_forces_g([0]*6, now_ns=now)
    now += 20_000_000
    result = drive.command_forces_n(g_to_n([400, -400, 5000, -5000, 300, -300]), dry_run=True,
        now_ns=now, braking_axes=[braking]*6)
    assert [f['target_force_g'] for f in result['frames']] == [400, -400, 1500, -1500, 300, -300]
    assert result['command_phase'] == 'normal' and not result['normal_force_slew_enabled']
    now += 20_000_000
    result = drive.command_forces_n(g_to_n([-400, 400, -1500, 1500, -300, 300]), dry_run=True,
        now_ns=now, braking_axes=[True]*6)
    assert [f['target_force_g'] for f in result['frames']] == [-400, 400, -1500, 1500, -300, 300]


def test_recovery_is_explicit_and_5000_only_when_requested():
    now = 1_000_000_000
    drive = SixAxisForceDrive(None, dual_imu_force_limits(), clock=lambda: now)
    drive.anchor_hold_forces_g([400]*6, now_ns=now)
    now += 20_000_000
    result = drive.command_forces_n([0]*6, dry_run=True, now_ns=now, recovery=True)
    assert [f['target_force_g'] for f in result['frames']] == [300]*6
    assert result['command_phase'] == 'recovery'
    now += 20_000_000
    result = drive.command_forces_n([0]*6, dry_run=True, now_ns=now)
    assert [f['target_force_g'] for f in result['frames']] == [0]*6  # no residual recovery limiter


def test_other_entry_force_limits_and_default_pid_slew_compensation_unchanged():
    now = 1_000_000_000
    drive = SixAxisForceDrive(None, ForceLimitsConfig(brake_rate_g_s=(5000.,)*6), clock=lambda: now)
    drive.anchor_hold_forces_g([0]*6, now_ns=now)
    now += 20_000_000
    result = drive.command_forces_n(g_to_n([400]*6), dry_run=True, now_ns=now)
    assert [f['target_force_g'] for f in result['frames']] == [10]*6
    assert pid_config(ZPlan([1000]*6)).joint_max_reference_speed_m_s == .0002
    assert JointPositionForcePID(clock=lambda: now).force_slew_compensation


def test_no_fictitious_normal_5000_slew_delay_in_position_pid():
    now = 1_000_000_000
    config = pid_config(ZPlan([1000]*6))
    pid = JointPositionForcePID(config, clock=lambda: now, force_slew_compensation=False)
    q = np.full(6, .05)
    pid.reset(q, sample_ns=now, now_ns=now, position_stamps_ns=[now]*6)
    pid.last_applied[:] = 4.
    now += 20_000_000
    _, diag = pid.update(q, q, sample_ns=now, now_ns=now, position_stamps_ns=[now]*6)
    assert diag['joint_prediction_horizon_s'] == pytest.approx([.02]*6)
    assert pid.config.joint_kp_n_m == config.joint_kp_n_m
    assert pid.config.joint_kd_n_s_m == config.joint_kd_n_s_m


@pytest.mark.parametrize('fault', ['imu', 'current', 'current_shared', 'communication_in_recovery'])
def test_live_runner_mock_recovery_phase_only_and_urgent_fault_bypass(tmp_path, fault):
    class Source(SyntheticImuSource):
        count = 0
        def activate(self, elapsed_s):
            self.count += 1
            if self.count == 3 and not fault.startswith('current'):
                raise ValueError('platform IMU orientation stale')
        def snapshot(self, ns):
            # Real target steps in a static Mock transport, not a moving plant.
            q = [0., 0., 0., 1.]
            if self.count:
                c = Rotation.from_euler('z', 90, degrees=True).as_matrix()
                q = Rotation.from_matrix(c.T@Rotation.from_euler('x', 1., degrees=True).as_matrix()@c).as_quat().tolist()
            return dict(sample_ns=ns, now_ns=ns, base_q_xyzw=q, platform_q_xyzw=[0.,0.,0.,1.])
    source = Source()
    bus = MockServoBus(initial_positions=[1000]*6, initial_modes=[0]*6)
    original_write = bus.write
    failed_force_writes = []
    def write(frame):
        from pus6_motor_driver.protocol import parse_frame
        f = parse_frame(frame)
        if f.register_addr == 0x27 and fault == 'current_shared' and source.count in (1,2):
            bus.currents[2] = 800
        if f.register_addr == 0x27 and source.count >= 3:
            if fault.startswith('current'): bus.currents[2] = 1501 if fault=='current_shared' else 501
            elif fault == 'communication_in_recovery':
                failed_force_writes.append(frame)
                raise OSError('injected recovery force write failure')
        return original_write(frame)
    bus.write = write
    parameters = yaml.safe_load((ROOT/'bone_ws/src/pus6_force_control/config/mode3_dual_imu.yaml').read_text())['attitude']
    factory = partial(DualImuPlan, source=source, parameters=parameters, duration_s=.15)
    config = dict(epoch=1, hardware_mode='mock', output_dir=str(tmp_path), report_filename='dual_imu_report.json',
        execute=True, dry_run=False, hardware_write_enabled=True, is_hardware_qualified=True, control_mode='force',
        maintenance_only=True, force_fault_recovery='position_reanchor', max_force_handover_displacement_steps=6)
    settings_class = DualImuMotionConfig if fault=='current_shared' else MotionConfig
    report = run(config, settings_class(hold_seconds=.02, settle_seconds=.02, settle_timeout_s=.5), bus=bus, plan_factory=factory)
    assert report['outcome'] in (('fault', 'exit_unconfirmed') if fault.startswith('current') else ('fault',))
    assert report['hardware_write_count'] == 0 and report['serial_closed']
    assert report['force_policy']['normal_max_rate_g_s'] is None
    assert report['force_policy']['exceptional_recovery_rate_g_s'] == [5000.]*6
    assert report['force_policy']['pid_slew_delay_compensation'] is False
    rows = [json.loads(x) for x in (tmp_path/'tracking.jsonl').open()]
    for row in rows:
        expected = np.rint(np.clip(np.asarray(report['hold_forces_g'])+row['pid_output_g'], -1500, 1500))
        assert row['target_force_g'] == pytest.approx(expected, abs=0)
    if fault == 'imu':
        assert report['force_recovery']['status'] == 'holding_bias_reached'
        assert report['all_mode0_exit_confirmed']
        recovery = [json.loads(x) for x in (tmp_path/'recovery_force.jsonl').open()]
        previous = rows[-1]['target_force_g']; previous_ns = rows[-1]['target_timestamp_ns']
        for r in recovery:
            dt = (r['command_timestamp_ns']-previous_ns)*1e-9
            assert max(abs(np.asarray(r['target_force_g'])-previous)) <= 5000*dt+1.01
            previous, previous_ns = r['target_force_g'], r['command_timestamp_ns']
        assert previous == report['hold_forces_g']
        assert report['force_recovery']['motion_stop_confirmed'] is False
    elif fault.startswith('current'):
        if fault == 'current_shared':
            assert report['plan']['settings']['max_current_ma']==1500
            assert len([r for r in rows if r['phase']=='stabilize'])==2  # 800 mA did not hit a hidden 500 mA layer
        assert report['force_recovery']['cycles'] == 0
        assert not (tmp_path/'recovery_force.jsonl').exists()
        assert report['terminal_recovery']['pause_requested']
    else:
        assert report['force_recovery']['status'] == 'aborted_no_retry'
        assert len(failed_force_writes) == 1
        assert report['all_mode0_exit_confirmed']
    assert all(w['register_address'] != 0x29 for w in bus.write_audit)
