"""Bounded dual-IMU integration checks. No real serial port or robot motion."""
from dataclasses import asdict
from functools import partial
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'pus6_control'))
from pus6_force_control.dual_imu_attitude import DualQuaternionBuffer, PositionLadrcAttitude
from pus6_force_control.mode3_dual_imu import DualImuPlan, SyntheticImuSource, RosDualImuSource, main
from pus6_force_control.z_position_motion import MotionConfig, run, pid_config
from pus6_force_control.joint_position_force_pid import JointPositionForcePID
from pus6_force_control.six_axis_force_drive import SixAxisForceDrive, ForceLimitsConfig
from pus6_motor_driver.fake_bus import MockServoBus
from pus6_motor_driver.force_units import g_to_n

ROOT = Path(__file__).resolve().parents[4]
PARAMETERS = ROOT/'bone_ws/src/pus6_force_control/config/mode3_dual_imu.yaml'
I = [0., 0., 0., 1.]
C = Rotation.from_euler('z', 90, degrees=True).as_matrix()


@pytest.fixture
def parameters():
    return yaml.safe_load(PARAMETERS.read_text())['attitude']


def sample(ns, base=I, platform=I):
    return dict(sample_ns=ns, now_ns=ns, base_q_xyzw=list(base), platform_q_xyzw=list(platform))


def raw_mechanical(rotvec, reference=I):
    return (Rotation.from_quat(reference)*Rotation.from_matrix(C.T@Rotation.from_rotvec(rotvec).as_matrix()@C)).as_quat()


def test_reference_algorithm_and_parameters_are_unmodified_position_copies(parameters):
    position = ROOT.parent/'6pus_bone_drill_position_control/bone_ws/src/h30_imu'
    if not position.exists():
        pytest.skip('portable Force checkout has no adjacent read-only Position source')
    original = yaml.safe_load((position/'config/j13_ladrc_params.yaml').read_text())['j13_ladrc_shadow_runner']['ros__parameters']
    assert parameters == original
    assert (ROOT/'bone_ws/src/pus6_force_control/pus6_force_control/position_ladrc.py').read_bytes() == (position/'scripts/j13_ladrc.py').read_bytes()


def test_asynchronous_quaternions_common_timestamp_and_stale_rejection():
    buffer = DualQuaternionBuffer()
    t = 1_000_000_000
    buffer.push('base', I, t, 100)
    buffer.push('base', raw_mechanical([.02, 0, 0]), t+20_000_000, 120)
    buffer.push('platform', I, t, 200)
    buffer.push('platform', raw_mechanical([0, .01, 0]), t+10_000_000, 210)
    pair = buffer.snapshot(t+25_000_000)
    assert pair['sample_ns'] == t+10_000_000
    assert pair['base_timing']['interpolation_alpha'] == .5
    rb = C@Rotation.from_quat(pair['base_q_xyzw']).as_matrix()@C.T
    assert Rotation.from_matrix(rb).as_rotvec() == pytest.approx([.01, 0, 0], abs=1e-12)
    with pytest.raises(ValueError, match='stale'):
        buffer.snapshot(t+200_000_000)
    with pytest.raises(ValueError, match='rollback'):
        buffer.push('base', I, t)
    with pytest.raises(ValueError, match='zero quaternion'):
        buffer.push('base', [0]*4, t+30_000_000)


def test_partial_gyro_publications_do_not_refresh_orientation_age():
    source = RosDualImuSource.__new__(RosDualImuSource)
    source.buffer = DualQuaternionBuffer()
    source.receive('base', SimpleNamespace(orientation_covariance=[-1.]))
    assert not source.buffer.history['base']
    with pytest.raises(ValueError, match='history'):
        source.snapshot(1_000_000_000)


@pytest.mark.parametrize('axis', range(3))
def test_mechanical_base_direction_ik_position_pid_and_native_force(axis, parameters):
    now = 1_000_000_000
    source = SyntheticImuSource()
    plan = DualImuPlan([1004, 1012, 999, 1007, 1018, 995], source=source, parameters=parameters,
                      clock=lambda: now)
    controller = PositionLadrcAttitude(parameters)
    controller.capture(plan.initial_pose, sample(now))
    pid = JointPositionForcePID(pid_config(plan), clock=lambda: now, force_slew_compensation=False)
    pid.reset(plan.initial_q, sample_ns=now, now_ns=now, position_stamps_ns=[now]*6)
    hold = np.array([-342, -62, 66, 852, -347, -131])
    drive = SixAxisForceDrive(None, plan.force_limits(), clock=lambda: now)
    drive.anchor_hold_forces_g(hold, now_ns=now)
    disturbance = np.eye(3)[axis]*np.deg2rad(1.5)
    qb = raw_mechanical(disturbance)
    for _ in range(80):
        now += 20_000_000
        target, diag = controller.update(sample(now, qb, I))
        q = plan.initial_q+np.asarray(plan.kinematics.differential_inverse_kinematics(target, plan.initial_pose))
        delta, diagnostic = pid.update(q, plan.initial_q, sample_ns=now, now_ns=now, position_stamps_ns=[now]*6)
        command = drive.command_forces_n(np.asarray(g_to_n(hold))+delta, dry_run=True, now_ns=now, braking_axes=pid.braking_axes)
        force_g = [f['target_force_g'] for f in command['frames']]
        pid.track_applied(np.asarray(g_to_n(force_g))-np.asarray(g_to_n(hold)))
    # At arbitrary measured R0 the correct base command is inverse Base @ R0.
    rtarget = Rotation.from_euler('xyz', target[3:]).as_matrix()
    assert rtarget == pytest.approx(Rotation.from_rotvec(-disturbance).as_matrix()@controller.r0, abs=1e-12)
    assert target[:3] == pytest.approx(plan.initial_pose[:3])
    applied = np.asarray(g_to_n(force_g))-np.asarray(g_to_n(hold))
    assert applied@(q-plan.initial_q) > 0  # force increment corrects joint position error
    assert max(abs(v) for v in force_g) <= 1500
    assert np.linalg.norm(applied) > .1  # includes Yaw beyond the retained one-step deadband
    assert diag['platform_error_world_rad'] == pytest.approx([0]*3, abs=1e-12)


@pytest.mark.parametrize('axis', range(3))
def test_platform_ladrc_error_and_residual_oppose_positive_rotation(axis, parameters):
    controller = PositionLadrcAttitude(parameters)
    t = 1_000_000_000
    controller.capture([0, 0, .1326, 0, 0, 0], sample(t))
    qp = raw_mechanical(np.eye(3)[axis]*.01)
    previous = np.zeros(3)
    for k in range(1, 51):
        pose, d = controller.update(sample(t+k*20_000_000, I, qp))
        assert d['prior_dispatched_residual_rad'] == pytest.approx(previous)
        previous = np.array(d['ladrc_residual_rad'])
        assert max(abs(previous)) <= np.deg2rad(.5)
    assert d['platform_error_world_rad'][axis] == pytest.approx(-.01)
    assert d['ladrc_residual_rad'][axis] < 0
    assert pose[axis+3] < 0
    assert pose[:3] == pytest.approx([0, 0, .1326])


def test_nonidentity_reference_noncommuting_so3_composition_and_repeated_sample(parameters):
    controller = PositionLadrcAttitude(parameters)
    pose = np.array([.002, -.001, .133, .02, -.03, .04])
    qb0 = Rotation.from_euler('xyz', [.3, -.2, .1]).as_quat()
    qp0 = Rotation.from_euler('xyz', [-.2, .1, -.3]).as_quat()
    t = 1_000_000_000
    controller.capture(pose, sample(t, qb0, qp0))
    vb = [.01, -.02, .03]; vp = [-.02, .01, .02]
    data = sample(t+20_000_000, raw_mechanical(vb, qb0), raw_mechanical(vp, qp0))
    target, diag = controller.update(data)
    r0 = Rotation.from_euler('xyz', pose[3:]).as_matrix()
    expected = Rotation.from_rotvec(vb).as_matrix().T@Rotation.from_rotvec(diag['ladrc_residual_rad']).as_matrix()@r0
    assert Rotation.from_euler('xyz', target[3:]).as_matrix() == pytest.approx(expected)
    z = controller.ladrc.z1.copy()
    again, d = controller.update(data)
    assert again == pytest.approx(target)
    assert controller.ladrc.z1 == pytest.approx(z)
    assert not d['observer_updated']
    with pytest.raises(ValueError, match='100 ms'):
        controller.update(sample(t+200_000_000))


def test_roll_pitch_selection_preserves_captured_yaw(parameters):
    c = PositionLadrcAttitude(parameters, axes=('roll', 'pitch'))
    pose = [0, 0, .133, .01, -.02, .07]
    c.capture(pose, sample(1_000_000_000))
    target, d = c.update(sample(1_020_000_000, raw_mechanical([.01, .02, .03]), raw_mechanical([.01, .02, .03])))
    assert target[5] == pose[5]
    assert d['ladrc_residual_rad'][2] == 0


@pytest.mark.parametrize('fault', [False, True])
def test_existing_mode3_runtime_logging_recovery_and_imu_loss(tmp_path, parameters, fault, capsys):
    class Source(SyntheticImuSource):
        def activate(self, elapsed):
            if fault:
                raise ValueError('platform IMU orientation stale')
            super().activate(elapsed)
    bus = MockServoBus(initial_positions=[1000]*6, initial_modes=[0]*6)
    source = Source(duration_s=.08, amplitude_deg=.005)
    config = dict(epoch=1, hardware_mode='mock', output_dir=str(tmp_path), report_filename='dual_imu_report.json',
        execute=True, dry_run=False, hardware_write_enabled=True, is_hardware_qualified=True,
        control_mode='force', maintenance_only=True, force_fault_recovery='position_reanchor', max_force_handover_displacement_steps=6)
    factory = partial(DualImuPlan, source=source, parameters=parameters, duration_s=.08)
    report = run(config, MotionConfig(hold_seconds=.02, settle_seconds=.02, settle_timeout_s=.5), bus=bus, plan_factory=factory)
    console = capsys.readouterr().out
    assert 'EXPERIMENT: axes=roll,pitch,yaw duration=0.08s' in console
    assert 'INITIAL_POSITIONS_STEPS: [1000, 1000, 1000, 1000, 1000, 1000]' in console
    assert 'MODE3_CONFIRMED: six axes' in console and 'PHASE: initial_hold' in console
    assert 'Z_POSITION_PLAN:' not in console and 'attitude_parameters' not in console
    saved = json.loads((tmp_path/'dual_imu_report.json').read_text())
    assert saved['plan']['attitude_parameters'] == parameters
    assert saved['pid'] == json.loads(json.dumps(report['pid']))
    assert 'sample' in saved['plan']['imu_reference']
    assert report['outcome'] == ('fault' if fault else 'motion_complete')
    assert report['hardware_write_count'] == 0
    assert report['all_mode0_exit_confirmed'] and report['serial_closed']
    assert all(w['register_address'] != 0x29 for w in bus.write_audit)
    entries = [w for w in bus.write_audit if w['register_address'] == 0x25]
    assert [w['payload'][0] for w in entries] == [3]*6+[0]*6
    rows = [json.loads(line) for line in (tmp_path/'tracking.jsonl').open()]
    assert all('imu' in row['attitude'] for row in rows)
    assert report['pid']['joint_max_reference_speed_m_s'] is None
    baseline = json.loads((ROOT/'results/z_position_motion/20261106_133122_655194/z_position_report.json').read_text())
    assert json.loads(json.dumps(report['pid'])) == dict(baseline['pid'], joint_max_reference_speed_m_s=None,
        joint_kd_n_s_m=[1000.]*6, joint_continuous_velocity=True, joint_integrate_while_moving=True, joint_integral_from_measured_error=True)
    if not fault:
        assert [r['phase'] for r in report['stages']] == ['initial_hold', 'stabilize', 'final_hold']


@pytest.mark.parametrize('error', [None, 'ValueError: joint reference tracking error exceeds configured bound',
    'RuntimeError: IK_TARGET_WORKSPACE_BOUND: {"requested_pose_m_rad": [1,2,3,4,5,6]}'])
def test_main_prints_compact_final_reason_without_dumping_diagnostics(tmp_path, monkeypatch, capsys, error):
    import pus6_force_control.mode3_dual_imu as entry
    import pus6_force_control.dual_imu_analysis as analysis
    def fake_run(config, settings, **kwargs):
        assert config['hardware_mode'] == 'mock'
        return dict(outcome='fault' if error else 'motion_complete', error=error,
            all_mode0_exit_confirmed=True, hardware_write_count=0)
    monkeypatch.setattr(entry, 'run', fake_run)
    monkeypatch.setattr(analysis, 'analyze_run', lambda *args: {})
    rc=main(['--dry-run', '--output-dir', str(tmp_path/'run'), '--duration-seconds', '60'])
    console=capsys.readouterr().out
    assert rc == (2 if error else 0)
    assert 'RESULT_DIRECTORY:' in console and 'OUTCOME:' in console and 'MODE0: True' in console
    assert 'ERROR: '+(error.split(': {',1)[0] if error else 'none') in console
    assert 'requested_pose_m_rad' not in console


def test_hardware_flags_and_synthetic_input_cannot_open_motor_owner(monkeypatch):
    import pus6_force_control.z_position_motion as runner
    monkeypatch.setattr(runner, 'ServoCore', lambda *a, **kw: pytest.fail('unapproved hardware owner'))
    for args in (['--hardware'], ['--hardware', '--hardware-qualified', '--imu-source', 'synthetic'],
                 ['--dry-run', '--hardware'], ['--axes', 'roll,invalid']):
        with pytest.raises(SystemExit) as exc:
            main(args)
        assert exc.value.code == 2


def test_ros_adapter_with_synthetic_publishers_in_isolated_domain(parameters, monkeypatch):
    # Only IMU topics in a separate DDS domain. No motor owner or H30 process.
    import threading
    import time
    import serial
    import rclpy
    from rclpy.context import Context
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import Imu
    monkeypatch.setenv('ROS_DOMAIN_ID', '223')
    monkeypatch.setattr(serial, 'Serial', lambda *a, **kw: pytest.fail('ROS IMU test opened serial'))
    context = Context(); rclpy.init(args=[], context=context, signal_handler_options=SignalHandlerOptions.NO)
    node = Node('mode3_test_synthetic_imus', context=context)
    pubs = [node.create_publisher(Imu, '/pus6/imu_'+name+'/data', qos_profile_sensor_data) for name in ('base', 'platform')]
    stop = threading.Event()
    def publish():
        while not stop.is_set():
            for pub in pubs:
                msg = Imu(); msg.orientation.w = 1.; msg.header.stamp = node.get_clock().now().to_msg()
                pub.publish(msg)
            stop.wait(.02)
    thread = threading.Thread(target=publish, daemon=True); thread.start()
    source = None
    try:
        source = RosDualImuSource(parameters)
        pair = source.snapshot(time.monotonic_ns())
        assert pair['base_q_xyzw'] == pytest.approx(I)
        assert pair['platform_q_xyzw'] == pytest.approx(I)
        assert pair['base_timing']['latest_age_s'] <= .1
        assert pair['platform_timing']['latest_header_ns'] > 0
        assert not source.processes
    finally:
        if source is not None: source.close()
        stop.set(); thread.join(); node.destroy_node(); context.shutdown()
