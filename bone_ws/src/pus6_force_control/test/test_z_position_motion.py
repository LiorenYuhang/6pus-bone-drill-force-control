"""Bounded software checks; never open serial, never claim a hardware result."""
import json
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'pus6_control'))
from pus6_force_control.z_position_motion import MotionConfig, ZPlan, hold_sample_settled, main, pid_config, run, slider_to_steps
from pus6_force_control.joint_position_force_pid import JointPositionForcePID
from pus6_force_control.six_axis_force_drive import ForceLimitsConfig, SixAxisForceDrive
from pus6_motor_driver.fake_bus import MockServoBus
from pus6_motor_driver.force_units import g_to_n


def test_full_10mm_ik_path_current_pose_and_return():
    start = [1004, 1012, 999, 1007, 1018, 995]
    plan = ZPlan(start, MotionConfig(move_seconds=200.))  # original slower trajectory remains available
    g = plan.geometry
    # Check every point against the actual IK, rather than hard-coding a step offset.
    for fraction in np.linspace(0., 1., 101):
        pose, q = plan.at_fraction(fraction)
        expected = plan.initial_q + np.asarray(plan.kinematics.differential_inverse_kinematics(pose, plan.initial_pose))
        assert q == pytest.approx(expected)
        assert pose[[0, 1, 3, 4, 5]] == pytest.approx(plan.initial_pose[[0, 1, 3, 4, 5]])
        plan.check_positions(slider_to_steps(q, g))
    assert plan.summary()['excursion_steps'] == pytest.approx([400]*6)
    assert slider_to_steps(plan.target('return', 200.)[1], g) == pytest.approx(start)
    assert plan.peak_speed_m_s == pytest.approx(.00009375)
    # Quintic velocities start and end at zero, with exact shared endpoints.
    assert plan.target('rise', 0.)[1] == pytest.approx(plan.initial_q)
    assert plan.target('rise', 200.)[1] == pytest.approx(plan.target('top_hold', 0.)[1])
    assert plan.target('return', 0.)[1] == pytest.approx(plan.target('top_hold', 0.)[1])


def test_100_second_round_trip_and_pid_reference_are_twice_as_fast():
    plan = ZPlan([1000]*6)
    assert plan.settings.move_seconds == 100.
    assert plan.peak_speed_m_s == pytest.approx(.0001875)
    assert plan.summary()['top_steps'] == pytest.approx([1400]*6)
    assert slider_to_steps(plan.target('return', 100.)[1], plan.geometry) == pytest.approx([1000]*6)
    config_ = pid_config(plan)
    assert config_.joint_max_reference_speed_m_s == .0002
    for phase in ('rise', 'return'):
        # At peak speed the old 0.1 mm/s limiter clipped this 20 ms target
        # increment. Verify the actual PID reference now follows both directions.
        now = 1_000_000_000
        pid = JointPositionForcePID(config_, clock=lambda: now)
        q = plan.target(phase, 49.99)[1]
        pid.reset(q, sample_ns=now, now_ns=now, position_stamps_ns=[now]*6)
        now += 20_000_000
        target = plan.target(phase, 50.01)[1]
        _, diagnostic = pid.update(target, q, sample_ns=now, now_ns=now, position_stamps_ns=[now]*6)
        assert abs(target-q) == pytest.approx([.00000375]*6, abs=1e-12)
        assert diagnostic['joint_reference_m'] == pytest.approx(target, abs=1e-12)


def test_stroke_envelope_and_infeasible_start_are_checked_before_entry():
    with pytest.raises(ValueError, match='stroke'):
        ZPlan([1500]*6)  # 1900 top exceeds the existing normal 1800 bound.
    with pytest.raises(ValueError, match='stroke'):
        ZPlan([1000]*6, device_limits=[(0, 1350)]*6)
    plan = ZPlan([1000]*6, device_limits=[(950, 1450)]*6)
    assert plan.envelope_min_steps == pytest.approx([950]*6)
    assert plan.envelope_max_steps == pytest.approx([1450]*6)
    plan.check_positions([1400]*6)  # Old 12-step protection cannot be used here.
    with pytest.raises(RuntimeError, match='envelope'):
        plan.check_positions([1000]*5 + [1451])
    with pytest.raises(ValueError, match='increase move-seconds'):
        ZPlan([1000]*6, MotionConfig(move_seconds=10.))


def test_offline_entry_never_constructs_uart_owner(monkeypatch, capsys):
    import pus6_force_control.z_position_motion as motion
    def forbidden(*a, **kw):
        pytest.fail('offline planning opened the UART owner')
    monkeypatch.setattr(motion, 'ServoCore', forbidden)
    assert main([]) == 0
    result = json.loads(capsys.readouterr().out)
    assert result['hardware_write_count'] == 0
    assert result['plan']['top_steps'] == pytest.approx([1400]*6)
    assert result['plan']['settings']['move_seconds'] == 100.
    assert result['plan']['peak_joint_speed_m_s'] == pytest.approx(.0001875)
    for arguments in (['--hardware'], ['--hardware', '--hardware-qualified', '--mock']):
        with pytest.raises(SystemExit) as exc:
            main(arguments)
        assert exc.value.code == 2


def config(tmp_path):
    return dict(epoch=1, hardware_mode='mock', output_dir=str(tmp_path), execute=True, dry_run=False,
                hardware_write_enabled=True, is_hardware_qualified=True, control_mode='force',
                maintenance_only=True, force_fault_recovery='position_reanchor', max_force_handover_displacement_steps=6)


def tiny_settings(**kw):
    return MotionConfig(rise_m=.000001, move_seconds=.05, hold_seconds=.02, settle_seconds=.02,
                        settle_timeout_s=.3, **kw)


def test_existing_mock_transport_sequence_logging_and_mode0_recovery(tmp_path):
    # Sub-encoder-step request exercises runtime phases with the existing static
    # MockServoBus. This tests software sequencing, NOT a moving 10 mm plant.
    bus = MockServoBus(initial_positions=[1000]*6, initial_modes=[0]*6)
    report = run(config(tmp_path), tiny_settings(), bus=bus)
    assert report['outcome'] == 'motion_complete'
    assert report['hardware_write_count'] == 0
    assert report['all_mode3_confirmed'] and report['all_mode0_exit_confirmed']
    assert [r['phase'] for r in report['stages']] == ['initial_hold', 'rise', 'top_hold', 'return', 'final_hold']
    writes = bus.write_audit
    assert any(w['register_address'] == 0x27 for w in writes)
    assert not any(w['register_address'] in (9, 0x29) for w in writes)
    entries = [w for w in writes if w['register_address'] == 0x25]
    assert [w['payload'][0] for w in entries] == [3]*6 + [0]*6
    assert all(len(w['payload']) == 5 for w in entries[6:])
    assert list(bus.modes.values()) == [0]*6
    rows = [json.loads(line) for line in (tmp_path/'tracking.jsonl').read_text().splitlines()]
    row = rows[-1]
    for field in ('target_z_m', 'actual_z_m', 'target_positions_steps', 'actual_positions_steps',
                  'position_error_steps', 'pid_output_g', 'target_force_g', 'target_force_27_g_readback',
                  'force_2c_g', 'currents_ma', 'modes_last_readback', 'fault_codes', 'pid_input_rx_ns'):
        assert field in row
    assert 'joint_velocity_m_s' in row['pid']


@pytest.mark.parametrize('problem', ['current', 'stroke', 'stuck', 'mode', 'readback', 'communication'])
def test_faults_exit_mode3_without_continuing_trajectory(tmp_path, problem):
    bus = MockServoBus(initial_positions=[1000]*6, initial_modes=[0]*6)
    original_request = bus.request
    original_write = bus.write
    triggered = [False]
    def write(frame):
        from pus6_motor_driver.protocol import parse_frame
        parsed = parse_frame(frame)
        if list(bus.modes.values()) == [3]*6 and parsed.register_addr == 0x27 and parsed.command == 0x32:
            triggered[0] = True
            if problem == 'current':
                bus.currents[4] = 501
            elif problem == 'stroke':
                bus.positions[2] = 1600
            elif problem == 'mode':
                bus.modes[5] = 0
            elif problem == 'communication':
                raise OSError('injected communication loss')
        return original_write(frame)
    def request(frame, **kwargs):
        from pus6_motor_driver.protocol import parse_frame
        parsed = parse_frame(frame)
        if problem == 'readback' and triggered[0] and parsed.command == 0x31 and parsed.register_addr == 0x27:
            bus.target_forces[parsed.motor_id] = 100
        return original_request(frame, **kwargs)
    bus.request = request
    bus.write = write
    # Stuck test: target is > allowed settling error but below tracking bound.
    settings = (MotionConfig(rise_m=.0001, move_seconds=2., hold_seconds=.02,
                             settle_seconds=.02, settle_timeout_s=.2) if problem == 'stuck'
                else tiny_settings())
    # Mode/readback checks are periodic: keep the first hold alive long enough.
    if problem in ('mode', 'readback'):
        settings = MotionConfig(rise_m=.000001, move_seconds=.05, hold_seconds=1.1,
                                settle_seconds=.02, settle_timeout_s=2.)
    report = run(config(tmp_path), settings, bus=bus)
    assert triggered[0]
    assert report['outcome'] in ('fault', 'exit_unconfirmed')
    assert report['hardware_write_count'] == 0
    expected = {'current': 'current protection', 'stroke': 'envelope', 'stuck': 'did not settle',
                'mode': 'Mode 3 lost', 'readback': 'readback mismatch', 'communication': 'communication loss'}
    assert expected[problem] in report['error']
    assert all(r['phase'] != 'return' for r in report['stages'])
    # A persistent injected bus failure may prevent confirmation; don't turn it
    # into a false successful physical-stop claim.
    assert report['terminal_recovery'] is not None
    assert report['terminal_recovery']['hardware_stop_confirmed'] is False


def test_write_authorization_and_active_mock_rejection(tmp_path):
    c = config(tmp_path)
    c['hardware_write_enabled'] = False
    with pytest.raises(ValueError, match='permissions'):
        run(c)
    c.update(hardware_write_enabled=True, hardware_mode='active')
    with pytest.raises(ValueError, match='forbids injected'):
        run(c, bus=MockServoBus())


def test_10mm_pid_chain_with_coupled_overdamped_numerical_plant():
    # A deliberately simple coupled plant only checks numerical loop behavior;
    # gains, friction, inertia and load are NOT identified hardware parameters.
    plan = ZPlan([1000]*6)
    ns = 1_000_000_000
    pid = JointPositionForcePID(pid_config(plan), clock=lambda: ns)
    q = plan.initial_q.copy()
    pid.reset(q, sample_ns=ns, now_ns=ns, position_stamps_ns=[ns]*6)
    hold = np.array([-300, -250, -200, -150, -100, -50])
    drive = SixAxisForceDrive(None, ForceLimitsConfig(brake_rate_g_s=(5000.,)*6), clock=lambda: ns)
    drive.anchor_hold_forces_g(hold, now_ns=ns)
    mobility = .00004 * (.85*np.eye(6) + .15*np.ones((6, 6))/6.)
    peak = q.copy()
    for phase, duration in [('rise', 100.), ('top_hold', 20.), ('return', 100.), ('final_hold', 20.)]:
        for t in np.arange(.02, duration+.001, .02):
            ns += 20_000_000
            measured = raw_encoder(q, plan)
            _, target = plan.target(phase, t)
            delta, _ = pid.update(target, measured, sample_ns=ns, now_ns=ns, position_stamps_ns=[ns]*6)
            result = drive.command_forces_n(np.asarray(g_to_n(hold))+delta, dry_run=True, now_ns=ns,
                                           braking_axes=pid.braking_axes)
            encoded = [r['target_force_g'] for r in result['frames']]
            applied = np.asarray(g_to_n(encoded))-np.asarray(g_to_n(hold))
            pid.track_applied(applied)
            q += (mobility @ applied)*.02
            peak = np.maximum(peak, q)
            assert max(abs(x) for x in encoded) <= 1500
            assert max(abs(pid.integral)) <= 2.5+1e-9
            plan.check_positions(slider_to_steps(measured, plan.geometry))
        assert max(abs(raw_encoder(q, plan)-target)) <= 2*plan.geometry.meters_per_step
    assert peak-plan.initial_q == pytest.approx([.01]*6, abs=.0001)


def raw_encoder(q, plan):
    return raw_steps(np.rint(slider_to_steps(q, plan.geometry)), plan.geometry)


def raw_steps(steps, geometry):
    from pus6_force_control.force_pipeline import raw_steps_to_slider_m
    return raw_steps_to_slider_m(steps, geometry)


def test_hold_boundary_roundoff_without_relaxing_real_motion_limits():
    settings = MotionConfig()
    errors = [0., 0., 1., 2.0000000000004547, -1.9999999999997726, 1.]
    assert hold_sample_settled(errors, [0.]*6, .14268, .14268, settings)
    errors[3] *= -1  # both signs at the boundary
    assert hold_sample_settled(errors, [0.]*6, .14268, .14268, settings)
    for error in (2.000001, 3., -3.):
        errors[3] = error
        assert not hold_sample_settled(errors, [0.]*6, .14268, .14268, settings)
    errors[3] = 2.
    assert not hold_sample_settled(errors, [.000051]+[0.]*5, .14268, .14268, settings)
    assert not hold_sample_settled(errors, [0.]*6, .1429, .14268, settings)
    assert not hold_sample_settled(errors, [0.]*6, None, .14268, settings)


def test_recorded_top_hold_roundoff_regression():
    # Read the original experiment in place; never rewrite its measurements.
    from dataclasses import asdict
    folder = Path(__file__).resolve().parents[4]/'results/z_position_motion/20261106_130621_472451'
    if not (folder/'tracking.jsonl').exists():
        pytest.skip('local hardware recording unavailable; boundary test remains portable')
    report = json.loads((folder/'z_position_report.json').read_text())
    settings = MotionConfig(**report['plan']['settings'])
    current_pid = asdict(pid_config(ZPlan(report['plan']['initial_steps'], settings)))
    expected_pid = dict(report['pid'], joint_max_reference_speed_m_s=.0002)
    # The only intentional configuration change since this 200 s recording is
    # the motion-reference speed setting; every gain/force/protection field stays.
    assert json.loads(json.dumps(current_pid)) == expected_pid
    rows = [r for line in (folder/'tracking.jsonl').open() if (r := json.loads(line))['phase'] == 'top_hold']
    assert len(rows) == 957
    required = settings.settle_seconds + settings.hold_seconds
    since = old_since = None
    first_completion = None
    old_longest = 0.
    position_roundoff_rejections = 0
    for row in rows:
        t = (row['ns']-rows[0]['ns'])*1e-9
        args = (row['position_error_steps'], row['pid']['joint_velocity_m_s'],
                row['actual_z_m'], row['target_z_m'], settings)
        fixed = hold_sample_settled(*args)
        exact_position = max(abs(e) for e in row['position_error_steps']) <= settings.tolerance_steps
        old = (exact_position and max(abs(v) for v in row['pid']['joint_velocity_m_s']) <= settings.velocity_tolerance_m_s
               and row['actual_z_m'] is not None and abs(row['actual_z_m']-row['target_z_m']) <= settings.z_tolerance_m)
        position_roundoff_rejections += (not exact_position and
            max(abs(e) for e in row['position_error_steps']) <= settings.tolerance_steps+1e-9)
        since = (since if since is not None else t) if fixed else None
        old_since = (old_since if old_since is not None else t) if old else None
        if old_since is not None:
            old_longest = max(old_longest, t-old_since)
        if since is not None and t-since >= required and first_completion is None:
            first_completion = row['phase_elapsed_s']
    assert position_roundoff_rejections == 131
    assert old_longest == pytest.approx(.580255335, abs=1e-6)
    assert first_completion == pytest.approx(4.026687190, abs=1e-6)
