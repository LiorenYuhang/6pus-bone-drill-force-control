"""Unit tests for Mode 3 Holding Force Handover and Execution Chain."""
import json
from pathlib import Path
import pytest
import rclpy
from rclpy.parameter import Parameter
from pus6_msgs.msg import TimedJointFeedback
from pus6_motor_driver.fake_bus import MockServoBus
from pus6_motor_driver.j14_process_uart import ServoCore
from pus6_force_control.mode3_hold_experiment import run, main
from pus6_force_control.active_stabilization_node import ForceActiveStabilizationNode


def test_hold_cli_qualification_guard():
    """Verify CLI blocks --hardware without --hardware-qualified."""
    with pytest.raises(SystemExit) as exc:
        main(['--hardware'])
    assert exc.value.code == 2


def test_hold_cli_guard_disallows_mock_with_hardware():
    """Verify CLI blocks --hardware combined with --mock (mock impersonation guard)."""
    with pytest.raises(SystemExit) as exc:
        main(['--hardware', '--hardware-qualified', '--mock'])
    assert exc.value.code == 2


def test_hold_cli_dry_run_default(tmp_path):
    """Verify CLI defaults to dry_run when --hardware is not specified."""
    res = main(['--mock', '--dry-run', '--output-dir', str(tmp_path / 'cli_dry_run')])
    assert res == 0


def test_hold_read_only_audit_in_mock(tmp_path):
    """Verify that execute=False performs read-only initial audit with zero writes."""
    config = {
        'epoch': 1,
        'hardware_mode': 'mock',
        'port': '/dev/null',
        'baudrate': 921600,
        'output_dir': str(tmp_path / 'audit_test'),
        'control_mode': 'force',
        'maintenance_only': True,
        'dry_run': True,
        'execute': False,
        'hardware_write_enabled': False,
        'is_hardware_qualified': False,
        'force_fault_recovery': 'position_reanchor',
        'max_current_ma': 500,
        'max_position_change_steps': 6,
        'max_force_handover_displacement_steps': 6,
        'duration': 1.0,
    }
    bus = MockServoBus(initial_positions=[2000] * 6)
    for mid in range(1, 7):
        bus.modes[mid] = 0

    report = run(config, bus=bus)
    assert report['outcome'] == 'read_only_complete'
    assert report['initial_positions_steps'] == [2000] * 6
    assert report['hardware_write_count'] == 0
    assert report['source'] == 'MOCK'


def test_hold_execution_in_mock(tmp_path):
    """Verify Mode 3 holding handover in mock: captures F_hold, holds constant targets, logs tracking, and exits to Mode 0."""
    out_dir = tmp_path / 'hold_exec_test'
    out_dir.mkdir(parents=True, exist_ok=True)

    config = {
        'epoch': 1,
        'hardware_mode': 'mock',
        'port': '/dev/null',
        'baudrate': 921600,
        'output_dir': str(out_dir),
        'control_mode': 'force',
        'maintenance_only': True,
        'dry_run': False,
        'execute': True,
        'hardware_write_enabled': True,
        'is_hardware_qualified': True,
        'force_fault_recovery': 'position_reanchor',
        'max_current_ma': 500,
        'max_position_change_steps': 6,
        'max_force_handover_displacement_steps': 6,
        'duration': 0.1,  # short test duration
    }
    bus = MockServoBus(initial_positions=[1500] * 6)
    for mid in range(1, 7):
        bus.modes[mid] = 0

    report = run(config, bus=bus)
    assert report['all_mode3_confirmed'] is True
    assert report['all_mode0_exit_confirmed'] is True
    assert report['outcome'] == 'bounded_window_complete'
    assert report['source'] == 'MOCK'
    assert report['tare_count'] == 0
    assert report['pause_count'] == 0
    assert len(report['hold_forces_g']) == 6

    # Verify hold_tracking.jsonl exists and contains required records
    tracking_file = out_dir / 'hold_tracking.jsonl'
    assert tracking_file.exists()
    lines = [json.loads(line) for line in tracking_file.read_text().strip().split('\n')]
    assert len(lines) >= 1
    sample = lines[0]
    for key in ('ns', 'cycle_id', 'phase', 'source', 'target_force_g', 'force_2c_g',
                'positions_steps', 'displacement_steps', 'currents_ma', 'fault_codes', 'mode_last_readback'):
        assert key in sample, f"Missing key in tracking: {key}"
    assert sample['target_force_g'] == report['hold_forces_g']


def test_mock_impersonation_guard_active_with_mock_bus(tmp_path):
    """Verify that hardware_mode='active' strictly rejects mock bus injection."""
    config = {
        'epoch': 1,
        'hardware_mode': 'active',
        'port': '/dev/null',
        'baudrate': 921600,
        'output_dir': str(tmp_path / 'spoof_test'),
        'control_mode': 'force',
        'maintenance_only': True,
        'dry_run': False,
        'execute': True,
        'hardware_write_enabled': True,
        'is_hardware_qualified': True,
        'force_fault_recovery': 'position_reanchor',
    }
    bus = MockServoBus()
    with pytest.raises(RuntimeError, match="mock bus injection forbidden"):
        run(config, bus=bus)


def test_hold_current_protection_trigger(tmp_path):
    """Verify that exceeding max_current_ma (500mA) triggers current protection and exits to Mode 0."""
    out_dir = tmp_path / 'current_prot_test'
    config = {
        'epoch': 1,
        'hardware_mode': 'mock',
        'port': '/dev/null',
        'baudrate': 921600,
        'output_dir': str(out_dir),
        'control_mode': 'force',
        'maintenance_only': True,
        'dry_run': False,
        'execute': True,
        'hardware_write_enabled': True,
        'is_hardware_qualified': True,
        'force_fault_recovery': 'position_reanchor',
        'max_current_ma': 500,
        'max_position_change_steps': 6,
        'max_force_handover_displacement_steps': 6,
        'duration': 1.0,
    }
    bus = MockServoBus(initial_positions=[1000] * 6)
    for mid in range(1, 7):
        bus.modes[mid] = 0

    # Inject excessive current on axis 3 during hold phase
    orig_request = bus.request
    call_count = [0]
    def hook_request(frame, **kwargs):
        call_count[0] += 1
        if call_count[0] > 20:
            bus.currents[3] = 600
        return orig_request(frame, **kwargs)
    bus.request = hook_request

    report = run(config, bus=bus)
    assert report['outcome'] == 'stopped'
    assert 'current protection exceeded' in report.get('error', '')
    # Mode 0 exit must still be performed during terminal recovery
    assert report.get('all_mode0_exit_confirmed') is True


def test_hold_displacement_protection_trigger(tmp_path):
    """Verify that exceeding displacement limit (6 steps) triggers displacement protection."""
    out_dir = tmp_path / 'disp_prot_test'
    config = {
        'epoch': 1,
        'hardware_mode': 'mock',
        'port': '/dev/null',
        'baudrate': 921600,
        'output_dir': str(out_dir),
        'control_mode': 'force',
        'maintenance_only': True,
        'dry_run': False,
        'execute': True,
        'hardware_write_enabled': True,
        'is_hardware_qualified': True,
        'force_fault_recovery': 'position_reanchor',
        'max_current_ma': 500,
        'max_position_change_steps': 6,
        'max_force_handover_displacement_steps': 6,
        'duration': 1.0,
    }
    bus = MockServoBus(initial_positions=[1000] * 6)
    for mid in range(1, 7):
        bus.modes[mid] = 0

    # Modify position on axis 1 after handover and Mode 3 entry
    orig_request = bus.request
    call_count = [0]
    def hook_request(frame, **kwargs):
        call_count[0] += 1
        if call_count[0] > 120 and bus.modes.get(1) == 3:
            bus.positions[1] = 1010  # 10 steps displacement > 6 steps
        return orig_request(frame, **kwargs)
    bus.request = hook_request

    report = run(config, bus=bus)
    assert report['outcome'] == 'stopped'
    assert 'displacement protection exceeded' in report.get('error', '')
    assert report.get('all_mode0_exit_confirmed') is True


def test_active_stabilization_node_current_protection():
    """Verify that ForceActiveStabilizationNode trips CURRENT_LIMIT when fb.currents_ma > max_current_ma."""
    rclpy.init()
    try:
        now = [1_000_000_000]
        node = ForceActiveStabilizationNode(clock=lambda: now[0])
        node.timer.cancel()
        node.enabled = True

        # Send excessive current in feedback
        fb = TimedJointFeedback()
        fb.header.stamp.sec = 1
        fb.uart_cycle_sequence = 1
        fb.snapshot_start_monotonic_ns = now[0] - 10_000_000
        fb.snapshot_end_monotonic_ns = now[0]
        fb.per_axis_rx_done_monotonic_ns = [now[0] - 5_000_000] * 6
        fb.positions_meters = [0.05] * 6
        fb.raw_steps = [1000] * 6
        fb.forces_g_raw = [0] * 6
        fb.force_adc_raw = [2000] * 6
        fb.force_valid = [True] * 6
        fb.fault_codes = [0] * 6
        fb.source = TimedJointFeedback.SOURCE_MOCK
        fb.force_mode_confirmed = True
        fb.force_hold_valid = True
        fb.force_hold_g = [0] * 6
        fb.force_hold_reference_monotonic_ns = now[0] - 20_000_000
        fb.currents_ma = [50, 60, 550, 40, 50, 60]  # axis 3 = 550 mA > 500 mA

        node.latest_timed_feedback = fb
        node._control_cycle()

        assert node.fault_latched is True
        assert "current protection exceeded" in node.fault_reason
    finally:
        rclpy.shutdown()
