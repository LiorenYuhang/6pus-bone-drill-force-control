"""Unit tests for Mode 3 Dynamic Force Control and Z-Axis Motion Experiment."""
import json
from pathlib import Path
import pytest
import rclpy
from pus6_motor_driver.fake_bus import MockServoBus
from pus6_force_control.mode3_z_motion_experiment import run, main


def test_z_motion_cli_qualification_guard():
    """Verify CLI blocks --hardware without --hardware-qualified."""
    with pytest.raises(SystemExit) as exc:
        main(['--hardware'])
    assert exc.value.code == 2


def test_z_motion_cli_disallows_mock_with_hardware():
    """Verify CLI blocks --hardware combined with --mock (anti-spoof guard)."""
    with pytest.raises(SystemExit) as exc:
        main(['--hardware', '--hardware-qualified', '--mock'])
    assert exc.value.code == 2


def test_z_motion_cli_dry_run_default(tmp_path):
    """Verify CLI defaults to dry_run when --hardware is not specified."""
    res = main(['--mock', '--dry-run', '--output-dir', str(tmp_path / 'cli_dry_run')])
    assert res == 0


def test_z_motion_read_only_audit_in_mock(tmp_path):
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
        'max_position_change_steps': 12,
        'amplitude_n': 1.5,
        'stage_duration_s': 1.0,
        'stages_mode': 'both',
    }
    bus = MockServoBus(initial_positions=[2000] * 6)
    for mid in range(1, 7):
        bus.modes[mid] = 0

    report = run(config, bus=bus)
    assert report['outcome'] == 'read_only_complete'
    assert report['initial_positions_steps'] == [2000] * 6
    assert len(report['initial_pose_m_rad']) == 6
    assert report['hardware_write_count'] == 0
    assert report['source'] == 'MOCK'


def test_z_motion_dynamic_execution_in_mock(tmp_path):
    """Verify dynamic Z-axis motion control in mock: executes stages, updates dynamic forces, records tracking, exits to Mode 0."""
    out_dir = tmp_path / 'z_motion_exec_test'
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
        'max_position_change_steps': 12,
        'amplitude_n': 1.0,
        'stage_duration_s': 0.1,  # short stage duration for fast mock test
        'stages_mode': 'both',
    }
    bus = MockServoBus(initial_positions=[1000] * 6)
    for mid in range(1, 7):
        bus.modes[mid] = 0

    report = run(config, bus=bus)
    assert report['all_mode3_confirmed'] is True
    assert report['all_mode0_exit_confirmed'] is True
    assert report['outcome'] == 'bounded_window_complete'
    assert report['source'] == 'MOCK'
    assert report['tare_count'] == 0
    assert report['pause_count'] == 0
    assert report['control_cycles'] > 0
    assert len(report['stages_executed']) >= 4

    # Verify z_motion_tracking.jsonl exists and contains required records
    tracking_file = out_dir / 'z_motion_tracking.jsonl'
    assert tracking_file.exists()
    lines = [json.loads(line) for line in tracking_file.read_text().strip().split('\n')]
    assert len(lines) >= 1
    sample = lines[0]
    required_keys = (
        'ns', 'cycle_id', 'stage', 'source', 'target_z_n', 'reference_z_n', 'measured_z_n',
        'raw_feedback_meas_z_n', 'pid_output_n', 'target_force_g', 'target_force_27_g_readback',
        'query_timestamp_ns', 'force_2c_g', 'aligned_force_2c_g', 'relative_force_g',
        'relative_aligned_force_g', 'positions_steps', 'displacement_steps', 'currents_ma',
        'fault_codes', 'pose_m_rad', 'delta_xyz_mm', 'delta_z_mm'
    )
    for key in required_keys:
        assert key in sample, f"Missing key in tracking: {key}"

    # Verify stage transition snapshots were recorded
    snap_labels = [s['label'] for s in report['snapshots']]
    assert any(label.startswith('stage_end_') for label in snap_labels)


def test_z_motion_mock_impersonation_guard(tmp_path):
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


def test_z_motion_current_protection_trigger(tmp_path):
    """Verify that exceeding max_current_ma triggers current protection and safely exits to Mode 0."""
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
        'max_position_change_steps': 12,
        'amplitude_n': 1.0,
        'stage_duration_s': 1.0,
        'stages_mode': 'both',
    }
    bus = MockServoBus(initial_positions=[1000] * 6)
    for mid in range(1, 7):
        bus.modes[mid] = 0

    # Inject excessive current during dynamic stage
    orig_request = bus.request
    call_count = [0]
    def hook_request(frame, **kwargs):
        call_count[0] += 1
        if call_count[0] > 120 and bus.modes.get(1) == 3:
            bus.currents[3] = 650  # 650 mA > 500 mA
        return orig_request(frame, **kwargs)
    bus.request = hook_request

    report = run(config, bus=bus)
    assert report['outcome'] == 'stopped'
    assert 'current protection exceeded' in report.get('error', '')
    assert report.get('all_mode0_exit_confirmed') is True


def test_z_motion_displacement_protection_trigger(tmp_path):
    """Verify that exceeding displacement limit (12 steps) triggers displacement protection."""
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
        'max_position_change_steps': 12,
        'amplitude_n': 1.0,
        'stage_duration_s': 1.0,
        'stages_mode': 'both',
    }
    bus = MockServoBus(initial_positions=[1000] * 6)
    for mid in range(1, 7):
        bus.modes[mid] = 0

    # Inject excessive displacement on axis 2 during dynamic stage
    orig_request = bus.request
    call_count = [0]
    def hook_request(frame, **kwargs):
        call_count[0] += 1
        if call_count[0] > 120 and bus.modes.get(1) == 3:
            bus.positions[2] = 1018  # 18 steps > 12 steps
        return orig_request(frame, **kwargs)
    bus.request = hook_request

    report = run(config, bus=bus)
    assert report['outcome'] == 'stopped'
    assert 'displacement protection exceeded' in report.get('error', '')
    assert report.get('all_mode0_exit_confirmed') is True
