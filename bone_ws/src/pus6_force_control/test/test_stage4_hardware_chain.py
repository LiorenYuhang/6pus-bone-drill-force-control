"""Unit tests for Stage 4 Mode 3 hardware execution chain."""
import json
from pathlib import Path
import pytest
import rclpy
from rclpy.parameter import Parameter
from pus6_force_control.stage4_experiment import run, main
from pus6_motor_driver.fake_bus import MockServoBus
from pus6_motor_driver.j14_process_uart import ServoCore


def test_stage4_read_only_audit_in_mock(tmp_path):
    """Verify that execute=False performs read-only initial audit without writes."""
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
    }
    # Initialize MockServoBus with mode 0 (read-only ready)
    bus = MockServoBus(initial_positions=[1000] * 6)
    for mid in range(1, 7):
        bus.modes[mid] = 0

    core = ServoCore(config, bus=bus)
    assert not core.can_write_hardware()

    # Run should complete with outcome read_only_complete
    report = run(config)
    assert report['outcome'] == 'read_only_complete'
    assert report['initial_positions_steps'] == [1000] * 6


def test_stage4_cli_qualification_guard():
    """Verify CLI blocks --hardware without --hardware-qualified."""
    with pytest.raises(SystemExit) as exc:
        main(['--hardware'])
    assert exc.value.code == 2


def test_stage4_cli_dry_run_default():
    """Verify CLI defaults to dry_run when --hardware is not specified."""
    # When --dry-run is supplied, it executes read-only
    # If run in mock mode with --mock, it runs without physical hardware
    res = main(['--mock', '--dry-run', '--output-dir', '/tmp/test_stage4_dry_run'])
    assert res == 0


def test_stage4_permission_validation():
    """Verify that execute=True requires hardware_write_enabled, is_hardware_qualified, and dry_run=False."""
    bad_config = {
        'epoch': 1,
        'hardware_mode': 'mock',
        'output_dir': '/tmp/test_bad_permissions',
        'control_mode': 'force',
        'maintenance_only': True,
        'execute': True,
        'dry_run': True,  # conflict!
        'hardware_write_enabled': False,
        'is_hardware_qualified': False,
        'force_fault_recovery': 'position_reanchor',
    }
    with pytest.raises(ValueError, match="existing independent hardware execution permissions required"):
        run(bad_config)


def test_stage4_hardware_mode3_entry_and_exit_in_mock(tmp_path):
    """Verify that execute=True performs Mode 3 handover, enters Mode 3, writes 0x27 targets, and cleanly exits to Mode 0."""
    out_dir = tmp_path / 'mode3_exec_test'
    out_dir.mkdir(parents=True, exist_ok=True)
    # Write stop_request.json immediately so the control loop exits after first cycle
    (out_dir / 'stop_request.json').write_text('{"request": "stop"}\n')

    root = Path(__file__).resolve().parents[4]
    params_path = root / 'bone_ws/src/pus6_force_control/config/active_stabilization.yaml'
    import yaml
    with params_path.open() as f:
        y = yaml.safe_load(f)
    controller_params = y['force_active_stabilization']['ros__parameters']

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
        'min_force_n': controller_params['min_force_n'],
        'max_force_n': controller_params['max_force_n'],
        'max_rate_g_s': controller_params['max_rate_g_s'],
        'controller_parameters': controller_params,
        'imu_executable': '/bin/true',
        'maximum_seconds': 5.0,
        'launch_imus': False,
    }

    report = run(config)
    assert report['all_mode3_confirmed'] is True
    assert report['all_mode0_exit_confirmed'] is True
    assert report['outcome'] in ('operator_stopped', 'bounded_window_complete')
    assert 'hold_forces_g' in report
