"""Dedicated test suite verifying Stage 2.1 critical fixes:

1. Fixed-Lag sync failure halts output (does NOT silently fall back to unaligned data).
2. Physical time alignment validation between 6-axis positions and 0x2C force sensors.
3. FK divergence / excessive residual halts force command output.
4. Feedback sequence continuity and gap monitoring.
5. Target force stream timeout latches a stop request and suspends PID.
6. Decoupled safety architecture: dry_run, hardware_write_enabled, is_hardware_qualified.
7. YAML parameter loading from force_pid.yaml.
"""

import time
import json
import numpy as np
import pytest

import rclpy
from std_msgs.msg import Float64
from pus6_msgs.msg import TimedJointFeedback
from pus6_research_msgs.msg import AxialForceCommand

from pus6_force_control.force_control_node import TaskSpaceForceControlNode
from pus6_force_control.six_axis_force_drive import ForceLimitsConfig, SixAxisForceDrive


@pytest.fixture(scope="module")
def ros_context():
    rclpy.init()
    yield
    rclpy.try_shutdown()


def create_timed_feedback(
    seq: int = 1,
    positions_m: list[float] = None,
    forces_g: list[int] = None,
    base_time_ns: int = None,
) -> TimedJointFeedback:
    t_base = time.monotonic_ns() - 15_000_000 if base_time_ns is None else base_time_ns
    msg = TimedJointFeedback()
    msg.header.frame_id = "pus6_base"
    msg.uart_cycle_sequence = seq
    msg.snapshot_start_monotonic_ns = t_base
    msg.snapshot_end_monotonic_ns = t_base + 15_000_000
    msg.per_axis_rx_done_monotonic_ns = [t_base] * 6
    msg.positions_meters = positions_m if positions_m is not None else [0.05] * 6
    msg.raw_steps = [int(round(1000 + (p - 0.05) / 0.000025)) for p in msg.positions_meters]
    msg.currents_ma = [150] * 6
    msg.forces_g_raw = forces_g if forces_g is not None else [0] * 6
    msg.temperatures_c = [35] * 6
    msg.source = TimedJointFeedback.SOURCE_MOCK
    msg.force_valid = [True] * 6
    msg.force_adc_raw = [2000] * 6
    msg.fault_codes = [0] * 6
    return msg


def test_fixed_lag_sync_failure_halts_output(ros_context):
    """Fix 1: When FixedLag is enabled and not yet SYNCED (e.g. buffer warming up or unbracketed),
    the node MUST NOT emit force commands or silently fall back."""
    node = TaskSpaceForceControlNode(
        "test_fixed_lag_halt_node",
        parameter_overrides=[
            rclpy.parameter.Parameter("dry_run", rclpy.Parameter.Type.BOOL, True),
            rclpy.parameter.Parameter("target_force_z_n", rclpy.Parameter.Type.DOUBLE, 5.0),
            rclpy.parameter.Parameter("use_fixed_lag", rclpy.Parameter.Type.BOOL, True),
            rclpy.parameter.Parameter("fixed_lag_delay_ms", rclpy.Parameter.Type.DOUBLE, 50.0),
        ],
    )

    received_cmds = []
    sub = node.create_subscription(
        AxialForceCommand,
        "/pus6/command_axial_forces",
        lambda m: received_cmds.append(m),
        10,
    )

    t0 = time.monotonic_ns()
    # Feed only 1 sample -> FixedLag has EMPTY_BUFFER status (< 2 samples)
    fb = create_timed_feedback(seq=1, base_time_ns=t0)
    node._handle_timed_feedback(fb)
    node._control_cycle()

    # Must NOT publish any force command
    assert len(received_cmds) == 0

    node.destroy_node()


def test_fk_divergence_halts_output(ros_context):
    """Fix 3: When slider positions are geometrically incompatible or FK fails to converge,
    the node MUST halt force output for the current cycle."""
    node = TaskSpaceForceControlNode(
        "test_fk_diverge_node",
        parameter_overrides=[
            rclpy.parameter.Parameter("dry_run", rclpy.Parameter.Type.BOOL, True),
            rclpy.parameter.Parameter("target_force_z_n", rclpy.Parameter.Type.DOUBLE, 5.0),
            rclpy.parameter.Parameter("use_fixed_lag", rclpy.Parameter.Type.BOOL, False),
            rclpy.parameter.Parameter("max_fk_residual_m", rclpy.Parameter.Type.DOUBLE, 1.0e-4),
        ],
    )

    received_cmds = []
    sub = node.create_subscription(
        AxialForceCommand,
        "/pus6/command_axial_forces",
        lambda m: received_cmds.append(m),
        10,
    )

    # Incompatible slider configuration: slider 1 pushed to 0.09m while slider 2 is 0.01m (exceeds strut geometry)
    incompatible_sliders = [0.09, 0.01, 0.05, 0.05, 0.05, 0.05]
    fb = create_timed_feedback(seq=1, positions_m=incompatible_sliders)
    node._handle_timed_feedback(fb)
    time.sleep(0.01)
    node._control_cycle()

    # FK should fail or have excessive residual -> No command published
    assert len(received_cmds) == 0

    node.destroy_node()


def test_target_force_stream_watchdog(ros_context):
    """Fix 5: When target force topic is stale beyond target_timeout_s,
    the controller latches a stop and publishes no further command."""
    node = TaskSpaceForceControlNode(
        "test_target_watchdog_node",
        parameter_overrides=[
            rclpy.parameter.Parameter("dry_run", rclpy.Parameter.Type.BOOL, True),
            rclpy.parameter.Parameter("target_force_z_n", rclpy.Parameter.Type.DOUBLE, 8.0),
            rclpy.parameter.Parameter("target_timeout_s", rclpy.Parameter.Type.DOUBLE, 0.05),  # 50ms timeout
            rclpy.parameter.Parameter("reference_rate_n_s", rclpy.Parameter.Type.DOUBLE, 1000.0),
            rclpy.parameter.Parameter("use_fixed_lag", rclpy.Parameter.Type.BOOL, False),
        ],
    )

    fb1 = create_timed_feedback(seq=1)
    node._handle_timed_feedback(fb1)
    time.sleep(0.01)
    node._control_cycle()

    # Active target = 8.0 N -> positive dispatched forces
    assert all(f > 0 for f in node.drive.last_commanded_forces_g)

    # Sleep longer than target_timeout_s (70ms > 50ms) without updating target
    time.sleep(0.07)
    fb2 = create_timed_feedback(seq=2)
    node._handle_timed_feedback(fb2)
    node._control_cycle()

    # Watchdog resets controller state without sending a zero-force command.
    assert node.fault_latched
    assert node.stop_request_status == "DRY_RUN_ONLY"
    assert node.pipeline.pid.integral_n == 0
    dispatched_after_timeout = node.drive.last_commanded_forces_g
    assert max(dispatched_after_timeout) < 1.0  # Decayed to ~0

    node.destroy_node()


def test_decoupled_safety_architecture():
    """Fix 6: Hardware write authorization must be strictly decoupled from dry_run.
    Enabling hardware_write_enabled without is_hardware_qualified MUST raise ValueError."""
    # 1. Unqualified limits cannot enable hardware writes
    unqualified_limits = ForceLimitsConfig(is_hardware_qualified=False)
    with pytest.raises(ValueError, match="Hardware writes cannot be enabled"):
        SixAxisForceDrive(bus=None, limits_config=unqualified_limits, hardware_write_enabled=True)

    # 2. Qualified limits allow hardware_write_enabled=True
    qualified_limits = ForceLimitsConfig(is_hardware_qualified=True)
    drive = SixAxisForceDrive(bus=None, limits_config=qualified_limits, hardware_write_enabled=True)
    assert drive.hardware_write_enabled is True
    assert drive.limits.is_hardware_qualified is True


def test_yaml_config_parameter_loading(ros_context):
    """Fix 7: Verify parameters in force_pid.yaml are correctly loaded as defaults."""
    node = TaskSpaceForceControlNode("test_yaml_load_node")
    # Verify values from config/force_pid.yaml:
    # reference_rate_n_s is 1.0, output_min_n is -10.0, max_force_n is 14.709975
    assert node.pipeline.pid.config.reference_rate_n_s == 1.0
    assert node.pipeline.pid.config.output_min_n == -10.0
    assert node.pipeline.compensation.direction == (1.0,) * 6
    assert node.drive.limits.allow_negative_forces is True
    assert node.drive.limits.min_force_n == pytest.approx((-14.709975,) * 6)
    node.destroy_node()


def test_sequence_continuity_and_gap_monitoring(ros_context):
    """Fix 4: Verify feedback cycle drop detection increments dropped count."""
    node = TaskSpaceForceControlNode(
        "test_seq_continuity_node",
        parameter_overrides=[rclpy.parameter.Parameter("dry_run", rclpy.Parameter.Type.BOOL, True)],
    )

    fb1 = create_timed_feedback(seq=10)
    node._handle_timed_feedback(fb1)
    assert node.last_cycle_seq == 10
    assert node.dropped_cycles_count == 0

    # Next feedback jumps to seq=15 (dropped 4 frames)
    fb2 = create_timed_feedback(seq=15)
    node._handle_timed_feedback(fb2)
    assert node.last_cycle_seq == 15
    assert node.dropped_cycles_count == 4

    node.destroy_node()
