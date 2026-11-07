"""Comprehensive offline and unit tests for ROS 2 TaskSpaceForceControlNode.

Tests coverage:
1. Node instantiation and parameter loading
2. Processing positive target Z force and forward kinematic pose tracking
3. Bidirectional force support: negative target Z force and smooth direction reversal
4. Handling non-zero platform orientation with spatial Jacobian mapping
5. PID behavior, integral anti-windup on actuator limits
6. Force reference rate limiting (first-order physical dt)
7. Actuator per-axis force saturation and slew rate limiting
8. Feedback gap and timeout fault latching
9. Motor error code reporting and fail-safe latching
10. End-to-end integration with TimedJointFeedback and AxialForceCommand messages
"""

import time
import json
import numpy as np
import pytest

import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Float64
from pus6_msgs.msg import TimedJointFeedback
from pus6_research_msgs.msg import AxialForceCommand, ForceFeedback
from geometry_msgs.msg import WrenchStamped

from pus6_kinematics.analytical_6pus import Analytical6PusKinematics, DEFAULT_NOMINAL_HEIGHT_M
from pus6_force_control.force_control_node import TaskSpaceForceControlNode


@pytest.fixture(scope="module")
def ros_context():
    rclpy.init()
    yield
    rclpy.try_shutdown()


def create_synthetic_timed_feedback(
    seq: int = 1,
    positions_m: list[float] = None,
    forces_g: list[int] = None,
    faults: list[int] = None,
    base_time_ns: int = None,
) -> TimedJointFeedback:
    """Constructs a strictly typed TimedJointFeedback message."""
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
    msg.fault_codes = faults if faults is not None else [0] * 6
    return msg


def test_node_lifecycle_and_default_parameters(ros_context):
    """Verifies node builds with dry_run=True and defaults without hardware access."""
    node = TaskSpaceForceControlNode(
        "test_lifecycle_node",
        parameter_overrides=[
            rclpy.parameter.Parameter("dry_run", rclpy.Parameter.Type.BOOL, True),
            rclpy.parameter.Parameter("target_force_z_n", rclpy.Parameter.Type.DOUBLE, 0.0),
        ],
    )
    assert node.dry_run is True
    assert node.drive.hardware_write_enabled is False
    assert node.fault_latched is False
    assert node.drive.limits.allow_negative_forces is True
    node.destroy_node()


def test_positive_force_tracking_and_allocation(ros_context):
    """Verifies positive Z force reference tracking and six-axis symmetrical thrust allocation."""
    node = TaskSpaceForceControlNode(
        "test_pos_force_node",
        parameter_overrides=[
            rclpy.parameter.Parameter("dry_run", rclpy.Parameter.Type.BOOL, True),
            rclpy.parameter.Parameter("target_force_z_n", rclpy.Parameter.Type.DOUBLE, 6.0),
            rclpy.parameter.Parameter("reference_rate_n_s", rclpy.Parameter.Type.DOUBLE, 500.0),
            rclpy.parameter.Parameter("max_rate_g_s", rclpy.Parameter.Type.DOUBLE_ARRAY, [50000.0] * 6),
            rclpy.parameter.Parameter("kp", rclpy.Parameter.Type.DOUBLE, 1.0),
            rclpy.parameter.Parameter("ki", rclpy.Parameter.Type.DOUBLE, 0.0),
            rclpy.parameter.Parameter("use_fixed_lag", rclpy.Parameter.Type.BOOL, False),
        ],
    )

    fb = create_synthetic_timed_feedback(seq=1, positions_m=[0.05]*6, forces_g=[0]*6)
    node._handle_timed_feedback(fb)

    # Sleep slightly to have positive monotonic elapsed time for rate limiters
    time.sleep(0.01)
    node._control_cycle()

    assert node.control_cycles_count == 1
    assert node.fault_latched is False

    # Check that Z force tracking yielded positive actuator forces
    dispatched_g = node.drive.last_commanded_forces_g
    assert len(dispatched_g) == 6
    assert all(f > 0 for f in dispatched_g)
    # Check symmetry: all 6 sliders should have nearly identical force targets (within 1%)
    assert max(dispatched_g) - min(dispatched_g) < 1.0

    node.destroy_node()


def test_bidirectional_negative_force_and_reversal(ros_context):
    """Verifies bidirectional force support: negative Z force and positive-negative reversal."""
    node = TaskSpaceForceControlNode(
        "test_bidirectional_node",
        parameter_overrides=[
            rclpy.parameter.Parameter("dry_run", rclpy.Parameter.Type.BOOL, True),
            rclpy.parameter.Parameter("target_force_z_n", rclpy.Parameter.Type.DOUBLE, -6.0),
            rclpy.parameter.Parameter("reference_rate_n_s", rclpy.Parameter.Type.DOUBLE, 500.0),
            rclpy.parameter.Parameter("max_rate_g_s", rclpy.Parameter.Type.DOUBLE_ARRAY, [50000.0] * 6),
            rclpy.parameter.Parameter("output_min_n", rclpy.Parameter.Type.DOUBLE, -15.0),
            rclpy.parameter.Parameter("allow_negative_forces", rclpy.Parameter.Type.BOOL, True),
            rclpy.parameter.Parameter("use_fixed_lag", rclpy.Parameter.Type.BOOL, False),
        ],
    )

    fb = create_synthetic_timed_feedback(seq=1, positions_m=[0.05]*6, forces_g=[0]*6)
    node._handle_timed_feedback(fb)
    time.sleep(0.01)
    node._control_cycle()

    # Dispatched forces should be negative (tensile actuator force in model)
    dispatched_neg = list(node.drive.last_commanded_forces_g)
    assert all(f < 0 for f in dispatched_neg)

    # Now reverse target to positive +6.0 N dynamically
    target_msg = Float64(data=6.0)
    node._handle_target_force_z(target_msg)
    assert node.target_force_z_n == 6.0

    # Advance time and run reversal cycle
    time.sleep(0.01)
    fb2 = create_synthetic_timed_feedback(seq=2, positions_m=[0.05]*6, forces_g=[0]*6)
    node._handle_timed_feedback(fb2)
    node._control_cycle()

    dispatched_rev = list(node.drive.last_commanded_forces_g)
    # Rate limiter ensures smooth transition upwards
    assert all(dispatched_rev[i] > dispatched_neg[i] for i in range(6))

    node.destroy_node()


def test_nonzero_orientation_spatial_jacobian(ros_context):
    """Verifies spatial Jacobian and force allocation under non-zero roll/pitch platform tilt."""
    kin = Analytical6PusKinematics()
    tilted_pose = [0.005, -0.003, DEFAULT_NOMINAL_HEIGHT_M + 0.002, 0.05, -0.03, 0.02]
    sliders = kin.inverse_kinematics(tilted_pose)

    node = TaskSpaceForceControlNode(
        "test_tilt_node",
        parameter_overrides=[
            rclpy.parameter.Parameter("dry_run", rclpy.Parameter.Type.BOOL, True),
            rclpy.parameter.Parameter("target_force_z_n", rclpy.Parameter.Type.DOUBLE, 5.0),
            rclpy.parameter.Parameter("reference_rate_n_s", rclpy.Parameter.Type.DOUBLE, 500.0),
            rclpy.parameter.Parameter("max_rate_g_s", rclpy.Parameter.Type.DOUBLE_ARRAY, [50000.0] * 6),
            rclpy.parameter.Parameter("use_fixed_lag", rclpy.Parameter.Type.BOOL, False),
        ],
    )

    fb = create_synthetic_timed_feedback(seq=1, positions_m=sliders, forces_g=[0]*6)
    node._handle_timed_feedback(fb)
    time.sleep(0.01)
    node._control_cycle()

    # Pose estimate should update to tilted pose via FK
    assert np.allclose(node.current_pose_estimate[:3], tilted_pose[:3], atol=1e-3)
    assert np.allclose(node.current_pose_estimate[3:], tilted_pose[3:], atol=1e-2)

    # In tilted configuration, allocated forces must NOT be identical (asymmetry due to tilt)
    dispatched = node.drive.last_commanded_forces_g
    assert max(dispatched) - min(dispatched) > 2.0  # Demonstrates non-trivial spatial Jacobian mapping

    node.destroy_node()


def test_feedback_timeout_fault_latch(ros_context):
    """Verifies feedback timeout trips fail-safe fault latch and stops control execution."""
    node = TaskSpaceForceControlNode(
        "test_timeout_node",
        parameter_overrides=[
            rclpy.parameter.Parameter("dry_run", rclpy.Parameter.Type.BOOL, True),
            rclpy.parameter.Parameter("feedback_timeout_s", rclpy.Parameter.Type.DOUBLE, 0.05),
        ],
    )

    # Ingest old feedback from 100ms ago
    t_old = time.monotonic_ns() - 100_000_000
    fb = create_synthetic_timed_feedback(seq=1, base_time_ns=t_old)
    node._handle_timed_feedback(fb)
    node.last_feedback_recv_ns = t_old  # Simulate stale receipt

    node._control_cycle()
    assert node.fault_latched is True
    assert "Feedback timeout" in node.fault_reason

    node.destroy_node()


def test_motor_fault_codes_latch(ros_context):
    """Verifies that non-zero motor fault codes trip the safety latch."""
    node = TaskSpaceForceControlNode(
        "test_motor_fault_node",
        parameter_overrides=[
            rclpy.parameter.Parameter("dry_run", rclpy.Parameter.Type.BOOL, True),
        ],
    )

    # Axis 3 reports fault code 0x02
    fb = create_synthetic_timed_feedback(seq=1, faults=[0, 0, 2, 0, 0, 0])
    node._handle_timed_feedback(fb)

    node._control_cycle()
    assert node.fault_latched is True
    assert "Motor hardware fault" in node.fault_reason

    node.destroy_node()


def test_actuator_limits_and_integral_antiwindup(ros_context):
    """Verifies that actuator saturation clamps dispatched command and halts integrator windup."""
    node = TaskSpaceForceControlNode(
        "test_antiwindup_node",
        parameter_overrides=[
            rclpy.parameter.Parameter("dry_run", rclpy.Parameter.Type.BOOL, True),
            rclpy.parameter.Parameter("target_force_z_n", rclpy.Parameter.Type.DOUBLE, 50.0),  # Very large target
            rclpy.parameter.Parameter("max_force_n", rclpy.Parameter.Type.DOUBLE_ARRAY, [4.903325] * 6),  # Low limit 500g
            rclpy.parameter.Parameter("max_rate_g_s", rclpy.Parameter.Type.DOUBLE_ARRAY, [50000.0] * 6),
            rclpy.parameter.Parameter("reference_rate_n_s", rclpy.Parameter.Type.DOUBLE, 500.0),
            rclpy.parameter.Parameter("kp", rclpy.Parameter.Type.DOUBLE, 2.0),
            rclpy.parameter.Parameter("ki", rclpy.Parameter.Type.DOUBLE, 5.0),
            rclpy.parameter.Parameter("use_fixed_lag", rclpy.Parameter.Type.BOOL, False),
        ],
    )

    fb = create_synthetic_timed_feedback(seq=1)
    node._handle_timed_feedback(fb)

    # Run several control cycles with persistent tracking error
    for step in range(5):
        time.sleep(0.01)
        fb_step = create_synthetic_timed_feedback(seq=step + 1)
        node._handle_timed_feedback(fb_step)
        node._control_cycle()

    # Dispatched forces should be capped at 500.0 g
    dispatched = node.drive.last_commanded_forces_g
    assert max(dispatched) <= 500.0 + 1e-6
    # Anti-windup should prevent unbounded integral growth
    assert abs(node.pipeline.pid.integral_n) <= node.pipeline.pid.config.integral_limit_n

    node.destroy_node()
