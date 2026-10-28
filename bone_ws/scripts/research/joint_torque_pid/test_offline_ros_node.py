import time
import struct
import pytest
import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import Float64MultiArray
from pus6_msgs.msg import TimedJointFeedback
from offline_ros_node import OfflineNode


def test_ros_callbacks_synthetic_feedback_only():
    rclpy.init()
    overrides = [Parameter('enabled', value=True),
                 Parameter('kp', value=[1.0]*6),
                 Parameter('torque_limit_nm', value=[1.0]*6),
                 Parameter('slew_limit_nm_s', value=[100.0]*6),
                 Parameter('direction', value=[1.0]*6)]
    node = OfflineNode(overrides)
    try:
        target = Float64MultiArray(data=[0.051]*6)
        node.on_target(target)
        feedback = TimedJointFeedback()
        feedback.positions_meters = [0.05]*6
        t = time.monotonic_ns()
        feedback.per_axis_rx_done_monotonic_ns = [t - 50_000_000]*6
        node.on_feedback(feedback)
        feedback.per_axis_rx_done_monotonic_ns = [t]*6
        node.on_feedback(feedback)
        assert all(abs(x - .001) < 1e-9 for x in node.pid.output)
        assert len(node.pipeline.backend.frames) == 1
        assert all(abs(x - .001) < 1e-9 for x in struct.unpack('!6d', node.pipeline.backend.frames[0]))
        feedback.fault_codes = [1, 0, 0, 0, 0, 0]
        node.on_feedback(feedback)
        assert node.latched and node.pid.output == [0]*6
        assert node.pipeline.backend.stops == 1
        assert node.pipeline.backend.hardware_write_count == 0
        assert node.publisher.topic_name == '/joint_torque_pid_offline/diagnostics'
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_dds_synthetic_to_mock_backend():
    rclpy.init()
    overrides = [Parameter('enabled', value=True), Parameter('kp', value=[1.0]*6),
                 Parameter('torque_limit_nm', value=[1.0]*6),
                 Parameter('slew_limit_nm_s', value=[100.0]*6),
                 Parameter('direction', value=[1.0]*6),
                 Parameter('target_timeout_s', value=2.0),
                 Parameter('max_feedback_age_s', value=2.0)]
    node = OfflineNode(overrides)
    sender = rclpy.create_node('torque_research_synthetic_sender')
    target_pub = sender.create_publisher(Float64MultiArray, '/joint_torque_pid_offline/desired_positions_m', 10)
    feedback_pub = sender.create_publisher(TimedJointFeedback, '/joint_torque_pid_offline/synthetic_feedback', 10)
    try:
        deadline = time.monotonic() + 4
        while (target_pub.get_subscription_count() == 0 or feedback_pub.get_subscription_count() == 0) and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.01)
        assert target_pub.get_subscription_count() and feedback_pub.get_subscription_count()
        target_pub.publish(Float64MultiArray(data=[.051]*6))
        while node.target is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.01)
        assert node.target is not None
        feedback = TimedJointFeedback()
        feedback.positions_meters = [.05]*6
        t = time.monotonic_ns()
        feedback.per_axis_rx_done_monotonic_ns = [t-50_000_000]*6
        feedback_pub.publish(feedback)
        while node.pid.previous_t is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.01)
        assert node.pid.previous_t is not None
        feedback.per_axis_rx_done_monotonic_ns = [t]*6
        feedback_pub.publish(feedback)
        while not node.pipeline.backend.frames and not node.latched and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.01)
        assert not node.latched
        assert struct.unpack('!6d', node.pipeline.backend.frames[0]) == pytest.approx([.001]*6)
        assert node.pipeline.backend.hardware_write_count == 0
    finally:
        sender.destroy_node()
        node.destroy_node()
        rclpy.shutdown()
