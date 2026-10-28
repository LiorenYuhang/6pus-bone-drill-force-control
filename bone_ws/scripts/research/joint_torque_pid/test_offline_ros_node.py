import time
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
        feedback.fault_codes = [1, 0, 0, 0, 0, 0]
        node.on_feedback(feedback)
        assert node.latched and node.pid.output == [0]*6
        assert node.publisher.topic_name == '/joint_torque_pid_offline/diagnostics'
    finally:
        node.destroy_node()
        rclpy.shutdown()
