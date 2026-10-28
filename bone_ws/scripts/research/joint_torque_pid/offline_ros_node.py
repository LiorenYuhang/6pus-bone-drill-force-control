#!/usr/bin/env python3
"""Research-only observer: diagnostic output only, no motor command backend.

No production subscriptions by default. Explicitly remap the private inputs to
synthetic topics only. This node cannot enable, stop, or command a motor.
"""
import json
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
from pus6_msgs.msg import TimedJointFeedback
from controller import Config, JointTorquePID, Fault
from mock_pipeline import MockBackend, MockPipeline, Mode


class OfflineNode(Node):
    def __init__(self, parameter_overrides=None):
        super().__init__('joint_torque_pid_offline', parameter_overrides=parameter_overrides)
        defaults = {'kp': [0.0]*6, 'ki': [0.0]*6, 'kd': [0.0]*6,
                    'torque_limit_nm': [0.0]*6, 'slew_limit_nm_s': [0.0]*6,
                    'direction': [0.0]*6, 'min_dt_s': .001, 'max_dt_s': .2,
                    'max_feedback_age_s': .2, 'target_timeout_s': .2,
                    'derivative_tau_s': .02, 'enabled': False}
        for name, value in defaults.items(): self.declare_parameter(name, value)
        get = lambda x: self.get_parameter(x).value
        self.enabled = get('enabled')
        self.target = None
        self.target_ns = None
        self.last_feedback_ns = None
        self.first_target_ns = None
        self.latched = False
        self.timeout = float(get('target_timeout_s'))
        self.pid = None
        self.pipeline = None
        if self.enabled:
            if not 0 < self.timeout < float('inf'): raise ValueError('invalid target timeout')
            self.pid = JointTorquePID(Config(tuple(get('kp')), tuple(get('ki')), tuple(get('kd')),
                tuple(get('torque_limit_nm')), tuple(get('slew_limit_nm_s')), tuple(get('direction')),
                get('min_dt_s'), get('max_dt_s'), get('max_feedback_age_s'), get('derivative_tau_s')))
            self.pipeline = MockPipeline(self.pid, MockBackend())
            self.pipeline.enter(Mode.TORQUE)
        self.publisher = self.create_publisher(String, '~/diagnostics', 10)
        self.create_subscription(Float64MultiArray, '~/desired_positions_m', self.on_target, 10)
        self.create_subscription(TimedJointFeedback, '~/synthetic_feedback', self.on_feedback, 10)
        self.create_timer(.05, self.check_timeout)
        self.get_logger().warning('OFFLINE ONLY: diagnostic output, no motor backend or safety stop')

    def publish(self, state):
        message = String()
        message.data = json.dumps(state, allow_nan=False)
        self.publisher.publish(message)

    def fail(self, reason):
        self.latched = True
        if self.pipeline:
            self.pipeline.stop(reason)
        if self.pid:
            try: self.pid.trip(reason)
            except Fault: pass
        self.publish({'fault': str(reason), 'tau_nm': [0.0]*6, 'hardware_stop_sent': False})

    def on_target(self, msg):
        if not self.enabled or self.latched: return
        if len(msg.data) != 6:
            self.fail('invalid target length'); return
        self.target = tuple(msg.data)
        self.target_ns = time.monotonic_ns()
        if self.first_target_ns is None: self.first_target_ns = self.target_ns

    def on_feedback(self, msg):
        if not self.enabled or self.latched or self.target is None: return
        now = time.monotonic_ns()
        if (now - self.target_ns) * 1e-9 > self.timeout:
            self.fail('stale target'); return
        try:
            self.pipeline.torque_cycle(self.target, msg.positions_meters,
                                       list(msg.per_axis_rx_done_monotonic_ns), now, msg.fault_codes)
        except (Fault, OSError) as exc:
            self.fail(exc); return
        self.last_feedback_ns = now
        self.publish(self.pid.state)

    def check_timeout(self):
        if not self.enabled or self.latched or self.target_ns is None: return
        now = time.monotonic_ns()
        if (now - self.target_ns) * 1e-9 > self.timeout:
            self.fail('stale target')
        elif (now - (self.last_feedback_ns or self.first_target_ns)) * 1e-9 > self.pid.config.max_age:
            self.fail('feedback missing')


def main():
    rclpy.init()
    node = OfflineNode()
    try: rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__': main()
