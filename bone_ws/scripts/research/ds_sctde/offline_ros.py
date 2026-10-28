#!/usr/bin/env python3
"""Synthetic ROS inputs -> selected research algorithm -> in-memory mock only."""
import json
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'joint_torque_pid'))
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from mock_pipeline import MockBackend
import controller as pid_module
from ds_sctde import DSSCTDE, Config, ControllerFault


class ResearchNode(Node):
    def __init__(self, overrides=None):
        super().__init__('ds_sctde_offline', parameter_overrides=overrides)
        self.declare_parameter('enabled', False)
        self.declare_parameter('algorithm', 'DISABLED')
        self.declare_parameter('config_json', '{}')
        self.declare_parameter('input_timeout_s', .2)
        self.enabled = self.get_parameter('enabled').value
        self.algorithm = self.get_parameter('algorithm').value
        self.timeout = self.get_parameter('input_timeout_s').value
        self.backend = MockBackend()
        self.control = None
        self.last_input_ns = None
        self.fault = None
        self.publisher = self.create_publisher(String, '~/diagnostics', 10)
        if self.enabled:
            if not 0 < self.timeout < float('inf'): raise ValueError('invalid input timeout')
            config = json.loads(self.get_parameter('config_json').value)
            if self.algorithm == 'DS_SCTDE': self.control = DSSCTDE(Config(**config))
            elif self.algorithm == 'PID_BASELINE': self.control = pid_module.JointTorquePID(pid_module.Config(**config))
            else: raise ValueError('select DS_SCTDE or PID_BASELINE')
        self.create_subscription(String, '~/synthetic_sample', self.on_sample, 10)
        self.create_timer(.05, self.check_timeout)
        self.get_logger().warning('Synthetic research only; no serial access or device stop')

    def publish(self, payload):
        m = String()
        m.data = json.dumps(payload, allow_nan=False)
        self.publisher.publish(m)

    def fail(self, exc):
        if self.fault is not None: return
        self.fault = str(exc)
        try: self.backend.stop()
        except OSError as stop_exc: self.fault += '; mock stop failed: ' + str(stop_exc)
        self.publish({'algorithm': self.algorithm, 'fault': self.fault, 'hardware_stop_sent': False})

    def on_sample(self, msg):
        if not self.enabled or self.fault is not None: return
        try:
            s = json.loads(msg.data)
            now = time.monotonic_ns()
            if self.algorithm == 'DS_SCTDE':
                out = self.control.update(s, now)
                state = dict(self.control.state)
            else:
                out = self.control.update(s['q_d_m'], s['q_m'], [s['timestamp_ns']]*6, now, s['fault_codes'])
                state = dict(self.control.state)
                if state['primed']: out = None
                state['algorithm'] = self.algorithm
            if out is not None: self.backend.write(out)
            self.last_input_ns = now
            state['mock_frames'] = len(self.backend.frames)
            state['hardware_stop_sent'] = False
            self.publish(state)
        except (ValueError, TypeError, KeyError, ControllerFault, pid_module.Fault, OSError) as exc:
            self.fail(exc)

    def check_timeout(self):
        if self.enabled and self.fault is None and self.last_input_ns is not None and (time.monotonic_ns()-self.last_input_ns)*1e-9 > self.timeout:
            self.fail('synthetic input timeout')


def main():
    rclpy.init()
    node = ResearchNode()
    try: rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__': main()
