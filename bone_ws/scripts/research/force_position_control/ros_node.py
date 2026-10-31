#!/usr/bin/env python3
"""Sensor-to-position ROS research node. Publishes targets; does not own UART."""
import json
import math
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
from pus6_msgs.msg import TimedJointFeedback
from pus6_research_msgs.msg import RawForceFeedback
from pus6_control.joint_trajectory_controller import StatefulJerkLimitedShaper
from controller import Config, ForcePositionController


class ForcePositionNode(Node):
    def __init__(self, parameter_overrides=None):
        super().__init__('research_force_position', parameter_overrides=parameter_overrides)
        self.declare_parameter('config_json','{}')
        self.declare_parameter('target_max_age_s',.2)
        self.declare_parameter('status_source',2)
        self.declare_parameter('desired_topic','/research_force_torque_mock/desired_lengths_m')
        self.declare_parameter('position_topic','/research_force_torque_mock/position_feedback')
        self.declare_parameter('force_topic','/research_force_torque_mock/raw_force_feedback')
        self.declare_parameter('anchor_to_first_feedback',False)
        config=json.loads(self.get_parameter('config_json').value)
        self.controller=ForcePositionController(Config.from_document(config['controller']))
        self.shaper_config=dict(config['shaper'])
        self.shaper_enabled=self.shaper_config.pop('enabled',True)
        if not isinstance(self.shaper_enabled,bool): raise ValueError('shaper.enabled must be boolean')
        # Null removes an extra application clamp, not a drive protection.
        # The existing third-order shaper and its bandwidth remain unchanged.
        for key in ('max_accel_m_per_sec2','max_jerk_m_per_sec3'):
            if key in self.shaper_config and self.shaper_config[key] is None:
                self.shaper_config[key]=math.inf
        self.shaper=None; self.last_shaper_ns=None
        self.target=None; self.target_ns=0; self.position=None; self.force=None
        self.last_stamps=None; self.fault=None
        self.target_offset=None
        self.out=self.create_publisher(Float64MultiArray,'~/desired_lengths_m',10)
        self.diag=self.create_publisher(String,'~/diagnostics',10)
        self.create_subscription(Float64MultiArray,self.get_parameter('desired_topic').value,self.on_target,10)
        self.create_subscription(TimedJointFeedback,self.get_parameter('position_topic').value,self.on_position,10)
        self.create_subscription(RawForceFeedback,self.get_parameter('force_topic').value,self.on_force,10)
        self.create_timer(.02,self.tick)

    def on_target(self,msg): self.target=list(msg.data); self.target_ns=time.monotonic_ns()
    def on_position(self,msg): self.position=msg
    def on_force(self,msg): self.force=msg

    def tick(self):
        if self.fault or self.target is None or self.position is None or self.force is None:
            return
        try:
            now=time.monotonic_ns()
            if (now-self.target_ns)*1e-9>float(self.get_parameter('target_max_age_s').value):
                raise ValueError('desired lengths stale')
            p=self.position; f=self.force
            if f.source != int(self.get_parameter('status_source').value):
                raise ValueError('force source mismatch')
            stamps=tuple(int(x) for x in p.per_axis_rx_done_monotonic_ns)
            force_stamps=tuple(int(x) for x in f.rx_monotonic_ns)
            if any(t<=0 or t>now or (now-t)*1e-9>self.controller.config.max_age_s for t in stamps+force_stamps):
                raise ValueError('position or force feedback stale')
            if stamps != force_stamps:
                return  # Wait for position and force from the same six status frames.
            if stamps==self.last_stamps:
                return
            actual=list(p.positions_meters)
            if self.target_offset is None:
                self.target_offset=[actual[i]-self.target[i] for i in range(6)] if self.get_parameter('anchor_to_first_feedback').value else [0.]*6
            desired=[self.target[i]+self.target_offset[i] for i in range(6)]
            if self.shaper_enabled and self.shaper is None:
                self.shaper=StatefulJerkLimitedShaper(initial_positions=actual,**self.shaper_config)
            faults=[int(a) | int(b) for a,b in zip(p.fault_codes,f.fault_codes)]
            corrected=self.controller.update(desired,actual,list(f.force_g_raw),stamps,
                                              list(f.valid),faults,now)
            self.last_stamps=stamps
            if corrected is None:
                self.last_shaper_ns=now
                return
            shaped=self.shaper.update(corrected,(now-self.last_shaper_ns)*1e-9) if self.shaper_enabled else list(corrected)
            self.last_shaper_ns=now
            data=dict(self.controller.state, desired_m=desired, actual_m=actual, shaped_command_m=shaped, trajectory_shaper_enabled=self.shaper_enabled, source=int(f.source),
                      raw_force=[int(v) for v in f.force_g_raw], force_adc=[int(v) for v in f.force_adc],
                      feedback_ns=list(stamps), output='position target published')
            self.diag.publish(String(data=json.dumps(data,allow_nan=False)))
            self.out.publish(Float64MultiArray(data=shaped))
        except Exception as exc:
            self.fault=str(exc)
            self.diag.publish(String(data=json.dumps({'fault':self.fault,'output':'stopped'})))
            self.get_logger().error(self.fault)


def main():
    rclpy.init(); node=ForcePositionNode()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()


if __name__=='__main__': main()
