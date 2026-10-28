#!/usr/bin/env python3
"""ROS IMUs -> research target lengths; all outputs are non-actuating."""
import json
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray, String
from imu_chain import ImuToLengths, quat

class ImuResearchNode(Node):
    def __init__(self, parameter_overrides=None):
        super().__init__('research_imu_lengths',parameter_overrides=parameter_overrides)
        self.declare_parameter('enabled',False); self.declare_parameter('config_json','{}')
        self.declare_parameter('period_s',.02); self.declare_parameter('imu_max_age_s',.2)
        self.declare_parameter('startup_timeout_s',.2); self.declare_parameter('reference_mode','configured')
        self.reference_captured=False
        self.base=self.platform=None; self.start_ns=time.monotonic_ns(); self.fault=None
        self.pub=self.create_publisher(Float64MultiArray,'/research_force_torque_mock/desired_lengths_m',10)
        self.diag=self.create_publisher(String,'~/diagnostics',10)
        self.create_subscription(Imu,'~/base_imu',lambda msg:self.receive('base',msg),qos_profile_sensor_data)
        self.create_subscription(Imu,'~/platform_imu',lambda msg:self.receive('platform',msg),qos_profile_sensor_data)
        self.chain=None
        if self.get_parameter('enabled').value:
            self.chain=ImuToLengths(**json.loads(self.get_parameter('config_json').value))
            self.create_timer(float(self.get_parameter('period_s').value),self.tick)
    def receive(self,name,msg):
        try:
            if msg.orientation_covariance[0] == -1: raise ValueError('IMU orientation unavailable')
            q=msg.orientation; q=quat((q.x,q.y,q.z,q.w))
            stamp_ns=int(msg.header.stamp.sec)*1000000000+int(msg.header.stamp.nanosec)
            age_ns=self.get_clock().now().nanoseconds-stamp_ns
            max_age=float(self.get_parameter('imu_max_age_s').value)*1e9
            if stamp_ns<=0 or age_ns<0 or age_ns>max_age: raise ValueError('IMU header timestamp invalid or stale')
            setattr(self,name,(q,time.monotonic_ns(),stamp_ns))
        except ValueError as exc:
            self.fault=str(exc); self.diag.publish(String(data='FAULT: '+self.fault))
    def tick(self):
        if self.fault or self.chain is None: return
        now=time.monotonic_ns(); maxage=float(self.get_parameter('imu_max_age_s').value)
        try:
            if self.base is None or self.platform is None:
                if (now-self.start_ns)*1e-9>float(self.get_parameter('startup_timeout_s').value): raise ValueError('IMU feedback missing')
                return
            ros_now=self.get_clock().now().nanoseconds
            if any((now-s[1])*1e-9>maxage or not 0<=ros_now-s[2]<=maxage*1e9 for s in (self.base,self.platform)):
                raise ValueError('IMU feedback stale')
            if not self.reference_captured:
                mode=self.get_parameter('reference_mode').value
                if mode=='first_valid': self.chain.base0=self.base[0]; self.chain.platform0=self.platform[0]
                elif mode!='configured': raise ValueError('invalid reference mode')
                self.reference_captured=True
                self.diag.publish(String(data=json.dumps({'reference_mode':mode,'base_reference_xyzw':self.chain.base0,'platform_reference_xyzw':self.chain.platform0})))
            result=self.chain.step(self.base[0],self.platform[0],now)
            if result is None: return
            result.update(base_rx_ns=self.base[1],platform_rx_ns=self.platform[1],base_header_ns=self.base[2],platform_header_ns=self.platform[2],frame='pus6_base',hardware_write_count=0)
            self.pub.publish(Float64MultiArray(data=result['desired_lengths_m']))
            self.diag.publish(String(data=json.dumps(result)))
        except Exception as exc:
            self.fault=str(exc); self.diag.publish(String(data='FAULT: '+self.fault))

def main():
    rclpy.init(); node=ImuResearchNode()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()
if __name__=='__main__': main()
