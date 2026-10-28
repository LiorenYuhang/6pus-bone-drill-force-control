#!/usr/bin/env python3
"""ROS raw status bridge; no port access, no actuator writes."""
import time
import rclpy
from rclpy.node import Node
from pus6_research_msgs.msg import RawMotorStatus, ForceFeedback
from std_msgs.msg import String
from status_bridge import StatusAssembler

class StatusSourcePublisher:
    """Publish samples from a bus already owned by the caller's I/O worker.

    Do not call poll_and_publish from a ROS subscriber callback. It does I/O.
    """
    def __init__(self, node, source, source_kind):
        if source_kind not in (RawMotorStatus.SOURCE_MOCK, RawMotorStatus.SOURCE_HARDWARE):
            raise ValueError('source must be explicit')
        self.source=source; self.source_kind=source_kind
        self.pub=node.create_publisher(RawMotorStatus,'/research_force_status/raw_status',10)
    def poll_and_publish(self, axis):
        axis,raw,stamp=self.source.poll(axis)
        msg=RawMotorStatus(); msg.axis_id=axis; msg.source=self.source_kind
        msg.rx_monotonic_ns=stamp; msg.frame=list(raw)
        self.pub.publish(msg)

class ForceStatusNode(Node):
    def __init__(self, parameter_overrides=None):
        super().__init__('research_force_status',parameter_overrides=parameter_overrides)
        self.declare_parameter('source','mock')
        self.declare_parameter('gram_force_confirmed',False)
        self.declare_parameter('max_age_s',.2)
        source={'mock':1,'hardware':2}.get(self.get_parameter('source').value)
        self.assembler=StatusAssembler(source,self.get_parameter('gram_force_confirmed').value,float(self.get_parameter('max_age_s').value))
        self.pub=self.create_publisher(ForceFeedback,'/research_force_torque_mock/force_feedback',10)
        self.diag=self.create_publisher(String,'~/diagnostics',10)
        self.create_subscription(RawMotorStatus,'~/raw_status',self.on_status,10)
        self.create_timer(.02,self.publish_feedback)
    def on_status(self,msg):
        try: self.assembler.ingest(msg.axis_id,msg.source,msg.frame,msg.rx_monotonic_ns,time.monotonic_ns())
        except Exception as exc:
            self.diag.publish(String(data='invalid status: '+str(exc)))
            if 1<=msg.axis_id<=6: self.assembler.rows[msg.axis_id-1]=None
    def publish_feedback(self):
        vals,ts,valid,faults=self.assembler.snapshot(time.monotonic_ns())
        msg=ForceFeedback(); msg.source=self.assembler.source; msg.header.stamp=self.get_clock().now().to_msg()
        msg.force_n=vals; msg.rx_monotonic_ns=ts; msg.valid=valid; msg.fault_codes=faults
        self.pub.publish(msg)

def main():
    rclpy.init(); node=ForceStatusNode()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()
if __name__=='__main__': main()
