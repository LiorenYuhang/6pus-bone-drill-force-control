#!/usr/bin/env python3
"""ROS raw status bridge; no port access, no actuator writes."""
import time
import rclpy
from rclpy.node import Node
from pus6_research_msgs.msg import RawMotorStatus, ForceFeedback, RawForceFeedback
from pus6_msgs.msg import TimedJointFeedback
from pus6_kinematics.geometry_config import load_geometry_config
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
        self.declare_parameter('publish_position',False)
        source={'mock':1,'hardware':2}.get(self.get_parameter('source').value)
        self.assembler=StatusAssembler(source,self.get_parameter('gram_force_confirmed').value,float(self.get_parameter('max_age_s').value))
        self.pub=self.create_publisher(ForceFeedback,'/research_force_torque_mock/force_feedback',10)
        self.raw_pub=self.create_publisher(RawForceFeedback,'/research_force_torque_mock/raw_force_feedback',10)
        self.diag=self.create_publisher(String,'~/diagnostics',10)
        self.position_pub=self.create_publisher(TimedJointFeedback,'/research_force_torque_mock/position_feedback',10) if self.get_parameter('publish_position').value else None
        self.geometry=load_geometry_config() if self.position_pub else None
        self.last_position_ns=None
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
        raw,adc,raw_ts,raw_valid,raw_faults=self.assembler.snapshot_raw(time.monotonic_ns())
        raw_msg=RawForceFeedback(); raw_msg.source=self.assembler.source; raw_msg.header.stamp=self.get_clock().now().to_msg()
        raw_msg.force_g_raw=raw; raw_msg.force_adc=adc; raw_msg.rx_monotonic_ns=raw_ts
        raw_msg.valid=raw_valid; raw_msg.fault_codes=raw_faults; self.raw_pub.publish(raw_msg)
        if self.position_pub:
            rows=self.assembler.rows; now=time.monotonic_ns()
            if all(row is not None and 0<=now-row[0]<=self.assembler.max_age*1e9 for row in rows):
                times=[row[0] for row in rows]; status=[row[1] for row in rows]
                if self.last_position_ns is not None and not all(t>old for t,old in zip(times,self.last_position_ns)):
                    return
                self.last_position_ns=times
                pos=TimedJointFeedback(); pos.header.stamp=self.get_clock().now().to_msg()
                pos.snapshot_start_monotonic_ns=min(times); pos.snapshot_end_monotonic_ns=max(times)
                pos.per_axis_rx_done_monotonic_ns=times
                pos.raw_steps=[int(v.current_position) for v in status]
                pos.positions_meters=[self.geometry.operating_home_stroke_m[i]+(pos.raw_steps[i]-self.geometry.operating_home_raw_steps[i])*self.geometry.meters_per_step for i in range(6)]
                pos.currents_ma=[int(v.current_mA) for v in status]
                pos.fault_codes=[int(v.error_code) for v in status]
                pos.temperatures_c=[int(v.temperature) for v in status]
                # forces_g_raw is intentionally not consumed as proof of validity.
                # No UART cycle sequence is present in this independent status stream.
                pos.uart_cycle_sequence=0
                self.position_pub.publish(pos)

def main():
    rclpy.init(); node=ForceStatusNode()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()
if __name__=='__main__': main()
