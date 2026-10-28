#!/usr/bin/env python3
"""Mock-only ROS bridge. No serial transport is imported or opened here."""
import json
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
from pus6_msgs.msg import TimedJointFeedback
from pus6_research_msgs.msg import TorqueCommand, AxialForceCommand, ForceFeedback
from core import (Mode, MeasuredForce, MockForceBus, MockTorqueBackend,
                  ForceProtocolMock, ForceOuterLoop, ResearchSession, TorqueNm)
from controller import Config, JointTorquePID

class OfflineControlNode(Node):
    def __init__(self, parameter_overrides=None):
        super().__init__('research_force_torque_mock', parameter_overrides=parameter_overrides)
        self.declare_parameter('enabled', False)
        self.declare_parameter('mode', 'IDLE')
        self.declare_parameter('config_json', '{}')
        self.declare_parameter('period_s', 0.02)
        self.declare_parameter('target_max_age_s', 0.2)
        self.session = None
        self.target = self.position = self.force = None
        self.target_ns = 0
        self.last_position_ns = None
        self.started_ns = time.monotonic_ns()
        self.pub_tau = self.create_publisher(TorqueCommand, '~/torque_command', 10)
        self.pub_force = self.create_publisher(AxialForceCommand, '~/force_command', 10)
        self.pub_diag = self.create_publisher(String, '~/diagnostics', 10)
        self.create_subscription(Float64MultiArray, '~/desired_lengths_m', self.on_target, 10)
        self.create_subscription(TimedJointFeedback, '~/position_feedback', self.on_position, 10)
        self.create_subscription(ForceFeedback, '~/force_feedback', self.on_force, 10)
        if self.get_parameter('enabled').value:
            c=json.loads(self.get_parameter('config_json').value)
            pid=JointTorquePID(Config(**c['pid']))
            f=ForceOuterLoop(**c['force'])
            bus=MockForceBus()
            self.session=ResearchSession(pid,f,MockTorqueBackend(),ForceProtocolMock(bus,max(f.limit)))
            self.session.select(Mode(self.get_parameter('mode').value))
            self.create_timer(float(self.get_parameter('period_s').value),self.tick)
    def on_target(self,msg):
        self.target=list(msg.data); self.target_ns=time.monotonic_ns()
    def on_position(self,msg): self.position=msg
    def on_force(self,msg):
        self.force=MeasuredForce(tuple(float(v) for v in msg.force_n),tuple(int(v) for v in msg.rx_monotonic_ns),tuple(bool(v) for v in msg.valid),tuple(int(v) for v in msg.fault_codes))
    def tick(self):
        s=self.session
        if not s or s.mode is Mode.FAULT: return
        now=time.monotonic_ns()
        try:
            if self.target is None or self.position is None:
                if (now-self.started_ns)*1e-9 > float(self.get_parameter('target_max_age_s').value):
                    raise ValueError('desired lengths or position feedback missing')
                return
            if (now-self.target_ns)*1e-9 > float(self.get_parameter('target_max_age_s').value):
                raise ValueError('desired lengths stale')
            p=self.position; stamps=tuple(int(v) for v in p.per_axis_rx_done_monotonic_ns)
            age = s.pid.config.max_age if s.mode is Mode.PID_TORQUE else s.force_loop.max_age
            if any(t <= 0 or t > now or (now-t)*1e-9 > age for t in stamps):
                raise ValueError('position feedback stale')
            if any(p.fault_codes): raise ValueError('position feedback fault')
            if s.mode is Mode.ACTUATOR_FORCE_CONTROL:
                if self.force is None: raise ValueError('force feedback missing')
                if not all(self.force.valid) or any(self.force.faults): raise ValueError('force feedback invalid or fault')
                if any(t <= 0 or t > now or (now-t)*1e-9 > age for t in self.force.rx_ns):
                    raise ValueError('force feedback stale')
            if stamps == self.last_position_ns: return
            cmd=s.cycle(self.target,list(p.positions_meters),stamps,list(p.fault_codes),now,self.force)
            self.last_position_ns=stamps
            state=s.pid.state if s.mode is Mode.PID_TORQUE else s.force_loop.state
            if s.mode is Mode.PID_TORQUE and not state['primed']:
                state=dict(state)
                state['saturated']=[abs(state['p_nm'][i]+state['i_nm'][i]+state['d_nm'][i]-state['tau_nm'][i]) > 1e-12 for i in range(6)]
            diag={'mode':s.mode.value,'desired_m':self.target,'actual_m':[float(v) for v in p.positions_meters],
                  'position_rx_ns':list(stamps),'state':state,'hardware_write_count':0,
                  'force_rx_ns':list(self.force.rx_ns) if self.force else None}
            self.pub_diag.publish(String(data=json.dumps(diag)))
            if cmd is not None:
                if isinstance(cmd,TorqueNm):
                    msg=TorqueCommand(); msg.torque_nm=list(cmd.values); pub=self.pub_tau
                else:
                    msg=AxialForceCommand(); msg.force_n=list(cmd.values); pub=self.pub_force
                msg.header.stamp=self.get_clock().now().to_msg(); msg.sample_monotonic_ns=now
                pub.publish(msg)
        except Exception as exc:
            s.fail(exc)
            self.pub_diag.publish(String(data=json.dumps({'mode':'FAULT','reason':str(exc),'stop_error':s.stop_error,'hardware_write_count':0})))
            self.get_logger().error('mock control fault: '+str(exc))

def main():
    rclpy.init(); node=OfflineControlNode()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()
if __name__=='__main__': main()
