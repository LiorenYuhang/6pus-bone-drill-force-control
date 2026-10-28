"""Explicit synthetic ROS inputs, generated from the same in-memory FakeBus."""
import math
import time
from rclpy.node import Node
from sensor_msgs.msg import Imu
from pus6_research_msgs.msg import RawMotorStatus
from status_bridge import ReadOnlyStatusSource
from status_node import StatusSourcePublisher

class MockInputs(Node):
    def __init__(self,bus,period_s=.02):
        super().__init__('research_mock_inputs')
        self.bus=bus; self.source=ReadOnlyStatusSource(bus)
        self.raw=StatusSourcePublisher(self,self.source,RawMotorStatus.SOURCE_MOCK)
        self.base=self.create_publisher(Imu,'/research_imu_lengths/base_imu',10)
        self.platform=self.create_publisher(Imu,'/research_imu_lengths/platform_imu',10)
        self.start_ns=time.monotonic_ns(); self.tick_count=0
        for i in range(1,7):
            self.bus.forces_g[i]=100; self.bus.positions[i]=1000
        self.create_timer(period_s,self.tick)
    def tick(self):
        # Fixed synthetic position and force; no simulated physical dynamics claimed.
        t=time.monotonic_ns(); elapsed=(t-self.start_ns)*1e-9
        stamp=self.get_clock().now().to_msg(); angle=.01*math.sin(elapsed)
        base=Imu(); base.header.stamp=stamp; base.header.frame_id='mock_base_imu'
        base.orientation.x=math.sin(angle/2); base.orientation.w=math.cos(angle/2)
        platform=Imu(); platform.header.stamp=stamp; platform.header.frame_id='mock_platform_imu'; platform.orientation.w=1.
        self.base.publish(base); self.platform.publish(platform)
        for axis in range(1,7): self.raw.poll_and_publish(axis)
        self.tick_count+=1
