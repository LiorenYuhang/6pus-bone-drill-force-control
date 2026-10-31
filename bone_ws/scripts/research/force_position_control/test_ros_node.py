import json
import time
import pytest
import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import Float64MultiArray
from pus6_msgs.msg import TimedJointFeedback
from pus6_research_msgs.msg import RawForceFeedback
from ros_node import ForcePositionNode


def test_synchronised_status_drives_force_controller_then_shaper():
    cfg={'controller':dict(kp=[100.]*6,ki=[0.]*6,kd=[0.]*6,force_limit_n=[2.]*6,
        admittance_m_per_ns=[.001]*6,correction_limit_m=[.001]*6,correction_rate_m_s=[.01]*6,
        raw_zero=[100.]*6,newtons_per_raw=[.01]*6,max_dt_s=.2,max_age_s=.2),'shaper':{}}
    rclpy.init()
    n=ForcePositionNode([Parameter('config_json',value=json.dumps(cfg))])
    try:
        n.on_target(Float64MultiArray(data=[.051]*6))
        p=TimedJointFeedback(); p.positions_meters=[.05]*6
        f=RawForceFeedback(); f.source=2; f.valid=[True]*6; f.force_g_raw=[100]*6
        t=time.monotonic_ns()-40_000_000
        p.per_axis_rx_done_monotonic_ns=[t]*6; f.rx_monotonic_ns=[t]*6
        n.on_position(p); n.on_force(f); n.tick()
        assert n.controller.state['primed']
        t=time.monotonic_ns(); p.per_axis_rx_done_monotonic_ns=[t]*6; f.rx_monotonic_ns=[t]*6
        n.on_position(p); n.on_force(f); n.tick()
        assert n.fault is None
        assert n.controller.state['force_cmd_n'] == pytest.approx([.1]*6)
        assert n.controller.state['correction_m'][0] > 0
        assert n.shaper.q[0] >= .05
    finally:
        n.destroy_node(); rclpy.shutdown()
