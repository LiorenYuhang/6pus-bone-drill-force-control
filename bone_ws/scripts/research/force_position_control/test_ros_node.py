import json
import time
from types import SimpleNamespace
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


def test_default_speed_profile_reaches_catalog_speed_on_every_axis():
    import math
    from pathlib import Path
    from runtime import load_config
    c=load_config(Path(__file__).with_name('calibrated_config.json'),active=True,hold=False)
    rclpy.init()
    n=ForcePositionNode([Parameter('config_json',value=json.dumps(c['control_config']))])
    try:
        assert not n.shaper_enabled
        assert n.shaper_config['max_speed_m_per_sec']==.017
        assert math.isinf(n.shaper_config['max_accel_m_per_sec2'])
        assert math.isinf(n.shaper_config['max_jerk_m_per_sec3'])
        outputs=[]
        n.out=SimpleNamespace(publish=lambda msg:outputs.append(list(msg.data)))
        n.on_target(Float64MultiArray(data=[.06,.04,.06,.04,.06,.04]))
        p=TimedJointFeedback(); p.positions_meters=[.05]*6
        f=RawForceFeedback(); f.source=2; f.valid=[True]*6
        f.force_g_raw=[round(v) for v in c['calibration']['raw_zero']]
        for stamp in (time.monotonic_ns()-40_000_000,time.monotonic_ns()):
            p.per_axis_rx_done_monotonic_ns=[stamp]*6; f.rx_monotonic_ns=[stamp]*6
            n.on_position(p); n.on_force(f); n.tick()
        assert n.fault is None and n.shaper is None
        assert outputs[-1]==n.controller.state['position_command_m']
        assert c['controller']['correction_rate_m_s']==[.017]*6
        assert c['period_s']==.02 and c['max_cycle_delta_steps'] is None
    finally:
        n.destroy_node(); rclpy.shutdown()
