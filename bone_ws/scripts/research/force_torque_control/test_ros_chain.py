import json
import time
import pytest
import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import Float64MultiArray
from pus6_msgs.msg import TimedJointFeedback
from pus6_research_msgs.msg import TorqueCommand, AxialForceCommand, ForceFeedback
from offline_node import OfflineControlNode

CONFIG={'pid':dict(kp=[10.]*6,ki=[0.]*6,kd=[0.]*6,torque_limit=[10.]*6,
 slew_limit=[1000.]*6,direction=[1.]*6,min_dt=.001,max_dt=2.,max_age=2.,derivative_tau=0.),
 'force':dict(kp_n_per_m=[100.]*6,max_force_n=[10.]*6,slew_n_s=[1000.]*6,direction=[1.]*6,max_age_s=2.)}

@pytest.mark.parametrize('mode,output_type,topic,field',[('PID_TORQUE',TorqueCommand,'torque_command','torque_nm'),
 ('ACTUATOR_FORCE_CONTROL',AxialForceCommand,'force_command','force_n')])
def test_dds_modes(mode,output_type,topic,field):
    rclpy.init()
    node=OfflineControlNode([Parameter('enabled',value=True),Parameter('mode',value=mode),
       Parameter('config_json',value=json.dumps(CONFIG)),Parameter('target_max_age_s',value=2.)])
    sender=rclpy.create_node('mock_research_test_sender'); received=[]
    base='/research_force_torque_mock/'
    target=sender.create_publisher(Float64MultiArray,base+'desired_lengths_m',10)
    position=sender.create_publisher(TimedJointFeedback,base+'position_feedback',10)
    force=sender.create_publisher(ForceFeedback,base+'force_feedback',10)
    sub=sender.create_subscription(output_type,base+topic,lambda m: received.append(m),10)
    try:
        deadline=time.monotonic()+4
        while (not target.get_subscription_count() or not position.get_subscription_count() or node.pub_tau.get_subscription_count()+node.pub_force.get_subscription_count()==0) and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.01); rclpy.spin_once(sender,timeout_sec=.01)
        assert target.get_subscription_count() and position.get_subscription_count()
        target.publish(Float64MultiArray(data=[.051]*6))
        t=time.monotonic_ns()
        f=ForceFeedback(); f.source=ForceFeedback.SOURCE_MOCK; f.valid=[True]*6; f.force_n=[1.]*6; f.rx_monotonic_ns=[t]*6; force.publish(f)
        p=TimedJointFeedback(); p.positions_meters=[.05]*6; p.per_axis_rx_done_monotonic_ns=[t-50_000_000]*6; position.publish(p)
        while (node.session.pid.previous_t is None if mode=='PID_TORQUE' else node.session.force_loop.previous_t is None) and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.01)
        p.per_axis_rx_done_monotonic_ns=[time.monotonic_ns()]*6; position.publish(p)
        while not received and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.01); rclpy.spin_once(sender,timeout_sec=.01)
        assert received and list(getattr(received[0],field))==pytest.approx([.01 if mode=='PID_TORQUE' else .1]*6)
        assert node.session.force.bus.hardware_write_count==0
    finally:
        sender.destroy_node(); node.destroy_node(); rclpy.shutdown()
