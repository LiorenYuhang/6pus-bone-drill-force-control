"""End-to-end synthetic observer state through DDS to the existing mock node."""
import json
import time
import numpy as np
import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import String
from realistic_simulation import CausalDifference, nonlinear_geometry, modeled_wrench
from offline_ros import ResearchNode


def test_synthetic_model_estimator_to_ros_mock():
    cfg=dict(mass=np.eye(3).tolist(),kp=[2.]*3,kd=[3.]*3,cross=[1.]*6,p=[2.]*6,
       kc=[3.]*6,k=[4.]*6,motor_inertia=[.5]*6,transmission=[.01]*6,
       limit=[100.]*6,slew=[10000.]*6,integral_limit=[1.]*6,min_dt=.001,max_dt=.2,max_age=1.,max_condition=10000.)
    rclpy.init()
    node=ResearchNode([Parameter('enabled',value=True),Parameter('algorithm',value='DS_SCTDE'),
       Parameter('config_json',value=json.dumps(cfg)),Parameter('input_timeout_s',value=1.)])
    sender=rclpy.create_node('model_estimator_test')
    publisher=sender.create_publisher(String,'/ds_sctde_offline/synthetic_sample',10)
    try:
        deadline=time.monotonic()+3.
        while not publisher.get_subscription_count() and time.monotonic()<deadline: rclpy.spin_once(node,timeout_sec=.01)
        assert publisher.get_subscription_count()
        observer=CausalDifference(); eta=np.zeros(3); q,j=nonlinear_geometry(eta)
        for i in range(3): estimate=observer.update(i*.01,eta,q)
        vel,qvel,acc=estimate
        w=modeled_wrench(eta,vel,q,qvel,[560,624,592,656,608,640],[4,4.5,4.2,4.8,4.1,4.6])
        t=time.monotonic_ns()-20_000_000
        def packet(ns,prev):
            return dict(timestamp_ns=ns,fault_codes=[0]*6,eta_rad=eta.tolist(),eta_d_rad=[0]*3,
                eta_dot_rad_s=vel.tolist(),eta_dot_d_rad_s=[0]*3,eta_ddot_rad_s2=acc.tolist(),eta_ddot_d_rad_s2=[0]*3,
                q_m=q.tolist(),q_d_m=q.tolist(),q_dot_m_s=qvel.tolist(),q_dot_d_m_s=[0]*6,q_ddot_d_m_s2=[0]*6,
                friction_motor_nm=[0]*6,jacobian_m_per_rad=j.tolist(),applied_wrench_timestamp_ns=prev,
                applied_task_wrench_prev_nm=w.tolist(),applied_wrench_source='model_estimate')
        publisher.publish(String(data=json.dumps(packet(t,t))))
        while node.last_input_ns is None and time.monotonic()<deadline: rclpy.spin_once(node,timeout_sec=.01)
        assert node.last_input_ns is not None and node.fault is None
        publisher.publish(String(data=json.dumps(packet(t+10_000_000,t))))
        while not node.backend.frames and node.fault is None and time.monotonic()<deadline: rclpy.spin_once(node,timeout_sec=.01)
        assert len(node.backend.frames)==1 and node.fault is None
        assert node.control.state['applied_wrench_source']=='model_estimate' and node.backend.hardware_write_count==0
    finally:
        sender.destroy_node();node.destroy_node();rclpy.shutdown()
