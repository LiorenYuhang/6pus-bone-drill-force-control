import json
import struct
import time
import numpy as np
import pytest
import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import String
from offline_ros import ResearchNode
from test_ds_sctde import config, sample


def conf_dict():
    cfg=config()
    return {k:(v.tolist() if isinstance(v,np.ndarray) else v) for k,v in vars(cfg).items()}


def pid_conf():
    return dict(kp=[1.]*6,ki=[0.]*6,kd=[0.]*6,torque_limit=[100.]*6,slew_limit=[10000.]*6,
                direction=[1.]*6,min_dt=.001,max_dt=.2,max_age=.5,derivative_tau=.02)


@pytest.mark.parametrize('algorithm',['PID_BASELINE','DS_SCTDE'])
def test_ros_dds_to_mock_same_synthetic_input(algorithm):
    rclpy.init()
    settings=conf_dict() if algorithm=='DS_SCTDE' else pid_conf()
    overrides=[Parameter('enabled',value=True),Parameter('algorithm',value=algorithm),
               Parameter('config_json',value=json.dumps(settings)),Parameter('input_timeout_s',value=1.0)]
    node=ResearchNode(overrides); sender=rclpy.create_node('ds_test_sender')
    pub=sender.create_publisher(String,'/ds_sctde_offline/synthetic_sample',10)
    try:
        deadline=time.monotonic()+4
        while not pub.get_subscription_count() and time.monotonic()<deadline: rclpy.spin_once(node,timeout_sec=.01)
        assert pub.get_subscription_count()
        t=time.monotonic_ns()-50_000_000
        first=sample(t); first['q_d_m']=[.001]*6
        pub.publish(String(data=json.dumps(first)))
        while node.last_input_ns is None and time.monotonic()<deadline: rclpy.spin_once(node,timeout_sec=.01)
        assert node.last_input_ns is not None
        second=sample(t+50_000_000); second['q_d_m']=[.001]*6
        second['applied_wrench_timestamp_ns']=t
        pub.publish(String(data=json.dumps(second)))
        while not node.backend.frames and node.fault is None and time.monotonic()<deadline: rclpy.spin_once(node,timeout_sec=.01)
        assert node.fault is None and len(node.backend.frames)==1
        values=struct.unpack('!6d',node.backend.frames[0])
        assert all(np.isfinite(values)) and node.backend.hardware_write_count==0
        if algorithm=='PID_BASELINE': assert values == pytest.approx([.001]*6)
        second['fault_codes']=[1,0,0,0,0,0]
        pub.publish(String(data=json.dumps(second)))
        while node.fault is None and time.monotonic()<deadline: rclpy.spin_once(node,timeout_sec=.01)
        assert node.fault and node.backend.stops==1
    finally:
        sender.destroy_node(); node.destroy_node(); rclpy.shutdown()


def test_invalid_selection_rejected():
    rclpy.init()
    try:
        with pytest.raises(ValueError): ResearchNode([Parameter('enabled',value=True),Parameter('algorithm',value='both')])
    finally: rclpy.shutdown()


def test_mock_write_failure_and_timeout_latch():
    rclpy.init()
    node=ResearchNode([Parameter('enabled',value=True), Parameter('algorithm',value='DS_SCTDE'),
                       Parameter('config_json',value=json.dumps(conf_dict()))])
    try:
        t=time.monotonic_ns()-50_000_000
        node.on_sample(String(data=json.dumps(sample(t))))
        node.backend.fail_write=True
        next_sample=sample(t+50_000_000); next_sample['applied_wrench_timestamp_ns']=t
        node.on_sample(String(data=json.dumps(next_sample)))
        assert node.fault and node.backend.stops==1 and node.backend.hardware_write_count==0
        node.check_timeout()
        assert node.backend.stops==1
    finally:
        node.destroy_node(); rclpy.shutdown()


def test_missing_synthetic_input_triggers_mock_stop():
    rclpy.init()
    node=ResearchNode([Parameter('enabled',value=True), Parameter('algorithm',value='DS_SCTDE'),
                       Parameter('config_json',value=json.dumps(conf_dict()))])
    try:
        t=time.monotonic_ns()
        node.on_sample(String(data=json.dumps(sample(t))))
        node.last_input_ns -= 1_000_000_000
        node.check_timeout()
        assert node.fault == 'synthetic input timeout' and node.backend.stops == 1
    finally:
        node.destroy_node(); rclpy.shutdown()
