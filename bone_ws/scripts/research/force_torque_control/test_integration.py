import json
import math
import time
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Imu
from pus6_msgs.msg import TimedJointFeedback
from pus6_research_msgs.msg import RawMotorStatus, ForceFeedback, TorqueCommand, AxialForceCommand
from core import MockForceBus, Mode, STANDARD_GRAVITY
from status_bridge import StatusAssembler, ReadOnlyStatusSource, gf_to_newtons, newtons_to_gf
from status_node import ForceStatusNode, StatusSourcePublisher
from imu_chain import ImuToLengths
from imu_node import ImuResearchNode
from offline_node import OfflineControlNode

IMU_CONFIG=dict(base_reference=[0.,0.,0.,1.],platform_reference=[0.,0.,0.,1.],
 kp=[.5]*3,ki=[0.]*3,kd=[0.]*3,residual_limit_rad=[.1]*3,integral_limit_rad_s=[.1]*3,shaper={},max_dt_s=.2)
CONTROL=dict(pid=dict(kp=[10.]*6,ki=[0.]*6,kd=[0.]*6,torque_limit=[10.]*6,
 slew_limit=[1000.]*6,direction=[1.]*6,min_dt=.001,max_dt=.2,max_age=1.,derivative_tau=0.),
 force=dict(kp_n_per_m=[100.]*6,max_force_n=[10.]*6,slew_n_s=[1000.]*6,direction=[1.]*6,max_age_s=1.))

def test_status_decode_and_unit_confirmation():
    assert gf_to_newtons(1000)==pytest.approx(STANDARD_GRAVITY)
    assert newtons_to_gf(-STANDARD_GRAVITY)==pytest.approx(-1000)
    with pytest.raises(ValueError): gf_to_newtons(float('nan'))
    bus=MockForceBus(); bus.forces_g[1]=-1000
    src=ReadOnlyStatusSource(bus,clock=lambda:1_000_000_000)
    axis,raw,ts=src.poll(1)
    a=StatusAssembler(1,False,.2); a.ingest(axis,1,raw,ts,ts)
    assert a.snapshot(ts)[2]==[False]*6
    a=StatusAssembler(1,True,.2); a.ingest(axis,1,raw,ts,ts)
    vals,stamps,valid,faults=a.snapshot(ts)
    assert vals[0]==pytest.approx(-STANDARD_GRAVITY) and valid==[True,False,False,False,False,False]
    assert not any(a.snapshot(ts+300_000_000)[2])
    with pytest.raises(ValueError): a.ingest(axis,2,raw,ts+1,ts+1)
    with pytest.raises(ValueError): a.ingest(2,1,raw,ts+1,ts+1)
    assert bus.hardware_write_count==0 and not any(event[2] in (0x25,0x27,0x29) for event in bus.events)

def test_imu_math_to_ik_and_shaper():
    c=ImuToLengths(**IMU_CONFIG)
    assert c.step([0,0,0,1],[0,0,0,1],1_000_000_000) is None
    q=Rotation.from_euler('x',.02).as_quat()
    result=c.step(q,[0,0,0,1],1_020_000_000)
    assert result['pose_m_rad'][4] == pytest.approx(-.02,abs=1e-8)
    assert len(result['desired_lengths_m'])==6
    assert max(abs(x-.05) for x in result['ik_lengths_m'])>1e-5
    assert max(abs(x-.05) for x in result['desired_lengths_m']) < max(abs(x-.05) for x in result['ik_lengths_m'])
    with pytest.raises(ValueError): c.step([0,0,0,0],[0,0,0,1],1_040_000_000)

@pytest.mark.parametrize('mode,msgtype,topic',[("PID_TORQUE",TorqueCommand,'torque_command'),("ACTUATOR_FORCE_CONTROL",AxialForceCommand,'force_command')])
def test_dds_imu_status_to_controller_and_mock(mode,msgtype,topic):
    rclpy.init(); executor=SingleThreadedExecutor()
    control=OfflineControlNode([Parameter('enabled',value=True),Parameter('mode',value=mode),
        Parameter('config_json',value=json.dumps(CONTROL)),Parameter('target_max_age_s',value=1.)])
    imu=ImuResearchNode([Parameter('enabled',value=True),Parameter('config_json',value=json.dumps(IMU_CONFIG)),Parameter('imu_max_age_s',value=1.)])
    status=ForceStatusNode([Parameter('source',value='mock'),Parameter('gram_force_confirmed',value=True),Parameter('max_age_s',value=1.)])
    sender=rclpy.create_node('integration_sender'); outputs=[]; feedback=[]
    for n in (control,imu,status,sender): executor.add_node(n)
    bpub=sender.create_publisher(Imu,'/research_imu_lengths/base_imu',10)
    ppub=sender.create_publisher(Imu,'/research_imu_lengths/platform_imu',10)
    jpub=sender.create_publisher(TimedJointFeedback,'/research_force_torque_mock/position_feedback',10)
    bus=MockForceBus(); bus.forces_g.update({i:100 for i in range(1,7)})
    src=ReadOnlyStatusSource(bus); publisher=StatusSourcePublisher(sender,src,RawMotorStatus.SOURCE_MOCK)
    spub=publisher.pub
    sender.create_subscription(msgtype,'/research_force_torque_mock/'+topic,lambda m:outputs.append(m),10)
    sender.create_subscription(ForceFeedback,'/research_force_torque_mock/force_feedback',lambda m:feedback.append(m),10)
    try:
        deadline=time.monotonic()+4
        while time.monotonic()<deadline and not (bpub.get_subscription_count() and ppub.get_subscription_count() and jpub.get_subscription_count() and spub.get_subscription_count()): executor.spin_once(timeout_sec=.01)
        assert bpub.get_subscription_count() and spub.get_subscription_count()
        # Establish complete force samples before target and position; no fake zero is used.
        for axis in range(1,7):
            publisher.poll_and_publish(axis)
        while time.monotonic()<deadline and not any(all(m.valid) for m in feedback): executor.spin_once(timeout_sec=.01)
        assert any(all(m.valid) and m.source==1 for m in feedback)
        base=Imu(); q=Rotation.from_euler('x',.02).as_quat(); base.orientation.x,base.orientation.y,base.orientation.z,base.orientation.w=map(float,q)
        plat=Imu(); plat.orientation.w=1.
        while time.monotonic()<deadline and not outputs and control.session.mode is not Mode.FAULT:
            base.header.stamp=sender.get_clock().now().to_msg(); plat.header.stamp=base.header.stamp
            bpub.publish(base); ppub.publish(plat)
            p=TimedJointFeedback(); p.positions_meters=[.05]*6; p.per_axis_rx_done_monotonic_ns=[time.monotonic_ns()]*6; jpub.publish(p)
            until=time.monotonic()+.04
            while time.monotonic()<until: executor.spin_once(timeout_sec=.005)
        assert outputs and not imu.fault and control.session.mode is not Mode.FAULT
        assert control.session.force.bus.hardware_write_count==0 and bus.hardware_write_count==0
        if mode=='PID_TORQUE': assert control.session.torque.commands and not control.session.force.bus.events
        else:
            assert control.session.force_loop.state['force_meas_n']==pytest.approx([.980665]*6)
            assert any(e[2]==0x27 for e in control.session.force.bus.events)
            assert not any(e[2]==0x29 for e in control.session.force.bus.events)
    finally:
        executor.shutdown()
        for n in (sender,status,imu,control): n.destroy_node()
        rclpy.shutdown()

def test_laf_injected_bus_never_opens_and_explicit_mode_only():
    from status_bridge import LafForceAdapter
    from core import AxialForceN, TorqueNm
    bus=MockForceBus()
    with pytest.raises(ValueError): LafForceAdapter(bus,[10.]*6)
    adapter=LafForceAdapter(bus,[10.]*6,gram_force_confirmed=True)
    assert not bus.events
    with pytest.raises(RuntimeError): adapter.write_force(AxialForceN((1.,)*6))
    adapter.set_force_mode_explicit()
    with pytest.raises(TypeError): adapter.write_force(TorqueNm((1.,)*6))
    with pytest.raises(ValueError): adapter.write_force(AxialForceN((-1.,)*6))
    adapter.write_force(AxialForceN((STANDARD_GRAVITY,)*6))
    assert len([e for e in bus.events if e[2]==0x27 and e[3]==[1000]])==6
    assert not any(e[2]==0x29 for e in bus.events)
    adapter.stop(); adapter.stop()
    assert sum(e[2]==0x1a for e in bus.events)==6
    assert bus.hardware_write_count==0

def test_imu_invalid_or_stale_header_faults():
    rclpy.init(); node=ImuResearchNode([Parameter('enabled',value=True),Parameter('config_json',value=json.dumps(IMU_CONFIG))])
    try:
        msg=Imu(); msg.orientation.w=1.
        node.receive('base',msg)
        assert 'timestamp' in node.fault
    finally:
        node.destroy_node(); rclpy.shutdown()


def test_status_invalid_and_fault_not_valid():
    bus=MockForceBus(); bus.faults[1]=3
    source=ReadOnlyStatusSource(bus,clock=lambda:1000)
    axis,raw,ts=source.poll(1)
    a=StatusAssembler(1,True,1.)
    a.ingest(axis,1,raw,ts,ts)
    assert not a.snapshot(ts)[2][0] and a.snapshot(ts)[3][0]==3
    corrupt=bytearray(raw); corrupt[-1] ^= 1
    with pytest.raises(Exception): a.ingest(axis,1,corrupt,ts+1,ts+1)

def test_laf_partial_write_fault_latches_and_never_retries():
    from status_bridge import LafForceAdapter
    from core import AxialForceN
    bus=MockForceBus(); adapter=LafForceAdapter(bus,[10.]*6,gram_force_confirmed=True)
    adapter.set_force_mode_explicit(); bus.fail_on_id=3
    with pytest.raises(OSError): adapter.write_force(AxialForceN((1.,)*6))
    assert adapter.fault and not adapter.entered and adapter.pause_attempted
    attempts=bus.attempt_count
    with pytest.raises(RuntimeError): adapter.write_force(AxialForceN((1.,)*6))
    assert bus.attempt_count==attempts and bus.hardware_write_count==0

def test_status_position_requires_all_axes_to_advance():
    rclpy.init()
    node=ForceStatusNode([Parameter('source',value='mock'),Parameter('publish_position',value=True),Parameter('max_age_s',value=2.)])
    captured=[]
    class Capture:
        def publish(self,msg): captured.append(msg)
    node.position_pub=Capture()
    bus=MockForceBus(); source=ReadOnlyStatusSource(bus)
    def sample(axis):
        _,raw,ts=source.poll(axis); msg=RawMotorStatus(); msg.axis_id=axis; msg.source=1
        msg.rx_monotonic_ns=ts; msg.frame=list(raw); node.on_status(msg)
    try:
        for axis in range(1,7): sample(axis)
        node.publish_feedback(); assert len(captured)==1
        sample(1); node.publish_feedback(); assert len(captured)==1
        for axis in range(2,7): sample(axis)
        node.publish_feedback(); assert len(captured)==2
        assert all(b>a for a,b in zip(captured[0].per_axis_rx_done_monotonic_ns,captured[1].per_axis_rx_done_monotonic_ns))
    finally:
        node.destroy_node(); rclpy.shutdown()
