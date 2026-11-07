"""Numerical/ROS/FakeBus verification only; no serial device is opened."""
import time
from types import SimpleNamespace
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from pus6_force_control.active_stabilization import ActiveStabilizer, StabilizationConfig, ImuHistory
from pus6_force_control.force_pipeline import TaskSpaceForcePipeline
from pus6_motor_driver.fake_bus import MockServoBus
from pus6_motor_driver.j14_process_uart import ServoCore
from pus6_motor_driver.j14_feedback_bridge import populate_force_feedback
from pus6_motor_driver.protocol import parse_frame

HOLD=[-68,-187,-133,-45,18,-6]
I=[0.,0.,0.,1.]


def test_imu_bracketing_sign_invariance_and_gyro_interpolation():
    h=ImuHistory();h.push(10,I,[0,0,0]);h.push(30,-np.array(I),[2,0,0])
    q,w=h.query(20,30);assert abs(q[3])==pytest.approx(1);assert w==pytest.approx([1,0,0])
    for t in (9,31):
        with pytest.raises(ValueError,match='bracket'):h.query(t,30)
    with pytest.raises(ValueError,match='gap'):h.query(20,10)
    with pytest.raises(ValueError):h.push(40,[0]*4,[0]*3)


@pytest.mark.parametrize('axis',range(3))
def test_world_attitude_restoring_moments_and_hold_force_encoding(axis):
    clock=[1_000_000_000];controller=ActiveStabilizer(clock=lambda:clock[0])
    pipe=TaskSpaceForcePipeline(clock=lambda:clock[0]);pipe.initialize_hold_forces_g(HOLD)
    pose=np.array([0,0,pipe.allocator.kin.nominal_height,0,0,0]);controller.capture(pose,I,I,clock[0])
    clock[0]+=20_000_000;delta=np.eye(3)[axis]*.01
    q=Rotation.from_rotvec(delta).as_quat()
    wrench,details=controller.update(pose,q,q,[0]*3,[0]*3,sample_ns=clock[0],now_ns=clock[0])
    mech=np.array([[0,-1,0],[1,0,0],[0,0,1]])@delta
    assert wrench[3:]@mech<0
    result=pipe.command_increment_wrench(wrench,HOLD,pose,now_ns=clock[0])
    assert result.pid is None
    assert result.drive_result['requested_forces_n']==pytest.approx(pipe.hold_forces_n+result.allocation.actuator_forces_n)
    assert result.measured_wrench==pytest.approx([0]*6)
    for frame in result.drive_result['frames']:
        parsed=parse_frame(bytes.fromhex(frame['frame_hex']));assert parsed.register_addr==0x27


def test_nonidentity_initial_pose_and_stable_world_platform_need_no_restoring_torque():
    clock=[1_000_000_000];c=ActiveStabilizer(clock=lambda:clock[0])
    p0=np.array([.001,.002,.133,.04,-.02,.03]);c.capture(p0,I,I,clock[0])
    # Perfect inverse-base compensation in FK while Platform IMU stays at its
    # captured world attitude, including a nonidentity initial platform pose.
    raw=Rotation.from_rotvec([.01,0,0]).as_quat();cb=np.array([[0,-1,0],[1,0,0],[0,0,1]])
    rb=cb@Rotation.from_quat(raw).as_matrix()@cb.T
    pose=p0.copy();pose[3:]=Rotation.from_matrix(rb.T@Rotation.from_euler('xyz',p0[3:]).as_matrix()).as_euler('xyz')
    clock[0]+=20_000_000
    w,d=c.update(pose,raw,I,[0]*3,[0]*3,sample_ns=clock[0],now_ns=clock[0])
    assert w[3:]==pytest.approx([0]*3,abs=1e-12)


def test_fk_pose_pid_captured_position_hold_and_actual_dt_rate_limit():
    clock=[1_000_000_000];c=ActiveStabilizer(clock=lambda:clock[0]);pose=np.array([0,0,.133,0,0,0])
    c.capture(pose,I,I,clock[0]);pose[0]+=.0001;pose[5]+=.01;clock[0]+=10_000_000
    w,d=c.update(pose,I,I,[0]*3,[0,0,.2],sample_ns=clock[0],now_ns=clock[0])
    assert w[0]<0 and w[5]<0
    assert np.max(abs(w[3:]))<=.1*.01+1e-12
    with pytest.raises(ValueError):c.update(pose,I,I,[0]*3,[0]*3,sample_ns=clock[0],now_ns=clock[0])


def test_uart_owner_mode3_hold_first_cycle_timeout_and_ipc_metadata(tmp_path):
    bus=MockServoBus();bus.modes={i:0 for i in range(1,7)}
    bus.positions={i:1000 for i in range(1,7)};bus.actual_forces=dict(enumerate(HOLD,1))
    core=ServoCore(dict(epoch=1,hardware_mode='mock',output_dir=str(tmp_path),control_mode='force',
        dry_run=False,min_force_n=-14.709975,max_force_n=14.709975,max_rate_g_s=500.,hold_anchor_on_timeout=True),bus=bus)
    try:
        core.initialize();before=len(bus.write_audit);packet=core.cycle(time.monotonic_ns())
        assert core.last_command_force_g==HOLD
        assert all(bus.modes[i]==3 for i in range(1,7))
        assert {w['register_address'] for w in bus.write_audit[before:]}=={0x27}
        fields=core.force_feedback_fields();msg=SimpleNamespace(per_axis_rx_done_monotonic_ns=core.rx_ns,fault_codes=core.faults)
        populate_force_feedback(msg,fields)
        assert msg.force_mode_confirmed and msg.force_hold_valid
        assert msg.force_hold_g==HOLD and msg.force_hold_reference_monotonic_ns>0
        core.command.command_ns=time.monotonic_ns()-1_000_000_000
        with pytest.raises(RuntimeError, match='force_target_timeout'):
            core.command.candidate_force([99.]*6,False,time.monotonic_ns())
        assert not any(w['register_address'] in (0x1A,0x29,9) for w in bus.write_audit)
    finally:
        core.writer.latch_fault_without_pause('offline test complete');core.close();core.rec._dumped=True


def test_legacy_hold_metadata_is_invalid_not_valid_zero():
    msg=SimpleNamespace();populate_force_feedback(msg,{})
    assert not msg.force_hold_valid and not msg.force_mode_confirmed


@pytest.mark.parametrize('failure',['imu_timeout','mode_lost','travel','invalid_force','hold_changed'])
def test_ros_node_fixed_lag_to_native_force_and_faults_preserve_mode(failure):
    import rclpy
    from sensor_msgs.msg import Imu
    from std_srvs.srv import SetBool
    from pus6_msgs.msg import TimedJointFeedback
    from pus6_force_control.active_stabilization_node import ForceActiveStabilizationNode
    now=[10_000_000_000];rclpy.init();node=ForceActiveStabilizationNode(clock=lambda:now[0]);node.timer.cancel()
    def imu(q):
        m=Imu();m.orientation.x,m.orientation.y,m.orientation.z,m.orientation.w=map(float,q);return m
    def fb(seq,mode=True,travel=False):
        m=TimedJointFeedback();m.uart_cycle_sequence=seq;m.source=m.SOURCE_MOCK
        m.snapshot_start_monotonic_ns=now[0]-2_000_000;m.snapshot_end_monotonic_ns=now[0]
        m.per_axis_rx_done_monotonic_ns=[now[0]-1_000_000]*6
        m.raw_steps=[1000]*6;m.positions_meters=[.05]*6
        if travel:m.raw_steps[2]+=7;m.positions_meters[2]+=.000175
        m.forces_g_raw=HOLD;m.force_valid=[True]*6;m.force_adc_raw=[2048]*6
        m.force_mode_confirmed=mode;m.force_hold_valid=True;m.force_hold_g=HOLD;m.force_hold_reference_monotonic_ns=9_000_000_000
        return m
    try:
        req=SetBool.Request();req.data=True;assert node._enable(req,SetBool.Response()).success
        seq=0
        for k in range(25):
            now[0]=10_000_000_000+k*10_000_000
            q=I if k<12 else Rotation.from_rotvec([.01,0,0]).as_quat()
            node._imu(imu(q),node.base_imu);node._imu(imu(q),node.platform_imu)
            if k%2==0:
                node._handle_timed_feedback(fb(seq));seq+=1;node._control_cycle()
        assert not node.fault_latched,node.fault_reason
        assert node.last_cycle_result is not None and node.last_cycle_result.pid is None
        assert node.last_cycle_result.commanded_wrench[4]<0
        assert node.pipeline.hold_forces_g==tuple(HOLD)
        assert node.axial_cmd_pub.topic_name.endswith('axial_forces_dry_run')
        previous=node.last_processed_cycle_seq;node._control_cycle();assert node.last_processed_cycle_seq==previous
        now[0]+=120_000_000 if failure=='imu_timeout' else 20_000_000
        if failure!='imu_timeout':
            node._imu(imu(q),node.base_imu);node._imu(imu(q),node.platform_imu)
        bad=fb(seq,mode=failure!='mode_lost',travel=failure=='travel')
        if failure=='invalid_force':bad.force_valid[2]=False
        if failure=='hold_changed':bad.force_hold_g[2]+=1
        node._handle_timed_feedback(bad);node._control_cycle()
        assert node.fault_latched
        assert node.stop_request_status == 'DRY_RUN_ONLY'
        assert node.pipeline.hold_forces_g == tuple(HOLD)
    finally:node.destroy_node();rclpy.shutdown()


def test_force_ros_command_qos_is_compatible_and_position_messages_cannot_enter_force_session():
    from rclpy.qos import qos_profile_sensor_data, qos_check_compatible, QoSCompatibility
    from pus6_motor_driver.j14_process_driver import J14ProcessMotorDriverNode, AXIAL_FORCE_COMMAND_QOS
    assert AXIAL_FORCE_COMMAND_QOS.reliability==qos_profile_sensor_data.reliability
    assert AXIAL_FORCE_COMMAND_QOS.durability==qos_profile_sensor_data.durability
    assert qos_check_compatible(qos_profile_sensor_data,AXIAL_FORCE_COMMAND_QOS)[0]!=QoSCompatibility.ERROR
    # These guards return before touching the client or base position backend.
    J14ProcessMotorDriverNode._enqueue_latest_target(SimpleNamespace(native_force_control=True),[1000]*6)
    J14ProcessMotorDriverNode._handle_command_axial_forces(SimpleNamespace(native_force_control=False),None)


def test_ros_dry_topic_delivers_si_force_command_to_parent_handler_without_uart():
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from pus6_research_msgs.msg import AxialForceCommand
    from pus6_force_control.active_stabilization_node import ForceActiveStabilizationNode
    from pus6_motor_driver.j14_process_driver import J14ProcessMotorDriverNode, AXIAL_FORCE_COMMAND_QOS
    rclpy.init();controller=ForceActiveStabilizationNode();controller.timer.cancel();observer=Node('force_parent_handler_offline')
    received=[]
    parent=SimpleNamespace(native_force_control=True,_j7_fault_latched=False,
        _process_client=SimpleNamespace(update_force=lambda f,**kw:received.append(list(f))))
    observer.create_subscription(AxialForceCommand,controller.axial_cmd_pub.topic_name,
        lambda msg:J14ProcessMotorDriverNode._handle_command_axial_forces(parent,msg),AXIAL_FORCE_COMMAND_QOS)
    executor=SingleThreadedExecutor();executor.add_node(observer);executor.add_node(controller)
    try:
        msg=AxialForceCommand();msg.force_n=(np.array(HOLD)*.00980665).tolist()
        deadline=time.monotonic()+2
        while not received and time.monotonic()<deadline:
            msg.sample_monotonic_ns=time.monotonic_ns();controller.axial_cmd_pub.publish(msg);executor.spin_once(timeout_sec=.02)
        assert received and received[0]==pytest.approx(msg.force_n)
    finally:executor.shutdown();controller.destroy_node();observer.destroy_node();rclpy.shutdown()


def test_guarded_native_handover_uses_existing_partial_recovery_without_pause(tmp_path):
    class MovingBus(MockServoBus):
        def request(self,frame,**kw):
            result=super().request(frame,**kw);f=parse_frame(frame)
            if f.command==0x32 and f.register_addr==0x25 and f.motor_id==3 and f.data[:2]==b'\x03\x00':
                self.positions[3]+=7
            return result
    bus=MovingBus();bus.modes={i:0 for i in range(1,7)};bus.positions={i:1000 for i in range(1,7)};bus.actual_forces=dict(enumerate(HOLD,1))
    core=ServoCore(dict(epoch=1,hardware_mode='mock',output_dir=str(tmp_path),control_mode='force',
        dry_run=False,max_force_handover_displacement_steps=6),bus=bus)
    try:
        with pytest.raises(Exception,match='displacement'):core.initialize()
        assert list(bus.modes.values())==[0]*6
        assert not any(w['register_address']==0x1A for w in bus.write_audit)
        assert not any(w['motor_id']>3 for w in bus.write_audit)
    finally:core.rec._dumped=True


def test_periodic_mode_readback_stops_force_writes_without_automatic_mode0(tmp_path):
    bus=MockServoBus();bus.modes={i:0 for i in range(1,7)};bus.positions={i:1000 for i in range(1,7)};bus.actual_forces=dict(enumerate(HOLD,1))
    core=ServoCore(dict(epoch=1,hardware_mode='mock',output_dir=str(tmp_path),control_mode='force',
        dry_run=False,hold_anchor_on_timeout=True,force_mode_check_period_s=1.),bus=bus)
    try:
        core.initialize();before=len(bus.write_audit);bus.modes[2]=0
        packet=core.cycle(time.monotonic_ns())
        assert packet['type']=='fault' and core.fault_latched
        assert not any(w['register_address'] == 0x27 for w in bus.write_audit[before:])  # no force commands
        assert not core.force_feedback_fields()['force_mode_confirmed']
    finally:core.writer.latch_fault_without_pause('done');core.close();core.rec._dumped=True


def test_formal_production_entry_passes_force_options_without_position_startup_or_real_process(tmp_path,monkeypatch):
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from pus6_motor_driver import production_driver as production
    from pus6_motor_driver.j14_process_driver import J14ProcessMotorDriverNode as Parent
    rclpy.init();node=Node('offline_production_force_options',parameter_overrides=[
        Parameter('control_mode',value='force'),Parameter('force_dry_run',value=False)])
    captured=[]
    class Client:
        def __init__(self,config):
            captured.append(config);self.ready={'pid':123};self.bridge_proc=SimpleNamespace(pid=124)
        def update(self,*a,**kw):raise AssertionError('position startup must never run')
    monkeypatch.setattr(production,'ProcessClient',Client)
    stub=SimpleNamespace(_production_options={**production.FORMAL_DEFAULTS,**production.THREE_DEFAULTS},
        _production_audit_path=tmp_path,_stall_recorder=None,servo_speed=50,maintenance_mode=True,
        target_timeout_sec=.2,reconnect_max_jump_steps=80,hardware_mode=SimpleNamespace(value='mock'),sub_cb_group=None,
        has_parameter=node.has_parameter,declare_parameter=node.declare_parameter,get_parameter=node.get_parameter,
        get_logger=node.get_logger,create_subscription=node.create_subscription,
        _handle_command_axial_forces=lambda m:None)
    stub._native_force_child_options=lambda:Parent._native_force_child_options(stub)
    stub._subscribe_native_force_commands=lambda:Parent._subscribe_native_force_commands(stub)
    try:
        production.ProductionDriverNode._init_j7_servo_backend(stub,'NEVER_OPEN',921600,.05)
        config=captured[0]
        assert config['control_mode'] == 'force'
        assert config['hardware_write_enabled'] is False and config['is_hardware_qualified'] is False
        assert stub._axial_forces_sub is not None
    finally:node.destroy_node();rclpy.shutdown()


@pytest.mark.parametrize('authorized,qualified',[(False,True),(True,False)])
def test_live_owner_rejects_missing_independent_permission_before_uart(authorized,qualified):
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from pus6_motor_driver.j14_process_driver import J14ProcessMotorDriverNode as Parent
    rclpy.init();node=Node('force_owner_permission_offline',parameter_overrides=[
        Parameter('control_mode',value='force'),Parameter('force_dry_run',value=False),
        Parameter('force_hardware_write_enabled',value=authorized),Parameter('force_is_hardware_qualified',value=qualified)])
    node.hardware_mode=SimpleNamespace(value='active')
    try:
        with pytest.raises(ValueError,match='before opening UART'):Parent._native_force_child_options(node)
        assert not hasattr(node,'_process_client')
    finally:node.destroy_node();rclpy.shutdown()


def test_partial_h30_orientation_and_gyro_preserve_their_own_source_times():
    h=ImuHistory();h.push(100,I,None);h.push(102,None,[0,0,0]);h.push(120,I,None);h.push(122,None,[2,0,0])
    assert h.latest_complete_ns==120
    q,gyro=h.query(110,30)
    assert abs(q[3])==pytest.approx(1) and gyro==pytest.approx([.8,0,0])
    h.push(140,I,None)
    assert h.latest_complete_ns==122  # an orientation cannot make stale gyro fresh


def test_partial_h30_messages_are_not_rejected_as_malformed_full_samples():
    import rclpy
    from sensor_msgs.msg import Imu
    from pus6_force_control.active_stabilization_node import ForceActiveStabilizationNode
    now=[10_000_000_000];rclpy.init();node=ForceActiveStabilizationNode(clock=lambda:now[0]);node.timer.cancel()
    try:
        q=Imu();q.orientation.w=1.;q.angular_velocity_covariance[0]=-1.
        node._imu(q,node.base_imu);now[0]+=1_000_000
        g=Imu();g.orientation_covariance[0]=-1.;node._imu(g,node.base_imu)
        assert not node.fault_latched
        assert node.base_imu.samples[-1][0]!=node.base_imu.gyros[-1][0]
    finally:node.destroy_node();rclpy.shutdown()
