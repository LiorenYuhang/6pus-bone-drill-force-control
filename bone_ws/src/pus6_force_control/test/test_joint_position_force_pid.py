"""Stage5 numeric and ROS tests only; no real UART is opened."""
import numpy as np
import pytest
from pus6_force_control.joint_position_force_pid import JointPositionForcePID,JointForcePIDConfig
from pus6_force_control.force_pipeline import TaskSpaceForcePipeline
from pus6_motor_driver.force_units import g_to_n


def test_position_error_signed_force_integral_derivative_and_real_dt():
    now=[1_000_000_000];p=JointPositionForcePID(clock=lambda:now[0]);q=np.full(6,.05)
    p.reset(q,sample_ns=now[0],now_ns=now[0]);target=q+np.array([1,-1,0,0,0,0])*.00005
    now[0]+=20_000_000;u,d=p.update(target,q,sample_ns=now[0],now_ns=now[0])
    assert u[:2]==pytest.approx([1.01,-1.01]);assert u[2:]==pytest.approx([0]*4)
    assert d['joint_d_n']==pytest.approx([0]*6) # no target-step D kick
    now[0]+=40_000_000;u,d=p.update(target,q,sample_ns=now[0],now_ns=now[0])
    assert u[:2]==pytest.approx([1.03,-1.03]);assert d['joint_dt_s']==pytest.approx(.04)
    with pytest.raises(ValueError,match='new synchronized'):p.update(target,q,sample_ns=now[0],now_ns=now[0]+1)


def test_integral_limits_actuator_feedback_and_quantization():
    now=[1_000_000_000];p=JointPositionForcePID(clock=lambda:now[0]);q=np.full(6,.05)
    p.reset(q,sample_ns=now[0],now_ns=now[0]);now[0]+=20_000_000
    u,d=p.update(q+.00005,q,sample_ns=now[0],now_ns=now[0]);i=p.integral.copy()
    p.track_applied(u-.001);assert p.integral==pytest.approx(i) # g rounding, not force cap
    p.track_applied(np.zeros(6));assert p.integral==pytest.approx([0]*6)
    for k in range(300):
        now[0]+=20_000_000;u,d=p.update(q+.00005,q,sample_ns=now[0],now_ns=now[0])
    assert u==pytest.approx([2.]*6);assert p.integral==pytest.approx([1.]*6)
    now[0]+=20_000_000;u,d=p.update(q,q+.000025,sample_ns=now[0],now_ns=now[0])
    assert max(d['joint_d_n'])<0 # measured-motion damping


def test_torque_limiter_absolute_bounds_and_reference_relative_guard():
    now=[1_000_000_000];p=JointPositionForcePID(clock=lambda:now[0]);q=np.full(6,.05)
    p.reset(q,sample_ns=now[0],now_ns=now[0]);now[0]+=1_000_000
    _,d=p.update(q+.005,q,sample_ns=now[0],now_ns=now[0])
    assert np.array(d['joint_reference_m'])-q==pytest.approx([.000017]*6)
    now[0]+=20_000_000
    with pytest.raises(ValueError,match='tracking'):p.update(q+.005,q,sample_ns=now[0],now_ns=now[0])
    with pytest.raises(ValueError,match='mechanical'):p.validate_position([.08]*6)


def test_actuator_pid_hold_units_feedback_and_force_limit_are_not_wrench_reallocated():
    now=[1_000_000_000];pipe=TaskSpaceForcePipeline(clock=lambda:now[0]);hold=[100,-200,300,-400,500,-600]
    pipe.initialize_hold_forces_g(hold,now_ns=now[0]);pose=[0,0,pipe.allocator.kin.nominal_height,0,0,0]
    delta=np.array([.2,-.3,.4,-.5,.6,-.7]);now[0]+=1_000_000_000
    cycle=pipe.command_actuator_increment(delta,np.array(hold)+[10,-10,20,-20,30,-30],pose,now_ns=now[0])
    assert cycle.drive_result['requested_forces_n']==pytest.approx(np.array(g_to_n(hold))+delta)
    assert cycle.allocation.actuator_forces_n==pytest.approx(delta)
    assert cycle.measured_wrench==pytest.approx(cycle.allocation.jacobian_6x6.T@np.array(g_to_n([10,-10,20,-20,30,-30])))
    for row,h,d in zip(cycle.drive_result['frames'],hold,delta):
        assert row['target_force_g']==pytest.approx(h+d/.00980665,abs=.5)
    now[0]+=10_000_000_000
    cycle=pipe.command_actuator_increment([100]*6,hold,pose,now_ns=now[0])
    assert max(cycle.drive_result['limited_forces_n'])<=14.709975
    assert cycle.actuator_limited


@pytest.mark.parametrize('latest,execution_only',[(False,False),(True,False),(True,True)])
def test_joint_backend_position_m2_ik_target_and_native_force_ros_chain(latest,execution_only):
    import rclpy
    from rclpy.parameter import Parameter
    from sensor_msgs.msg import Imu
    from std_srvs.srv import SetBool
    from pus6_msgs.msg import TimedJointFeedback
    from scipy.spatial.transform import Rotation
    from pus6_force_control.active_stabilization_node import ForceActiveStabilizationNode
    now=[10_000_000_000];rclpy.init()
    node=ForceActiveStabilizationNode(clock=lambda:now[0],parameter_overrides=[Parameter('execution_backend',value='joint_position_force'),
        Parameter('joint_use_latest_feedback',value=latest),Parameter('joint_active_braking',value=latest),
        Parameter('joint_execution_only',value=execution_only)]);node.timer.cancel()
    hold=[-120,-251,-223,-91,-90,0]
    try:
        req=SetBool.Request();req.data=True;node._enable(req,SetBool.Response())
        for k in range(35):
            now[0]=10_000_000_000+k*10_000_000
            m=Imu();q=Rotation.from_rotvec([.0001 if k>15 else 0.,0.,0.]).as_quat()
            m.orientation.x,m.orientation.y,m.orientation.z,m.orientation.w=map(float,q)
            if not execution_only:node._imu(m,node.base_imu);node._imu(m,node.platform_imu)
            if execution_only and k>15:node.joint_reference_override_m=np.full(6,.05005)
            if k%2==0:
                fb=TimedJointFeedback();fb.uart_cycle_sequence=k+1;fb.source=fb.SOURCE_MOCK
                fb.snapshot_start_monotonic_ns=now[0]-2_000_000;fb.snapshot_end_monotonic_ns=now[0]
                fb.per_axis_rx_done_monotonic_ns=[now[0]-1_000_000]*6
                fb.raw_steps=[1000]*6;fb.positions_meters=[.05]*6;fb.force_adc_raw=[2048]*6
                fb.forces_g_raw=hold;fb.force_valid=[True]*6;fb.force_mode_confirmed=True
                fb.force_hold_valid=True;fb.force_hold_g=hold;fb.force_hold_reference_monotonic_ns=9_000_000_000
                node._handle_timed_feedback(fb);node._control_cycle()
        assert not node.fault_latched,node.fault_reason
        d=node.stabilization_diagnostics
        target=np.array(d['joint_target_m']);desired=(node.joint_reference_override_m if execution_only else node.joint_capture_m+np.array(node.kinematics.inverse_kinematics(d['desired_pose_m_rad']))-node.ik_capture_m)
        assert target==pytest.approx(desired)
        assert node.pipeline.hold_forces_g==tuple(hold)
        assert np.max(abs(target-.05))>0
        assert node.last_cycle_result.drive_result['requested_forces_n']==pytest.approx(node.pipeline.hold_forces_n+np.array(d['joint_delta_force_n']))
        seq=node.last_processed_cycle_seq;node._control_cycle();assert node.last_processed_cycle_seq==seq
        assert node.axial_cmd_pub.topic_name.endswith('axial_forces_dry_run')
        # Fresh hardware batch but temporary missing interpolation bracket:
        # wait, keep controller state, consume no duplicate feedback.
        from types import SimpleNamespace
        from pus6_kinematics.state_synchronizer import SyncStatus
        original=node.synchronizer.query_sync_configuration
        node.synchronizer.query_sync_configuration=lambda t:SimpleNamespace(status=SyncStatus.WAIT_FUTURE_SAMPLE)
        now[0]+=20_000_000;fb.uart_cycle_sequence+=2
        fb.snapshot_start_monotonic_ns=now[0]-2_000_000;fb.snapshot_end_monotonic_ns=now[0]
        fb.per_axis_rx_done_monotonic_ns=[now[0]-1_000_000]*6
        node._imu(m,node.base_imu);node._imu(m,node.platform_imu);node._handle_timed_feedback(fb)
        node._control_cycle();assert not node.fault_latched,node.fault_reason
        assert (node.last_processed_cycle_seq>seq) if execution_only else (node.last_processed_cycle_seq==seq)
        if execution_only:
            assert node.last_cycle_result.pose is None
            assert node.last_cycle_result.measured_wrench is None # no mixed-time wrench is published
        node.synchronizer.query_sync_configuration=original
        if latest and not execution_only:
            for k in range(6):
                now[0]+=20_000_000;fb.uart_cycle_sequence+=2
                fb.snapshot_start_monotonic_ns=now[0]-2_000_000;fb.snapshot_end_monotonic_ns=now[0]
                fb.per_axis_rx_done_monotonic_ns=[now[0]-1_000_000]*6
                node._handle_timed_feedback(fb);node._control_cycle()
            assert not node.fault_latched,node.fault_reason
            assert node.imu_target_suspended and node.last_processed_cycle_seq>seq
    finally:node.destroy_node();rclpy.shutdown()


def test_planned_travel_is_not_start_relative_six_steps_and_candidates_remain_dry_run():
    import yaml
    from pathlib import Path
    config=yaml.safe_load((Path(__file__).parents[1]/'config'/'active_stabilization_joint_force.yaml').read_text())['force_active_stabilization']['ros__parameters']
    assert config['dry_run'] and not config['hardware_write_enabled']
    assert config['execution_backend']=='joint_position_force'
    assert config['max_displacement_steps']==0 # explicit planned motion, absolute bounds remain
    cfg=JointForcePIDConfig(**{k:v for k,v in config.items() if k.startswith('joint_')})
    assert cfg.joint_ki_n_m_s==(0.,)*6
    now=[1_000_000_000];p=JointPositionForcePID(cfg,clock=lambda:now[0]);q=np.full(6,.05)
    p.reset(q,sample_ns=now[0],now_ns=now[0])
    # A measured coordinated movement of 200 steps is inside the normal stroke.
    # Tracking error is a separate 2 steps, not a forbidden 202-step start drift.
    now[0]+=500_000_000;measured=q+.005
    u,d=p.update(measured+.00005,measured,sample_ns=now[0],now_ns=now[0])
    assert d['joint_error_m']==pytest.approx([.00005]*6)
    assert max(abs(u))<=2.
    now[0]+=500_000_000
    with pytest.raises(ValueError,match='tracking'):p.update(measured+.002,measured,sample_ns=now[0],now_ns=now[0])
