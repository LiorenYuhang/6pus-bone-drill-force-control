"""Targeted numerical checks of retained Position PID and Mode 3 conversion.

No serial port, hardware nodes, or physical plant is used.
"""
import math
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from pus6_force_control.position_attitude_pid import M2PidGains, M2PidLimits, M2TaskSpaceAttitudePid
from pus6_force_control.active_stabilization import (
    ActiveStabilizer, StabilizationConfig, PoseWrenchPID, C_IMU_TO_MECHANICAL as C)
from pus6_force_control.force_pipeline import TaskSpaceForcePipeline
from pus6_motor_driver.force_units import g_to_n

I=[0.,0.,0.,1.]


def test_retained_position_pid_terms_filter_and_angular_limit():
    # Independently evaluated Position recurrence, including its first-D rule.
    pid=M2TaskSpaceAttitudePid(M2PidGains(np.array([.05,.05,.025]),np.zeros(3),np.array([0.,.0015,.0015])),
        M2PidLimits(residual_limit_rad=np.radians([.5]*3),derivative_lpf_cutoff_hz=5.))
    e0=np.array([.01,-.02,.03]);e1=np.array([.02,-.01,.01]);e2=np.array([.01,0.,-.01]);dt=.02
    assert pid.update(e0,dt)==pytest.approx(pid.gains.kp*e0)
    d1=(e1-e0)/dt
    assert pid.update(e1,dt)==pytest.approx(pid.gains.kp*e1+pid.gains.kd*d1)
    alpha=(1/(2*math.pi*5))/(1/(2*math.pi*5)+dt)
    d2=alpha*d1+(1-alpha)*(e2-e1)/dt
    assert pid.update(e2,dt)==pytest.approx(pid.gains.kp*e2+pid.gains.kd*d2)
    assert pid.update([10]*3,dt)==pytest.approx(np.radians([.5]*3))
    old=pid.last_cmd.copy();assert pid.update(e0,.0001)==pytest.approx(old)
    pid.disable();assert pid.update(e0,dt)==pytest.approx([0]*3)
    pid.enable();assert not pid.has_prev_error


@pytest.mark.parametrize('axis',range(3))
def test_base_feedforward_without_platform_error_or_fusion(axis):
    now=[1_000_000_000];c=ActiveStabilizer(clock=lambda:now[0]);p=np.array([0,0,.133,0,0,0])
    c.capture(p,I,I,now[0]);now[0]+=20_000_000
    angle=np.eye(3)[axis]*.01;q=Rotation.from_rotvec(angle).as_quat()
    wrench,d=c.update(p,q,I,[0]*3,[0]*3,sample_ns=now[0],now_ns=now[0])
    assert d['platform_error_world_rad']==pytest.approx([0]*3,abs=1e-12)
    assert d['attitude_residual_world_rad']==pytest.approx([0]*3,abs=1e-12)
    assert wrench[3:]@(C@angle)<0  # inverse Base drives the force interface by itself
    expected=C@Rotation.from_quat(q).as_matrix().T@C.T
    assert np.array(d['command_rotation_base'])==pytest.approx(expected)


@pytest.mark.parametrize('axis',range(3))
def test_platform_pid_residual_and_fk_tracking_are_separate(axis):
    now=[1_000_000_000];c=ActiveStabilizer(clock=lambda:now[0]);p=np.array([0,0,.133,0,0,0])
    c.capture(p,I,I,now[0]);now[0]+=20_000_000
    angle=np.eye(3)[axis]*.01;q=Rotation.from_rotvec(angle).as_quat()
    wrench,d=c.update(p,I,q,[0]*3,[0]*3,sample_ns=now[0],now_ns=now[0])
    e=-C@angle;residual=np.array(c.config.attitude_kp)*e
    assert d['attitude_residual_world_rad']==pytest.approx(residual)
    assert d['pose_error_m_rad'][3:]==pytest.approx(residual)
    assert wrench[3:]==pytest.approx(np.array(c.config.pose_kp[3:])*residual)
    # The angular residual is never relabeled Nm or added as a force reference.
    assert not np.allclose(wrench[3:],residual,atol=1e-10)


def test_position_composition_noncommuting_rotations_and_nonidentity_capture():
    now=[1_000_000_000];c=ActiveStabilizer(clock=lambda:now[0])
    p=np.array([0,0,.133,.02,-.03,.04]);c.capture(p,I,I,now[0]);now[0]+=20_000_000
    qb=Rotation.from_rotvec([.01,-.02,.03]).as_quat();qp=Rotation.from_rotvec([-.02,.03,.01]).as_quat()
    _,d=c.update(p,qb,qp,[0]*3,[0]*3,sample_ns=now[0],now_ns=now[0])
    rb=C@Rotation.from_quat(qb).as_matrix()@C.T;r0=Rotation.from_euler('xyz',p[3:]).as_matrix()
    delta=Rotation.from_rotvec(d['attitude_residual_world_rad']).as_matrix()
    # Simplifies original Position conjugation without using the implementation's product.
    assert np.array(d['command_rotation_base'])==pytest.approx(rb.T@delta@r0)


def test_pose_pid_si_output_no_force_reference_feedforward_or_gyro_invention():
    c=StabilizationConfig(pose_kp=(10.,20.,30.,.2,.3,.4))
    pid=PoseWrenchPID(c);error=np.array([.001,-.002,.003,.01,-.02,.03])
    raw,out=pid.update(error,.02)
    assert raw==pytest.approx(np.array(c.pose_kp)*error)
    assert out==pytest.approx(raw)
    now=[1_000_000_000];controller=ActiveStabilizer(clock=lambda:now[0]);p=[0,0,.133,0,0,0]
    controller.capture(p,I,I,now[0]);now[0]+=20_000_000
    w,_=controller.update(p,I,I,[1,2,3],[4,5,6],sample_ns=now[0],now_ns=now[0])
    assert w==pytest.approx([0]*6)  # retained Position D acts on SO(3) error, not an invented gyro blend


def test_task_pid_saturation_and_downstream_limit_do_not_wind_up():
    cfg=StabilizationConfig(pose_kp=(0.,)*6,pose_ki=(1.,)*6)
    pid=PoseWrenchPID(StabilizationConfig(pose_kp=(.1,)*6,pose_ki=(1.,)*6))
    _,out=pid.update([100]*6,.02)
    assert out==pytest.approx([1.,1.,1.,.1,.1,.1])
    assert pid.integral==pytest.approx([0]*6)  # P+candidate I exceeds bounds: reject I
    pid=PoseWrenchPID(cfg)
    pid.update([.1]*6,.02);assert (pid.integral>0).all()
    pid.track_applied([1]*6,[0]*6)
    assert pid.integral==pytest.approx([0]*6)
    now=[1_000_000_000];controller=ActiveStabilizer(cfg,clock=lambda:now[0]);p=np.array([0,0,.133,0,0,0])
    controller.capture(p,I,I,now[0]);now[0]+=20_000_000
    p[3]+=.001;controller.update(p,I,I,[0]*3,[0]*3,sample_ns=now[0],now_ns=now[0])
    controller.track_actuator_output([0]*6,True)
    assert controller.pose_pid.integral==pytest.approx([0]*6)
    controller.reset();assert controller.reference is None
    assert controller.pose_pid.integral==pytest.approx([0]*6)
    assert controller.attitude_pid.integral==pytest.approx([0]*3)


def test_actuator_force_feedback_changes_monitoring_not_hold_or_reference():
    now=[1_000_000_000];pipe=TaskSpaceForcePipeline(clock=lambda:now[0]);hold=[-68,-187,-133,-45,18,-6]
    pipe.initialize_hold_forces_g(hold);p=[0,0,pipe.allocator.kin.nominal_height,0,0,0]
    now[0]+=20_000_000;w=np.array([0,0,0,.0001,-.0002,.0001])
    measured=np.array(hold)+[10,-20,30,-40,50,-60]
    cycle=pipe.command_increment_wrench(w,measured,p,now_ns=now[0])
    assert cycle.measured_wrench==pytest.approx(cycle.allocation.jacobian_6x6.T@np.array(g_to_n(measured-np.array(hold))))
    assert cycle.drive_result['requested_forces_n']==pytest.approx(pipe.hold_forces_n+cycle.allocation.actuator_forces_n)
    assert pipe.hold_forces_g==tuple(hold)
    assert cycle.commanded_wrench==pytest.approx(w)


@pytest.mark.parametrize('kw',[
    {'pose_kp':(1.,)*3}, {'attitude_kp':(-1.,0.,0.)}, {'attitude_min_dt_s':0.},
    {'attitude_max_dt_s':float('nan')}, {'moment_limit_nm':(0.,)*3}])
def test_invalid_pid_configuration_is_rejected(kw):
    with pytest.raises(ValueError):StabilizationConfig(**kw)


def test_configuration_and_installed_ros_parameter_interface():
    from pathlib import Path
    import yaml
    import rclpy
    from rclpy.parameter import Parameter
    from pus6_force_control.active_stabilization_node import ForceActiveStabilizationNode
    path=Path(__file__).parents[1]/'config'/'active_stabilization.yaml'
    values=yaml.safe_load(path.read_text())['force_active_stabilization']['ros__parameters']
    rclpy.init()
    node=None
    try:
        node=ForceActiveStabilizationNode(parameter_overrides=[Parameter(k,value=v) for k,v in values.items()])
        node.timer.cancel()
        assert node.stabilizer.attitude_pid.gains.kp==pytest.approx([.05,.05,.025])
        assert node.stabilizer.attitude_pid.gains.kd==pytest.approx([0.,.0015,.0015])
        assert node.stabilizer.config.pose_kp==pytest.approx([10.,10.,10.,.2,.2,.2])
        assert node.dry_run and not node.enabled
        assert not node.has_parameter('imu_error_weight')
        assert not node.has_parameter('attitude_stiffness_nm_rad')
    finally:
        if node is not None:node.destroy_node()
        rclpy.shutdown()


def test_disable_and_recapture_clear_both_pid_states_without_changing_hold():
    import rclpy
    from std_srvs.srv import SetBool
    from pus6_force_control.active_stabilization_node import ForceActiveStabilizationNode
    rclpy.init();now=[1_000_000_000];node=ForceActiveStabilizationNode(clock=lambda:now[0]);node.timer.cancel()
    try:
        hold=[-68,-187,-133,-45,18,-6];node.initialize_hold_forces_g(hold,now_ns=now[0])
        node.stabilizer.capture([0,0,.133,0,0,0],I,I,now[0])
        node.stabilizer.attitude_pid.integral[:]=1.;node.stabilizer.pose_pid.integral[:]=1.
        request=SetBool.Request();request.data=False
        assert node._enable(request,SetBool.Response()).success
        assert node.stabilizer.reference is None
        assert node.stabilizer.pose_pid.integral==pytest.approx([0]*6)
        assert node.stabilizer.attitude_pid.integral==pytest.approx([0]*3)
        assert node.pipeline.hold_forces_g==tuple(hold)
        request.data=True;assert not node._enable(request,SetBool.Response()).success
        assert node.fault_latched  # explicit reset + new Mode3 handover required after terminal pause
        from std_srvs.srv import Trigger
        node._reset_fault(Trigger.Request(),Trigger.Response())
        assert node._enable(request,SetBool.Response()).success
        assert node.capture_pending  # a new attitude reference, never an implicit force tare
    finally:node.destroy_node();rclpy.shutdown()
