"""Only the new Stage 3C exit policy, offline; not a hardware result."""
import pytest
import rclpy
from rclpy.parameter import Parameter
from pus6_motor_driver.fake_bus import MockServoBus
from pus6_motor_driver.j7_six_axis_servo_transport import J7SixAxisServoTransport
from pus6_motor_driver.j14_process_uart import ServoCore
from pus6_motor_driver.protocol import parse_frame
from pus6_force_control.force_control_node import TaskSpaceForceControlNode


def test_fault_latched_recovery_does_not_reopen_normal_commands():
    bus=MockServoBus();bus.modes={i:3 for i in range(1,7)}
    pauses=[];writer=J7SixAxisServoTransport(bus,pause_callback=pauses.append)
    writer.latch_fault_without_pause('test fault')
    anchors=writer.transition_to_position_mode(recovery=True)
    assert len(anchors)==6 and all(v==0 for v in bus.modes.values())
    assert writer.stopped and not pauses
    with pytest.raises(Exception):writer.send_force_cycle([0]*6,cycle_id=1,target_timestamp_ns=1)


def test_force_maintenance_fault_policy_never_calls_pause(tmp_path):
    bus=MockServoBus();bus.modes={i:0 for i in range(1,7)}
    core=ServoCore(dict(epoch=1,output_dir=str(tmp_path),hardware_mode='mock',
        maintenance_only=True,control_mode='force',force_fault_recovery='position_reanchor'),bus=bus)
    try:
        core.initialize();core.stop('injected fault')
        assert not core.pauses
        core.writer.transition_to_position_mode(recovery=True)
        assert core.writer.stopped
    finally:
        core.writer.latch_fault_without_pause('done');core.close();core.rec._dumped=True


def test_node_requests_external_reanchor_without_pause():
    rclpy.init()
    node=TaskSpaceForceControlNode('test_stage3c_fault',parameter_overrides=[
        Parameter('dry_run',value=False),Parameter('hardware_write_enabled',value=True),
        Parameter('is_hardware_qualified',value=True),
        Parameter('fault_stop_policy',value='external_position_reanchor')])
    try:
        node.timer.cancel();node._trip_fault('TEST','injected',node.clock())
        assert node.fault_latched and node.stop_request_status=='EXTERNAL_REANCHOR_REQUIRED'
        assert node.pipeline.pid.integral_n==0
    finally:node.destroy_node();rclpy.shutdown()


def test_stage3c_position_mapping_matches_existing_timed_feedback():
    import numpy as np
    from pus6_force_control.force_pipeline import raw_steps_to_slider_m
    from pus6_kinematics.analytical_6pus import Analytical6PusKinematics
    kin=Analytical6PusKinematics();cfg=kin.geometry_config
    home=raw_steps_to_slider_m(cfg.operating_home_raw_steps,cfg)
    assert home==pytest.approx(cfg.operating_home_stroke_m)
    q=raw_steps_to_slider_m([999,998,1001,998,999,998],cfg)
    assert q==pytest.approx([.05+(s-1000)*.000025 for s in [999,998,1001,998,999,998]])
    fk=kin.forward_kinematics_with_metrics(home)
    assert fk.converged and fk.pose[2]==pytest.approx(kin.nominal_height,abs=1e-8)


def test_increment5_profile_cannot_include_10n_or_repeat_static_hold():
    from pus6_force_control.stage3c_experiment import response_stages
    stages=response_stages({'increment_5_response':True,'graded_force_response':False})
    assert [s[1] for s in stages]==[0.,0.,5.,0.,-5.,0.]
    assert stages[0][2]==.10


@pytest.mark.parametrize('positive,limited,allowed',[(4.8,False,True),(-1.,False,False),(4.8,True,False)])
def test_positive_response_must_be_normal_before_negative(positive,limited,allowed):
    from pus6_force_control.stage3c_experiment import positive_response_check
    records=[]
    for stage,measured,target in [('zero_pid',.1,0),('positive_5',positive,5),('return_zero_positive_5',.1,0)]:
        for i in range(5):
            records.append(dict(stage=stage,monotonic_ns=i*100_000_000,measured_z_n=measured,
                target_error_n=target-measured,pid_saturated=False,actuator_limited=limited))
    assert positive_response_check(records)['allowed']==allowed
