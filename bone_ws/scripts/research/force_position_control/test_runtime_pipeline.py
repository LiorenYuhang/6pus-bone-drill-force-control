import json
import time
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from std_msgs.msg import Float64MultiArray, Empty
from pus6_motor_driver.fake_bus import FakeBus
from ros_node import ForcePositionNode
from runtime import UartNode, SessionLog, HERE, load_config
from uart_driver import SixAxisPositionDrive
from calibration import relative_config


def test_dds_force_position_output_reaches_position_protocol(tmp_path):
    c=json.loads((HERE/'hardware_config.json').read_text())
    c['calibration']={'raw_zero':[0.]*6,'newtons_per_raw':[.01]*6}
    c['controller'].update(raw_zero=[0.]*6,newtons_per_raw=[.01]*6)
    c['controller']['max_age_s']=1.
    c['shaper']={}
    rclpy.init(); log=SessionLog(tmp_path/'telemetry.jsonl')
    uart=UartNode(c,log,active=True,hold_current=False,select_servo=False)
    control=ForcePositionNode([
        Parameter('config_json',value=json.dumps({'controller':c['controller'],'shaper':c['shaper']})),
        Parameter('target_max_age_s',value=2.),
        Parameter('position_topic',value='/force_position/position_feedback'),
        Parameter('force_topic',value='/force_position/raw_force_feedback')])
    bus=FakeBus(); bus.positions={i:1000 for i in range(1,7)}; bus.targets=bus.positions.copy()
    bus.modes={i:1 for i in range(1,7)}
    uart.drive=SixAxisPositionDrive(bus,max_start_delta_steps=c['max_start_delta_steps'],
        max_cycle_delta_steps=c['max_cycle_delta_steps'],event=log.record,
        max_speed_m_s=c['shaper'].get('max_speed_m_per_sec',.017))
    uart.drive.initialize(active=True)
    pub=uart.create_publisher(Float64MultiArray,'/research_force_torque_mock/desired_lengths_m',10)
    ex=SingleThreadedExecutor(); ex.add_node(control); ex.add_node(uart)
    try:
        end=time.monotonic()+2
        while time.monotonic()<end:
            pub.publish(Float64MultiArray(data=[.0502]*6))
            snapshot=uart.drive.command(uart.target[0]) if uart.target else uart.drive.read()
            uart.publish_snapshot(snapshot)
            ex.spin_once(timeout_sec=.005)
            ex.spin_once(timeout_sec=.005)
            if uart.drive.position_acknowledgements>=12 and any(v!=1000 for v in bus.targets.values()): break
            time.sleep(.01)
        assert control.fault is None
        assert uart.drive.position_acknowledgements>=12
        assert any(v!=1000 for v in bus.targets.values())
        assert control.controller.state['force_cmd_n'] is not None
        assert {e[2] for e in bus.events if e[1]==0x32}=={0x29}
        uart.on_stop(Empty())
        assert uart.stop_event.is_set() and uart.stop_reason=='stop topic requested'
    finally:
        uart.drive.pause_once(); ex.shutdown()
        control.destroy_node(); uart.destroy_node(); log.close(); rclpy.shutdown()


def test_uncalibrated_active_rejected_before_port_access():
    with pytest.raises(ValueError,match='calibration required'):
        load_config(HERE/'hardware_config.json',active=True,hold=False)


def test_relative_runtime_does_not_publish_or_record_calibrated_newtons(tmp_path):
    c=relative_config(json.loads((HERE/'hardware_config.json').read_text()),[100.]*6,{})
    rclpy.init(); log=SessionLog(tmp_path/'relative.jsonl')
    uart=UartNode(c,log,active=False,hold_current=False,select_servo=False)
    bus=FakeBus()
    uart.drive=SixAxisPositionDrive(bus,max_start_delta_steps=20,max_cycle_delta_steps=2,event=log.record)
    try:
        uart.publish_snapshot(uart.drive.read())
        assert uart.relative_tared and not uart.calibrated
        assert uart.latest_feedback['force_relative']==[-100.]*6
        assert uart.latest_feedback['force_n'] is None
        assert uart.latest_feedback['force_unit']=='relative_raw'
    finally:
        uart.destroy_node(); log.close(); rclpy.shutdown()
