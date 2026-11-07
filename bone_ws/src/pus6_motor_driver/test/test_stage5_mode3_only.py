"""Explicit new Mode3-only policy must never inherit Mode0/Pause recovery."""
import time
import pytest
from pus6_motor_driver.fake_bus import MockServoBus
from pus6_motor_driver.protocol import parse_frame
from pus6_motor_driver.j7_six_axis_servo_transport import J7SixAxisServoTransport
from pus6_motor_driver.j14_process_uart import ServoCore


@pytest.mark.parametrize('lost_ack',[False,True])
def test_partial_entry_no_position_recovery_no_pause(lost_ack):
    class Failure(MockServoBus):
        def request(self,raw,**kw):
            f=parse_frame(raw)
            if f.command==0x32 and f.register_addr==0x25 and f.motor_id==3:
                if lost_ack:super().request(raw,**kw)
                raise TimeoutError('no retry')
            return super().request(raw,**kw)
    bus=Failure();bus.modes={i:0 for i in range(1,7)};pauses=[]
    w=J7SixAxisServoTransport(bus,pause_callback=pauses.append)
    with pytest.raises(Exception) as e:w.configure_force_mode([-100]*6,recover_to_position_on_failure=False)
    report=e.value.handover_report
    assert report['status']=='latched_mode_unchanged'
    assert [a['final_mode'] for a in report['axes']]==[3,3,3 if lost_ack else 0,0,0,0]
    assert not any(a['recovery_attempted'] for a in report['axes'])
    assert all(a['register_address']==0x25 and a['payload'][0]==3 for a in bus.write_audit)
    assert not pauses


def test_readonly_attach_then_managed_mode3_close_anchors_mode0(tmp_path):
    bus=MockServoBus();bus.modes={i:3 for i in range(1,7)}
    bus.positions={i:1000 for i in range(1,7)};bus.actual_forces={i:-100 for i in range(1,7)}
    core=ServoCore(dict(epoch=1,hardware_mode='mock',output_dir=str(tmp_path),control_mode='force',
        maintenance_only=True,mode3_only=True,force_fault_recovery='mode3_hold',dry_run=False),bus=bus)
    try:
        before=len(bus.write_audit);core.initialize();assert len(bus.write_audit)==before
        hold=core.prepare_force_handover()['holding_forces_g'];assert hold==[-100]*6
        core.enter_force_mode_with_handover(hold,static_baseline_g=hold)
        core.writer.send_force_cycle(hold,cycle_id=0,target_timestamp_ns=time.monotonic_ns())
        assert core.force_feedback_fields()['force_mode_confirmed']
    finally:
        core.writer.latch_fault_without_pause('test complete');core.close();core.rec._dumped=True
    assert list(bus.modes.values())==[0]*6
    assert core.force_shutdown_report['position_hold_confirmed']
    assert not any(w['register_address'] in (9,0x1a,0x29) for w in bus.write_audit)


def test_stage5_trace_blocks_mode0_tare_pause_before_any_serial_write(tmp_path):
    from pus6_force_control.stage5_experiment import Mode3OnlyTrace
    from pus6_motor_driver.protocol import pack_write_register_cmd
    class Port:
        def __init__(self):self.writes=[]
        def write(self,b):self.writes.append(b);return len(b)
    port=Port();trace=Mode3OnlyTrace(port,tmp_path)
    for reg,values in [(9,[1]),(0x1a,[1]),(0x29,[1000]),(0x25,[0,0,0,0,1000]),(0x25,[3,0,0,0,1000])]:
        with pytest.raises(RuntimeError):trace.write(pack_write_register_cmd(1,reg,values))
    assert port.writes==[]
    for reg,values in [(0x25,[3,0,65500]),(0x27,[65500])]:trace.write(pack_write_register_cmd(1,reg,values))
    assert len(port.writes)==2


def test_actual_stage5_moving_trace_is_never_accepted_as_current_force_stop():
    import json
    from pathlib import Path
    from pus6_force_control.stage5_experiment import stationary_position_window
    repo=Path(__file__).parents[4]
    rows=[json.loads(l) for l in (repo/'results/stage5_position_force/20261104_164657_joint_trials/tracking.jsonl').read_text().splitlines()]
    assert not stationary_position_window(rows[-100:])
    rows=[json.loads(l) for l in (repo/'results/stage5_position_force/20261104_164611_static/tracking.jsonl').read_text().splitlines()]
    assert stationary_position_window(rows[-100:])
    assert not stationary_position_window(rows[-10:]) # too short, never assume stopped
