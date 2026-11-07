"""Software-only verification of fault-axis-first existing same-frame exit."""
import pytest
from pus6_motor_driver.fake_bus import MockServoBus
from pus6_motor_driver.protocol import parse_frame
from pus6_motor_driver.j7_six_axis_servo_transport import J7SixAxisServoTransport, ServoTransportError


class RecordedBus(MockServoBus):
    def __init__(self):
        super().__init__()
        self.positions={i:1000+i for i in range(1,7)}
        self.modes={i:3 for i in range(1,7)}
        self.calls=[]

    def request(self,frame,**kwargs):
        self.calls.append((parse_frame(frame),kwargs))
        return super().request(frame,**kwargs)


def mode_writes(bus):
    return [f for f,_ in bus.calls if f.command==0x32 and f.register_addr==0x25]


def test_fault_axis_is_anchored_first_with_id_ordered_return_and_no_pause():
    bus=RecordedBus();pauses=[];writer=J7SixAxisServoTransport(bus,pause_callback=pauses.append)
    writer.latch_fault_without_pause('axis 4 travel')
    anchors=writer.transition_to_position_mode(recovery=True,priority_motor_ids=[4])
    assert [f.motor_id for f in mode_writes(bus)]==[4,1,2,3,5,6]
    assert anchors==[1001,1002,1003,1004,1005,1006]
    assert all(m==0 for m in bus.modes.values())
    assert writer.stopped and not pauses
    assert all(f.register_addr!=0x1A for f,_ in bus.calls)
    assert all(kw.get('retries')==0 for _,kw in bus.calls)
    for f in mode_writes(bus):
        assert f.data[:2]==b'\x00\x00' and len(f.data)==10
        assert int.from_bytes(f.data[8:10],'little')==1000+f.motor_id
    with pytest.raises(ServoTransportError):writer.send_force_cycle([0]*6,cycle_id=1)


def test_supplied_anchors_remain_in_id_order_and_default_order_is_unchanged():
    for priority,order in [([],[1,2,3,4,5,6]),([4,2],[4,2,1,3,5,6])]:
        bus=RecordedBus();writer=J7SixAxisServoTransport(bus)
        assert writer.transition_to_position_mode([901,902,903,904,905,906],recovery=True,
            priority_motor_ids=priority)==[901,902,903,904,905,906]
        assert [f.motor_id for f in mode_writes(bus)]==order
        for f in mode_writes(bus):assert int.from_bytes(f.data[8:10],'little')==900+f.motor_id


def test_already_mode0_axis_gets_only_reads():
    bus=RecordedBus();bus.modes[4]=0;writer=J7SixAxisServoTransport(bus)
    anchors=writer.transition_to_position_mode(recovery=True,priority_motor_ids=[4])
    assert anchors[3]==1004
    assert [f.motor_id for f in mode_writes(bus)]==[1,2,3,5,6]


def test_priority_write_failure_latches_and_does_not_retry_or_touch_later_axes():
    bus=RecordedBus();bus.fail_on_id=4;pauses=[]
    writer=J7SixAxisServoTransport(bus,pause_callback=pauses.append)
    with pytest.raises(ServoTransportError):writer.transition_to_position_mode(recovery=True,priority_motor_ids=[4])
    assert writer.stopped and not pauses
    assert [f.motor_id for f in mode_writes(bus)]==[4]


@pytest.mark.parametrize('priority',[[0],[7],[4,4],[True]])
def test_invalid_priority_rejected_before_any_uart_transaction(priority):
    bus=RecordedBus();writer=J7SixAxisServoTransport(bus)
    with pytest.raises(ValueError):writer.transition_to_position_mode(recovery=True,priority_motor_ids=priority)
    assert not bus.calls
