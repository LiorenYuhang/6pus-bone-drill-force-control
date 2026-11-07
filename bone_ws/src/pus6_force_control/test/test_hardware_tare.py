"""Targeted tests for the new Tare/exit integration, without hardware access."""
import threading
import pytest

from pus6_force_control.hardware_tare import HardwareTareController
from pus6_motor_driver.fake_bus import MockServoBus
from pus6_motor_driver.protocol import parse_frame
from pus6_motor_driver.j7_real_uart_adapter import J7UartAdapter
from pus6_motor_driver.j7_six_axis_servo_transport import J7SixAxisServoTransport
from pus6_motor_driver.j14_process_uart import ServoCore


def completion(ack):
    raw = bytearray(ack)
    raw[4] = 0x40
    raw[5:7] = b'\x1c\x00'
    raw[-1] = sum(raw[2:-1]) & 255
    return bytes(raw)


class TareBus(MockServoBus):
    def __init__(self, fail=None, zero=True):
        super().__init__()
        self.modes = {mid:0 for mid in range(1,7)}
        self.actual_forces = {mid:-100*mid for mid in range(1,7)}
        self.pending = None
        self.calls = []
        self.fail = fail
        self.zero = zero

    def request(self, frame, **kwargs):
        p = parse_frame(frame)
        self.calls.append((p, kwargs))
        assert self.pending is None, 'ordinary query interleaved before async completion'
        raw = super().request(frame, **kwargs)
        if p.command == 0x32 and p.register_addr == 9:
            self.pending = (p.motor_id, completion(raw))
        return raw

    def wait_for_response(self, *, expected_id, expected_command, timeout_s):
        assert expected_command == 0x40 and self.pending[0] == expected_id
        mid, raw = self.pending
        if mid == self.fail:
            raise TimeoutError('missing completion')
        if self.zero:self.actual_forces[mid]=0
        self.pending=None
        return raw


def controller(bus, **overrides):
    args=dict(hardware_write_enabled=True,hardware_qualified=True)
    args.update(overrides)
    return HardwareTareController(bus,threading.RLock(),**args)


def test_six_axes_are_serial_one_shot_and_keep_raw_before_values():
    bus=TareBus();tare=controller(bus)
    result=tare.tare_all_axes(samples=3,sample_interval_s=0)
    assert [r['before_force_g'] for r in result]==[-100,-200,-300,-400,-500,-600]
    assert all(r['state']=='confirmed' and r['after_forces_g']==[0]*3 for r in result)
    tare.tare_all_axes(samples=3,sample_interval_s=0)
    writes=[(p.motor_id,p.register_addr) for p,_ in bus.calls if p.command==0x32]
    assert writes==[(mid,9) for mid in range(1,7)]
    assert all(kwargs['retries']==0 for _,kwargs in bus.calls)


@pytest.mark.parametrize('gate', ['dry_run','hardware_write_enabled','hardware_qualified'])
def test_independent_tare_write_conditions(gate):
    bus=TareBus();value=True if gate=='dry_run' else False
    with pytest.raises(RuntimeError):controller(bus,**{gate:value}).tare_axis(1)
    assert not bus.write_audit


def test_missing_completion_aborts_later_axes_and_cannot_resend():
    bus=TareBus(fail=3);tare=controller(bus)
    with pytest.raises(TimeoutError):tare.tare_all_axes(samples=2,sample_interval_s=0)
    with pytest.raises(RuntimeError,match='fault latched'):tare.tare_axis(3)
    assert [p.motor_id for p,_ in bus.calls if p.command==0x32]==[1,2,3]
    assert tare.axes[3].state=='failed' and not tare.axes[4].write_attempted


def test_completion_alone_is_not_zero_confirmation():
    bus=TareBus(zero=False);tare=controller(bus)
    with pytest.raises(RuntimeError,match='confirmation tolerance'):
        tare.tare_axis(1,samples=2,sample_interval_s=0)
    assert tare.axes[1].state=='failed'


def test_mode3_tare_is_rejected_before_write():
    bus=TareBus();bus.modes[1]=3
    with pytest.raises(RuntimeError,match='Mode 0'):controller(bus).tare_axis(1)
    assert not bus.write_audit


def test_recorded_completion_can_be_verified_without_repeating_tare():
    bus=TareBus();first=controller(bus)
    first.tare_axis(1,samples=2,sample_interval_s=0,completion_settle_s=0)
    prior=first.snapshot()[1]
    prior['state']='failed';prior['error']='post-completion query timeout'
    second=controller(bus)
    second.verify_completed_axis(prior,samples=2,sample_interval_s=0)
    second.tare_axis(1,samples=2,sample_interval_s=0)
    assert len(bus.write_audit)==1
    bad=dict(prior,completion_hex=None)
    with pytest.raises(RuntimeError,match='No recorded completed'):
        controller(bus).verify_completed_axis(bad)


def test_existing_rx_owner_retains_completion_already_behind_ack():
    bus=TareBus()
    from pus6_motor_driver.protocol import pack_write_register_cmd
    ack=bus.request(pack_write_register_cmd(1,9,[1]))
    class EmptySerial:
        in_waiting=0
        def read(self,size):return b''
    adapter=J7UartAdapter(EmptySerial())
    adapter.rx_owner.ingest(ack+completion(ack))
    assert adapter.rx_owner.take_matching(expected_id=1,expected_command=0x32,
                                          expected_register=9).raw==ack
    assert adapter.wait_for_response(expected_id=1,expected_command=0x40)==completion(ack)
    assert adapter.rx_owner.residual_frame_count==0


def test_batched_mode0_exit_and_partial_failure_rollback_never_pause():
    bus=TareBus();bus.modes={mid:3 for mid in range(1,7)}
    pauses=[];writer=J7SixAxisServoTransport(bus,pause_callback=pauses.append)
    original=dict(bus.positions)
    writer.transition_to_position_mode()
    writes=[p for p,_ in bus.calls if p.command==0x32]
    assert len(writes)==6
    for p in writes:
        vals=[int.from_bytes(p.data[i:i+2],'little') for i in range(0,len(p.data),2)]
        assert p.register_addr==0x25 and len(vals)==5
        assert vals[0]==0 and vals[4]==original[p.motor_id]
    assert not pauses
    bus=TareBus();bus.fail_on_id=3
    writer=J7SixAxisServoTransport(bus,pause_callback=pauses.append)
    with pytest.raises(Exception):writer.configure_force_mode([0]*6)
    assert all(bus.modes[mid]==0 for mid in range(1,7))
    assert not any(p.command==0x32 and p.motor_id>3 for p,_ in bus.calls)
    assert not pauses and writer.stopped


def test_child_maintenance_does_not_tare_on_start_and_hardware_reference_has_no_subtraction(tmp_path):
    bus=TareBus()
    core=ServoCore(dict(epoch=1,hardware_mode='mock',control_mode='force',maintenance_only=True,
                       output_dir=str(tmp_path),dry_run=False,hardware_write_enabled=True,
                       is_hardware_qualified=True),bus=bus)
    try:
        core.initialize()
        assert not bus.write_audit
        core.tare_all_force_axes(samples=2,sample_interval_s=0)
        assert core.baseline_manager.software_baseline_g is None
        snapshot=core.prepare_force_handover()
        assert snapshot['software_baseline_subtracted'] is False
        assert core.baseline_manager.software_baseline_g is None
        core.enter_force_mode()
        assert core.mode_3_active
        assert core.force_handover_snapshot['software_baseline_subtracted'] is False
        assert list(bus.target_forces.values())==[0]*6
        core.exit_force_mode_safe()
        assert list(bus.modes.values())==[0]*6
        core.writer.latch_fault_without_pause('maintenance complete')
        core.close()
        assert not core.pauses
    finally:
        core.rec._dumped=True
