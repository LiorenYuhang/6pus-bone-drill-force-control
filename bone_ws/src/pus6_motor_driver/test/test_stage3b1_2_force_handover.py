"""Targeted Stage 3B-1.2 FakeBus tests. No serial device is opened."""
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import pytest
from pus6_motor_driver.fake_bus import MockServoBus
from pus6_motor_driver.j14_process_uart import ServoCore
from pus6_motor_driver.j7_six_axis_servo_transport import J7SixAxisServoTransport
from pus6_motor_driver.j7_servo_transport import ServoTransportError
from pus6_motor_driver.protocol import parse_frame
from pus6_motor_driver.safety import ForceBaselineManager, ForceHandoverState, SafetyViolation


FORCES = [3754, -687, -6176, 793, -671, -6994]


class HandoverBus(MockServoBus):
    def __init__(self, *, fail_axis=None, lost_ack=False, fail_recovery_axis=None, unreadable_axis=None):
        super().__init__()
        self.modes = {mid: 0 for mid in range(1, 7)}
        self.positions = {mid: 990 + mid for mid in range(1, 7)}
        self.targets = dict(self.positions)
        self.actual_forces = dict(enumerate(FORCES, 1))
        self.fail_axis = fail_axis
        self.lost_ack = lost_ack
        self.fail_recovery_axis = fail_recovery_axis
        self.unreadable_axis = unreadable_axis
        self.failed = False
        self.calls = []

    def request(self, frame, **kwargs):
        parsed = parse_frame(frame)
        self.calls.append((parsed, kwargs))
        mid, reg, cmd = parsed.motor_id, parsed.register_addr, parsed.command
        is_entry = cmd == 0x32 and reg == 0x25 and parsed.data[:2] == b'\x03\x00'
        if is_entry and mid == self.fail_axis and not self.failed:
            self.failed = True
            if self.lost_ack:
                super().request(frame, **kwargs)  # motor executes; caller loses ACK
            raise TimeoutError("injected Mode 3 write failure")
        if self.failed and cmd == 0x31 and reg == 0x25 and mid == self.unreadable_axis:
            raise TimeoutError("injected unreadable mode")
        if (self.failed and cmd == 0x32 and reg == 0x25 and len(parsed.data) == 10
                and parsed.data[:2] == b'\x00\x00' and mid == self.fail_recovery_axis):
            raise TimeoutError("injected anchor write failure")
        return super().request(frame, **kwargs)


def writer(bus):
    pause_calls = []
    transport = J7SixAxisServoTransport(bus, pause_callback=pause_calls.append)
    return transport, pause_calls


def assert_no_pause(bus, pause_calls=()):
    assert not pause_calls
    assert not any(frame.command == 0x32 and frame.register_addr == 0x1A for frame, _ in bus.calls)


def config(path, **overrides):
    result = dict(epoch=1, hardware_mode='mock', output_dir=str(path),
                  control_mode='force', min_force_n=-98.0665, max_force_n=98.0665,
                  dry_run=False, max_rate_g_s=500, allow_negative_forces=True)
    result.update(overrides)
    return result


def test_all_six_enter_mode3_with_signed_hold():
    bus = HandoverBus()
    transport, pauses = writer(bus)
    configs = transport.configure_force_mode(FORCES)
    assert len(configs) == 6
    assert list(bus.modes.values()) == [3] * 6
    assert list(bus.target_forces.values()) == FORCES
    assert transport.last_force_handover['status'] == 'mode3_active'
    assert all(c.mode_readback == 3 for c in configs)
    assert_no_pause(bus, pauses)


@pytest.mark.parametrize('lost_ack', [False, True])
def test_third_axis_failure_rolls_back_only_switched_axes(lost_ack):
    bus = HandoverBus(fail_axis=3, lost_ack=lost_ack)
    transport, pauses = writer(bus)
    with pytest.raises(ServoTransportError) as error:
        transport.configure_force_mode(FORCES)
    report = error.value.handover_report
    assert report['failed_axis'] == 3 and report['recovery_complete']
    assert list(bus.modes.values()) == [0] * 6
    assert transport.stopped
    restored = {1, 2, 3} if lost_ack else {1, 2}
    anchor_axes = {frame.motor_id for frame, _ in bus.calls
                   if frame.command == 0x32 and frame.register_addr == 0x25 and len(frame.data) == 10}
    assert anchor_axes == restored
    assert not any(frame.command == 0x32 and frame.motor_id >= 4 for frame, _ in bus.calls)
    for mid in restored:
        frames = [f for f, _ in bus.calls if f.motor_id == mid]
        anchor = next(i for i, f in enumerate(frames)
                      if f.command == 0x32 and f.register_addr == 0x25 and len(f.data) == 10)
        assert frames[anchor - 2].command == 0x31 and frames[anchor - 2].register_addr == 0x2A
        assert frames[anchor - 1].command == 0x31 and frames[anchor - 1].register_addr == 0x27
        assert frames[anchor].data[:2] == b'\x00\x00'
        assert int.from_bytes(frames[anchor].data[8:10], 'little') == bus.positions[mid]
        assert frames[anchor + 1].command == 0x31 and frames[anchor + 1].register_addr == 0x25
        assert frames[anchor + 2].command == 0x31 and frames[anchor + 2].register_addr == 0x29
        assert bus.targets[mid] == bus.positions[mid]
    assert all(kwargs.get('retries') == 0 for _, kwargs in bus.calls)
    with pytest.raises(ServoTransportError, match='stopped'):
        transport.send_force_cycle(FORCES, cycle_id=0)
    transport.stop('outer exception')
    transport.stop('cleanup')
    assert_no_pause(bus, pauses)


def test_recovery_failure_latches_and_reports_actual_modes():
    bus = HandoverBus(fail_axis=3, fail_recovery_axis=1)
    transport, pauses = writer(bus)
    with pytest.raises(ServoTransportError) as error:
        transport.configure_force_mode(FORCES)
    report = error.value.handover_report
    assert not report['recovery_complete']
    assert [axis['final_mode'] for axis in report['axes']] == [3, 0, 0, 0, 0, 0]
    assert report['axes'][0]['error']
    assert bus.modes[2] == 0  # recovery of other switched axes still attempted
    assert transport.stopped
    assert_no_pause(bus, pauses)


def test_failed_axis_with_unknown_mode_is_not_declared_mode0():
    bus = HandoverBus(fail_axis=3, lost_ack=True, unreadable_axis=3)
    transport, pauses = writer(bus)
    with pytest.raises(ServoTransportError) as error:
        transport.configure_force_mode(FORCES)
    report = error.value.handover_report
    assert report['axes'][2]['final_mode'] is None
    assert not report['recovery_complete']
    assert bus.modes[3] == 3
    assert_no_pause(bus, pauses)


def test_live_owner_capture_separates_zero_from_holding_target(tmp_path):
    bus = HandoverBus()
    core = ServoCore(config(tmp_path), bus=bus)
    try:
        # Exercise the existing Child entry path; reads 0x2C, not a historical constant.
        snapshot = core.prepare_force_handover()
        assert snapshot['source'] == 'MOCK'
        assert snapshot['baseline_g'] == FORCES
        assert snapshot['holding_forces_g'] == FORCES
        assert snapshot['relative_forces_g'] == [0] * 6
        assert all(frame.command == 0x31 for frame, _ in bus.calls)
        assert core.baseline_manager.sensor_zero_g == (0,) * 6
        with pytest.raises(SafetyViolation, match='all zero'):
            core.enter_force_mode_with_handover([0] * 6)
        core.enter_force_mode_with_handover(snapshot['holding_forces_g'])
        assert list(bus.target_forces.values()) == FORCES
        assert core.handover_state == ForceHandoverState.MODE3_FORCE_ACTIVE
        assert core.baseline_manager.relative_forces_g([f + 2 for f in FORCES]) == (2,) * 6
        assert core.baseline_manager.command_from_delta_g([0] * 6) == tuple(FORCES)
        assert core.baseline_manager.command_from_delta_g([2] * 6) == tuple(f + 2 for f in FORCES)
    finally:
        core.rec._dumped = True


def test_initialize_without_initial_target_uses_current_reading(tmp_path):
    bus = HandoverBus()
    # Fake status scans return zero; the actual 0x2C reads must supply this baseline.
    core = ServoCore(config(tmp_path), bus=bus)
    try:
        core.initialize()
        assert core.last_command_force_g == FORCES
        assert core.baseline_manager.software_baseline_g == tuple(FORCES)
        packet = core.cycle(time.monotonic_ns())
        assert packet['type'] == 'feedback'
        assert core.last_command_force_g == FORCES  # no target stream: preserve support
        assert_no_pause(bus)
    finally:
        core.rec._dumped = True


def test_core_nested_exception_and_close_never_auto_pause(tmp_path):
    bus = HandoverBus(fail_axis=3)
    core = ServoCore(config(tmp_path), bus=bus)
    with pytest.raises(ServoTransportError):
        core.initialize()
    assert core.fault_latched
    assert core.handover_state == ForceHandoverState.FAULT_LATCHED
    assert all(mode == 0 for mode in bus.modes.values())
    with pytest.raises(RuntimeError, match='fault is latched'):
        core.cycle(time.monotonic_ns())
    core.stop('outer child exception')
    core.close('finally cleanup')
    assert_no_pause(bus)
    assert core.pauses == []


def test_explicit_pause_remains_available_after_handover_fault(tmp_path):
    bus = HandoverBus(fail_axis=3)
    core = ServoCore(config(tmp_path), bus=bus)
    try:
        with pytest.raises(ServoTransportError):
            core.initialize()
        assert_no_pause(bus)
        core.pause_explicit('operator request')
        assert {frame.motor_id for frame, _ in bus.calls
                if frame.command == 0x32 and frame.register_addr == 0x1A} == set(range(1, 7))
    finally:
        core.rec._dumped = True


def test_hold_outside_configured_limits_is_rejected_before_write(tmp_path):
    bus = HandoverBus()
    core = ServoCore(config(tmp_path, min_force_n=-14.709975, max_force_n=14.709975), bus=bus)
    try:
        snapshot = core.prepare_force_handover()
        with pytest.raises(SafetyViolation, match='configured force limits'):
            core.enter_force_mode_with_handover(snapshot['holding_forces_g'])
        assert all(frame.command == 0x31 for frame, _ in bus.calls)
    finally:
        core.rec._dumped = True


def test_missing_baseline_is_not_implicitly_a_valid_zero():
    baseline = ForceBaselineManager()
    with pytest.raises(SafetyViolation, match='not been captured'):
        baseline.relative_forces_g([0] * 6)


def test_core_recovery_failure_retains_mixed_mode_report_without_pause(tmp_path):
    bus = HandoverBus(fail_axis=3, fail_recovery_axis=1)
    core = ServoCore(config(tmp_path), bus=bus)
    with pytest.raises(ServoTransportError):
        core.initialize()
    assert core.fault_latched
    assert core.writer.last_force_handover['axes'][0]['final_mode'] == 3
    with pytest.raises(RuntimeError, match='mode recovery is incomplete'):
        core.reset_fault_latch()
    assert core.fault_latched
    assert core.handover_state == ForceHandoverState.FAULT_LATCHED
    with pytest.raises(RuntimeError, match='fault is latched'):
        core.cycle(time.monotonic_ns())
    core.close()
    assert_no_pause(bus)


def test_mode0_exit_failure_requests_one_pause_and_reports_unconfirmed(tmp_path):
    bus = HandoverBus(fail_recovery_axis=1)
    core = ServoCore(config(tmp_path), bus=bus)
    try:
        core.initialize()
        bus.failed = True  # activate anchor failure after all axes entered Mode 3
        with pytest.raises(ServoTransportError):
            core.exit_force_mode_safe()
        assert core.fault_latched
        core.stop('outer exception')
        core.close()
        assert sum(f.command==0x32 and f.register_addr==0x1A for f,_ in bus.calls)==6
        assert core.force_shutdown_report['independent_stop_required']
        assert sum(f.command==0x32 and f.register_addr==0x25 and f.motor_id==1 and len(f.data)==10 for f,_ in bus.calls)==1
    finally:
        core.rec._dumped = True


def test_prepared_baseline_is_refreshed_from_registers_not_history(tmp_path):
    bus = HandoverBus()
    bus.actual_forces = {mid: value + 123 for mid, value in enumerate(FORCES, 1)}
    core = ServoCore(config(tmp_path), bus=bus)
    try:
        snapshot = core.enter_force_mode_from_current_state()
        assert snapshot['baseline_g'] == [value + 123 for value in FORCES]
        assert list(bus.target_forces.values()) == [value + 123 for value in FORCES]
        assert snapshot['relative_forces_g'] == [0] * 6
    finally:
        core.rec._dumped = True


def test_mode3_and_nonzero_hold_share_one_frame_and_readback():
    bus=HandoverBus();transport,pauses=writer(bus)
    transport.configure_force_mode(FORCES)
    writes=[f for f,_ in bus.calls if f.command==0x32]
    assert len(writes)==6
    for f,hold in zip(writes,FORCES):
        assert f.register_addr==0x25
        assert f.data==b'\x03\x00\x00\x00'+(hold&0xffff).to_bytes(2,'little')
    assert [r['target_force_g_readback'] for r in transport.last_force_handover['axes']]==FORCES
    assert_no_pause(bus,pauses)


def test_target_mismatch_stops_entry_and_recovers_without_pause():
    class WrongTargetBus(HandoverBus):
        def request(self,frame,**kwargs):
            result=super().request(frame,**kwargs)
            parsed=parse_frame(frame)
            if parsed.motor_id==3 and parsed.command==0x32 and parsed.data[:2]==b'\x03\x00':
                self.target_forces[3]=0
            return result
    bus=WrongTargetBus();transport,pauses=writer(bus)
    with pytest.raises(ServoTransportError,match='target readback mismatch') as exc:
        transport.configure_force_mode(FORCES)
    assert exc.value.handover_report['failed_axis']==3
    assert exc.value.handover_report['recovery_complete']
    assert list(bus.modes.values())==[0]*6
    assert not any(f.command==0x32 and f.motor_id>=4 for f,_ in bus.calls)
    assert_no_pause(bus,pauses)


def test_current_hold_after_prior_hardware_tare_still_uses_live_force(tmp_path):
    bus=HandoverBus();bus.actual_forces={i: -10*i for i in range(1,7)}
    core=ServoCore(config(tmp_path),bus=bus)
    try:
        core.force_feedback_reference='hardware_tare_assembly_reference'
        state=core.enter_force_mode_from_current_state()
        assert state['holding_forces_g']==[-10*i for i in range(1,7)]
        assert state['relative_forces_g']==[0]*6
        assert list(bus.target_forces.values())==state['holding_forces_g']
        assert_no_pause(bus)
    finally:core.rec._dumped=True
