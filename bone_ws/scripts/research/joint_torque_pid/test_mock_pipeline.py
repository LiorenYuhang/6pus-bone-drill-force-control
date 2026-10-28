import struct
import pytest
from controller import Fault, JointTorquePID
from mock_pipeline import MockBackend, MockPipeline, Mode
from test_controller import cfg


def test_mock_end_to_end_and_mode_exclusion():
    b = MockBackend(); p = MockPipeline(JointTorquePID(cfg()), b)
    p.enter(Mode.TORQUE)
    with pytest.raises(RuntimeError): p.enter(Mode.POSITION)
    with pytest.raises(RuntimeError): p.position_cycle()
    p.torque_cycle([.1]*6, [0]*6, [1_000_000_000]*6, 1_000_000_000, [0]*6)
    assert not b.frames
    p.torque_cycle([.1]*6, [0]*6, [1_100_000_000]*6, 1_100_000_000, [0]*6)
    assert struct.unpack('!6d', b.frames[0]) == pytest.approx([.1]*6)
    assert b.hardware_write_count == 0


def test_feedback_timeout_stops_and_latches():
    b = MockBackend(); p = MockPipeline(JointTorquePID(cfg()), b); p.enter(Mode.TORQUE)
    with pytest.raises(Fault):
        p.torque_cycle([0]*6, [0]*6, [1_000_000_000]*6, 1_500_000_000, [0]*6)
    assert p.mode is Mode.FAULT and b.stops == 1 and not b.frames
    with pytest.raises(RuntimeError): p.enter(Mode.POSITION)


def test_communication_failure_and_failed_stop_are_reported_once():
    b = MockBackend(); p = MockPipeline(JointTorquePID(cfg()), b); p.enter(Mode.TORQUE)
    p.torque_cycle([0]*6, [0]*6, [1_000_000_000]*6, 1_000_000_000, [0]*6)
    b.fail_write = True; b.fail_stop = True
    with pytest.raises(OSError):
        p.torque_cycle([1]*6, [0]*6, [1_100_000_000]*6, 1_100_000_000, [0]*6)
    p.stop('again')
    assert p.mode is Mode.FAULT and b.stops == 1 and p.stop_error and b.hardware_write_count == 0


def test_position_mode_cannot_send_torque():
    b = MockBackend(); p = MockPipeline(JointTorquePID(cfg()), b); p.enter(Mode.POSITION)
    p.position_cycle()
    with pytest.raises(RuntimeError): p.torque_cycle([0]*6, [0]*6, [1]*6, 1, [0]*6)
    assert not b.frames


def test_missing_feedback_watchdog():
    b = MockBackend(); p = MockPipeline(JointTorquePID(cfg()), b); p.enter(Mode.TORQUE)
    with pytest.raises(Fault): p.check_timeout(2_000_000_000, 1_950_000_000, 1_000_000_000, .2)
    assert b.stops == 1 and p.mode is Mode.FAULT
