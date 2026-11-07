"""Stage 2.1: real recorded fields + official parser, IPC, Bridge and ROS node.

FC1 data are measured historical fields. Re-encoded frames below are reconstructed
replay frames, NOT captured wire bytes. Vendor example is a literal manual frame.
All buses are Fake/Mock; no serial device or real Pause service is used.
"""
import json
from pathlib import Path
import time
from unittest.mock import MagicMock

import numpy as np
import pytest
import rclpy
from rclpy.parameter import Parameter
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import serialize_message, deserialize_message
from std_msgs.msg import Float64
from std_srvs.srv import Trigger

from pus6_msgs.msg import TimedJointFeedback
from pus6_research_msgs.msg import AxialForceCommand
from pus6_force_control.force_control_node import TaskSpaceForceControlNode
from pus6_motor_driver.fake_bus import MockServoBus
from pus6_motor_driver.protocol import unpack_status_response, ProtocolError
from pus6_motor_driver.j7_servo_feedback_scheduler import J7ServoFeedbackScheduler
from pus6_motor_driver.j7_six_axis_servo_transport import J7SixAxisServoTransport
from pus6_motor_driver.j14_process_uart import ServoCore, socket_pair, send_packet, receive_packets
from pus6_motor_driver.j14_feedback_bridge import Pus6FeedbackBridgeNode, populate_force_feedback
from pus6_kinematics.state_synchronizer import FixedLagJointSynchronizer, SyncStatus


@pytest.fixture(scope="module", autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.try_shutdown()


@pytest.fixture
def node():
    now = [10_000_000_000]
    instance = TaskSpaceForceControlNode("test_chain", parameter_overrides=[
        Parameter("use_fixed_lag", value=False), Parameter("target_force_z_n", value=1.0),
    ], clock=lambda: now[0])
    instance.timer.cancel()
    for name in ("axial_cmd_pub", "wrench_pub", "force_fb_pub", "diag_pub", "dry_stop_pub"):
        setattr(instance, name, MagicMock())
    instance.pause_client = MagicMock()
    yield instance, now
    instance.destroy_node()


def message(now, seq=1, force=0):
    msg = TimedJointFeedback()
    msg.source = TimedJointFeedback.SOURCE_MOCK
    msg.force_valid = [True] * 6
    msg.force_adc_raw = [2000] * 6
    msg.uart_cycle_sequence = seq
    msg.snapshot_start_monotonic_ns = now - 20_000_000
    msg.snapshot_end_monotonic_ns = now - 1_000_000
    msg.per_axis_rx_done_monotonic_ns = [now - 10_000_000] * 6
    msg.positions_meters = [0.05] * 6
    msg.raw_steps = [1000] * 6
    msg.forces_g_raw = [force] * 6
    return msg


def recorded_rows():
    root = Path(__file__).resolve().parents[4]
    path = root / "results/fc1_z_motion/fc1_20261101-170030/telemetry.jsonl"
    with path.open() as stream:
        return [json.loads(next(stream)) for _ in range(6)]


def replay_frame(row, axis):
    """Reconstruct the vendor status layout from measured FC1 fields (not raw capture)."""
    i = axis - 1
    fields = ((row["target_steps"][i], 2, True), (row["actual_steps"][i], 2, True),
              (row["current_mA"][i], 2, False), (row["force_raw_g"][i], 2, True),
              (row["force_adc"][i], 2, False), (row["temperature_c"][i], 1, True),
              (row["error_code"][i], 1, False))
    data = bytes([0xAA, 0x55, 0x0F, axis, 0x32, 0x29, 0])
    for value, size, signed in fields:
        data += int(value).to_bytes(size, "little", signed=signed)
    return data + bytes([sum(data[2:]) & 0xFF])


def test_vendor_literal_frame_and_signed_domain():
    # LAF V2.0.4cn section 3.5.1, status reply: zero g / zero ADC / 32 C.
    literal = bytes.fromhex("aa 55 0f 01 30 00 00 00 00 00 00 00 00 00 00 00 00 20 00 5f")
    # The manual prints 0x5F, while its displayed bytes sum to 0x60.
    # Preserve rejection of the literal; do not relax the production parser.
    with pytest.raises(ProtocolError):
        unpack_status_response(literal)
    literal = literal[:-1] + b"\x60"  # explicitly reconstructed valid example
    status = unpack_status_response(literal)
    assert (status.force_g, status.force_adc, status.temperature) == (0, 0, 32)
    for value in (-32768, -1, 0, 32767):
        data = bytearray(literal)
        data[13:15] = value.to_bytes(2, "little", signed=True)
        data[-1] = sum(data[2:-1]) & 0xFF
        assert unpack_status_response(bytes(data)).force_g == value
    data[-1] ^= 1
    with pytest.raises(ProtocolError):
        unpack_status_response(bytes(data))


def test_measured_fields_through_scheduler_child_ipc_bridge_node(tmp_path):
    rows = recorded_rows()
    now = [rows[0]["monotonic_ns"]]
    core = ServoCore(dict(epoch=123, hardware_mode="mock", output_dir=str(tmp_path)), bus=MockServoBus())
    # Use actual scheduler implementation with historical timestamps; never initialize/write a bus.
    core.feedback = J7ServoFeedbackScheduler(core.writer, clock_ns=lambda: now[0])
    left, right = socket_pair()
    fb_left, fb_right = socket_pair()
    bridge = Pus6FeedbackBridgeNode(fb_left, dict(epoch=123, output_dir=str(tmp_path)))
    bridge.timed_feedback_pub = MagicMock()
    controller = TaskSpaceForceControlNode("replay_force_chain", clock=lambda: now[0])
    controller.timer.cancel()
    for name in ("axial_cmd_pub", "wrench_pub", "force_fb_pub", "diag_pub", "dry_stop_pub"):
        setattr(controller, name, MagicMock())
    try:
        for seq, row in enumerate(rows):
            now[0] = row["monotonic_ns"]
            statuses = {axis: unpack_status_response(replay_frame(row, axis)) for axis in range(1, 7)}
            audit = core.feedback.ingest_cycle_feedback(
                statuses, servo_cycle_id=seq, servo_cycle_timestamp_ns=min(row["rx_ns"]),
                per_axis_rx_done_ns=dict(enumerate(row["rx_ns"], 1)))
            core.update(audit)
            payload = dict(type="feedback", epoch=123, cycle_seq=seq,
                           cycle_start_ns=min(row["rx_ns"]), cycle_end_ns=now[0],
                           positions=list(core.positions), currents=list(core.currents),
                           faults=list(core.faults), rx_ns=list(core.rx_ns),
                           targets=row["target_steps"], tx_start_ns=[0]*6, tx_end_ns=[0]*6,
                           **core.force_feedback_fields())
            assert send_packet(left, payload)
            packets, _ = receive_packets(right)
            bridge._process_feedback_packet(packets[0], now[0], 0, 0)
            published = bridge.timed_feedback_pub.publish.call_args.args[0]
            # Exercise generated ROS serialization, not a hand-written message substitute.
            published = deserialize_message(serialize_message(published), TimedJointFeedback)
            assert list(published.forces_g_raw) == row["force_raw_g"]
            assert list(published.force_adc_raw) == row["force_adc"]
            assert list(published.per_axis_rx_done_monotonic_ns) == row["rx_ns"]
            assert list(published.fault_codes) == row["error_code"]
            assert all(published.force_valid)
            controller._handle_timed_feedback(published)
            controller._control_cycle()
        assert not controller.fault_latched
        assert controller.last_processed_cycle_seq == len(rows) - 1
        assert controller.axial_cmd_pub.publish.call_count > 0
        diagnostics = json.loads(controller.diag_pub.publish.call_args.args[0].data)
        details = diagnostics["details"]
        assert details["measurement_monotonic_ns"] == now[0] - 30_000_000
        # Independent interpolation of the measured data at the same host reference time.
        query = details["measurement_monotonic_ns"]
        for i in range(6):
            expected_f = np.interp(query, [r["rx_ns"][i] for r in rows], [r["force_raw_g"][i] for r in rows])
            expected_q = np.interp(query, [r["rx_ns"][i] for r in rows], [r["actual_m"][i] for r in rows])
            assert details["aligned_forces_g_raw"][i] == pytest.approx(expected_f)
            assert details["aligned_positions_m"][i] == pytest.approx(expected_q)
        assert any(x < 0 for x in details["aligned_forces_g_raw"])
        assert any(x > 0 for x in details["aligned_forces_g_raw"])
        assert not core.bus.write_audit and not core.bus.written_frames
    finally:
        # No core.close(): it normally issues Pause; tests have never opened hardware.
        core.rec._dumped = True
        controller.destroy_node()
        bridge.destroy_node()
        for sock in (left, right, fb_left, fb_right):
            sock.close()


def test_uninitialized_zero_and_incomplete_payload_are_invalid(node):
    controller, now = node
    msg = message(now[0])
    # Old/absent Child fields cannot turn default zero arrays into measured zero.
    populate_force_feedback(msg, {})
    assert not any(msg.force_valid)
    controller._handle_timed_feedback(msg)
    assert controller.fault_latched
    controller._control_cycle()
    controller.axial_cmd_pub.publish.assert_not_called()
    controller.pause_client.call_async.assert_not_called()


def test_duplicate_does_not_refresh_watchdog_or_repeat_pid(node):
    controller, now = node
    now[0] += 20_000_000
    msg = message(now[0])
    controller._handle_timed_feedback(msg)
    controller._control_cycle()
    recv_ns = controller.last_feedback_recv_ns
    integral = controller.pipeline.pid.integral_n
    now[0] += 10_000_000
    controller._handle_timed_feedback(msg)
    controller._control_cycle()
    assert controller.last_feedback_recv_ns == recv_ns
    assert controller.pipeline.pid.integral_n == integral
    assert controller.axial_cmd_pub.publish.call_count == 1
    now[0] += 210_000_000
    controller._control_cycle()
    assert controller.fault_latched
    assert controller.stop_request_status == "DRY_RUN_ONLY"
    controller.pause_client.call_async.assert_not_called()


@pytest.mark.parametrize("kind", ["invalid", "old", "future", "adc", "fault", "source", "rollback", "gap", "time_rollback"])
def test_bad_feedback_latches_and_does_not_compute(node, kind):
    controller, now = node
    initial = message(now[0])
    controller._handle_timed_feedback(initial)
    now[0] += 20_000_000
    msg = message(now[0], seq=2)
    if kind == "invalid": msg.force_valid[4] = False
    if kind == "old": msg.per_axis_rx_done_monotonic_ns = [now[0] - 300_000_000]*6
    if kind == "future": msg.per_axis_rx_done_monotonic_ns = [now[0] + 1]*6
    if kind == "adc": msg.force_adc_raw[0] = 4096
    if kind == "fault": msg.fault_codes[2] = 2
    if kind == "source": msg.source = 0
    if kind == "rollback": msg.uart_cycle_sequence = 0
    if kind == "gap":
        now[0] += 250_000_000
        msg = message(now[0], seq=2)
    if kind == "time_rollback": msg.per_axis_rx_done_monotonic_ns = initial.per_axis_rx_done_monotonic_ns
    controller._handle_timed_feedback(msg)
    controller._control_cycle()
    assert controller.fault_latched
    controller.axial_cmd_pub.publish.assert_not_called()
    assert controller.pipeline.pid.integral_n == 0


def test_target_timeout_stops_and_explicit_reset_requires_new_inputs(node):
    controller, now = node
    controller.target_timeout_s = 0.025
    now[0] += 20_000_000
    controller._handle_timed_feedback(message(now[0]))
    controller._control_cycle()
    assert controller.axial_cmd_pub.publish.call_count == 1
    now[0] += 20_000_000
    controller._handle_timed_feedback(message(now[0], seq=2))
    controller._control_cycle()
    assert controller.fault_latched and "target force" in controller.fault_reason
    assert controller.axial_cmd_pub.publish.call_count == 1
    controller._reset_fault(None, Trigger.Response())
    now[0] += 20_000_000
    controller._handle_timed_feedback(message(now[0], seq=3))
    controller._control_cycle()
    assert controller.axial_cmd_pub.publish.call_count == 1
    controller._handle_target_force_z(Float64(data=-1.0))
    controller._control_cycle()
    assert not controller.fault_latched
    assert controller.axial_cmd_pub.publish.call_count == 2
    assert controller.pipeline.pid.reference.value < 0


def test_hardware_source_preserved_in_dry_run(node):
    controller, now = node
    now[0] += 20_000_000
    msg = message(now[0], force=-100)
    msg.source = TimedJointFeedback.SOURCE_HARDWARE  # e.g. offline replay of hardware data
    controller._handle_timed_feedback(msg)
    controller._control_cycle()
    calibrated = controller.force_fb_pub.publish.call_args.args[0]
    assert calibrated.source == 2
    assert list(calibrated.force_n) == pytest.approx([-0.980665]*6)
    assert controller.dry_run and not controller.drive.hardware_write_enabled


def test_pause_routing_calls_existing_interface_once_with_fake_client(node):
    controller, now = node
    # Test the authorized path only against a fake service client, not ROS/hardware.
    controller.dry_run = False
    controller.hardware_write_enabled = controller.is_hardware_qualified = True
    controller.pause_client.service_is_ready.return_value = True
    controller._trip_fault("TEST", "simulated transport interruption", now[0])
    controller._trip_fault("TEST", "duplicate fault", now[0])
    controller.pause_client.call_async.assert_called_once()
    assert controller.stop_request_status == "REQUESTED_NOT_CONFIRMED"
    assert controller.pipeline.pid.integral_n == 0


def test_force_sync_uses_same_weights_and_invalid_endpoints():
    sync = FixedLagJointSynchronizer()
    for stamp, force, position in ((100, -100, 0.05), (200, 100, 0.06)):
        sync.push_raw_snapshot([stamp+i for i in range(6)], [position]*6, [1000]*6,
                               forces_g_raw=[force]*6, force_valid=[True]*6,
                               force_adc_raw=[2000]*6, fault_codes=[0]*6, source=1)
    result = sync.query_sync_configuration(150)
    assert result.status == SyncStatus.SYNCED
    assert all(result.force_valid)
    for i, alpha in enumerate(result.per_axis_interp_weights):
        assert result.forces_g_raw[i] == pytest.approx(-100 + 200*alpha)
        assert result.positions_meters[i] == pytest.approx(0.05 + 0.01*alpha)
    sync.push_raw_snapshot([300+i for i in range(6)], [0.07]*6, [1000]*6,
                           forces_g_raw=[200]*6, force_valid=[True, False, True, True, True, True],
                           force_adc_raw=[2000]*6, fault_codes=[0]*6, source=1)
    assert not sync.query_sync_configuration(250).force_valid[1]


def test_sync_loss_and_fk_failure_suspend_output(node):
    controller, now = node
    controller.use_fixed_lag = True
    controller.fixed_lag_delay_ns = 20_000_000
    controller._handle_timed_feedback(message(now[0]))
    now[0] += 20_000_000
    controller._handle_timed_feedback(message(now[0], seq=2))
    controller._control_cycle()
    assert controller.axial_cmd_pub.publish.call_count == 1
    now[0] += 20_000_000
    controller.fixed_lag_delay_ns = 2_000_000_000  # query older than synchronizer buffer
    controller._handle_timed_feedback(message(now[0], seq=3))
    controller._control_cycle()
    assert controller.fault_latched and "alignment" in controller.fault_reason
    assert controller.axial_cmd_pub.publish.call_count == 1
    controller._reset_fault(None, Trigger.Response())
    controller.use_fixed_lag = False
    controller._handle_target_force_z(Float64(data=1.0))
    controller.kinematics.forward_kinematics_with_metrics = MagicMock(side_effect=ValueError("FK failed"))
    now[0] += 20_000_000
    controller._handle_timed_feedback(message(now[0], seq=4))
    controller._control_cycle()
    assert controller.fault_latched and "FK failed" in controller.fault_reason
    assert controller.axial_cmd_pub.publish.call_count == 1


def test_actual_ros_subscription_and_dry_run_topic_isolation():
    now = [10_000_000_000]
    controller = TaskSpaceForceControlNode("force_ros_delivery", parameter_overrides=[
        Parameter("use_fixed_lag", value=False), Parameter("target_force_z_n", value=-1.0),
    ], clock=lambda: now[0])
    controller.timer.cancel()
    probe = Node("force_test_probe")
    pub = probe.create_publisher(TimedJointFeedback, "/pus6/joint_feedback/timed", qos_profile_sensor_data)
    received_dry, received_live = [], []
    probe.create_subscription(AxialForceCommand, "/pus6/force_control/axial_forces_dry_run",
                              received_dry.append, qos_profile_sensor_data)
    probe.create_subscription(AxialForceCommand, "/pus6/command_axial_forces",
                              received_live.append, qos_profile_sensor_data)
    executor = SingleThreadedExecutor()
    executor.add_node(controller)
    executor.add_node(probe)
    try:
        assert controller.axial_cmd_pub.topic_name == "/pus6/force_control/axial_forces_dry_run"
        now[0] += 20_000_000
        msg = message(now[0], force=-100)
        deadline = time.monotonic() + 2.0
        while controller.last_cycle_seq < 1 and time.monotonic() < deadline:
            pub.publish(msg)
            executor.spin_once(timeout_sec=0.01)
        assert controller.last_cycle_seq == 1
        controller._control_cycle()
        while not received_dry and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.01)
        assert received_dry and not received_live
        assert controller.last_processed_cycle_seq == 1
        assert controller.pipeline.pid.previous_measured_n == pytest.approx(-5.88399)
    finally:
        executor.shutdown()
        probe.destroy_node()
        controller.destroy_node()


def test_missing_axis_invalidates_child_force_cache(tmp_path):
    now = [10_000_000_000]
    core = ServoCore(dict(epoch=1, hardware_mode="mock", output_dir=str(tmp_path)), bus=MockServoBus())
    core.feedback = J7ServoFeedbackScheduler(core.writer, clock_ns=lambda: now[0])
    row = recorded_rows()[0]
    statuses = {axis: unpack_status_response(replay_frame(row, axis)) for axis in range(1, 7)}
    try:
        full = core.feedback.ingest_cycle_feedback(statuses, servo_cycle_id=0, servo_cycle_timestamp_ns=now[0])
        core.update(full)
        assert all(core.force_valid)
        old_force, old_stamp = core.forces_g_raw[2], core.rx_ns[2]
        now[0] += 20_000_000
        del statuses[3]
        partial = core.feedback.ingest_cycle_feedback(statuses, servo_cycle_id=1, servo_cycle_timestamp_ns=now[0])
        core.update(partial)
        assert not core.force_valid[2]
        assert core.forces_g_raw[2] == old_force and core.rx_ns[2] == old_stamp
        assert not core.bus.written_frames
    finally:
        core.rec._dumped = True


def test_child_fault_packet_preserves_raw_fields_with_invalid_flags(tmp_path):
    bus = MockServoBus(initial_positions=[1000]*6)
    core = ServoCore(dict(epoch=1, hardware_mode="mock", output_dir=str(tmp_path)), bus=bus)
    sock, peer = socket_pair()
    bridge = Pus6FeedbackBridgeNode(sock, dict(epoch=1, output_dir=str(tmp_path)))
    bridge.timed_feedback_pub = MagicMock()
    try:
        core.initialize()  # mocked Mode 1 setup only; no actual serial or Mode 3
        core.bus.faults[3] = 2
        packet = core.cycle(time.monotonic_ns())
        assert packet['type'] == 'fault'
        assert packet['faults'][2] == 2
        assert len(packet['forces_g_raw']) == 6
        assert not any(packet['force_valid'])
        bridge._publish_fault_feedback(packet)
        msg = bridge.timed_feedback_pub.publish.call_args.args[0]
        assert msg.fault_codes[2] == 2
        assert list(msg.forces_g_raw) == packet['forces_g_raw']
        assert not any(msg.force_valid)
    finally:
        core.rec._dumped = True
        bridge.destroy_node()
        sock.close()
        peer.close()
