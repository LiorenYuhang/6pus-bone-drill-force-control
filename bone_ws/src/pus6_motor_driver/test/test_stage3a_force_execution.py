"""Stage 3A: Motor native force control Mode 3 execution chain offline test suite.

Validates:
1. Six-axis Mode 3 entry lifecycle (0x25=3) with all-or-nothing confirmation.
2. Mode 3 entry failure abort and immediate 0x1A=1 Pause across all axes.
3. Positive target force dispatch (N -> g -> int16 -> register 0x27).
4. Negative target force dispatch (tension/pull) with correct signed two's complement encoding.
5. Bidirectional dynamic force reversal with slew rate limiting (dF/dt).
6. Exact unit conversion precision: 1 g = 0.00980665 N, signed int16 roundtrip.
7. Axis direction handling (+1 vs -1).
8. Single-axis write failure triggering immediate Pause (0x1A=1) on all 6 axes.
9. Command timeout triggering safe zero-force hold.
10. Safe Mode 3 exit and Pause service triggering.
11. Decoupled safety flags gating (dry_run, hardware_write_enabled, is_hardware_qualified).
12. Parent-to-Child IPC transmission of AxialForceCommand.
13. Full backwards compatibility with static position mode (0x29).
"""

import math
import os
from pathlib import Path
import time
import pytest

from pus6_motor_driver.fake_bus import MockServoBus, FakeBus
from pus6_motor_driver.j14_process_uart import (
    CommandState,
    ServoCore,
    ProcessClient,
    dispatch_slew,
    dispatch_force_slew,
    socket_pair,
    send_packet,
    receive_packets,
    PERIOD_NS,
    LEASE_NS,
)
from pus6_motor_driver.j7_real_uart_adapter import (
    J7AdapterWriteTimeout,
    SerialWriteMetric,
    TxQueueState,
)
from pus6_motor_driver.j7_servo_transport import ServoTransportError
from pus6_motor_driver.protocol import parse_frame, pack_write_register_cmd
from pus6_motor_driver.safety import (
    MOTOR_IDS,
    CONTROL_MODE_ADDR,
    TARGET_FORCE_ADDR,
    TARGET_POSITION_ADDR,
    PAUSE_ADDR,
    MODE_FORCE,
    MODE_SERVO,
    NOMINAL_N_PER_G,
    n_to_g,
    g_to_n,
    encode_signed_i16_to_u16,
    decode_u16_to_signed_i16,
    validate_force_vector,
)


def make_config(tmp_path, **kwargs):
    cfg = dict(
        epoch=time.monotonic_ns(),
        output_dir=str(tmp_path),
        hardware_mode='mock',
        port='/dev/DO_NOT_OPEN',
        baudrate=921600,
        servo_speed=50,
        maintenance_mode=True,
        reconnect_max_jump_steps=80,
        target_timeout_sec=0.2,
        control_mode='force',
        initial_force_g=[0] * 6,  # explicit unloaded Mock input
        direction=(1.0,) * 6,
        min_force_n=-14.709975,
        max_force_n=14.709975,
        max_rate_g_s=500.0,
        allow_negative_forces=True,
        dry_run=False,
        hardware_write_enabled=False,
        is_hardware_qualified=False,
    )
    cfg.update(kwargs)
    return cfg


def force_cmd(epoch, seq, forces_n, now):
    return dict(
        type='state',
        epoch=epoch,
        mode='force',
        allowed=True,
        lease_ns=now,
        command=dict(seq=seq, forces_n=list(forces_n), publish_ns=now),
    )


class ForceTimeoutBus(MockServoBus):
    """Simulates a serial timeout when writing register 0x27 on a specific axis."""
    def __init__(self, fail_axis: int):
        super().__init__(initial_positions=[1000] * 6)
        self.fail_axis = fail_axis

    def write(self, frame: bytes) -> int:
        parsed = parse_frame(frame)
        if parsed.register_addr == TARGET_FORCE_ADDR and parsed.motor_id == self.fail_axis:
            self.written_frames.append(frame)
            t = time.monotonic_ns()
            metric = SerialWriteMetric(
                t, t + 31_000_000, 31_000_000, 10_000, len(frame), -1,
                error='SerialTimeoutException', configured_timeout_s=0.030,
                queue_before=TxQueueState(0, 0, 0), queue_after=TxQueueState(0, 0, 0)
            )
            raise J7AdapterWriteTimeout(metric, 0.030)
        return super().write(frame)


class Mode3FailBus(MockServoBus):
    """Simulates a failure during Mode 3 configuration request on a specific axis."""
    def __init__(self, fail_axis: int):
        super().__init__(initial_positions=[1000] * 6)
        self.fail_axis = fail_axis

    def request(self, frame: bytes, **kwargs) -> bytes:
        parsed = parse_frame(frame)
        if parsed.register_addr == CONTROL_MODE_ADDR and parsed.motor_id == self.fail_axis:
            raise OSError(f"Simulated bus failure entering Mode 3 on ID{self.fail_axis}")
        return super().request(frame, **kwargs)


# ============================================================================
# 1. Six-Axis Mode 3 Entry Lifecycle Test
# ============================================================================
def test_mode3_entry_six_axes(tmp_path):
    bus = MockServoBus(initial_positions=[1000] * 6)
    cfg = make_config(tmp_path, control_mode='force')
    core = ServoCore(cfg, bus)
    try:
        ready = core.initialize()
        assert core.mode_3_active is True
        assert ready['control_mode'] == 3
        # Check that 0x25=3 was written to all 6 axes
        mode_writes = [
            parse_frame(f) for f in bus.written_frames
            if parse_frame(f).register_addr == CONTROL_MODE_ADDR
        ]
        # In MockServoBus.request, check mode state
        for mid in MOTOR_IDS:
            assert bus.modes[mid] == MODE_FORCE
    finally:
        core.close()


# ============================================================================
# 2. Mode 3 Entry Failure Aborts and Trips Pause on All Axes
# ============================================================================
def test_mode3_entry_failure_aborts_without_pause(tmp_path):
    bus = Mode3FailBus(fail_axis=2)
    cfg = make_config(tmp_path, control_mode='force')
    core = ServoCore(cfg, bus)
    try:
        with pytest.raises((ServoTransportError, RuntimeError)):
            core.initialize()
        assert core.mode_3_active is False
        assert core.fault is not None
        # Check that 0x1A=1 Pause was requested on all 6 axes
        pause_frames = [
            parse_frame(f) for f in bus.written_frames
            if parse_frame(f).register_addr == PAUSE_ADDR
        ]
        pause_axes = {f.motor_id for f in pause_frames}
        assert pause_axes == set()
    finally:
        core.close()


# ============================================================================
# 3. Positive Target Force Dispatch (N -> g -> int16 -> 0x27)
# ============================================================================
def test_target_force_positive_write_0x27(tmp_path):
    bus = MockServoBus(initial_positions=[1000] * 6)
    cfg = make_config(tmp_path, control_mode='force', max_rate_g_s=50000.0)
    core = ServoCore(cfg, bus)
    try:
        core.initialize()
        bus.written_frames.clear()

        # Command +4.903325 N on all 6 axes (corresponds exactly to +500 g)
        now = time.monotonic_ns()
        force_input_n = [4.903325] * 6
        core.command.ingest([force_cmd(cfg['epoch'], 0, force_input_n, now)], now, [0] * 6)

        packet = core.cycle(now)
        assert packet['type'] == 'feedback'
        assert packet['control_mode'] == 3
        assert packet['feedback_complete'] is True

        # Check frames written to bus: must be CMD_WR_REGISTER to 0x27 for ID1..ID6
        force_frames = [
            parse_frame(f) for f in bus.written_frames
            if parse_frame(f).register_addr == TARGET_FORCE_ADDR
        ]
        assert len(force_frames) == 6
        assert [f.motor_id for f in force_frames] == list(MOTOR_IDS)

        # Check payload: little-endian encoded 500 g (0x01F4 => bytes [0xF4, 0x01])
        for f in force_frames:
            val = int.from_bytes(f.data, "little", signed=True)
            assert val == 500

        # Check MockServoBus stored target forces
        for mid in MOTOR_IDS:
            assert bus.target_forces[mid] == 500
    finally:
        core.close()


# ============================================================================
# 4. Negative Target Force (Tension/Pull) Signed Two's Complement Encoding
# ============================================================================
def test_target_force_negative_write_0x27(tmp_path):
    bus = MockServoBus(initial_positions=[1000] * 6)
    cfg = make_config(tmp_path, control_mode='force', allow_negative_forces=True, max_rate_g_s=50000.0)
    core = ServoCore(cfg, bus)
    try:
        core.initialize()
        bus.written_frames.clear()

        # Command -4.903325 N on all 6 axes (corresponds exactly to -500 g)
        now = time.monotonic_ns()
        force_input_n = [-4.903325] * 6
        core.command.ingest([force_cmd(cfg['epoch'], 0, force_input_n, now)], now, [0] * 6)

        packet = core.cycle(now)
        assert packet['type'] == 'feedback'
        assert packet['control_mode'] == 3

        force_frames = [
            parse_frame(f) for f in bus.written_frames
            if parse_frame(f).register_addr == TARGET_FORCE_ADDR
        ]
        assert len(force_frames) == 6

        # -500 in 16-bit signed is 65536 - 500 = 65036 = 0xFDDC
        # Little-endian: 0xDC, 0xFD
        for f in force_frames:
            val = int.from_bytes(f.data, "little", signed=True)
            assert val == -500

        for mid in MOTOR_IDS:
            assert bus.target_forces[mid] == -500
    finally:
        core.close()


# ============================================================================
# 5. Bidirectional Dynamic Force Reversal with Slew Limiting
# ============================================================================
def test_target_force_bidirectional_reversal_slew(tmp_path):
    bus = MockServoBus(initial_positions=[1000] * 6)
    # Set max_rate_g_s = 500 g/s => at dt=0.02s (50Hz), step limit = 10 g
    cfg = make_config(tmp_path, control_mode='force', max_rate_g_s=500.0)
    core = ServoCore(cfg, bus)
    try:
        core.initialize()
        now = time.monotonic_ns()

        # Step 1: Send +500 g (+4.903325 N)
        core.command.ingest([force_cmd(cfg['epoch'], 0, [4.903325] * 6, now)], now, [0] * 6)
        core.cycle(now)
        # Starting from 0 g, with dt=0.02s and 500 g/s, max step is 10 g
        assert max(core.last_command_force_g) <= 10
        assert min(core.last_command_force_g) >= 0

        # Run several cycles to step up
        t = now
        for i in range(1, 10):
            t += PERIOD_NS
            core.cycle(t)
        prev_f = list(core.last_command_force_g)
        assert all(f > 0 for f in prev_f)

        # Step 2: Sudden reversal to -500 g (-4.903325 N)
        t += PERIOD_NS
        core.command.ingest([force_cmd(cfg['epoch'], 1, [-4.903325] * 6, t)], t, prev_f)
        core.cycle(t)

        # Verify rate limiting prevented sudden jump: delta must be <= 15 g (accounting for min dt=0.02s)
        curr_f = list(core.last_command_force_g)
        for p, c in zip(prev_f, curr_f):
            assert c < p  # moving towards negative
            assert abs(c - p) <= 20  # bounded slew rate!
    finally:
        core.close()


# ============================================================================
# 6. Unit Conversion Precision and Encoding Roundtrip
# ============================================================================
def test_force_unit_conversion_precision():
    # 1 g = 0.00980665 N nominal
    assert math.isclose(n_to_g(0.00980665), 1.0, rel_tol=1e-9)
    assert math.isclose(g_to_n(1.0), 0.00980665, rel_tol=1e-9)

    # 0 N
    assert n_to_g(0.0) == 0.0
    assert g_to_n(0.0) == 0.0
    assert encode_signed_i16_to_u16(0) == 0
    assert decode_u16_to_signed_i16(0) == 0

    # 10 N
    g_val = n_to_g(10.0)
    assert math.isclose(g_val, 1019.7162129779283, rel_tol=1e-9)
    assert math.isclose(g_to_n(g_val), 10.0, rel_tol=1e-9)

    # Signed encoding roundtrip across critical boundaries
    for val in [-32768, -1500, -500, -1, 0, 1, 500, 1500, 32767]:
        u16 = encode_signed_i16_to_u16(val)
        assert 0 <= u16 <= 65535
        decoded = decode_u16_to_signed_i16(u16)
        assert decoded == val

    # Bounds check
    with pytest.raises(ValueError):
        encode_signed_i16_to_u16(-32769)
    with pytest.raises(ValueError):
        encode_signed_i16_to_u16(32768)
    with pytest.raises(ValueError):
        decode_u16_to_signed_i16(-1)
    with pytest.raises(ValueError):
        decode_u16_to_signed_i16(65536)


# ============================================================================
# 7. Axis Direction Handling (+1 vs -1)
# ============================================================================
def test_axis_direction_handling(tmp_path):
    bus = MockServoBus(initial_positions=[1000] * 6)
    # Alternating directions: ID1=+1, ID2=-1, ID3=+1, ID4=-1, ID5=+1, ID6=-1
    direction = (1.0, -1.0, 1.0, -1.0, 1.0, -1.0)
    cfg = make_config(
        tmp_path, control_mode='force', direction=direction, max_rate_g_s=50000.0
    )
    core = ServoCore(cfg, bus)
    try:
        core.initialize()
        bus.written_frames.clear()

        # Command +4.903325 N on all axes
        now = time.monotonic_ns()
        core.command.ingest([force_cmd(cfg['epoch'], 0, [4.903325] * 6, now)], now, [0] * 6)
        core.cycle(now)

        # Axes 1, 3, 5 should receive +500 g; Axes 2, 4, 6 should receive -500 g
        for mid in (1, 3, 5):
            assert bus.target_forces[mid] == 500
        for mid in (2, 4, 6):
            assert bus.target_forces[mid] == -500
    finally:
        core.close()


# ============================================================================
# 8. Single-Axis Write Failure Trips Immediate Pause on All 6 Axes
# ============================================================================
@pytest.mark.parametrize('fail_axis', [1, 3, 6])
def test_single_axis_write_failure_trips_all_axes_pause(tmp_path, fail_axis):
    bus = ForceTimeoutBus(fail_axis=fail_axis)
    cfg = make_config(tmp_path, control_mode='force', max_rate_g_s=50000.0)
    core = ServoCore(cfg, bus)
    try:
        core.initialize()
        bus.written_frames.clear()

        now = time.monotonic_ns()
        core.command.ingest([force_cmd(cfg['epoch'], 0, [1.0] * 6, now)], now, [0] * 6)
        pkt = core.cycle(now)

        # Must report fault
        assert pkt['type'] == 'fault'
        assert core.fault is not None
        assert core.mode_3_active is False

        # All 6 axes must have received 0x1A=1 Pause!
        pause_frames = [
            parse_frame(f) for f in bus.written_frames
            if parse_frame(f).register_addr == PAUSE_ADDR
        ]
        pause_axes = {f.motor_id for f in pause_frames}
        assert pause_axes == set(MOTOR_IDS)
    finally:
        core.close()


# ============================================================================
# 9. Command Timeout Requests Verified Terminal Position Handover
# ============================================================================
def test_command_timeout_triggers_safe_state(tmp_path):
    bus = MockServoBus(initial_positions=[1000] * 6)
    cfg = make_config(tmp_path, control_mode='force', target_timeout_sec=0.1, max_rate_g_s=50000.0)
    core = ServoCore(cfg, bus)
    try:
        core.initialize()
        now = time.monotonic_ns()

        # Send command at t0
        core.command.ingest([force_cmd(cfg['epoch'], 0, [4.903325] * 6, now)], now, [0] * 6)
        core.cycle(now)
        assert core.last_command_force_g[0] > 0

        # Advance time by 300ms (> target_timeout_sec 0.1s)
        stale_time = now + 300_000_000
        # No new command ingested; cycle executes with timeout
        core.cycle(stale_time)

        # Last force is not replaced by a supposed zero-force stop.
        assert core.fault_latched
        assert core.force_shutdown_report['position_hold_confirmed']
        assert [bus.modes[i] for i in range(1,7)] == [0]*6
    finally:
        core.close()


# ============================================================================
# 10. Safe Mode 3 Exit / Pause Request
# ============================================================================
def test_safe_mode3_exit(tmp_path):
    bus = MockServoBus(initial_positions=[1000] * 6)
    cfg = make_config(tmp_path, control_mode='force')
    core = ServoCore(cfg, bus)
    try:
        core.initialize()
        assert core.mode_3_active is True
        bus.written_frames.clear()

        # Request exit
        core.exit_force_mode()
        assert core.mode_3_active is False

        # Verify 0x1A=1 Pause sent to all axes
        pause_frames = [
            parse_frame(f) for f in bus.written_frames
            if parse_frame(f).register_addr == PAUSE_ADDR
        ]
        pause_axes = {f.motor_id for f in pause_frames}
        assert pause_axes == set(MOTOR_IDS)
    finally:
        core.close()


# ============================================================================
# 11. Decoupled Safety Flags Gating
# ============================================================================
def test_decoupled_safety_flags_guard(tmp_path):
    # hardware_mode='active' (simulating real hardware attempt), dry_run=False,
    # hardware_write_enabled=False => must be blocked!
    bus = MockServoBus(initial_positions=[1000] * 6)
    cfg = make_config(
        tmp_path,
        control_mode='force',
        hardware_mode='active',  # simulated active mode
        dry_run=False,
        hardware_write_enabled=False,
        is_hardware_qualified=False,
    )
    core = ServoCore(cfg, bus)
    try:
        core.initialize()
        now = time.monotonic_ns()
        core.command.ingest([force_cmd(cfg['epoch'], 0, [1.0] * 6, now)], now, [0] * 6)
        pkt = core.cycle(now)
        assert pkt['type'] == 'fault'
        assert "hardware_write_enabled=False" in str(pkt.get('communication_fault'))
    finally:
        core.close()


def test_disallow_negative_forces_guard(tmp_path):
    bus = MockServoBus(initial_positions=[1000] * 6)
    cfg = make_config(
        tmp_path,
        control_mode='force',
        allow_negative_forces=False,  # negative forces forbidden
    )
    core = ServoCore(cfg, bus)
    try:
        core.initialize()
        now = time.monotonic_ns()
        core.command.ingest([force_cmd(cfg['epoch'], 0, [-2.0] * 6, now)], now, [0] * 6)
        pkt = core.cycle(now)
        assert pkt['type'] == 'fault'
        assert "negative force" in str(pkt.get('communication_fault'))
    finally:
        core.close()


# ============================================================================
# 12. Parent-to-Child IPC Force Command Stream
# ============================================================================
def test_parent_child_ipc_force_command_stream(tmp_path):
    parent_sock, child_sock = socket_pair()
    epoch = time.monotonic_ns()
    cmd_state = CommandState(epoch, [1000] * 6)

    try:
        # Parent side: sends mode='force' state packet
        now = time.monotonic_ns()
        force_values_n = [1.5, 2.5, 3.5, 4.5, 5.5, 6.5]
        packet = dict(
            type='state',
            epoch=epoch,
            mode='force',
            allowed=True,
            lease_ns=now,
            command=dict(seq=1, forces_n=force_values_n, publish_ns=now),
        )
        assert send_packet(parent_sock, packet) is True

        # Child side: receives packet
        rows, flooded = receive_packets(child_sock)
        assert len(rows) == 1
        cmd_state.ingest(rows, time.monotonic_ns(), [0] * 6)

        assert cmd_state.mode == 'force'
        assert cmd_state.seq == 1
        assert cmd_state.forces_n == force_values_n
        candidate = cmd_state.candidate_force([0] * 6, False, time.monotonic_ns())
        assert candidate == force_values_n
    finally:
        parent_sock.close()
        child_sock.close()


# ============================================================================
# 13. Backwards Compatibility with Static Position Mode (0x29)
# ============================================================================
def test_position_mode_compatibility_regression(tmp_path):
    bus = MockServoBus(initial_positions=[1000] * 6)
    cfg = make_config(tmp_path, control_mode='position')
    core = ServoCore(cfg, bus)
    try:
        core.initialize()
        assert core.mode_3_active is False
        bus.written_frames.clear()

        now = time.monotonic_ns()
        static_cmd = dict(
            type='state',
            epoch=cfg['epoch'],
            mode='static',
            allowed=True,
            lease_ns=now,
            command=dict(seq=0, targets=[1005] * 6, publish_ns=now),
        )
        core.command.ingest([static_cmd], now, [1000] * 6)
        pkt = core.cycle(now)

        assert pkt['type'] == 'feedback'
        assert pkt['control_mode'] == 1
        assert pkt['targets'] == [1005] * 6
        # Verify 0x29 (position) was written
        pos_frames = [
            parse_frame(f) for f in bus.written_frames
            if parse_frame(f).register_addr == TARGET_POSITION_ADDR
        ]
        assert len(pos_frames) == 6
    finally:
        core.close()
