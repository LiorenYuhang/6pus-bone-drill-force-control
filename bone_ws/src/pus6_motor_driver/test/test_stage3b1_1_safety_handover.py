"""Stage 3B-1.1: Force Control Mode Safety Handover Mechanism Offline Verification Suite.

Validates the 9 core safety scenarios using FakeBus/Mock without real hardware:
1. Three-way hardware write authorization gating (default blocked, requires explicit enable + qualification).
2. dry-run complete isolation (under dry_run, zero bytes/frames sent to physical bus).
3. Six-axis Mode 3 entry state machine full flow (pre-check -> initial force injection -> sequential switch -> readback confirm -> active).
4. Missing or zero initial force protection when non-zero static baseline is registered.
5. Partial switch failure handling (abort, position rollback without Pause, fault latching).
6. Target force direction, signed 16-bit encoding, and sensor consistency.
7. Safe exit to Mode 0 with position pre-anchoring (read 0x2A -> write 0x29 -> write 0x25=0 -> verify).
8. Target force command timeout fallback to static support load (preventing mechanical drop).
9. Fault latching blocks subsequent commands until explicit reset.
"""

import time
import pytest

from pus6_motor_driver.fake_bus import MockServoBus, FakeBus
from pus6_motor_driver.j14_process_uart import (
    CommandState,
    ServoCore,
    dispatch_force_slew,
    PERIOD_NS,
)
from pus6_motor_driver.j7_servo_transport import ServoTransportError
from pus6_motor_driver.protocol import parse_frame
from pus6_motor_driver.safety import (
    MOTOR_IDS,
    CONTROL_MODE_ADDR,
    TARGET_FORCE_ADDR,
    TARGET_POSITION_ADDR,
    ACTUAL_POSITION_ADDR,
    PAUSE_ADDR,
    MODE_FORCE,
    MODE_POSITION,
    SafetyViolation,
    TargetValidationError,
    ForceHandoverState,
    ForceBaselineManager,
    STAGE3B1_STATIC_BASELINE_G,
    validate_initial_handover_forces,
    encode_signed_i16_to_u16,
    decode_u16_to_signed_i16,
    n_to_g,
    g_to_n,
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
        initial_force_g=[0] * 6,  # explicit unloaded Mock input, not a live default
        direction=(1.0,) * 6,
        min_force_n=-98.0665,
        max_force_n=98.0665,
        max_rate_g_s=50000.0,
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


# ============================================================================
# 1. Three-way Hardware Write Authorization Gating Test
# ============================================================================
def test_three_way_authorization_gating(tmp_path):
    """Verify that hardware writes are blocked unless all conditions are met."""
    bus = MockServoBus(initial_positions=[1000] * 6)

    # Case A: Default (hardware_write_enabled=False, is_hardware_qualified=False)
    cfg_default = make_config(
        tmp_path / "def",
        hardware_mode='active',
        hardware_write_enabled=False,
        is_hardware_qualified=False,
    )
    core = ServoCore(cfg_default, bus)
    try:
        assert core.can_write_hardware() is False
        with pytest.raises(RuntimeError, match="hardware_write_enabled=False"):
            core.check_hardware_write_permission()
    finally:
        core.close()

    # Case B: hardware_write_enabled=True, but is_hardware_qualified=False
    cfg_unqualified = make_config(
        tmp_path / "unq",
        hardware_mode='active',
        hardware_write_enabled=True,
        is_hardware_qualified=False,
    )
    core = ServoCore(cfg_unqualified, bus)
    try:
        assert core.can_write_hardware() is False
        with pytest.raises(RuntimeError, match="is_hardware_qualified=False"):
            core.check_hardware_write_permission()
    finally:
        core.close()

    # Case C: hardware_write_enabled=False, is_hardware_qualified=True
    cfg_unauthorized = make_config(
        tmp_path / "una",
        hardware_mode='active',
        hardware_write_enabled=False,
        is_hardware_qualified=True,
    )
    core = ServoCore(cfg_unauthorized, bus)
    try:
        assert core.can_write_hardware() is False
        with pytest.raises(RuntimeError, match="hardware_write_enabled=False"):
            core.check_hardware_write_permission()
    finally:
        core.close()

    # Case D: Both True, active mode -> Authorized
    cfg_authorized = make_config(
        tmp_path / "auth",
        hardware_mode='active',
        hardware_write_enabled=True,
        is_hardware_qualified=True,
    )
    core = ServoCore(cfg_authorized, bus)
    try:
        assert core.can_write_hardware() is True
        core.check_hardware_write_permission()  # does not raise
    finally:
        core.close()


# ============================================================================
# 2. dry-run Complete Isolation Test
# ============================================================================
def test_dry_run_complete_isolation(tmp_path):
    """Under dry_run=True, confirm that ZERO write frames are sent to the bus."""
    bus = MockServoBus(initial_positions=[1000] * 6)
    cfg = make_config(
        tmp_path,
        hardware_mode='active',
        dry_run=True,
        hardware_write_enabled=False,
        is_hardware_qualified=False,
        control_mode='force',
    )
    core = ServoCore(cfg, bus)
    try:
        core.initialize()
        bus.written_frames.clear()

        # Command several force cycles
        now = time.monotonic_ns()
        for i in range(5):
            t = now + i * PERIOD_NS
            core.command.ingest([force_cmd(cfg['epoch'], i, [10.0] * 6, t)], t, [0] * 6)
            pkt = core.cycle(t)
            assert pkt['type'] == 'feedback'
            assert pkt['dry_run'] is True
            assert pkt['control_mode'] == 3

        # ABSOLUTE ISOLATION CHECK: Not a single byte written to the bus!
        assert len(bus.written_frames) == 0
    finally:
        core.close()


# ============================================================================
# 3. Six-Axis Mode 3 Safe Entry State Machine Full Flow Test
# ============================================================================
def test_mode3_entry_state_machine_full_flow(tmp_path):
    """Test full Mode 3 transition: pre-check -> initial force -> sequential switch -> 0x25 readback -> active."""
    bus = MockServoBus(initial_positions=[1000] * 6)
    cfg = make_config(tmp_path, control_mode='position')  # start in Mode 0
    core = ServoCore(cfg, bus)
    try:
        core.initialize()
        assert core.handover_state == ForceHandoverState.MODE0_POSITION
        assert core.mode_3_active is False
        bus.written_frames.clear()

        # Use the Stage 3B-1 measured static baseline
        initial_forces = list(STAGE3B1_STATIC_BASELINE_G)
        core.enter_force_mode_with_handover(
            initial_forces,
            static_baseline_g=STAGE3B1_STATIC_BASELINE_G,
            handover_tolerance_g=500.0,
        )

        assert core.handover_state == ForceHandoverState.MODE3_FORCE_ACTIVE
        assert core.mode_3_active is True
        assert core.last_command_force_g == initial_forces

        # Verify that all 6 motors received Mode 3 and their corresponding initial forces
        for mid, exp_force in zip(MOTOR_IDS, initial_forces):
            assert bus.modes[mid] == MODE_FORCE
            assert bus.target_forces[mid] == exp_force
    finally:
        core.close()


# ============================================================================
# 4. Zero Initial Force Rejection Under Non-Zero Static Load Test
# ============================================================================
def test_zero_initial_force_rejected_under_load(tmp_path):
    """Verify that entering Mode 3 with [0]*6 is rejected when non-zero static load exists."""
    bus = MockServoBus(initial_positions=[1000] * 6)
    cfg = make_config(tmp_path, control_mode='position')
    core = ServoCore(cfg, bus)
    try:
        core.initialize()

        # Attempt A: All zeros when static baseline has ~68N load
        with pytest.raises(SafetyViolation, match="Zero-force handover would collapse"):
            core.enter_force_mode_with_handover(
                [0, 0, 0, 0, 0, 0],
                static_baseline_g=STAGE3B1_STATIC_BASELINE_G,
                handover_tolerance_g=500.0,
            )
        assert core.mode_3_active is False
        assert core.handover_state != ForceHandoverState.MODE3_FORCE_ACTIVE

        # Attempt B: Exceeds handover tolerance (e.g. ID1 baseline=3754, commanded=1000, dev=2754 > 500)
        deviated_forces = [1000, -687, -6176, 793, -671, -6994]
        with pytest.raises(SafetyViolation, match="deviates from static baseline"):
            core.enter_force_mode_with_handover(
                deviated_forces,
                static_baseline_g=STAGE3B1_STATIC_BASELINE_G,
                handover_tolerance_g=500.0,
            )
        assert core.mode_3_active is False
    finally:
        core.close()


# ============================================================================
# 5. Partial Switch Failure Handling Test
# ============================================================================
def test_partial_switch_failure_aborts_and_latches(tmp_path):
    """Simulate failure on ID4: abort, restore switched axes, no Pause, latch fault."""
    bus = MockServoBus(initial_positions=[1000] * 6)
    # Reject ID4 write before execution; ID1..3 must be position-anchored back.

    cfg = make_config(tmp_path, control_mode='position')
    core = ServoCore(cfg, bus)
    try:
        core.initialize()
        bus.written_frames.clear()
        bus.fail_on_id = 4

        with pytest.raises(ServoTransportError, match="Force mode configuration failed"):
            core.enter_force_mode_with_handover(
                list(STAGE3B1_STATIC_BASELINE_G),
                static_baseline_g=STAGE3B1_STATIC_BASELINE_G,
                handover_tolerance_g=1000.0,
            )

        # Invariants on failure:
        # 1. State must transition to FAULT_LATCHED
        assert core.handover_state == ForceHandoverState.FAULT_LATCHED
        assert core.fault_latched is True
        assert core.mode_3_active is False

        # 2. Motor 5 and Motor 6 must NOT have been switched to Mode 3!
        assert bus.modes[5] != MODE_FORCE
        assert bus.modes[6] != MODE_FORCE

        # 3. Switched axes recovered; no automatic Pause at any layer.
        pause_frames = [
            parse_frame(f) for f in bus.written_frames
            if parse_frame(f).register_addr == PAUSE_ADDR
        ]
        pause_axes = {f.motor_id for f in pause_frames}
        assert pause_axes == set()
        assert all(bus.modes[mid] == MODE_POSITION for mid in (1, 2, 3))
    finally:
        core.close()


# ============================================================================
# 6. Target Force Direction, Signs, and Sensor Consistency Test
# ============================================================================
def test_target_force_signed_conversion_and_sensor_consistency():
    """Verify signed int16 encoding/decoding and direction consistency matching force sensor."""
    # Stage 3B-1 baseline values
    for mid, g_val in zip(MOTOR_IDS, STAGE3B1_STATIC_BASELINE_G):
        # 1. Unit conversion
        n_val = g_to_n(g_val, direction=1.0)
        recovered_g = round(n_to_g(n_val, direction=1.0))
        assert recovered_g == g_val

        # 2. Signed int16 encoding
        u16_encoded = encode_signed_i16_to_u16(g_val)
        decoded = decode_u16_to_signed_i16(u16_encoded)
        assert decoded == g_val

    # Direction inversion test
    n_pos = 10.0
    g_dir_pos = n_to_g(n_pos, direction=1.0)
    g_dir_neg = n_to_g(n_pos, direction=-1.0)
    assert g_dir_neg == -g_dir_pos


# ============================================================================
# 7. Safe Exit to Mode 0 with Position Anchoring Test
# ============================================================================
def test_safe_exit_position_anchoring_mode0(tmp_path):
    """Test the real-verified same-frame Mode 0 and current-position anchor."""
    bus = MockServoBus(initial_positions=[1000] * 6)
    cfg = make_config(tmp_path, control_mode='force')
    core = ServoCore(cfg, bus)
    try:
        core.initialize()
        assert core.mode_3_active is True

        # Simulate dynamic movement in Mode 3 (actual positions differ from 1000)
        dynamic_positions = [1050, 980, 1025, 990, 1010, 975]
        for mid, pos in zip(MOTOR_IDS, dynamic_positions):
            bus.positions[mid] = pos

        bus.written_frames.clear()

        # Execute safe exit
        anchored = core.exit_force_mode_safe()
        assert anchored == dynamic_positions
        assert core.handover_state == ForceHandoverState.MODE0_POSITION
        assert core.mode_3_active is False

        # Verify that all motors returned to Mode 0
        for mid in MOTOR_IDS:
            assert bus.modes[mid] == MODE_POSITION

        # Verify target position 0x29 was anchored to the actual positions
        for mid, exp_pos in zip(MOTOR_IDS, dynamic_positions):
            assert bus.targets[mid] == exp_pos

        # One continuous Mode 0/position write per axis, followed by readbacks.
        frames = [parse_frame(f) for f in bus.written_frames]
        pos_writes = [f for f in frames if f.register_addr == TARGET_POSITION_ADDR and f.command == 0x32]
        mode_writes = [f for f in frames if f.register_addr == CONTROL_MODE_ADDR and f.command == 0x32]
        mode_reads = [f for f in frames if f.register_addr == CONTROL_MODE_ADDR and f.command == 0x31]

        assert len(pos_writes) == 0
        assert len(mode_writes) == 6
        assert all(len(frame.data) == 10 for frame in mode_writes)
        assert len(mode_reads) == 6
    finally:
        core.close()


# ============================================================================
# 8. Command Timeout Fallback to Static Support Load Test
# ============================================================================
def test_command_timeout_fallback_to_static_support(tmp_path):
    """Verify that command timeout holds static support force rather than 0 g."""
    static_support_n = [g_to_n(g) for g in STAGE3B1_STATIC_BASELINE_G]
    bus = MockServoBus(initial_positions=[1000] * 6)
    cfg = make_config(
        tmp_path,
        control_mode='force',
        initial_force_g=list(STAGE3B1_STATIC_BASELINE_G),
        target_timeout_sec=0.1,
        fallback_force_n=static_support_n,
    )
    core = ServoCore(cfg, bus)
    try:
        core.initialize()
        now = time.monotonic_ns()

        # Send an active interaction command at t0
        active_cmd_n = [fn + 5.0 for fn in static_support_n]
        core.command.ingest([force_cmd(cfg['epoch'], 0, active_cmd_n, now)], now, [0] * 6)
        core.cycle(now)

        # Advance time by 300ms (> 0.1s timeout) without sending new commands
        stale_time = now + 300_000_000
        core.cycle(stale_time)

        # In timeout, commanded force must fall back to the static support baseline (NOT 0 g!)
        for actual_g, exp_g in zip(core.last_command_force_g, STAGE3B1_STATIC_BASELINE_G):
            assert abs(actual_g - exp_g) <= 20
    finally:
        core.close()


# ============================================================================
# 9. Fault Latching Blocks Subsequent Commands Test
# ============================================================================
def test_fault_latching_blocks_subsequent_commands(tmp_path):
    """Verify that once a fault is latched, subsequent normal commands are rejected."""
    bus = MockServoBus(initial_positions=[1000] * 6)
    cfg = make_config(tmp_path, control_mode='force')
    core = ServoCore(cfg, bus)
    try:
        core.initialize()
        now = time.monotonic_ns()

        # Step 1: Force a fault trip
        core.stop("test_simulated_safety_trip")
        assert core.fault_latched is True
        assert core.handover_state == ForceHandoverState.FAULT_LATCHED

        # Step 2: Attempt to dispatch another cycle
        with pytest.raises(RuntimeError, match="fault is latched"):
            core.cycle(now + PERIOD_NS)

        # Step 3: Attempt to re-enter force mode while latched
        with pytest.raises(RuntimeError, match="fault is latched"):
            core.enter_force_mode_with_handover(list(STAGE3B1_STATIC_BASELINE_G))

        # Step 4: Operator explicit reset
        core.reset_fault_latch()
        assert core.fault_latched is False
        assert core.handover_state == ForceHandoverState.MODE0_POSITION
    finally:
        core.close()
