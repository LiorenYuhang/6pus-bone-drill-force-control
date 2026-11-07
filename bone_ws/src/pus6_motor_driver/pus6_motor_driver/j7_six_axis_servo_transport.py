"""Isolated six-axis Servo transport for the J7 experiment path.

The transport owns one injected bus and one lock.  Configuration is
request/response and verified per axis; motion cycles are ordered ID1..ID6
write-only ``0x29`` transmissions.  No serial device is opened here and the
module is intentionally independent of the ROS2 node and the frozen UART
modules.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from typing import Callable, Deque, Protocol
import math
import threading
import time

try:
    from .position_feedback import position_word_to_steps, recovery_exit_target_steps
    from .j7_real_uart_adapter import J7AdapterWriteTimeout, RxDrainAudit, RxDrainMetrics, SerialWriteMetric
    from .j7_servo_transport import (
        RxResidualRecord,
        RxSafetyError,
        ServoConfig,
        ServoRxPolicy,
        ServoTransportError,
    )
    from .protocol import (
        CMD_RD_REGISTER,
        CMD_WR_REGISTER,
        Frame,
        FrameStreamParser,
        ProtocolError,
        RSP_FRAME_HEADER,
        pack_read_register_cmd,
        pack_write_register_cmd,
        unpack_read_register_response,
        unpack_status_response,
        unpack_write_response,
    )
    from .safety import (
        ACTUAL_POSITION_ADDR,
        CONTROL_MODE_ADDR,
        MOTOR_IDS,
        TARGET_FORCE_ADDR,
        TARGET_POSITION_ADDR,
        TARGET_SPEED_ADDR,
        MODE_FORCE,
        MODE_POSITION,
        encode_signed_i16_to_u16,
        validate_target_vector,
    )
except ImportError:  # pragma: no cover - direct module execution fallback.
    from j7_real_uart_adapter import J7AdapterWriteTimeout, RxDrainAudit, RxDrainMetrics, SerialWriteMetric  # type: ignore
    from j7_servo_transport import (  # type: ignore
        RxResidualRecord,
        RxSafetyError,
        ServoConfig,
        ServoRxPolicy,
        ServoTransportError,
    )
    from protocol import (  # type: ignore
        CMD_RD_REGISTER,
        CMD_WR_REGISTER,
        Frame,
        FrameStreamParser,
        ProtocolError,
        RSP_FRAME_HEADER,
        StatusFeedback,
        pack_read_register_cmd,
        pack_write_register_cmd,
        unpack_read_register_response,
        unpack_status_response,
        unpack_write_response,
    )
    from safety import (  # type: ignore
        CONTROL_MODE_ADDR,
        MOTOR_IDS,
        TARGET_FORCE_ADDR,
        TARGET_POSITION_ADDR,
        TARGET_SPEED_ADDR,
        MODE_FORCE,
        encode_signed_i16_to_u16,
        validate_target_vector,
    )


class SixAxisServoBus(Protocol):
    """Injected bus surface used by configuration and the write-only stream."""

    def request(
        self,
        frame: bytes,
        *,
        expected_id: int | None = None,
        expected_command: int | None = None,
        expected_register: int | None = None,
        retries: int = 0,
    ) -> bytes: ...

    def write(self, frame: bytes) -> int | None: ...

    def read_available(self) -> bytes: ...


@dataclass(frozen=True)
class SixAxisTxRecord:
    motor_id: int
    cycle_id: int
    target_position: int
    target_timestamp_ns: int
    tx_start_ns: int
    tx_end_ns: int
    response_wait: bool
    retry_count: int
    residual_frames_before: int
    residual_frames_after: int
    duration_ns: int = 0
    cpu_duration_ns: int = 0
    requested_bytes: int = 0
    written_bytes: int = 0
    error: str | None = None
    serial_write_metric: SerialWriteMetric | None = None


@dataclass(frozen=True)
class SixAxisCycleAudit:
    cycle_id: int
    target_timestamp_ns: int
    cycle_start_ns: int
    cycle_end_ns: int
    cycle_jitter_ns: int
    target_age_ns: int
    tx_records: tuple[SixAxisTxRecord, ...]
    tx_skew_ns: int
    batch_complete_ns: int
    deadline_miss: bool
    feedbacks: dict[int, StatusFeedback] = field(default_factory=dict)
    feedback_rx_done_ns: dict[int, int] = field(default_factory=dict)
    prior_rx_drain_wall_ns: int = 0
    prior_rx_drain_cpu_ns: int = 0
    prior_rx_drained_frames: int = 0
    inter_frame_wait_total_ns: int = 0
    motion_drain_wall_ns: int = 0
    motion_drain_cpu_ns: int = 0
    rx_drain_metrics: Any = None
    error_phase: str | None = None
    error_motor_id: int | None = None
    error_class: str | None = None


@dataclass(frozen=True)
class SixAxisServoTransportAudit:
    cycles: tuple[SixAxisCycleAudit, ...]
    rx_records: tuple[RxResidualRecord, ...]
    response_wait_count: int
    motion_retry_count: int
    delivery_uncertain: bool
    stopped: bool
    stop_reason: str | None
    rx_motion_write_response_count: int = 0
    rx_stale_response_count: int = 0


@dataclass(frozen=True)
class _ResidualFrame:
    frame: Frame
    received_ns: int


class J7SixAxisServoTransport:
    """Single-writer six-axis Servo transport.

    ``cycle_rate_hz`` is fixed to 50 Hz for J7-C5.  The class does not
    implement feedback scheduling; a future scheduler must use this same bus
    and lock boundary when it is added.
    """

    def __init__(
        self,
        bus: SixAxisServoBus,
        *,
        cycle_rate_hz: float = 50.0,
        rx_policy: ServoRxPolicy = ServoRxPolicy.DEFER_TO_FEEDBACK,
        max_residual_frames: int = 192,
        max_residual_bytes: int = 24_576,
        min_frame_interval_s: float = 0.0008,
        rx_drain_budget_s: float = 0.0040,
        clock_ns: Callable[[], int] | None = None,
        sleep_fn: Callable[[float], None] | None = None,
        pause_callback: Callable[[str], None] | None = None,
        recoverable_write_timeouts: bool = False,
    ) -> None:
        if cycle_rate_hz != 50.0:
            raise ValueError("J7-C5 six-axis Servo transport requires a fixed 50 Hz cycle")
        if max_residual_frames < 1 or max_residual_bytes < 1:
            raise ValueError("RX residual limits must be positive")
        if min_frame_interval_s < 0.0:
            raise ValueError("min_frame_interval_s must be non-negative")

        self.bus = bus
        self.cycle_rate_hz = cycle_rate_hz
        self.period_ns = int(1e9 / cycle_rate_hz)
        self.rx_policy = ServoRxPolicy(rx_policy)
        self.max_residual_frames = max_residual_frames
        self.max_residual_bytes = max_residual_bytes
        self.min_frame_interval_ns = int(min_frame_interval_s * 1e9)
        self.rx_drain_budget_s = rx_drain_budget_s
        self._clock_ns = clock_ns or time.monotonic_ns
        self._sleep_fn = sleep_fn or time.sleep
        self._pause_callback = pause_callback
        self.recoverable_write_timeouts = recoverable_write_timeouts
        self.communication_transient = False
        self.recovery_feedback_only = False
        self.recovery_static_targets = None
        self._static_first_cycle_id = None
        self._static_last_cycle_id = None
        self._static_first_deadline_ns = None
        self._recovery_verification = False
        self._last_recovery_verification_audit = None

        self._lock = threading.RLock()
        self._parser = FrameStreamParser(expected_header=RSP_FRAME_HEADER)
        self._residual: Deque[_ResidualFrame] = deque()
        self._rx_records: list[RxResidualRecord] = []
        self._cycles: list[SixAxisCycleAudit] = []
        self._configured: tuple[ServoConfig, ...] | None = None
        self._last_tx_end_ns: int | None = None
        self._last_cycle_start_ns: int | None = None
        self._stopped = False
        self._stop_reason: str | None = None
        self._pause_sent = False
        self._delivery_uncertain = False
        self._malformed_count = 0
        self._response_wait_count = 0
        self._motion_retry_count = 0
        self._motion_write_response_count = 0
        self._stale_response_count = 0
        self._pending_rx_drain_audit: RxDrainAudit | None = None

    @property
    def configured(self) -> tuple[ServoConfig, ...] | None:
        return self._configured

    @property
    def uart_lock(self) -> threading.RLock:
        """Shared serialization boundary for the J7 feedback scheduler."""
        return self._lock

    @property
    def stopped(self) -> bool:
        return self._stopped

    @property
    def residual_frame_count(self) -> int:
        return len(self._residual)

    @property
    def residual_bytes(self) -> int:
        return sum(len(item.frame.raw) for item in self._residual)

    def configure_servo(self, *, speed: int) -> tuple[ServoConfig, ...]:
        """Enter Servo mode with acknowledged writes, without runtime readback.

        The mode/speed payload is sent as a normal request so every axis must
        acknowledge the command.  The returned readback fields stay ``None``:
        runtime Servo entry does not depend on ``0x31`` reads of ``0x25`` or
        ``0x28``.
        """
        if not isinstance(speed, int) or speed < 0 or speed > 0xFFFF:
            raise ValueError("speed must be an unsigned 16-bit integer")
        with self._lock:
            self._ensure_running_locked()
            try:
                self._ingest_rx_locked()
                self._drain_locked(ServoRxPolicy.DISCARD_BEFORE_TX)
                configs: list[ServoConfig] = []
                for motor_id in MOTOR_IDS:
                    response = self.bus.request(
                        pack_write_register_cmd(
                            motor_id, CONTROL_MODE_ADDR, [1, 0, 0, speed]
                        ),
                        expected_id=motor_id,
                        expected_command=CMD_WR_REGISTER,
                        retries=0,
                    )
                    status = unpack_write_response(response)
                    if status is None:
                        raise ServoTransportError(
                            f"ID{motor_id} Servo mode command response is invalid"
                        )
                    if status.error_code != 0:
                        raise ServoTransportError(
                            f"ID{motor_id} Servo mode command fault={status.error_code}"
                        )
                    configs.append(ServoConfig(motor_id, 1, speed, None, None))
            except Exception:
                self._delivery_uncertain = True
                self.stop("servo_startup_write_failure")
                raise
            self._configured = tuple(configs)
            self._last_cycle_start_ns = None
            return self._configured

    def configure_servo_for_stream(self, *, speed: int = 50) -> tuple[ServoConfig, ...]:
        """Compatibility alias for the acknowledged Servo configuration path."""
        return self.configure_servo(speed=speed)

    def read_force_handover_state(self) -> list[dict]:
        """Read current mode, signed 0x2C and position through the existing owner."""
        rows = []
        with self._lock:
            self._ensure_running_locked()
            for mid in MOTOR_IDS:
                mode = self._read_register_locked(mid, CONTROL_MODE_ADDR)
                force_u16 = self._read_register_locked(mid, 0x2C)
                force_rx_ns = self._clock_ns()
                position = self._read_register_locked(mid, ACTUAL_POSITION_ADDR)
                rows.append(dict(motor_id=mid, mode=mode, force_raw_u16=force_u16,
                                 force_g_raw=force_u16 - 65536 if force_u16 >= 32768 else force_u16,
                                 force_rx_ns=force_rx_ns, position_raw_u16=position,
                                 position_steps=position_word_to_steps(position),
                                 position_rx_ns=self._clock_ns()))
        return rows

    def configure_force_mode(
        self, initial_forces_g: Sequence[int] | int = 0, *, axis_verifier=None,
        recover_to_position_on_failure: bool = True, before_recovery=None,
        allow_feedback_tolerance_anchors: bool = False,
        axis_preparer=None, axis_status_verifier=None,
    ) -> tuple[ServoConfig, ...]:
        """Sequential Mode 3 entry; default failure recovery anchors WITHOUT Pause.

        The explicit low-level opt-out inventories/latches actual modes but is not
        used by ServoCore lifecycle exits. Retaining force does not stop motion.

        Every attempted axis is tracked. A failed write may already have executed:
        query that axis before deciding whether it also needs position anchoring.
        Failure always latches the transport, even when all axes recover to Mode 0.
        """
        forces = ([initial_forces_g] * len(MOTOR_IDS)
                  if isinstance(initial_forces_g, (int, float)) else list(initial_forces_g))
        if len(forces) != len(MOTOR_IDS):
            raise ValueError("six initial forces required")
        # Validate/encode ALL six before the first write, retaining signed force.
        if any(not isinstance(f, (int, float)) or isinstance(f, bool) or not math.isfinite(f) for f in forces):
            raise ValueError("six finite initial forces required")
        encoded = [encode_signed_i16_to_u16(int(round(f))) for f in forces]
        with self._lock:
            self._ensure_running_locked()
            report = dict(status='switching', failed_axis=None, recovery_complete=False,
                          axes=[dict(motor_id=mid, initial_mode=None, switch_attempted=False,
                                     confirmed_mode3=False, recovery_attempted=False,
                                     final_mode=None, anchor_position=None, error=None)
                                for mid in MOTOR_IDS])
            self.last_force_handover = report
            configs = []
            current = None
            try:
                self._ingest_rx_locked()
                self._drain_locked(ServoRxPolicy.DISCARD_BEFORE_TX)
                for row in report['axes']:
                    current = row
                    row['initial_mode'] = self._read_register_locked(row['motor_id'], CONTROL_MODE_ADDR)
                    row['final_mode'] = row['initial_mode']
                for idx, row in enumerate(report['axes']):
                    current = row
                    mid = row['motor_id']
                    if axis_preparer is not None:
                        fresh_force = axis_preparer(mid)
                        if (isinstance(fresh_force, bool) or not isinstance(fresh_force, (int, float))
                                or not math.isfinite(fresh_force)):
                            raise ValueError('finite signed holding force required')
                        encoded[idx] = encode_signed_i16_to_u16(int(round(fresh_force)))
                    row['hold_force_g'] = encoded[idx] - 65536 if encoded[idx] >= 32768 else encoded[idx]
                    row['switch_attempted'] = True
                    row['final_mode'] = None  # write/ACK loss can leave an uncertain mode
                    response = self.bus.request(
                        pack_write_register_cmd(mid, CONTROL_MODE_ADDR, [MODE_FORCE, 0, encoded[idx]]),
                        expected_id=mid, expected_command=CMD_WR_REGISTER, retries=0)
                    status = unpack_write_response(response)
                    if status is None or status.error_code != 0:
                        raise ServoTransportError(f"ID{mid} force mode command fault")
                    if axis_status_verifier is not None:
                        axis_status_verifier(mid, status)
                    mode = self._read_register_locked(mid, CONTROL_MODE_ADDR)
                    row['final_mode'] = mode
                    if mode != MODE_FORCE:
                        raise ServoTransportError(
                            f"ID{mid} Force mode readback verification failed: expected {MODE_FORCE}, got {mode}")
                    row['confirmed_mode3'] = True
                    target_word = self._read_register_locked(mid, TARGET_FORCE_ADDR)
                    row['target_force_g_readback'] = target_word - 65536 if target_word >= 32768 else target_word
                    row['hold_force_g'] = encoded[idx] - 65536 if encoded[idx] >= 32768 else encoded[idx]
                    if target_word != encoded[idx]:
                        raise ServoTransportError(f"ID{mid} Force target readback mismatch: "
                                                  f"expected {row['hold_force_g']}, got {row['target_force_g_readback']}")
                    if axis_verifier is not None:
                        axis_verifier(mid)
                    configs.append(ServoConfig(motor_id=mid, mode=MODE_FORCE, speed=0,
                                               mode_readback=mode, speed_readback=None))
            except Exception as exc:
                if current is not None:
                    report['failed_axis'] = current['motor_id']
                    current['error'] = str(exc)
                # Recovery is bounded to axes confirmed in Mode 3. Inspect an
                # attempted-but-unconfirmed axis because a timeout is not rejection.
                recovery_failed = False
                if recover_to_position_on_failure and before_recovery is not None:
                    before_recovery()  # open only the owner's terminal frame allowlist
                # Stop the axis whose measured motion triggered the guard first;
                # preserve final reports in motor-ID order. Never retry writes.
                priority = getattr(exc, 'recovery_priority_motor_id', None)
                report['recovery_priority_motor_id'] = priority
                recovery_rows = sorted(report['axes'], key=lambda row: row['motor_id'] != priority)
                for row in recovery_rows:
                    if not row['switch_attempted']:
                        continue
                    recover = row['confirmed_mode3']
                    if not recover:
                        try:
                            row['final_mode'] = self._read_register_locked(row['motor_id'], CONTROL_MODE_ADDR)
                            recover = row['final_mode'] == MODE_FORCE
                            row['confirmed_mode3'] = recover
                            if row['final_mode'] != MODE_POSITION and not recover:
                                recovery_failed = True
                        except Exception as read_error:
                            row['final_mode'] = None
                            row['error'] = str(read_error)
                            recovery_failed = True
                    if recover and recover_to_position_on_failure:
                        row['recovery_attempted'] = True
                        try:
                            row['anchor_position'] = self._anchor_axis_to_position_mode(row['motor_id'],
                                allow_feedback_tolerance=allow_feedback_tolerance_anchors)
                            row['final_mode'] = MODE_POSITION
                        except Exception as recovery_error:
                            row['final_mode'] = None
                            row['error'] = str(recovery_error)
                            recovery_failed = True
                # Read-only final inventory; never write to untouched axes.
                for row in report['axes']:
                    try:
                        row['final_mode'] = self._read_register_locked(row['motor_id'], CONTROL_MODE_ADDR)
                    except Exception as read_error:
                        row['final_mode'] = None
                        row['error'] = str(read_error)
                report['recovery_complete'] = (not recovery_failed and all(
                    row['final_mode'] == MODE_POSITION for row in report['axes']))
                report['status'] = ('recovered_mode0' if report['recovery_complete'] else 'recovery_incomplete') if recover_to_position_on_failure else 'latched_mode_unchanged'
                report['position_recovery_enabled'] = recover_to_position_on_failure
                self._configured = None
                self._delivery_uncertain = not report['recovery_complete']
                self.latch_fault_without_pause(f"force_mode_startup_failure: {exc}")
                error = ServoTransportError(f"Force mode configuration failed: {exc}; {report['status']}")
                error.handover_report = report
                raise error from exc
            report['status'] = 'mode3_active'
            self._configured = tuple(configs)
            self._last_cycle_start_ns = None
            return self._configured

    def _anchor_axis_to_position_mode(self, motor_id: int, position: int | None = None,
                                      *, allow_feedback_tolerance: bool = False) -> int:
        """Read current position; send Mode 0 and anchor in ONE validated frame."""
        if position is None:
            position = self._read_register_locked(motor_id, ACTUAL_POSITION_ADDR)
            position = position_word_to_steps(position)
        # Only the manual recovery owner explicitly permits an inward endpoint
        # target. The measured position is never rewritten or declared in-stroke.
        if allow_feedback_tolerance:
            position = recovery_exit_target_steps(position)
        # Validate every command before the first write. No HOME substitution.
        if not isinstance(position, int) or not 0 <= position <= 2000:
            raise ServoTransportError(f"ID{motor_id} invalid position anchor {position}")
        force_word = self._read_register_locked(motor_id, TARGET_FORCE_ADDR)
        response = self.bus.request(
            pack_write_register_cmd(motor_id, CONTROL_MODE_ADDR,
                                    [MODE_POSITION, 0, force_word, 0, position]),
            expected_id=motor_id, expected_command=CMD_WR_REGISTER, retries=0)
        status = unpack_write_response(response)
        if status is None or status.error_code != 0:
            raise ServoTransportError(f"ID{motor_id} simultaneous Mode 0/position handover failed")
        mode = self._read_register_locked(motor_id, CONTROL_MODE_ADDR)
        if mode != MODE_POSITION:
            raise ServoTransportError(f"ID{motor_id} Mode 0 readback verification failed: got {mode}")
        target = self._read_register_locked(motor_id, TARGET_POSITION_ADDR)
        if target != position:
            raise ServoTransportError(f"ID{motor_id} anchor target readback {target} != {position}")
        return position

    def transition_to_position_mode(
        self, current_positions: Sequence[int] | None = None, *, recovery: bool = False,
        priority_motor_ids: Sequence[int] = (),
    ) -> list[int]:
        """Same-frame Mode 0 anchor; optional fault-axis priority, never Pause.

        Priority changes only transaction order. Return values and supplied
        anchors always remain in ID1..ID6 order. The transition is not atomic.
        """
        if current_positions is not None and len(current_positions) != len(MOTOR_IDS):
            raise ValueError("six position anchors required")
        priority = tuple(priority_motor_ids)
        if (any(type(mid) is not int or mid not in MOTOR_IDS for mid in priority)
                or len(set(priority)) != len(priority)):
            raise ValueError("priority motor IDs must be unique IDs 1..6")
        order = priority + tuple(mid for mid in MOTOR_IDS if mid not in priority)
        with self._lock:
            if not recovery:
                self._ensure_running_locked()
            try:
                self._ingest_rx_locked()
                self._drain_locked(ServoRxPolicy.DISCARD_BEFORE_TX)
                anchored = [None] * len(MOTOR_IDS)
                for mid in order:
                    i = MOTOR_IDS.index(mid)
                    # A latched force fault may only recover observed Mode 3
                    # axes; it does not re-enable normal command delivery.
                    mode = self._read_register_locked(mid, CONTROL_MODE_ADDR) if recovery else MODE_FORCE
                    if mode == MODE_POSITION:
                        anchored[i] = self._read_register_locked(mid, ACTUAL_POSITION_ADDR)
                    elif mode == MODE_FORCE:
                        anchored[i] = self._anchor_axis_to_position_mode(mid,
                            int(round(current_positions[i])) if current_positions is not None else None)
                    else:
                        raise ServoTransportError(f"ID{mid} recovery found unsupported mode {mode}")
            except Exception as exc:
                self._delivery_uncertain = True
                self.latch_fault_without_pause(f"position_mode_transition_failure: {exc}")
                raise ServoTransportError(f"Position mode transition failed: {exc}") from exc
            self._configured = None
            self._last_cycle_start_ns = None
            return anchored

    def latch_fault_without_pause(self, reason: str) -> None:
        """Reject normal commands without issuing a physical stop command."""
        with self._lock:
            self._stopped = True
            self._stop_reason = str(reason)

    def pause_explicit(self, reason: str = 'explicit_pause') -> None:
        """Separate one-shot Pause, available even after a no-Pause fault latch."""
        with self._lock:
            self.latch_fault_without_pause(reason)
            if self._pause_callback is not None and not self._pause_sent:
                self._pause_sent = True
                self._pause_callback(str(reason))

    def _read_register_locked(self, motor_id: int, register_address: int) -> int:
        raw = self.bus.request(
            pack_read_register_cmd(motor_id, register_address),
            expected_id=motor_id,
            expected_command=CMD_RD_REGISTER,
            retries=0,
        )
        response = unpack_read_register_response(raw, register_address, 1)
        return response.values[0]

    def send_cycle(
        self,
        targets: list[int] | tuple[int, ...],
        *,
        cycle_id: int,
        target_timestamp_ns: int | None = None,
    ) -> SixAxisCycleAudit:
        """Send one ordered six-axis write-only ``0x29`` batch."""
        if not isinstance(cycle_id, int) or cycle_id < 0:
            raise ValueError("cycle_id must be a non-negative integer")
        validate_target_vector(targets)
        with self._lock:
            self._ensure_running_locked()
            if self.recovery_static_targets is not None:
                if tuple(targets) != self.recovery_static_targets:
                    raise ServoTransportError("static recovery forbids trajectory continuation")
                expected = self._static_first_cycle_id if self._static_last_cycle_id is None else self._static_last_cycle_id + 1
                if cycle_id != expected:
                    raise ServoTransportError("static recovery requires a new consecutive transaction epoch")
                if self._static_last_cycle_id is None and self._static_first_deadline_ns is not None and self._clock_ns() >= self._static_first_deadline_ns:
                    raise ServoTransportError("existing 1000ms freshness boundary expired before first resume write")
                self._static_last_cycle_id = cycle_id
            if self._configured is None:
                raise ServoTransportError("configure_servo() must succeed before motion TX")

            timestamp_ns = self._clock_ns() if target_timestamp_ns is None else target_timestamp_ns
            if not isinstance(timestamp_ns, int) or timestamp_ns < 0:
                raise ValueError("target_timestamp_ns must be a non-negative integer")

            t_drain_start_ns = self._clock_ns()
            t_drain_cpu_start = time.thread_time_ns()
            try:
                residual_before = self._prepare_motion_rx_locked()
            except RxSafetyError:
                self._delivery_uncertain = True
                self.stop("rx_safety_failure")
                raise
            t_drain_end_ns = self._clock_ns()
            t_drain_cpu_end = time.thread_time_ns()
            prior_rx_drain_wall_ns = t_drain_end_ns - t_drain_start_ns
            prior_rx_drain_cpu_ns = t_drain_cpu_end - t_drain_cpu_start

            cycle_start_ns = self._clock_ns()
            if self._last_cycle_start_ns is None:
                cycle_jitter_ns = 0
            else:
                cycle_jitter_ns = (
                    cycle_start_ns - self._last_cycle_start_ns - self.period_ns
                )

            records: list[SixAxisTxRecord] = []
            inter_frame_wait_total_ns = 0
            transport_err_phase: str | None = None
            transport_err_mid: int | None = None
            transport_err_cls: str | None = None

            try:
                for motor_id, target in zip(MOTOR_IDS, targets):
                    t_w_start = self._clock_ns()
                    self._wait_for_frame_interval_locked()
                    inter_frame_wait_total_ns += (self._clock_ns() - t_w_start)

                    frame = pack_write_register_cmd(
                        motor_id, TARGET_POSITION_ADDR, [target]
                    )
                    tx_start_ns = self._clock_ns()
                    tx_cpu_start = time.thread_time_ns()
                    written = self.bus.write(frame)
                    if written is not None and written != len(frame):
                        raise ServoTransportError(
                            f"short Servo write on ID{motor_id}: {written}/{len(frame)} bytes"
                        )
                    tx_end_ns = self._clock_ns()
                    tx_cpu_end = time.thread_time_ns()
                    self._last_tx_end_ns = tx_end_ns
                    records.append(
                        SixAxisTxRecord(
                            motor_id=motor_id,
                            cycle_id=cycle_id,
                            target_position=target,
                            target_timestamp_ns=timestamp_ns,
                            tx_start_ns=tx_start_ns,
                            tx_end_ns=tx_end_ns,
                            response_wait=False,
                            retry_count=0,
                            residual_frames_before=(
                                residual_before if not records else len(self._residual)
                            ),
                            residual_frames_after=len(self._residual),
                            duration_ns=tx_end_ns - tx_start_ns,
                            cpu_duration_ns=tx_cpu_end - tx_cpu_start,
                            requested_bytes=len(frame),
                            written_bytes=len(frame) if written is None else written,
                            error=None,
                            serial_write_metric=getattr(self.bus, "last_write_metric", None),
                        )
                    )
            except Exception as exc:
                self._delivery_uncertain = True
                transport_err_phase = f"write_id{motor_id}"
                transport_err_mid = motor_id
                transport_err_cls = type(exc).__name__
                if isinstance(exc, J7AdapterWriteTimeout):
                    # Preserve the failed write before STOP writes can overwrite adapter metrics.
                    tx_end_ns = self._clock_ns()
                    records.append(SixAxisTxRecord(
                        motor_id=motor_id, cycle_id=cycle_id, target_position=target,
                        target_timestamp_ns=timestamp_ns, tx_start_ns=tx_start_ns,
                        tx_end_ns=tx_end_ns, response_wait=False, retry_count=0,
                        residual_frames_before=residual_before if not records else len(self._residual),
                        residual_frames_after=len(self._residual),
                        duration_ns=tx_end_ns - tx_start_ns,
                        cpu_duration_ns=time.thread_time_ns() - tx_cpu_start,
                        requested_bytes=len(frame), written_bytes=-1,
                        error="SerialTimeoutException",
                        serial_write_metric=exc.write_metric,
                    ))
                    starts = [record.tx_start_ns for record in records]
                    failed_audit = SixAxisCycleAudit(
                        cycle_id=cycle_id, target_timestamp_ns=timestamp_ns,
                        cycle_start_ns=cycle_start_ns, cycle_end_ns=tx_end_ns,
                        cycle_jitter_ns=cycle_jitter_ns, target_age_ns=cycle_start_ns - timestamp_ns,
                        tx_records=tuple(records), tx_skew_ns=max(starts) - min(starts),
                        batch_complete_ns=tx_end_ns,
                        deadline_miss=(tx_end_ns - cycle_start_ns) > self.period_ns,
                        prior_rx_drain_wall_ns=prior_rx_drain_wall_ns,
                        prior_rx_drain_cpu_ns=prior_rx_drain_cpu_ns,
                        prior_rx_drained_frames=residual_before,
                        inter_frame_wait_total_ns=inter_frame_wait_total_ns,
                        error_phase=transport_err_phase, error_motor_id=motor_id,
                        error_class="SerialTimeoutException",
                    )
                    self._cycles.append(failed_audit)
                    exc.cycle_audit = failed_audit
                    if self.recoverable_write_timeouts and not self._recovery_verification:
                        self.communication_transient = True
                        raise
                self.stop("motion_write_failure")
                raise

            # Ingest and unmarshal 0x29 StatusFeedback responses directly from the bus
            feedbacks: dict[int, StatusFeedback] = {}
            feedback_rx_done_ns: dict[int, int] = {}
            drain_fn = getattr(self.bus, "drain_motion_feedbacks", None)
            t_motion_drain_start_ns = self._clock_ns()
            t_motion_drain_cpu_start = time.thread_time_ns()
            drain_iterations = 0
            cum_in_waiting_calls = 0
            cum_read_calls = 0
            cum_bytes_read = 0
            max_read_lat_ns = 0
            if drain_fn is not None:
                deadline_ns = self._clock_ns() + int(self.rx_drain_budget_s * 1e9)
                while True:
                    drain_iterations += 1
                    new_fbs = drain_fn()
                    # Host completion of this drain, shared by all frames in that read.
                    # No device sample timestamp is available in the vendor protocol.
                    received_ns = self._clock_ns()
                    for mid in new_fbs:
                        feedback_rx_done_ns[mid] = received_ns
                    rx_m = getattr(self.bus, "last_rx_drain_metrics", None)
                    if rx_m is not None:
                        cum_in_waiting_calls += getattr(rx_m, "in_waiting_calls", 0)
                        cum_read_calls += getattr(rx_m, "read_calls", 0)
                        cum_bytes_read += getattr(rx_m, "bytes_read", 0)
                        lat = getattr(rx_m, "max_read_latency_ns", 0)
                        if lat > max_read_lat_ns:
                            max_read_lat_ns = lat
                    feedbacks.update(new_fbs)
                    if len(feedbacks) >= len(MOTOR_IDS) or self._clock_ns() >= deadline_ns:
                        break
                    pass
            t_motion_drain_end_ns = self._clock_ns()
            t_motion_drain_cpu_end = time.thread_time_ns()
            motion_drain_wall_ns = t_motion_drain_end_ns - t_motion_drain_start_ns
            motion_drain_cpu_ns = t_motion_drain_cpu_end - t_motion_drain_cpu_start
            rx_metrics = None
            if drain_fn is not None:
                rx_metrics = RxDrainMetrics(
                    timestamp_start_ns=t_motion_drain_start_ns,
                    timestamp_end_ns=t_motion_drain_end_ns,
                    wall_duration_ns=motion_drain_wall_ns,
                    cpu_duration_ns=motion_drain_cpu_ns,
                    in_waiting_calls=cum_in_waiting_calls,
                    read_calls=cum_read_calls,
                    bytes_read=cum_bytes_read,
                    max_read_latency_ns=max_read_lat_ns,
                    feedbacks_extracted=len(feedbacks),
                    drain_iterations=drain_iterations,
                )

            batch_complete_ns = self._clock_ns()
            cycle_end_ns = batch_complete_ns
            tx_starts = [record.tx_start_ns for record in records]
            tx_skew_ns = max(tx_starts) - min(tx_starts) if tx_starts else 0
            audit = SixAxisCycleAudit(
                cycle_id=cycle_id,
                target_timestamp_ns=timestamp_ns,
                cycle_start_ns=cycle_start_ns,
                cycle_end_ns=cycle_end_ns,
                cycle_jitter_ns=cycle_jitter_ns,
                target_age_ns=cycle_start_ns - timestamp_ns,
                tx_records=tuple(records),
                tx_skew_ns=tx_skew_ns,
                batch_complete_ns=batch_complete_ns,
                deadline_miss=(batch_complete_ns - cycle_start_ns) > self.period_ns,
                feedbacks=feedbacks,
                feedback_rx_done_ns=feedback_rx_done_ns,
                prior_rx_drain_wall_ns=prior_rx_drain_wall_ns,
                prior_rx_drain_cpu_ns=prior_rx_drain_cpu_ns,
                prior_rx_drained_frames=residual_before,
                inter_frame_wait_total_ns=inter_frame_wait_total_ns,
                motion_drain_wall_ns=motion_drain_wall_ns,
                motion_drain_cpu_ns=motion_drain_cpu_ns,
                rx_drain_metrics=rx_metrics,
                error_phase=transport_err_phase,
                error_motor_id=transport_err_mid,
                error_class=transport_err_cls,
            )
            self._cycles.append(audit)
            self._last_cycle_start_ns = cycle_start_ns
            return audit

    def send_force_cycle(
        self,
        target_forces_g: list[int] | tuple[int, ...],
        *,
        cycle_id: int,
        target_timestamp_ns: int | None = None,
    ) -> SixAxisCycleAudit:
        """Send one ordered six-axis write-only 0x27 target force batch."""
        if not isinstance(cycle_id, int) or cycle_id < 0:
            raise ValueError("cycle_id must be a non-negative integer")
        if len(target_forces_g) != len(MOTOR_IDS):
            raise ValueError(f"target_forces_g must have {len(MOTOR_IDS)} elements")
        for f in target_forces_g:
            if not isinstance(f, (int, float)) or not math.isfinite(f):
                raise ValueError("target_forces_g elements must be finite numbers")
        with self._lock:
            self._ensure_running_locked()
            if self._configured is None:
                raise ServoTransportError("configure_force_mode() must succeed before force TX")

            timestamp_ns = self._clock_ns() if target_timestamp_ns is None else target_timestamp_ns
            if not isinstance(timestamp_ns, int) or timestamp_ns < 0:
                raise ValueError("target_timestamp_ns must be a non-negative integer")

            t_drain_start_ns = self._clock_ns()
            t_drain_cpu_start = time.thread_time_ns()
            try:
                residual_before = self._prepare_motion_rx_locked()
            except RxSafetyError:
                self._delivery_uncertain = True
                self.stop("rx_safety_failure")
                raise
            t_drain_end_ns = self._clock_ns()
            t_drain_cpu_end = time.thread_time_ns()
            prior_rx_drain_wall_ns = t_drain_end_ns - t_drain_start_ns
            prior_rx_drain_cpu_ns = t_drain_cpu_end - t_drain_cpu_start

            cycle_start_ns = self._clock_ns()
            if self._last_cycle_start_ns is None:
                cycle_jitter_ns = 0
            else:
                cycle_jitter_ns = (
                    cycle_start_ns - self._last_cycle_start_ns - self.period_ns
                )

            records: list[SixAxisTxRecord] = []
            inter_frame_wait_total_ns = 0
            transport_err_phase: str | None = None
            transport_err_mid: int | None = None
            transport_err_cls: str | None = None

            try:
                for motor_id, force in zip(MOTOR_IDS, target_forces_g):
                    t_w_start = self._clock_ns()
                    self._wait_for_frame_interval_locked()
                    inter_frame_wait_total_ns += (self._clock_ns() - t_w_start)

                    target_int = int(round(force))
                    u16_val = encode_signed_i16_to_u16(target_int)
                    frame = pack_write_register_cmd(
                        motor_id, TARGET_FORCE_ADDR, [u16_val]
                    )
                    tx_start_ns = self._clock_ns()
                    tx_cpu_start = time.thread_time_ns()
                    written = self.bus.write(frame)
                    if written is not None and written != len(frame):
                        raise ServoTransportError(
                            f"short Force write on ID{motor_id}: {written}/{len(frame)} bytes"
                        )
                    tx_end_ns = self._clock_ns()
                    tx_cpu_end = time.thread_time_ns()
                    self._last_tx_end_ns = tx_end_ns
                    records.append(
                        SixAxisTxRecord(
                            motor_id=motor_id,
                            cycle_id=cycle_id,
                            target_position=target_int,
                            target_timestamp_ns=timestamp_ns,
                            tx_start_ns=tx_start_ns,
                            tx_end_ns=tx_end_ns,
                            response_wait=False,
                            retry_count=0,
                            residual_frames_before=(
                                residual_before if not records else len(self._residual)
                            ),
                            residual_frames_after=len(self._residual),
                            duration_ns=tx_end_ns - tx_start_ns,
                            cpu_duration_ns=tx_cpu_end - tx_cpu_start,
                            requested_bytes=len(frame),
                            written_bytes=len(frame) if written is None else written,
                            error=None,
                            serial_write_metric=getattr(self.bus, "last_write_metric", None),
                        )
                    )
            except Exception as exc:
                self._delivery_uncertain = True
                transport_err_phase = f"write_id{motor_id}"
                transport_err_mid = motor_id
                transport_err_cls = type(exc).__name__
                if isinstance(exc, J7AdapterWriteTimeout):
                    tx_end_ns = self._clock_ns()
                    records.append(SixAxisTxRecord(
                        motor_id=motor_id, cycle_id=cycle_id, target_position=int(round(force)),
                        target_timestamp_ns=timestamp_ns, tx_start_ns=tx_start_ns,
                        tx_end_ns=tx_end_ns, response_wait=False, retry_count=0,
                        residual_frames_before=residual_before if not records else len(self._residual),
                        residual_frames_after=len(self._residual),
                        duration_ns=tx_end_ns - tx_start_ns,
                        cpu_duration_ns=time.thread_time_ns() - tx_cpu_start,
                        requested_bytes=len(frame), written_bytes=-1,
                        error="SerialTimeoutException",
                        serial_write_metric=exc.write_metric,
                    ))
                    starts = [record.tx_start_ns for record in records]
                    failed_audit = SixAxisCycleAudit(
                        cycle_id=cycle_id, target_timestamp_ns=timestamp_ns,
                        cycle_start_ns=cycle_start_ns, cycle_end_ns=tx_end_ns,
                        cycle_jitter_ns=cycle_jitter_ns, target_age_ns=cycle_start_ns - timestamp_ns,
                        tx_records=tuple(records), tx_skew_ns=max(starts) - min(starts),
                        batch_complete_ns=tx_end_ns,
                        deadline_miss=(tx_end_ns - cycle_start_ns) > self.period_ns,
                        prior_rx_drain_wall_ns=prior_rx_drain_wall_ns,
                        prior_rx_drain_cpu_ns=prior_rx_drain_cpu_ns,
                        prior_rx_drained_frames=residual_before,
                        inter_frame_wait_total_ns=inter_frame_wait_total_ns,
                        error_phase=transport_err_phase, error_motor_id=motor_id,
                        error_class="SerialTimeoutException",
                    )
                    self._cycles.append(failed_audit)
                    exc.cycle_audit = failed_audit
                    if self.recoverable_write_timeouts and not self._recovery_verification:
                        self.communication_transient = True
                        raise
                self.stop("motion_write_failure")
                raise

            feedbacks: dict[int, StatusFeedback] = {}
            feedback_rx_done_ns: dict[int, int] = {}
            drain_fn = getattr(self.bus, "drain_motion_feedbacks", None)
            t_motion_drain_start_ns = self._clock_ns()
            t_motion_drain_cpu_start = time.thread_time_ns()
            drain_iterations = 0
            cum_in_waiting_calls = 0
            cum_read_calls = 0
            cum_bytes_read = 0
            max_read_lat_ns = 0
            if drain_fn is not None:
                deadline_ns = self._clock_ns() + int(self.rx_drain_budget_s * 1e9)
                while True:
                    drain_iterations += 1
                    new_fbs = drain_fn()
                    received_ns = self._clock_ns()
                    for mid in new_fbs:
                        feedback_rx_done_ns[mid] = received_ns
                    rx_m = getattr(self.bus, "last_rx_drain_metrics", None)
                    if rx_m is not None:
                        cum_in_waiting_calls += getattr(rx_m, "in_waiting_calls", 0)
                        cum_read_calls += getattr(rx_m, "read_calls", 0)
                        cum_bytes_read += getattr(rx_m, "bytes_read", 0)
                        lat = getattr(rx_m, "max_read_latency_ns", 0)
                        if lat > max_read_lat_ns:
                            max_read_lat_ns = lat
                    feedbacks.update(new_fbs)
                    if len(feedbacks) >= len(MOTOR_IDS) or self._clock_ns() >= deadline_ns:
                        break
            t_motion_drain_end_ns = self._clock_ns()
            t_motion_drain_cpu_end = time.thread_time_ns()
            motion_drain_wall_ns = t_motion_drain_end_ns - t_motion_drain_start_ns
            motion_drain_cpu_ns = t_motion_drain_cpu_end - t_motion_drain_cpu_start
            rx_metrics = None
            if drain_fn is not None:
                rx_metrics = RxDrainMetrics(
                    timestamp_start_ns=t_motion_drain_start_ns,
                    timestamp_end_ns=t_motion_drain_end_ns,
                    wall_duration_ns=motion_drain_wall_ns,
                    cpu_duration_ns=motion_drain_cpu_ns,
                    in_waiting_calls=cum_in_waiting_calls,
                    read_calls=cum_read_calls,
                    bytes_read=cum_bytes_read,
                    max_read_latency_ns=max_read_lat_ns,
                    feedbacks_extracted=len(feedbacks),
                    drain_iterations=drain_iterations,
                )

            batch_complete_ns = self._clock_ns()
            cycle_end_ns = batch_complete_ns
            tx_starts = [record.tx_start_ns for record in records]
            tx_skew_ns = max(tx_starts) - min(tx_starts) if tx_starts else 0
            audit = SixAxisCycleAudit(
                cycle_id=cycle_id,
                target_timestamp_ns=timestamp_ns,
                cycle_start_ns=cycle_start_ns,
                cycle_end_ns=cycle_end_ns,
                cycle_jitter_ns=cycle_jitter_ns,
                target_age_ns=cycle_start_ns - timestamp_ns,
                tx_records=tuple(records),
                tx_skew_ns=tx_skew_ns,
                batch_complete_ns=batch_complete_ns,
                deadline_miss=(batch_complete_ns - cycle_start_ns) > self.period_ns,
                feedbacks=feedbacks,
                feedback_rx_done_ns=feedback_rx_done_ns,
                prior_rx_drain_wall_ns=prior_rx_drain_wall_ns,
                prior_rx_drain_cpu_ns=prior_rx_drain_cpu_ns,
                prior_rx_drained_frames=residual_before,
                inter_frame_wait_total_ns=inter_frame_wait_total_ns,
                motion_drain_wall_ns=motion_drain_wall_ns,
                motion_drain_cpu_ns=motion_drain_cpu_ns,
                rx_drain_metrics=rx_metrics,
                error_phase=transport_err_phase,
                error_motor_id=transport_err_mid,
                error_class=transport_err_cls,
            )
            self._cycles.append(audit)
            self._last_cycle_start_ns = cycle_start_ns
            return audit

    def _wait_for_frame_interval_locked(self) -> None:
        if self._last_tx_end_ns is None:
            return
        remaining_ns = self.min_frame_interval_ns - (
            self._clock_ns() - self._last_tx_end_ns
        )
        if remaining_ns > 0:
            if remaining_ns < 1_500_000 and self._clock_ns is time.monotonic_ns:
                target_ns = self._clock_ns() + remaining_ns
                while self._clock_ns() < target_ns:
                    pass
            else:
                self._sleep_fn(remaining_ns / 1e9)

    def _read_available_locked(self) -> bytes:
        reader = getattr(self.bus, "read_available", None)
        if reader is None:
            return b""
        data = reader()
        if data is None:
            return b""
        if not isinstance(data, (bytes, bytearray)):
            raise ServoTransportError("read_available() must return bytes-like data")
        return bytes(data)

    def _ingest_rx_locked(self) -> int:
        drain = getattr(self.bus, "drain_rx", None)
        if callable(drain):
            try:
                audit = drain()
            except Exception as exc:
                self._malformed_count += 1
                raise RxSafetyError(str(exc)) from exc
            self._pending_rx_drain_audit = audit
            self._motion_write_response_count += audit.motion_write_responses
            self._stale_response_count += audit.stale_responses
            return audit.drained_frames

        data = self._read_available_locked()
        if not data:
            return 0
        try:
            frames = self._parser.feed(data)
        except Exception as exc:
            self._malformed_count += 1
            self._parser.clear()
            raise RxSafetyError(f"malformed RX data: {exc}") from exc
        received_ns = self._clock_ns()
        for frame in frames:
            self._residual.append(_ResidualFrame(frame, received_ns))
        if (
            len(self._residual) > self.max_residual_frames
            or self.residual_bytes > self.max_residual_bytes
        ):
            raise RxSafetyError(
                "RX residual limit exceeded: "
                f"frames={len(self._residual)}/{self.max_residual_frames}, "
                f"bytes={self.residual_bytes}/{self.max_residual_bytes}"
            )
        return len(frames)

    def _prepare_motion_rx_locked(self) -> int:
        before = len(self._residual)
        self._ingest_rx_locked()
        if self.rx_policy == ServoRxPolicy.DISCARD_BEFORE_TX:
            self._drain_locked(self.rx_policy)
        elif self.rx_policy == ServoRxPolicy.REJECT_RESIDUAL and self._residual:
            raise RxSafetyError(
                f"unexpected RX residual before Servo batch: {len(self._residual)} frame(s)"
            )
        return before

    def _drain_locked(self, policy: ServoRxPolicy) -> RxResidualRecord:
        before = len(self._residual)
        discarded = 0
        discarded_bytes = 0
        motion_write_responses = 0
        stale_responses = 0
        drain_start_ns = 0
        drain_end_ns = 0
        unified_audit = self._pending_rx_drain_audit
        self._pending_rx_drain_audit = None
        if unified_audit is not None:
            discarded = unified_audit.drained_frames
            discarded_bytes = unified_audit.drained_bytes
            motion_write_responses = unified_audit.motion_write_responses
            stale_responses = unified_audit.stale_responses
            drain_start_ns = unified_audit.timestamp_start_ns
            drain_end_ns = unified_audit.timestamp_end_ns
        if policy == ServoRxPolicy.DISCARD_BEFORE_TX:
            if self._residual:
                for item in tuple(self._residual):
                    if self._is_motion_write_status(item.frame):
                        self._motion_write_response_count += 1
                        motion_write_responses += 1
                    else:
                        self._stale_response_count += 1
                        stale_responses += 1
                discarded += before
                discarded_bytes += self.residual_bytes
                self._residual.clear()
        record = RxResidualRecord(
            timestamp_ns=self._clock_ns(),
            policy=policy,
            residual_frames_before=before,
            discarded_frames=discarded,
            discarded_bytes=discarded_bytes,
            malformed_frames=self._malformed_count,
            residual_frames_after=len(self._residual),
            motion_write_responses=motion_write_responses,
            stale_responses=stale_responses,
            drain_start_ns=drain_start_ns,
            drain_end_ns=drain_end_ns,
        )
        self._rx_records.append(record)
        return record

    @staticmethod
    def _is_motion_write_status(frame: Frame) -> bool:
        if (
            frame.motor_id not in MOTOR_IDS
            or frame.command != CMD_WR_REGISTER
            or frame.register_addr != TARGET_POSITION_ADDR
        ):
            return False
        try:
            unpack_status_response(frame.raw)
        except ProtocolError as exc:
            raise RxSafetyError(
                f"invalid motion-write StatusFeedback for ID{frame.motor_id}: {exc}"
            ) from exc
        return True

    def drain_residual(self, *, policy: ServoRxPolicy | None = None) -> RxResidualRecord:
        """Ingest and classify pending RX without issuing a motion frame."""
        with self._lock:
            selected = ServoRxPolicy(policy or self.rx_policy)
            self._ingest_rx_locked()
            return self._drain_locked(selected)

    def stop(self, reason: str) -> None:
        """Enter a one-way stopped state and invoke the pause hook once."""
        with self._lock:
            if self._stopped:
                return
            self._stopped = True
            self._stop_reason = str(reason)
            if self._pause_callback is not None and not self._pause_sent:
                self._pause_sent = True
                self._pause_callback(self._stop_reason)

    def verify_recovery_hold(self, positions: list[int], *, cycle_id: int) -> SixAxisCycleAudit:
        """A new measured-position transaction, never a retry of the failed batch."""
        with self._lock:
            if self.recovery_feedback_only or self.recovery_static_targets is not None:
                raise ServoTransportError("fresh-state qualification inhibits all target writes")
            if not self.communication_transient or self._stopped:
                raise ServoTransportError("no recoverable communication transient")
            self._recovery_verification = True
            try:
                audit = self.send_cycle(positions, cycle_id=cycle_id)
                self._last_recovery_verification_audit = audit
                return audit
            finally:
                self._recovery_verification = False

    def begin_recovery_feedback_only(self) -> None:
        with self._lock:
            if self._stopped or not self.communication_transient:
                raise ServoTransportError("recovery-only communication requires a pending transient")
            self.recovery_feedback_only = True

    def request_recovery_status(self, motor_id, **kwargs):
        with self._lock:
            if self._stopped or not self.communication_transient or not self.recovery_feedback_only:
                raise ServoTransportError("recovery feedback path is not armed")
            return self.bus.request_recovery_status(motor_id, **kwargs)

    def authorize_static_resume(self, targets, *, fresh_positions, first_cycle_id, jump_limit, first_deadline_ns=None):
        """Release only the synchronized Control/Shaper reference, never ACTIVE."""
        with self._lock:
            if self._stopped or not self.communication_transient or not self.recovery_feedback_only:
                raise ServoTransportError("static resume requires a paused recovery epoch")
            validate_target_vector(targets)
            if len(fresh_positions) != len(MOTOR_IDS):
                raise ServoTransportError("static resume requires six fresh positions")
            if not isinstance(first_cycle_id, int) or not self._cycles or first_cycle_id <= self._cycles[-1].cycle_id:
                raise ServoTransportError("static resume must start after the failed transaction")
            if max(abs(t-p) for t,p in zip(targets,fresh_positions)) > jump_limit:
                raise ServoTransportError("static resume exceeds existing recovery jump boundary")
            self.recovery_static_targets = tuple(targets)
            self._static_first_cycle_id = first_cycle_id
            self._static_last_cycle_id = None
            self._static_first_deadline_ns = first_deadline_ns
            self.recovery_feedback_only = False

    def confirm_static_resume(self, audit):
        with self._lock:
            if self._stopped or self.recovery_static_targets is None or audit.cycle_id != self._static_first_cycle_id:
                raise ServoTransportError("not the first static resume transaction")
            if not self._cycles or audit is not self._cycles[-1] or set(audit.feedbacks) != set(MOTOR_IDS):
                raise ServoTransportError("first static transaction requires six new feedbacks")
            self.communication_transient = False
            self._delivery_uncertain = False

    def complete_communication_recovery(self, audit: SixAxisCycleAudit) -> None:
        with self._lock:
            if self.recovery_feedback_only or self.recovery_static_targets is not None:
                raise ServoTransportError("fresh-state qualification cannot resume motion")
            if self._stopped or not self.communication_transient:
                raise ServoTransportError("cannot resume a latched transport")
            if audit is not self._last_recovery_verification_audit:
                raise ServoTransportError("cannot resume using an old transaction snapshot")
            if set(audit.feedbacks) != set(MOTOR_IDS) or audit.error_class is not None:
                raise ServoTransportError("recovery requires a complete fresh transaction")
            self.communication_transient = False
            self._delivery_uncertain = False

    def audit_snapshot(self) -> SixAxisServoTransportAudit:
        with self._lock:
            return SixAxisServoTransportAudit(
                cycles=tuple(self._cycles),
                rx_records=tuple(self._rx_records),
                response_wait_count=self._response_wait_count,
                motion_retry_count=self._motion_retry_count,
                delivery_uncertain=self._delivery_uncertain,
                stopped=self._stopped,
                stop_reason=self._stop_reason,
                rx_motion_write_response_count=self._motion_write_response_count,
                rx_stale_response_count=self._stale_response_count,
            )

    def _ensure_running_locked(self) -> None:
        if self.recovery_feedback_only:
            raise ServoTransportError("recovery-only communication inhibits motion")
        if self._stopped:
            raise ServoTransportError("transport is stopped")
        if self.communication_transient and not self._recovery_verification and self.recovery_static_targets is None:
            raise ServoTransportError("communication recovery required before motion")


__all__ = [
    "J7SixAxisServoTransport",
    "SixAxisCycleAudit",
    "SixAxisServoBus",
    "SixAxisServoTransportAudit",
    "SixAxisTxRecord",
]
