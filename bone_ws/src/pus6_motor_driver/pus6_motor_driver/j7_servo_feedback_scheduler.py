"""Isolated J7 Servo feedback scheduler.

Design-B uses two feedback paths. The primary runtime path is the
``StatusFeedback`` returned with each 50 Hz ``0x29`` motion write and ingested
through :meth:`ingest_cycle_feedback`. The retained ``0x30`` status path is an
auxiliary explicit poll used for safety scans and diagnostics. Both paths share
the writer's UART lock and validate the expected ID and command; polling is
non-blocking with respect to an imminent Servo deadline.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable
import time

try:
    from .j7_real_uart_adapter import J7AdapterTimeout
    from .j7_six_axis_servo_transport import J7SixAxisServoTransport
    from .protocol import (
        CMD_RD_STATUS,
        FrameFieldError,
        ProtocolError,
        StatusFeedback,
        pack_read_status_cmd,
        unpack_status_response,
        validate_response,
    )
    from .safety import (
        FEEDBACK_LOWER_LIMIT,
        FEEDBACK_UPPER_LIMIT,
        MAX_CURRENT_MA,
        MOTOR_IDS,
    )
except ImportError:  # pragma: no cover - direct module execution fallback.
    from j7_real_uart_adapter import J7AdapterTimeout  # type: ignore
    from j7_six_axis_servo_transport import J7SixAxisServoTransport  # type: ignore
    from protocol import (  # type: ignore
        CMD_RD_STATUS,
        FrameFieldError,
        ProtocolError,
        StatusFeedback,
        pack_read_status_cmd,
        unpack_status_response,
        validate_response,
    )
    from safety import (  # type: ignore
        FEEDBACK_LOWER_LIMIT,
        FEEDBACK_UPPER_LIMIT,
        MAX_CURRENT_MA,
        MOTOR_IDS,
    )


class FeedbackSchedulerError(RuntimeError):
    """Base error for the isolated feedback scheduler."""


class FeedbackRxError(FeedbackSchedulerError):
    """Raised when RX bytes cannot be parsed or safely classified."""


@dataclass(frozen=True)
class FeedbackSample:
    motor_id: int
    timestamp_ns: int
    position: int | None
    current_mA: int | None
    fault: int | None
    mode: int | None
    feedback_age_ns: int
    timeout: bool
    error: str | None = None
    force_g: int | None = None
    force_adc: int | None = None
    temperature_c: int | None = None
    force_valid: bool = False


@dataclass(frozen=True)
class FeedbackCycleAudit:
    servo_cycle_id: int
    feedback_cycle_id: int
    timestamp_ns: int
    motor_ids: tuple[int, ...]
    samples: tuple[FeedbackSample, ...]
    full_scan: bool
    deferred: bool
    defer_count: int
    timeout_count: int
    rx_residual_count: int
    stale_response_count: int
    mismatch_count: int
    malformed_count: int
    rx_drain_start_ns: int = 0
    rx_drain_end_ns: int = 0
    transaction_start_ns: int = 0
    transaction_end_ns: int = 0
    scheduler_gap_ns: int = 0


@dataclass(frozen=True)
class FeedbackSchedulerAudit:
    cycles: tuple[FeedbackCycleAudit, ...]
    defer_count: int
    timeout_count: int
    rx_residual_count: int
    stale_response_count: int
    mismatch_count: int
    malformed_count: int
    stopped: bool
    stop_reason: str | None
    motion_write_response_count: int = 0
    watchdog_stop_snapshot: "FeedbackAgeWatchdogStopSnapshot | None" = None


@dataclass(frozen=True)
class FeedbackAgeWatchdogStopSnapshot:
    axis_id: int
    check_timestamp_ns: int
    last_valid_timestamp_ns: int
    age_ns: int
    threshold_ns: int
    armed: bool
    servo_cycle_id: int | None
    feedback_cycle_id: int | None
    last_poll_completion_ns: int
    rx_residual_count: int
    stale_response_count: int
    motion_write_response_count: int
    transport_stopped: bool
    stop_reason: str


# J8-E6 measured the maximum per-axis round-robin interval at 121.662 ms
# with a 50 Hz Servo cycle. One additional Servo period leaves the age
# watchdog for scheduler stalls without preempting the second axis poll.
MEASURED_MAX_AXIS_INTERVAL_S = 0.121662
FEEDBACK_AGE_MARGIN_CYCLES = 1.0


DEFAULT_WARNING_FEEDBACK_AGE_S: float = 0.2833
DEFAULT_HARD_STOP_FEEDBACK_AGE_S: float = 1.0


class J7ServoFeedbackScheduler:
    """Schedules non-blocking feedback scans over an existing Servo transport."""

    def __init__(
        self,
        writer: J7SixAxisServoTransport,
        *,
        full_scan_period_s: float = 0.100,
        deadline_guard_s: float = 0.001,
        feedback_budget_s: float = 0.010,
        servo_cycle_period_s: float = 0.020,
        timeout_escalation_misses: int = 2,
        max_feedback_age_s: float | None = None,
        warning_feedback_age_s: float = DEFAULT_WARNING_FEEDBACK_AGE_S,
        hard_stop_feedback_age_s: float = DEFAULT_HARD_STOP_FEEDBACK_AGE_S,
        clock_ns: Callable[[], int] | None = None,
        feedback_loss_policy: str = "default",
        warn_callback: Callable[[str], None] | None = None,
    ) -> None:
        if full_scan_period_s <= 0.0:
            raise ValueError("full_scan_period_s must be positive")
        if deadline_guard_s < 0.0 or feedback_budget_s <= 0.0:
            raise ValueError("deadline_guard_s must be non-negative and feedback_budget_s positive")
        if servo_cycle_period_s <= 0.0:
            raise ValueError("servo_cycle_period_s must be positive")
        if timeout_escalation_misses < 2:
            raise ValueError("timeout_escalation_misses must be at least 2")
        if feedback_loss_policy not in ("default", "consecutive_misses", "age_based"):
            raise ValueError(
                f"feedback_loss_policy must be 'default', 'consecutive_misses', or 'age_based', got {feedback_loss_policy!r}"
            )
        self.feedback_loss_policy = feedback_loss_policy
        self._warn_callback = warn_callback
        nominal_axis_interval_s = len(MOTOR_IDS) * servo_cycle_period_s
        if max_feedback_age_s is None:
            measured_or_nominal_max_s = max(
                nominal_axis_interval_s,
                MEASURED_MAX_AXIS_INTERVAL_S,
            )
            max_feedback_age_s = (
                2.0 * measured_or_nominal_max_s
                + FEEDBACK_AGE_MARGIN_CYCLES * servo_cycle_period_s
            )
        if max_feedback_age_s < nominal_axis_interval_s:
            raise ValueError("max_feedback_age_s must cover at least one nominal axis interval")
        self.writer = writer
        self.bus = writer.bus
        self.full_scan_period_ns = int(full_scan_period_s * 1e9)
        self.deadline_guard_ns = int(deadline_guard_s * 1e9)
        self.feedback_budget_ns = int(feedback_budget_s * 1e9)
        self.servo_cycle_period_ns = int(servo_cycle_period_s * 1e9)
        self.nominal_axis_interval_ns = len(MOTOR_IDS) * self.servo_cycle_period_ns
        self.timeout_escalation_misses = timeout_escalation_misses
        if self.feedback_loss_policy == "age_based":
            self.warning_feedback_age_ns = int(warning_feedback_age_s * 1e9)
            self.hard_stop_feedback_age_ns = int(hard_stop_feedback_age_s * 1e9)
            self.max_feedback_age_ns = self.hard_stop_feedback_age_ns
        else:
            self.warning_feedback_age_ns = int(max_feedback_age_s * 1e9)
            self.hard_stop_feedback_age_ns = int(max_feedback_age_s * 1e9)
            self.max_feedback_age_ns = int(max_feedback_age_s * 1e9)
        self._clock_ns = clock_ns or time.monotonic_ns
        self._uart_lock = writer.uart_lock

        self._cycles: list[FeedbackCycleAudit] = []
        self._next_axis_index = 0
        self._feedback_cycle_id = 0
        self._last_full_scan_ns = self._clock_ns()
        self._defer_count = 0
        self._timeout_count = 0
        self._rx_residual_count = 0
        self._stale_response_count = 0
        self._mismatch_count = 0
        self._malformed_count = 0
        self._stopped = False
        self._stop_reason: str | None = None
        self._last_feedback_ns: dict[int, int] = {mid: 0 for mid in MOTOR_IDS}
        self._consecutive_timeouts: dict[int, int] = {mid: 0 for mid in MOTOR_IDS}
        self._consecutive_incomplete_cycles = 0
        self._initialized_axes: set[int] = set()
        self._feedback_age_watchdog_armed = False
        self._motion_write_response_count = 0
        self._last_rx_drain_start_ns = 0
        self._last_rx_drain_end_ns = 0
        self._last_poll_completion_ns = 0
        self._last_servo_cycle_id: int | None = None
        self._last_feedback_cycle_id: int | None = None
        self._watchdog_stop_snapshot: FeedbackAgeWatchdogStopSnapshot | None = None
        self._last_scheduler_gap_ns = 0

    @property
    def stopped(self) -> bool:
        return self._stopped or self.writer.stopped

    @property
    def last_scheduler_gap_ns(self) -> int:
        return self._last_scheduler_gap_ns

    @property
    def feedback_age_watchdog_armed(self) -> bool:
        return self._feedback_age_watchdog_armed

    def notify_servo_cycle(self, servo_cycle_id: int, timestamp_ns: int) -> None:
        """Validate the cycle identity supplied by the Servo writer."""
        if not isinstance(servo_cycle_id, int) or servo_cycle_id < 0:
            raise ValueError("servo_cycle_id must be a non-negative integer")
        if not isinstance(timestamp_ns, int) or timestamp_ns < 0:
            raise ValueError("timestamp_ns must be a non-negative integer")

    def poll_once(
        self,
        *,
        servo_cycle_id: int,
        servo_cycle_timestamp_ns: int,
        servo_cycle_due_ns: int | None = None,
        force_full_scan: bool = False,
    ) -> FeedbackCycleAudit:
        """Poll one round-robin axis, or all axes when a full scan is due.

        The UART lock is acquired non-blocking.  A busy writer or an imminent
        Servo deadline produces an audit-only deferred cycle and performs no
        I/O.
        """
        self.notify_servo_cycle(servo_cycle_id, servo_cycle_timestamp_ns)
        if self.stopped:
            raise FeedbackSchedulerError("feedback scheduler is stopped")

        now_ns = self._clock_ns()
        # This check precedes the next read. The default threshold is above
        # two measured axis intervals, so a first timeout remains recoverable
        # until the next same-axis poll can execute.
        self._enforce_feedback_age(now_ns)
        if servo_cycle_due_ns is not None and now_ns + self.deadline_guard_ns >= servo_cycle_due_ns:
            return self._deferred_audit(
                servo_cycle_id, now_ns, full_scan=False, reason="servo_deadline"
            )
        if not self._uart_lock.acquire(blocking=False):
            return self._deferred_audit(
                servo_cycle_id, now_ns, full_scan=False, reason="uart_busy"
            )

        try:
            cycle_timestamp_ns = self._clock_ns()
            full_scan = force_full_scan or (
                cycle_timestamp_ns - self._last_full_scan_ns >= self.full_scan_period_ns
            )
            axis_ids = tuple(MOTOR_IDS) if full_scan else (MOTOR_IDS[self._next_axis_index],)
            if full_scan:
                self._last_full_scan_ns = cycle_timestamp_ns
                self._next_axis_index = 0
            else:
                self._next_axis_index = (self._next_axis_index + 1) % len(MOTOR_IDS)

            residual_count = self._clear_writer_and_bus_residual_locked()
            samples: list[FeedbackSample] = []
            timeout_count = 0
            rx_drain_start_ns = self._last_rx_drain_start_ns
            rx_drain_end_ns = self._last_rx_drain_end_ns
            transaction_start_ns = self._clock_ns()
            for motor_id in axis_ids:
                sample = self._read_axis_locked(
                    motor_id,
                    servo_cycle_timestamp_ns,
                    residual_count,
                )
                samples.append(sample)
                if sample.timeout:
                    timeout_count += 1
                    break

            transaction_end_ns = self._clock_ns()
            self._last_poll_completion_ns = transaction_end_ns
            self._feedback_cycle_id += 1
            self._last_feedback_cycle_id = self._feedback_cycle_id
            audit = FeedbackCycleAudit(
                servo_cycle_id=servo_cycle_id,
                feedback_cycle_id=self._feedback_cycle_id,
                timestamp_ns=cycle_timestamp_ns,
                motor_ids=axis_ids,
                samples=tuple(samples),
                full_scan=full_scan,
                deferred=False,
                defer_count=self._defer_count,
                timeout_count=timeout_count,
                rx_residual_count=residual_count,
                stale_response_count=self._stale_response_count,
                mismatch_count=self._mismatch_count,
                malformed_count=self._malformed_count,
                rx_drain_start_ns=rx_drain_start_ns,
                rx_drain_end_ns=rx_drain_end_ns,
                transaction_start_ns=transaction_start_ns,
                transaction_end_ns=transaction_end_ns,
            )
            if (
                full_scan
                and not self._feedback_age_watchdog_armed
                and tuple(sample.motor_id for sample in samples) == tuple(MOTOR_IDS)
                and all(self._sample_is_healthy(sample) for sample in samples)
            ):
                self._arm_feedback_age_watchdog()
            self._cycles.append(audit)
            return audit
        finally:
            self._uart_lock.release()

    def full_scan(
        self,
        *,
        servo_cycle_id: int,
        servo_cycle_timestamp_ns: int,
        servo_cycle_due_ns: int | None = None,
    ) -> FeedbackCycleAudit:
        """Request a low-frequency full ID1-ID6 safety scan."""
        return self.poll_once(
            servo_cycle_id=servo_cycle_id,
            servo_cycle_timestamp_ns=servo_cycle_timestamp_ns,
            servo_cycle_due_ns=servo_cycle_due_ns,
            force_full_scan=True,
        )

    def ingest_cycle_feedback(
        self,
        feedbacks: dict[int, StatusFeedback],
        *,
        servo_cycle_id: int,
        servo_cycle_timestamp_ns: int,
        per_axis_rx_done_ns: dict[int, int] | None = None,
    ) -> FeedbackCycleAudit:
        """Ingest 0x29 StatusFeedback responses received directly from 50Hz motion writes.

        This method implements the Two-Tier Design B safety contract:
        - Tier 1: Per-cycle feedback completeness tracking (missing responses & incomplete cycles)
        - Tier 2: Servo scheduler / deadline watchdog on execution gaps
        """
        now_ns = self._clock_ns()

        # Tier 2: Servo scheduler / deadline watchdog
        scheduler_gap_ns = 0
        if self._feedback_age_watchdog_armed and self._last_poll_completion_ns > 0:
            scheduler_gap_ns = now_ns - self._last_poll_completion_ns
            self._last_scheduler_gap_ns = scheduler_gap_ns
            if self.feedback_loss_policy == "age_based":
                if scheduler_gap_ns > self.hard_stop_feedback_age_ns:
                    reason = "scheduler_deadline_exceeded"
                    err_msg = f"Servo scheduler deadline exceeded: gap {scheduler_gap_ns / 1e6:.1f}ms > {self.hard_stop_feedback_age_ns / 1e6:.1f}ms"
                    self._watchdog_stop_snapshot = FeedbackAgeWatchdogStopSnapshot(
                        axis_id=0,
                        check_timestamp_ns=now_ns,
                        last_valid_timestamp_ns=self._last_poll_completion_ns,
                        age_ns=scheduler_gap_ns,
                        threshold_ns=self.hard_stop_feedback_age_ns,
                        armed=self._feedback_age_watchdog_armed,
                        servo_cycle_id=self._last_servo_cycle_id,
                        feedback_cycle_id=self._last_feedback_cycle_id,
                        last_poll_completion_ns=self._last_poll_completion_ns,
                        rx_residual_count=self._rx_residual_count,
                        stale_response_count=self._stale_response_count,
                        motion_write_response_count=self._motion_write_response_count,
                        transport_stopped=self.writer.stopped,
                        stop_reason=reason,
                    )
                    self._stop(reason)
                    raise FeedbackSchedulerError(err_msg)
                elif scheduler_gap_ns > self.warning_feedback_age_ns:
                    if self._warn_callback is not None:
                        self._warn_callback(
                            f"Servo scheduler gap {scheduler_gap_ns / 1e6:.1f}ms > {self.warning_feedback_age_ns / 1e6:.1f}ms "
                            f"(threshold <= {self.hard_stop_feedback_age_ns / 1e6:.1f}ms): "
                            f"transient scheduling stall warning, continuing in research mode."
                        )
            else:
                if scheduler_gap_ns > self.max_feedback_age_ns:
                    reason = "scheduler_deadline_exceeded"
                    err_msg = f"Servo scheduler deadline exceeded: gap {scheduler_gap_ns / 1e6:.1f}ms > {self.max_feedback_age_ns / 1e6:.1f}ms"
                    self._watchdog_stop_snapshot = FeedbackAgeWatchdogStopSnapshot(
                        axis_id=0,
                        check_timestamp_ns=now_ns,
                        last_valid_timestamp_ns=self._last_poll_completion_ns,
                        age_ns=scheduler_gap_ns,
                        threshold_ns=self.max_feedback_age_ns,
                        armed=self._feedback_age_watchdog_armed,
                        servo_cycle_id=self._last_servo_cycle_id,
                        feedback_cycle_id=self._last_feedback_cycle_id,
                        last_poll_completion_ns=self._last_poll_completion_ns,
                        rx_residual_count=self._rx_residual_count,
                        stale_response_count=self._stale_response_count,
                        motion_write_response_count=self._motion_write_response_count,
                        transport_stopped=self.writer.stopped,
                        stop_reason=reason,
                    )
                    self._stop(reason)
                    raise FeedbackSchedulerError(err_msg)

        samples: list[FeedbackSample] = []
        timeout_count = 0

        for motor_id in MOTOR_IDS:
            if motor_id in feedbacks:
                status = feedbacks[motor_id]
                age_ns = self._feedback_age_ns(motor_id, now_ns)
                self._validate_status(motor_id, status)
                self._initialized_axes.add(motor_id)
                received_ns = (per_axis_rx_done_ns or {}).get(motor_id, now_ns)
                self._last_feedback_ns[motor_id] = received_ns
                self._consecutive_timeouts[motor_id] = 0
                sample = FeedbackSample(
                    motor_id=motor_id,
                    timestamp_ns=received_ns,
                    position=status.current_position,
                    current_mA=status.current_mA,
                    fault=status.error_code,
                    mode=None,
                    feedback_age_ns=age_ns,
                    timeout=False,
                    force_g=status.force_g,
                    force_adc=status.force_adc,
                    temperature_c=status.temperature,
                    force_valid=self._force_is_valid(status),
                )
            else:
                timeout_count += 1
                self._record_timeout(motor_id, now_ns)
                sample = FeedbackSample(
                    motor_id=motor_id,
                    timestamp_ns=now_ns,
                    position=None,
                    current_mA=None,
                    fault=None,
                    mode=None,
                    feedback_age_ns=self._feedback_age_ns(motor_id, now_ns),
                    timeout=True,
                    error="missing_0x29_feedback",
                )
            samples.append(sample)

        # Tier 1: Per-cycle feedback completeness
        if len(feedbacks) < len(MOTOR_IDS):
            self._consecutive_incomplete_cycles += 1
            if (
                self._feedback_age_watchdog_armed
                and self._consecutive_incomplete_cycles >= self.timeout_escalation_misses
            ):
                if self.feedback_loss_policy != "age_based":
                    self._stop("persistent_incomplete_cycle_feedback_loss")
                    raise FeedbackSchedulerError("persistent incomplete cycle feedback loss")
        else:
            self._consecutive_incomplete_cycles = 0

        # In age-based policy, enforce per-axis feedback age watchdog across all initialized axes
        if self.feedback_loss_policy == "age_based":
            self._enforce_feedback_age(now_ns)

        if len(self._initialized_axes) == len(MOTOR_IDS):
            self._feedback_age_watchdog_armed = True

        self._last_poll_completion_ns = now_ns
        self._feedback_cycle_id += 1
        self._last_feedback_cycle_id = self._feedback_cycle_id
        audit = FeedbackCycleAudit(
            servo_cycle_id=servo_cycle_id,
            feedback_cycle_id=self._feedback_cycle_id,
            timestamp_ns=now_ns,
            motor_ids=tuple(MOTOR_IDS),
            samples=tuple(samples),
            full_scan=True,
            deferred=False,
            defer_count=self._defer_count,
            timeout_count=timeout_count,
            rx_residual_count=0,
            stale_response_count=self._stale_response_count,
            mismatch_count=self._mismatch_count,
            malformed_count=self._malformed_count,
            rx_drain_start_ns=now_ns,
            rx_drain_end_ns=now_ns,
            transaction_start_ns=now_ns,
            transaction_end_ns=now_ns,
            scheduler_gap_ns=scheduler_gap_ns,
        )
        if (
            not self._feedback_age_watchdog_armed
            and self._initialized_axes == set(MOTOR_IDS)
            and all(self._sample_is_healthy(sample) for sample in samples)
        ):
            self._arm_feedback_age_watchdog()
        self._cycles.append(audit)
        return audit

    def _deferred_audit(
        self,
        servo_cycle_id: int,
        timestamp_ns: int,
        *,
        full_scan: bool,
        reason: str,
    ) -> FeedbackCycleAudit:
        self._defer_count += 1
        self._feedback_cycle_id += 1
        audit = FeedbackCycleAudit(
            servo_cycle_id=servo_cycle_id,
            feedback_cycle_id=self._feedback_cycle_id,
            timestamp_ns=timestamp_ns,
            motor_ids=tuple(),
            samples=tuple(),
            full_scan=full_scan,
            deferred=True,
            defer_count=self._defer_count,
            timeout_count=0,
            rx_residual_count=0,
            stale_response_count=self._stale_response_count,
            mismatch_count=self._mismatch_count,
            malformed_count=self._malformed_count,
        )
        self._cycles.append(audit)
        return audit

    def _clear_writer_and_bus_residual_locked(self) -> int:
        """Drain RX through the transport-owned path before one 0x30 request."""
        try:
            record = self.writer.drain_residual(policy="discard_before_tx")
        except Exception as exc:
            self._malformed_count += 1
            self._stop("rx_residual_failure")
            raise FeedbackRxError(str(exc)) from exc
        self._last_rx_drain_start_ns = record.drain_start_ns
        self._last_rx_drain_end_ns = record.drain_end_ns
        self._rx_residual_count += record.discarded_frames
        self._stale_response_count += record.stale_responses
        self._motion_write_response_count += record.motion_write_responses
        return record.discarded_frames

    def _read_axis_locked(
        self,
        motor_id: int,
        servo_cycle_timestamp_ns: int,
        residual_count: int,
    ) -> FeedbackSample:
        start_ns = self._clock_ns()
        try:
            status_raw = self.bus.request(
                pack_read_status_cmd(motor_id),
                expected_id=motor_id,
                expected_command=CMD_RD_STATUS,
                expected_register=0x0000,
                retries=0,
            )
            status_frame = validate_response(
                status_raw,
                expected_id=motor_id,
                expected_command=CMD_RD_STATUS,
            )
            if status_frame.register_addr != 0x0000:
                raise FrameFieldError(
                    f"response register 0x{status_frame.register_addr:04X} != expected 0x0000"
                )
            status = unpack_status_response(status_frame.raw)
            end_ns = self._clock_ns()
            age_ns = self._feedback_age_ns(motor_id, end_ns)
            self._validate_status(motor_id, status)
            self._initialized_axes.add(motor_id)
            self._last_feedback_ns[motor_id] = end_ns
            self._consecutive_timeouts[motor_id] = 0
            if end_ns - start_ns > self.feedback_budget_ns:
                self._timeout_count += 1
            return FeedbackSample(
                motor_id=motor_id,
                timestamp_ns=end_ns,
                position=status.current_position,
                current_mA=status.current_mA,
                fault=status.error_code,
                mode=None,
                feedback_age_ns=age_ns,
                timeout=False,
                force_g=status.force_g,
                force_adc=status.force_adc,
                temperature_c=status.temperature,
                force_valid=self._force_is_valid(status),
            )
        except (TimeoutError, J7AdapterTimeout) as exc:
            self._timeout_count += 1
            timestamp_ns = self._clock_ns()
            self._record_timeout(motor_id, timestamp_ns)
            return FeedbackSample(
                motor_id=motor_id,
                timestamp_ns=timestamp_ns,
                position=None,
                current_mA=None,
                fault=None,
                mode=None,
                feedback_age_ns=self._feedback_age_ns(motor_id, timestamp_ns),
                timeout=True,
                error=str(exc),
            )
        except FrameFieldError as exc:
            self._mismatch_count += 1
            self._stop("feedback_mismatch")
            raise FeedbackRxError(str(exc)) from exc
        except ProtocolError as exc:
            self._malformed_count += 1
            self._stop("malformed_feedback_response")
            raise FeedbackRxError(str(exc)) from exc
        except Exception as exc:
            self._stop("feedback_bus_failure")
            raise FeedbackRxError(str(exc)) from exc

    @staticmethod
    def _force_is_valid(status) -> bool:
        # Wire validity and device fault status; not physical force calibration.
        return (type(status.force_g) is int and -32768 <= status.force_g <= 32767
                and type(status.force_adc) is int and 0 <= status.force_adc <= 4095
                and status.error_code == 0)

    def _validate_status(self, motor_id: int, status) -> None:
        if status.error_code != 0:
            self._stop(f"motor_fault_ID{motor_id}")
            return
        if status.current_mA > MAX_CURRENT_MA:
            self._stop(f"overcurrent_ID{motor_id}")
            return
        if not (FEEDBACK_LOWER_LIMIT <= status.current_position <= FEEDBACK_UPPER_LIMIT):
            self._stop(f"feedback_position_out_of_range_ID{motor_id}")

    def _sample_is_healthy(self, sample: FeedbackSample) -> bool:
        return (
            not sample.timeout
            and sample.position is not None
            and sample.current_mA is not None
            and sample.fault == 0
            and FEEDBACK_LOWER_LIMIT <= sample.position <= FEEDBACK_UPPER_LIMIT
            and sample.current_mA <= MAX_CURRENT_MA
        )

    def _arm_feedback_age_watchdog(self) -> None:
        if self._initialized_axes != set(MOTOR_IDS):
            raise FeedbackSchedulerError(
                "feedback age watchdog requires fresh valid feedback for all axes"
            )
        self._consecutive_timeouts = {mid: 0 for mid in MOTOR_IDS}
        self._feedback_age_watchdog_armed = True

    def _feedback_age_ns(self, motor_id: int, now_ns: int) -> int:
        if motor_id not in self._initialized_axes:
            return 0
        return max(0, now_ns - self._last_feedback_ns[motor_id])

    def consecutive_timeouts(self, motor_id: int) -> int:
        return self._consecutive_timeouts.get(motor_id, 0)

    def feedback_age_ms(self, motor_id: int, now_ns: int | None = None) -> float:
        current_ns = self._clock_ns() if now_ns is None else now_ns
        return self._feedback_age_ns(motor_id, current_ns) / 1e6

    def _record_timeout(self, motor_id: int, now_ns: int) -> None:
        if not self._feedback_age_watchdog_armed:
            return
        self._consecutive_timeouts[motor_id] += 1
        if self.feedback_loss_policy == "age_based":
            age_ns = self._feedback_age_ns(motor_id, now_ns)
            if age_ns > self.hard_stop_feedback_age_ns:
                reason = f"feedback_age_exceeded_ID{motor_id}"
                err_msg = f"ID{motor_id} feedback age exceeded {self.hard_stop_feedback_age_ns / 1e6:.1f}ms"
                last_ns = self._last_feedback_ns[motor_id]
                self._watchdog_stop_snapshot = FeedbackAgeWatchdogStopSnapshot(
                    axis_id=motor_id,
                    check_timestamp_ns=now_ns,
                    last_valid_timestamp_ns=last_ns,
                    age_ns=age_ns,
                    threshold_ns=self.hard_stop_feedback_age_ns,
                    armed=self._feedback_age_watchdog_armed,
                    servo_cycle_id=self._last_servo_cycle_id,
                    feedback_cycle_id=self._last_feedback_cycle_id,
                    last_poll_completion_ns=self._last_poll_completion_ns,
                    rx_residual_count=self._rx_residual_count,
                    stale_response_count=self._stale_response_count,
                    motion_write_response_count=self._motion_write_response_count,
                    transport_stopped=self.writer.stopped,
                    stop_reason=reason,
                )
                self._stop(reason)
                raise FeedbackSchedulerError(err_msg)
            elif age_ns > self.warning_feedback_age_ns or self._consecutive_timeouts[motor_id] >= self.timeout_escalation_misses:
                age_ms = age_ns / 1e6
                if self._warn_callback is not None:
                    self._warn_callback(
                        f"ID{motor_id} transient feedback miss count={self._consecutive_timeouts[motor_id]} "
                        f"(feedback_age={age_ms:.1f}ms <= {self.hard_stop_feedback_age_ns / 1e6:.1f}ms): "
                        f"transient jitter warning, continuing in research mode."
                    )
        else:
            if self._consecutive_timeouts[motor_id] >= self.timeout_escalation_misses:
                self._stop(f"persistent_feedback_loss_ID{motor_id}")

    def _enforce_feedback_age(self, now_ns: int) -> None:
        if not self._feedback_age_watchdog_armed:
            return
        if self.feedback_loss_policy == "age_based":
            stale_axes = [
                mid
                for mid in MOTOR_IDS
                if mid in self._initialized_axes
                and now_ns - self._last_feedback_ns[mid] > self.hard_stop_feedback_age_ns
            ]
            if stale_axes:
                trigger_mid = stale_axes[0]
                reason = f"feedback_age_exceeded_ID{trigger_mid}"
                err_msg = f"ID{trigger_mid} feedback age exceeded {self.hard_stop_feedback_age_ns / 1e6:.1f}ms"
                last_ns = self._last_feedback_ns[trigger_mid]
                self._watchdog_stop_snapshot = FeedbackAgeWatchdogStopSnapshot(
                    axis_id=trigger_mid,
                    check_timestamp_ns=now_ns,
                    last_valid_timestamp_ns=last_ns,
                    age_ns=now_ns - last_ns,
                    threshold_ns=self.hard_stop_feedback_age_ns,
                    armed=self._feedback_age_watchdog_armed,
                    servo_cycle_id=self._last_servo_cycle_id,
                    feedback_cycle_id=self._last_feedback_cycle_id,
                    last_poll_completion_ns=self._last_poll_completion_ns,
                    rx_residual_count=self._rx_residual_count,
                    stale_response_count=self._stale_response_count,
                    motion_write_response_count=self._motion_write_response_count,
                    transport_stopped=self.writer.stopped,
                    stop_reason=reason,
                )
                self._stop(reason)
                raise FeedbackSchedulerError(err_msg)

            warning_axes = [
                mid
                for mid in MOTOR_IDS
                if mid in self._initialized_axes
                and self.warning_feedback_age_ns < (now_ns - self._last_feedback_ns[mid]) <= self.hard_stop_feedback_age_ns
            ]
            if warning_axes and self._warn_callback is not None:
                trigger_mid = warning_axes[0]
                age_ms = (now_ns - self._last_feedback_ns[trigger_mid]) / 1e6
                self._warn_callback(
                    f"ID{trigger_mid} feedback age warning: age={age_ms:.1f}ms > {self.warning_feedback_age_ns / 1e6:.1f}ms "
                    f"(threshold <= {self.hard_stop_feedback_age_ns / 1e6:.1f}ms), continuing in research mode."
                )
        else:
            stale_axes = [
                mid
                for mid in MOTOR_IDS
                if mid in self._initialized_axes
                and now_ns - self._last_feedback_ns[mid] > self.max_feedback_age_ns
            ]
            if not stale_axes:
                return
            trigger_mid = stale_axes[0]
            reason = f"feedback_age_exceeded_ID{trigger_mid}"
            err_msg = f"ID{trigger_mid} feedback age exceeded {self.max_feedback_age_ns / 1e6:.1f}ms"
            last_ns = self._last_feedback_ns[trigger_mid]
            self._watchdog_stop_snapshot = FeedbackAgeWatchdogStopSnapshot(
                axis_id=trigger_mid,
                check_timestamp_ns=now_ns,
                last_valid_timestamp_ns=last_ns,
                age_ns=now_ns - last_ns,
                threshold_ns=self.max_feedback_age_ns,
                armed=self._feedback_age_watchdog_armed,
                servo_cycle_id=self._last_servo_cycle_id,
                feedback_cycle_id=self._last_feedback_cycle_id,
                last_poll_completion_ns=self._last_poll_completion_ns,
                rx_residual_count=self._rx_residual_count,
                stale_response_count=self._stale_response_count,
                motion_write_response_count=self._motion_write_response_count,
                transport_stopped=self.writer.stopped,
                stop_reason=reason,
            )
            self._stop(reason)
            raise FeedbackSchedulerError(err_msg)

    def is_feedback_stale_warning(self, now_ns: int | None = None) -> bool:
        """Returns True if feedback age or scheduler gap is in warning range (283.3ms < age <= 1.0s)."""
        if not self._feedback_age_watchdog_armed:
            return False
        current_ns = self._clock_ns() if now_ns is None else now_ns
        if self._last_poll_completion_ns > 0:
            sched_gap = current_ns - self._last_poll_completion_ns
            if self.warning_feedback_age_ns < sched_gap <= self.hard_stop_feedback_age_ns:
                return True
        for mid in self._initialized_axes:
            age_ns = self._feedback_age_ns(mid, current_ns)
            if self.warning_feedback_age_ns < age_ns <= self.hard_stop_feedback_age_ns:
                return True
        return False

    def is_feedback_normal(self, now_ns: int | None = None) -> bool:
        """Returns True if all initialized axes and scheduler gap are <= warning_feedback_age_ns."""
        if not self._feedback_age_watchdog_armed:
            return True
        current_ns = self._clock_ns() if now_ns is None else now_ns
        if self._last_poll_completion_ns > 0:
            if (current_ns - self._last_poll_completion_ns) > self.warning_feedback_age_ns:
                return False
        for mid in self._initialized_axes:
            if self._feedback_age_ns(mid, current_ns) > self.warning_feedback_age_ns:
                return False
        return True

    def _stop(self, reason: str) -> None:
        if self._stopped:
            return
        self._stopped = True
        self._stop_reason = reason
        self.writer.stop(reason)

    def audit_snapshot(self) -> FeedbackSchedulerAudit:
        return FeedbackSchedulerAudit(
            cycles=tuple(self._cycles),
            defer_count=self._defer_count,
            timeout_count=self._timeout_count,
            rx_residual_count=self._rx_residual_count,
            stale_response_count=self._stale_response_count,
            mismatch_count=self._mismatch_count,
            malformed_count=self._malformed_count,
            stopped=self.stopped,
            stop_reason=self._stop_reason,
            motion_write_response_count=self._motion_write_response_count,
            watchdog_stop_snapshot=self._watchdog_stop_snapshot,
        )


__all__ = [
    "FeedbackCycleAudit",
    "FeedbackSample",
    "FeedbackSchedulerAudit",
    "FeedbackSchedulerError",
    "FeedbackRxError",
    "FeedbackAgeWatchdogStopSnapshot",
    "J7ServoFeedbackScheduler",
]
