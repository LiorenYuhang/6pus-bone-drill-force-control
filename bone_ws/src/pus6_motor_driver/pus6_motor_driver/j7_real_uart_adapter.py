"""Isolated J7 UART adapter and six-axis stop path.

The adapter accepts an already-created serial-like object.  It never opens a
port itself, which keeps C9 Mock validation physically safe.  One RX owner
parses complete response frames before exposing canonical bytes to the C5/C6
transport consumers; all physical access is serialized by ``uart_lock``.
"""

from __future__ import annotations

from collections import deque
from dataclasses import asdict, dataclass
from typing import Callable, Deque, Protocol
import threading
import time
import resource
from serial import SerialTimeoutException

# J14-A1: A0 retained-window P99=3.144ms; 100--300ms stalls are pathological.
# This is a per-write failure deadline, not a 20ms cycle scheduling guarantee.
J14_UART_WRITE_TIMEOUT_S = 0.030

try:
    from .protocol import (
        CMD_WR_REGISTER,
        Frame,
        FrameStreamParser,
        ProtocolError,
        RSP_FRAME_HEADER,
        StatusFeedback,
        pack_write_register_cmd,
        unpack_status_response,
        validate_response,
    )
    from .safety import MOTOR_IDS, PAUSE_ADDR, TARGET_POSITION_ADDR, TARGET_FORCE_ADDR
except ImportError:  # pragma: no cover - direct module execution fallback.
    from protocol import (  # type: ignore
        CMD_WR_REGISTER,
        Frame,
        FrameStreamParser,
        ProtocolError,
        RSP_FRAME_HEADER,
        StatusFeedback,
        pack_write_register_cmd,
        unpack_status_response,
        validate_response,
    )
    from safety import MOTOR_IDS, PAUSE_ADDR, TARGET_POSITION_ADDR, TARGET_FORCE_ADDR  # type: ignore


class J7SerialLike(Protocol):
    def read(self, size: int = 1) -> bytes: ...

    def write(self, data: bytes) -> int | None: ...


class J7AdapterError(RuntimeError):
    """Base adapter error."""


class J7AdapterTimeout(J7AdapterError):
    """A bounded request or stop response timed out."""


class J7AdapterWriteTimeout(J7AdapterError):
    """Finite serial TX deadline expired; partial frame delivery is unknown."""

    def __init__(self, metric: SerialWriteMetric, timeout_s: float) -> None:
        self.write_metric = metric
        self.timeout_s = timeout_s
        super().__init__(
            f"SerialTimeoutException: UART write timeout after "
            f"{metric.wall_duration_ns / 1e6:.3f}ms (deadline={timeout_s * 1000:.1f}ms)"
        )


class J7AdapterRxError(J7AdapterError):
    """RX bytes could not be safely parsed or bounded."""


@dataclass(frozen=True)
class RxOwnerAudit:
    residual_frames: int
    residual_bytes: int
    stale_response_count: int
    motion_write_response_count: int
    feedback_response_count: int
    malformed_count: int
    overflow_count: int


@dataclass(frozen=True)
class RxDrainAudit:
    timestamp_start_ns: int
    timestamp_end_ns: int
    drained_frames: int
    drained_bytes: int
    motion_write_responses: int
    stale_responses: int
    residual_frames_after: int
    wall_duration_ns: int = 0
    cpu_duration_ns: int = 0


@dataclass(frozen=True)
class TxQueueState:
    """Read-only tty queue snapshot; unavailable values remain unknown."""

    out_waiting: int | None
    in_waiting: int | None
    wall_duration_ns: int
    error: str | None = None


@dataclass(frozen=True)
class SerialWriteMetric:
    timestamp_start_ns: int
    timestamp_end_ns: int
    wall_duration_ns: int
    cpu_duration_ns: int
    requested_bytes: int
    written_bytes: int
    error: str | None = None
    configured_timeout_s: float | None = None
    queue_before: TxQueueState | None = None
    queue_after: TxQueueState | None = None
    write_seq: int | None = None
    native_thread_id: int | None = None
    process_cpu_duration_ns: int | None = None
    voluntary_ctx_delta: int | None = None
    involuntary_ctx_delta: int | None = None
    snapshot_enter_ns: int | None = None
    snapshot_exit_ns: int | None = None
    snapshot_overhead_ns: int | None = None
    clock_sync_monotonic_before_ns: int | None = None
    clock_sync_epoch_ns: int | None = None
    clock_sync_monotonic_after_ns: int | None = None


@dataclass(frozen=True)
class SerialReadMetric:
    read_seq: int
    native_thread_id: int
    t_read_enter_ns: int
    t_read_exit_ns: int
    read_wall_ns: int
    serial_call_wall_ns: int
    snapshot_overhead_ns: int
    t_snapshot_enter_ns: int
    t_snapshot_exit_ns: int
    thread_cpu_before_ns: int
    thread_cpu_after_ns: int
    read_thread_cpu_ns: int
    process_cpu_before_ns: int
    process_cpu_after_ns: int
    read_process_cpu_ns: int
    vol_ctx_before: int
    vol_ctx_after: int
    read_vol_ctx_delta: int
    invol_ctx_before: int
    invol_ctx_after: int
    read_invol_ctx_delta: int
    in_waiting_before: int
    requested_read_bytes: int
    returned_read_bytes: int | None
    configured_read_timeout_s: float | None
    read_error: str = ""


@dataclass(frozen=True)
class RxDrainMetrics:
    timestamp_start_ns: int
    timestamp_end_ns: int
    wall_duration_ns: int
    cpu_duration_ns: int
    in_waiting_calls: int
    read_calls: int
    bytes_read: int
    max_read_latency_ns: int
    feedbacks_extracted: int
    drain_iterations: int = 1


class J7RxOwner:
    """Single parser/queue owner for all response bytes from one UART."""

    def __init__(self, *, max_frames: int = 256, max_bytes: int = 32_768) -> None:
        if max_frames < 1 or max_bytes < 1:
            raise ValueError("RX limits must be positive")
        self.max_frames = max_frames
        self.max_bytes = max_bytes
        self._parser = FrameStreamParser(expected_header=RSP_FRAME_HEADER)
        self._frames: Deque[Frame] = deque()
        self._residual_bytes = 0
        self._stale_response_count = 0
        self._motion_write_response_count = 0
        self._feedback_response_count = 0
        self._malformed_count = 0
        self._overflow_count = 0

    @property
    def residual_frame_count(self) -> int:
        return len(self._frames)

    @property
    def residual_bytes(self) -> int:
        return self._residual_bytes

    def ingest(self, data: bytes) -> tuple[Frame, ...]:
        if not isinstance(data, (bytes, bytearray)):
            raise J7AdapterRxError("RX data must be bytes-like")
        if not data:
            return tuple()
        try:
            frames = self._parser.feed(bytes(data))
        except Exception as exc:
            self._malformed_count += 1
            self._parser.clear()
            raise J7AdapterRxError(f"malformed RX frame: {exc}") from exc
        for frame in frames:
            if (
                frame.motor_id in MOTOR_IDS
                and frame.command == CMD_WR_REGISTER
                and frame.register_addr in (TARGET_POSITION_ADDR, TARGET_FORCE_ADDR)
            ):
                self._is_motion_write_status(frame)
            self._frames.append(frame)
            self._residual_bytes += len(frame.raw)
        if len(self._frames) > self.max_frames or self._residual_bytes > self.max_bytes:
            self._overflow_count += 1
            raise J7AdapterRxError(
                f"RX residual limit exceeded: frames={len(self._frames)}/{self.max_frames}, "
                f"bytes={self._residual_bytes}/{self.max_bytes}"
            )
        return tuple(frames)

    @staticmethod
    def _is_motion_write_status(frame: Frame) -> bool:
        if (
            frame.motor_id not in MOTOR_IDS
            or frame.command != CMD_WR_REGISTER
            or frame.register_addr not in (TARGET_POSITION_ADDR, TARGET_FORCE_ADDR)
        ):
            return False
        try:
            unpack_status_response(frame.raw)
        except ProtocolError as exc:
            raise J7AdapterRxError(
                f"invalid motion-write StatusFeedback for ID{frame.motor_id}: {exc}"
            ) from exc
        return True

    def take_matching(
        self,
        *,
        expected_id: int,
        expected_command: int,
        expected_register: int | None = None,
    ) -> Frame | None:
        """Return one matching frame and classify all other complete frames."""
        while self._frames:
            frame = self._frames.popleft()
            self._residual_bytes -= len(frame.raw)
            if self._is_motion_write_status(frame):
                self._motion_write_response_count += 1
                continue
            if (
                frame.motor_id == expected_id
                and frame.command == expected_command
                and (expected_register is None or frame.register_addr == expected_register)
            ):
                self._feedback_response_count += 1
                return frame
            self._stale_response_count += 1
        return None

    def take_motion_write_feedbacks(self) -> dict[int, StatusFeedback]:
        """Extract all valid 0x29 StatusFeedback frames from the RX queue.

        Returns a dictionary mapping motor_id -> StatusFeedback for all received
        motion-write responses. Any other frames are classified as stale.
        """
        feedbacks: dict[int, StatusFeedback] = {}
        while self._frames:
            frame = self._frames.popleft()
            self._residual_bytes -= len(frame.raw)
            if self._is_motion_write_status(frame):
                self._motion_write_response_count += 1
                status = unpack_status_response(frame.raw)
                if frame.motor_id not in feedbacks:
                    feedbacks[frame.motor_id] = status
                else:
                    self._stale_response_count += 1
            else:
                self._stale_response_count += 1
        return feedbacks

    def drain(self) -> tuple[Frame, ...]:
        """Remove pending frames and classify motion responses separately."""
        frames = tuple(self._frames)
        self._frames.clear()
        self._residual_bytes = 0
        for frame in frames:
            if self._is_motion_write_status(frame):
                self._motion_write_response_count += 1
            else:
                self._stale_response_count += 1
        return frames

    def take_for_consumer(self) -> tuple[Frame, ...]:
        frames = tuple(self._frames)
        self._frames.clear()
        self._residual_bytes = 0
        return frames

    def audit_snapshot(self) -> RxOwnerAudit:
        return RxOwnerAudit(
            residual_frames=len(self._frames),
            residual_bytes=self._residual_bytes,
            stale_response_count=self._stale_response_count,
            motion_write_response_count=self._motion_write_response_count,
            feedback_response_count=self._feedback_response_count,
            malformed_count=self._malformed_count,
            overflow_count=self._overflow_count,
        )


@dataclass(frozen=True)
class J7RequestAudit:
    motor_id: int
    command: int
    register_address: int
    tx_start_ns: int
    tx_end_ns: int
    response_received: bool
    timeout: bool
    retry_count: int
    error: str | None = None


class J7UartAdapter:
    """C5/C6 bus adapter around an injected serial-like object.

    The object is intentionally not a serial-port factory.  A real caller
    must explicitly construct and authorize the serial instance outside this
    module; Mock tests inject ``MockSerial`` instead.
    """

    def __init__(
        self,
        serial_instance: J7SerialLike,
        *,
        baudrate: int = 921600,
        timeout_s: float = 0.05,
        min_frame_interval_s: float = 0.001,
        rx_owner: J7RxOwner | None = None,
        clock_ns: Callable[[], int] | None = None,
        sleep_fn: Callable[[float], None] | None = None,
        tx_queue_diagnostics: bool = False,
        read_observer: Callable[[SerialReadMetric], None] | None = None,
    ) -> None:
        if serial_instance is None:
            raise ValueError("serial_instance is required; this adapter never opens a port")
        if baudrate <= 0 or timeout_s <= 0.0 or min_frame_interval_s < 0.0:
            raise ValueError("invalid UART timing configuration")
        self.serial = serial_instance
        self.baudrate = baudrate
        self.timeout_s = timeout_s
        self.min_frame_interval_s = min_frame_interval_s
        self.tx_queue_diagnostics = tx_queue_diagnostics
        self.read_observer = read_observer
        self._read_seq = 0
        self._write_seq = 0
        self.last_read_observer_error: str | None = None
        self.rx_owner = rx_owner or J7RxOwner()
        self.uart_lock = threading.RLock()
        self._clock_ns = clock_ns or time.monotonic_ns
        self._sleep_fn = sleep_fn or time.sleep
        self._last_tx_end_ns: int | None = None
        self._request_audit: list[J7RequestAudit] = []
        self._last_drain_audit: RxDrainAudit | None = None
        self._last_write_metric: SerialWriteMetric | None = None
        self._last_rx_drain_metrics: RxDrainMetrics | None = None
        self._in_waiting_call_count: int = 0
        self._read_call_count: int = 0
        self._total_bytes_read: int = 0
        self._max_read_latency_ns: int = 0

    @property
    def last_write_metric(self) -> SerialWriteMetric | None:
        return self._last_write_metric

    @property
    def last_rx_drain_metrics(self) -> RxDrainMetrics | None:
        return self._last_rx_drain_metrics

    def _available(self) -> int:
        self._in_waiting_call_count += 1
        value = getattr(self.serial, "in_waiting", 0)
        try:
            return max(0, int(value))
        except (TypeError, ValueError):
            return 0

    def _read_chunk(self) -> bytes:
        size = self._available()
        if size <= 0:
            return b""
        if self.read_observer is None:
            t_r_start = time.perf_counter_ns()
            data = self.serial.read(size)
            t_r_dur = time.perf_counter_ns() - t_r_start
        else:
            # All counter pairs surround this one read, never adjacent cycle entries.
            # Sequential snapshots have small skew, retained as observational overhead.
            snapshot_enter_ns = time.monotonic_ns()
            thread_before = time.thread_time_ns()
            process_before = time.process_time_ns()
            ru_before = resource.getrusage(resource.RUSAGE_THREAD)
            data = None
            read_error = ""
            enter_ns = time.monotonic_ns()
            t_r_start = time.perf_counter_ns()
            try:
                data = self.serial.read(size)
            except Exception as exc:
                read_error = type(exc).__name__
                raise
            finally:
                t_r_dur = time.perf_counter_ns() - t_r_start
                exit_ns = time.monotonic_ns()
                ru_after = resource.getrusage(resource.RUSAGE_THREAD)
                process_after = time.process_time_ns()
                thread_after = time.thread_time_ns()
                snapshot_exit_ns = time.monotonic_ns()
                self._read_seq += 1
                returned = len(data) if isinstance(data, (bytes, bytearray)) else (0 if data is None else None)
                metric = SerialReadMetric(
                    self._read_seq, threading.get_native_id(), enter_ns, exit_ns,
                    exit_ns - enter_ns, t_r_dur,
                    max(0, snapshot_exit_ns - snapshot_enter_ns - (exit_ns - enter_ns)),
                    snapshot_enter_ns, snapshot_exit_ns,
                    thread_before, thread_after, thread_after - thread_before,
                    process_before, process_after, process_after - process_before,
                    ru_before.ru_nvcsw, ru_after.ru_nvcsw, ru_after.ru_nvcsw - ru_before.ru_nvcsw,
                    ru_before.ru_nivcsw, ru_after.ru_nivcsw, ru_after.ru_nivcsw - ru_before.ru_nivcsw,
                    size, size, returned, getattr(self.serial, "timeout", None), read_error,
                )
                try:
                    self.read_observer(metric)
                except Exception as exc:
                    # Observation failures must not replace data or the original read exception.
                    self.last_read_observer_error = type(exc).__name__
        self._read_call_count += 1
        if t_r_dur > self._max_read_latency_ns:
            self._max_read_latency_ns = t_r_dur
        if data is None:
            return b""
        if not isinstance(data, (bytes, bytearray)):
            raise J7AdapterRxError("serial.read() must return bytes-like data")
        b = bytes(data)
        self._total_bytes_read += len(b)
        return b

    def _ingest_available_locked(self) -> tuple[Frame, ...]:
        frames: list[Frame] = []
        while self._available() > 0:
            data = self._read_chunk()
            if not data:
                break
            frames.extend(self.rx_owner.ingest(data))
        return tuple(frames)

    def _wait_frame_interval_locked(self) -> None:
        if self._last_tx_end_ns is None:
            return
        remaining_ns = self._last_tx_end_ns + int(self.min_frame_interval_s * 1e9) - self._clock_ns()
        if remaining_ns > 0:
            if remaining_ns < 1_500_000 and self._clock_ns is time.monotonic_ns:
                target_ns = self._clock_ns() + remaining_ns
                while self._clock_ns() < target_ns:
                    pass
            else:
                self._sleep_fn(remaining_ns / 1e9)

    def write(self, frame: bytes) -> int | None:
        """Write one frame without reading or waiting for a response."""
        with self.uart_lock:
            self._wait_frame_interval_locked()
            queue_before = self._read_tx_queue_state() if self.tx_queue_diagnostics else None
            interval = {}
            if self.tx_queue_diagnostics:
                self._write_seq += 1
                write_tid = threading.get_native_id()
                sync_before = time.monotonic_ns()
                sync_epoch = time.time_ns()
                sync_after = time.monotonic_ns()
                snapshot_enter = time.perf_counter_ns()
                process_before = time.process_time_ns()
                usage_before = resource.getrusage(resource.RUSAGE_THREAD)
            t_w_cpu_start = time.thread_time_ns()
            t_w_wall_start = time.perf_counter_ns()
            written = None
            try:
                try:
                    written = self.serial.write(frame)
                finally:
                    # These counters surround this exact call on both success
                    # and exception. Never substitute adjacent servo entries.
                    t_w_wall_end = time.perf_counter_ns()
                    t_w_cpu_end = time.thread_time_ns()
                    if self.tx_queue_diagnostics:
                        process_after = time.process_time_ns()
                        usage_after = resource.getrusage(resource.RUSAGE_THREAD)
                        snapshot_exit = time.perf_counter_ns()
                        interval = dict(write_seq=self._write_seq, native_thread_id=write_tid,
                            process_cpu_duration_ns=process_after-process_before,
                            voluntary_ctx_delta=usage_after.ru_nvcsw-usage_before.ru_nvcsw,
                            involuntary_ctx_delta=usage_after.ru_nivcsw-usage_before.ru_nivcsw,
                            snapshot_enter_ns=snapshot_enter, snapshot_exit_ns=snapshot_exit,
                            snapshot_overhead_ns=(snapshot_exit-snapshot_enter)-(t_w_wall_end-t_w_wall_start),
                            clock_sync_monotonic_before_ns=sync_before,
                            clock_sync_epoch_ns=sync_epoch, clock_sync_monotonic_after_ns=sync_after)
                if written is not None and written != len(frame):
                    raise J7AdapterError(f"short UART write: {written}/{len(frame)} bytes")
            except Exception as exc:
                self._last_write_metric = SerialWriteMetric(
                    timestamp_start_ns=t_w_wall_start,
                    timestamp_end_ns=t_w_wall_end,
                    wall_duration_ns=t_w_wall_end - t_w_wall_start,
                    cpu_duration_ns=t_w_cpu_end - t_w_cpu_start,
                    requested_bytes=len(frame),
                    # PySerial does not expose the partial byte count on timeout.
                    written_bytes=-1 if isinstance(exc, SerialTimeoutException) else (0 if written is None else written),
                    error=type(exc).__name__,
                    configured_timeout_s=getattr(self.serial, "write_timeout", None),
                    queue_before=queue_before,
                    queue_after=self._read_tx_queue_state() if self.tx_queue_diagnostics else None,
                    **interval,
                )
                if isinstance(exc, SerialTimeoutException):
                    raise J7AdapterWriteTimeout(
                        self._last_write_metric, self.serial.write_timeout
                    ) from exc
                raise J7AdapterError(f"UART write failed: {exc}") from exc
            self._last_tx_end_ns = self._clock_ns()
            self._last_write_metric = SerialWriteMetric(
                timestamp_start_ns=t_w_wall_start,
                timestamp_end_ns=t_w_wall_end,
                wall_duration_ns=t_w_wall_end - t_w_wall_start,
                cpu_duration_ns=t_w_cpu_end - t_w_cpu_start,
                requested_bytes=len(frame),
                written_bytes=len(frame) if written is None else written,
                error=None,
                configured_timeout_s=getattr(self.serial, "write_timeout", None),
                queue_before=queue_before,
                queue_after=self._read_tx_queue_state() if self.tx_queue_diagnostics else None,
                **interval,
            )
            return written

    def _read_tx_queue_state(self) -> TxQueueState:
        """Only read PySerial TIOCOUTQ/TIOCINQ; no drain, flush, reset or retry."""
        start = time.perf_counter_ns()
        values = {}
        errors = []
        for name in ("out_waiting", "in_waiting"):
            try:
                values[name] = int(getattr(self.serial, name))
            except Exception as exc:
                values[name] = None
                errors.append(f"{name}:{type(exc).__name__}")
        return TxQueueState(
            values["out_waiting"], values["in_waiting"],
            time.perf_counter_ns() - start, ";".join(errors) or None,
        )

    def read_available(self) -> bytes:
        """Drain currently available bytes through the single RX owner.

        Complete canonical frames are returned for C5/C6 consumers; physical
        bytes never bypass ``J7RxOwner``.
        """
        with self.uart_lock:
            self._ingest_available_locked()
            frames = self.rx_owner.take_for_consumer()
            return b"".join(frame.raw for frame in frames)

    def drain_rx(self) -> RxDrainAudit:
        """Ingest and classify all currently available RX frames exactly once."""
        with self.uart_lock:
            start_ns = self._clock_ns()
            t_cpu_start = time.thread_time_ns()
            before = self.rx_owner.audit_snapshot()
            self._ingest_available_locked()
            frames = self.rx_owner.drain()
            after = self.rx_owner.audit_snapshot()
            end_ns = self._clock_ns()
            t_cpu_end = time.thread_time_ns()
            audit = RxDrainAudit(
                timestamp_start_ns=start_ns,
                timestamp_end_ns=end_ns,
                drained_frames=len(frames),
                drained_bytes=sum(len(frame.raw) for frame in frames),
                motion_write_responses=(
                    after.motion_write_response_count
                    - before.motion_write_response_count
                ),
                stale_responses=after.stale_response_count - before.stale_response_count,
                residual_frames_after=after.residual_frames,
                wall_duration_ns=end_ns - start_ns,
                cpu_duration_ns=t_cpu_end - t_cpu_start,
            )
            self._last_drain_audit = audit
            return audit

    def drain_motion_feedbacks(self) -> dict[int, StatusFeedback]:
        """Ingest all available RX bytes and return all unpacked 0x29 StatusFeedback objects."""
        with self.uart_lock:
            t_start = time.perf_counter_ns()
            t_cpu_start = time.thread_time_ns()
            self._in_waiting_call_count = 0
            self._read_call_count = 0
            self._total_bytes_read = 0
            self._max_read_latency_ns = 0
            self._ingest_available_locked()
            feedbacks = self.rx_owner.take_motion_write_feedbacks()
            t_end = time.perf_counter_ns()
            t_cpu_end = time.thread_time_ns()
            self._last_rx_drain_metrics = RxDrainMetrics(
                timestamp_start_ns=t_start,
                timestamp_end_ns=t_end,
                wall_duration_ns=t_end - t_start,
                cpu_duration_ns=t_cpu_end - t_cpu_start,
                in_waiting_calls=self._in_waiting_call_count,
                read_calls=self._read_call_count,
                bytes_read=self._total_bytes_read,
                max_read_latency_ns=self._max_read_latency_ns,
                feedbacks_extracted=len(feedbacks),
            )
            return feedbacks

    def _request_once_locked(
        self,
        frame: bytes,
        *,
        expected_id: int | None,
        expected_command: int | None,
        expected_register: int | None,
    ) -> tuple[bytes, J7RequestAudit]:
        # Classify any response left by a previous write-only transaction.
        self._ingest_available_locked()
        self.rx_owner.drain()
        parsed_command = frame[4] if len(frame) > 4 else -1
        parsed_register = int.from_bytes(frame[5:7], "little") if len(frame) >= 7 else -1
        parsed_id = frame[3] if len(frame) > 3 else -1
        self._wait_frame_interval_locked()
        start_ns = self._clock_ns()
        self.write(frame)
        deadline_ns = self._clock_ns() + int(self.timeout_s * 1e9)
        while self._clock_ns() <= deadline_ns:
            self._ingest_available_locked()
            matched = self.rx_owner.take_matching(
                expected_id=expected_id if expected_id is not None else parsed_id,
                expected_command=(
                    expected_command if expected_command is not None else parsed_command
                ),
                expected_register=expected_register,
            )
            if matched is not None:
                raw = matched.raw
                validate_response(
                    raw,
                    expected_id=expected_id,
                    expected_command=expected_command,
                )
                end_ns = self._clock_ns()
                audit = J7RequestAudit(
                    motor_id=parsed_id,
                    command=parsed_command,
                    register_address=parsed_register,
                    tx_start_ns=start_ns,
                    tx_end_ns=end_ns,
                    response_received=True,
                    timeout=False,
                    retry_count=0,
                )
                return raw, audit
            self._sleep_fn(self.min_frame_interval_s)
        audit = J7RequestAudit(
            motor_id=parsed_id,
            command=parsed_command,
            register_address=parsed_register,
            tx_start_ns=start_ns,
            tx_end_ns=self._clock_ns(),
            response_received=False,
            timeout=True,
            retry_count=0,
            error="UART response timeout",
        )
        raise J7AdapterTimeout("UART response timeout")

    def request(
        self,
        frame: bytes,
        *,
        expected_id: int | None = None,
        expected_command: int | None = None,
        expected_register: int | None = None,
        retries: int = 0,
    ) -> bytes:
        """Request one validated response; C5/C6 use ``retries=0``."""
        if retries < 0:
            raise ValueError("retries must be non-negative")
        with self.uart_lock:
            last_error: Exception | None = None
            for attempt in range(retries + 1):
                try:
                    raw, audit = self._request_once_locked(
                        frame,
                        expected_id=expected_id,
                        expected_command=expected_command,
                        expected_register=expected_register,
                    )
                    self._request_audit.append(audit)
                    return raw
                except Exception as exc:
                    last_error = exc
                    if attempt >= retries:
                        raise
            raise J7AdapterError(str(last_error))

    def audit_snapshot(self) -> tuple[J7RequestAudit, ...]:
        return tuple(self._request_audit)

    def wait_for_response(self, *, expected_id: int, expected_command: int,
                          timeout_s: float = 1.0) -> bytes:
        """Wait for an asynchronous frame through the existing RX owner; no TX/drain.

        In particular, preserve a completion already queued behind a write ACK.
        The caller must hold its operation lock across write and completion wait.
        """
        if timeout_s <= 0:
            raise ValueError('positive asynchronous response timeout required')
        with self.uart_lock:
            deadline = self._clock_ns() + int(timeout_s * 1e9)
            while self._clock_ns() <= deadline:
                self._ingest_available_locked()
                frame = self.rx_owner.take_matching(expected_id=expected_id,
                                                     expected_command=expected_command)
                if frame is not None:
                    validate_response(frame.raw, expected_id=expected_id,
                                      expected_command=expected_command)
                    return frame.raw
                self._sleep_fn(max(.0001, self.min_frame_interval_s))
        raise J7AdapterTimeout(f'ID{expected_id} asynchronous 0x{expected_command:02X} timeout')

    def request_recovery_status(self, motor_id, *, refresh_epoch_ns, hard_deadline_ns, observer):
        """One read-only 0x30 transaction in the paused recovery epoch.

        The ordinary request/read/drain policy is untouched. At the existing
        request deadline, inspect available RX once before declaring timeout;
        do not sleep again, extend the wait, or retransmit the query.
        """
        from .protocol import CMD_RD_STATUS, pack_read_status_cmd

        if motor_id not in MOTOR_IDS:
            raise ValueError('recovery status requires ID1..6')
        record = dict(motor_id=motor_id, refresh_epoch_ns=refresh_epoch_ns,
                      request_hex=pack_read_status_cmd(motor_id).hex(),
                      retry_count=0, reads=[], response_received=False)

        def emit(stage):
            record['stage'] = stage
            record['monotonic_ns'] = self._clock_ns()
            observer(record)

        def ingest():
            while self._available() > 0:
                data = self._read_chunk()
                rx_done = self._clock_ns()
                read_record = dict(rx_done_ns=rx_done, raw_hex=data.hex(), parsed_frames=[])
                record['reads'].append(read_record)
                frames = self.rx_owner.ingest(data)
                read_record['parsed_frames'] = [dict(motor_id=f.motor_id, command=f.command,
                                                    register=f.register_addr) for f in frames]
                if 'matched_rx_done_ns' not in record and any(
                    f.motor_id == motor_id and f.command == CMD_RD_STATUS and f.register_addr == 0
                    for f in frames
                ):
                    record['matched_rx_done_ns'] = rx_done
                record['last_rx_done_ns'] = rx_done
                if not data:
                    break

        with self.uart_lock:
            try:
                record['request_start_ns'] = self._clock_ns()
                record['serial_open'] = getattr(self.serial, 'is_open', None)
                emit('REQUEST_START')
                if record['request_start_ns'] <= refresh_epoch_ns:
                    raise J7AdapterError('recovery query predates pause completion')
                if record['request_start_ns'] >= hard_deadline_ns:
                    raise J7AdapterTimeout('existing 1000ms freshness boundary expired')
                # Drop completed pre-query frames; never use the feedback cache.
                self._ingest_available_locked()
                old = self.rx_owner.drain()
                record['discarded_pre_query_frames'] = [f.raw.hex() for f in old]
                self.rx_owner._parser.clear()  # Recovery only: discard pre-query partial bytes too.
                self._wait_frame_interval_locked()
                record['tx_start_ns'] = self._clock_ns()
                if record['tx_start_ns'] >= hard_deadline_ns:
                    raise J7AdapterTimeout('existing 1000ms freshness boundary expired before query')
                self.write(bytes.fromhex(record['request_hex']))
                record['tx_done_ns'] = self._clock_ns()
                record['serial_write_metric'] = asdict(self.last_write_metric)
                deadline = min(record['tx_done_ns'] + int(self.timeout_s * 1e9), hard_deadline_ns)
                record['response_wait_deadline_ns'] = deadline
                emit('QUERY_SENT')
                final_collection = False
                while True:
                    # Completion is checked before expiry, including one final
                    # collection after a descheduled sleep crosses the deadline.
                    ingest()
                    matched = self.rx_owner.take_matching(expected_id=motor_id,
                        expected_command=CMD_RD_STATUS, expected_register=0)
                    checked_ns = self._clock_ns()
                    if checked_ns >= hard_deadline_ns:
                        raise J7AdapterTimeout('existing 1000ms freshness boundary expired')
                    if matched is not None:
                        status = unpack_status_response(matched.raw)
                        rx_done = record['matched_rx_done_ns']
                        if rx_done <= refresh_epoch_ns or rx_done < record['tx_done_ns']:
                            raise J7AdapterError('stale recovery acquisition timestamp')
                        record.update(response_hex=matched.raw.hex(), response_received=True,
                            feedback_rx_done_ns=rx_done, status=asdict(status),
                            fresh_after_pause=True, final_collection=final_collection,
                            completed_after_wait_deadline=checked_ns > deadline,
                            rx_owner=asdict(self.rx_owner.audit_snapshot()))
                        emit('FRESH_RESPONSE_PARSED')
                        return status, rx_done, record
                    if checked_ns >= deadline:
                        raise J7AdapterTimeout('UART recovery status response timeout')
                    self._sleep_fn(self.min_frame_interval_s)
                    final_collection = self._clock_ns() >= deadline
            except Exception as exc:
                record.update(error=f'{type(exc).__name__}: {exc}',
                              rx_owner=asdict(self.rx_owner.audit_snapshot()))
                emit('QUERY_FAILED')
                raise

    def rx_audit_snapshot(self) -> RxOwnerAudit:
        with self.uart_lock:
            return self.rx_owner.audit_snapshot()


@dataclass(frozen=True)
class StopAxisRecord:
    motor_id: int
    tx_start_ns: int
    tx_end_ns: int
    response_received: bool
    timeout: bool
    error: str | None = None
    request_hex: str = ""
    response_hex: str = ""
    status: StatusFeedback | None = None


@dataclass(frozen=True)
class StopAudit:
    reason: str
    records: tuple[StopAxisRecord, ...]
    success: bool
    callback_count: int


class J7StopAdapter:
    """One-shot six-axis ``0x1A`` stop callback for C7/C8 runner wiring."""

    def __init__(self, uart: J7UartAdapter, audit_observer: Callable[[StopAudit], None] | None = None) -> None:
        self.uart = uart
        self.audit_observer = audit_observer
        self._called = False
        self._callback_count = 0
        self._last_audit: StopAudit | None = None

    @property
    def called(self) -> bool:
        return self._called

    def __call__(self, reason: str) -> StopAudit:
        return self.stop_all(reason)

    def stop_all(self, reason: str) -> StopAudit:
        if self._called and self._last_audit is not None:
            return self._last_audit
        self._called = True
        self._callback_count += 1
        records: list[StopAxisRecord] = []
        with self.uart.uart_lock:
            for motor_id in MOTOR_IDS:
                start_ns = self.uart._clock_ns()
                request = pack_write_register_cmd(motor_id, PAUSE_ADDR, [1])
                try:
                    response = self.uart.request(
                        request,
                        expected_id=motor_id,
                        expected_command=CMD_WR_REGISTER,
                        expected_register=PAUSE_ADDR,
                        retries=0,
                    )
                    records.append(
                        StopAxisRecord(
                            motor_id=motor_id,
                            tx_start_ns=start_ns,
                            tx_end_ns=self.uart._clock_ns(),
                            response_received=True,
                            timeout=False,
                            request_hex=request.hex(), response_hex=response.hex(),
                            status=unpack_status_response(response),
                        )
                    )
                except J7AdapterTimeout as exc:
                    records.append(
                        StopAxisRecord(
                            motor_id=motor_id,
                            tx_start_ns=start_ns,
                            tx_end_ns=self.uart._clock_ns(),
                            response_received=False,
                            timeout=True,
                            error=str(exc),
                            request_hex=request.hex(),
                        )
                    )
                except Exception as exc:
                    records.append(
                        StopAxisRecord(
                            motor_id=motor_id,
                            tx_start_ns=start_ns,
                            tx_end_ns=self.uart._clock_ns(),
                            response_received=False,
                            timeout=False,
                            error=str(exc),
                            request_hex=request.hex(),
                        )
                    )
        self._last_audit = StopAudit(
            reason=str(reason),
            records=tuple(records),
            success=all(record.response_received and record.status is not None
                        and record.status.error_code == 0 for record in records),
            callback_count=self._callback_count,
        )
        if self.audit_observer is not None:
            self.audit_observer(self._last_audit)
        return self._last_audit

    def arm_after_confirmed_recovery(self) -> None:
        """Start a new pause epoch only after an acknowledged successful recovery."""
        if self._last_audit is None or not self._last_audit.success:
            raise J7AdapterError("cannot re-arm an unconfirmed pause")
        self._called = False

    def audit_snapshot(self) -> StopAudit | None:
        return self._last_audit


__all__ = [
    "J7AdapterError",
    "J7AdapterRxError",
    "J7AdapterTimeout",
    "J7RxOwner",
    "J7SerialLike",
    "J7StopAdapter",
    "J7UartAdapter",
    "J7RequestAudit",
    "RxDrainAudit",
    "RxDrainMetrics",
    "RxOwnerAudit",
    "SerialWriteMetric",
    "StopAudit",
    "StopAxisRecord",
]
