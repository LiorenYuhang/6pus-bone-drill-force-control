"""Fixed-Lag State Synchronizer for 6PUS Parallel Manipulator.

Maintains per-axis monotonically increasing measurement timeline buffers,
and provides strictly bounded (zero-extrapolation) multi-axis configuration
reconstruction for arbitrary query timestamps (e.g. IMU sample stamps).
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from enum import Enum
import threading
import math
from typing import Sequence

try:
    from pus6_msgs.msg import TimedJointFeedback
except ImportError:
    TimedJointFeedback = None


class SyncStatus(Enum):
    SYNCED = "synced"                           # All 6 axes bracket query timestamp (0% extrapolation)
    WAIT_FUTURE_SAMPLE = "wait_future_sample"   # Query timestamp is newer than latest available axis sample
    WAIT_TOO_EARLY = "wait_too_early"           # Query timestamp is older than oldest available buffer sample
    EMPTY_BUFFER = "empty_buffer"               # Buffers contain fewer than 2 samples per axis


@dataclass
class SyncConfigurationResult:
    """Synchronized 6-axis configuration at query timestamp."""
    query_timestamp_ns: int
    positions_meters: list[float]
    raw_steps: list[float]
    status: SyncStatus
    per_axis_left_stamps_ns: list[int] = field(default_factory=list)
    per_axis_right_stamps_ns: list[int] = field(default_factory=list)
    per_axis_interp_weights: list[float] = field(default_factory=list)
    forces_g_raw: list[float] = field(default_factory=list)
    force_valid: list[bool] = field(default_factory=list)
    fault_codes: list[int] = field(default_factory=list)
    sources: list[int] = field(default_factory=list)
    per_axis_force_adc_left: list[int] = field(default_factory=list)
    per_axis_force_adc_right: list[int] = field(default_factory=list)


class FixedLagJointSynchronizer:
    """Thread-safe, strictly bounded 6-axis sliding timeline state synchronizer."""

    def __init__(self, max_history_size: int = 500) -> None:
        self._lock = threading.RLock()
        self.max_history_size = max_history_size
        # Per-axis samples: (host_ns, position_m, steps, force_g, valid, ADC, fault, source).
        # Position and force share one bracket and interpolation weight.
        self._buffers: list[deque[tuple]] = [
            deque(maxlen=max_history_size) for _ in range(6)
        ]
        self._total_pushed_frames: int = 0
        self._last_pushed_stamps_ns: list[int] = [0] * 6

    def push_feedback(self, msg: TimedJointFeedback) -> None:
        """Push a TimedJointFeedback message into per-axis history buffers.

        Args:
            msg: TimedJointFeedback containing per_axis_rx_done_monotonic_ns, positions_meters, raw_steps.
        """
        rx_stamps = list(msg.per_axis_rx_done_monotonic_ns)
        positions = list(msg.positions_meters)
        raw_steps = list(msg.raw_steps)

        self.push_raw_snapshot(
            rx_stamps, positions, raw_steps,
            forces_g_raw=getattr(msg, 'forces_g_raw', None),
            force_valid=getattr(msg, 'force_valid', None),
            force_adc_raw=getattr(msg, 'force_adc_raw', None),
            fault_codes=getattr(msg, 'fault_codes', None),
            source=getattr(msg, 'source', 0),
        )

    def push_raw_snapshot(
        self,
        per_axis_timestamps_ns: Sequence[int],
        positions_meters: Sequence[float],
        raw_steps: Sequence[float],
        *, forces_g_raw=None, force_valid=None, force_adc_raw=None, fault_codes=None, source=0,
    ) -> None:
        """Push raw 6-axis snapshot directly into per-axis history buffers.

        Args:
            per_axis_timestamps_ns: 6 timestamps in nanoseconds.
            positions_meters: 6 actuator positions in SI meters.
            raw_steps: 6 raw step values.
        """
        if len(per_axis_timestamps_ns) != 6 or len(positions_meters) != 6 or len(raw_steps) != 6:
            raise ValueError("All inputs must have length 6")

        force = list(forces_g_raw) if forces_g_raw is not None else [0.0] * 6
        valid = list(force_valid) if force_valid is not None else [False] * 6
        adc = list(force_adc_raw) if force_adc_raw is not None else [0] * 6
        faults = list(fault_codes) if fault_codes is not None else [0] * 6
        if any(len(x) != 6 for x in (force, valid, adc, faults)):
            raise ValueError("Force metadata must have length 6")
        if not all(math.isfinite(float(x)) for x in positions_meters):
            raise ValueError("positions must be finite")

        with self._lock:
            for i in range(6):
                t_ns = int(per_axis_timestamps_ns[i])
                pos_m = float(positions_meters[i])
                step_val = float(raw_steps[i])

                # Monotonicity check
                if len(self._buffers[i]) > 0:
                    last_t = self._buffers[i][-1][0]
                    if t_ns <= last_t:
                        # Ignore duplicate or non-monotonic sample
                        continue

                f = float(force[i])
                ok = (bool(valid[i]) and source in (1, 2) and faults[i] == 0
                      and math.isfinite(f) and -32768 <= f <= 32767 and 0 <= adc[i] <= 4095)
                self._buffers[i].append((t_ns, pos_m, step_val, f, ok, adc[i], faults[i], source))
                self._last_pushed_stamps_ns[i] = t_ns

            self._total_pushed_frames += 1

    def query_sync_configuration(self, query_timestamp_ns: int) -> SyncConfigurationResult:
        """Query 6-axis synchronized configuration at exact monotonic timestamp.

        Enforces strict bracketing rule:
            t_i[k-1] <= t_query <= t_i[k]  for ALL i in {1..6}
        If any axis is not bracketed, state is NOT published (returns non-SYNCED status).

        Args:
            query_timestamp_ns: Target host CLOCK_MONOTONIC query timestamp in nanoseconds.

        Returns:
            SyncConfigurationResult with status and interpolated configuration.
        """
        with self._lock:
            # Check minimum buffer depth (at least 2 samples per axis)
            for i in range(6):
                if len(self._buffers[i]) < 2:
                    return SyncConfigurationResult(
                        query_timestamp_ns=query_timestamp_ns,
                        positions_meters=[],
                        raw_steps=[],
                        status=SyncStatus.EMPTY_BUFFER,
                    )

            # Check temporal bounds across all 6 axes
            for i in range(6):
                t_oldest = self._buffers[i][0][0]
                t_newest = self._buffers[i][-1][0]
                if query_timestamp_ns < t_oldest:
                    return SyncConfigurationResult(
                        query_timestamp_ns=query_timestamp_ns,
                        positions_meters=[],
                        raw_steps=[],
                        status=SyncStatus.WAIT_TOO_EARLY,
                    )
                if query_timestamp_ns > t_newest:
                    return SyncConfigurationResult(
                        query_timestamp_ns=query_timestamp_ns,
                        positions_meters=[],
                        raw_steps=[],
                        status=SyncStatus.WAIT_FUTURE_SAMPLE,
                    )

            interp_positions: list[float] = []
            interp_steps: list[float] = []
            left_stamps: list[int] = []
            right_stamps: list[int] = []
            weights: list[float] = []
            forces, valid, faults, sources, adc_left, adc_right = [], [], [], [], [], []

            # Perform linear interpolation for each axis
            for i in range(6):
                buf = self._buffers[i]
                # Binary search for right bracket
                # buf entries are sorted by timestamp
                low = 0
                high = len(buf) - 1
                right_idx = -1

                while low <= high:
                    mid = (low + high) // 2
                    if buf[mid][0] >= query_timestamp_ns:
                        right_idx = mid
                        high = mid - 1
                    else:
                        low = mid + 1

                if right_idx <= 0:
                    # Exactly at the oldest sample
                    t_right, p_right, s_right = buf[0][:3]
                    interp_positions.append(p_right)
                    interp_steps.append(s_right)
                    left_stamps.append(t_right)
                    right_stamps.append(t_right)
                    weights.append(0.0)
                else:
                    left_idx = right_idx - 1
                    t_left, p_left, s_left = buf[left_idx][:3]
                    t_right, p_right, s_right = buf[right_idx][:3]

                    dt = t_right - t_left
                    if dt <= 0:
                        alpha = 0.0
                    else:
                        alpha = (query_timestamp_ns - t_left) / float(dt)

                    # Strict clamp to [0.0, 1.0]
                    alpha = max(0.0, min(1.0, alpha))
                    p_interp = p_left + alpha * (p_right - p_left)
                    s_interp = s_left + alpha * (s_right - s_left)

                    interp_positions.append(p_interp)
                    interp_steps.append(s_interp)
                    left_stamps.append(t_left)
                    right_stamps.append(t_right)
                    weights.append(alpha)

                # Use exactly the same endpoints/alpha as position; no latest-force fallback.
                right = buf[max(0, right_idx)]
                left = buf[max(0, right_idx - 1)]
                alpha = weights[-1]
                if left_stamps[-1] == right_stamps[-1] or alpha == 1.0:
                    left = right
                forces.append(left[3] + alpha * (right[3] - left[3]))
                valid.append(left[4] and right[4] and left[7] == right[7])
                faults.append(int(left[6]) | int(right[6]))
                sources.append(right[7] if left[7] == right[7] else 0)
                adc_left.append(left[5])
                adc_right.append(right[5])

            return SyncConfigurationResult(
                query_timestamp_ns=query_timestamp_ns,
                positions_meters=interp_positions,
                raw_steps=interp_steps,
                status=SyncStatus.SYNCED,
                per_axis_left_stamps_ns=left_stamps,
                per_axis_right_stamps_ns=right_stamps,
                per_axis_interp_weights=weights,
                forces_g_raw=forces, force_valid=valid, fault_codes=faults, sources=sources,
                per_axis_force_adc_left=adc_left, per_axis_force_adc_right=adc_right,
            )

    def clear(self) -> None:
        """Clear all per-axis history buffers."""
        with self._lock:
            for buf in self._buffers:
                buf.clear()
            self._total_pushed_frames = 0
            self._last_pushed_stamps_ns = [0] * 6

    @property
    def buffer_lengths(self) -> list[int]:
        """Current number of elements in each axis buffer."""
        with self._lock:
            return [len(buf) for buf in self._buffers]

    def bracketed_axes(self, query_timestamp_ns: int) -> list[bool]:
        """Return per-axis strict bracket availability for diagnostics.

        This is a read-only diagnostic view. It does not alter the frozen
        interpolation or zero-extrapolation decision in
        :meth:`query_sync_configuration`.
        """
        with self._lock:
            return [
                len(buf) >= 2
                and buf[0][0] <= query_timestamp_ns <= buf[-1][0]
                for buf in self._buffers
            ]

    @property
    def time_bounds(self) -> tuple[int, int] | None:
        """Oldest and newest monotonic timestamps across all 6 axes (ns)."""
        with self._lock:
            for buf in self._buffers:
                if len(buf) < 1:
                    return None
            t_min = max(self._buffers[i][0][0] for i in range(6))
            t_max = min(self._buffers[i][-1][0] for i in range(6))
            return (t_min, t_max)
