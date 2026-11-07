from __future__ import annotations

import contextlib
import logging
import time
from dataclasses import dataclass, field
from typing import Mapping, Sequence

try:
    from .motor import Motor
    from .protocol import (
        ReadRegisterResponse,
        StatusFeedback,
        unpack_read_register_response,
        unpack_status_response,
        unpack_write_response,
    )
    from .safety import (
        CONTROL_MODE_ADDR,
        DEFAULT_SPEED,
        DriverError,
        HardwareMode,
        LOWER_LIMIT_ADDR,
        ACTUAL_POSITION_ADDR,
        MAX_CURRENT_MA,
        MOTOR_IDS,
        NUM_MOTORS,
        PAUSE_ADDR,
        ReadOnlyModeError,
        SOFTWARE_LOWER_LIMIT,
        SOFTWARE_UPPER_LIMIT,
        SPEED_MODE,
        STATIC_POSITION_TOLERANCE_STEPS,
        STEPS_PER_MM,
        STEPS_PER_METER,
        SafetyViolation,
        TARGET_POSITION_ADDR,
        TARGET_SPEED_ADDR,
        UPPER_LIMIT_ADDR,
        meters_to_steps,
        parse_hardware_mode,
        steps_to_meters,
        uint16_to_int16,
        steps_to_mm,
        validate_status_feedback,
        validate_target_vector,
    )
    from .uart_bus import UartBus
except ImportError:
    from motor import Motor
    from protocol import (
        ReadRegisterResponse,
        StatusFeedback,
        unpack_read_register_response,
        unpack_status_response,
        unpack_write_response,
    )
    from safety import (
        CONTROL_MODE_ADDR,
        DEFAULT_SPEED,
        DriverError,
        HardwareMode,
        LOWER_LIMIT_ADDR,
        ACTUAL_POSITION_ADDR,
        MAX_CURRENT_MA,
        MOTOR_IDS,
        NUM_MOTORS,
        PAUSE_ADDR,
        ReadOnlyModeError,
        SOFTWARE_LOWER_LIMIT,
        SOFTWARE_UPPER_LIMIT,
        SPEED_MODE,
        STATIC_POSITION_TOLERANCE_STEPS,
        STEPS_PER_MM,
        STEPS_PER_METER,
        SafetyViolation,
        TARGET_POSITION_ADDR,
        TARGET_SPEED_ADDR,
        UPPER_LIMIT_ADDR,
        meters_to_steps,
        parse_hardware_mode,
        steps_to_meters,
        uint16_to_int16,
        steps_to_mm,
        validate_status_feedback,
        validate_target_vector,
    )
    from uart_bus import UartBus

logger = logging.getLogger("pus6_motor_driver.six_motor_controller")


@dataclass
class SixAxisStateSnapshot:
    """Ordered sequential telemetry snapshot for ID1..ID6 on a shared single-bus UART.

    Note on Synchronization:
        Because all 6 motors share a single half-duplex UART bus (921600 8N1),
        status acquisition is executed sequentially in order ID1 -> ID2 -> ... -> ID6.
        This is an ordered 6-axis polling cycle (~15-20ms total duration), NOT a simultaneous
        hardware-latched sample.
    """
    timestamp_start: float
    timestamp_end: float
    per_axis_timestamps: dict[int, float]
    states: list[StatusFeedback]
    positions_steps: list[int]
    positions_meters: list[float]
    currents_mA: list[int]
    forces_g_raw: list[int]
    temperatures_C: list[int]
    fault_codes: list[int]
    forces_adc: list[int] = field(default_factory=list)
    timestamp_start_ns: int = 0
    timestamp_end_ns: int = 0
    per_axis_start_timestamps_ns: dict[int, int] = field(default_factory=dict)
    per_axis_end_timestamps_ns: dict[int, int] = field(default_factory=dict)
    per_axis_timestamps_ns: dict[int, int] = field(default_factory=dict)


@dataclass
class SixAxisPositionsSnapshot:
    """Ordered sequential position-only snapshot for ID1..ID6 using CMD 0x31 (Reg 0x2A)."""
    timestamp_start: float
    timestamp_end: float
    timestamp_start_ns: int
    timestamp_end_ns: int
    per_axis_timestamps: dict[int, float]
    per_axis_timestamps_ns: dict[int, int]
    positions_steps: list[int]
    positions_meters: list[float]
    per_axis_start_timestamps_ns: dict[int, int] = field(default_factory=dict)
    per_axis_end_timestamps_ns: dict[int, int] = field(default_factory=dict)


@dataclass
class SixAxisCommandResult:
    """Telemetry and timing record for a 6-target sequential dispatch cycle."""
    timestamp_start: float
    timestamp_end: float
    per_axis_write_timestamps: dict[int, float]
    commanded_targets: list[int]
    responses: list[StatusFeedback]
    per_axis_write_completion_timestamps: dict[int, float] = field(default_factory=dict)
    per_axis_write_timestamps_ns: dict[int, int] = field(default_factory=dict)
    per_axis_write_completion_timestamps_ns: dict[int, int] = field(default_factory=dict)

    def to_state_snapshot(self) -> SixAxisStateSnapshot:
        """Convert 6-axis write responses directly into a SixAxisStateSnapshot (0 additional UART reads)."""
        positions_steps = [r.current_position for r in self.responses]
        positions_meters = [steps_to_meters(p) for p in positions_steps]
        currents_mA = [r.current_mA for r in self.responses]
        forces_g_raw = [r.force_g for r in self.responses]
        forces_adc = [getattr(r, 'force_adc', 2048) for r in self.responses]
        temperatures_C = [r.temperature for r in self.responses]
        fault_codes = [r.error_code for r in self.responses]
        return SixAxisStateSnapshot(
            timestamp_start=self.timestamp_start,
            timestamp_end=self.timestamp_end,
            per_axis_timestamps=dict(self.per_axis_write_timestamps),
            states=list(self.responses),
            positions_steps=positions_steps,
            positions_meters=positions_meters,
            currents_mA=currents_mA,
            forces_g_raw=forces_g_raw,
            temperatures_C=temperatures_C,
            fault_codes=fault_codes,
            forces_adc=forces_adc,
            timestamp_start_ns=self.per_axis_write_timestamps_ns.get(1, 0),
            timestamp_end_ns=self.per_axis_write_completion_timestamps_ns.get(6, 0),
            per_axis_start_timestamps_ns=dict(self.per_axis_write_timestamps_ns),
            per_axis_end_timestamps_ns=dict(self.per_axis_write_completion_timestamps_ns),
            per_axis_timestamps_ns=dict(self.per_axis_write_completion_timestamps_ns),
        )


@dataclass
class SingleAxisCommandResult:
    """Telemetry and timing record for an isolated single-axis bring-up move."""
    motor_id: int
    commanded_target: int
    timestamp_write: float
    response: StatusFeedback
    observer_states: dict[int, StatusFeedback]


def _unwrap_register_value(resp: ReadRegisterResponse | int | Sequence[int]) -> int:
    if isinstance(resp, ReadRegisterResponse):
        return resp.values[0] if resp.values else 0
    if isinstance(resp, Sequence) and not isinstance(resp, (bytes, bytearray)):
        return resp[0] if resp else 0
    return int(resp)


class SixMotorController:
    """Unified controller for 6 Inspire electric cylinders (ID1..ID6) on a shared UART bus."""

    def __init__(
        self,
        port: str = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",
        baudrate: int = 921600,
        timeout: float = 0.05,
        retries: int = 2,
        bus: UartBus | None = None,
        hardware_mode: HardwareMode | str = HardwareMode.MOCK,
        is_mock: bool | None = None,
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.retries = retries
        self._bus: UartBus | None = bus
        self._motors: dict[int, Motor] = {}
        self._limits: dict[int, tuple[int, int]] = {}
        self._connected: bool = False
        self._last_snapshot: SixAxisStateSnapshot | None = None
        self._write_count: int = 0
        self._last_pause_results: dict[int, bool] = {}

        # Handle hardware mode with backward compatibility for is_mock
        if is_mock is not None:
            self._hardware_mode = HardwareMode.MOCK if is_mock else HardwareMode.ACTIVE
        else:
            self._hardware_mode = parse_hardware_mode(hardware_mode)

    @property
    def hardware_mode(self) -> HardwareMode:
        return self._hardware_mode

    @property
    def write_count(self) -> int:
        """Total count of UART register write commands issued by this controller instance."""
        return self._write_count

    @property
    def bus(self):
        """Underlying UART or simulated bus."""
        return self._bus

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def limits(self) -> dict[int, tuple[int, int]]:
        return dict(self._limits)

    @property
    def last_snapshot(self) -> SixAxisStateSnapshot | None:
        return self._last_snapshot

    @property
    def _bus_lock_ctx(self):
        lock = getattr(self._bus, "lock", None)
        return lock if lock is not None else contextlib.nullcontext()

    def connect(self) -> None:
        """Connect to the UART bus and initialize motor handles for ID1..ID6."""
        if self._connected:
            return

        if self._bus is None:
            if self._hardware_mode == HardwareMode.MOCK:
                try:
                    from .fake_bus import FakeBus
                except ImportError:
                    from fake_bus import FakeBus
                self._bus = FakeBus()
            else:
                self._bus = UartBus(
                    self.port,
                    baudrate=self.baudrate,
                    timeout=self.timeout,
                    retries=self.retries,
                    read_only=(self._hardware_mode == HardwareMode.READ_ONLY),
                )

        self._motors = {mid: Motor(self._bus, mid) for mid in MOTOR_IDS}
        self._connected = True
        logger.info(
            "Connected to 6PUS motors ID1..ID6 on %s (mode=%s)",
            self.port,
            self._hardware_mode.value,
        )

    def read_limits(self) -> dict[int, tuple[int, int]]:
        """Read hardware lower and upper limits (0x24, 0x23) for all 6 motors sequentially."""
        self._ensure_connected()
        limits: dict[int, tuple[int, int]] = {}
        for mid in MOTOR_IDS:
            motor = self._motors[mid]
            lower = _unwrap_register_value(motor.read_register(LOWER_LIMIT_ADDR))
            upper = _unwrap_register_value(motor.read_register(UPPER_LIMIT_ADDR))
            limits[mid] = (lower, upper)
        self._limits = limits
        return dict(limits)

    def read_single_axis_preflight(self, motor_id: int) -> dict[str, int]:
        """Read J2-B target-axis configuration and live status without writes."""
        self._ensure_connected()
        if motor_id not in MOTOR_IDS:
            raise TargetValidationError(f"Invalid motor_id {motor_id}. Must be in {MOTOR_IDS}")
        if not self._limits:
            self.read_limits()

        motor = self._motors[motor_id]
        control_mode = _unwrap_register_value(motor.read_register(CONTROL_MODE_ADDR))
        target_speed = _unwrap_register_value(motor.read_register(TARGET_SPEED_ADDR))
        target_position = _unwrap_register_value(motor.read_register(TARGET_POSITION_ADDR))
        status = motor.read_status()
        validate_status_feedback(status, motor_id, self._limits.get(motor_id))
        return {
            "motor_id": motor_id,
            "control_mode": control_mode,
            "target_speed": target_speed,
            "target_position": target_position,
            "actual_position": status.current_position,
            "current_mA": status.current_mA,
            "fault": status.error_code,
        }

    def read_positions(self) -> list[int]:
        """Read current actual positions for ID1..ID6 sequentially in step units."""
        states = self.read_states()
        return [s.current_position for s in states]

    def read_positions_meters(self) -> list[float]:
        """Read current actual positions for ID1..ID6 sequentially in SI meters."""
        steps = self.read_positions()
        return [steps_to_meters(s) for s in steps]

    def read_state_snapshot(self) -> SixAxisStateSnapshot:
        """Perform an ordered sequential telemetry sampling cycle across ID1..ID6.

        Returns:
            SixAxisStateSnapshot containing overall cycle start/end timestamps,
            per-axis sampling start and end timestamps, raw step feedback, SI converted positions,
            currents, raw force, temperature, and hardware fault codes.
        """
        self._ensure_connected()
        t_start = time.time()
        t_start_ns = time.monotonic_ns()
        per_axis_ts: dict[int, float] = {}
        per_axis_start_ts_ns: dict[int, int] = {}
        per_axis_end_ts_ns: dict[int, int] = {}
        states: list[StatusFeedback] = []

        with self._bus_lock_ctx:
            for mid in MOTOR_IDS:
                motor = self._motors[mid]
                t_axis_start = time.time()
                t_axis_start_ns = time.monotonic_ns()
                status = motor.read_status()
                t_axis_end_ns = time.monotonic_ns()

                per_axis_ts[mid] = t_axis_start
                per_axis_start_ts_ns[mid] = t_axis_start_ns
                per_axis_end_ts_ns[mid] = t_axis_end_ns
                limit = self._limits.get(mid)
                validate_status_feedback(status, mid, limit)
                states.append(status)

        t_end = time.time()
        t_end_ns = time.monotonic_ns()
        pos_steps = [s.current_position for s in states]
        pos_meters = [steps_to_meters(p) for p in pos_steps]
        currents = [s.current_mA for s in states]
        forces = [s.force_g for s in states]
        forces_adc = [getattr(s, 'force_adc', 2048) for s in states]
        temps = [s.temperature for s in states]
        faults = [s.error_code for s in states]

        snapshot = SixAxisStateSnapshot(
            timestamp_start=t_start,
            timestamp_end=t_end,
            timestamp_start_ns=t_start_ns,
            timestamp_end_ns=t_end_ns,
            per_axis_timestamps=per_axis_ts,
            per_axis_start_timestamps_ns=per_axis_start_ts_ns,
            per_axis_end_timestamps_ns=per_axis_end_ts_ns,
            per_axis_timestamps_ns=per_axis_end_ts_ns,
            states=states,
            positions_steps=pos_steps,
            positions_meters=pos_meters,
            currents_mA=currents,
            forces_g_raw=forces,
            temperatures_C=temps,
            fault_codes=faults,
            forces_adc=forces_adc,
        )
        self._last_snapshot = snapshot
        return snapshot

    def read_positions_snapshot(self) -> SixAxisPositionsSnapshot:
        """Perform an ordered sequential position-only sampling cycle across ID1..ID6 via CMD 0x31 Reg 0x2A."""
        self._ensure_connected()
        t_start = time.time()
        t_start_ns = time.perf_counter_ns()
        per_axis_ts: dict[int, float] = {}
        per_axis_start_ts_ns: dict[int, int] = {}
        per_axis_end_ts_ns: dict[int, int] = {}
        positions_steps: list[int] = []

        with self._bus_lock_ctx:
            for mid in MOTOR_IDS:
                motor = self._motors[mid]
                t_axis_start = time.time()
                t_axis_start_ns = time.perf_counter_ns()
                resp = motor.read_register(ACTUAL_POSITION_ADDR, count=1)
                t_axis_end_ns = time.perf_counter_ns()
                # Reg 0x2A carries signed int16 step feedback in a uint16 transport field.
                # Other register reads remain unsigned configuration/limit values.
                pos = uint16_to_int16(_unwrap_register_value(resp))

                per_axis_ts[mid] = t_axis_start
                per_axis_start_ts_ns[mid] = t_axis_start_ns
                per_axis_end_ts_ns[mid] = t_axis_end_ns
                positions_steps.append(pos)

        t_end = time.time()
        t_end_ns = time.perf_counter_ns()
        positions_meters = [steps_to_meters(p) for p in positions_steps]

        return SixAxisPositionsSnapshot(
            timestamp_start=t_start,
            timestamp_end=t_end,
            timestamp_start_ns=t_start_ns,
            timestamp_end_ns=t_end_ns,
            per_axis_timestamps=per_axis_ts,
            per_axis_start_timestamps_ns=per_axis_start_ts_ns,
            per_axis_end_timestamps_ns=per_axis_end_ts_ns,
            per_axis_timestamps_ns=per_axis_end_ts_ns,
            positions_steps=positions_steps,
            positions_meters=positions_meters,
        )

    def read_states(self) -> list[StatusFeedback]:
        """Read full status feedback for ID1..ID6 in sequential order."""
        snapshot = self.read_state_snapshot()
        return snapshot.states

    def configure_all(
        self,
        mode: int = SPEED_MODE,
        speed: int = DEFAULT_SPEED,
    ) -> None:
        """Configure control mode and speed for all 6 motors via 0x25.

        The configuration block intentionally stops at register 0x28:
        [0x25 mode, 0x26 voltage, 0x27 force, 0x28 speed].
        Register 0x29 is the absolute target-position register and is written
        only by ``command_positions``.
        """
        self._ensure_connected()
        if self._hardware_mode == HardwareMode.READ_ONLY:
            raise ReadOnlyModeError("configure_all is strictly forbidden in READ_ONLY mode (0 writes enforced).")

        for mid in MOTOR_IDS:
            motor = self._motors[mid]
            payload = [mode, 0, 0, speed]
            motor.write_register(CONTROL_MODE_ADDR, payload)
            self._write_count += 1

            # Verify mode & speed
            read_mode = _unwrap_register_value(motor.read_register(CONTROL_MODE_ADDR))
            read_speed = _unwrap_register_value(motor.read_register(TARGET_SPEED_ADDR))
            if read_mode != mode:
                raise DriverError(
                    f"ID{mid} mode write verification mismatch: expected {mode}, got {read_mode}"
                )
            if read_speed != speed:
                raise DriverError(
                    f"ID{mid} speed write verification mismatch: expected {speed}, got {read_speed}"
                )

        logger.info("Configured all 6 motors: mode=%d, speed=%d", mode, speed)

    def command_positions(self, q_targets: Sequence[int]) -> list[StatusFeedback]:
        """Execute full 6-target pre-validation followed by sequential UART target dispatch.

        Execution Protocol:
            1. Pre-validation Phase: Strictly validate all 6 target values against integer types,
               global bounds, and per-motor hardware limits. If any target is invalid, abort
               immediately with zero UART writes.
            2. Sequential Dispatch Phase: Issue 0x29 write sequentially to ID1 -> ID2 -> ... -> ID6.
               If any write raises an error, immediately issue emergency pause_all() to all motors.

        Args:
            q_targets: Exactly 6 integer target positions in steps for ID1..ID6.

        Returns:
            List of 6 StatusFeedback frames resulting from target writes.

        Raises:
            ReadOnlyModeError: If in READ_ONLY mode.
            TargetValidationError: If target vector is invalid (zero writes performed).
            SafetyViolation: If a runtime safety constraint is violated.
        """
        result = self.command_positions_with_metrics(q_targets)
        return result.responses

    def command_positions_with_metrics(self, q_targets: Sequence[int]) -> SixAxisCommandResult:
        """Execute pre-validation + sequential UART dispatch, returning detailed write timestamps."""
        # Step 1: Mode check
        if self._hardware_mode == HardwareMode.READ_ONLY:
            raise ReadOnlyModeError("command_positions is strictly forbidden in READ_ONLY mode (0 writes enforced).")

        # q_targets is the trajectory command vector, never feedback actual_position.
        # Step 2: Pre-validate vector length and data types immediately (zero bus activity)
        validate_target_vector(q_targets)

        self._ensure_connected()
        if not self._limits:
            self.read_limits()

        # Step 3: Validate against per-motor limits
        validated_targets = validate_target_vector(q_targets, self._limits)

        # Step 4: Sequentially write 0x29 target position
        t_start = time.time()
        per_axis_write_ts: dict[int, float] = {}
        per_axis_write_completion_ts: dict[int, float] = {}
        per_axis_write_ts_ns: dict[int, int] = {}
        per_axis_write_completion_ts_ns: dict[int, int] = {}
        responses: list[StatusFeedback] = []
        try:
            with self._bus_lock_ctx:
                for idx, mid in enumerate(MOTOR_IDS):
                    target = validated_targets[idx]
                    motor = self._motors[mid]
                    t_write = time.time()
                    t_write_ns = time.perf_counter_ns()
                    per_axis_write_ts[mid] = t_write
                    per_axis_write_ts_ns[mid] = t_write_ns
                    resp = motor.write_register(TARGET_POSITION_ADDR, [target])
                    self._write_count += 1
                    if isinstance(resp, StatusFeedback):
                        validate_status_feedback(resp, mid, self._limits.get(mid))
                        responses.append(resp)
                    else:
                        status = motor.read_status()
                        validate_status_feedback(status, mid, self._limits.get(mid))
                        responses.append(status)
                    per_axis_write_completion_ts[mid] = time.time()
                    per_axis_write_completion_ts_ns[mid] = time.perf_counter_ns()

            t_end = time.time()
            return SixAxisCommandResult(
                timestamp_start=t_start,
                timestamp_end=t_end,
                per_axis_write_timestamps=per_axis_write_ts,
                commanded_targets=validated_targets,
                responses=responses,
                per_axis_write_completion_timestamps=per_axis_write_completion_ts,
                per_axis_write_timestamps_ns=per_axis_write_ts_ns,
                per_axis_write_completion_timestamps_ns=per_axis_write_completion_ts_ns,
            )
        except Exception as exc:
            logger.error(
                "Exception during command_positions sequential write: %s. Executing emergency pause_all.",
                exc,
            )
            self.pause_all()
            raise

    def command_single_axis_for_bringup(
        self,
        motor_id: int,
        target_step: int,
    ) -> StatusFeedback:
        """Execute isolated single-axis bring-up movement with strict observer zero-write guarantee.

        Safety Rules for J2-B:
            - Exactly 1 motor (motor_id) receives 0x29 write;
            - The other 5 observer motors strictly receive 0 writes;
            - All 5 observer motors are health-checked in read-only mode before write;
            - Target step is validated against motor_id's physical limits and [0, 1800];
            - If write fails, emergency pause is sent ONLY to motor_id.

        Note:
            This method is dedicated to single-axis bring-up (Stage J2-B) and is NOT the
            general 6-axis control interface.
        """
        result = self.command_single_axis_with_metrics(motor_id, target_step)
        return result.response

    def configure_single_axis_for_bringup(
        self,
        motor_id: int,
        baseline_position: int,
        mode: int = SPEED_MODE,
        speed: int = DEFAULT_SPEED,
        baseline_band: tuple[int, int] | None = None,
        target_band: tuple[int, int] | None = None,
    ) -> dict[str, int]:
        """Apply and verify the validated M6 single-axis setup block.

        The setup is one CMD_WR_REGISTER frame starting at 0x25 with five
        contiguous values for 0x25..0x29. The target value is deliberately
        the live baseline, so this step does not introduce a new position
        target before the separate motion write.

        J2-B's real-hardware runner currently keeps this operation behind a
        Mock-only gate; active-hardware approval is a separate step.
        """
        if self._hardware_mode == HardwareMode.READ_ONLY:
            raise ReadOnlyModeError(
                "single-axis configuration is forbidden in READ_ONLY mode (0 writes enforced)."
            )
        if motor_id not in MOTOR_IDS:
            raise TargetValidationError(f"Invalid motor_id {motor_id}. Must be in {MOTOR_IDS}")
        if mode != SPEED_MODE or speed != DEFAULT_SPEED:
            raise TargetValidationError(
                f"J2-B setup is fixed to mode={SPEED_MODE}, speed={DEFAULT_SPEED}"
            )
        self._ensure_connected()
        if not self._limits:
            self.read_limits()
        lower, upper = self._limits[motor_id]
        safe_lower = max(SOFTWARE_LOWER_LIMIT, lower)
        safe_upper = min(SOFTWARE_UPPER_LIMIT, upper)
        if not isinstance(baseline_position, int) or isinstance(baseline_position, bool):
            raise TargetValidationError("baseline_position must be an integer")
        if not safe_lower <= baseline_position <= safe_upper:
            raise TargetValidationError(
                f"baseline position {baseline_position} outside [{safe_lower}, {safe_upper}]"
            )

        motor = self._motors[motor_id]
        before = motor.read_status()
        validate_status_feedback(before, motor_id, self._limits.get(motor_id))
        for band_name, band in (("baseline_band", baseline_band), ("target_band", target_band)):
            if band is not None and (
                not isinstance(band, tuple)
                or len(band) != 2
                or any(not isinstance(value, int) or isinstance(value, bool) for value in band)
                or band[0] > band[1]
                or band[1] - band[0] > 2 * STATIC_POSITION_TOLERANCE_STEPS
            ):
                raise TargetValidationError(
                    f"{band_name} must be an ordered integer band spanning at most "
                    f"{2 * STATIC_POSITION_TOLERANCE_STEPS} steps"
                )
        if baseline_band is None:
            baseline_position_ok = before.current_position == baseline_position
        else:
            baseline_position_ok = baseline_band[0] <= before.current_position <= baseline_band[1]
        if not baseline_position_ok:
            expected = baseline_position if baseline_band is None else list(baseline_band)
            raise SafetyViolation(
                f"ID{motor_id} baseline changed before setup: "
                f"expected={expected}, actual={before.current_position}"
            )
        if before.current_mA != 0 or before.error_code != 0:
            raise SafetyViolation(
                f"ID{motor_id} unsafe before setup: current={before.current_mA}mA "
                f"fault=0x{before.error_code:02X}"
            )

        payload = [mode, 0, 0, speed, baseline_position]
        response = motor.write_register(CONTROL_MODE_ADDR, payload)
        self._write_count += 1
        validate_status_feedback(response, motor_id, self._limits.get(motor_id))

        readback_mode = _unwrap_register_value(motor.read_register(CONTROL_MODE_ADDR))
        readback_speed = _unwrap_register_value(motor.read_register(TARGET_SPEED_ADDR))
        readback_target = _unwrap_register_value(motor.read_register(TARGET_POSITION_ADDR))
        after = motor.read_status()
        validate_status_feedback(after, motor_id, self._limits.get(motor_id))
        target_position_ok = (
            readback_target == baseline_position
            if target_band is None
            else target_band[0] <= readback_target <= target_band[1]
        )
        if (
            readback_mode != mode
            or readback_speed != speed
            or not target_position_ok
        ):
            raise DriverError(
                "single-axis setup readback mismatch: "
                f"mode={readback_mode}, speed={readback_speed}, target={readback_target}"
            )
        after_position_ok = (
            after.current_position == baseline_position
            if baseline_band is None
            else baseline_band[0] <= after.current_position <= baseline_band[1]
        )
        if not after_position_ok or after.current_mA != 0 or after.error_code != 0:
            raise SafetyViolation(
                f"ID{motor_id} unsafe after setup: actual={after.current_position} "
                f"current={after.current_mA}mA fault=0x{after.error_code:02X}"
            )
        return {
            "motor_id": motor_id,
            "control_mode": readback_mode,
            "target_speed": readback_speed,
            "target_position": readback_target,
            "actual_position": after.current_position,
            "current_mA": after.current_mA,
            "fault": after.error_code,
        }

    def write_validated_config_block(
        self,
        motor_id: int,
        reference_position: int,
        mode: int = SPEED_MODE,
        speed: int = DEFAULT_SPEED,
    ) -> StatusFeedback:
        """Write one validated J2-C/M6 contiguous configuration block once.

        This method deliberately performs no retry and no readback. The J2-C
        runner completes all six writes first, then performs a separate
        all-axis readback gate before dispatching any target position.
        """
        if self._hardware_mode == HardwareMode.READ_ONLY:
            raise ReadOnlyModeError(
                "J2-C configuration is forbidden in READ_ONLY mode (0 writes enforced)."
            )
        if motor_id not in MOTOR_IDS:
            raise TargetValidationError(f"Invalid motor_id {motor_id}. Must be in {MOTOR_IDS}")
        if mode != SPEED_MODE or speed != DEFAULT_SPEED:
            raise TargetValidationError(
                f"J2-C setup is fixed to mode={SPEED_MODE}, speed={DEFAULT_SPEED}"
            )
        if not isinstance(reference_position, int) or isinstance(reference_position, bool):
            raise TargetValidationError("reference_position must be an integer")
        self._ensure_connected()
        if not self._limits:
            self.read_limits()
        lower, upper = self._limits[motor_id]
        safe_lower = max(SOFTWARE_LOWER_LIMIT, lower)
        safe_upper = min(SOFTWARE_UPPER_LIMIT, upper)
        if not safe_lower <= reference_position <= safe_upper:
            raise TargetValidationError(
                f"reference position {reference_position} outside [{safe_lower}, {safe_upper}]"
            )

        response = self._motors[motor_id].write_register(
            CONTROL_MODE_ADDR, [mode, 0, 0, speed, reference_position]
        )
        self._write_count += 1
        validate_status_feedback(response, motor_id, self._limits.get(motor_id))
        return response

    def command_single_axis_with_metrics(
        self,
        motor_id: int,
        target_step: int,
    ) -> SingleAxisCommandResult:
        """Execute isolated single-axis move with detailed metrics and observer status."""
        if self._hardware_mode == HardwareMode.READ_ONLY:
            raise ReadOnlyModeError("command_single_axis_for_bringup is forbidden in READ_ONLY mode (0 writes).")

        if motor_id not in MOTOR_IDS:
            raise TargetValidationError(f"Invalid motor_id {motor_id}. Must be in {MOTOR_IDS}")

        if not isinstance(target_step, int) or isinstance(target_step, bool):
            raise TargetValidationError(f"Target step must be an integer, got {type(target_step).__name__}")

        self._ensure_connected()
        if not self._limits:
            self.read_limits()

        # Validate target against motor_id limits
        hw_lower, hw_upper = self._limits[motor_id]
        safe_lower = max(SOFTWARE_LOWER_LIMIT, hw_lower)
        safe_upper = min(SOFTWARE_UPPER_LIMIT, hw_upper)
        if not (safe_lower <= target_step <= safe_upper):
            raise TargetValidationError(
                f"Motor ID{motor_id} target {target_step} outside safe boundary [{safe_lower}, {safe_upper}]"
            )

        # Pre-read and health check observer motors (read-only)
        observers: dict[int, StatusFeedback] = {}
        for mid in MOTOR_IDS:
            if mid != motor_id:
                obs_status = self._motors[mid].read_status()
                validate_status_feedback(obs_status, mid, self._limits.get(mid))
                observers[mid] = obs_status

        # Write 0x29 ONLY to motor_id
        target_motor = self._motors[motor_id]
        t_write = time.time()
        try:
            resp = target_motor.write_register(TARGET_POSITION_ADDR, [target_step])
            self._write_count += 1
            if isinstance(resp, StatusFeedback):
                validate_status_feedback(resp, motor_id, self._limits.get(motor_id))
                moving_status = resp
            else:
                moving_status = target_motor.read_status()
                validate_status_feedback(moving_status, motor_id, self._limits.get(motor_id))

            return SingleAxisCommandResult(
                motor_id=motor_id,
                commanded_target=target_step,
                timestamp_write=t_write,
                response=moving_status,
                observer_states=observers,
            )
        except Exception as exc:
            logger.error(
                "Exception during single axis command on ID%d: %s. Pausing moving motor.",
                motor_id,
                exc,
            )
            try:
                target_motor.write_register(PAUSE_ADDR, [1])
                self._write_count += 1
            except Exception as pause_exc:
                logger.error("Failed to pause moving motor ID%d: %s", motor_id, pause_exc)
            raise

    def pause_all(self) -> dict[int, bool]:
        """Emergency stop / pause all 6 motors by writing 0x1A: [1] sequentially."""
        self._ensure_connected()
        if self._hardware_mode == HardwareMode.READ_ONLY:
            raise ReadOnlyModeError("pause_all write is forbidden in READ_ONLY mode (0 writes enforced).")

        results: dict[int, bool] = {}
        for mid in MOTOR_IDS:
            try:
                motor = self._motors[mid]
                motor.write_register(PAUSE_ADDR, [1])
                self._write_count += 1
                results[mid] = True
            except Exception as exc:
                logger.error("Failed to pause motor ID%d: %s", mid, exc)
                results[mid] = False

        self._last_pause_results = dict(results)
        return results

    @property
    def last_pause_results(self) -> dict[int, bool]:
        """Results from the most recent six-axis pause attempt."""
        return dict(self._last_pause_results)

    def pause_single_axis(self, motor_id: int) -> StatusFeedback:
        """Pause exactly one axis and return its immediate status feedback."""
        self._ensure_connected()
        if self._hardware_mode == HardwareMode.READ_ONLY:
            raise ReadOnlyModeError("pause_single_axis is forbidden in READ_ONLY mode (0 writes enforced).")
        if motor_id not in MOTOR_IDS:
            raise TargetValidationError(f"Invalid motor_id {motor_id}. Must be in {MOTOR_IDS}")

        motor = self._motors[motor_id]
        response = motor.write_register(PAUSE_ADDR, [1])
        self._write_count += 1
        if isinstance(response, StatusFeedback):
            status = response
        else:
            status = motor.read_status()
        validate_status_feedback(status, motor_id, self._limits.get(motor_id))
        return status

    def close(self) -> None:
        """Close the UART connection and release resources (0 writes)."""
        if self._bus is not None:
            try:
                self._bus.close()
            except Exception as exc:
                logger.warning("Error closing bus: %s", exc)
        self._connected = False
        self._motors.clear()

    def __enter__(self) -> SixMotorController:
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.close()

    def _ensure_connected(self) -> None:
        if not self._connected or not self._motors:
            raise DriverError("SixMotorController is not connected. Call connect() first.")
