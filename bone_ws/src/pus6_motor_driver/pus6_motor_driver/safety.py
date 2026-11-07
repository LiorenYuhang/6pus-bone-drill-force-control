"""Safety invariants, bounds checking, and validation rules for 6PUS motor driver."""

from dataclasses import dataclass
from enum import Enum
from typing import Mapping, Sequence
import sys
import math
from pathlib import Path

try:
    from pus6_kinematics.geometry_config import load_geometry_config
except ImportError:
    _kinematics_pkg = Path(__file__).resolve().parents[2] / "pus6_kinematics"
    if str(_kinematics_pkg) not in sys.path:
        sys.path.insert(0, str(_kinematics_pkg))
    from pus6_kinematics.geometry_config import load_geometry_config

_GEOMETRY_CONFIG = load_geometry_config()

try:
    from .protocol import StatusFeedback
except ImportError:
    from protocol import StatusFeedback


MOTOR_IDS: tuple[int, ...] = (1, 2, 3, 4, 5, 6)
NUM_MOTORS: int = 6

CONTROL_MODE_ADDR: int = 0x25
TARGET_FORCE_ADDR: int = 0x27
TARGET_SPEED_ADDR: int = 0x28
TARGET_POSITION_ADDR: int = 0x29
ACTUAL_POSITION_ADDR: int = 0x2A
ACTUAL_FORCE_ADDR: int = 0x2C
FORCE_ADC_ADDR: int = 0x2D
LOWER_LIMIT_ADDR: int = 0x24
UPPER_LIMIT_ADDR: int = 0x23
PAUSE_ADDR: int = 0x1A

MODE_POSITION: int = 0
MODE_SERVO: int = 1
MODE_SPEED: int = 2
MODE_FORCE: int = 3

# Native registers retain signed g; nominal SI conversion lives in force_units.
from .force_units import (NOMINAL_N_PER_G, NOMINAL_G_PER_N, g_to_n, n_to_g,
                          encode_signed_i16_to_u16, decode_u16_to_signed_i16)

SPEED_MODE: int = 2
DEFAULT_SPEED: int = 50
MAX_CURRENT_MA: int = 1500
# Formal control-layer deadband for static position checks.  This is an
# acceptance band around the selected reference; raw register/status values
# are never modified or normalized.
STATIC_POSITION_TOLERANCE_STEPS: int = 2

ABSOLUTE_RAW_LOWER_LIMIT: int = _GEOMETRY_CONFIG.absolute_raw_min_steps
ABSOLUTE_RAW_UPPER_LIMIT: int = _GEOMETRY_CONFIG.absolute_raw_max_steps
SOFTWARE_LOWER_LIMIT: int = _GEOMETRY_CONFIG.software_command_min_steps
SOFTWARE_UPPER_LIMIT: int = _GEOMETRY_CONFIG.software_command_max_steps
NORMAL_MOTION_LOWER_LIMIT: int = _GEOMETRY_CONFIG.normal_motion_min_steps
NORMAL_MOTION_UPPER_LIMIT: int = _GEOMETRY_CONFIG.normal_motion_max_steps

# Commanded targets and encoder feedback have different safety contracts. A
# target is never allowed outside the absolute raw command range. Feedback is
# permitted a small signed zero-reference deviation so values such as -2 from
# an encoder at software zero do not abort an otherwise valid motion command.
FEEDBACK_POSITION_TOLERANCE_STEPS: int = 5
FEEDBACK_LOWER_LIMIT: int = ABSOLUTE_RAW_LOWER_LIMIT - FEEDBACK_POSITION_TOLERANCE_STEPS
FEEDBACK_UPPER_LIMIT: int = ABSOLUTE_RAW_UPPER_LIMIT + FEEDBACK_POSITION_TOLERANCE_STEPS

# Validated standalone-runner UART default. Keep this aligned with
# pus6_motor_driver/config/motors.yaml and the ROS bringup configuration.
DEFAULT_SERIAL_PORT: str = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"

# Hardware Dual-Pose Constants (Storage Park vs Operating Home)
STORAGE_PARK_RAW_STEPS: tuple[int, ...] = _GEOMETRY_CONFIG.storage_park_raw_steps
OPERATING_HOME_RAW_STEPS: tuple[int, ...] = _GEOMETRY_CONFIG.operating_home_raw_steps
STORAGE_PARK_RAW_STEP: int = STORAGE_PARK_RAW_STEPS[0]
OPERATING_HOME_RAW_STEP: int = OPERATING_HOME_RAW_STEPS[0]

# Startup State Acceptance Bands (Provisional +/- 5 step tolerance)
PARK_MEASUREMENT_BAND_LOWER: int = STORAGE_PARK_RAW_STEP - FEEDBACK_POSITION_TOLERANCE_STEPS  # -5
PARK_MEASUREMENT_BAND_UPPER: int = STORAGE_PARK_RAW_STEP + FEEDBACK_POSITION_TOLERANCE_STEPS  # +5
HOME_MEASUREMENT_BAND_LOWER: int = OPERATING_HOME_RAW_STEP - FEEDBACK_POSITION_TOLERANCE_STEPS  # 995
HOME_MEASUREMENT_BAND_UPPER: int = OPERATING_HOME_RAW_STEP + FEEDBACK_POSITION_TOLERANCE_STEPS  # 1005


class StartupClassification(str, Enum):
    """Authoritative physical startup classification for 6PUS platform."""
    VALID_PARK = "valid_park"              # All 6 axes within provisional PARK measurement band [-5, 5] steps
    VALID_HOME = "valid_home"              # All 6 axes within HOME measurement band [995, 1005] steps
    VALID_IN_ENVELOPE = "valid_in_envelope" # All 6 axes within valid commanded envelope [0, 2000] steps
    INVALID_OUT_OF_RANGE = "invalid_out_of_range" # One or more axes outside feedback limits [-5, 2005] steps
    MIXED_UNCLASSIFIED = "mixed_unclassified" # Inconsistent/mixed positions that cannot be safely classified


@dataclass(frozen=True)
class StartupStateAssessment:
    classification: StartupClassification
    measured_positions: tuple[int, ...]
    planning_seed: tuple[int, ...]
    is_park: bool
    is_home: bool
    is_valid: bool
    description: str


def classify_startup_state(
    measured_positions: Sequence[int],
    *,
    history_samples: Sequence[Sequence[int]] | None = None,
    stability_tolerance_steps: int = STATIC_POSITION_TOLERANCE_STEPS,
) -> StartupStateAssessment:
    """Classify 6-axis measured startup state and derive authoritative planning seed.

    Contract:
    1. If any measurement is outside [FEEDBACK_LOWER_LIMIT, FEEDBACK_UPPER_LIMIT] ([-5, 2005]):
       -> INVALID_OUT_OF_RANGE (is_valid=False, cannot plan)
    2. If history_samples is provided and position drift on any axis > stability_tolerance_steps:
       -> MIXED_UNCLASSIFIED (is_valid=False, readings unstable)
    3. If ALL 6 axes are within the provisional PARK measurement band [-5, +5]:
       -> VALID_PARK (is_valid=True, planning_seed is set to exactly [0, 0, 0, 0, 0, 0])
       Raw telemetry is preserved; motion planning starts from [0]*6 so all commanded targets remain >= 0.
    4. If ALL 6 axes are within the HOME measurement band [995, 1005]:
       -> VALID_HOME (is_valid=True, planning_seed is set to [1000, 1000, 1000, 1000, 1000, 1000])
    5. If ALL 6 axes are strictly within the command envelope [0, 2000]:
       -> VALID_IN_ENVELOPE (is_valid=True, planning_seed is set to measured_positions)
    6. Otherwise (e.g. mixed state with negative measurements on some axes without all being at PARK):
       -> MIXED_UNCLASSIFIED (is_valid=False, cannot plan; generic clamping is strictly forbidden)
    """
    if len(measured_positions) != NUM_MOTORS:
        raise ValueError(f"Expected {NUM_MOTORS} measured positions, got {len(measured_positions)}")

    raw_tuple = tuple(int(p) for p in measured_positions)

    # 1. Feedback bounds check
    for mid, pos in zip(MOTOR_IDS, raw_tuple):
        if not (FEEDBACK_LOWER_LIMIT <= pos <= FEEDBACK_UPPER_LIMIT):
            return StartupStateAssessment(
                classification=StartupClassification.INVALID_OUT_OF_RANGE,
                measured_positions=raw_tuple,
                planning_seed=(),
                is_park=False,
                is_home=False,
                is_valid=False,
                description=f"Motor ID{mid} measured position {pos} outside valid feedback limits [{FEEDBACK_LOWER_LIMIT}, {FEEDBACK_UPPER_LIMIT}]",
            )

    # 2. Stability check across history samples if provided
    if history_samples and len(history_samples) > 1:
        for idx, mid in enumerate(MOTOR_IDS):
            axis_vals = [s[idx] for s in history_samples]
            drift = max(axis_vals) - min(axis_vals)
            if drift > stability_tolerance_steps:
                return StartupStateAssessment(
                    classification=StartupClassification.MIXED_UNCLASSIFIED,
                    measured_positions=raw_tuple,
                    planning_seed=(),
                    is_park=False,
                    is_home=False,
                    is_valid=False,
                    description=f"Motor ID{mid} position unstable across preflight samples (drift {drift} > {stability_tolerance_steps} steps)",
                )

    # 3. Classify VALID_PARK: all 6 axes within [-5, +5] steps
    if all(PARK_MEASUREMENT_BAND_LOWER <= p <= PARK_MEASUREMENT_BAND_UPPER for p in raw_tuple):
        return StartupStateAssessment(
            classification=StartupClassification.VALID_PARK,
            measured_positions=raw_tuple,
            planning_seed=STORAGE_PARK_RAW_STEPS,
            is_park=True,
            is_home=False,
            is_valid=True,
            description=f"All 6 axes within provisional PARK measurement band [{PARK_MEASUREMENT_BAND_LOWER}, {PARK_MEASUREMENT_BAND_UPPER}] steps",
        )

    # 4. Classify VALID_HOME: all 6 axes within [995, 1005] steps
    if all(HOME_MEASUREMENT_BAND_LOWER <= p <= HOME_MEASUREMENT_BAND_UPPER for p in raw_tuple):
        return StartupStateAssessment(
            classification=StartupClassification.VALID_HOME,
            measured_positions=raw_tuple,
            planning_seed=OPERATING_HOME_RAW_STEPS,
            is_park=False,
            is_home=True,
            is_valid=True,
            description=f"All 6 axes within HOME measurement band [{HOME_MEASUREMENT_BAND_LOWER}, {HOME_MEASUREMENT_BAND_UPPER}] steps",
        )

    # 5. Classify VALID_IN_ENVELOPE: all 6 axes strictly within [0, 2000] steps
    if all(ABSOLUTE_RAW_LOWER_LIMIT <= p <= ABSOLUTE_RAW_UPPER_LIMIT for p in raw_tuple):
        return StartupStateAssessment(
            classification=StartupClassification.VALID_IN_ENVELOPE,
            measured_positions=raw_tuple,
            planning_seed=raw_tuple,
            is_park=False,
            is_home=False,
            is_valid=True,
            description=f"All 6 axes within commanded envelope [{ABSOLUTE_RAW_LOWER_LIMIT}, {ABSOLUTE_RAW_UPPER_LIMIT}] steps",
        )

    # 6. Mixed / unclassified state (e.g. some axes in [-5, -1] while others are > 5)
    return StartupStateAssessment(
        classification=StartupClassification.MIXED_UNCLASSIFIED,
        measured_positions=raw_tuple,
        planning_seed=(),
        is_park=False,
        is_home=False,
        is_valid=False,
        description=f"Mixed/unclassified startup state {list(raw_tuple)}: non-uniform negative offsets cannot be safely seeded",
    )

# SI / Step Unit Conversion Constants (40 step/mm = 40,000 step/m)
STEPS_PER_METER: float = _GEOMETRY_CONFIG.steps_per_meter
STEPS_PER_MM: float = STEPS_PER_METER / 1000.0
METERS_PER_STEP: float = _GEOMETRY_CONFIG.meters_per_step
MM_PER_STEP: float = METERS_PER_STEP * 1000.0


def steps_to_meters(steps: int | float) -> float:
    """Convert device step counts to SI meters."""
    return float(steps) * METERS_PER_STEP


def uint16_to_int16(value: int) -> int:
    """Decode one uint16 transport value as a signed int16 step count.

    Position feedback registers are transported as uint16 on the wire but
    represent signed step counts. This conversion is explicit and applies
    no offset and no safety-limit relaxation.
    """
    if not isinstance(value, int) or isinstance(value, bool):
        raise TypeError(f"uint16 register value must be an integer, got {type(value).__name__}")
    if not 0 <= value <= 0xFFFF:
        raise ValueError(f"uint16 register value outside [0, 65535]: {value}")
    return value - 0x10000 if value >= 0x8000 else value


def meters_to_steps(meters: float) -> int:
    """Convert SI meters to nearest integer device step counts."""
    return int(round(meters * STEPS_PER_METER))


def steps_to_mm(steps: int | float) -> float:
    """Convert device step counts to millimeters."""
    return float(steps) * MM_PER_STEP


def mm_to_steps(mm: float) -> int:
    """Convert millimeters to nearest integer device step counts."""
    return int(round(mm * STEPS_PER_MM))


def raw_to_relative_steps(raw_steps: int | Sequence[int]) -> int | list[int]:
    """Convert raw device step counts to software relative steps (q_rel = q_raw - 1000)."""
    if isinstance(raw_steps, (int, float)):
        return int(round(raw_steps)) - OPERATING_HOME_RAW_STEP
    return [int(round(r)) - OPERATING_HOME_RAW_STEP for r in raw_steps]


def relative_to_raw_steps(rel_steps: int | Sequence[int]) -> int | list[int]:
    """Convert software relative steps to raw device step counts (q_raw = q_rel + 1000)."""
    if isinstance(rel_steps, (int, float)):
        return int(round(rel_steps)) + OPERATING_HOME_RAW_STEP
    return [int(round(r)) + OPERATING_HOME_RAW_STEP for r in rel_steps]


from enum import Enum


class HardwareMode(str, Enum):
    """Execution mode for SixMotorController and Pus6MotorDriverNode."""
    MOCK = "mock"            # No real serial port opened; uses fake/mock backend
    READ_ONLY = "read_only"  # Real serial port opened; telemetry reads allowed; ALL writes strictly blocked (0 writes)
    ACTIVE = "active"        # Real serial port opened; validated hardware writes permitted


def parse_hardware_mode(mode: str | HardwareMode) -> HardwareMode:
    """Parse and validate hardware mode string."""
    if isinstance(mode, HardwareMode):
        return mode
    val = str(mode).strip().lower()
    for member in HardwareMode:
        if member.value == val or member.name.lower() == val:
            return member
    raise ValueError(
        f"Invalid hardware mode '{mode}'. Must be one of: {[m.value for m in HardwareMode]}"
    )


class DriverError(Exception):
    """Base exception for pus6_motor_driver errors."""


class SafetyViolation(DriverError):
    """Raised when a hardware safety condition or boundary constraint is violated."""


class ReadOnlyModeError(SafetyViolation):
    """Raised when a write operation is attempted while in READ_ONLY hardware mode."""


class J2F0SafetyViolation(SafetyViolation):
    """Raised when J2-F0 safety invariants (current threshold, fault codes) are violated."""


class J2F0Stop(SafetyViolation):
    """Raised when J2-F0 real-time STOP conditions (>3 step drift, bus errors) are triggered."""


class TargetValidationError(SafetyViolation):
    """Raised when target position vector validation fails before write execution."""


def validate_target_vector(
    targets: Sequence[int],
    limits: Mapping[int, tuple[int, int]] | None = None,
) -> list[int]:
    """Atomically validate a 6-element target vector for ID1..ID6.
    This function validates commanded trajectory targets only; feedback positions
    must be kept separate and must not be passed here.

    Args:
        targets: 6-element integer sequence corresponding to ID1..ID6.
        limits: Optional mapping of motor_id -> (lower_limit, upper_limit).

    Returns:
        Validated target positions as a list of integers.

    Raises:
        TargetValidationError: If length != 6, non-integer, or target outside safe limits.
    """
    if len(targets) != NUM_MOTORS:
        raise TargetValidationError(
            f"Target position vector length must be exactly {NUM_MOTORS} (for ID1..ID6), got {len(targets)}"
        )

    validated: list[int] = []
    for idx, mid in enumerate(MOTOR_IDS):
        val = targets[idx]
        if not isinstance(val, int) or isinstance(val, bool):
            raise TargetValidationError(
                f"Target position for ID{mid} must be an integer, got {type(val).__name__} ({val})"
            )

        # This is a commanded trajectory target, so use the absolute raw
        # command range. The normal-motion envelope (currently [200, 1800])
        # is a separate planning policy and must not reject PARK/HOME targets.
        lower = ABSOLUTE_RAW_LOWER_LIMIT
        upper = ABSOLUTE_RAW_UPPER_LIMIT

        if limits is not None and mid in limits:
            hw_lower, hw_upper = limits[mid]
            lower = max(lower, hw_lower)
            upper = min(upper, hw_upper)

        if not (lower <= val <= upper):
            raise TargetValidationError(
                f"Motor ID{mid} target {val} outside safe boundary [{lower}, {upper}]"
            )
        validated.append(val)

    return validated


def validate_status_feedback(
    status: StatusFeedback,
    motor_id: int,
    limits: tuple[int, int] | None = None,
) -> None:
    """Validate status feedback health and the independent encoder range.

    The optional ``limits`` argument is retained for API compatibility with
    callers that also validate commanded targets; feedback position checking
    intentionally uses the dedicated zero-offset tolerance below.

    Args:
        status: Feedback status frame.
        motor_id: Logical motor ID (1..6).
        limits: Optional command-limit tuple retained for call-site compatibility.

    Raises:
        SafetyViolation: If the status reports a fault, overcurrent, or
            feedback position outside the monitoring range.
    """
    if status.error_code != 0:
        raise SafetyViolation(
            f"Motor ID{motor_id} reported hardware fault error code 0x{status.error_code:02X}"
        )

    if status.current_mA > MAX_CURRENT_MA:
        raise SafetyViolation(
            f"Motor ID{motor_id} current {status.current_mA}mA exceeds threshold {MAX_CURRENT_MA}mA"
        )

    if not (FEEDBACK_LOWER_LIMIT <= status.current_position <= FEEDBACK_UPPER_LIMIT):
        raise SafetyViolation(
            f"Motor ID{motor_id} actual position {status.current_position} "
            f"outside feedback range [{FEEDBACK_LOWER_LIMIT}, {FEEDBACK_UPPER_LIMIT}]"
        )


def validate_force_vector(
    forces_n: Sequence[float],
    limits_n: tuple[float, float] | None = None,
) -> list[float]:
    """Validate that forces_n is a 6-element sequence of finite floats within limits."""
    import math
    if len(forces_n) != NUM_MOTORS:
        raise TargetValidationError(f"Expected 6 force values, got {len(forces_n)}")
    validated = []
    for mid, val in zip(MOTOR_IDS, forces_n):
        if not math.isfinite(val):
            raise TargetValidationError(f"Motor ID{mid} force {val} is not finite")
        if limits_n is not None:
            lower, upper = limits_n
            if not (lower <= val <= upper):
                raise TargetValidationError(
                    f"Motor ID{mid} force {val} N outside safe limit [{lower}, {upper}] N"
                )
        validated.append(float(val))
    return validated


import fcntl
import json
import os
import time
from datetime import datetime, timezone

DEFAULT_UART_LOCKFILE: str = "/tmp/pus6_uart_owner.lock"


class SerialPortLock:
    """Kernel-enforced OS-level mutual exclusion lock for serial port access.

    Provides two complementary layers of mutual exclusion:
    1. Direct file lock (fcntl.flock) on the open serial port file descriptor.
    2. Global advisory lockfile (/tmp/pus6_uart_owner.lock) recording PID, owner name,
       and acquisition timestamp, also guarded with flock.

    Guarantees:
    - Independent processes (e.g. read_only_diagnostic vs normal Child) cannot open or
      access the serial port concurrently.
    - If a process crashes or is killed, kernel automatically releases all locks.
    - Does NOT rely on fragile process name searching (e.g. ps / pgrep).
    """

    def __init__(
        self,
        serial_fd: int | None = None,
        port: str = "/dev/ttyUSB0",
        owner: str = "unknown",
        lockfile_path: str = DEFAULT_UART_LOCKFILE,
    ) -> None:
        self.serial_fd = serial_fd
        self.port = port
        self.owner = owner
        self.lockfile_path = lockfile_path
        self._lockfile_fd: int | None = None
        self._serial_locked = False
        self._lockfile_locked = False

    def acquire(self) -> None:
        # Layer 1: Acquire lock on global lockfile and record owner metadata
        try:
            self._lockfile_fd = os.open(
                self.lockfile_path,
                os.O_RDWR | os.O_CREAT,
                0o666,
            )
            fcntl.flock(self._lockfile_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            self._lockfile_locked = True

            # Write owner record
            info = {
                "pid": os.getpid(),
                "owner": self.owner,
                "port": self.port,
                "timestamp_ns": time.time_ns(),
                "timestamp_iso": datetime.now(timezone.utc).isoformat(),
            }
            os.ftruncate(self._lockfile_fd, 0)
            os.lseek(self._lockfile_fd, 0, os.SEEK_SET)
            os.write(self._lockfile_fd, json.dumps(info).encode("utf-8"))
        except (BlockingIOError, OSError) as exc:
            existing_owner = "unknown"
            existing_pid = "unknown"
            if self._lockfile_fd is not None:
                try:
                    os.lseek(self._lockfile_fd, 0, os.SEEK_SET)
                    content = os.read(self._lockfile_fd, 1024).decode("utf-8", errors="ignore")
                    data = json.loads(content)
                    existing_owner = data.get("owner", "unknown")
                    existing_pid = data.get("pid", "unknown")
                except Exception:
                    pass
                try:
                    os.close(self._lockfile_fd)
                except Exception:
                    pass
                self._lockfile_fd = None
            raise BlockingIOError(
                f"UART port {self.port} is already owned by PID {existing_pid} ({existing_owner}). "
                f"Exclusive single-owner constraint prevents concurrent access."
            ) from exc

        # Layer 2: Acquire direct lock on the serial device file descriptor (if open)
        if self.serial_fd is not None:
            try:
                fcntl.flock(self.serial_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
                self._serial_locked = True
            except (BlockingIOError, OSError) as exc:
                self.release()
                raise BlockingIOError(
                    f"Direct lock on serial device fd {self.serial_fd} ({self.port}) failed: already locked by another process"
                ) from exc

    def release(self) -> None:
        if self._serial_locked and self.serial_fd is not None:
            try:
                fcntl.flock(self.serial_fd, fcntl.LOCK_UN)
            except OSError:
                pass
            self._serial_locked = False

        if self._lockfile_locked and self._lockfile_fd is not None:
            try:
                os.ftruncate(self._lockfile_fd, 0)
                fcntl.flock(self._lockfile_fd, fcntl.LOCK_UN)
            except OSError:
                pass
            self._lockfile_locked = False

        if self._lockfile_fd is not None:
            try:
                os.close(self._lockfile_fd)
            except OSError:
                pass
            self._lockfile_fd = None

    def __enter__(self) -> "SerialPortLock":
        self.acquire()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.release()


class ForceHandoverState(str, Enum):
    """Lifecycle state machine for 6-axis Mode 3 force control handover."""
    MODE0_POSITION = "mode0_position"
    PREPARING_FORCE_HANDOVER = "preparing_force_handover"
    SWITCHING_MODE3 = "switching_mode3"
    MODE3_FORCE_ACTIVE = "mode3_force_active"
    EXITING_TO_POSITION = "exiting_to_position"
    FAULT_LATCHED = "fault_latched"
    SAFE_STOP = "safe_stop"


# Historical Stage 3B-1 measured load in g (ID1..ID6), not a live handover baseline.
# ID1: 3754 g, ID2: -687 g, ID3: -6176 g
# ID4: 793 g, ID5: -671 g, ID6: -6994 g
STAGE3B1_STATIC_BASELINE_G: tuple[int, ...] = (3754, -687, -6176, 793, -671, -6994)


def validate_initial_handover_forces(
    initial_forces_g: Sequence[int] | None = None,
    *,
    static_baseline_g: Sequence[int] | None = None,
    handover_tolerance_g: float = 1000.0,
) -> list[int]:
    """Validate 6-axis initial force vector before transitioning into Mode 3.

    All force quantities are in motor native units: g (gram).
    Manufacturer sales confirmed: signed int16 [-32768, 32767], negative forces are valid.

    Invariants:
    1. Must have exactly 6 integer elements.
    2. Must be within 16-bit signed range [-32768, 32767].
    3. If static_baseline_g is provided and has significant static load (> 100 g on any axis),
       a zero vector [0, 0, 0, 0, 0, 0] is strictly forbidden (would drop the platform).
    4. If static_baseline_g is provided, initial_forces_g must not deviate from baseline
       by more than handover_tolerance_g per axis (unless tolerance is None or <= 0).
    """
    if initial_forces_g is None:
        raise TargetValidationError("initial_forces_g required")

    if len(initial_forces_g) != NUM_MOTORS:
        raise TargetValidationError(f"Expected {NUM_MOTORS} initial forces, got {len(initial_forces_g)}")

    validated: list[int] = []
    for mid, val in zip(MOTOR_IDS, initial_forces_g):
        if not isinstance(val, int) or isinstance(val, bool):
            raise TargetValidationError(f"Initial force for ID{mid} must be integer, got {type(val).__name__}")
        if not (-32768 <= val <= 32767):
            raise TargetValidationError(f"Initial force {val} for ID{mid} outside signed 16-bit range")
        validated.append(int(val))

    if static_baseline_g is not None and len(static_baseline_g) == NUM_MOTORS:
        has_static_load = any(abs(b) > 100 for b in static_baseline_g)
        if has_static_load and all(v == 0 for v in validated):
            raise SafetyViolation(
                f"Initial force handover rejected: static baseline is non-zero "
                f"{list(static_baseline_g)} g, but commanded initial force is all zero! "
                f"Zero-force handover would collapse the loaded platform."
            )
        if handover_tolerance_g is not None and handover_tolerance_g > 0:
            for mid, v, b in zip(MOTOR_IDS, validated, static_baseline_g):
                dev = abs(v - b)
                if dev > handover_tolerance_g:
                    raise SafetyViolation(
                        f"Initial force handover for ID{mid} ({v} g) deviates from static baseline "
                        f"({b} g) by {dev:.1f} g > allowed tolerance {handover_tolerance_g:.1f} g"
                    )

    return validated


@dataclass
class ForceBaselineManager:
    """Explicitly partitions and manages force domain baselines and components.

    Unit: g (gram). Manufacturer sales confirmed native force unit is g.
    - raw_sensor_g: 0x2C register raw reading directly from the motor built-in force sensor.
    - static_assembly_load_g: Measured static equilibrium forces under gravity and mechanical assembly constraint.
    - sensor_zero_g: Intrinsic sensor zero tare (unmounted/unloaded zero bias).
    - gravity_support_comp_g: Theoretical/model-based gravity and support compensation force component.
    - controller_desired_force_n: Active task-space / interaction force commanded by upstream controller in Newtons.
    """
    raw_sensor_g: tuple[int, ...] = (0, 0, 0, 0, 0, 0)
    static_assembly_load_g: tuple[int, ...] = STAGE3B1_STATIC_BASELINE_G
    sensor_zero_g: tuple[int, ...] = (0, 0, 0, 0, 0, 0)
    gravity_support_comp_g: tuple[int, ...] = (0, 0, 0, 0, 0, 0)
    controller_desired_force_n: tuple[float, ...] = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    software_baseline_g: tuple[int, ...] | None = None


    def capture_initial_state(self, raw_forces_g: Sequence[int]) -> tuple[int, ...]:
        """Current signed load becomes the relative reference, never a sensor error."""
        values = tuple(validate_initial_handover_forces(raw_forces_g))
        self.raw_sensor_g = values
        self.static_assembly_load_g = values
        self.software_baseline_g = values
        return values

    def relative_forces_g(self, raw_forces_g: Sequence[int]) -> tuple[int, ...]:
        if self.software_baseline_g is None:
            raise SafetyViolation("Current force baseline has not been captured")
        raw = validate_initial_handover_forces(raw_forces_g)
        return tuple(value - base for value, base in zip(raw, self.software_baseline_g))


    def command_from_delta_g(self, delta_forces_g: Sequence[float]) -> tuple[int, ...]:
        """F_command = F_hold + delta, in explicitly raw actuator g units."""
        if self.software_baseline_g is None:
            raise SafetyViolation("Current force baseline has not been captured")
        if len(delta_forces_g) != NUM_MOTORS or not all(math.isfinite(v) for v in delta_forces_g):
            raise TargetValidationError("six finite force increments in g required")
        return tuple(validate_initial_handover_forces([
            int(round(base + delta)) for base, delta in zip(self.software_baseline_g, delta_forces_g)
        ]))


    def validate_initial_forces(
        self,
        initial_forces_g: Sequence[int] | None = None,
        *,
        handover_tolerance_g: float = 1000.0,
        ) -> list[int]:
        return validate_initial_handover_forces(
            initial_forces_g,
            static_baseline_g=self.static_assembly_load_g,
            handover_tolerance_g=handover_tolerance_g,
        )
