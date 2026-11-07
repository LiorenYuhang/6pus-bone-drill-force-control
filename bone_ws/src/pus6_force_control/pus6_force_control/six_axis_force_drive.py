"""Independent Six-Axis Native Force Control Drive for Inspire-Robots cylinders.

Safety Framework:
- A zero-force target is NOT a stop or a position hold. Terminal handling belongs
  to the UART owner; independent hardware stopping covers loss of that owner.
- Strictly enforces per-axis force limits; normal slew is explicitly optional.
- Prohibits hardware writes (hardware_write_enabled=True) unless a verified,
  hardware-qualified ForceLimitsConfig is explicitly provided.
- Manufacturer sales confirmed: Native force data unit is g (gram); signed negative force is valid reverse/pull force.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
import time
from typing import Callable, Iterable, List, Optional, Sequence, Tuple

from pus6_motor_driver.protocol import pack_write_register_cmd
from pus6_motor_driver.safety import validate_status_feedback
from pus6_motor_driver.motor import Motor
from pus6_motor_driver.force_units import (
    DEFAULT_FORCE_LIMIT_N, NOMINAL_N_PER_G, g_to_n, n_to_g, quantize_force_g,
    quantize_force_g_with_n_limits,
    encode_signed_i16_to_u16, decode_u16_to_signed_i16,
)


CONTROL_MODE_REG = 0x25
OUTPUT_VOLTAGE_REG = 0x26
TARGET_FORCE_REG = 0x27
TARGET_SPEED_REG = 0x28
TARGET_POSITION_REG = 0x29
ACTUAL_POSITION_REG = 0x2A
CURRENT_REG = 0x2B
ACTUAL_FORCE_REG = 0x2C
FORCE_ADC_REG = 0x2D
TEMPERATURE_REG = 0x2E
ERROR_CODE_REG = 0x2F

MODE_POSITION = 0
MODE_SERVO = 1
MODE_SPEED = 2
MODE_FORCE = 3
MODE_VOLTAGE = 4
MODE_SPEED_FORCE = 5


@dataclass(frozen=True)
class ForceLimitsConfig:
    """Safety and dynamics limits for six-axis force control.

    Requirements:
    - Per-axis force saturation [min_force_n, max_force_n].
    - Per-axis normal slew rate limits [max_rate_g_s], or None to bypass.
    - Explicit recovery slew [brake_rate_g_s] is separate from normal bypass.
    - Bidirectional forces supported (signed int16 [-32768, 32767]); allow_negative_forces enables reverse/pulling force.
    - Hardware write qualification flag: must be explicitly True to enable physical writes.
    """
    min_force_n: Tuple[float, ...] = (-DEFAULT_FORCE_LIMIT_N,) * 6     # Per-axis minimum allowed force (N)
    max_force_n: Tuple[float, ...] = (DEFAULT_FORCE_LIMIT_N,) * 6      # Per-axis maximum allowed force (N)
    max_rate_g_s: Optional[Tuple[float, ...]] = (500.0,) * 6  # None: no normal slew; recovery remains separate
    feedback_max_age_s: float = 0.2                     # Sensor feedback timeout (s)
    allow_negative_forces: bool = True                  # Software support for bidirectional force
    is_hardware_qualified: bool = False                 # Must be True for hardware writes
    brake_rate_g_s: Optional[Tuple[float, ...]] = None   # explicit optional braking slew, g/s

    def __post_init__(self):
        for name in ("min_force_n", "max_force_n", "max_rate_g_s"):
            val = getattr(self, name)
            if name == 'max_rate_g_s' and val is None:
                continue
            if len(val) != 6 or not all(math.isfinite(x) for x in val):
                raise ValueError(f"{name} must contain 6 finite floats")
        if any(lo >= hi for lo, hi in zip(self.min_force_n, self.max_force_n)):
            raise ValueError("min_force_n must be strictly less than max_force_n")
        if self.max_rate_g_s is not None and any(r <= 0 for r in self.max_rate_g_s):
            raise ValueError("max_rate_g_s must be strictly positive")
        if self.brake_rate_g_s is not None and (len(self.brake_rate_g_s)!=6 or any(not math.isfinite(r) or r<=0 or
                (self.max_rate_g_s is not None and r<self.max_rate_g_s[i]) for i,r in enumerate(self.brake_rate_g_s))):
            raise ValueError('braking rates must be six finite values at least normal slew')
        if not math.isfinite(self.feedback_max_age_s) or self.feedback_max_age_s <= 0:
            raise ValueError("feedback_max_age_s must be positive and finite")
        if not self.allow_negative_forces and any(lo < 0 for lo in self.min_force_n):
            raise ValueError("allow_negative_forces is False but negative min_force_n was specified")

    @classmethod
    def create_qualified_conservative_config(
        cls,
        max_force_n: float = DEFAULT_FORCE_LIMIT_N,
        max_rate_g_s: float = 500.0,
        allow_negative_forces: bool = True,
    ) -> ForceLimitsConfig:
        """Explicit conservative configuration qualified for physical hardware testing."""
        min_f = (-max_force_n if allow_negative_forces else 0.0,) * 6
        return cls(
            min_force_n=min_f,
            max_force_n=(max_force_n,) * 6,
            max_rate_g_s=(max_rate_g_s,) * 6,
            feedback_max_age_s=0.2,
            allow_negative_forces=allow_negative_forces,
            is_hardware_qualified=True,
        )


class SixAxisForceDrive:
    """Six-axis native force control drive interface.

    Safety Architecture:
    - hardware_write_enabled: bool, default False.
    - Physical writes require hardware_write_enabled=True AND is_hardware_qualified=True.
    - Distinguishes software zero-force / controlled stop from physical hardware E-Stop.
    """

    def __init__(
        self,
        bus,
        limits_config: Optional[ForceLimitsConfig] = None,
        *,
        hardware_write_enabled: bool = False,
        event_logger: Optional[Callable[[str, dict], None]] = None,
        clock: Callable[[], int] = time.monotonic_ns,
        direction: Sequence[float] = (1.0,) * 6,
        newtons_per_g: Sequence[float] = (NOMINAL_N_PER_G,) * 6,
    ) -> None:
        self.bus = bus
        self.limits = limits_config or ForceLimitsConfig()
        self.hardware_write_enabled = hardware_write_enabled
        self.event = event_logger or (lambda kind, data: None)
        self.clock = clock
        self.direction = tuple(direction)
        self.newtons_per_g = tuple(newtons_per_g)
        g_to_n([0.0]*6, self.direction, self.newtons_per_g)  # validate calibration

        if self.hardware_write_enabled and not self.limits.is_hardware_qualified:
            raise ValueError(
                "Hardware writes cannot be enabled with unverified safety limits. "
                "Explicitly provide a qualified ForceLimitsConfig (is_hardware_qualified=True)."
            )

        self.motors = [Motor(bus, i) for i in range(1, 7)]
        self.last_commanded_forces_g: Optional[List[float]] = None
        self.last_command_time_ns: Optional[int] = None
        self.last_feedback_snapshot: Optional[dict] = None
        self.mode_3_active = False

    def anchor_hold_forces_g(self, force_g_raw, *, now_ns=None):
        """Validate signed native hold and seed the existing actual-dt limiter.

        Pure software: no clipping, Tare, mode selection or UART write.
        Limits apply to the absolute hold + increment, not just the increment.
        """
        values = list(force_g_raw)
        if len(values) != 6:
            raise ValueError("six native hold forces required")
        hold = [quantize_force_g(v) for v in values]
        if any(float(v) != q for v, q in zip(values, hold)):
            raise ValueError("hold must be actual integer 0x2C readings")
        hold_n = g_to_n(hold, self.direction, self.newtons_per_g)
        if any(not lo <= f <= hi for lo, f, hi in
               zip(self.limits.min_force_n, hold_n, self.limits.max_force_n)):
            raise ValueError("hold exceeds absolute actuator force limits")
        if not self.limits.allow_negative_forces and any(v < 0 for v in hold):
            raise ValueError("negative native hold is disabled")
        self.last_commanded_forces_g = list(hold)
        self.last_command_time_ns = self.clock() if now_ns is None else now_ns
        return tuple(hold)

    def _tare_interface(self, dry_run=False):
        from .hardware_tare import HardwareTareController
        if not hasattr(self, '_hardware_tare'):
            lock = getattr(self.bus, 'uart_lock', None) or getattr(self.bus, 'lock', None)
            if lock is None:
                raise RuntimeError('Tare requires the existing UART owner lock')
            self._hardware_tare = HardwareTareController(
                self.bus, lock, hardware_write_enabled=self.hardware_write_enabled,
                hardware_qualified=self.limits.is_hardware_qualified,
                dry_run=dry_run, event=self.event)
        self._hardware_tare.dry_run = dry_run
        self._hardware_tare.hardware_write_enabled = self.hardware_write_enabled
        self._hardware_tare.hardware_qualified = self.limits.is_hardware_qualified
        return self._hardware_tare

    def tare_axis(self, motor_id, *, dry_run=False, **settings):
        """Explicit device reference change; never called by initialization/PID."""
        return self._tare_interface(dry_run).tare_axis(motor_id, **settings)

    def tare_all_axes(self, *, dry_run=False, **settings):
        return self._tare_interface(dry_run).tare_all_axes(**settings)

    def build_target_force_frame(self, motor_id: int, force_g: int) -> bytes:
        """Construct official CMD_WR_REGISTER frame for register 0x27."""
        if not 1 <= motor_id <= 6:
            raise ValueError(f"motor_id must be in 1..6, got {motor_id}")
        encoded_u16 = encode_signed_i16_to_u16(quantize_force_g(force_g))
        return pack_write_register_cmd(motor_id, TARGET_FORCE_REG, [encoded_u16])

    def build_mode3_entry_frame(self, motor_id: int, initial_force_g: int = 0) -> bytes:
        """Construct multi-register frame: 0x25=3, 0x26=0, 0x27=initial_force_g."""
        if not 1 <= motor_id <= 6:
            raise ValueError(f"motor_id must be in 1..6, got {motor_id}")
        encoded_force = encode_signed_i16_to_u16(quantize_force_g(initial_force_g))
        return pack_write_register_cmd(motor_id, CONTROL_MODE_REG, [MODE_FORCE, 0, encoded_force])

    def clamp_and_rate_limit(self, target_forces_g: Sequence[float], now_ns: Optional[int] = None, *, braking_axes=None, recovery=False) -> List[float]:
        """Apply per-axis force saturation and slew-rate limiting."""
        if len(target_forces_g) != 6 or not all(math.isfinite(float(f)) for f in target_forces_g):
            raise ValueError("Six finite target force values required")

        now = self.clock() if now_ns is None else now_ns
        targets = [float(f) for f in target_forces_g]

        # Bounds are configured in actuator SI force. Map the endpoints, not
        # the target, to g; this native API retains a single command conversion.
        a = n_to_g(self.limits.min_force_n, self.direction, self.newtons_per_g)
        b = n_to_g(self.limits.max_force_n, self.direction, self.newtons_per_g)
        lower = [min(x, y) for x, y in zip(a, b)]
        upper = [max(x, y) for x, y in zip(a, b)]
        saturated = [max(lower[i], min(upper[i], targets[i])) for i in range(6)]

        mask=[False]*6 if braking_axes is None else list(braking_axes)
        if len(mask)!=6 or any(not isinstance(v,(bool,)) for v in mask):raise ValueError('six boolean braking flags required')
        if recovery:
            if self.limits.brake_rate_g_s is None:raise ValueError('recovery slew must be explicitly configured')
            rates = self.limits.brake_rate_g_s
        elif self.limits.max_rate_g_s is None:
            # Applies to reversals and force opposing velocity too. PID braking
            # flags cannot silently re-enable 5000 g/s during normal operation.
            return saturated
        else:
            if any(mask) and self.limits.brake_rate_g_s is None:raise ValueError('braking slew must be explicitly configured')
            rates=[self.limits.brake_rate_g_s[i] if mask[i] else self.limits.max_rate_g_s[i] for i in range(6)]
        # 3. Apply the same Torque actual-dt first-order formula with selected rate
        if self.last_commanded_forces_g is not None and self.last_command_time_ns is not None:
            dt_s = (now - self.last_command_time_ns) * 1e-9
            if dt_s > 0:
                rate_limited = []
                for i in range(6):
                    prev = self.last_commanded_forces_g[i]
                    max_step = rates[i] * dt_s
                    clamped = prev + max(-max_step, min(max_step, saturated[i] - prev))
                    rate_limited.append(clamped)
                dispatched = rate_limited
            else:
                dispatched = list(self.last_commanded_forces_g)
        else:
            dispatched = saturated

        return dispatched

    def command_forces_n(self, target_force_n, *, dry_run=None, now_ns=None, braking_axes=None, recovery=False):
        """SI force -> absolute N saturation -> g -> selected normal/recovery policy."""
        target_force_n = list(target_force_n)
        if len(target_force_n) != 6 or not all(math.isfinite(f) for f in target_force_n):
            raise ValueError('six finite target forces in N required')
        limited_n = [max(lo, min(hi, f)) for f, lo, hi in
                     zip(target_force_n, self.limits.min_force_n, self.limits.max_force_n)]
        target_force_g = n_to_g(limited_n, self.direction, self.newtons_per_g)
        result = self.command_forces(target_force_g, dry_run=dry_run, now_ns=now_ns, braking_axes=braking_axes, recovery=recovery)
        result['target_forces_g'] = target_force_g
        result['requested_forces_n'] = target_force_n
        result['limited_forces_n'] = limited_n
        return result

    def command_forces(
        self,
        target_forces_g: Sequence[float],
        *,
        dry_run: Optional[bool] = None,
        now_ns: Optional[int] = None,
        braking_axes=None,
        recovery=False,
    ) -> dict:
        """Dispatch or dry-run target forces for all 6 axes.

        If hardware_write_enabled is False or dry_run is True:
        validates limits, applies rate limiting, builds frames, but does NOT write to serial.
        """
        is_dry_run = dry_run if dry_run is not None else (not self.hardware_write_enabled)
        now_ns = self.clock() if now_ns is None else now_ns
        if (not isinstance(now_ns, int) or now_ns < 0 or
                (self.last_command_time_ns is not None and now_ns <= self.last_command_time_ns)):
            raise ValueError("nonpositive command interval or invalid monotonic timestamp")

        dispatched = self.clamp_and_rate_limit(target_forces_g, now_ns=now_ns,braking_axes=braking_axes, recovery=recovery)
        encoded_g = quantize_force_g_with_n_limits(dispatched, self.limits.min_force_n,
                                                   self.limits.max_force_n, self.direction, self.newtons_per_g)
        frames_audit = []

        for mid in range(1, 7):
            val_g = encoded_g[mid - 1]
            frame_bytes = self.build_target_force_frame(mid, val_g)
            frames_audit.append({
                "axis": mid,
                "target_force_g": val_g,
                "frame_hex": frame_bytes.hex(" "),
                "register": TARGET_FORCE_REG,
            })

        if not is_dry_run:
            if not self.hardware_write_enabled:
                raise RuntimeError("Hardware writes are blocked by safety guard (hardware_write_enabled=False)")
            if not self.limits.is_hardware_qualified:
                raise RuntimeError("Hardware writes blocked: safety limits not qualified for hardware.")
            if not self.limits.allow_negative_forces and any(f < 0 for f in dispatched):
                raise RuntimeError("Hardware writes blocked: negative force requested while negative forces unconfirmed.")

            # Transmit frames to physical motors
            responses = []
            for mid in range(1, 7):
                m = self.motors[mid - 1]
                val_g = encoded_g[mid - 1]
                encoded_u16 = encode_signed_i16_to_u16(val_g)
                resp = m.write_register(TARGET_FORCE_REG, [encoded_u16])
                validate_status_feedback(resp, mid)
                responses.append(resp)
        else:
            responses = None

        self.last_commanded_forces_g = list(dispatched)
        self.last_command_time_ns = now_ns

        result = {
            "command_phase": "recovery" if recovery else "normal",
            "normal_force_slew_enabled": self.limits.max_rate_g_s is not None,
            "dispatched_forces_g": dispatched,
            "dry_run": is_dry_run,
            "timestamp_ns": now_ns,
            "frames": frames_audit,
            "responses": responses,
        }
        self.event("force_command", result)
        return result

    def software_force_stop(self) -> dict:
        """Legacy API: command zero native force; this is NOT a motion stop.

        Zero force can unload the mechanism. It neither brakes Mode3 nor holds
        position, and must never replace an energized position-servo stop.
        """
        result=self.command_forces([0.0]*6)
        result['motion_stop_confirmed']=False
        return result

    def read_feedback(self) -> dict:
        """Reads status feedback from all six motors (CMD_RD_STATUS 0x30, safe read-only)."""
        rows = []
        rx_times = []
        for m in self.motors:
            s = m.read_status()
            t = self.clock()
            validate_status_feedback(s, m.motor_id)
            rows.append(s)
            rx_times.append(t)

        snapshot = {
            "status": rows,
            "rx_ns": rx_times,
            "force_g_raw": [s.force_g for s in rows],
            "force_adc": [s.force_adc for s in rows],
            "position_steps": [s.current_position for s in rows],
            "current_mA": [s.current_mA for s in rows],
            "error_codes": [s.error_code for s in rows],
            "temperatures_c": [s.temperature for s in rows],
            "all_healthy": all(s.error_code == 0 for s in rows),
        }
        self.last_feedback_snapshot = snapshot
        return snapshot

    def is_feedback_fresh(self, max_age_s: Optional[float] = None) -> bool:
        """Check if latest feedback snapshot is fresh within configured threshold."""
        if self.last_feedback_snapshot is None:
            return False
        now = self.clock()
        threshold_ns = (max_age_s if max_age_s is not None else self.limits.feedback_max_age_s) * 1e9
        return all(0 <= now - t <= threshold_ns for t in self.last_feedback_snapshot["rx_ns"])
