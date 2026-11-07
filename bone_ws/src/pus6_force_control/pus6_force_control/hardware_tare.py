"""Explicit device Tare through the existing locked UART owner; never on startup."""
from dataclasses import asdict, dataclass, field
import math
import time

from pus6_motor_driver.protocol import (
    CMD_RD_REGISTER, CMD_WR_REGISTER, CMD_SAVE_SUCCESS, pack_read_register_cmd,
    pack_write_register_cmd, unpack_read_register_response, unpack_write_response,
    validate_response, is_save_success_response,
)


@dataclass
class AxisTareState:
    motor_id: int
    state: str = 'not_requested'
    write_attempted: bool = False
    before_force_g: int | None = None
    after_forces_g: list[int] = field(default_factory=list)
    position_before: int | None = None
    position_after: int | None = None
    fault_after: int | None = None
    ack_hex: str | None = None
    completion_hex: str | None = None
    completion_register: int | None = None
    completion_latency_s: float | None = None
    error: str | None = None


class HardwareTareController:
    """One-shot per-axis Tare, serialised against the existing feedback/writer.

    Current-device vendor evidence: write 0x09=1, ACK then asynchronous 0x40.
    Only an explicit call can alter the device reference. A failed attempted Tare
    is not retried, including a later tare_all() call on this same owner.
    """
    def __init__(self, bus, operation_lock, *, hardware_write_enabled=False,
                 hardware_qualified=False, dry_run=False, event=None):
        self.bus = bus
        self.lock = operation_lock
        self.hardware_write_enabled = hardware_write_enabled
        self.hardware_qualified = hardware_qualified
        self.dry_run = dry_run
        self.event = event or (lambda *_: None)
        self.axes = {mid: AxisTareState(mid) for mid in range(1, 7)}
        self.failed = False

    @staticmethod
    def signed(word):
        return word - 65536 if word >= 32768 else word

    def read(self, mid, reg):
        raw = self.bus.request(pack_read_register_cmd(mid, reg), expected_id=mid,
                               expected_command=CMD_RD_REGISTER, retries=0)
        return unpack_read_register_response(raw, reg, 1).values[0]

    def _state_dict(self, row: AxisTareState) -> dict:
        return asdict(row)

    def snapshot(self):
        return {mid: self._state_dict(row) for mid, row in self.axes.items()}

    def _confirm(self, row, samples, tolerance_g, sample_interval_s, position_tolerance_steps):
        row.after_forces_g = []
        for _ in range(samples):
            row.after_forces_g.append(self.signed(self.read(row.motor_id, 0x2C)))
            if sample_interval_s: time.sleep(sample_interval_s)
        row.position_after = self.read(row.motor_id, 0x2A)
        row.fault_after = self.read(row.motor_id, 0x2F)
        if row.fault_after:
            raise RuntimeError(f'Device fault after Tare: {row.fault_after}')
        if self.read(row.motor_id, 0x25) != 0:
            raise RuntimeError('Mode changed during Tare')
        if abs(row.position_after - row.position_before) > position_tolerance_steps:
            raise RuntimeError('Position moved during Tare')
        if any(abs(f) > tolerance_g for f in row.after_forces_g):
            raise RuntimeError('Device force is outside Tare confirmation tolerance')
        row.state = 'confirmed'
        row.error = None

    def verify_completed_axis(self, prior, *, samples=20, tolerance_g=50,
                              sample_interval_s=.01, position_tolerance_steps=2):
        """Explicit read-only reconciliation after a recorded completion, NEVER a retry.

        An ACK alone or missing completion cannot authorize continuation.
        """
        row = AxisTareState(**prior)
        if (samples < 2 or not math.isfinite(tolerance_g) or tolerance_g < 0
                or not math.isfinite(sample_interval_s) or sample_interval_s < 0
                or position_tolerance_steps < 0):
            raise ValueError('invalid read-only Tare confirmation settings')
        if row.motor_id not in self.axes or not row.write_attempted or not row.completion_hex:
            raise RuntimeError('No recorded completed Tare to reconcile')
        if not is_save_success_response(bytes.fromhex(row.completion_hex), expected_id=row.motor_id):
            raise RuntimeError('Invalid recorded Tare completion')
        with self.lock:
            if self.failed or self.axes[row.motor_id].write_attempted:
                raise RuntimeError('Reconciliation requires a fresh explicit maintenance owner')
            self.axes[row.motor_id] = row
            row.state = 'verifying_prior_completion'
            try:
                self._confirm(row, samples, tolerance_g, sample_interval_s, position_tolerance_steps)
                self.event('hardware_tare_prior_completion_verified_without_write', self._state_dict(row))
                return self._state_dict(row)
            except Exception as exc:
                row.state='failed';row.error=str(exc);self.failed=True
                raise

    def tare_axis(self, motor_id, *, samples=20, tolerance_g=50,
                  completion_timeout_s=1.0, sample_interval_s=.01,
                  position_tolerance_steps=2, completion_settle_s=.1):
        if motor_id not in self.axes or isinstance(motor_id, bool):
            raise ValueError('Tare requires an individual motor ID 1..6')
        if samples < 2 or not math.isfinite(tolerance_g) or tolerance_g < 0:
            raise ValueError('invalid Tare confirmation settings')
        if (not all(math.isfinite(v) for v in (completion_timeout_s,sample_interval_s,completion_settle_s))
                or completion_timeout_s <= 0 or sample_interval_s < 0
                or position_tolerance_steps < 0 or completion_settle_s < 0):
            raise ValueError('invalid Tare timing/position tolerance')
        with self.lock:
            row = self.axes[motor_id]
            if self.failed:
                raise RuntimeError('Tare fault latched; no subsequent axis or retry permitted')
            if row.state == 'confirmed':
                return self._state_dict(row)  # idempotent: no second Tare transmission
            if row.write_attempted:
                raise RuntimeError('Tare already attempted; do not resend')
            if self.dry_run or not self.hardware_write_enabled or not self.hardware_qualified:
                raise RuntimeError('Tare requires non-dry-run, write authorization and hardware qualification')
            if not callable(getattr(self.bus, 'wait_for_response', None)):
                raise RuntimeError('existing UART owner must support asynchronous completion waits')
            try:
                if self.read(motor_id, 0x25) != 0:
                    raise RuntimeError('Tare is allowed only in confirmed Mode 0')
                row.position_before = self.read(motor_id, 0x2A)
                row.before_force_g = self.signed(self.read(motor_id, 0x2C))
                row.state = 'write_attempted'
                row.write_attempted = True  # set BEFORE TX; a lost ACK never permits retry
                start = time.monotonic()
                ack = self.bus.request(pack_write_register_cmd(motor_id, 0x09, [1]),
                                       expected_id=motor_id, expected_command=CMD_WR_REGISTER,
                                       retries=0)
                row.ack_hex = ack.hex(' ')
                frame = validate_response(ack, expected_id=motor_id, expected_command=CMD_WR_REGISTER)
                if frame.register_addr != 0x09 or unpack_write_response(ack).error_code:
                    raise RuntimeError('Tare write ACK register/fault mismatch')
                row.state = 'awaiting_completion'
                done = self.bus.wait_for_response(expected_id=motor_id,
                    expected_command=CMD_SAVE_SUCCESS, timeout_s=completion_timeout_s)
                completion = validate_response(done, expected_id=motor_id, expected_command=CMD_SAVE_SUCCESS)
                row.completion_hex = done.hex(' ')
                row.completion_register = completion.register_addr
                row.completion_latency_s = time.monotonic() - start
                if not is_save_success_response(done, expected_id=motor_id):
                    raise RuntimeError('Unexpected Tare completion register/layout')
                # Real Stage 3B-3: 0x40/0x1C precedes query readiness; immediate
                # queries can be ignored. Do not interleave or resend the write.
                if completion_settle_s: time.sleep(completion_settle_s)
                self._confirm(row, samples, tolerance_g, sample_interval_s, position_tolerance_steps)
                self.event('hardware_tare_confirmed', self._state_dict(row))
                return self._state_dict(row)
            except Exception as exc:
                row.state = 'failed'
                row.error = str(exc)
                self.failed = True
                self.event('hardware_tare_failed', self._state_dict(row))
                raise

    def tare_all_axes(self, **settings):
        # Exactly one owner and one in-flight axis; exceptions stop this loop.
        return [self.tare_axis(mid, **settings) for mid in range(1, 7)]
