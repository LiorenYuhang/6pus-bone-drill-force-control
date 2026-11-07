"""Fake UART bus implementation for testing and simulation without real hardware."""

from __future__ import annotations

import threading
import time

try:
    from .protocol import (
        CMD_RD_REGISTER,
        CMD_RD_STATUS,
        CMD_WR_REGISTER,
        parse_frame,
    )
    from .safety import (
    ACTUAL_POSITION_ADDR,
        CONTROL_MODE_ADDR,
        LOWER_LIMIT_ADDR,
        MOTOR_IDS,
        PAUSE_ADDR,
        TARGET_FORCE_ADDR,
        TARGET_POSITION_ADDR,
        TARGET_SPEED_ADDR,
        UPPER_LIMIT_ADDR,
        ReadOnlyModeError,
    )
except ImportError:
    from protocol import (
        CMD_RD_REGISTER,
        CMD_RD_STATUS,
        CMD_WR_REGISTER,
        parse_frame,
    )
    from safety import (
        ACTUAL_POSITION_ADDR,
        CONTROL_MODE_ADDR,
        LOWER_LIMIT_ADDR,
        MOTOR_IDS,
        PAUSE_ADDR,
        TARGET_FORCE_ADDR,
        TARGET_POSITION_ADDR,
        TARGET_SPEED_ADDR,
        UPPER_LIMIT_ADDR,
        ReadOnlyModeError,
    )


class FakeBus:
    """Simulated UART bus emulating response frames for ID1..ID6."""

    def __init__(self, read_only: bool = False) -> None:
        self.read_only = read_only
        self.events: list[tuple[int, int, int, list[int]]] = []
        self.write_audit: list[dict] = []
        self.closed = False
        self.fail_on_id: int | None = None
        self.fail_mode_readback_on_axis: int | None = None
        self.positions = {mid: 50 for mid in MOTOR_IDS}
        self.targets = {mid: 50 for mid in MOTOR_IDS}
        self.target_forces = {mid: 0 for mid in MOTOR_IDS}
        self.actual_forces = {mid: 0 for mid in MOTOR_IDS}
        self.currents = {mid: 0 for mid in MOTOR_IDS}
        self.faults = {mid: 0 for mid in MOTOR_IDS}
        self.limits = {mid: (0, 1800) for mid in MOTOR_IDS}
        self.modes = {mid: 2 for mid in MOTOR_IDS}
        self.speeds = {mid: 50 for mid in MOTOR_IDS}
        self.tx_cmd_counts: dict[int, int] = {}
        self.total_tx_count: int = 0
        self.transaction_count: int = 0
        self.attempt_count: int = 0
        self.retry_count: int = 0
        self.timeout_count: int = 0
        self.checksum_error_count: int = 0
        self.protocol_error_count: int = 0
        self.failed_transaction_count: int = 0
        self._lock = threading.RLock()

    @property
    def lock(self) -> threading.RLock:
        return self._lock

    def request(self, frame: bytes, **kwargs) -> bytes:
        if self.closed:
            raise OSError("FakeBus is closed")

        parsed = parse_frame(frame)
        mid = parsed.motor_id
        cmd = parsed.command
        reg = parsed.register_addr

        if self.read_only and cmd == CMD_WR_REGISTER:
            raise ReadOnlyModeError(
                "Bus-level write protection violation: CMD_WR_REGISTER (0x32) blocked in READ_ONLY mode"
            )

        self.tx_cmd_counts[cmd] = self.tx_cmd_counts.get(cmd, 0) + 1
        self.total_tx_count += 1
        self.transaction_count += 1
        self.attempt_count += 1
        payload = [
            int.from_bytes(parsed.data[i : i + 2], "little")
            for i in range(0, len(parsed.data), 2)
        ]
        if cmd == CMD_WR_REGISTER:
            self.write_audit.append(
                {
                    "timestamp_ns": time.time_ns(),
                    "timestamp_monotonic_ns": time.perf_counter_ns(),
                    "motor_id": mid,
                    "register_address": reg,
                    "payload": list(payload),
                }
            )
        self.events.append((mid, cmd, reg, payload))

        if self.fail_on_id == mid and cmd == CMD_WR_REGISTER:
            raise OSError(f"Simulated write error on ID{mid}")

        if cmd == CMD_RD_REGISTER:
            if reg == LOWER_LIMIT_ADDR:
                val = self.limits[mid][0]
            elif reg == UPPER_LIMIT_ADDR:
                val = self.limits[mid][1]
            elif reg == CONTROL_MODE_ADDR:
                if self.fail_mode_readback_on_axis == mid:
                    val = 0 if self.modes[mid] == 3 else 3
                else:
                    val = self.modes[mid]
            elif reg == TARGET_SPEED_ADDR:
                val = self.speeds[mid]
            elif reg == TARGET_POSITION_ADDR:
                val = self.targets[mid]
            elif reg == ACTUAL_POSITION_ADDR:
                val = self.positions[mid]
            elif reg == 0x2C:
                val = self.actual_forces[mid] & 0xFFFF
            elif reg == TARGET_FORCE_ADDR:
                val = self.target_forces[mid] & 0xFFFF
            else:
                val = 0

            # 10-byte response frame: AA 55 05 mid 31 reg val(2B) checksum
            data = bytes([
                0xAA, 0x55, 0x05, mid, 0x31,
                reg & 0xFF, (reg >> 8) & 0xFF,
                val & 0xFF, (val >> 8) & 0xFF,
            ])
            ck = sum(data[2:]) & 0xFF
            return data + bytes([ck])

        elif cmd == CMD_RD_STATUS:
            tgt = self.targets[mid]
            cur_pos = self.positions[mid]
            cur_mA = self.currents[mid]
            force_g = 0
            force_adc = 0
            temp = 30
            err = self.faults[mid]
            data = bytearray([
                0xAA, 0x55, 0x0F, mid, 0x30, 0x00, 0x00,
                tgt & 0xFF, (tgt >> 8) & 0xFF,
                cur_pos & 0xFF, (cur_pos >> 8) & 0xFF,
                cur_mA & 0xFF, (cur_mA >> 8) & 0xFF,
                force_g & 0xFF, (force_g >> 8) & 0xFF,
                force_adc & 0xFF, (force_adc >> 8) & 0xFF,
                temp & 0xFF,
                err & 0xFF,
            ])
            ck = sum(data[2:]) & 0xFF
            data.append(ck)
            return bytes(data)

        elif cmd == CMD_WR_REGISTER:
            # Model contiguous register blocks used by the verified M6 setup.
            for offset, value in enumerate(payload):
                address = reg + offset
                if address == CONTROL_MODE_ADDR:
                    self.modes[mid] = value
                elif address == TARGET_SPEED_ADDR:
                    self.speeds[mid] = value
                elif address == TARGET_POSITION_ADDR:
                    self.targets[mid] = value
                    self.positions[mid] = value
                elif address == TARGET_FORCE_ADDR:
                    signed_val = int.from_bytes(bytes([value & 0xFF, (value >> 8) & 0xFF]), "little", signed=True)
                    self.target_forces[mid] = signed_val
            # Return the same 0x0F status layout as CMD_RD_STATUS so the
            # formal Motor.write_register() parser is exercised in Mock mode.
            tgt = self.positions[mid]
            cur_pos = self.positions[mid]
            cur_mA = self.currents[mid]
            force_g = self.target_forces.get(mid, 0)
            force_adc = 0
            temp = 30
            err = self.faults[mid]
            data = bytearray([
                0xAA, 0x55, 0x0F, mid, 0x32, reg & 0xFF, (reg >> 8) & 0xFF,
                tgt & 0xFF, (tgt >> 8) & 0xFF,
                cur_pos & 0xFF, (cur_pos >> 8) & 0xFF,
                cur_mA & 0xFF, (cur_mA >> 8) & 0xFF,
                force_g & 0xFF, (force_g >> 8) & 0xFF,
                force_adc & 0xFF, (force_adc >> 8) & 0xFF,
                temp & 0xFF, err & 0xFF,
            ])
            data.append(sum(data[2:]) & 0xFF)
            return bytes(data)

        return bytes([0xAA, 0x55, 0x02, mid, cmd, 0x00, 0x00])

    def close(self) -> None:
        self.closed = True

    def get_telemetry(self) -> dict:
        """Return simulated UART bus telemetry."""
        return {
            "transaction_count": self.transaction_count,
            "attempt_count": self.attempt_count,
            "retry_count": self.retry_count,
            "timeout_count": self.timeout_count,
            "checksum_error_count": self.checksum_error_count,
            "protocol_error_count": self.protocol_error_count,
            "failed_transaction_count": self.failed_transaction_count,
            "tx_cmd_counts": {f"0x{k:02X}": v for k, v in self.tx_cmd_counts.items()},
            "total_tx_count": self.total_tx_count,
            "write_audit": [
                {**record, "payload": list(record["payload"])} for record in self.write_audit
            ],
        }


class MockServoBus(FakeBus):
    """Mock UART bus simulating J7 Servo responses for offline testing."""

    def __init__(self, initial_positions: Sequence[int] | None = None, initial_modes: Sequence[int] | None = None) -> None:
        super().__init__()
        from collections import deque
        self.uart_lock = threading.RLock()
        self._clock_ns = time.monotonic_ns
        self.rx_queue: deque[bytes] = deque()
        self.written_frames: list[bytes] = []
        self.timeout_on_axis: int | None = None
        self.fault_on_axis: int | None = None
        if initial_positions is not None:
            for i, mid in enumerate(MOTOR_IDS):
                self.positions[mid] = int(initial_positions[i])
                self.targets[mid] = int(initial_positions[i])
        if initial_modes is not None:
            for i, mid in enumerate(MOTOR_IDS):
                self.modes[mid] = int(initial_modes[i])

    def write(self, frame: bytes) -> int:
        self.written_frames.append(bytes(frame))
        parsed = parse_frame(frame)
        mid = parsed.motor_id
        if parsed.command == CMD_WR_REGISTER:
            if parsed.register_addr == 0x25:
                mode_val = int.from_bytes(parsed.data, "little", signed=False)
                self.modes[mid] = mode_val
            elif parsed.register_addr == 0x29:
                pos_val = int.from_bytes(parsed.data, "little", signed=True)
                self.positions[mid] = pos_val
                self.targets[mid] = pos_val
            elif parsed.register_addr == TARGET_FORCE_ADDR or parsed.register_addr == 0x27:
                force_val = int.from_bytes(parsed.data, "little", signed=True)
                self.target_forces[mid] = force_val
            elif parsed.register_addr == 0x1A:
                pass
            if self.fault_on_axis is not None and mid == 1:
                self.faults[mid] = self.fault_on_axis
            if self.timeout_on_axis != mid:
                resp = super().request(frame)
                self.rx_queue.append(resp)
        return len(frame)

    def drain_motion_feedbacks(self) -> dict[int, StatusFeedback]:
        feedbacks: dict[int, StatusFeedback] = {}
        try:
            from .protocol import unpack_status_response
        except ImportError:
            from protocol import unpack_status_response  # type: ignore

        while self.rx_queue:
            raw = self.rx_queue.popleft()
            try:
                fb = unpack_status_response(raw)
                mid = raw[3]
                feedbacks[mid] = fb
            except Exception:
                pass
        return feedbacks

    def request(self, frame: bytes, **kwargs) -> bytes:
        self.written_frames.append(bytes(frame))
        parsed = parse_frame(frame)
        mid = parsed.motor_id
        if self.timeout_on_axis == mid:
            raise TimeoutError(f"Mock timeout on ID{mid}")
        if parsed.command == CMD_RD_STATUS and self.fault_on_axis is not None and mid == 1:
            self.faults[mid] = self.fault_on_axis
        return super().request(frame)

    def read_available(self) -> bytes:
        data = b"".join(self.rx_queue)
        self.rx_queue.clear()
        return data
