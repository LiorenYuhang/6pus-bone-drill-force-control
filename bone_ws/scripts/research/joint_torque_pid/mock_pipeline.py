"""In-process mock only. No serial backend, interprocess lock or hardware stop."""
from enum import Enum
import struct
from math import isfinite
from controller import Fault, vector


class Mode(Enum):
    IDLE = 'idle'
    POSITION = 'position'
    TORQUE = 'torque'
    FAULT = 'fault'


class MockBackend:
    """Six big-endian float64 values, solely a test format, not a device protocol."""
    def __init__(self):
        self.frames = []
        self.stops = 0
        self.fail_write = False
        self.fail_stop = False
        self.hardware_write_count = 0

    def write(self, torque_nm):
        if self.fail_write:
            raise OSError('mock communication failure')
        self.frames.append(struct.pack('!6d', *vector(torque_nm, 'torque_nm')))

    def stop(self):
        self.stops += 1
        if self.fail_stop:
            raise OSError('mock stop failure')


class MockPipeline:
    def __init__(self, pid, backend):
        if type(backend) is not MockBackend:
            raise TypeError('only the in-memory mock backend is supported')
        self.pid, self.backend = pid, backend
        self.mode = Mode.IDLE
        self.reason = None
        self.stop_error = None

    def enter(self, mode):
        if self.mode is not Mode.IDLE or mode not in (Mode.POSITION, Mode.TORQUE):
            raise RuntimeError('mode transition requires idle')
        self.mode = mode

    def stop(self, reason):
        if self.mode is Mode.FAULT:
            return
        self.mode, self.reason = Mode.FAULT, str(reason)
        try:
            self.backend.stop()
        except OSError as exc:
            self.stop_error = str(exc)

    def torque_cycle(self, desired, actual, feedback_ns, now_ns, faults):
        if self.mode is not Mode.TORQUE:
            raise RuntimeError('torque mode is not active')
        try:
            output = self.pid.update(desired, actual, feedback_ns, now_ns, faults)
            # First sample primes state; do not send a pretend command.
            if not self.pid.state['primed']:
                self.backend.write(output)
            return output
        except (Fault, OSError) as exc:
            self.stop(exc)
            raise

    def position_cycle(self):
        if self.mode is not Mode.POSITION:
            raise RuntimeError('position mode is not active')
        # Position baseline is deliberately not emulated or connected here.

    def check_timeout(self, now_ns, last_target_ns, last_feedback_ns, max_age_s):
        if self.mode is not Mode.TORQUE:
            raise RuntimeError('torque mode is not active')
        if not isfinite(max_age_s) or max_age_s <= 0 or any(t <= 0 or t > now_ns or (now_ns-t)*1e-9 > max_age_s
                                 for t in (last_target_ns, last_feedback_ns)):
            self.stop('target or feedback timeout')
            raise Fault(self.reason)
