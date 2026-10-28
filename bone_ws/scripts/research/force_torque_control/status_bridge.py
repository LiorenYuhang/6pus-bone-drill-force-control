"""Raw status -> explicitly qualified force. Does not open or own a port."""
import math
import time
from pus6_motor_driver.protocol import (parse_frame, unpack_status_response,
    pack_read_status_cmd, CMD_RD_STATUS)
from core import STANDARD_GRAVITY


def gf_to_newtons(gf):
    if not math.isfinite(float(gf)): raise ValueError('nonfinite gram-force')
    return float(gf) * STANDARD_GRAVITY / 1000.0


def newtons_to_gf(force):
    if not math.isfinite(float(force)): raise ValueError('nonfinite newtons')
    return float(force) * 1000.0 / STANDARD_GRAVITY


class StatusAssembler:
    """Six independently timestamped status responses. Source provenance is explicit."""
    def __init__(self, source, gram_force_confirmed=False, max_age_s=.2):
        if source not in (1, 2): raise ValueError('explicit mock or hardware source required')
        if not math.isfinite(max_age_s) or max_age_s<=0: raise ValueError('invalid max age')
        self.source=source; self.confirmed=gram_force_confirmed; self.max_age=max_age_s
        self.rows=[None]*6
    def ingest(self, axis, source, raw, rx_ns, now_ns):
        if source != self.source: raise ValueError('source mismatch')
        if axis not in range(1,7): raise ValueError('axis must be 1..6')
        if rx_ns<=0 or rx_ns>now_ns: raise ValueError('invalid receipt timestamp')
        frame=parse_frame(bytes(raw))
        if frame.motor_id != axis or frame.command != CMD_RD_STATUS:
            raise ValueError('status axis or command mismatch')
        status=unpack_status_response(bytes(raw))
        old=self.rows[axis-1]
        if old is not None and rx_ns<=old[0]: raise ValueError('out of order status')
        self.rows[axis-1]=(int(rx_ns),status)
    def snapshot(self,now_ns):
        values=[]; stamps=[]; valid=[]; faults=[]
        for row in self.rows:
            if row is None:
                values.append(0.); stamps.append(0); valid.append(False); faults.append(0); continue
            t,status=row
            ok=self.confirmed and 0<=now_ns-t<=self.max_age*1e9 and status.error_code==0
            values.append(gf_to_newtons(status.force_g) if self.confirmed else 0.)
            stamps.append(t); valid.append(ok); faults.append(status.error_code)
        return values, stamps, valid, faults


class ReadOnlyStatusSource:
    """Adapter for an already-owned transport. Caller schedules I/O off ROS callbacks.

    No construction or opening of serial transport and no write or retry API here.
    Do not attach alongside the J14 owner: integrate it into the single UART owner.
    """
    def __init__(self,bus,clock=time.monotonic_ns): self.bus=bus; self.clock=clock
    def poll(self,axis):
        if axis not in range(1,7): raise ValueError('axis must be 1..6')
        frame=self.bus.request(pack_read_status_cmd(axis), expected_id=axis, expected_command=CMD_RD_STATUS)
        rx=self.clock()
        parsed=parse_frame(frame)
        if parsed.motor_id!=axis or parsed.command!=CMD_RD_STATUS: raise ValueError('wrong status response')
        unpack_status_response(frame)
        return axis,bytes(frame),rx


class LafForceAdapter:
    """Candidate force protocol over a caller-owned bus; never opens a port.

    Not wired into the research ROS entry or qualified for real operation. Caller
    must separately establish transport ownership and authorize mode switching.
    """
    def __init__(self, bus, axis_limits_n, *, gram_force_confirmed=False):
        from core import vector
        from pus6_motor_driver.motor import Motor
        limits=vector(axis_limits_n,'axis_limits_n')
        if not gram_force_confirmed: raise ValueError('meaning of manual g not confirmed')
        if any(v<=0 or newtons_to_gf(v)>32767 for v in limits): raise ValueError('invalid axis force limit')
        self.motors=[Motor(bus,i) for i in range(1,7)]
        self.limits=limits; self.entered=False; self.fault=None; self.pause_attempted=False
    def _pause_once(self):
        if self.pause_attempted: return []
        self.pause_attempted=True; errors=[]
        for m in self.motors:
            try:
                status=m.write_register(0x1a,[1])
                if status.error_code: raise RuntimeError('pause status fault')
            except Exception as exc: errors.append(str(exc))
        return errors
    def _trip(self,exc):
        self.fault=str(exc); self.entered=False
        errors=self._pause_once()
        if errors: self.fault += '; pause unconfirmed: '+', '.join(errors)
    def set_force_mode_explicit(self):
        if self.fault or self.entered: raise RuntimeError('mode cannot be entered')
        try:
            for m in self.motors:
                if m.write_register(0x25,[3]).error_code: raise RuntimeError('mode status fault')
            self.entered=True
        except Exception as exc:
            self._trip(exc); raise
    def write_force(self,command):
        from core import AxialForceN
        if type(command) is not AxialForceN: raise TypeError('axial force N required')
        if not self.entered or self.fault: raise RuntimeError('force mode not entered')
        if any(v<0 or v>lim for v,lim in zip(command.values,self.limits)):
            raise ValueError('force outside configured axis limits')
        raw=[round(newtons_to_gf(v)) for v in command.values]
        try:
            for m,value in zip(self.motors,raw):
                if m.write_register(0x27,[value]).error_code: raise RuntimeError('force status fault')
        except Exception as exc:
            self._trip(exc); raise
    def stop(self):
        self.entered=False
        errors=self._pause_once()
        if errors: self.fault='pause unconfirmed: '+', '.join(errors); raise RuntimeError(self.fault)
