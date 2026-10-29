"""One-axis, explicitly bounded position-mode transport. No serial factory.

This is separate from the N m PID and force control interfaces. The caller
owns the UART and supplies the physical displacement bound; creation and
preflight perform read-only queries. Motion/stop methods perform writes.
"""
from dataclasses import dataclass
from numbers import Integral
import time
from pus6_motor_driver.motor import Motor

@dataclass(frozen=True)
class PositionStepTarget:
    axis: int
    steps: int
    def __post_init__(self):
        if not isinstance(self.axis,Integral) or self.axis not in range(1,7): raise ValueError('axis must be 1..6')
        if not isinstance(self.steps,Integral): raise ValueError('position target must be integer steps')

class PositionProbe:
    """One command only; never retries and never changes drive control mode."""
    def __init__(self,bus,axis,max_delta_steps,event):
        if not isinstance(axis,Integral) or axis not in range(1,7): raise ValueError('axis must be 1..6')
        if not isinstance(max_delta_steps,Integral) or max_delta_steps<=0: raise ValueError('explicit positive step bound required')
        self.motor=Motor(bus,int(axis)); self.axis=int(axis); self.bound=int(max_delta_steps); self.event=event
        self.anchor=None; self.lower=None; self.upper=None; self.issued=False; self.stop_attempted=False; self.fault=None
    def _record(self,kind,**data): self.event({'kind':kind,'host_monotonic_ns':time.monotonic_ns(),**data})
    def preflight(self):
        if self.anchor is not None or self.issued or self.fault: raise RuntimeError('session already used')
        status=self.motor.read_status(); rx=time.monotonic_ns()
        mode=self.motor.read_register(0x25).values[0]
        upper=self.motor.read_register(0x23).values[0]; lower=self.motor.read_register(0x24).values[0]
        if status.error_code or mode!=0 or not lower<=status.current_position<=upper:
            raise RuntimeError('preflight failed: fault, non-position mode, or outside drive limits')
        self.anchor=int(status.current_position); self.lower=int(lower); self.upper=int(upper)
        self._record('preflight',axis=self.axis,position_steps=self.anchor,mode=mode,upper=upper,lower=lower,
                     feedback_rx_ns=rx,fault=status.error_code,force_g_raw=status.force_g,force_adc=status.force_adc)
        return status
    def command(self,target):
        if type(target) is not PositionStepTarget: raise TypeError('position steps required; torque/force are not accepted')
        if self.anchor is None or self.issued or self.fault: raise RuntimeError('position session not ready or already used')
        if target.axis!=self.axis or abs(target.steps-self.anchor)>self.bound or target.steps==self.anchor:
            raise ValueError('target axis or displacement outside explicitly selected bound')
        if not self.lower<=target.steps<=self.upper: raise ValueError('target outside configured drive limits')
        current=self.motor.read_status()
        self._record('pre_command_feedback',axis=self.axis,position_steps=current.current_position,fault=current.error_code)
        if current.error_code or current.current_position != self.anchor:
            raise RuntimeError('axis moved or faulted after preflight; no command issued')
        self.issued=True
        self._record('position_write_attempt',axis=self.axis,register='0x29',target_steps=target.steps)
        try:
            reply=self.motor.write_register(0x29,[int(target.steps)])
            self._record('position_write_response',position_steps=reply.current_position,fault=reply.error_code)
            if reply.error_code: raise RuntimeError('drive fault in write response')
            return reply
        except Exception as exc:
            self.fault=str(exc); self._record('position_write_error',error=self.fault); raise
    def read(self):
        if self.anchor is None: raise RuntimeError('preflight required')
        s=self.motor.read_status()
        self._record('feedback',axis=self.axis,position_steps=s.current_position,current_ma=s.current_mA,
                     force_g_raw=s.force_g,force_adc=s.force_adc,fault=s.error_code)
        if s.error_code: self.fault='drive fault'; raise RuntimeError(self.fault)
        return s
    def pause_once(self):
        if not self.issued or self.stop_attempted: return None
        self.stop_attempted=True
        # A full or failed log device must not prevent the one pause attempt.
        try: self._record('pause_write_attempt',axis=self.axis,register='0x1a')
        except Exception: pass
        try:
            reply=self.motor.write_register(0x1a,[1])
            self._record('pause_response',fault=reply.error_code,position_steps=reply.current_position)
            if reply.error_code: raise RuntimeError('drive fault in pause response')
            return reply
        except Exception as exc:
            self.fault=str(exc); self._record('pause_error',error=self.fault); raise
