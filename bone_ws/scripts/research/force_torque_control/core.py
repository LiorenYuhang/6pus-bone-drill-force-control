"""Offline force/torque research: explicit units and in-process mock ownership."""
from dataclasses import dataclass
from enum import Enum
from math import isfinite
from numbers import Integral
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'joint_torque_pid'))
from controller import JointTorquePID, Fault, vector
from pus6_motor_driver.fake_bus import FakeBus
from pus6_motor_driver.motor import Motor
from pus6_motor_driver.protocol import parse_frame, CMD_RD_STATUS, CMD_WR_REGISTER

STANDARD_GRAVITY = 9.80665  # Assumes manual's g means gram-force; calibration unverified.
FORCE_TARGET_REGISTER = 0x27
MODE_REGISTER = 0x25
PAUSE_REGISTER = 0x1A


class Mode(Enum):
    IDLE='IDLE'
    PID_TORQUE='PID_TORQUE'
    FF_PID_TORQUE='FF_PID_TORQUE'
    ACTUATOR_FORCE_CONTROL='ACTUATOR_FORCE_CONTROL'
    DS_SCTDE='DS_SCTDE'
    FAULT='FAULT'
    STOPPED='STOPPED'


@dataclass(frozen=True)
class TorqueNm:
    values: tuple
    def __post_init__(self): object.__setattr__(self,'values',vector(self.values,'torque_nm'))


@dataclass(frozen=True)
class AxialForceN:
    values: tuple
    def __post_init__(self): object.__setattr__(self,'values',vector(self.values,'force_n'))


@dataclass(frozen=True)
class MeasuredForce:
    values: tuple
    rx_ns: tuple
    valid: tuple
    faults: tuple


class MockForceBus(FakeBus):
    """Fake serial protocol responses only; never opens a device."""
    def __init__(self):
        super().__init__(); self.forces_g = {i:0 for i in range(1,7)}; self.hardware_write_count=0
    def request(self, frame, **kwargs):
        response=bytearray(super().request(frame,**kwargs)); req=parse_frame(frame)
        if req.command in (CMD_RD_STATUS,CMD_WR_REGISTER) and len(response)==20:
            response[13:15]=int(self.forces_g[req.motor_id]).to_bytes(2,'little',signed=True)
            response[-1]=sum(response[2:-1]) & 255
        return bytes(response)


class MockTorqueBackend:
    """No known real motor torque protocol. In-memory interface only."""
    def __init__(self): self.commands=[]; self.stops=0; self.hardware_write_count=0
    def write(self, command):
        if type(command) is not TorqueNm: raise TypeError('motor torque in N m required')
        self.commands.append(command)
    def stop(self): self.stops+=1


class ForceProtocolMock:
    """LAF 0x25 mode 3 / 0x27 gram-force candidate, FakeBus only.

    No serial factory, no real transport allowed; physical limits, zero/sign,
    conversion and mode/stop behavior remain hardware-unverified.
    """
    def __init__(self,bus,max_force_n):
        if type(bus) is not MockForceBus: raise TypeError('only MockForceBus accepted')
        if not isfinite(max_force_n) or max_force_n<=0 or max_force_n>32767*STANDARD_GRAVITY/1000:
            raise ValueError('invalid research force cap')
        self.bus=bus; self.max_force_n=max_force_n; self.motors=[Motor(bus,i) for i in range(1,7)]
        self.stopped=False
    def enter(self):
        if self.stopped: raise RuntimeError('backend stopped')
        for m in self.motors:
            result=m.write_register(MODE_REGISTER,[3])
            if result.error_code: raise RuntimeError('mock mode error')
    def read_feedback(self,clock=time.monotonic_ns):
        samples=[]; stamps=[]; faults=[]
        for m in self.motors:
            status=m.read_status(); stamps.append(clock()); samples.append(status.force_g*STANDARD_GRAVITY/1000)
            faults.append(status.error_code)
        return MeasuredForce(tuple(samples),tuple(stamps),(True,)*6,tuple(faults))
    def write(self,command):
        if type(command) is not AxialForceN: raise TypeError('axial force in N required')
        if self.stopped: raise RuntimeError('backend stopped')
        if any(v<0 or v>self.max_force_n for v in command.values): raise ValueError('force outside mock limits')
        # Manual only specifies g and a positive example; do not infer negative commands.
        vals=[round(v*1000/STANDARD_GRAVITY) for v in command.values]
        for m,raw in zip(self.motors,vals):
            status=m.write_register(FORCE_TARGET_REGISTER,[raw])
            if status.error_code: raise RuntimeError('mock force write error')
    def stop(self):
        if self.stopped: return
        self.stopped=True
        errors=[]
        for m in self.motors:
            try: m.write_register(PAUSE_REGISTER,[1])
            except Exception as exc: errors.append(str(exc))
        if errors: raise RuntimeError('mock pause unconfirmed: '+'; '.join(errors))


class ForceOuterLoop:
    """Position P -> positive axial force reference; actuator owns inner force loop."""
    def __init__(self,kp_n_per_m,max_force_n,slew_n_s,direction,max_age_s):
        self.kp=vector(kp_n_per_m,'kp_n_per_m'); self.limit=vector(max_force_n,'max_force_n')
        self.slew=vector(slew_n_s,'slew_n_s'); self.direction=vector(direction,'direction')
        if any(x<0 for x in self.kp) or any(x<=0 for x in self.limit+self.slew) or any(x not in (-1,1) for x in self.direction): raise ValueError('invalid force configuration')
        if not isfinite(max_age_s) or max_age_s<=0: raise ValueError('invalid max age')
        self.max_age=max_age_s; self.previous_t=None; self.previous=[0.]*6; self.state=None
    def update(self,desired,actual,position_ns,force,now_ns):
        desired=vector(desired,'desired_m'); actual=vector(actual,'actual_m')
        if type(force) is not MeasuredForce: raise ValueError('force feedback required')
        values=vector(force.values,'measured_force_n')
        if len(position_ns)!=6 or len(force.rx_ns)!=6 or len(force.valid)!=6 or len(force.faults)!=6 or any(force.faults) or not all(v is True for v in force.valid): raise ValueError('force invalid or fault')
        for t in tuple(position_ns)+tuple(force.rx_ns):
            if not isinstance(t,Integral) or t<=0 or t>now_ns or (now_ns-t)*1e-9>self.max_age: raise ValueError('feedback stale')
        if self.previous_t is None:
            self.previous_t=now_ns; self.state={'primed':True}; return None
        dt=(now_ns-self.previous_t)*1e-9
        if dt<=0 or dt>self.max_age: raise ValueError('invalid control interval')
        e=[desired[i]-actual[i] for i in range(6)]; requested=[]; saturated=[]
        for i in range(6):
            raw=self.direction[i]*self.kp[i]*e[i]
            target=max(0.,min(self.limit[i],raw))
            out=max(self.previous[i]-self.slew[i]*dt,min(self.previous[i]+self.slew[i]*dt,target))
            requested.append(out); saturated.append(out != raw)
        self.previous_t=now_ns; self.previous=requested
        self.state={'primed':False,'error_m':e,'force_ref_n':requested[:], 'force_meas_n':list(values),
                    'force_error_n':[requested[i]-values[i] for i in range(6)], 'saturated':saturated}
        return AxialForceN(tuple(requested))


class ResearchSession:
    """One selected mock controller; not interprocess physical device ownership."""
    def __init__(self,torque_pid,force_loop,torque_backend,force_backend):
        self.mode=Mode.IDLE; self.pid=torque_pid; self.force_loop=force_loop
        self.torque=torque_backend; self.force=force_backend; self.reason=None; self.stop_error=None
    def select(self,mode):
        if self.mode is not Mode.IDLE: raise RuntimeError('mode already selected')
        if mode not in (Mode.PID_TORQUE,Mode.FF_PID_TORQUE,Mode.ACTUATOR_FORCE_CONTROL): raise RuntimeError('mode unavailable')
        self.mode=mode
        if mode is Mode.ACTUATOR_FORCE_CONTROL:
            try: self.force.enter()
            except Exception as exc: self.fail(exc); raise
    def fail(self,reason):
        if self.mode is Mode.FAULT: return
        previous=self.mode; self.mode=Mode.FAULT; self.reason=str(reason)
        try:
            if previous is Mode.ACTUATOR_FORCE_CONTROL: self.force.stop()
            elif previous in (Mode.PID_TORQUE,Mode.FF_PID_TORQUE): self.torque.stop()
        except Exception as exc: self.stop_error=str(exc)
    def stop(self):
        if self.mode in (Mode.STOPPED, Mode.FAULT, Mode.IDLE): return
        previous=self.mode
        try:
            if previous is Mode.ACTUATOR_FORCE_CONTROL: self.force.stop()
            elif previous in (Mode.PID_TORQUE,Mode.FF_PID_TORQUE): self.torque.stop()
            self.mode=Mode.STOPPED
        except Exception as exc:
            self.mode=Mode.FAULT; self.reason='mock stop failed'; self.stop_error=str(exc)
            raise
    def cycle(self,desired,actual,timestamps,faults,now_ns,force_feedback=None,feedforward=None):
        try:
            if self.mode is Mode.PID_TORQUE:
                out=self.pid.update(desired,actual,timestamps,now_ns,faults)
                if self.pid.state['primed']: return None
                cmd=TorqueNm(tuple(out)); self.torque.write(cmd); return cmd
            if self.mode is Mode.FF_PID_TORQUE:
                if feedforward is None: raise ValueError('torque feedforward required')
                values, stamps = feedforward
                out=self.pid.update(desired,actual,timestamps,now_ns,faults,values,stamps)
                if self.pid.state['primed']: return None
                cmd=TorqueNm(tuple(out)); self.torque.write(cmd); return cmd
            if self.mode is Mode.ACTUATOR_FORCE_CONTROL:
                if len(faults)!=6 or any(faults): raise ValueError('position feedback fault')
                cmd=self.force_loop.update(desired,actual,timestamps,force_feedback,now_ns)
                if cmd is not None: self.force.write(cmd)
                return cmd
            raise RuntimeError('mode not selected or faulted')
        except (Fault,ValueError,TypeError,OSError,RuntimeError) as exc:
            self.fail(exc); raise
