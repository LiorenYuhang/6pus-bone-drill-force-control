"""Six-axis force-position research controller. No driver or ROS dependencies.

All force calibration and gains must come from the caller. This module does not
establish their physical validity, motor ownership, or a safe machine state.
"""
from dataclasses import dataclass
from math import isfinite


def vec(v, name):
    if len(v) != 6 or not all(isfinite(float(x)) for x in v):
        raise ValueError(f'{name}: expected six finite values')
    return tuple(float(x) for x in v)


@dataclass(frozen=True)
class Config:
    kp: tuple                 # N/m
    ki: tuple                 # N/(m s)
    kd: tuple                 # N s/m
    force_limit_n: tuple       # N, symmetric
    admittance_m_per_ns: tuple  # m/(N s), sign includes mechanics and sensor direction
    correction_limit_m: tuple # m, symmetric
    correction_rate_m_s: tuple # m/s, symmetric
    raw_zero: tuple           # calibrated 0x2C zero, raw units
    newtons_per_raw: tuple    # calibrated N/raw, signed
    max_dt_s: float
    max_age_s: float

    def __post_init__(self):
        for name in ('kp','ki','kd','force_limit_n','admittance_m_per_ns',
                     'correction_limit_m','correction_rate_m_s','raw_zero','newtons_per_raw'):
            object.__setattr__(self,name,vec(getattr(self,name),name))
        if any(x < 0 for n in ('kp','ki','kd') for x in getattr(self,n)):
            raise ValueError('negative PID gain')
        if any(x <= 0 for n in ('force_limit_n','correction_limit_m','correction_rate_m_s') for x in getattr(self,n)):
            raise ValueError('limits must be positive')
        if any(x == 0 for x in self.admittance_m_per_ns + self.newtons_per_raw):
            raise ValueError('calibration and admittance must be nonzero')
        if not all(isfinite(x) and x > 0 for x in (self.max_dt_s,self.max_age_s)):
            raise ValueError('invalid timing')


class ForcePositionController:
    """Explicit Euler integral admittance: delta_q_dot = A * (F_cmd - F_actual).

    First sample initializes state only. Software limits cannot stop a real drive.
    Timestamps are six host-monotonic receipt times, not device sample times.
    """
    def __init__(self, config):
        self.config=config
        self.last_ns=None
        self.prev_error=None
        self.integral=[0.]*6
        self.correction=[0.]*6
        self.fault=None
        self.state=None

    def update(self, desired_m, actual_m, raw_force, feedback_ns, valid, faults, now_ns):
        if self.fault:
            raise RuntimeError(self.fault)
        try:
            desired=vec(desired_m,'desired_m'); actual=vec(actual_m,'actual_m')
            raw=vec(raw_force,'raw_force')
            if len(feedback_ns)!=6 or len(valid)!=6 or len(faults)!=6 or not all(v is True for v in valid) or any(faults):
                raise ValueError('invalid feedback or drive fault')
            if not isinstance(now_ns,int) or any(not isinstance(t,int) or t<=0 or t>now_ns or (now_ns-t)*1e-9>self.config.max_age_s for t in feedback_ns):
                raise ValueError('invalid or stale feedback time')
            e=[desired[i]-actual[i] for i in range(6)]
            measured=[(raw[i]-self.config.raw_zero[i])*self.config.newtons_per_raw[i] for i in range(6)]
            if not all(isfinite(v) for v in e+measured):
                raise ValueError('nonfinite input')
            if self.last_ns is None:
                self.last_ns=tuple(feedback_ns); self.prev_error=e
                self.state={'primed':True,'error_m':e,'force_actual_n':measured}
                return None
            dts=[(feedback_ns[i]-self.last_ns[i])*1e-9 for i in range(6)]
            if any(not 0<dt<=self.config.max_dt_s for dt in dts):
                raise ValueError('invalid feedback interval')
            integral=[]; correction=[]; commands=[]; force_cmd=[]; force_error=[]
            for i,dt in enumerate(dts):
                c=self.config
                d=(e[i]-self.prev_error[i])/dt
                integ=self.integral[i]+e[i]*dt
                raw_cmd=c.kp[i]*e[i]+c.ki[i]*integ+c.kd[i]*d
                if (raw_cmd>c.force_limit_n[i] and e[i]>0) or (raw_cmd < -c.force_limit_n[i] and e[i]<0):
                    integ=self.integral[i]
                    raw_cmd=c.kp[i]*e[i]+c.ki[i]*integ+c.kd[i]*d
                force=max(-c.force_limit_n[i],min(c.force_limit_n[i],raw_cmd))
                ferr=force-measured[i]
                velocity=max(-c.correction_rate_m_s[i],min(c.correction_rate_m_s[i],c.admittance_m_per_ns[i]*ferr))
                dq=max(-c.correction_limit_m[i],min(c.correction_limit_m[i],self.correction[i]+velocity*dt))
                command=desired[i]+dq
                if not all(isfinite(x) for x in (d,integ,raw_cmd,force,ferr,velocity,dq,command)):
                    raise ValueError('nonfinite control state')
                integral.append(integ); correction.append(dq); commands.append(command)
                force_cmd.append(force); force_error.append(ferr)
            self.integral=integral; self.correction=correction
            self.last_ns=tuple(feedback_ns); self.prev_error=e
            self.state={'primed':False,'error_m':e,'force_cmd_n':force_cmd,
                        'force_actual_n':measured,'force_error_n':force_error,
                        'correction_m':correction[:],'position_command_m':commands[:]}
            return tuple(commands)
        except (ValueError,TypeError,OverflowError) as exc:
            self.fault=str(exc)
            raise RuntimeError(self.fault) from exc
