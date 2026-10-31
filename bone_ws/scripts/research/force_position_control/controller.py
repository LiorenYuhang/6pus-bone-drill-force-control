"""Six-axis force-position research controller. No driver or ROS dependencies.

All force calibration or relative-zero references and gains come from the caller. This module does not
establish their physical validity, motor ownership, or a safe machine state.
"""
from dataclasses import dataclass
from math import isfinite


def vec(v, name):
    if len(v) != 6 or not all(isfinite(float(x)) for x in v):
        raise ValueError(f'{name}: expected six finite values')
    return tuple(float(x) for x in v)


def position_bounds_from_geometry(g, lower_steps=None, upper_steps=None):
    """Original absolute step range intersected with stroke and drive limits.

    Legacy normal-motion/software planning envelopes do not define the stroke.
    """
    lower=[]; upper=[]
    for i in range(6):
        lo=g.absolute_raw_min_steps
        hi=g.absolute_raw_max_steps
        if lower_steps is not None: lo=max(lo,lower_steps[i])
        if upper_steps is not None: hi=min(hi,upper_steps[i])
        lower.append(max(g.stroke_min_m,g.operating_home_stroke_m[i]+(lo-g.operating_home_raw_steps[i])*g.meters_per_step))
        upper.append(min(g.stroke_max_m,g.operating_home_stroke_m[i]+(hi-g.operating_home_raw_steps[i])*g.meters_per_step))
    if any(lo>=hi for lo,hi in zip(lower,upper)):
        raise ValueError('no usable position range within geometry and drive limits')
    return tuple(lower),tuple(upper)


@dataclass(frozen=True)
class Config:
    kp: tuple                 # force_unit/m; legacy field names below retain SI API compatibility
    ki: tuple                 # force_unit/(m s)
    kd: tuple                 # force_unit s/m
    force_limit_n: tuple       # force_unit, symmetric; legacy SI API name
    admittance_m_per_ns: tuple  # m/(force_unit s); legacy SI API name
    correction_limit_m: tuple # m, symmetric
    correction_rate_m_s: tuple # m/s, symmetric
    raw_zero: tuple           # absolute or user-defined relative 0x2C reference
    newtons_per_raw: tuple    # signed force_unit/raw; legacy SI API name
    max_dt_s: float
    max_age_s: float
    force_unit: str = 'N'     # relative_raw uses matching relative gains and no N claims

    @classmethod
    def from_document(cls, values):
        values=dict(values)
        if values.get('force_unit','N')=='relative_raw':
            # Retain the numerical PID implementation and legacy SI API while
            # exposing relative-unit parameter names in saved configurations.
            for public,legacy in (('force_limit_relative','force_limit_n'),
                                  ('admittance_m_per_relative_s','admittance_m_per_ns'),
                                  ('relative_scale','newtons_per_raw')):
                values[legacy]=values.pop(public)
        return cls(**values)

    def __post_init__(self):
        if self.force_unit not in ('N','relative_raw'):
            raise ValueError('force_unit must be N or relative_raw')
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

    def describe_state(self, state, raw):
        state['force_unit']=self.config.force_unit
        state['delta_force_raw']=[raw[i]-self.config.raw_zero[i] for i in range(6)]
        if self.config.force_unit=='relative_raw':
            for name in ('force_actual','force_cmd','force_error'):
                if name+'_n' in state:
                    state[name+'_relative']=state.pop(name+'_n')
        return state

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
                self.state=self.describe_state({'primed':True,'error_m':e,'force_actual_n':measured},raw)
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
            self.state=self.describe_state({'primed':False,'error_m':e,'force_cmd_n':force_cmd,
                        'force_actual_n':measured,'force_error_n':force_error,
                        'correction_m':correction[:],'position_command_m':commands[:]},raw)
            return tuple(commands)
        except (ValueError,TypeError,OverflowError) as exc:
            self.fault=str(exc)
            raise RuntimeError(self.fault) from exc
