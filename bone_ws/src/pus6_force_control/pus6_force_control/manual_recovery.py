"""Manual, bounded Mode 3 recovery; default CLI is OFFLINE planning only.

The user moves the whole supported platform. This never performs HOME/PARK,
Tare or independent slider moves. Terminal anchoring is delegated to ServoCore.
"""
import argparse
from dataclasses import dataclass
import json
from pathlib import Path
import select
import signal
import sys
import termios
import time
import tty

import numpy as np
from pus6_kinematics.analytical_6pus import Analytical6PusKinematics
from pus6_motor_driver.force_units import g_to_n
from pus6_motor_driver.motor import Motor
from pus6_motor_driver.safety import validate_status_feedback, SafetyViolation
from pus6_motor_driver.position_feedback import (
    position_word_to_steps, validate_feedback_position_steps, recovery_exit_target_steps)
from pus6_motor_driver.safety import (
    FEEDBACK_LOWER_LIMIT, FEEDBACK_UPPER_LIMIT,
    NORMAL_MOTION_LOWER_LIMIT, NORMAL_MOTION_UPPER_LIMIT)
from .force_pipeline import raw_steps_to_slider_m
from .six_axis_force_drive import ForceLimitsConfig, SixAxisForceDrive


@dataclass(frozen=True)
class ManualRecoveryConfig:
    control_strategy: str = 'native_support'
    damping_n_s_m: float = 500.
    max_increment_n: float = .5
    max_speed_m_s: float = .002
    max_current_ma: int = 200
    velocity_filter_s: float = .08
    operator_timeout_s: float = .75
    maximum_seconds: float = 120.
    min_force_n: float = -14.709975
    max_force_n: float = 14.709975
    max_rate_g_s: float = 500.
    feedback_timeout_s: float = .2

    def __post_init__(self):
        if self.control_strategy not in ('native_support', 'velocity_damping'):
            raise ValueError('manual strategy must be native_support or velocity_damping')
        if not all(np.isfinite(v) and v > 0 for v in (
            self.damping_n_s_m, self.max_increment_n, self.max_speed_m_s,
            self.max_current_ma, self.velocity_filter_s, self.operator_timeout_s,
            self.maximum_seconds, self.max_rate_g_s, self.feedback_timeout_s)):
            raise ValueError('manual recovery parameters must be finite and positive')
        if not (np.isfinite(self.min_force_n) and np.isfinite(self.max_force_n)
                and self.min_force_n < 0 < self.max_force_n):
            raise ValueError('finite signed absolute force limits required')
        if self.min_force_n < -14.709975 or self.max_force_n > 14.709975:
            raise ValueError('manual recovery cannot enlarge existing absolute force limits')


def validate_positions(positions):
    q = np.asarray(positions, dtype=float)
    if q.shape != (6,) or not np.isfinite(q).all() or np.any(q != np.rint(q)):
        raise ValueError('six real integer positions in motor ID order required')
    for position in q:
        validate_feedback_position_steps(position)
    return q


def recovery_plan(positions):
    """Use measured sliders in the existing FK/IK. Output is never executable."""
    result = dict(source='OFFLINE_USER_SUPPLIED_POSITIONS', input_steps=list(positions),
                  hardware_write_count=0, automatic_motion=False)
    try:
        raw = validate_positions(positions)
    except ValueError as exc:
        result.update(recovery_ready=False, reason=str(exc));return result
    kin = Analytical6PusKinematics()
    result.update(manual_input_valid=True, recovery_ready=True, manual_only=True,
        exit_target_steps_if_unchanged=[recovery_exit_target_steps(p) for p in raw],
        feedback_tolerance_steps=[FEEDBACK_LOWER_LIMIT,FEEDBACK_UPPER_LIMIT],
        position_command_range_steps=[0,2000])
    q = raw_steps_to_slider_m(raw, kin.geometry_config)
    fk = kin.forward_kinematics_with_metrics(q)
    result.update(valid_anchors=True, normal_range=bool(np.all((raw>=200)&(raw<=1800))),
                  fk_converged=bool(fk.converged), fk_residual_m=float(fk.residual_norm))
    if not fk.converged:
        result.update(reason='manual feedback accepted; measured FK did not converge; no automatic pose trajectory')
        return result
    result['pose_m_rad'] = list(fk.pose)
    if np.any(raw<0) or np.any(raw>2000):
        result.update(reason='endpoint feedback accepted unchanged; manual inward recovery only, no automatic pose trajectory')
        return result
    # A possible coordinated *future* path, not six independent ramps to 1000.
    # Verify each IK sample and the round-trip against actual starting sliders.
    home = kin.forward_kinematics_with_metrics([.05]*6)
    try:
        path=[]
        for a in np.linspace(0.,1.,51):
            pose=(1-a)*np.asarray(fk.pose)+a*np.asarray(home.pose)
            sliders=np.asarray(kin.inverse_kinematics(pose))
            steps=(sliders-.05)/kin.geometry_config.meters_per_step+1000
            if np.any(steps<0) or np.any(steps>2000):
                raise ValueError('candidate coordinated IK path leaves absolute stroke')
            path.append(steps.tolist())
        if np.max(abs(np.asarray(path[0])-raw))>1.:
            raise ValueError('FK/IK branch does not reproduce measured starting sliders')
        result['candidate_coordinated_path_steps']=path
        result['candidate_path_is_hardware_qualified']=False
    except Exception as exc:
        result['candidate_path_error']=str(exc)
    result.update(recovery_ready=True, manual_only=True,
        reason='valid inputs for manual mode; motion and stop behavior require onsite validation')
    return result


class ManualHandoverGuard:
    """Check real per-axis ACK/status without a full six-axis scan per write.

    A complete barrier is still mandatory before allowing manual movement.
    """
    def __init__(self, positions, config):
        self.positions=validate_positions(positions)
        self.config=config
        self.events=[]

    def check(self, mid, status, phase):
        try:
            validate_status_feedback(status,mid)
        except Exception as exc:
            exc.recovery_priority_motor_id=mid
            raise
        self.check_values(mid,position_steps=status.current_position,force_g=status.force_g,
            force_adc=status.force_adc,current_ma=status.current_mA,fault_code=status.error_code,phase=phase)

    def check_values(self, mid, *, position_steps, force_g, force_adc, current_ma, fault_code, phase):
        self.events.append(dict(ns=time.monotonic_ns(),motor_id=mid,phase=phase,
            position_steps=position_steps,force_g=force_g,current_ma=current_ma,fault_code=fault_code))
        try:
            validate_feedback_position_steps(position_steps)
            if fault_code:
                raise SafetyViolation(f'ID{mid} hardware fault during handover')
            if not (-32768<=force_g<=32767 and 0<=force_adc<=4095):
                raise SafetyViolation(f'ID{mid} invalid force/ADC during handover')
            force_n=g_to_n(force_g)
            if not self.config.min_force_n<=force_n<=self.config.max_force_n:
                raise SafetyViolation(f'ID{mid} actual force exceeds existing limit')
            if (abs(position_steps-self.positions[mid-1])>2
                    or abs(current_ma)>self.config.max_current_ma):
                raise SafetyViolation(f'ID{mid} manual handover movement/current protection')
        except Exception as exc:
            exc.recovery_priority_motor_id=mid
            raise

    def confirm_ready(self, snapshot, hold):
        if not snapshot.get('completed') or len(snapshot['axes'])!=6:
            raise SafetyViolation('incomplete manual entry barrier')
        for mid,axis in enumerate(snapshot['axes'],1):
            if (axis['motor_id']!=mid or axis['mode']!=3
                    or axis['target_force_g']!=hold[mid-1]):
                raise SafetyViolation(f'ID{mid} manual entry mode/target barrier failed')
            # Same guard and original anchors; do not reset away entry motion.
            self.check_values(mid,position_steps=axis['position_steps'],force_g=axis['force_g'],
                force_adc=axis['force_adc'],current_ma=axis['current_ma'],
                fault_code=axis['fault_code'],phase='ready_barrier')


class ManualCompliance:
    """Constant native support by default; optional legacy velocity damping.

    No external position stiffness, force amplification, implicit Tare or
    zero-force substitution. A support force alone is NOT a safe stop.
    """
    def __init__(self, positions, hold_g, now_ns, config=ManualRecoveryConfig(), *, position_ns=None):
        self.config=config
        self.initial=validate_positions(positions)
        self.previous=self.initial.copy();self.best=self.initial.copy()
        self.hold_g=np.asarray(hold_g,dtype=float)
        self.hold_n=np.asarray(g_to_n(hold_g))
        if self.hold_n.shape != (6,) or not np.isfinite(self.hold_n).all():
            raise ValueError('six valid signed measured holding forces required')
        if np.any(self.hold_n<config.min_force_n) or np.any(self.hold_n>config.max_force_n):
            raise ValueError('holding load exceeds existing absolute force limits')
        self.last_ns=now_ns;self.velocity=np.zeros(6)
        self.position_ns=self._source_times(position_ns,now_ns)
        self.drive=SixAxisForceDrive(None,ForceLimitsConfig(
            min_force_n=(config.min_force_n,)*6,max_force_n=(config.max_force_n,)*6,
            max_rate_g_s=(config.max_rate_g_s,)*6),clock=lambda:now_ns)
        self.drive.anchor_hold_forces_g(hold_g,now_ns=now_ns)

    def _source_times(self, position_ns, now_ns):
        stamps=np.asarray([now_ns]*6 if position_ns is None else position_ns,dtype=np.int64)
        if (stamps.shape!=(6,) or np.any(stamps<=0) or np.any(stamps>now_ns)
                or np.any(now_ns-stamps>self.config.feedback_timeout_s*1e9)):
            raise RuntimeError('invalid/stale per-axis position timestamp')
        return stamps

    def update(self, positions, forces_g, currents_ma, faults, *, now_ns, operator_ns, position_ns=None):
        c=self.config;q=validate_positions(positions)
        if now_ns-operator_ns>c.operator_timeout_s*1e9 or operator_ns>now_ns:
            raise RuntimeError('operator deadman expired; terminal handover required')
        dt=(now_ns-self.last_ns)*1e-9
        if not 0<dt<=c.feedback_timeout_s:
            raise RuntimeError('position/force feedback gap; terminal handover required')
        if any(faults) or np.max(abs(np.asarray(currents_ma)))>c.max_current_ma:
            raise RuntimeError('fault/current protection; terminal handover required')
        measured_n=np.asarray(g_to_n(forces_g))
        if measured_n.shape!=(6,) or not np.isfinite(measured_n).all() or np.any(measured_n<c.min_force_n) or np.any(measured_n>c.max_force_n):
            raise RuntimeError('invalid/excessive actual force; terminal handover required')
        stamps=self._source_times(position_ns,now_ns)
        axis_dt=(stamps-self.position_ns)*1e-9
        if np.any(axis_dt<=0) or np.any(axis_dt>c.feedback_timeout_s):
            raise RuntimeError('duplicate/stale per-axis position feedback')
        speed=(q-self.previous)*.000025/axis_dt
        if np.max(abs(speed))>c.max_speed_m_s:
            raise RuntimeError('manual motion speed protection; terminal handover required')
        # Start outside normal band only to move inward. Two-step allowance is
        # encoder fluctuation, not an expanded physical command stroke.
        high=self.initial>NORMAL_MOTION_UPPER_LIMIT;low=self.initial<NORMAL_MOTION_LOWER_LIMIT
        if np.any(high & (q>self.best+2)) or np.any(low & (q<self.best-2)):
            raise RuntimeError('outward motion near end of stroke; terminal handover required')
        inside=~(high|low)
        if np.any(inside & ((q<200)|(q>1800))):
            raise RuntimeError('left normal band; terminal handover required')
        self.best=np.where(high,np.minimum(self.best,q),np.where(low,np.maximum(self.best,q),q))
        self.velocity += axis_dt/(c.velocity_filter_s+axis_dt)*(speed-self.velocity)
        increment=(np.clip(-c.damping_n_s_m*self.velocity,-c.max_increment_n,c.max_increment_n)
                   if c.control_strategy=='velocity_damping' else np.zeros(6))
        # Near an end, damping may brake outward motion but must not add an
        # outward force increment while the operator is moving inward. F_hold
        # itself remains the measured native load reference, not zero force.
        near_upper=q>=NORMAL_MOTION_UPPER_LIMIT
        near_lower=q<=NORMAL_MOTION_LOWER_LIMIT
        increment=np.where(near_upper,np.minimum(increment,0.),increment)
        increment=np.where(near_lower,np.maximum(increment,0.),increment)
        command=np.clip(self.hold_n+increment,c.min_force_n,c.max_force_n)
        result=self.drive.command_forces_n(command,dry_run=True,now_ns=now_ns)
        applied_delta=np.asarray(g_to_n([f['target_force_g'] for f in result['frames']]))-self.hold_n
        if np.any(near_upper & (applied_delta>1e-10)) or np.any(near_lower & (applied_delta < -1e-10)):
            raise RuntimeError('slewed force still points outward near stroke end; terminal handover required')
        self.previous=q.copy();self.last_ns=now_ns;self.position_ns=stamps
        return dict(control_strategy=c.control_strategy,position_source_ns=stamps.tolist(),
            hold_force_g=self.hold_g.tolist(),measured_force_g=list(forces_g),
            measured_delta_force_n=(measured_n-self.hold_n).tolist(),
            target_force_n=command.tolist(),target_force_g=[f['target_force_g'] for f in result['frames']],
            delta_force_n=increment.tolist(),velocity_m_s=self.velocity.tolist(),
            measured_position_steps=q.tolist(),near_upper=near_upper.tolist(),near_lower=near_lower.tolist())


def read_manual_snapshot(core, row=None):
    """Read real protocol StatusFeedback; transport failures raise exceptions.

    ``timeout`` and ``force_valid`` belong to scheduler FeedbackSample, not
    Motor.read_status(). Preserve partial reads for an honest failure report.
    """
    row = {} if row is None else row
    start=time.monotonic_ns()
    row.update(ns=start,axes=[],invalid_axis_ids=[],completed=False)
    try:
        with core.writer.uart_lock:
            for mid in range(1,7):
                axis=dict(motor_id=mid,mode=None)
                row['axes'].append(axis)
                motor=Motor(core.bus,mid)
                status=motor.read_status()  # parsed/validated frame or exception; never a timeout flag
                axis.update(position_steps=status.current_position,force_g=status.force_g,
                    force_adc=status.force_adc,current_ma=status.current_mA,
                    fault_code=status.error_code,status_rx_ns=time.monotonic_ns())
                validate_status_feedback(status,mid)
                if not (-32768<=status.force_g<=32767 and 0<=status.force_adc<=4095):
                    raise SafetyViolation(f'ID{mid} invalid sensor force/ADC')
                if abs(status.current_mA)>core.config.get('max_current_ma',200):
                    raise SafetyViolation(f'ID{mid} manual recovery current protection')
                raw=motor.read_register(0x2A).values[0]
                measured=position_word_to_steps(raw)
                axis.update(position_2a_raw=raw,position_2a_steps=measured)
                axis['mode']=motor.read_register(0x25).values[0]
                target=motor.read_register(0x27).values[0]
                force=motor.read_register(0x2C).values[0]
                axis.update(target_force_g=target-65536 if target>=32768 else target,
                    force_2c_g=force-65536 if force>=32768 else force,rx_ns=time.monotonic_ns())
                validate_feedback_position_steps(measured)
                if abs(measured-status.current_position)>2:
                    raise SafetyViolation(f'ID{mid} inconsistent same-session position feedback')
        row['completed']=True
        return row
    except Exception as exc:
        row['error']=f'{type(exc).__name__}: {exc}'
        if row['axes']:row['invalid_axis_ids'].append(row['axes'][-1]['motor_id'])
        raise  # no missing-frame fallback, default force or default success
    finally:
        row.update(ns=time.monotonic_ns(),read_duration_ns=time.monotonic_ns()-start)


def manual_stop_status(core, terminal, snapshot):
    """Separate observed terminal stopping from 'never entered Mode3'."""
    if terminal is not None:
        return dict(hardware_stop_confirmed=terminal['hardware_stop_confirmed'],
            independent_stop_required=terminal['independent_stop_required'],
            stop_status=terminal['status'])
    if core is not None and core._force_control_managed:
        return dict(hardware_stop_confirmed=False,independent_stop_required=True,
                    stop_status='MODE3_OWNER_STOP_UNCONFIRMED')
    axes=snapshot.get('axes',[]) if snapshot else []
    mode0_observed=bool(snapshot and snapshot.get('completed') is True and len(axes)==6
        and all(a['mode']==0 and a['fault_code']==0 for a in axes))
    return dict(hardware_stop_confirmed=False,independent_stop_required=not mode0_observed,
        stop_status=('NO_MODE3_ENTRY_MODE0_OBSERVED' if mode0_observed else 'HARDWARE_STATE_UNCONFIRMED'))


def run_manual(config):
    """Future real execution on ServoCore only. Never invoked by offline planning."""
    if not (config.get('execute') and config.get('hardware_write_enabled')
            and config.get('is_hardware_qualified') and config.get('dry_run') is False):
        raise ValueError('existing hardware authorization, qualification and non-dry-run conditions required')
    if not sys.stdin.isatty():
        raise ValueError('interactive terminal required for hold-space deadman')
    from pus6_motor_driver.j14_process_uart import ServoCore
    from .stage5_experiment import Mode3OnlyTrace
    c=ManualRecoveryConfig(**config.get('manual_parameters',{}))
    out=Path(config['output_dir'])/time.strftime('%Y%m%d_%H%M%S');out.mkdir(parents=True,exist_ok=False)
    config=dict(config,output_dir=str(out))
    settings=termios.tcgetattr(sys.stdin);old_handlers={};stop=[False]
    core=None;report=dict(source='REAL_HARDWARE',outcome='starting',manual_ready=False,hardware_stop_confirmed=False,
                          independent_stop_required=True,stop_status='HARDWARE_STATE_UNCONFIRMED')
    def sample():
        row={}
        try:
            return read_manual_snapshot(core,row)
        finally:
            # Log partial reads too; a failed read must not look like six healthy axes.
            report['last_snapshot']=row
            with (out/'manual_recovery.jsonl').open('a') as f:f.write(json.dumps(row)+'\n')
    def keys():
        value=''
        while select.select([sys.stdin],[],[],0)[0]:
            char=sys.stdin.read(1)
            if not char:stop[0]=True;break
            value+=char
        return value
    try:
        for sig in (signal.SIGINT,signal.SIGTERM):
            old_handlers[sig]=signal.signal(sig,lambda *_:stop.__setitem__(0,True))
        # Same serial factory, exclusive lock, transport and safety owner as J14.
        owner=dict(config,control_mode='force',maintenance_only=True,mode3_only=True,
                   min_force_n=c.min_force_n,max_force_n=c.max_force_n,max_current_ma=c.max_current_ma,
                   allow_recovery_feedback_tolerance=True)
        core=ServoCore(owner)
        report['source']='MOCK' if core.config['hardware_mode']=='mock' else 'REAL_HARDWARE'
        trace=Mode3OnlyTrace(core.serial,out)
        core.serial=trace;core.bus.serial=trace;core.initialize()
        first=sample();positions=[a['position_steps'] for a in first['axes']]
        report['initial_snapshot']=first
        validate_positions(positions)
        if any(a['mode']!=0 or a['fault_code'] or abs(a['current_ma'])>c.max_current_ma for a in first['axes']):
            raise RuntimeError('manual recovery entry requires observed Mode0 and normal status')
        report['initial_snapshot']=first;report['offline_plan']=recovery_plan(positions)
        hold=[a['force_g'] for a in first['axes']]
        controller=ManualCompliance(positions,hold,first['ns'],c)  # validate before any write
        tty.setcbreak(sys.stdin.fileno())
        print('Hold SPACE to arm entry. DO NOT PUSH until ALL_MODE3_READY. F/Q = cancel/exit.',flush=True)
        deadline=time.monotonic()+30
        while ' ' not in keys():
            if stop[0] or time.monotonic()>deadline:raise RuntimeError('manual start cancelled')
            time.sleep(.02)
        # Recapture immediately: no old load/position cache is used for entry.
        first=sample();positions=[a['position_steps'] for a in first['axes']]
        if any(a['mode']!=0 or a['fault_code'] for a in first['axes']):raise RuntimeError('state changed before entry')
        hold=[a['force_g'] for a in first['axes']]
        handover=core.prepare_force_handover()
        hold=handover['holding_forces_g']
        positions=[a['position_steps'] for a in handover['axes']]
        report['handover_snapshot']=handover
        controller=ManualCompliance(positions,hold,time.monotonic_ns(),c)
        guard=ManualHandoverGuard(positions,c)
        report['entry_checks']=guard.events
        def prepare_axis(mid):
            # Same exclusive owner. Refresh THIS axis after previous axes have
            # changed mode, rather than reusing a stale all-Mode0 load vector.
            with core.writer.uart_lock:
                motor=Motor(core.bus,mid)
                status=motor.read_status()
                guard.check(mid,status,'before_entry')
                mode=motor.read_register(0x25).values[0]
                if mode!=0:raise SafetyViolation(f'ID{mid} changed mode before entry')
                raw=motor.read_register(0x2C).values[0]
                fresh=raw-65536 if raw>=32768 else raw
                guard.events.append(dict(ns=time.monotonic_ns(),motor_id=mid,
                                         phase='fresh_hold_2c',force_g=fresh))
                return fresh
        def entry_check(_mid):
            chars=keys().lower()
            if stop[0] or any(ch in chars for ch in ('f','q','\x03')):
                raise RuntimeError('operator cancelled force handover')
        core.enter_force_mode_with_handover(hold,static_baseline_g=hold,axis_verifier=entry_check,
            axis_preparer=prepare_axis,
            axis_status_verifier=lambda mid,status:guard.check(mid,status,'entry_ack'))
        hold=list(core.force_hold_g)
        report['holding_forces_g']=hold
        baseline=sample();guard.confirm_ready(baseline,hold)
        controller=ManualCompliance(positions,hold,baseline['ns'],c,
            position_ns=[a['status_rx_ns'] for a in baseline['axes']])
        # Retain entry anchors for endpoint checks, but velocity starts at the
        # freshly confirmed barrier (no entry displacement counted as speed).
        controller.previous=validate_positions([a['position_steps'] for a in baseline['axes']])
        report['manual_ready']=True;report['ready_snapshot']=baseline
        print('ALL_MODE3_READY: hold SPACE; gently move the supported platform inward. F = finish, Q = stop.',flush=True)
        begin=time.monotonic();operator_ns=time.monotonic_ns();cycle=0
        while not stop[0] and time.monotonic()-begin<c.maximum_seconds:
            chars=keys()
            if ' ' in chars:operator_ns=time.monotonic_ns()
            if any(ch in chars.lower() for ch in ('f','q','\x03')):break
            row=sample();axes=row['axes'];now=row['ns']
            if now-operator_ns>c.operator_timeout_s*1e9:
                report['outcome']='operator_deadman_released';break
            if any(a['mode']!=3 for a in axes):raise RuntimeError('native Mode3 lost')
            command=controller.update([a['position_steps'] for a in axes],[a['force_g'] for a in axes],
                [a['current_ma'] for a in axes],[a['fault_code'] for a in axes],now_ns=now,operator_ns=operator_ns,
                position_ns=[a['status_rx_ns'] for a in axes])
            core.writer.send_force_cycle(command['target_force_g'],cycle_id=cycle,target_timestamp_ns=now)
            cycle+=1
            with (out/'manual_commands.jsonl').open('a') as f:f.write(json.dumps(dict(ns=now,**command))+'\n')
            time.sleep(.02)
        if report['outcome']!='operator_deadman_released':report['outcome']='manual_session_finished'
    except BaseException as exc:
        report.update(outcome='stopped',error=f'{type(exc).__name__}: {exc}')
        report['recovery_priority_motor_id']=getattr(exc,'recovery_priority_motor_id',None)
    finally:
        terminal=None
        if core:
            try:
                priority=report.get('recovery_priority_motor_id')
                terminal=core.finish_force_control('manual_recovery_finished',fault=report['outcome']=='stopped',
                    priority_motor_ids=(priority,) if priority is not None else ())
            except Exception as exc:
                report['terminal_error']=f'{type(exc).__name__}: {exc}'
            finally:
                try:core.close('manual_recovery_finished')
                except Exception as exc:report['close_error']=f'{type(exc).__name__}: {exc}'
            # close() can finish an interrupted terminal attempt. Read the same
            # owner's final result, never invent confirmation from port closure.
            if terminal is None:terminal=core.force_shutdown_report
            report['terminal_recovery']=terminal
            report.update(manual_stop_status(core,terminal,report.get('last_snapshot')))
            if terminal:
                report['final_positions_steps']=[a['observations'][-1]['position_steps']
                    if a.get('observations') else None for a in terminal['axes']]
            report['hardware_write_count']=len(trace.write_commands) if 'trace' in locals() else 0
        termios.tcsetattr(sys.stdin,termios.TCSADRAIN,settings)
        for sig,handler in old_handlers.items():signal.signal(sig,handler)
        (out/'manual_recovery_result.json').write_text(json.dumps(report,indent=2)+'\n')
    return report


def main(args=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--positions',nargs=6,type=int,help='offline user-supplied positions; never opens serial')
    p.add_argument('--output',help='new offline plan JSON')
    p.add_argument('--config',help='future explicit real execution configuration')
    p.add_argument('--execute',action='store_true')
    p.add_argument('--hardware-write-enabled',action='store_true')
    p.add_argument('--hardware-qualified',action='store_true')
    p.add_argument('--no-dry-run',action='store_true')
    a=p.parse_args(args)
    if a.execute:
        if not a.config:p.error('--execute requires --config')
        config=json.loads(Path(a.config).read_text());config.update(execute=True,
            hardware_write_enabled=a.hardware_write_enabled, is_hardware_qualified=a.hardware_qualified,
            dry_run=not a.no_dry_run)
        result=run_manual(config)
    else:
        if a.positions is None:p.error('offline default requires six --positions')
        result=recovery_plan(a.positions)
        if a.output:
            dest=Path(a.output);dest.parent.mkdir(parents=True,exist_ok=True)
            if dest.exists():raise FileExistsError('preserve existing recovery plan')
            dest.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
