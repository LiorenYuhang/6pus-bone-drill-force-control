"""Human-supported, Cartesian coordinated HOME in native Mode 0.

One existing ServoCore owns UART. Default read-only operation never clears a
force target or anchors a position. Execution requires explicit support consent.
"""
import argparse
from dataclasses import asdict, dataclass
import json
from pathlib import Path
import signal
import time

import numpy as np
from scipy.spatial.transform import Rotation
from pus6_control.joint_trajectory_controller import quintic_progress
from pus6_kinematics.analytical_6pus import Analytical6PusKinematics
from pus6_motor_driver.force_units import g_to_n
from pus6_motor_driver.motor import Motor
from pus6_motor_driver.protocol import parse_frame
from pus6_motor_driver.safety import SafetyViolation, validate_status_feedback
from pus6_motor_driver.position_feedback import validate_position_target_steps
from .force_pipeline import raw_steps_to_slider_m
from .manual_recovery import read_manual_snapshot, validate_positions
from .six_axis_tare_experiment import SerialTrace


@dataclass(frozen=True)
class AssistedHomeConfig:
    max_step_per_segment: int = 2
    speed_steps_s: float = 10.  # 0.25 mm/s reference, not firmware 0x28
    max_current_ma: int = 200
    max_force_n: float = 14.709975
    tracking_error_steps: int = 5
    segment_settle_steps: int = 2
    segment_timeout_s: float = 1.
    feedback_timeout_s: float = .2
    max_observed_speed_m_s: float = .002
    home_tolerance_steps: int = 5
    home_samples: int = 3
    maximum_seconds: float = 600.

    def __post_init__(self):
        if not all(np.isfinite(v) and v>0 for v in asdict(self).values()):
            raise ValueError('positive finite recovery parameters required')
        if (self.max_step_per_segment>2 or self.speed_steps_s>10
                or self.max_current_ma>200 or self.max_force_n>14.709975
                or self.tracking_error_steps>5 or self.max_observed_speed_m_s>.002):
            raise ValueError('dedicated recovery cannot enlarge protective limits')


def build_home_plan(actual_steps, config=AssistedHomeConfig()):
    """FK(real measurements) -> SE(3) path -> existing IK -> legal small steps.

    Endpoint feedback remains untouched. Skip only the initial portion whose
    *quantized commands* are outside 0..2000, and require the first legal command
    to be within two steps of every actual axis. Never clip measured positions.
    """
    actual=validate_positions(actual_steps)
    k=Analytical6PusKinematics()
    fk=k.forward_kinematics_with_metrics(raw_steps_to_slider_m(actual,k.geometry_config),max_iter=100)
    if not fk.converged:
        raise ValueError(f'measured FK failed: residual {fk.residual_norm}')
    pose=np.asarray(fk.pose)
    home=np.array([0.,0.,k.nominal_height,0.,0.,0.])
    rotation=Rotation.from_euler('xyz',pose[3:])
    to_home=rotation.inv().as_rotvec()
    roundtrip=(np.asarray(k.inverse_kinematics(pose))-.05)/k.geometry_config.meters_per_step+1000
    if np.max(abs(roundtrip-actual))>.1:
        raise ValueError('FK/IK branch does not reproduce the actual measured start')
    previous=actual.copy();points=[];entered=False;duration=0.
    # A shared quintic Cartesian progress is reused from Position. This is not
    # the online third-order position shaper or six independent joint ramps.
    for tau in np.linspace(0.,1.,6001):
        progress=quintic_progress(float(tau))
        desired=np.r_[(1-progress)*pose[:3]+progress*home[:3],
            (rotation*Rotation.from_rotvec(progress*to_home)).as_euler('xyz')]
        sliders=np.asarray(k.inverse_kinematics(desired))
        continuous=(sliders-.05)/k.geometry_config.meters_per_step+1000
        target=np.rint(continuous).astype(int)
        if np.any(target<0) or np.any(target>2000):
            if entered:raise ValueError('Cartesian HOME path leaves command stroke')
            continue
        entered=True
        delta=float(np.max(abs(target-previous)))
        if delta>config.max_step_per_segment:
            raise ValueError(f'HOME segment exceeds {config.max_step_per_segment} steps: {delta}')
        if not delta:continue
        duration+=max(.1,delta/config.speed_steps_s)
        points.append(dict(progress=progress,pose_m_rad=desired.tolist(),
            continuous_steps=continuous.tolist(),targets=target.tolist(),
            minimum_segment_s=max(.1,delta/config.speed_steps_s)))
        previous=target
    if not entered or np.max(abs(previous-1000))>0:
        raise ValueError('HOME path did not reach six-axis HOME')
    return dict(initial_feedback_steps=actual.tolist(),initial_pose_m_rad=pose.tolist(),
        fk_residual_m=float(fk.residual_norm),home_pose_m_rad=home.tolist(),
        home_steps=[1000]*6,waypoints=points,estimated_minimum_seconds=duration,
        position_command_range=[0,2000],max_segment_steps=config.max_step_per_segment,
        reference_speed_m_s=config.speed_steps_s*k.geometry_config.meters_per_step,
        source='existing 6PUS FK/IK and Position quintic_progress; shared Cartesian path')


class AssistedHomeTrace(SerialTrace):
    allowed_write_registers=(0x25,0x27,0x29,0x1a)

    def write(self,data):
        frame=parse_frame(data)
        if frame.command==0x32:
            words=[int.from_bytes(frame.data[i:i+2],'little') for i in range(0,len(frame.data),2)]
            anchor=(frame.register_addr==0x25 and len(words)==5 and words[:2]==[0,0]
                    and words[3]==0 and 0<=words[4]<=2000)
            target=frame.register_addr==0x29 and len(words)==1 and 0<=words[0]<=2000
            clear=(frame.register_addr==0x27 and words==[0]
                   and getattr(self,'_mode0_verified',False))
            pause=(frame.register_addr==0x1a and words==[1]
                   and getattr(self,'_terminal_recovery',False))
            if not (anchor or clear or target or pause):
                raise SafetyViolation('assisted HOME permits only Mode0 anchors, legal positions, Mode0 force cleanup and terminal Pause')
        return super().write(data)


def validate_snapshot(row,c,previous=None,targets=None):
    if not row.get('completed') or len(row.get('axes',[]))!=6:
        raise SafetyViolation('incomplete six-axis recovery feedback')
    for index,a in enumerate(row['axes']):
        mid=index+1
        try:
            if a['motor_id']!=mid or a['mode']!=0 or a['fault_code']:
                raise SafetyViolation(f'ID{mid} recovery mode/fault invalid')
            validate_positions([a['position_steps']]*6)
            if abs(a['current_ma'])>c.max_current_ma or abs(g_to_n(a['force_g']))>c.max_force_n:
                raise SafetyViolation(f'ID{mid} recovery current/force limit')
            if previous:
                b=previous['axes'][index]
                dt=(a['status_rx_ns']-b['status_rx_ns'])*1e-9
                if not 0<dt<=c.feedback_timeout_s:
                    raise SafetyViolation(f'ID{mid} feedback stale/duplicate')
                velocity=(a['position_steps']-b['position_steps'])*.000025/dt
                if abs(velocity)>c.max_observed_speed_m_s:
                    raise SafetyViolation(f'ID{mid} recovery observed speed limit')
            if targets is not None and abs(a['position_steps']-targets[index])>c.tracking_error_steps:
                raise SafetyViolation(f'ID{mid} recovery tracking error')
        except Exception as exc:
            exc.recovery_priority_motor_id=mid
            raise


def read_motion_snapshot(core):
    axes=[]
    with core.writer.uart_lock:
        for mid in range(1,7):
            motor=Motor(core.bus,mid);status=motor.read_status()
            stamp=time.monotonic_ns();validate_status_feedback(status,mid)
            mode=motor.read_register(0x25).values[0]
            axes.append(dict(motor_id=mid,position_steps=status.current_position,
                target_position_steps=status.target_position,force_g=status.force_g,
                current_ma=status.current_mA,fault_code=status.error_code,mode=mode,status_rx_ns=stamp))
    return dict(ns=time.monotonic_ns(),completed=True,axes=axes)


def run(config):
    from pus6_motor_driver.j14_process_uart import ServoCore
    c=AssistedHomeConfig(**config.get('recovery_parameters',{}))
    execute=config.get('execute',False)
    if execute and not (config.get('support_confirmed') and config.get('hardware_write_enabled')
            and config.get('is_hardware_qualified') and config.get('dry_run') is False):
        raise ValueError('onsite support confirmation and existing hardware write conditions required')
    out=Path(config['output_dir'])/time.strftime('%Y%m%d_%H%M%S');out.mkdir(parents=True,exist_ok=False)
    report=dict(source='UNOBSERVED',outcome='starting',hardware_write_count=0,home_confirmed=False,
        support_confirmed=config.get('support_confirmed',False),parameters=asdict(c))
    core=trace=None;stop=[False];handlers={};priority=()
    def save():
        if trace:report['hardware_write_count']=len(trace.write_commands)
        (out/'experiment.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    def sample(label,full=False):
        row=read_manual_snapshot(core) if full else read_motion_snapshot(core)
        row['label']=label
        with (out/'feedback.jsonl').open('a') as f:f.write(json.dumps(row)+'\n')
        return row
    def cancel():
        if stop[0]:raise RuntimeError('operator stop requested')
    try:
        for sig in (signal.SIGINT,signal.SIGTERM):handlers[sig]=signal.signal(sig,lambda *_:stop.__setitem__(0,True))
        core=ServoCore(dict(config,output_dir=str(out),control_mode='force',maintenance_only=True,
            mode3_only=True,max_current_ma=c.max_current_ma,allow_recovery_feedback_tolerance=True))
        report['source']='MOCK' if core.config['hardware_mode']=='mock' else 'REAL_HARDWARE'
        trace=AssistedHomeTrace(core.serial,out);core.serial=trace;core.bus.serial=trace
        core.initialize()  # existing single UART owner, read-only maintenance startup
        initial=sample('initial',True);report['initial_snapshot']=initial
        validate_snapshot(initial,c)
        plan=build_home_plan([a['position_steps'] for a in initial['axes']],c)
        (out/'plan.json').write_text(json.dumps(plan,indent=2)+'\n')
        report['initial_plan_waypoints']=len(plan['waypoints']);report['minimum_motion_s']=plan['estimated_minimum_seconds']
        if not execute:
            report['outcome']='read_only_plan_complete';return report
        core.check_hardware_write_permission();cancel()
        core._force_control_managed=True  # terminal handling is armed before any authorized write
        # Mode0 current-position anchoring also handles a real endpoint 2001
        # with a legal 2000 command. This is an explicit small inward target,
        # never a rewritten measurement or assumed HOME.
        with core.writer.uart_lock:
            for mid in range(1,7):
                cancel()
                if Motor(core.bus,mid).read_register(0x25).values[0]!=0:
                    raise SafetyViolation(f'ID{mid} mode changed before initial anchor')
                core.writer._anchor_axis_to_position_mode(mid,allow_feedback_tolerance=True)
            anchored=sample('initial_mode0_anchors');validate_snapshot(anchored,c)
            object.__setattr__(trace,'_mode0_verified',True)
            for mid in range(1,7):
                cancel();motor=Motor(core.bus,mid)
                if motor.read_register(0x25).values[0]!=0:
                    raise SafetyViolation('never clear native force target outside observed Mode0')
                status=motor.write_register(0x27,[0]);validate_status_feedback(status,mid)
                if motor.read_register(0x27).values[0]!=0:raise SafetyViolation('force target cleanup readback mismatch')
        current=sample('prepared',True);validate_snapshot(current,c)
        report['prepared_snapshot']=current
        plan=build_home_plan([a['position_steps'] for a in current['axes']],c)
        (out/'executed_plan.json').write_text(json.dumps(plan,indent=2)+'\n')
        save();begin=time.monotonic();completed=0
        for index,point in enumerate(plan['waypoints']):
            cancel()
            if time.monotonic()-begin>c.maximum_seconds:raise RuntimeError('assisted HOME motion timeout')
            target=point['targets']
            for value in target:validate_position_target_steps(value)
            stage=time.monotonic()
            with core.writer.uart_lock:
                for mid,value in enumerate(target,1):
                    cancel()
                    status=Motor(core.bus,mid).write_register(0x29,[value])
                    validate_status_feedback(status,mid)
                    if abs(status.current_mA)>c.max_current_ma or abs(g_to_n(status.force_g))>c.max_force_n:
                        error=SafetyViolation(f'ID{mid} waypoint write current/force protection')
                        error.recovery_priority_motor_id=mid;raise error
            with (out/'commands.jsonl').open('a') as f:
                f.write(json.dumps(dict(ns=time.monotonic_ns(),segment=index,**point))+'\n')
            deadline=stage+c.segment_timeout_s
            while True:
                cancel();row=sample('motion');validate_snapshot(row,c,current,target);current=row
                settled=max(abs(a['position_steps']-t) for a,t in zip(row['axes'],target))<=c.segment_settle_steps
                if settled and time.monotonic()-stage>=point['minimum_segment_s']:break
                if time.monotonic()>deadline:raise RuntimeError('coordinated segment did not settle; no trajectory continuation')
                time.sleep(.03)
            completed+=1
            if index%50==0:print(f'HOME segment {index+1}/{len(plan["waypoints"])}: {[a["position_steps"] for a in current["axes"]]}',flush=True)
        samples=[]
        for _ in range(c.home_samples):
            time.sleep(.05);row=sample('home_confirmation');validate_snapshot(row,c)
            if any(abs(a['position_steps']-1000)>c.home_tolerance_steps for a in row['axes']):
                raise RuntimeError('actual HOME feedback outside original 995..1005 band')
            samples.append(row)
        report.update(outcome='home_confirmed',home_confirmed=True,home_samples=samples,completed_segments=completed)
    except BaseException as exc:
        report.update(outcome='stopped',error=f'{type(exc).__name__}: {exc}')
        mid=getattr(exc,'recovery_priority_motor_id',None);priority=(mid,) if mid else ()
    finally:
        if core:
            try:
                terminal=core.finish_force_control('assisted_home_finished',fault=report['outcome']=='stopped',
                    priority_motor_ids=priority,reanchor_mode0=True)
                report['terminal_recovery']=terminal
                if terminal:
                    report.update(hardware_stop_confirmed=terminal['hardware_stop_confirmed'],
                        independent_stop_required=terminal['independent_stop_required'])
                report['final_snapshot']=sample('final',True)
            except Exception as exc:report['terminal_error']=f'{type(exc).__name__}: {exc}'
            finally:
                try:core.close('assisted_home_finished')
                except Exception as exc:report['close_error']=str(exc)
        for sig,handler in handlers.items():signal.signal(sig,handler)
        save()
    return report


def main(args=None):
    parser=argparse.ArgumentParser(description=__doc__)
    action=parser.add_mutually_exclusive_group(required=True)
    action.add_argument('--positions',nargs=6,type=int,help='offline plan only')
    action.add_argument('--read-only',action='store_true')
    action.add_argument('--execute',action='store_true')
    parser.add_argument('--config')
    parser.add_argument('--support-confirmed',action='store_true')
    parser.add_argument('--hardware-write-enabled',action='store_true')
    parser.add_argument('--hardware-qualified',action='store_true')
    parser.add_argument('--no-dry-run',action='store_true')
    a=parser.parse_args(args)
    if a.positions is not None:
        plan=build_home_plan(a.positions)
        print(json.dumps({k:v for k,v in plan.items() if k!='waypoints'},indent=2));return
    if not a.config:parser.error('--config required for real owner')
    config=json.loads(Path(a.config).read_text());config.update(execute=a.execute,
        support_confirmed=a.support_confirmed,hardware_write_enabled=a.hardware_write_enabled,
        is_hardware_qualified=a.hardware_qualified,dry_run=not a.no_dry_run)
    print(json.dumps(run(config),ensure_ascii=False,indent=2))


if __name__=='__main__':main()
