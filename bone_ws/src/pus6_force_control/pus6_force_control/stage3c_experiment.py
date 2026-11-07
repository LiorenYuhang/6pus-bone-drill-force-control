"""Bounded live experiment in the existing UART Child, using the real ROS PID node.

No Tare, independent UART, gain changes, dynamic position reference or auto Pause.
"""
from dataclasses import asdict
from datetime import datetime
import json
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import Float64
from pus6_msgs.msg import TimedJointFeedback
from pus6_motor_driver.motor import Motor
from pus6_motor_driver.j14_process_uart import ServoCore
from pus6_motor_driver.j14_feedback_bridge import populate_force_feedback
from pus6_motor_driver.force_units import n_to_g, g_to_n, quantize_force_g_with_n_limits
from .force_control_node import TaskSpaceForceControlNode
from .force_pipeline import raw_steps_to_slider_m
from .six_axis_tare_experiment import SerialTrace


def response_stages(config):
    """Only the requested staged amplitudes; no gain/force-cap changes."""
    if config.get('hold_only'):
        return [('zero_native', 0., 2.5)]
    if config.get('increment_5_response'):
        # Only enough native hold to fill the existing 30 ms Fixed-Lag history;
        # not another Stage 3C.2 static qualification experiment.
        return [('zero_native',0.,.10),('zero_pid',0.,1.),('positive_5',5.,8.),
                ('return_zero_positive_5',0.,7.),('negative_5',-5.,8.),
                ('return_zero_negative_5',0.,7.)]
    if not config.get('graded_force_response'):
        return [('zero_native',0.,2.5),('zero_pid',0.,2.5),('positive',.5,5.),
                ('return_zero_positive',0.,5.),('negative',-.5,5.),('return_zero_negative',0.,5.)]
    return [('zero_native',0.,2.5),('zero_pid',0.,2.5),('positive_5',5.,8.),
            ('return_zero_positive_5',0.,7.),('negative_5',-5.,8.),('return_zero_negative_5',0.,7.),
            ('positive_10',10.,13.),('return_zero_positive_10',0.,12.),
            ('negative_10',-10.,13.),('return_zero_negative_10',0.,12.)]


def tier_one_expansion(records):
    """Requested direction/fault/limiting check, using the completed real first tier."""
    means={};reasons=[]
    for stage in ('zero_pid','positive_5','negative_5'):
        rows=[r for r in records if r['stage']==stage and r.get('measured_z_n') is not None]
        if not rows:
            reasons.append(stage+': missing completed force response');continue
        tail=[r for r in rows if r['monotonic_ns']>=rows[-1]['monotonic_ns']-2_000_000_000]
        means[stage]=sum(r['measured_z_n'] for r in tail)/len(tail)
        if stage!='zero_pid':
            # Detect repeated large force swings, not sub-resolution noise.
            errors=[r['target_z_n']-r['measured_z_n'] for r in tail]
            signs=[1 if e>1. else -1 for e in errors if abs(e)>1.]
            crossings=sum(a!=b for a,b in zip(signs,signs[1:]))
            span=max(r['measured_z_n'] for r in tail)-min(r['measured_z_n'] for r in tail)
            if crossings>=4 and span>abs(tail[-1]['target_z_n']):
                reasons.append(stage+': repeated large force oscillation')
        if stage!='zero_pid' and any(r.get('pid_saturated',False) or r.get('actuator_limited',False) for r in tail):
            reasons.append(stage+': steady-window saturation or slew limiting')
    if len(means)==3:
        if not means['positive_5']>max(0.,means['zero_pid']):reasons.append('positive force did not show expected positive response')
        if not means['negative_5']<min(0.,means['zero_pid']):reasons.append('negative force did not show expected negative response')
    return dict(allowed=not reasons,reasons=reasons,last2s_measured_n=means)


def positive_response_check(records):
    """Use the existing direction/oscillation/limiting criteria before reversal."""
    reasons=[];means={}
    for stage in ('zero_pid','positive_5','return_zero_positive_5'):
        rows=[r for r in records if r['stage']==stage and r.get('measured_z_n') is not None]
        if not rows:
            reasons.append(stage+': no real PID samples');continue
        tail=[r for r in rows if r['monotonic_ns']>=rows[-1]['monotonic_ns']-2_000_000_000]
        means[stage]=float(np.mean([r['measured_z_n'] for r in tail]))
        if stage!='zero_pid':
            signs=[1 if r['target_error_n']>1 else -1 for r in tail if abs(r['target_error_n'])>1]
            crossings=sum(a!=b for a,b in zip(signs,signs[1:]))
            span=max(r['measured_z_n'] for r in tail)-min(r['measured_z_n'] for r in tail)
            if crossings>=4 and span>5:
                reasons.append(stage+': repeated large force oscillation')
            if any(r.get('pid_saturated') or r.get('actuator_limited') for r in tail):
                reasons.append(stage+': persistent output/actuator limiting')
    if 'positive_5' in means and 'zero_pid' in means:
        if means['positive_5']<=max(0.,means['zero_pid']):
            reasons.append('positive force did not show the expected positive response')
    return dict(allowed=not reasons,reasons=reasons,last2s_measured_n=means)


def ingest_snapshot_feedback(core, statuses, rx_ns, start_ns, sequence):
    """Feed the already-read live StatusFeedback, with its actual RX timestamps.

    Mode-entry inventories are real feedback too; don't let their acquisition
    starve the existing scheduler until the first 0x27 streaming cycle.
    """
    feedback = core.feedback.ingest_cycle_feedback(statuses, servo_cycle_id=sequence,
        servo_cycle_timestamp_ns=start_ns, per_axis_rx_done_ns=rx_ns)
    core.update(feedback)


def run(config):
    if (config.get('hardware_mode') != 'active' or config.get('dry_run') is not False
            or not config.get('hardware_write_enabled') or not config.get('is_hardware_qualified')
            or config.get('force_fault_recovery') != 'position_reanchor'
            or not config.get('maintenance_only') or config.get('control_mode') != 'force'):
        raise ValueError('Stage 3C needs explicit real execution and existing write/qualification permissions')
    out=Path(config['output_dir']);out.mkdir(parents=True,exist_ok=True)
    report=dict(source='REAL_HARDWARE',start_time=datetime.now().astimezone().isoformat(),
        outcome='started',tare_count=0,pause_count=0,preload_relation='not independently calibrated',
        historical_tare_reference=config['historical_tare_reference'],snapshots=[],stages=[],pid_cycles=0)
    core=node=trace=None;entry_attempted=False;records=[];seq=0
    json_default=lambda value:value.tolist() if isinstance(value,np.ndarray) else value.item()
    def save():
        if trace:
            report['hardware_write_count']=len(trace.write_commands)
            report['tare_count']=sum(w['register']==9 for w in trace.write_commands)
        (out/'experiment.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,default=json_default)+'\n')
    def check(positions,currents,faults,forces):
        if any(faults):raise RuntimeError('hardware fault: '+str(faults))
        if max(abs(x) for x in currents)>config['max_current_ma']:raise RuntimeError('current monitor stop')
        if max(abs(x-y) for x,y in zip(positions,initial_positions))>config['max_position_change_steps']:
            raise RuntimeError('position displacement monitor stop')
        if max(abs(x) for x in g_to_n(forces))>14.709975:raise RuntimeError('force feedback limit stop')
    def snapshot(label):
        nonlocal seq
        rows=[];statuses={};status_rx_ns={};scan_start_ns=time.monotonic_ns()
        with core.writer.uart_lock:
            for mid in range(1,7):
                m=Motor(core.bus,mid);s=m.read_status()
                statuses[mid]=s;status_rx_ns[mid]=time.monotonic_ns()
                registers=(0x25,0x27,0x2A,0x2C,0x2B,0x2F) if (config.get('graded_force_response') or config.get('hold_only') or config.get('increment_5_response')) and label in ('before_mode3','all_mode3_hold_targets','final_after_mode0_exit') else (0x25,0x27,0x2A)
                values={reg:m.read_register(reg).values[0] for reg in registers}
                target=values[0x27];target=target-65536 if target>=32768 else target
                force_word=values.get(0x2C)
                direct_force=(force_word-65536 if force_word>=32768 else force_word) if force_word is not None else None
                rows.append(dict(motor_id=mid,mode=values[0x25],target_force_g=target,
                                 direct_force_2c_g=direct_force,register_words={hex(k):v for k,v in values.items()},
                                 position_2a=values[0x2A],rx_done_ns=status_rx_ns[mid],**asdict(s)))
        if not core.feedback.stopped:
            seq+=1
            ingest_snapshot_feedback(core,statuses,status_rx_ns,scan_start_ns,seq)
        item=dict(label=label,time_iso=datetime.now().astimezone().isoformat(),axes=rows)
        report['snapshots'].append(item);save()
        with (out/'snapshots.jsonl').open('a') as stream:stream.write(json.dumps(item)+'\n')
        return rows
    def feedback_message(audit):
        # Same status response and host receive-completion timestamp for q and f.
        fb=core.feedback.ingest_cycle_feedback(audit.feedbacks,servo_cycle_id=seq,
            servo_cycle_timestamp_ns=audit.cycle_start_ns,per_axis_rx_done_ns=audit.feedback_rx_done_ns)
        core.update(fb)
        if core.feedback.stopped or core.writer.stopped:raise RuntimeError('feedback/transport stopped')
        if not all(core.force_valid):raise RuntimeError('invalid true force feedback')
        check(core.positions,core.currents,core.faults,core.forces_g_raw)
        msg=TimedJointFeedback();msg.uart_cycle_sequence=seq
        msg.snapshot_start_monotonic_ns=audit.cycle_start_ns
        msg.snapshot_end_monotonic_ns=time.monotonic_ns()
        msg.per_axis_rx_done_monotonic_ns=list(core.rx_ns)
        msg.raw_steps=list(core.positions);msg.positions_meters=raw_steps_to_slider_m(core.positions,node.kinematics.geometry_config).tolist()
        msg.currents_ma=list(core.currents);msg.fault_codes=list(core.faults)
        populate_force_feedback(msg,core.force_feedback_fields())
        if msg.source != TimedJointFeedback.SOURCE_HARDWARE:raise RuntimeError('non-hardware force source')
        node._handle_timed_feedback(msg)
    try:
        # Construct the actual ROS control node; its timer is called once per new
        # real feedback by this bounded owner, preventing competing timer cycles.
        rclpy.init()
        overrides=[Parameter(k,value=v) for k,v in dict(dry_run=False,hardware_write_enabled=True,
            is_hardware_qualified=True,fault_stop_policy='external_position_reanchor',
            reference_rate_n_s=float(config.get('reference_rate_n_s',.2)),use_fixed_lag=True,fixed_lag_delay_ms=30.).items()]
        node=TaskSpaceForceControlNode('stage3c_z_force_pid',parameter_overrides=overrides)
        node.timer.cancel()
        core=ServoCore(config)
        core.bus.timeout_s=.05
        trace=SerialTrace(core.serial,out);core.serial=trace;core.bus.serial=trace
        core.initialize() # confirmed Mode 0, reads only
        before=snapshot('before_mode3')
        initial_positions=[r['current_position'] for r in before]
        if any(r['mode']!=0 or r['error_code'] for r in before):raise RuntimeError('initial mode/fault check')
        check(initial_positions,[r['current_mA'] for r in before], [r['error_code'] for r in before],
              [r['force_g'] for r in before])
        # Compute requested task-force allocations at the actual measured pose
        # BEFORE the first write. The maximum PID output is checked as well.
        q=raw_steps_to_slider_m(initial_positions,node.kinematics.geometry_config)
        fk=node.kinematics.forward_kinematics_with_metrics(q)
        if not fk.converged:raise RuntimeError('initial FK failed')
        report['initial_pose_m_rad']=list(fk.pose)
        # Read live register feedback immediately before preparing entry, without
        # Tare or the historical hardware-tare branch that chose zero targets.
        handover=core.prepare_force_handover()
        handover_axes=handover['axes']
        if any(r['mode'] != 0 for r in handover_axes):
            raise RuntimeError('fresh handover inventory is not uniformly Mode 0')
        holding_g=[r['force_g_raw'] for r in handover_axes]
        node.initialize_hold_forces_g(holding_g)
        report['handover_axes']=handover_axes
        report['hold_forces_g']=holding_g
        report['hold_forces_n']=node.pipeline.hold_forces_n.tolist()
        report['feedback_reference']='current_measured_hold; relative = raw - hold; no Tare'
        check([r['position_steps'] for r in handover_axes], [r['current_mA'] for r in before],
              [r['error_code'] for r in before], holding_g)
        report['allocation_before_writes']=[]
        for force in ((0.,) if config.get('hold_only') else (-10.,-5.,0.,5.,10.)):
            allocated=node.pipeline.hold_forces_n + node.allocator.allocate([0,0,force,0,0,0],fk.pose).actuator_forces_n
            within=all(lo<=f<=hi for lo,f,hi in zip(node.drive.limits.min_force_n,allocated,node.drive.limits.max_force_n))
            report['allocation_before_writes'].append(dict(task_z_n=force,actuator_force_n=allocated.tolist(),within_limits=within))
            if not within:raise RuntimeError('requested task allocation exceeds existing per-axis limits')
        report['effective_pid_config']=asdict(node.pipeline.pid.config)
        report['force_compensation']=asdict(node.pipeline.compensation)
        report['force_limits']=asdict(node.drive.limits)
        save()
        def verify_entry(mid):
            rows=snapshot('after_mode3_axis_'+str(mid))
            check([r['current_position'] for r in rows],[r['current_mA'] for r in rows],
                  [r['error_code'] for r in rows],[r['force_g'] for r in rows])
        entry_attempted=True
        core.enter_force_mode_with_handover(holding_g,static_baseline_g=holding_g,axis_verifier=verify_entry)
        report['mode_handover']=core.writer.last_force_handover
        report['entry_confirmed_ns']=time.monotonic_ns()
        active=snapshot('all_mode3_hold_targets')
        if any(r['mode']!=3 or r['target_force_g']!=hold for r,hold in zip(active,holding_g)):
            raise RuntimeError('mode3/hold target readback failed')
        check([r['current_position'] for r in active],[r['current_mA'] for r in active],
              [r['error_code'] for r in active],[r['force_g'] for r in active])
        report['all_mode3_confirmed']=True;save()
        stages=response_stages(config)
        cycle_id=0;last_modes=[r['mode'] for r in active];mode_stamp=time.monotonic_ns()
        for stage,target,duration in stages:
            if config.get('increment_5_response') and stage=='negative_5':
                report['positive_response_check']=positive_response_check(records);save()
                if not report['positive_response_check']['allowed']:
                    raise RuntimeError('positive response not normal; reverse skipped: '+
                                       '; '.join(report['positive_response_check']['reasons']))
            if stage=='positive_10':
                report['tier_one_expansion']=tier_one_expansion(records);save()
                if not report['tier_one_expansion']['allowed']:
                    print('SKIP TIER 10',report['tier_one_expansion']['reasons'],flush=True)
                    report['tier_10_skipped']=True
                    break
            stage_start=time.monotonic();report['stages'].append(dict(name=stage,target_z_n=target,start_ns=time.monotonic_ns()))
            print('STAGE',stage,'target_N',target,flush=True)
            if stage=='zero_pid':node._reset_control_state(time.monotonic_ns())
            while time.monotonic()-stage_start<duration:
                loop_start=time.monotonic_ns();node._handle_target_force_z(Float64(data=target))
                cycle=None
                if stage!='zero_native':
                    node._control_cycle()
                    if node.fault_latched:raise RuntimeError('PID node: '+node.fault_reason)
                    if node.last_cycle_result is not None and node.last_processed_cycle_seq==seq:
                        cycle=node.last_cycle_result
                target_n=node.last_axial_command_n if cycle is not None else node.pipeline.hold_forces_n.tolist()
                comp=node.pipeline.compensation;limits=node.drive.limits
                target_g=quantize_force_g_with_n_limits(n_to_g(target_n,comp.direction,comp.newtons_per_g),
                    limits.min_force_n,limits.max_force_n,comp.direction,comp.newtons_per_g)
                # Preserve the command and its unique synchronized feedback even
                # if the following live response triggers a displacement stop.
                command_context=dict(stage=stage,control_start_ns=loop_start,
                    pid_feedback_sequence=node.last_processed_cycle_seq if cycle else None,
                    target_z_n=target,reference_z_n=cycle.pid.reference_n if cycle else None,
                    measured_z_n=float(cycle.measured_wrench[2]) if cycle else None,
                    pid_output_n=cycle.pid.output_n if cycle else None,
                    tracking_error_n=cycle.pid.error_n if cycle else None,
                    pid_saturated=cycle.pid.saturated if cycle else None,
                    actuator_limited=cycle.actuator_limited if cycle else None,
                    hold_forces_g=holding_g,target_force_n=list(target_n),final_written_g=target_g,
                    allocated_delta_force_n=cycle.allocation.actuator_forces_n.tolist() if cycle else [0.]*6,
                    pose_m_rad=cycle.pose.tolist() if cycle else None)
                if cycle:
                    sync=node.synchronizer.query_sync_configuration(node.last_sync_stamp_ns)
                    command_context.update(measurement_ns=node.last_sync_stamp_ns,
                        aligned_force_g_raw=list(sync.forces_g_raw),
                        aligned_relative_force_n=node.pipeline.relative_feedback_n(sync.forces_g_raw).tolist(),
                        aligned_positions_m=list(sync.positions_meters),
                        measurement_left_ns=sync.per_axis_left_stamps_ns,measurement_right_ns=sync.per_axis_right_stamps_ns)
                with (out/'command_contexts.jsonl').open('a') as stream:
                    stream.write(json.dumps(command_context)+'\n')
                core.check_hardware_write_permission()
                audit=core.writer.send_force_cycle(target_g,cycle_id=cycle_id,target_timestamp_ns=loop_start)
                cycle_id+=1;seq+=1;feedback_message(audit)
                row=dict(time_iso=datetime.now().astimezone().isoformat(),monotonic_ns=time.monotonic_ns(),
                    control_start_ns=loop_start,stage=stage,source='REAL_HARDWARE',feedback_sequence=seq,pid_feedback_sequence=node.last_processed_cycle_seq,
                    target_z_n=target,reference_z_n=cycle.pid.reference_n if cycle else None,
                    measured_z_n=float(cycle.measured_wrench[2]) if cycle else None,
                    pid_output_n=cycle.pid.output_n if cycle else None,
                    pid_saturated=cycle.pid.saturated if cycle else None,
                    tracking_error_n=cycle.pid.error_n if cycle else None,
                    target_error_n=target-float(cycle.measured_wrench[2]) if cycle else None,
                    raw_feedback_g=list(core.forces_g_raw),raw_feedback_n=g_to_n(core.forces_g_raw),
                    hold_forces_g=holding_g,relative_feedback_g=[f-h for f,h in zip(core.forces_g_raw,holding_g)],
                    relative_feedback_n=node.pipeline.relative_feedback_n(core.forces_g_raw).tolist(),
                    target_force_n=list(target_n),final_written_g=target_g,positions_steps=list(core.positions),
                    currents_ma=list(core.currents),fault_codes=list(core.faults),force_valid=list(core.force_valid),
                    source_rx_ns=list(core.rx_ns),mode_last_readback=last_modes,mode_readback_ns=mode_stamp,
                    tx_write_duration_ms=[r.duration_ns/1e6 for r in audit.tx_records],
                    host_tx_to_rx_drain_ms=[(audit.feedback_rx_done_ns[r.motor_id]-r.tx_start_ns)/1e6 for r in audit.tx_records],
                    cycle_transaction_ms=(audit.cycle_end_ns-audit.cycle_start_ns)/1e6)
                if cycle:
                    sync=node.synchronizer.query_sync_configuration(node.last_sync_stamp_ns)
                    row.update(pose_m_rad=cycle.pose.tolist(),measurement_ns=node.last_sync_stamp_ns,
                        allocated_force_n=cycle.allocation.actuator_forces_n.tolist(),
                        allocated_delta_force_n=cycle.allocation.actuator_forces_n.tolist(),
                        aligned_relative_force_n=node.pipeline.relative_feedback_n(sync.forces_g_raw).tolist(),
                        limited_force_n=list(cycle.drive_result['limited_forces_n']),
                        aligned_force_g_raw=list(sync.forces_g_raw),force_n=comp.feedback_n(sync.forces_g_raw).tolist(),
                        aligned_positions_m=list(sync.positions_meters),measurement_left_ns=sync.per_axis_left_stamps_ns,
                        measurement_right_ns=sync.per_axis_right_stamps_ns,actuator_limited=cycle.actuator_limited)
                    report['pid_cycles']+=1
                row['control_elapsed_ms']=(time.monotonic_ns()-loop_start)/1e6
                row['control_start_interval_ms']=(loop_start-previous_loop_ns)/1e6 if 'previous_loop_ns' in locals() else None
                previous_loop_ns=loop_start
                records.append(row)
                with (out/'tracking.jsonl').open('a') as stream:stream.write(json.dumps(row)+'\n')
                if cycle_id%50==0:
                    with core.writer.uart_lock:
                        last_modes=[Motor(core.bus,mid).read_register(0x25).values[0] for mid in range(1,7)]
                    mode_stamp=time.monotonic_ns()
                    if last_modes!=[3]*6:raise RuntimeError('mode readback changed: '+str(last_modes))
                remaining=.02-(time.monotonic_ns()-loop_start)/1e9
                if remaining>0:time.sleep(remaining)
        report['outcome']='tier_one_complete_tier_two_skipped' if report.get('tier_10_skipped') else 'sequence_complete'
    except BaseException as exc:
        report['outcome']='stopped';report['error']=f'{type(exc).__name__}: {exc}'
        report['stopped_stage']=stage if 'stage' in locals() else 'startup'
        if 'command_context' in locals():report['last_command_context']=command_context
        if core is not None:
            report['mode_handover']=getattr(core.writer,'last_force_handover',None)
            report['last_feedback_at_stop']=dict(positions_steps=list(core.positions),currents_ma=list(core.currents),
                faults=list(core.faults),force_g_raw=list(core.forces_g_raw),force_valid=list(core.force_valid),
                rx_ns=list(core.rx_ns),feedback_sequence=seq)
        print('STOP',report['error'],flush=True)
    finally:
        if node:node.timer.cancel()
        if core:
            terminal=core.finish_force_control('stage3c_finished',fault=report.get('outcome')=='stopped')
            if terminal is not None:
                report['terminal_recovery']=terminal
                report['all_mode0_exit_confirmed']=terminal['position_hold_confirmed']
                report['hardware_stop_confirmed']=terminal['hardware_stop_confirmed']
                report['independent_stop_required']=terminal['independent_stop_required']
            core.close('stage3c_finished')
        if node:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        report['end_time']=datetime.now().astimezone().isoformat();save()
        print('RESULT',report['outcome'],'PID_CYCLES',report['pid_cycles'],
              'MODE0',report.get('all_mode0_exit_confirmed'),'DIR',out,flush=True)
    return report
