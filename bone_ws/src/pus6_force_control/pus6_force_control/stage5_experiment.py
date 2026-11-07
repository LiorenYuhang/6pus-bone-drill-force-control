"""Stage 5: same UART owner, native Mode3 ONLY, measured-position outer PID.

The first trials keep the six-step total excursion monitor. Planned posture
motion uses absolute stroke + reference-relative tracking error instead.
Normal execution writes only Mode 3 force targets. Terminal handling uses
verified same-frame Mode 0 anchoring and the existing explicit/fault Pause path. No Tare.
"""
from dataclasses import asdict
from datetime import datetime
import json
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions
from scipy.spatial.transform import Rotation
from std_srvs.srv import SetBool
from pus6_msgs.msg import TimedJointFeedback
from pus6_kinematics.geometry_config import load_geometry_config
from pus6_motor_driver.j14_process_uart import ServoCore
from pus6_motor_driver.j14_feedback_bridge import populate_force_feedback
from pus6_motor_driver.motor import Motor
from pus6_motor_driver.protocol import parse_frame
from pus6_motor_driver.force_units import g_to_n, n_to_g, quantize_force_g_with_n_limits
from .active_stabilization_node import ForceActiveStabilizationNode
from .force_pipeline import raw_steps_to_slider_m
from .six_axis_tare_experiment import SerialTrace
from .stage3c_experiment import ingest_snapshot_feedback


def json_default(value):
    return value.tolist() if isinstance(value,np.ndarray) else value.item()


class Mode3OnlyTrace(SerialTrace):
    allowed_write_registers = (0x25, 0x27, 0x1A)

    def enable_terminal_recovery(self):
        object.__setattr__(self, '_terminal_recovery', True)

    def write(self,data):
        frame=parse_frame(data)
        if frame.command==0x32:
            words=[int.from_bytes(frame.data[i:i+2],'little') for i in range(0,len(frame.data),2)]
            force_write=frame.register_addr==0x27 and len(words)==1
            entry_write=frame.register_addr==0x25 and len(words)==3 and words[:2]==[3,0]
            terminal = getattr(self, '_terminal_recovery', False)
            anchor_write = (terminal and frame.register_addr == 0x25 and len(words) == 5
                            and words[:2] == [0, 0] and words[3] == 0 and 0 <= words[4] <= 2000)
            pause_write = terminal and frame.register_addr == 0x1A and words == [1]
            if not (force_write or entry_write or anchor_write or pause_write):
                raise RuntimeError('Stage5 only permits Mode3 commands or explicit terminal recovery; no Tare')
        return super().write(data)


def stationary_position_window(rows, minimum_s=1., tolerance_steps=1):
    """Force matching is permitted only from an already stationary window.

    This checks observed standstill; it does not make force matching a brake.
    """
    if not rows:return False
    end=rows[-1]['ns'];recent=[r for r in rows if end-r['ns']<=int((minimum_s+.1)*1e9)]
    if not recent or end-recent[0]['ns']<int(minimum_s*1e9):return False
    if any(any(r['fault_codes']) for r in recent):return False
    return bool(np.max(np.ptp(np.asarray([r['positions_steps'] for r in recent]),axis=0))<=tolerance_steps)


def run(config):
    execute=config.get('execute',False)
    if execute and not (config.get('hardware_write_enabled') and config.get('is_hardware_qualified')
                        and config.get('dry_run') is False):
        raise ValueError('existing independent hardware execution permissions required')
    if config.get('hardware_mode')!='active' or not config.get('maintenance_only') or config.get('force_fault_recovery') not in ('position_reanchor','mode3_hold') or not config.get('mode3_only'):
        raise ValueError('Stage5 requires the existing real maintenance owner with explicit Mode3-only policy')
    out=Path(config['output_dir']);out.mkdir(parents=True,exist_ok=True)
    report=dict(source='REAL_HARDWARE',start_time=datetime.now().astimezone().isoformat(),
        version='tracking_v2' if config.get('tracking_v2') else 'stage5',outcome='started',snapshots=[],static_ready=False,disturbance_started=False,control_cycles=0)
    core=node=executor=trace=None;entered=False;sequence=0;stop=[False];imus=[];logs=[]
    initial_positions=None;initial_pose=None;device_limits=None;last_target_g=None;recent_rows=[]
    def save():
        if trace:
            report.update(hardware_write_count=len(trace.write_commands),
                tare_count=sum(w['register']==9 for w in trace.write_commands),
                pause_count=sum(w['register']==0x1A for w in trace.write_commands),
                mode0_write_count=sum(w['register']==0x25 and w['values'][0]==0 for w in trace.write_commands))
        (out/'experiment.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,default=json_default)+'\n')
    def record(name,row):
        with (out/name).open('a') as stream:stream.write(json.dumps(row,default=json_default)+'\n')
    def spin():
        for _ in range(12):executor.spin_once(timeout_sec=0.)
    def check(positions,currents,faults,forces):
        if any(faults):raise RuntimeError('hardware fault: '+str(faults))
        if max(abs(v) for v in currents)>config['max_current_ma']:raise RuntimeError('existing current protection')
        if device_limits is not None and any(not lo<=p<=hi for p,(lo,hi) in zip(positions,device_limits)):
            raise RuntimeError('actual normal/device stroke bound exceeded')
        if initial_positions is not None and config.get('small_trial',True) and max(abs(x-y) for x,y in zip(positions,initial_positions))>config.get('max_position_change_steps',6):
            raise RuntimeError('configured experiment displacement protection')
        if max(abs(v) for v in g_to_n(forces))>14.709975:raise RuntimeError('existing force feedback limit')
    def snapshot(label):
        nonlocal sequence
        rows=[];statuses={};rx={};start=time.monotonic_ns()
        with core.writer.uart_lock:
            for mid in range(1,7):
                m=Motor(core.bus,mid);s=m.read_status();rx[mid]=time.monotonic_ns();statuses[mid]=s
                words={r:m.read_register(r).values[0] for r in (0x25,0x27,0x2A,0x2C,0x2B,0x2F,0x23,0x24)}
                signed=lambda x:x-65536 if x>=32768 else x
                rows.append(dict(motor_id=mid,mode=words[0x25],target_force_g=signed(words[0x27]),
                    force_2c_g=signed(words[0x2C]),position_2a=words[0x2A],rx_done_ns=rx[mid],
                    register_words={hex(k):v for k,v in words.items()},**asdict(s)))
        if not core.feedback.stopped:
            sequence+=1;ingest_snapshot_feedback(core,statuses,rx,start,sequence)
        item=dict(label=label,ns=time.monotonic_ns(),time_iso=datetime.now().astimezone().isoformat(),axes=rows)
        report['snapshots'].append(item);record('snapshots.jsonl',item);save()
        return rows
    def feed(audit):
        nonlocal sequence
        sequence+=1
        feedback=core.feedback.ingest_cycle_feedback(audit.feedbacks,servo_cycle_id=sequence,
            servo_cycle_timestamp_ns=audit.cycle_start_ns,per_axis_rx_done_ns=audit.feedback_rx_done_ns)
        core.update(feedback)
        msg=TimedJointFeedback();msg.uart_cycle_sequence=sequence
        msg.snapshot_start_monotonic_ns=audit.cycle_start_ns;msg.snapshot_end_monotonic_ns=time.monotonic_ns()
        msg.per_axis_rx_done_monotonic_ns=list(core.rx_ns);msg.raw_steps=list(core.positions)
        msg.positions_meters=raw_steps_to_slider_m(core.positions,node.kinematics.geometry_config).tolist()
        msg.currents_ma=list(core.currents);msg.fault_codes=list(core.faults)
        populate_force_feedback(msg,core.force_feedback_fields())
        if msg.source!=msg.SOURCE_HARDWARE:raise RuntimeError('not real hardware feedback')
        node._handle_timed_feedback(msg)
    try:
        if execute:
            rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
            for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,lambda *_:stop.__setitem__(0,True))
            params=dict(config['controller_parameters'])
            params.update(dry_run=False,hardware_write_enabled=True,is_hardware_qualified=True,
                          execution_backend='joint_position_force',fault_stop_policy='external_position_reanchor')
            node=ForceActiveStabilizationNode(parameter_overrides=[Parameter(k,value=v) for k,v in params.items()])
            node.timer.cancel();executor=SingleThreadedExecutor();executor.add_node(node)
            original_imu=node._imu
            def imu(msg,history):
                previous_q_ns=history.samples[-1][0] if history.samples else None
                previous_g_ns=history.gyros[-1][0] if history.gyros else None
                original_imu(msg,history)
                # Preserve the actual timestamps used by ImuHistory. The
                # recorder timestamp is later and is not an interpolation stamp.
                orientation_ns=(history.samples[-1][0] if history.samples and history.samples[-1][0]!=previous_q_ns else None)
                gyro_ns=(history.gyros[-1][0] if history.gyros and history.gyros[-1][0]!=previous_g_ns else None)
                record('imu.jsonl',dict(ns=time.monotonic_ns(),orientation_receive_ns=orientation_ns,gyro_receive_ns=gyro_ns,
                    role='base' if history is node.base_imu else 'platform',
                    orientation_xyzw=[msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w],
                    angular_velocity_rad_s=[msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z],
                    orientation_valid=orientation_ns is not None,gyro_valid=gyro_ns is not None,
                    header_sec=msg.header.stamp.sec,header_nanosec=msg.header.stamp.nanosec))
            node._imu=imu
            # Exactly the existing Position H30 port mapping and executable.
            for role,serial_id in ([] if node.joint_execution_only else [('base','5B32028000'),('platform','5B0B029000')]):
                port='/dev/serial/by-id/usb-1a86_USB_Single_Serial_'+serial_id+'-if00'
                log=(out/(role+'_imu.log')).open('w');logs.append(log)
                command=[config['imu_executable'],'--ros-args','-r',f'__ns:=/pus6/imu_{role}',
                    '-p',f'port:={port}','-p','baud_rate:=460800','-p',f'frame_id:=pus6_{role}_imu_link',
                    '-p','world_frame:=world','-p',f'imu_topic:=/pus6/imu_{role}/data']
                imus.append(subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT))
            deadline=time.monotonic()+8.
            while time.monotonic()<deadline:
                if node.joint_execution_only:break
                spin()
                if all(h.latest_complete_ns and time.monotonic_ns()-h.latest_complete_ns<100_000_000 for h in (node.base_imu,node.platform_imu)):break
                if any(p.poll() is not None for p in imus):raise RuntimeError('H30 process exited')
                time.sleep(.005)
            else:raise RuntimeError('no fresh dual IMU input; no motor mode write')
            report['effective_stabilization_config']=asdict(node.stabilizer.config)
        core=ServoCore(config);core.bus.timeout_s=.05
        trace=Mode3OnlyTrace(core.serial,out);core.serial=trace;core.bus.serial=trace
        core.initialize()  # existing maintenance startup is strictly read-only
        before=snapshot('before_mode3');initial_positions=[r['current_position'] for r in before]
        report['initial_positions_steps']=initial_positions
        initial_modes=[r['mode'] for r in before]
        if initial_modes not in ([0]*6,[3]*6):raise RuntimeError('mixed/unknown initial modes; read-only stop')
        device_limits=[];geometry=load_geometry_config()
        for r in before:
            lo=max(geometry.normal_motion_min_steps,r['register_words']['0x24']);hi=min(geometry.normal_motion_max_steps,r['register_words']['0x23'])
            if lo>=hi:raise RuntimeError('invalid actual device stroke bounds')
            device_limits.append((lo,hi))
        report['effective_stroke_limits_steps']=device_limits
        if not execute:
            for k in range(2):
                time.sleep(.3);snapshot('read_only_stationary_'+str(k))
            report['outcome']='read_only_complete';return report
        # Reflect the same device intersection in the numeric controller.
        from dataclasses import replace
        from .joint_position_force_pid import JointPositionForcePID
        geom=node.kinematics.geometry_config
        joint_config=replace(node.joint_pid.config,
            joint_min_position_m=tuple(raw_steps_to_slider_m([v[0] for v in device_limits],geom)),
            joint_max_position_m=tuple(raw_steps_to_slider_m([v[1] for v in device_limits],geom)))
        node.joint_pid=JointPositionForcePID(joint_config,clock=node.clock)
        report['effective_joint_pid_config']=asdict(joint_config)
        check(initial_positions,[r['current_mA'] for r in before],[r['error_code'] for r in before],[r['force_g'] for r in before])
        print('INITIAL',[(r['motor_id'],r['mode'],r['current_position'],r['force_g'],r['current_mA'],r['error_code']) for r in before],flush=True)
        if not execute:
            report['outcome']='read_only_complete';return report
        spin()
        q=raw_steps_to_slider_m(initial_positions,node.kinematics.geometry_config)
        fk=node.kinematics.forward_kinematics_with_metrics(q)
        if not fk.converged:raise RuntimeError('initial actual-position FK failed')
        initial_pose=np.array(fk.pose);report['initial_pose_m_rad']=initial_pose.tolist()
        handover=core.prepare_force_handover();hold=handover['holding_forces_g']
        report.update(hold_forces_g=hold,handover_axes=handover['axes']);save()
        def verify_entry(mid):
            rows=snapshot('after_mode3_axis_'+str(mid));spin()
            check([r['current_position'] for r in rows],[r['current_mA'] for r in rows],
                  [r['error_code'] for r in rows],[r['force_g'] for r in rows])
        core.enter_force_mode_with_handover(hold,static_baseline_g=hold,axis_verifier=verify_entry)
        entered=True;last_target_g=list(hold);report['mode_handover']=core.writer.last_force_handover
        active=snapshot('all_mode3_hold_targets');spin()
        if any(r['mode']!=3 or r['target_force_g']!=h for r,h in zip(active,hold)):raise RuntimeError('Mode 3 / hold readback mismatch')
        check([r['current_position'] for r in active],[r['current_mA'] for r in active],
              [r['error_code'] for r in active],[r['force_g'] for r in active])
        report['all_mode3_confirmed']=True
        request=SetBool.Request();request.data=True;node._enable(request,SetBool.Response())
        begin=time.monotonic();last_modes_ns=time.monotonic_ns();modes=[3]*6;previous=None;phase='static';disturb_start=None
        cycle_id=0;trial_index=0;trial_start=None;trial_samples=[];trials=[]
        v2=bool(config.get('tracking_v2'));plan=list(config.get('phase_plan',[]));ending=False;ending_ns=None;ending_target=None;ending_reason=None;stable_since=None
        for axis in config.get('trial_axes',list(range(6))):
            for step in (2,0,-2,0):trials.append((axis,step))
        if config.get('static_only',False):trials=[]
        save()
        while (v2 or not stop[0]) and (time.monotonic()-begin<config.get('maximum_seconds',120.) or (v2 and ending)):
            start=time.monotonic_ns();spin()
            if node.fault_latched:raise RuntimeError('active stabilization node: '+node.fault_reason)
            cycle=node.last_cycle_result
            if v2 and ending:
                node.joint_reference_override_m=ending_target.copy();phase='controlled_stop_hold'
            elif v2 and trial_start is not None and trial_index<len(plan):
                phase=plan[trial_index]['name']
                node.joint_reference_override_m=node.joint_capture_m+np.asarray(plan[trial_index]['offset_steps'])*.000025
            elif trial_start is not None and trial_index<len(trials):
                axis,step=trials[trial_index]
                wanted=node.joint_capture_m.copy();wanted[axis]+=step*.000025
                node.joint_reference_override_m=wanted
                phase=f'ID{axis+1}_target_{step:+d}_steps'
            elif (config.get('small_trial',True) or v2) and node.joint_capture_m is not None:
                node.joint_reference_override_m=node.joint_capture_m.copy()
            target_n=node.last_axial_command_n if cycle is not None else g_to_n(hold)
            target_g=quantize_force_g_with_n_limits(n_to_g(target_n),node.drive.limits.min_force_n,node.drive.limits.max_force_n)
            core.check_hardware_write_permission()
            context=dict(ns=start,command_index=cycle_id,phase=phase,feedback_sequence=node.last_processed_cycle_seq,hold_g=hold,
                target_force_n=list(target_n),target_force_g=target_g,command_computed_ns=node.last_control_cycle_ns,diagnostics=dict(node.stabilization_diagnostics))
            if cycle is not None and cycle.pose is not None:
                sync=node.synchronizer.query_sync_configuration(node.last_sync_stamp_ns)
                context.update(measurement_ns=node.last_sync_stamp_ns,aligned_force_g=list(sync.forces_g_raw),
                    aligned_relative_force_n=node.pipeline.relative_feedback_n(sync.forces_g_raw).tolist(),
                    aligned_positions_m=list(sync.positions_meters),pose_m_rad=cycle.pose.tolist(),
                    measured_wrench_n_nm=cycle.measured_wrench.tolist(),commanded_wrench_n_nm=cycle.commanded_wrench.tolist(),
                    allocated_delta_force_n=cycle.allocation.actuator_forces_n.tolist(),actuator_limited=cycle.actuator_limited)
            if cycle is not None:report['control_cycles']+=1
            record('command_contexts.jsonl',context)
            audit=core.writer.send_force_cycle(target_g,cycle_id=cycle_id,target_timestamp_ns=start)
            last_target_g=list(target_g);cycle_id+=1;feed(audit)
            # Unlike an independent ROS UART owner, this bounded maintenance
            # loop also performs the occasional register inventories. Compute
            # only AFTER fresh status ingestion, never query a future timestamp
            # from the old batch following a mode-register scan. Keep the same
            # Fixed-Lag delay, unique-feedback rule, PID and source timestamps.
            spin();node._control_cycle()
            if node.stabilizer.reference is not None and 'attitude_reference' not in report:
                pose0,base_q0,platform_q0=node.stabilizer.reference
                report['attitude_reference']=dict(pose_m_rad=pose0.tolist(),base_quat_xyzw=base_q0.tolist(),
                    platform_quat_xyzw=platform_q0.tolist(),measurement_ns=node.last_sync_stamp_ns)
            row=dict(ns=time.monotonic_ns(),phase=phase,source='REAL_HARDWARE',positions_steps=list(core.positions),
                force_g=list(core.forces_g_raw),force_n=g_to_n(core.forces_g_raw),currents_ma=list(core.currents),fault_codes=list(core.faults),
                force_valid=list(core.force_valid),rx_ns=list(core.rx_ns),mode_last_readback=modes,mode_readback_ns=last_modes_ns,
                target_force_g=target_g,sent_command_index=context['command_index'],
                sent_command_feedback_sequence=context['feedback_sequence'],
                sent_command_measurement_ns=context.get('measurement_ns'),
                next_command_feedback_sequence=node.last_processed_cycle_seq,
                next_command_measurement_ns=node.last_sync_stamp_ns,
                # Compatibility: this historical field describes NEXT computation.
                # Use sent_command_* to associate this row's actual 0x27 writes.
                control_sequence=node.last_processed_cycle_seq,
                cycle_transaction_ms=(audit.cycle_end_ns-audit.cycle_start_ns)/1e6,
                tx_to_status_rx_ms=[(audit.feedback_rx_done_ns[r.motor_id]-r.tx_start_ns)/1e6 for r in audit.tx_records],
                control_interval_ms=(start-previous)/1e6 if previous else None)
            record('tracking.jsonl',row);previous=start
            recent_rows.append(row)
            if len(recent_rows)>100:recent_rows.pop(0)
            check(core.positions,core.currents,core.faults,core.forces_g_raw)
            if core.feedback.stopped or core.writer.stopped or not all(core.force_valid):raise RuntimeError('transport/feedback stopped or invalid force')
            if node.fault_latched:raise RuntimeError('active stabilization node: '+node.fault_reason)
            if time.monotonic_ns()-last_modes_ns>1_000_000_000:
                with core.writer.uart_lock:modes=[Motor(core.bus,i).read_register(0x25).values[0] for i in range(1,7)]
                last_modes_ns=time.monotonic_ns();record('mode_checks.jsonl',dict(ns=last_modes_ns,modes=modes))
                if report.get('live_hold_active'):
                    report['live_hold_status']=dict(ns=last_modes_ns,modes=modes,positions_steps=list(core.positions),currents_ma=list(core.currents),fault_codes=list(core.faults))
                    save()
                if modes!=[3]*6:raise RuntimeError('Mode 3 lost: '+str(modes))
            if not report['static_ready'] and time.monotonic()-begin>=3. and cycle is not None:
                report.update(static_ready=True,static_ready_ns=time.monotonic_ns(),outcome='static_ready_waiting_for_operator')
                save();print('STATIC_READY: real Mode3 position-feedback static hold',flush=True)
                if plan or trials:trial_start=time.monotonic();phase='execution_trial'
                elif config.get('static_only',False):report['outcome']='static_complete';break
            if not v2 and trial_start is not None and trial_index<len(trials):
                trial_samples.append(dict(ns=row['ns'],q=list(core.positions),force_g=list(core.forces_g_raw),
                    target_g=list(target_g),current=list(core.currents)))
                if time.monotonic()-trial_start>=config.get('trial_seconds',3.):
                    axis,step=trials[trial_index]
                    target_steps=float((node.joint_capture_m[axis]-.05)/.000025+1000+step)
                    tail=trial_samples[-20:]
                    mean_error=float(np.mean([s['q'][axis]-target_steps for s in tail]))
                    result=dict(axis=axis+1,step=step,target_steps=target_steps,mean_tail_error_steps=mean_error,
                        range_steps=[min(s['q'][axis] for s in trial_samples),max(s['q'][axis] for s in trial_samples)],
                        current_peak_ma=max(s['current'][axis] for s in trial_samples),
                        force_range_g=[min(s['force_g'][axis] for s in trial_samples),max(s['force_g'][axis] for s in trial_samples)])
                    report.setdefault('trial_results',[]).append(result);save();print('TRIAL',result,flush=True)
                    if abs(mean_error)>1.01:raise RuntimeError('small position-force trial did not track within one step; no dynamic disturbance')
                    trial_index+=1;trial_samples=[];trial_start=time.monotonic()
                    if trial_index==len(trials):
                        report['execution_ready']=True;report['outcome']='execution_trials_complete';save()
                        print('EXECUTION_READY: small position-force trials passed',flush=True)
                        if not config.get('allow_manual_disturbance',False):break
                        config['small_trial']=False;node.displacement_steps=0;node.joint_reference_override_m=None
                        phase='waiting_for_manual_disturbance'
            if v2:
                if not ending and trial_start is not None and trial_index<len(plan):
                    trial_samples.append(row)
                    current_plan=plan[trial_index]
                    elapsed=time.monotonic()-trial_start
                    wanted_now=(node.joint_capture_m-.05)/.000025+1000+np.asarray(current_plan['offset_steps'])
                    tail_now=trial_samples[-40:]
                    settled_now=(len(tail_now)>=40 and np.max(abs(np.mean([r['positions_steps'] for r in tail_now],axis=0)-wanted_now))<=config.get('tracking_tolerance_steps',2.)+1e-6
                        and np.max(np.ptp([r['positions_steps'] for r in tail_now],axis=0))<=1)
                    if (elapsed>=current_plan.get('seconds',8.) and settled_now) or elapsed>=current_plan.get('maximum_seconds',current_plan.get('seconds',8.)):

                        wanted=(node.joint_capture_m-.05)/.000025+1000+np.asarray(current_plan['offset_steps'])
                        tail=trial_samples[-40:]
                        error=np.array([r['positions_steps'] for r in tail])-wanted
                        result=dict(name=current_plan['name'],target_steps=wanted.tolist(),elapsed_seconds=elapsed,
                            mean_tail_error_steps=error.mean(0).tolist(),max_tail_error_steps=abs(error).max(0).tolist(),
                            tail_range_steps=np.ptp([r['positions_steps'] for r in tail],axis=0).tolist(),
                            current_peak_ma=np.max([r['currents_ma'] for r in trial_samples],axis=0).tolist())
                        report.setdefault('trial_results',[]).append(result);save();print('TRIAL',result,flush=True)
                        if np.max(abs(error.mean(0)))>config.get('tracking_tolerance_steps',2.)+1e-6 or np.max(np.ptp([r['positions_steps'] for r in tail],axis=0))>2:
                            ending_reason='tracking_failed';ending=True
                        else:
                            trial_index+=1;trial_samples=[];trial_start=time.monotonic()
                            if trial_index==len(plan):ending_reason='sequence_complete';ending=True
                # Replacing a normal stop by closing the force stream was wrong:
                # freeze a measured-position reference and keep correcting/braking.
                if not ending and ((out/'stop_request.json').exists() or stop[0] or time.monotonic()-begin>=config.get('maximum_seconds',120.)-.5):
                    ending_reason='operator_stop' if (out/'stop_request.json').exists() or stop[0] else 'time_limit';ending=True
                if not ending and np.max(abs(np.array(core.positions)-np.array(initial_positions)))>config.get('soft_stop_excursion_steps',40):
                    ending_reason='soft_excursion_stop';ending=True
                if report.get('live_hold_active') and (stop[0] or (out/'stop_request.json').exists()):
                    report['outcome']='operator_exit_from_active_hold';break
                if ending:
                    if ending_ns is None:
                        ending_ns=time.monotonic_ns();ending_target=raw_steps_to_slider_m(core.positions,node.kinematics.geometry_config)
                        node.joint_pid.reference.reset(ending_target,now_ns=node.clock())
                        report['controlled_stop_target_steps']=list(core.positions);report['controlled_stop_reason']=ending_reason
                        report['controlled_stop_start_ns']=ending_ns;save()
                        print('CONTROLLED_STOP',ending_reason,'target',list(core.positions),flush=True)
                    # Wait for actual settled position while still sending fresh
                    # feedback-based force commands, not a zero/measured-force substitution.
                    stationary=stationary_position_window(recent_rows,minimum_s=1.,tolerance_steps=1)
                    stop_error=np.max(abs(raw_steps_to_slider_m(core.positions,node.kinematics.geometry_config)-ending_target))/.000025
                    if stationary and stop_error<=config.get('tracking_tolerance_steps',2.):
                        if stable_since is None:stable_since=time.monotonic()
                    else:stable_since=None
                    if stable_since is not None and time.monotonic()-stable_since>=config.get('settled_hold_seconds',3.):
                        report['outcome']=ending_reason+'_controlled_stop';report['position_hold_confirmed']=True
                        if config.get('hold_after_experiment',False) and not stop[0] and not (out/'stop_request.json').exists():
                            if not report.get('live_hold_active'):
                                report.update(live_hold_active=True,experiment_completed_ns=time.monotonic_ns(),
                                    outcome=ending_reason+'_controlled_hold_active')
                                save();print('EXPERIMENT_COMPLETE_MODE3_POSITION_HOLD_ACTIVE',flush=True)
                        else:break
                    if not report.get('live_hold_active') and (time.monotonic_ns()-ending_ns)>int(config.get('stop_timeout_seconds',12.)*1e9):
                        raise RuntimeError('Mode3 position braking/hold did not settle; independent onsite stop required')
                if cycle is None and time.monotonic()-begin>2.:raise RuntimeError('no position controller output')
                time.sleep(max(0.,.02-(time.monotonic_ns()-start)/1e9))
                continue
            if (out/'disturbance_start.json').exists() and report.get('execution_ready') and disturb_start is None:
                phase='manual_disturbance';disturb_start=time.monotonic()
                report.update(disturbance_started=True,disturbance_start_ns=time.monotonic_ns());save()
                print('MANUAL_DISTURBANCE_RECORDING',flush=True)
            if (out/'stop_request.json').exists():report['outcome']='operator_stopped';break
            if disturb_start is not None and time.monotonic()-disturb_start>=config.get('disturbance_seconds',60.):
                report['outcome']='disturbance_window_complete';break
            if cycle is None and time.monotonic()-begin>2.:raise RuntimeError('active node has not produced a control cycle')
            time.sleep(max(0.,.02-(time.monotonic_ns()-start)/1e9))
        if v2:pass
        elif stop[0]:report['outcome']='signal_stopped'
        elif report['outcome'] not in ('operator_stopped','disturbance_window_complete','execution_trials_complete','static_complete'):report['outcome']='bounded_window_complete'
    except BaseException as exc:
        report.update(outcome='stopped',error=f'{type(exc).__name__}: {exc}')
        report['hardware_stop_confirmed']=False;report['independent_stop_required']=bool(entered or any(a['mode']==3 for s in report['snapshots'] for a in s['axes']))
        print('STOP_NOT_CONFIRMED',report['error'],flush=True)
    finally:
        if node:node.timer.cancel();node.enabled=False
        if core:
            # The owner performs terminal recovery even if the writer was
            # already fault-latched. Closing the port is never the stop action.
            try:
                terminal = core.finish_force_control('stage5_finished',
                    fault=report.get('outcome') == 'stopped')
                if terminal is not None:
                    report['terminal_recovery'] = terminal
                    report['hardware_stop_confirmed'] = terminal['hardware_stop_confirmed']
                    report['independent_stop_required'] = terminal['independent_stop_required']
                    report['all_mode0_final_confirmed'] = terminal['position_hold_confirmed']
                    report['final_positions_steps'] = [a.get('observations', [{}])[-1].get('position_steps')
                        for a in terminal['axes']]
            except Exception as exc:
                report.update(exit_error=str(exc),hardware_stop_confirmed=False,independent_stop_required=True)
            core.close('stage5_finished')
        if executor:executor.shutdown()
        if node:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        for process in imus:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:process.wait(timeout=3.)
                except subprocess.TimeoutExpired:process.terminate();process.wait(timeout=2.)
        for log in logs:log.close()
        report.update(end_time=datetime.now().astimezone().isoformat(),serial_closed=bool(core and core.closed))
        save();print('RESULT',report['outcome'],'MODE0',report.get('all_mode0_final_confirmed'),'DIR',out,flush=True)
    return report
