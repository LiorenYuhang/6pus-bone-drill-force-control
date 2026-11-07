"""Bounded Stage 4 experiment in the existing maintenance UART owner.

Reuses Stage 3C's actual status ingestion, trace, handover and exit. The formal
active-stabilization node is evaluated once per fresh feedback, as in Stage 3C.
No Tare, gain changes, position motion, automatic Pause or extra UART owner.
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
from pus6_motor_driver.j14_process_uart import ServoCore
from pus6_motor_driver.j14_feedback_bridge import populate_force_feedback
from pus6_motor_driver.motor import Motor
from pus6_motor_driver.force_units import g_to_n, n_to_g, quantize_force_g_with_n_limits
from .active_stabilization_node import ForceActiveStabilizationNode
from .force_pipeline import raw_steps_to_slider_m
from .six_axis_tare_experiment import SerialTrace
from .stage3c_experiment import ingest_snapshot_feedback


def json_default(value):
    return value.tolist() if isinstance(value,np.ndarray) else value.item()


def run(config):
    execute=config.get('execute',False)
    if execute and not (config.get('hardware_write_enabled') and config.get('is_hardware_qualified')
                        and config.get('dry_run') is False):
        raise ValueError('existing independent hardware execution permissions required')
    if config.get('hardware_mode') not in ('active', 'mock') or not config.get('maintenance_only') or config.get('force_fault_recovery')!='position_reanchor':
        raise ValueError('only existing real maintenance UART owner and verified recovery are supported')
    out=Path(config['output_dir']);out.mkdir(parents=True,exist_ok=True)
    report=dict(source='REAL_HARDWARE',start_time=datetime.now().astimezone().isoformat(),
        outcome='started',snapshots=[],static_ready=False,disturbance_started=False,control_cycles=0)
    core=node=executor=trace=None;entered=False;sequence=0;stop=[False];imus=[];logs=[]
    initial_positions=None;initial_pose=None
    def save():
        if trace:
            report.update(hardware_write_count=len(trace.write_commands),
                tare_count=sum(w['register']==9 for w in trace.write_commands),
                pause_count=sum(w['register']==0x1A for w in trace.write_commands))
        (out/'experiment.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,default=json_default)+'\n')
    def record(name,row):
        with (out/name).open('a') as stream:stream.write(json.dumps(row,default=json_default)+'\n')
    def spin():
        for _ in range(12):executor.spin_once(timeout_sec=0.)
    def check(positions,currents,faults,forces):
        if any(faults):raise RuntimeError('hardware fault: '+str(faults))
        if max(abs(v) for v in currents)>config['max_current_ma']:raise RuntimeError('existing current protection')
        if initial_positions is not None and max(abs(x-y) for x,y in zip(positions,initial_positions))>6:
            raise RuntimeError('existing 6 steps / 0.15 mm displacement protection')
        if max(abs(v) for v in g_to_n(forces))>14.709975:raise RuntimeError('existing force feedback limit')
    def snapshot(label):
        nonlocal sequence
        rows=[];statuses={};rx={};start=time.monotonic_ns()
        with core.writer.uart_lock:
            for mid in range(1,7):
                m=Motor(core.bus,mid);s=m.read_status();rx[mid]=time.monotonic_ns();statuses[mid]=s
                words={r:m.read_register(r).values[0] for r in (0x25,0x27,0x2A,0x2C,0x2B,0x2F)}
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
        if config.get('hardware_mode') == 'active' and msg.source != msg.SOURCE_HARDWARE:
            raise RuntimeError('not real hardware feedback')
        node._handle_timed_feedback(msg)
    try:
        if execute:
            rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
            for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,lambda *_:stop.__setitem__(0,True))
            params=dict(config['controller_parameters'])
            params.update(dry_run=False,hardware_write_enabled=True,is_hardware_qualified=True,
                          fault_stop_policy='external_position_reanchor',
                          allow_mock_hardware=(config.get('hardware_mode') == 'mock'),
                          max_current_ma=config.get('max_current_ma', 500))
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
            if config.get('launch_imus', True):
                for role,serial_id in [('base','5B32028000'),('platform','5B0B029000')]:
                    port = config.get(f'{role}_imu_port')
                    if not port:
                        by_id = f'/dev/serial/by-id/usb-1a86_USB_Single_Serial_{serial_id}-if00'
                        if Path(by_id).exists():
                            port = by_id
                        elif role == 'base' and Path('/dev/ttyACM1').exists():
                            port = '/dev/ttyACM1'
                        elif role == 'platform' and Path('/dev/ttyACM0').exists():
                            port = '/dev/ttyACM0'
                        else:
                            port = by_id
                    log=(out/(role+'_imu.log')).open('w');logs.append(log)
                    command=[config['imu_executable'],'--ros-args','-r',f'__ns:=/pus6/imu_{role}',
                        '-p',f'port:={port}','-p','baud_rate:=460800','-p',f'frame_id:=pus6_{role}_imu_link',
                        '-p','world_frame:=world','-p',f'imu_topic:=/pus6/imu_{role}/data']
                    imus.append(subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT))
                deadline=time.monotonic()+8.
                while time.monotonic()<deadline:
                    spin()
                    if all(h.latest_complete_ns and time.monotonic_ns()-h.latest_complete_ns<100_000_000 for h in (node.base_imu,node.platform_imu)):break
                    if any(p.poll() is not None for p in imus):raise RuntimeError('H30 process exited')
                    time.sleep(.005)
                else:raise RuntimeError('no fresh dual IMU input; no motor mode write')
            report['effective_stabilization_config']=asdict(node.stabilizer.config)
        core=ServoCore(config);core.bus.timeout_s=.05
        if core.serial is not None:
            trace=SerialTrace(core.serial,out);core.serial=trace;core.bus.serial=trace
        else:
            trace=None
        core.initialize()  # existing maintenance startup is strictly read-only
        before=snapshot('before_mode3');initial_positions=[r['current_position'] for r in before]
        report['initial_positions_steps']=initial_positions
        if any(r['mode']!=0 for r in before):raise RuntimeError('initial mode is not uniformly Mode 0')
        check(initial_positions,[r['current_mA'] for r in before],[r['error_code'] for r in before],[r['force_g'] for r in before])
        print('INITIAL',[(r['motor_id'],r['mode'],r['current_position'],r['force_g'],r['current_mA'],r['error_code']) for r in before],flush=True)
        if not execute:
            report['outcome']='read_only_complete';return report
        spin()
        q=raw_steps_to_slider_m(initial_positions,node.kinematics.geometry_config)
        fk=node.kinematics.forward_kinematics_with_metrics(q)
        if not fk.converged:raise RuntimeError('initial actual-position FK failed')
        initial_pose=np.array(fk.pose);report['initial_pose_m_rad']=initial_pose.tolist()
        # Feasibility information only. These are NEVER sent as position commands.
        report['position_reference_stroke_comparison']=[]
        for axis in range(3):
            for deg in (10.,20.):
                pose=initial_pose.copy()
                pose[3:]=Rotation.from_matrix(Rotation.from_rotvec(-np.eye(3)[axis]*np.radians(deg)).as_matrix()@Rotation.from_euler('xyz',initial_pose[3:]).as_matrix()).as_euler('xyz')
                try:
                    wanted=node.kinematics.inverse_kinematics(pose)
                    report['position_reference_stroke_comparison'].append(dict(axis=axis,base_angle_deg=deg,
                        max_actuator_change_steps=float(np.max(np.abs(np.array(wanted)-q))/.000025)))
                except Exception as exc:report['position_reference_stroke_comparison'].append(dict(axis=axis,base_angle_deg=deg,error=str(exc)))
        handover=core.prepare_force_handover();hold=handover['holding_forces_g']
        report.update(hold_forces_g=hold,handover_axes=handover['axes']);save()
        def verify_entry(mid):
            rows=snapshot('after_mode3_axis_'+str(mid));spin()
            check([r['current_position'] for r in rows],[r['current_mA'] for r in rows],
                  [r['error_code'] for r in rows],[r['force_g'] for r in rows])
        core.enter_force_mode_with_handover(hold,static_baseline_g=hold,axis_verifier=verify_entry)
        entered=True;report['mode_handover']=core.writer.last_force_handover
        active=snapshot('all_mode3_hold_targets');spin()
        if any(r['mode']!=3 or r['target_force_g']!=h for r,h in zip(active,hold)):raise RuntimeError('Mode 3 / hold readback mismatch')
        check([r['current_position'] for r in active],[r['current_mA'] for r in active],
              [r['error_code'] for r in active],[r['force_g'] for r in active])
        report['all_mode3_confirmed']=True
        request=SetBool.Request();request.data=True;node._enable(request,SetBool.Response())
        begin=time.monotonic();last_modes_ns=time.monotonic_ns();modes=[3]*6;previous=None;phase='static';disturb_start=None
        cycle_id=0;save()
        while not stop[0] and time.monotonic()-begin<config.get('maximum_seconds',120.):
            start=time.monotonic_ns();spin()
            if node.fault_latched:raise RuntimeError('active stabilization node: '+node.fault_reason)
            cycle=node.last_cycle_result
            target_n=node.last_axial_command_n if cycle is not None else g_to_n(hold)
            target_g=quantize_force_g_with_n_limits(n_to_g(target_n),node.drive.limits.min_force_n,node.drive.limits.max_force_n)
            core.check_hardware_write_permission()
            context=dict(ns=start,command_index=cycle_id,phase=phase,feedback_sequence=node.last_processed_cycle_seq,hold_g=hold,
                target_force_n=list(target_n),target_force_g=target_g,diagnostics=dict(node.stabilization_diagnostics))
            if cycle is not None:
                sync=node.synchronizer.query_sync_configuration(node.last_sync_stamp_ns)
                context.update(measurement_ns=node.last_sync_stamp_ns,aligned_force_g=list(sync.forces_g_raw),
                    aligned_relative_force_n=node.pipeline.relative_feedback_n(sync.forces_g_raw).tolist(),
                    aligned_positions_m=list(sync.positions_meters),pose_m_rad=cycle.pose.tolist(),
                    measured_wrench_n_nm=cycle.measured_wrench.tolist(),commanded_wrench_n_nm=cycle.commanded_wrench.tolist(),
                    allocated_delta_force_n=cycle.allocation.actuator_forces_n.tolist(),actuator_limited=cycle.actuator_limited)
                report['control_cycles']+=1
            record('command_contexts.jsonl',context)
            audit=core.writer.send_force_cycle(target_g,cycle_id=cycle_id,target_timestamp_ns=start)
            cycle_id+=1;feed(audit)
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
            check(core.positions,core.currents,core.faults,core.forces_g_raw)
            if core.feedback.stopped or core.writer.stopped or not all(core.force_valid):raise RuntimeError('transport/feedback stopped or invalid force')
            if node.fault_latched:raise RuntimeError('active stabilization node: '+node.fault_reason)
            if time.monotonic_ns()-last_modes_ns>1_000_000_000:
                with core.writer.uart_lock:modes=[Motor(core.bus,i).read_register(0x25).values[0] for i in range(1,7)]
                last_modes_ns=time.monotonic_ns();record('mode_checks.jsonl',dict(ns=last_modes_ns,modes=modes))
                if modes!=[3]*6:raise RuntimeError('Mode 3 lost: '+str(modes))
            if not report['static_ready'] and time.monotonic()-begin>=5. and cycle is not None:
                report.update(static_ready=True,static_ready_ns=time.monotonic_ns(),outcome='static_ready_waiting_for_operator')
                save();print('STATIC_READY: five-second real Mode 3 active static hold; waiting for operator disturbance',flush=True)
            if (out/'disturbance_start.json').exists() and report['static_ready'] and disturb_start is None:
                phase='manual_disturbance';disturb_start=time.monotonic()
                report.update(disturbance_started=True,disturbance_start_ns=time.monotonic_ns());save()
                print('MANUAL_DISTURBANCE_RECORDING',flush=True)
            if (out/'stop_request.json').exists():report['outcome']='operator_stopped';break
            if disturb_start is not None and time.monotonic()-disturb_start>=config.get('disturbance_seconds',60.):
                report['outcome']='disturbance_window_complete';break
            if cycle is None and time.monotonic()-begin>2.:raise RuntimeError('active node has not produced a control cycle')
            time.sleep(max(0.,.02-(time.monotonic_ns()-start)/1e9))
        if stop[0]:report['outcome']='signal_stopped'
        elif report['outcome'] not in ('operator_stopped','disturbance_window_complete'):report['outcome']='bounded_window_complete'
    except BaseException as exc:
        report.update(outcome='stopped',error=f'{type(exc).__name__}: {exc}')
        print('STOP',report['error'],flush=True)
    finally:
        if node:node.timer.cancel();node.enabled=False
        if core:
            terminal=core.finish_force_control('stage4_finished',fault=report.get('outcome')=='stopped')
            if terminal is not None:
                report['terminal_recovery']=terminal
                report['all_mode0_exit_confirmed']=terminal['position_hold_confirmed']
                report['hardware_stop_confirmed']=terminal['hardware_stop_confirmed']
                report['independent_stop_required']=terminal['independent_stop_required']
            core.close('stage4_finished')
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
        save();print('RESULT',report['outcome'],'MODE0',report.get('all_mode0_exit_confirmed'),'DIR',out,flush=True)
    return report


def main(argv=None):
    import argparse
    import yaml
    parser = argparse.ArgumentParser(description="6PUS Mode 3 Real Hardware Active Stabilization PID Runner")
    parser.add_argument('--hardware', action='store_true', help='Enable real hardware Mode 3 force control write interface')
    parser.add_argument('--execute', action='store_true', help='Alias for --hardware')
    parser.add_argument('--hardware-qualified', action='store_true', help='Explicit hardware qualification confirmation')
    parser.add_argument('--dry-run', action='store_true', help='Dry-run mode: read-only motor audit, zero hardware writes')
    parser.add_argument('--port', default=None, help='Motors UART port (default: autodetect /dev/serial/by-id or /dev/ttyUSB0)')
    parser.add_argument('--baudrate', type=int, default=921600, help='Motors UART baudrate')
    parser.add_argument('--base-imu-port', default=None, help='Base IMU port (default: autodetect)')
    parser.add_argument('--plat-imu-port', default=None, help='Platform IMU port (default: autodetect)')
    parser.add_argument('--params-file', default=None, help='Active stabilization YAML config file')
    parser.add_argument('--output-dir', default=None, help='Output directory for experiment data')
    parser.add_argument('--duration', type=float, default=120.0, help='Maximum run duration in seconds')
    parser.add_argument('--mock', action='store_true', help='Run in mock mode (using MockServoBus for testing)')
    parser.add_argument('--no-launch-imus', action='store_true', help='Do not launch IMU subprocesses')
    args = parser.parse_args(argv)

    is_hardware = (args.hardware or args.execute) and not args.dry_run
    if is_hardware and not args.hardware_qualified:
        parser.error('--hardware requires explicit --hardware-qualified authorization')
    if is_hardware and args.mock:
        parser.error('--hardware cannot run with --mock (real hardware execution cannot be spoofed by mock)')

    root = Path(__file__).resolve().parents[4]
    params_path = Path(args.params_file) if args.params_file else (root / 'bone_ws/src/pus6_force_control/config/active_stabilization.yaml')
    if not params_path.exists():
        params_path = root / 'bone_ws/install/pus6_force_control/share/pus6_force_control/config/active_stabilization.yaml'

    controller_params = {}
    if params_path.exists():
        with params_path.open() as f:
            y = yaml.safe_load(f)
            controller_params = y.get('force_active_stabilization', {}).get('ros__parameters', {})

    out_dir = Path(args.output_dir) if args.output_dir else (root / 'results/stage4_active_stabilization' / datetime.now().strftime('%Y%m%d_%H%M%S'))

    imu_exec = root / 'bone_ws/install/h30_imu/lib/h30_imu/h30_imu_node'
    if not imu_exec.exists():
        imu_exec = root / 'bone_ws/build/h30_imu/h30_imu_node'

    motor_port = args.port
    if not motor_port:
        by_id_motor = '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'
        if Path(by_id_motor).exists():
            motor_port = by_id_motor
        elif Path('/dev/ttyUSB0').exists():
            motor_port = '/dev/ttyUSB0'
        else:
            motor_port = by_id_motor

    config = dict(
        epoch=int(time.time()),
        hardware_mode='mock' if args.mock else 'active',
        port=motor_port,
        baudrate=args.baudrate,
        output_dir=str(out_dir),
        control_mode='force',
        maintenance_only=True,
        dry_run=not is_hardware,
        execute=is_hardware,
        hardware_write_enabled=is_hardware,
        is_hardware_qualified=bool(args.hardware_qualified if is_hardware else False),
        force_fault_recovery='position_reanchor',
        max_current_ma=500,
        max_position_change_steps=6,
        max_force_handover_displacement_steps=6,
        min_force_n=controller_params.get('min_force_n', [-14.709975] * 6),
        max_force_n=controller_params.get('max_force_n', [14.709975] * 6),
        max_rate_g_s=controller_params.get('max_rate_g_s', [500.0] * 6),
        controller_parameters=controller_params,
        imu_executable=str(imu_exec),
        base_imu_port=args.base_imu_port,
        platform_imu_port=args.plat_imu_port,
        maximum_seconds=args.duration,
        disturbance_seconds=60.0,
        launch_imus=not args.no_launch_imus,
    )

    result = run(config)
    print(f"OUTCOME: {result.get('outcome')} | MODE0_EXIT: {result.get('all_mode0_exit_confirmed')} | WRITES: {result.get('hardware_write_count', 0)}")
    return 0 if result.get('outcome') in ('bounded_window_complete', 'operator_stopped', 'read_only_complete', 'signal_stopped', 'disturbance_window_complete') else 2


if __name__ == '__main__':
    raise SystemExit(main())
