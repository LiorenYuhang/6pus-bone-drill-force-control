"""Dual-IMU Parent; J14 ProcessClient owns the UART Child and Feedback Bridge."""
from dataclasses import asdict
import json
from pathlib import Path
import signal
import time
import numpy as np

from pus6_motor_driver.j14_process_uart import ProcessClient
from .mode3_dual_imu import DualImuPlan, DualImuMotionConfig
from .z_position_motion import pid_config, slider_to_steps
from .mode3_process_policy import PID_OVERRIDES, force_limits, policy_summary


def run(config,settings,*,source,parameters,axes,duration):
    out=Path(config['output_dir']);stop=[False];handlers={};client=None;plan=None
    parent_error=None;outcome=None;status=None;phase=None;phase_start=None;stages=[]
    for sig in (signal.SIGINT,signal.SIGTERM):
        handlers[sig]=signal.signal(sig,lambda *_:stop.__setitem__(0,True))
    limits=force_limits()
    config=dict(config,joint_position_pid=True,production=True,feedback_bridge=True,
        enable_coordinated_handshake=True,telemetry_mode='production',telemetry_capacity_cycles=4096,
        max_servo_duration_s=0.,max_rate_g_s=None,max_current_ma=settings.max_current_ma,
        min_force_n=list(limits.min_force_n),max_force_n=list(limits.max_force_n),
        imu_age_limits_s=dict(base=parameters['max_base_imu_age_s'],platform=parameters['max_platform_imu_age_s']),
        motion_settings=asdict(settings))
    try:
        client=ProcessClient(config)
        plan=DualImuPlan(client.ready['positions'],settings,source=source,parameters=parameters,
            axes=axes,duration_s=duration,device_limits=client.ready['device_limits'])
        plan.pid_overrides=dict(plan.pid_overrides,**PID_OVERRIDES)
        summary=plan.summary()
        print(plan.console_plan(),flush=True)
        (out/'motion_parent_configuration.json').write_text(json.dumps(dict(config=config,plan=summary,
            effective_pid=asdict(pid_config(plan)), output_policy=policy_summary()),indent=2)+'\n')
        last_console=0;last_phase=None;last_status_ns=time.monotonic_ns()
        while True:
            begin=time.monotonic_ns()
            if stop[0] or (out/'stop_request.json').exists():
                outcome='operator_stopped';client.stop('operator_stopped');break
            for row in client.receive():
                if row.get('type')=='motion_status':status=row;last_status_ns=begin
            if client.proc.poll() is not None or client.stopped:
                raise RuntimeError(client.stop_reason or 'UART Child exited')
            if client.bridge_proc.poll() is not None:raise RuntimeError('Feedback Bridge exited')
            # Reuse J14's 1000 ms peer-health boundary; no new motion envelope.
            if begin-last_status_ns>1_000_000_000:raise RuntimeError('Feedback Bridge status timeout')
            # Initial hold starts when Mode 3 AND a fresh post-handover IMU/IK
            # vector are ready, not at the earlier process-start handshake.
            started=status.get('reference_ready_ns') if status else None
            if started and phase is None:
                phase='initial_hold';phase_start=started
                print('MODE3_CONFIRMED: six axes',flush=True)
            elapsed=(begin-phase_start)*1e-9 if phase_start else 0.
            if phase=='initial_hold' and elapsed>=1.:
                stages.append(dict(phase=phase,elapsed_s=elapsed));phase='stabilize';phase_start=begin;elapsed=0.
            elif phase=='stabilize' and elapsed>=duration:
                stages.append(dict(phase=phase,elapsed_s=elapsed));phase='final_hold';phase_start=begin;elapsed=0.
            elif phase=='final_hold':
                fk=status.get('fk') or {}
                if fk.get('phase')=='final_hold' and fk.get('hold_complete'):
                    stages.append(dict(phase=phase,elapsed_s=elapsed,settled=True))
                    outcome='motion_complete';client.stop('normal_exit');break
                if elapsed>=settings.settle_timeout_s:
                    raise RuntimeError('final_hold did not settle/hold within timeout')
            target_phase=phase or 'initial_hold'
            if target_phase!=last_phase:print('PHASE:',target_phase,flush=True);last_phase=target_phase
            pose,q=plan.target(target_phase,elapsed)
            command=dict(publish_ns=time.monotonic_ns(),target_m=q.tolist(),
                target_steps=slider_to_steps(q,plan.geometry).tolist(),target_pose_m_rad=pose.tolist(),
                phase=target_phase,phase_elapsed_s=elapsed,attitude=plan.control_diagnostics,
                target_request=plan.target_request,pid_config=asdict(pid_config(plan)),
                observed_uart_cycle_sequence=status.get('uart_cycle_sequence') if status else None)
            client.update_joint_reference(command);client.send_state()
            if status and status.get('target_pose_m_rad') and begin-last_console>=5_000_000_000:
                last_console=begin;fk=status.get('fk') or {};actual=fk.get('actual_pose_m_rad')
                print(f"[{target_phase} {elapsed:.1f}s] targetRPYdeg={np.rad2deg(status['target_pose_m_rad'][3:]).round(3).tolist()}"
                    f" FKdeg={np.rad2deg(actual[3:]).round(3).tolist() if actual else None}"
                    f" max|e|={status['max_error_steps']:.2f}steps Imax={status['max_current_ma']}mA",flush=True)
            time.sleep(max(0.,.02-(time.monotonic_ns()-begin)*1e-9))
    except Exception as exc:
        parent_error=f'{type(exc).__name__}: {exc}';outcome='fault'
        if client:client.stop(parent_error)
        (out/'parent_fault.json').write_text(json.dumps(dict(error=parent_error,ns=time.monotonic_ns(),
            target_request=getattr(plan,'target_request',None),last_status=status),indent=2)+'\n')
    finally:
        if client:client.close()
        for sig,handler in handlers.items():signal.signal(sig,handler)
    child_path=out/'motion_child_report.json'
    report=json.loads(child_path.read_text()) if child_path.exists() else dict(
        outcome='exit_unconfirmed',all_mode0_exit_confirmed=False,serial_closed=False,
        source='REAL_HARDWARE' if config['hardware_mode']=='active' else 'MOCK',
        hardware_write_count=None if config['hardware_mode']=='active' else 0,
        error='UART Child exited without a terminal report; physical stopping unconfirmed')
    child_outcome=report['outcome']
    if (out/'motion_bridge_report.json').exists():
        report['bridge_recording']=json.loads((out/'motion_bridge_report.json').read_text())
    report['recording_complete']=all(report.get(name,{}).get('closed') is True and
        report.get(name,{}).get('dropped')==0 and report.get(name,{}).get('error') is None
        for name in ('recording','bridge_recording'))
    if (out/'fault_snapshot.json').exists():
        report['fault_timestamp_ns']=json.loads((out/'fault_snapshot.json').read_text())['fault_timestamp_ns']
    report.update(architecture='j14_three_process_joint_pid',stages=stages,
        plan=plan.summary() if plan else None,workspace_result=plan.workspace_result(outcome) if plan else None,
        parent_error=parent_error,child_returncode=client.proc.returncode if client else None,
        bridge_returncode=client.bridge_proc.returncode if client else None,
        force_policy=dict(normal_max_rate_g_s=None,exceptional_recovery_rate_g_s=[5000.]*6,pid_slew_delay_compensation=False))
    report['output_policy']=policy_summary()
    if parent_error and child_outcome!='fault':report.update(outcome='fault',error=parent_error)
    if not report['all_mode0_exit_confirmed']:report['outcome']='exit_unconfirmed'
    (out/'dual_imu_report.json').write_text(json.dumps(report,indent=2)+'\n')
    return report
