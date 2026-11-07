"""Replay recorded IK/encoder inputs through both PI-D output policies. No UART.

Feedback is fixed historical data: changed commands do not predict new motion.
The pre-first-cycle velocity history was not recorded; its reset is estimated.
An additional isolated comparison uses the exact recorded last force request.
"""
import argparse
from dataclasses import asdict, replace
from pathlib import Path
import json
import numpy as np

from pus6_motor_driver.force_units import g_to_n
from .joint_position_force_pid import JointForcePIDConfig, JointPositionForcePID
from .six_axis_force_drive import SixAxisForceDrive
from .dual_imu_force_policy import dual_imu_force_limits
from .mode3_process_policy import PID_OVERRIDES, force_limits, policy_summary
from .force_pipeline import raw_steps_to_slider_m
from pus6_kinematics.geometry_config import load_geometry_config


def replay(history, output):
    history=Path(history).resolve();out=Path(output).resolve()
    report=json.loads((history/'dual_imu_report.json').read_text())
    rows=[json.loads(s) for s in (history/'tracking.jsonl').read_text().splitlines() if s.strip()]
    if not rows:raise ValueError('no completed input frames')
    snapshot=json.loads((history/'fault_snapshot.json').read_text())
    completed=len(rows);last=rows[-1];pending=snapshot.get('pending_cycle') or {}
    geometry=load_geometry_config()
    if pending.get('stage')=='pid_update':
        feedback=snapshot['latest_feedback']
        rows.append(dict(target_timestamp_ns=pending['ns'],phase=pending['phase'],recorded_fault_frame=True,
            target_positions_steps=pending['target_positions_steps'],pid_input_rx_ns=feedback['feedback_rx_ns'],
            pid=dict(joint_target_m=raw_steps_to_slider_m(pending['target_positions_steps'],geometry).tolist(),
                joint_measured_m=raw_steps_to_slider_m(feedback['actual_positions_steps'],geometry).tolist())))
    old_config=JointForcePIDConfig(**report['pid']);new_config=replace(old_config,**PID_OVERRIDES)
    old=JointPositionForcePID(old_config,clock=lambda:0,force_slew_compensation=False)
    new=JointPositionForcePID(new_config,clock=lambda:0,force_slew_compensation=False)
    first=rows[0];dt_ns=round(first['pid']['joint_dt_s']*1e9)
    reset_stamps=np.array(first['pid_input_rx_ns'])-dt_ns
    for pid in (old,new):
        pid.reset(first['pid']['joint_measured_m'],sample_ns=int(max(reset_stamps)),
            now_ns=first['target_timestamp_ns']-dt_ns,position_stamps_ns=reset_stamps.tolist())
    hold=np.array(g_to_n(report['hold_forces_g']))
    old_drive=SixAxisForceDrive(None,dual_imu_force_limits(),clock=lambda:0)
    new_drive=SixAxisForceDrive(None,force_limits(),clock=lambda:0)
    out.mkdir(parents=True,exist_ok=False)
    new_commands=[];integrals=[];new_diagnostics=[];new_errors=[];old_errors=[]
    old_record_difference=[];final_caps=0;reference_difference=0.;baseline_fault=None
    with (out/'replay.jsonl').open('w') as stream:
        for index,row in enumerate(rows):
            now=row['target_timestamp_ns'];stamps=row['pid_input_rx_ns'];d=row['pid']
            args=dict(sample_ns=max(stamps),now_ns=now,position_stamps_ns=stamps)
            target=d['joint_target_m'];measured=d['joint_measured_m']
            baseline_force=None;baseline_diag=None
            if baseline_fault is None:
                try:
                    delta,baseline_diag=old.update(target,measured,**args)
                    command=old_drive.command_forces_n(hold+delta,dry_run=True,now_ns=now)
                    baseline_force=[r['target_force_g'] for r in command['frames']]
                    old.track_applied(np.array(g_to_n(baseline_force))-hold)
                    if 'target_force_g' in row:
                        old_record_difference.append(np.array(baseline_force)-row['target_force_g'])
                except ValueError as exc:baseline_fault=dict(frame_index=index,error=str(exc))
            delta,diag=new.update(target,measured,**args)
            command=new_drive.command_forces_n(hold+delta,dry_run=True,now_ns=now)
            forces=[r['target_force_g'] for r in command['frames']]
            blocked=new.track_applied(np.array(g_to_n(forces))-hold)
            limited=(abs(np.array(command['requested_forces_n'])-command['limited_forces_n'])>1e-10).tolist()
            final_caps+=int(any(limited));new_commands.append(forces);integrals.append(diag['joint_i_n'])
            reference_difference=max(reference_difference,float(np.max(abs(np.array(diag['joint_reference_m'])-target))))
            if not row.get('recorded_fault_frame'):
                new_errors.append(np.array(diag['joint_d_n'])-d['joint_d_n'])
                if baseline_diag is not None:old_errors.append(np.array(baseline_diag['joint_d_n'])-d['joint_d_n'])
            result=dict(frame_index=index,ns=now,phase=row['phase'],recorded_fault_frame=row.get('recorded_fault_frame',False),
                source='OFFLINE_FIXED_FEEDBACK_REPLAY',hardware_write_count=0,pid=diag,
                recorded_target_force_g=row.get('target_force_g'),baseline_recomputed_target_force_g=baseline_force,
                target_force_g=forces,absolute_force_limited_axes=limited,final_integral_blocked_axes=blocked)
            new_diagnostics.append(result);stream.write(json.dumps(result,allow_nan=False)+'\n')
    # Isolate removal of ±4 N using the EXACT -12.4287445 N recorded request,
    # separate from the full replay's changed accumulated integral history.
    requested=np.array(last['pid']['joint_requested_delta_force_n'])
    isolated=SixAxisForceDrive(None,force_limits(),clock=lambda:0).command_forces_n(hold+requested,dry_run=True,now_ns=1)
    forces=np.array(new_commands);physical=g_to_n(forces.min(0).tolist())+g_to_n(forces.max(0).tolist())
    summary=dict(source='OFFLINE_FIXED_FEEDBACK_REPLAY',hardware_write_count=0,input_directory=str(history),
        completed_recorded_frames=completed,replayed_frames=len(rows),baseline_fault=baseline_fault,
        new_pid=asdict(new_config),output_policy=policy_summary(),absolute_force_limited_frames=final_caps,
        target_force_min_g=forces.min(0).tolist(),target_force_max_g=forces.max(0).tolist(),
        all_encoded_forces_within_catalog_50n=bool(max(abs(v) for v in physical)<=50),
        peak_integral_n=np.max(abs(np.array(integrals)),axis=0).tolist(),max_reference_shaping_error_m=reference_difference,
        exact_recorded_request_comparison=dict(recorded_request_n=requested.tolist(),
            recorded_target_force_g=last['target_force_g'],
            without_increment_cap_target_force_g=[f['target_force_g'] for f in isolated['frames']]),
        new_last_completed_input=new_diagnostics[completed-1],new_recorded_fault_input=new_diagnostics[-1],
        estimator_reset='estimated pre-first-cycle positions/timestamps; original reset history unavailable',
        baseline_recomputed_vs_recorded_force_max_error_g=np.max(abs(np.array(old_record_difference)),axis=0).tolist(),
        last_recomputed_vs_recorded_d_error_n=new_errors[-1].tolist(),
        max_recomputed_vs_recorded_d_error_n=np.max(abs(np.array(new_errors)),axis=0).tolist(),
        firmware_50n_clamp_verified=False,
        conclusion='Software output comparison only; cannot infer changed motion, force response or stability success')
    (out/'replay_report.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
    return summary


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--history',required=True);p.add_argument('--output-dir',required=True)
    args=p.parse_args();r=replay(args.history,args.output_dir)
    print(json.dumps(dict(output_dir=str(Path(args.output_dir).resolve()),replayed_frames=r['replayed_frames'],
        hardware_write_count=0,exact_recorded_request_comparison=r['exact_recorded_request_comparison']),indent=2))


if __name__=='__main__':main()
