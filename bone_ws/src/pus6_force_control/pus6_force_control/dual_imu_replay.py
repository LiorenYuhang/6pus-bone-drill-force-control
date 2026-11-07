"""Offline replay against recorded measurements; never opens UART or ROS.

Outputs are counterfactual commands on the old measured trajectory, not a
prediction of the newly controlled robot's motion, current or stability.
"""
from dataclasses import asdict
import json
from pathlib import Path

import numpy as np

from .force_pipeline import raw_steps_to_slider_m
from .joint_position_force_pid import JointPositionForcePID, JointForcePIDConfig
from .mode3_dual_imu import DualImuPlan, DualImuMotionConfig
from .six_axis_force_drive import SixAxisForceDrive, ForceLimitsConfig
from .z_position_motion import MotionConfig, pid_config, slider_to_steps
from pus6_motor_driver.force_units import g_to_n, n_to_g, quantize_force_g_with_n_limits


def replay_history(history_dir, output_dir, *, runtime_schedule=False):
    history = Path(history_dir).resolve()
    original = json.loads((history/'dual_imu_report.json').read_text())
    rows = [json.loads(line) for line in (history/'tracking.jsonl').open()]
    completed_input_frames = len(rows)
    snapshot_path = history/'fault_snapshot.json'
    if snapshot_path.exists():
        snapshot = json.loads(snapshot_path.read_text())
        pending = snapshot.get('pending_cycle') or {}
        if pending.get('stage') == 'pid_update' and snapshot.get('attitude',{}).get('imu'):
            latest = snapshot['latest_feedback']
            rows.append(dict(phase=pending['phase'], phase_elapsed_s=pending['phase_elapsed_s'],
                target_timestamp_ns=pending['ns'], target_pose_m_rad=pending['target_pose_m_rad'],
                target_positions_steps=pending['target_positions_steps'],
                attitude=snapshot['attitude'], pid_input_rx_ns=latest['feedback_rx_ns'],
                pid=dict(joint_measured_m=None), recorded_actual_steps=latest['actual_positions_steps'],
                recorded_fault_frame=True))
    if not rows:
        raise ValueError('recording has no completed control frames')
    out = Path(output_dir).resolve()
    out.mkdir(parents=True, exist_ok=False)
    recorded_plan = original['plan']
    class RecordedImu:
        kind = 'RECORDED_IMU_OFFLINE'
        current = recorded_plan['imu_reference']['sample']
        def snapshot(self, now_ns): return self.current
    source = RecordedImu()
    now = source.current['now_ns']
    plan = DualImuPlan(recorded_plan['initial_steps'], DualImuMotionConfig(**recorded_plan['settings']),
        source=source, parameters=recorded_plan['attitude_parameters'], axes=tuple(recorded_plan['axes']),
        clock=lambda: now)
    for row in rows:
        if row.get('recorded_fault_frame'):
            row['pid']['joint_measured_m']=raw_steps_to_slider_m(row['recorded_actual_steps'],plan.geometry).tolist()
    limits = plan.force_limits()
    pid = JointPositionForcePID(pid_config(plan), clock=lambda: now, force_slew_compensation=limits.max_rate_g_s is not None)
    baseline = JointPositionForcePID(JointForcePIDConfig(**original['pid']), clock=lambda: now,
        force_slew_compensation=original.get('force_policy',{}).get('pid_slew_delay_compensation', True))
    first = rows[0]
    first_stamps = first['pid_input_rx_ns']
    measured = np.array(first['pid']['joint_measured_m'])
    now = first['target_timestamp_ns']-20_000_000
    reset_q=measured;reset_stamps=[s-20_000_000 for s in first_stamps]
    reset_source='estimated_one_period_before_first_frame'
    # Recover the actual pre-loop reset sample when it was recorded, so early
    # filter timing/encoder jitter is not mistaken for an algorithm difference.
    feedback_path=history/'feedback.jsonl'
    if feedback_path.exists() and 'joint_dt_s' in first['pid']:
        reset_sample_ns=max(first_stamps)-round(first['pid']['joint_dt_s']*1e9)
        for line in feedback_path.open():
            feedback=json.loads(line)
            if max(feedback['rx_ns'])==reset_sample_ns:
                reset_q=raw_steps_to_slider_m(feedback['positions_steps'],plan.geometry)
                reset_stamps=feedback['rx_ns'];reset_source='recorded_pre_loop_feedback'
                break
            if max(feedback['rx_ns'])>reset_sample_ns:break
    for instance in (pid,baseline):
        instance.reset(reset_q,sample_ns=max(reset_stamps),now_ns=now,position_stamps_ns=reset_stamps)
    drive = SixAxisForceDrive(None, limits, clock=lambda: now)
    hold = np.asarray(original['hold_forces_g'])
    drive.anchor_hold_forces_g(hold, now_ns=now)
    count = saturated = clipped = 0
    reference_difference = raw_pose_difference = 0.
    commands = []; previous_reference = previous_ns = None
    peak_reference_speed = np.zeros(6)
    fault = None
    shaping_error = 0.
    peak_normal_force_slew_g_s = np.zeros(6)
    previous_force = None
    baseline_records = []; optimized_records = []
    phases = {}; handover = None; previous_phase = None; baseline_record_error = 0.
    runtime_start_ns=rows[0]['target_timestamp_ns']-int(rows[0]['phase_elapsed_s']*1e9)
    with (out/'replay.jsonl').open('x') as stream:
        for index, row in enumerate(rows):
            source.current = row['attitude']['imu']
            now = row['target_timestamp_ns']
            measured = np.asarray(row['pid']['joint_measured_m'])
            stamps = row['pid_input_rx_ns']
            phase, elapsed = row['phase'], row['phase_elapsed_s']
            if runtime_schedule:
                runtime_elapsed=(now-runtime_start_ns)*1e-9
                if runtime_elapsed < 1.:
                    phase, elapsed='initial_hold', runtime_elapsed
                elif runtime_elapsed < 1.+plan.phase_schedule[1][1]:
                    phase, elapsed='stabilize', runtime_elapsed-1.
                else:
                    phase, elapsed='final_hold', runtime_elapsed-1.-plan.phase_schedule[1][1]
            if phase=='stabilize' and previous_phase=='initial_hold':
                handover=dict(frame_index=index, ns=now, runtime_elapsed_s=(now-runtime_start_ns)*1e-9,
                    integral_before_n=pid.integral.tolist(), velocity_before_m_s=pid.velocity.tolist(),
                    state_action='carry_without_reset')
            previous_phase=phase
            try:
                pose, target = plan.target(phase, elapsed)
                plan.check_positions(slider_to_steps(measured, plan.geometry))
                delta, diag = pid.update(target, measured, sample_ns=max(stamps), now_ns=now, position_stamps_ns=stamps)
            except (ValueError,RuntimeError) as exc:
                fault = dict(frame_index=index, ns=now, phase=phase, phase_elapsed_s=elapsed, error=str(exc),
                    recorded_fault_frame=row.get('recorded_fault_frame',False),
                    actual_m=measured.tolist(), target_request=plan.target_request)
                break
            frames = drive.command_forces_n(np.asarray(g_to_n(hold))+delta, dry_run=True, now_ns=now, braking_axes=pid.braking_axes)
            forces = [f['target_force_g'] for f in frames['frames']]
            expected = quantize_force_g_with_n_limits(n_to_g(frames['limited_forces_n']),
                limits.min_force_n, limits.max_force_n)
            shaping_error = max(shaping_error, float(np.abs(np.asarray(forces)-expected).max()))
            pid.track_applied(np.asarray(g_to_n(forces))-np.asarray(g_to_n(hold)))
            old_target=raw_steps_to_slider_m(row['target_positions_steps'],plan.geometry)
            old_delta, old_diag = baseline.update(old_target, measured, sample_ns=max(stamps), now_ns=now, position_stamps_ns=stamps)
            old_force = quantize_force_g_with_n_limits(n_to_g(np.asarray(g_to_n(hold))+old_delta),limits.min_force_n,limits.max_force_n)
            baseline.track_applied(np.asarray(g_to_n(old_force))-np.asarray(g_to_n(hold)))
            if row.get('target_force_g') is not None:
                baseline_record_error=max(baseline_record_error,float(np.max(abs(np.asarray(old_force)-row['target_force_g']))))
            baseline_records.append(dict(pid=old_diag,force=list(old_force),ns=now))
            optimized_records.append(dict(pid=diag,force=forces,ns=now))
            phases[phase]=phases.get(phase,0)+1
            if handover is not None and handover['frame_index']==index:
                handover.update(integral_after_n=diag['joint_i_n'], integral_error_m=diag['joint_integral_error_m'],
                    pid_dt_s=diag['joint_dt_s'], observer_updated=plan.control_diagnostics['observer_updated'])
            reference = np.asarray(diag['joint_reference_m'])
            reference_difference = max(reference_difference, float(np.abs(reference-target).max()))
            raw_pose_difference = max(raw_pose_difference, float(np.abs(np.asarray(plan.target_request['requested_pose_m_rad'])-row['target_pose_m_rad']).max()))
            if previous_reference is not None:
                peak_reference_speed = np.maximum(peak_reference_speed, np.abs(reference-previous_reference)/((now-previous_ns)*1e-9))
                peak_normal_force_slew_g_s = np.maximum(peak_normal_force_slew_g_s,
                    np.abs(np.asarray(forces)-previous_force)/((now-previous_ns)*1e-9))
            previous_reference, previous_ns = reference, now
            previous_force = np.asarray(forces)
            saturated += int(plan.control_diagnostics['target_constraint']['saturated'])
            clipped += int(diag['joint_output_limited'])
            commands.append(forces); count += 1
            stream.write(json.dumps(dict(source='OFFLINE_REPLAY', hardware_write_count=0, frame_index=index,
                ns=now, phase=phase, phase_elapsed_s=elapsed, recorded_phase=row['phase'], target_pose_m_rad=pose.tolist(),
                target_positions_steps=slider_to_steps(target, plan.geometry).tolist(),
                recorded_actual_positions_steps=slider_to_steps(measured, plan.geometry).tolist(),
                attitude=plan.control_diagnostics, pid=diag, pid_output_n=delta.tolist(), target_force_g=forces,
                baseline_pid=old_diag, baseline_target_force_g=list(old_force)))+'\n')
    forces = np.asarray(commands)
    def metrics(records):
        if not records: return None
        v=np.array([x['pid']['joint_velocity_m_s'] for x in records])*1000
        d=np.array([x['pid']['joint_d_n'] for x in records]);f=np.array([x['force'] for x in records])
        requested=np.array([x['pid']['joint_requested_delta_force_n'] for x in records])
        e=np.array([x['pid']['joint_error_m'] for x in records]);delta=np.array([x['pid']['joint_delta_force_n'] for x in records])
        ie=np.array([x['pid']['joint_integral_error_m'] for x in records])
        early=[x for x in records if x['ns']-runtime_start_ns<=10_000_000_000]
        return dict(integral_input_active_inside_measured_deadband_axes=np.sum(
                (abs(e)<=pid.config.joint_position_deadband_m+1e-12)&(abs(ie)>1e-12),0).tolist(),
            integral_n_at_last_frame_within_10s=early[-1]['pid']['joint_i_n'] if early else None,
            final_integral_n=records[-1]['pid']['joint_i_n'],
            peak_d_n=np.max(abs(d),0).tolist(), max_velocity_jump_mm_s=np.max(abs(np.diff(v,axis=0)),0).tolist(),
            max_d_jump_n=np.max(abs(np.diff(d,axis=0)),0).tolist(),
            target_force_total_variation_g=np.sum(abs(np.diff(f,axis=0)),0).tolist(),
            force_jumps_at_least_800g=np.sum(abs(np.diff(f,axis=0))>=800,0).tolist(),
            output_clipped_axes=np.sum(abs(requested)>np.array(pid.config.joint_increment_limit_n)+1e-10,0).tolist(),
            output_clipped_frames=int(np.any(abs(requested)>np.array(pid.config.joint_increment_limit_n)+1e-10,axis=1).sum()),
            force_opposing_non_deadband_error_axes=np.sum((delta*e<0)&(abs(e)>pid.config.joint_position_deadband_m+1e-12),0).tolist())
    report = dict(source='OFFLINE_REPLAY', hardware_write_count=0, input_directory=str(history),
        input_frames=len(rows), completed_input_frames=completed_input_frames, replayed_frames=count, fault=fault,
        comparison=dict(baseline=metrics(baseline_records),optimized=metrics(optimized_records)),
        replay_phase_policy='current_runtime_schedule' if runtime_schedule else 'recorded_phases_for_pid_comparison',
        pid_reset_source=reset_source,
        phase_counts=phases, initial_hold_to_stabilize=handover,
        baseline_record_force_max_error_g=baseline_record_error,
        max_p_difference_n=float(np.max(abs(np.array([x['pid']['joint_p_n'] for x in optimized_records])-
            np.array([x['pid']['joint_p_n'] for x in baseline_records])))) if count else None,
        max_d_difference_n=float(np.max(abs(np.array([x['pid']['joint_d_n'] for x in optimized_records])-
            np.array([x['pid']['joint_d_n'] for x in baseline_records])))) if count else None,
        baseline_pid=asdict(baseline.config),
        outcome='completed_recorded_frames' if fault is None else 'stopped_on_recorded_measurements',
        pid=asdict(pid.config), target_saturated_frames=saturated, pid_clipped_frames=clipped,
        force_policy=dict(normal_max_rate_g_s=limits.max_rate_g_s, exceptional_recovery_rate_g_s=list(limits.brake_rate_g_s),
            pid_slew_delay_compensation=pid.force_slew_compensation),
        max_normal_force_shaping_error_g=shaping_error,
        per_axis_peak_normal_force_slew_g_s=peak_normal_force_slew_g_s.tolist(),
        max_reference_minus_bounded_ik_m=reference_difference,
        max_recomputed_raw_pose_minus_recorded_pose_si=raw_pose_difference,
        per_axis_peak_reference_speed_mm_s=(peak_reference_speed*1000).tolist(),
        target_force_min_g=forces.min(0).tolist() if count else None,
        target_force_max_g=forces.max(0).tolist() if count else None,
        original_outcome=original['outcome'], original_error=original.get('error'),
        limitations=['Uses recorded IMU and pre-write motor measurements, not a simulated physical plant.',
            'Changed commands do not predict actual motion, current or stability.',
            'Runtime-schedule mode changes targets after 1s but uses the old experiment motor positions; it cannot certify physical tracking.',
            'A recorded pid_update fault frame is checked explicitly when available; its unchanged measured tracking error may still trip.',
            'Clipping counts alone do not establish quality; P/D cancellation and braking direction must be read with the trajectory.'])
    (out/'replay_report.json').write_text(json.dumps(report, indent=2)+'\n')
    return report
