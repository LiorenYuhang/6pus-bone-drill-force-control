"""Post-close analysis of measured three-axis stabilization. Never opens UART."""
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


def analyze_run(run_dir, output_dir):
    run = Path(run_dir).resolve()
    report = json.loads((run/'dual_imu_report.json').read_text())
    tracking_path = run/'tracking.jsonl'
    rows = [json.loads(s) for s in tracking_path.read_text().splitlines() if s.strip()] if tracking_path.exists() else []
    feedback_path = run/'feedback.jsonl'
    feedback_rows = [json.loads(s) for s in feedback_path.read_text().splitlines() if s.strip()] if feedback_path.exists() else []
    active = [r for r in rows if r['phase'] == 'stabilize' and 'base_rotation_mechanical' in r.get('attitude', {})]
    out = Path(output_dir).resolve(); out.mkdir(parents=True, exist_ok=False)
    summary = dict(source=report['source'], run_dir=str(run), outcome=report['outcome'],
        error=report.get('error'), fault_timestamp_ns=report.get('fault_timestamp_ns'),
        hardware_write_count=report['hardware_write_count'], analysis_hardware_write_count=0,
        all_mode0_exit_confirmed=report.get('all_mode0_exit_confirmed'), serial_closed=report.get('serial_closed'),
        completed_active_samples=len(active), workspace_result=report.get('workspace_result'),
        tracking_available=tracking_path.exists(), completed_control_samples=len(rows),
        analysis_status='ACTIVE_SAMPLES_AVAILABLE' if active else 'NO_ACTIVE_SAMPLES',
        full_unscaled_target_completed=False,
        stability_verdict='NOT_ESTABLISHED',
        overshoot_definition='signed residual peaks about the fixed world attitude reference; not step-response percent overshoot')
    fault_path = run/'fault_snapshot.json'
    if fault_path.exists():
        fault = json.loads(fault_path.read_text())
        summary.update(fault_stage=(fault.get('pending_cycle') or {}).get('stage'),
            rejected_target_request=fault.get('target_request'), fault_latest_feedback=fault.get('latest_feedback'))
    if active:
        t = np.array([r['ns'] for r in active]); t = (t-t[0])*1e-9
        base_rotation = Rotation.from_matrix(np.array([r['attitude']['base_rotation_mechanical'] for r in active]))
        base = base_rotation.as_euler('xyz', degrees=True)
        platform = Rotation.from_matrix(np.array([r['attitude']['platform_relative_rotation_mechanical'] for r in active]))
        platform_euler = platform.as_euler('xyz', degrees=True)
        r0 = Rotation.from_euler('xyz', report['plan']['initial_pose_m_rad'][3:])
        # World error Log(R0 (R0 Rrelative)^T), including unselected channels.
        error = (r0*(r0*platform).inv()).as_rotvec()*180/np.pi
        fk_error = []; fk_indices = []
        for i, r in enumerate(active):
            if r['actual_pose_m_rad'] is not None:
                target = Rotation.from_euler('xyz', r['target_pose_m_rad'][3:])
                actual = Rotation.from_euler('xyz', r['actual_pose_m_rad'][3:])
                fk_error.append((target*actual.inv()).as_rotvec()*180/np.pi); fk_indices.append(i)
        actual = np.array([r['positions_steps'] for r in feedback_rows] if feedback_rows else [r['actual_positions_steps'] for r in rows], float)
        target_steps = np.array([r['target_positions_steps'] for r in rows], float)
        snapshot_path = run/'fault_snapshot.json'
        snapshot = json.loads(snapshot_path.read_text()) if snapshot_path.exists() else None
        if snapshot:
            actual = np.vstack([actual, snapshot['latest_feedback']['actual_positions_steps']])
        lower = np.array(report['plan']['mechanical_min_steps']); upper = np.array(report['plan']['mechanical_max_steps'])
        constraints = [r['attitude'].get('target_constraint', {}) for r in rows]
        terminal = report.get('terminal_recovery', {}).get('axes', [])
        terminal_positions = [a['anchor_steps'] for a in sorted(terminal,key=lambda a:a['motor_id'])] if len(terminal)==6 else None
        error_message = report.get('error') or ''
        summary['protection_trigger'] = error_message.split(':',2)[1].strip() if ':' in error_message else error_message or 'NONE'
        summary['fault_stage'] = (snapshot.get('pending_cycle') or {}).get('stage') if snapshot else None
        summary['terminal_position_steps'] = terminal_positions
        if terminal_positions is not None:
            summary['terminal_remaining_lower_steps'] = (np.array(terminal_positions)-lower).tolist()
            summary['terminal_remaining_upper_steps'] = (upper-np.array(terminal_positions)).tolist()
            summary['terminal_minus_captured_steps'] = (np.array(terminal_positions)-report['plan']['initial_steps']).tolist()
        if snapshot:
            summary['rejected_target_request'] = snapshot.get('target_request')
            summary['fault_attempted_target_force_g'] = (snapshot.get('pending_cycle') or {}).get('target_force_g')
            summary['fault_latest_feedback'] = snapshot.get('latest_feedback')
        summary['protection_thresholds'] = dict(current_ma=report['plan']['settings']['max_current_ma'],
            tracking_error_m=(report.get('pid') or {}).get('joint_max_tracking_error_m',report['plan']['settings']['tracking_error_m']),
            mechanical_min_steps=lower.tolist(),mechanical_max_steps=upper.tolist(),
            actual_fault_min_steps=report['plan']['envelope_min_steps'],actual_fault_max_steps=report['plan']['envelope_max_steps'],
            target_min_steps=report['plan'].get('target_min_steps'),target_max_steps=report['plan'].get('target_max_steps'))
        rejected = bool(snapshot and snapshot.get('target_request', {}).get('target_reachable') is False)
        max_component = np.max(abs(error), axis=0)
        summary.update(active_elapsed_s=float(t[-1]), base_min_deg=base.min(0).tolist(), base_max_deg=base.max(0).tolist(),
            base_peak_abs_deg=np.max(abs(base),0).tolist(), base_max_so3_deg=float(np.rad2deg(base_rotation.magnitude()).max()),
            platform_relative_peak_abs_deg=np.max(abs(platform_euler),0).tolist(),
            world_error_max_abs_deg=max_component.tolist(), world_error_rms_deg=np.sqrt(np.mean(error**2,0)).tolist(),
            world_error_so3_max_deg=float(np.linalg.norm(error,axis=1).max()),
            world_error_so3_rms_deg=float(np.sqrt(np.mean(np.sum(error**2,axis=1)))),
            transient_positive_peak_deg=error.max(0).tolist(), transient_negative_peak_deg=error.min(0).tolist(),
            transient_peak_time_s=[float(t[np.argmax(abs(error[:,i]))]) for i in range(3)],
            actual_min_steps=actual.min(0).tolist(), actual_max_steps=actual.max(0).tolist(),
            target_min_steps=target_steps.min(0).tolist(), target_max_steps=target_steps.max(0).tolist(),
            actual_remaining_lower_steps=(actual.min(0)-lower).tolist(), actual_remaining_upper_steps=(upper-actual.max(0)).tolist(),
            target_remaining_lower_steps=(target_steps.min(0)-lower).tolist(), target_remaining_upper_steps=(upper-target_steps.max(0)).tolist(),
            force_readback_mismatch_frames=sum(r['target_force_27_g_readback'] != r['target_force_g'] for r in rows if r.get('target_force_27_g_readback') is not None),
            cycle_work_max_ms=max(r['cycle_ms'] for r in rows),
            ik_saturated_recorded_frames=sum(bool(c.get('saturated')) for c in constraints), rejected_fault_target=rejected,
            pid_clipped_frames=sum(r['pid']['joint_output_limited'] for r in active),
            absolute_force_limited_frames=sum(any(r.get('absolute_force_limited_axes',[])) for r in rows),
            max_current_ma=np.max([r['currents_ma'] for r in (feedback_rows or rows)],0).tolist(),
            fault_codes_seen=sorted(set(v for r in (feedback_rows or rows) for v in r['fault_codes'])),
            modes_seen=sorted(set(v for r in rows for v in r['modes_last_readback'])),
            target_force_min_g=np.min([r['target_force_g'] for r in rows],0).tolist(),
            target_force_max_g=np.max([r['target_force_g'] for r in rows],0).tolist(),
            feedback_force_min_g=np.min([r['force_2c_g'] for r in (feedback_rows or rows)],0).tolist(),
            feedback_force_max_g=np.max([r['force_2c_g'] for r in (feedback_rows or rows)],0).tolist(),
            tail_3s_world_error_rms_deg=np.sqrt(np.mean(error[t>=max(0,t[-1]-3)]**2,0)).tolist(),
            last_fk_pose_m_rad=active[-1]['actual_pose_m_rad'],
            captured_pose_m_rad=report['plan']['initial_pose_m_rad'])
        if fk_error:
            values = np.array(fk_error)
            summary['fk_target_error_max_abs_deg'] = np.max(abs(values),0).tolist()
            summary['fk_target_error_rms_deg'] = np.sqrt(np.mean(values**2,0)).tolist()
        # No fabricated pass threshold. Completion and saturation are explicit;
        # measured peaks/RMS remain the evidence for assessing stabilization.
        summary['full_unscaled_target_completed'] = bool(report['outcome']=='motion_complete' and
            not rejected and not summary['ik_saturated_recorded_frames'])
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(4,1,figsize=(12,12),sharex=True)
        for i, name in enumerate(('Roll','Pitch','Yaw')):
            axes[0].plot(t,base[:,i],label=name); axes[1].plot(t,error[:,i],label=name)
            if fk_error: axes[2].plot(t[fk_indices],np.array(fk_error)[:,i],label=name)
        axes[0].set_ylabel('Base relative (deg)'); axes[1].set_ylabel('World SO(3) error (deg)')
        axes[2].set_ylabel('Target - FK SO(3) (deg)')
        for i in range(6): axes[3].plot(t,[r['currents_ma'][i] for r in active],label=str(i+1))
        axes[3].set_ylabel('Current (mA)'); axes[3].set_xlabel('Active time (s)')
        for ax in axes: ax.grid(True); ax.legend(ncol=6)
        fig.tight_layout();fig.savefig(out/'attitude_and_current.png',dpi=140);plt.close(fig)
        fig, axes = plt.subplots(3,2,figsize=(13,10),sharex=True)
        for i, ax in enumerate(axes.flat):
            ax.plot(t,[r['target_positions_steps'][i] for r in active],label='target')
            ax.plot(t,[r['actual_positions_steps'][i] for r in active],label='feedback')
            ax.axhline(lower[i],color='r',ls=':');ax.axhline(upper[i],color='r',ls=':')
            ax.set_ylabel(f'M{i+1} steps');ax.grid(True);ax.legend()
        fig.tight_layout();fig.savefig(out/'joint_workspace.png',dpi=140);plt.close(fig)
        fig, axes = plt.subplots(3,2,figsize=(13,10),sharex=True)
        for i, ax in enumerate(axes.flat):
            ax.plot(t,[r['target_force_g'][i] for r in active],label='0x27 target')
            ax.plot(t,[r['force_2c_g'][i] for r in active],label='0x2C measured')
            ax.set_ylabel(f'M{i+1} g');ax.grid(True);ax.legend()
        fig.tight_layout();fig.savefig(out/'force.png',dpi=140);plt.close(fig)
    (out/'analysis.json').write_text(json.dumps(summary,indent=2)+'\n')
    return summary
