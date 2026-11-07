"""Offline summaries/plots of combined-pose recordings after serial cleanup."""
import json
from pathlib import Path

import numpy as np

from pus6_kinematics.analytical_6pus import Analytical6PusKinematics
from .force_pipeline import raw_steps_to_slider_m
from .single_axis_motion import pose_error

NAMES = ('X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw')
SCALE = np.array([1000.]*3+[180./np.pi]*3)


def stats(values):
    a = np.asarray(values, dtype=float)
    if not a.size:
        return None
    return dict(mean=a.mean(0).tolist(), rmse=np.sqrt((a*a).mean(0)).tolist(),
                max_abs=np.abs(a).max(0).tolist(), min=a.min(0).tolist(), max=a.max(0).tolist())


def correlation(left, right):
    """Rows=left DOF, columns=right DOF; undefined correlations are null."""
    if len(left) < 3:
        return [[None]*6 for _ in range(6)]
    a = np.asarray(left)-np.mean(left, axis=0)
    b = np.asarray(right)-np.mean(right, axis=0)
    denominator = np.sqrt((a*a).sum(0)[:, None]*(b*b).sum(0)[None, :])
    result = np.divide(a.T@b, denominator, out=np.full((6, 6), np.nan), where=denominator > 1e-18)
    return [[float(v) if np.isfinite(v) else None for v in row] for row in result]


def analyze_combined_run(directory):
    out = Path(directory)
    report = json.loads((out/'combined_report.json').read_text())
    path = out/'tracking.jsonl'
    rows = [json.loads(line) for line in path.open()] if path.exists() else []
    plan = report.get('plan', {})
    initial = np.asarray(plan.get('initial_pose_m_rad', [0.]*6))
    valid = [r for r in rows if r.get('actual_pose_m_rad') is not None]
    motion = [r for r in valid if r['phase'] == 'combined_motion']
    errors = lambda selected: np.array([pose_error(r['actual_pose_m_rad'], r['target_pose_m_rad'])*SCALE for r in selected])
    targets = lambda selected: np.array([pose_error(r['target_pose_m_rad'], initial)*SCALE for r in selected])
    actuals = lambda selected: np.array([pose_error(r['actual_pose_m_rad'], initial)*SCALE for r in selected])
    e = errors(motion)
    target = targets(motion)
    summary = dict(source=report['source'], outcome=report['outcome'], error=report.get('error'),
        hardware_write_count=report.get('hardware_write_count'),
        all_mode0_exit_confirmed=report.get('all_mode0_exit_confirmed', False),
        serial_closed=report.get('serial_closed', False), units=['mm']*3+['deg']*3,
        rows=len(rows), fk_valid_rows=len(valid), fk_unavailable_rows=len(rows)-len(valid),
        motion_fk_valid_rows=len(motion), plan=plan, stages=report.get('stages', []),
        pose_error_actual_minus_target_all_mm_deg=stats(errors(valid)),
        pose_error_motion_mm_deg=stats(e), target_motion_delta_mm_deg=stats(target),
        actual_motion_delta_mm_deg=stats(actuals(motion)),
        last_mode3_return_error_mm_deg=(pose_error(valid[-1]['actual_pose_m_rad'], initial)*SCALE).tolist() if valid else None,
        last_mode3_phase=valid[-1]['phase'] if valid else None,
        mode0_return_error_mm_deg=None,
        motion_error_correlations=correlation(e, e),
        motion_target_error_correlations=correlation(target, e),
        interpretation=['FK derives from sequential actuator encoder feedback, not an external pose sensor.',
            'Target is evaluated before the feedback response; host timestamps are not synchronized device samples.',
            'All six DOFs are commanded. Correlations describe residual associations, not causal coupling or decoupling.',
            'MOCK data checks software only; interrupted/fault runs are not completed trajectories.'])
    axes = (report.get('terminal_recovery') or {}).get('axes', [])
    if len(axes) == 6 and all(a.get('final_mode') == 0 and a.get('observations') for a in axes) and plan:
        kin = Analytical6PusKinematics()
        q = raw_steps_to_slider_m([a['observations'][-1]['position_steps'] for a in axes], kin.geometry_config)
        fk = kin.forward_kinematics_with_metrics(q, initial_guess=initial)
        if fk.converged and np.isfinite(fk.pose).all():
            summary['mode0_return_error_mm_deg'] = (pose_error(fk.pose, initial)*SCALE).tolist()
    if rows:
        vector = lambda field: np.array([r[field] for r in rows])
        pid = lambda field: np.array([r['pid'][field] for r in rows])
        requested = pid('joint_requested_delta_force_n')
        delta = pid('joint_delta_force_n')
        force = vector('target_force_g')
        desired = np.asarray(report['hold_forces_g'])+vector('pid_output_g')
        limits = np.asarray(report['pid']['joint_integral_limit_n'])
        summary.update(joint_position_error_steps=stats(vector('position_error_steps')),
            joint_velocity_mm_s=stats(pid('joint_velocity_m_s')*1000),
            pid_output_g=stats(vector('pid_output_g')), pid_p_n=stats(pid('joint_p_n')),
            pid_i_n=stats(pid('joint_i_n')), pid_d_n=stats(pid('joint_d_n')),
            pid_increment_clipped_rows_per_axis=(np.abs(requested-delta)>1e-9).sum(0).tolist(),
            integral_at_cap_rows_per_axis=(np.abs(pid('joint_i_n'))>=limits-1e-9).sum(0).tolist(),
            force_shaped_rows_per_axis=(np.abs(force-desired)>.51).sum(0).tolist(),
            force_absolute_cap_rows_per_axis=(np.abs(force)>=1500).sum(0).tolist(),
            target_force_g=stats(force), feedback_force_g=stats(vector('force_2c_g')),
            current_ma=stats(vector('currents_ma')), cycle_work_ms=stats(vector('cycle_ms')),
            control_interval_ms=stats(pid('joint_dt_s')*1000),
            mode3_mismatch_rows=sum(any(m != 3 for m in r['modes_last_readback']) for r in rows),
            fault_rows=sum(any(r['fault_codes']) for r in rows),
            target_27_readback_rows=sum(r['target_force_27_g_readback'] is not None for r in rows),
            target_27_mismatch_rows=sum(r['target_force_27_g_readback'] is not None and
                r['target_force_27_g_readback'] != r['target_force_g'] for r in rows))
    summary['yaw'] = dict(motion_error_mm_deg_index=5,
        motion_rmse_deg=summary['pose_error_motion_mm_deg']['rmse'][5] if motion else None,
        motion_max_abs_error_deg=summary['pose_error_motion_mm_deg']['max_abs'][5] if motion else None,
        target_actual_correlation=correlation(target, actuals(motion))[5][5],
        mode0_return_error_deg=summary['mode0_return_error_mm_deg'][5] if summary['mode0_return_error_mm_deg'] else None)
    (out/'summary.json').write_text(json.dumps(summary, indent=2, allow_nan=False)+'\n')
    lines = ['# Six-DOF combined trajectory result', '',
        f"Source: {summary['source']}; outcome: {summary['outcome']}; Mode 0 confirmed: {summary['all_mode0_exit_confirmed']}",
        f"FK valid/unavailable: {len(valid)}/{len(rows)-len(valid)}. Error = actual − target.", '',
        '| DOF | motion RMSE | motion max absolute error | Mode 0 return error |',
        '|---|---:|---:|---:|']
    for i, name in enumerate(NAMES):
        values = [summary['pose_error_motion_mm_deg']['rmse'][i] if motion else None,
                  summary['pose_error_motion_mm_deg']['max_abs'][i] if motion else None,
                  summary['mode0_return_error_mm_deg'][i] if summary['mode0_return_error_mm_deg'] else None]
        lines.append('| '+name+' ('+summary['units'][i]+') | '+' | '.join('UNAVAILABLE' if v is None else f'{v:.6f}' for v in values)+' |')
    lines += ['', f"Yaw: {summary['yaw']}",
        f"PID clipped rows per motor: {summary.get('pid_increment_clipped_rows_per_axis', 'UNAVAILABLE')}",
        f"Force shaped rows per motor (slew/absolute clipping, excludes ≤0.51 g rounding): {summary.get('force_shaped_rows_per_axis', 'UNAVAILABLE')}",
        f"Absolute force cap rows: {summary.get('force_absolute_cap_rows_per_axis', 'UNAVAILABLE')}",
        f"Maximum |current| mA: {summary.get('current_ma', {}).get('max_abs', 'UNAVAILABLE')}",
        f"Control interval ms: {summary.get('control_interval_ms', 'UNAVAILABLE')}",
        f"Recorded runtime error: {summary['error']}", '', *summary['interpretation'], '',
        'Full per-axis force/P/I/D/current/interval/readback statistics are in summary.json.',
        'A successful existing final hold uses six joint errors/velocities plus Z tolerance; full six-DOF return errors are reported, not silently accepted as zero.']
    (out/'summary.md').write_text('\n'.join(lines)+'\n')
    if rows:
        plot_recording(out, rows, initial, summary)
    return summary


def plot_recording(out, rows, initial, summary):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    t = np.array([(r['ns']-rows[0]['ns'])*1e-9 for r in rows])
    pose_target = np.array([pose_error(r['target_pose_m_rad'], initial)*SCALE for r in rows])
    pose_actual = np.array([pose_error(r['actual_pose_m_rad'], initial)*SCALE
        if r['actual_pose_m_rad'] is not None else [np.nan]*6 for r in rows])
    stage_changes = [t[i] for i in range(1, len(rows)) if rows[i]['phase'] != rows[i-1]['phase']]

    def decorate(ax, ylabel):
        ax.set_ylabel(ylabel); ax.grid(alpha=.25)
        for x in stage_changes:
            ax.axvline(x, color='grey', alpha=.3, linewidth=.8)

    def save(fig, filename, title):
        fig.suptitle(summary['source']+' — '+title+' — '+summary['outcome'])
        fig.tight_layout(); fig.savefig(out/filename, dpi=150); plt.close(fig)

    fig, axs = plt.subplots(6, 2, figsize=(14, 13), sharex=True)
    for i, ax in enumerate(axs[:, 0]):
        ax.plot(t, pose_target[:, i], label='target'); ax.plot(t, pose_actual[:, i], label='FK', linewidth=.8)
        decorate(ax, NAMES[i]+' '+summary['units'][i])
        # Wrap RPY error exactly as in numeric metrics, even near +/-pi.
        error = np.array([pose_error(r['actual_pose_m_rad'], r['target_pose_m_rad'])[i]*SCALE[i]
            if r['actual_pose_m_rad'] is not None else np.nan for r in rows])
        axs[i, 1].plot(t, error, linewidth=.8); decorate(axs[i, 1], 'FK − target / '+summary['units'][i])
    axs[0, 0].legend(loc='upper right')
    for ax in axs[-1]: ax.set_xlabel('elapsed host time / s')
    save(fig, 'pose_tracking.png', 'pose change from measured start')
    fig, axs = plt.subplots(6, 2, figsize=(14, 13), sharex=True)
    target = np.array([r['target_positions_steps'] for r in rows]); actual = np.array([r['actual_positions_steps'] for r in rows])
    for i in range(6):
        axs[i, 0].plot(t, target[:, i], label='target'); axs[i, 0].plot(t, actual[:, i], label='measured', linewidth=.8)
        axs[i, 1].plot(t, target[:, i]-actual[:, i]); decorate(axs[i, 0], f'M{i+1} / steps'); decorate(axs[i, 1], 'target − measured / steps')
    axs[0, 0].legend(); axs[-1, 0].set_xlabel('time / s'); axs[-1, 1].set_xlabel('time / s')
    save(fig, 'joint_tracking.png', 'actuator tracking and error')
    fig, axs = plt.subplots(6, 3, figsize=(17, 14), sharex=True)
    for i in range(6):
        for key, label in [('joint_p_n', 'P'), ('joint_i_n', 'I'), ('joint_d_n', 'D'),
                           ('joint_requested_delta_force_n', 'requested'), ('joint_delta_force_n', 'clipped output')]:
            axs[i, 0].plot(t, [r['pid'][key][i] for r in rows], label=label, linewidth=.8)
        for key, label in [('target_force_g', 'target 0x27'), ('force_2c_g', 'actual 0x2C')]:
            axs[i, 1].plot(t, [r[key][i] for r in rows], label=label, linewidth=.8)
        reads = [(x, r['target_force_27_g_readback'][i]) for x, r in zip(t, rows) if r['target_force_27_g_readback'] is not None]
        if reads:
            axs[i, 1].scatter(*zip(*reads), s=4, label='0x27 readback')
        axs[i, 2].plot(t, [r['currents_ma'][i] for r in rows], linewidth=.8)
        for j, unit in enumerate(('PID / N', 'force / g', 'current / mA')):
            decorate(axs[i, j], f'M{i+1} '+unit)
    axs[0, 0].legend(fontsize=8); axs[0, 1].legend(fontsize=8)
    for ax in axs[-1]: ax.set_xlabel('time / s')
    save(fig, 'force_pid_current.png', 'PI-D, forces and currents')
    fig, axs = plt.subplots(1, 2, figsize=(12, 5))
    for ax, key, title in zip(axs, ('motion_error_correlations', 'motion_target_error_correlations'), ('residual / residual', 'target / residual')):
        matrix = np.array([[np.nan if v is None else v for v in r] for r in summary[key]])
        im = ax.imshow(matrix, vmin=-1, vmax=1, cmap='coolwarm')
        ax.set_xticks(range(6), NAMES); ax.set_yticks(range(6), NAMES); ax.set_title(title)
        fig.colorbar(im, ax=ax, shrink=.8)
    save(fig, 'error_correlations.png', 'descriptive correlations; not causal coupling')
