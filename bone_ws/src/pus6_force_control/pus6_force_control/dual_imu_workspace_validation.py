"""Offline large-angle IK/reference-chain evaluation. No ROS or serial owners.

Geometry samples and ideal one-cycle-delayed position feedback are software
evidence only; neither predicts stopping distance nor verifies stabilization.
"""
import csv
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from .dual_imu_attitude import load_attitude_parameters
from .mode3_dual_imu import DualImuPlan
from .joint_position_force_pid import JointPositionForcePID
from .six_axis_force_drive import SixAxisForceDrive
from .z_position_motion import pid_config, slider_to_steps
from pus6_motor_driver.force_units import g_to_n


HISTORIES = (
    ('large_angle_A', 'j12/j12_1_large_angle_static_load/20261005_104403/j12_1a_static_load_neg25deg_feedforward_powered.csv'),
    ('large_angle_B', 'j12/j12_1_large_angle_static_load/20261005_110720/j12_1b_static_load_neg30deg_feedforward_powered.csv'),
    ('j11_ff', 'j11/runtime_validation/20260918_112354/J11_DISTURB_QUAL_feedforward_powered.csv'),
    ('j11_pid', 'j11/m2_pid/20260920_095850/j11_m2_7a_pitchd_cand2_repeat_phaseB_dynamic_telemetry_powered.csv'),
    ('j14_pid', 'j14/controller_compare/pid/run_002/telemetry.csv'),
    ('j14_adrc', 'j14/controller_compare/adrc/run_002/telemetry.csv'),
    ('combined', 'j12/j12_2_combined_quasistatic/20261005_122117/j12_2_combined_screening_feedforward_powered.csv'),
    ('dynamic', 'j12/j12_3_dynamic_performance/20261005_152615/j12_3_dynamic_screening_telemetry_powered.csv'),
)
C = Rotation.from_euler('z', 90, degrees=True).as_matrix()


def compensation_pose(plan, base_rpy_deg, residual_deg=(0., 0., 0.)):
    """Same retained rotation-vector mask, world residual and Euler mask."""
    rb = Rotation.from_euler('xyz', base_rpy_deg, degrees=True)
    rb = Rotation.from_rotvec(rb.as_rotvec()*plan.controller.mask)
    command = rb.inv()*Rotation.from_rotvec(np.deg2rad(residual_deg))*Rotation.from_matrix(plan.controller.r0)
    pose = plan.initial_pose.copy()
    pose[3:] = np.where(plan.controller.mask, command.as_euler('xyz'), pose[3:])
    return pose


def validate_workspace(reference_run, output_dir, *, profile_samples=3001):
    reference = Path(reference_run).resolve()
    original = json.loads((reference/'dual_imu_report.json').read_text())
    terminal = original.get('terminal_recovery', {})
    axes = terminal.get('axes', [])
    if not original.get('all_mode0_exit_confirmed') or len(axes) != 6:
        raise ValueError('reference must contain six confirmed measured Mode 0 anchors')
    steps = [a['anchor_steps'] for a in sorted(axes, key=lambda a: a['motor_id'])]
    out = Path(output_dir).resolve(); out.mkdir(parents=True, exist_ok=False)
    parameters = load_attitude_parameters()

    class Input:
        kind = 'OFFLINE_ORIENTATION'
        now = 1_000_000_000
        qb = [0., 0., 0., 1.]
        def snapshot(self, now):
            return dict(sample_ns=self.now, now_ns=self.now,
                base_q_xyzw=self.qb, platform_q_xyzw=[0., 0., 0., 1.])

    def make_plan(enabled=('roll', 'pitch', 'yaw')):
        source = Input()
        return DualImuPlan(steps, source=source, parameters=parameters, axes=enabled,
            clock=lambda: source.now)

    plan = make_plan()
    groups = {}
    def evaluate(name, samples, enabled=('roll', 'pitch', 'yaw'), residuals=((0., 0., 0.),)):
        p = make_plan(enabled); low = np.full(6, np.inf); high = -low.copy()
        count = rejected = 0; bad_example = None
        with (out/(name+'.csv')).open('x', newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(['base_roll_deg','base_pitch_deg','base_yaw_deg','residual_roll_deg','residual_pitch_deg','residual_yaw_deg','reachable']+[f'q{i}_steps' for i in range(1,7)])
            for base in samples:
                for residual in residuals:
                    pose = compensation_pose(p, base, residual)
                    try:
                        _, q, diagnostic = p.constrain_pose(pose)
                    except RuntimeError:
                        rejected += 1; diagnostic = p.target_request
                        if bad_example is None: bad_example = diagnostic.copy()
                    q_steps = diagnostic['requested_positions_steps']
                    if q_steps is not None:
                        low = np.minimum(low, q_steps); high = np.maximum(high, q_steps)
                    count += 1
                    writer.writerow(list(base)+list(residual)+[diagnostic['target_reachable']]+(q_steps or [None]*6))
        result = dict(samples=count, rejected=rejected, enabled_axes=list(enabled),
            min_steps=low.tolist(), max_steps=high.tolist(), total_travel_mm=((high-low)*.025).tolist(),
            mechanical_remaining_lower_steps=(low-p.lower_steps).tolist(),
            mechanical_remaining_upper_steps=(p.upper_steps-high).tolist(),
            target_remaining_lower_steps=(low-p.target_min_steps).tolist(),
            target_remaining_upper_steps=(p.target_max_steps-high).tolist(), first_rejected=bad_example)
        groups[name] = result
        return result

    # Three-dimensional ray boundaries, not a 20-degree angular ceiling.
    # Eight residual corners reserve the unchanged +/-0.5deg LADRC output.
    residual3 = [(r, p, y) for r in (-.5, .5) for p in (-.5, .5) for y in (-.5, .5)]
    directions = [np.array([r,p,y],float) for r in (-2,-1,1,2) for p in (-2,-1,1,2) for y in (-2,-1,1,2)]
    directions += [np.array(d,float) for d in ((4,1,1),(-4,1,1),(1,4,1),(1,-4,1),(1,1,4),(1,1,-4))]
    rays = []
    for direction in directions:
        direction /= np.linalg.norm(direction)
        def feasible(degrees):
            base = Rotation.from_rotvec(direction*np.deg2rad(degrees)).as_euler('xyz',degrees=True)
            for residual in residual3:
                try: plan.constrain_pose(compensation_pose(plan, base, residual))
                except RuntimeError: return False
            return True
        lo = 0.; hi = None
        for degrees in range(1,76):
            if not feasible(degrees): hi = float(degrees); break
            lo = float(degrees)
        if hi is not None:
            for _ in range(24):
                mid = (lo+hi)/2
                if feasible(mid): lo = mid
                else: hi = mid
        base = Rotation.from_rotvec(direction*np.deg2rad(lo)).as_euler('xyz',degrees=True)
        rays.append(dict(rotation_vector_direction=direction.tolist(), feasible_so3_deg=lo,
            boundary_base_rpy_deg=base.tolist(), boundary_found=hi is not None,
            residual_corners_checked_deg=.5, boundary_reason='IK target workspace or kinematic reachability'))
    (out/'three_axis_boundaries.json').write_text(json.dumps(rays,indent=2)+'\n')
    cases = [(15,10,5),(15,-10,-5),(-15,10,-5),(-15,-10,5)]
    evaluate('historical_case_ABCD', cases, residuals=residual3)

    # Read both repositories' raw ACTIVE records; do not treat copied histories
    # as independent Force or native-torque experiments.
    root = Path(__file__).resolve().parents[4]
    history_evidence = []
    for repo in ('position', 'torque'):
        for name, relative in HISTORIES:
            path = root.parent/('6pus_bone_drill_'+repo+'_control')/'results'/relative
            if not path.exists():
                history_evidence.append(dict(path=str(path), available=False)); continue
            rows = [r for r in csv.DictReader(path.open()) if r['state'] == 'ACTIVE']
            values = np.array([[float(r['base_rel_'+a+'_deg']) for a in ('roll', 'pitch', 'yaw')] for r in rows])
            history_evidence.append(dict(path=str(path), available=True, active_samples=len(rows),
                base_min_deg=values.min(0).tolist(), base_max_deg=values.max(0).tolist(),
                base_max_so3_deg=float(np.rad2deg(Rotation.from_euler('xyz',values,degrees=True).magnitude()).max())))
            evaluate(repo+'_'+name+'_recorded_full_rpy', values, ('roll', 'pitch', 'yaw'))

    # Smooth closed paths through the actual controller -> IK -> PI-D -> g.
    # Measured q is the previous reference, not a model of the real plant.
    # One maximum-excursion three-axis closed path. These are explicitly
    # constructed offline test waypoints, never scaling live IMU requests.
    selected = [rays[i] for i in (0,3,12,15,48,51,60,63,64,65,66,67,68,69)]
    waypoints = [(0.,0.,0.)]+[Rotation.from_rotvec(np.array(r['rotation_vector_direction'])*
        np.deg2rad(.95*r['feasible_so3_deg'])).as_euler('xyz',degrees=True) for r in selected]+[(0.,0.,0.)]
    profiles = [('maximum_three_axis', waypoints, ('roll','pitch','yaw'))]
    profiles_report = {}
    for name, waypoints, enabled in profiles:
        p = make_plan(enabled); src = p.source; limits = p.force_limits()
        pid = JointPositionForcePID(pid_config(p), clock=lambda: src.now, force_slew_compensation=False)
        pid.reset(p.initial_q, sample_ns=src.now, now_ns=src.now, position_stamps_ns=[src.now]*6)
        drive = SixAxisForceDrive(None, limits, clock=lambda: src.now)
        hold = np.asarray(original['hold_forces_g']); drive.anchor_hold_forces_g(hold, now_ns=src.now)
        measured = p.initial_q.copy(); targets = []; forces = []; clipped = 0
        peak_ref_speed = np.zeros(6); previous_target = measured.copy(); previous_t = 0.
        with (out/('trajectory_'+name+'.jsonl')).open('x') as stream:
            for t in np.linspace(0, 60, profile_samples):
                progress = t/60*(len(waypoints)-1)
                k = min(int(progress), len(waypoints)-2); u = progress-k
                smooth = 10*u**3-15*u**4+6*u**5
                base = np.asarray(waypoints[k])+(np.asarray(waypoints[k+1])-waypoints[k])*smooth
                rb = Rotation.from_euler('xyz', base, degrees=True).as_matrix()
                src.qb = Rotation.from_matrix(C.T@rb@C).as_quat().tolist()
                src.now = 1_020_000_000+int(t*1e9)
                pose, target = p.target('stabilize', t)
                expected = compensation_pose(p, base)
                if not np.allclose(pose, expected, rtol=0, atol=1e-12):
                    raise AssertionError('offline geometry composition differs from retained controller')
                p.check_positions(slider_to_steps(measured, p.geometry))
                delta, diag = pid.update(target, measured, sample_ns=src.now, now_ns=src.now, position_stamps_ns=[src.now]*6)
                command = drive.command_forces_n(np.asarray(g_to_n(hold))+delta, dry_run=True, now_ns=src.now)
                force = [f['target_force_g'] for f in command['frames']]
                pid.track_applied(np.asarray(g_to_n(force))-np.asarray(g_to_n(hold)))
                if t > previous_t: peak_ref_speed = np.maximum(peak_ref_speed, abs(target-previous_target)/(t-previous_t))
                previous_target = target.copy(); previous_t = t
                targets.append(slider_to_steps(target, p.geometry)); forces.append(force)
                clipped += int(diag['joint_output_limited'])
                stream.write(json.dumps(dict(time_s=float(t), base_rpy_deg=base.tolist(), target_pose_m_rad=pose.tolist(),
                    target_positions_steps=targets[-1].tolist(), ideal_measured_steps=slider_to_steps(measured,p.geometry).tolist(),
                    target_constraint=p.target_request, pid=diag, target_force_g=force, hardware_write_count=0))+'\n')
                measured = target.copy()
        q = np.asarray(targets); f = np.asarray(forces)
        profiles_report[name] = dict(samples=len(q), duration_s=60, min_steps=q.min(0).tolist(), max_steps=q.max(0).tolist(),
            return_error_steps=(q[-1]-steps).tolist(), peak_reference_speed_mm_s=(peak_ref_speed*1000).tolist(),
            force_min_g=f.min(0).tolist(), force_max_g=f.max(0).tolist(), pid_clipped_frames=clipped)

    report = dict(source='OFFLINE_IK_AND_IDEAL_FEEDBACK', hardware_write_count=0, reference_run=str(reference),
        reference_basis='last measured terminal Mode 0 anchors; not a live/current-state observation',
        initial_steps=steps, initial_pose_m_rad=plan.initial_pose.tolist(), workspace=plan.summary(),
        three_axis_boundaries=rays, groups=groups, profiles=profiles_report, history_evidence=history_evidence,
        offline_path_waypoints_base_rpy_deg=[list(x) for x in waypoints], offline_boundary_fraction=.95,
        validation_completed=True,
        all_requested_targets_reachable=all(v['rejected'] == 0 for v in groups.values()),
        limitations=['Directional boundary search and sampled historical poses are geometry evidence, not collision or full continuous workspace certification.',
            'Profiles use ideal one-cycle delayed q and a fixed world Platform IMU; not plant dynamics or hardware stability.',
            'Historical data in Position and Torque are shared legacy records, not independent Mode 3 force trials.',
            'Stopping reserve is an engineering margin; worst-case stopping distance at future large-angle motion is unmeasured.'])
    (out/'workspace_report.json').write_text(json.dumps(report, indent=2)+'\n')
    return report
