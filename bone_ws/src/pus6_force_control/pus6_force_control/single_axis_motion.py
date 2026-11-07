"""Sequential measured-start X/Y/Roll/Pitch/Yaw trials on the Z PI-D baseline.

Only the pose trajectory and experiment sequencing are extended. Native force
commands, PI-D, measurements, bounds and terminal handling use the Z runner.
"""
from dataclasses import asdict
from datetime import datetime
from functools import partial
import argparse
import json
import math
from pathlib import Path
import time

import numpy as np

from pus6_control.joint_trajectory_controller import quintic_progress
from pus6_kinematics.analytical_6pus import Analytical6PusKinematics
from pus6_kinematics.kinematics_interface import KinematicsError
from .force_pipeline import raw_steps_to_slider_m
from .z_position_motion import MotionConfig, ZPlan, run, slider_to_steps

AXES = ('x', 'y', 'z', 'roll', 'pitch', 'yaw')
EXPERIMENT_AXES = ('x', 'y', 'roll', 'pitch', 'yaw')
VERIFIED_PEAK_SPEED_M_S = .0001875  # successful 10 mm / 100 s Z quintic


class SingleAxisPlan(ZPlan):
    def __init__(self, initial_steps, settings=None, *, axis, amplitude,
                 kinematics=None, device_limits=None):
        if axis not in AXES or not np.isfinite(amplitude) or amplitude <= 0:
            raise ValueError('valid pose axis and positive SI amplitude required')
        self.axis = axis
        self.axis_index = AXES.index(axis)
        self.requested_amplitude = float(amplitude)
        self.amplitude = float(amplitude)
        self.settings = settings or MotionConfig()
        self.kinematics = kinematics or Analytical6PusKinematics()
        self.geometry = g = self.kinematics.geometry_config
        self.initial_q = raw_steps_to_slider_m(initial_steps, g)
        fk = self.kinematics.forward_kinematics_with_metrics(self.initial_q)
        if not fk.converged or not np.isfinite(fk.pose).all():
            raise ValueError('initial measured-position FK failed')
        self.initial_pose = np.asarray(fk.pose)
        self.ik_start = np.asarray(self.kinematics.inverse_kinematics(self.initial_pose))
        self.lower_steps = np.full(6, max(g.normal_motion_min_steps, g.absolute_raw_min_steps), dtype=float)
        self.upper_steps = np.full(6, min(g.normal_motion_max_steps, g.absolute_raw_max_steps,
                                         g.software_command_max_steps), dtype=float)
        if device_limits is not None:
            limits = np.asarray(device_limits, dtype=float)
            if limits.shape != (6, 2) or not np.isfinite(limits).all() or (limits[:, 0] >= limits[:, 1]).any():
                raise ValueError('six valid device stroke limits required')
            self.lower_steps = np.maximum(self.lower_steps, limits[:, 0])
            self.upper_steps = np.minimum(self.upper_steps, limits[:, 1])
        start = np.asarray(initial_steps)
        if (start < self.lower_steps).any() or (start > self.upper_steps).any():
            raise ValueError('initial actual position outside normal/device mechanical stroke')
        # Sample the entire +/- pose path before Mode 3 entry. Reduce only the
        # requested amplitude, never the physical stroke boundaries.
        for _ in range(25):
            try:
                path = np.array([slider_to_steps(self.at_fraction(f)[1], g) for f in np.linspace(-1., 1., 1001)])
                if not np.isfinite(path).all() or (path < self.lower_steps-1e-7).any() or (path > self.upper_steps+1e-7).any():
                    raise ValueError('IK path exceeds mechanical stroke')
                break
            except (ValueError, KinematicsError):
                self.amplitude *= .8
        else:
            raise ValueError('no bounded single-axis IK trajectory at this actual start')
        self.path_min_steps = path.min(0)
        self.path_max_steps = path.max(0)
        self.envelope_min_steps = np.maximum(self.lower_steps, self.path_min_steps-self.settings.envelope_margin_steps)
        self.envelope_max_steps = np.minimum(self.upper_steps, self.path_max_steps+self.settings.envelope_margin_steps)
        self.segment_fractions = dict(positive_move=(0., 1.), negative_move=(1., -1.), return_to_start=(-1., 0.))
        self.segment_seconds = {}
        self.segment_peak_speeds_m_s = {}
        tau = np.linspace(0., 1., 1001)
        blend = np.array([quintic_progress(t) for t in tau])
        for phase, (a, b) in self.segment_fractions.items():
            q = np.array([self.at_fraction(a+(b-a)*s)[1] for s in blend])
            # Numerical IK derivative over normalized time includes curvature;
            # 1% margin keeps the sampled peak below the verified Z speed.
            peak_dq_dtau = float(np.abs(np.gradient(q, tau, axis=0)).max())
            duration = max(min(20., self.settings.move_seconds),
                           math.ceil(1.01*peak_dq_dtau/VERIFIED_PEAK_SPEED_M_S))
            self.segment_seconds[phase] = duration
            self.segment_peak_speeds_m_s[phase] = peak_dq_dtau/duration
        self.peak_speed_m_s = max(self.segment_peak_speeds_m_s.values())
        self.phase_schedule = [('initial_hold', None), ('positive_move', self.segment_seconds['positive_move']),
            ('positive_hold', None), ('negative_move', self.segment_seconds['negative_move']),
            ('negative_hold', None), ('return_to_start', self.segment_seconds['return_to_start']), ('final_hold', None)]

    def at_fraction(self, fraction):
        pose = self.initial_pose.copy()
        # Existing model convention: mechanical XYZ meters, ZYX Euler radians.
        pose[self.axis_index] += self.amplitude*float(fraction)
        return pose, self.initial_q+np.asarray(self.kinematics.inverse_kinematics(pose))-self.ik_start

    def target(self, phase, elapsed_s):
        if phase in self.segment_fractions:
            a, b = self.segment_fractions[phase]
            fraction = a+(b-a)*quintic_progress(elapsed_s/self.segment_seconds[phase])
        else:
            fraction = {'positive_hold': 1., 'negative_hold': -1.}.get(phase, 0.)
        return self.at_fraction(fraction)

    def summary(self):
        return dict(super().summary(), axis=self.axis, requested_amplitude_si=self.requested_amplitude,
                    selected_amplitude_si=self.amplitude,
                    negative_steps=slider_to_steps(self.at_fraction(-1.)[1], self.geometry).tolist(),
                    segment_seconds=self.segment_seconds, segment_peak_speeds_m_s=self.segment_peak_speeds_m_s,
                    phase_schedule=self.phase_schedule, coordinate_convention='mechanical XYZ / ZYX Euler')


class YawHoldContinuationPlan(SingleAxisPlan):
    """Resume an ordinary hold timeout, retaining the original trial origin.

    The UART owner still reads the actual entry position, re-captures holding
    force and resets the unchanged PI-D there. This is the unfinished negative
    hold and return of the same yaw experiment, not a new combined trajectory.
    """
    def __init__(self, actual_steps, settings=None, *, reference_report, **kwargs):
        reference = reference_report
        if (reference.get('plan', {}).get('axis') != 'yaw'
                or not reference.get('all_mode0_exit_confirmed')
                or reference.get('error') != 'RuntimeError: negative_hold did not settle/hold within timeout'):
            raise ValueError('continuation only supports a confirmed Mode0 ordinary yaw hold timeout')
        super().__init__(reference['plan']['initial_steps'], settings, axis='yaw',
                         amplitude=reference['plan']['selected_amplitude_si'], **kwargs)
        self.entry_q = raw_steps_to_slider_m(actual_steps, self.geometry)
        entry_fk = self.kinematics.forward_kinematics_with_metrics(self.entry_q)
        if not entry_fk.converged:
            raise ValueError('actual continuation entry FK failed')
        self.entry_pose = np.asarray(entry_fk.pose)
        self.check_positions(actual_steps)
        previous_duration = self.segment_seconds['return_to_start']
        self.segment_seconds['return_to_start'] = max(60., previous_duration)
        self.segment_peak_speeds_m_s['return_to_start'] *= previous_duration/self.segment_seconds['return_to_start']
        self.peak_speed_m_s = self.segment_peak_speeds_m_s['return_to_start']
        self.phase_schedule = [('initial_hold', None), ('negative_hold', None),
            ('return_to_start', self.segment_seconds['return_to_start']), ('final_hold', None)]

    def target(self, phase, elapsed_s):
        if phase == 'initial_hold':
            return self.entry_pose.copy(), self.entry_q.copy()
        return super().target(phase, elapsed_s)

    def summary(self):
        return dict(super().summary(), continuation='negative_hold_then_original_start',
                    origin_role='ORIGINAL_EXPERIMENT_ORIGIN',
                    measured_entry_steps=slider_to_steps(self.entry_q, self.geometry).tolist(),
                    measured_entry_pose_m_rad=self.entry_pose.tolist())


def pose_error(actual, target):
    error = np.asarray(actual)-np.asarray(target)
    error[..., 3:] = (error[..., 3:]+np.pi) % (2*np.pi)-np.pi
    return error


def analyze_run(directory):
    """Read saved measurements after Mode 0 exit, before starting the next axis."""
    directory = Path(directory)
    report = json.loads((directory/'z_position_report.json').read_text())
    rows = [json.loads(line) for line in (directory/'tracking.jsonl').open()] if (directory/'tracking.jsonl').exists() else []
    result = dict(source=report['source'], outcome=report['outcome'], axis=report.get('plan', {}).get('axis'),
                  all_mode0_exit_confirmed=report.get('all_mode0_exit_confirmed', False),
                  hardware_write_count=report.get('hardware_write_count', 0), error=report.get('error'),
                  phase_metrics={}, fk_source='FK_FROM_SEQUENTIAL_ACTUATOR_FEEDBACK')
    terminal_axes = sorted((report.get('terminal_recovery') or {}).get('axes', []), key=lambda a: a['motor_id'])
    if (len(terminal_axes) == 6 and all(a.get('final_mode') == 0 and a.get('observations') for a in terminal_axes)
            and report.get('plan')):
        terminal_positions = [a['observations'][-1]['position_steps'] for a in terminal_axes]
        kin = Analytical6PusKinematics()
        fk = kin.forward_kinematics_with_metrics(raw_steps_to_slider_m(terminal_positions, kin.geometry_config))
        result['mode0_observed_positions_steps'] = terminal_positions
        result['mode0_fk_converged'] = bool(fk.converged)
        if fk.converged:
            result['mode0_observed_pose_m_rad'] = list(fk.pose)
            result['mode0_return_error_mm_deg'] = (pose_error(fk.pose, report['plan']['initial_pose_m_rad'])
                * np.array([1000.]*3+[180./np.pi]*3)).tolist()
    if rows:
        valid = [r for r in rows if r['actual_pose_m_rad'] is not None]
        currents = np.asarray([r['currents_ma'] for r in rows])
        result.update(peak_current_ma=np.abs(currents).max(0).tolist(),
            max_joint_error_steps=np.abs([r['position_error_steps'] for r in rows]).max(0).tolist(),
            pid_output_g_range=np.array([np.min([r['pid_output_g'] for r in rows], axis=0),
                                         np.max([r['pid_output_g'] for r in rows], axis=0)]).tolist(),
            target_force_g_range=[np.min([r['target_force_g'] for r in rows], axis=0).tolist(),
                                  np.max([r['target_force_g'] for r in rows], axis=0).tolist()],
            actual_force_g_range=[np.min([r['force_2c_g'] for r in rows], axis=0).tolist(),
                                  np.max([r['force_2c_g'] for r in rows], axis=0).tolist()],
            fault_rows=sum(any(r['fault_codes']) for r in rows),
            mode_loss_rows=sum(r['modes_last_readback'] != [3]*6 for r in rows),
            force_readback_mismatches=sum(r['target_force_27_g_readback'] != r['target_force_g']
                for r in rows if r['target_force_27_g_readback'] is not None), missing_fk_rows=len(rows)-len(valid),
            pid_limited_cycle_fraction=sum(r['pid']['joint_output_limited'] for r in rows)/len(rows))
        if valid:
            initial = np.array(report['plan']['initial_pose_m_rad'])
            actual = np.array([r['actual_pose_m_rad'] for r in valid])
            target = np.array([r['target_pose_m_rad'] for r in valid])
            scale = np.array([1000.]*3+[180./np.pi]*3)
            delta = pose_error(actual, initial)*scale
            index = AXES.index(result['axis'])
            cross = np.max(abs(delta), axis=0);cross[index] = 0.
            result.update(actual_delta_range_mm_deg=[delta.min(0).tolist(), delta.max(0).tolist()],
                cross_motion_peak_mm_deg=cross.tolist(),
                pose_tracking_rmse_mm_deg=(np.sqrt(np.mean(pose_error(actual, target)**2, axis=0))*scale).tolist(),
                final_return_error_mm_deg=(pose_error(actual[-1], initial)*scale).tolist())
            for phase in dict.fromkeys(r['phase'] for r in rows):
                part = [r for r in valid if r['phase'] == phase]
                if not part:
                    continue
                errors = pose_error([r['actual_pose_m_rad'] for r in part], [r['target_pose_m_rad'] for r in part])*scale
                tail = [r for r in part if r['ns'] >= part[-1]['ns']-1_000_000_000]
                tail_errors = pose_error([r['actual_pose_m_rad'] for r in tail], [r['target_pose_m_rad'] for r in tail])*scale
                result['phase_metrics'][phase] = dict(samples=len(part),
                    axis_error_rmse_mm_or_deg=float(np.sqrt(np.mean(errors[:, index]**2))),
                    tail_axis_mean_error_mm_or_deg=float(tail_errors[:, index].mean()),
                    tail_pose_mean_error_mm_deg=tail_errors.mean(0).tolist(),
                    tail_joint_mean_error_steps=np.mean([r['position_error_steps'] for r in tail], axis=0).tolist(),
                    peak_current_ma=np.abs([r['currents_ma'] for r in part]).max(0).tolist())
    (directory/'analysis.json').write_text(json.dumps(result, indent=2)+'\n')
    return result


def run_suite(root, config, settings=None, *, axes=EXPERIMENT_AXES):
    root = Path(root)
    root.mkdir(parents=True, exist_ok=True)
    results = []
    for axis in axes:
        out = root/axis
        amplitude = .005 if axis in ('x', 'y') else np.deg2rad(2.)
        print('BEGIN_AXIS:', axis, flush=True)
        run(dict(config, output_dir=str(out)), settings,
            plan_factory=partial(SingleAxisPlan, axis=axis, amplitude=amplitude))
        # Reload the actual result directory, rather than relying on memory of
        # the command. No later experiment starts before this analysis finishes.
        analysis = analyze_run(out)
        results.append(analysis)
        (root/'suite_summary.json').write_text(json.dumps(results, indent=2)+'\n')
        print('AXIS_ANALYZED:', json.dumps(analysis), flush=True)
        if analysis['outcome'] != 'motion_complete' or not analysis['all_mode0_exit_confirmed']:
            print('STOP_SUITE:', axis, analysis.get('error'), flush=True)
            return results
    print('FIVE_AXES_COMPLETE: no combined trajectory will be started', flush=True)
    return results


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--hardware', action='store_true')
    parser.add_argument('--hardware-qualified', action='store_true')
    parser.add_argument('--dry-run', action='store_true')
    parser.add_argument('--port', default='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0')
    parser.add_argument('--output-dir')
    parser.add_argument('--axes', nargs='+', choices=EXPERIMENT_AXES, default=list(EXPERIMENT_AXES))
    args = parser.parse_args(argv)
    if args.hardware and not args.dry_run and not args.hardware_qualified:
        parser.error('--hardware requires --hardware-qualified')
    if not args.hardware or args.dry_run:
        for axis in args.axes:
            plan = SingleAxisPlan([1000]*6, axis=axis, amplitude=.005 if axis in ('x', 'y') else np.deg2rad(2.))
            print(json.dumps(dict(source='OFFLINE', hardware_write_count=0, plan=plan.summary())))
        return 0
    root = Path(args.output_dir) if args.output_dir else Path(__file__).resolve().parents[4]/'results/mode3_single_axis'/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    config = dict(epoch=int(time.time()), hardware_mode='active', port=args.port, baudrate=921600,
        execute=True, dry_run=False, hardware_write_enabled=True, is_hardware_qualified=True,
        control_mode='force', maintenance_only=True, force_fault_recovery='position_reanchor',
        max_force_handover_displacement_steps=6)
    results = run_suite(root, config, axes=args.axes)
    print('SUITE_DIRECTORY:', root, flush=True)
    return 0 if len(results) == len(args.axes) and all(r['outcome'] == 'motion_complete' and r['all_mode0_exit_confirmed'] for r in results) else 2
