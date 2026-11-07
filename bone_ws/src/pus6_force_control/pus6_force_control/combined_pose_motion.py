"""One simultaneous six-DOF closed trajectory on the native Mode 3 PI-D runner.

No controller retuning, wrench allocation or automatic hardware execution.
Default invocation is offline; --read-only cannot send register writes.
"""
from dataclasses import asdict, dataclass
from datetime import datetime
from functools import partial
import argparse
import json
import math
from pathlib import Path
import subprocess
import time

import numpy as np

from pus6_control.joint_trajectory_controller import quintic_progress
from pus6_kinematics.analytical_6pus import Analytical6PusKinematics
from pus6_kinematics.kinematics_interface import KinematicsError
from pus6_motor_driver.motor import Motor
from pus6_motor_driver.safety import SerialPortLock
from pus6_motor_driver.uart_bus import UartBus
from .force_pipeline import raw_steps_to_slider_m
from .single_axis_motion import VERIFIED_PEAK_SPEED_M_S
from .z_position_motion import MotionConfig, ZPlan, run, slider_to_steps

NAMES = ('X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw')
SCALE = np.array([1000.]*3+[180./np.pi]*3)


@dataclass(frozen=True)
class CombinedTrajectoryConfig:
    amplitudes_si: tuple = (.002, .002, .002, np.pi/180., np.pi/180., np.pi/90.)
    harmonics: tuple = (2, 3, 1, 4, 5, 6)
    requested_period_s: float = 180.
    ramp_fraction: float = 1./6.

    def __post_init__(self):
        a = np.asarray(self.amplitudes_si, dtype=float)
        h = np.asarray(self.harmonics, dtype=float)
        if a.shape != (6,) or not np.isfinite(a).all() or (a <= 0).any():
            raise ValueError('six finite positive SI amplitudes required')
        if h.shape != (6,) or not np.isfinite(h).all() or (h < 1).any() or (h != h.astype(int)).any():
            raise ValueError('six positive integer harmonics required')
        if not np.isfinite(self.requested_period_s) or self.requested_period_s <= 0:
            raise ValueError('positive finite trajectory period required')
        if not np.isfinite(self.ramp_fraction) or not 0 < self.ramp_fraction < .25:
            raise ValueError('ramp fraction must be between zero and one quarter')


class CombinedPosePlan(ZPlan):
    def __init__(self, actual_steps, settings=None, *, trajectory=None, kinematics=None, device_limits=None):
        self.settings = settings or MotionConfig()
        self.trajectory = trajectory or CombinedTrajectoryConfig()
        self.kinematics = kinematics or Analytical6PusKinematics()
        self.geometry = g = self.kinematics.geometry_config
        self.initial_q = raw_steps_to_slider_m(actual_steps, g)
        fk = self.kinematics.forward_kinematics_with_metrics(self.initial_q)
        if not fk.converged or not np.isfinite(fk.pose).all():
            raise ValueError('initial actual-position FK failed')
        self.initial_fk_metrics = dict(converged=bool(fk.converged), residual_norm_m=float(fk.residual_norm), iterations=int(fk.iterations))
        self.initial_pose = np.asarray(fk.pose)
        self.ik_start = np.asarray(self.kinematics.inverse_kinematics(self.initial_pose))
        self.lower_steps = np.full(6, max(g.absolute_raw_min_steps, g.normal_motion_min_steps), dtype=float)
        self.upper_steps = np.full(6, min(g.absolute_raw_max_steps, g.normal_motion_max_steps,
                                         g.software_command_max_steps), dtype=float)
        if device_limits is not None:
            limits = np.asarray(device_limits, dtype=float)
            if limits.shape != (6, 2) or not np.isfinite(limits).all() or (limits[:, 0] >= limits[:, 1]).any():
                raise ValueError('six valid device stroke limits required')
            self.lower_steps = np.maximum(self.lower_steps, limits[:, 0])
            self.upper_steps = np.minimum(self.upper_steps, limits[:, 1])
        start = np.asarray(actual_steps)
        if (start < self.lower_steps).any() or (start > self.upper_steps).any():
            raise ValueError('actual initial position outside normal/device stroke')
        self.amplitude_scale = 1.
        u = np.linspace(0., 1., 18001)
        self.sample_count = len(u)
        # Whole combined path, not a sum of independent single-axis maxima.
        for _ in range(25):
            try:
                poses_and_q = [self.at_fraction(t) for t in u]
                poses = np.array([p for p, _ in poses_and_q])
                path_q = np.array([q for _, q in poses_and_q])
                path = slider_to_steps(path_q, g)
                if not np.isfinite(path).all() or (path < self.lower_steps-1e-7).any() or (path > self.upper_steps+1e-7).any():
                    raise ValueError('combined IK path exceeds mechanical stroke')
                break
            except (ValueError, KinematicsError):
                self.amplitude_scale *= .8
        else:
            raise ValueError('no bounded combined IK trajectory at current actual start')
        self.path_min_steps = path.min(0)
        self.path_max_steps = path.max(0)
        self.envelope_min_steps = np.maximum(self.lower_steps, self.path_min_steps-self.settings.envelope_margin_steps)
        self.envelope_max_steps = np.minimum(self.upper_steps, self.path_max_steps+self.settings.envelope_margin_steps)
        peak_dq_du = np.abs(np.gradient(path_q, u, axis=0)).max(0)
        # Time scaling preserves all amplitudes first. A 2% numerical margin
        # covers the discretized derivative; no physical limit is increased.
        self.period_s = max(self.trajectory.requested_period_s,
                            float(math.ceil(1.02*peak_dq_du.max()/VERIFIED_PEAK_SPEED_M_S)))
        self.per_axis_peak_speed_m_s = peak_dq_du/self.period_s
        self.peak_speed_m_s = float(self.per_axis_peak_speed_m_s.max())
        self.sampled_pose_delta_min = (poses-self.initial_pose).min(0)
        self.sampled_pose_delta_max = (poses-self.initial_pose).max(0)
        self.phase_schedule = [('initial_hold', None), ('combined_motion', self.period_s), ('final_hold', None)]

    def at_fraction(self, fraction):
        u = float(np.clip(fraction, 0., 1.))
        if u == 0. or u == 1.:
            return self.initial_pose.copy(), self.initial_q.copy()
        ramp = self.trajectory.ramp_fraction
        envelope = quintic_progress(min(u/ramp, (1.-u)/ramp, 1.))
        delta = (np.asarray(self.trajectory.amplitudes_si)*self.amplitude_scale*envelope
                 * np.sin(2*np.pi*np.asarray(self.trajectory.harmonics)*u))
        pose = self.initial_pose+delta
        q = self.initial_q+np.asarray(self.kinematics.inverse_kinematics(pose))-self.ik_start
        return pose, q

    def target(self, phase, elapsed_s):
        return self.at_fraction(elapsed_s/self.period_s if phase == 'combined_motion' else 0.)

    def summary(self):
        return dict(trajectory='simultaneous_six_dof_quintic_enveloped_harmonics',
            initial_fk_metrics=self.initial_fk_metrics,
            initial_pose_m_rad=self.initial_pose.tolist(), initial_steps=slider_to_steps(self.initial_q, self.geometry).tolist(),
            requested_amplitudes_si=list(self.trajectory.amplitudes_si), amplitude_scale=self.amplitude_scale,
            selected_amplitudes_si=(np.asarray(self.trajectory.amplitudes_si)*self.amplitude_scale).tolist(),
            harmonics=list(self.trajectory.harmonics), requested_period_s=self.trajectory.requested_period_s,
            selected_period_s=self.period_s, ramp_seconds=self.period_s*self.trajectory.ramp_fraction,
            sampled_pose_delta_min_mm_deg=(self.sampled_pose_delta_min*SCALE).tolist(),
            sampled_pose_delta_max_mm_deg=(self.sampled_pose_delta_max*SCALE).tolist(),
            path_min_steps=self.path_min_steps.tolist(), path_max_steps=self.path_max_steps.tolist(),
            path_range_mm=((self.path_max_steps-self.path_min_steps)*self.geometry.meters_per_step*1000).tolist(),
            envelope_min_steps=self.envelope_min_steps.tolist(), envelope_max_steps=self.envelope_max_steps.tolist(),
            mechanical_min_steps=self.lower_steps.tolist(), mechanical_max_steps=self.upper_steps.tolist(),
            per_axis_peak_reference_speed_mm_s=(self.per_axis_peak_speed_m_s*1000).tolist(),
            peak_joint_speed_m_s=self.peak_speed_m_s, speed_check_samples=self.sample_count,
            derivative_margin=1.02, phase_schedule=self.phase_schedule, settings=asdict(self.settings),
            coordinate_convention='mechanical XYZ meters / ZYX Euler radians')

    def console_line(self, row):
        target = (np.asarray(row['target_pose_m_rad'])-self.initial_pose)*SCALE
        actual = None if row['actual_pose_m_rad'] is None else np.round((np.asarray(row['actual_pose_m_rad'])-self.initial_pose)*SCALE, 3).tolist()
        return (f"[{row['phase']}] target ΔXYZmm/RPYdeg={np.round(target,3).tolist()} actual={actual}"
                f" max|e|={max(abs(e) for e in row['position_error_steps']):.2f} steps"
                f" Imax={max(abs(v) for v in row['currents_ma'])} mA")


def read_only_precheck(output_dir, trajectory=None, *, port='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'):
    """No ServoCore, mode selection, force command, Tare or Pause on this path."""
    import serial
    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=False)
    report = dict(source='REAL_HARDWARE_READ_ONLY', timestamp=datetime.now().astimezone().isoformat(),
                  port=port, hardware_write_count=0, passes=[], ready=False)
    ser = bus = lock = None
    try:
        if subprocess.run(['fuser', '-s', port], capture_output=True).returncode != 1:
            raise RuntimeError('serial port is owned or ownership check failed; no query sent')
        ser = serial.Serial(port, 921600, timeout=.05, write_timeout=.03, exclusive=True)
        lock = SerialPortLock(ser.fileno(), port=port, owner='combined_pose_read_only')
        lock.acquire()
        bus = UartBus(serial_instance=ser, timeout=.05, retries=0, read_only=True)
        device_limits = None
        for scan in range(2):
            axes = []
            for mid in range(1, 7):
                motor = Motor(bus, mid)
                status = motor.read_status()
                rx_ns = time.monotonic_ns()
                mode = motor.read_register(0x25).values[0]
                lower = motor.read_register(0x24).values[0]
                upper = motor.read_register(0x23).values[0]
                axes.append(dict(motor_id=mid, mode=mode, lower_steps=lower, upper_steps=upper,
                                 status_rx_ns=rx_ns, **asdict(status)))
            report['passes'].append(axes)
            device_limits = [(a['lower_steps'], a['upper_steps']) for a in axes]
            if any(a['mode'] != 0 or a['error_code'] or abs(a['current_mA']) > 500 for a in axes):
                raise RuntimeError('read-only initial mode/current/fault check failed; no recovery writes attempted')
            if scan == 0:
                time.sleep(.1)
        # Planning uses the most recent actual status, never nominal HOME.
        plan = CombinedPosePlan([a['current_position'] for a in axes], trajectory=trajectory, device_limits=device_limits)
        report.update(ready=True, plan=plan.summary())
    except Exception as exc:
        report['error'] = f'{type(exc).__name__}: {exc}'
    finally:
        if bus is not None:
            report.update(hardware_write_count=len(bus.write_audit),
                          tx_command_counts={hex(k): v for k, v in bus.tx_cmd_counts.items()})
            bus.close()
        elif ser is not None:
            ser.close()
        if lock is not None:
            lock.release()
        report['serial_closed'] = ser is not None and not ser.is_open
        (out/'precheck.json').write_text(json.dumps(report, indent=2)+'\n')
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--hardware', action='store_true')
    mode.add_argument('--read-only', action='store_true')
    mode.add_argument('--dry-run', action='store_true')
    parser.add_argument('--hardware-qualified', action='store_true')
    parser.add_argument('--period-seconds', type=float, default=180.)
    parser.add_argument('--amplitudes-mm', nargs=3, type=float, default=[2., 2., 2.])
    parser.add_argument('--amplitudes-deg', nargs=3, type=float, default=[1., 1., 2.])
    parser.add_argument('--initial-steps', nargs=6, type=float, default=[1000.]*6, help='offline input only')
    parser.add_argument('--port', default='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0')
    parser.add_argument('--output-dir')
    args = parser.parse_args(argv)
    if args.hardware and not args.hardware_qualified:
        parser.error('--hardware requires explicit --hardware-qualified authorization')
    try:
        trajectory = CombinedTrajectoryConfig(amplitudes_si=tuple(np.asarray(args.amplitudes_mm)/1000.)
            + tuple(np.deg2rad(args.amplitudes_deg)), requested_period_s=args.period_seconds)
    except ValueError as exc:
        parser.error(str(exc))
    root = Path(__file__).resolve().parents[4]/'results/mode3_combined_pose'
    prefix = 'precheck_' if args.read_only else 'run_'
    out = Path(args.output_dir) if args.output_dir else root/(prefix+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    if args.read_only:
        report = read_only_precheck(out, trajectory, port=args.port)
        print(json.dumps(report, indent=2));print('RESULT_DIRECTORY:', out, flush=True)
        return 0 if report['ready'] else 2
    if not args.hardware:
        plan = CombinedPosePlan(args.initial_steps, trajectory=trajectory)
        print(json.dumps(dict(source='OFFLINE', hardware_write_count=0, plan=plan.summary()), indent=2))
        return 0
    out.mkdir(parents=True, exist_ok=False)
    print('RESULT_DIRECTORY:', out, flush=True)
    config = dict(epoch=int(time.time()), hardware_mode='active', port=args.port, baudrate=921600,
        output_dir=str(out), report_filename='combined_report.json', execute=True, dry_run=False,
        hardware_write_enabled=True, is_hardware_qualified=True, control_mode='force', maintenance_only=True,
        force_fault_recovery='position_reanchor', max_force_handover_displacement_steps=6)
    report = run(config, plan_factory=partial(CombinedPosePlan, trajectory=trajectory))
    # Analysis/plotting happens only after the existing runner has recovered
    # Mode 0 and closed the port (including fault/stop exits).
    from .combined_pose_analysis import analyze_combined_run
    try:
        analyze_combined_run(out)
    except Exception as exc:
        (out/'analysis_error.txt').write_text(f'{type(exc).__name__}: {exc}\n')
        print('ANALYSIS_ERROR:', str(exc), flush=True)
        return 2
    print('OUTCOME:', report['outcome'], 'MODE0:', report.get('all_mode0_exit_confirmed'), flush=True)
    return 0 if report['outcome'] in ('motion_complete', 'operator_stopped') and report.get('all_mode0_exit_confirmed') else 2
