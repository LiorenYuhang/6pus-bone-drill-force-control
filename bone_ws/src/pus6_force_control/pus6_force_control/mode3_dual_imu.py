"""Dual IMU -> retained Position SO(3)/LADRC -> IK -> proven Mode 3 PI-D.

Default: synthetic orientations + static MockServoBus, no real serial opened.
"""
import argparse
from dataclasses import asdict, dataclass
from typing import ClassVar
from pus6_motor_driver.safety import MAX_CURRENT_MA
from datetime import datetime
from functools import partial
import json
from pathlib import Path
import signal
import subprocess
import threading
import time

import numpy as np
from scipy.spatial.transform import Rotation

from pus6_kinematics.analytical_6pus import Analytical6PusKinematics
from pus6_kinematics.kinematics_interface import KinematicsError
from .dual_imu_attitude import AXES, DualQuaternionBuffer, PositionLadrcAttitude, load_attitude_parameters
from .dual_imu_force_policy import dual_imu_force_limits, recover_holding_bias
from .force_pipeline import raw_steps_to_slider_m
from .z_position_motion import MotionConfig, ZPlan, run, slider_to_steps


# Use the existing shared driver/geometry baseline, without another inner box.
ACTUAL_STOP_RESERVE_STEPS = 0.
TARGET_OVERSHOOT_MARGIN_STEPS = 0.


@dataclass(frozen=True)
class DualImuMotionConfig(MotionConfig):
    CURRENT_CEILING_MA: ClassVar[int] = MAX_CURRENT_MA
    max_current_ma: int = MAX_CURRENT_MA


class DualImuPlan(ZPlan):
    """Online attitude target, fixed captured XYZ and unchanged joint safeguards."""
    pid_overrides = dict(joint_kd_n_s_m=(1000.,)*6, joint_continuous_velocity=True,
        joint_integrate_while_moving=True, joint_integral_from_measured_error=True, joint_velocity_window_s=.08, joint_derivative_filter_tau_s=.04)
    joint_reference_speed_m_s = None  # bypass position reference rate limiter, not a large substitute speed
    capture_fault_snapshot = True
    force_limits = staticmethod(dual_imu_force_limits)
    recover_holding_bias = staticmethod(recover_holding_bias)
    def __init__(self, initial_steps, settings=None, *, source, parameters, axes=AXES,
                 duration_s=60., kinematics=None, device_limits=None, clock=time.monotonic_ns):
        self.settings = settings or DualImuMotionConfig()
        self.kinematics = kinematics or Analytical6PusKinematics()
        self.geometry = g = self.kinematics.geometry_config
        self.initial_q = raw_steps_to_slider_m(initial_steps, g)
        fk = self.kinematics.forward_kinematics_with_metrics(self.initial_q)
        if not fk.converged or not np.isfinite(fk.pose).all():
            raise ValueError('initial measured-position FK failed')
        self.initial_pose = np.asarray(fk.pose)
        self.ik_start = np.asarray(self.kinematics.inverse_kinematics(self.initial_pose))
        self.lower_steps = np.full(6, max(g.normal_motion_min_steps, g.absolute_raw_min_steps), dtype=float)
        self.upper_steps = np.full(6, min(g.normal_motion_max_steps, g.absolute_raw_max_steps, g.software_command_max_steps), dtype=float)
        if device_limits is not None:
            bounds = np.asarray(device_limits, dtype=float)
            if bounds.shape != (6, 2) or not np.isfinite(bounds).all() or (bounds[:, 0]>=bounds[:, 1]).any():
                raise ValueError('six valid device stroke limits required')
            self.lower_steps = np.maximum(self.lower_steps, bounds[:, 0])
            self.upper_steps = np.minimum(self.upper_steps, bounds[:, 1])
        if (np.asarray(initial_steps)<self.lower_steps).any() or (np.asarray(initial_steps)>self.upper_steps).any():
            raise ValueError('initial actual position outside mechanical stroke')
        # Independent absolute workspaces: never a +/-100-step box at the start.
        self.envelope_min_steps = self.lower_steps + ACTUAL_STOP_RESERVE_STEPS
        self.envelope_max_steps = self.upper_steps - ACTUAL_STOP_RESERVE_STEPS
        self.target_min_steps = self.envelope_min_steps + TARGET_OVERSHOOT_MARGIN_STEPS
        self.target_max_steps = self.envelope_max_steps - TARGET_OVERSHOOT_MARGIN_STEPS
        if (self.target_min_steps >= self.target_max_steps).any():
            raise ValueError('DUAL_IMU_WORKSPACE_EMPTY: device stroke cannot retain stopping/target margins')
        self.target_checks = self.unreachable_targets = 0
        self.clock = clock; self.source = source
        self.controller = PositionLadrcAttitude(parameters, axes)
        sample = source.snapshot(clock())
        self.controller.capture(self.initial_pose, sample)
        self.control_diagnostics = self.controller.diagnostics
        self.last_pose = self.initial_pose.copy()
        self.target_request = None
        # The runner starts this clock after confirmed Mode 3. Each target()
        # still validates both IMUs; encoder/FK settling is not a startup gate.
        # One PID instance spans these phases, carrying its I/velocity state.
        self.phase_schedule = [('initial_hold', 1.), ('stabilize', duration_s), ('final_hold', None)]

    def check_positions(self, steps):
        """Actual feedback protection, distinct from reference feasibility."""
        q = np.asarray(steps, dtype=float)
        if q.shape != (6,) or not np.isfinite(q).all():
            raise RuntimeError('ACTUAL_POSITION_INVALID: six finite feedback values required')
        self.control_diagnostics['actual_workspace'] = dict(
            positions_steps=q.tolist(), mechanical_remaining_lower_steps=(q-self.lower_steps).tolist(),
            mechanical_remaining_upper_steps=(self.upper_steps-q).tolist(),
            fault_remaining_lower_steps=(q-self.envelope_min_steps).tolist(),
            fault_remaining_upper_steps=(self.envelope_max_steps-q).tolist())
        for lower, upper, reason in ((self.lower_steps, self.upper_steps, 'ACTUAL_POSITION_MECHANICAL_BOUND'),):
            bad = (q < lower) | (q > upper)
            if bad.any():
                raise RuntimeError(reason+': '+json.dumps(dict(motor_ids=(np.flatnonzero(bad)+1).tolist(),
                    actual_steps=q.tolist(), lower_steps=lower.tolist(), upper_steps=upper.tolist())))

    def constrain_pose(self, requested_pose):
        """Exact IK or an explicit pre-write fault. Never scale an unreachable target."""
        raw = np.asarray(requested_pose, dtype=float)
        if raw.shape != (6,) or not np.isfinite(raw).all():
            raise ValueError('IK_TARGET_INVALID: six finite pose values required')
        raw = raw.copy(); raw[:3] = self.initial_pose[:3]
        raw[3:] = np.where(self.controller.mask, raw[3:], self.initial_pose[3:])
        q = steps = None
        reason = 'NONE'
        try:
            q = self.initial_q+np.asarray(self.kinematics.inverse_kinematics(raw))-self.ik_start
            steps = slider_to_steps(q, self.geometry)
            if not np.isfinite(steps).all():
                raise ValueError('nonfinite IK')
            if ((steps < self.target_min_steps) | (steps > self.target_max_steps)).any():
                reason = 'IK_TARGET_WORKSPACE_BOUND'
        except (KinematicsError, ValueError):
            q = steps = None
            reason = 'IK_TARGET_KINEMATIC_UNREACHABLE'
        reachable = reason == 'NONE'
        diagnostic = dict(target_reachable=reachable, kinematic_reachable=steps is not None,
            saturated=not reachable, ik_saturated=not reachable, reason=reason,
            action='direct_exact_ik' if reachable else 'reject_before_force_write',
            rotation_scale=1. if reachable else None, requested_pose_m_rad=raw.tolist(),
            bounded_pose_m_rad=raw.tolist() if reachable else None,
            requested_positions_steps=None if steps is None else steps.tolist(),
            bounded_positions_steps=steps.tolist() if reachable else None,
            lower_steps=self.target_min_steps.tolist(), upper_steps=self.target_max_steps.tolist(),
            mechanical_remaining_lower_steps=None if steps is None else (steps-self.lower_steps).tolist(),
            mechanical_remaining_upper_steps=None if steps is None else (self.upper_steps-steps).tolist(),
            target_remaining_lower_steps=None if steps is None else (steps-self.target_min_steps).tolist(),
            target_remaining_upper_steps=None if steps is None else (self.target_max_steps-steps).tolist())
        self.target_checks += 1
        self.unreachable_targets += int(not reachable)
        self.target_request = dict(self.target_request or {}, **diagnostic)
        self.control_diagnostics = dict(self.control_diagnostics, target_constraint=diagnostic)
        if not reachable:
            raise RuntimeError(reason+': '+json.dumps(diagnostic))
        return raw, q, diagnostic

    def workspace_result(self, outcome):
        return dict(target_checks=self.target_checks, unreachable_targets=self.unreachable_targets,
            full_target_motion_completed=outcome == 'motion_complete' and self.unreachable_targets == 0,
            stability_verdict='NOT_EVALUATED_BY_RUNTIME')

    def target(self, phase, elapsed_s):
        self.target_request = dict(ns=self.clock(), phase=phase, phase_elapsed_s=elapsed_s)
        sample = self.source.snapshot(self.clock())
        if phase == 'initial_hold':
            # Establish the existing joint hold first. No undispatched outer
            # residual is fed to the observer during this startup dwell.
            self.controller.last_ns = sample['sample_ns']
            self.control_diagnostics = dict(status='initial_hold', imu=sample, observer_updated=False)
            pose = self.initial_pose.copy()
        elif phase == 'stabilize':
            if hasattr(self.source, 'activate'):
                self.source.activate(elapsed_s)
                sample = self.source.snapshot(self.clock())
            pose, self.control_diagnostics = self.controller.update(sample)
        else:
            # Finish holding the last dispatched attitude, never jump back HOME
            # while the operator may still be tilting the base.
            pose = self.last_pose.copy()
            self.control_diagnostics = dict(self.controller.diagnostics, status='final_hold', imu=sample, observer_updated=False)
        self.target_request['requested_pose_m_rad'] = pose.tolist()
        pose, q, constraint = self.constrain_pose(pose)
        self.target_request.update(constraint)
        self.control_diagnostics = dict(self.control_diagnostics, target_constraint=constraint)
        if phase == 'stabilize': self.last_pose = pose.copy()
        return pose, q

    def summary(self):
        return dict(trajectory='dual_imu_so3_ladrc_attitude', initial_pose_m_rad=self.initial_pose.tolist(),
            initial_steps=slider_to_steps(self.initial_q, self.geometry).tolist(), axes=list(self.controller.axes),
            phase_schedule=self.phase_schedule, settings=asdict(self.settings),
            mechanical_min_steps=self.lower_steps.tolist(), mechanical_max_steps=self.upper_steps.tolist(),
            envelope_min_steps=self.envelope_min_steps.tolist(), envelope_max_steps=self.envelope_max_steps.tolist(),
            target_min_steps=self.target_min_steps.tolist(), target_max_steps=self.target_max_steps.tolist(),
            shared_current_limit_ma=MAX_CURRENT_MA, pid_overrides=self.pid_overrides,
            actual_stop_reserve_steps=ACTUAL_STOP_RESERVE_STEPS,
            target_overshoot_margin_steps=TARGET_OVERSHOOT_MARGIN_STEPS,
            legacy_initial_envelope_margin_ignored=True,
            attitude_parameters=self.controller.parameters, imu_reference=self.controller.reference,
            initial_hold_policy='fixed_1s_after_confirmed_mode3_and_valid_imu',
            pid_phase_handover='carry_integral_and_velocity_state_without_reset',
            imu_source=self.source.kind, prior_input_semantics='previous dispatched angular residual; not measured force or achieved pose',
            position_reference_mode='direct_latest_bounded_ik', joint_max_reference_speed_m_s=None,
            normal_force_max_rate_g_s=None, exceptional_recovery_rate_g_s=[5000.]*6,
            normal_pid_slew_delay_compensation=False,
            target_constraint='exact IK in absolute target workspace; unreachable request faults before force write; no scaling',
            translation_reference='captured actual FK XYZ, held constant', final_hold_reference='last dispatched target attitude')

    def console_plan(self):
        initial = np.rint(slider_to_steps(self.initial_q, self.geometry)).astype(int).tolist()
        return (f"EXPERIMENT: axes={','.join(self.controller.axes)} duration={self.phase_schedule[1][1]:g}s\n"
                f"INITIAL_POSITIONS_STEPS: {initial}")

    def console_line(self, row):
        actual = None if row['actual_pose_m_rad'] is None else np.rad2deg(row['actual_pose_m_rad'][3:]).round(3).tolist()
        return (f"[{row['phase']} {row['phase_elapsed_s']:.1f}s] targetRPYdeg={np.rad2deg(row['target_pose_m_rad'][3:]).round(3).tolist()}"
                f" FKdeg={actual}"
                f" max|e|={max(abs(x) for x in row['position_error_steps']):.2f}steps"
                f" Imax={max(abs(x) for x in row['currents_ma'])}mA")


class SyntheticImuSource:
    """Closed smooth three-axis input; no claim to simulate physical dynamics."""
    kind = 'SYNTHETIC_IMU'
    def __init__(self, duration_s=6., amplitude_deg=.2):
        self.duration_s = duration_s; self.amplitude_rad = np.deg2rad(amplitude_deg)
        self.elapsed_s = None

    def activate(self, elapsed_s):
        self.elapsed_s = elapsed_s

    def snapshot(self, now_ns):
        angles = np.zeros(3)
        if self.elapsed_s is not None:
            u = np.clip(self.elapsed_s/self.duration_s, 0., 1.)
            angles = self.amplitude_rad*np.sin(np.pi*u)**2*np.sin(2*np.pi*np.array([1, 2, 3])*u)
        # Generate raw H30 quaternions through the inverse retained mount map.
        c = Rotation.from_euler('z', 90, degrees=True).as_matrix()
        raw = Rotation.from_matrix(c.T@Rotation.from_rotvec(angles).as_matrix()@c).as_quat().tolist()
        return dict(sample_ns=int(now_ns), now_ns=int(now_ns), base_q_xyzw=raw, platform_q_xyzw=raw,
                    time_basis='synthetic_host_monotonic')

    def close(self):
        pass


class RosDualImuSource:
    kind = 'ROS_H30_DUAL_IMU'
    def __init__(self, parameters, *, start_imus=False, output_dir=None):
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from rclpy.signals import SignalHandlerOptions
        from sensor_msgs.msg import Imu
        self.buffer = DualQuaternionBuffer(min(parameters['max_base_imu_age_s'], parameters['max_platform_imu_age_s']))
        self.processes = []; self.logs = []; self.stop = threading.Event()
        self.context = Context()
        rclpy.init(args=[], context=self.context, signal_handler_options=SignalHandlerOptions.NO)
        self.node = Node('mode3_dual_imu_input', context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context); self.executor.add_node(self.node)
        for name in ('base', 'platform'):
            self.node.create_subscription(Imu, '/pus6/imu_'+name+'/data',
                lambda msg, name=name: self.receive(name, msg), qos_profile_sensor_data)
        self.thread = threading.Thread(target=self.spin, daemon=True); self.thread.start()
        try:
            if start_imus:
                self.start_sensors(output_dir)
            deadline = time.monotonic()+10.
            while True:
                try:
                    self.snapshot(time.monotonic_ns()); break
                except ValueError:
                    if time.monotonic() >= deadline:
                        raise RuntimeError('fresh synchronized dual IMU orientations unavailable; motor owner not started')
                    time.sleep(.02)
            for name in ('base', 'platform'):
                if self.node.count_publishers('/pus6/imu_'+name+'/data') != 1:
                    raise RuntimeError(name+' IMU requires one unambiguous publisher')
        except BaseException:
            self.close(); raise

    def receive(self, name, msg):
        # Partial H30 gyro/acceleration publications do not refresh orientation.
        if msg.orientation_covariance[0] < 0:
            return
        try:
            q = msg.orientation
            self.buffer.push(name, [q.x, q.y, q.z, q.w], time.monotonic_ns(),
                msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec)
        except Exception as exc:
            self.buffer.error = f'{name} IMU invalid: {exc}'

    def spin(self):
        try:
            while not self.stop.is_set():
                self.executor.spin_once(timeout_sec=.02)
        except Exception as exc:
            self.buffer.error = 'IMU executor failed: '+str(exc)

    def start_sensors(self, output_dir):
        from ros2run.api import get_executable_path
        executable = get_executable_path(package_name='h30_imu', executable_name='h30_imu_node')
        if executable is None:
            raise RuntimeError('installed H30 executable unavailable')
        ports = dict(base='/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B32028000-if00',
                     platform='/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B0B029000-if00')
        time.sleep(.5)  # bounded DDS discovery before considering sensor startup
        for name, port in ports.items():
            publishers = self.node.count_publishers('/pus6/imu_'+name+'/data')
            if publishers > 1:
                raise RuntimeError(name+' IMU has multiple publishers')
            if publishers == 1:
                continue
            if subprocess.run(['fuser', '-s', port], capture_output=True).returncode != 1:
                raise RuntimeError(name+' IMU port occupied; no duplicate sensor started')
            log = (Path(output_dir)/(name+'_imu.log')).open('x'); self.logs.append(log)
            self.processes.append(subprocess.Popen([executable, '--ros-args', '-r', '__ns:=/pus6/imu_'+name,
                '-p', 'port:='+port, '-p', 'baud_rate:=460800', '-p', 'frame_id:=pus6_'+name+'_imu_link',
                '-p', 'world_frame:=world', '-p', 'imu_topic:=/pus6/imu_'+name+'/data'],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True))

    def snapshot(self, now_ns):
        return self.buffer.snapshot(now_ns)

    def close(self):
        self.stop.set(); self.thread.join(timeout=1.)
        self.executor.shutdown(timeout_sec=1.); self.node.destroy_node(); self.context.shutdown()
        for process in self.processes:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try: process.wait(timeout=2.)
                except subprocess.TimeoutExpired:
                    process.terminate(); process.wait(timeout=2.)
        for log in self.logs: log.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--hardware', action='store_true'); group.add_argument('--dry-run', action='store_true')
    group.add_argument('--validate-workspace', metavar='REFERENCE_RUN', help='offline maximum three-axis workspace and historical trajectory validation; no ROS or UART')
    group.add_argument('--analyze-history', metavar='RUN', help='offline measured-run analysis; requires a new --output-dir')
    group.add_argument('--replay-history', help='offline historical dual-IMU run directory; no ROS or UART')
    parser.add_argument('--replay-runtime-schedule', action='store_true', help='offline replay with the current 1s startup and active phase, rather than recorded phase labels')
    parser.add_argument('--three-process', action='store_true', help='J14 Parent / UART Child / Feedback Bridge with Child-local joint PI-D')
    parser.add_argument('--hardware-qualified', action='store_true')
    parser.add_argument('--imu-source', choices=['synthetic', 'ros'])
    parser.add_argument('--start-imus', action='store_true', help='start only missing H30 publishers; never a Position motor node')
    parser.add_argument('--axes', default='roll,pitch,yaw')
    parser.add_argument('--duration-seconds', type=float)
    parser.add_argument('--parameters')
    parser.add_argument('--output-dir')
    parser.add_argument('--initial-steps', nargs=6, type=float, default=[1000.]*6, help='Mock only')
    parser.add_argument('--port', default='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0')
    args = parser.parse_args(argv)
    if args.replay_runtime_schedule and not args.replay_history:
        parser.error('--replay-runtime-schedule requires --replay-history')
    if args.analyze_history:
        if not args.output_dir or args.start_imus or args.imu_source is not None:
            parser.error('offline analysis needs a new --output-dir and cannot start/subscribe to IMUs')
        from .dual_imu_analysis import analyze_run
        print(json.dumps(analyze_run(args.analyze_history, args.output_dir), indent=2))
        return 0
    if args.validate_workspace:
        if args.start_imus or args.imu_source is not None:
            parser.error('offline validation cannot start or subscribe to IMUs')
        from .dual_imu_workspace_validation import validate_workspace
        validation_out = (Path(args.output_dir) if args.output_dir else
            Path(__file__).resolve().parents[4]/'results/mode3_dual_imu'/('workspace_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f')))
        result = validate_workspace(args.validate_workspace, validation_out)
        print(json.dumps(result, indent=2)); print('RESULT_DIRECTORY:', validation_out, flush=True)
        return 0 if result['validation_completed'] else 2
    if args.replay_history:
        if args.start_imus or args.imu_source is not None:
            parser.error('offline replay cannot start or subscribe to IMUs')
        from .dual_imu_replay import replay_history
        replay_out = (Path(args.output_dir) if args.output_dir else
            Path(__file__).resolve().parents[4]/'results/mode3_dual_imu'/('replay_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f')))
        result = replay_history(args.replay_history, replay_out, runtime_schedule=args.replay_runtime_schedule)
        print(json.dumps(result, indent=2)); print('RESULT_DIRECTORY:', replay_out, flush=True)
        return 0 if result['fault'] is None else 2
    axes = tuple(args.axes.lower().split(','))
    if not axes or any(a not in AXES for a in axes) or len(set(axes)) != len(axes):
        parser.error('select unique axes: roll,pitch,yaw')
    duration = args.duration_seconds if args.duration_seconds is not None else (60. if args.hardware else 6.)
    if not np.isfinite(duration) or duration <= 0:
        parser.error('positive finite duration required')
    if args.hardware and (not args.hardware_qualified or args.imu_source == 'synthetic'):
        parser.error('hardware requires --hardware-qualified and real ROS IMUs')
    source_name = args.imu_source or ('ros' if args.hardware else 'synthetic')
    if args.start_imus and source_name != 'ros':
        parser.error('--start-imus requires --imu-source ros or --hardware')
    root = Path(__file__).resolve().parents[4]/'results/mode3_dual_imu'
    out = Path(args.output_dir) if args.output_dir else root/(('run_' if args.hardware else 'dry_run_')+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    parameters = load_attitude_parameters(args.parameters)
    # Validate retained controller parameters before starting sensors or motors.
    PositionLadrcAttitude(parameters, axes)
    out.mkdir(parents=True, exist_ok=False)
    print('RESULT_DIRECTORY:', out, flush=True)
    source = None
    motor_runner_started = False
    try:
        source = (SyntheticImuSource(duration) if source_name == 'synthetic' else
            RosDualImuSource(parameters, start_imus=args.start_imus, output_dir=out))
        config = dict(epoch=int(time.time()), hardware_mode='active' if args.hardware else 'mock',
            port=args.port, baudrate=921600, output_dir=str(out), report_filename='dual_imu_report.json',
            execute=True, dry_run=False, hardware_write_enabled=True, is_hardware_qualified=True,
            control_mode='force', maintenance_only=True, force_fault_recovery='position_reanchor',
            max_force_handover_displacement_steps=6)
        bus = None
        if not args.hardware:
            from pus6_motor_driver.fake_bus import MockServoBus
            bus = MockServoBus(initial_positions=[int(x) for x in args.initial_steps], initial_modes=[0]*6)
        factory = partial(DualImuPlan, source=source, parameters=parameters, axes=axes, duration_s=duration)
        motor_runner_started = True
        if args.three_process:
            from .mode3_process_runtime import run as run_three_process
            report = run_three_process(dict(config,mock_initial_positions=[int(x) for x in args.initial_steps]),DualImuMotionConfig(),source=source,
                parameters=parameters,axes=axes,duration=duration)
        else:
            report = run(config, DualImuMotionConfig(), bus=bus, plan_factory=factory)
        source.close(); source = None  # all analysis runs after motor and sensor closure
        from .dual_imu_analysis import analyze_run
        try:
            analyze_run(out, out/'analysis')
        except Exception as exc:
            (out/'analysis_error.json').write_text(json.dumps(dict(error=str(exc)), indent=2)+'\n')
            print('ANALYSIS_ERROR:', str(exc), flush=True)
        reason = report.get('error') or report.get('exit_error') or report.get('close_error') or 'none'
        # Structured exception details remain in the saved report/snapshot.
        print('OUTCOME:', report['outcome'], 'ERROR:', reason.split(': {', 1)[0],
            'MODE0:', report.get('all_mode0_exit_confirmed'),
            'hardware_write_count:', report['hardware_write_count'], flush=True)
        return 0 if report['outcome'] == 'motion_complete' and report.get('all_mode0_exit_confirmed') else 2
    except Exception as exc:
        (out/'startup_error.json').write_text(json.dumps(dict(error=str(exc), motor_runner_started=motor_runner_started), indent=2)+'\n')
        print('OUTCOME: startup_error ERROR:', str(exc).split(': {', 1)[0], flush=True); return 2
    finally:
        if source is not None:
            source.close()
