"""Measured-start Cartesian Z trajectory -> IK -> joint PI-D -> native Mode 3.

No task-space wrench allocation. Offline planning never opens a serial port.
Hardware execution uses the existing UART owner, authorization and recovery.
"""
from dataclasses import asdict, dataclass, replace
from typing import ClassVar
from datetime import datetime
import argparse
import json
from pathlib import Path
import signal
import time

import numpy as np

from pus6_control.joint_trajectory_controller import quintic_progress
from pus6_kinematics.analytical_6pus import Analytical6PusKinematics
from pus6_motor_driver.force_units import NOMINAL_N_PER_G, g_to_n
from pus6_motor_driver.j14_process_uart import ServoCore
from pus6_motor_driver.motor import Motor
from .force_pipeline import raw_steps_to_slider_m
from .joint_position_force_pid import JointForcePIDConfig, JointPositionForcePID
from .six_axis_force_drive import ForceLimitsConfig, SixAxisForceDrive
from .six_axis_tare_experiment import SerialTrace


# Motion-reference setting for the requested 100 s, 10 mm quintic segments.
# Shared by planning and the existing PID reference limiter; gains and physical
# current/force/stroke/tracking-error protections are unchanged.
MAX_REFERENCE_SPEED_M_S = .0002


@dataclass(frozen=True)
class MotionConfig:
    CURRENT_CEILING_MA: ClassVar[int] = 500
    rise_m: float = .010
    move_seconds: float = 100.
    hold_seconds: float = 3.
    settle_timeout_s: float = 20.
    settle_seconds: float = 1.
    tolerance_steps: float = 2.
    z_tolerance_m: float = .0001
    velocity_tolerance_m_s: float = .00005
    tracking_error_m: float = .0025
    envelope_margin_steps: float = 100.
    max_current_ma: int = 500

    def __post_init__(self):
        if any(not np.isfinite(v) or v <= 0 for v in asdict(self).values()):
            raise ValueError('motion settings must be finite and positive')
        if self.max_current_ma > self.CURRENT_CEILING_MA:
            raise ValueError(f'this experiment retains the {self.CURRENT_CEILING_MA} mA current ceiling')
        if self.hold_seconds + self.settle_seconds >= self.settle_timeout_s:
            raise ValueError('settle timeout must allow settling and hold')


def slider_to_steps(q, geometry):
    return (np.asarray(geometry.operating_home_raw_steps)
            + (np.asarray(q) - geometry.operating_home_stroke_m) / geometry.meters_per_step)


class ZPlan:
    def __init__(self, initial_steps, settings=None, *, kinematics=None, device_limits=None):
        self.settings = settings or MotionConfig()
        self.kinematics = kinematics or Analytical6PusKinematics()
        self.geometry = self.kinematics.geometry_config
        self.initial_q = raw_steps_to_slider_m(initial_steps, self.geometry)
        fk = self.kinematics.forward_kinematics_with_metrics(self.initial_q)
        if not fk.converged or not np.isfinite(fk.pose).all():
            raise ValueError('initial measured-position FK failed')
        self.initial_pose = np.asarray(fk.pose)
        # Differential IK preserves the measured start despite FK solver residual.
        self.ik_start = np.asarray(self.kinematics.inverse_kinematics(self.initial_pose))
        g = self.geometry
        low = max(g.normal_motion_min_steps, g.absolute_raw_min_steps)
        high = min(g.normal_motion_max_steps, g.absolute_raw_max_steps, g.software_command_max_steps)
        self.lower_steps = np.full(6, low, dtype=float)
        self.upper_steps = np.full(6, high, dtype=float)
        if device_limits is not None:
            limits = np.asarray(device_limits, dtype=float)
            if limits.shape != (6, 2) or not np.isfinite(limits).all() or (limits[:, 0] >= limits[:, 1]).any():
                raise ValueError('six valid device stroke limits required')
            self.lower_steps = np.maximum(self.lower_steps, limits[:, 0])
            self.upper_steps = np.minimum(self.upper_steps, limits[:, 1])
        # Evaluate the complete ascending path; return traverses the same path.
        path = np.array([slider_to_steps(self.at_fraction(f)[1], g) for f in np.linspace(0., 1., 201)])
        if (path < self.lower_steps - 1e-7).any() or (path > self.upper_steps + 1e-7).any():
            raise ValueError('IK trajectory exceeds normal/device mechanical stroke')
        self.path_min_steps = path.min(axis=0)
        self.path_max_steps = path.max(axis=0)
        self.envelope_min_steps = np.maximum(self.lower_steps, self.path_min_steps - self.settings.envelope_margin_steps)
        self.envelope_max_steps = np.minimum(self.upper_steps, self.path_max_steps + self.settings.envelope_margin_steps)
        # For fixed X/Y/RPY the analytical vertical-slider IK is affine in Z.
        # The quintic peak speed is 1.875 * excursion / duration.
        self.peak_speed_m_s = float(np.max(np.abs(self.at_fraction(1.)[1] - self.initial_q)) * 1.875 / self.settings.move_seconds)
        if self.peak_speed_m_s > MAX_REFERENCE_SPEED_M_S + 1e-12:
            raise ValueError('trajectory exceeds the 0.2 mm/s joint reference limit; increase move-seconds')

    def at_fraction(self, fraction):
        pose = self.initial_pose.copy()
        pose[2] += self.settings.rise_m * float(np.clip(fraction, 0., 1.))
        q = self.initial_q + np.asarray(self.kinematics.inverse_kinematics(pose)) - self.ik_start
        return pose, q

    def target(self, phase, elapsed_s):
        if phase == 'rise':
            fraction = quintic_progress(elapsed_s / self.settings.move_seconds)
        elif phase == 'return':
            fraction = 1. - quintic_progress(elapsed_s / self.settings.move_seconds)
        else:
            fraction = 1. if phase == 'top_hold' else 0.
        return self.at_fraction(fraction)

    def summary(self):
        return dict(initial_pose_m_rad=self.initial_pose.tolist(),
                    initial_steps=slider_to_steps(self.initial_q, self.geometry).tolist(),
                    top_steps=slider_to_steps(self.at_fraction(1.)[1], self.geometry).tolist(),
                    excursion_steps=((self.at_fraction(1.)[1] - self.initial_q) / self.geometry.meters_per_step).tolist(),
                    path_min_steps=self.path_min_steps.tolist(), path_max_steps=self.path_max_steps.tolist(),
                    envelope_min_steps=self.envelope_min_steps.tolist(), envelope_max_steps=self.envelope_max_steps.tolist(),
                    mechanical_min_steps=self.lower_steps.tolist(), mechanical_max_steps=self.upper_steps.tolist(),
                    peak_joint_speed_m_s=self.peak_speed_m_s, settings=asdict(self.settings))

    def check_positions(self, steps):
        steps = np.asarray(steps)
        if steps.shape != (6,) or not np.isfinite(steps).all():
            raise RuntimeError('invalid six-axis position feedback')
        if (steps < self.envelope_min_steps).any() or (steps > self.envelope_max_steps).any():
            raise RuntimeError('actual position outside IK trajectory envelope/mechanical stroke')


def pid_config(plan):
    g = plan.geometry
    config = JointForcePIDConfig(
        joint_kp_n_m=(8000.,)*6, joint_ki_n_m_s=(6000.,)*6, joint_kd_n_s_m=(5000.,)*6,
        joint_integral_limit_n=(2.5,)*6, joint_increment_limit_n=(4.,)*6,
        joint_min_position_m=tuple(raw_steps_to_slider_m(plan.lower_steps, g)),
        joint_max_position_m=tuple(raw_steps_to_slider_m(plan.upper_steps, g)),
        joint_max_reference_speed_m_s=getattr(plan, 'joint_reference_speed_m_s', MAX_REFERENCE_SPEED_M_S),
        joint_max_tracking_error_m=plan.settings.tracking_error_m,
        joint_position_deadband_m=g.meters_per_step, joint_use_latest_feedback=True,
        joint_active_braking=True)
    return replace(config, **getattr(plan, 'pid_overrides', {}))


def hold_sample_settled(position_error_steps, velocity_m_s, actual_z_m, target_z_m, settings):
    # IK/meters -> steps can represent an exact 2-step error as
    # 2.0000000000004547. Tolerate only arithmetic roundoff (1e-9 steps),
    # preserving the physical position, velocity and Z tolerances.
    return (max(abs(x) for x in position_error_steps) <= settings.tolerance_steps + 1e-9
            and max(abs(x) for x in velocity_m_s) <= settings.velocity_tolerance_m_s
            and actual_z_m is not None and abs(actual_z_m-target_z_m) <= settings.z_tolerance_m)


def run(config, settings=None, *, bus=None, plan_factory=ZPlan):
    """Execute only with explicit permission. Injected buses are mock-only."""
    settings = settings or MotionConfig()
    if config.get('execute') is not True or config.get('dry_run') is not False:
        raise ValueError('run requires explicit execution; use ZPlan for offline planning')
    if not (config.get('hardware_write_enabled') and config.get('is_hardware_qualified')):
        raise ValueError('existing independent hardware execution permissions required')
    if config.get('hardware_mode') not in ('mock', 'active'):
        raise ValueError('invalid hardware_mode')
    if config['hardware_mode'] == 'active' and bus is not None:
        raise ValueError('active mode forbids injected mock bus')
    if not config.get('maintenance_only') or config.get('force_fault_recovery') != 'position_reanchor':
        raise ValueError('existing maintenance owner and position_reanchor recovery required')
    out = Path(config['output_dir'])
    out.mkdir(parents=True, exist_ok=True)
    report_file = out / config.get('report_filename', 'z_position_report.json')
    if report_file.exists() or (out / 'tracking.jsonl').exists():
        raise ValueError('output directory already contains a run; choose a new output directory')
    report = dict(source='REAL_HARDWARE' if config['hardware_mode'] == 'active' else 'MOCK',
                  outcome='started', hardware_write_count=0, control_cycles=0, feedback_cycles=0, stages=[],
                  actual_z_source='FK_FROM_SEQUENTIAL_ACTUATOR_FEEDBACK')
    core = trace = None
    plan = None
    pending_cycle = last_complete_frame = fault_snapshot = None
    stop = [False]
    handlers = {}
    kinematics = Analytical6PusKinematics()

    def record(name, row):
        with (out / name).open('a') as stream:
            stream.write(json.dumps(row) + '\n')

    def inventory():
        with core.writer.uart_lock:
            return [dict(mode=Motor(core.bus, i).read_register(0x25).values[0],
                         target_g=_signed(Motor(core.bus, i).read_register(0x27).values[0])) for i in range(1, 7)]

    def read_positions():
        statuses, stamps = {}, {}
        start = time.monotonic_ns()
        with core.writer.uart_lock:
            for i in range(1, 7):
                statuses[i] = Motor(core.bus, i).read_status()
                stamps[i] = time.monotonic_ns()
        return ingest(statuses, stamps, start)

    def ingest(statuses, stamps, start):
        # No stale cached axis may masquerade as fresh feedback.
        if set(statuses) != set(range(1, 7)) or set(stamps) != set(range(1, 7)):
            raise RuntimeError('incomplete six-axis status response')
        report['feedback_cycles'] += 1
        feedback = core.feedback.ingest_cycle_feedback(statuses,
            servo_cycle_id=report['feedback_cycles'], servo_cycle_timestamp_ns=start,
            per_axis_rx_done_ns=stamps)
        core.update(feedback)
        record('feedback.jsonl', dict(ns=time.monotonic_ns(), positions_steps=list(core.positions),
            force_2c_g=list(core.forces_g_raw), currents_ma=list(core.currents),
            fault_codes=list(core.faults), force_valid=list(core.force_valid), rx_ns=list(core.rx_ns)))
        if core.feedback.stopped or core.writer.stopped or not all(core.force_valid):
            raise RuntimeError('communication/status feedback stopped or invalid')
        if any(core.faults):
            raise RuntimeError('motor fault: ' + str(core.faults))
        if max(abs(x) for x in core.currents) > settings.max_current_ma:
            raise RuntimeError(f'{settings.max_current_ma} mA current protection exceeded')
        if max(abs(x) for x in core.forces_g_raw) > 1500:
            raise RuntimeError('force feedback exceeds existing 1500 g bound')
        now = time.monotonic_ns()
        if any(not 0 <= now - t <= 200_000_000 for t in core.rx_ns):
            raise RuntimeError('missing/stale actual position feedback')
        return now

    try:
        for sig in (signal.SIGINT, signal.SIGTERM):
            handlers[sig] = signal.signal(sig, lambda *_: stop.__setitem__(0, True))
        core = ServoCore(dict(config, max_current_ma=settings.max_current_ma), bus=bus)
        core.bus.timeout_s = .05
        if core.serial is not None:
            trace = SerialTrace(core.serial, out)
            core.serial = core.bus.serial = trace
        core.initialize()  # existing read-only Mode 0 startup
        read_positions()
        initial = list(core.positions)
        if [r['mode'] for r in inventory()] != [0]*6:
            raise RuntimeError('initial six-axis Mode 0 required')
        with core.writer.uart_lock:
            device_limits = [(Motor(core.bus, i).read_register(0x24).values[0],
                              Motor(core.bus, i).read_register(0x23).values[0]) for i in range(1, 7)]
        handover = core.prepare_force_handover()
        # Capture the actual start immediately before entry, not a historical HOME.
        read_positions()
        initial = list(core.positions)
        plan = plan_factory(initial, settings, kinematics=kinematics, device_limits=device_limits)
        report['plan'] = plan.summary()
        compact_console = hasattr(plan, 'console_plan')
        if compact_console:
            print(plan.console_plan(), flush=True)
        else:
            print('Z_POSITION_PLAN:', json.dumps(report['plan']), flush=True)
        hold = handover['holding_forces_g']
        report['hold_forces_g'] = list(hold)
        limits = plan.force_limits() if hasattr(plan, 'force_limits') else ForceLimitsConfig(brake_rate_g_s=(5000.,)*6)
        # This object only prepares bounded frames; ServoCore is the sole writer.
        drive = SixAxisForceDrive(core.bus, limits)
        drive.anchor_hold_forces_g(hold)

        def verify_entry(_mid):
            if stop[0]:
                raise InterruptedError('operator stopped during Mode 3 handover')
            read_positions()
            plan.check_positions(core.positions)
            if max(abs(a-b) for a, b in zip(core.positions, initial)) > 6:
                raise RuntimeError('Mode 3 handover displacement exceeds 6 steps')

        if stop[0]:
            raise InterruptedError('operator stopped before Mode 3 entry')
        core.enter_force_mode_with_handover(hold, static_baseline_g=hold, axis_verifier=verify_entry)
        modes = inventory()
        if [r['mode'] for r in modes] != [3]*6 or [r['target_g'] for r in modes] != list(hold):
            raise RuntimeError('Mode 3 / holding force readback mismatch')
        report['all_mode3_confirmed'] = True
        if compact_console:
            print('MODE3_CONFIRMED: six axes', flush=True)
        else:
            print('MODE3_CONFIRMED: six-axis position PI-D, holding bias g =', hold, flush=True)
        read_positions()
        pid = JointPositionForcePID(pid_config(plan), clock=time.monotonic_ns,
                                    force_slew_compensation=limits.max_rate_g_s is not None)
        now = time.monotonic_ns()
        q = raw_steps_to_slider_m(core.positions, plan.geometry)
        pid.reset(q, sample_ns=max(core.rx_ns), now_ns=now, position_stamps_ns=core.rx_ns)
        drive.anchor_hold_forces_g(hold, now_ns=now)
        read_positions()  # PID requires a new measured sample after reset.
        report['pid'] = asdict(pid.config)
        if hasattr(plan, 'force_limits'):
            report['force_policy'] = dict(normal_max_rate_g_s=limits.max_rate_g_s,
                exceptional_recovery_rate_g_s=list(limits.brake_rate_g_s),
                pid_slew_delay_compensation=pid.force_slew_compensation)
        schedule = getattr(plan, 'phase_schedule', [
            ('initial_hold', None), ('rise', settings.move_seconds),
            ('top_hold', None), ('return', settings.move_seconds), ('final_hold', None)])
        phase_index = 0
        phase, movement_duration = schedule[phase_index]
        phase_start = time.monotonic()
        if compact_console:
            print('PHASE:', phase, flush=True)
        stable_since = None
        readback_ns = time.monotonic_ns()
        console_ns = readback_ns
        pose_guess = plan.initial_pose.tolist()
        # Each new command is computed from the previous complete, fresh actual
        # feedback. The force-write ACKs provide the next measured feedback.
        while True:
            if stop[0] or (out / 'stop_request.json').exists():
                report['outcome'] = 'operator_stopped'
                break  # terminal Mode 0 anchor; no automatic return after stop
            loop_ns = time.monotonic_ns()
            elapsed = time.monotonic() - phase_start
            pending_cycle = dict(ns=loop_ns, phase=phase, phase_elapsed_s=elapsed, stage='target_generation',
                                 pid=None, pid_output_g=None, target_force_g=None)
            pose_target, target = plan.target(phase, elapsed)
            pending_cycle.update(stage='actual_position_check', target_pose_m_rad=pose_target.tolist(),
                target_positions_steps=slider_to_steps(target, plan.geometry).tolist())
            plan.check_positions(core.positions)
            sample_ns = max(core.rx_ns)
            q = raw_steps_to_slider_m(core.positions, plan.geometry)
            pending_cycle.update(stage='pid_update', actual_positions_steps=list(core.positions),
                position_error_steps=(slider_to_steps(target, plan.geometry)-core.positions).tolist())
            delta_n, diag = pid.update(target, q, sample_ns=sample_ns, now_ns=loop_ns,
                                      position_stamps_ns=core.rx_ns)
            pending_cycle.update(stage='force_limits', pid=diag, pid_output_g=(delta_n/NOMINAL_N_PER_G).tolist())
            command = drive.command_forces_n(np.asarray(g_to_n(hold)) + delta_n,
                dry_run=True, now_ns=loop_ns, braking_axes=pid.braking_axes)
            target_g = [r['target_force_g'] for r in command['frames']]
            pending_cycle.update(stage='force_write', target_force_g=target_g)
            core.check_hardware_write_permission()
            audit = core.writer.send_force_cycle(target_g, cycle_id=report['control_cycles'], target_timestamp_ns=loop_ns)
            report['control_cycles'] += 1
            pid.track_applied(np.asarray(g_to_n(target_g)) - np.asarray(g_to_n(hold)))
            pending_cycle['stage'] = 'feedback_ingest'
            ingest(audit.feedbacks, audit.feedback_rx_done_ns, audit.cycle_start_ns)
            pending_cycle['stage'] = 'actual_position_check_after_feedback'
            plan.check_positions(core.positions)
            pending_cycle['stage'] = 'mode_and_force_readback'
            readback = None
            if time.monotonic_ns() - readback_ns >= 1_000_000_000:
                modes = inventory()
                readback_ns = time.monotonic_ns()
                readback = [r['target_g'] for r in modes]
                if [r['mode'] for r in modes] != [3]*6:
                    raise RuntimeError('Mode 3 lost during position control')
                if readback != target_g:
                    raise RuntimeError('0x27 target force readback mismatch')
            fk = kinematics.forward_kinematics_with_metrics(
                raw_steps_to_slider_m(core.positions, plan.geometry), initial_guess=pose_guess)
            # FK is monitoring, never a replacement for the fresh joint loop.
            actual_pose = list(fk.pose) if fk.converged and np.isfinite(fk.pose).all() else None
            if actual_pose is not None:
                pose_guess = actual_pose
            row = dict(ns=time.monotonic_ns(), phase=phase, phase_elapsed_s=elapsed,
                       target_timestamp_ns=loop_ns,
                       source=report['source'], target_pose_m_rad=pose_target.tolist(),
                       actual_pose_m_rad=actual_pose, target_z_m=float(pose_target[2]),
                       actual_z_m=actual_pose[2] if actual_pose is not None else None,
                       fk_converged=bool(fk.converged),
                       target_positions_steps=slider_to_steps(target, plan.geometry).tolist(),
                       actual_positions_steps=list(core.positions),
                       position_error_steps=(slider_to_steps(target, plan.geometry)-core.positions).tolist(),
                       pid=diag, pid_input_rx_ns=list(pid.position_history[i][-1][0] for i in range(6)),
                       pid_output_g=(delta_n/NOMINAL_N_PER_G).tolist(), target_force_g=target_g,
                       target_force_27_g_readback=readback, force_readback_ns=readback_ns if readback is not None else None,
                       force_2c_g=list(core.forces_g_raw), currents_ma=list(core.currents),
                       modes_last_readback=[r['mode'] for r in modes], mode_readback_ns=readback_ns,
                       fault_codes=list(core.faults), feedback_rx_ns=list(core.rx_ns),
                       cycle_ms=(time.monotonic_ns()-loop_ns)/1e6)
            if hasattr(plan, 'control_diagnostics'):
                row['attitude'] = plan.control_diagnostics
            record('tracking.jsonl', row)
            last_complete_frame = row
            if row['ns'] - console_ns >= 5_000_000_000:
                console_ns = row['ns']
                if hasattr(plan, 'console_line'):
                    print(plan.console_line(row), flush=True)
                else:
                    index = getattr(plan, 'axis_index', 2)
                    name = getattr(plan, 'axis', 'z')
                    scale, unit = (1000., 'mm') if index < 3 else (180./np.pi, 'deg')
                    actual_value = 'UNAVAILABLE' if actual_pose is None else f'{scale*(actual_pose[index]-plan.initial_pose[index]):+.3f}'
                    print(f'[{phase}] target Δ{name}={scale*(pose_target[index]-plan.initial_pose[index]):+.3f} {unit}'
                          f' actual Δ{name}={actual_value} {unit} max |e|={max(abs(x) for x in row["position_error_steps"]):.2f} steps'
                          f' max current={max(abs(x) for x in core.currents)} mA', flush=True)
            # All six measured positions and measured velocity must settle;
            # merely reaching the planned time is not a successful stop.
            settled = hold_sample_settled(row['position_error_steps'], pid.velocity,
                                         row['actual_z_m'], row['target_z_m'], settings)
            if movement_duration is None:
                stable_since = (stable_since if stable_since is not None else time.monotonic()) if settled else None
                dwell = settings.settle_seconds + (settings.hold_seconds if phase != 'initial_hold' else 0.)
                if stable_since is not None and time.monotonic() - stable_since >= dwell:
                    report['stages'].append(dict(phase=phase, settled=True, elapsed_s=elapsed))
                    if phase_index == len(schedule)-1:
                        report['outcome'] = 'motion_complete'
                        break
                    phase_index += 1
                    phase, movement_duration = schedule[phase_index]
                    phase_start = time.monotonic()
                    stable_since = None
                    print('PHASE:', phase, flush=True)
                elif elapsed >= settings.settle_timeout_s:
                    raise RuntimeError(phase + ' did not settle/hold within timeout')
            elif elapsed >= movement_duration:
                report['stages'].append(dict(phase=phase, elapsed_s=elapsed))
                phase_index += 1
                phase, movement_duration = schedule[phase_index]
                phase_start = time.monotonic()
                stable_since = None
                print('PHASE:', phase, flush=True)
            time.sleep(max(0., .02-(time.monotonic_ns()-loop_ns)*1e-9))
    except BaseException as exc:
        report.update(outcome='fault', error=f'{type(exc).__name__}: {exc}', fault_timestamp_ns=time.monotonic_ns())
        if plan is not None and getattr(plan, 'capture_fault_snapshot', False):
            # Copy fault state before recovery changes feedback. Defer disk I/O
            # until after the existing stop/recovery/close sequence.
            fault_snapshot = dict(fault_timestamp_ns=report['fault_timestamp_ns'], error=report['error'],
                source=report['source'], pending_cycle=pending_cycle,
                target_request=getattr(plan, 'target_request', None),
                attitude=getattr(plan, 'control_diagnostics', None), last_complete_frame=last_complete_frame,
                latest_feedback=dict(actual_positions_steps=list(core.positions), currents_ma=list(core.currents),
                    force_2c_g=list(core.forces_g_raw), fault_codes=list(core.faults), feedback_rx_ns=list(core.rx_ns)),
                pid_output_note='pending_cycle output is null if PID failed before computing it; last_complete_frame retains the last computed output')
    finally:
        if core:
            try:
                if (plan is not None and hasattr(plan, 'recover_holding_bias') and report.get('all_mode3_confirmed')):
                    try:
                        report['force_recovery'] = plan.recover_holding_bias(core=core, drive=drive, hold=hold,
                            outcome=report['outcome'], error=report.get('error'), first_cycle_id=report['control_cycles'],
                            read_positions=read_positions, inventory=inventory, ingest=ingest,
                            check_positions=plan.check_positions, record=record)
                    except Exception as exc:
                        report['force_recovery'] = dict(status='aborted_no_retry', error=str(exc), cycles=0)
                    if report['force_recovery']['status'] == 'aborted_no_retry' and report['outcome'] != 'fault':
                        report.update(outcome='fault', error='FORCE_RECOVERY_FAILED: '+report['force_recovery']['error'],
                                      fault_timestamp_ns=time.monotonic_ns())
                terminal = core.finish_force_control('z_position_finished', fault=report['outcome'] == 'fault')
                report['terminal_recovery'] = terminal
                report['all_mode0_exit_confirmed'] = bool(terminal and terminal.get('position_hold_confirmed'))
                if terminal is not None and not report['all_mode0_exit_confirmed']:
                    report['outcome'] = 'exit_unconfirmed'
            except Exception as exc:
                report.update(outcome='exit_unconfirmed', exit_error=str(exc), all_mode0_exit_confirmed=False)
            finally:
                try:
                    core.close('z_position_finished')
                except Exception as exc:
                    report.update(outcome='exit_unconfirmed', close_error=str(exc))
        if trace:
            report['hardware_write_count'] = len(trace.write_commands)
        if bus is not None:
            report['mock_write_count'] = len(getattr(bus, 'write_audit', []))
        for sig, previous in handlers.items():
            signal.signal(sig, previous)
        report['serial_closed'] = bool(core and core.closed)
        if plan is not None and hasattr(plan, 'workspace_result'):
            report['workspace_result'] = plan.workspace_result(report['outcome'])
        if fault_snapshot is not None:
            fault_snapshot['all_mode0_exit_confirmed'] = report.get('all_mode0_exit_confirmed', False)
            fault_snapshot['serial_closed'] = report['serial_closed']
            (out/'fault_snapshot.json').write_text(json.dumps(fault_snapshot, indent=2)+'\n')
        report_file.write_text(json.dumps(report, indent=2) + '\n')
    return report


def _signed(value):
    return value - 65536 if value >= 32768 else value


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--hardware', action='store_true')
    parser.add_argument('--hardware-qualified', action='store_true', help='existing explicit write authorization flag')
    parser.add_argument('--mock', action='store_true', help='execute against the existing static MockServoBus (no physical plant)')
    parser.add_argument('--dry-run', action='store_true', help='offline IK planning only, no serial access')
    parser.add_argument('--initial-steps', type=float, nargs=6, default=[1000.]*6, help='offline planning input only')
    parser.add_argument('--rise-mm', type=float, default=10.)
    parser.add_argument('--move-seconds', type=float, default=MotionConfig.move_seconds)
    parser.add_argument('--hold-seconds', type=float, default=3.)
    parser.add_argument('--port', default='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0')
    parser.add_argument('--baudrate', type=int, default=921600)
    parser.add_argument('--output-dir')
    args = parser.parse_args(argv)
    if args.hardware and args.mock:
        parser.error('--hardware and --mock cannot be combined')
    if args.hardware and not args.dry_run and not args.hardware_qualified:
        parser.error('--hardware requires explicit --hardware-qualified')
    try:
        settings = MotionConfig(rise_m=args.rise_mm/1000., move_seconds=args.move_seconds, hold_seconds=args.hold_seconds)
        if args.dry_run or not (args.hardware or args.mock):
            print(json.dumps(dict(source='OFFLINE', hardware_write_count=0,
                                  plan=ZPlan(args.initial_steps, settings).summary()), indent=2))
            return 0
    except ValueError as exc:
        parser.error(str(exc))
    root = Path(__file__).resolve().parents[4]
    out = args.output_dir or str(root/'results/z_position_motion'/datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    config = dict(epoch=int(time.time()), hardware_mode='active' if args.hardware else 'mock',
                  port=args.port, baudrate=args.baudrate, output_dir=out, execute=True, dry_run=False,
                  hardware_write_enabled=True, is_hardware_qualified=bool(args.hardware_qualified or args.mock),
                  control_mode='force', maintenance_only=True, force_fault_recovery='position_reanchor',
                  max_force_handover_displacement_steps=6)
    report = run(config, settings)
    print(json.dumps(report, indent=2))
    return 0 if report['outcome'] in ('motion_complete', 'operator_stopped') else 2


if __name__ == '__main__':
    raise SystemExit(main())
