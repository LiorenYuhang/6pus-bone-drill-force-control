"""Independent Mode 3 dynamic force control and Z-axis motion experiment.

Captures actual signed holding force (0x2C), enters native Mode 3 with holding
force handover, applies task-space Z-axis dynamic force commands (+/- amplitude N)
via the existing Jacobian force allocation, observes platform upward/downward
Z-axis motion via forward kinematics (FK) and actual force feedback, records all
axis targets, feedback, positions, currents, and task-space motions, and cleanly
exits to Mode 0 with position re-anchoring.
"""
from dataclasses import asdict
from datetime import datetime
import json
from pathlib import Path
import signal
import sys
import time

import numpy as np
import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import Float64

from pus6_msgs.msg import TimedJointFeedback
from pus6_motor_driver.motor import Motor
from pus6_motor_driver.j14_process_uart import ServoCore
from pus6_motor_driver.j14_feedback_bridge import populate_force_feedback
from pus6_motor_driver.force_units import g_to_n, n_to_g, quantize_force_g_with_n_limits
from .force_control_node import TaskSpaceForceControlNode
from .force_pipeline import raw_steps_to_slider_m
from .six_axis_tare_experiment import SerialTrace
from .stage3c_experiment import ingest_snapshot_feedback


def json_default(value):
    return value.tolist() if isinstance(value, np.ndarray) else value.item()


def run(config, bus=None):
    execute = config.get('execute', False)
    if execute and not (config.get('hardware_write_enabled') and config.get('is_hardware_qualified')
                        and config.get('dry_run') is False):
        raise ValueError('existing independent hardware execution permissions required')
    if config.get('hardware_mode') not in ('active', 'mock'):
        raise ValueError("hardware_mode must be 'active' or 'mock'")
    if not config.get('maintenance_only') or config.get('force_fault_recovery') != 'position_reanchor':
        raise ValueError('only existing real maintenance UART owner and position_reanchor recovery are supported')

    # Mock impersonation guard: active mode strictly forbids mock bus injection
    if config.get('hardware_mode') == 'active' and bus is not None:
        raise RuntimeError('active hardware_mode requires real hardware; mock bus injection forbidden')

    out = Path(config['output_dir'])
    out.mkdir(parents=True, exist_ok=True)

    report = dict(
        source='REAL_HARDWARE' if config.get('hardware_mode') == 'active' else 'MOCK',
        test_name='mode3_z_motion_dynamic',
        start_time=datetime.now().astimezone().isoformat(),
        outcome='started',
        hardware_write_count=0,
        tare_count=0,
        pause_count=0,
        snapshots=[],
        hold_forces_g=[],
        initial_positions_steps=[],
        initial_pose_m_rad=[],
        final_positions_steps=[],
        final_pose_m_rad=[],
        max_current_ma_observed=0,
        max_displacement_steps_observed=0,
        max_z_motion_mm_observed=0.0,
        min_z_motion_mm_observed=0.0,
        all_mode3_confirmed=False,
        all_mode0_exit_confirmed=False,
        stages_executed=[],
        control_cycles=0,
    )

    core = None
    node = None
    trace = None
    sequence = 0
    stop = [False]
    initial_positions = None
    initial_pose = None

    def save():
        if trace:
            report.update(
                hardware_write_count=len(trace.write_commands),
                tare_count=sum(w['register'] == 9 for w in trace.write_commands),
                pause_count=sum(w['register'] == 0x1A for w in trace.write_commands),
            )
        (out / 'z_motion_experiment.json').write_text(
            json.dumps(report, ensure_ascii=False, indent=2, default=json_default) + '\n'
        )

    def record(name, row):
        with (out / name).open('a') as stream:
            stream.write(json.dumps(row, default=json_default) + '\n')

    def check(positions, currents, faults, forces):
        if any(faults):
            raise RuntimeError(f'hardware fault reported: {faults}')
        max_curr = max(abs(v) for v in currents) if currents else 0
        if max_curr > report['max_current_ma_observed']:
            report['max_current_ma_observed'] = int(max_curr)
        if max_curr > config.get('max_current_ma', 500):
            raise RuntimeError(f'current protection exceeded: {max_curr} > {config.get("max_current_ma", 500)} mA')
        if initial_positions is not None:
            max_disp = max(abs(x - y) for x, y in zip(positions, initial_positions))
            if max_disp > report['max_displacement_steps_observed']:
                report['max_displacement_steps_observed'] = int(max_disp)
            if max_disp > config.get('max_position_change_steps', 12):
                raise RuntimeError(
                    f'displacement protection exceeded: {max_disp} > {config.get("max_position_change_steps", 12)} steps'
                )
        if max(abs(v) for v in g_to_n(forces)) > 14.709975:
            raise RuntimeError('force feedback limit exceeded (> 14.71 N)')

    def snapshot(label):
        nonlocal sequence
        rows = []
        statuses = {}
        rx = {}
        start = time.monotonic_ns()
        with core.writer.uart_lock:
            for mid in range(1, 7):
                m = Motor(core.bus, mid)
                s = m.read_status()
                rx[mid] = time.monotonic_ns()
                statuses[mid] = s
                words = {r: m.read_register(r).values[0] for r in (0x25, 0x27, 0x2A, 0x2C, 0x2B, 0x2F)}
                signed = lambda x: x - 65536 if x >= 32768 else x
                rows.append(dict(
                    motor_id=mid,
                    mode=words[0x25],
                    target_force_g=signed(words[0x27]),
                    force_2c_g=signed(words[0x2C]),
                    position_2a=words[0x2A],
                    rx_done_ns=rx[mid],
                    register_words={hex(k): v for k, v in words.items()},
                    **asdict(s),
                ))
        if not core.feedback.stopped:
            sequence += 1
            ingest_snapshot_feedback(core, statuses, rx, start, sequence)
        item = dict(label=label, ns=time.monotonic_ns(), time_iso=datetime.now().astimezone().isoformat(), axes=rows)
        report['snapshots'].append(item)
        record('snapshots.jsonl', item)
        save()
        return rows

    def feedback_message(audit):
        nonlocal sequence
        fb = core.feedback.ingest_cycle_feedback(
            audit.feedbacks,
            servo_cycle_id=sequence,
            servo_cycle_timestamp_ns=audit.cycle_start_ns,
            per_axis_rx_done_ns=audit.feedback_rx_done_ns,
        )
        core.update(fb)
        if core.feedback.stopped or core.writer.stopped:
            raise RuntimeError('feedback/transport stopped')
        if not all(core.force_valid):
            raise RuntimeError('invalid force feedback')
        check(core.positions, core.currents, core.faults, core.forces_g_raw)
        msg = TimedJointFeedback()
        msg.uart_cycle_sequence = sequence
        msg.snapshot_start_monotonic_ns = audit.cycle_start_ns
        msg.snapshot_end_monotonic_ns = time.monotonic_ns()
        msg.per_axis_rx_done_monotonic_ns = list(core.rx_ns)
        msg.raw_steps = list(core.positions)
        msg.positions_meters = raw_steps_to_slider_m(core.positions, node.kinematics.geometry_config).tolist()
        msg.currents_ma = list(core.currents)
        msg.fault_codes = list(core.faults)
        populate_force_feedback(msg, core.force_feedback_fields())
        if config.get('hardware_mode') == 'active' and msg.source != TimedJointFeedback.SOURCE_HARDWARE:
            raise RuntimeError('non-hardware force source detected in active mode')
        node._handle_timed_feedback(msg)

    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.__setitem__(0, True))

    try:
        # Initialize TaskSpaceForceControlNode
        if not rclpy.ok():
            rclpy.init()
        overrides = [
            Parameter('dry_run', value=not execute),
            Parameter('hardware_write_enabled', value=execute),
            Parameter('is_hardware_qualified', value=execute),
            Parameter('fault_stop_policy', value='external_position_reanchor'),
            Parameter('reference_rate_n_s', value=1.0),
            Parameter('use_fixed_lag', value=True),
            Parameter('fixed_lag_delay_ms', value=30.0),
            Parameter('max_current_ma', value=config.get('max_current_ma', 500)),
        ]
        node = TaskSpaceForceControlNode('mode3_z_motion_node', parameter_overrides=overrides)
        node.timer.cancel()

        core = ServoCore(config, bus=bus)
        core.bus.timeout_s = 0.05
        if core.serial is not None:
            trace = SerialTrace(core.serial, out)
            core.serial = trace
            core.bus.serial = trace
        else:
            trace = None
            if config.get('hardware_mode') == 'active':
                raise RuntimeError('active hardware_mode requires valid serial port, but none available')

        core.initialize()  # strictly read-only Mode 0 check
        before = snapshot('before_mode3')
        initial_positions = [r['current_position'] for r in before]
        report['initial_positions_steps'] = initial_positions
        if any(r['mode'] != 0 for r in before):
            raise RuntimeError(f"initial mode is not uniformly Mode 0: {[r['mode'] for r in before]}")
        check(initial_positions, [r['current_mA'] for r in before], [r['error_code'] for r in before], [r['force_g'] for r in before])

        # Compute initial platform pose via FK
        q0 = raw_steps_to_slider_m(initial_positions, node.kinematics.geometry_config)
        fk0 = node.kinematics.forward_kinematics_with_metrics(q0)
        if not fk0.converged:
            raise RuntimeError('initial FK did not converge')
        initial_pose = np.array(fk0.pose)
        report['initial_pose_m_rad'] = initial_pose.tolist()
        print('INITIAL_AUDIT_OK: 6 motors Mode 0. Initial Pose [XYZ mm, RPY deg]:',
              np.round(initial_pose[:3] * 1000, 2).tolist(),
              np.round(np.rad2deg(initial_pose[3:]), 2).tolist(), flush=True)

        if not execute:
            report['outcome'] = 'read_only_complete'
            return report

        # Latch holding force from current 0x2C feedback
        handover = core.prepare_force_handover()
        holding_g = handover['holding_forces_g']
        report.update(hold_forces_g=list(holding_g), handover_axes=handover['axes'])
        node.initialize_hold_forces_g(holding_g, now_ns=time.monotonic_ns())
        save()
        print('HOLDING_FORCE_LATCHED:', holding_g, flush=True)

        def verify_entry(mid):
            rows = snapshot(f'after_mode3_axis_{mid}')
            check([r['current_position'] for r in rows], [r['current_mA'] for r in rows],
                  [r['error_code'] for r in rows], [r['force_g'] for r in rows])

        # Enter Mode 3 with holding baseline
        core.enter_force_mode_with_handover(holding_g, static_baseline_g=holding_g, axis_verifier=verify_entry)
        report['mode_handover'] = core.writer.last_force_handover

        # Confirm all 6 axes entered Mode 3
        active = snapshot('all_mode3_hold_targets')
        if any(r['mode'] != 3 or r['target_force_g'] != hold for r, hold in zip(active, holding_g)):
            raise RuntimeError('Mode 3 / hold target readback mismatch')
        check([r['current_position'] for r in active], [r['current_mA'] for r in active],
              [r['error_code'] for r in active], [r['force_g'] for r in active])
        report['all_mode3_confirmed'] = True
        last_target_forces_readback = [int(r['target_force_g']) for r in active]
        print('MODE3_ALL_AXES_CONFIRMED: Mode 3 verified on all 6 axes.', flush=True)

        # Define dynamic stages
        amp = float(config.get('amplitude_n', 1.5))
        stage_dur = float(config.get('stage_duration_s', 3.0))
        mode = config.get('stages_mode', 'both')

        stages = [
            ('zero_native', 0.0, 0.5),   # native hold to prime Fixed-Lag buffer
            ('zero_pid', 0.0, 1.0),      # closed-loop PID holding baseline
        ]
        if mode in ('both', 'positive_only'):
            stages.extend([
                ('positive_z', amp, stage_dur),      # Upward Z force command
                ('return_zero_pos', 0.0, 2.0),       # Return to zero force
            ])
        if mode in ('both', 'negative_only'):
            stages.extend([
                ('negative_z', -amp, stage_dur),     # Downward Z force command
                ('return_zero_final', 0.0, 2.0),     # Return to zero force
            ])

        cycle_id = 0
        last_modes_ns = time.monotonic_ns()
        last_log_time = time.monotonic()
        modes = [3] * 6

        for stage_name, target_z, duration in stages:
            if stop[0]:
                break
            stage_start = time.monotonic()
            report['stages_executed'].append(dict(name=stage_name, target_z_n=target_z, duration_s=duration))
            save()
            print(f"\n>>> START STAGE: {stage_name} | Target Z: {target_z:+.2f} N | Duration: {duration:.1f}s", flush=True)

            if stage_name == 'zero_pid':
                node._reset_control_state(time.monotonic_ns())

            while not stop[0] and (time.monotonic() - stage_start) < duration:
                loop_start = time.monotonic_ns()
                node._handle_target_force_z(Float64(data=target_z))
                cycle = None

                if stage_name != 'zero_native':
                    node._control_cycle()
                    if node.fault_latched:
                        raise RuntimeError(f"PID node fault: {node.fault_reason}")
                    if node.last_cycle_result is not None and node.last_processed_cycle_seq == sequence:
                        cycle = node.last_cycle_result

                # Actuator targets from existing Jacobian allocator and PID
                target_n = node.last_axial_command_n if cycle is not None else node.pipeline.hold_forces_n.tolist()
                comp = node.pipeline.compensation
                limits = node.drive.limits
                target_g = quantize_force_g_with_n_limits(
                    n_to_g(target_n, comp.direction, comp.newtons_per_g),
                    limits.min_force_n, limits.max_force_n, comp.direction, comp.newtons_per_g
                )

                core.check_hardware_write_permission()
                audit = core.writer.send_force_cycle(target_g, cycle_id=cycle_id, target_timestamp_ns=loop_start)
                cycle_id += 1
                sequence += 1
                feedback_message(audit)
                report['control_cycles'] += 1

                # Calculate platform XYZ motion via FK
                current_q = raw_steps_to_slider_m(core.positions, node.kinematics.geometry_config)
                current_fk = node.kinematics.forward_kinematics_with_metrics(
                    current_q, initial_guess=node.current_pose_estimate
                )
                if current_fk.converged:
                    node.current_pose_estimate = list(current_fk.pose)
                    current_pose = np.array(current_fk.pose)
                else:
                    current_pose = np.array(node.current_pose_estimate)

                delta_xyz_mm = (current_pose[:3] - initial_pose[:3]) * 1000.0
                delta_z_mm = float(delta_xyz_mm[2])
                report['max_z_motion_mm_observed'] = max(report['max_z_motion_mm_observed'], delta_z_mm)
                report['min_z_motion_mm_observed'] = min(report['min_z_motion_mm_observed'], delta_z_mm)

                # Periodic register readback audit (every 1.0s): verify Mode 3 (0x25) and target force (0x27)
                if time.monotonic_ns() - last_modes_ns > 1_000_000_000:
                    signed_i16 = lambda v: v - 65536 if v >= 32768 else v
                    with core.writer.uart_lock:
                        modes = [Motor(core.bus, i).read_register(0x25).values[0] for i in range(1, 7)]
                        target_forces_readback = [
                            signed_i16(Motor(core.bus, i).read_register(0x27).values[0]) for i in range(1, 7)
                        ]
                    last_target_forces_readback = list(target_forces_readback)
                    last_modes_ns = time.monotonic_ns()
                    record('mode_checks.jsonl', dict(
                        ns=last_modes_ns,
                        modes=modes,
                        target_forces_g_readback=target_forces_readback,
                        target_forces_g_commanded=list(target_g),
                        target_forces_g_error=[rb - cmd for rb, cmd in zip(target_forces_readback, target_g)],
                    ))
                    if modes != [3] * 6:
                        raise RuntimeError(f"Mode 3 lost during Z motion: {modes}")

                aligned_g = (
                    list(node.last_aligned_forces_g)
                    if getattr(node, 'last_aligned_forces_g', None) is not None
                    else list(core.forces_g_raw)
                )
                raw_feedback_meas_z_n = float(
                    np.sum(g_to_n(np.array(core.forces_g_raw) - np.array(holding_g)))
                )

                # Tracking record
                row = dict(
                    ns=time.monotonic_ns(),
                    time_iso=datetime.now().astimezone().isoformat(),
                    cycle_id=cycle_id,
                    stage=stage_name,
                    source=report['source'],
                    target_z_n=target_z,
                    reference_z_n=float(cycle.pid.reference_n) if cycle and cycle.pid else None,
                    measured_z_n=float(cycle.measured_wrench[2]) if cycle and cycle.measured_wrench is not None else None,
                    raw_feedback_meas_z_n=raw_feedback_meas_z_n,
                    pid_output_n=float(cycle.pid.output_n) if cycle and cycle.pid else None,
                    tracking_error_n=float(cycle.pid.error_n) if cycle and cycle.pid else None,
                    target_force_g=target_g,
                    target_force_27_g_readback=list(last_target_forces_readback),
                    query_timestamp_ns=int(node.last_sync_stamp_ns) if node.last_sync_stamp_ns else loop_start - 30_000_000,
                    force_2c_g=list(core.forces_g_raw),
                    aligned_force_2c_g=aligned_g,
                    relative_force_g=[f - h for f, h in zip(core.forces_g_raw, holding_g)],
                    relative_aligned_force_g=[f - h for f, h in zip(aligned_g, holding_g)],
                    positions_steps=list(core.positions),
                    displacement_steps=[p - p0 for p, p0 in zip(core.positions, initial_positions)],
                    currents_ma=list(core.currents),
                    fault_codes=list(core.faults),
                    mode_last_readback=modes,
                    pose_m_rad=current_pose.tolist(),
                    delta_xyz_mm=delta_xyz_mm.tolist(),
                    delta_z_mm=delta_z_mm,
                    cycle_ms=(audit.cycle_end_ns - audit.cycle_start_ns) / 1e6,
                )
                record('z_motion_tracking.jsonl', row)

                # Periodic console log (every 0.5s)
                if time.monotonic() - last_log_time >= 0.5:
                    last_log_time = time.monotonic()
                    meas_z = f"{cycle.measured_wrench[2]:+.2f}N" if cycle and cycle.measured_wrench is not None else "N/A"
                    raw_z = f"{raw_feedback_meas_z_n:+.2f}N"
                    ref_z = f"{cycle.pid.reference_n:+.2f}N" if cycle and cycle.pid else "0.0N"
                    max_c = max(abs(c) for c in core.currents)
                    print(f"[{stage_name:<16}] Tgt={target_z:+.2f}N Ref={ref_z} Meas={meas_z}(Raw={raw_z}) | ΔZ={delta_z_mm:+.3f}mm (ΔX={delta_xyz_mm[0]:+.2f}, ΔY={delta_xyz_mm[1]:+.2f}) | I_max={max_c}mA", flush=True)

                if (out / 'stop_request.json').exists():
                    report['outcome'] = 'operator_stopped'
                    break

                elapsed = (time.monotonic_ns() - loop_start) / 1e9
                time.sleep(max(0.0, 0.02 - elapsed))

            # Stage completion snapshot and verification
            if not stop[0] and not (out / 'stop_request.json').exists():
                stage_snap = snapshot(f'stage_end_{stage_name}')
                check([r['current_position'] for r in stage_snap], [r['current_mA'] for r in stage_snap],
                      [r['error_code'] for r in stage_snap], [r['force_g'] for r in stage_snap])
                last_target_forces_readback = [int(r['target_force_g']) for r in stage_snap]

        if stop[0]:
            report['outcome'] = 'signal_stopped'
        elif report['outcome'] != 'operator_stopped':
            report['outcome'] = 'bounded_window_complete'

    except BaseException as exc:
        report.update(outcome='stopped', error=f"{type(exc).__name__}: {exc}")
        print('Z_MOTION_EXPERIMENT_ERROR:', report['error'], flush=True)
    finally:
        if core:
            terminal = core.finish_force_control('z_motion_finished', fault=report.get('outcome') == 'stopped')
            if terminal is not None:
                report['terminal_recovery'] = terminal
                report['all_mode0_exit_confirmed'] = terminal.get('position_hold_confirmed', False)
                report['hardware_stop_confirmed'] = terminal.get('hardware_stop_confirmed', False)
                report['independent_stop_required'] = terminal.get('independent_stop_required', False)
            after = snapshot('final_after_mode0_exit')
            report['final_positions_steps'] = [r['current_position'] for r in after]
            report['all_mode0_exit_confirmed'] = all(r['mode'] == 0 for r in after)
            if node:
                q_final = raw_steps_to_slider_m([r['current_position'] for r in after], node.kinematics.geometry_config)
                fk_final = node.kinematics.forward_kinematics_with_metrics(q_final)
                if fk_final.converged:
                    report['final_pose_m_rad'] = list(fk_final.pose)
            core.close('z_motion_finished')
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        report.update(
            end_time=datetime.now().astimezone().isoformat(),
            serial_closed=bool(core and core.closed),
        )
        save()
        print('RESULT:', report['outcome'], '| MODE0_EXIT:', report.get('all_mode0_exit_confirmed'),
              '| SOURCE:', report.get('source'), '| ΔZ_RANGE: [', round(report['min_z_motion_mm_observed'], 3),
              ',', round(report['max_z_motion_mm_observed'], 3), '] mm | DIR:', out, flush=True)

    return report


def main(argv=None):
    import argparse
    parser = argparse.ArgumentParser(description="6PUS Mode 3 Dynamic Z-Axis Force Control Experiment")
    parser.add_argument('--hardware', action='store_true', help='Enable real hardware Mode 3 force control write interface')
    parser.add_argument('--execute', action='store_true', help='Alias for --hardware')
    parser.add_argument('--hardware-qualified', action='store_true', help='Explicit hardware qualification confirmation')
    parser.add_argument('--dry-run', action='store_true', help='Dry-run mode: read-only motor audit, zero hardware writes')
    parser.add_argument('--amplitude', type=float, default=1.5, help='Task-space Z force command amplitude in N (default: 1.5N)')
    parser.add_argument('--stage-duration', type=float, default=3.0, help='Duration of positive/negative force stages in seconds (default: 3.0s)')
    parser.add_argument('--positive-only', action='store_true', help='Execute positive Z force stage only')
    parser.add_argument('--negative-only', action='store_true', help='Execute negative Z force stage only')
    parser.add_argument('--max-displacement-steps', type=int, default=12, help='Motor displacement protection limit in steps (default: 12 steps / 0.3mm)')
    parser.add_argument('--max-current-ma', type=int, default=500, help='Unified motor current protection in mA (default: 500mA)')
    parser.add_argument('--port', default=None, help='Motors UART port (default: autodetect /dev/serial/by-id or /dev/ttyUSB0)')
    parser.add_argument('--baudrate', type=int, default=921600, help='Motors UART baudrate')
    parser.add_argument('--output-dir', default=None, help='Output directory for experiment data')
    parser.add_argument('--mock', action='store_true', help='Run in mock mode (for unit testing)')
    args = parser.parse_args(argv)

    is_hardware = (args.hardware or args.execute) and not args.dry_run
    if is_hardware and not args.hardware_qualified:
        parser.error('--hardware requires explicit --hardware-qualified authorization')
    if is_hardware and args.mock:
        parser.error('--hardware cannot run with --mock (real hardware execution cannot be spoofed by mock)')

    root = Path(__file__).resolve().parents[4]
    motor_port = args.port
    if not motor_port:
        by_id_motor = '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'
        if Path(by_id_motor).exists():
            motor_port = by_id_motor
        elif Path('/dev/ttyUSB0').exists():
            motor_port = '/dev/ttyUSB0'
        else:
            motor_port = by_id_motor

    out_dir = Path(args.output_dir) if args.output_dir else (
        root / 'results/mode3_z_motion' / datetime.now().strftime('%Y%m%d_%H%M%S')
    )

    stages_mode = 'both'
    if args.positive_only and not args.negative_only:
        stages_mode = 'positive_only'
    elif args.negative_only and not args.positive_only:
        stages_mode = 'negative_only'

    config = dict(
        epoch=int(time.time()),
        hardware_mode='mock' if args.mock else 'active',
        port=motor_port,
        baudrate=args.baudrate,
        output_dir=str(out_dir),
        control_mode='force',
        maintenance_only=True,
        dry_run=not is_hardware,
        execute=is_hardware,
        hardware_write_enabled=is_hardware,
        is_hardware_qualified=bool(args.hardware_qualified if is_hardware else False),
        force_fault_recovery='position_reanchor',
        max_current_ma=args.max_current_ma,
        max_position_change_steps=args.max_displacement_steps,
        max_force_handover_displacement_steps=6,
        amplitude_n=args.amplitude,
        stage_duration_s=args.stage_duration,
        stages_mode=stages_mode,
    )

    result = run(config)
    print(f"OUTCOME: {result.get('outcome')} | MODE0_EXIT: {result.get('all_mode0_exit_confirmed')} | WRITES: {result.get('hardware_write_count', 0)}")
    return 0 if result.get('outcome') in ('bounded_window_complete', 'operator_stopped', 'read_only_complete', 'signal_stopped') else 2


if __name__ == '__main__':
    raise SystemExit(main())
