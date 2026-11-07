"""Independent Mode 3 holding force handover test runner.

Captures actual signed holding force (0x2C), enters native Mode 3 with constant
0x27 targets equal to the captured holding force (no PID correction, Delta F = 0),
maintains force control for a short bounded duration (default 3.0s), logs all
axis modes, targets, forces, positions, and currents, and cleanly exits to Mode 0
with measured position re-anchoring.
"""
from dataclasses import asdict
from datetime import datetime
import json
from pathlib import Path
import signal
import sys
import time

import numpy as np

from pus6_motor_driver.j14_process_uart import ServoCore
from pus6_motor_driver.motor import Motor
from pus6_motor_driver.force_units import g_to_n
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

    # Mock impersonation guard: active mode strictly forbids mock bus injection or missing hardware
    if config.get('hardware_mode') == 'active' and bus is not None:
        raise RuntimeError('active hardware_mode requires real hardware; mock bus injection forbidden')

    out = Path(config['output_dir'])
    out.mkdir(parents=True, exist_ok=True)

    report = dict(
        source='REAL_HARDWARE' if config.get('hardware_mode') == 'active' else 'MOCK',
        test_name='mode3_holding_force_handover',
        start_time=datetime.now().astimezone().isoformat(),
        outcome='started',
        hardware_write_count=0,
        tare_count=0,
        pause_count=0,
        snapshots=[],
        hold_forces_g=[],
        initial_positions_steps=[],
        final_positions_steps=[],
        max_current_ma_observed=0,
        max_displacement_steps_observed=0,
        all_mode3_confirmed=False,
        all_mode0_exit_confirmed=False,
        hold_cycles_completed=0,
        holding_duration_seconds=0.0,
    )

    core = None
    trace = None
    entered = False
    sequence = 0
    stop = [False]
    initial_positions = None

    def save():
        if trace:
            report.update(
                hardware_write_count=len(trace.write_commands),
                tare_count=sum(w['register'] == 9 for w in trace.write_commands),
                pause_count=sum(w['register'] == 0x1A for w in trace.write_commands),
            )
        (out / 'hold_experiment.json').write_text(
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
            if max_disp > config.get('max_position_change_steps', 6):
                raise RuntimeError(f'displacement protection exceeded: {max_disp} > {config.get("max_position_change_steps", 6)} steps')
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

    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.__setitem__(0, True))

    try:
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

        core.initialize()  # existing maintenance startup is strictly read-only
        before = snapshot('before_mode3')
        initial_positions = [r['current_position'] for r in before]
        report['initial_positions_steps'] = initial_positions
        if any(r['mode'] != 0 for r in before):
            raise RuntimeError(f"initial mode is not uniformly Mode 0: {[r['mode'] for r in before]}")
        check(initial_positions, [r['current_mA'] for r in before], [r['error_code'] for r in before], [r['force_g'] for r in before])
        print('INITIAL_AUDIT_OK:', [(r['motor_id'], r['mode'], r['current_position'], r['force_g'], r['current_mA'], r['error_code']) for r in before], flush=True)

        if not execute:
            report['outcome'] = 'read_only_complete'
            return report

        # Latch holding force reference from current signed 0x2C feedback
        handover = core.prepare_force_handover()
        hold = handover['holding_forces_g']
        report.update(hold_forces_g=list(hold), handover_axes=handover['axes'])
        save()
        print('HOLDING_FORCE_LATCHED:', hold, flush=True)

        def verify_entry(mid):
            rows = snapshot(f'after_mode3_axis_{mid}')
            check([r['current_position'] for r in rows], [r['current_mA'] for r in rows],
                  [r['error_code'] for r in rows], [r['force_g'] for r in rows])

        # Enter Mode 3 with holding force baseline
        core.enter_force_mode_with_handover(hold, static_baseline_g=hold, axis_verifier=verify_entry)
        entered = True
        report['mode_handover'] = core.writer.last_force_handover

        # Confirm all 6 axes entered Mode 3 with target == hold
        active = snapshot('all_mode3_hold_targets')
        if any(r['mode'] != 3 or r['target_force_g'] != h for r, h in zip(active, hold)):
            raise RuntimeError('Mode 3 / hold target readback mismatch')
        check([r['current_position'] for r in active], [r['current_mA'] for r in active],
              [r['error_code'] for r in active], [r['force_g'] for r in active])
        report['all_mode3_confirmed'] = True
        print('MODE3_ALL_AXES_CONFIRMED: Mode 3 verified on all 6 axes with target == hold.', flush=True)

        begin = time.monotonic()
        target_g = list(hold)  # Constant target, Delta F = 0
        cycle_id = 0
        last_modes_ns = time.monotonic_ns()
        modes = [3] * 6
        target_duration = config.get('duration', 3.0)

        # 50 Hz holding loop
        while not stop[0] and (time.monotonic() - begin) < target_duration:
            start = time.monotonic_ns()
            core.check_hardware_write_permission()
            audit = core.writer.send_force_cycle(target_g, cycle_id=cycle_id, target_timestamp_ns=start)
            cycle_id += 1

            feedback = core.feedback.ingest_cycle_feedback(
                audit.feedbacks,
                servo_cycle_id=cycle_id,
                servo_cycle_timestamp_ns=audit.cycle_start_ns,
                per_axis_rx_done_ns=audit.feedback_rx_done_ns,
            )
            core.update(feedback)

            # Impersonation guard on live feedback
            if config.get('hardware_mode') == 'active' and getattr(core.bus, 'is_mock', False):
                raise RuntimeError('hardware_mode active but mock bus detected; cannot spoof real hardware')

            check(core.positions, core.currents, core.faults, core.forces_g_raw)
            if core.feedback.stopped or core.writer.stopped or not all(core.force_valid):
                raise RuntimeError('transport/feedback stopped or invalid force reported')

            # Periodic register 0x25 audit (every 1.0s)
            if time.monotonic_ns() - last_modes_ns > 1_000_000_000:
                with core.writer.uart_lock:
                    modes = [Motor(core.bus, i).read_register(0x25).values[0] for i in range(1, 7)]
                last_modes_ns = time.monotonic_ns()
                record('mode_checks.jsonl', dict(ns=last_modes_ns, modes=modes))
                if modes != [3] * 6:
                    raise RuntimeError(f'Mode 3 lost during holding loop: {modes}')

            # Tracking log
            row = dict(
                ns=time.monotonic_ns(),
                cycle_id=cycle_id,
                phase='hold',
                source=report['source'],
                target_force_g=target_g,
                force_2c_g=list(core.forces_g_raw),
                positions_steps=list(core.positions),
                displacement_steps=[p - p0 for p, p0 in zip(core.positions, initial_positions)],
                currents_ma=list(core.currents),
                fault_codes=list(core.faults),
                force_valid=list(core.force_valid),
                mode_last_readback=modes,
                cycle_transaction_ms=(audit.cycle_end_ns - audit.cycle_start_ns) / 1e6,
            )
            record('hold_tracking.jsonl', row)

            if (out / 'stop_request.json').exists():
                report['outcome'] = 'operator_stopped'
                break

            elapsed = (time.monotonic_ns() - start) / 1e9
            time.sleep(max(0.0, 0.02 - elapsed))

        report['hold_cycles_completed'] = cycle_id
        report['holding_duration_seconds'] = time.monotonic() - begin
        if stop[0]:
            report['outcome'] = 'signal_stopped'
        elif report['outcome'] != 'operator_stopped':
            report['outcome'] = 'bounded_window_complete'

    except BaseException as exc:
        report.update(outcome='stopped', error=f'{type(exc).__name__}: {exc}')
        print('HOLD_TEST_ERROR:', report['error'], flush=True)
    finally:
        if core:
            terminal = core.finish_force_control('mode3_hold_finished', fault=report.get('outcome') == 'stopped')
            if terminal is not None:
                report['terminal_recovery'] = terminal
                report['all_mode0_exit_confirmed'] = terminal.get('position_hold_confirmed', False)
                report['hardware_stop_confirmed'] = terminal.get('hardware_stop_confirmed', False)
                report['independent_stop_required'] = terminal.get('independent_stop_required', False)
            after = snapshot('final_after_mode0_exit')
            report['final_positions_steps'] = [r['current_position'] for r in after]
            report['all_mode0_exit_confirmed'] = all(r['mode'] == 0 for r in after)
            core.close('mode3_hold_finished')
        report.update(
            end_time=datetime.now().astimezone().isoformat(),
            serial_closed=bool(core and core.closed),
        )
        save()
        print('RESULT:', report['outcome'], '| MODE0_EXIT:', report.get('all_mode0_exit_confirmed'),
              '| SOURCE:', report.get('source'), '| DIR:', out, flush=True)

    return report


def main(argv=None):
    import argparse
    parser = argparse.ArgumentParser(description="6PUS Mode 3 Holding Force Handover Test Runner")
    parser.add_argument('--hardware', action='store_true', help='Enable real hardware Mode 3 force control write interface')
    parser.add_argument('--execute', action='store_true', help='Alias for --hardware')
    parser.add_argument('--hardware-qualified', action='store_true', help='Explicit hardware qualification confirmation')
    parser.add_argument('--dry-run', action='store_true', help='Dry-run mode: read-only motor audit, zero hardware writes')
    parser.add_argument('--port', default=None, help='Motors UART port (default: autodetect /dev/serial/by-id or /dev/ttyUSB0)')
    parser.add_argument('--baudrate', type=int, default=921600, help='Motors UART baudrate')
    parser.add_argument('--duration', type=float, default=3.0, help='Holding duration in seconds (default: 3.0s)')
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
        root / 'results/mode3_hold_handover' / datetime.now().strftime('%Y%m%d_%H%M%S')
    )

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
        max_current_ma=500,
        max_position_change_steps=6,
        max_force_handover_displacement_steps=6,
        duration=args.duration,
    )

    result = run(config)
    print(f"OUTCOME: {result.get('outcome')} | MODE0_EXIT: {result.get('all_mode0_exit_confirmed')} | WRITES: {result.get('hardware_write_count', 0)}")
    return 0 if result.get('outcome') in ('bounded_window_complete', 'operator_stopped', 'read_only_complete', 'signal_stopped') else 2


if __name__ == '__main__':
    raise SystemExit(main())
