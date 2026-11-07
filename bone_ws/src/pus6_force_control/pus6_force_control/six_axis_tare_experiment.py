"""Bounded Stage 3B-3 maintenance operation INSIDE the existing UART Child."""
from dataclasses import asdict
from datetime import datetime
import json
from pathlib import Path
import time

from pus6_motor_driver.motor import Motor
from pus6_motor_driver.protocol import FrameStreamParser, RSP_FRAME_HEADER, parse_frame
from pus6_motor_driver.j14_process_uart import ServoCore


class SerialTrace:
    """Record bytes on the owner's existing port; no second port or RX consumer."""
    allowed_write_registers = (9, 0x25, 0x27)

    def __init__(self, serial, directory):
        object.__setattr__(self, '_serial', serial)
        object.__setattr__(self, '_directory', Path(directory))
        object.__setattr__(self, '_parser', FrameStreamParser(expected_header=RSP_FRAME_HEADER))
        object.__setattr__(self, 'write_commands', [])

    def enable_terminal_recovery(self):
        object.__setattr__(self, '_terminal_recovery', True)

    def __getattr__(self, name):
        return getattr(self._serial, name)

    def __setattr__(self, name, value):
        setattr(self._serial, name, value)

    def record(self, direction, raw):
        row = dict(time_iso=datetime.now().astimezone().isoformat(), ns=time.monotonic_ns(),
                   direction=direction, raw_hex=bytes(raw).hex(' '))
        with (self._directory/'serial_bytes.jsonl').open('a') as stream:
            stream.write(json.dumps(row)+'\n')

    def write(self, data):
        self.record('TX', data)
        frame = parse_frame(data)
        if frame.motor_id not in range(1,7):
            raise RuntimeError('Stage 3B-3 forbids other IDs and broadcast')
        if frame.command == 0x32:
            if frame.register_addr not in self.allowed_write_registers and not (
                    frame.register_addr == 0x1A and getattr(self, '_terminal_recovery', False)
                    and frame.data == b'\x01\x00'):
                raise RuntimeError('Stage 3B-3 forbids Pause/protection/other writes')
            self.write_commands.append(dict(motor_id=frame.motor_id,register=frame.register_addr,
                values=[int.from_bytes(frame.data[i:i+2],'little') for i in range(0,len(frame.data),2)]))
        return self._serial.write(data)

    def read(self, size=1):
        data = self._serial.read(size)
        if data:
            self.record('RX',data)
            frames = self._parser.feed(data)
            with (self._directory/'rx_frames.jsonl').open('a') as stream:
                for frame in frames:
                    stream.write(json.dumps(dict(ns=time.monotonic_ns(), motor_id=frame.motor_id,
                        command=frame.command,register=frame.register_addr,raw_hex=frame.raw.hex(' ')))+'\n')
        return data


def normalize_archived_units(value):
    """Read old experiment schemas without rewriting archived g readings.

    The historical `_gf` suffix was a label error. This is a file-reader
    migration, not a compatibility alias in the live driver/Tare interfaces.
    """
    if isinstance(value, dict):
        return {key[:-3] + '_g' if key.endswith('_gf') else key:
                normalize_archived_units(item) for key, item in value.items()}
    if isinstance(value, list):
        return [normalize_archived_units(item) for item in value]
    return value


def run(config):
    """No PID, trajectory, background streaming or automatic Tare at startup."""
    out=Path(config['output_dir']);out.mkdir(parents=True,exist_ok=True)
    report=dict(source='REAL_HARDWARE',start_time=datetime.now().astimezone().isoformat(),
                outcome='started',snapshots=[],pid_started=False,software_baseline_subtracted=False)
    core=None;trace=None
    def save():
        if core is not None:
            report['tare_states']=core.hardware_tare.snapshot() if hasattr(core,'hardware_tare') else None
            report['handover_report']=getattr(core.writer,'last_force_handover',None)
        if trace is not None:
            report['hardware_write_count']=len(trace.write_commands)
            report['write_commands']=trace.write_commands
        (out/'experiment.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    def snapshot(label):
        rows=[]
        with core.writer.uart_lock:
            for mid in range(1,7):
                motor=Motor(core.bus,mid)
                status=motor.read_status()
                mode=motor.read_register(0x25).values[0]
                target=motor.read_register(0x27).values[0]
                force=motor.read_register(0x2C).values[0]
                rows.append(dict(motor_id=mid,mode=mode,target_force_u16=target,
                    force_2c_g=force-65536 if force>=32768 else force,
                    rx_done_ns=time.monotonic_ns(),**asdict(status)))
        entry=dict(label=label,time_iso=datetime.now().astimezone().isoformat(),axes=rows)
        report['snapshots'].append(entry)
        with (out/'snapshots.jsonl').open('a') as stream:stream.write(json.dumps(entry)+'\n')
        return rows
    def check(rows, positions, tolerance):
        for row in rows:
            if row['error_code'] or row['current_mA']>config.get('max_current_ma',1000):
                raise RuntimeError(f"ID{row['motor_id']} current/fault monitor stop")
            if abs(row['current_position']-positions[row['motor_id']-1])>tolerance:
                raise RuntimeError(f"ID{row['motor_id']} position monitor stop")
    try:
        core=ServoCore(config)
        # Maintenance reads include post-save readiness, not a 50 Hz motion
        # deadline. Normal UART Child timing remains unchanged.
        core.bus.timeout_s=.05
        trace=SerialTrace(core.serial,out)
        core.serial=trace;core.bus.serial=trace
        core.initialize()  # maintenance_only: reads only, no Tare/mode setup
        before=snapshot('before_tare')
        report['pre_tare_raw_g']=[r['force_2c_g'] for r in before]
        initial_positions=[r['current_position'] for r in before]
        if any(r['mode']!=0 or r['error_code'] for r in before):
            raise RuntimeError('initial actual modes/faults are unsuitable')
        second=snapshot('stationary_before_tare')
        check(second,initial_positions,2)
        if not config.get('execute_tare_and_switch',False):
            report['outcome']='read_only_complete';return report
        if not config.get('mechanical_support_confirmed',False):
            raise RuntimeError('independent mechanical support/stopping conditions are not confirmed')
        # One saved record BEFORE the first device reference change. Never repeat
        # this operation automatically on process restart or a timeout.
        marker=out/'tare_operation_started.json'
        with marker.open('x') as stream:
            json.dump(dict(time_iso=datetime.now().astimezone().isoformat(),pre_tare=before),stream)
        if config.get('resume_from'):
            prior=normalize_archived_units(json.loads(Path(config['resume_from']).read_text()))
            report['resume_from']=config['resume_from']
            report['original_pre_tare_raw_g']=prior.get('original_pre_tare_raw_g',prior['pre_tare_raw_g'])
            for row in (prior.get('tare_states') or {}).values():
                if row['write_attempted']:
                    core.verify_completed_tare_axis(row,samples=20,tolerance_g=config.get('tare_tolerance_g',50))
                    save()
                    print(f"ID{row['motor_id']} prior completion verified; Tare NOT resent",flush=True)
        for mid in range(1,7):
            core.tare_force_axis(mid,samples=20,tolerance_g=config.get('tare_tolerance_g',50),
                                 completion_timeout_s=1.0,sample_interval_s=.01)
            save()
            print(f'ID{mid} Tare confirmed',flush=True)
        after=snapshot('after_all_tare')
        check(after,initial_positions,2)
        if any(abs(r['force_2c_g'])>config.get('tare_tolerance_g',50) for r in after):
            raise RuntimeError('six-axis Tare reference did not remain within tolerance')
        report['pre_entry_target_words']=[r['target_force_u16'] for r in after]
        core.enter_force_mode_after_hardware_tare(axis_verifier=lambda mid:
            check(snapshot(f'after_switch_ID{mid}'),initial_positions,config.get('max_position_change_steps',10)))
        active=snapshot('all_mode3')
        if any(r['mode']!=3 or r['target_force_u16']!=0 for r in active):
            raise RuntimeError('six actual modes/zero targets were not confirmed')
        report['all_mode3_confirmed']=True
        end=time.monotonic()+config.get('hold_seconds',2.0)
        while time.monotonic()<end:
            check(snapshot('static_mode3_hold'),initial_positions,config.get('max_position_change_steps',10))
            time.sleep(.02)
        report['exit_anchors']=core.exit_force_mode_safe()
        final=snapshot('after_mode0_exit')
        check(final,report['exit_anchors'],5)
        if any(r['mode']!=0 for r in final):raise RuntimeError('Mode 0 exit incomplete')
        report['all_mode0_exit_confirmed']=True
        report['outcome']='tare_mode3_hold_and_exit_complete'
        return report
    except Exception as exc:
        report['outcome']='stopped'
        report['error']=str(exc)
        # Entry partial-failure already performs its bounded per-axis rollback.
        # A hold fault first exits using the validated same-frame method.
        if core is not None:
            if core.mode_3_active:
                try:report['fault_exit_anchors']=core.exit_force_mode_safe()
                except Exception as recovery:report['recovery_error']=str(recovery)
            core.writer.latch_fault_without_pause(str(exc))
            core.stop(str(exc))
            try: snapshot('fault_final_readback')
            except Exception as final_error:report['final_readback_error']=str(final_error)
        raise
    finally:
        report['end_time']=datetime.now().astimezone().isoformat()
        save()
        if core is not None:
            # Maintenance close must not silently introduce Pause after a switch
            # fault or a successful static experiment/Mode 0 exit.
            core.writer.latch_fault_without_pause('maintenance_closed_without_pause')
            core.close('stage3b3_complete')
