"""A1.9 opt-in UART process experiment. No ROS imports; no automatic recovery."""
from __future__ import annotations
import argparse
from collections import deque
from dataclasses import asdict
import json
import math
import os
from pathlib import Path
import resource
import select
import signal
import socket
import subprocess
import sys
import threading
import time

from .fake_bus import MockServoBus
from .j7_real_uart_adapter import J7UartAdapter, J7StopAdapter, J14_UART_WRITE_TIMEOUT_S
from .j7_six_axis_servo_transport import J7SixAxisServoTransport, ServoRxPolicy, SixAxisCycleAudit, SixAxisTxRecord
from .j7_servo_feedback_scheduler import J7ServoFeedbackScheduler
from .stall_diagnostic_recorder import J11StallDiagnosticRecorder
from .protocol import (
    CMD_FRAME_HEADER,
    CMD_RD_REGISTER,
    RSP_FRAME_HEADER,
    StatusFeedback,
    bytes_to_hex,
    parse_frame,
    pack_read_register_cmd,
    unpack_read_register_response,
    validate_response,
    FrameLengthError,
)
from .safety import (
    MOTOR_IDS,
    CONTROL_MODE_ADDR,
    ACTUAL_FORCE_ADDR,
    TARGET_POSITION_ADDR,
    TARGET_FORCE_ADDR,
    SerialPortLock,
    SafetyViolation,
    ForceHandoverState,
    ForceBaselineManager,
    STAGE3B1_STATIC_BASELINE_G,
    validate_initial_handover_forces,
    validate_target_vector,
    n_to_g,
    g_to_n,
    encode_signed_i16_to_u16,
    decode_u16_to_signed_i16,
)

from .force_units import DEFAULT_FORCE_LIMIT_N, NOMINAL_N_PER_G, clamp_force_n, quantize_force_g, quantize_force_g_with_n_limits

PERIOD_NS = 20_000_000
LEASE_NS = 1_000_000_000
PACKET_MAX = 16384
DRAIN_MAX = 64


def socket_pair():
    pair = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)
    for s in pair:
        s.setblocking(False)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 8192)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8192)
    return pair


def send_packet(sock, row):
    payload = json.dumps(row, separators=(',', ':')).encode()
    if len(payload) > PACKET_MAX:
        raise ValueError('IPC packet too large')
    try:
        if sock.send(payload) != len(payload):
            raise RuntimeError('partial seqpacket send')
        return True
    except BlockingIOError:
        return False


def receive_packets(sock):
    rows = []
    for _ in range(DRAIN_MAX):
        try:
            data, _, flags, _ = sock.recvmsg(PACKET_MAX)
        except BlockingIOError:
            return rows, False
        if not data:
            if rows:
                return rows, False
            raise EOFError('IPC peer exited')
        if flags & socket.MSG_TRUNC:
            raise ValueError('truncated IPC packet')
        row = json.loads(data)
        if not isinstance(row, dict):
            raise ValueError('IPC object required')
        rows.append(row)
    # A flooded receiver does bounded work; no old vector may be dispatched.
    return rows, True


def append_rows(path, rows):
    with Path(path).open('a') as f:
        for row in rows:
            f.write(json.dumps(row) + '\n')


class CommandState:
    """A single whole-vector reference and separate parent lease supporting static position and force control."""
    def __init__(
        self,
        epoch,
        initial,
        target_timeout_s=.2,
        jump_limit=80,
        fallback_force_n=None,
        hold_anchor_on_timeout=False,
    ):
        self.epoch = epoch
        self.seq = -1
        self.mode = 'static'
        self.targets = list(initial)
        self.forces_n = [0.0] * 6
        self.command_ns = 0
        self.receive_ns = 0
        self.lease_ns = time.monotonic_ns()
        self.timeout_ns = int(target_timeout_s * 1e9)
        self.jump_limit = jump_limit
        self.fallback_force_n = list(fallback_force_n) if fallback_force_n is not None else None
        self.hold_anchor_on_timeout = hold_anchor_on_timeout
        self.consumed = True
        self.stop_reason = None
        self.rejected = 0

    def ingest(self, rows, now_ns, anchor):
        newest = None
        for row in rows:
            if row.get('epoch') != self.epoch:
                raise ValueError('IPC epoch mismatch')
            if row.get('type') == 'stop':
                self.stop_reason = row.get('reason', 'parent_stop')
                continue
            row_mode = row.get('mode')
            if row.get('type') not in ('state', 'start') or row_mode not in ('static', 'force') or row.get('allowed') is not True:
                raise ValueError('IPC mode/permission invalid')
            lease = row.get('lease_ns')
            if not isinstance(lease, int) or lease > now_ns:
                raise ValueError('IPC lease invalid')
            self.lease_ns = max(self.lease_ns, lease)
            cmd = row.get('command')
            if cmd is None:
                continue
            seq, stamp = cmd.get('seq'), cmd.get('publish_ns')
            if type(seq) is not int or type(stamp) is not int or stamp > now_ns or stamp <= 0:
                raise ValueError('IPC sequence/timestamp invalid')
            if row_mode == 'static':
                targets = cmd.get('targets')
                if not isinstance(targets, list) or len(targets) != 6 or any(type(v) is not int for v in targets):
                    raise ValueError('whole six integer targets required')
                validate_target_vector(targets)
            elif row_mode == 'force':
                forces_n = cmd.get('forces_n')
                if not isinstance(forces_n, list) or len(forces_n) != 6 or any(type(v) not in (int, float) or not math.isfinite(v) for v in forces_n):
                    raise ValueError('whole six finite float forces required')
            if seq > self.seq and (newest is None or seq > newest['seq']):
                newest = dict(cmd, row_mode=row_mode)
        if self.stop_reason or newest is None:
            return
        row_mode = newest.get('row_mode', 'static')
        if row_mode == 'static':
            stale = self.command_ns > 0 and now_ns - self.command_ns > self.timeout_ns
            if stale and max(abs(a-b) for a,b in zip(newest['targets'], anchor)) > self.jump_limit:
                self.rejected += 1
                return
            self.seq = newest['seq']
            self.mode = 'static'
            self.targets = list(newest['targets'])
        elif row_mode == 'force':
            self.seq = newest['seq']
            self.mode = 'force'
            self.forces_n = [float(v) for v in newest['forces_n']]
        self.command_ns = newest['publish_ns']
        self.receive_ns = now_ns
        self.consumed = False

    def candidate(self, anchor, warning):
        if warning or self.consumed:
            return list(anchor)
        self.consumed = True
        return list(self.targets)

    def candidate_force(self, anchor_force, warning, now_ns):
        stale = self.command_ns > 0 and (now_ns - self.command_ns > self.timeout_ns)
        if warning or stale or self.consumed:
            if stale:
                raise RuntimeError('force_target_timeout: terminal handover required')
            return list(anchor_force)
        self.consumed = True
        return list(self.forces_n)


def dispatch_slew(candidate, anchor, last_write_ns, now_ns):
    """Same driver final-dispatch vector rule; upstream ROS shaper is unchanged."""
    dt = max(.010, (now_ns-last_write_ns)/1e9) if last_write_ns else .020
    allowance = min(20, max(5, int(200.0*dt)))
    gaps = [c-a for c,a in zip(candidate, anchor)]
    gap = max(map(abs, gaps))
    if gap <= allowance:
        return list(candidate)
    alpha = allowance / gap
    result = [int(round(a+alpha*g)) for a,g in zip(anchor, gaps)]
    return [a+min(20,max(-20,d-a)) for a,d in zip(anchor,result)]


def dispatch_force_slew(
    candidate_g=None,
    anchor_g=None,
    max_rate_g_s=500.0,
    last_write_ns=0,
    now_ns=0,
    limits_g=(-32768, 32767),
):
    """Applies per-axis slew-rate limiting (dF/dt) and saturation bounds to target forces in g.

    Manufacturer sales confirmed native unit is g (gram).
    """
    if candidate_g is None or anchor_g is None:
        raise ValueError("candidate_g and anchor_g are required")

    dt = max(0.010, min(0.040, (now_ns - last_write_ns) / 1e9)) if last_write_ns else 0.020
    rates = max_rate_g_s if isinstance(max_rate_g_s, (list, tuple)) else [max_rate_g_s] * 6
    min_g, max_g = limits_g
    result = []
    for c, a, r in zip(candidate_g, anchor_g, rates):
        max_step = float(r) * dt
        delta = float(c) - float(a)
        clamped_step = max(-max_step, min(max_step, delta))
        val = float(a) + clamped_step
        sat = max(min_g, min(max_g, val))
        result.append(quantize_force_g(sat))
    return result


class ServoCore:
    """Single-thread UART/safety owner shared by thread/process mock A/B."""
    def __init__(self, config, bus=None):
        self.config = config
        if any(k in config for k in ('min_force_g', 'max_force_g')):
            raise ValueError('Use min_force_n/max_force_n in N; convert old g limits explicitly')
        self.out = Path(config['output_dir'])
        self.out.mkdir(parents=True, exist_ok=True)
        self.rec = J11StallDiagnosticRecorder(output_dir=str(self.out), enabled=True)
        self.lifecycle = []
        self.force_shutdown_report = None
        self._force_control_managed = False
        self.pauses = []
        self.warnings = deque(maxlen=64)
        self.rows = []  # timing scalars only; trial is bounded to 60 seconds
        self.write_rows = []
        self.production = bool(config.get('production'))
        self.telemetry_mode = config.get('telemetry_mode','debug')
        self.health = dict(cycles=0,motion_writes=0,tx_timeouts=0,fault_cycles=0,
            max_serial_write_ns=0,max_rx_drain_ns=0,max_wake_lateness_ns=0,
            wake_watchlist_count=0)
        if self.production:
            capacity = 300 if self.telemetry_mode=='production' else config['telemetry_capacity_cycles']
            self.rows=deque(maxlen=capacity); self.write_rows=deque(maxlen=capacity*6)
            self.rec.tier1_summary=deque(maxlen=capacity)
            self.rec.all_stall_events=deque(maxlen=capacity)
            self.rec.rx_slow_read_events=deque(maxlen=capacity)
        self.serial = None
        if bus is not None:
            self.bus = bus
        elif config['hardware_mode'] == 'mock':
            init_modes = config.get('mock_initial_modes')
            if init_modes is None and config.get('control_mode') == 'force':
                init_modes = [0] * 6
            self.bus = MockServoBus(initial_positions=config.get('mock_initial_positions',[1000]*6), initial_modes=init_modes)
        elif config['hardware_mode'] == 'active':
            import serial
            self.serial = serial.Serial(config['port'], config['baudrate'],
                bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                timeout=.005, write_timeout=J14_UART_WRITE_TIMEOUT_S,
                exclusive=True)
            fileno_fn = getattr(self.serial, 'fileno', None)
            fd = fileno_fn() if callable(fileno_fn) else None
            self.port_lock = SerialPortLock(fd, port=config['port'], owner='uart_child')
            self.port_lock.acquire()
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()
            self.bus = J7UartAdapter(self.serial, baudrate=config['baudrate'], timeout_s=.010,
                min_frame_interval_s=.0001,
                tx_queue_diagnostics=(not self.production or self.telemetry_mode == 'debug'),
                read_observer=self.rec.record_single_read)
        else:
            raise ValueError('trial permits mock or active only')
        self.stop_adapter = J7StopAdapter(self.bus, audit_observer=self._pause_audit)
        self.writer = J7SixAxisServoTransport(self.bus, cycle_rate_hz=50,
            rx_policy=ServoRxPolicy.DISCARD_BEFORE_TX, min_frame_interval_s=.0006,
            rx_drain_budget_s=.004, pause_callback=self._transport_pause,
            recoverable_write_timeouts=True)
        self.feedback = J7ServoFeedbackScheduler(self.writer, full_scan_period_s=1e9,
            feedback_budget_s=.015, servo_cycle_period_s=.020, timeout_escalation_misses=4,
            max_feedback_age_s=1., warning_feedback_age_s=.2833, hard_stop_feedback_age_s=1.,
            feedback_loss_policy='age_based', warn_callback=self.warnings.append)
        if self.production:
            # Bound instance-owned evidence, not the frozen transport implementations.
            self.writer._cycles=deque(maxlen=300)
            self.writer._rx_records=deque(maxlen=300)
            self.feedback._cycles=deque(maxlen=300)
        self.positions = [1000]*6
        self.currents = [0]*6
        self.faults = [0]*6
        self.rx_ns = [0]*6
        self.forces_g_raw = [0]*6
        self.force_adc_raw = [0]*6
        self.temperatures_c = [0]*6
        self.force_valid = [False]*6
        self.last_command = None
        self.last_command_force_g = None
        self.mode_3_active = False
        self.force_hold_g = None
        self.force_hold_reference_ns = 0
        self.handover_state = ForceHandoverState.MODE0_POSITION
        self.baseline_manager = ForceBaselineManager()
        self.force_handover_snapshot = None
        self.fault_latched = False
        self.last_write_ns = 0
        self.cycle_seq = 0
        self.fault = None
        self.last_packet = None
        self.closed = False
        self.servo_start_ns=0
        self.motion_stop_ns=0


    def can_write_hardware(self) -> bool:
        """Independently verifies 3-way hardware write authorization conditions."""
        if self.fault_latched or self.fault is not None:
            return False
        if self.config.get('dry_run', False):
            return False
        if self.config.get('hardware_mode') == 'mock':
            return True
        if self.config.get('hardware_mode') == 'active':
            return bool(
                self.config.get('hardware_write_enabled', False)
                and self.config.get('is_hardware_qualified', False)
            )
        return False

    def check_hardware_write_permission(self):
        """Raises descriptive RuntimeError if hardware writes are not authorized."""
        if self.config.get('dry_run', False):
            raise RuntimeError("hardware writes blocked: dry_run=True")
        if self.config['hardware_mode'] != 'mock':
            if not self.config.get('hardware_write_enabled', False):
                raise RuntimeError("hardware writes blocked: hardware_write_enabled=False")
            if not self.config.get('is_hardware_qualified', False):
                raise RuntimeError("hardware writes blocked: is_hardware_qualified=False")

    def _simulate_dry_run_audit(self, dispatched, cycle_id, timestamp_ns, is_force=False):
        """Simulate a cycle audit in dry_run mode without touching physical bus."""
        feedbacks = {
            mid: StatusFeedback(
                target_position=dispatched[i] if not is_force else self.positions[i],
                current_position=self.positions[i],
                current_mA=self.currents[i],
                force_g=dispatched[i] if is_force else self.forces_g_raw[i],
                force_adc=self.force_adc_raw[i],
                temperature=self.temperatures_c[i],
                error_code=0,
            )
            for i, mid in enumerate(MOTOR_IDS)
        }
        feedback_rx_done_ns = {mid: timestamp_ns for mid in MOTOR_IDS}
        tx_records = tuple(
            SixAxisTxRecord(
                motor_id=mid,
                cycle_id=cycle_id,
                target_position=dispatched[i],
                target_timestamp_ns=timestamp_ns,
                tx_start_ns=timestamp_ns,
                tx_end_ns=timestamp_ns,
                response_wait=False,
                retry_count=0,
                residual_frames_before=0,
                residual_frames_after=0,
                duration_ns=0,
                cpu_duration_ns=0,
                requested_bytes=0,
                written_bytes=0,
                error=None,
                serial_write_metric=None,
            )
            for i, mid in enumerate(MOTOR_IDS)
        )
        return SixAxisCycleAudit(
            cycle_id=cycle_id,
            target_timestamp_ns=timestamp_ns,
            cycle_start_ns=timestamp_ns,
            cycle_end_ns=timestamp_ns,
            cycle_jitter_ns=0,
            target_age_ns=0,
            tx_records=tx_records,
            tx_skew_ns=0,
            batch_complete_ns=timestamp_ns,
            deadline_miss=False,
            feedbacks=feedbacks,
            feedback_rx_done_ns=feedback_rx_done_ns,
            prior_rx_drain_wall_ns=0,
            prior_rx_drain_cpu_ns=0,
            prior_rx_drained_frames=0,
            inter_frame_wait_total_ns=0,
            motion_drain_wall_ns=0,
            motion_drain_cpu_ns=0,
            rx_drain_metrics=None,
            error_phase=None,
            error_motor_id=None,
            error_class=None,
        )

    def _enable_terminal_trace(self):
        hook = getattr(self.serial, 'enable_terminal_recovery', None)
        if hook is not None:
            hook()

    def _transport_pause(self, reason):
        """Restore the original one-shot fault hook without bypassing permissions."""
        if self.config.get('control_mode') == 'force':
            self.check_hardware_write_permission()
            hook = getattr(self.serial, 'enable_terminal_recovery', None)
            if hook is not None:
                hook()
        return self.stop_adapter(reason)

    def _pause_audit(self, audit):
        # Existing one-shot audit persists after all six pause requests.
        row = asdict(audit)
        self.pauses.append(row)
        append_rows(self.out/'stop_pause_audit.jsonl', [row])
        self.lifecycle.append(dict(event=('pause_status_ack' if self.config.get('control_mode') == 'force'
                                          else 'pause_complete'),
            ns=time.monotonic_ns(), success=row['success'], physical_stop_verified=False))

    def update(self, audit):
        self.force_valid = [False]*6
        for s in audit.samples:
            if s.position is not None:
                i=s.motor_id-1
                self.positions[i]=s.position
                self.currents[i]=s.current_mA or 0
                self.faults[i]=s.fault or 0
                self.rx_ns[i]=s.timestamp_ns
                self.forces_g_raw[i]=s.force_g if s.force_g is not None else 0
                self.force_adc_raw[i]=s.force_adc if s.force_adc is not None else 0
                self.temperatures_c[i]=s.temperature_c if s.temperature_c is not None else 0
                self.force_valid[i]=s.force_valid and not s.timeout

    def initialize(self):
        self.lifecycle.append(dict(event='serial_owner', ns=time.monotonic_ns(), pid=os.getpid(),
                                   hardware_mode=self.config['hardware_mode']))
        scan = self.feedback.full_scan(servo_cycle_id=0, servo_cycle_timestamp_ns=time.monotonic_ns())
        self.update(scan)
        if any(s.timeout or s.position is None for s in scan.samples) or len(scan.samples)!=6:
            raise RuntimeError('incomplete startup status')
        if self.feedback.stopped:
            raise RuntimeError('unsafe startup status')
        try:
            if self.config.get('maintenance_only', False):
                # Read-only owner startup. Tare and mode changes require explicit calls.
                modes=[row['mode'] for row in self.writer.read_force_handover_state()]
                if modes != [0]*6 and not (self.config.get('mode3_only') and modes == [3]*6):
                    raise SafetyViolation('maintenance startup requires uniform actual Mode 0, or explicit Mode3-only attachment')
                self.mode_3_active = False
                self.handover_state = (ForceHandoverState.MODE0_POSITION if modes==[0]*6
                    else ForceHandoverState.PREPARING_FORCE_HANDOVER)
            elif self.config.get('control_mode') == 'force':
                init_f = self.config.get('initial_force_g')
                if init_f is None:
                    if self.config.get('dry_run', False):
                        # dry-run needs an explicit input, not fabricated hardware readings.
                        init_f = self.config.get('static_baseline_g')
                        if init_f is None:
                            raise ValueError("dry-run requires explicit initial forces or baseline")
                    else:
                        init_f = self.prepare_force_handover()['holding_forces_g']
                if isinstance(init_f, (int, float)):
                    initial_forces = [int(round(init_f))] * 6
                else:
                    initial_forces = [int(round(f)) for f in init_f]

                is_dry_run = self.config.get('dry_run', False)
                if is_dry_run:
                    self.mode_3_active = True
                    self.handover_state = ForceHandoverState.MODE3_FORCE_ACTIVE
                    self.last_command_force_g = list(initial_forces)
                elif self.config['hardware_mode'] != 'mock' and not self.can_write_hardware():
                    # Unauthorized: do not perform any hardware writes; remain unswitched
                    self.mode_3_active = False
                    self.handover_state = ForceHandoverState.MODE0_POSITION
                else:
                    self.handover_state = ForceHandoverState.PREPARING_FORCE_HANDOVER
                    baseline = self.baseline_manager.software_baseline_g or self.config.get('static_baseline_g')
                    if self.config.get('check_static_baseline', False):
                        initial_forces = validate_initial_handover_forces(
                            initial_forces, static_baseline_g=baseline or STAGE3B1_STATIC_BASELINE_G
                        )
                    self.enter_force_mode_with_handover(
                        initial_forces, static_baseline_g=baseline)
            else:
                is_dry_run = self.config.get('dry_run', False)
                if is_dry_run:
                    self.mode_3_active = False
                    self.handover_state = ForceHandoverState.MODE0_POSITION
                elif self.config['hardware_mode'] != 'mock' and not self.can_write_hardware():
                    self.mode_3_active = False
                    self.handover_state = ForceHandoverState.MODE0_POSITION
                else:
                    self.writer.configure_servo(speed=self.config.get('servo_speed', 50))
                    self.mode_3_active = False
                    self.handover_state = ForceHandoverState.MODE0_POSITION
        except Exception as exc:
            self.fault_latched = True
            self.handover_state = ForceHandoverState.FAULT_LATCHED
            if self.config.get('control_mode') == 'force':
                self.writer.latch_fault_without_pause(f'mode_configure_failure: {exc}')
            self.stop(f'mode_configure_failure: {exc}')
            raise
        if self.mode_3_active and self.force_hold_g is None:
            self._capture_force_hold_reference(self.last_command_force_g)
        fallback_fn = self.config.get('fallback_force_n')
        if self.config.get('hold_anchor_on_timeout') and self.force_hold_g is not None:
            fallback_fn = g_to_n(self.force_hold_g,self.config.get('direction',(1.,)*6),
                                 self.config.get('newtons_per_g',(NOMINAL_N_PER_G,)*6))
        hold_anchor = self.config.get('hold_anchor_on_timeout', False)
        self.command = CommandState(self.config['epoch'], self.positions,
            self.config.get('target_timeout_sec',.2),self.config.get('reconnect_max_jump_steps',80),
            fallback_force_n=fallback_fn, hold_anchor_on_timeout=hold_anchor)
        if self.mode_3_active:
            self.command.mode='force'
            self.command.forces_n=g_to_n(self.last_command_force_g,self.config.get('direction',(1.,)*6),
                                         self.config.get('newtons_per_g',(NOMINAL_N_PER_G,)*6))
            self.command.consumed=True  # no incoming target yet: hold, never zero

        return dict(type='ready', epoch=self.config['epoch'], pid=os.getpid(),
                    parent_pid=os.getppid(),positions=self.positions, currents=self.currents, faults=self.faults, rx_ns=self.rx_ns,
                    modules_without_ros=not any(x=='rclpy' or x.startswith(('rclpy.','pus6_msgs','sensor_msgs')) for x in sys.modules),
                    serial_write_timeout_s=.030, serial_read_timeout_s=.005,
                    control_mode=(0 if self.config.get('maintenance_only', False) else
                                  3 if self.mode_3_active else 1),
                    force_handover_snapshot=self.force_handover_snapshot)

    def prepare_force_handover(self) -> dict:
        """Capture current signed force and position using only the existing UART owner.

        No mode/force writes. This is a relative load reference, NOT sensor tare.
        Tests use an injected FakeBus. Only a later authorized call can switch modes.
        """
        if self.fault_latched or self.fault is not None:
            raise RuntimeError("Cannot prepare handover: fault is latched")
        if self.config.get('dry_run', False):
            raise RuntimeError("dry-run must supply explicit baseline; no live capture")
        rows = self.writer.read_force_handover_state()
        modes=[row['mode'] for row in rows]
        if modes != [0]*6 and not (self.config.get('mode3_only') and modes == [3]*6):
            raise SafetyViolation("current-state force handover requires uniform Mode 0 or explicit Mode3-only attachment")
        forces = [row['force_g_raw'] for row in rows]
        # A previous hardware Tare does not guarantee zero force NOW. Generic
        # current-state entry always holds the current signed 0x2C readings.
        hardware_reference = getattr(self, 'force_feedback_reference', None) == 'hardware_tare_assembly_reference'
        baseline = tuple(forces) if hardware_reference else self.baseline_manager.capture_initial_state(forces)
        self.forces_g_raw = list(baseline)
        self.positions = [row['position_steps'] for row in rows]
        rel_g = [f-b for f,b in zip(forces,baseline)]
        self.force_handover_snapshot = dict(
            source='MOCK' if self.config['hardware_mode'] == 'mock' else 'HARDWARE',
            axes=rows, baseline_g=list(baseline),
            holding_forces_g=list(baseline),
            relative_forces_g=rel_g, software_baseline_subtracted=not hardware_reference)
        return self.force_handover_snapshot

    def _force_tare_interface(self):
        from pus6_force_control.hardware_tare import HardwareTareController
        if not hasattr(self, 'hardware_tare'):
            self.hardware_tare = HardwareTareController(
                self.bus, self.writer.uart_lock,
                hardware_write_enabled=self.config.get('hardware_write_enabled', False),
                hardware_qualified=self.config.get('is_hardware_qualified', False),
                dry_run=self.config.get('dry_run', False),
                event=lambda event, row: self.lifecycle.append(dict(event=event, ns=time.monotonic_ns(), data=row)))
        self.hardware_tare.hardware_write_enabled = self.config.get('hardware_write_enabled', False)
        self.hardware_tare.hardware_qualified = self.config.get('is_hardware_qualified', False)
        self.hardware_tare.dry_run = self.config.get('dry_run', False)
        return self.hardware_tare

    def tare_force_axis(self, motor_id, **settings):
        if self.fault_latched or self.fault is not None or self.mode_3_active:
            raise RuntimeError('Tare rejected in active force mode or latched fault')
        self.check_hardware_write_permission()
        try:
            result = self._force_tare_interface().tare_axis(motor_id, **settings)
            # Do not subtract the pre-Tare load again from device-tared feedback.
            self.baseline_manager.software_baseline_g = None
            self.force_feedback_reference = 'hardware_tare_assembly_reference'
            return result
        except Exception as exc:
            self.writer.latch_fault_without_pause(f'tare_failure: {exc}')
            self.stop(f'tare_failure: {exc}')
            raise

    def tare_all_force_axes(self, **settings):
        return [self.tare_force_axis(mid, **settings) for mid in MOTOR_IDS]

    def verify_completed_tare_axis(self, prior, **settings):
        if self.fault_latched or self.mode_3_active:
            raise RuntimeError('Cannot reconcile Tare in fault/force mode')
        try:
            result = self._force_tare_interface().verify_completed_axis(prior, **settings)
        except Exception as exc:
            self.writer.latch_fault_without_pause(f'tare_confirmation_failure: {exc}')
            self.stop(f'tare_confirmation_failure: {exc}')
            raise
        self.baseline_manager.software_baseline_g = None
        self.force_feedback_reference = 'hardware_tare_assembly_reference'
        return result

    def enter_force_mode_after_hardware_tare(self, *, axis_verifier=None):
        """Zero native target relative to the explicitly established device reference."""
        if self.fault_latched or self.fault is not None:
            raise RuntimeError('Cannot enter force mode: fault is latched')
        self.check_hardware_write_permission()
        tare = self._force_tare_interface()
        if any(row.state != 'confirmed' for row in tare.axes.values()):
            raise SafetyViolation('all six hardware Tare confirmations are required')
        current = self.writer.read_force_handover_state()
        tolerance = self.config.get('tare_tolerance_g', 50)
        if any(row['mode'] != 0 or abs(row['force_g_raw']) > tolerance for row in current):
            raise SafetyViolation('current mode/force no longer matches hardware Tare reference')
        pre_tare = [row.before_force_g for row in tare.axes.values()]
        self.force_handover_snapshot = dict(reference='hardware_tare_assembly_reference',
            axes=current, holding_forces_g=[0]*6, software_baseline_subtracted=False,
            pre_tare_forces_g=pre_tare)
        self.handover_state = ForceHandoverState.SWITCHING_MODE3
        try:
            self._force_control_managed = True
            self.force_shutdown_report = None
            self.writer.configure_force_mode([0]*6, axis_verifier=axis_verifier,
                before_recovery=self._enable_terminal_trace,
                allow_feedback_tolerance_anchors=self.config.get('allow_recovery_feedback_tolerance',False))
            self.force_entry_ns = time.monotonic_ns()
            self._capture_force_hold_reference([0]*6)
            self.last_command_force_g = [0]*6
            self.mode_3_active = True
            self.handover_state = ForceHandoverState.MODE3_FORCE_ACTIVE
        except Exception as exc:
            self.writer.latch_fault_without_pause(f'enter_tared_force_failure: {exc}')
            self.stop(f'enter_force_mode_failure: {exc}')
            raise

    def enter_force_mode_from_current_state(self):
        """Explicit read-current-state -> authorized nonzero signed holding handover."""
        snapshot = self.prepare_force_handover()
        self.enter_force_mode_with_handover(
            snapshot['holding_forces_g'], static_baseline_g=snapshot['baseline_g'],
            handover_tolerance_g=0.0)
        return snapshot

    def enter_force_mode_with_handover(
        self,
        initial_forces_g: Sequence[int] | None = None,
        *,
        static_baseline_g: Sequence[int] | None = None,
        handover_tolerance_g: float = 1000.0,
        axis_verifier=None,
        axis_preparer=None, axis_status_verifier=None,
        ):

        if self.fault_latched or self.fault is not None:
            raise RuntimeError(f"Cannot enter force mode: fault is latched ({self.fault})")
        if self.mode_3_active:
            return

        self.handover_state = ForceHandoverState.PREPARING_FORCE_HANDOVER
        validated_forces = validate_initial_handover_forces(
            initial_forces_g,
            static_baseline_g=(static_baseline_g if static_baseline_g is not None
                               else self.baseline_manager.software_baseline_g),
            handover_tolerance_g=handover_tolerance_g,
        )

        # A target accepted at entry must not be clipped to a different load by
        # the first ordinary hold cycle. Do not silently widen configured limits.
        direction = self.config.get('direction', (1.0,)*6)
        scales = self.config.get('newtons_per_g', (NOMINAL_N_PER_G,)*6)
        initial_n = g_to_n(validated_forces, direction, scales)
        limited_n = clamp_force_n(initial_n, self.config.get('min_force_n', -DEFAULT_FORCE_LIMIT_N),
                                 self.config.get('max_force_n', DEFAULT_FORCE_LIMIT_N))
        if any(abs(a-b) > 1e-10 for a,b in zip(initial_n, limited_n)):
            raise SafetyViolation("initial holding forces exceed configured force limits in N")
        is_dry_run = self.config.get('dry_run', False)
        if is_dry_run:
            self.mode_3_active = True
            self.handover_state = ForceHandoverState.MODE3_FORCE_ACTIVE
            self.last_command_force_g = list(validated_forces)
            self._capture_force_hold_reference(validated_forces)
            return

        if self.config['hardware_mode'] != 'mock':
            try:
                self.check_hardware_write_permission()
            except RuntimeError as exc:
                self.fault_latched = True
                self.handover_state = ForceHandoverState.FAULT_LATCHED
                raise

        if axis_verifier is None and self.config.get('max_force_handover_displacement_steps') is not None:
            axes=(self.force_handover_snapshot or {}).get('axes')
            if not axes:
                raise SafetyViolation('guarded handover requires current-state capture')
            anchors=[r['position_steps'] for r in axes]
            max_delta=self.config['max_force_handover_displacement_steps']
            def axis_verifier(_mid):
                current=self.writer.read_force_handover_state()
                if any(abs(r['position_steps']-q)>max_delta for r,q in zip(current,anchors)):
                    raise SafetyViolation('Mode 3 handover displacement exceeds existing guard')
        self.handover_state = ForceHandoverState.SWITCHING_MODE3
        def prepare_axis(mid):
            candidate = axis_preparer(mid)
            # Refreshing an axis must not bypass the same signed encoding,
            # calibration or absolute-force limits used for the initial vector.
            fresh = list(validated_forces)
            fresh[mid-1] = candidate
            fresh = validate_initial_handover_forces(fresh, static_baseline_g=None,
                                                     handover_tolerance_g=handover_tolerance_g)
            force_n = g_to_n(fresh, direction, scales)
            limited = clamp_force_n(force_n, self.config.get('min_force_n', -DEFAULT_FORCE_LIMIT_N),
                                    self.config.get('max_force_n', DEFAULT_FORCE_LIMIT_N))
            if any(abs(a-b)>1e-10 for a,b in zip(force_n, limited)):
                raise SafetyViolation('fresh holding forces exceed configured force limits in N')
            validated_forces[mid-1] = fresh[mid-1]
            return fresh[mid-1]
        try:
            self._force_control_managed = True
            self.force_shutdown_report = None
            self.writer.configure_force_mode(initial_forces_g=validated_forces, axis_verifier=axis_verifier,
                before_recovery=self._enable_terminal_trace,
                allow_feedback_tolerance_anchors=self.config.get('allow_recovery_feedback_tolerance',False),
                axis_preparer=prepare_axis if axis_preparer is not None else None,
                axis_status_verifier=axis_status_verifier)
            self.mode_3_active = True
            self.handover_state = ForceHandoverState.MODE3_FORCE_ACTIVE
            self.last_command_force_g = list(validated_forces)
            self._capture_force_hold_reference(validated_forces)
            self.force_entry_ns = time.monotonic_ns()
        except Exception as exc:
            self.fault_latched = True
            self.handover_state = ForceHandoverState.FAULT_LATCHED
            self.writer.latch_fault_without_pause(f"enter_force_mode_failure: {exc}")
            self.stop(f"enter_force_mode_failure: {exc}")
            raise

    def enter_force_mode(self, initial_force_g: int | Sequence[int] | None = None):
        if self.mode_3_active:
            return
        if (initial_force_g is None and self.config.get('initial_force_g') is None
                and getattr(self, 'force_feedback_reference', None) == 'hardware_tare_assembly_reference'):
            return self.enter_force_mode_after_hardware_tare()
        if initial_force_g is not None:
            if isinstance(initial_force_g, (list, tuple)):
                forces = list(initial_force_g)
            else:
                forces = [int(round(initial_force_g))] * 6
        else:
            init_cfg = self.config.get('initial_force_g')
            if init_cfg is not None:
                if isinstance(init_cfg, (list, tuple)):
                    forces = list(init_cfg)
                else:
                    forces = [int(round(init_cfg))] * 6
            elif self.config.get('static_baseline_g'):
                forces = list(self.config.get('static_baseline_g'))
            else:
                return self.enter_force_mode_from_current_state()

        baseline = (self.config.get('static_baseline_g')) if self.config.get('check_static_baseline', False) else None
        tol = self.config.get('handover_tolerance_g', 1000.0)
        self.enter_force_mode_with_handover(forces, static_baseline_g=baseline, handover_tolerance_g=tol)

    def exit_force_mode_safe(self, current_positions: Sequence[int] | None = None):
        """Terminal handover always reads fresh positions, never cached anchors."""
        if not self.mode_3_active:
            return
        if self.config.get('dry_run', False):
            self.mode_3_active = False
            self.handover_state = ForceHandoverState.MODE0_POSITION
            return
        result = self.finish_force_control('normal_exit')
        if not result or not result['position_hold_confirmed']:
            from .j7_servo_transport import ServoTransportError
            raise ServoTransportError('position handover incomplete; inspect force_shutdown report')
        self.positions = [a['anchor_steps'] for a in result['axes']]
        return list(self.positions)

    def exit_force_mode(self, use_position_anchor: bool = False):
        if not self.mode_3_active:
            return
        if use_position_anchor:
            self.exit_force_mode_safe()
        else:
            return self.pause_explicit("exit_force_mode")

    def pause_explicit(self, reason='explicit_pause'):
        """Explicit Pause remains available after terminal recovery or a fault."""
        self.check_hardware_write_permission()
        self._enable_terminal_trace()
        self._force_control_managed = True
        self.writer.latch_fault_without_pause(reason)
        audit = self.stop_adapter(reason)
        if self.force_shutdown_report is not None:
            # A new explicit Pause cannot reuse old position observations as
            # proof of its physical effect. Do not retry uncertain anchor writes.
            self.force_shutdown_report = dict(reason=reason,
                source='MOCK' if self.config['hardware_mode']=='mock' else 'REAL_HARDWARE',
                status='PAUSE_ACKNOWLEDGED_STOP_UNVERIFIED', axes=[],
                pause_requested=True, pause_acknowledged=bool(audit.success),
                position_hold_confirmed=False, hardware_stop_confirmed=False,
                independent_stop_required=True)
            (self.out/'force_shutdown.json').write_text(json.dumps(self.force_shutdown_report,indent=2)+'\n')
            return self.force_shutdown_report
        return self.finish_force_control(reason, fault=True)

    def _capture_force_hold_reference(self, values):
        self.force_hold_g=tuple(quantize_force_g(values))
        self.force_hold_reference_ns=time.monotonic_ns()
        if hasattr(self,'command') and self.config.get('hold_anchor_on_timeout'):
            self.command.fallback_force_n=g_to_n(self.force_hold_g,self.config.get('direction',(1.,)*6),
                self.config.get('newtons_per_g',(NOMINAL_N_PER_G,)*6))

    def force_feedback_fields(self):
        """Complete force payload from the same per-axis status/timestamps as position."""
        return dict(forces_g_raw=list(self.forces_g_raw), force_adc_raw=list(self.force_adc_raw),
                    temperatures_c=list(self.temperatures_c), force_valid=list(self.force_valid),
                    feedback_source=1 if self.config['hardware_mode'] == 'mock' else 2,
                    force_mode_confirmed=self.mode_3_active and not self.config.get('dry_run',False),
                    force_hold_valid=self.force_hold_g is not None,
                    force_hold_g=list(self.force_hold_g or (0,)*6),
                    force_hold_reference_monotonic_ns=self.force_hold_reference_ns)

    def finish_force_control(self, reason='normal_exit', *, fault=False, priority_motor_ids=(), reanchor_mode0=False):
        """One terminal transaction sequence, including after a latched fault.

        Read-only attachment is deliberately never promoted to write ownership.
        """
        if self.force_shutdown_report is not None:
            return self.force_shutdown_report
        if not self._force_control_managed:
            return None
        self.writer.latch_fault_without_pause(reason)
        try:
            self.check_hardware_write_permission()
            terminal_hook = getattr(self.serial, 'enable_terminal_recovery', None)
            if terminal_hook is not None:
                terminal_hook()
            from .force_lifecycle import finish_force_control
            report = finish_force_control(self.writer, reason=reason, pause=self.stop_adapter,
                pause_first=fault and not reason.startswith('enter_force_mode_failure'),
                allow_pause_fallback=not reason.startswith('enter_force_mode_failure'),
                max_current_ma=self.config.get('max_current_ma', 500),
                priority_motor_ids=priority_motor_ids,
                reanchor_mode0=reanchor_mode0,
                allow_feedback_tolerance_anchors=self.config.get('allow_recovery_feedback_tolerance',False),
                skip_anchor_ids=tuple(a['motor_id'] for a in
                    (getattr(self.writer, 'last_force_handover', None) or {}).get('axes', ())
                    if a.get('recovery_attempted') and a.get('error') and a.get('final_mode') != 0))
        except Exception as exc:
            report = dict(reason=reason, status='STOP_UNCONFIRMED', error=str(exc),
                axes=[], hardware_stop_confirmed=False, position_hold_confirmed=False,
                independent_stop_required=True)
        report['source'] = 'MOCK' if self.config['hardware_mode'] == 'mock' else 'REAL_HARDWARE'
        if report['source'] == 'MOCK':
            report['hardware_stop_confirmed'] = False
        self.force_shutdown_report = report
        self.mode_3_active = False
        self.fault_latched = True
        self.handover_state = (ForceHandoverState.MODE0_POSITION if report['position_hold_confirmed']
                               else ForceHandoverState.FAULT_LATCHED)
        self.lifecycle.append(dict(event='force_terminal', ns=time.monotonic_ns(), **report))
        (self.out/'force_shutdown.json').write_text(json.dumps(report, indent=2)+'\n')
        return report

    def stop(self, reason):
        if not self.fault:
            self.fault = reason
        self.fault_latched = True
        if not self.motion_stop_ns:
            self.motion_stop_ns = time.monotonic_ns()  # request time, not proof of physical stopping
        if self.config.get('control_mode') == 'force':
            normal = reason in ('normal_exit', 'shutdown', 'parent_shutdown', 'window_complete',
                                'service_pause', 'signal_stop', 'stage5_finished', 'manual_recovery_finished')
            self.finish_force_control(reason, fault=not normal)
            self.writer.latch_fault_without_pause(reason)
            self.mode_3_active = False
        else:
            self.handover_state = ForceHandoverState.FAULT_LATCHED
            self.mode_3_active = False
            self.writer.stop(reason)

    def reset_fault_latch(self):
        report = getattr(self.writer, 'last_force_handover', None)
        if report and report['status'] == 'recovery_incomplete':
            raise RuntimeError("Cannot reset fault: mode recovery is incomplete; inspect per-axis handover report")
        if self.force_shutdown_report and not self.force_shutdown_report['position_hold_confirmed']:
            raise RuntimeError('terminal recovery unconfirmed; no automatic resume')
        self.fault = None
        self.fault_latched = False
        self.handover_state = ForceHandoverState.MODE0_POSITION

    def cycle(self, expected_ns, hold_for_ipc_backlog=False):
        start=time.monotonic_ns(); tc=time.thread_time_ns(); pc=time.process_time_ns()
        ru=resource.getrusage(resource.RUSAGE_THREAD)
        self.rec.start_cycle(self.cycle_seq,expected_start_ns=expected_ns)
        audit=None
        if self.fault_latched or self.fault is not None:
            raise RuntimeError(f"Command cycle rejected: fault is latched ({self.fault})")
        is_force_cycle = (self.config.get('control_mode') == 'force' or self.command.mode == 'force')
        try:
            if is_force_cycle:
                if not self.mode_3_active:
                    self.enter_force_mode()
                if (self.command.command_ns == 0 and start-getattr(self, 'force_entry_ns', start) > self.command.timeout_ns):
                    raise RuntimeError('force_target_timeout: no initial publisher command')
                mode_period=self.config.get('force_mode_check_period_s')
                if mode_period and start-getattr(self,'last_force_mode_check_ns',0)>=mode_period*1e9:
                    with self.writer.uart_lock:
                        modes=[self.writer._read_register_locked(mid,CONTROL_MODE_ADDR) for mid in MOTOR_IDS]
                    self.last_force_mode_check_ns=time.monotonic_ns()
                    if modes!=[3]*6:
                        self.writer.latch_fault_without_pause('native_force_mode_lost')
                        raise RuntimeError('native Force mode readback lost: '+str(modes))
                direction = self.config.get('direction', (1.0,)*6)
                anchor_g = self.last_command_force_g or [0] * 6
                scales = self.config.get('newtons_per_g', (NOMINAL_N_PER_G,)*6)
                anchor_n = g_to_n(anchor_g, direction, scales)
                cycle_time_ns = max(start, expected_ns) if expected_ns else start
                candidate_n = self.command.candidate_force(
                    anchor_n,
                    hold_for_ipc_backlog or self.feedback.is_feedback_stale_warning(cycle_time_ns),
                    cycle_time_ns,
                )
                self.rec.mark_target_lock_done()
                candidate_n = clamp_force_n(candidate_n, self.config.get('min_force_n', -DEFAULT_FORCE_LIMIT_N),
                                             self.config.get('max_force_n', DEFAULT_FORCE_LIMIT_N))
                candidate_g = n_to_g(candidate_n, direction, scales)
                max_rate_g_s = self.config.get('max_rate_g_s', 500.0)
                dispatched = dispatch_force_slew(
                    candidate_g, anchor_g, max_rate_g_s, self.last_write_ns, cycle_time_ns
                )
                dispatched = quantize_force_g_with_n_limits(
                    dispatched, self.config.get('min_force_n', -DEFAULT_FORCE_LIMIT_N),
                    self.config.get('max_force_n', DEFAULT_FORCE_LIMIT_N), direction, scales)
                self.rec.mark_slew_done()
                if not self.config.get('allow_negative_forces', True) and any(f < 0 for f in dispatched):
                    raise RuntimeError("negative force requested but allow_negative_forces is False")
                if any(self.faults):
                    raise RuntimeError("cached motor fault")
                is_dry_run = self.config.get('dry_run', False)
                if is_dry_run:
                    self.rec.mark_preflight_done()
                    self.rec.mark_transport_entry()
                    audit = self._simulate_dry_run_audit(dispatched, cycle_id=self.cycle_seq, timestamp_ns=start, is_force=True)
                    self.last_command_force_g = list(dispatched)
                    self.last_write_ns = cycle_time_ns
                else:
                    if self.config['hardware_mode'] != 'mock':
                        if not self.config.get('hardware_write_enabled', False):
                            raise RuntimeError("hardware writes blocked: hardware_write_enabled=False")
                        if not self.config.get('is_hardware_qualified', False):
                            raise RuntimeError("hardware writes blocked: is_hardware_qualified=False")
                    self.rec.mark_preflight_done()
                    self.rec.mark_transport_entry()
                    audit = self.writer.send_force_cycle(dispatched, cycle_id=self.cycle_seq, target_timestamp_ns=start)
                    self.last_command_force_g = list(dispatched)
                    self.last_write_ns = cycle_time_ns
            else:
                anchor=self.last_command or self.positions
                candidate=self.command.candidate(anchor,hold_for_ipc_backlog or self.feedback.is_feedback_stale_warning(start))
                self.rec.mark_target_lock_done()
                dispatched=dispatch_slew(candidate, self.last_command or candidate,self.last_write_ns,start)
                self.rec.mark_slew_done()
                validate_target_vector(dispatched, limits=None if self.config.get('maintenance_mode',True) else {mid:(200,1800) for mid in MOTOR_IDS})
                if self.last_command and max(abs(a-b) for a,b in zip(dispatched,self.last_command))>20:
                    raise RuntimeError('inter-frame delta >20')
                if any(self.faults):
                    raise RuntimeError('cached motor fault')
                is_dry_run = self.config.get('dry_run', False)
                if is_dry_run:
                    self.rec.mark_preflight_done()
                    self.rec.mark_transport_entry()
                    audit = self._simulate_dry_run_audit(dispatched, cycle_id=self.cycle_seq, timestamp_ns=start, is_force=False)
                    self.last_command = list(dispatched)
                    self.last_write_ns = time.monotonic_ns()
                else:
                    if self.config['hardware_mode'] != 'mock':
                        if not self.config.get('hardware_write_enabled', False):
                            raise RuntimeError("hardware writes blocked: hardware_write_enabled=False")
                        if not self.config.get('is_hardware_qualified', False):
                            raise RuntimeError("hardware writes blocked: is_hardware_qualified=False")
                    self.rec.mark_preflight_done()
                    self.rec.mark_transport_entry()
                    audit=self.writer.send_cycle(dispatched,cycle_id=self.cycle_seq,target_timestamp_ns=start)
                    self.last_command=list(dispatched)
                    self.last_write_ns=time.monotonic_ns()
            self.rec.mark_transport_exit(audit); self.rec.mark_ingest_start()
            fb=self.feedback.ingest_cycle_feedback(audit.feedbacks,servo_cycle_id=self.cycle_seq,
                                                   servo_cycle_timestamp_ns=start,
                                                   per_axis_rx_done_ns=audit.feedback_rx_done_ns)
            self.rec.mark_ingest_done(fb.scheduler_gap_ns/1e6)
            self.update(fb)  # preserve status/force/fault even when the scheduler stopped
            if is_force_cycle and (any(p < 0 or p > 2000 for p in self.positions)
                    or not all(self.force_valid) or any(self.faults)):
                raise RuntimeError('invalid native-force position/force/fault feedback')
            if self.feedback.stopped or self.writer.stopped:
                raise RuntimeError(self.feedback.audit_snapshot().stop_reason or 'transport stopped')
            self.rec.mark_unpack_done()
            end=time.monotonic_ns()
            direction_out = self.config.get('direction', (1.0,)*6)
            packet=dict(type='feedback',epoch=self.config['epoch'],cycle_seq=self.cycle_seq,
                command_seq=self.command.seq, command_publish_ns=self.command.command_ns,
                command_receive_ns=self.command.receive_ns,cycle_start_ns=start,cycle_end_ns=end,
                feedback_ready_ns=end, positions=list(self.positions),currents=list(self.currents),
                faults=list(self.faults),rx_ns=list(self.rx_ns), targets=dispatched,
                control_mode=3 if is_force_cycle else 1,
                dispatched_forces_g=list(dispatched) if is_force_cycle else None,
                dispatched_forces_n=g_to_n(dispatched, direction_out, self.config.get('newtons_per_g', (NOMINAL_N_PER_G,)*6)) if is_force_cycle else None,
                handover_state=self.handover_state.value,
                dry_run=bool(self.config.get('dry_run', False)),
                **self.force_feedback_fields(),
                tx_start_ns=[r.tx_start_ns for r in audit.tx_records],
                tx_end_ns=[r.tx_end_ns for r in audit.tx_records],
                tx_wall_ns=[r.duration_ns for r in audit.tx_records],
                rx_drain_ns=audit.motion_drain_wall_ns,
                feedback_age_ns=[end-n for n in self.rx_ns],
                feedback_complete=len(audit.feedbacks)==6,
                scheduler_gap_ns=fb.scheduler_gap_ns, communication_fault=None,
                rejected_commands=self.command.rejected)
            self.last_packet=packet
            self.rec.mark_telemetry_done()
            self.rec.finish_cycle(feedback_age_max_ms=max((s.feedback_age_ns/1e6 for s in fb.samples),default=0))
        except Exception as exc:
            self.stop(f'{type(exc).__name__}: {exc}')
            failed=getattr(exc,'cycle_audit',None)
            if failed is not None:
                self.rec.mark_transport_exit(failed)
            self.rec.record_fault_cycle(self.fault,fault_phase='child_local_fault')
            self.rec.finish_cycle(fault_reason=self.fault)
            self.force_valid = [False]*6  # cached values remain observable, never current/valid
            fault_ns = time.monotonic_ns()
            packet=dict(type='fault',epoch=self.config['epoch'],cycle_seq=self.cycle_seq,
                        communication_fault=self.fault, feedback_ready_ns=fault_ns,
                        force_shutdown_report=self.force_shutdown_report,
                handover_report=getattr(self.writer, 'last_force_handover', None),
                        cycle_start_ns=start, cycle_end_ns=fault_ns,
                        positions=list(self.positions), currents=list(self.currents),
                        faults=list(self.faults), rx_ns=list(self.rx_ns),
                        **self.force_feedback_fields(),
                        failed_audit=asdict(failed) if failed else None)
            self.last_packet=packet
        # Keep all strict write scalar measurements, not just the legacy 6-second ring.
        if locals().get('failed') is not None:
            audit = failed
        if audit is not None:
            for r in audit.tx_records:
                m=r.serial_write_metric
                self.health['motion_writes']+=1
                self.health['tx_timeouts']+=int(r.error=='SerialTimeoutException')
                if m is not None:
                    self.health['max_serial_write_ns']=max(self.health['max_serial_write_ns'],m.wall_duration_ns)
                if self.production and self.telemetry_mode=='production' and not r.error and (m is None or m.wall_duration_ns<1_000_000):
                    continue  # recent full TX evidence remains in the existing bounded recorder
                self.write_rows.append(dict(cycle_seq=r.cycle_id,motor_id=r.motor_id,
                    wrapper_wall_ns=r.duration_ns,wrapper_thread_cpu_ns=r.cpu_duration_ns,
                    error=r.error,serial_metric=asdict(m) if m is not None else None))
        self.health['cycles']+=1
        self.health['fault_cycles']+=int(self.fault is not None)
        self.health['max_wake_lateness_ns']=max(self.health['max_wake_lateness_ns'],start-expected_ns)
        self.health['wake_watchlist_count']+=int(start-expected_ns>30_000_000)
        if audit is not None:self.health['max_rx_drain_ns']=max(self.health['max_rx_drain_ns'],audit.motion_drain_wall_ns)
        end=time.monotonic_ns(); after=resource.getrusage(resource.RUSAGE_THREAD)
        self.rows.append(dict(cycle_seq=self.cycle_seq, expected_ns=expected_ns,start_ns=start,end_ns=end,
            wake_lateness_ns=start-expected_ns, thread_cpu_ns=time.thread_time_ns()-tc,
            process_cpu_ns=time.process_time_ns()-pc,voluntary_cs=after.ru_nvcsw-ru.ru_nvcsw,
            involuntary_cs=after.ru_nivcsw-ru.ru_nivcsw,
            command_seq=self.command.seq, command_publish_ns=self.command.command_ns,
            command_receive_ns=self.command.receive_ns, fault=self.fault))
        self.cycle_seq+=1
        return packet

    def close(self, reason='shutdown'):
        if self.closed:
            return
        try:
            if self.config.get('control_mode') == 'force':
                self.stop(reason)
            else:
                self.writer.stop(reason)
        finally:
            if hasattr(self, 'port_lock') and self.port_lock is not None:
                self.port_lock.release()
                self.port_lock = None
            if self.serial is not None:
                self.serial.close()
            else:
                getattr(self.bus, 'serial', self.bus).close()
            self.closed=True
            self.lifecycle.append(dict(event='serial_closed',ns=time.monotonic_ns()))
            self.rec.dump()
            if self.production:
                self.health.update(telemetry_mode=self.telemetry_mode,retained_cycles=len(self.rows),
                    retained_write_records=len(self.write_rows),closed=True)
                (self.out/'child_health_summary.json').write_text(json.dumps(self.health,indent=2)+'\n')
            append_rows(self.out/'child_cycles.jsonl',self.rows)
            append_rows(self.out/'child_write_metrics.jsonl',self.write_rows)
            append_rows(self.out/'child_warnings.jsonl',list(self.warnings))
            append_rows(self.out/'child_lifecycle.jsonl',self.lifecycle)
            if self.last_packet and self.last_packet['type']=='fault':
                append_rows(self.out/'child_fault.jsonl',[self.last_packet])
            (self.out/'child_final.json').write_text(json.dumps(dict(pid=os.getpid(),cycles=self.cycle_seq,
                fault=self.fault,closed=self.closed,servo_start_ns=self.servo_start_ns,
                force_shutdown_report=self.force_shutdown_report,
                handover_report=getattr(self.writer, 'last_force_handover', None),
                force_handover_snapshot=self.force_handover_snapshot,
                hardware_tare=getattr(self, 'hardware_tare', None).snapshot() if hasattr(self, 'hardware_tare') else None,
                motion_stop_ns=self.motion_stop_ns, hardware_write_count=0 if self.config['hardware_mode']=='mock' else None,
                pause_success=bool(self.pauses) and self.pauses[-1]['success']),indent=2)+'\n')


def child_run(sock, config, bus=None, fb_sock=None):
    core=None; signal_stop=[False]
    out_fb = fb_sock if fb_sock is not None else sock
    # Only actual child main thread installs handlers (thread A/B calls this too).
    if threading.current_thread() is threading.main_thread():
        for sig in (signal.SIGINT,signal.SIGTERM):
            signal.signal(sig,lambda *_: signal_stop.__setitem__(0,True))
    try:
        if config.get('joint_position_pid'):
            from pus6_force_control.mode3_process_child import JointPositionServoCore
            core=JointPositionServoCore(config,bus)
        else:
            core=ServoCore(config,bus)
        ready=core.initialize()
        t_child_ready_ns = time.monotonic_ns()
        ready['t_child_ready_ns'] = t_child_ready_ns
        ready['type'] = 'child_ready'
        if not send_packet(sock,ready):
            raise RuntimeError('startup IPC full')
        if out_fb is not sock:
            send_packet(out_fb, ready)

            # J14-A1.13 Coordinated Three-Process Handshake
            parent_ready = False
            bridge_ready = False
            t_bridge_ready_ns = None
            t_parent_ready_ns = None
            handshake_timeout_s = config.get('handshake_timeout_s', 5.0)
            handshake_deadline = time.monotonic() + handshake_timeout_s

            while not (parent_ready and bridge_ready) and not signal_stop[0]:
                remaining = handshake_deadline - time.monotonic()
                if remaining <= 0:
                    core.stop('startup_handshake_timeout')
                    fault_pkt = dict(
                        type='fault',
                        epoch=config['epoch'],
                        communication_fault='startup_handshake_timeout',
                    )
                    send_packet(sock, fault_pkt)
                    send_packet(out_fb, fault_pkt)
                    return

                rlist, _, _ = select.select([sock, out_fb], [], [], min(0.020, max(0.001, remaining)))
                if sock in rlist:
                    rows, _ = receive_packets(sock)
                    for row in rows:
                        if row.get('type') in ('start', 'state') and row.get('allowed'):
                            parent_ready = True
                            t_parent_ready_ns = row.get('t_parent_ready_ns') or time.monotonic_ns()
                            core.command.ingest([row], time.monotonic_ns(), core.last_command or core.positions)
                        elif row.get('type') == 'stop':
                            core.stop(row.get('reason', 'parent_stop'))
                            return
                        elif row.get('type') == 'fault':
                            core.stop(row.get('communication_fault', 'parent_fault'))
                            return
                if out_fb in rlist:
                    rows, _ = receive_packets(out_fb)
                    for row in rows:
                        if row.get('type') == 'bridge_ready':
                            bridge_ready = True
                            t_bridge_ready_ns = row.get('t_bridge_ready_ns') or time.monotonic_ns()
                            send_packet(sock, dict(
                                type='bridge_ready',
                                epoch=config['epoch'],
                                pid=row.get('pid'),
                                t_bridge_ready_ns=t_bridge_ready_ns,
                            ))
                        elif row.get('type') == 'fault':
                            core.stop(row.get('communication_fault', 'bridge_fault'))
                            return

            if signal_stop[0]:
                core.stop('signal_stop')
                return

            servo_start_ns = time.monotonic_ns()
            ready['servo_start_ns'] = servo_start_ns
            core.servo_start_ns = servo_start_ns
            start_packet = dict(
                type='servo_start',
                epoch=config['epoch'],
                servo_start_ns=servo_start_ns,
                t_child_ready_ns=t_child_ready_ns,
                t_bridge_ready_ns=t_bridge_ready_ns,
                t_parent_ready_ns=t_parent_ready_ns,
            )
            send_packet(sock, start_packet)
            send_packet(out_fb, start_packet)
            append_rows(core.out / 'child_ready.jsonl', [ready, start_packet])
            deadline = servo_start_ns
        else:
            deadline=time.monotonic_ns()
            ready['servo_start_ns']=deadline
            core.servo_start_ns=deadline
            append_rows(core.out/'child_ready.jsonl',[ready])

        duration=config.get('max_servo_duration_s',60)
        if config.get('production'):
            if duration<0:raise ValueError('production duration must be nonnegative')
            trial_end_ns=deadline+int(duration*1e9) if duration else None
        else:
            if not 0<duration<=60: raise ValueError('A1.9 duration must be <=60 s')
            trial_end_ns=deadline+int(duration*1e9)
        flooded=False
        bridge_health_ns=time.monotonic_ns()
        while not signal_stop[0]:
            if config.get('joint_position_pid') and out_fb is not sock:
                try:bridge_rows,_=receive_packets(out_fb)
                except (EOFError,BrokenPipeError,ConnectionResetError):
                    core.stop('feedback_ipc_closed');break
                for row in bridge_rows:
                    if row.get('epoch')!=config['epoch']:raise ValueError('bridge epoch mismatch')
                    if row.get('type')=='motion_status':
                        bridge_health_ns=time.monotonic_ns()
                        send_packet(sock,row)
                if time.monotonic_ns()-bridge_health_ns>LEASE_NS:
                    core.stop('feedback_bridge_lease_expired');break
            rows,flooded=receive_packets(sock)
            core.command.ingest(rows,time.monotonic_ns(),core.last_command or core.positions)
            if core.command.stop_reason:
                core.stop(core.command.stop_reason); break
            if time.monotonic_ns()-core.command.lease_ns>LEASE_NS:
                core.stop('parent_lease_expired'); break
            now=time.monotonic_ns()
            if trial_end_ns is not None and now>=trial_end_ns:
                core.stop('window_complete'); break
            if now<deadline:
                select.select([sock],[],[],(deadline-now)/1e9)
                continue
            # Bounded receive work cannot starve UART. Hold last safe vector if
            # a backlog remains; admit its newest vector only after drain completes.
            packet=core.cycle(deadline,hold_for_ipc_backlog=flooded)
            try:
                send_packet(out_fb,packet)  # bounded, no waiting for feedback consumer
                if out_fb is not sock and packet.get('type') == 'fault':
                    send_packet(sock, packet)
            except (BrokenPipeError,ConnectionResetError,EOFError):
                core.stop('feedback_ipc_closed' if out_fb is not sock else 'parent_ipc_closed'); break
            if core.fault:
                break
            deadline+=PERIOD_NS
            now=time.monotonic_ns()
            if now-deadline>PERIOD_NS:
                deadline=now
    except (EOFError,BrokenPipeError,ConnectionResetError) as exc:
        if core: core.stop('parent_ipc_closed')
    except Exception as exc:
        if core:
            core.stop(f'child_exception: {type(exc).__name__}: {exc}')
        else:
            Path(config['output_dir']).mkdir(parents=True,exist_ok=True)
            append_rows(Path(config['output_dir'])/'child_start_failure.jsonl',[dict(error=repr(exc))])
    finally:
        if core: core.close()
        sock.close()
        if out_fb is not sock:
            out_fb.close()
        if core:
            append_rows(core.out/'child_lifecycle.jsonl',[dict(event='child_exit',ns=time.monotonic_ns())])


class ProcessClient:
    """ROS-independent parent endpoint; no serial factory or fallback UART path."""
    max_feedback_age_ns=LEASE_NS
    def __init__(self, config):
        self.config=config
        self.use_feedback_bridge = bool(
            config.get('feedback_bridge') or not config.get('production') and os.environ.get('PUS6_J14_FEEDBACK_BRIDGE') == '1'
        )
        self.sock,child_sock=socket_pair()
        self.bridge_proc = None
        self.bridge_stopped = False

        # Section XIV: Capture actual socket buffer values without altering them
        buffer_audit = {
            'command_channel': {
                'parent_so_sndbuf': self.sock.getsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF),
                'parent_so_rcvbuf': self.sock.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF),
                'child_so_sndbuf': child_sock.getsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF),
                'child_so_rcvbuf': child_sock.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF),
            }
        }

        if self.use_feedback_bridge:
            fb_child_sock, fb_bridge_sock = socket_pair()
            buffer_audit['feedback_channel'] = {
                'child_so_sndbuf': fb_child_sock.getsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF),
                'child_so_rcvbuf': fb_child_sock.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF),
                'bridge_so_sndbuf': fb_bridge_sock.getsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF),
                'bridge_so_rcvbuf': fb_bridge_sock.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF),
            }
            out_dir = Path(config['output_dir'])
            out_dir.mkdir(parents=True, exist_ok=True)
            (out_dir / 'socket_buffer_audit.json').write_text(json.dumps(buffer_audit, indent=2) + '\n')

            argv_child = [
                sys.executable, str(Path(__file__).with_name('j14_uart_child_main.py')),
                '--fd-cmd', str(child_sock.fileno()),
                '--fd-fb', str(fb_child_sock.fileno()),
                '--config', json.dumps(config)
            ]
            try:
                self.proc = subprocess.Popen(argv_child, pass_fds=(child_sock.fileno(), fb_child_sock.fileno()), start_new_session=True)
            finally:
                child_sock.close()
                fb_child_sock.close()

            bridge_entry = (['-m','pus6_force_control.mode3_process_bridge'] if config.get('joint_position_pid') else
                            [str(Path(__file__).with_name('j14_feedback_bridge_main.py'))])
            argv_bridge = [
                sys.executable, *bridge_entry,
                '--fd', str(fb_bridge_sock.fileno()),
                '--config', json.dumps(config)
            ]
            try:
                self.bridge_proc = subprocess.Popen(argv_bridge, pass_fds=(fb_bridge_sock.fileno(),), start_new_session=True)
            finally:
                fb_bridge_sock.close()
        else:
            out_dir = Path(config['output_dir'])
            out_dir.mkdir(parents=True, exist_ok=True)
            (out_dir / 'socket_buffer_audit.json').write_text(json.dumps(buffer_audit, indent=2) + '\n')

            argv = [
                sys.executable, str(Path(__file__).with_name('j14_uart_child_main.py')),
                '--fd', str(child_sock.fileno()),
                '--config', json.dumps(config)
            ]
            try:
                self.proc = subprocess.Popen(argv, pass_fds=(child_sock.fileno(),), start_new_session=True)
            finally:
                child_sock.close()

        self.lock=threading.Lock()
        self.latest=None; self.latest_force=None; self.seq=0; self.terminal=None
        self.stopped=False; self.stop_reason=None
        self.rows=deque(maxlen=config['telemetry_capacity_cycles']) if config.get('production') else []; self.last_lease_sent_ns=0; self.closed=False
        self.ready=None
        self.child_ready=None
        self.bridge_ready=None
        self.servo_started=None
        handshake_timeout_s = config.get('handshake_timeout_s', 5.0)
        until = time.monotonic() + handshake_timeout_s
        while time.monotonic() < until:
            if not self.use_feedback_bridge:
                self.send_state()
                for row in self.receive():
                    if row.get('type') in ('ready', 'child_ready'): self.ready = row
                    elif row.get('type') == 'fault': self.stopped = True; self.stop_reason = row['communication_fault']
                if self.ready: return
                if self.stopped: break
            else:
                for row in self.receive():
                    if row.get('type') in ('ready', 'child_ready'):
                        self.child_ready = row
                        self.ready = row
                    elif row.get('type') == 'bridge_ready':
                        self.bridge_ready = row
                    elif row.get('type') == 'servo_start':
                        self.servo_started = row
                    elif row.get('type') == 'fault':
                        self.stopped = True
                        self.stop_reason = row.get('communication_fault')
                if self.stopped:
                    break
                if self.child_ready and self.bridge_ready and not self.servo_started:
                    now = time.monotonic_ns()
                    send_packet(self.sock, dict(
                        type='start',
                        epoch=self.config['epoch'],
                        mode='static',
                        allowed=True,
                        lease_ns=now,
                        t_parent_ready_ns=now,
                        command=None,
                    ))
                if self.servo_started:
                    append_rows(Path(self.config['output_dir']) / 'parent_lifecycle.jsonl', [
                        dict(
                            event='startup_handshake_complete',
                            ns=time.monotonic_ns(),
                            child_ready_ns=self.child_ready.get('t_child_ready_ns'),
                            bridge_ready_ns=self.bridge_ready.get('t_bridge_ready_ns'),
                            servo_start_ns=self.servo_started.get('servo_start_ns'),
                        )
                    ])
                    return
            time.sleep(0.005)
        self.close()
        raise RuntimeError(f'UART child / bridge startup failed: {self.stop_reason or "handshake_timeout"}')

    def update(self, targets, publish_ns=None, seq=None):
        if len(targets)!=6 or any(type(v) is not int for v in targets):
            raise ValueError('whole six integer target required')
        with self.lock:
            if self.terminal or self.stopped: return
            self.latest=dict(seq=self.seq if seq is None else seq,targets=list(targets),
                             publish_ns=publish_ns or time.monotonic_ns())
            self.seq=max(self.seq+1,(seq+1) if seq is not None else 0)

    def update_force(self, forces_n, publish_ns=None, seq=None):
        if len(forces_n) != 6 or any(type(v) not in (int, float) or not math.isfinite(v) for v in forces_n):
            raise ValueError('whole six finite float forces required')
        with self.lock:
            if self.terminal or self.stopped: return
            self.latest_force = dict(seq=self.seq if seq is None else seq, forces_n=[float(v) for v in forces_n],
                                     publish_ns=publish_ns or time.monotonic_ns())
            self.seq = max(self.seq + 1, (seq + 1) if seq is not None else 0)

    def update_joint_reference(self, command):
        """Latest complete IK reference; forces are computed only by the Child."""
        with self.lock:
            if self.terminal or self.stopped:return
            self.latest_joint=dict(command,seq=self.seq)
            self.seq+=1

    def send_state(self):
        now=time.monotonic_ns()
        with self.lock:
            latest=self.latest
            latest_force=getattr(self, 'latest_force', None)
            latest_joint=getattr(self, 'latest_joint', None)
            if self.terminal:
                row=dict(type='stop',epoch=self.config['epoch'],reason=self.terminal)
            elif latest_joint is not None:
                row=dict(type='state',epoch=self.config['epoch'],mode='joint_position',allowed=True,
                         lease_ns=now,command=latest_joint)
            elif latest_force is not None:
                row=dict(type='state',epoch=self.config['epoch'],mode='force',allowed=True,
                         lease_ns=now,command=latest_force)
            elif latest is not None or now-self.last_lease_sent_ns>=100_000_000:
                row=dict(type='state',epoch=self.config['epoch'],mode='static',allowed=True,
                         lease_ns=now,command=latest)
            else: return
            try:
                sent=send_packet(self.sock,row)
            except (BrokenPipeError,ConnectionResetError,OSError):
                self.stopped=True; self.stop_reason='child_ipc_closed'; return
            if sent:
                if getattr(self,'latest_joint',None) is latest_joint:self.latest_joint=None
                if self.latest is latest: self.latest=None
                if getattr(self, 'latest_force', None) is latest_force: self.latest_force=None
                self.last_lease_sent_ns=now
                cmd_seq = (latest_joint['seq'] if latest_joint else
                           latest_force['seq'] if latest_force else (latest['seq'] if latest else None))
                self.rows.append(dict(event='parent_send',ns=now,process_cpu_ns=time.process_time_ns(),
                                      thread_cpu_ns=time.thread_time_ns(),
                                      seq=cmd_seq, mode=row.get('mode'),
                                      **(dict(command_publish_ns=latest_joint.get('publish_ns') if latest_joint else None,
                                              lease_ns=row.get('lease_ns')) if self.config.get('joint_position_pid') else {})))

    def receive(self):
        if self.closed: return []
        try:
            rows,_=receive_packets(self.sock)
        except (EOFError,ConnectionResetError,OSError):
            self.stopped=True; self.stop_reason=self.stop_reason or 'child_exit_or_EOF'
            final_path=Path(self.config['output_dir'])/'child_final.json'
            if final_path.exists():
                final=json.loads(final_path.read_text())
                self.stop_reason=final.get('fault') or self.stop_reason
            return []
        for row in rows:
            now=time.monotonic_ns(); row['parent_receive_ns']=now
            cpu=time.process_time_ns()
            tcpu=time.thread_time_ns()
            self.rows.append(dict(event='parent_receive',ns=now,process_cpu_ns=cpu,
                thread_cpu_ns=tcpu,
                cycle_seq=row.get('cycle_seq'),seq=row.get('command_seq'),type=row.get('type'),
                child_ready_ns=row.get('feedback_ready_ns'),command_publish_ns=row.get('command_publish_ns'),
                command_receive_ns=row.get('command_receive_ns')))
            if row.get('type')=='fault':
                self.stopped=True; self.stop_reason=row['communication_fault']
        if self.proc.poll() is not None:
            self.stopped=True; self.stop_reason=self.stop_reason or 'child_exited'
        bridge = getattr(self, 'bridge_proc', None)
        if bridge is not None and bridge.poll() is not None:
            self.bridge_stopped = True
        return rows

    def stop(self, reason):
        with self.lock: self.terminal=str(reason)
        self.send_state()

    def close(self):
        if self.closed: return
        if self.config.get('joint_position_pid') and self.terminal is not None:self.send_state()
        else:self.stop('parent_shutdown')
        # EOF is independent of a full parent->child buffer.
        try:
            if self.proc.poll() is None: self.sock.shutdown(socket.SHUT_WR)
        except OSError:
            pass
        try:
            self.proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            self.proc.terminate()
            self.proc.wait(timeout=3)
            self.rows.append(dict(event='shutdown_escalated_SIGTERM',ns=time.monotonic_ns()))
        if self.bridge_proc is not None:
            try:
                self.bridge_proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.bridge_proc.terminate()
                try:
                    self.bridge_proc.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    self.bridge_proc.kill()
                    self.bridge_proc.wait(timeout=1)
            append_rows(Path(self.config['output_dir'])/'parent_lifecycle.jsonl',[
                dict(event='bridge_joined',ns=time.monotonic_ns(),bridge_returncode=self.bridge_proc.returncode)])
        self.closed=True; self.sock.close()
        append_rows(Path(self.config['output_dir'])/'parent_ipc.jsonl',self.rows)
        append_rows(Path(self.config['output_dir'])/'parent_lifecycle.jsonl',[
            dict(event='child_joined',ns=time.monotonic_ns(),child_returncode=self.proc.returncode)])


class ReadOnlyDiagnosticSession:
    """Strictly pure read-only hardware diagnostic session for 6PUS motors ID1..ID6.

    Guarantees:
    1. ONLY sends CMD_RD_REGISTER (0x31) frames.
    2. Software-level interceptor: if any frame has command != CMD_RD_REGISTER (0x31)
       or register_addr is outside the explicitly selected read whitelist, it raises SafetyViolation
       before reaching serial.write().
    3. Acquires exclusive OS-level SerialPortLock, guaranteeing mutual exclusion with normal Child.
    4. Performs NO initialization writes (no mode write, no speed write, no force write, no limits write).
    5. On exit / close: cleanly closes the serial port without sending Pause (0x1A) or any control frame.
    """

    def __init__(
        self,
        port: str = "/dev/ttyUSB0",
        baudrate: int = 921600,
        timeout: float = 0.050,
        retries: int = 1,
        inter_frame_delay_s: float = 0.003,
        serial_instance = None,
        allowed_registers = None,
    ) -> None:
        self.allowed_registers = frozenset((CONTROL_MODE_ADDR, ACTUAL_FORCE_ADDR)
            if allowed_registers is None else allowed_registers)
        if not self.allowed_registers or not self.allowed_registers <= {0x25,0x27,0x2A,0x2B,0x2C,0x2F}:
            raise ValueError('read-only diagnostic permits only mode/target/position/current/force/fault')
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.retries = retries
        self.inter_frame_delay_s = inter_frame_delay_s
        self.serial = serial_instance
        self.port_lock = None
        self._owns_serial = serial_instance is None

    def connect(self) -> None:
        if self.serial is None:
            import serial
            self.serial = serial.Serial(
                self.port,
                self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.timeout,
                write_timeout=0.030,
                exclusive=True,
            )
            fileno_fn = getattr(self.serial, 'fileno', None)
            fd = fileno_fn() if callable(fileno_fn) else None
            self.port_lock = SerialPortLock(fd, port=self.port, owner="read_only_diagnostic")
            self.port_lock.acquire()
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()

    def close(self) -> None:
        if self.port_lock is not None:
            self.port_lock.release()
            self.port_lock = None
        if self.serial is not None:
            close_fn = getattr(self.serial, "close", None)
            if callable(close_fn):
                close_fn()
            if self._owns_serial:
                self.serial = None

    def __enter__(self) -> "ReadOnlyDiagnosticSession":
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.close()

    def _safe_transceive(self, motor_id: int, register_addr: int) -> dict:
        """Sends pack_read_register_cmd(0x31) and awaits length-delimited response."""
        if motor_id not in MOTOR_IDS or isinstance(motor_id, bool):
            raise SafetyViolation('read-only diagnostic requires an individual ID1..ID6')
        req_frame = pack_read_register_cmd(motor_id, register_addr, count=1)

        # Invariant verification: strictly 0x31 and authorized register only
        parsed_cmd = parse_frame(req_frame, expected_header=CMD_FRAME_HEADER)
        if parsed_cmd.command != CMD_RD_REGISTER:
            raise SafetyViolation(
                f"FATAL: Attempted non-read command 0x{parsed_cmd.command:02X} in read-only diagnostic mode!"
            )
        if parsed_cmd.register_addr not in self.allowed_registers:
            raise SafetyViolation(
                f"FATAL: Attempted to read unauthorized register 0x{parsed_cmd.register_addr:02X}!"
            )

        attempts = 0
        last_error = None
        t_start_wall = time.time()

        for attempt in range(self.retries + 1):
            attempts += 1
            if attempt > 0:
                time.sleep(self.inter_frame_delay_s)

            try:
                # Flush input buffer to clear any line noise or stray bytes
                reset_in = getattr(self.serial, "reset_input_buffer", None)
                if callable(reset_in):
                    reset_in()

                t_tx_ns = time.monotonic_ns()
                written = self.serial.write(req_frame)
                flush = getattr(self.serial, "flush", None)
                if callable(flush):
                    flush()

                # Length-delimited read
                deadline = time.monotonic() + self.timeout
                raw_resp = self._read_response_frame(deadline)
                t_rx_ns = time.monotonic_ns()
                duration_ms = (t_rx_ns - t_tx_ns) / 1e6

                # Validate response
                validate_response(raw_resp, expected_id=motor_id, expected_command=CMD_RD_REGISTER)
                unpacked = unpack_read_register_response(raw_resp, expected_addr=register_addr, expected_count=1)

                return {
                    "motor_id": motor_id,
                    "register_addr": register_addr,
                    "success": True,
                    "raw_req": req_frame,
                    "raw_resp": raw_resp,
                    "raw_req_hex": bytes_to_hex(req_frame),
                    "raw_resp_hex": bytes_to_hex(raw_resp),
                    "raw_value_u16": unpacked.values[0],
                    "latency_ms": duration_ms,
                    "tx_monotonic_ns": t_tx_ns,
                    "rx_monotonic_ns": t_rx_ns,
                    "attempts": attempts,
                    "error": None,
                }
            except Exception as exc:
                last_error = exc

        return {
            "motor_id": motor_id,
            "register_addr": register_addr,
            "success": False,
            "raw_req": req_frame,
            "raw_resp": None,
            "raw_req_hex": bytes_to_hex(req_frame),
            "raw_resp_hex": None,
            "raw_value_u16": None,
            "latency_ms": (time.time() - t_start_wall) * 1000.0,
            "attempts": attempts,
            "error": f"{type(last_error).__name__}: {last_error}",
        }

    def read_register(self, motor_id: int, register_addr: int) -> dict:
        """Explicit register read through the same owner; never a write or Pause."""
        return self._safe_transceive(motor_id, register_addr)

    def _read_exact(self, size: int, deadline: float) -> bytes:
        result = bytearray()
        while len(result) < size:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError(f"UART read timed out after {self.timeout:.3f}s")
            old_timeout = getattr(self.serial, "timeout", None)
            try:
                if old_timeout is not None:
                    self.serial.timeout = min(float(old_timeout), max(0.001, remaining))
                chunk = self.serial.read(size - len(result))
            finally:
                if old_timeout is not None:
                    self.serial.timeout = old_timeout
            if not chunk:
                raise TimeoutError(
                    f"UART read timed out after {self.timeout:.3f}s (received {len(result)}/{size} bytes)"
                )
            result.extend(chunk)
        return bytes(result)

    def _read_response_frame(self, deadline: float) -> bytes:
        while True:
            first = self._read_exact(1, deadline)
            if first != RSP_FRAME_HEADER[:1]:
                continue
            second = self._read_exact(1, deadline)
            if second == RSP_FRAME_HEADER[1:2]:
                break
        length = self._read_exact(1, deadline)[0]
        if length < 3:
            raise FrameLengthError(f"Invalid response data length {length}")
        remainder = self._read_exact(length + 2, deadline)
        raw = RSP_FRAME_HEADER + bytes((length,)) + remainder
        parse_frame(raw, expected_header=RSP_FRAME_HEADER)
        return raw

    def read_axis(self, motor_id: int) -> dict:
        """Reads register 0x25 (Mode) and 0x2C (Actual Force) for a single axis."""
        res_25 = self._safe_transceive(motor_id, CONTROL_MODE_ADDR)
        time.sleep(self.inter_frame_delay_s)
        res_2c = self._safe_transceive(motor_id, ACTUAL_FORCE_ADDR)
        time.sleep(self.inter_frame_delay_s)

        mode_raw = res_25["raw_value_u16"]
        mode_name = None
        if mode_raw is not None:
            mode_map = {
                0: "Mode 0 (Position)",
                1: "Mode 1 (Servo)",
                2: "Mode 2 (Speed)",
                3: "Mode 3 (Native Force)",
            }
            mode_name = mode_map.get(mode_raw, f"Mode {mode_raw} (Unknown)")

        force_u16 = res_2c["raw_value_u16"]
        force_g = decode_u16_to_signed_i16(force_u16) if force_u16 is not None else None
        force_n = g_to_n(force_g) if force_g is not None else None

        return {
            "motor_id": motor_id,
            "online": res_25["success"] or res_2c["success"],
            "mode_25_raw": mode_raw,
            "mode_25_name": mode_name,
            "mode_25_req_hex": res_25["raw_req_hex"],
            "mode_25_resp_hex": res_25["raw_resp_hex"],
            "mode_25_latency_ms": res_25["latency_ms"],
            "mode_25_error": res_25["error"],
            "force_2c_raw_u16": force_u16,
            "force_2c_g": force_g,
            "force_2c_n": force_n,
            "force_2c_req_hex": res_2c["raw_req_hex"],
            "force_2c_resp_hex": res_2c["raw_resp_hex"],
            "force_2c_latency_ms": res_2c["latency_ms"],
            "force_2c_error": res_2c["error"],
        }

    def read_all_axes(self, motor_ids: Sequence[int] = MOTOR_IDS) -> list[dict]:
        """Sequentially reads 0x25 and 0x2C for all specified motors."""
        results = []
        for mid in motor_ids:
            results.append(self.read_axis(mid))
        return results


def _print_pass_summary(results: list[dict], pass_idx: int) -> None:
    print(f"\n--- Pass {pass_idx} Diagnostic Summary ---")
    header = f"{'ID':<4} | {'Status':<10} | {'0x25 Raw':<8} | {'Mode Description':<22} | {'0x2C Raw':<8} | {'Force (g)':<10} | {'Force (N)':<10} | {'Latency (ms)':<12}"
    print(header)
    print("-" * len(header))
    for r in results:
        mid = r['motor_id']
        status = "ONLINE" if r['online'] else "NO_RESP"
        m_raw = f"0x{r['mode_25_raw']:04X}" if r['mode_25_raw'] is not None else "N/A"
        m_name = r['mode_25_name'] or "N/A"
        f_raw = f"0x{r['force_2c_raw_u16']:04X}" if r['force_2c_raw_u16'] is not None else "N/A"
        f_g = f"{r['force_2c_g']} g" if r['force_2c_g'] is not None else "N/A"
        f_n = f"{r['force_2c_n']:.3f} N" if r['force_2c_n'] is not None else "N/A"
        lat = f"{r['mode_25_latency_ms']:.1f}+{r['force_2c_latency_ms']:.1f}"
        print(f"ID{mid:<2} | {status:<10} | {m_raw:<8} | {m_name:<22} | {f_raw:<8} | {f_g:<10} | {f_n:<10} | {lat:<12}")

    print("\n--- Raw Request / Response Frames ---")
    for r in results:
        mid = r['motor_id']
        print(f"Motor ID{mid}:")
        print(f"  0x25 Mode  : TX=[{r['mode_25_req_hex']}] RX=[{r['mode_25_resp_hex'] or r['mode_25_error']}]")
        print(f"  0x2C Force : TX=[{r['force_2c_req_hex']}] RX=[{r['force_2c_resp_hex'] or r['force_2c_error']}]")


def run_read_only_diagnostic(
    port: str = "/dev/ttyUSB0",
    baudrate: int = 921600,
    motor_ids: Sequence[int] = MOTOR_IDS,
    timeout: float = 0.050,
    retries: int = 1,
    num_passes: int = 2,
) -> list[list[dict]]:
    """Runs a pure read-only diagnostic on real motors ID1..ID6 and prints a comprehensive report."""
    print("=" * 80)
    print(f"6PUS REAL MOTOR READ-ONLY DIAGNOSTIC (PORT: {port}, BAUDRATE: {baudrate})")
    print("=" * 80)
    print("Strict Safety Invariant: ONLY sending 0x31 (CMD_RD_REGISTER) for 0x25 and 0x2C.")
    print("Zero writes, zero motion, zero Pause commands, no mode switching.")
    print("-" * 80)

    all_passes = []
    with ReadOnlyDiagnosticSession(port=port, baudrate=baudrate, timeout=timeout, retries=retries) as session:
        for pass_idx in range(1, num_passes + 1):
            print(f"\n>>> Running Read-Only Diagnostic Pass {pass_idx}/{num_passes}...")
            results = session.read_all_axes(motor_ids=motor_ids)
            all_passes.append(results)
            _print_pass_summary(results, pass_idx)
            if pass_idx < num_passes:
                time.sleep(0.020)

    print("\n" + "=" * 80)
    print("DIAGNOSTIC COMPLETE - SERIAL PORT CLOSED CLEANLY WITHOUT PAUSE/WRITES")
    print("=" * 80)
    return all_passes


def main():
    p = argparse.ArgumentParser(description="6PUS UART Child process or read-only diagnostic.")
    p.add_argument('--read-only-diagnostic', action='store_true', help='Run strictly pure read-only hardware diagnostic (0x31 only, 0x25 & 0x2C)')
    p.add_argument('--port', default='/dev/ttyUSB0', help='Serial port for read-only diagnostic')
    p.add_argument('--baudrate', type=int, default=921600, help='Serial baudrate')
    p.add_argument('--fd', type=int, required=False)
    p.add_argument('--fd-cmd', type=int, required=False)
    p.add_argument('--fd-fb', type=int, required=False)
    p.add_argument('--config', required=False)
    p.add_argument('--stage5-config', help='Mode3-only joint position-force experiment in the existing UART owner')
    p.add_argument('--stage4-config', help='Bounded real active-stabilization experiment; same maintenance UART owner')
    p.add_argument('--stage3c-config', help='Explicit bounded live Z force PID config; same UART Child owner')
    p.add_argument('--maintenance-config', help='Explicit bounded Force maintenance config; existing UART Child owner')
    a = p.parse_args()

    if a.read_only_diagnostic:
        run_read_only_diagnostic(port=a.port, baudrate=a.baudrate)
        return

    if a.stage5_config:
        from pus6_force_control.stage5_experiment import run
        result=run(json.loads(Path(a.stage5_config).read_text()))
        if result.get('outcome')=='stopped' or (result.get('outcome')!='read_only_complete' and not result.get('all_mode0_final_confirmed')):
            raise SystemExit(2)
        return

    if a.stage4_config:
        from pus6_force_control.stage4_experiment import run
        result = run(json.loads(Path(a.stage4_config).read_text()))
        if result.get('outcome') == 'stopped' or result.get('exit_error'):
            raise SystemExit(2)
        return

    if a.stage3c_config:
        from pus6_force_control.stage3c_experiment import run
        result = run(json.loads(Path(a.stage3c_config).read_text()))
        if result.get('outcome') != 'sequence_complete' or not result.get('all_mode0_exit_confirmed'):
            raise SystemExit(2)
        return
    if a.maintenance_config:
        from pus6_force_control.six_axis_tare_experiment import run
        run(json.loads(Path(a.maintenance_config).read_text()))
        return

    if a.config is None:
        raise ValueError('--config is required when not in --read-only-diagnostic mode')
    cmd_fd = a.fd_cmd if a.fd_cmd is not None else a.fd
    if cmd_fd is None:
        raise ValueError('Either --fd or --fd-cmd is required')
    cmd_s = socket.socket(fileno=cmd_fd); cmd_s.setblocking(False)
    if a.fd_fb is not None:
        fb_s = socket.socket(fileno=a.fd_fb); fb_s.setblocking(False)
    else:
        fb_s = None
    child_run(cmd_s, json.loads(a.config), fb_sock=fb_s)


if __name__ == '__main__':
    main()
