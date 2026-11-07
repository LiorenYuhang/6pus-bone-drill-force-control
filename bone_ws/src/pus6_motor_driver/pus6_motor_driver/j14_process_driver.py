"""A1.9/A1.11 experimental ROS parent with minimal feedback pipeline instrumentation.

UART transactions are strictly forbidden in this class.
"""
from __future__ import annotations

import os
import select
import threading
import time
from pathlib import Path
from typing import Any

from builtin_interfaces.msg import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger
from rclpy.qos import qos_profile_sensor_data

AXIAL_FORCE_COMMAND_QOS = qos_profile_sensor_data

from .motor_driver_node import Pus6MotorDriverNode, ServoTelemetrySnapshot
from .j14_process_uart import ProcessClient, append_rows
from .j14_feedback_bridge import populate_force_feedback
from .safety import DriverError, steps_to_meters, HardwareMode

try:
    from pus6_msgs.msg import TimedJointCommand, TimedJointFeedback
except ImportError:
    TimedJointCommand = None  # type: ignore
    TimedJointFeedback = None  # type: ignore

try:
    from pus6_research_msgs.msg import AxialForceCommand
except ImportError:
    AxialForceCommand = None  # type: ignore


class J14ProcessMotorDriverNode(Pus6MotorDriverNode):
    def _native_force_child_options(self):
        """Shared by the formal production entry and the compatibility parent."""
        from .force_units import NOMINAL_N_PER_G, DEFAULT_FORCE_LIMIT_N
        defaults=dict(direction=[1.]*6,newtons_per_g=[NOMINAL_N_PER_G]*6,
            min_force_n=[-DEFAULT_FORCE_LIMIT_N]*6,max_force_n=[DEFAULT_FORCE_LIMIT_N]*6,
            max_rate_g_s=[500.]*6,allow_negative_forces=True,control_mode='position',mode3_only=False,
            force_dry_run=True,force_hardware_write_enabled=False,force_is_hardware_qualified=False)
        for name,value in defaults.items():
            if not self.has_parameter(name):self.declare_parameter(name,value)
        mode=self.get_parameter('control_mode').value
        if mode not in ('position','force'):raise ValueError('control_mode must be position or force')
        self.native_force_control=mode=='force'
        options={k:self.get_parameter(k).value for k in defaults if not k.startswith('force_') and k!='control_mode'}
        if self.native_force_control:
            options.update(control_mode='force',dry_run=self.get_parameter('force_dry_run').value,
                hardware_write_enabled=self.get_parameter('force_hardware_write_enabled').value,
                is_hardware_qualified=self.get_parameter('force_is_hardware_qualified').value,
                hold_anchor_on_timeout=False,max_force_handover_displacement_steps=6,force_mode_check_period_s=1.)
            if options['dry_run']:
                raise ValueError('Force owner needs an explicit handover; use FakeBus offline or authorized live entry')
            if self.hardware_mode.value=='active' and not (options['hardware_write_enabled'] and options['is_hardware_qualified']):
                raise ValueError('live Mode 3 owner requires independent write authorization and hardware qualification before opening UART')
        return options

    def _subscribe_native_force_commands(self):
        self._axial_forces_sub=(self.create_subscription(AxialForceCommand,'/pus6/command_axial_forces',
            self._handle_command_axial_forces,AXIAL_FORCE_COMMAND_QOS,callback_group=self.sub_cb_group)
            if AxialForceCommand is not None and self.native_force_control else None)

    def _init_j7_servo_backend(self, p_port, p_baudrate, p_timeout):
        if self.servo_scheduler_mode != 'dedicated_thread' or self.telemetry_publish_mode != 'async_latest':
            raise ValueError('A1.9/A1.11 requires dedicated_thread / async_latest')
        if self.feedback_loss_policy != 'age_based' or self.reconnect_max_jump_steps != 80:
            raise ValueError('A1.9/A1.11 requires unchanged J13 safety configuration')
        if self._stall_recorder is not None:
            self._stall_recorder.enabled = False  # child owns original UART recorder files
            self._stall_recorder._dumped = True  # prevent its registered atexit hook overwriting child evidence
        self._stall_recorder = None
        self._pipeline_records: list[dict[str, Any]] = []
        self._current_pending_snap: ServoTelemetrySnapshot | None = None
        self._instrumentation_enabled = (os.environ.get('PUS6_J14_FEEDBACK_PIPELINE_INSTRUMENTATION', '1') == '1')
        self.use_feedback_bridge = (os.environ.get('PUS6_J14_FEEDBACK_BRIDGE') == '1')
        config = dict(
            port=p_port,
            baudrate=p_baudrate,
            hardware_mode=self.hardware_mode.value,
            output_dir=str(self._communication_audit_dir()),
            epoch=time.monotonic_ns(),
            servo_speed=self.servo_speed,
            maintenance_mode=self.maintenance_mode,
            target_timeout_sec=self.target_timeout_sec,
            reconnect_max_jump_steps=self.reconnect_max_jump_steps,
            max_servo_duration_s=float(os.environ.get('PUS6_J14_PROCESS_WINDOW_S', '60')),
            feedback_bridge=self.use_feedback_bridge,
        )
        config.update(self._native_force_child_options())
        self._process_client = ProcessClient(config)
        self.j7_adapter = self._process_client  # endpoint only, no serial handle
        self.j7_writer = self._process_client
        self.j7_feedback = self._process_client
        self.j7_stop_adapter = None

        if self.use_feedback_bridge:
            # Section X: Prevent duplicate publisher semantics; Bridge owns feedback topics
            self.joint_state_pub = None
            self.timed_feedback_pub = None
            self.timed_command_pub = None
            self.get_logger().info("A1.12 Dedicated Feedback Bridge active: Parent suppressed duplicate feedback publishers")

        ready = self._process_client.ready
        # Send the same measured static startup reference, with no new motion target.
        if not self.native_force_control:
            self._process_client.update(ready['positions'], seq=0)
        for i, mid in enumerate(range(1, 7)):
            self._latest_positions_steps[mid] = ready['positions'][i]
            self._latest_currents[mid] = ready['currents'][i]
            self._latest_faults[mid] = ready['faults'][i]
            self._latest_rx_done_ns[mid] = ready['rx_ns'][i]
        self.get_logger().info(
            f"A1.9/A1.12 exclusive UART child pid={ready['pid']}; "
            f"ROS-free={ready['modules_without_ros']}; pipeline_diag={self._instrumentation_enabled}; "
            f"bridge={self.use_feedback_bridge}"
        )
        self._subscribe_native_force_commands()

    def _handle_command_axial_forces(self, msg: AxialForceCommand) -> None:
        if not self.native_force_control:
            return  # force topic must never switch an ordinary position session
        if self._j7_fault_latched:
            return
        forces = getattr(msg, "force_n", None)
        if forces is None or len(forces) != 6:
            self.get_logger().error("AxialForceCommand must have 6 force values")
            return
        import math
        if any(not isinstance(v, (int, float)) or not math.isfinite(v) for v in forces):
            self.get_logger().error("AxialForceCommand forces must be finite numbers")
            return
        now_ns = time.monotonic_ns()
        stamp_ns = getattr(msg, "sample_monotonic_ns", 0)
        if not isinstance(stamp_ns, int) or stamp_ns <= 0 or stamp_ns > now_ns:
            stamp_ns = now_ns
        client = getattr(self, '_process_client', None)
        if client is not None:
            client.update_force(forces, publish_ns=stamp_ns)

    def _enqueue_latest_target(self, targets_steps):
        if getattr(self, "native_force_control", False):
            return  # no ordinary position targets in a native-force session
        if self._j7_fault_latched:
            return
        super()._enqueue_latest_target(targets_steps)
        client = getattr(self, '_process_client', None)
        if client is not None:
            with self._target_lock:
                envelope = self._latest_target
                if envelope is not None:
                    client.update(envelope.targets_steps, publish_ns=envelope.stamp_ns, seq=envelope.seq + 1)

    def _execute_servo_cycle(self, expected_start_ns=0):
        raise RuntimeError('A1.9/A1.11 parent cannot execute UART servo cycles')

    def _servo_dedicated_worker_loop(self):
        threading.current_thread().name = 'Pus6ParentIPCThread'
        client = getattr(self, '_process_client', None)
        if client is None:
            self._latch_j7_fault('A1.9/A1.11 child initialization failed')
            return
        try:
            while not self._servo_thread_stop_event.is_set():
                client.send_state()
                rows = client.receive()
                for row in rows:
                    if row.get('type') == 'feedback':
                        self._accept_child_feedback(row)
                    elif row.get('type') == 'fault' and row.get('failed_audit'):
                        a = row['failed_audit']
                        self.get_logger().error(
                            f"UART TX failure: cycle_seq={a['cycle_id']} motor_id={a['error_motor_id']} child-local pause executed"
                        )
                if self.use_feedback_bridge and client.bridge_proc is not None:
                    if client.bridge_proc.poll() is not None:
                        self.get_logger().error(
                            f"Feedback Bridge process exited (returncode={client.bridge_proc.returncode}); feedback path unavailable"
                        )
                        self._latch_j7_fault("Feedback Bridge died; feedback path unavailable")
                        break
                if client.stopped:
                    if client.stop_reason == 'window_complete':
                        self._fresh_state_hold = True
                        self.get_logger().info('A1.9/A1.11 static window complete; child paused and closed')
                    else:
                        self._latch_j7_fault(client.stop_reason or 'UART child exited')
                    time.sleep(0.050)
                    self._flush_pipeline_records()
                    break
                select.select([client.sock], [], [], 0.010)
        except Exception as exc:
            client.stop(f'parent_IPC_exception: {exc}')
            self._latch_j7_fault(f'parent_IPC_exception: {exc}')

    def _accept_child_feedback(self, row):
        if row['epoch'] != self._process_client.config['epoch']:
            raise ValueError('child feedback epoch mismatch')
        for name in ('positions', 'currents', 'faults', 'rx_ns', 'targets', 'tx_start_ns', 'tx_end_ns'):
            if len(row[name]) != 6:
                raise ValueError('partial feedback vector')
        for i, mid in enumerate(range(1, 7)):
            self._latest_positions_steps[mid] = row['positions'][i]
            self._latest_currents[mid] = row['currents'][i]
            self._latest_faults[mid] = row['faults'][i]
            self._latest_rx_done_ns[mid] = row['rx_ns'][i]
        if row.get('control_mode') != 3:
            self._last_commanded_steps = list(row['targets'])
        self._uart_cycle_seq = row['cycle_seq'] + 1
        stamp = self.get_clock().now().to_msg()
        snap = ServoTelemetrySnapshot(
            uart_cycle_seq=row['cycle_seq'],
            cycle_start_ns=row['cycle_start_ns'],
            cycle_end_ns=row['cycle_end_ns'],
            stamp_sec=stamp.sec,
            stamp_nanosec=stamp.nanosec,
            positions_steps=row['positions'],
            currents_mA=row['currents'],
            fault_codes=row['faults'],
            rx_done_ns=row['rx_ns'],
            dispatched_targets=row['targets'],
            tx_start_ns=row['tx_start_ns'],
            tx_end_ns=row['tx_end_ns'],
            diag_data=[
                float(self._target_in_count),
                float(self._target_overwrite_count),
                float(row['command_seq'] + 1),
                0.0,
                (row['cycle_start_ns'] - row['command_publish_ns']) / 1e6 if row['command_publish_ns'] else 0.0,
                20.0,
                sum(row['tx_wall_ns']) / 1e6,
                row['rx_drain_ns'] / 1e6,
                0.0,
                (row['cycle_end_ns'] - row['cycle_start_ns']) / 1e6,
                0.0,
            ],
            benchmark_seq_id=0,
            t_entry_ns=row['cycle_start_ns'],
        )
        snap.epoch = row['epoch']
        snap.force_fields = {key: row[key] for key in (
            'forces_g_raw', 'force_adc_raw', 'temperatures_c', 'force_valid', 'feedback_source',
            'force_mode_confirmed', 'force_hold_valid', 'force_hold_g', 'force_hold_reference_monotonic_ns'
        ) if key in row}
        snap.command_seq = row.get('command_seq', -1)
        snap.t_child_ready_ns = row.get('feedback_ready_ns', 0)
        snap.t_parent_recv_ns = row.get('parent_receive_ns', 0)
        snap._published = False

        with self._telemetry_lock:
            prev_snap = getattr(self, '_current_pending_snap', None)
            if prev_snap is not None and getattr(prev_snap, '_published', False) is False:
                # The previous snapshot was superseded before Pus6TelemetryThread could publish it.
                if getattr(self, '_instrumentation_enabled', False):
                    self._pipeline_records.append(
                        dict(
                            epoch=getattr(prev_snap, 'epoch', None),
                            cycle_seq=prev_snap.uart_cycle_seq,
                            command_seq=getattr(prev_snap, 'command_seq', None),
                            status='superseded',
                            superseded_by=snap.uart_cycle_seq,
                            t_child_ready_ns=getattr(prev_snap, 't_child_ready_ns', 0),
                            t_parent_recv_ns=getattr(prev_snap, 't_parent_recv_ns', 0),
                            t_snapshot_update_ns=getattr(prev_snap, 't_snapshot_update_ns', 0),
                            t_telemetry_wake_ns=None,
                            t_message_build_done_ns=None,
                            t_pub_js_enter_ns=None,
                            t_pub_js_exit_ns=None,
                            t_pub_tf_enter_ns=None,
                            t_pub_tf_exit_ns=None,
                            t_pub_tc_enter_ns=None,
                            t_pub_tc_exit_ns=None,
                            telemetry_thread_cpu_ns=None,
                            message_build_thread_cpu_ns=None,
                            pub_js_thread_cpu_ns=None,
                            pub_tf_thread_cpu_ns=None,
                            pub_tc_thread_cpu_ns=None,
                            process_cpu_ns=time.process_time_ns(),
                        )
                    )
            t_snap_update = time.monotonic_ns()
            snap.t_snapshot_update_ns = t_snap_update
            self._latest_telemetry_snapshot = snap
            self._current_pending_snap = snap
        self._telemetry_ready_event.set()

    def _telemetry_publisher_loop(self) -> None:
        """Dedicated worker loop for non-blocking asynchronous telemetry publishing (J11-RT2B/A1.11)."""
        threading.current_thread().name = 'Pus6TelemetryThread'
        while not self._telemetry_thread_stop_event.is_set():
            if not self._telemetry_ready_event.wait(timeout=0.020):
                continue
            t_wake = time.monotonic_ns()
            tc_wake = time.thread_time_ns()
            pc_wake = time.process_time_ns()
            self._telemetry_ready_event.clear()
            with self._telemetry_lock:
                snap = self._latest_telemetry_snapshot
                if snap is not None:
                    snap._published = True
                    if getattr(self, '_current_pending_snap', None) is snap:
                        self._current_pending_snap = None
            if snap is not None:
                self._publish_telemetry_from_snapshot(
                    snap, record_stall=False, t_wake=t_wake, tc_wake=tc_wake, pc_wake=pc_wake
                )

    def _publish_telemetry_from_snapshot(
        self,
        snap: ServoTelemetrySnapshot,
        record_stall: bool = False,
        t_wake: int | None = None,
        tc_wake: int | None = None,
        pc_wake: int | None = None,
    ) -> None:
        """Publishes all telemetry topics from a snapshot with J14-A1.11 pipeline timestamps."""
        if getattr(self, 'use_feedback_bridge', False) or self.joint_state_pub is None:
            return

        if not getattr(self, '_instrumentation_enabled', False):
            super()._publish_telemetry_from_snapshot(snap, record_stall=record_stall)
            return

        t_entry = time.monotonic_ns()
        tc_entry = time.thread_time_ns()
        t_wake = t_wake or t_entry

        # Phase D: Message Construction
        stamp_msg = Time(sec=snap.stamp_sec, nanosec=snap.stamp_nanosec)

        # 1. JointState message
        js_msg = JointState()
        js_msg.header.stamp = stamp_msg
        js_msg.name = self.joint_names
        js_msg.position = [steps_to_meters(s) for s in snap.positions_steps]

        # 2. TimedJointFeedback message
        tf_msg = None
        if self.timed_feedback_pub is not None:
            tf_msg = TimedJointFeedback()
            tf_msg.header.stamp = stamp_msg
            tf_msg.header.frame_id = 'pus6_base'
            tf_msg.snapshot_start_monotonic_ns = snap.cycle_start_ns
            tf_msg.snapshot_end_monotonic_ns = snap.cycle_end_ns
            tf_msg.per_axis_rx_done_monotonic_ns = list(snap.rx_done_ns)
            tf_msg.raw_steps = list(snap.positions_steps)
            tf_msg.positions_meters = [0.050 + (s - 1000) * 0.000025 for s in snap.positions_steps]
            tf_msg.currents_ma = list(snap.currents_mA)
            tf_msg.fault_codes = list(snap.fault_codes)
            tf_msg.uart_cycle_sequence = snap.uart_cycle_seq
            populate_force_feedback(tf_msg, getattr(snap, 'force_fields', {}))

        # 3. TimedJointCommand message
        tc_msg = None
        if self.timed_command_pub is not None and self.hardware_mode != HardwareMode.READ_ONLY and not self.native_force_control:
            tc_msg = TimedJointCommand()
            tc_msg.header.stamp = stamp_msg
            tc_msg.transaction_start_monotonic_ns = snap.cycle_start_ns
            tc_msg.transaction_end_monotonic_ns = snap.cycle_end_ns
            tc_msg.per_axis_tx_start_monotonic_ns = list(snap.tx_start_ns)
            tc_msg.per_axis_tx_done_monotonic_ns = list(snap.tx_end_ns)
            tc_msg.target_positions_meters = [steps_to_meters(s) for s in snap.dispatched_targets]
            tc_msg.target_steps = list(snap.dispatched_targets)
            tc_msg.uart_cycle_sequence = snap.uart_cycle_seq

        t_build_done = time.monotonic_ns()
        tc_build_done = time.thread_time_ns()

        # Phase E: Publication Calls (ordered: joint_states -> timed_feedback -> timed_command)
        # Topic 1: JointState
        t_pub_js_enter = time.monotonic_ns()
        tc_pub_js_enter = time.thread_time_ns()
        self.joint_state_pub.publish(js_msg)
        t_pub_js_exit = time.monotonic_ns()
        tc_pub_js_exit = time.thread_time_ns()

        # Topic 2: TimedJointFeedback
        t_pub_tf_enter = t_pub_tf_exit = tc_pub_tf_enter = tc_pub_tf_exit = 0
        if tf_msg is not None:
            t_pub_tf_enter = time.monotonic_ns()
            tc_pub_tf_enter = time.thread_time_ns()
            self.timed_feedback_pub.publish(tf_msg)
            t_pub_tf_exit = time.monotonic_ns()
            tc_pub_tf_exit = time.thread_time_ns()

        # Topic 3: TimedJointCommand
        t_pub_tc_enter = t_pub_tc_exit = tc_pub_tc_enter = tc_pub_tc_exit = 0
        if tc_msg is not None:
            t_pub_tc_enter = time.monotonic_ns()
            tc_pub_tc_enter = time.thread_time_ns()
            self.timed_command_pub.publish(tc_msg)
            t_pub_tc_exit = time.monotonic_ns()
            tc_pub_tc_exit = time.thread_time_ns()

        # Diagnostics (deferred by default)
        if not self._defer_driver_diagnostics:
            diag_msg = Float64MultiArray()
            diag_msg.data = list(snap.diag_data)
            self.driver_diagnostics_pub.publish(diag_msg)

        if self.publish_raw_diagnostics:
            self._publish_array(self.raw_steps_pub, snap.positions_steps)
            self._publish_array(self.raw_currents_pub, snap.currents_mA)
            self._publish_array(self.raw_faults_pub, snap.fault_codes)

        t_exit = time.monotonic_ns()
        tc_exit = time.thread_time_ns()
        pc_exit = time.process_time_ns()

        record = dict(
            epoch=getattr(snap, 'epoch', None),
            cycle_seq=snap.uart_cycle_seq,
            command_seq=getattr(snap, 'command_seq', None),
            status='published',
            superseded_by=None,
            t_child_ready_ns=getattr(snap, 't_child_ready_ns', 0),
            t_parent_recv_ns=getattr(snap, 't_parent_recv_ns', 0),
            t_snapshot_update_ns=getattr(snap, 't_snapshot_update_ns', 0),
            t_telemetry_wake_ns=t_wake,
            t_message_build_done_ns=t_build_done,
            t_pub_js_enter_ns=t_pub_js_enter,
            t_pub_js_exit_ns=t_pub_js_exit,
            t_pub_tf_enter_ns=t_pub_tf_enter,
            t_pub_tf_exit_ns=t_pub_tf_exit,
            t_pub_tc_enter_ns=t_pub_tc_enter,
            t_pub_tc_exit_ns=t_pub_tc_exit,
            telemetry_thread_cpu_ns=tc_exit - (tc_wake or tc_entry),
            message_build_thread_cpu_ns=tc_build_done - tc_entry,
            pub_js_thread_cpu_ns=tc_pub_js_exit - tc_pub_js_enter,
            pub_tf_thread_cpu_ns=tc_pub_tf_exit - tc_pub_tf_enter,
            pub_tc_thread_cpu_ns=tc_pub_tc_exit - tc_pub_tc_enter,
            process_cpu_ns=pc_exit,
        )
        self._pipeline_records.append(record)

    def _handle_pause_all(self, request, response):
        client = getattr(self, '_process_client', None)
        if client is None:
            response.success = False
            response.message = 'UART child unavailable'
        else:
            client.stop('service_pause')
            response.success = False
            response.message = 'Child terminal stop requested; await per-axis Mode0 anchoring and observations; service is not stop confirmation'
        return response

    def _handle_configure_motors(self, request, response):
        response.success = False
        response.message = 'A1.9/A1.11 static trial does not support runtime reconfigure/resume'
        return response

    def _handle_command_single_step(self, msg):
        self.get_logger().error('A1.9/A1.11 accepts only whole six-axis references')

    def _flush_pipeline_records(self):
        records = getattr(self, '_pipeline_records', None)
        if records:
            out_dir = self._communication_audit_dir()
            if out_dir:
                append_rows(out_dir / 'parent_feedback_pipeline.jsonl', records)
                records.clear()

    @property
    def telemetry_health_ok(self) -> bool:
        if self._j7_fault_latched:
            return False
        client = getattr(self, '_process_client', None)
        if client is not None:
            if client.stopped:
                return False
            if self.use_feedback_bridge and client.bridge_proc is not None and client.bridge_proc.poll() is not None:
                return False
        return super().telemetry_health_ok

    def destroy_node(self):
        client = getattr(self, '_process_client', None)
        if client is not None:
            self._servo_thread_stop_event.set()
            if self._servo_thread is not None:
                self._servo_thread.join(timeout=1)
            if self._telemetry_thread is not None:
                self._telemetry_thread_stop_event.set()
                self._telemetry_ready_event.set()
                self._telemetry_thread.join(timeout=1)
            self._flush_pipeline_records()
            client.close()
        super().destroy_node()
        if client is not None:
            append_rows(
                self._communication_audit_dir() / 'parent_lifecycle.jsonl',
                [dict(event='parent_node_destroyed', ns=time.monotonic_ns(), pid=os.getpid())],
            )
