#!/usr/bin/env python3
"""Dedicated Feedback Bridge Process (J14-A1.12).

Consumes UART Child feedback packets over an independent AF_UNIX SOCK_SEQPACKET
and publishes all feedback ROS 2 topics under a dedicated Python interpreter,
GIL, and ROS context, completely isolated from Driver Parent contention.
"""
from __future__ import annotations

from collections import deque
import argparse
import json
import os
from pathlib import Path
import select
import signal
import socket
import sys
import threading
import time
from typing import Any

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from builtin_interfaces.msg import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32MultiArray, String

from .safety import steps_to_meters, MOTOR_IDS
from .j14_process_uart import append_rows, receive_packets, send_packet

try:
    from pus6_msgs.msg import TimedJointCommand, TimedJointFeedback
except ImportError:
    TimedJointCommand = None  # type: ignore
    TimedJointFeedback = None  # type: ignore


def populate_force_feedback(msg, row):
    """Forward raw fields; absent legacy metadata is UNKNOWN/invalid, never zero force."""
    hold=row.get('force_hold_g')
    hold_stamp=row.get('force_hold_reference_monotonic_ns',0)
    msg.force_mode_confirmed=row.get('force_mode_confirmed') is True
    msg.force_hold_valid=(row.get('force_hold_valid') is True and hold is not None
        and len(hold)==6 and all(type(v) is int and -32768<=v<=32767 for v in hold)
        and type(hold_stamp) is int and hold_stamp>0)
    msg.force_hold_g=list(hold) if msg.force_hold_valid else [0]*6
    msg.force_hold_reference_monotonic_ns=hold_stamp if msg.force_hold_valid else 0
    names = ('forces_g_raw', 'force_adc_raw', 'temperatures_c', 'force_valid')
    complete = all(name in row and len(row[name]) == 6 for name in names)
    source = row.get('feedback_source', 0)
    msg.source = source if source in (1, 2) else 0
    if not complete:
        msg.force_valid = [False] * 6
        return
    msg.forces_g_raw = list(row['forces_g_raw'])
    msg.force_adc_raw = list(row['force_adc_raw'])
    msg.temperatures_c = list(row['temperatures_c'])
    msg.force_valid = [
        bool(row['force_valid'][i] is True and msg.source != 0
        and -32768 <= row['forces_g_raw'][i] <= 32767
        and 0 <= row['force_adc_raw'][i] <= 4095
        and msg.per_axis_rx_done_monotonic_ns[i] > 0 and msg.fault_codes[i] == 0)
        for i in range(6)
    ]


class Pus6FeedbackBridgeNode(Node):
    """Dedicated ROS 2 Node executing only feedback ingestion and publication."""

    def __init__(self, fb_sock: socket.socket, config: dict[str, Any]):
        super().__init__("pus6_feedback_bridge_node")
        if config.get('production') and (TimedJointFeedback is None or TimedJointCommand is None):
            raise RuntimeError('production requires pus6_msgs timed feedback/command types')
        self.fb_sock = fb_sock
        self.config = config
        self.out_dir = Path(config["output_dir"])
        self.out_dir.mkdir(parents=True, exist_ok=True)
        self.joint_names = [f"motor_{mid}" for mid in MOTOR_IDS]

        # Publishers matching production names and QoS exactly
        self.joint_state_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.transport_fault_pub = self.create_publisher(String, "/pus6/force_control/transport_fault", 10)
        if TimedJointFeedback is not None:
            self.timed_feedback_pub = self.create_publisher(
                TimedJointFeedback, "/pus6/joint_feedback/timed", qos_profile_sensor_data
            )
            self.timed_command_pub = self.create_publisher(
                TimedJointCommand, "/pus6/joint_command/timed", qos_profile_sensor_data
            )
        else:
            self.timed_feedback_pub = None
            self.timed_command_pub = None

        self._records: list[dict[str, Any]] = []
        self._last_recv_ns: int = 0
        self._last_cycle_seq: int = -1
        self._drop_count: int = 0
        self._total_received: int = 0
        self.production=bool(config.get('production'))
        self.telemetry_mode=config.get('telemetry_mode','debug')
        self._max_ipc_age_ms=0.;self._max_feedback_gap_ms=0.
        self._ipc_watchlist_count=0;self._gap_watchlist_count=0
        self._exit_reason='running'
        if self.production:
            capacity=300 if self.telemetry_mode=='production' else config['telemetry_capacity_cycles']
            self._records=deque(maxlen=capacity)
        self._running = True
        self._stop_event = threading.Event()

        t_now = time.monotonic_ns()
        append_rows(
            self.out_dir / "bridge_lifecycle.jsonl",
            [dict(event="bridge_started", ns=t_now, pid=os.getpid())],
        )

        # Section V, VII & X: Emit BRIDGE_READY now that node and all publishers are fully ready
        t_bridge_ready = time.monotonic_ns()
        self.t_bridge_ready_ns = t_bridge_ready
        self.t_servo_start_ns: int | None = None
        try:
            send_packet(
                self.fb_sock,
                dict(
                    type="bridge_ready",
                    epoch=self.config.get("epoch"),
                    pid=os.getpid(),
                    t_bridge_ready_ns=t_bridge_ready,
                ),
            )
        except (BrokenPipeError, ConnectionResetError, OSError):
            self._running = False
        append_rows(
            self.out_dir / "bridge_lifecycle.jsonl",
            [dict(event="bridge_ready", ns=t_bridge_ready, pid=os.getpid())],
        )

    def run_bridge_loop(self):
        """Dedicated high-rate socket consumption and publication loop."""
        threading.current_thread().name = "Pus6BridgeWorker"
        while self._running and not self._stop_event.is_set():
            try:
                rlist, _, _ = select.select([self.fb_sock], [], [], 0.020)
                if not rlist:
                    continue
                rows, flooded = receive_packets(self.fb_sock)
                if not rows:
                    # EOF from child
                    self.get_logger().info("Child closed feedback socket; bridge exiting")
                    break

                for row in rows:
                    t_recv = time.monotonic_ns()
                    tc_recv = time.thread_time_ns()
                    pc_recv = time.process_time_ns()

                    if row.get("type") == "feedback":
                        self._process_feedback_packet(row, t_recv, tc_recv, pc_recv)
                    elif row.get("type") in ("ready", "child_ready"):
                        self.get_logger().info(f"Feedback Bridge observed child ready pid={row.get('pid')}")
                    elif row.get("type") == "servo_start":
                        self.t_servo_start_ns = row.get("servo_start_ns")
                        self.get_logger().info(f"Feedback Bridge observed servo_start ns={self.t_servo_start_ns}")
                        append_rows(
                            self.out_dir / "bridge_lifecycle.jsonl",
                            [dict(event="bridge_observed_servo_start", ns=t_recv, servo_start_ns=self.t_servo_start_ns)],
                        )
                    elif row.get("type") == "fault":
                        self.get_logger().error(f"Feedback Bridge observed child fault: {row.get('communication_fault')}")
                        self._publish_fault_feedback(row)
                        self.transport_fault_pub.publish(String(data=str(row.get('communication_fault', 'child_fault'))))
            except (EOFError, BrokenPipeError, ConnectionResetError):
                self._exit_reason='child_feedback_eof'
                self.transport_fault_pub.publish(String(data=self._exit_reason))
                break
            except Exception as exc:
                self.get_logger().error(f"Bridge exception: {exc}")
                self._exit_reason=f'bridge_exception: {exc}'
                self.transport_fault_pub.publish(String(data=self._exit_reason))
                break

        self._flush_diagnostics()

    def _publish_fault_feedback(self, row):
        """Retain last raw per-axis values/time/faults without asserting fresh validity."""
        if self.timed_feedback_pub is None or not all(
                len(row.get(key, [])) == 6 for key in ('positions', 'currents', 'faults', 'rx_ns')):
            return
        msg = TimedJointFeedback()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'pus6_base'
        msg.snapshot_start_monotonic_ns = row.get('cycle_start_ns', 0)
        msg.snapshot_end_monotonic_ns = row.get('cycle_end_ns', 0)
        msg.per_axis_rx_done_monotonic_ns = list(row['rx_ns'])
        msg.raw_steps = list(row['positions'])
        msg.positions_meters = [0.050 + (s - 1000) * 0.000025 for s in row['positions']]
        msg.currents_ma = list(row['currents'])
        msg.fault_codes = list(row['faults'])
        msg.uart_cycle_sequence = row['cycle_seq']
        populate_force_feedback(msg, row)
        msg.force_valid = [False] * 6
        self.timed_feedback_pub.publish(msg)

    def _process_feedback_packet(
        self, row: dict[str, Any], t_recv: int, tc_recv: int, pc_recv: int
    ):
        if self.production:
            if row.get('epoch')!=self.config['epoch']:raise ValueError('feedback epoch mismatch')
            for name in ('positions','currents','faults','rx_ns','targets','tx_start_ns','tx_end_ns'):
                if len(row[name])!=6:raise ValueError('incomplete feedback vector')
            if row['cycle_seq']<=self._last_cycle_seq:raise ValueError('replayed feedback cycle')
        self._total_received += 1
        seq = row["cycle_seq"]
        if self._last_cycle_seq >= 0:
            expected = self._last_cycle_seq + 1
            if seq > expected:
                self._drop_count += (seq - expected)
        self._last_cycle_seq = seq

        t_child_ready = row.get("feedback_ready_ns") or row.get("cycle_end_ns", t_recv)
        feedback_gap_ms = (t_recv - self._last_recv_ns) / 1e6 if self._last_recv_ns > 0 else 0.0
        self._last_recv_ns = t_recv
        ipc_age_ms = (t_recv - t_child_ready) / 1e6
        self._max_ipc_age_ms=max(self._max_ipc_age_ms,ipc_age_ms)
        self._max_feedback_gap_ms=max(self._max_feedback_gap_ms,feedback_gap_ms)
        self._ipc_watchlist_count+=int(ipc_age_ms>100.)
        self._gap_watchlist_count+=int(feedback_gap_ms>100.)

        # Phase 1: Build Messages
        t_build_start = time.monotonic_ns()
        stamp_msg = self.get_clock().now().to_msg()

        # 1. JointState
        js_msg = JointState()
        js_msg.header.stamp = stamp_msg
        js_msg.name = self.joint_names
        js_msg.position = [steps_to_meters(s) for s in row["positions"]]

        # 2. TimedJointFeedback
        if self.timed_feedback_pub is not None:
            tf_msg = TimedJointFeedback()
            tf_msg.header.stamp = stamp_msg
            tf_msg.header.frame_id = 'pus6_base'
            tf_msg.snapshot_start_monotonic_ns = row["cycle_start_ns"]
            tf_msg.snapshot_end_monotonic_ns = row["cycle_end_ns"]
            tf_msg.per_axis_rx_done_monotonic_ns = list(row["rx_ns"])
            tf_msg.raw_steps = list(row["positions"])
            tf_msg.positions_meters = [0.050 + (s - 1000) * 0.000025 for s in row["positions"]]
            tf_msg.currents_ma = list(row["currents"])
            tf_msg.fault_codes = list(row["faults"])
            tf_msg.uart_cycle_sequence = row["cycle_seq"]
            populate_force_feedback(tf_msg, row)
        else:
            tf_msg = None

        # 3. TimedJointCommand
        if self.timed_command_pub is not None and row.get('control_mode') != 3:
            tc_msg = TimedJointCommand()
            tc_msg.header.stamp = stamp_msg
            tc_msg.transaction_start_monotonic_ns = row["cycle_start_ns"]
            tc_msg.transaction_end_monotonic_ns = row["cycle_end_ns"]
            tc_msg.per_axis_tx_start_monotonic_ns = list(row["tx_start_ns"])
            tc_msg.per_axis_tx_done_monotonic_ns = list(row["tx_end_ns"])
            tc_msg.target_positions_meters = [steps_to_meters(s) for s in row["targets"]]
            tc_msg.target_steps = list(row["targets"])
            tc_msg.uart_cycle_sequence = row["cycle_seq"]
        else:
            tc_msg = None

        t_build_done = time.monotonic_ns()
        tc_build_done = time.thread_time_ns()

        # Phase 2: Sequential Publish
        t_pub_js_enter = time.monotonic_ns()
        tc_pub_js_enter = time.thread_time_ns()
        self.joint_state_pub.publish(js_msg)
        t_pub_js_exit = time.monotonic_ns()
        tc_pub_js_exit = time.thread_time_ns()

        t_pub_tf_enter = time.monotonic_ns()
        tc_pub_tf_enter = time.thread_time_ns()
        if self.timed_feedback_pub is not None and tf_msg is not None:
            self.timed_feedback_pub.publish(tf_msg)
        t_pub_tf_exit = time.monotonic_ns()
        tc_pub_tf_exit = time.thread_time_ns()

        t_pub_tc_enter = time.monotonic_ns()
        tc_pub_tc_enter = time.thread_time_ns()
        if self.timed_command_pub is not None and tc_msg is not None:
            self.timed_command_pub.publish(tc_msg)
        t_pub_tc_exit = time.monotonic_ns()
        tc_pub_tc_exit = time.thread_time_ns()

        pc_exit = time.process_time_ns()

        if self.production and self.telemetry_mode in ('production','qualification'):
            record=dict(epoch=row.get('epoch'),cycle_seq=seq,status='published',
                t_child_ready_ns=t_child_ready,t_bridge_recv_ns=t_recv,
                ipc_age_ms=ipc_age_ms,feedback_gap_ms=feedback_gap_ms)
        else:
            record = dict(
                    epoch=row.get("epoch"),
                    cycle_seq=seq,
                    command_seq=row.get("command_seq"),
                    status="published",
                    t_child_ready_ns=t_child_ready,
                    t_bridge_recv_ns=t_recv,
                    t_message_build_done_ns=t_build_done,
                    t_pub_js_enter_ns=t_pub_js_enter,
                    t_pub_js_exit_ns=t_pub_js_exit,
                    t_pub_tf_enter_ns=t_pub_tf_enter,
                    t_pub_tf_exit_ns=t_pub_tf_exit,
                    t_pub_tc_enter_ns=t_pub_tc_enter,
                    t_pub_tc_exit_ns=t_pub_tc_exit,
                    ipc_age_ms=ipc_age_ms,
                    feedback_gap_ms=feedback_gap_ms,
                    bridge_thread_cpu_ns=tc_pub_tc_exit - tc_recv,
                    message_build_thread_cpu_ns=tc_build_done - tc_recv,
                    pub_js_thread_cpu_ns=tc_pub_js_exit - tc_pub_js_enter,
                    pub_tf_thread_cpu_ns=tc_pub_tf_exit - tc_pub_tf_enter,
                    pub_tc_thread_cpu_ns=tc_pub_tc_exit - tc_pub_tc_enter,
                    process_cpu_ns=pc_exit,
                )
        self._records.append(record)

    def _flush_diagnostics(self):
        if self._records:
            append_rows(self.out_dir / "bridge_feedback_pipeline.jsonl", self._records)
            self._records.clear()
        append_rows(
            self.out_dir / "bridge_lifecycle.jsonl",
            [
                dict(
                    event="bridge_stopped",
                    ns=time.monotonic_ns(),
                    total_received=self._total_received,
                    drop_count=self._drop_count,
                    max_ipc_age_ms=self._max_ipc_age_ms,max_feedback_gap_ms=self._max_feedback_gap_ms,
                    ipc_watchlist_count=self._ipc_watchlist_count,gap_watchlist_count=self._gap_watchlist_count,
                    exit_reason=self._exit_reason,health_ok=False,
                    pid=os.getpid(),
                )
            ],
        )

    def stop(self):
        self._running = False
        self._stop_event.set()


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--fd", type=int, required=True)
    p.add_argument("--config", required=True)
    a = p.parse_args()

    s = socket.socket(fileno=a.fd)
    s.setblocking(False)
    config = json.loads(a.config)

    rclpy.init()
    node = Pus6FeedbackBridgeNode(s, config)

    # Clean signal handling
    def sig_handler(*_):
        node.stop()

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    try:
        node.run_bridge_loop()
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        try:
            s.close()
        except OSError:
            pass


if __name__ == "__main__":
    main()
