"""ROS 2 Node for Z-Axis Task Space Force Control (Mode 3).

Stage 2.1: validates explicit source/force validity, aligns q and force using
one FixedLagJointSynchronizer, consumes each feedback cycle at most once and
routes faults to the existing /pus6/pause_all interface. No UART is opened here.
Dry-run commands and stop requests have dedicated topics and cannot reach Child.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
import time
from typing import Optional, Sequence

from ament_index_python.packages import get_package_share_directory
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float64, String
from std_srvs.srv import Trigger
import yaml

from pus6_msgs.msg import TimedJointFeedback
from pus6_research_msgs.msg import AxialForceCommand, ForceFeedback

from pus6_kinematics.analytical_6pus import Analytical6PusKinematics, DEFAULT_NOMINAL_HEIGHT_M
from pus6_kinematics.state_synchronizer import FixedLagJointSynchronizer, SyncStatus
from pus6_force_control.rate_limiter import ForceReferenceRateLimiter
from pus6_force_control.task_space_force_pid import ForcePIDConfig, TaskSpaceForcePID
from pus6_force_control.wrench_allocation import SixAxisWrenchAllocator
from pus6_motor_driver.force_units import NOMINAL_N_PER_G, DEFAULT_FORCE_LIMIT_N
from pus6_force_control.six_axis_force_drive import SixAxisForceDrive, ForceLimitsConfig
from pus6_force_control.force_pipeline import ForceCompensation, TaskSpaceForcePipeline


class TaskSpaceForceControlNode(Node):
    """ROS 2 Node orchestrating Z-axis task space force control."""

    def __init__(self, node_name: str = "pus6_force_control_node", parameter_overrides=None, *, clock=None):
        super().__init__(node_name, parameter_overrides=parameter_overrides)

        self.clock = clock or time.monotonic_ns

        # 1. Config file path declaration & preloading
        self.declare_parameter("config_file", "")
        config_path_str = self.get_parameter("config_file").get_parameter_value().string_value
        yaml_config = self._load_yaml_config(config_path_str)

        pid_y = yaml_config.get("pid", {})
        comp_y = yaml_config.get("compensation", {})
        drive_y = yaml_config.get("drive", {})

        if any(k in drive_y for k in ("min_force_g", "max_force_g")):
            raise ValueError("Convert old drive limits to min_force_n/max_force_n in N")

        # 2. Execution & Safety Parameters
        self.declare_parameter("dry_run", True)
        self.declare_parameter("hardware_write_enabled", False)
        self.declare_parameter("is_hardware_qualified", False)
        self.declare_parameter("fault_stop_policy", "pause_all")
        self.fault_stop_policy = str(self.get_parameter("fault_stop_policy").value)
        if self.fault_stop_policy not in ("pause_all", "external_position_reanchor", "external_mode3_hold"):
            raise ValueError("unsupported force fault stop policy")
        self.last_cycle_result = None
        self.last_axial_command_n = None
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("target_force_z_n", 0.0)
        self.declare_parameter("target_timeout_s", 0.50)  # Target force freshness watchdog
        self.declare_parameter("feedback_timeout_s", 0.20)
        self.declare_parameter("max_feedback_gap_s", 0.20)
        self.declare_parameter("use_fixed_lag", True)
        self.declare_parameter("fixed_lag_delay_ms", 30.0)
        self.declare_parameter("max_fk_residual_m", 1.0e-3)

        # 3. PID parameters (YAML fallback)
        self.declare_parameter("kp", float(pid_y.get("kp", 1.0)))
        self.declare_parameter("ki", float(pid_y.get("ki", 0.0)))
        self.declare_parameter("kd", float(pid_y.get("kd", 0.0)))
        self.declare_parameter("output_min_n", float(pid_y.get("output_min_n", -10.0)))
        self.declare_parameter("output_max_n", float(pid_y.get("output_max_n", 10.0)))
        self.declare_parameter("integral_limit_n", float(pid_y.get("integral_limit_n", 5.0)))
        self.declare_parameter("derivative_filter_tau_s", float(pid_y.get("derivative_filter_tau_s", 0.05)))
        self.declare_parameter("reference_rate_n_s", float(pid_y.get("reference_rate_n_s", 1.0)))
        self.declare_parameter("output_mode", str(pid_y.get("output_mode", "reference_plus_pid")))

        # 4. Compensation parameters (YAML fallback)
        self.declare_parameter("sensor_zero_g", [float(x) for x in comp_y.get("sensor_zero_g", [0.0]*6)])
        self.declare_parameter("direction", [float(x) for x in comp_y.get("direction", [1.0]*6)])
        self.declare_parameter("newtons_per_g", [float(x) for x in comp_y.get("newtons_per_g", [NOMINAL_N_PER_G]*6)])
        self.declare_parameter("actuator_bias_n", [float(x) for x in comp_y.get("actuator_bias_n", [0.0]*6)])

        # 5. Actuator limits (YAML fallback)
        self.declare_parameter("min_force_n", [float(x) for x in drive_y.get("min_force_n", [-DEFAULT_FORCE_LIMIT_N]*6)])
        self.declare_parameter("max_force_n", [float(x) for x in drive_y.get("max_force_n", [DEFAULT_FORCE_LIMIT_N]*6)])
        self.declare_parameter("max_rate_g_s", [float(x) for x in drive_y.get("max_rate_g_s", [500.0]*6)])
        self.declare_parameter("allow_negative_forces", bool(drive_y.get("allow_negative_forces", True)))

        # Retrieve parsed parameters
        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.hardware_write_enabled = bool(self.get_parameter("hardware_write_enabled").value)
        self.is_hardware_qualified = bool(self.get_parameter("is_hardware_qualified").value)
        if self.hardware_write_enabled and not self.is_hardware_qualified:
            raise ValueError("hardware write authorization requires explicit qualification")
        if not self.dry_run and not (self.hardware_write_enabled and self.is_hardware_qualified):
            raise ValueError("non-dry command publication requires hardware authorization and qualification")
        rate_hz = float(self.get_parameter("rate_hz").value)
        self.period_s = 1.0 / max(1.0, rate_hz)
        self.target_force_z_n = float(self.get_parameter("target_force_z_n").value)
        self.target_timeout_s = float(self.get_parameter("target_timeout_s").value)
        self.feedback_timeout_s = float(self.get_parameter("feedback_timeout_s").value)
        self.max_feedback_gap_s = float(self.get_parameter("max_feedback_gap_s").value)
        self.declare_parameter("max_current_ma", 500)
        self.max_current_ma = int(self.get_parameter("max_current_ma").value)
        self.use_fixed_lag = bool(self.get_parameter("use_fixed_lag").value)
        self.fixed_lag_delay_ns = int(float(self.get_parameter("fixed_lag_delay_ms").value) * 1e6)
        self.max_fk_residual_m = float(self.get_parameter("max_fk_residual_m").value)
        for name, value in (("rate_hz", rate_hz), ("feedback_timeout_s", self.feedback_timeout_s),
                            ("max_feedback_gap_s", self.max_feedback_gap_s),
                            ("max_fk_residual_m", self.max_fk_residual_m)):
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be positive and finite")
        if not math.isfinite(self.target_timeout_s) or self.target_timeout_s < 0 or self.fixed_lag_delay_ns < 0:
            raise ValueError("target timeout and Fixed-Lag delay must be finite and nonnegative")

        # Build PID configuration object
        pid_cfg = ForcePIDConfig(
            kp=float(self.get_parameter("kp").value),
            ki=float(self.get_parameter("ki").value),
            kd=float(self.get_parameter("kd").value),
            output_min_n=float(self.get_parameter("output_min_n").value),
            output_max_n=float(self.get_parameter("output_max_n").value),
            integral_limit_n=float(self.get_parameter("integral_limit_n").value),
            derivative_filter_tau_s=float(self.get_parameter("derivative_filter_tau_s").value),
            reference_rate_n_s=float(self.get_parameter("reference_rate_n_s").value),
            output_mode=str(self.get_parameter("output_mode").value),
        )

        # Build ForceCompensation
        comp = ForceCompensation(
            sensor_zero_g=tuple(float(x) for x in self.get_parameter("sensor_zero_g").value),
            direction=tuple(float(x) for x in self.get_parameter("direction").value),
            newtons_per_g=tuple(float(x) for x in self.get_parameter("newtons_per_g").value),
            actuator_bias_n=tuple(float(x) for x in self.get_parameter("actuator_bias_n").value),
        )

        # Build ForceLimitsConfig (strictly decoupled hardware qualification)
        limits_cfg = ForceLimitsConfig(
            min_force_n=tuple(float(x) for x in self.get_parameter("min_force_n").value),
            max_force_n=tuple(float(x) for x in self.get_parameter("max_force_n").value),
            max_rate_g_s=tuple(float(x) for x in self.get_parameter("max_rate_g_s").value),
            feedback_max_age_s=self.feedback_timeout_s,
            allow_negative_forces=bool(self.get_parameter("allow_negative_forces").value),
            is_hardware_qualified=self.is_hardware_qualified,
        )

        # Subsystems: Kinematics, Synchronizer & Pipeline
        self.kinematics = Analytical6PusKinematics()
        self.synchronizer = FixedLagJointSynchronizer(max_history_size=500)
        self.allocator = SixAxisWrenchAllocator(kinematics=self.kinematics)
        self.drive = SixAxisForceDrive(
            bus=None,
            limits_config=limits_cfg,
            hardware_write_enabled=False,  # numerical pipeline never owns UART
            clock=self.clock,
        )
        self.pipeline = TaskSpaceForcePipeline(
            pid_config=pid_cfg,
            drive=self.drive,
            allocator=self.allocator,
            compensation=comp,
            clock=self.clock,
        )

        # Telemetry & Health Tracking
        self.latest_timed_feedback: Optional[TimedJointFeedback] = None
        self.last_feedback_recv_ns: int = 0
        self.last_cycle_seq: int = -1
        self.dropped_cycles_count: int = 0
        self.last_target_update_ns: int = self.clock()
        self.current_pose_estimate: list[float] = [0.0, 0.0, DEFAULT_NOMINAL_HEIGHT_M, 0.0, 0.0, 0.0]
        self.fault_latched: bool = False
        self.fault_reason: str = ""
        self.control_cycles_count: int = 0
        self.last_control_cycle_ns: int = 0
        self.last_aligned_forces_g: Optional[list[float]] = None
        self.last_processed_cycle_seq = -1
        self.last_sync_stamp_ns = 0
        self._last_input_stamps = [0] * 6
        self.stop_request_status = "NONE"
        self.pause_client = self.create_client(Trigger, "/pus6/pause_all")
        self.dry_stop_pub = self.create_publisher(String, "/pus6/force_control/stop_request_dry_run", 10)
        self.reset_service = self.create_service(Trigger, "~/reset_fault", self._reset_fault)
        self.transport_fault_sub = self.create_subscription(
            String, "/pus6/force_control/transport_fault",
            lambda msg: self._trip_fault("TRANSPORT_FAULT", msg.data, self.clock()), 10,
        )

        # Subscriptions
        self.feedback_sub = self.create_subscription(
            TimedJointFeedback,
            "/pus6/joint_feedback/timed",
            self._handle_timed_feedback,
            qos_profile_sensor_data,
        )

        control_cmd_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.target_sub = self.create_subscription(
            Float64,
            "/pus6/force_control/target_force_z",
            self._handle_target_force_z,
            control_cmd_qos,
        )

        # Publishers
        self.axial_cmd_pub = self.create_publisher(
            AxialForceCommand,
            "/pus6/force_control/axial_forces_dry_run" if self.dry_run else "/pus6/command_axial_forces",
            qos_profile_sensor_data,
        )
        self.wrench_pub = self.create_publisher(
            WrenchStamped,
            "/pus6/force_control/reconstructed_wrench",
            qos_profile_sensor_data,
        )
        self.force_fb_pub = self.create_publisher(
            ForceFeedback,
            "/pus6/force_control/calibrated_feedback",
            qos_profile_sensor_data,
        )
        self.diag_pub = self.create_publisher(
            String,
            "/pus6/force_control/diagnostics",
            10,
        )

        # 50 Hz Control Timer
        self.timer = self.create_timer(self.period_s, self._control_cycle)

        self.get_logger().info(
            f"TaskSpaceForceControlNode initialized (rate={rate_hz}Hz, dry_run={self.dry_run}, "
            f"hw_write={self.hardware_write_enabled}, qualified={self.is_hardware_qualified}, "
            f"use_fixed_lag={self.use_fixed_lag})"
        )

    def _load_yaml_config(self, config_path_str: str) -> dict:
        """Load YAML configuration with fallback to default installed config."""
        path = None
        if config_path_str:
            p = Path(config_path_str)
            if p.is_file():
                path = p
        if path is None:
            try:
                pkg_dir = Path(get_package_share_directory("pus6_force_control"))
                p = pkg_dir / "config/force_pid.yaml"
                if p.is_file():
                    path = p
            except Exception:
                pass
        if path is None:
            # Source tree fallback
            p = Path(__file__).resolve().parent.parent / "config/force_pid.yaml"
            if p.is_file():
                path = p

        if path and path.is_file():
            try:
                data = yaml.safe_load(path.read_text())
                if isinstance(data, dict):
                    self.get_logger().info(f"Loaded force parameters from {path}")
                    return data
            except Exception as exc:
                self.get_logger().warn(f"Failed to parse YAML config {path}: {exc}")
        return {}

    def initialize_hold_forces_g(self, force_g_raw, *, now_ns=None):
        """Called by the UART handover owner with its freshly read 0x2C vector."""
        if self.last_cycle_result is not None:
            raise RuntimeError("cannot change hold during active force control")
        hold = self.pipeline.initialize_hold_forces_g(force_g_raw, now_ns=now_ns)
        self.last_axial_command_n = self.pipeline.hold_forces_n.tolist()
        return hold

    def _reset_control_state(self, now_ns):
        # No output is sent by reset. Recovery starts reference/derivative/integral afresh.
        self.pipeline.pid.reset(now_ns=now_ns)
        self.drive.last_commanded_forces_g = list(self.pipeline.hold_forces_g or (0.0,) * 6)
        self.drive.last_command_time_ns = now_ns
        self.last_aligned_forces_g = None

    def _trip_fault(self, status, reason, now_ns):
        if self.fault_latched:
            return
        self.fault_latched = True
        self.fault_reason = reason
        self.synchronizer.clear()
        self._reset_control_state(now_ns)
        # Stop computing/publishing commands; zero reference is NOT a safe stop.
        if not rclpy.ok() or not hasattr(self, 'context') or not self.context.ok():
            self.stop_request_status = "SHUTDOWN"
            return
        if self.dry_run:
            self.stop_request_status = "DRY_RUN_ONLY"
            try:
                self.dry_stop_pub.publish(String(data=json.dumps({"reason": reason, "dry_run": True})))
            except Exception:
                pass
        elif self.fault_stop_policy == "external_position_reanchor":
            # In-process experiment owner observes this latch and executes its
            # terminal handler in finally; production also has the target lease.
            self.stop_request_status = "EXTERNAL_REANCHOR_REQUIRED"
        elif self.hardware_write_enabled and self.is_hardware_qualified:
            try:
                if self.pause_client.service_is_ready():
                    self.stop_request_status = "REQUESTED_NOT_CONFIRMED"
                    future = self.pause_client.call_async(Trigger.Request())
                    future.add_done_callback(self._pause_done)
                else:
                    self.stop_request_status = "SERVICE_UNAVAILABLE_NOT_STOPPED"
            except Exception:
                self.stop_request_status = "SERVICE_UNAVAILABLE_NOT_STOPPED"
        else:
            self.stop_request_status = "UNAUTHORIZED_NOT_STOPPED"
        self.get_logger().error(reason)
        try:
            self._emit_diagnostics(status, {"fault_reason": reason, "stop_request": self.stop_request_status}, now_ns)
        except Exception:
            pass

    def request_terminal_stop(self, reason):
        """Ask the existing owner to exit. A request/service ACK is not stop proof."""
        if not rclpy.ok() or not hasattr(self, 'context') or not self.context.ok():
            self.stop_request_status = "SHUTDOWN"
            return
        self._trip_fault("OPERATOR_STOP", reason, self.clock())

    def _pause_done(self, future):
        try:
            response = future.result()
            self.stop_request_status = "ACKNOWLEDGED_PHYSICAL_STOP_UNVERIFIED" if response.success else "REQUESTED_NOT_CONFIRMED"
        except Exception:
            self.stop_request_status = "REQUEST_FAILED_NOT_STOPPED"
        self._emit_diagnostics("STOP_STATUS", {"stop_request": self.stop_request_status}, self.clock())

    def _reset_fault(self, request, response):
        self.latest_timed_feedback = None
        self.synchronizer.clear()
        self.last_feedback_recv_ns = 0
        self.last_cycle_seq = -1
        self.last_processed_cycle_seq = -1
        self._last_input_stamps = [0] * 6
        self.last_sync_stamp_ns = 0
        self.last_target_update_ns = 0  # require a NEW explicit target after recovery
        self.fault_latched = False
        self.fault_reason = ""
        self.stop_request_status = "NONE"
        self._reset_control_state(self.clock())
        response.success = True
        response.message = "Controller reset; waiting for new feedback and target; motor mode is unchanged"
        return response

    def _feedback_error(self, fb, now_ns):
        fields = (fb.positions_meters, fb.raw_steps, fb.forces_g_raw, fb.force_adc_raw,
                  fb.force_valid, fb.fault_codes, fb.per_axis_rx_done_monotonic_ns)
        if any(len(values) != 6 for values in fields):
            return "incomplete feedback vector"
        if fb.source not in (TimedJointFeedback.SOURCE_MOCK, TimedJointFeedback.SOURCE_HARDWARE):
            return "unknown feedback source"
        if any(code != 0 for code in fb.fault_codes):
            return "Motor hardware fault codes reported: " + str(list(fb.fault_codes))
        if not all(fb.force_valid):
            return "force feedback explicitly invalid or uninitialized"
        if (not np.isfinite(fb.positions_meters).all()
                or any(not -32768 <= f <= 32767 for f in fb.forces_g_raw)
                or any(not 0 <= adc <= 4095 for adc in fb.force_adc_raw)):
            return "force/position/ADC outside protocol domain"
        stamps = [int(t) for t in fb.per_axis_rx_done_monotonic_ns]
        if any(t <= 0 or t > now_ns for t in stamps):
            return "invalid or future source receive timestamp"
        if any(now_ns - t > self.feedback_timeout_s * 1e9 for t in stamps):
            return "Feedback timeout at source timestamp"
        if fb.snapshot_end_monotonic_ns < max(stamps) or fb.snapshot_start_monotonic_ns > fb.snapshot_end_monotonic_ns:
            return "inconsistent snapshot timestamps"
        return None

    def _handle_timed_feedback(self, msg: TimedJointFeedback) -> None:
        if self.fault_latched:
            return
        now_ns = self.clock()
        seq = int(msg.uart_cycle_sequence)
        if seq == self.last_cycle_seq:
            return  # replay cannot refresh either receive watchdog or PID
        if seq < self.last_cycle_seq:
            self._trip_fault("SEQUENCE_ROLLBACK", "feedback sequence rollback", now_ns)
            return
        error = self._feedback_error(msg, now_ns)
        if error:
            self._trip_fault("INVALID_FEEDBACK", error, now_ns)
            return
        if hasattr(msg, 'currents_ma') and len(msg.currents_ma) == 6:
            if max(abs(c) for c in msg.currents_ma) > self.max_current_ma:
                self._trip_fault("CURRENT_LIMIT", f"configured motor current protection exceeded ({max(abs(c) for c in msg.currents_ma)} > {self.max_current_ma} mA)", now_ns)
                return
        stamps = [int(t) for t in msg.per_axis_rx_done_monotonic_ns]
        if self.last_cycle_seq >= 0:
            if any(t <= old for t, old in zip(stamps, self._last_input_stamps)):
                self._trip_fault("SOURCE_TIME_ROLLBACK", "source timestamps did not advance", now_ns)
                return
            if any(t - old > self.max_feedback_gap_s * 1e9 for t, old in zip(stamps, self._last_input_stamps)):
                self._trip_fault("FEEDBACK_GAP", "source feedback gap exceeds configured limit", now_ns)
                return
            self.dropped_cycles_count += max(0, seq - self.last_cycle_seq - 1)
        self.last_cycle_seq = seq
        self._last_input_stamps = stamps
        self.last_feedback_recv_ns = now_ns
        self.latest_timed_feedback = msg
        self.synchronizer.push_feedback(msg)

    def _handle_target_force_z(self, msg: Float64) -> None:
        val = float(msg.data)
        if not math.isfinite(val):
            self._trip_fault("INVALID_TARGET", "non-finite force target", self.clock())
            return
        self.target_force_z_n = val
        self.last_target_update_ns = self.clock()

    def _control_cycle(self) -> None:
        now_ns = self.clock()
        self.control_cycles_count += 1
        if self.fault_latched:
            self._emit_diagnostics("FAULT_LATCHED", {"fault_reason": self.fault_reason,
                                   "stop_request": self.stop_request_status}, now_ns)
            return
        fb = self.latest_timed_feedback
        if fb is None:
            self._emit_diagnostics("WAITING_FOR_FEEDBACK", {}, now_ns)
            return
        error = self._feedback_error(fb, now_ns)
        if error or now_ns - self.last_feedback_recv_ns > self.feedback_timeout_s * 1e9:
            self._trip_fault("FEEDBACK_TIMEOUT", error or "Feedback timeout at receipt", now_ns)
            return
        if self.last_target_update_ns == 0:
            self._emit_diagnostics("WAITING_FOR_TARGET", {}, now_ns)
            return
        if self.target_timeout_s > 0 and now_ns - self.last_target_update_ns > self.target_timeout_s * 1e9:
            self._trip_fault("TARGET_TIMEOUT", "target force input timed out; request configured stop", now_ns)
            return
        if fb.uart_cycle_sequence == self.last_processed_cycle_seq:
            return

        if self.use_fixed_lag:
            query_stamp = self._sync_query_stamp(now_ns,fb)
            sync = self.synchronizer.query_sync_configuration(query_stamp)
            if sync.status != SyncStatus.SYNCED:
                if (sync.status == SyncStatus.WAIT_FUTURE_SAMPLE and self.last_processed_cycle_seq >= 0
                        and (now_ns - getattr(self, 'last_control_cycle_ns', 0) < self.feedback_timeout_s * 1e9
                             or now_ns - self.last_sync_stamp_ns < (self.feedback_timeout_s + self.fixed_lag_delay_ns * 1e-9) * 1e9)):
                    # A host scheduling overrun can place this query just past
                    # one axis. Wait for the next REAL bracket; no extrapolation,
                    # duplicate PID update, reset or zero target. Existing feedback
                    # timeout bounds a persistent failure to produce aligned data.
                    self._emit_diagnostics("WAITING_FOR_SYNC", {"sync_status":sync.status.value},now_ns)
                    return
                if self.last_processed_cycle_seq >= 0:
                    self._trip_fault("SYNC_LOST", "Fixed-Lag lost alignment: " + sync.status.value, now_ns)
                else:
                    self._emit_diagnostics("WAITING_FOR_SYNC", {"sync_status": sync.status.value}, now_ns)
                return
            if (not all(sync.force_valid) or any(sync.fault_codes)
                    or len(set(sync.sources)) != 1
                    or any(now_ns - t > self.feedback_timeout_s * 1e9 for t in sync.per_axis_left_stamps_ns)
                    or any(r - l > self.max_feedback_gap_s * 1e9 for l, r in zip(
                        sync.per_axis_left_stamps_ns, sync.per_axis_right_stamps_ns))):
                self._trip_fault("INVALID_SYNC_FORCE", "invalid/stale force interpolation endpoints", now_ns)
                return
            positions_m = sync.positions_meters
            raw_forces_g = sync.forces_g_raw
            left_stamps, right_stamps = sync.per_axis_left_stamps_ns, sync.per_axis_right_stamps_ns
            weights = sync.per_axis_interp_weights
        else:
            # Only a genuinely common host timestamp can bypass interpolation.
            stamps = [int(t) for t in fb.per_axis_rx_done_monotonic_ns]
            if len(set(stamps)) != 1:
                self._trip_fault("UNALIGNED_FEEDBACK", "per-axis receive times differ; enable Fixed-Lag", now_ns)
                return
            query_stamp = stamps[0]
            positions_m, raw_forces_g = list(fb.positions_meters), list(fb.forces_g_raw)
            left_stamps = right_stamps = stamps
            weights = [0.0] * 6
        if query_stamp <= self.last_sync_stamp_ns:
            return
        try:
            fk = self.kinematics.forward_kinematics_with_metrics(positions_m, initial_guess=self.current_pose_estimate)
            if not fk.converged or not math.isfinite(fk.residual_norm) or fk.residual_norm > self.max_fk_residual_m or not np.isfinite(fk.pose).all():
                raise ValueError("FK nonconvergence or excessive residual")
            self.current_pose_estimate = list(fk.pose)
            cycle = self._compute_force_cycle(raw_forces_g,self.current_pose_estimate,now_ns,query_stamp)
        except Exception as exc:
            self._trip_fault("CONTROL_FAILURE", f"FK/pipeline failure: {exc}", now_ns)
            return
        self._publish_control_cycle(cycle,fb,now_ns,query_stamp,raw_forces_g,positions_m,left_stamps,right_stamps,weights)

    def _sync_query_stamp(self,now_ns,fb):
        return now_ns-self.fixed_lag_delay_ns

    def _publish_control_cycle(self,cycle,fb,now_ns,query_stamp,raw_forces_g,positions_m,
                               left_stamps=None,right_stamps=None,weights=None,source_stamps_ns=None):
        if not rclpy.ok() or not hasattr(self, 'context') or not self.context.ok():
            return
        self.last_cycle_result = cycle
        self.last_processed_cycle_seq = int(fb.uart_cycle_sequence)
        self.last_aligned_forces_g = [float(f) for f in raw_forces_g] if raw_forces_g is not None else None
        if query_stamp is not None:self.last_sync_stamp_ns = query_stamp
        self.last_control_cycle_ns = now_ns

        stamp = self.get_clock().now().to_msg()
        comp = self.pipeline.compensation
        cmd = AxialForceCommand()
        cmd.header.stamp = stamp
        cmd.header.frame_id = "pus6_base"
        cmd.sample_monotonic_ns = now_ns
        encoded_g = [row["target_force_g"] for row in cycle.drive_result["frames"]]
        cmd.force_n = list(comp.command_n(encoded_g))
        self.last_axial_command_n = list(cmd.force_n)
        try:
            self.axial_cmd_pub.publish(cmd)
        except Exception:
            pass
        w = cycle.measured_wrench
        if w is not None:
            wrench = WrenchStamped()
            wrench.header.stamp = stamp
            wrench.header.frame_id = "pus6_base"
            wrench.wrench.force.x, wrench.wrench.force.y, wrench.wrench.force.z = map(float, w[:3])
            wrench.wrench.torque.x, wrench.wrench.torque.y, wrench.wrench.torque.z = map(float, w[3:])
            try:
                self.wrench_pub.publish(wrench)
            except Exception:
                pass
        calibrated = ForceFeedback()
        calibrated.header.stamp = stamp
        calibrated.header.frame_id = "pus6_base"
        calibrated.source = int(fb.source)  # data provenance is independent of dry_run
        calibrated.rx_monotonic_ns = ([query_stamp]*6 if source_stamps_ns is None else list(source_stamps_ns))
        calibrated.force_n = list(comp.feedback_n(raw_forces_g))
        calibrated.valid = [True] * 6  # established above for the actual interpolation endpoints
        calibrated.fault_codes = [0] * 6
        try:
            self.force_fb_pub.publish(calibrated)
        except Exception:
            pass
        self._emit_diagnostics("CONTROL_OK", {
            "feedback_sequence": self.last_processed_cycle_seq, "source": int(fb.source),
            "measurement_monotonic_ns": query_stamp, "source_left_ns": [int(t) for t in left_stamps] if left_stamps is not None else None,
            "source_right_ns": [int(t) for t in right_stamps] if right_stamps is not None else None, "interpolation_weights": list(weights) if weights is not None else None,
            "aligned_forces_g_raw": [float(f) for f in raw_forces_g] if positions_m is not None else None,
            "axis_forces_g_raw": [float(f) for f in raw_forces_g], "aligned_positions_m": [float(q) for q in positions_m] if positions_m is not None else None,
            "hold_forces_g": self.pipeline.hold_forces_g,
            "relative_feedback_n": self.pipeline.relative_feedback_n(raw_forces_g).tolist(),
            "target_z_n": self.target_force_z_n, "measured_z_n": float(w[2]) if w is not None else None,
            "reference_z_n": cycle.pid.reference_n if cycle.pid is not None else None,
            "pid_output_n": cycle.pid.output_n if cycle.pid is not None else None,
            "actuator_limited": cycle.actuator_limited, "dry_run": self.dry_run,
            "hardware_write_enabled": self.hardware_write_enabled,
            "is_hardware_qualified": self.is_hardware_qualified,
        }, now_ns)

    def _compute_force_cycle(self, raw_forces_g, pose, now_ns, measurement_ns):
        return self.pipeline.update(self.target_force_z_n,raw_forces_g,pose,now_ns=now_ns)

    def _emit_diagnostics(self, status: str, details: dict, now_ns: int) -> None:
        """Publish structured JSON status diagnostics."""
        if not rclpy.ok() or not hasattr(self, 'context') or not self.context.ok():
            return
        payload = {
            "status": status,
            "timestamp_ns": now_ns,
            "dry_run": self.dry_run,
            "details": details,
        }
        try:
            self.diag_pub.publish(String(data=json.dumps(payload)))
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = TaskSpaceForceControlNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            if rclpy.ok() and hasattr(node, 'context') and node.context.ok():
                node.request_terminal_stop('controller_process_exit')
        except Exception:
            pass
        finally:
            try:
                node.destroy_node()
            except Exception:
                pass
            rclpy.try_shutdown()


if __name__ == "__main__":
    main()
