"""ROS 2 Node wrapping SixMotorController for 6PUS robot platform with decoupled UART worker."""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass, asdict
from pathlib import Path
import os
import tempfile
from typing import Any
from enum import Enum
import rclpy
from builtin_interfaces.msg import Time
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Int32MultiArray, Int64MultiArray
from std_srvs.srv import Trigger

try:
    from .safety import (
        MOTOR_IDS,
        NUM_MOTORS,
        ABSOLUTE_RAW_LOWER_LIMIT,
        ABSOLUTE_RAW_UPPER_LIMIT,
        NORMAL_MOTION_LOWER_LIMIT,
        NORMAL_MOTION_UPPER_LIMIT,
        DriverError,
        HardwareMode,
        J2F0SafetyViolation,
        J2F0Stop,
        ReadOnlyModeError,
        SafetyViolation,
        TargetValidationError,
        meters_to_steps,
        parse_hardware_mode,
        steps_to_meters,
    )
    from .six_motor_controller import SixAxisStateSnapshot, SixMotorController
    from .j7_six_axis_servo_transport import J7SixAxisServoTransport, ServoRxPolicy
    from .j7_servo_feedback_scheduler import J7ServoFeedbackScheduler
    from .j7_real_uart_adapter import J7UartAdapter, J7StopAdapter, J7AdapterWriteTimeout, J14_UART_WRITE_TIMEOUT_S
    from .communication_recovery import CommunicationRecovery, append_evidence
    from .fake_bus import MockServoBus
    from .stall_diagnostic_recorder import J11StallDiagnosticRecorder
except ImportError:
    from safety import (
        MOTOR_IDS,
        NUM_MOTORS,
        ABSOLUTE_RAW_LOWER_LIMIT,
        ABSOLUTE_RAW_UPPER_LIMIT,
        NORMAL_MOTION_LOWER_LIMIT,
        NORMAL_MOTION_UPPER_LIMIT,
        DriverError,
        HardwareMode,
        J2F0SafetyViolation,
        J2F0Stop,
        ReadOnlyModeError,
        SafetyViolation,
        TargetValidationError,
        meters_to_steps,
        parse_hardware_mode,
        steps_to_meters,
    )
    from six_motor_controller import SixAxisStateSnapshot, SixMotorController
    from j7_six_axis_servo_transport import J7SixAxisServoTransport, ServoRxPolicy
    from j7_servo_feedback_scheduler import J7ServoFeedbackScheduler
    from j7_real_uart_adapter import J7UartAdapter, J7StopAdapter, J7AdapterWriteTimeout, J14_UART_WRITE_TIMEOUT_S
    from pus6_motor_driver.communication_recovery import CommunicationRecovery, append_evidence
    from fake_bus import MockServoBus
    try:
        from stall_diagnostic_recorder import J11StallDiagnosticRecorder
    except ImportError:
        J11StallDiagnosticRecorder = None

try:
    from pus6_msgs.msg import TimedJointCommand, TimedJointFeedback
except ImportError:
    TimedJointCommand = None  # type: ignore
    TimedJointFeedback = None  # type: ignore


@dataclass
class TargetEnvelope:
    """Latest target container for non-blocking producer-consumer buffering."""
    targets_steps: list[int]
    seq: int
    stamp_ns: int
    consumed: bool = False


@dataclass
class ServoTelemetrySnapshot:
    """Decoupled telemetry snapshot container for asynchronous publication."""
    uart_cycle_seq: int
    cycle_start_ns: int
    cycle_end_ns: int
    stamp_sec: int
    stamp_nanosec: int
    positions_steps: list[int]
    currents_mA: list[int]
    fault_codes: list[int]
    rx_done_ns: list[int]
    dispatched_targets: list[int]
    tx_start_ns: list[int]
    tx_end_ns: list[int]
    diag_data: list[float] | tuple[int | float, ...]
    benchmark_seq_id: int
    t_entry_ns: int


class Pus6MotorDriverNode(Node):
    """ROS 2 Node providing decoupled command dispatch and sensor telemetry for 6PUS platform."""

    def __init__(
        self,
        *,
        hardware_mode: str | HardwareMode | None = None,
        write_rate_hz: float | None = None,
        poll_rate_hz: float | None = None,
        read_mode: str | None = None,
        port: str | None = None,
        baudrate: int | None = None,
        timeout: float | None = None,
        auto_connect: bool | None = None,
        auto_configure: bool | None = None,
        publish_raw_diagnostics: bool | None = None,
        enable_j2_f0_instrumentation: bool | None = None,
        maintenance_mode: bool | None = None,
        use_j7_servo: bool | None = None,
        servo_speed: int | None = None,
        servo_scheduler_mode: str | None = None,
        telemetry_publish_mode: str | None = None,
        feedback_loss_policy: str | None = None,
        bus: Any | None = None,
        node_name: str = "pus6_motor_driver_node",
        **kwargs,
    ) -> None:
        super().__init__(node_name, **kwargs)
        self.mock_bus = bus if isinstance(bus, MockServoBus) else None

        # Declare parameters (default to safe mock mode)
        self.declare_parameter("port", "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0")
        self.declare_parameter("baudrate", 921600)
        self.declare_parameter("timeout", 0.05)
        self.declare_parameter("poll_rate_hz", 10.0)        # Background low-rate telemetry
        self.declare_parameter("write_rate_hz", 50.0)       # Dedicated high-rate UART dispatch loop
        self.declare_parameter("target_timeout_sec", 0.20)  # Watchdog threshold for target stream (200 ms)
        self.declare_parameter("timeout_behavior", "stop_write") # "stop_write" | "hold" | "pause"
        self.declare_parameter("reconnect_max_jump_steps", 10)   # Max jump allowed when recovering from timeout (10 steps = 0.25mm)
        self.declare_parameter("hardware_mode", "mock")     # Default must be "mock"
        self.declare_parameter("mock_hardware", True)       # Deprecated backward-compat alias
        self.declare_parameter("auto_connect", True)
        self.declare_parameter("auto_configure", False)     # Never auto-configure on launch
        self.declare_parameter("read_mode", "full_state")   # "full_state" | "position_only"
        self.declare_parameter("publish_raw_diagnostics", True)
        self.declare_parameter("enable_j2_f0_instrumentation", False)
        self.declare_parameter("maintenance_mode", False)
        self.declare_parameter("use_j7_servo", True)
        self.declare_parameter("servo_speed", 50)
        self.declare_parameter("servo_scheduler_mode", "ros_timer")  # "ros_timer" (default) | "dedicated_thread"
        self.declare_parameter("telemetry_publish_mode", "auto")     # "auto" (default) | "inline" | "async_latest"
        self.declare_parameter("feedback_loss_policy", "default")    # "default" (J7/J8 strict 4-miss) | "age_based" (J11 research mode)
        self.declare_parameter("enable_runtime_diagnostics", False)  # Runtime-Fix-1B: Default False (explicit flag required)

        p_port = port if port is not None else self.get_parameter("port").get_parameter_value().string_value
        p_baudrate = baudrate if baudrate is not None else self.get_parameter("baudrate").get_parameter_value().integer_value
        p_timeout = timeout if timeout is not None else self.get_parameter("timeout").get_parameter_value().double_value
        p_poll_rate = poll_rate_hz if poll_rate_hz is not None else self.get_parameter("poll_rate_hz").get_parameter_value().double_value
        self.use_j7_servo = use_j7_servo if use_j7_servo is not None else self.get_parameter("use_j7_servo").get_parameter_value().bool_value
        self.servo_speed = servo_speed if servo_speed is not None else self.get_parameter("servo_speed").get_parameter_value().integer_value
        self.write_rate_hz = write_rate_hz if write_rate_hz is not None else self.get_parameter("write_rate_hz").get_parameter_value().double_value
        self.target_timeout_sec = self.get_parameter("target_timeout_sec").get_parameter_value().double_value
        self.timeout_behavior = self.get_parameter("timeout_behavior").get_parameter_value().string_value
        self.reconnect_max_jump_steps = self.get_parameter("reconnect_max_jump_steps").get_parameter_value().integer_value
        p_sched_mode = self.get_parameter("servo_scheduler_mode").get_parameter_value().string_value
        self.servo_scheduler_mode = servo_scheduler_mode if servo_scheduler_mode is not None else p_sched_mode
        self._servo_thread: threading.Thread | None = None
        self._servo_thread_stop_event = threading.Event()

        p_fb_policy = self.get_parameter("feedback_loss_policy").get_parameter_value().string_value
        self.feedback_loss_policy = feedback_loss_policy if feedback_loss_policy is not None else p_fb_policy

        p_telem_mode = self.get_parameter("telemetry_publish_mode").get_parameter_value().string_value
        raw_telem_mode = telemetry_publish_mode if telemetry_publish_mode is not None else p_telem_mode
        if raw_telem_mode == "auto":
            if self.servo_scheduler_mode == "dedicated_thread":
                self.telemetry_publish_mode = "async_latest"
            else:
                self.telemetry_publish_mode = "inline"
        else:
            self.telemetry_publish_mode = raw_telem_mode

        self._telemetry_lock = threading.Lock()
        self._telemetry_ready_event = threading.Event()
        self._telemetry_thread_stop_event = threading.Event()
        self._latest_telemetry_snapshot: ServoTelemetrySnapshot | None = None
        self._telemetry_thread: threading.Thread | None = None
        mode_str = hardware_mode if hardware_mode is not None else self.get_parameter("hardware_mode").get_parameter_value().string_value
        p_auto_connect = auto_connect if auto_connect is not None else self.get_parameter("auto_connect").get_parameter_value().bool_value
        p_auto_configure = auto_configure if auto_configure is not None else self.get_parameter("auto_configure").get_parameter_value().bool_value
        self.read_mode = read_mode if read_mode is not None else self.get_parameter("read_mode").get_parameter_value().string_value
        self.publish_raw_diagnostics = publish_raw_diagnostics if publish_raw_diagnostics is not None else self.get_parameter("publish_raw_diagnostics").get_parameter_value().bool_value
        self.enable_j2_f0_instrumentation = enable_j2_f0_instrumentation if enable_j2_f0_instrumentation is not None else self.get_parameter("enable_j2_f0_instrumentation").get_parameter_value().bool_value
        self.maintenance_mode = maintenance_mode if maintenance_mode is not None else self.get_parameter("maintenance_mode").get_parameter_value().bool_value
        if self.enable_j2_f0_instrumentation:
            self.use_j7_servo = False
        else:
            self.use_j7_servo = use_j7_servo if use_j7_servo is not None else self.get_parameter("use_j7_servo").get_parameter_value().bool_value
        self.servo_speed = servo_speed if servo_speed is not None else self.get_parameter("servo_speed").get_parameter_value().integer_value

        self.hardware_mode = parse_hardware_mode(mode_str)
        self.runtime_error: Exception | None = None
        self._last_state_snapshot: SixAxisStateSnapshot | None = None
        self._last_positions_snapshot: Any = None
        self._last_commanded_steps: list[int] | None = None

        # Persistent 6-axis state cache for J7 round-robin feedback
        self._latest_positions_steps: dict[int, int] = {mid: 1000 for mid in MOTOR_IDS}
        self._latest_currents: dict[int, int] = {mid: 0 for mid in MOTOR_IDS}
        self._latest_faults: dict[int, int] = {mid: 0 for mid in MOTOR_IDS}
        self._latest_rx_done_ns: dict[int, int] = {mid: 0 for mid in MOTOR_IDS}
        self.j7_adapter = None
        self.j7_stop_adapter: J7StopAdapter | None = None
        self.j7_writer: J7SixAxisServoTransport | None = None
        self.j7_feedback: J7ServoFeedbackScheduler | None = None
        self._force_hold_reference_monotonic_ns: int = 0
        self._force_hold_g: list[int] = [0] * 6

        self.get_logger().info(
            f"Initializing Pus6MotorDriverNode (mode={self.hardware_mode.value}, "
            f"write_rate={self.write_rate_hz}Hz, timeout={self.target_timeout_sec*1e3:.0f}ms, "
            f"poll_rate={p_poll_rate}Hz, port={p_port}, baud={p_baudrate}, use_j7_servo={self.use_j7_servo})"
        )

        self.controller = SixMotorController(
            port=p_port,
            baudrate=p_baudrate,
            timeout=p_timeout,
            hardware_mode=self.hardware_mode,
        )

        # Standard ROS 2 Feedback Publisher
        self.joint_state_pub = self.create_publisher(JointState, "/pus6/joint_states", 10)

        # Raw Diagnostic Telemetry Publishers
        self.raw_steps_pub = self.create_publisher(Int32MultiArray, "/pus6/joint_steps_raw", 10)
        self.raw_currents_pub = self.create_publisher(Int32MultiArray, "/pus6/joint_currents_raw", 10)
        self.raw_forces_pub = self.create_publisher(Int32MultiArray, "/pus6/joint_forces_raw", 10)
        self.raw_temps_pub = self.create_publisher(Int32MultiArray, "/pus6/joint_temperatures_raw", 10)
        self.raw_faults_pub = self.create_publisher(Int32MultiArray, "/pus6/joint_faults_raw", 10)

        # Driver Performance & Scheduling Diagnostics Publisher
        # [target counters, target age, 6x0x29 ms, RX drain ms, 0x30 ms, worker ms, stop age ms]
        self.driver_diagnostics_pub = self.create_publisher(
            Float64MultiArray, "/pus6/driver_diagnostics", 10
        )

        # Dedicated Benchmark Stamp Publisher [seq, t_pub_mono_ns] (gated behind param)
        if self.enable_j2_f0_instrumentation:
            self.benchmark_stamp_pub = self.create_publisher(
                Int64MultiArray, "/pus6/benchmark_stamp", qos_profile_sensor_data
            )
        else:
            self.benchmark_stamp_pub = None
        self._benchmark_seq_id = 0
        self.benchmark_publish_timestamps_ns: list[int] = []

        # J5-C1 Timed Telemetry Publishers (Host CLOCK_MONOTONIC Timing)
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
        self._uart_cycle_seq = 0

        # Latest-Value-Wins decoupling buffer and statistics
        self._target_lock = threading.Lock()
        self._latest_target: TargetEnvelope | None = None
        self._target_in_count: int = 0
        self._target_overwrite_count: int = 0
        self._target_consumed_count: int = 0
        self._deadline_miss_count: int = 0
        self._last_target_age_ms: float = 0.0
        self._last_write_duration_ms: float = 0.0
        self._last_write_stamp_ns: int = 0
        self._last_target_rx_time_ns: int = 0
        self._stale_target_timeout_count: int = 0
        self._stream_timed_out: bool = False
        self._j7_fault_latched = False
        self._j7_fault_reason: str | None = None
        self._j7_fault_log_emitted = False
        self._consecutive_comm_failures: int = 0
        p_enable_diag = self.get_parameter("enable_runtime_diagnostics").get_parameter_value().bool_value
        self.enable_runtime_diagnostics = p_enable_diag
        self._stall_recorder = (
            J11StallDiagnosticRecorder(enabled=self.enable_runtime_diagnostics)
            if (J11StallDiagnosticRecorder is not None and self.enable_runtime_diagnostics)
            else None
        )
        self._communication_recovery = None
        self._recovery_in_progress = False
        self._fresh_state_only = os.environ.get('PUS6_J14_RECOVERY_STAGE') == 'fresh_state_only'
        self._fresh_state_hold = False
        self._static_resume = os.environ.get('PUS6_J14_RECOVERY_STAGE') == 'static_resume'
        self._static_recovery_target = None
        self._reanchor_first_target = None
        self._reanchor_ack = threading.Event()
        self._reanchor_expected = None

        # Segmented Timing and Contention Instrumentation
        self._last_worker_start_ns: int = 0
        self._target_fetch_ms: float = 0.0
        self._final_preflight_ms: float = 0.0
        self._uart_lock_wait_ms: float = 0.0
        self._six_axis_uart_transaction_ms: float = 0.0
        self._rx_classification_ms: float = 0.0
        self._feedback_transaction_ms: float = 0.0
        self._watchdog_stop_age_ms: float = 0.0
        self._feedback_parse_ms: float = 0.0
        self._ros_publish_ms: float = 0.0
        self._worker_total_ms: float = 0.0
        self._worker_start_to_start_ms: float = 0.0
        self._per_motor_ms: dict[int, float] = {}
        self._background_poll_count: int = 0
        self._background_poll_during_active_count: int = 0
        self._uart_lock_contention_count: int = 0
        self._timeout_events: list[dict[str, Any]] = []
        self._slew_recovery_count: int = 0
        self._last_slew_alpha: float = 1.0
        self._last_slew_original_gap: int = 0
        self._last_slew_sent_gap: int = 0
        self._last_slew_remaining_gap: int = 0

        # Dedicated Callback Groups for thread decoupling in MultiThreadedExecutor
        self.sub_cb_group = MutuallyExclusiveCallbackGroup()
        self._comm_anchor_pub = self.create_publisher(Int64MultiArray, '/pus6/communication_reanchor', 10)
        self._comm_anchor_sub = self.create_subscription(
            Int64MultiArray, '/pus6/communication_reanchor_ack', self._on_communication_reanchor_ack,
            10, callback_group=self.sub_cb_group)
        self.worker_cb_group = MutuallyExclusiveCallbackGroup()
        self.poll_cb_group = MutuallyExclusiveCallbackGroup()

        # Dedicated QoS for high-rate continuous control stream: Latest-Value-Wins (Keep last 1, Best Effort)
        control_cmd_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.declare_parameter("command_ingress_mode", "steps") # "steps" (authoritative) | "meters" | "all"
        self.command_ingress_mode = self.get_parameter("command_ingress_mode").get_parameter_value().string_value

        # Subscriptions (Non-blocking callbacks isolated in sub_cb_group)
        # Default authoritative command ingress: steps only (prevents dual-topic overwrite race)
        if self.command_ingress_mode in ("steps", "all"):
            self.cmd_steps_sub = self.create_subscription(
                Int32MultiArray,
                "/pus6/command_joint_steps",
                self._handle_command_joint_steps,
                control_cmd_qos,
                callback_group=self.sub_cb_group,
            )
        else:
            self.cmd_steps_sub = None

        if self.command_ingress_mode in ("meters", "all"):
            self.cmd_positions_sub = self.create_subscription(
                Float64MultiArray,
                "/pus6/command_joint_positions",
                self._handle_command_positions_meters,
                control_cmd_qos,
                callback_group=self.sub_cb_group,
            )
        else:
            self.cmd_positions_sub = None

        self.cmd_single_step_sub = self.create_subscription(
            Int32MultiArray,
            "/pus6/command_single_step",
            self._handle_command_single_step,
            10,
            callback_group=self.sub_cb_group,
        )

        # Services
        self.pause_srv = self.create_service(
            Trigger,
            "/pus6/pause_all",
            self._handle_pause_all,
            callback_group=self.sub_cb_group,
        )
        self.config_srv = self.create_service(
            Trigger,
            "/pus6/configure_motors",
            self._handle_configure_motors,
            callback_group=self.sub_cb_group,
        )

        self.joint_names = [f"motor_{mid}" for mid in MOTOR_IDS]
        self.is_j7_servo_active = bool(self.use_j7_servo and not self.enable_j2_f0_instrumentation and abs(self.write_rate_hz - 50.0) < 1e-3)
        # J14-A1.8: isolate only the performance-diagnostics channel.
        # Feedback/fault publications and the default execution path stay live.
        self._defer_driver_diagnostics = (
            os.environ.get('PUS6_J14_DEFER_DRIVER_DIAGNOSTICS') == '1'
            and self.is_j7_servo_active and self.telemetry_publish_mode == 'async_latest'
        )

        if p_auto_connect:
            if self.is_j7_servo_active:
                try:
                    self._init_j7_servo_backend(p_port, p_baudrate, p_timeout)
                except Exception as exc:
                    self.get_logger().error(f"Failed to initialize J7 Servo backend on startup: {exc}")
            else:
                try:
                    self.controller.connect()
                    if p_auto_configure:
                        if self.hardware_mode == HardwareMode.READ_ONLY:
                            self.get_logger().warn("auto_configure skipped: Node is in READ_ONLY mode.")
                        else:
                            self.controller.configure_all()
                except Exception as exc:
                    self.get_logger().error(f"Failed to connect to hardware on startup: {exc}")

        # Dedicated high-rate UART dispatch worker (Latest-Value-Wins consumer)
        if self.write_rate_hz > 0.0:
            if self.servo_scheduler_mode == "dedicated_thread":
                self.write_timer = None
                self._servo_thread = threading.Thread(
                    target=self._servo_dedicated_worker_loop,
                    name="Pus6ServoThread",
                    daemon=True,
                )
                self._servo_thread.start()
                self.get_logger().info(
                    f"Servo Loop Scheduling: DEDICATED_THREAD mode engaged ({self.write_rate_hz:.1f}Hz absolute deadline)."
                )
            else:
                write_interval = 1.0 / max(1.0, self.write_rate_hz)
                self.write_timer = self.create_timer(
                    write_interval, self._uart_worker_cycle, callback_group=self.worker_cb_group
                )
                self.get_logger().info(
                    f"Servo Loop Scheduling: ROS_TIMER mode engaged (default, {self.write_rate_hz:.1f}Hz)."
                )
        else:
            self.write_timer = None

        # Asynchronous Telemetry Publisher Worker (J11-RT2B)
        if self.telemetry_publish_mode == "async_latest":
            self._telemetry_thread = threading.Thread(
                target=self._telemetry_publisher_loop,
                name="Pus6TelemetryThread",
                daemon=True,
            )
            self._telemetry_thread.start()
            self.get_logger().info(
                "Telemetry Publication: ASYNC_LATEST mode engaged (dedicated Pus6TelemetryThread worker)."
            )
        else:
            self.get_logger().info(
                "Telemetry Publication: INLINE mode engaged (synchronous in servo cycle)."
            )

        self.get_logger().info(
            f"Feedback Loss Policy: {self.feedback_loss_policy.upper()} engaged."
        )

        # Background low-rate polling timer (in poll_cb_group) - only for legacy non-J7 mode
        if not self.use_j7_servo and p_poll_rate > 0.0:
            poll_interval = 1.0 / max(1.0, p_poll_rate)
            self.poll_timer = self.create_timer(
                poll_interval, self._background_poll_and_publish, callback_group=self.poll_cb_group
            )
        else:
            self.poll_timer = None

    def _init_j7_servo_backend(self, p_port: str, p_baudrate: int, p_timeout: float) -> None:
        """Initializes the validated J7 50Hz Servo transport and round-robin feedback scheduler."""
        if self.mock_bus is not None:
            self.j7_adapter = self.mock_bus
        elif self.hardware_mode == HardwareMode.MOCK:
            self.mock_bus = MockServoBus(initial_positions=[1000] * NUM_MOTORS)
            self.j7_adapter = self.mock_bus
        else:
            import serial
            ser = serial.Serial(
                port=p_port,
                baudrate=p_baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=min(0.005, p_timeout),
                write_timeout=J14_UART_WRITE_TIMEOUT_S,
            )
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            self.j7_adapter = J7UartAdapter(
                ser,
                baudrate=p_baudrate,
                timeout_s=0.010,
                min_frame_interval_s=0.0001,
                tx_queue_diagnostics=self.enable_runtime_diagnostics,
                read_observer=(self._stall_recorder.record_single_read
                               if self._stall_recorder is not None else None),
            )

        self.j7_stop_adapter = J7StopAdapter(self.j7_adapter, audit_observer=self._persist_pause_audit)
        self.j7_writer = J7SixAxisServoTransport(
            self.j7_adapter,
            cycle_rate_hz=self.write_rate_hz,
            rx_policy=ServoRxPolicy.DISCARD_BEFORE_TX,
            min_frame_interval_s=0.0006,
            rx_drain_budget_s=0.0040,
            pause_callback=self.j7_stop_adapter,
            recoverable_write_timeouts=True,
        )
        max_fb_age_s = 1.0 if self.feedback_loss_policy == "age_based" else None
        self.j7_feedback = J7ServoFeedbackScheduler(
            self.j7_writer,
            full_scan_period_s=1e9,
            feedback_budget_s=0.015,
            servo_cycle_period_s=1.0 / self.write_rate_hz,
            timeout_escalation_misses=4,
            max_feedback_age_s=max_fb_age_s,
            warning_feedback_age_s=0.2833,
            hard_stop_feedback_age_s=1.0,
            feedback_loss_policy=self.feedback_loss_policy,
            warn_callback=self._on_feedback_timeout_warning,
        )

        # Preflight full status scan across ID1..ID6
        t_preflight_ns = time.monotonic_ns()
        preflight_audit = self.j7_feedback.full_scan(
            servo_cycle_id=0,
            servo_cycle_timestamp_ns=t_preflight_ns,
        )
        for sample in preflight_audit.samples:
            if sample.position is not None:
                self._latest_positions_steps[sample.motor_id] = sample.position
            if sample.current_mA is not None:
                self._latest_currents[sample.motor_id] = sample.current_mA
            if sample.fault is not None:
                self._latest_faults[sample.motor_id] = sample.fault
            self._latest_rx_done_ns[sample.motor_id] = sample.timestamp_ns

        self._last_commanded_steps = None

        if self.hardware_mode != HardwareMode.READ_ONLY:
            self.j7_writer.configure_servo(speed=self.servo_speed)
            self.get_logger().info(f"J7 Servo backend configured mode=1, speed={self.servo_speed} successfully.")

    def _handle_command_positions_meters(self, msg: Float64MultiArray) -> None:
        """Non-blocking ingestion of 6-axis position targets in SI meters."""
        if self.hardware_mode == HardwareMode.READ_ONLY:
            self.get_logger().error("Command rejected: pus6_motor_driver is in READ_ONLY mode.")
            return

        if len(msg.data) != NUM_MOTORS:
            self.get_logger().error(
                f"Invalid target meters vector length: {len(msg.data)} (expected {NUM_MOTORS}). Ignored."
            )
            return

        targets_steps = [meters_to_steps(p) for p in msg.data]
        self._enqueue_latest_target(targets_steps)

    def _handle_command_joint_steps(self, msg: Int32MultiArray) -> None:
        """Non-blocking ingestion of 6-axis position targets in integer steps."""
        if self.hardware_mode == HardwareMode.READ_ONLY:
            self.get_logger().error("Command rejected: pus6_motor_driver is in READ_ONLY mode.")
            return

        targets_steps = list(msg.data)
        if len(targets_steps) != NUM_MOTORS:
            self.get_logger().error(
                f"Invalid target steps vector length: {len(targets_steps)} (expected {NUM_MOTORS}). Ignored."
            )
            return

        self._enqueue_latest_target(targets_steps)

    def _enqueue_latest_target(self, targets_steps: list[int]) -> None:
        """Atomic Latest-Value-Wins buffer update with timeout recovery gating."""
        now_ns = time.perf_counter_ns()
        with self._target_lock:
            if self._recovery_in_progress or self._fresh_state_hold or self._static_recovery_target is not None:
                # Do not accumulate commands produced by an old shaper trajectory.
                return
            # If recovering from a timed-out state, verify jump threshold against last confirmed position
            if self._stream_timed_out:
                last_known = self._last_commanded_steps or (
                    self._last_state_snapshot.positions_steps if self._last_state_snapshot else None
                )
                if last_known is not None:
                    max_jump = max(abs(t - p) for t, p in zip(targets_steps, last_known))
                    if max_jump > self.reconnect_max_jump_steps:
                        self.get_logger().error(
                            f"Timeout recovery target rejected: max jump {max_jump} steps > limit {self.reconnect_max_jump_steps} steps. "
                            "Target generator must re-anchor before resuming.",
                            throttle_duration_sec=1.0,
                        )
                        return
                self.get_logger().info(
                    f"Target stream re-armed after timeout (max jump: {max_jump if last_known else 0} steps)."
                )
                self._stream_timed_out = False

            self._last_target_rx_time_ns = now_ns
            if self._latest_target is not None and not self._latest_target.consumed:
                self._target_overwrite_count += 1
            self._latest_target = TargetEnvelope(
                targets_steps=targets_steps,
                seq=self._target_in_count,
                stamp_ns=now_ns,
                consumed=False,
            )
            self._target_in_count += 1

    def _communication_audit_dir(self) -> Path:
        if self.hardware_mode != HardwareMode.ACTIVE or self.mock_bus is not None:
            return Path(tempfile.gettempdir()) / f'j14_mock_recovery_{os.getpid()}_{id(self)}'
        if self._stall_recorder is not None:
            return self._stall_recorder.get_effective_output_dir()
        for name in ('/tmp/j14_latest_run_dir', '/tmp/j11_latest_run_dir'):
            pointer = Path(name)
            if pointer.exists() and pointer.read_text().strip():
                return Path(pointer.read_text().strip())
        return Path(tempfile.gettempdir()) / f'j14_communication_recovery_{os.getpid()}'

    def _persist_pause_audit(self, audit) -> None:
        record = asdict(audit)
        record['source'] = 'mock' if self.mock_bus is not None or self.hardware_mode == HardwareMode.MOCK else 'hardware'
        append_evidence(self._communication_audit_dir() / 'stop_pause_audit.jsonl', record)

    def _on_communication_reanchor_ack(self, msg) -> None:
        if self._static_resume:
            payload = list(msg.data)
            if len(payload) == 13 and self._reanchor_expected is not None and payload[:7] == self._reanchor_expected:
                self._reanchor_first_target = payload[7:]
                self._reanchor_ack.set()
            return
        if self._reanchor_expected is not None and list(msg.data) == self._reanchor_expected:
            self._reanchor_ack.set()

    def _reanchor_communication_state(self, positions, epoch_ns, deadline_ns) -> bool:
        old = dict(old_command=self._last_commanded_steps,
                   old_latest_target=asdict(self._latest_target) if self._latest_target is not None else None,
                   old_last_write_ns=self._last_write_stamp_ns)
        with self._target_lock:
            self._latest_target = None
            self._last_commanded_steps = list(positions)
            self._last_write_stamp_ns = time.monotonic_ns()
            self._last_target_rx_time_ns = 0
            self._stream_timed_out = True  # Existing jump admission checks use the fresh anchor.
        self._reanchor_expected = [epoch_ns, *positions]
        self._reanchor_first_target = None
        self._reanchor_ack.clear()
        while self._comm_anchor_pub.get_subscription_count() == 0:
            if time.monotonic_ns() >= deadline_ns:
                return False
            time.sleep(.010)
        msg = Int64MultiArray()
        msg.data = self._reanchor_expected
        self._comm_anchor_pub.publish(msg)
        confirmed = self._reanchor_ack.wait(max(0.0, (deadline_ns-time.monotonic_ns())/1e9))
        if self._static_resume:
            append_evidence(self._communication_audit_dir()/'driver_reanchor_audit.jsonl',
                dict(epoch_ns=epoch_ns, before=old, fresh_positions=list(positions),
                     new_last_command=list(self._last_commanded_steps), latest_target_cleared=self._latest_target is None,
                     stream_timed_out=self._stream_timed_out, reanchor_confirmed=confirmed,
                     first_control_shaper_target=self._reanchor_first_target, reanchor_done_ns=time.monotonic_ns()))
        return confirmed

    def _recover_communication_timeout(self, exc, candidate_targets) -> None:
        self._recovery_in_progress = True
        try:
            if self._communication_recovery is None:
                self._communication_recovery = CommunicationRecovery(
                    self.j7_writer, self.j7_stop_adapter, self.j7_feedback,
                    reanchor=self._reanchor_communication_state,
                    audit_path=self._communication_audit_dir() / 'communication_recovery_events.jsonl',
                    recovery_jump_limit=self.reconnect_max_jump_steps,
                    fresh_state_only=self._fresh_state_only, static_resume=self._static_resume,
                    first_resume_target=lambda: self._reanchor_first_target)
            result = self._communication_recovery.recover(exc, candidate_targets,
                old_command=self._last_commanded_steps)
            for sample in result.feedback_audit.samples:
                self._latest_positions_steps[sample.motor_id] = sample.position
                self._latest_currents[sample.motor_id] = sample.current_mA
                self._latest_faults[sample.motor_id] = sample.fault
                self._latest_rx_done_ns[sample.motor_id] = sample.timestamp_ns
            if self._fresh_state_only:
                self._fresh_state_hold = True
                if self._stall_recorder is not None:
                    self._stall_recorder.finish_cycle()
                self.get_logger().warn('J14-A1.4 REFRESH_STATE PASS: HOLD; motion inhibited; no re-anchor or resume')
                return
            if self._static_resume:
                self._static_recovery_target = list(result.event['first_resume_target'])
                self._uart_cycle_seq = result.event['first_resume_cycle_seq']
                if self._stall_recorder is not None:
                    self._stall_recorder.finish_cycle()
                self.get_logger().warn('J14-A1.5 REANCHOR PASS: next production cycle resumes static reference only')
                return
            self._uart_cycle_seq = result.verification_audit.cycle_id + 1
            if self._stall_recorder is not None:
                self._stall_recorder.finish_cycle()
            self.get_logger().warn(
                f'COMMUNICATION_TRANSIENT recovered: cycle={exc.cycle_audit.cycle_id} '
                f'ID{exc.cycle_audit.error_motor_id}, pause=6/6, fresh_feedback=6/6, '
                f'recovery_count={self._communication_recovery.recovery_count}')
        finally:
            self._recovery_in_progress = False

    def _execute_safety_preflight(self, targets_steps: list[int]) -> bool:
        """Exhaustive safety preflight check right before issuing 6-axis UART command."""
        # 1. Connection check
        is_conn = (self.j7_adapter is not None) if self.is_j7_servo_active else self.controller.is_connected
        if not is_conn:
            self.get_logger().error("Safety preflight FAIL: Controller not connected.")
            return False

        # 2. Dimensions check
        if len(targets_steps) != NUM_MOTORS:
            self.get_logger().error(f"Safety preflight FAIL: Target length {len(targets_steps)} != {NUM_MOTORS}.")
            return False

        # 3. Hardware absolute range [0, 2000] steps (0 ~ 50mm)
        for i, s in enumerate(targets_steps):
            if not (ABSOLUTE_RAW_LOWER_LIMIT <= s <= ABSOLUTE_RAW_UPPER_LIMIT):
                self.get_logger().error(f"Safety preflight FAIL: Motor {i+1} target {s} out of absolute range [{ABSOLUTE_RAW_LOWER_LIMIT}, {ABSOLUTE_RAW_UPPER_LIMIT}].")
                return False

        # 4. Normal J4 commands stay in the operating envelope. A raw PARK
        # target is accepted only when the node is explicitly in maintenance mode.
        if not self.maintenance_mode:
            for i, s in enumerate(targets_steps):
                if not (NORMAL_MOTION_LOWER_LIMIT <= s <= NORMAL_MOTION_UPPER_LIMIT):
                    self.get_logger().error(
                        f"Safety preflight FAIL: Motor {i+1} target {s} out of normal operating envelope "
                        f"[{NORMAL_MOTION_LOWER_LIMIT}, {NORMAL_MOTION_UPPER_LIMIT}]. "
                        "Set maintenance_mode=true only for an authorized PARK workflow."
                    )
                    return False

        # 5. Inter-frame delta check (against last commanded steps)
        if self._last_commanded_steps is not None:
            max_step_delta = 20  # 20 steps = 0.50 mm (final hard safety preflight gate)
            for i, (t_new, t_old) in enumerate(zip(targets_steps, self._last_commanded_steps)):
                if abs(t_new - t_old) > max_step_delta:
                    self.get_logger().error(
                        f"Safety preflight FAIL: Motor {i+1} inter-frame delta {abs(t_new - t_old)} > {max_step_delta} steps."
                    )
                    return False

        # 6. Fresh feedback & Motor Fault Gate
        if self._last_state_snapshot is not None:
            now_ns = time.perf_counter_ns()
            feedback_age_ms = (now_ns - self._last_state_snapshot.timestamp_end_ns) / 1e6
            if feedback_age_ms > 1000.0 and self.hardware_mode == HardwareMode.ACTIVE:
                self.get_logger().error(
                    f"Safety preflight FAIL: State feedback is stale ({feedback_age_ms:.1f}ms > 1000ms)."
                )
                return False

            for i, f in enumerate(self._last_state_snapshot.fault_codes):
                if f != 0:
                    self.get_logger().error(f"Safety preflight FAIL: Motor {i+1} has active fault code {f}.")
                    return False
        elif self.is_j7_servo_active:
            for mid in MOTOR_IDS:
                if self._latest_faults.get(mid, 0) != 0:
                    self.get_logger().error(f"Safety preflight FAIL: Motor {mid} has active fault code {self._latest_faults[mid]}.")
                    return False
        elif self.hardware_mode == HardwareMode.ACTIVE:
            self.get_logger().error("Safety preflight FAIL: No valid state snapshot available prior to physical write.")
            return False

        return True

    def _apply_vector_slew_recovery(
        self, candidate_targets_steps: list[int], now_ns: int
    ) -> tuple[list[int], bool, float, int, int, int]:
        """Applies Time-Aware Vector Slew Recovery if candidate targets step gap exceeds dynamic allowance.

        Preserves exact 6-axis vector proportion without silent per-axis independent clamping.
        Caps single-dispatch step delta strictly <= 20 steps.

        Returns:
            (dispatched_steps, recovery_active, alpha, original_gap, sent_gap, remaining_gap)
        """
        # Anchor is the last successfully dispatched 6-axis vector
        anchor = self._last_commanded_steps
        if anchor is None:
            if self._last_state_snapshot is not None:
                anchor = list(self._last_state_snapshot.positions_steps)
            else:
                anchor = list(candidate_targets_steps)

        # Elapsed time since last successful write dispatch
        if self._last_write_stamp_ns > 0:
            dt_elapsed_s = max(0.010, (now_ns - self._last_write_stamp_ns) / 1e9)
        else:
            dt_elapsed_s = 1.0 / max(1.0, self.write_rate_hz)

        # Allowance calculation based on shaper max speed (0.005 m/s = 200 steps/s)
        # Cap strictly <= 20 steps (the final hard safety gate)
        vmax_steps_per_sec = 200.0
        raw_allowance = int(vmax_steps_per_sec * dt_elapsed_s)
        allowance = min(20, max(5, raw_allowance))

        gaps = [c - a for c, a in zip(candidate_targets_steps, anchor)]
        max_gap = max(abs(g) for g in gaps)

        if max_gap <= allowance:
            self._last_slew_alpha = 1.0
            self._last_slew_original_gap = max_gap
            self._last_slew_sent_gap = max_gap
            self._last_slew_remaining_gap = 0
            return list(candidate_targets_steps), False, 1.0, max_gap, max_gap, 0

        # Vector proportional scaling across all 6 axes
        alpha = float(allowance) / float(max_gap)
        dispatched = [int(round(a + alpha * g)) for a, g in zip(anchor, gaps)]

        # Strict safety cap <= 20 steps from anchor for all axes
        for i in range(len(dispatched)):
            d = dispatched[i] - anchor[i]
            if abs(d) > 20:
                dispatched[i] = anchor[i] + (20 if d > 0 else -20)

        sent_gap = max(abs(d - a) for d, a in zip(dispatched, anchor))
        remaining_gap = max_gap - sent_gap

        self._slew_recovery_count += 1
        self._last_slew_alpha = alpha
        self._last_slew_original_gap = max_gap
        self._last_slew_sent_gap = sent_gap
        self._last_slew_remaining_gap = remaining_gap

        self.get_logger().warn(
            f"Vector slew recovery active: original_gap={max_gap} steps > allowance={allowance} steps (dt={dt_elapsed_s*1000:.1f}ms). "
            f"Scaling 6-axis vector by alpha={alpha:.3f} -> sent_gap={sent_gap} steps (remaining={remaining_gap} steps).",
            throttle_duration_sec=1.0,
        )

        return dispatched, True, alpha, max_gap, sent_gap, remaining_gap



    @property
    def driver_fault_latched(self) -> bool:
        return self._j7_fault_latched



    @property
    def driver_fault_reason(self) -> str | None:
        return self._j7_fault_reason



    @property
    def telemetry_health_ok(self) -> bool:
        if not self.is_j7_servo_active or self._j7_fault_latched:
            return not self._j7_fault_latched
        if self.j7_writer is None or self.j7_feedback is None:
            return False
        if self.j7_writer.stopped or self.j7_feedback.stopped:
            return False
        now_ns = time.monotonic_ns()
        if any(
            self._latest_rx_done_ns[mid] <= 0
            or now_ns - self._latest_rx_done_ns[mid] > self.j7_feedback.max_feedback_age_ns
            for mid in MOTOR_IDS
        ):
            return False
        return all(self._latest_faults[mid] == 0 for mid in MOTOR_IDS)

    def _j7_stop_reason(self) -> str:
        if self._j7_fault_reason is not None:
            return self._j7_fault_reason
        if self.j7_feedback is not None:
            reason = self.j7_feedback.audit_snapshot().stop_reason
            if reason:
                return reason
        if self.j7_writer is not None:
            reason = self.j7_writer.audit_snapshot().stop_reason
            if reason:
                return reason
        return "j7_transport_stopped"

    def _on_feedback_timeout_warning(self, msg: str) -> None:
        t0 = time.perf_counter_ns()
        self.get_logger().warn(msg, throttle_duration_sec=1.0)
        dur_ms = (time.perf_counter_ns() - t0) / 1e6
        if self._stall_recorder is not None:
            self._stall_recorder.record_warning_log_duration(dur_ms)

    def _latch_j7_fault(self, reason: str) -> None:
        if not self._j7_fault_latched:
            self._j7_fault_latched = True
            self._j7_fault_reason = str(reason)
            self.runtime_error = DriverError(f"STOP: {self._j7_fault_reason}")
            if self._stall_recorder is not None:
                try:
                    self._stall_recorder.record_fault_cycle(reason, fault_phase="latch_j7_fault")
                    self._stall_recorder.dump()
                except Exception as exc:
                    self.get_logger().warn(f"Error dumping stall diagnostics on latch: {exc}")
        if not self._j7_fault_log_emitted:
            self._j7_fault_log_emitted = True
            self.get_logger().fatal(f"J7 driver STOP latched: {self._j7_fault_reason}")

    def _uart_worker_cycle(self) -> None:
        """ROS timer callback delegating to the unified servo cycle execution core."""
        self._execute_servo_cycle()

    def _servo_dedicated_worker_loop(self) -> None:
        """Dedicated high-precision absolute-deadline worker thread loop for J11-RT2."""
        interval_ns = int(1e9 / max(1.0, self.write_rate_hz))
        next_deadline_ns = time.perf_counter_ns()
        while not self._servo_thread_stop_event.is_set():
            if self._j7_fault_latched:
                break
            expected_start_ns = next_deadline_ns
            self._execute_servo_cycle(expected_start_ns=expected_start_ns)
            next_deadline_ns += interval_ns
            now_ns = time.perf_counter_ns()
            sleep_ns = next_deadline_ns - now_ns
            if sleep_ns > 0:
                time.sleep(sleep_ns / 1e9)
            else:
                if -sleep_ns > interval_ns:
                    next_deadline_ns = now_ns

    def _telemetry_publisher_loop(self) -> None:
        """Dedicated worker loop for non-blocking asynchronous telemetry publishing (J11-RT2B)."""
        while not self._telemetry_thread_stop_event.is_set():
            if not self._telemetry_ready_event.wait(timeout=0.020):
                continue
            self._telemetry_ready_event.clear()
            with self._telemetry_lock:
                snap = self._latest_telemetry_snapshot
            if snap is not None:
                try:
                    self._publish_telemetry_from_snapshot(snap, record_stall=False)
                except Exception as exc:
                    self.get_logger().warn(f"Telemetry publication error: {exc}")

    def _publish_telemetry_from_snapshot(self, snap: ServoTelemetrySnapshot, record_stall: bool = False) -> None:
        """Publishes all telemetry topics from a snapshot."""
        stamp_msg = Time(sec=snap.stamp_sec, nanosec=snap.stamp_nanosec)
        t_p1 = time.perf_counter_ns()
        js_msg = JointState()
        js_msg.header.stamp = stamp_msg
        js_msg.name = self.joint_names
        js_msg.position = [steps_to_meters(s) for s in snap.positions_steps]
        self.joint_state_pub.publish(js_msg)
        t_p2 = time.perf_counter_ns()
        if record_stall and self._stall_recorder is not None:
            self._stall_recorder.mark_pub_joint_state((t_p2 - t_p1) / 1e6)

        if self.timed_feedback_pub is not None:
            t_tf1 = time.perf_counter_ns()
            tf_msg = TimedJointFeedback()
            tf_msg.header.stamp = stamp_msg
            tf_msg.snapshot_start_monotonic_ns = snap.cycle_start_ns
            tf_msg.snapshot_end_monotonic_ns = snap.cycle_end_ns
            tf_msg.per_axis_rx_done_monotonic_ns = list(snap.rx_done_ns)
            tf_msg.raw_steps = list(snap.positions_steps)
            tf_msg.positions_meters = [0.050 + (s - 1000) * 0.000025 for s in snap.positions_steps]
            tf_msg.currents_ma = list(snap.currents_mA)
            tf_msg.fault_codes = list(snap.fault_codes)
            tf_msg.forces_g_raw = list(getattr(snap, 'forces_g_raw', [0]*6))
            tf_msg.temperatures_c = list(getattr(snap, 'temperatures_C', [0]*6))
            tf_msg.uart_cycle_sequence = snap.uart_cycle_seq

            if getattr(self, '_force_hold_reference_monotonic_ns', 0) == 0:
                self._force_hold_reference_monotonic_ns = snap.cycle_start_ns or time.monotonic_ns()
                self._force_hold_g = list(tf_msg.forces_g_raw)

            tf_msg.source = (
                TimedJointFeedback.SOURCE_MOCK
                if self.hardware_mode == HardwareMode.MOCK
                else TimedJointFeedback.SOURCE_HARDWARE
            )
            tf_msg.force_valid = [
                bool(snap.fault_codes[i] == 0 and -32768 <= tf_msg.forces_g_raw[i] <= 32767)
                for i in range(6)
            ]
            tf_msg.force_adc_raw = [2048] * 6
            tf_msg.force_mode_confirmed = True
            tf_msg.force_hold_valid = True
            tf_msg.force_hold_g = list(self._force_hold_g)
            tf_msg.force_hold_reference_monotonic_ns = self._force_hold_reference_monotonic_ns

            self.timed_feedback_pub.publish(tf_msg)
            t_tf2 = time.perf_counter_ns()
            if record_stall and self._stall_recorder is not None:
                self._stall_recorder.mark_pub_timed_feedback((t_tf2 - t_tf1) / 1e6)

        if self.timed_command_pub is not None and self.hardware_mode != HardwareMode.READ_ONLY:
            t_tc1 = time.perf_counter_ns()
            tc_msg = TimedJointCommand()
            tc_msg.header.stamp = stamp_msg
            tc_msg.transaction_start_monotonic_ns = snap.cycle_start_ns
            tc_msg.transaction_end_monotonic_ns = snap.cycle_end_ns
            tc_msg.per_axis_tx_start_monotonic_ns = list(snap.tx_start_ns)
            tc_msg.per_axis_tx_done_monotonic_ns = list(snap.tx_end_ns)
            tc_msg.target_positions_meters = [steps_to_meters(s) for s in snap.dispatched_targets]
            tc_msg.target_steps = list(snap.dispatched_targets)
            tc_msg.uart_cycle_sequence = snap.uart_cycle_seq
            self.timed_command_pub.publish(tc_msg)
            t_tc2 = time.perf_counter_ns()
            if record_stall and self._stall_recorder is not None:
                self._stall_recorder.mark_pub_timed_command((t_tc2 - t_tc1) / 1e6)

        if not self._defer_driver_diagnostics:
            t_diag1 = time.perf_counter_ns()
            diag_msg = Float64MultiArray()
            diag_msg.data = list(snap.diag_data)
            self.driver_diagnostics_pub.publish(diag_msg)
            t_diag2 = time.perf_counter_ns()
            if record_stall and self._stall_recorder is not None:
                self._stall_recorder.mark_pub_diagnostics((t_diag2 - t_diag1) / 1e6)

        if self.publish_raw_diagnostics:
            t_raw1 = time.perf_counter_ns()
            self._publish_array(self.raw_steps_pub, snap.positions_steps)
            self._publish_array(self.raw_currents_pub, snap.currents_mA)
            self._publish_array(self.raw_faults_pub, snap.fault_codes)
            t_raw2 = time.perf_counter_ns()
            if record_stall and self._stall_recorder is not None:
                self._stall_recorder.mark_pub_raw_diag((t_raw2 - t_raw1) / 1e6)

        if self.benchmark_stamp_pub is not None:
            t_bench1 = time.perf_counter_ns()
            t_pub_ns = time.perf_counter_ns()
            bench_msg = Int64MultiArray()
            bench_msg.data = [
                snap.benchmark_seq_id,
                snap.t_entry_ns,
                snap.cycle_start_ns,
                snap.cycle_end_ns,
                snap.cycle_end_ns,
                t_pub_ns,
                t_pub_ns,
            ]
            self.benchmark_stamp_pub.publish(bench_msg)
            self.benchmark_publish_timestamps_ns.append(snap.cycle_end_ns)
            t_bench2 = time.perf_counter_ns()
            if record_stall and self._stall_recorder is not None:
                self._stall_recorder.mark_pub_benchmark((t_bench2 - t_bench1) / 1e6)

    def _execute_servo_cycle(self, expected_start_ns: int = 0) -> None:
        """High-rate UART worker: dispatches latest target with watchdog, preflight, and segmented telemetry."""
        if self.is_j7_servo_active and self.j7_writer is not None:
            if self._fresh_state_hold:
                return
            if self._j7_fault_latched:
                return
            if self.j7_writer.stopped or (self.j7_feedback is not None and self.j7_feedback.stopped):
                self._latch_j7_fault(self._j7_stop_reason())
                return
            t_entry_ns = time.perf_counter_ns()
            if self._stall_recorder is not None:
                self._stall_recorder.start_cycle(self._uart_cycle_seq, expected_start_ns=expected_start_ns)
            if self._last_worker_start_ns > 0:
                self._worker_start_to_start_ms = (t_entry_ns - self._last_worker_start_ns) / 1e6
            self._last_worker_start_ns = t_entry_ns

            candidate_targets: list[int] | None = None
            with self._target_lock:
                if self._last_target_rx_time_ns > 0:
                    age_since_rx_ms = (t_entry_ns - self._last_target_rx_time_ns) / 1e6
                    if age_since_rx_ms > (self.target_timeout_sec * 1000.0):
                        if not self._stream_timed_out:
                            self._stream_timed_out = True
                            self._stale_target_timeout_count += 1

                feedback_in_warning = False
                if self.j7_feedback is not None and getattr(self.j7_feedback, "feedback_loss_policy", "") == "age_based":
                    feedback_in_warning = self.j7_feedback.is_feedback_stale_warning(t_entry_ns)

                if self._static_recovery_target is not None:
                    candidate_targets = list(self._static_recovery_target)
                elif feedback_in_warning:
                    # Warning state: do not expand new compensation commands based on stale feedback; hold recent safe target
                    if self._last_commanded_steps is not None:
                        candidate_targets = list(self._last_commanded_steps)
                    else:
                        candidate_targets = [self._latest_positions_steps[mid] for mid in MOTOR_IDS]
                elif self._latest_target is not None and not self._latest_target.consumed:
                    candidate_targets = list(self._latest_target.targets_steps)
                    self._latest_target.consumed = True
                    self._target_consumed_count += 1
                elif self._last_commanded_steps is not None:
                    # Continuous 50Hz streaming: hold the last valid shaped target if no new Cartesian target arrived
                    candidate_targets = list(self._last_commanded_steps)
                else:
                    candidate_targets = [self._latest_positions_steps[mid] for mid in MOTOR_IDS]

            if self._stall_recorder is not None:
                self._stall_recorder.mark_target_lock_done()

            dispatched_targets, rec_active, alpha, orig_gap, sent_gap, rem_gap = self._apply_vector_slew_recovery(
                candidate_targets, t_entry_ns
            )
            if self._stall_recorder is not None:
                self._stall_recorder.mark_slew_done()

            preflight_ok = self._execute_safety_preflight(dispatched_targets)
            if self._stall_recorder is not None:
                self._stall_recorder.mark_preflight_done()
            if not preflight_ok:
                if self._static_recovery_target is not None:
                    self._communication_recovery.fail_static_resume(
                        DriverError('existing safety preflight failure'), 'FIRST_RESUME_WRITE' if not self._communication_recovery.static_event.get('resume_success') else 'STATIC_HOLD')
                self.get_logger().error("Aborting J7 Servo UART write cycle due to safety preflight failure.")
                if self._stall_recorder is not None:
                    self._stall_recorder.record_fault_cycle("safety_preflight_failure", fault_phase="T3_preflight")
                self._latch_j7_fault("safety_preflight_failure")
                if self.j7_writer is not None:
                    self.j7_writer.stop("safety_preflight_failure")
                return

            t_write_start_ns = time.perf_counter_ns()
            try:
                cycle_start_ns = t_entry_ns
                cycle_end_ns = t_entry_ns
                tx_records = []

                if self.hardware_mode != HardwareMode.READ_ONLY:
                    if self._stall_recorder is not None:
                        self._stall_recorder.mark_transport_entry()
                    servo_audit = self.j7_writer.send_cycle(
                        dispatched_targets,
                        cycle_id=self._uart_cycle_seq,
                        target_timestamp_ns=t_entry_ns,
                    )
                    self._consecutive_comm_failures = 0
                    self._last_commanded_steps = list(dispatched_targets)
                    t_write_end_ns = time.perf_counter_ns()
                    write_dur_ms = (servo_audit.cycle_end_ns - servo_audit.cycle_start_ns) / 1e6
                    self._six_axis_uart_transaction_ms = write_dur_ms
                    self._last_write_duration_ms = write_dur_ms
                    self._last_write_stamp_ns = t_write_end_ns
                    cycle_start_ns = servo_audit.cycle_start_ns
                    cycle_end_ns = servo_audit.cycle_end_ns
                    tx_records = servo_audit.tx_records

                    if self._stall_recorder is not None:
                        self._stall_recorder.mark_transport_exit(servo_audit)
                        self._stall_recorder.mark_ingest_start()

                    # 2. Design B: 50Hz primary feedback from 0x29 StatusFeedback (0x30 blocking poll eliminated)
                    feedback_audit = self.j7_feedback.ingest_cycle_feedback(
                        servo_audit.feedbacks,
                        servo_cycle_id=self._uart_cycle_seq,
                        servo_cycle_timestamp_ns=t_entry_ns,
                    )
                else:
                    # READ_ONLY mode: 50Hz 0x30 round-robin polling
                    if self._stall_recorder is not None:
                        self._stall_recorder.mark_ingest_start()
                    feedback_audit = self.j7_feedback.poll_once(
                        servo_cycle_id=self._uart_cycle_seq,
                        servo_cycle_timestamp_ns=t_entry_ns,
                        force_full_scan=False,
                    )

                if self._stall_recorder is not None:
                    gap_ms = getattr(feedback_audit, "scheduler_gap_ns", 0) / 1e6
                    if gap_ms == 0.0 and self.j7_feedback is not None:
                        gap_ms = getattr(self.j7_feedback, "last_scheduler_gap_ns", 0) / 1e6
                    self._stall_recorder.mark_ingest_done(gap_ms)

                if self.j7_feedback.stopped or self.j7_writer.stopped:
                    if self._static_recovery_target is not None:
                        raise DriverError(self._j7_stop_reason())
                    self._latch_j7_fault(self._j7_stop_reason())
                if self._static_recovery_target is not None:
                    self._communication_recovery.observe_static_cycle(servo_audit, feedback_audit)
                self._feedback_transaction_ms = max(0.0, (feedback_audit.transaction_end_ns - feedback_audit.transaction_start_ns) / 1e6)
                self._rx_classification_ms = max(0.0, (feedback_audit.rx_drain_end_ns - feedback_audit.rx_drain_start_ns) / 1e6)
                stop_snapshot = self.j7_feedback.audit_snapshot().watchdog_stop_snapshot
                if stop_snapshot is not None:
                    self._watchdog_stop_age_ms = stop_snapshot.age_ns / 1e6

                for sample in feedback_audit.samples:
                    if sample is not None and sample.position is not None:
                        self._latest_positions_steps[sample.motor_id] = sample.position
                        self._latest_currents[sample.motor_id] = sample.current_mA if sample.current_mA is not None else 0
                        self._latest_faults[sample.motor_id] = sample.fault if sample.fault is not None else 0
                        self._latest_rx_done_ns[sample.motor_id] = sample.timestamp_ns

                if self._stall_recorder is not None:
                    self._stall_recorder.mark_unpack_done()

                current_uart_seq = self._uart_cycle_seq
                self._uart_cycle_seq += 1

                # 3. Telemetry Publication (RT2B: Decoupled snapshot handoff or inline baseline)
                now_stamp = self.get_clock().now().to_msg()
                tx_starts = [rec.tx_start_ns for rec in tx_records] if tx_records else []
                tx_ends = [rec.tx_end_ns for rec in tx_records] if tx_records else []
                snap = ServoTelemetrySnapshot(
                    uart_cycle_seq=current_uart_seq,
                    cycle_start_ns=cycle_start_ns,
                    cycle_end_ns=cycle_end_ns,
                    stamp_sec=now_stamp.sec,
                    stamp_nanosec=now_stamp.nanosec,
                    positions_steps=[self._latest_positions_steps[mid] for mid in MOTOR_IDS],
                    currents_mA=[self._latest_currents[mid] for mid in MOTOR_IDS],
                    fault_codes=[self._latest_faults[mid] for mid in MOTOR_IDS],
                    rx_done_ns=[self._latest_rx_done_ns[mid] for mid in MOTOR_IDS],
                    dispatched_targets=list(dispatched_targets),
                    tx_start_ns=tx_starts,
                    tx_end_ns=tx_ends,
                    diag_data=[
                        float(self._target_in_count),
                        float(self._target_overwrite_count),
                        float(self._target_consumed_count),
                        float(self._deadline_miss_count),
                        float(self._last_target_age_ms),
                        float(self._worker_start_to_start_ms),
                        float(self._last_write_duration_ms),
                        float(self._rx_classification_ms),
                        float(self._feedback_transaction_ms),
                        float(self._worker_total_ms),
                        float(self._watchdog_stop_age_ms),
                    ] if not self._defer_driver_diagnostics else (
                        self._target_in_count, self._target_overwrite_count,
                        self._target_consumed_count, self._deadline_miss_count,
                        self._last_target_age_ms, self._worker_start_to_start_ms,
                        self._last_write_duration_ms, self._rx_classification_ms,
                        self._feedback_transaction_ms, self._worker_total_ms,
                        self._watchdog_stop_age_ms,
                    ),
                    benchmark_seq_id=self._benchmark_seq_id,
                    t_entry_ns=t_entry_ns,
                )
                if self.benchmark_stamp_pub is not None:
                    self._benchmark_seq_id += 1

                if self.telemetry_publish_mode == "async_latest":
                    t_snap_start = time.perf_counter_ns()
                    with self._telemetry_lock:
                        self._latest_telemetry_snapshot = snap
                    self._telemetry_ready_event.set()
                    t_snap_dur_ms = (time.perf_counter_ns() - t_snap_start) / 1e6
                    if self._stall_recorder is not None:
                        if hasattr(self._stall_recorder, "mark_snapshot_handoff"):
                            self._stall_recorder.mark_snapshot_handoff(t_snap_dur_ms)
                        else:
                            self._stall_recorder.mark_pub_joint_state(t_snap_dur_ms)
                else:
                    self._publish_telemetry_from_snapshot(snap, record_stall=True)

                if self._stall_recorder is not None:
                    self._stall_recorder.mark_telemetry_done()

                self._worker_total_ms = (time.perf_counter_ns() - t_entry_ns) / 1e6
                if self._stall_recorder is not None:
                    # Existing sample age is measured at ingest, before refreshing valid axes.
                    # It includes feedback staleness across the completed TX/drain interval.
                    fb_age_max = max(
                        (sample.feedback_age_ns / 1e6 for sample in feedback_audit.samples if sample is not None),
                        default=0.0,
                    )
                    self._stall_recorder.finish_cycle(
                        fault_reason=self._j7_fault_reason or "",
                        feedback_age_max_ms=fb_age_max,
                    )

            except Exception as exc:
                self._worker_total_ms = (time.perf_counter_ns() - t_entry_ns) / 1e6
                failed_audit = getattr(exc, "cycle_audit", None)
                if failed_audit is not None:
                    self.get_logger().error(
                        f"UART TX failure: cycle_seq={failed_audit.cycle_id} "
                        f"motor_id={failed_audit.error_motor_id} "
                        f"classification={failed_audit.error_class}: {exc}"
                    )
                    if self._stall_recorder is not None:
                        self._stall_recorder.mark_transport_exit(failed_audit)
                if self._static_recovery_target is not None:
                    first = not self._communication_recovery.static_event.get('resume_success', False)
                    phase = ('FIRST_RESUME_WRITE' if locals().get('servo_audit') is None else 'FIRST_RESUME_FEEDBACK') if first else 'STATIC_HOLD'
                    self.j7_writer.stop('communication_static_resume_failed')
                    self._communication_recovery.fail_static_resume(exc, phase, locals().get('servo_audit'))
                elif isinstance(exc, J7AdapterWriteTimeout) and self.j7_writer.communication_transient:
                    try:
                        self._recover_communication_timeout(exc, candidate_targets)
                        return
                    except Exception as recovery_exc:
                        self.get_logger().error(f'Communication recovery failed: {recovery_exc}')
                fault_reason = self._j7_stop_reason() if (self.j7_writer.stopped or (self.j7_feedback is not None and self.j7_feedback.stopped)) else f"worker_exception: {exc}"
                if self._stall_recorder is not None:
                    self._stall_recorder.record_fault_cycle(fault_reason, fault_phase="worker_exception")
                self._latch_j7_fault(fault_reason)
                if self.j7_writer is not None and not self.j7_writer.stopped:
                    self.j7_writer.stop(self._j7_fault_reason or "worker_exception")
                return
            return

        if not self.controller.is_connected:
            return

        t_entry_ns = time.perf_counter_ns()
        if self._last_worker_start_ns > 0:
            self._worker_start_to_start_ms = (t_entry_ns - self._last_worker_start_ns) / 1e6
        self._last_worker_start_ns = t_entry_ns

        target_env: TargetEnvelope | None = None

        t_fetch_start_ns = time.perf_counter_ns()
        with self._target_lock:
            # Watchdog check: If active streaming has started and target stream timed out
            if self._last_target_rx_time_ns > 0:
                age_since_rx_ms = (t_entry_ns - self._last_target_rx_time_ns) / 1e6
                if age_since_rx_ms > (self.target_timeout_sec * 1000.0):
                    if not self._stream_timed_out:
                        self._stream_timed_out = True
                        self._stale_target_timeout_count += 1
                        self.get_logger().warn(
                            f"Target stream timed out ({age_since_rx_ms:.1f}ms > {self.target_timeout_sec*1000:.0f}ms). "
                            f"Halting continuous writes (behavior={self.timeout_behavior}).",
                            throttle_duration_sec=2.0,
                        )
                        if self.timeout_behavior == "pause" and self.hardware_mode != HardwareMode.READ_ONLY:
                            try:
                                self.controller.pause_all()
                            except Exception as p_exc:
                                self.get_logger().error(f"Failed to pause on timeout: {p_exc}")
                    # In timeout state: do NOT repeat stale targets over UART!
                    return

            if self._latest_target is not None and not self._latest_target.consumed:
                target_env = self._latest_target
                target_env.consumed = True
                self._target_consumed_count += 1

        t_fetch_end_ns = time.perf_counter_ns()
        self._target_fetch_ms = (t_fetch_end_ns - t_fetch_start_ns) / 1e6

        if target_env is None:
            return

        # Time-Aware Vector Slew Recovery before preflight
        dispatched_targets, rec_active, alpha, orig_gap, sent_gap, rem_gap = self._apply_vector_slew_recovery(
            target_env.targets_steps, t_entry_ns
        )

        # Final safety preflight right before hardware dispatch
        t_preflight_start_ns = time.perf_counter_ns()
        preflight_ok = self._execute_safety_preflight(dispatched_targets)
        t_preflight_end_ns = time.perf_counter_ns()
        self._final_preflight_ms = (t_preflight_end_ns - t_preflight_start_ns) / 1e6

        if not preflight_ok:
            self.get_logger().error("Aborting UART write cycle due to safety preflight failure.")
            return

        age_ms = (t_entry_ns - target_env.stamp_ns) / 1e6
        self._last_target_age_ms = age_ms

        t_write_start_ns = time.perf_counter_ns()
        try:
            cmd_result = self.controller.command_positions_with_metrics(dispatched_targets)
            self._consecutive_comm_failures = 0
            self._last_commanded_steps = list(dispatched_targets)
            t_write_end_ns = time.perf_counter_ns()
            write_dur_ms = (t_write_end_ns - t_write_start_ns) / 1e6
            self._six_axis_uart_transaction_ms = write_dur_ms
            self._last_write_duration_ms = write_dur_ms
            self._last_write_stamp_ns = t_write_end_ns

            # Per-motor transaction breakdown
            for mid in MOTOR_IDS:
                t_m_start = cmd_result.per_axis_write_timestamps_ns.get(mid, 0)
                t_m_end = cmd_result.per_axis_write_completion_timestamps_ns.get(mid, 0)
                if t_m_start > 0 and t_m_end > 0:
                    self._per_motor_ms[mid] = (t_m_end - t_m_start) / 1e6

            target_period_ms = 1000.0 / max(1.0, self.write_rate_hz)
            if write_dur_ms > target_period_ms:
                self._deadline_miss_count += 1

            # Convert 0x29 write responses directly into a SixAxisStateSnapshot
            t_parse_start_ns = time.perf_counter_ns()
            snapshot = cmd_result.to_state_snapshot()
            self._last_state_snapshot = snapshot
            t_parse_end_ns = time.perf_counter_ns()
            self._feedback_parse_ms = (t_parse_end_ns - t_parse_start_ns) / 1e6

            # Shared sequence ID for 1-to-1 pairing of command and companion feedback
            current_uart_seq = self._uart_cycle_seq
            self._uart_cycle_seq += 1

            # ROS publish telemetry
            t_pub_start_ns = time.perf_counter_ns()
            self._publish_snapshot_telemetry(snapshot, current_uart_seq)
            self._publish_driver_diagnostics()

            # J5-C1 Timed Command Publisher (Dispatched target with actual transaction timestamps)
            if self.timed_command_pub is not None:
                tc_msg = TimedJointCommand()
                tc_msg.header.stamp = self.get_clock().now().to_msg()
                t_start_cmd = cmd_result.per_axis_write_timestamps_ns.get(1, 0)
                t_end_cmd = cmd_result.per_axis_write_completion_timestamps_ns.get(6, 0)
                tc_msg.transaction_start_monotonic_ns = t_start_cmd
                tc_msg.transaction_end_monotonic_ns = t_end_cmd
                tc_msg.per_axis_tx_start_monotonic_ns = [
                    cmd_result.per_axis_write_timestamps_ns.get(mid, 0) for mid in MOTOR_IDS
                ]
                tc_msg.per_axis_tx_done_monotonic_ns = [
                    cmd_result.per_axis_write_completion_timestamps_ns.get(mid, 0) for mid in MOTOR_IDS
                ]
                tc_msg.target_positions_meters = [steps_to_meters(s) for s in dispatched_targets]
                tc_msg.target_steps = list(dispatched_targets)
                tc_msg.uart_cycle_sequence = current_uart_seq
                self.timed_command_pub.publish(tc_msg)

            t_pub_end_ns = time.perf_counter_ns()
            self._ros_publish_ms = (t_pub_end_ns - t_pub_start_ns) / 1e6

            self._worker_total_ms = (t_pub_end_ns - t_entry_ns) / 1e6

        except (SafetyViolation, J2F0SafetyViolation, J2F0Stop, ReadOnlyModeError) as exc:
            self.runtime_error = exc
            self.get_logger().fatal(f"FATAL safety violation in UART worker write: {exc}")
            raise
        except Exception as exc:
            t_err_ns = time.perf_counter_ns()
            self._consecutive_comm_failures += 1
            err_record = {
                "timestamp_ns": t_err_ns,
                "error": str(exc),
                "target_steps": list(target_env.targets_steps),
                "consecutive_count": self._consecutive_comm_failures,
                "duration_before_err_ms": (t_err_ns - t_write_start_ns) / 1e6,
            }
            self._timeout_events.append(err_record)
            self.get_logger().warn(
                f"UART transient communication error (count={self._consecutive_comm_failures}): {exc}"
            )
            # Delivery Uncertain State Machine:
            # 1. Hardware already paused via pause_all() in six_motor_controller exception handler.
            # 2. Attempt to read fresh 6-axis state snapshot to establish true ground-truth anchor.
            try:
                if self.controller.is_connected:
                    recovery_snap = self.controller.read_state_snapshot()
                    # Validate feedback completeness & faults
                    if (
                        recovery_snap is not None
                        and all(f == 0 for f in recovery_snap.fault_codes)
                        and all(200 <= p <= 1800 for p in recovery_snap.positions_steps)
                    ):
                        self._last_state_snapshot = recovery_snap
                        # Re-anchor command anchor to true physical positions
                        self._last_commanded_steps = list(recovery_snap.positions_steps)
                        self._last_write_stamp_ns = time.perf_counter_ns()
                        self.get_logger().info(
                            f"Delivery-uncertain resync SUCCESS: Re-anchored to physical positions {self._last_commanded_steps}."
                        )
            except Exception as resync_exc:
                self.get_logger().warn(f"Delivery-uncertain snapshot read failed: {resync_exc}")

            if self._consecutive_comm_failures >= 3:
                self.runtime_error = DriverError(
                    f"STOP: {self._consecutive_comm_failures} consecutive UART communication failures: {exc}"
                )
                self.get_logger().fatal(f"FATAL: 3 consecutive UART communication failures. Halting.")
                raise self.runtime_error

    def _background_poll_and_publish(self) -> None:
        """Low-rate background telemetry (active only during idle or READ_ONLY mode)."""
        if not self.controller.is_connected:
            return

        self._background_poll_count += 1
        now_ns = time.perf_counter_ns()
        # If an active target command stream is ongoing (< 500ms since last target arrival/write dispatch),
        # skip standalone poll to prevent bus contention.
        has_active_stream = (
            (self._last_target_rx_time_ns > 0 and (now_ns - self._last_target_rx_time_ns) < 500_000_000)
            or (self._last_write_stamp_ns > 0 and (now_ns - self._last_write_stamp_ns) < 500_000_000)
        )
        if has_active_stream and self.hardware_mode != HardwareMode.READ_ONLY:
            self._background_poll_during_active_count += 1
            return

        t_cb_start_ns = time.perf_counter_ns()
        try:
            t_scan_start_ns = time.perf_counter_ns()
            if self.read_mode == "position_only":
                pos_snap = self.controller.read_positions_snapshot()
                self._last_positions_snapshot = pos_snap
                t_scan_end_ns = time.perf_counter_ns()
                t_pub_start_ns = time.perf_counter_ns()
                msg = JointState()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.name = self.joint_names
                msg.position = pos_snap.positions_meters
                self.joint_state_pub.publish(msg)
                if self.publish_raw_diagnostics:
                    self._publish_array(self.raw_steps_pub, pos_snap.positions_steps)
                t_pub_end_ns = time.perf_counter_ns()
            else:
                snap = self.controller.read_state_snapshot()
                self._last_state_snapshot = snap
                t_scan_end_ns = time.perf_counter_ns()
                t_pub_start_ns = time.perf_counter_ns()
                self._publish_snapshot_telemetry(snap)
                t_pub_end_ns = time.perf_counter_ns()

            t_cb_end_ns = time.perf_counter_ns()
            if self.benchmark_stamp_pub is not None:
                bench_msg = Int64MultiArray()
                bench_msg.data = [
                    self._benchmark_seq_id,
                    t_cb_start_ns,
                    t_scan_start_ns,
                    t_scan_end_ns,
                    t_pub_start_ns,
                    t_pub_end_ns,
                    t_cb_end_ns,
                ]
                self.benchmark_stamp_pub.publish(bench_msg)
                self.benchmark_publish_timestamps_ns.append(t_pub_start_ns)
                self._benchmark_seq_id += 1

        except Exception as exc:
            self.get_logger().warn(f"Error in background poll: {exc}", throttle_duration_sec=2.0)

    def _poll_and_publish(self) -> None:
        """Alias for backward compatibility with existing tests and audit tools."""
        self._background_poll_and_publish()

    def _publish_snapshot_telemetry(self, snapshot: SixAxisStateSnapshot, uart_seq: int | None = None) -> None:
        """Publish standard JointState and raw diagnostic arrays from a snapshot."""
        # Standard JointState (ROS header.stamp retains standard ROS node clock semantics)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = snapshot.positions_meters
        msg.velocity = []
        msg.effort = []
        self.joint_state_pub.publish(msg)

        # J5-C1 Timed Telemetry Feedback (Host CLOCK_MONOTONIC Timing)
        if self.timed_feedback_pub is not None:
            tf_msg = TimedJointFeedback()
            tf_msg.header.stamp = msg.header.stamp
            tf_msg.snapshot_start_monotonic_ns = snapshot.timestamp_start_ns
            tf_msg.snapshot_end_monotonic_ns = snapshot.timestamp_end_ns
            tf_msg.per_axis_rx_done_monotonic_ns = [
                snapshot.per_axis_end_timestamps_ns.get(mid, 0) for mid in MOTOR_IDS
            ]
            tf_msg.positions_meters = list(snapshot.positions_meters)
            tf_msg.raw_steps = list(snapshot.positions_steps)
            tf_msg.currents_ma = list(snapshot.currents_mA)
            tf_msg.forces_g_raw = list(snapshot.forces_g_raw)
            tf_msg.temperatures_c = list(snapshot.temperatures_C)
            tf_msg.fault_codes = list(snapshot.fault_codes)
            if uart_seq is not None:
                tf_msg.uart_cycle_sequence = uart_seq
            else:
                tf_msg.uart_cycle_sequence = self._uart_cycle_seq
                self._uart_cycle_seq += 1

            if getattr(self, '_force_hold_reference_monotonic_ns', 0) == 0:
                self._force_hold_reference_monotonic_ns = snapshot.timestamp_start_ns or time.monotonic_ns()
                self._force_hold_g = list(snapshot.forces_g_raw)

            tf_msg.source = (
                TimedJointFeedback.SOURCE_MOCK
                if self.hardware_mode == HardwareMode.MOCK
                else TimedJointFeedback.SOURCE_HARDWARE
            )
            tf_msg.force_valid = [
                bool(snapshot.fault_codes[i] == 0 and -32768 <= snapshot.forces_g_raw[i] <= 32767)
                for i in range(6)
            ]
            adc_list = getattr(snapshot, 'forces_adc', None)
            if adc_list and len(adc_list) == 6:
                tf_msg.force_adc_raw = [int(v) for v in adc_list]
            else:
                tf_msg.force_adc_raw = [2048] * 6
            tf_msg.force_mode_confirmed = True
            tf_msg.force_hold_valid = True
            tf_msg.force_hold_g = list(self._force_hold_g)
            tf_msg.force_hold_reference_monotonic_ns = self._force_hold_reference_monotonic_ns

            self.timed_feedback_pub.publish(tf_msg)

        # Raw Telemetry
        if self.publish_raw_diagnostics:
            self._publish_array(self.raw_steps_pub, snapshot.positions_steps)
            self._publish_array(self.raw_currents_pub, snapshot.currents_mA)
            self._publish_array(self.raw_forces_pub, snapshot.forces_g_raw)
            self._publish_array(self.raw_temps_pub, snapshot.temperatures_C)
            self._publish_array(self.raw_faults_pub, snapshot.fault_codes)

    def _publish_driver_diagnostics(self) -> None:
        """Publish scheduling and deadline metrics."""
        diag_msg = Float64MultiArray()
        # [target_in_count, overwrite_count, consumed_count, deadline_miss_count, target_age_ms, write_duration_ms]
        diag_msg.data = [
            float(self._target_in_count),
            float(self._target_overwrite_count),
            float(self._target_consumed_count),
            float(self._deadline_miss_count),
            float(self._last_target_age_ms),
            float(self._worker_start_to_start_ms),
            float(self._last_write_duration_ms),
            float(self._rx_classification_ms),
            float(self._feedback_transaction_ms),
            float(self._worker_total_ms),
            float(self._watchdog_stop_age_ms),
        ]
        self.driver_diagnostics_pub.publish(diag_msg)

    def _publish_array(self, pub, data: list[int]) -> None:
        arr_msg = Int32MultiArray()
        arr_msg.data = data
        pub.publish(arr_msg)

    def _handle_command_single_step(self, msg: Int32MultiArray) -> None:
        """Handle isolated single-axis bring-up command: data = [motor_id, target_step]."""
        if self.hardware_mode == HardwareMode.READ_ONLY:
            self.get_logger().error("Single-axis command rejected: pus6_motor_driver is in READ_ONLY mode.")
            return

        if len(msg.data) != 2:
            self.get_logger().error(
                f"Single-axis command requires [motor_id, target_step], got {len(msg.data)} elements."
            )
            return

        motor_id = int(msg.data[0])
        target_step = int(msg.data[1])

        try:
            self.controller.command_single_axis_with_metrics(motor_id, target_step)
            self.get_logger().info(
                f"Executed isolated single-axis move: ID{motor_id} -> {target_step} step"
            )
        except Exception as exc:
            self.get_logger().error(f"Single-axis move error on ID{motor_id}: {exc}")

    def _handle_pause_all(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        if self.hardware_mode == HardwareMode.READ_ONLY:
            response.success = False
            response.message = "Rejected: Pause write is forbidden in READ_ONLY mode."
            self.get_logger().warn(response.message)
            return response

        try:
            if self.is_j7_servo_active and self.j7_stop_adapter is not None:
                self.j7_stop_adapter.stop_all("service_pause")
                response.success = True
                response.message = "Successfully issued 0x1A stop to all 6 motors via J7StopAdapter."
            else:
                res = self.controller.pause_all()
                all_ok = all(res.values())
                response.success = all_ok
                response.message = f"Pause all results: {res}"
            self.get_logger().warn(f"Pause all triggered: {response.message}")
        except Exception as exc:
            response.success = False
            response.message = f"Pause all exception: {exc}"
            self.get_logger().error(f"Pause all failed: {exc}")
        return response

    def _handle_configure_motors(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        if self.hardware_mode == HardwareMode.READ_ONLY:
            response.success = False
            response.message = "Rejected: Motor configure is forbidden in READ_ONLY mode."
            self.get_logger().warn(response.message)
            return response

        try:
            if self.is_j7_servo_active and self.j7_writer is not None:
                self.j7_writer.configure_servo(speed=self.servo_speed)
                response.success = True
                response.message = f"Configured mode=1 (Servo), speed={self.servo_speed} on all 6 motors successfully."
            else:
                self.controller.configure_all()
                response.success = True
                response.message = "Configured mode=2, speed=50 on all 6 motors successfully."
            self.get_logger().info("Configured all motors successfully via service.")
        except Exception as exc:
            response.success = False
            response.message = f"Configuration failed: {exc}"
            self.get_logger().error(f"Configuration failed: {exc}")
        return response

    def destroy_node(self) -> None:
        """Safe shutdown: execute best-effort pause_all/0x1A ONLY if active real hardware is connected."""
        if self._telemetry_thread is not None and self._telemetry_thread.is_alive():
            self._telemetry_thread_stop_event.set()
            self._telemetry_ready_event.set()
            self._telemetry_thread.join(timeout=1.0)
        if self._servo_thread is not None and self._servo_thread.is_alive():
            self._servo_thread_stop_event.set()
            self._servo_thread.join(timeout=1.0)
        if self._defer_driver_diagnostics and self._latest_telemetry_snapshot is not None:
            # Latest-only export after UART stops. No new queue, worker, or
            # shared lock; per-cycle failure evidence stays in the existing ring.
            if self._servo_thread is None or not self._servo_thread.is_alive():
                try:
                    snap = self._latest_telemetry_snapshot
                    append_evidence(self._communication_audit_dir()/'driver_diagnostics_deferred.jsonl',
                        dict(cycle_seq=snap.uart_cycle_seq, retention='latest_only',
                             workload='driver_diagnostics', data=[float(x) for x in snap.diag_data]))
                except Exception as exc:
                    self.get_logger().warn(f'Error dumping deferred driver diagnostics: {exc}')
        if self._stall_recorder is not None:
            try:
                self._stall_recorder.dump()
            except Exception as exc:
                self.get_logger().warn(f"Error dumping stall diagnostics on destroy: {exc}")
        if self.is_j7_servo_active and self.j7_stop_adapter is not None:
            if self.hardware_mode == HardwareMode.ACTIVE:
                try:
                    self.get_logger().info("Active node shutting down: executing J7 0x1A stop_all.")
                    self.j7_stop_adapter.stop_all("shutdown")
                except Exception as exc:
                    self.get_logger().error(f"Error during J7 shutdown stop_all: {exc}")
            # The node owns this serial handle; close it after the existing one-shot pause path.
            serial_handle = getattr(self.j7_adapter, "serial", None)
            if serial_handle is not None:
                serial_handle.close()
        elif self.controller.is_connected:
            if self.hardware_mode == HardwareMode.ACTIVE:
                try:
                    self.get_logger().info("Active node shutting down: executing best-effort hardware pause_all.")
                    self.controller.pause_all()
                except Exception as exc:
                    self.get_logger().error(f"Error during shutdown pause_all: {exc}")
            self.controller.close()
        super().destroy_node()


def legacy_debug_main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    if os.environ.get('PUS6_J14_SEPARATE_UART_PROCESS') == '1':
        from .j14_process_driver import J14ProcessMotorDriverNode
        node = J14ProcessMotorDriverNode()
    else:
        node = Pus6MotorDriverNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if os.environ.get('PUS6_J14_SEPARATE_UART_PROCESS') == '1':
            if rclpy.ok():
                rclpy.shutdown()
        else:
            rclpy.shutdown()


def main(args: list[str] | None = None) -> None:
    from .production_driver import main as production_main
    production_main(args)


if __name__ == "__main__":
    main()
