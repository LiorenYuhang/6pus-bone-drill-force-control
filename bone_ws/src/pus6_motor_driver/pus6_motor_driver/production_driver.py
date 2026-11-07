"""J14 production entry: formal three-process configuration and legacy rollback."""
import json
from pathlib import Path
import threading
import time
import rclpy
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from .motor_driver_node import Pus6MotorDriverNode
from .j14_process_driver import J14ProcessMotorDriverNode
from .j14_process_uart import ProcessClient, append_rows

FORMAL_DEFAULTS = dict(driver_architecture='three_process',enable_coordinated_handshake=True,
    driver_telemetry_mode='production',driver_audit_directory='',servo_run_duration_s=0.0,
    driver_telemetry_capacity_cycles=4096)
THREE_DEFAULTS = dict(servo_scheduler_mode='dedicated_thread',telemetry_publish_mode='async_latest',
    feedback_loss_policy='age_based',reconnect_max_jump_steps=80,target_timeout_sec=.2,
    write_rate_hz=50.0,baudrate=921600,use_j7_servo=True,enable_j2_f0_instrumentation=False,
    hardware_mode='mock')


def validate_configuration(options):
    if options['driver_architecture'] not in ('three_process','legacy_single_process'):
        raise ValueError('driver_architecture must be three_process or legacy_single_process')
    if options['driver_telemetry_mode'] not in ('production','qualification','debug'):
        raise ValueError('invalid driver_telemetry_mode')
    if not 1<=options['driver_telemetry_capacity_cycles']<=32768:
        raise ValueError('driver_telemetry_capacity_cycles must be 1..32768')
    if options['servo_run_duration_s']<0:
        raise ValueError('servo_run_duration_s must be nonnegative (0 = until stop)')
    if options['driver_architecture']=='three_process':
        if not options['enable_coordinated_handshake']:
            raise ValueError('three_process requires coordinated handshake')
        for key,value in THREE_DEFAULTS.items():
            if key!='hardware_mode' and options[key]!=value:
                raise ValueError(f'three_process frozen setting {key}={value!r} required')
        if options['hardware_mode'] not in ('mock','active'):
            raise ValueError('three_process supports mock/active; select legacy_single_process for read_only')
    return options


def read_configuration(parameter_overrides=None):
    """Use ROS's actual CLI/YAML precedence before choosing the implementation."""
    probe=Node('pus6_motor_driver_node',enable_rosout=False,start_parameter_services=False,
               parameter_overrides=parameter_overrides)
    try:
        values={key:probe.declare_parameter(key,value).value for key,value in
                {**FORMAL_DEFAULTS,**THREE_DEFAULTS}.items()}
        return validate_configuration(values)
    finally:probe.destroy_node()


class ProductionDriverNode(J14ProcessMotorDriverNode):
    """Command/lease/health parent; no UART, feedback consumer or publisher."""
    def __init__(self, *, configuration=None, parameter_overrides=None, **kwargs):
        options=configuration or read_configuration(parameter_overrides)
        self._production_options=validate_configuration(options)
        directory=options['driver_audit_directory']
        if not directory:
            directory=str(Path.home()/'bone_drill/results/j14/production'/str(time.monotonic_ns()))
        self._production_audit_path=Path(directory)
        if (self._production_audit_path/'child_final.json').exists():
            raise ValueError('refusing to reuse a completed driver audit directory')
        self._production_audit_path.mkdir(parents=True,exist_ok=True)
        effective=[Parameter(k,value=v) for k,v in options.items()]
        # Preserve non-architecture overrides such as maintenance/port/auto_connect.
        supplied={p.name:p for p in (parameter_overrides or [])}
        supplied.update({p.name:p for p in effective})
        super().__init__(parameter_overrides=list(supplied.values()),**kwargs)

    def create_publisher(self,msg_type,topic,*args,**kwargs):
        if topic in ('/joint_states','/pus6/joint_states','/pus6/joint_feedback/timed',
                     '/pus6/joint_command/timed','/pus6/driver_diagnostics') or topic.startswith('/pus6/joint_') and topic.endswith('_raw'):
            return None
        return super().create_publisher(msg_type,topic,*args,**kwargs)

    def _communication_audit_dir(self):
        return self._production_audit_path

    def _init_j7_servo_backend(self, p_port,p_baudrate,p_timeout):
        for key,value in FORMAL_DEFAULTS.items():
            if not self.has_parameter(key):self.declare_parameter(key,self._production_options[key])
        if self._stall_recorder is not None:
            self._stall_recorder.enabled=False;self._stall_recorder._dumped=True
        self._stall_recorder=None
        self._pipeline_records=[];self._current_pending_snap=None
        self._instrumentation_enabled=False;self.use_feedback_bridge=True
        self._defer_driver_diagnostics=False
        options=self._production_options
        config=dict(production=True,feedback_bridge=True,enable_coordinated_handshake=True,
            telemetry_mode=options['driver_telemetry_mode'],telemetry_capacity_cycles=options['driver_telemetry_capacity_cycles'],
            output_dir=str(self._production_audit_path),epoch=time.monotonic_ns(),
            port=p_port,baudrate=p_baudrate,hardware_mode=self.hardware_mode.value,
            servo_speed=self.servo_speed,maintenance_mode=self.maintenance_mode,
            target_timeout_sec=self.target_timeout_sec,reconnect_max_jump_steps=self.reconnect_max_jump_steps,
            max_servo_duration_s=options['servo_run_duration_s'])
        config.update(self._native_force_child_options())
        (self._production_audit_path/'production_configuration.json').write_text(json.dumps(dict(parameters=options,child_configuration=config),indent=2)+'\n')
        self._process_client=ProcessClient(config)
        self.j7_adapter=self.j7_writer=self.j7_feedback=self._process_client
        self.j7_stop_adapter=None
        ready=self._process_client.ready
        if not self.native_force_control:
            self._process_client.update(ready['positions'],seq=0)
        self._subscribe_native_force_commands()
        self.get_logger().info(f"Production architecture=three_process child={ready['pid']} bridge={self._process_client.bridge_proc.pid}; handshake complete")

    def _telemetry_publisher_loop(self):
        # Base constructor keeps its compatibility scaffolding; no parent feedback work.
        return

    def _accept_child_feedback(self,row):
        raise RuntimeError('production parent cannot consume feedback packets')

    @property
    def telemetry_health_ok(self):
        client=getattr(self,'_process_client',None)
        return bool(client and not self._j7_fault_latched and not client.stopped
            and client.proc.poll() is None and client.bridge_proc is not None
            and client.bridge_proc.poll() is None and client.servo_started)


def create_driver(parameter_overrides=None):
    options=read_configuration(parameter_overrides)
    if options['driver_architecture']=='three_process':
        return ProductionDriverNode(configuration=options,parameter_overrides=parameter_overrides)
    node=Pus6MotorDriverNode(parameter_overrides=parameter_overrides)
    for key,value in FORMAL_DEFAULTS.items():
        node.declare_parameter(key,options[key])
    return node


def main(args=None):
    rclpy.init(args=args)
    node=None;executor=None
    try:
        node=create_driver()
        # Production ROS callbacks only update the bounded latest target or
        # request a stop. Execute them inline: with the threaded executor, a
        # ready subscription can provoke repeated wait-set rescans while its
        # pool handler is pending, starving the independent IPC lease thread.
        # UART and feedback remain in their existing separate processes.
        executor=(SingleThreadedExecutor() if isinstance(node,ProductionDriverNode)
                  else MultiThreadedExecutor(num_threads=3))
        executor.add_node(node);executor.spin()
    except KeyboardInterrupt:pass
    finally:
        if node is not None:node.destroy_node()
        if executor is not None:executor.shutdown(timeout_sec=1)
        if rclpy.ok():rclpy.shutdown()
