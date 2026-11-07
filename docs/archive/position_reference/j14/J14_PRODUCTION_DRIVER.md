> 历史归档：原路径 `docs/j14/J14_PRODUCTION_DRIVER.md`。正文保留当时的方案、参数及证据语境；当前原生 Mode 3 入口以[项目 README](../../../../README.md)为准。历史命令不作为当前推荐启动命令。

# J14 production driver operation

The normal driver entry uses three_process. Parent owns ROS command ingress and process coordination; UART Child owns the motor serial port; Feedback Bridge is the sole feedback socket consumer and publisher. Hardware mode still defaults to mock.

```bash
source /opt/ros/jazzy/setup.bash
source /home/lyapunov/bone_drill/bone_ws/install/setup.bash
ros2 launch pus6_bringup pus6_system.launch.py hardware_mode:=mock
```

The corresponding active entry is `ros2 launch pus6_bringup pus6_system.launch.py hardware_mode:=active`. This writes real hardware. P1 hardware regression uses the unchanged J13 YAML through `config_file:=/home/lyapunov/bone_drill/bone_ws/src/h30_imu/config/j13_ladrc_params.yaml`; no experiment exports are required. P1 authorizes only 30/60-second STATIC SERVO checks.

Driver configuration precedence: installed `three_process.yaml` -> selected system/J13 YAML -> explicit nonempty launch arguments. Driver defaults are independent of controller/geometry configuration; existing J13 and system YAML are retained. Effective options and child configuration are written to production_configuration.json in the audit directory.

| Parameter | Default | Meaning |
|---|---|---|
| driver_architecture | three_process | legacy_single_process explicitly selects rollback |
| enable_coordinated_handshake | true | required for three_process; false rejected before UART |
| driver_telemetry_mode | production | production / qualification / debug evidence retention |
| driver_audit_directory | empty | unique directory under results/j14/production; explicit path selects a run directory |
| servo_run_duration_s | 0.0 | until explicit shutdown; positive values bound qualification windows |
| driver_telemetry_capacity_cycles | 4096 | qualification/debug retained cycles; production recent ring is 300 |

Production profiles freeze 921600 baud, 50Hz, 30ms TX, 5ms RX, 4ms drain, .2833s warning, 1000ms feedback hard stop, 1000ms IPC lease, 200ms target freshness/reconnect semantics, 80-step reconnect rejection and 1500mA validator. These are not tuning parameters in three_process. Neither IPC-age/gap 100ms nor wake lateness 30ms is a hardware STOP threshold; they are recorded watchlist counts.

CHILD_READY + BRIDGE_READY + PARENT_READY precede SERVO_START and Cycle0. No fixed startup sleep replaces readiness. Handshake failure pauses/closes and reports failure. Parent manages and joins both subprocesses. Stop with Ctrl-C; Child uses its existing one-shot six-axis pause then closes the UART. Parent death/lease loss and Bridge socket death invoke local Child pause. A killed Child cannot itself pause; Parent faults and Bridge closes its feedback path. Timeout recovery/reconfigure/resume remain unavailable in this architecture, preserving the qualified A1.13 static behavior; they are not newly production-qualified.

Explicit software rollback:

```bash
ros2 launch pus6_bringup pus6_system.launch.py hardware_mode:=mock driver_architecture:=legacy_single_process
```

The original legacy implementation is retained. Its historically configured feedback names differ from Bridge's `/joint_states`; timed topics stay `/pus6/joint_feedback/timed` and `/pus6/joint_command/timed`. Select legacy explicitly for read_only or original J7/J8 execution policies; pass the desired original scheduler/feedback settings or original driver configuration. P1 tests rollback only with Mock, not another real-hardware experiment.

The normal entry ignores PUS6_J14 experiment flags for architecture/window/bridge selection. `motor_driver_legacy_debug` is a separate, explicitly named executable preserving historical experiment entry behavior; old capture scripts are retained as LEGACY DEBUG references, not as evidence of the new production path. Offline analyses are KEEP. No historical A1 results/docs/raw data were removed or rewritten, and no scripts were physically archived for this cleanup.

Production evidence always retains lifecycle/fault/pause audits, bounded recent cycle and TX/RX evidence, cumulative Child health/extrema and Bridge receive/drop/health summaries. Qualification retains cycle/write/Bridge timing rows up to its configured capacity, enabling distribution and sequence analysis. Debug also retains phase publication timing. No per-cycle synchronous file IO; startup/fault/pause/exit events flush at their boundaries. Qualification longer than approximately 80 seconds needs a larger capacity (e.g. 32768 covers 600s at 50Hz); this is evidence capacity, not a safety gate. P1 itself must stop after 60 seconds. Longer qualification requires the next authorized stage.

Feedback interfaces remain:

| Topic | Type | Local publisher QoS |
|---|---|---|
| /joint_states | sensor_msgs/msg/JointState | KEEP_LAST 10, RELIABLE, VOLATILE |
| /pus6/joint_feedback/timed | pus6_msgs/msg/TimedJointFeedback | sensor_data: KEEP_LAST 5, BEST_EFFORT, VOLATILE |
| /pus6/joint_command/timed | pus6_msgs/msg/TimedJointCommand | sensor_data: KEEP_LAST 5, BEST_EFFORT, VOLATILE |

The existing ROS installation uses rmw_fastrtps_cpp / eProsima Fast DDS. P1 does not switch RMW, tune QoS, socket buffers, OS scheduling or hardware protocols. Child UART/Bridge short-run success does not establish long-duration qualification or a confirmed GIL root cause.
