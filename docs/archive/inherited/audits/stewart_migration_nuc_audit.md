> 历史归档：原路径 `agent_docs/audits/stewart_migration_nuc_audit.md`。正文保留当时的方案、参数及证据语境；当前原生 Mode 3 入口以[项目 README](../../../../README.md)为准。历史命令不作为当前推荐启动命令。

# 1. 审计范围与当前版本

审计日期：2026-10-13，Asia/Shanghai。研究支线：6PUS Stewart 平台 TDE + Adaptive Robust Control + tanh 平滑鲁棒反馈 + σ-modification 迁移准备度。对象仅为 `/home/lyapunov/bone_drill` 与其 `bone_ws` 的现有 Ubuntu NUC ROS2 工程。

**总裁决：运动执行链可复用；独立运动级姿态残差控制器的接口与离线准备为 PREPARABLE；原论文力级控制律直接迁移为 BLOCKED。** 当前接口输入是目标位置整步，既不是任务空间广义力，也不是已知转矩输入。J13 可提供软件接入参考，不能替代新控制律的模型、量纲、因果关系及稳定性论证。

本轮只读取源码、配置、文档、安装路径和已有实验记录；唯一新增文件是本报告。没有执行项目测试、编译、部署、ROS2 节点、串口查询、电机命令或真机实验，没有生成测试数据、安装依赖、清理日志、commit/push。**本轮 hardware_write_count = 0（审计操作无任何执行器写入；没有以旧 runner 的同名计数冒充 UART 写入计数）。**

## 1.1 当前版本与运行路径边界

| 对象 | 只读观测 | 结论边界 |
| --- | --- | --- |
| 当前 Git | `main`；HEAD `117384dc37248c30ce0ec4b9c36f1754acba9b9f`；提交时间 2026-10-11 19:37:07 +08:00；`docs: archive J14 runtime issue observations` | 本报告的源码基准 |
| J14 冻结 tag | `v1.0-j14-a-three-process-runtime-reliability-qualified` → `ce48ee97f5c827a12591d43a82d32ce97fd4689d` | 资格证据基准，不代表所有当前行为 |
| tag 到 HEAD 的源码差异 | `git diff --name-only <tag> HEAD -- bone_ws/src` 无输出 | 所查生产包、控制算法及包内 YAML 与该 tag 的 Git 内容一致 |
| tag 到 HEAD 的脚本差异 | 7 个 `bone_ws/scripts` 文件：Demo controller/home/launcher、两个 Mock wrapper、startup state machine 测试、比较分析器 | 启动生命周期、遥测记录和离线比较有后续演进，不整体继承 tag 的验证结论 |
| 工作树 | 无 tracked 修改；已有 untracked `blog_materials/`、`results/j14/controller_compare/` 和多次 2026-10-11 Demo 数据 | 未提交实验数据保留，不能当作冻结 tag 内证据 |
| 本次 shell | `ROS_DISTRO=jazzy`；`AMENT_PREFIX_PATH=/opt/ros/jazzy`；未显示项目 overlay | 不等于用户其他终端的 source 状态 |
| Python 安装映射 | 两包 egg-link 指向 `bone_ws/build/...`；所查 production driver、FK 的 build 文件链接解析到当前 `bone_ws/src/...` | 这两个入口有源码路径证据；不扩大为所有安装包一致性证明 |
| H30 安装入口 | `install/h30_imu/lib/h30_imu/h30_imu_node` 解析到 `build/h30_imu/h30_imu_node` | 是已构建二进制；本轮未编译或证明二进制与当前 C++ 源码完全等价 |
| 进程快照 | 未见匹配的 pus6/H30/J13/J14 控制进程，排除本次检索进程 | 不据此推断电源、串口或机器人当前物理状态 |

优先级为当前源码/具体原始字段 → 最新有效交接 → 历史总结。历史文档中的 RS485、话题名称、配置 readback、Fixed-Lag、watchdog、Base 外参和“benchmark”等表述，均逐项核对；发现的差异见后文。不重新认证 J1～J14。

未找到 `stewart_migration_readiness_checklist.md`：在本机用户目录进行该精确文件名检索，覆盖可访问路径；有不可访问路径，不能宣称全磁盘不存在。**本报告不虚构 R01～R59 对照关系**；第 9 节保留可与 NUC-ROS2 / BOTH 主清单合并的检查项名称。未来提供主清单后再映射编号，本轮不改任何已有清单。

## 1.2 状态和证据用法

- **CONFIRMED**：有直接证据，且满足该行明确的用途；不自动扩展到其他用途。
- **PARTIAL**：部分链路/数据/验证已有，仍有关键范围缺口。
- **UNKNOWN**：现有检查不能确定，包括厂商内部细节及论文未提供部分。
- **MISSING**：在明确的当前源码、接口或所选日志范围内未实现/未记录；不是全世界或全部历史不存在。
- **PREPARABLE**：可开始接口定义、既有数据离线准备；不表示准许实机或控制律已成立。
- **BLOCKED**：具体研究主张或执行步骤所需关键前置条件不足。

证据类型：C=源码/配置静态存在；D=文档声明；H=已有实机记录；O=本轮只读离线核查；S=已有软件测试资产。本輪不新增 S/H/仿真验证。下列 E 编号是本报告内部证据索引，不是主清单 R 编号。

| 证据 | 位置与用途 |
| --- | --- |
| E01 C/D | [项目约束](/home/lyapunov/bone_drill/AGENTS.md)、[项目入口](/home/lyapunov/bone_drill/README_CN.md)、[阶段索引](/home/lyapunov/bone_drill/agent_docs/README.md:54) |
| E02 D/H | [J14 最终交接](/home/lyapunov/bone_drill/agent_docs/stages/J14/J14_FINAL_HANDOFF.md)、[生产驱动说明](/home/lyapunov/bone_drill/docs/j14/J14_PRODUCTION_DRIVER.md) |
| E03 C | [生产选择/Parent](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/production_driver.py:14)、[launch](/home/lyapunov/bone_drill/bone_ws/src/pus6_bringup/launch/pus6_system.launch.py:23) |
| E04 C | [Demo 控制器选择/记录器](/home/lyapunov/bone_drill/bone_ws/scripts/j14_demo_controller.py:36)、[launcher](/home/lyapunov/bone_drill/bone_ws/scripts/j14_demo_launcher.py:347)、[node host](/home/lyapunov/bone_drill/bone_ws/scripts/j14_demo_node_host.py:20) |
| E05 C | [J13 控制 tick](/home/lyapunov/bone_drill/bone_ws/src/h30_imu/scripts/j13_ladrc_shadow_runner.py:532)、[LADRC 算法](/home/lyapunov/bone_drill/bone_ws/src/h30_imu/scripts/j13_ladrc.py:390) |
| E06 C | [J11 P/PD 与 SO(3) 合成](/home/lyapunov/bone_drill/bone_ws/src/h30_imu/scripts/j11_m2_pid_residual_runner.py:198)、[Base 前馈](/home/lyapunov/bone_drill/bone_ws/src/h30_imu/scripts/j11_base_feedforward_runner.py:590) |
| E07 C | [IK 节点](/home/lyapunov/bone_drill/bone_ws/src/pus6_kinematics/pus6_kinematics/kinematics_node.py:165)、[解析 IK/数值 FK](/home/lyapunov/bone_drill/bone_ws/src/pus6_kinematics/pus6_kinematics/analytical_6pus.py:130) |
| E08 C | [Shaper 节点](/home/lyapunov/bone_drill/bone_ws/src/pus6_control/pus6_control/control_node.py:205)、[Shaper/Hook](/home/lyapunov/bone_drill/bone_ws/src/pus6_control/pus6_control/joint_trajectory_controller.py:242)、[ControllerInterface](/home/lyapunov/bone_drill/bone_ws/src/pus6_control/pus6_control/controller_interface.py:15) |
| E09 C | [ServoCore/Child](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/j14_process_uart.py:236)、[最终 vector slew](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/j14_process_uart.py:139) |
| E10 C | [Servo 配置](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/j7_six_axis_servo_transport.py:252)、[伴随反馈 drain](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/j7_six_axis_servo_transport.py:449) |
| E11 C | [状态解析](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/protocol.py:260)、[反馈 ingest](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/j7_servo_feedback_scheduler.py:363)、[安全/单位](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/safety.py:200) |
| E12 C | [Bridge 构造消息](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/j14_feedback_bridge.py:180)、[兼容 relay](/home/lyapunov/bone_drill/bone_ws/scripts/j14_a2_q3_feedback_relay.py:77) |
| E13 C | [TimedJointFeedback](/home/lyapunov/bone_drill/bone_ws/src/pus6_msgs/msg/TimedJointFeedback.msg)、[TimedJointCommand](/home/lyapunov/bone_drill/bone_ws/src/pus6_msgs/msg/TimedJointCommand.msg) |
| E14 C | [几何和 P 副坐标配置](/home/lyapunov/bone_drill/bone_ws/src/pus6_kinematics/config/6pus_geometry.yaml:52)、[J13 YAML](/home/lyapunov/bone_drill/bone_ws/src/h30_imu/config/j13_ladrc_params.yaml)、[J11 YAML](/home/lyapunov/bone_drill/bone_ws/src/h30_imu/config/j11_m2_pid_params.yaml) |
| E15 D | [厂商 V2.0.4 手册](</home/lyapunov/bone_drill/hardware/电机手册/LAF因时机器人微型伺服电缸（力闭环专用）--LA用户手册（电气）V2.0.4cn.pdf>)：§1.1、§1.2、§3.4、§3.5.4；本轮只读提取相关文字 |
| E16 D/H | [原始 UART 力基线核对](/home/lyapunov/bone_drill/up/motor/uart_multi/UART力基线核对.md)、[六电机接口核对](/home/lyapunov/bone_drill/up/motor/uart_multi/UART六电机编号与测试方案.md:375) |
| E17 C | [H30 publish](/home/lyapunov/bone_drill/bone_ws/src/h30_imu/src/h30_imu_node.cpp:84)、[H30 parser](/home/lyapunov/bone_drill/bone_ws/src/h30_imu/src/h30_protocol.cpp:35) |
| E18 C | [Fixed-Lag synchronizer](/home/lyapunov/bone_drill/bone_ws/src/pus6_kinematics/pus6_kinematics/state_synchronizer.py:54)、[FK 节点](/home/lyapunov/bone_drill/bone_ws/src/pus6_kinematics/pus6_kinematics/fk_node.py:61) |
| E19 D/H | [J9 总结](/home/lyapunov/bone_drill/agent_docs/stages/J9/J9_FINAL_SUMMARY.md)、[J11 Base 轴映射资格记录](/home/lyapunov/bone_drill/agent_docs/stages/J11/README.md:469)、[对应 axis summary](/home/lyapunov/bone_drill/results/j11/j11_3a1/j11_3a1_base_imu_axis_summary.json) |
| E20 C/D/O | [离线角加速度分析](/home/lyapunov/bone_drill/bone_ws/src/h30_imu/scripts/j9_1_platform_lever_arm_analysis.py:233)、[原始双 IMU 记录工具](/home/lyapunov/bone_drill/bone_ws/src/h30_imu/scripts/j9_1_dual_imu_translation_recorder.py:164)、[Platform recorder](/home/lyapunov/bone_drill/bone_ws/src/h30_imu/scripts/j9_1_platform_imu_recorder.py:225) |
| E21 D/H | [J8 资格总结](/home/lyapunov/bone_drill/agent_docs/decisions/J8_FINAL_HARDWARE_QUALIFICATION.md)、[J10 单轴工作包络](/home/lyapunov/bone_drill/agent_docs/stages/J10/J10_FINAL_SUMMARY.md)、[HOME 附近跟踪证据](/home/lyapunov/bone_drill/agent_docs/reports/REAL_HARDWARE_PERFORMANCE_SUMMARY.md) |
| E22 D/H | [J12 总结](/home/lyapunov/bone_drill/agent_docs/stages/J12/J12_FINAL_SUMMARY.md)、[J13 总结](/home/lyapunov/bone_drill/agent_docs/stages/J13/J13_FINAL_SUMMARY.md) |
| E23 H/O | [Q2-R3 600 s 原始 cycles](/home/lyapunov/bone_drill/results/j14/j14_a2_q2_r3_static_600s/child_cycles.jsonl)、[详细统计](/home/lyapunov/bone_drill/results/j14/j14_a2_q2_r3_static_600s/q2_detailed_analysis.json) |
| E24 H/O | [Q3 Classical 原始 cycles](/home/lyapunov/bone_drill/results/j14/j14_a2_q3_r1d_classical_60s/child_cycles.jsonl)、[analysis](/home/lyapunov/bone_drill/results/j14/j14_a2_q3_r1d_classical_60s/q3_analysis.json) |
| E25 H/O | [Q3 LADRC 原始 cycles](/home/lyapunov/bone_drill/results/j14/j14_a2_q3_r2_ladrc_60s/child_cycles.jsonl)、[analysis](/home/lyapunov/bone_drill/results/j14/j14_a2_q3_r2_ladrc_60s/q3_analysis.json)、[控制遥测](/home/lyapunov/bone_drill/results/j14/j14_a2_q3_r2_ladrc_60s/j14_a2_q3_r1_ladrc_telemetry_powered.csv) |
| E26 H/O | [最新 PD run_002](/home/lyapunov/bone_drill/results/j14/controller_compare/pid/run_002/telemetry.csv)、[metadata](/home/lyapunov/bone_drill/results/j14/controller_compare/pid/run_002/metadata.json)、[配置](/home/lyapunov/bone_drill/results/j14/controller_compare/pid/run_002/production_configuration.json) |
| E27 H/O | [最新 LADRC run_002](/home/lyapunov/bone_drill/results/j14/controller_compare/adrc/run_002/telemetry.csv)、[metadata](/home/lyapunov/bone_drill/results/j14/controller_compare/adrc/run_002/metadata.json)、[配置](/home/lyapunov/bone_drill/results/j14/controller_compare/adrc/run_002/production_configuration.json) |
| E28 O/D/H | [第二轮比较](/home/lyapunov/bone_drill/results/j14/controller_compare/comparison/real_run_002/summary.md)、[归一化比较边界](/home/lyapunov/bone_drill/results/j14/controller_compare/comparison/real_run_002/normalized_disturbance_summary.md)、[比较分析器](/home/lyapunov/bone_drill/bone_ws/scripts/analyze_pid_adrc_compare.py:29) |
| E29 D/H | [IMU freshness 故障](/home/lyapunov/bone_drill/agent_docs/issues/J14_IMU_FRESHNESS_TIMEOUT_OBSERVATION.md)、[电机 USB-UART 故障](/home/lyapunov/bone_drill/agent_docs/issues/J14_USB_UART_DISCONNECT_RUNTIME_OBSERVATION.md) |
| E30 H/O | [J9 Pitch+ 采样统计](/home/lyapunov/bone_drill/results/j9_1/rotation_lever_arm_characterization/j9_1_rotation_lever_arm_pitch_pos05deg_20260907_115030_metadata.json)、[J9 X− 配对/FK 数据](/home/lyapunov/bone_drill/results/j9_1/translation_characterization/j9_1_x_translation_neg05mm_20260903_204626_metadata.json) |

# 2. J1～J14 可复用资产

本节只盘点与迁移有关的资产；历史阶段验证不逐项重跑。阶段索引 E01 是导航，具体复用以当前 C 和相应 H 范围为准。

| 阶段 | 可以利用的资产 | 当前用途与限制 |
| --- | --- | --- |
| J1 | 六 ID UART 帧协议、状态解析、原始力基线 | 通信字段与测量语义参考；不新增主动 UART 客户端 |
| J2/J3 | 6PUS 解析 IK、SI 与整步转换、latest-target 六轴向量调度、FakeBus 资产 | 当前实现存在；接入新姿态目标可复用，不推定完整机构动力学已建模 |
| J4 | StatefulJerkLimitedShaper、速度/加速度/jerk 限制、HOME/PARK | 运动平滑和生命周期复用；其内部 dq/ddq 是参考轨迹状态，不是实际测量速度/加速度 |
| J5 | Timed command/feedback、逐轴历史、Fixed-Lag、数值 FK | 状态与输入记录基础；不能直接视为任务空间动态估计器 |
| J6 | 装配后零位、机械方向、HOME/PARK 的历史流程与几何管理 | 保留约定；几何 YAML 仍含 preassembly/pending 标签，不能当最终精密标定证书 |
| J7 | mode=1 六轴 Servo、0x29 位置流、零重试、暂停、安全验证器 | J14 Child 当前仍使用成熟 transport/scheduler；早期 C6 低频轮询说明不代表当前 ACTIVE |
| J8 | 0x29 伴随反馈、连续 Cartesian 链、同步 FK、九阶段实机记录 | 可复用运动/观测资产；J8 的旧时序与 watchdog 不能直接代表 J14 |
| J9 | 双 H30、相对四元数、Platform 轴变换、联合 IMU/关节/FK 日志、力臂分析 | 有姿态/gyro/加速度历史；联合记录不等于硬件同步；力臂未可靠辨识 |
| J10 | HOME 附近单轴/多目标 harness、工作包络和到位指标 | 可选后续小范围研究点；单轴包络不是耦合 6D 全域，更不是外部精度认证 |
| J11 | Base SO(3) 前馈、世界系误差、Roll P/Pitch-Yaw PD、单轴/多轴 runner | 现行 PD 参照、姿态合成及实验生命周期；Ki=0，不标完整 PID |
| J12 | 静态/组合姿态/手持高动态 runner 和统计、1500 mA 保护依据 | 负载/执行限制研究参考；其统计不能跨不同扰动直接当公平算法 benchmark |
| J13 | 独立 ThreeAxisLadrc、shadow/roll_active/3axis_active、接管检查、负载 A/B 原始记录 | 最接近 TDE 的外环软件模板；一阶运动级模型不等于旧论文力级二阶模型 |
| J14 | Parent/Child/Bridge、握手、lease、生产/qualification 记录模式、三 Demo、完整 controller CSV 选项、比较绘图 | 当前首选运行基础；新算法仍需独立版本与独立验证，尚无 TDE 模式 |

# 3. 当前 ROS2 控制数据流

## 3.1 实际链路

现行姿态稳定中，姿态控制器先生成 Cartesian Target；并不是一个外部 Cartesian Target 先进入通用姿态控制节点。J14 Demo 的选择器调用包外研究 runner：

```text
Base H30 /pus6/imu_base/data ── relative quaternion + Rz(+90°) ── R_WB
Platform H30 /pus6/imu_platform/data ── relative quaternion + 同一轴变换 ── R_WP
  R_WB → Base SO(3) 逆旋转前馈 R_BP_FF
  R_WP → 世界系 SO(3) 误差 e_R_W → Classical P/PD 或 J13 LADRC → Δφ_W [rad]
  → SO(3) 合成 R_BP_cmd；平移目标 [0, 0, z0]（Base 系）
  → /pus6/target_cartesian_pose : geometry_msgs/PoseStamped
  → pus6_kinematics_node : 解析 differential IK
  → /pus6/desired_joint_deltas : std_msgs/Float64MultiArray，6 个相对 HOME 位移 [m]
  → pus6_control_node : JointTrajectoryController + 有状态三阶 shaper
  → /pus6/command_joint_positions [m] / /pus6/command_joint_steps [step]
  → ProductionDriverNode Parent：ROS 最新目标 → command IPC / lease
  → UART Child：final vector slew → ID1..6 的 0x29 目标位置写入
  → 电缸内部位置插补/跟随 → 0x29 应答 StatusFeedback
  → Child ingest：实测位置、电流、fault → feedback IPC
  → Feedback Bridge：/joint_states、/pus6/joint_feedback/timed、/pus6/joint_command/timed
     ├─ compatibility relay → /pus6/joint_states → shaper 的观测/初始化
     ├─ compatibility relay → /pus6/joint_steps_raw → controller 的反馈监测
     └─ FixedLagJointSynchronizer → FK → /pus6/fk/state、/pus6/fk/pose（观测支路）
Platform IMU 的姿态测量返回姿态残差闭环；FK 当前不返回 J13 控制律。
```

数据流根据 E03～E13、E18 还原；不是采用历史总结的示意话题。Parent 不做 UART 事务，Child 独占 UART，Bridge 独占反馈 IPC。Demo 同时启动兼容 relay；**仅运行通用 production launch，不能自动假定 `/pus6/joint_states` 与旧 raw 话题已桥接齐全**。默认 `/joint_states` 与 shaper 订阅的 `/pus6/joint_states` 名称不同，这是现有兼容边界，不在本轮修复。

## 3.2 控制器节点、函数和数据类型

- PD：`J11M2PidResidualRunner`，ROS 节点 `j11_m2_pid_residual_runner`；`_on_control_tick()` 调用该 runner 文件内 `WorldFrameAttitudePid.update(e_R_W_rad, dt_s)`；不能仅看 `j11_task_space_attitude_pid.py` 的通用类就认定现行 D 项使用 gyro。
- LADRC：`J13LadrcShadowRunner`，ROS 节点 `j13_ladrc_shadow_runner`；`_on_control_tick()` → `ThreeAxisLadrc.step(e_R_W_rad, actual_previous_input_rad, dt)` → 三个 `FirstOrderLadrcChannel`。入口虽名为 Shadow，Demo 通过 `controller_mode='3axis_active'` 选择真实控制权。
- 前馈：`R_BP_FF = R_WB.T`；误差为 `Log(R_WP_des @ R_WP.T)` 的世界系旋转向量，当前 `R_WP_des=I`，W 是启动捕获参考，不是独立校准的全局绝对参考。
- 合成：`ΔR_W=Exp(Δφ_W)`；`ΔR_B=R_WB.T @ ΔR_W @ R_WB`；`R_BP_cmd=ΔR_B @ R_BP_FF`，等价于此参考条件下 `R_BP_FF @ ΔR_W`。不能随意改成欧拉角逐项相加。
- 内部输入：3 元 `Sequence[float]`/NumPy `float64`，误差 rad、历史补偿角 rad、dt s。LADRC 返回 `(u_clipped, u_raw, z1, z2, saturated)`，前三/四项为三轴数组，最后为布尔列表；输出是补偿角 rad。
- ROS 姿态输入是 `sensor_msgs/Imu`；控制输出是 `PoseStamped` 的 m 和 quaternion xyzw；关节执行是六元向量，不是广义力消息。

源码存在 `ControllerInterface.reset/update(current_state,target_state,dt)`，当前实现它的是六轴 `JointTrajectoryController`。J11/J13 姿态控制器没有统一采用这一接口；`OuterLoopControllerHook` 默认输出六轴零修正，**它既不是当前 LADRC 入口，也不是任务空间 TDE 的现成插件接口**。统一姿态接口：PARTIAL。

# 4. 电机驱动接口与真实反馈

## 4.1 实际控制模式和输入

**CONFIRMED（当前实现与已有记录）：Type-D UART Servo/follow-up mode=1，位置目标控制。** Child 启动读状态后，`configure_servo()` 写 `0x25..0x28=[1,0,0,speed]`；连续运行通过 `CMD_WR_REGISTER=0x32` 写 `0x29` 目标位置，每周期 ID1～ID6 顺序发送。当前标称 50 Hz、921600 baud；`speed=50` 是该配置字段，不能称每周期 ROS 速度命令或保证真实恒速 50 step/s。

注意跨阶段差异：早期 J7 C0/C1 设计写 mode/speed 后读回 `0x25/0x28`；**当前 six-axis transport 的实际函数明确是 acknowledged write，无运行时寄存器 readback，返回 readback 字段为 None**。历史 J6 的 mode=0 定位路径、厂商支持的 mode=2/3/4/5，都不代表当前 ACTIVE 模式。协议有力控模式能力，现行 ROS2 production 未接入力控，更未验证任务空间广义力分配。

厂商手册 E15 确认内置绝对位置传感器及 position servo/插补；可确认设备内部存在位置跟随控制功能。内部独立位置/速度/电流环的具体拓扑、增益、频率、带宽、滤波和锁存时刻为 **UNKNOWN**。不能把名称“Servo”当作已知三环模型，也不能把电流反馈当作电流输入接口。

## 4.2 反馈源与测量质量

| 量 | 真实来源 | 当前可用程度 |
| --- | --- | --- |
| command steps | 外环预测 `predicted_steps`；shaper 发布 `commanded_steps`；Child 最终 `targets`；驱动状态另有 target_position 回显 | 各层含义不同；前两者不等于最终 UART TX/内部接受/实际到位 |
| actual steps | `StatusFeedback.current_position`；实际位置传感器对应的驱动状态读数，非命令复制 | CONFIRMED 可作为驱动报告位置；不是外部尺/光学 ground truth；传感器内部实现细节 UNKNOWN |
| P 副位置 | `raw_steps=s`；设备伸出量 `s*25 μm`；几何坐标 `q=0.050+(s−1000)*25 μm` | CONFIRMED 有两种表示；HOME 分别为 0.025 m 和 0.050 m，固定相差 0.025 m；按用途转换，不能混用 |
| current | 状态应答 `current_mA`，uint16；Bridge `currents_ma` | CONFIRMED 驱动电流测量字段/传递路径存在；采集位置、标定及力矩换算 UNKNOWN；不是已知电机相电流或广义力 |
| force | 状态应答 `force_g` int16 与 `force_adc` uint16，协议 `0x2C/0x2D`；历史 LAF 力基线有实机读数 | PARTIAL；有传感/原始字段，但当前 Child 的 FeedbackSample/IPC 不保留，Bridge 不填 `forces_g_raw`，没有当前完整六轴校准力序列 |
| temperature / fault | 原始协议温度 int8、fault uint8；Child/Bridge 保存 fault | fault 可用；Bridge 不填 temperature，不能把 Timed 消息默认零认作温度实测 |
| torque / task wrench | 未见当前节点提供校准六维任务力/力矩 | MISSING（所查 production 接口）；力/电流转换、Jacobian 映射和标定仍不足 |
| actual velocity / acceleration | JointState 当前不填 velocity/effort；无实测电缸速度/加速度字段 | MISSING（当前反馈消息路径）；可以从位置估计，但未有合格在线估计 |

E16 的 ID1 无载力基线约 −428 g，翻转约 −382 g，旧版零点地址与现行协议不能等同。**不是“没有力传感器”，也不是“已有可直接用于 TDE 的真实广义力”。** 保留协议值；未经标定的 force_g/ADC、current_mA 不用于反推已知输入转矩。本轮不写任何校准/模式寄存器。

J14 的额外断层：ProductionDriverNode 抑制旧 raw publishers；Bridge 发 Timed 和 JointState；relay 仅转发 raw_steps 与 JointState，没有转发 `/pus6/joint_currents_raw`、`/pus6/joint_faults_raw`。PD/J13 runner 的这两个旧话题缓冲初始为六个零，J13 的 timed 回调只保存消息用于时延分析，没有用它更新旧缓冲。**本轮核查 E26/E27 全部 ACTIVE 行：`joint_currents_raw_mA`、`faults` 均只有 `[0,0,0,0,0,0]` 一种值。** 这些 CSV 列不能视为当前六轴实测电流/无故障证明；底层 Child validator 和 `/rosout` fatal 传播仍存在，该缺口不等于底层过流保护失效。

## 4.3 限制与异常处理

当前源码/配置：raw 命令硬范围 `[0,2000]`；历史 software `[0,1800]`、normal `[200,1800]` 与 maintenance 分支分别存在，所查 J14 实机配置 `maintenance_mode=true` 使用硬范围。Shaper 速度 0.005 m/s、加速度 0.050 m/s²、jerk 0.500 m/s³；Child vector slew 使用约 200 step/s 的 nominal allowance，`min(20,max(5,int(200*dt)))`，最终单次差值≤20 step，所以不能把它简化为严格恒定 200 step/s。

保护包括：1500 mA 主机状态验证、非零 fault、位置范围、200 ms target freshness、80 step reconnect 拒绝、1000 ms Child feedback/Parent lease、单次六轴 0x1A Pause、写命令零自动重试。TX 30 ms、serial RX 5 ms、drain budget 4 ms 是局部配置，**不是完整控制回路 20 ms 最坏时延保证**。controller 另有 100 ms IMU age、1500 ms motor feedback 回调 age；这三个时间尺度不是同一个 watchdog。精确阈值以 E09/E11 当前实现为准，旧 J8 四个 miss/263 ms 描述不能套用 J14 age_based。

正常 Demo 退出：停止外环 → Cartesian 链受控 HOME → 反馈确认 → Pause → 资源清理；FAULT 跳过 HOME。完整断连后的 Pause ACK 未获确认时，不能因进程清理成功就宣称执行器已确认停止。自动 recovery/resume 仍未 qualified。

# 5. IMU / FK / 状态估计

## 5.1 当前姿态与速度

H30 C++ 节点发布 quaternion、gyro（deg/s 转 rad/s）、线加速度（m/s²）；缺失字段以 covariance[0]=−1 标记。header 是 `now()` 的主机 ROS 发布时间，parser 的 tid 没有成为 sensor time/sequence 字段。可确认传感接口与历史 H 存在，不能凭接口字段证明本次每帧都有 gyro、accel 或准确内部采样时刻。

当前 J11/J13 姿态 callbacks 只保留 quaternion 和主机回调 `perf_counter_ns`，丢弃原消息 header、tid、gyro、accel。因此“ROS 话题能提供”与“控制器输入/CSV 已包含”必须区分：平台姿态 **CONFIRMED**，gyro 可获取但现行残差控制与最新 CSV **PARTIAL**；平台角加速度在线估计 **MISSING**。

现行 P/PD 的 D 项是 SO(3) 误差在 controller tick 间的差分，再经 5 Hz 一阶 LPF，不是实测 gyro。通用 TaskSpaceAttitudePid 的可选 gyro 参数不是现行 runner 的使用证据。误差导数不等于平台角速度；SO(3) 误差、body/world angular velocity 与 Euler rate 都要按参考和旋转 Jacobian 区分。

跨阶段外参更新：J9 总结说 `R_B_IB` 未知；J11-3A1 后续做了机械轴候选核对，现行 Base 和 Platform 均使用 `Rz(+90°)` 对相对旋转作共轭变换。故当前轴映射不是照抄 J9 的 UNKNOWN。**用途为既有姿态坐标转换可确认；连续精密外参、安装变化后重复标定及 IMU 杆臂仍 PARTIAL/UNKNOWN**。姿态“world”是捕获参考，不等于外部绝对姿态准确度。

## 5.2 FK 与平移

FK 直接读取 Timed feedback 的几何 P 副坐标，经 FixedLagJointSynchronizer 逐轴历史线性插值、warm-start 数值 FK，输出 pose、synced q/raw steps、query_monotonic_timestamp_ns、残差和分支连续性诊断。只有能 bracket 查询时间时才插值，不外推。

源码类默认 query 200 Hz、lag 50 ms；**通用 launch 明确覆盖 lag 为 160 ms**，system YAML 也有 160 ms，J13 YAML 不提供 FK 节点块。当前 Demo 走该 launch，因此不能把类默认 50 ms 当当前配置，也不能把 200 Hz FK 查询当 200 Hz 独立物理测量。FK 每次读取的是已延迟反馈；后续细节受调度和 bracket 缺失影响。

J9/J10 有 FK 平移及 cross-axis 统计，能支持 HOME 附近局部追踪；最新 J14 对照 CSV 没有 FK pose 或 XYZ hold error，虽然 fk_node_host.json 显示该节点曾启动。**节点运行记录不证明保存了对应完整 FK 序列。** 当前外环输出固定 Base 系 `[0,0,z0]`，没有平移误差控制输入；移动基座的世界位置保持不能据此认定成立。

几何 YAML 仍标 `6pus_preassembly_software_reference`、多个 pending，含 IMU null 外参和杆臂。部分功能已由 J9/J11 实机建立，标签未全部更新；它仍是当前计算模型，不是完整实测机构/惯性参数表。质量、惯量、偏心质心、关节柔性/摩擦与驱动内环模型在所查当前控制路径中未形成可用完整动力学。

## 5.3 加速度与因果性

J9 力臂工具 E20 已计算 `omega` 平滑后 `numpy.gradient(...,t)` 的角加速度；工具明确属于离线 moving-average + central difference，内点使用未来样本。**它是已有离线分析资产，不是因果在线 TDE 估计器。** 当前没有可直接复用的在线 angular acceleration module。

迁移建议（推理，不是验证结论）：优先保存并转换实测 gyro，再比较后向差分+因果低通/观测器方案；使用真实 sample interval，记录滤波延迟和噪声。直接对 50 Hz 姿态两次差分会放大高频误差，量化幅度尚 UNKNOWN，必须用已有原始 gyro/姿态离线评估，不能只从数学式宣布可用。传感器 linear_acceleration 不是 angular_acceleration，也不是平台中心平动加速度；后者需要重力、杆臂和基座运动处理。

# 6. 采样、延迟与 TDE 适用性

## 6.1 时间字段的真实意义

| 数据源 | 可核查时间 | 不能据此得到的时间 |
| --- | --- | --- |
| controller | timestamp_ns=perf_counter_ns，dt_ms 为实际 tick 差；Base/Platform callback age | IMU 内部采样时间、每帧原始接收历史 |
| H30 Imu | header=驱动主机 ROS now；旧 recorder 另存主机接收时间 | 已校准设备时间或两只 IMU 硬件锁存同步 |
| UART TX | per_axis_tx_start/done、transaction_start/end、cycle seq | 电机内部接受/插补开始/物理到位的精确时刻 |
| UART feedback | per_axis_rx_done、snapshot start/end、cycle seq | 全部六轴同时物理采样时刻 |
| FK | query_monotonic_timestamp_ns；另有 header publish time | 当前时刻姿态，不加 lag 的瞬时 ground truth |

更具体的当前时序差异：`ingest_cycle_feedback()` 在 drain 后读取一个 `now_ns`，对本周期所有收到应答的轴赋相同 FeedbackSample.timestamp_ns；Child 传给 `rx_ns`，Bridge 写 `per_axis_rx_done_monotonic_ns`。因此 ACTIVE 路径该字段实质上是**同批 host ingest 完成代理**；缺失轴保留旧时间。不能仅凭数组名称认定当前拥有六个独立逐帧 RX 完成时刻。启动 full_scan 的逐轴事务时间语义也不能无条件移植到 ACTIVE。按该时间插值是可用软件 reconstruction，但没有恢复串行物理采样偏差。

Host monotonic/perf_counter 和 ROS/epoch 是不同字段域；跨进程/日志需显式确认同一主机、boot、clock 实现与 epoch，不能把数值相近直接相减。runner 日志的 `hardware_write_issued` 表示发布 Cartesian target，runner 的 hardware_write_count 在发布函数中累加；它们不表示 UART TX 已确认。

## 6.2 名义周期和已有实测

以下原始统计只读核查，不产生新实验数据。Child interval 指相邻 `child_cycles.jsonl.start_ns` 差；controller 指 ACTIVE timestamp 差。窗口不同，不作相互等同。旧分析器 p99 插值与本轮取排序分位的细微差异不改变判断；表中仅列可直接核对的 median/max。

| 记录 | 范围 | 实际周期 median / max | 说明 |
| --- | --- | --- | --- |
| Q2-R3，600 s STATIC | 29,995 Child cycles | 19.999731 / 68.541268 ms | 3 次 deadline reset；cycle wall max 68.375108 ms；Bridge gap max 80.774919 ms；无本次 TX timeout，不是零长尾 |
| Q3-R1D Classical | 3,529 Child session cycles | 20.000071 / 20.686566 ms | controller 2,999 行、ACTIVE 2,945 行；driver 时长含不同生命周期窗口 |
| Q3-R2 LADRC | 3,539 Child session cycles | 20.000091 / 21.181148 ms | controller 2,997 行、ACTIVE 2,942 行；不得用全部 driver cycles 除以标称 controller 60 s 推断控制率 |
| 最新 PD run_002 | 1,215 ACTIVE 行，24.279969163 s | 20.000161 / 21.030759 ms | 原始时间严格增长，反映该窗口约 50 Hz |
| 最新 LADRC run_002 | 1,163 ACTIVE 行，23.239906447 s | 19.999573 / 20.420006 ms | 原始时间严格增长，不能外推到全部长期会话 |

E30 的 J9 Pitch+ metadata 显示 Platform/Base effective callback rates 约 131.50/128.73 Hz；J11 文档示意 400 Hz 不能代替所有既有记录的实测。J9 X− 的 nearest timestamp 配对 P95 5.8776773 ms、max 15.833161 ms，属于 host 配对，不是硬件同步。本轮无法从最新 J14 CSV 得到 IMU 的原始采样周期或丢包率，因为它们是 50 Hz latest-value 快照。

J14 修复缓解了已查 production 时序瓶颈，不能据 Q2/Q3 PASS 声称 TDE 延迟条件成立。2026-10-11 的 controller IMU age timeout 和 CH341 EIO 仍有证据（E29）；后续通过不证明该类中断彻底消除。freshness 超时反映该 controller 回调未更新，不自动等于 IMU 硬件停采；CH341 断连是电机通信中断，不自动归因于控制器算法。故障 tick 可能在 telemetry append 前返回，精确瞬时状态缺失。

## 6.3 历史输入、状态与 TDE 的关系

已有：上一周期选中/限幅 residual `u_plant_actual`、controller telemetry_history、完整 CSV 选项；Timed TX/feedback 的共同 UART cycle sequence；Fixed-Lag q 历史；多个旧 IMU 原始 recorder。

缺少：统一状态历史、姿态/gyro 原始消息 seq 与 source/rx 时间、外环 residual 到 shaper 到最终 TX 的完整关联、与对应状态有效时刻一致的输入历史、因果加速度、sample validity、完整运行窗口保留。J13 的 `self.u_plant_actual=u_combined_actual` 在 IK 检查/发布前就保存，不是基于 actuator feedback 重构的执行量；shaper 和 Child slew 还会改变随后实际下发曲线。

最新 run_002 的完整 controller CSV 存在，但 `driver_telemetry_mode=production`，两组 Child/cycle_summary_full 仅保留 300 cycles，RX event CSV 只有表头，write records 可为空。文件名 `full` 不代表全窗口。LADRC 有 timed_fb_seq 和时延列，PD 无同等 timed 字段；原始 IMU/Gyro/FK 都未在这两个 controller CSV 内保存。旧 qualification Q2/Q3 有更完整 Child/Bridge 时序，但也不能补回未保存的 IMU 消息序列。

**TDE 历史条件：PARTIAL，而非基本齐全可直接运行。** 可利用既有数据做离线窗口、输入语义和对齐准备；固定 delay L 不应简单理解为“取上一行 20 ms”。若采用时刻查询历史，必须只用当时已获得的数据；Fixed-Lag 使用 t_query 之后的 bracket 样本，对过去状态重建可因果于当前，但不能把重建结果伪标为 t_query 时已可获得，更不能在离线重放中产生前视。

# 7. PD / LADRC 与现有实验资产

## 7.1 控制参数和输出语义

现行 Classical：Kp=[0.05,0.05,0.025]，Kd=[0,0.0015,0.0015] s，Ki=0，D LPF=5 Hz，三轴 residual 限幅 ±0.5°。现行 LADRC：b0=[20,10,6] s⁻¹，ωc=[0.8,1,0.25]，ωo=[2.5,4,1.2]，限幅 ±0.5°，一阶模型 `e_dot=f−b0*u`。z1 为误差 rad，z2 为等效扰动 rad/s；这既不是 Nm，也不是角加速度。

三轴 active 时 `u_combined_actual` 是选用 LADRC residual，`pid_total_residual` 是并行 Classical 计算。shadow 时 Classical 保持控制权；roll_active 只替换 Roll。所有模式仍共享同一 SO(3) FF/合成/Cartesian 输出。数据比较应按模式选择真实输出列，不能以 `pid_total_residual` 评价 LADRC，也不能把 raw、clipped、shaped、TX、actual-position 归为同一个 u。

LADRC 的 dt 超出 [1,100] ms 会改用 nominal 20 ms；Classical 会 clamp 过大的 dt 至 100 ms。这是已有算法防护语义，不是严格历史对齐证明；不建议盲目继承为 TDE 的 delay policy。

## 7.2 可以利用的 runner 与统计

现有三模式 Demo shell → `j14_demo_launcher.py` → `j14_demo_controller.py` 可作为下一阶段共同 HOME、reference、ACTIVE、正常停机基础；当前 selector 不支持 TDE，必须在独立新研究命名空间适配。J13 的 staged launcher/shadow runner 可借鉴 shadow 和单轴接管；旧 launcher 会启动硬件组件、涉及固定 marker 与特定配置，不能直接作为通用三算法开关。

J10 单目标 workspace harness、J9 单轴姿态/平移及原始双 IMU recorder、J12 static/combined/dynamic runner 有可利用代码。后续以同一新 runner 驱动三种算法，保留相同几何、FF、Shaper、runtime、配置和初始状态，比直接拼接旧阶段 runner 更适合公平比较；本轮只记录需求，不运行。

当前 `analyze_pid_adrc_compare.py` 支持现有 PD/LADRC identity、ACTIVE、共同窗口、原始 sample RMS/range/peak 与绘图；TDE identity/output 未支持。它可复用指标核心，需要新阶段适配；采样权 RMS 也不是非均匀采样的时间积分 RMS。J12 动态/负载统计、J13 observer 离线 replay 与 J14 transport/latency 分解代码均可参考。

## 7.3 偏心负载与公平性

E22 的 J13 偏心 A/B 有真实闭环数据，但 payload 质量、质心力臂和安装偏角未精确标定，人工 Base 波形不同；LADRC Pitch 限幅也参与结果。E28 的最新两轮 PD/LADRC 比较明确为 representative comparison。最新第二轮 Base RMS 比例在 Roll/Pitch/Yaw 不同，RMS 归一化不能消除频谱、跨轴相关性、饱和、参考和时序差异。

因此历史数据适合提出问题、设定预期幅度、构造数据 schema 和离线检查，**不满足三算法公平定量因果排名，也不证明偏心模型不确定性的具体参数**。后续需记录实际 payload mass/CoM/安装位置、扰动 waveform/频谱、共同持续时间、重复次数、失败与饱和率，以及平移保持误差；不把 runtime PASS 当姿态性能 PASS。

# 8. 新算法建议接入位置

建议使用独立的新阶段姿态 runner/算法模块，位于 **Base SO(3) FF 与 Cartesian pose 合成之间的世界系姿态残差层**，输出 `Δφ_W [rad]` 后走已有 IK/Shaper/Child。J13 是参考，不原地修改 frozen J13，不用关节 Hook 绕过姿态几何合成。此建议是基于当前输入能力的工程路线，不是力级原算法的等价实现结论。

| 用户问题 | 状态 | 明确判断 |
| --- | --- | --- |
| 1. 接入节点/控制层 | PREPARABLE | 新阶段独立 runner，参考 J13 `_on_control_tick()` residual 分支；原有唯一 Cartesian 发布权和生命周期需要保留 |
| 2. 直接复用 LADRC I/O | PARTIAL | 可沿用 e_R_W、dt、三轴 residual rad、诊断/限幅和 Pose 输出格式；不能仅替换 step 函数就满足旧论文力输入/二阶状态假设 |
| 3. 新状态/缓存 | MISSING | 新增原始 q/gyro、参考轨迹及其导数、有效时刻、原始/限幅 residual、shaped/TX/actual q、各层 seq/validity、延迟历史查询、自适应参数及 σ 状态 |
| 4. 速度/加速度估计 | PARTIAL | gyro 已有传感接口；需新因果速度/加速度处理和噪声/延迟评价。若重新推导一阶运动级 TDE，不一定必须 angular acceleration；旧二阶律则需要 |
| 5. TDE 估计对象 | UNKNOWN | 可研究位置内环以上的等效姿态运动残差，包含耦合/负载/执行滞后；不能将其解释为已测真实力矩。需先定义阶次、输入映射和 residual 量纲 |
| 6. 运动级内环模型 | BLOCKED | 缺补偿角→IK→shaper→slew→位置内环→测量姿态的局部有效动态/输入增益/延迟模型；LADRC b0 不能视为质量/惯量参数 |
| 7. 平移姿态耦合约束 | PARTIAL | 复用完整六轴 IK、固定 Base 系平移目标、全轴行程/分支约束；需同步 FK XYZ hold 指标、耦合模型/局部有效域，不能宣称世界平移保持或轴向完全解耦 |
| 8. IK/FK/Shaper/Driver 是否要改 | CONFIRMED（接口需求） | 输出 pose 的新控制器无需为接入修改这些底层算法；历史输入/状态记录可先在新旁路 recorder 准备。若要求当前未传递的力 ADC/真实逐帧 RX，则需另行评估独立版本的数据路径变更 |
| 9. 通信时序是否有已知阻碍 | PARTIAL | 可开始静态/离线研究；固定 20 ms 力级 TDE 假设未满足，有长尾、丢帧/持值和 USB/freshness 事件，需要明确定义 delay 与 validity；不建议先全面重构或新增运动 gate |
| 10. 日志/runner 是否支持论文 | PARTIAL | 实验生命周期和基础统计可用；尚缺统一原始全窗口 schema、TDE 输出、自适应状态、同步、已知 payload 与同源扰动及重复实验 |

理论与算法迁移的必要区分：用户描述的旧论文、方程、质量矩阵/坐标定义、TDE delay 约束、tanh 边界层和 σ-modification 自适应律未在本轮提供。其假设与稳定性证明为 UNKNOWN，不臆造数值或宣称“移植成功”。运动级候选模型需说明 u 是 residual 角、目标角速度还是经级联积分的参考；力级表达式不能通过改单位就输出角度。

最小缓存建议（尚未实现）：按 host monotonic 维护有限时长环形记录，包含 source/rx/tick/query/TX 时间、device/frame/cycle sequence、SO(3) 姿态和对应坐标 gyro、估计器状态/有效性、FF/target/reference、raw/clipped residual、shaped reference、final TX steps、驱动 actual steps。TDE 使用共同有效时刻的历史输入与状态；重复持值、缺帧、stop/restart/跨 epoch 均显式标记，延迟查询不偷偷补未来值。σ-modification 的参数初值/界/泄漏项、tanh 尺度与饱和后的自适应策略由后续理论定义，当前 MISSING。

完整 6PUS 处理：不要把六支链当六个独立直线 force actuator，不能直接复用通用 Stewart 6UPS 的 Jacobian。保持当前 P 副方向、U/S、几何分支、HOME 和 SI 约定；在完整 6DOF IK 中生成全部 q，记录真实 FK 平移。偏心负载、Base 转动与平动引起的动惯性/耦合应进入候选有效动态和实验解释；现有三 SISO LADRC 只是已有工程实现，不证明新 TDE 可忽略这些项。

# 9. 已有、部分已有、未知和缺失清单

本表是供主清单合并的 NUC-ROS2 / BOTH 检查项名称。没有原主清单时不分配 R01～R59 编号。状态按具体用途写明；每一行都有后续动作，不将未知厂商细节标为 CONFIRMED。

| 模块 | 检查项 | 当前状态 | 证据位置 | 缺失内容 | 下一动作 |
| --- | --- | --- | --- | --- | --- |
| 版本 | 当前 main 与冻结版本边界 | CONFIRMED | §1.1；E01/E02/E04 | 实际未来启动路径仍需运行前确认 | 保留两个版本标识 |
| 版本 | 已 source workspace 与安装可追溯性 | PARTIAL | §1.1；egg-link/build 路径 | 其他终端、H30 二进制等价性 | 后续启动前确认，无需本轮编译 |
| 架构 | Cartesian→IK→Shaper→六轴 Servo 链 | CONFIRMED | E03/E07～E12 | 新 TDE 尚未接入 | 保留底层，定义新外环 |
| 架构 | 统一姿态控制器接口 | PARTIAL | E05/E06/E08 | 姿态算法未共用 ABC，状态输入不统一 | 新阶段定义小型状态/输出契约 |
| 架构 | LADRC residual 接入参考 | CONFIRMED | E04/E05/E06 | 非理论等价性证明 | 参考控制权、合成和诊断 |
| 架构 | 独立 TDE runner/选择模式 | MISSING | E04 selector；bone_ws/src 定向检索 | 无 TDE mode/实现 | 新阶段再实现，本轮不改 |
| 前馈 | Base SO(3) 逆旋转前馈 | CONFIRMED | E06/E19 | 精密绝对坐标外参 | 使用既有相对参考约定 |
| 坐标 | SO(3) 世界系误差及 residual 合成 | CONFIRMED | E06；§3.2 | 新模型的误差导数关系 | 明确 world/body 与符号 |
| 驱动 | 实际位置 Servo mode=1 / 0x29 | CONFIRMED | E09/E10/E15；E26/E27 配置 | 当前通电寄存器瞬时状态未查询 | 使用已有记录，不在线探测 |
| 驱动 | 当前任务空间广义力输入 | MISSING | E03/E09/E10/E13 | 无 wrench 控制/分配/资格路径 | 力级直接迁移 BLOCKED |
| 驱动 | 厂商内部位置跟随功能 | CONFIRMED | E15 §3.5.4 | 环路具体结构/动态 | 先确认资料，再定义等效模型 |
| 驱动 | 内部速度/电流环模型 | UNKNOWN | E15/E16 | 频率、增益、锁存、带宽、摩擦 | 厂商说明/后续局部辨识需求 |
| 反馈 | 六轴驱动 actual position | CONFIRMED | E11/E12；E26/E27 变化的 feedback_steps | 外部位置校准、精确设备采样时刻 | 保留驱动测量与外部精度区别 |
| 反馈 | 原始步数与 P 副几何坐标映射 | CONFIRMED | E11/E12/E14 | 精密零偏标定 | 记录伸出量与几何 q 的偏置 |
| 反馈 | 电流经 Child→Timed 发布 | CONFIRMED | E09/E11/E12 | current 的绝对标定/力矩转换 | 从 timed 原始消息旁路记录 |
| 反馈 | 最新 controller CSV 电流/fault 真实性 | PARTIAL | E12/E26/E27；§4.2 | 旧 raw topic 未转发，零值无 validity | 新 recorder 直接订阅 timed；原 CSV 不改 |
| 反馈 | 原始力与 ADC 协议能力 | PARTIAL | E11/E15/E16 | 当前 Child/Bridge 不传、六轴标定不足 | 优先保留 raw 语义，后续单独评估路径 |
| 反馈 | 校准六轴力/任务空间 wrench/torque | MISSING | E09/E12/E13；§4.2 | 零点、标定、映射、完整运行序列 | 不用原始 g/电流替代广义力 |
| 反馈 | 实测 P 副速度/加速度 | MISSING | E12/E13 | 消息 velocity 不填、无在线估计 | 对既有位置做噪声/因果性准备 |
| IMU | Base/Platform quaternion 测量 | CONFIRMED | E17/E19/E26/E27 | 外部绝对 accuracy | 作为相对惯性姿态测量 |
| IMU | gyro/linear accel 原始获取 | PARTIAL | E17/E20/E30 | 最新 J14 runner 不保存原消息 | 设计原始双 IMU recorder schema |
| IMU | 当前 Base 与 Platform 机械轴映射 | CONFIRMED（既有姿态用途） | E06/E19 | 安装变化后、精密连续标定 | 不照抄 J9 Base 未知状态 |
| IMU | 平台/基座 IMU 杆臂和中心加速度 | PARTIAL | E19/E20/E14 pending | 可靠杆臂与中心重建验证 | 只记录后续标定需求 |
| IMU | 设备时间与硬件同步 | MISSING | E17 parser/publish；E19/E20 | source timestamp、sensor seq、latch sync | 使用可追溯 host 时间并标限制 |
| 状态 | 因果平台角速度输入 | PARTIAL | E17；E05/E06 callbacks | 转换后的 gyro 未纳入现行接口 | 新状态接口接入有效 gyro |
| 状态 | 因果角加速度估计器 | MISSING | E20 central gradient；§5.3 | 在线滤波/差分/质量标准 | 既有原始数据离线设计 |
| 状态 | 离线角加速度分析 | CONFIRMED（离线用途） | E20 | 因果实时资格 | 仅作敏感性/噪声参考 |
| 状态 | Fixed-Lag q 历史和 FK | CONFIRMED（观测用途） | E18/E21/E30 | 不提供当前瞬时完整状态 | 记录 query 时刻与 lag |
| 状态 | J14 对照全窗口 FK/XYZ hold error | MISSING | E26/E27 CSV；fk_node_host.json | 节点启动但无对应 CSV 字段 | 新旁路 recorder 同步保存 FK |
| 状态 | 统一 Base+Platform+FK 动态估计器 | MISSING | E05/E18/E19；当前路径检索 | 无合格融合估计链 | 先明确是否研究必需，避免全面融合 |
| 时序 | controller 实际 dt | CONFIRMED（所查窗口） | E05/E26/E27；§6.2 | 不保证所有运行 20 ms | 用真实 dt，保留失配 |
| 时序 | UART/Bridge 实际周期分布 | CONFIRMED（已有记录范围） | E23～E25 | 长期所有工况保证 | 按 H 范围报告，不外推 |
| 时序 | 逐轴物理采样或独立帧 RX 时刻 | PARTIAL | E11/E12；§6.1 | ACTIVE 同批 now 代理，非硬件时刻 | 标语义，后续评估时间代理误差 |
| 时序 | sensor/filter/actuator 总延迟模型 | UNKNOWN | E15/E17/E18；§6 | 内部滤波、锁存、执行滞后 | 先从已有序列评估可辨识性 |
| 时序 | USB/freshness 已知中断处理 | PARTIAL | E29 | 物理根因、完整 fault tick、恢复未 qualified | 保留事件，不盲加 gate/放宽阈值 |
| TDE | 历史 residual 命令 | CONFIRMED（命令用途） | E05/E26/E27 | 不等于 physical applied input | 重命名研究语义，关联 TX |
| TDE | 最终 TX/actual position 历史基础 | PARTIAL | E09/E13/E23～E27 | 最新 production 只保留 300 cycles；接受时刻未知 | 全窗口记录 timed command/feedback |
| TDE | 历史输入与状态有效时间对齐 | PARTIAL | E13/E18/E25/E27 | 统一缓存、raw IMU、跨层 seq 不完整 | 离线重建缺口，不用下一行代替 delay |
| TDE | 有效运动级内环/外环对象模型 | MISSING | E05/E07～E10；§8 | gain、order、delay、耦合有效域 | 优先定义/辨识候选局部对象 |
| TDE | 原论文假设/量纲/稳定性映射 | UNKNOWN | 本轮未提供论文；§8 | 方程、广义力定义、TDE 条件 | 获取原论文后逐式核对 |
| TDE | 力级原律直接复用 | BLOCKED | E09/E10；§4/§8 | 输入类型不匹配、无已知 wrench | 转运动级重推导或独立级联路线 |
| TDE | 运动级 residual 方案准备 | PREPARABLE | E05/E06/E23～E30 | 准备不等于算法成立 | 只做接口/时序/既有数据准备 |
| 自适应 | tanh + σ-modification 实现/状态 | MISSING | bone_ws/src 定向检索；§8 | adaptive law、初始化、泄漏、尺度、诊断 | 待理论确定后独立实现 |
| 模型 | 完整 6PUS 耦合几何 | PARTIAL | E07/E14/E21 | 精密标定、完整 coupled workspace | 保留完整六轴 IK 与局部域 |
| 模型 | 移动基座惯性项与偏心负载参数 | PARTIAL | E19/E20/E22 | mass/CoM/inertia、基座平移/角加速度 | 建立待补参数与可测变量清单 |
| 控制资产 | 当前 P/PD 基线 | CONFIRMED | E06/E14/E26 | 非完整 PID；不是泛化最优参数 | 固定基线用于未来比较 |
| 控制资产 | 三轴一阶 LADRC 基线 | CONFIRMED | E05/E14/E22/E25/E27 | z2 非 torque、无理论泛化证明 | 保留 frozen 算法和参数 |
| 实验 | 单轴/多轴 runner 和 HOME 生命周期 | CONFIRMED（已有模式） | E04/E21/E22 | 三算法共享选择/记录未统一 | 新研究 runner 最小适配 |
| 实验 | 偏心负载已有实机记录 | PARTIAL | E22 | 精密 payload 参数、同源激励/重复统计 | 定义后续采集需求，不执行 |
| 实验 | PD/LADRC/TDE 公平比较数据 | MISSING | E22/E28 | 无 TDE、人工波形不一致 | 设计同源条件和重复方案 |
| 日志 | controller 全窗口 CSV 选项 | CONFIRMED（emitted rows） | E04/E26/E27 | fault 前返回、queue error 仍需检查 | 使用现有记录功能并保留异常 |
| 日志 | 同步原始 IMU/FK/TX/current 全窗口 | PARTIAL | E20/E23～E27 | 最新对照未统一、原始 IMU 序列缺失 | 新阶段旁路记录规格 |
| 分析 | 现有统计/绘图用于三算法 | PARTIAL | E28；J12 analyzers | identity/TDE/adaptation/time-weight 指标 | 复用核心，独立适配 |
| 安全 | 限幅/Fault/STOP/一发 Pause | CONFIRMED（实现存在） | E09～E12/E29 | 断连时 ACK/实际停止不可保证 | 保留既有机制，报告真实 ACK |
| 主清单 | R01～R59 编号合并 | UNKNOWN | §1 文件名检索 | 未取得主清单及 owner 分类 | 按本表检查项名称日后映射 |

# 10. 阻塞项与下一步最小工作

## 10.1 最优先的三个问题

1. **先确定输入级别和有效对象。** 对照原论文，把广义力律与当前 residual-angle→位置 Servo 区分；选一阶运动级重推导、二阶运动级方案或有明确内环模型的级联结构。定义模型阶次、坐标、量纲、名义 gain、shaper/slew/执行延迟和 6PUS 耦合有效域。在此之前，力级控制律直接执行 BLOCKED；不改 Driver/IK 来迁就公式。
2. **补齐可追溯历史与因果状态的准备规格。** 利用既有数据核查 clock/seq/validity 和可对齐窗口；定义原始双 IMU gyro/姿态接收历史、FK query、final TX、actual steps、current/fault 的全窗口旁路记录，明确默认零/旧缓存和 batch ingest 时间。评估因果差分/滤波噪声与延迟，不能把 u_actual 名称、连续 CSV 或 offline gradient 当成立条件。
3. **建立可解释的比较条件与时序适用范围。** 保留现有 PD/LADRC 参数与共同 runtime，明确 payload mass/CoM、扰动一致性、窗口与重复条件；同时量化已有长尾/USB/freshness 对候选 TDE delay 的影响，规定数据缺失怎样记录/处理。先准备方案与既有数据离线检查，再在未来授权下采集，不能复用非同源历史直接排名。

## 10.2 可立即开始的最小准备与尚不可执行的步骤

可以准备：原论文变量到当前量的映射表；新阶段状态/输出/历史记录 schema；既有 CSV 字段可信度与时间轴核查；纯离线因果估计候选和有效对象可辨识性评估；公平三组 protocol。均为 PREPARABLE，后续需另行任务授权，本轮只完成审计。

尚不能据本审计执行：力级 TDE 实机接管、直接沿用旧稳定性证明、宣称已有加速度/力矩完整状态、以当前 controller 零电流列判断实际电流、将不同人工扰动的 PD/LADRC 历史作为严格 benchmark。新算法代码、Mock/仿真、参数、硬件验证属于后续阶段，不在本轮产生。

**停止点：本轮完成审计与报告后停止。现有 ROS2 代码、冻结资产、参数、日志与未提交实验文件未修改；没有新实机运行，没有 commit/push。**
