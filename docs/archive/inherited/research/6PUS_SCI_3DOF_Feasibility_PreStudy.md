> 历史归档：原路径 `agent_docs/research/6PUS_SCI_3DOF_Feasibility_PreStudy.md`。正文保留当时的方案、参数及证据语境；当前原生 Mode 3 入口以[项目 README](../../../../README.md)为准。历史命令不作为当前推荐启动命令。

# 6PUS SCI 3DOF Feasibility Pre-Study

审计日期：2026-10-16（Asia/Shanghai）。对象：当前 NUC `/home/lyapunov/bone_drill`，正式工作空间 `/home/lyapunov/bone_drill/bone_ws`。

**结论：B——3DOF 可以成立，但需要补充明确条件。** 可以将第一篇论文的任务限定为世界参考下三轴姿态保持，释放平台相对基座的三个平移变量；不必为此扩大到完整 6DOF 基座运动或 XYZ 闭环。当前代码及历史硬件记录支持这一工程起点，本次离线计算也确认构型会影响运动约束下的补偿需求/行程可行性。但是：**三个平移变量作用不同；实际最大速度、加速度和完整伺服可实现边界仍 UNKNOWN；在线重构的净收益及论文创新尚未建立。** [CODE][RESULT][INFERENCE][UNKNOWN]

## 1. Audit Scope

本次先读取 Windows 侧 Round1/Round2 报告，再审计 NUC 当前源码、YAML、安装链接、历史 J10/J11/J13/J14 原始记录与现有审计。没有重新开展文献综述或联网检索；文献判断仅承接输入报告的阅读等级和未取得正文的限制，不宣称本次独立复核其全部论文。没有设计新控制器或最终实验。[LITERATURE][DOC]

只新增本报告。非侵入式 Python 计算仅导入纯运动学/数学函数，显式读取当前 YAML；禁止写入 Python 字节码。没有启动 ROS 节点、打开串口、查询电机、发布目标、编译、安装依赖、改源码/配置/结果、commit、push 或移动 tag。**本次 `hardware_write_count = 0`：指审计操作的执行器写入数为零。** 历史 runner 的同名计数往往只是 Cartesian 目标发布次数，另见 §5。[CODE]

| 项目 | 本次观测 | 边界 |
|---|---|---|
| Git | `main`；HEAD `117384dc37248c30ce0ec4b9c36f1754acba9b9f`；2026-10-11 提交 | 当前源码快照；不等于所有数据均已提交 |
| 工作树 | 开始时没有 tracked 修改；已有 audits、输入资料、blog_materials、多个结果目录等 untracked 资产 | 保留全部既有资产；本报告也不提交 |
| J14 历史冻结 tag | `v1.0-j14-a-three-process-runtime-reliability-qualified` 解析到 `ce48ee97f5c827a12591d43a82d32ce97fd4689d` | 历史资格来源；不替代当前路径核查 |
| 本审计 shell | `ROS_DISTRO=jazzy`；`AMENT_PREFIX_PATH=/opt/ros/jazzy` | 此 shell 未显示 bone_ws overlay；不推断其他终端 |
| Demo 入口 | shell source Jazzy 和 bone_ws/install；launcher 选择 J11/J13 YAML 与 production launch | 此次只读，未执行入口 |
| Python 安装 | kinematics 的 install egg-link → build → src 链接 | 核对到当前运动学源码；不是全部安装包认证 |
| H30 安装 | install 可执行入口 → build/h30_imu/h30_imu_node | 已构建二进制；未重新验证与 C++ 源码完全一致 |
| 进程快照 | 排除本次检索进程后，未见匹配的 J11/J13/J14、H30、FK/IK 控制进程 | 不据此判断电源、物理位置或其他全部资源状态 |

## 2. Sources and Evidence Classification

证据标签：`[CODE]` 当前实现；`[CONFIG]` 配置；`[HARDWARE]` 既有硬件记录支持的限定事实；`[RESULT]` 历史结果；`[DOC]` 文档描述；`[LITERATURE]` 两轮输入文献报告；`[INFERENCE]` 本次数学推导/离线模型计算；`[UNKNOWN]` 证据不足。**本次新增数值一律不是硬件实验。**

执行器约束另用四类：`CONFIRMED_HARDWARE`（只限已记录运行点/行为）、`CONFIGURED_UNVERIFIED`（软件存在，不证明物理极限）、`THEORETICAL_ONLY`（模型/型号值）、`UNKNOWN`（未可靠确定）。同一物理量可同时具有“某运行点已测”和“极限未知”两种状态，不能混写。

以下编号是本报告的复核入口。链接指向本机文件；ZIP 内成员名只作历史数学资料定位，不据其推断当前运行状态。

| ID | 权威入口及本次用途 |
|---|---|
| L1 | [Round1](/home/lyapunov/bone_drill/agent_docs/research/6PUS_SCI_PreStudy_Input/6PUS_SCI_Literature_Review_Round1.md)：姿态任务冗余、位置伺服与力接口边界、直接冲突文献 |
| L2 | [Round2](/home/lyapunov/bone_drill/agent_docs/research/6PUS_SCI_PreStudy_Input/6PUS_SCI_Literature_Review_Round2.md)：§11 瞬时等价/平行导轨不变性，§13 持续可行性生存条件 |
| C1 | [analytical_6pus.py](/home/lyapunov/bone_drill/bone_ws/src/pus6_kinematics/pus6_kinematics/analytical_6pus.py:53)：几何、IK、differential IK、数值 FK |
| C2 | [6pus_geometry.yaml](/home/lyapunov/bone_drill/bone_ws/src/pus6_kinematics/config/6pus_geometry.yaml)、[geometry_config.py](/home/lyapunov/bone_drill/bone_ws/src/pus6_kinematics/pus6_kinematics/geometry_config.py)：锚点、行程、换算和 pending 状态 |
| C3 | [kinematics_node.py](/home/lyapunov/bone_drill/bone_ws/src/pus6_kinematics/pus6_kinematics/kinematics_node.py:43)：Cartesian 输入、HOME 差分 IK、包络检查 |
| C4 | [joint_trajectory_controller.py](/home/lyapunov/bone_drill/bone_ws/src/pus6_control/pus6_control/joint_trajectory_controller.py:89)、[control_node.py](/home/lyapunov/bone_drill/bone_ws/src/pus6_control/pus6_control/control_node.py:34)：位置参考整形、dq/ddq 状态、raw 映射 |
| C5 | [fk_node.py](/home/lyapunov/bone_drill/bone_ws/src/pus6_kinematics/pus6_kinematics/fk_node.py:67)、[state_synchronizer.py](/home/lyapunov/bone_drill/bone_ws/src/pus6_kinematics/pus6_kinematics/state_synchronizer.py)：有固定滞后的反馈 FK |
| C6 | [J11 M2 runner](/home/lyapunov/bone_drill/bone_ws/src/h30_imu/scripts/j11_m2_pid_residual_runner.py:198)、[J11 M1 runner](/home/lyapunov/bone_drill/bone_ws/src/h30_imu/scripts/j11_base_feedforward_runner.py)：实际误差、合成及姿态输出 |
| C7 | [J13 runner](/home/lyapunov/bone_drill/bone_ws/src/h30_imu/scripts/j13_ladrc_shadow_runner.py:532)、[LADRC](/home/lyapunov/bone_drill/bone_ws/src/h30_imu/scripts/j13_ladrc.py)：选用残差、模式、日志与限幅 |
| C8 | [J11 YAML](/home/lyapunov/bone_drill/bone_ws/src/h30_imu/config/j11_m2_pid_params.yaml)、[J13 YAML](/home/lyapunov/bone_drill/bone_ws/src/h30_imu/config/j13_ladrc_params.yaml)、[system_params.yaml](/home/lyapunov/bone_drill/bone_ws/src/pus6_bringup/config/system_params.yaml)：配置覆盖与限速 |
| C9 | [production_driver.py](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/production_driver.py)、[Child](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/j14_process_uart.py:259)：三进程与最终下发 |
| C10 | [六轴 Servo transport](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/j7_six_axis_servo_transport.py:252)、[safety.py](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/safety.py:38)：mode=1、位置寄存器、边界与电流检查 |
| C11 | [Bridge](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/j14_feedback_bridge.py:180)、[compatibility relay](/home/lyapunov/bone_drill/bone_ws/scripts/j14_a2_q3_feedback_relay.py:77)：坐标偏置、旧话题缺口 |
| C12 | [TimedJointFeedback](/home/lyapunov/bone_drill/bone_ws/src/pus6_msgs/msg/TimedJointFeedback.msg)、[TimedJointCommand](/home/lyapunov/bone_drill/bone_ws/src/pus6_msgs/msg/TimedJointCommand.msg)、[feedback scheduler](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/j7_servo_feedback_scheduler.py:362)：时间字段真实来源 |
| C13 | [H30 node](/home/lyapunov/bone_drill/bone_ws/src/h30_imu/src/h30_imu_node.cpp:84)、[parser](/home/lyapunov/bone_drill/bone_ws/src/h30_imu/src/h30_protocol.cpp)、[Sample](/home/lyapunov/bone_drill/bone_ws/src/h30_imu/include/h30_imu/h30_protocol.hpp)：姿态、gyro、host stamp、tid |
| C14 | [Demo launcher](/home/lyapunov/bone_drill/bone_ws/scripts/j14_demo_launcher.py)、[controller/recorder](/home/lyapunov/bone_drill/bone_ws/scripts/j14_demo_controller.py:61)、[launch](/home/lyapunov/bone_drill/bone_ws/src/pus6_bringup/launch/pus6_system.launch.py)：当前运行及记录入口 |
| D1 | [J11 总结](/home/lyapunov/bone_drill/agent_docs/stages/J11/J11_FINAL_SUMMARY.md)、[J13 总结](/home/lyapunov/bone_drill/agent_docs/stages/J13/J13_FINAL_SUMMARY.md)、[J14 交接](/home/lyapunov/bone_drill/agent_docs/stages/J14/J14_FINAL_HANDOFF.md)：历史能力与边界 |
| D2 | [既有 NUC 迁移审计](/home/lyapunov/bone_drill/agent_docs/audits/stewart_migration_nuc_audit.md)：导航与交叉核对；重要接口结论本次再查源码 |
| D3 | [J10 Z 范围报告](/home/lyapunov/bone_drill/agent_docs/reports/j10/J10_A12_Z_COMPLETE_OPERATIONAL_CHARACTERIZATION_20260909.md)、[HOME 附近性能口径](/home/lyapunov/bone_drill/agent_docs/reports/REAL_HARDWARE_PERFORMANCE_SUMMARY.md)：端点未标定与内部 FK 非外部真值 |
| H1 | [J11 M2 原始 CSV](/home/lyapunov/bone_drill/results/j11/m2_pid/20260920_095850/j11_m2_7a_pitchd_cand2_repeat_phaseB_dynamic_telemetry_powered.csv)、[summary](/home/lyapunov/bone_drill/results/j11/m2_pid/20260920_095850/j11_m2_7a_pitchd_cand2_repeat_phaseB_dynamic_summary_powered.json) |
| H2 | [J13 三轴空载 CSV](/home/lyapunov/bone_drill/results/j13/j13_2c1_3axis_active/20261007_141708/j13_2c1_3axis_active_10s/j13_2c1_3axis_active_10s_telemetry_powered.csv) |
| H3 | [J13 三轴偏心负载 CSV](/home/lyapunov/bone_drill/results/j13/j13_2c1_3axis_active/20261007_143036/j13_3axis_payload_ladrc_10s/j13_3axis_payload_ladrc_10s_telemetry_powered.csv)、[summary](/home/lyapunov/bone_drill/results/j13/j13_2c1_3axis_active/20261007_143036/j13_3axis_payload_ladrc_10s/j13_3axis_payload_ladrc_10s_summary_powered.json) |
| H4 | [J14 Classical 原始 CSV](/home/lyapunov/bone_drill/results/j14/j14_a2_q3_r1d_classical_60s/j14_a2_q3_r1_classical_telemetry_powered.csv)、[raw Child cycles](/home/lyapunov/bone_drill/results/j14/j14_a2_q3_r1d_classical_60s/child_cycles.jsonl)、[实际运行配置](/home/lyapunov/bone_drill/results/j14/j14_a2_q3_r1d_classical_60s/production_configuration.json) |
| H5 | [J14 LADRC 原始 CSV](/home/lyapunov/bone_drill/results/j14/j14_a2_q3_r2_ladrc_60s/j14_a2_q3_r1_ladrc_telemetry_powered.csv)、[raw Child cycles](/home/lyapunov/bone_drill/results/j14/j14_a2_q3_r2_ladrc_60s/child_cycles.jsonl) |
| H6 | [600 s 静态 metadata](/home/lyapunov/bone_drill/results/j14/j14_a2_q2_r3_static_600s/run_metadata.json)、[raw Child cycles](/home/lyapunov/bone_drill/results/j14/j14_a2_q2_r3_static_600s/child_cycles.jsonl)、[analysis](/home/lyapunov/bone_drill/results/j14/j14_a2_q2_r3_static_600s/q2_detailed_analysis.json) |
| H7 | [Z−20 mm metadata](/home/lyapunov/bone_drill/results/J10_A12/J10_workspace_z_neg200mm_20260909_214929_metadata.json)、[CSV](/home/lyapunov/bone_drill/results/J10_A12/J10_workspace_z_neg200mm_20260909_214929.csv)、[Z+24.5 mm metadata](/home/lyapunov/bone_drill/results/J10_A12/J10_workspace_z_pos245mm_20260909_215903_metadata.json)、[CSV](/home/lyapunov/bone_drill/results/J10_A12/J10_workspace_z_pos245mm_20260909_215903.csv) |
| M1 | [本机 Kinematics.zip](/home/lyapunov/bone_drill/hardware/调试工具/Kinematics.zip)：只读读取 `params.m`、`inver.m`、`diff_q6.m`、`forward_kinematics.m`、`joint_constraint_params.m`、`check_joint_constraints.m`；补充历史数学定义，不替代 C1/C2 |

## 3. Current 6PUS Kinematics Capability

### 3.1 几何、坐标与支链

C1/C2 当前模型：基座分布半径 40 mm，平台半径 27.5 mm，固定 U–S 杆长 84 mm，λ=30°、μ=60°；HOME 标称高度 `z0=132.61226796206452 mm`。YAML 明确标记 `6pus_preassembly_software_reference`、`pending_final_assembly_confirmation`，锚点、杆长、零映射及外参仍含 pending/null。后续已有真实机构运行，故这些标签**不能证明当前机构尚未装配**；但它们也不是最终几何计量/标定证书。[CONFIG][HARDWARE][UNKNOWN]

B 是运动学基座坐标：锚点平面 z=0、导轨均沿 +Z；P 是平台局部坐标，`b_i` 位于其 z=0 平面，平台原点在 B 中为 p。使用右手旋转矩阵 `R_BP=Rz(yaw)Ry(pitch)Rx(roll)`，SI m/rad。软件姿态任务零位是 HOME，**并非把运动学绝对 z 或 q 设成零**。W 在现行 runner 中是启动静止捕获后形成的参考系；并非已独立标定的地理/计量世界系。[CODE] C1/C6

| ID/支链 | 导轨固定 (a_x,a_y) mm | P 系连接点 (b_x,b_y) mm | 连接 |
|---:|---:|---:|---|
| 1 | (−10.352762, 38.637033) | (−13.75, 23.815699) | P1→U1→固定杆1→S1 |
| 2 | (−38.637033, −10.352762) | (−27.5, 0) | P2→U2→固定杆2→S2 |
| 3 | (−28.284271, −28.284271) | (−13.75, −23.815699) | P3→U3→固定杆3→S3 |
| 4 | (28.284271, −28.284271) | (13.75, −23.815699) | P4→U4→固定杆4→S4 |
| 5 | (38.637033, −10.352762) | (27.5, 0) | P5→U5→固定杆5→S5 |
| 6 | (10.352762, 38.637033) | (13.75, 23.815699) | P6→U6→固定杆6→S6 |

顺序来自当前 YAML，数组按 ID1…ID6 映射。它是实现约定；本次没有物理追线复核 ID/装配几何。[CONFIG]

### 3.2 可调用实现与缺项

| 功能 | 类/函数；文件 | 输入 → 输出；数学意义 |
|---|---|---|
| IK | `Analytical6PusKinematics.inverse_kinematics`；C1:129 | `[x,y,z,roll,pitch,yaw]` → 六个几何滑块坐标 q [m]；固定负根装配分支 |
| HOME 差分 IK | `differential_inverse_kinematics`；C1:184 | desired/reference pose → `IK(des)−IK(ref)` [m]；不是时间微分或速度逆解 |
| 差分整步 | `differential_inverse_kinematics_steps`；C1:194 | 同上 → round(Δq×40000)；供目标预测/派发参考 |
| FK | `forward_kinematics_with_metrics`；C1:239 | 六个几何 q、可选初值 → `ForwardKinematicsResult`：pose、位置、R、收敛、迭代数、残差 |
| FK 简化接口 | `forward_kinematics`；C1:357 | 同上 → 六维 pose；不收敛抛错 |
| FK 方法 | C1 循环 | 固定杆长约束的阻尼 Gauss–Newton/LM 数值迭代，变量是位置和 Euler 参数；不是解析 FK，也不是全局唯一解证明 |
| 当前 Jacobian | C1 FK 循环内 `J`、`_drot_dalpha/dbeta/dgamma` | `∂(||p+Rb_i−slider_i||−L)/∂[p,Euler]`，只在求解器内部使用 |
| ROS 可调用任务/速度 Jacobian | **NOT FOUND**：所查当前 `bone_ws/src`、`bone_ws/scripts` 的运动学/控制实现 | 没有导出 `J_R(q)` 或 `qdot=G[v,ω]` 的现成公共接口；§8 由当前闭链推导，不能冒充已有代码 |
| 历史 MATLAB Jacobian | M1 `diff_q6.m` | 符号计算 `∂(IK(p,Euler)−q)/∂[p,Euler]`，`∂E/∂q=−I`；不是 ROS 在线 Jacobian。`forward_kinematics.m` 为旧数值 FK |
| 动力学/加速度 Jacobian | **NOT FOUND**：所查当前执行控制路径 | 未见可直接用于真实力/力矩分配的完整惯性、摩擦与封装伺服模型 |

IK 实际检查高度范围和根号可达性，但**没有使用已加载的 min/max stroke 对每个 q 执行限幅/拒绝**。因此 `inverse_kinematics()` 返回成功不等于行程、被动关节和碰撞全部可行。现行路径另在驱动层检查 raw 范围。FK 注释中的 “SE(3)” 不改变其实际 Euler 参数迭代方式，内部残差 J 也不能直接当作空间角速度 Jacobian。[CODE] C1/C3/C9/C10

### 3.3 q、raw 与速度/加速度变量

本报告固定记号：`n_i` 为设备 raw step；`q_i` 为几何 P 副/U 中心坐标；`r_i` 为相对 HOME 位移。当前链路满足：[CODE][CONFIG] C2/C4/C11

\[
r_i=(n_i-1000)/40000,\quad
q_i=0.050+r_i=0.025+n_i/40000\quad[\mathrm m].
\]

`1 mm=40 step`，`1 step=0.025 mm`。HOME：`n=1000`、`r=0`、`q=50 mm`；PARK：`n=0`、`r=−25 mm`、`q=25 mm`。**不要直接将 n/40000 当作 FK 的 q**：Bridge 的普通 JointState.position 是 raw×25 μm，而 TimedJointFeedback.positions_meters 特意加了几何偏置；TimedJointCommand.target_positions_meters 则仍是 raw×25 μm。做输入/反馈比较宜优先统一 raw，再明确转换。[CODE] C11/C12

Shaper 内有 q/dq/ddq/dddq，`get_motion_state()` 可返回参考位置、速度、加速度、jerk。它们是**命令参考状态**；`JointState.velocity/effort` 在所查驱动路径为空。实测 qdot/qddot 传感通道 **NOT FOUND**；可对有时间/有效性信息的位置反馈估计，但量化、持值、异步和滤波延迟必须另评估。[CODE][INFERENCE][UNKNOWN]

## 4. Current Actuator Constraints

### 4.1 位置、速度、加速度：不得混为“硬件最大值”

| 量 | 当前证据与数值 | 分类及适用边界 |
|---|---|---|
| raw 指令绝对范围 | `0≤n_i≤2000`；映射几何 `25≤q_i≤75 mm`，相对 HOME ±25 mm；C2/C9/C10 | **CONFIGURED_UNVERIFIED**：作为代码边界确认；不是六轴机械端点的计量认证 |
| 普通模式范围 | `200…1800 step` → q=30…70 mm；YAML 另有 software command 0…1800 | **CONFIGURED_UNVERIFIED**：普通/旧策略。不能假定所有路径都使用此范围 |
| J11/J13/J14 实际选用策略 | J11/J13 YAML `maintenance_mode=true`；H4 实际 Child config 也为 true；C9 `validate_target_vector(limits=None)` 使用绝对 0…2000 | 配置与 H4 会话的实际运行路径已核对。`200…1800` **不是这些姿态 Demo 必然生效的硬界**；本报告不更改策略 |
| 可达到的历史运行点 | H7：六轴到 `[200]*6`（Z−20 mm）和 `[1980]*6`（Z+24.5 mm），均记录 motion_executed、到位、HOME 返回、CSV | **CONFIRMED_HARDWARE**：这些慢速单轴/共模运行点；并非 2000 端点或任意耦合姿态资格 |
| q_min/q_max 真正机械/连续工作边界 | YAML raw zero/stroke 仍 pending；D3 上边界为端点接近时主动停止，未试压机械限位 | **UNKNOWN**：不能以 `0…2000` 宣称全范围在负载/动态下均可使用 |
| qdot_max 实测连续能力 | 未见覆盖六轴、构型、负载与误差容限的极限辨识 | **UNKNOWN**；不从电机型号/丝杠/速度寄存器反推 |
| 参考速度上限 | C4/C8：`0.005 m/s =5 mm/s =200 step/s` | **CONFIGURED_UNVERIFIED**：Shaper 参考限值；运行该配置不等于实测最大速度为 5 mm/s |
| qddot_max 实测连续能力 | 未见对应实测能力边界 | **UNKNOWN** |
| 参考加速度、jerk | `0.050 m/s²=50 mm/s²`；`0.500 m/s³=500 mm/s³` | **CONFIGURED_UNVERIFIED**：参考整形限制；不是实测负载能力 |
| 几何/型号中的总行程 | M1 `params.m` 给 50 mm，当前配置采用相应映射 | 单独的名义模型值为 **THEORETICAL_ONLY**；局部硬件到位证据不能自动升级为完整机械标定 |

H7 的 FK 高度是内部模型推算，不能把“Z+24.5 mm 目标已运行”改写为“外部位移计确认精确移动 24.5 mm”。H7 还记录 `post_run_teardown_warning=true`；这里仅引用到位/返回证据，不将其扩成完整退出无异常结论。[RESULT][HARDWARE]

### 4.2 电机、通信与任务输出限制

| 量 | 实际实现/记录 | 分类与解释 |
|---|---|---|
| 执行模式 | C10 `configure_servo(speed=50)` 写 mode=1，后续六轴 `0x29` 目标位置流；H4/H5 为 active | **CONFIRMED_HARDWARE**：位置伺服执行路径已运行；不是力矩控制 |
| speed=50 | 当前 Servo 配置参数；启动有 ACK，无运行 readback（返回 readback=None） | **CONFIGURED_UNVERIFIED**：不能解释为 50 mm/s 或可靠连续极限 |
| 主机电流保护 | C10 `MAX_CURRENT_MA=1500`，scheduler 检查超限；非零 fault 停止 | **CONFIGURED_UNVERIFIED**：这是当前主机保护阈值；不等于驱动硬件额定/峰值电流或力能力 |
| YAML max_current_mA=500、mode=2 | motors/system/J11/J13 中存在；所查 Node 未找到消费 `max_current_mA` 的声明/读取，Servo transport 实际 mode=1 | **CONFIGURED_UNVERIFIED**：不能以这些旧字段覆盖实际 1500 mA validator 或真实 Servo 模式 |
| 实际电流观测 | H1/H2/H3 存在非零反馈；H3 六轴峰值分别 98/301/121/118/348/555 mA | **CONFIRMED_HARDWARE**：这些记录的观测值；不证明最大允许电流、力矩或负载边界 |
| 驱动内部电流限制/降额 | 所查当前 active 配置无完整六轴硬件寄存器限值证据 | **UNKNOWN**；与主机阈值分开 |
| Servo/目标名义周期 | controller、Shaper、Child 均名义 50 Hz/20 ms；H4/H5 原始 Child interval 见 §5 | **CONFIRMED_HARDWARE**：有接近 50 Hz 的特定历史窗口；不是硬实时或内环带宽 |
| 帧间隔 | C10 六轴 transport 默认 `0.0008 s` | **CONFIGURED_UNVERIFIED**：串行 TX 调度间隔，不是六轴同一采样时刻 |
| 每次发送位移 | Child slew：`allowance=min(20,max(5,int(200*dt)))`，每轴每次 ≤20 step | **CONFIGURED_UNVERIFIED**：命令防跳/限斜率机制；下限5、实际 dt 与量化意味着它并非精确物理 qdot_max |
| target freshness/reconnect | 0.20 s；80 step | **CONFIGURED_UNVERIFIED**：运行准入/恢复语义，不是速度能力 |
| Parent lease/反馈 liveness | 当前 Child lease 1000 ms；age_based 反馈检查亦有 1000 ms；controller 电机回调 age 1500 ms | **CONFIGURED_UNVERIFIED**：通信/主机时间；不等同 20 ms 误差 |
| UART 局部 timeout | J14 TX 30 ms、serial RX 5 ms；H4 child_ready 实际记录一致 | **CONFIRMED_HARDWARE** 仅限该配置曾在硬件路径运行；超时触发路径的完整 qualification 不能据此推定 |
| Base/Platform IMU age | runner 100 ms，使用 host callback freshness | **CONFIGURED_UNVERIFIED**：不等同设备采样 age |
| 姿态 residual | J11/J13 三轴 ±0.5° | **CONFIGURED_UNVERIFIED**：外环补偿角限幅；不是执行器行程/角速度界或最大可稳定基座角 |
| 被动 U/S 和碰撞 | M1 有 U/S 名义30°、保留3°→27°检查；当前 ROS IK 未集成相应完整检查 | 名义参数 **THEORETICAL_ONLY**；真实安装关节/碰撞边界 **UNKNOWN** |

**研究建模必须分别保留：几何/软件允许域、已测运行域、真实伺服可执行域。** 当前足够开展归一化和配置约束分析，尚不足以给出“真实机器人最大可补偿扰动”数值。[INFERENCE][UNKNOWN]

## 5. Current J11/J13 Attitude-Control Capability

### 5.1 实际闭环和输入输出

**三轴姿态闭环已经真实运行，但持续保持指定精度、约束冗余重构和新算法性能没有因此获得证明。** J11 M1 是 Base SO(3) 前馈，Platform 只观测；J11 M2 才含 Platform 姿态反馈。J13 文件名仍含 shadow，但 `3axis_active` 模式实际选择三轴 LADRC。须按运行模式核对，不能按文件名分类。[CODE][HARDWARE] C6/C7/H1/H3

现行链路：[CODE] C3/C4/C6–C14

```text
Base H30 → 启动参考/机械轴转换 → R_WB → R_BP,FF=R_WB^T
Platform H30 → 启动参考/机械轴转换 → R_WP → SO(3)误差 → 三轴残差角 Δφ_W
→ SO(3)合成 R_BP,cmd，平移固定 [0,0,z0]（B系）
→ /pus6/target_cartesian_pose (PoseStamped)
→ IK、相对HOME的六轴Δq → /pus6/desired_joint_deltas
→ Shaper → /pus6/command_joint_steps
→ Production Parent → UART Child final slew → 六轴0x29目标位置
→ 实际位置/电流/fault → Bridge → Timed feedback及兼容relay
→ Platform IMU返回姿态闭环；FK是带滞后的观测支路
```

Base 提供相对启动参考的姿态，形成逆旋转前馈；Platform 提供独立姿态观测，形成外环误差，不是把同一个 Base 预测值当成 Platform 测量。两个传感器的工程机械轴变换均使用 `Rz(+90°)`。独立标定的共同绝对航向、安装误差和长期漂移仍需限定。[CODE][UNKNOWN] C6/C7

实际世界系误差及残差合成是：

\[
e_R^W=\operatorname{Log}(R_{WP,d}R_{WP}^{T})^\vee,\quad R_{WP,d}=I,
\]
\[
\Delta R_W=\operatorname{Exp}([\Delta\phi_W]_\times),\quad
\Delta R_B=R_{WB}^{T}\Delta R_W R_{WB},\quad
R_{BP,cmd}=\Delta R_B R_{WB}^{T}.
\]

这是旋转向量误差及旋转乘法，不能替成 Euler 角逐项相减/相加。J11 retained 为 Roll=P、Pitch/Yaw=PD，Ki=0；现行 D 来自 SO(3)误差的 tick 差分和5 Hz低通，**不是实际 gyro 直接反馈**。J13 输出也为残差角 rad；`z2` 是等效角速度扰动估计，不能当物理力矩。[CODE][DOC] C6/C7/C8/D1

控制器输出既不是六轴力，也不是平台广义力。Shaper、整数化及最终 slew 会再次改变下发序列。`u_combined_actual` 表示“选用/限幅的外环残差”，不是实际机器人已施加的补偿量；J13 的并行 `pid_total_residual` 在三轴 active 时不是主执行输出。[CODE]

### 5.2 历史原始数据重新核对

下表按 `state==ACTIVE`，时间跨度为末条减首条 host timestamp；不是 nominal 总会话时长。[RESULT][HARDWARE]

| 证据 | 全部/ACTIVE 行 | ACTIVE跨度 s | ACTIVE tick间隔 median/max ms | 支持什么 |
|---|---:|---:|---:|---|
| H1 J11 M2 | 2999 / 2946 | 58.920119 | 20.000351 / 56.376481 | 三轴前馈+P/PD、六轴实际位置变化、非零电流 |
| H2 J13 空载三轴 | 500 / 430 | 8.580182 | 19.997658 / 28.781191 | 三轴 active 执行；“10 s会话”不是10 s ACTIVE |
| H3 J13 偏心三轴 | 500 / 446 | 8.901115 | 19.999843 / 21.100349 | `3axis_active`、非零反馈/电流；不是标准化负载计量 |
| H4 J14 Classical | 2999 / 2945 | 58.900160 | 20.000118 / 55.510637 | 当前运行基础上的姿态闭环已执行 |
| H5 J14 LADRC | 2997 / 2942 | 58.880015 | 20.000280 / 63.631712 | 三轴 LADRC 已执行；不是新论文性能证据 |

H1 summary 的 `hardware_write_count=2946`、H3 为446，与 ACTIVE 目标发布相符；C7 `_dispatch_cartesian_target()` 在 publish 后计数。其数学含义是控制器派发，不能换算成 UART ACK 数；真实六轴下发还须看 Child/transport/H4 metadata。[CODE][RESULT]

从原始 Child `start_ns` 再算：H4 3529 周期，间隔 median/max=20.000063/20.686566 ms；H5 3539 周期，20.000087/21.181148 ms；H6 静态600 s共29995周期，19.999729/68.541268 ms。**controller、Child、IMU和内置电机伺服是不同频率/窗口，不能互相替代。** H6 不是600 s动态姿态稳定证明。[RESULT][HARDWARE]

原始 Platform SO(3)误差 RMS/peak：H1=2.8917°/5.9700°；H3=1.7485°/4.2308°；H4=12.0339°/29.9079°；H5=16.0044°/38.5131°。输入和工况不同，不据此排名控制器。尤其 H4/H5 的 runtime PASS 不能解释成高精度姿态保持 PASS，也不能用其大误差直接定位为某一个算法缺陷；控制器、限速、输入及传感链的因果归属还需额外证据。[RESULT][UNKNOWN]

这些历史实验均**可作为参考证据**：证明工程链路和观测基础。不能直接作为新冗余方法的论文对比结果，人工摇动不能视为同一激励，历史未知偏心质量/力臂不能改写成已标定外载。[DOC][RESULT]

## 6. Sensor and State-Observation Capability

| 对象/量 | 现在已有 | 可通过软件获得 | 需要新增传感器/无法可靠获得 |
|---|---|---|---|
| Base姿态 | H30 quaternion，既有三轴相对参考记录 | 原消息、有效标记和接收历史可另记录 | 长期绝对航向/外部精度未验证；高精度绝对声明需独立参考 |
| Base angular velocity | C13解析gyro，deg/s→rad/s，消息有字段 | 可旁路保存并做机械轴/坐标变换 | 当前runner回调和所查主CSV没有原始gyro序列；实际每帧有效性/率未由这些CSV确定 |
| Platform姿态 | 独立H30与SO(3)误差日志 | 同上 | 用闭环同一IMU验证自身绝对精度不独立 |
| Platform angular velocity | 同一C13接口支持 | 可记录gyro；不是现行D项的同义词 | angular acceleration没有直接传感通道，因果估计质量UNKNOWN |
| 双IMU时间戳 | ROS header为节点publish时的`now()`；runner tick和callback为host时间 | 保存header、host接收、有效性可改善追溯 | parser `tid` 未发布为序号/设备时钟；真实采样时间、两IMU硬件同步 **NOT FOUND** |
| actuator command position | 上层command steps、TimedJointCommand.target_steps、Child记录 | 联合各层seq与时间可分开target/shaped/final TX | 上层commanded_steps不是UART ACK或实测运动 |
| feedback position | 六轴0x29状态，raw及几何q；历史非零运动记录 | FK及差分速度/加速度可离线获得 | 六轴并非同一物理瞬时采样；差分不直接代表可靠物理极限 |
| current/fault | Child validator和TimedJointFeedback有实际字段 | 应直接记录Timed消息及其有效性；所查旧CSV不能保证这些字段已接通 | 电流→力/力矩标定UNKNOWN；缺校准不能称外载力测量 |
| actuator timestamp | host TX start/done、cycle seq、反馈ingest时间 | 可统一host时钟并记录阶段差 | 电机内部采样/到位时间 **NOT FOUND**；见下方语义限制 |
| joint state/FK/IK | C1/C3/C5/C11现有 | 可复用带滞后FK和完整逆解 | FK为相对B的模型估计，不是世界位置外部真值 |
| task Jacobian | ROS公共函数 **NOT FOUND**；内部FK残差J和历史符号资料存在 | 本报告从当前闭链推得G/J_R，数值核查见§9 | 不需仅为Jacobian增加传感器；但参数误差需计量 |
| Base translation / world platform position | 双IMU不直接测绝对位置 | 只能在另有基座位置参考时，由p_WP=p_WB+R_WB p_BP构成 | 可靠世界XYZ需外部位置观测/有效机械约束；纯IMU积分不能长期替代 |

**已确认的日志断层：** C11兼容relay仅转发raw steps和JointState，不转发旧 `/pus6/joint_currents_raw`、`/pus6/joint_faults_raw`；C7 timed回调只保存消息，不更新这两个旧缓冲。H4/H5全部ACTIVE行的电流/fault列均为六个零。它们是当前路径下未获更新的默认缓冲，不能作为六轴零电流/无故障证据。底层保护仍读取实际状态，故不能进一步说“过流保护失效”。H1/H3旧记录有非零电流，说明历史能力与当前relay接通情况须分别判断。[CODE][RESULT]

**时间字段限制：** C12的ACTIVE `ingest_cycle_feedback()` 在批处理后取一个now_ns，给本批收到的各轴FeedbackSample赋相同时间；Bridge把这些值放入`per_axis_rx_done_monotonic_ns`。因此字段名不能证明六路独立物理采样时刻；漏帧轴还可能持有旧值。启动full_scan逐轴事务时间和ACTIVE批次时间亦不能混用。[CODE]

**FK时间限制：** C5用逐轴历史线性插值，只接受可bracket的查询，不外推；launch默认160 ms固定lag、200 Hz查询。200 Hz FK查询不意味着200 Hz独立新反馈，也不意味着现在时刻的位置真值。当前姿态残差闭环不使用FK作误差反馈。[CODE][CONFIG]

**数据可用性的裁决：** 已有基本科研观测链；完整、统一、全窗口、带有效性/源时间说明的双IMU原始流+最终TX+反馈q/current/fault尚未形成可直接宣称齐全的数据集。Demo `--record-telemetry`默认关闭；启用后记录已emit的行，故障可能在append前返回，production底层记录还可能仅保留有界尾窗。补充这些主要属于后续软件记录工作，不自动要求更换全部传感器；外部绝对精度/位置主张才需要额外参考。[CODE][UNKNOWN] C14/D2

## 7. 3DOF Attitude Task Feasibility

**Q1：可以只研究 `R_WP→R_WP,d`，不做XYZ闭环。** 六维机构不强迫论文采用六维任务。只要明确任务误差定义、参考坐标、时间范围、扰动集合、允许平移域和实际输入接口，三轴姿态稳定就是独立控制任务。[CODE][INFERENCE][LITERATURE] C6/C7/L1/L2

此任务仍需完整六支链闭链和六轴执行约束，不能把机构简化成三个独立转台；“XYZ不作为主任务”也不等于平移无限制。允许平移集合必须受行程、被动角、碰撞、载荷和实验环境约束。若后续加入固定钻尖、钻轴进入点或临床接触，其约束会重新占用冗余，此时本报告的三维平移自由结论须重算。[INFERENCE]

当前W可严谨定义为**启动静止参考下的惯性姿态参考**，并限定长期漂移、初始对齐与外部验证范围。不能把“IMU闭环误差小”直接等同于独立计量意义的绝对世界姿态精度。[CODE][UNKNOWN]

现行J11/J13已经固定B系平移`[0,0,z0]`，尚无运行中的三平移在线重构策略。因此“平台具备这些任务冗余变量”已可论证，“当前控制器已经利用冗余扩大能力”则 **NOT FOUND**。[CODE][INFERENCE]

## 8. Three Translational Redundancy Variables

### 8.1 从本机闭链推导，而非从自由度计数推断

令当前几何的固定杆向量与转动后锚点为

\[
\rho_i=Rb_i,\quad s_i=p+\rho_i-a_i-q_i e_z,\quad \|s_i\|=L=0.084\ \mathrm m.
\]

当前负根分支满足

\[
h_i=\sqrt{L^2-(p_x+\rho_{ix}-a_{ix})^2-(p_y+\rho_{iy}-a_{iy})^2}>0,
\quad q_i=p_z+\rho_{iz}-h_i,
\]

即`s_iz=h_i`。对B系相对位置导数v=dot p、B系表达的相对空间角速度ω（`dot R=[ω]×R`）：

\[
\dot q_i=\frac{s_i^Tv+(\rho_i\times s_i)^T\omega}{h_i},\qquad
\dot q=G_pv+G_R\omega=:G[v^T,\omega^T]^T.
\tag{1}
\]

`G=[G_p,G_R]`是本报告推导的**逆速度映射**，不同于C1内部FK残差J或Euler导数。正则构型下

\[
J_R=[0\ I_3]G^{-1},\qquad J_R\dot q=\omega.
\tag{2}
\]

若世界姿态固定，`R=R_WB^T R_WP,d`、`ω=−R_WB^T ω_WB^W`。p固定于B时v=0；世界位置固定则v通常额外含基座平动/转动杠杆项，两者不是同一个任务。测试采用前者，不强加额外XYZ稳定要求。[CODE][INFERENCE] C1/L2

在本次测试正则点，rank(G)=6、rank(J_R)=3、G_p列秩=3，且`J_R G_p=0`。因此平移方向确为姿态任务的三维局部零空间，具备实际运动学参数化，不只是“6−3=3”。这并不证明全域连通、全局三维可用或可免费瞬间切换。[INFERENCE]

### 8.2 本机特有的作用区分

| 改变p但保持R | 本机模型中的结论 | 限制 |
|---|---|---|
| 六个驱动q | 一般改变；Z改变使全部q等量变化 | q不是固定杆长度 |
| 六根U–S杆长 | **不改变，始终84 mm** | 改变的是滑块坐标/杆向量/构型；不要套用6UPS伸缩腿解释 |
| X/Y | 一般改变s、G、J_R与方向需求 | 不保证每个扰动方向都改善，亦不保证净重构收益 |
| 共同Z | `p→p+αe_z, q→q+α1`，s不变，G/J_R完全不变 | 在该几何/分支内成立；改变行程余量而非瞬时速度集合 |
| 奇异接近程度 | X/Y可能影响h_i及G秩/条件；Z不改变这些纯几何指标 | 真实碰撞、柔性/载荷限制不由此消除 |
| 实际持续能力 | q、总速度/加速度、过渡代价、伺服状态共同决定 | 尚无本机在线重构硬件证明 |

当R=I时，`G_R`前两列恰为`[b_iy,−b_ix]`，与p_x/p_y无关；Yaw列仍依赖横向构型。因此**固定B系平移的纯Roll/Pitch瞬时需求在这个姿态下不随X/Y变化**。这比“通常Jacobian会变”更严格，也说明选择单一扰动方向可能看不到收益。[INFERENCE]

对本机，更准确的研究结构是：**两个横向构型变量可改变部分方向的速度映射；一个共同竖直变量调整绝对行程分配；三者的运动都占用同六轴执行能力。** 不应预设三平移在速度优化上具有相同作用。[INFERENCE]

## 9. Test A: Instantaneous Compensation Demand

### 9.1 条件与验证

使用当前C1/C2，R_WB=R_WP,d=R_BP=I；相同B系基座角速度`ω_B=[0.1,0.1,0.1] rad/s`，相对补偿`ω=−ω_B`；各样本先固定v=0。X/Y选择±10 mm，Z选择HOME±10 mm；这些是局部模型点，不是新增硬件测试。所有q均落在几何30…70 mm普通范围，根号远离零。[INFERENCE]

用实际IK做SO(3)左扰动中心差分，核对式(1)：在HOME和非零位置/姿态点，各列最大绝对误差`7.80×10^−11`以内（平移列与旋转列单位分别比较，不把其混为精度指标）。六个样本FK均收敛，位置round-trip误差≤`4.58×10^−11 m`。这是模型内部一致性，不是几何或绝对定位精度。[INFERENCE]

### 9.2 固定平移需求及行程余量

`m_q=min_i(q_i−25,75−q_i)`，单位mm；normal margin另按30…70 mm计算。位置写作`[x,y,z−z0] mm`。

| p偏置 mm | q范围 mm | 绝对/normal margin mm | max\|qdot_i\| mm/s | \|qdot\|₂ mm/s |
|---|---:|---:|---:|---:|
| [0,0,0] | 50.000…50.000 | 25.000 / 20.000 | 4.101193 | 6.788784 |
| [10,0,0] | 48.853852…52.399417 | 22.600583 / 17.600583 | 3.813043 | 6.393521 |
| [0,10,0] | 48.819586…51.879799 | 23.120201 / 18.120201 | 3.937278 | 6.387279 |
| [0,−10,0] | 49.354583…52.435213 | 22.564787 / 17.564787 | 4.283156 | 7.213469 |
| [0,0,10] | 60.000…60.000 | 15.000 / 10.000 | 4.101193 | 6.788784 |
| [0,0,−10] | 40.000…40.000 | 15.000 / 10.000 | 4.101193 | 6.788784 |

HOME六轴qdot为`[−4.101193,−2.405377,+0.661947,+4.101193,+2.405377,−0.661947] mm/s`。X+10 mm使此指定方向峰值需求约下降7.03%，Y−10 mm则上升；不能归纳成“横向平移总有益”。Z±10 mm完全不改变速度，却降低双向最小位置裕量。[INFERENCE]

尺度明确的奇异诊断：取长度尺度ℓ=27.5 mm，`Ghat=G diag(I3,I3/ℓ)`，HOME的condition≈11.3887；X+10≈11.4013，Y+10≈11.3864，Y−10≈11.4137，Z±10与HOME相同。所有样本rank=6，最小h_i≥80.177 mm。未使用混合m/rad的原始condition宣称奇异距离，也未作全工作空间认证。[INFERENCE]

### 9.3 不能把固定v=0需求误称最优姿态能力

允许瞬时平移速度时，可求`min_v ||G_Rω+G_pv||₂`，HOME最小二范数为4.663296 mm/s，已低于固定v=0的6.788784；X+10为4.422703，Y+10为4.396253。这只是瞬时分配，v的积分、允许位置和过渡约束尚未纳入。[INFERENCE]

再用线性规划作**离线等价性/盒约束核查**，求`min_{J_R u=ω} ||u||∞`，不是开发在线控制器：

| p偏置 mm | 最小峰值需求 mm/s（同一混合角速度） |
|---|---:|
| [0,0,0] | 2.133796 |
| [10,0,0] | 2.097282 |
| [0,10,0] | 2.030497 |
| [0,−10,0] | 2.326259 |
| [0,0,±10] | 2.133796 |

等式残差≤`7×10^−17 rad/s`。进一步纯Yaw单位需求`ω=−e_z rad/s`：最小峰值HOME=3.446233 mm/s，X+10=4.996428 mm/s。**连允许全部瞬时平移分配后的盒约束方向能力也不同**，因此不只展示了某个固定逆解的数值改变。但这种点态能力差异本身正属于已有姿态velocity manipulability；尚非研究创新或长期收益。[INFERENCE][LITERATURE]

## 10. Test B: Actuator Constraints

实际物理qdot_max/qddot_max为UNKNOWN，因此分两层解释：[CONFIG][INFERENCE][UNKNOWN]

1. **位置几何/软件层**：按当前绝对raw范围换算25…75 mm；另给30…70 mm normal参考。Test A全部点都满足这两套位置区间。R=I、Z偏置±24 mm时q=74/26 mm，绝对余量仅1 mm，而normal已经超出4 mm；它们适用于绝对范围模型反例，不能标成普通模式可用点。
2. **速度配置层**：定义`ρ_v=max|qdot_i|/(5 mm/s)`、`m_v,cfg=5−max|qdot_i|`；这只是相对Shaper参数的余量。真实速度裕量`qdot_max,hardware−|qdot|`为UNKNOWN。

| Test A点 | ρ_v（固定v=0） | m_v,cfg mm/s | 按配置的瞬时需求 |
|---|---:|---:|---|
| HOME | 0.820239 | 0.898807 | 未超过参考上限 |
| X+10 | 0.762609 | 1.186957 | 未超过参考上限 |
| Y+10 | 0.787456 | 1.062722 | 未超过参考上限 |
| Y−10 | 0.856631 | 0.716844 | 未超过参考上限 |
| Z±10 | 0.820239 | 0.898807 | 相同速度、不同位置余量 |

若将相同方向角速度统一放大1.2倍，固定v=0时HOME峰值4.921432 mm/s，Y−10为5.139787 mm/s：一个满足5 mm/s配置、另一个超过。**这是指定固定平移策略的不可执行参考，不代表姿态任务没有任何可行分配**；上节最小峰值分配仍可能满足。不能把伪逆/固定平移饱和当作整项任务不可行。[INFERENCE]

即使连续理想IK轨迹满足速度/加速度界，实际三阶Shaper也未必零误差跟踪；例如有滤波、离散化、量化和最终slew。因此下面的“未超配置”是必要筛查层面的结果，不是当前完整闭环的硬件PASS。越过配置时可以确定理想零误差参考无法由该限速参考直接原样重放，但不能从此推断真实电机最大能力。[CODE][INFERENCE]

## 11. Test C: Persistent Base Rotational Disturbance

### 11.1 设置

设`R_WB(t)=Exp([e_a θ(t)]×)`，`θ(t)=A sin(2πft)`，`A=5°`，目标`R_WP,d=I`，相对指令`R_BP(t)=R_WB(t)^T`。在各自固定的B系p上逐时调用实际IK；空间角速度为`−e_a dot θ`，以式(1)算qdot。区间0…10 s，步长1 ms；qddot由qdot的数值导数作模型诊断。A/f仅为明确的数学探针，不代表校准过的手持扰动/安全真机参数。[INFERENCE]

### 11.2 Roll，f=0.2 Hz：相同瞬时Jacobian，持续轨迹的行程可行性不同

| p偏置 mm | 全轨迹q范围 mm | 最小绝对位置裕量 mm | 峰值速度 mm/s | 峰值加速度 mm/s² | 首次位置越界（采样） |
|---|---:|---:|---:|---:|---:|
| [0,0,0] | 47.929277…52.091985 | 22.908015 | 2.612001 | 3.321091 | 无 |
| [10,0,0] | 46.783061…54.480192 | 20.519808 | 2.612009 | 3.321738 | 无 |
| [0,0,24] | 71.929277…76.091985 | −1.091985 | 2.612001 | 3.321091 | t≈0.400 s，raw>2000 |
| [0,0,−24] | 23.929277…28.091985 | −1.070723 | 2.612001 | 3.321091 | t≈0.402 s，raw<0 |

HOME与Z+24的G/J_R沿整条参考轨迹完全相同，速度、加速度也相同，但后者上界超过约43.68 step。Z−24下界低于约−42.83 step。这是**持续执行过程中行程不足的模型反例**，且相关端点邻近性有H7历史参考；本次没有在端点附近运行姿态控制。[INFERENCE][RESULT]

这里超过绝对25…75 mm已足以否定该固定p策略，无需假定未知速度极限。关节被动角、碰撞和负载不在这一可行性证书中；“HOME未超这些数值界”不能升级为全部物理条件可行。[INFERENCE][UNKNOWN]

### 11.3 Yaw，f=1.5 Hz：横向构型改变配置约束下的整段需求

| p偏置 mm | 全轨迹q范围 mm | 峰值速度 mm/s | 峰值加速度 mm/s² | 相对配置5 mm/s / 50 mm/s² |
|---|---:|---:|---:|---|
| [0,0,0] | 49.748965…50.350042 | 2.971509 | 35.450502 | 两项未超 |
| [10,0,0] | 48.839882…53.026915 | 5.470582 | 60.917786 | 两项超过 |

此处行程均有余量，固定p参考却在X+10超过配置速度/加速度。它支持“相同基座运动下构型影响持续参考可执行性”，不证明X+10处允许平移速度后的姿态任务整体不可行，也不证明实际电机无法运动得更快。[INFERENCE]

### 11.4 持续扰动不等于无界累积

两种正弦均在10 s包含整数周期，全部样本`q(10)−q(0)`在浮点精度内为零（最大约`1.4×10^−14 mm`）。**本机单值IK对该周期轨迹没有逐周期耗尽行程的必然漂移。** 行程裕量在每个周期内下降、越界或恢复；速度/加速度需求周期性重复。不能写成“只要持续扰动足够久，执行器必然饱和”。[INFERENCE]

上述反例不是在线重构的净增益证明：各策略初始p不同；没有从同一初态计算重构路径、当前速度、采样/伺服滞后、扰动未知未来信息或误差容限。正弦在t=0具有非零角速度，测试评价的是给定持续参考段；启动进入该周期运动的过渡未建模，不把t=0速度违反解释成从静止启动的物理失败。[INFERENCE]

### 11.5 可复核的最小计算代码（报告内嵌，不另生成脚本）

以下只调用纯运动学；不会创建ROS节点或触及硬件。A/B表使用`points`和`d`；C使用固定p周期参考。需要本机已有NumPy/SciPy/PyYAML，本次未安装依赖。

```python
import sys
sys.dont_write_bytecode = True
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from scipy.optimize import linprog
sys.path.insert(0, '/home/lyapunov/bone_drill/bone_ws/src/pus6_kinematics')
from pus6_kinematics.analytical_6pus import Analytical6PusKinematics

cfg = '/home/lyapunov/bone_drill/bone_ws/src/pus6_kinematics/config/6pus_geometry.yaml'
k = Analytical6PusKinematics(config_path=cfg)
a = np.asarray(k.base_anchors)
b = np.asarray(k.platform_anchors_body)
z0 = k.nominal_height

def state(p, R):
    angles = Rot.from_matrix(R).as_euler('xyz')
    q = np.asarray(k.inverse_kinematics([*p, *angles]))
    rb = b @ R.T
    s = p + rb - a
    s[:, 2] -= q
    assert np.min(s[:, 2]) > 0
    G = np.c_[s / s[:, 2, None], np.cross(rb, s) / s[:, 2, None]]
    return q, G

def least_peak(Jr, d):
    # Variables are six qdot plus peak bound t; strictly offline capacity audit.
    ans = linprog(
        np.r_[np.zeros(6), 1.0],
        A_ub=np.r_[np.c_[np.eye(6), -np.ones(6)],
                   np.c_[-np.eye(6), -np.ones(6)]],
        b_ub=np.zeros(12), A_eq=np.c_[Jr, np.zeros(3)], b_eq=d,
        bounds=[(None, None)] * 6 + [(0, None)], method='highs')
    assert ans.success
    assert np.max(np.abs(Jr @ ans.x[:6] - d)) < 1e-9
    return ans.fun

points = [(0, 0, 0), (.01, 0, 0), (0, .01, 0),
          (0, -.01, 0), (0, 0, .01), (0, 0, -.01)]
d = -np.ones(3) * .1
for dx, dy, dz in points:
    p = np.array([dx, dy, z0 + dz])
    q, G = state(p, np.eye(3))
    Jr = np.linalg.inv(G)[3:, :]
    assert np.linalg.matrix_rank(G) == 6
    assert np.linalg.matrix_rank(Jr) == 3
    assert np.linalg.matrix_rank(G[:, :3]) == 3
    assert np.max(abs(Jr @ G[:, :3])) < 1e-9
    fk = k.forward_kinematics_with_metrics(q)
    assert fk.converged
    print('FK', 'position_roundtrip_error_m', np.linalg.norm(np.array(fk.pose[:3])-p),
          'constraint_residual_m', fk.residual_norm)
    fixed = G[:, 3:] @ d
    best2 = fixed - G[:, :3] @ np.linalg.lstsq(G[:, :3], fixed, rcond=None)[0]
    margins = [min(np.min(q-lo), np.min(hi-q))*1000
               for lo, hi in [(.025, .075), (.030, .070)]]
    print('A/B', (dx, dy, dz), 'q_mm', q*1000, 'margins_mm', margins,
          'fixed_max_l2_mm_s', np.max(abs(fixed))*1000, np.linalg.norm(fixed)*1000,
          'min_l2_mm_s', np.linalg.norm(best2)*1000,
          'min_peak_mm_s', least_peak(Jr, d)*1000,
          'yaw_unit_min_peak_mm_s', least_peak(Jr, -np.eye(3)[2])*1000,
          'condition_scaled', np.linalg.cond(G @ np.diag([1,1,1,1/.0275,1/.0275,1/.0275])))

# Check spatial-twist differentiation against existing IK, including nonzero attitude.
max_error = 0.0
for p, e in [(np.array([0.,0.,z0]), np.zeros(3)),
             (np.array([.01,-.005,z0+.01]), np.array([.04,-.06,.03]))]:
    R = Rot.from_euler('xyz', e).as_matrix()
    q, G = state(p, R)
    for j in range(6):
        h = 1e-7 if j < 3 else 1e-6
        u = np.eye(6)[j]
        Rp = Rot.from_rotvec(h*u[3:]).as_matrix() @ R
        Rm = Rot.from_rotvec(-h*u[3:]).as_matrix() @ R
        qp, _ = state(p+h*u[:3], Rp)
        qm, _ = state(p-h*u[:3], Rm)
        max_error = max(max_error, np.max(abs((qp-qm)/(2*h)-G[:,j])))
print('derivative_max_abs_error', max_error)

for axis, f in [(0, .2), (2, 1.5)]:
    for dx, dy, dz in [(0,0,0), (.01,0,0), (0,0,.024), (0,0,-.024)]:
        p = np.array([dx, dy, z0+dz])
        t = np.linspace(0, 10, 10001)
        theta = np.deg2rad(5)*np.sin(2*np.pi*f*t)
        theta_dot = np.deg2rad(5)*2*np.pi*f*np.cos(2*np.pi*f*t)
        qs, vs = [], []
        for ang, rate in zip(theta, theta_dot):
            rotvec = np.zeros(3); rotvec[axis] = -ang
            q, G = state(p, Rot.from_rotvec(rotvec).as_matrix())
            w = np.zeros(3); w[axis] = -rate
            qs.append(q); vs.append(G[:,3:] @ w)
        qs, vs = np.array(qs), np.array(vs)
        acc = np.gradient(vs, .001, axis=0, edge_order=2)
        bad = np.any((qs < .025) | (qs > .075), axis=1)
        print('C', axis, f, (dx,dy,dz),
              'q_bounds_mm', qs.min()*1000, qs.max()*1000,
              'peak_v_a_mm', abs(vs).max()*1000, abs(acc).max()*1000,
              'first_position_violation_s', t[np.flatnonzero(bad)[0]] if bad.any() else None,
              'net_q_mm', (qs[-1]-qs[0])*1000)
```

## 12. Relation to Manipulability

### 12.1 已可明确否定的创新包装

在正则构型、理想速度执行器、仅瞬时姿态任务和给定速度盒`U_v`下：

\[
\mathcal C_\omega(q)=\{d:\exists u\in\mathcal U_v,\ J_R(q)u=d\}
=J_R(q)\mathcal U_v.
\tag{3}
\]

这就是姿态任务velocity capability集合。若用二范数归一速度球，最弱方向半径为`σ_min(J_R V)`；若用盒约束，必须算真实多面体/最小峰值需求，不能把二范数球公式当作精确盒能力。§9的离线计算展示的是这个已知对象。[INFERENCE][LITERATURE] L2 §11

**如果最终贡献只是“调平移让指定方向qdot更小”或给式(3)/最小奇异值另起抗扰指标名称：该方向创新风险高，不建议继续。** 保留它作为工程指标可以，但不能以此立论新科学机制。[LITERATURE][INFERENCE]

### 12.2 一个本机的非等价反例，以及它不能证明什么

§11 Roll下HOME和Z+24具有相同G/J_R、相同任意速度盒映射、相同条件数/奇异值与理想速度需求，却有不同的固定策略整段行程可行性。因此**仅以当前瞬时速度manipulability不可能区分这两个持续轨迹**；q的绝对位置与未来演化不能省略。[INFERENCE]

这不是对所有已有manipulability/dynamic capability理论的反证。加入joint-limit weighting、动态剩余能力、受限路径/可达域的成熟方法可以表示部分或全部现象；本次未证明其无法直接解决。本机共模Z的端点效应也相当直观，不能凭这个反例就宣称新定理或已找到SCI创新。[LITERATURE][INFERENCE]

继续研究时还必须有

\[
\dot q=G_p\dot p+G_R\omega,\qquad
\ddot q=\dot G_p\dot p+G_p\ddot p+\dot G_R\omega+G_R\dot\omega.
\tag{4}
\]

有限行程取决于整个q(t)，加速度取决于速度/构型变化，实际位置伺服又有历史状态和时延。不能只优化最终p的J，再把到达该p的过程当作免费。即使在有限时间问题中出现严格收益，也还需与L2的N03/N05/N06等动态/剩余能力先例核对；“有限时间”不是创新豁免条件。[INFERENCE][LITERATURE]

## 13. 3DOF vs 6DOF Feasibility

| 要素 | 仅3DOF姿态任务 | 完整世界6DOF位姿任务 |
|---|---|---|
| 目标 | `R_WP→R_d` | `T_WP→T_d`，包括世界XYZ |
| 当前传感器 | 双IMU可提供参考姿态/gyro接口；需漂移、标定与有效性说明 | 仍需可靠p_WB或直接p_WP外部观测；双IMU不足 |
| 相对机构状态 | 六轴反馈、带滞后FK提供p_BP/R_BP模型估计 | 可复用，但无法消除整机共同世界平移不可观测性 |
| 输入接口 | 已有位置伺服六轴；新研究可输出受限Cartesian参考 | 同一接口原则上可用，但需完整位姿估计/参考生成与同步 |
| 实验数据 | Base/Platform姿态、gyro、q/current/fault、TX时序和误差 | 在左列基础上新增独立世界位置/基座平移及其时间关系 |
| 工程开发量 | 记录/约束语义补齐、模型运动学接口、独立研究入口；已有姿态链可复用 | 额外位置观测、外参、融合/同步与6DOF扰动验证，显著增加范围 |
| 任务冗余 | 正则局部三平移可用，需约束 | 六轴六维完整任务一般无这三维自由冗余 |
| 当前证据结论 | 工程起点成立，科研性能与创新待建立 | 当前不能宣称已具备完整移动基座世界6DOF稳定能力 |

共同平移任意改变p_WB而保持相对机构状态及姿态不变时，编码器/FK和两路姿态观测无法唯一确定世界位置。IMU加速度积分存在初值、偏置、重力分离及漂移问题；**不能以长期双积分作为可靠绝对XYZ测量**。若基座位置由已标定的固定转轴/机械装置约束而已知，可以使用该外部约束；本次未发现可直接认定为现有移动基座世界位置参考的证据。[CODE][INFERENCE][UNKNOWN]

**Q3：不必研究完整6DOF base motion。** 最严谨的题目范围为 **“3-DOF attitude stabilization of a 6-PUS parallel robot under base rotational disturbances”**，中文“基座旋转扰动下6PUS并联机器人三轴姿态稳定”。若使用 **“base-motion-induced attitude stabilization”**，须在摘要/方法中立即限定为rotational base motion。未测/未系统施加基座平移，不应声称 **“6-DOF moving-base stabilization”**。[INFERENCE]

“仅旋转扰动”也不要求声称人工手持绝无伴随平移；人工动作的平移未测则写未测，不可写成已验证纯旋转。只控制姿态时p_WB不出现在姿态组合式中，故这项不确定性不自动迫使加入XYZ闭环；伴随平移的惯性/载荷影响和对其他条件的混杂仍需按实验范围说明。[INFERENCE][UNKNOWN]

## 14. Current Candidate Research Problem

**建议保留的候选数学问题：在固定导轨6PUS的实际位置伺服边界下，允许且可达的平移重构，何时能严格扩大给定基座旋转扰动族的有限时间姿态可维持域？** 这不是已确认创新点，不指定最终控制算法。[INFERENCE]

可先利用本机结构写成

\[
q_i(t)=z(t)+c_i(x(t),y(t),R_{WB}(t)^T R_d),
\quad c_i=(R b_i)_z-h_i.
\tag{5}
\]

在给定x/y/R时，几何位置约束给出共模高度区间：

\[
\max_i(q_{i,\min}-c_i(t))\ \le z(t)\le\
\min_i(q_{i,\max}-c_i(t)).
\tag{6}
\]

固定z贯穿整个区间的条件是

\[
\sup_{t\in[0,H]}\max_i(q_{i,\min}-c_i(t))\le
\inf_{t\in[0,H]}\min_i(q_{i,\max}-c_i(t)).
\tag{7}
\]

式(6)逐时非空不等于在限速/限加速度下存在可达的z(t)；X/Y重构亦会改变c_i及总运动需求。这个“横向形状变化+竖直共模行程分配”的分解是本机可明确利用的数学结构，但其原创性和超出现有受限规划的价值仍待判断。[INFERENCE][LITERATURE]

更完整的候选对象是可维持初态集合：

\[
\mathcal V_H(\mathcal B,\varepsilon)=\{x_0:\exists\text{因果平移选择策略},
\ \forall b(\cdot)\in\mathcal B,
\ d_{SO(3)}(R_{WP},R_d)\le\varepsilon,
\ \text{全部实际约束在}[0,H]\text{内满足}\}.
\tag{8}
\]

x0至少包含相对构型、必要速度及影响输出的参考整形/伺服状态；B明确旋转方向、幅频/导数界、持续时间及未来信息。尚未辨识的实际伺服/约束不能被假定已知，故式(8)目前是**问题候选定义**。[INFERENCE][UNKNOWN]

对照固定B系辅助平移，在相同x0、扰动、误差界、传感信息和约束下，检验允许重构后的集合是否有**严格包含或明确容量严格增益**。非严格“≥”往往只是策略集合包含固定策略的平凡结果；只比较两个不同初始p不足以证明在线收益。已知整段扰动的离线解不能冒充未知未来条件下的因果能力。[LITERATURE][INFERENCE]

**Q5的优先级：先证明/否定净严格收益及其成立/失效条件，再决定是否值得控制器研发。** 若只有静态选点有效，诚实定位为构型选择/受限轨迹应用问题；不靠算法组合延续宽泛创新表述。

## 15. Missing Evidence

| 缺口 | 当前状态 | 它限制什么结论 |
|---|---|---|
| 六轴真实持续速度/加速度与负载、位置误差关系 | **UNKNOWN** | 无法给硬件最大基座补偿幅频边界 |
| 封装位置伺服动态、跟踪误差/滞后/饱和与温升降额 | **UNKNOWN** | 几何轨迹不能替代真实可实现性 |
| 最终锚点/杆长/机械零位/行程端点及不确定度 | YAML仍pending；有局部运行证据 | 模型容量数值不能变成计量级机器人规格 |
| 实际U/S安装角域、碰撞/杆交叉、柔性和耦合工作空间 | 当前ROS完整检查 **NOT FOUND**；历史名义检查存在 | IK有解不能称全部物理可行 |
| 在线三平移重构的控制/实验记录 | **NOT FOUND**于所查J11/J13/J14路径 | 尚不能声称已有冗余重构硬件能力或净收益 |
| 原始双IMUgyro、source/rx时刻、有效性、持续全窗口 | 接口有，现有主CSV不齐 | 无法以快照差分证明精确物理角速度/角加速度、同步或高动态界 |
| 当前controller电流/fault日志真实性 | 所查J14旧缓冲为默认零 | 零值不能支持无过流/无故障主张 |
| 全窗口最终TX/feedback与FK关联 | timed接口存在；production可截尾；批次时间语义 | 无法仅以文件名full或上层command代表全部已执行运动 |
| 独立姿态参考、绝对航向与长期漂移 | **UNKNOWN/未验证** | 不能给外部绝对三轴稳定精度结论 |
| 基座平移/平台世界位置 | 当前双IMU+FK不能可靠给出 | 完整世界6DOF主张不成立；不阻止限定姿态任务 |
| 同一扰动/初态/负载/信息条件下的净收益 | **NOT FOUND** | 现有人工扰动历史不能作公平算法对比 |
| 最近文献正文差异 | L2 N08 Khan、N09 Lin、N10 Cui仍有正文缺口 | 工程正面结果不能证明论文创新空白 |

这里的NOT FOUND限定在本次列出的源码、配置、相关历史路径和输入报告，不表示全盘所有未读数据或全球研究均不存在。没有用缺失项阻断本报告交付，也没有自行填补任何物理参数。[UNKNOWN]

## 16. Innovation Risk

**总体创新风险：高；工程问题成立与创新成立是两项判断。** [LITERATURE][INFERENCE]

| 候选表述 | 审计判断 |
|---|---|
| 六自由度平台执行三轴姿态，其余三维作任务冗余 | 已有直接Stewart先例；不可作独立创新 |
| 同扰动下调构型降低速度需求 | Test A成立，但瞬时版本属于任务velocity capability |
| 从条件数改成最小奇异值/方向半径/最小峰值 | 已有定向/最弱方向能力理论；更名不成立 |
| 增加行程/速度/加速度/有限时间 | 必要建模条件；通用受限规划和动态能力已有，并不自动新颖 |
| 本机Z共模不变性导致相同行程外形、不同端点余量 | 有明确数学事实，可帮助缩小问题；单独很可能只是结构性质/已知关节限位效应 |
| 可达重构在相同初态/信息下严格扩大持续补偿域 | 尚待证明/实测；即使成立还需排除已有动态/剩余能力方法可直接覆盖 |
| 位置伺服实机而非广义力仿真 | 工程价值明确；接口更换本身不保证方法创新 |

L2已经报告N01任务冗余抗扰、N03剩余力能力、N05全轨迹能力、N06受限输入/沿程最小能力等正面先例。不能说“以往只有瞬时manipulability”，也不能由Khan/Lin/Cui正文未取得推出它们没有做本题。优先核对具体问题与保证，而不是重新堆叠控制器名称。[LITERATURE]

若后续证据仅支持“传统指标选更好的p”，**淘汰这一创新主张**；可以保留工程应用结果，不能为保持候选题目而宣称新抗扰理论。

## 17. Engineering Risk

| 风险 | 已有事实/可能后果 | 对研究范围的含义 |
|---|---|---|
| 位置伺服被误建成力输入 | 实际只发位置；residual不是真实wrench | MATLAB必须保持相同接口层次，理想力输入结果不能当ROS可执行证明 |
| 行程坐标错用 | q_geo与raw米相差25 mm | 将直接偏移FK/边界；必须统一显式映射 |
| 无限/免费辅助平移 | IK未覆盖全部实体约束；过渡占用执行能力 | 需要有界允许域与过渡状态，不把自由变量当无限运动 |
| 默认配置被当实际配置 | normal/maintenance、旧mode/current字段与实际Servo不同 | 每个证据使用其实际配置；不改变冻结运行策略 |
| 观测不完整 | 默认零电流、batch时间、IMU原始流缺失 | 可能把日志缺口当物理结果；需数据语义补齐 |
| 不同层运行PASS混淆 | J14通信已运行；所查姿态误差仍可很大 | 先定义姿态误差/持续时长标准，不能沿用runtime PASS作论文性能PASS |
| 时序与外部USB | H6有长尾；D1记录真实USB/IMU中断 | 时延/故障是执行条件；不据单次PASS宣称任意长时可靠 |
| 边界靠近实体限位 | H7为接近端点后主动停止，模型仍pending | Test C端点反例只离线，不提供端点运动授权 |
| 研究范围扩张 | 世界XYZ需要额外位置观测；钻尖约束会占用冗余 | 当前论文可明确限定三轴姿态与基座旋转 |

上述风险不意味着需要新增多层motion gate或重构生产架构。本次不修改任何保护；后续应按证据补齐科研能力边界，分别处理物理风险、记录完整性和性能指标。[INFERENCE]

## 18. Recommendation

### 五个必须回答的问题

| 问题 | 明确回答 |
|---|---|
| **Q1：只做R_WP、不做XYZ？** | **可以。** 是定义完整的三维姿态任务；既有闭环是工程基础。需限定参考、误差、扰动与允许平移域；不承诺已有SCI创新或高精度硬件结论。 |
| **Q2：三平移可作为task redundancy？** | **正则局部可以。** 当前闭链与rank/零空间核查支持。X/Y影响部分速度映射，Z是共同行程分配方向；不是三个同等的速度优化变量，也不是完整6DOF任务下的驱动冗余。 |
| **Q3：必须完整6DOF base motion？** | **不必须。** 基座旋转扰动下三轴姿态稳定即可严谨成立；当前双IMU不可靠支持长期世界XYZ。未测伴随平移须如实说明。 |
| **Q4：不同平移真的改变持续补偿能力？** | **模型/配置层成立；真实硬件能力改善尚未验证。** Test A给需求和方向能力差异，Test C给相同J却不同整段行程可行性，以及不同横向构型的配置速度/加速度违反。尚无同初态在线重构净收益证明，正弦也不必然逐周期耗尽行程。 |
| **Q5：下一阶段核心数学问题？** | **可达平移重构何时严格扩大有限时间姿态可维持域，以及何时不能。** 用本机“X/Y形状+Z共模”结构，统一行程、总速度/加速度、伺服状态、误差界与因果信息；先证实严格收益/失效条件，不先确定最终算法。 |

### 最终裁决：B——3DOF可以成立，但需要补充明确条件

3DOF足以构成一篇独立研究的**任务范围**：主任务、有限执行能力、允许辅助变量和可维持条件均可以自洽定义，也有现成硬件测量/执行链支撑。是否足以形成一篇有实质贡献的SCI论文，仍取决于理论/机制差异、可信边界与新实验，**本次不作可发表性保证**。[CODE][LITERATURE][INFERENCE]

三个平移变量应进入受限轨迹/可达性问题，而不是笼统的“姿态优先+冗余优化”标签。最值得保留的是式(5)–(8)所表达的持续可行性问题：构型选择、共模行程再分配与过渡消耗之间是否有可解释的严格收益。若不能排除成熟能力指标的直接重述，或效应小于真实模型/观测误差，应停止该创新主张。[INFERENCE]

当前工程**能够支撑后续MATLAB运动学/配置约束预研，也具有ROS2位置伺服真机研究的可复用基础**；不能直接支撑未经辨识的力级仿真迁移、准确物理极限、无限时间保证或已完成的在线重构。先明确真实边界/记录语义、统一参考与因果信息，再判断窄问题是否有净收益；这是研究前提，不是本报告授权执行新硬件实验。[CODE][UNKNOWN]

不选A：真实边界、净收益、创新冲突尚未闭合。不选C：局部任务冗余和构型相关可行性已有当前模型/工程支持，不能仅因创新风险高就否定全部3DOF问题。不选D：没有工程或数学理由强迫转向世界6DOF任务，而其感知与验证成本明显增加。

交付复核：已从本报告提取并重新执行内嵌离线代码，核对主要数值、秩/零空间与FK一致性；全部本机来源链接可访问，18节结构齐全。Git tracked diff为空。本次验证没有运行完整生产链，不能替代真机验证。

**交付边界：仅本报告；生产代码、配置、历史结果未修改；未commit；本次 `hardware_write_count = 0`。审计完成，不进入控制器开发或真机运行。**
