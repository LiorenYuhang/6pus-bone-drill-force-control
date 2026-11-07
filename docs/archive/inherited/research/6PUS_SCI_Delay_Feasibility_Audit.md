> 历史归档：原路径 `agent_docs/research/6PUS_SCI_Delay_Feasibility_Audit.md`。正文保留当时的方案、参数及证据语境；当前原生 Mode 3 入口以[项目 README](../../../../README.md)为准。历史命令不作为当前推荐启动命令。

# 6PUS SCI Delay Feasibility Audit

审计日期：2026-10-16，Asia/Shanghai。工程：`/home/lyapunov/bone_drill`；ROS 2 workspace：`/home/lyapunov/bone_drill/bone_ws`。

**核心回答：数据不足以可靠确定现有移动基座姿态闭环的完整“感知—计算—执行—物理响应”有效时延，也不足以证明时延是主要性能瓶颈。此两项均为 UNKNOWN。** 现有数据可以计算若干主机软件时间段；J11 人工扰动数据可以做有限的低频幅相描述；另有一组旧 J8 周期参考到反馈 FK 的单频记录，但它不能替代当前 J11/J13/J14 的移动基座双 IMU 端到端辨识。

本次只有读取、内存中的离线统计/谱分析和本报告写入。没有运行 ROS 节点、项目 runner、串口查询、驱动、Mock 实验或真实硬件；没有发送任何控制/电机指令，没有修改生产代码、配置或历史结果，没有 commit。**本次 `hardware_write_count = 0`。** 上一份 3DOF 可行性报告保持不变。

证据分级：`[CODE]` 当前源码；`[CONFIG]` 配置；`[RESULT]` 保存的历史数据；`[CALCULATED]` 本次对真实记录的离线计算；`[INFERENCE]` 依赖假设的解释/模型推导；`[UNKNOWN]` 当前不能确定。主机事件计时、采样/通信代理时间和物理响应时延分别处理。

## A. 数据来源与路径

### A.1 范围与来源登记

开始时工程为 `main`，HEAD=`117384dc37248c30ce0ec4b9c36f1754acba9b9f`，tracked diff 为空；已有未提交报告、文献输入和历史实验资产，全部保留。本次检查当前源码，并检查 `results/j11`、`results/j13`、`results/j14` 的文件名及 CSV 表头：分别327、106、871份CSV，共1304份；这些包含静态、失败、dry-run、Mock、备份/打包副本，**不是1304个独立硬件实验**。在这三个展开目录未找到 `.db3`/`.mcap`。没有把所有 ZIP、旧 Windows 包或全盘未读文件宣称为已审计。

对可能有用的记录进一步逐行读取。只在发现直接相关线索时补查旧 J8 周期记录、J9 原始 IMU 记录。未开展新文献综述，也未根据文件名中的“real”“raw”“full”“delay”直接判定其含义。[RESULT][CODE]

| 编号 | 本机来源 | 用途与边界 |
|---|---|---|
| R1 | [J11 M1 CSV](/home/lyapunov/bone_drill/results/j11/runtime_validation/20260918_112354/J11_DISTURB_QUAL_feedforward_powered.csv)、[summary](/home/lyapunov/bone_drill/results/j11/runtime_validation/20260918_112354/J11_DISTURB_QUAL_feedforward_summary.json) | 约120 s人工扰动会话，Base逆旋转前馈；Platform用于观测，适合探索扰动→剩余误差谱 |
| R2 | [J11 M2 CSV](/home/lyapunov/bone_drill/results/j11/m2_pid/20260920_095850/j11_m2_7a_pitchd_cand2_repeat_phaseB_dynamic_telemetry_powered.csv) | retained P/PD人工扰动；控制器tick、IMU callback age、位置快照 |
| R3 | [J13 Roll active CSV](/home/lyapunov/bone_drill/results/j13/j13_2b1_roll_active/20261006_193913/j13_2b2_roll_active_20s/j13_2b2_roll_active_20s_telemetry_powered.csv)、[summary](/home/lyapunov/bone_drill/results/j13/j13_2b1_roll_active/20261006_193913/j13_2b2_roll_active_20s/j13_2b2_roll_active_20s_summary_powered.json) | ACTIVE约18.30 s；最终 `scheduler_deadline_exceeded` FAULT，不能称完整正常20 s实验 |
| R4 | [J13三轴偏心负载CSV](/home/lyapunov/bone_drill/results/j13/j13_2c1_3axis_active/20261007_143036/j13_3axis_payload_ladrc_10s/j13_3axis_payload_ladrc_10s_telemetry_powered.csv) | 三轴active、短时记录；不能辨识低频完整频响 |
| R5 | [J14 Classical CSV](/home/lyapunov/bone_drill/results/j14/j14_a2_q3_r1d_classical_60s/j14_a2_q3_r1_classical_telemetry_powered.csv)、[Child cycles](/home/lyapunov/bone_drill/results/j14/j14_a2_q3_r1d_classical_60s/child_cycles.jsonl)、[逐轴write metrics](/home/lyapunov/bone_drill/results/j14/j14_a2_q3_r1d_classical_60s/child_write_metrics.jsonl)、[Bridge pipeline](/home/lyapunov/bone_drill/results/j14/j14_a2_q3_r1d_classical_60s/bridge_feedback_pipeline.jsonl) | qualification窗口；可按cycle_seq/command_seq关联部分驱动事件 |
| R6 | [J14 LADRC CSV](/home/lyapunov/bone_drill/results/j14/j14_a2_q3_r2_ladrc_60s/j14_a2_q3_r1_ladrc_telemetry_powered.csv)、[Child cycles](/home/lyapunov/bone_drill/results/j14/j14_a2_q3_r2_ladrc_60s/child_cycles.jsonl)、[逐轴write metrics](/home/lyapunov/bone_drill/results/j14/j14_a2_q3_r2_ladrc_60s/child_write_metrics.jsonl)、[Bridge pipeline](/home/lyapunov/bone_drill/results/j14/j14_a2_q3_r2_ladrc_60s/bridge_feedback_pipeline.jsonl) | 同上，另有timed-feedback分段代理时间；并非与R5同一扰动 |
| R7P | [PID run_002 telemetry](/home/lyapunov/bone_drill/results/j14/controller_compare/pid/run_002/telemetry.csv)、[metadata](/home/lyapunov/bone_drill/results/j14/controller_compare/pid/run_002/metadata.json)、[Child tail](/home/lyapunov/bone_drill/results/j14/controller_compare/pid/run_002/child_cycles.jsonl) | 完整已emit控制器行；底层仅300周期尾窗 |
| R7A | [LADRC run_002 telemetry](/home/lyapunov/bone_drill/results/j14/controller_compare/adrc/run_002/telemetry.csv)、[metadata](/home/lyapunov/bone_drill/results/j14/controller_compare/adrc/run_002/metadata.json)、[Child tail](/home/lyapunov/bone_drill/results/j14/controller_compare/adrc/run_002/child_cycles.jsonl) | 同上；实际mode=3axis_active，不能按文件名shadow误判 |
| R8 | [J11 Base“raw”CSV](/home/lyapunov/bone_drill/results/j11/j11_3a1/j11_3a1_base_imu_raw_data.csv) | 机械轴资格用50 Hz timer快照；**不是逐条传感器原始消息日志** |
| R9 | [J9 Pitch+原始联合CSV](/home/lyapunov/bone_drill/results/j9_1/rotation_lever_arm_characterization/j9_1_rotation_lever_arm_pitch_pos05deg_20260907_115030.csv)、[metadata](/home/lyapunov/bone_drill/results/j9_1/rotation_lever_arm_characterization/j9_1_rotation_lever_arm_pitch_pos05deg_20260907_115030_metadata.json) | 旧平台gyro/姿态、Base匹配、host/ROS stamp；用于说明历史观测能力，不能移植采样率到J14 |
| R10 | [旧J8 metadata](/home/lyapunov/bone_drill/results/stewart3_j8_figure6_20260916_135013/metadata.json)、[target_pose](/home/lyapunov/bone_drill/results/stewart3_j8_figure6_20260916_135013/target_pose.csv)、[command_steps](/home/lyapunov/bone_drill/results/stewart3_j8_figure6_20260916_135013/command_steps.csv)、[joint_feedback](/home/lyapunov/bone_drill/results/stewart3_j8_figure6_20260916_135013/joint_feedback.csv)、[FK](/home/lyapunov/bone_drill/results/stewart3_j8_figure6_20260916_135013/fk.csv)、[stage_events](/home/lyapunov/bone_drill/results/stewart3_j8_figure6_20260916_135013/stage_events.csv) | 保存了周期组合目标与关节/FK；**无Base/Platform IMU流**，不是移动基座姿态抗扰数据 |

### A.2 时间语义的源码入口

| 编号 | 来源与关键位置 |
|---|---|
| C1 | [H30 publish](/home/lyapunov/bone_drill/bone_ws/src/h30_imu/src/h30_imu_node.cpp:84)、[parser](/home/lyapunov/bone_drill/bone_ws/src/h30_imu/src/h30_protocol.cpp:32)：host `now()`、parser tid |
| C2 | [J11 callbacks/runner](/home/lyapunov/bone_drill/bone_ws/src/h30_imu/scripts/j11_m2_pid_residual_runner.py:624)、[M1 runner](/home/lyapunov/bone_drill/bone_ws/src/h30_imu/scripts/j11_base_feedforward_runner.py) |
| C3 | [J13 callbacks](/home/lyapunov/bone_drill/bone_ws/src/h30_imu/scripts/j13_ladrc_shadow_runner.py:452)、[tick与分段计算](/home/lyapunov/bone_drill/bone_ws/src/h30_imu/scripts/j13_ladrc_shadow_runner.py:532)、[Cartesian publish](/home/lyapunov/bone_drill/bone_ws/src/h30_imu/scripts/j13_ladrc_shadow_runner.py:1094) |
| C4 | [Parent目标入口](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/motor_driver_node.py:593)、[IPC传递](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/j14_process_driver.py:85)、[Child ingest](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/j14_process_uart.py:95)、[cycle/记录](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/j14_process_uart.py:259) |
| C5 | [UART计时](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/j7_real_uart_adapter.py:498)、[Bridge](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/j14_feedback_bridge.py:170)、[feedback ingest](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/j7_servo_feedback_scheduler.py:362) |
| C6 | [TimedJointCommand](/home/lyapunov/bone_drill/bone_ws/src/pus6_msgs/msg/TimedJointCommand.msg)、[TimedJointFeedback](/home/lyapunov/bone_drill/bone_ws/src/pus6_msgs/msg/TimedJointFeedback.msg)、[FK node](/home/lyapunov/bone_drill/bone_ws/src/pus6_kinematics/pus6_kinematics/fk_node.py) |
| C7 | [三阶Shaper](/home/lyapunov/bone_drill/bone_ws/src/pus6_control/pus6_control/joint_trajectory_controller.py:174)、[control node](/home/lyapunov/bone_drill/bone_ws/src/pus6_control/pus6_control/control_node.py)、[J13 YAML](/home/lyapunov/bone_drill/bone_ws/src/h30_imu/config/j13_ladrc_params.yaml) |
| C8 | [J11“raw”记录器](/home/lyapunov/bone_drill/bone_ws/src/h30_imu/scripts/j11_base_imu_mechanical_axis_qualification.py:137)、[Demo记录器](/home/lyapunov/bone_drill/bone_ws/scripts/j14_demo_controller.py:61) |
| C9 | [旧J8周期生成](/home/lyapunov/bone_drill/bone_ws/scripts/run_j8_final_video_demonstration.py:238)、[旁路记录器](/home/lyapunov/bone_drill/bone_ws/scripts/stewart3_j8_figure6_recorder.py:184)、[E3正弦验证代码](/home/lyapunov/bone_drill/bone_ws/scripts/run_j8_e3_sinusoidal_validation.py:424) |
| D1 | [J11总结](/home/lyapunov/bone_drill/agent_docs/stages/J11/J11_M1_FINAL_SUMMARY.md)、[J11 README](/home/lyapunov/bone_drill/agent_docs/stages/J11/README.md:572)、[J14交接](/home/lyapunov/bone_drill/agent_docs/stages/J14/J14_FINAL_HANDOFF.md)、[既有NUC审计](/home/lyapunov/bone_drill/agent_docs/audits/stewart_migration_nuc_audit.md)；仅作导航/交叉核对，关键时间定义回查源码 |

## B. 时间戳与频率审计

### B.1 必须分开的事件

```text
真实基座运动 / 设备采样
  → IMU内部解算、传输 → H30节点publish → controller订阅回调
  → controller tick读取 → 计算 → Cartesian publish
  → IK → Shaper → command_joint_steps publish → Driver Parent接收
  → IPC / Child ingest → Child周期 → 逐轴serial.write
  → USB/UART传送、驱动器内部位置伺服 → 真实平台运动
  → Platform IMU采样/解算/传输 → publish → controller回调 / 快照
```

现有记录只覆盖其中部分事件。小的某段时间不能证明整个链路小；多个不同对象的age/分位数也不能简单相加得到总时延。[CODE][INFERENCE]

| 对象/字段 | 实际语义 | 能否确定物理感知—执行时延 |
|---|---|---|
| Base、Platform `Imu.header.stamp` | H30节点 `publish()` 中的host ROS `now()`；不是设备采样时刻 | 不能。当前主controller CSV还未保存这些原始header |
| IMU `tid` | parser解析16位tid，但C1没有将它发布成设备时间/消息序号 | 不能反推出device sample time或丢包率 |
| `timestamp_ns` | C2/C3控制器tick开始的`perf_counter_ns()` | 可算tick间隔；不是输出publish时刻，也不是同一行全部信号的采样时刻 |
| `base_imu_age_ms`、`platform_imu_age_ms` | `tick_start−latest_callback_receive_time`；callbacks只保留最新quaternion和host接收时间 | 可量化所选缓存的回调年龄；不能当sensor内部或sensor→command全延迟 |
| 控制器输入/输出快照 | 当前tick读取的姿态、算出的R_BP_cmd/残差共用行时间戳 | 有tick级对应；无计算起止/publish结束的独立计时 |
| Cartesian `PoseStamped.header` | 近publish处另取ROS `now()`；之后IK转换为无header数组 | 在线有header；所查CSV/下游链未保留跨IK/Shaper的统一来源ID/时间 |
| `commanded_steps` | controller看到的最近一次Shaper命令 | 是旧/最新命令快照，不是该tick刚算出的Cartesian输出或UART ACK |
| `command_publish_ns`（Child JSON） | 名称虽含publish，源自Driver Parent `_enqueue_latest_target()`的本地接收stamp | **不是**IMU时间、Cartesian publish或Shaper publish时间 |
| `command_receive_ns` | Child对IPC命令做ingest的host时间 | 可计算Parent入口到Child ingest，含缓存/轮询调度 |
| `serial_metric.timestamp_start/end_ns` | 主机调用`serial.write()`的开始/返回 | 可计算主机写调用；返回不等于线上最后一位发完、电机收到或机械运动开始 |
| Timed command / feedback | host事务、cycle和逐轴数组，含共同UART cycle seq | 是重要软件关联基础；设备内部执行/采样时间未找到 |
| `per_axis_rx_done_monotonic_ns` | ACTIVE ingest以同一批`now_ns`赋给收到应答的各轴；缺失轴可保持旧值 | 字段名不能证明逐轴独立物理采样时刻；启动full_scan语义也不同 |
| FK query/header | query为固定lag过去时刻；header为之后publish时刻 | 可给模型重建状态时间，不能把固定lag当姿态闭环执行时延 |

本机本次Python查询显示`perf_counter`与`monotonic`都使用`CLOCK_MONOTONIC`，与记录数量级/源码约定一致。跨host单调时钟不成立；本次只在同机同会话/相同epoch中关联。ROS/epoch时间与monotonic不直接相减；不使用目录日期建立毫秒级对应。当前时钟实现检查也不是历史时钟漂移/设备同步的校准证书。[CODE][INFERENCE]

### B.2 真实采样频率与记录频率

**Base IMU与Platform IMU的设备内部采样率、姿态输出频率、固件滤波延迟，在所查J11/J13/J14数据中均为 UNKNOWN。** C1的2 ms串口poll timer不证明500 Hz传感器数据；D1示意图的400 Hz不等于每个会话实测400 Hz。R8有2503行，但C8明确由20 ms timer采最新值，不能拿它当逐帧IMU日志。[CODE][UNKNOWN]

R9 metadata另报告旧会话Platform/Base effective rates约131.50/128.73 Hz，Platform stamp间隔median约5.983 ms、max153.794 ms；这些是该次host/发布流的统计，不是J14设备采样规格。R9保存原始gyro/姿态接口这一事实可以复用；频率数值和延迟不能跨会话继承。[RESULT]

以下只取CSV中`state==ACTIVE`，用相邻`timestamp_ns`重新计算间隔，**不以行号×20 ms代替真实时间**。所有ACTIVE行保留，不剪掉长尾、不裁dt；仅排除预备状态是任务状态选择，不是剔除异常。

| 数据 | 全行/ACTIVE行 | ACTIVE跨度s | tick间隔P50/P95/Max ms |
|---|---:|---:|---:|
| R1 J11 M1 | 5999 / 5929 | 118.579807 | 20.003 / 20.683 / 46.795 |
| R2 J11 M2 | 2999 / 2946 | 58.920119 | 20.000 / 20.478 / 56.376 |
| R3 J13 Roll | 995 / 916 | 18.299940 | 20.000 / 20.097 / 20.685 |
| R4 J13三轴 | 500 / 446 | 8.901115 | 20.000 / 20.316 / 21.100 |
| R5 J14 Classical | 2999 / 2945 | 58.900160 | 20.000 / 20.281 / 55.511 |
| R6 J14 LADRC | 2997 / 2942 | 58.880015 | 20.000 / 20.333 / 63.632 |
| R7P PID2 | 1359 / 1215 | 24.279969 | 20.000 / 20.203 / 21.031 |
| R7A LADRC2 | 1215 / 1163 | 23.239906 | 20.000 / 20.179 / 20.420 |

R1 summary的`active_cycles=5928`与按CSV状态筛出的5929不同；本报告统计明确使用后者。R3最后FAULT不能因ACTIVE行间隔较规则就否认底层故障。**约50 Hz是观察到的tick速率，不是20 ms总延迟。** [RESULT][CALCULATED]

## C. 可计算的延迟与年龄

### C.1 可以直接证明的主机分段计时

对R5/R6，以Child `cycle_seq`连接逐轴write metrics，同一`command_seq`仅取首次在所保存Child周期中出现的记录；要求正的合法时间、Parent入口≤Child ingest≤cycle start。没有命令的启动记录排除；重复持有同一命令不作为新的输入样本。统计只代表**实际出现在该Child记录中的命令子集**，被latest-value覆盖的命令不在内。

R5保留3529个Child周期、21174个write记录，3435个不同有效命令；R6为3539、21234、3443。窗口覆盖驱动生命周期，**不等同于ACTIVE姿态CSV窗口**。以下单位均ms，P50/P95/P99/Max。[CALCULATED]

| 时间段 | R5 Classical | R6 LADRC |
|---|---:|---:|
| Parent目标入口→Child ingest | 5.281 / 9.609 / 10.028 / 11.054 | 6.839 / 9.616 / 9.997 / 11.331 |
| Child ingest→首个记录周期start | 9.851 / 13.269 / 13.594 / 13.924 | 12.991 / 13.568 / 13.772 / 13.998 |
| Parent入口→首个记录周期start | 15.137 / 15.481 / 15.597 / 15.761 | 19.838 / 20.178 / 20.311 / 20.909 |
| Parent入口→ID1 serial.write开始 | 15.433 / 15.761 / 15.901 / 16.154 | 20.137 / 20.450 / 20.596 / 21.254 |
| Parent入口→ID6 serial.write开始 | 18.831 / 19.239 / 19.450 / 23.487 | 23.496 / 23.932 / 24.245 / 27.119 |
| 单次serial.write调用wall，全部六轴记录 | 0.0376 / 0.0784 / 0.0991 / 1.6107 | 0.0344 / 0.0829 / 0.1085 / 2.8735 |
| Child反馈ready→Bridge收到 | 0.4548 / 0.5842 / 0.6657 / 1.3358 | 0.4109 / 0.5555 / 0.6646 / 1.4124 |

公式例：

\[
\Delta_{P\to TX_i}=
(\text{serial_metric.timestamp_start_ns}_i-\text{command_publish_ns})/10^6.
\]

这些可以证明主机事件时间差，**不能**改称sensor-to-command、command-to-physical-platform或总有效时延。逐轴发送时间不同，不能把六轴当作同一瞬间下发。两会话的差异也可能来自周期相位/调度，而非控制算法计算量；本报告不作算法优劣比较。各列分位数不可相加恢复另一列分位数。[CODE][CALCULATED][INFERENCE]

还需注意：Child当前command_seq与最终下发向量不是“原目标原样执行”的保证；feedback hold、持值和final slew可使实际TX仍为锚点或部分推进。C.1统计的是携带该命令序号的主机事件关联，不是该目标位置已到达、或该目标原值首次作用于机械系统的时间。

### C.2 由记录计算的IMU回调年龄：可用代理，非sensor latency

下表P50/P95/Max，ms；仅ACTIVE行。[CALCULATED]

| 数据 | Base callback age | Platform callback age |
|---|---:|---:|
| R1 | 2.186 / 4.736 / 9.827 | 2.266 / 4.588 / 39.170 |
| R2 | 0.748 / 4.606 / 36.887 | 3.624 / 6.148 / 42.366 |
| R3 | 1.348 / 5.298 / 5.522 | 0.947 / 4.944 / 5.201 |
| R4 | 0.282 / 5.480 / 6.394 | 1.405 / 1.618 / 5.599 |
| R5 | 0.731 / 2.797 / 56.178 | 2.194 / 6.226 / 57.830 |
| R6 | 2.207 / 2.572 / 65.880 | 2.985 / 3.384 / 66.803 |
| R7P | 1.729 / 1.905 / 6.069 | 1.423 / 1.715 / 7.837 |
| R7A | 4.822 / 5.201 / 5.393 | 5.052 / 5.334 / 5.567 |

可重建所选callback接收时间代理：`t_rx≈t_tick−age×10^6`。它没有恢复被覆盖的中间IMU消息、设备采样时刻、该姿态估计内部滤波历史，也没有恢复tick后计算/发布时间。因而不能根据这个代理计算可靠sensor sample rate或完整sensor-to-command delay。小age也不能排除设备内部大滤波延迟。[CODE][INFERENCE]

R3的`driver_feedback_age_ms` P95/Max约577.885/1072.114 ms；R4为225.324/352.395 ms，均是旧runner反馈回调年龄。R5对应4.985/59.784，R6为19.412/82.219。它们证明不同运行记录的反馈新鲜度不同，**不是电机机械响应时间**，也不是同条件修复前后性能比较。[CALCULATED]

### C.3 R6 timed-feedback“latency”列的正确解释

| 列 | P50/P95/Max ms | 审计解释 |
|---|---:|---|
| delta_t_bus_ms | 6.438 / 6.914 / 9.628 | cycle/snapshot start到最新host反馈ingest代理；不是独立纯UART传播时延 |
| delta_t_driver_proc_ms | 0.069 / 0.095 / 0.227 | snapshot end减所选最新反馈代理；局部分段 |
| delta_t_dds_transport_ms | 1.360 / 2.728 / 8.098 | ROS header publish到runner收到timed-feedback回调，含传输/调度等；不是只有DDS网络时间 |
| timed_feedback_age_at_tick_ms | 18.111 / 20.274 / 82.088 | tick时距最近timed-feedback回调的年龄 |

旧别名`delta_t_runner_queue_ms`在当前代码等同最后一项，**不是消息在executor queue中等待的测量**。缺失/无效时间在某些分支填0，不能把0当零延迟；本表R6 ACTIVE行相关数据为正。`runner_wake_lateness_ms`还有负值：R7A全部1163个ACTIVE行为负，P50≈−5.308 ms，反映代码的expected-tick参考相位定义，不能当“负计算/传输时延”。[CODE][CALCULATED]

### C.4 时间对应关系能建立到哪一层

| 关联 | 结论 |
|---|---|
| Base快照→同tick计算的Cartesian姿态 | **PARTIAL**：同一控制器行可对应；未记录实际输入消息ID和计算/输出独立时间 |
| Cartesian→IK→Shaper→Parent接收 | **数据不足**：数组丢失上游header/统一seq；不能仅按最近时间匹配断言因果 |
| Parent接收→Child周期→主机逐轴写入 | **FOUND**：R5/R6可用seq连接，结果见C.1 |
| Child反馈→Bridge→runner观察 | **PARTIAL**：R6有timed_fb_seq和代理分段；它是反馈通路而非IMU输入通路 |
| TX→真实平台运动→Platform原始消息 | **数据不足**：当前相关记录缺少统一设备时间、独立运动参考与完整原始Platform消息序列 |

数值相同的步数可能重复发送/持值，且有Shaper、slew和异步采样；数值匹配不能替代来源ID。把同一CSV行的`cmd_roll_deg`与`platform_rel_roll_deg`直接相减时间，也不会产生command-to-response delay。[CODE][INFERENCE]

## D. 无法可靠计算的延迟

| 目标量 | 结论 | 具体缺口 |
|---|---|---|
| Base设备采样→H30姿态输出 | **UNKNOWN** | 无设备采样时间、内部融合/滤波时延及原始采样率证据 |
| H30 publish→controller回调（逐帧） | **UNKNOWN于主记录** | controller只存最新quaternion/age，原IMU header、逐帧接收历史和seq未保存 |
| sensor-to-command delay | **数据不足，UNKNOWN** | 缺采样时刻、完整输入消息关联、tick计算结束/Cartesian publish、IK/Shaper来源传播 |
| 姿态控制器纯计算时间 | **UNKNOWN** | CSV时间为tick开头；`console_flush_dur_ms`仅输出日志耗时，不是控制计算总耗时 |
| 主机write返回→电机接受/内部执行 | **UNKNOWN** | 未记录线上/设备执行时间；host write时间不能替代 |
| command-to-platform-response delay | **数据不足，UNKNOWN** | 没有当前移动基座链下的独立且带时间的物理响应参考/充分激励；Platform世界姿态同时直接受Base运动影响 |
| overall effective delay | **数据不足，UNKNOWN** | 前后传感链、执行动态与软件来源关联未闭合，且有效相位可能随频率变化 |
| 完整闭环Bode/相位裕度/频率相关纯延迟 | **UNKNOWN** | 闭环剩余误差谱不等于开环传递函数；没有足够且可归因的宽频/多频激励与一致测量链 |

**不能补出的部分：** 不把50 Hz写成20 ms总延迟；不把FK 160 ms lag加入姿态闭环；不把100 ms freshness门限、30 ms串口timeout当实测延迟；不把driver feedback age当机械跟随时间；不把互相关峰值自动当物理传播时间。

当前姿态反馈使用Platform IMU，FK是带滞后的观测支路，未作为J11/J13姿态误差反馈。FK固定lag只说明“为重建过去的关节状态而等待”，**不能由此判定姿态控制有160 ms延迟**。[CODE]

## E. 可用于频率响应分析的数据

### E.1 实际找到的激励与没有找到的记录

| 类别 | 发现 | 能支持什么 |
|---|---|---|
| J11/J13/J14人工扰动 | 多组slow/gentle/manual/phaseB/repeat，R1/R2等有连续Base/Platform快照 | 探索性低频谱、重复手动试次描述；不是相同扰动的重复实现 |
| 明确固定频率、幅值与轴隔离的Base正弦/扫频 | 所查相关展开记录未找到具备这些元数据和原始多流数据的合格集合 | 当前不能建立可靠多频移动基座幅相曲线 |
| “sweep” | J11文档多为Kp/Kd参数扫值；早期通信文档为timer/poll rate测试 | 不是基座扰动频率扫频 |
| Mock周期IMU | `j14_demo_mock_imu.py`有0.25 Hz正弦，相关Mock验证存在 | 软件测试，排除出物理频响证据 |
| 上一份预研的正弦Test C | 数学IK计算 | 不是历史真实频响实验 |
| 旧J8 E3代码 | C9定义0.1/0.2/0.5 Hz等正弦、输出JSON可选且默认不保存 | 代码存在不证明数据保存；本次未找到可复核的对应多频结果集合 |
| 旧J8 R10 | 实际保存了名义0.1 Hz、2周期的组合6DOF参考与关节/FK | 有单频跟随相位线索；无双IMU、无移动基座扰动链，不能当本题多频实机证据 |

### E.2 J11人工扰动的探索性谱：确实有可分析信息，但不能换成纯时延

对R1全部5929条ACTIVE快照，保留118.579807 s全区间；按实际tick时间线性插值到20 ms均匀网格，仅用于频谱估计，未改原数据。使用SciPy Welch/CSD：fs=50 Hz、Hann窗500点（10 s）、overlap250点、每窗去常数、nfft=500，频率间隔0.1 Hz。

输入`b=base_rel_axis_deg`，输出`y=platform_rel_axis_deg`（相对启动世界姿态的**剩余运动**）。定义

\[
\widehat S(f)=P_{by}(f)/P_{bb}(f),\qquad
\gamma^2(f)=|P_{by}|^2/(P_{bb}P_{yy}).
\]

SciPy `csd(b,y)`采用共轭输入乘输出；正相位表示按该信号定义输出超前。幅值无量纲（deg/deg）。以下是**记录信号的探索性估计，不是已辨识的物理LTI频响或纯延迟**。[CALCULATED][INFERENCE]

| 轴 | f Hz | 幅值比 | 相位° | coherence |
|---|---:|---:|---:|---:|
| Roll | 0.1 | 0.29678 | +82.97 | 0.9128 |
| Roll | 0.2 | 0.37617 | +83.93 | 0.9371 |
| Roll | 0.3 | 0.50128 | +79.17 | 0.9067 |
| Pitch | 0.1 | 0.23757 | +79.68 | 0.7865 |
| Pitch | 0.2 | 0.40955 | +80.31 | 0.9162 |
| Pitch | 0.3 | 0.55942 | +73.12 | 0.9411 |

输入低频能量明显，因而不能说历史数据“完全不能做谱分析”。但0.1 Hz在每窗只有1周期，0.2/0.3只有2/3周期，受泄漏和非平稳影响；人为激励的谱峰不能当作设定且稳定的正弦频率。Yaw在这些低频的coherence仅约0.002…0.053，不报告其相位为可靠结果。

稳定性检查：R1 Roll 0.2 Hz前/后半段幅值比约0.4464/0.3634，相位80.84°/84.27°；Pitch 0.3 Hz前/后为0.5116/0.6052、78.83°/67.54°。R5 Roll 0.2 Hz前/后为0.3750/0.7058、73.43°/56.58°，变化更大。没有据此估计置信区间或LTI参数；高coherence也不保证线性、无混杂或单轴可辨识。R1 Base Roll/Pitch在0.2 Hz之间coherence约0.5686，多轴输入并非互相独立。[CALCULATED]

**为什么不能把+80°换成“负时延”：** 小角度单轴示意中，世界平台姿态`y=b+r`，前馈命令`c=−b`，若相对平台运动`r=Hc`，则`y=(1−H)b`。即使H含正时延，`1−H`也可有正的剩余误差相位。若假设`H=e^(−jωτ)`，低频`1−H≈jωτ`，相位接近+90°。这只是说明剩余误差传递与执行跟随传递不同，**没有用该简化式拟合本机τ**。[INFERENCE]

因此，R1可作为后续记录方案与分析方法的探索材料；不能仅用`−phase/(2πf)`从本表得出overall effective delay，更不能由两个controller在不同人工输入下的谱比证明优劣。

### E.3 补充旧J8单频数据：可估相位等效时间，不能当当前总时延

R10实存target6390行、command6909行、feedback6901行、FK6900行，另有18条stage事件。6DOF阶段host时间`17137.694616…17158.826261 s`，跨度21.131645 s。生成代码按循环步数产生名义0.1 Hz两周期并sleep，实际host时间变长；**不能直接以名义0.1 Hz拟合**。[CODE][RESULT]

本次取阶段起点后2.5 s至终点前2.5 s，共16.131645 s；target763行，FK807行。根据target Roll实际host接收时间，在0.08…0.12 Hz内最小化正弦回归残差，得到频率约**0.094396 Hz**。各轴按`a+b sin(2πft)+c cos(2πft)`独立拟合：target用记录器host接收时间，FK使用其**query_monotonic_timestamp_ns**，而非稍后的收到/发布时间。

| 轴 | FK/target基频幅值比 | FK query对齐相位° | 相位等效时间 `−φ/(360f)` ms | 若误用FK收到时间，相位° |
|---|---:|---:|---:|---:|
| Roll | 1.0151 | −8.187 | 240.9 | −13.716 |
| Pitch | 1.0058 | −7.485 | 220.3 | −13.014 |
| Yaw | 0.9581 | −8.812 | 259.3 | −14.343 |

target各轴回归R²约0.99993以上，FK分别约0.99736/0.99613/0.97309；高拟合度仅说明这段信号接近该单频形状。改变两端裁切为2.0/3.5 s，Roll相位等效时间约239.2/240.7 ms，Pitch225.1/220.3 ms，Yaw250.8/278.8 ms；这是一项窗口敏感性检查，**不是统计置信区间**。[CALCULATED]

FK收到时间减query时间的P50约162.520 ms。表中约5.53°相位差与这个观测等待有关；使用query时间避免把它算进运动跟随相位。**这不是从姿态控制环中“扣除160 ms”，因为该旧记录本来就是目标→反馈FK的观测。**

本表只能称**旧会话、指定组合轨迹下的单频模型反馈跟随相位及其等效时间**，不能称“本机固定纯延迟约240 ms”：

1. 无双IMU，不能建立Base感知→controller→平台物理姿态完整链；FK是编码器/模型重建，不是独立姿态真值。
2. 只有约两周期，中心拟合区间约1.52周期；不足以验证频率依赖、稳态、因果纯延迟或统计重复性。
3. 六个参考同时含同频sin/cos，彼此相关，不能把每轴幅相比视为隔离的六轴传递函数。
4. 数字滤波、Shaper、限速、位置跟随、接收时间、FK插值/模型误差都可能进入相位；单频相位无法把这些与纯transport delay分离。
5. 这是2026-09-16旧路径，不是当前J14 qualification；未记录的基座状态也不能据此认定为已测“严格静止”。

所以：**有可用于单频跟随相位探索的历史数据；没有足以认证当前移动基座姿态闭环完整频响/总时延的数据。**

## F. 能否支持“时延是主要性能瓶颈”

**不能。当前结论：UNKNOWN。** 已有证据证明某些旧J13/运行时会话有严重反馈老化、通信/调度中断，时序问题确实可能妨碍控制。但“存在时序问题”“某次因时序故障停止”“正常动态精度主要由时延限制”是不同结论。

| 证据 | 支持的判断 | 不支持的扩大结论 |
|---|---|---|
| R3最终scheduler_deadline_exceeded，反馈age可超过1 s | 该会话存在实际运行中断，相关实时链是明确工程问题 | 不能由此辨识物理闭环τ，或认定其他正常会话误差也由同一原因主导 |
| R5/R6有完整Child计时，部分目标到host写入为十几至二十几ms | 可量化驱动部分的调度/IPC/写入时间 | 不能推定sensor→platform总时延即这些值 |
| R1剩余误差谱有显著低频相位 | 补偿链存在频率相关动态效果的线索 | 不能唯一分离纯时延、Shaper、限速、模型和传感因素 |
| R10单频跟随相位 | 旧路径具有可计算的表观相位滞后 | 不能证明当前移动基座总延迟，更不能证明它主导性能 |

一个具体的替代解释来自现有代码，而非猜测故障根因：C7的参考整形为三阶有状态滤波并含速度/加速度/jerk饱和。若只在**连续、未饱和、固定目标输入映射的线性模型**中讨论，单关节近似

\[
H_{shape}(s)=\frac{\omega_n^3}{(s+\omega_n)^3},\qquad
\phi_{shape}(\omega)=-3\arctan(\omega/\omega_n).
\]

配置`ω_n=20 rad/s`，低频相位等效时间趋近`3/ω_n=0.150 s`。**这是从源码得到的理想模型推导，不是实测150 ms时延**；离散实现、限速和Cartesian/关节映射会改变实际效果。它说明表观滞后不一定来自通信纯延迟，不能仅用相位拟合强迫得到一个transport τ。[CODE][CONFIG][INFERENCE]

此外还没有统一控制的扰动幅频/方向、负载、姿态、初态、饱和占用、传感器滤波/漂移、日志覆盖及独立真值。现有数据没有完成“其他条件相同，只改变有效时延”的因果比较，也没有完成足够可信的模型分解。不能将J14修复前后的非同源人工记录当成此类对照。[RESULT][UNKNOWN]

D1中“near-zero phase lag”“Healthy sensor latency”等文字，缺少本任务所需的采样/输出事件定义和定量辨识，不能直接用作SCI结论；本报告采用实际字段语义和上述限定计算。

## G. 对第一篇候选方向的建议

### G.1 当前可以写与不能写的研究依据

**可以写：** 本平台已有三轴姿态前馈/反馈与位置伺服工程基础；控制器、Driver Parent、Child、主机写入及反馈具有部分可追溯时间；既有记录显示约50 Hz工作节奏和特定运行时长尾；人工扰动记录存在低频谱信息；旧周期参考有单频跟随相位的探索价值。[CODE][RESULT][CALCULATED]

**现在不能写：** 已测得完整感知—执行总延迟；双IMU采样同步/真实采样率已知；总延迟恒定20/160/240 ms；时延已被证明是主要瓶颈；已获得当前移动基座系统可靠多频Bode图、相位裕度、纯延迟模型或时延补偿方法的性能证明。[UNKNOWN]

**建议保留3DOF姿态研究边界，把时序可观测性与执行动态作为需要明确的工程条件，暂不把“时延主导性能”作为第一篇论文的既定前提。** 本审计不足以单独把第一篇改定为时延补偿论文，也不构成放弃3DOF或改做完整6DOF的理由。

若继续审查“时延可行性”候选问题，首先需要定义所谓时延究竟是：设备采样到输出、主机事件差、纯transport dead time，还是指定频率下的相位等效时间。不同定义要求不同数据，不能互相替换。

### G.2 可靠判断所缺的最小证据（仅列需求，不执行新实验）

- 同一会话的两路原始IMU姿态/gyro/有效标记，保存原header及host接收时间；明确设备采样/内部滤波时间可否获得，拿不到就保持该部分UNKNOWN。
- controller读取的输入消息身份、tick开始、输出计算/实际publish时刻；Cartesian→IK→Shaper→Driver的来源关联，区分预整形、已整形、最终TX和实际反馈。
- 同一epoch、可确认覆盖完整窗口的命令/反馈时序；明确批次ingest和逐轴物理采样的差别，不把默认零/持值当新测量。
- 足够持续且可归因的周期/宽频输入及相应响应；若想识别物理总延迟，还需明确传感输出本身延迟，必要时使用独立的带时间运动参考。现有IMU闭环自测不能自动给绝对物理响应时间。
- 相同工况下的重复性、非平稳/多轴相关/饱和检查，以及区分滤波、限速与纯延迟的证据。

以上是数据缺口，不是新增保护机制、算法开发任务或硬件运行授权；本次没有实施。

### G.3 离线复核口径与最终裁决

C节用原始JSON/CSV和源码事件定义：`numpy.percentile(...,[50,95,99,100])`；各会话分别计算，不拼接不同boot/epoch，不对缺失事件补值。B/C取ACTIVE状态时排除未就绪的1e9 ms sentinel；没有剔除ACTIVE长尾。E.2线性重采样只生成内存数组，不能恢复丢失/覆盖的IMU消息。E.3用实际记录时间拟合频率并明确query/receive两种FK时间。所有新增数值是离线估计，原始数据未改写。

| 最终科研问题 | 裁决 |
|---|---|
| 现有J11/J13/J14数据能否确定完整感知—计算—执行有效时延？ | **数据不足，UNKNOWN。** 可确定若干host分段和callback年龄，完整物理时延未闭合。 |
| 有无可用于频率/相位分析的历史记录？ | **有，有限可用。** J11人工扰动可做低频剩余误差谱；旧J8可做指定轨迹单频跟随相位探索。 |
| 能否据此给当前移动基座的可靠多频频响或纯延迟？ | **不能，UNKNOWN。** 缺充分激励、设备时间及跨阶段因果关联。 |
| 能否判断时延是主要性能瓶颈？ | **不能，UNKNOWN。** 时序故障有证据，正常精度的主导原因尚不能归因。 |
| 对第一篇方向的处置 | **保留3DOF问题范围；不预设时延主导，不据本次探索结果提前确定时延补偿算法或创新点。** |

**交付：仅本Markdown报告。未修改生产代码、配置、历史结果或上一份报告；未运行硬件或发送任何控制指令；未commit；`hardware_write_count = 0`。**
