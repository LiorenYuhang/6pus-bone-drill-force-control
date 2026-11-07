> 历史归档：原路径 `docs/research/6PUS_DS_SCTDE_PARAMETER_AND_SIGNAL_INVENTORY.md`。正文保留当时的方案、参数及证据语境；当前原生 Mode 3 入口以[项目 README](../../../README.md)为准。历史命令不作为当前推荐启动命令。

# 6PUS 参数与信号证据清单

2026-10-28；以当前力矩研究工作树中的源码和手册为依据。CONFIRMED 表示源码/文件中可直接核实，不自动表示已现场标定；UNKNOWN 不是零。用途均只指研究软件，不表示硬件可用。

| 名称 | 物理量 / 单位 | 具体来源 | 可信度与当前用途 | 未确认的影响 |
|---|---|---|---|---|
| base / platform anchors | 两组安装点，基座/平台系 m，六轴顺序 | `bone_ws/src/pus6_kinematics/config/6pus_geometry.yaml`；`analytical_6pus.py` | CONFIRMED 软件模型；YAML 显式标记 final assembly confirmation pending | 实际装配误差、偏心与柔性未确定；模型雅可比不能视为实测 |
| 杆长 0.084、半径 0.040 / 0.0275 | m | 同 YAML geometry、`hardware/调试工具/Kinematics.zip` 的模型来源 | CONFIRMED 配置值，不是本次物理测量 | 参数误差会影响 q、J、力映射 |
| ZYX、固定平移 | 姿态 roll/pitch/yaw rad 与工作参考 | `analytical_6pus.py:inverse_kinematics`；`j11_base_feedforward_runner.py` | CONFIRMED 软件坐标；固定平移是本次研究假设 | 六自由度机构平移不由此保证受到约束 |
| HOME、步数/行程映射 | 1000 steps、0.050 m、0.000025 m/step | geometry YAML actuator；`j14_feedback_bridge.py` | CONFIRMED 软件变换；数值以反馈为输入 | 不等同外部长度真值或绝对装配零点验收 |
| 实际滑块位置 | 6×m，主机接收完成单调时钟 ns | `pus6_msgs/msg/TimedJointFeedback.msg`、`j14_process_uart.py:update`、`j14_feedback_bridge.py` | PARTIALLY_CONFIRMED 有 host RX completion（不是采样时间）；各轴异步且可能保持旧值 | TDE 需要同一物理时刻，不能假定六轴/IMU 已同步 |
| 每轴滑块速度 | m/s | 上述 TimedJointFeedback 无速度字段；`j7_servo_feedback_scheduler.py:FeedbackSample` 无速度 | UNKNOWN 可信在线速度；本研究用有延迟差分和滤波演示 | 25 μm 量化、异步和噪声可放大导数；仿真估计未硬件标定 |
| 双 IMU 姿态和角速度 | quaternion / rad/s | `bone_ws/src/h30_imu/src/h30_imu_node.cpp` 使用 `now()` stamp，gyro deg/s 转 rad/s | PARTIALLY_CONFIRMED 字段和转换；基座、平台的安装/时钟必须另核对 | 不直接提供同步的基座系 Euler 坐标导数 |
| 角加速度 | rad/s² | H30 Imu 消息中的 `linear_acceleration` 为线加速度；现有研究缺少验证的实时角加速度估计 | UNKNOWN；本研究用姿态连续差分产生假设估计 | 噪声、姿态包络、差分和对齐误差进入 TDE |
| 控制周期 | 50 Hz 配置 | `production_driver.py:THREE_DEFAULTS`、`j11_m2_pid_params.yaml` | CONFIRMED 配置值；非实测周期 | 不证明驱动、传感器、控制或反馈闭环稳定 50 Hz |
| 电流 / 轴向力 | mA / 手册 g | `protocol.py`、LAF 手册寄存器 0x2B/0x2C；`FeedbackSample`、`j14_process_uart.py`、桥 | 电流有字段；力字段虽在消息定义中，所查 J14 桥未赋值，故不能将默认零视为测量 | 电流到轴力矩标定、力反馈采样/符号/时间未证实，不能重构任务输入 |
| τ实际、传动 n、摩擦及驱动动态 | N·m、m/rad、N / N·m 和秒 | LAF 电气手册 0x25–0x2C，工程驱动 | UNKNOWN；手册列位置/伺服/速度/力/电压模式，未见已确认的电机轴力矩指令 | 不能驱动真实 τ 或假设力命令、电压、电流等同电机轴力矩 |
| 型号 | LAF50-024D | `hardware/tempp/因时电机ModbusRTU多电机控制方案.md` | PARTIALLY_CONFIRMED 文档方案写明该型号；本次没有读取实体铭牌 | 模型和固件不一致时寄存器/额定限制不可套用 |
| 模型 inertia、弹簧、阻尼、滞后及 ±误差 | kg、N/m、N s/m、s | `scripts/research/ds_sctde_realistic/realistic_simulation.py`、结果 `conditions.json` | ASSUMED 仅合成数值；±2%、±30%、2× 是灵敏度试探，不是可信区间 | 不能用于真机整定、稳定性或安全限值 |
| 实施的任务力估计 | `Jᵀ[K_model(q−IK(η))+B_model(q̇−Jη̇)]` N·m | 同研究脚本 `modeled_wrench` | 假设弹簧/阻尼已知且可同步观测 q/η；仿真独立实现，非论文原式，也不是已知可在当前硬件使用的观测器 | 缺少标定的 K/B、结构约束、真实时间与模型质量；误差不等于零 |
