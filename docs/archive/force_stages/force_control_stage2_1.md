> 历史归档：原路径 `docs/force_control_stage2_1.md`。正文保留当时的方案、参数及证据语境；当前原生 Mode 3 入口以[项目 README](../../../README.md)为准。历史命令不作为当前推荐启动命令。

# Stage 2.1：六轴真实力反馈数据链

本阶段完成软件数据链修复和离线验证。没有连接串口、执行实际 Pause、进入 Mode 3 或发送真实 0x27；`hardware_write_count = 0`。PID 算法、双向分配和 Torque 一阶 Rate Limiter 未改动。

## 原始解析与丢失位置

`pus6_motor_driver/protocol.py::unpack_status_response()` 已正确解析力：状态 payload 的 `[6:8]`，即完整帧的字节 `[13:15]`，小端、有符号 16 位；`force_adc` 是 payload `[8:10]` 的独立无符号字段。0x30 状态应答和正常 0x32 写寄存器应答均有这两个字段。

本地 LAF V2.0.4cn 手册第 3.3、3.4 节规定：0x2C 实际受力单位为 g，0x2D 原始 ADC 范围 0–4095。力协议表示范围为 −32768…32767；这不是物理可用力范围。保留符号；SI 换算仍经过现有 `ForceCompensation`，名义 1 gf = 0.00980665 N，实际增益、零点与物理方向仍须标定。

修复前 Force 当前源码中确实存在三处丢失：

1. `FeedbackSample` 只保留位置、电流和故障，没有力、ADC 和温度。
2. `ServoCore.update()` 与反馈 IPC 包没有这些字段。
3. Bridge/兼容 Parent 发布 `TimedJointFeedback` 时未赋值 `forces_g_raw`，导致消息呈现默认六个零。

此前节点还把 Fixed-Lag 位置与最新力值混用；原文档中的“已物理时间对齐”描述不成立。

## 修改文件与最终链路

| 文件（相对 bone_ws/src） | 修改 |
|---|---|
| `pus6_msgs/msg/TimedJointFeedback.msg` | 增加 `source`、逐轴 `force_valid`、`force_adc_raw`；保留原始力、位置、故障和时间字段 |
| `pus6_motor_driver/.../j7_six_axis_servo_transport.py` | 为每批收到的状态记录主机 drain 完成时间，逐轴随 audit 传递 |
| `pus6_motor_driver/.../j7_servo_feedback_scheduler.py` | 两条状态采集路径均保留力/ADC/温度；力与位置共用同一应答的时间戳；缺轴无效 |
| `pus6_motor_driver/.../j14_process_uart.py` | Child 缓存、IPC 与故障包携带真实字段和有效性，保留旧值时保留旧时间并置无效 |
| `pus6_motor_driver/.../j14_feedback_bridge.py` | 发布完整原始字段与来源；转发 Child 故障；故障反馈保留字段但标为无效 |
| `pus6_motor_driver/.../j14_process_driver.py` | 兼容 Parent 发布路径同步补齐字段，复用 Bridge 字段填充函数 |
| `pus6_kinematics/.../state_synchronizer.py` | 扩展既有历史样本与同步结果，用同一组端点/权重插值位置和力 |
| `pus6_force_control/.../force_control_node.py` | 使用对齐力反馈，处理序号、源时间、异常/超时、停止请求、恢复及 dry-run 输出隔离 |
| `pus6_force_control/package.xml` | 补全节点实际使用的 ROS 消息、服务和 rclpy 依赖 |
| `pus6_force_control/test/test_force_feedback_chain.py` | 新增历史实测回放、协议/IPC/Bridge、实际 ROS 发布订阅、同步与故障测试 |
| `pus6_force_control/test/test_ros_node.py`, `test_stage2_1_fixes.py` | 原合成测试显式填写来源/有效标志和非未来时间，更新目标超时预期 |

厂家协议实现 `protocol.py`、`uart_bus.py`、`motor.py` 未修改。没有新增 UART 所有者或 IPC 通信架构；生产 Parent/Child/Bridge 分工保留。

## 时间对齐与有效性

同一轴的位置与力来自同一份 `StatusFeedback`。显式 0x30 查询使用该事务完成时间；原 0x29 应答路径使用收到该状态的 drain 完成时间，同一批读取内的轴可能共享时间戳。这个协议没有设备采样时间，不能把这些主机时间称作硬件同步采样。

`FixedLagJointSynchronizer` 在原逐轴缓冲中附加力值、有效标志、ADC、故障和来源。对查询时刻 t，位置 q(t) 与力 f(t) 使用完全相同的左右端点和线性插值权重，然后计算 FK、空间雅可比和广义力。结果保留源端点时间、ADC 端点和权重供追溯。插值力可能为小数 g，原始 ROS 消息中的整数力值不被覆盖。

无效端点、混合来源、过期端点、过大的历史断点和无法括住查询时刻时，不能输出力控命令。初始缓冲不足时等待；已经控制过后丢失同步则锁存故障。禁用 Fixed-Lag 时只接受六轴确实具有同一主机参考时间的消息，否则拒绝混用逐轴不同时刻的数据。

六个零只在“完整状态解析、来源已知、逐轴有效、时间有效、无故障”均成立时才是有效零读数。旧发布端没有新增字段时，消息默认为 UNKNOWN/invalid。源标记区分 MOCK 和 HARDWARE，独立于 dry-run；回放历史硬件来源不意味着当前进行了硬件采集。

## 节点故障与停止策略

- 每个递增 UART 序列最多执行一次 PID；重复反馈不能刷新看门狗。序号回退或逐轴源时间不前进会停止输出，序号跳跃仅累计丢帧数，仍须验证源时间。
- 力/ADC/位置异常、设备故障、通信故障或反馈超时：不执行 PID、不沿用旧位姿输出。
- FK 不收敛、非有限值、超出既有残差上限或分配计算异常：停止当前周期及后续输出。
- 目标超时：停止发布并请求既有 Pause，**不再将目标改为零后继续施加闭环力**。`target_timeout_s=0` 是显式禁用目标超时；正常配置默认 0.5 s。
- 发生故障后清空积分、微分/参考状态和同步历史，保持锁存。`~/reset_fault` 显式复位后等待新反馈和新目标，不自动恢复电机模式。

硬件路径需同时满足非 dry-run、写入授权和资格确认；驱动数值计算对象始终不开串口。硬件故障停止请求复用 `/pus6/pause_all`，由原 Parent 转发到 Child 的 stop/Pause。服务不可用或返回未确认时明确报告“未确认停止”，不宣称硬件已停。

dry-run 使用 `/pus6/force_control/axial_forces_dry_run` 和 `/pus6/force_control/stop_request_dry_run`；不向 `/pus6/command_axial_forces` 发布，不调用 Pause 服务。标定反馈和广义力观测仍可发布。停止传递分支只用假服务客户端验证，本阶段没有执行实际硬件 Pause。

## 数据与验证范围

实际使用的历史测量：`results/fc1_z_motion/fc1_20261101-170030/telemetry.jsonl` 前六条记录，包含六轴正负力、位置、ADC、温度、故障和逐轴接收时间。第一条力值为 `[3754,-269,-6110,825,-744,-6928]` g。FC0 的 `fc0_20261101-165755/summary.json` 另有状态力与直接 0x2C 读取的交叉检查，属于先后两次读取，不能当作同一次采样。

测试用 FC1 **已解析的真实测量字段重编码**状态帧，经过正式解析器、真实调度器、Child 字段构造、实际 AF_UNIX socket、Bridge、ROS 序列化及力控节点。它不是原始线上字节抓包。独立核对每轴插值的 q(t)、f(t)，而不是仅检查数组长度。

厂家手册 3.5.1 的零力应答示例也纳入测试：印刷校验和为 0x5F，而示例字节按当前协议重算为 0x60。保留对原示例的拒绝，另显式重建有效示例；没有放宽生产解析器。符号极限、坏校验和、超时、缺轴、故障、重复/倒退、同步丢失和 FK 失败等边界使用合成数据。既有 FakeBus/MockServoBus 测试保留。

还通过实际本地 ROS 发布/订阅验证了反馈进入节点、原始负力参与 PID，以及 dry-run 与硬件命令话题隔离。所有这些都是软件测试，不能证明当前电机在 Mode 3 下的真实力控效果。

本阶段验证结果：受影响依赖链 5 个 ROS2 包构建成功；力控核心/节点、Fixed-Lag、调度器、六轴传输、三进程 IPC 和 Bridge 共 **119 项相关测试通过**（9.28 s，测试 ROS_DOMAIN_ID=171）。仅运行软件、FakeBus/Mock 和假串口工厂，未执行真实串口写入。

## 构建与 Stage 3

沿用第一阶段的本项目构建目录：

```bash
cd /home/lyapunov/6pus_bone_drill_force_control/bone_ws
source /opt/ros/jazzy/setup.bash
colcon --log-base log/force_stage2_1 build --base-paths src \
  --build-base build/force_stage1 --install-base install/force_stage1 \
  --packages-up-to pus6_force_control
source install/force_stage1/local_setup.bash
```

消息定义已变化，Force 工作空间内相关发布端、订阅端应统一重建并使用该安装环境。不要混用 Position/Torque 的旧消息二进制。

Stage 3 仍需在现有 Child 中接入力命令、实现 Mode 3 的进入/退出、0x27 执行和应答处理，并验证真实 Pause/断联行为。还需确认物理方向、负力支持、零点/增益、重力与支撑补偿及即时硬件安全条件；随后才能获得真实闭环结果。此处没有把软件参数许可字段当作硬件验证结论。
