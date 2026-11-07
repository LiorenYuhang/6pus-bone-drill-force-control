> 历史归档：原路径 `docs/mode3_dual_imu_three_process.md`。正文保留当时的方案、参数及证据语境；当前原生 Mode 3 入口以[项目 README](../../../README.md)为准。历史命令不作为当前推荐启动命令。

# Mode 3 双 IMU：J14 三进程运行入口

新增入口：`bone_ws/scripts/run_mode3_dual_imu_three_process.sh`。复用 Force 中继承并适配的 J14 `ProcessClient`、`child_run`、UART transport 和 Feedback Bridge；不另建工程。原双 IMU 入口及 Z／单轴／组合轨迹入口保留，历史数据不改写。Position/Torque 工程未修改。

## 运行分工

```text
Parent：双 IMU接收与主机时间对齐 → 原 SO(3)/LADRC → 原 IK
        ↓ 最新完整位置参考（q、姿态、IMU时间、序号、诊断），非力指令FIFO
UART Child：真实六轴反馈 → 原位置 PI-D → 保持力偏置 → 原限幅/量化 → Mode 3 / 0x27
        ↓ J14独立反馈socket；UART不等待ROS发布、FK或文件记录
Feedback Bridge：原ROS反馈发布 + 异步FK/完整tracking记录
        ↓ 健康/阶段状态经Child转发给Parent；与电机反馈通道隔离
```

三进程指电机运行架构；两路现有H30节点仍是传感器进程。UART Child不导入ROS，不运行FK；Parent不打开电机串口。Bridge是反馈socket的唯一消费者，完整运动记录与FK由其有界工作队列处理。Mock Bridge使用独立DDS域223，避免向实际实验域发布模拟反馈。

适配文件：

- `pus6_force_control/mode3_process_runtime.py`：Parent阶段与姿态/IK；复用ProcessClient启动、握手、停止和回收进程。
- `pus6_force_control/mode3_process_child.py`：ServoCore子类；六轴位置PI-D、Mode 3交接、保护、分摊寄存器读回和原终端恢复。
- `pus6_force_control/mode3_process_bridge.py`：继承原Feedback Bridge；FK、终点保持判据和tracking记录移出UART循环。
- `pus6_force_control/process_recording.py`：有界异步文件写入及SerialTrace适配。
- `pus6_motor_driver/j14_process_uart.py`：仅增加该入口的factory、joint-reference消息及Bridge健康转发；其他运行路径不启用这些分支。
- `mode3_dual_imu.py`：增加`--three-process`选择；新脚本默认选择此后端。

## 控制参数与边界

PID仍来自现有`pid_config(DualImuPlan)`：Kp/Ki/Kd为8000/6000/1000（SI），编码器积分死区1 step；预测P、80 ms连续速度回归及40 ms D滤波不变。2026-11-07输出策略更新仅在三进程入口取消积分±2.5 N、增量±4 N及2.5 mm跟踪硬退出，最终目标力使用厂家LAF50-024D的50 N最大推拉力规格边界，并保留最终输出饱和的条件抗积分饱和。完整依据与回放见 [原生能力输出策略](../../control/native_capacity.md)。一个PID实例跨initial_hold、stabilize和final_hold，切换不重置积分或速度。

参考位置直接使用最新IK；正常力指令不增加斜率整形，**没有恢复0.2 mm/s或500 g/s**。5000 g/s只保留在原IMU故障/操作停止的健康反馈偏置恢复路径；机械越界、通信等故障仍直接走原Pause/Mode 0处理。LADRC代码、参数和SO(3)目标未修改。

电流继续沿用Position/Torque共享1500 mA基线；三进程入口不再使用1500 g反馈硬退出或2.5 mm跟踪硬退出。目标与实际位置使用原几何/设备边界交集，通常200–1800 steps；不增加角度限制或工作空间内框。最终目标为保持偏置加PI-D，按厂家50 N规格编码，整数g边界为±5098 g；不能把该主机边界称为已验证的固件50 N自动保护。启动仅读回0x20/0x21/0x22保存硬件保护设置，不改写。IMU与实际反馈原时效检查保持；Force维护RPC超时仍为50 ms，TX超时仍为30 ms，运动写入不重试。

模式与0x27读回由一次12个同步请求改为分摊的逐轴单寄存器查询，保持一秒级完整轮询，每周期最多一个请求。每次记录电机ID、寄存器、预期整数值、真实读值和时间；发现模式/设定值不符仍停止。不同周期读到的六个力设定值不再冒充“同一周期完整向量”。

## IPC与停止

复用J14非阻塞AF_UNIX/SOCK_SEQPACKET、16 KiB消息上限、每轮最多64包、最新完整向量、epoch/单调序号及握手。接收积压未清空时继续用上次已接纳且未过期的参考计算本地PI-D，不执行中间排队目标。Parent只发送IK位置参考，**不存在跨进程排队执行旧力指令**。

Parent Lease为原1000 ms；引用现有200 ms目标新鲜度，Lease续期不能刷新目标时间。Bridge健康复用1000 ms边界。Parent通道EOF、Bridge退出/失联、目标过期或原控制保护故障，由UART Child本地执行退出恢复。Child本身被SIGKILL/系统故障终止时不能再发停止指令；Parent明确报告`exit_unconfirmed`，不宣称Mode 0成功，也不另外打开串口自动接管。真实硬件仍需要可独立于软件工作的停止手段；本轮未改变任何硬件设置。

Mode 3 切换完成后先等待**确认完成之后发布**的完整 IMU/IK 参考，再开始 PI-D；切换期间排队的旧参考不下发，也不修改其时间戳。此等待使用原200 ms时限，从切换确认完成计时；超时仍走原故障停止。等待期间保持已写入的保持力偏置，继续读取实际位置、电流和故障状态。正常运行仍按真实参考发布时间判断200 ms时效，独立于Lease；恢复正常参考后不会重新开启启动等待。Parent的固定1秒initial_hold从这次参考就绪开始计时。沿用J14的ready/start、单槽覆盖及有界接收，不增加运动限制。

新记录中：

- `command_seq`：本周期实际使用的Parent参考序号，可跨多个UART周期重复。
- `reference_timestamp_ns`：该参考生成时间。
- `uart_cycle_sequence`：本次六轴力事务序号，从0开始。
- `pid_input_cycle_seq`：产生PID输入反馈的上一事务序号；首次为−1（初始化反馈）。
- `pid_input_rx_ns` / `feedback_rx_ns`：PID输入和本次新反馈的六轴主机接收时间；不冒充设备同步采样。
- `reference_observed_uart_cycle_seq`：Parent生成参考时观察到的反馈序号。
- `mode3_start_ns` / `reference_ready_ns`：硬件模式切换确认完成、首个有效切换后参考就绪，分别记录；Child lifecycle另存切换开始及参考序号、发送/接收时间和Lease时间。Parent IPC及Bridge状态携带参考时间、序号，故障快照保存触发时参考元数据。
- 目标姿态、实际FK、目标/实际位置、PID分项、力、ADC、电流、模式和故障码继续保存。周期记录另含预期唤醒、实际唤醒及六轴TX时间。

完整串口字节与解析帧异步落盘；UART相关内存记录沿用J14 production的有界保留，完整tracking由Bridge连续记录。存储队列满时统计丢弃与序号间隙，**不因非关键记录丢失自动触发运动保护**。最终`recording_complete`独立于控制结果。旧的空stall recorder不用于声称“零卡顿”；使用tracking时序、Child health和Bridge序号统计。

## 运行与结果

纯软件入口（合成IMU、Mock电机，不打开硬件串口）：

```bash
bash /home/lyapunov/6pus_bone_drill_force_control/bone_ws/scripts/run_mode3_dual_imu_three_process.sh --dry-run --duration-seconds 30
```

实际流程仍为：实测起点/有效IMU → 六轴Mode 3确认 → 固定1秒initial_hold → 指定时长三轴stabilize → 原final_hold收敛与保持 → Mode 0。平移参考保持本次实测FK起点；终点保持最后目标，不自动HOME。

每次使用唯一目录：`results/mode3_dual_imu/dry_run_*`或`run_*`。保留原`tracking.jsonl`、`dual_imu_report.json`、`force_shutdown.json`及自动分析；新增Parent配置/故障、Child运动报告、Bridge记录完整性报告。UART故障快照仍在恢复前捕获。

本次软件验证：两包`pus6_motor_driver`、`pus6_force_control`构建成功；19项针对性测试通过。覆盖真实三个Mock进程、非HOME起点、Parent通道EOF、Bridge被终止、Child被强杀后的未确认报告、原保护、完整序号对应、积压参考处理、积分交接及慢存储有界行为。最后的记录字段完善后，相关正常进程/算法一致性2项复测通过；未执行真机。

30秒主动阶段Mock记录：`results/mode3_dual_imu/dry_run_20261107_123706_268110`。1755个完整控制周期（51初始保持、1500主动、204终点保持），序号连续，Bridge无丢包、记录无丢弃，Mode 0确认，`hardware_write_count=0`。周期中位19.9966 ms、P99 20.5377 ms、最大20.9332 ms；工作时间中位5.3858 ms、最大17.5656 ms。**这些是Mock软件调度数据，不是实机串口时延或真实稳定性证据。**

下一次真机长时间验证命令（本轮未执行）：

```bash
bash /home/lyapunov/6pus_bone_drill_force_control/bone_ws/scripts/run_mode3_dual_imu_three_process.sh --hardware --hardware-qualified --start-imus --axes roll,pitch,yaw --duration-seconds 600
```

预计主动阶段600秒，另加1秒初始保持、原终点收敛/保持及启动退出时间。继续观察回撤滞后、输出削顶、周期/序号及停止状态；必要机械、电流及通信保护仍可能提前终止。Mock或历史反馈回放不能证明之前M1回撤超差已经解决。上面600秒命令最初用于架构迁移验证，新的输出策略尚未真机验证，本轮没有执行该命令。

## 2026-11-07 启动竞态修复

实机记录 `results/mode3_dual_imu/run_20261107_131549_798122` 的首次故障快照时间为单调时钟 **187003098894623 ns**（映射串口主机时间约 **13:15:51.5196 +08:00**）。触发位置是 `JointPositionServoCore.cycle()` 的 `IK_target_timeout` 检查，非Lease失效或执行器跟踪超差。切换前首参考发送至最后一次切换反馈约255.6 ms，超过200 ms目标时效。Mode 3同步切换/确认期间Child无法接收IPC，已入socket的早期参考占满缓冲区；Parent持续生成目标、单槽覆盖待发参考，但Child恢复接收后立刻按早期参考触发过期，没有等待新参考补入。快照目标对应seq 2，发布时间可由目标请求与发送记录限定，故障时年龄约215–217 ms；旧记录未保存精确publish/receive字段，不伪造精确年龄。

Parent成功发送序号为0、1、2、13、14，13/14已在首次故障后的退出过程中；目标可达检查共45次，无不可达目标。原三进程ready/start握手已完成，但Force的Mode 3切换在其之后，原适配缺少这段UART切换与位置控制流之间的参考刷新交接。仅续Lease或放宽200 ms不能解决这个语义错误。

串口468条字节记录对应234组TX/RX。18次写事务逐帧核对为：

|阶段|次数|寄存器及实际内容|
|---|---:|---|
|进入Mode 3|6|0x25连续写入 `[3, 0, hold_g]`，包含0x27偏置；六轴为−205、93、166、856、−236、−131 g|
|故障暂停|6|0x1A = 1，各轴响应确认|
|恢复Mode 0|6|0x25起连续写入模式0与实际位置锚点，含0x27及0x29字段；未切换Position模式|

没有以0x27为起始寄存器的正常PI-D目标力写入，完整控制周期为0。六轴Mode 3进入确认成功；故障后六轴Pause、Mode 0及位置保持观察全部成功，串口关闭。`pending_cycle`和最后完整控制帧均为null，故无tracking文件。分析程序现在支持缺失/空tracking及null pending，输出 `NO_ACTIVE_SAMPLES`、原故障和退出状态，不声称稳定通过。

本次修改仅涉及Child启动交接、Parent阶段起点、Bridge/IPC参考元数据、空记录分析及相应测试。PID/LADRC、力限幅、所有原保护阈值和正常不限速/不限力斜率策略均未修改；Position/Torque未修改。

软件验证：两包构建成功；14项针对性测试通过。测试中仅在Mock Child注入300 ms切换延迟，修改前复现同一IK超时，修改后首个实际使用参考由seq 0跨至seq 16，切换后20.35 ms就绪，无旧参考执行；另验证缺少新参考即使Lease更新仍超时恢复Mode 0、正常运行的目标/IMU/跟踪保护及原PI-D输出一致性。

独立三进程Mock入口结果：`results/mode3_dual_imu/dry_run_20261107_132346_864393`，406个完整周期，initial_hold 1.019 s、stabilize 3.018 s、final_hold 4.068 s，`motion_complete`、Mode 0确认、记录完整、`hardware_write_count=0`。这些结果只证明软件启动和交接路径，未验证实机动态跟踪。历史目录保持不变；修复后对原记录的离线分析保存在 `results/mode3_dual_imu/offline_startup_analysis_20261107_132406_050393/analysis.json`，其历史写入数仍为18，分析自身硬件写入数为0。下一次实机命令沿用上面的600秒三轴命令，本轮未执行。
