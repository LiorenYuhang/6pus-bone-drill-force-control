# 手动柔顺恢复：端点反馈与 Mode 3 交接

> 独立恢复入口的历史记录，不是三进程主动稳定的当前参数，也不是正常退出自动 Home；当前启停见[长期运行](long_running.md)。正文保留原安全条件和当时验证状态。

> 2026-11-05 第二轮修正：当前默认控制策略与进入/退出说明见 [手动柔顺恢复 V2](../archive/force_history/force_manual_recovery_v2.md)。下文保留此前实现与验证记录，其中默认阻尼公式已被 V2 的 native_support 替代。


2026-11-05。本次只修改程序和离线验证，未连接电机、未发送真实写入、未切换模式，`hardware_write_count = 0`。上一轮三次真实只读记录保留在 `results/manual_recovery_readonly/20261105_100841_015551/`；本次没有重读电机。

## 已修改

- **真实位置反馈**接受既有 −5～2005 steps 容差。`0xFFFF` 按协议显示 −1，原始 u16 和有符号 steps 同时保存，不把它改成 0。
- **位置命令**仍严格为 0～2000 steps。柔顺运行只发 Mode 3 的力目标，不产生自动 HOME/PARK 或回中位置轨迹。
- **退出交接**继续采用既有同帧 Mode 0＋位置目标。仅手动恢复入口启用端点容差策略：实测 −1→合法退出目标 0，实测 2004/2005→合法退出目标 2000；其它行程内位置原值锚定。记录 `measured_position_steps`、`anchor_steps`、原始反馈及端点目标选择标志。这里的目标选择不是修改测量；最多 5 steps 的向内目标差也不是保证完全无运动。
- 普通控制默认仍按原严格锚定策略，未将反馈容差扩大为普通位置命令范围。小于 −5 或大于 2005 仍中止，并保留原始反馈。
- ID1、ID6 所处高端区不增加正向力增量；低端区不增加负向力增量。保留实测 `F_hold`，不是把绝对目标统一清成零。持续向外超过既有 2-step 波动容限、超过反馈边界、速度/电流/力/通信异常时进入已有终止处理。限速后的实际编码增量也检查方向，避免请求已安全但旧输出仍指向边界。
- 保留 500 g/s、每轴 ±14.709975 N、手动速度 2 mm/s、电流 200 mA、松键超时 0.75 s、最长 120 s；不修改 Position/Torque、不重复 Tare。

## Mode 3 初始化顺序

依据本地厂家 V2.0.4 电气手册**印刷第 23 页（PDF 第 25 页）方法二**，允许从 `0x25` 连续写入模式、输出电压占位和力目标。现有 `j7_six_axis_servo_transport.configure_force_mode()` 已实现并在 Stage 3C.2 六轴实机验证过该报文。

1. 维护模式 UART 所有者只读启动，确认六轴 Mode 0、反馈在容差内、电流/故障正常。
2. 操作员按空格开始后，再通过 `prepare_force_handover()` 逐轴读取真实 `0x2C`，保存候选 `F_hold[6]` 及实际位置、时间戳。旧 `0x27=-1 g` 只作记录，不沿用。
3. 验证六个候选保持力的有符号 int16 编码及原绝对 N 限幅；不通过时在写入前拒绝，不裁剪成另一组保持力。
4. 对 ID1～ID6 逐轴发送一个报文：起始 `0x25`，数据 `[3, 0, encode_i16(F_hold_i)]`。其中 `0x25=3`、`0x26=0`、`0x27=F_hold_i`，小端 16 位编码。
5. 每轴检查写应答，再分别读回 `0x25==3` 和 `0x27==F_hold_i`，并检查位置/电流/故障，正确后继续下一轴。六轴切换不是硬件原子操作。
6. 六轴确认后，从 `F_hold` 限速起点进入手动柔顺：`F_command = F_hold + ΔF_damping`。不启动姿态 PID 或自动位置跟踪。
7. 部分进入失败时停止后续轴，复用已有选择性 Mode 0 同帧恢复；手动入口的端点合法退出目标同样适用，不因一次 ACK 丢失重发模式写入，不额外自动 Pause。

本次不依赖“Mode 0 先单写 0x27、再写 0x25 后是否保留”的未验证固件行为，也不采用先零力切换再补保持力的方法。

用户给出的当前力 `[-394,143,-300,-580,247,-480] g` 均在现有 ±1500 g 对应的绝对软件限幅内，但**仅为准备信息**；真正执行时仍重新读取，不从本报告或历史 JSON 装入保持目标。

## 启动方式

离线检查（不会打开串口）：

```bash
cd /home/lyapunov/6pus_bone_drill_force_control
source /opt/ros/jazzy/setup.bash
source bone_ws/install/setup.bash
ros2 run pus6_force_control force_manual_recovery \
  --positions 1955 350 122 232 934 2004
```

该组反馈现在可进入手动恢复输入检查；实测保持为 2004，退出目标若仍在该位置则为 2000。FK 只用于记录/辅助分析，不能因 FK 无法生成自动轨迹而伪造当前位置；没有自动执行候选轨迹。

**下一轮现场执行命令，本次未运行：**

```bash
ros2 run pus6_force_control force_manual_recovery \
  --config bone_ws/src/pus6_force_control/config/manual_recovery.json \
  --execute --no-dry-run --hardware-write-enabled --hardware-qualified
```

仍使用现有串口独占锁及 UART 单一所有者。运行前正常控制 Child 应已退出；不终止或抢占其它串口所有者。三个既有执行条件独立保留，软件测试不授予真机资格。

提示出现后按住空格键，程序完成 Mode 3 交接后再人工调整整个有独立支撑的动平台。靠近上限的轴向内调整。松开空格超过 0.75 s、按 `F` 完成、按 `Q`/Ctrl+C 停止均进入退出交接。不要将键盘作为独立硬件急停；进程被强杀或总线失效仍需独立硬件停止。

数据保存到 `results/manual_recovery/<时间>/`：原始 TX/RX、实测位置、原始力、保持/目标力、电流、故障、逐轴进入/退出状态和最终实测位置。退出后确认 Mode 0 位置目标、实际位置及观察窗口，不能用关闭串口或 Pause ACK 冒充停止成功。

## 验证边界与文件

主要改动：`manual_recovery.py`、驱动新增 `position_feedback.py`、`j7_six_axis_servo_transport.py`、`j14_process_uart.py`、`force_lifecycle.py`；针对性测试覆盖端点反馈不变、目标合法、保持力同帧写入、部分切换失败及靠近端部的力增量方向。协议冻结文件、PID、Tare 和历史真实数据未修改。

本轮 51 项联合测试通过；另 19 项既有生命周期用例在前一批测试通过；`pus6_motor_driver` 和 `pus6_force_control` 两包构建成功。已通过安装后的 ROS 入口离线运行用户给定六轴位置，反馈 2004 原样保留，合法退出目标单独为 2000。

测试/构建记录见 `results/manual_recovery_preparation/20261105_endpoint_tolerance/`。这证明软件接口与报文逻辑，不代表新柔顺策略或端点退出已经真机通过。


## 启动接口修复（2026-11-05）

真实启动 `results/manual_recovery/20261105_102623/` 在首个状态读取后报 `StatusFeedback.timeout` 不存在，该次记录写入数为 0。根因是将调度器 `FeedbackSample` 的 `timeout`、`force_valid` 字段用于 `Motor.read_status()` 返回的协议 `StatusFeedback`。后者只有目标位置、实际位置、电流、力、ADC、温度和故障码；读取超时/损坏帧通过异常报告。

手动入口现在直接按实际字段进行位置、故障、电流、力/ADC 检查；保留部分读取与异常，禁止将缺帧当作成功。正常调度器/ROS 消息的有效性字段仍保留，没有删除它们的检查。新回归使用实际 Motor/协议解析器，并通过 FakeBus 走完整入口，不只测试控制数学。

`hardware_stop_confirmed=false` 不等于一定需要独立停止：完整读到六轴 Mode 0 且尚未进入 Mode 3 时，报告 `NO_MODE3_ENTRY_MODE0_OBSERVED`，不冒称执行过硬件停车；模式未知、读取不完整或 Mode 3 终止未确认时，`independent_stop_required=true`。终止已确认时采用 UART 所有者的结果；Mock 的位置退出观察不会成为真实硬件停车通过。终止/关闭错误单独记录，不覆盖原启动异常。

修复只涉及 Force 的手动入口及回归测试；Mode 3 初始化、保持力、Pause、Mode 0 交接和限幅未重新设计。重新构建后仍使用上文同一启动命令。本次没有连接电机或写入真实硬件。

本次启动修复验证：33 项入口/协议/端点回归与 20 项生命周期测试通过，`pus6_force_control` 重建成功。记录：`results/manual_recovery_preparation/20261105_status_feedback_fix/software_validation.json`。
