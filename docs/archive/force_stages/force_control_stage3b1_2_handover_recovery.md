> 历史归档：原路径 `docs/force_control_stage3b1_2_handover_recovery.md`。正文保留当时的方案、参数及证据语境；当前原生 Mode 3 入口以[项目 README](../../../README.md)为准。历史命令不作为当前推荐启动命令。

# Stage 3B-1.2：当前承载力交接与部分切换失败恢复

日期：2026-11-03。仅修改 Force 工程；本阶段未打开真实串口、未切换真实电机模式，也未发送任何真实电机写命令。下面的执行结果均来自 FakeBus，不代表真机恢复或力控性能已经验证。

## 实验定义与初始力交接

复用 `ServoCore`、现有 UART Child 单一所有者和寄存器请求接口。新增的 `prepare_force_handover()` 逐轴读取 `0x25`、`0x2C`、`0x2A`；要求读回六轴 Mode 0。读取失败直接抛错，不用默认零值补齐。

`0x2C` 的 16 位有符号值按原始方向保存为本次 `F_baseline[6]`，单位 gf，不取绝对值、不擅自取反。逐轴力与位置各自保留主机接收完成时间；该串行快照不宣称六轴同时采样。它是当前姿态、机械载荷下的相对参考，不是传感器卸载零点，也不写硬件 Tare 寄存器。

`ForceBaselineManager` 的接口为：

- `capture_initial_state(raw_forces_gf)`：保存本次原始载荷和软件基线；不修改 `sensor_zero_gf`。
- `relative_forces_gf(raw_forces_gf)`：返回 `F_raw - F_baseline`；初始值六轴均为零。未采集基线时拒绝计算。
- `command_from_delta_gf(delta_forces_gf)`：返回 `F_baseline + delta_F`，检查六轴有限增量及有符号 16 位输出范围。

软件相对零点与电机绝对目标保持分离：初始 `F_hold = F_baseline`，后续 `F_command = F_hold + delta_F_PID`。此接口接收执行器 gf；任务空间 PID 的 N 输出仍须经过现有六轴分配、单位和方向换算，不能直接当作 gf 相加。本阶段没有修改 PID、力分配或 Torque 一阶变化率算法。

`enter_force_mode_from_current_state()` 将读取快照与交接衔接起来；Force 初始化未显式指定初始目标时也使用此读取路径，不再隐式构造六轴零目标。dry-run 必须提供明确的模拟初始向量或基线，不冒充实时硬件采集。保留已有显式目标接口供已有调用方使用；受载实验应使用当前状态入口。

初始保持目标用现有有符号编码，逐轴发送连续寄存器写 `[0x25=3, 0x26=0, 0x27=F_hold[i]]`，然后读回该轴 `0x25`。进入后无新目标的普通保持周期沿用此向量。初始载荷若超出现有 `min_force_g/max_force_g`，在任何切换写入前拒绝，防止切入时保持载荷、首个周期却被限幅为另一载荷。没有自动放宽物理限值；默认 ±1500 gf 不能覆盖历史记录中的全部载荷，真机前须明确配置适用于该机构和电机的范围。

快照含来源 `MOCK`/`HARDWARE`、逐轴寄存器读数/接收时间、基线、保持目标和初始相对力，随现有 ready 数据及 `child_final.json` 保存。历史 Stage 3B-1 常量和原始实验结果均保留，但不是本次实时基线的替代物。

## 部分切换失败：恢复位置模式，不自动 Pause

`J7SixAxisServoTransport.configure_force_mode()` 保留逐轴、非原子的切换方式：

1. 读取初始逐轴模式；顺序写入 Mode 3 与该轴保持力，并读回模式。
2. 某轴异常后立即停止后续 Mode 3 写入，不重试不确定的切换写。
3. 对已经确认 Mode 3 的轴逐轴执行：读取 `0x2A` 当前实际位置 → 写 `0x29` 位置锚点 → 写 `0x25=0` → 读回 `0x25=0`。
4. 对写入已尝试但未确认的失败轴，先读回模式。若实际为 3，同样重锚定恢复；若为 0，不做多余写入；读不出则明确标记未知。ACK 丢失不等于电机未执行。
5. 对未尝试切换的轴不写任何寄存器。最后只读盘点六轴模式；全部确认 0 且恢复步骤无异常，才报告 `recovered_mode0`。
6. 恢复某轴失败，仍尝试恢复其余已确认切入轴，最终报告 `recovery_incomplete`。每轴保留初始模式、是否尝试切换/恢复、最终读回模式、锚点和错误；读不到的最终模式用 `None`，不伪造为 0。

无论恢复是否完成，原模式切换失败都会锁存传输层及 Core 故障，拒绝后续正常指令。恢复不完整时 `reset_fault_latch()` 也拒绝把混合/未知模式声明为 Mode 0。恢复完成并不自动重启控制。

模式切换/位置回切异常使用 `latch_fault_without_pause()`；外层 `ServoCore.stop()`、Child 异常处理和 `close()` 因已有锁存而不会补发 Pause。`handover_report` 随异常及现有故障/最终记录暴露逐轴状态。独立 `pause_explicit()` 保留为明确调用的操作，仍受 dry-run、硬件写入授权、硬件资格确认约束；模式切换异常不调用它。其他正常运行故障的既有停止策略没有在本阶段重写。

## 文件改动

- `bone_ws/src/pus6_motor_driver/pus6_motor_driver/j7_six_axis_servo_transport.py`：当前状态读取、逐轴切入跟踪、位置重锚定复用、恢复报告、不自动 Pause 的故障锁存及独立显式 Pause。
- `bone_ws/src/pus6_motor_driver/pus6_motor_driver/j14_process_uart.py`：现有 Child 接入实时交接、保持目标校验、异常/清理传递、快照及恢复结果保存、恢复不完整禁止复位。
- `bone_ws/src/pus6_motor_driver/pus6_motor_driver/safety.py`：独立的软件基线、相对反馈与增量目标合成接口。
- `bone_ws/src/pus6_motor_driver/pus6_motor_driver/fake_bus.py`：将模拟实际力 `0x2C` 与目标力 `0x27` 分开存储和读取。
- `bone_ws/src/pus6_motor_driver/test/test_stage3b1_2_force_handover.py`：本阶段专项验证。
- `bone_ws/src/pus6_motor_driver/test/test_stage3b1_1_safety_handover.py`、`test_stage3a_force_execution.py`：更新部分失败的旧 Pause 断言，明确已有空载模拟用例的零目标输入。
- 本文及 `docs/force_control_stage3b1_1_safety_handover.md`：记录新策略并标注历史策略已被替换。

## 针对性验证

```bash
PYTHONPATH=bone_ws/src/pus6_motor_driver python3 -m pytest -q \
  bone_ws/src/pus6_motor_driver/test/test_stage3b1_2_force_handover.py \
  bone_ws/src/pus6_motor_driver/test/test_stage3b1_1_safety_handover.py \
  bone_ws/src/pus6_motor_driver/test/test_stage3a_force_execution.py \
  -k 'stage3b1 or mode3_entry'
```

结果：**25 passed，14 deselected**。未运行全工程测试或硬件实验。

覆盖：六轴正常切入、第三轴失败立即中止后续切换、ACK 丢失但已执行的恢复、仅恢复已切入轴、锚定顺序及读回、恢复失败及模式未知的锁存/报告、外层异常和清理无 Pause、显式 Pause 独立可用、实时基线更新、初始相对力为零、有符号保持目标、增量合成、拒绝错误统一零目标、初始保持与周期限幅一致。

测试通过现有 FakeBus/MockServoBus 执行真实协议编码和解析，故障为人工注入。向量 `[3754,-687,-6176,793,-671,-6994]` 借用历史数值构造模拟寄存器，另以改变后的向量验证重新读取；这不是本阶段新的真实硬件测量，也没有回放真实串口响应帧。

## 下一步

本阶段软件工作完成，可以安排下一步受控真机验证；尚未实际验证受载 Mode 3 切换与位置重锚定恢复的机械行为。真机时重新采集实时基线，明确适用的目标力范围，并确认现场支撑、运动空间、独立急停与现有写入授权条件。首次实验保持初始载荷，不启动 Z 轴 PID；观察位置、力与通信，按真实逐轴模式判断结果。不能将本次 FakeBus 通过描述为真机切换或力控通过。
