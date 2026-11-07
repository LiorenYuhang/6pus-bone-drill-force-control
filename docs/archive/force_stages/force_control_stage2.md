> 历史归档：原路径 `docs/force_control_stage2.md`。正文保留当时的方案、参数及证据语境；当前原生 Mode 3 入口以[项目 README](../../../README.md)为准。历史命令不作为当前推荐启动命令。

# 6PUS Force Control 第二阶段（历史功能说明）

当前数据链实现、时间戳语义、停止策略和验证边界见 [Stage 2.1 数据链说明](force_control_stage2_1.md)。下方测试数量与输出为 Stage 2 的历史记录，不是本次验证结果。

工程：`/home/lyapunov/6pus_bone_drill_force_control`
正式核心包：`bone_ws/src/pus6_force_control`
节点入口：`pus6_force_control_node`（对应模块 `pus6_force_control/force_control_node.py`）

---

## 一、Stage 2.1 针对性修正说明

在 Stage 2 基础之上，本次针对 7 项关键工程与时序控制逻辑进行了严密修复：

### 1. Fixed-Lag 时间对齐失败时的保护
- **修正前**：Fixed-Lag 状态若未达到 `SYNCED`（如初始缓冲区累积中或时间跨度未覆盖），静默回退至最新未对齐快照继续执行力控。
- **修正后**：取消静默回退机制。若 `use_fixed_lag=True` 且状态为非 `SYNCED`（如 `WAIT_FUTURE_SAMPLE`、`EMPTY_BUFFER`），立即中止本周期的力控指令输出，发布 `WAITING_FOR_SYNC` 诊断，杜绝未对齐数据污染积分与控制输出。

### 2. 位置与力反馈的时间对齐（已由本次 Stage 2.1 修正）
原实现只对位置做 Fixed-Lag，而力使用最新消息；原“已在物理时间上对齐”的说明不成立。现已扩展既有同步历史，使同一 StatusFeedback 的位置和力使用相同端点、相同权重重建到同一主机参考时刻。主机接收完成时间不等于设备采样时间，详细范围见上方链接。

### 3. FK 不收敛或几何异常时立即熔断
- **修正前**：FK 若收敛残差稍大仅打印 warning，仍沿用上一周期位姿计算雅可比与力分配。
- **修正后**：引入严格残差门限检查（`max_fk_residual_m = 1.0e-3`，即 1 mm 几何杆长误差上限）。一旦出现数值发散、超残差或非有限浮点数（NaN/Inf），直接中断本周期控制指令下发，发布 `FK_DIVERGED` / `FK_EXCEPTION` 诊断，防止病态雅可比导致输出暴冲。

### 4. 完善反馈源时间戳、序列连续性和数据新鲜度检查
- **复用机制**：借鉴 Position `j14_feedback_bridge` 的序列检测机制，跟踪 `TimedJointFeedback.uart_cycle_sequence`。
- **检测项目**：
  - 严格递增序列：检测并累计丢帧数 `dropped_cycles_count`（`seq > last_seq + 1`），告警序列乱序回退。
  - 纳秒级 Gap 监控：两次接收间隔超过 `max_feedback_gap_s`（默认 200 ms）即记录警告。
  - 绝对新鲜度门禁：接收时间滞后超过 `feedback_timeout_s`（默认 200 ms）立即触发 `FEEDBACK_TIMEOUT` 故障锁死。

### 5. 目标力超时（已由本次 Stage 2.1 修正）
原来超时后改为 0 N 目标并继续 PID，不能视为安全停止。现改为停止命令发布、锁存故障、重置 PID 状态，并通过既有 Pause 接口请求 Child 停止；dry-run 只发布隔离的停止请求，不执行真实 Pause。

### 6. 解耦硬件写入授权、dry-run 与硬件资格确认
- **安全解耦**：
  - `dry_run`: 纯软件仿真/回放开关。
  - `hardware_write_enabled`: 底层总线物理写操作许可开关。
  - `is_hardware_qualified`: 硬件参数（行程、幅值、变化率）经真机审查资格标志。
- **强制约束**：若 `hardware_write_enabled=True`，必须显式保证 `is_hardware_qualified=True`，否则驱动构造期直接抛出 `ValueError`，防止未经验证的实验配置误发物理写指令。

### 7. force_pid.yaml 实际加载与参数生效
- **配置链路**：力控节点启动时优先检索并解析 `config/force_pid.yaml`，若未通过 ROS 参数显式覆盖，则完全遵循 YAML 文件中定义的增益、双向力范围、变化率限制和标定补偿参数，并支持外部 `--ros-args -p config_file:=...` 指定自定义配置。

---

## 二、Stage 2 历史测试记录

在 ROS 2 Jazzy 环境下，`pus6_force_control` 的全部 28 个单元测试与集成测试 100% 通过（耗时 0.86s）：

```text
bone_ws/src/pus6_force_control/test/test_force_core.py ............... [ 53%]
bone_ws/src/pus6_force_control/test/test_ros_node.py .......          [ 78%]
bone_ws/src/pus6_force_control/test/test_stage2_1_fixes.py ......     [100%]
============================== 28 passed in 0.86s ==============================
```

测试重点覆盖：
1. `test_fixed_lag_sync_failure_halts_output`：FixedLag 未对齐时不发命令。
2. `test_fk_divergence_halts_output`：非正常几何构型导致 FK 发散时熔断指令。
3. `test_target_force_stream_watchdog`：历史预期为目标力超时后降为 0 N；本次已更新为停止输出与故障锁存。
4. `test_decoupled_safety_architecture`：未认证硬件资格下强制写硬件被安全拦截。
5. `test_yaml_config_parameter_loading`：正确自 YAML 加载默认配置。
6. `test_sequence_continuity_and_gap_monitoring`：准确识别丢帧与序号跳变。

---

## 三、Stage 3

当前剩余工作及软件验证边界见 [Stage 2.1 数据链说明](force_control_stage2_1.md)。不要把本历史文档的测试 PASS 或软件许可字段解释为真机接入资格。
