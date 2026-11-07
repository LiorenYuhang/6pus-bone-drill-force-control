# 双 IMU 原生力控长期运行

主入口为 [run_force_dual_imu_native_pid.sh](../../bone_ws/scripts/run_force_dual_imu_native_pid.sh)。它加载 `/opt/ros/jazzy/setup.bash` 和本工程 `bone_ws/install/setup.bash`，调用 `run_mode3_dual_imu_three_process.sh --continuous --auto-home --axes roll,pitch,yaw`，再进入 [mode3_dual_imu.py](../../bone_ws/src/pus6_force_control/pus6_force_control/mode3_dual_imu.py)。控制算法和保护见[README](../../README.md)。

## 软件检查与真实启动

仅检查现有环境、文件和 argparse 帮助；不启动节点、IMU 或 UART：

```bash
/home/lyapunov/6pus_bone_drill_force_control/bone_ws/scripts/run_force_dual_imu_native_pid.sh --software-check
```

现场真实运行（会发送 Mode 3、`0x27`，正常退出还会位置回 Home）：

```bash
/home/lyapunov/6pus_bone_drill_force_control/bone_ws/scripts/run_force_dual_imu_native_pid.sh --hardware --hardware-qualified --start-imus
```

不指定硬件选项时使用合成 IMU/Mock 电机，不能用来评价物理稳定性。`--start-imus` 使用现有唯一 IMU 发布者，缺失时按既有 H30 方式启动；不启动 Position/Torque 控制程序。现场保持初始基座稳定，捕获当前实际姿态，确认 RS485 没有其他控制入口占用。

连续入口不接受 `--duration-seconds`。原有限时实验使用 `run_mode3_dual_imu_three_process.sh`，不带 `--auto-home` 时不自动返回 Home；不应将有限时 `final_hold` 与连续 Ctrl+C 流程混为一谈。

## 阶段与正常停止

模式/有效参考就绪 → 固定 1 秒 `initial_hold` → 持续 `stabilize`。起点为本次实际反馈，主动阶段 XYZ 固定、Roll/Pitch/Yaw 同时控制。

按一次 Ctrl+C 后，Parent 停止目标流，Child 执行原安全恢复并确认 Mode 0，相关电机进程及 IMU 关闭。仅正常 `operator_stopped`、无 Parent error 且六轴 Mode 0 成功时才进入独立位置回 Home。终端继续显示 Home 阶段，不能在刚看到 `OUTCOME` 后就假设整个程序已经结束。需要禁止正常退出回 Home 时可在原命令末尾显式加 `--no-auto-home`。

故障、启动失败和 Mode 0 未确认不会自动 Home；不会为消除故障而自动重试运动。回 Home 中再次 Ctrl+C 走现有 runner 的中断/Pause 路径，结果可能为 `HOME_FAILED` 或 `HOME_ABORTED`，以报告为准。[Home 实现和局限](home.md)解释最后模式及停止证据。

## 日志与结果

| 记录 | 内容 |
|---|---|
| `results/mode3_dual_imu/console/native_*.log` | 控制台全过程；tee 在 Ctrl+C 后继续接收清理和 Home 输出 |
| `results/mode3_dual_imu/run_*` | 真机唯一结果目录；Mock 为 `dry_run_*` |
| `tracking.jsonl` | IMU、姿态/FK、参考与实际位置、PI-D、力、电流、周期和阶段 |
| `serial_bytes.jsonl`、Child 生命周期/周期 | 原生力阶段串口及退出过程 |
| `dual_imu_report.json` / `force_shutdown.json` | 主动阶段结果与六轴 Mode 0 恢复 |
| `motion_parent_configuration.json` | 完整配置、有效 PI-D 和输出策略；终端不打印整份 JSON |
| `home_recovery.json` | 后续独立 Home 的起终点、到位误差、Pause 和返回状态 |

`OUTCOME: operator_stopped` 是人为结束，不等同 `motion_complete` 或稳定性能通过。原生力报告的写入计数和串口追踪范围不包含后续独立 J7 Home 事务；Home 摘要另存，但没有与力控阶段相同的完整逐帧串口归档。退出码 0 仅表示相应程序条件通过，不能单独推出长期稳定或所有硬件异常恢复均经过真机测试。

当前资料仍存在的实现边界见[Home](home.md)与[实验索引](../experiments/README.md)。本轮文档整理未执行上述真实命令或任何检查启动命令。
