> 历史归档：原路径 `docs/j11/J11_FINAL_CONTROL_COMPARISON.md`。正文保留当时的方案、参数及证据语境；当前原生 Mode 3 入口以[项目 README](../../../../README.md)为准。历史命令不作为当前推荐启动命令。

# J11 最终控制方案对比

**状态**：**J11 FINAL FREEZE READY**；Task 2 Demo smoke test 已完成；pending Task 4 final Git freeze

**数据等级**：已有真实硬件 qualification evidence（本次未新增真机实验）

**最终控制器名称**：Base-IMU SO(3) Feedforward + Platform-IMU Classical P/PD Attitude Feedback

## 1. 一页汇报摘要

J11 的目标是在不改变既有 IK、六轴 Servo、Driver、几何与安全链的前提下，提高 6PUS 平台对动基座姿态扰动的抑制能力。M1 使用 **Base-IMU SO(3) Feedforward**；M2 在同一执行链上增加 **Platform-IMU task-space Classical P/PD Attitude Feedback**。

最终保留参数：

```text
Kp = [0.05, 0.05, 0.025]
Ki = [0.0, 0.0, 0.0]
Kd = [0.0, 0.0015, 0.0015] s
Derivative LPF fc = 5.0 Hz
Residual limit = +/-0.5 deg / axis
实际反馈：Roll = P，Pitch = PD，Yaw = PD
```

归一化响应定义为 `G = Platform attitude response / Base attitude disturbance`，因此 **G 越小表示姿态抗扰越好**。

| Metric | M1 Feedforward | M2 Feedforward + P/PD | 代表性实验中观测到的归一化指标下降 |
|---|---:|---:|---:|
| Roll G_Peak | 0.4778 | 0.3107 | ~35.0% |
| Roll G_RMS | 0.4071 | 0.2662 | ~34.6% |
| Pitch G_Peak | 0.5946 | 0.3279 | ~44.9% |
| Pitch G_RMS | 0.4153 | 0.2607 | ~37.2% |
| Yaw G_Peak | 0.7983 | 0.4424 | ~44.6% |
| Yaw G_RMS | 0.2912 | 0.2297 | ~21.1% |
| **SO(3) G_Peak** | **0.7353** | **0.3063** | **~58.3%** |
| **SO(3) G_RMS** | **0.3713** | **0.2575** | **~30.6%** |

核心结果：SO(3) `G_Peak` 从 **0.7353 降至 0.3063**，观测下降约 **58.3%**；SO(3) `G_RMS` 从 **0.3713 降至 0.2575**，观测下降约 **30.6%**。

> M1/M2 的人工扰动轨迹不完全一致，因此该表用于 representative normalized hardware characterization，而不是严格同输入传递函数比较。

## 2. J11 目标

- M1：建立并冻结 Base IMU 到全 SO(3) Feedforward、IK 与既有六轴 Servo 的真机基线。
- M2：使用 Platform IMU 的世界系姿态误差形成小幅 task-space residual feedback。
- 收口：在已有 qualified hardware evidence 上确定 classical controller baseline，作为 J12 ADRC 的 A/B 对照。

## 3. 对比方案

| 项目 | M1：纯前馈 | M2：前馈 + P/PD |
|---|---|---|
| Base IMU | 使用 | 使用 |
| Platform IMU | 观测 / 验证 | 闭环反馈 |
| SO(3) 前馈 | 有 | 有 |
| Platform 姿态反馈 | 无 | 有 |
| Roll Feedback | 无 | P |
| Pitch Feedback | 无 | PD |
| Yaw Feedback | 无 | PD |
| Kp | N/A / Feedback disabled | [0.05, 0.05, 0.025] |
| Ki | N/A / Feedback disabled | [0, 0, 0] |
| Kd | N/A / Feedback disabled | [0, 0.0015, 0.0015] s |
| D 项低通 | N/A | 5 Hz |
| 姿态残差限幅 | N/A | +/-0.5 deg / axis |
| IK | 相同 | 相同 |
| 六轴 Servo | 相同 | 相同 |
| Driver | 相同 | 相同 |
| Safety | 相同 | 相同 |

## 4. 控制架构对比

M1：

```text
Base IMU -> R_WB -> SO(3) Feedforward -> R_BP_cmd -> IK
         -> Six-axis Servo -> Platform
Platform IMU -> Observer / Validation only
```

M2：

```text
Base IMU -> SO(3) Feedforward -------------------+
                                                   -> compose final R_BP_cmd
Platform IMU -> R_WP attitude error -> P/PD residual +
               -> IK -> Six-axis Servo -> Platform
```

## 5. 参数对比

M1 没有启用 Platform attitude feedback controller，因此其 `Kp/Ki/Kd` 记为 N/A，而不是零增益控制器。M2 的最终 retained controller 为：

| Axis | Kp | Ki | Kd (s) | 实际形式 |
|---|---:|---:|---:|---|
| Roll | 0.05 | 0 | 0 | P |
| Pitch | 0.05 | 0 | 0.0015 | PD |
| Yaw | 0.025 | 0 | 0.0015 | PD |

## 6. 数据来源与实验依据

| 方案 | 用途 | Dataset | Raw Evidence | Summary Evidence | 备注 |
|---|---|---|---|---|---|
| M1 | Frozen FF baseline | `results/j11/runtime_validation/20260918_112354/` | `J11_DISTURB_QUAL_feedforward_powered.csv` | `J11_DISTURB_QUAL_feedforward_summary.json` | canonical qualification；120 s；summary verdict `PASS` |
| M2 | Final retained controller | `results/j11/m2_pid/20260920_095850/` | `j11_m2_7a_pitchd_cand2_repeat_phaseB_dynamic_telemetry_powered.csv` | `j11_m2_7a_pitchd_cand2_repeat_phaseB_dynamic_summary_powered.json` | representative qualified run；60 s；无 fault reason |

可追溯计算口径：

- M1 使用 raw CSV 中全部 `ACTIVE` 样本（cycle `71..5999`，5929 行）。
- M2 使用 representative manual-disturbance effective window（cycle `396..2999`，2604 行，约 52.08 s）。
- 每轴 `G_Peak = max(abs(platform)) / max(abs(base))`。
- 每轴 `G_RMS = RMS(platform) / RMS(base)`；SO(3) 使用对应 magnitude/error 列同法计算。
- 表中下降率由 `(1 - G_M2 / G_M1) * 100%` 重新计算并四舍五入到 0.1%。

M2 的 summary JSON 记录运行阶段、真实硬件执行模式、2946 次 hardware write、2999 个总周期、最终 `ACTIVE` 且 `fault_reason` 为空；归一化性能数字来自同目录 raw telemetry，而不是从 summary JSON 推断。

## 评价指标定义与物理意义

设分析窗口内某一姿态误差序列为 $\theta(t)$，其离散样本为 $\theta_k$（$k=1,\ldots,N$）。

### Peak：峰值误差

定义为：

$$
\mathrm{Peak}=\max_t |\theta(t)|
$$

离散形式为：

$$
\mathrm{Peak}=\max_k |\theta_k|
$$

Peak 表示整个分析窗口中出现过的最大绝对姿态偏差，用于衡量“最坏时刻平台最大偏离目标姿态多少”。它对瞬时大偏差、尖峰和最坏情况较敏感。

### RMS：均方根误差

定义为：

$$
\mathrm{RMS}=\sqrt{\frac{1}{N}\sum_{k=1}^{N}\theta_k^2}
$$

RMS 综合整个分析时间窗口内的所有姿态误差，反映“平台在整个动态过程中总体抖动有多大”。因此，Peak 更偏向最坏瞬时表现，RMS 更偏向整体持续误差水平。在 J11 中，Peak 和 RMS 必须结合使用，不能只用其中一个指标评价防抖效果。

### G_Peak：归一化峰值姿态响应比

定义为：

$$
G_{\mathrm{Peak}}=\frac{\mathrm{Platform\ Peak}}{\mathrm{Base\ Peak}}
$$

例如，若 Base Roll Peak 为 $10^\circ$，Platform Roll Peak 为 $3^\circ$，则 $G_{\mathrm{Peak}}=0.3$。

$G_{\mathrm{Peak}}$ 表示 Platform 最大姿态响应相对于 Base 最大姿态扰动的比例。一般而言，$G_{\mathrm{Peak}}$ 越小，说明最坏情况下 Platform 对 Base 扰动的姿态响应越小。

### G_RMS：归一化 RMS 姿态响应比

定义为：

$$
G_{\mathrm{RMS}}=\frac{\mathrm{Platform\ RMS}}{\mathrm{Base\ RMS}}
$$

$G_{\mathrm{RMS}}$ 表示 Platform 整体姿态误差水平相对于 Base 整体姿态扰动水平的比例。一般而言，$G_{\mathrm{RMS}}$ 越小，说明整个动态过程中 Platform 对 Base 扰动的总体响应越小。

### G 指标的解释边界

对于当前 J11 的人工扰动实验，$G_{\mathrm{Peak}}$ 和 $G_{\mathrm{RMS}}$ 用于减少不同人工扰动幅值对绝对姿态误差比较的影响。但是，M1 与 M2 的 Base 人工扰动轨迹并不完全相同，所以这两个指标属于 **representative normalized hardware characterization**，不能解释为严格相同输入下的传递函数增益。

因此，不能写成“$G=0.3$ 表示严格衰减 70% 的系统传递函数”。更准确的表述是：“在该代表性真机实验中，Platform 的归一化姿态响应比例约为 0.3。”

### SO(3) 姿态误差

Roll、Pitch、Yaw 分别描述三个姿态分量；SO(3) 指标描述 Platform 当前三维姿态相对于目标姿态的整体旋转误差大小。因此，Roll、Pitch、Yaw 用于观察各轴表现，SO(3) 用于观察整体三维姿态稳定效果。

在 J11 汇报中，SO(3) $G_{\mathrm{Peak}}$ 可用于观察整体姿态的最坏归一化响应，SO(3) $G_{\mathrm{RMS}}$ 可用于观察整个动态过程中的整体归一化姿态误差水平。

| 指标 | 回答的问题 | 越小是否越好 |
|---|---|:---:|
| Peak | 最坏时刻最大偏了多少？ | 是 |
| RMS | 整个过程中总体抖得多大？ | 是 |
| G_Peak | Platform 最大偏差相对 Base 最大扰动有多大？ | 是 |
| G_RMS | Platform 总体误差相对 Base 总体扰动有多大？ | 是 |
| SO(3) Peak/RMS | 整体三维姿态偏差，而非单独某一欧拉角 | 是 |

## 7. 动态性能对比

在两组代表性真机扰动实验中，加入 Platform IMU task-space feedback 后，各轴及 SO(3) 的归一化扰动响应指标整体降低。主表中的八项 reduction 均为基于所列 raw evidence 的 observed normalized ratio reduction。

这些结果支持“最终 retained P/PD controller 在已测试条件下具有更低的代表性归一化响应”，不支持“全局最优”或“严格同输入下精确提升”的结论。

## 8. 静态资格验证说明

- M1：`results/j11/runtime_validation/20260918_111527/J11_RT3_STATIC_feedforward_summary.json` 记录 300 s Static qualification，verdict 为 `PASS`。
- M2 retained 参数：`results/j11/m2_pid/20260920_095502/j11_m2_7a_pitchd_cand2_phaseA_static_summary_powered.json` 记录 1500 周期、1430 次 hardware write、最终 `ACTIVE`、空 fault reason；对应 telemetry 为同目录 `j11_m2_7a_pitchd_cand2_phaseA_static_telemetry_powered.csv`。
- 多个其他 M2 candidate 也保留了 Phase A static evidence。由于 M1/M2 静态记录并非完全相同统计口径，本文件不强行生成静态 Peak/RMS 百分比。

已有证据表明两种方案均完成静态运行验证；M2 feedback 在所记录的静态运行中未表现出异常持续振荡或 fault termination。本 Task 未重新运行 Static。

## 9. 执行层与安全机制对比

| Item | M1 FF | M2 FF+P/PD |
|---|---|---|
| Driver | Same | Same |
| IK | Same | Same |
| Servo | Same | Same |
| Geometry | Same | Same |
| Base IMU | Yes | Yes |
| Platform IMU Closed-loop | No | Yes |
| Safety Logic | Same | Same |

M1 到 M2 的主要改变是 Platform IMU task-space feedback；底层 `IK -> existing six-axis servo` 执行链、几何与安全机制不变。

## 10. J11 调参收口

- P-only retained：`Kp = [0.05, 0.05, 0.025]`。
- Yaw-D retained：`Kd_yaw = 0.0015 s`。
- Pitch-D retained：`Kd_pitch = 0.0015 s`。
- Roll-D：`Kd_roll = 0.0010 s` 完成 qualified repeat，但 **NOT RETAINED**；最终 `Kd_roll = 0.0`。
- Final：`Kd = [0.0, 0.0015, 0.0015] s`；Roll-D tuning CLOSED，不继续增益扫描。

详细调参历史见 [`J11_M1_M2_CONTROL_COMPARISON.md`](../../../../agent_docs/stages/J11/J11_M1_M2_CONTROL_COMPARISON.md)。

## 11. 工程结论

J11 最终 classical baseline 已明确：Base-IMU SO(3) Feedforward + Platform-IMU Classical P/PD Attitude Feedback。该方案保留 M1 已验证的前馈、IK、Driver、Servo 与安全链，只增加有限幅的 Platform attitude residual。现有 qualified evidence 支持将其作为 J12 的 classical A/B baseline。

**Task 2 Demo 状态**：

- Demo A — Feedforward Only：**PASS**。
- Demo B — Feedforward + P/PD：**PASS ON REPEAT**。首次 Demo B 因 motor CH340 USB-UART 瞬时 disconnect / re-enumeration 导致 `motion_write_failure` 提前结束，归类为 runtime USB-UART transport interruption（非 controller defect）；相同 controller / gains 的随后 repeat 正常完成。详见 [`J11_DEMO_B_USB_UART_FAULT_AUDIT.md`](../../../../agent_docs/stages/J11/J11_DEMO_B_USB_UART_FAULT_AUDIT.md)。

Demo smoke-test run 不作为新的 performance evidence，主表仍使用 M1 canonical（112354）与 M2 representative（095850）。

## 12. 演示入口

两个最终 Demo wrapper 均根据自身 `BASH_SOURCE` 定位 repository/workspace，并自行按顺序加载 ROS 2 Jazzy 与 `bone_ws/install/setup.bash`。因此，从刚开机、尚未手工 source 环境的新 Terminal 直接运行即可；已经 source Jazzy/workspace 的开发 Terminal 也同样支持。

纯前馈：

```bash
cd /home/lyapunov/bone_drill
bash bone_ws/scripts/run_j11_demo_feedforward.sh
```

前馈 + P/PD：

```bash
cd /home/lyapunov/bone_drill
bash bone_ws/scripts/run_j11_demo_feedforward_pd.sh
```

两个 wrapper 均复用现有 staged launcher 与 runner；`CONTROL ACTIVE` 只能在 `/tmp/j11_active_signaled` gate 出现后显示。演示中只按一次 `Ctrl+C`，wrapper 会受控结束并返回 `OPERATING_HOME`。开发/排错时可显式 source `/opt/ros/jazzy/setup.bash` 与 `/home/lyapunov/bone_drill/bone_ws/install/setup.bash`，但这不是最终 Demo 的强制要求。两个 wrapper 均已通过 fresh-shell 与 already-sourced 两种终端的 `--software-check`（`hardware_write_count=0`），并已完成 Task 2 真机 Demo smoke test。

## 13. 数据解释边界

- 两组实验均由人工施加 Base disturbance，轨迹、幅值与能量不完全一致。
- 归一化 G 指标降低可用于 representative hardware characterization，但不是 transfer-function measurement。
- 不得表述为“完全相同扰动下性能精确提升 58.3%”。
- 不得称最终方案为 full PID、3-axis full PID 或 global optimum。

## 14. J12 下一阶段

J12 面向 **External Payload / Force Disturbance + ADRC Disturbance Rejection**。基线为本文件中的 J11 classical P/PD controller；候选方案为 Base IMU SO(3) Feedforward + Platform attitude ADRC residual。A/B 比较应使用相同 payload、安装位置、初始条件与扰动 protocol，不预设 ADRC 一定更优。
