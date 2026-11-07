> 历史归档：原路径 `docs/force_units.md`。正文保留当时的方案、参数及证据语境；当前原生 Mode 3 入口以[项目 README](../../../README.md)为准。历史命令不作为当前推荐启动命令。

# Force 混合单位架构与参数迁移

2026-11-03。本轮仅软件修改与离线测试，不连接串口、不发送硬件指令、不执行 Tare。

## 单位边界

厂家原生力单位统一称为 **g（克）**，`0x2C` 与 `0x27` 保留有符号 int16 语义。原始 ADC 属于 `0x2D`，不参与 g→N 换算。

集中实现位于 `bone_ws/src/pus6_motor_driver/pus6_motor_driver/force_units.py`：

- `STANDARD_GRAVITY_M_S2 = 9.80665`，`NOMINAL_N_PER_G = g0 / 1000`。
- `g_to_n()`、`n_to_g()` 接受标量或 ID1..ID6 六轴数组；支持显式方向及每轴 N/g 标定比例。默认方向 +1，不对原始力取绝对值。
- `quantize_force_g()` 在寄存器边界四舍五入到最近整数，半整数沿用 Python ties-to-even；拒绝非有限值和 int16 溢出，不进行回绕。端点往返只容忍机器浮点舍入误差。
- `quantize_force_g_with_n_limits()` 防止整数 g 舍入跨过 N 限幅；共享有符号编码/解码函数。

这是克当量读数到名义物理力的换算，并非传感器绝对标定。后续可用实测标定替换每轴 `newtons_per_g`，不改变电机协议。

```text
0x2C 原始有符号 g + 有效性/源时间戳
 → TimedJointFeedback.forces_g_raw（仍为 g）
 → 现有 FixedLag 同步 q(t)、f(t)
 → ForceCompensation.feedback_n：可选电子偏置、方向、g→N（一次）
 → J(q)^T · force_n → wrench_n_nm [N,N,N,Nm,Nm,Nm]
 → 现有静载/重力处理 → Z PID（N）
 → 原有 J^T 力分配 → actuator_forces_n（N）
 → 逐轴 min_force_n/max_force_n 限幅
 → n_to_g → 一阶变化率限制（g/s，沿用已有时间逻辑）
 → 整数 g 编码 → 0x27
```

ROS `AxialForceCommand.force_n`、`ForceFeedback.force_n` 保持 N，`WrenchStamped` 为 N/Nm。节点为 dry-run 构造的 g 帧仅用于预测/显示；实际 UART Child 收到的是 SI 指令，自行执行 N 限幅→g 转换→g/s 限速→0x27。两者不是把同一份原始反馈连续换算两遍。Child 回传 `dispatched_forces_n` 是由实际编码 g 反算的命令值，不是测得反馈。

`SixAxisForceDrive.command_forces_n()` 是算法侧 SI 入口；已有 `command_forces()` 保留原生 g 接口。`WrenchAllocationResult.actuator_forces_n` 是分配主输出；保留的 g 报告字段仅作旧探索接口的派生显示，主控制链不使用它们。新增 `reconstruct_task_wrench_n()` 提供纯 SI 重建；旧 g 输入重建入口内部只换算一次。雅可比公式、坐标系、力分配求解、PID 和 Torque 运动限速算法未重写。

## 参数与限幅

| 参数 | 当前定义 | 变更 |
|---|---|---|
| PID 输出 | ±10 N | 保留 |
| 目标力参考变化率 | 1 N/s | 保留；节点无配置时的回退也对齐为 1 |
| 执行器目标变化率 | 每轴 500 g/s | 保留，未误用为 N/s |
| 执行器绝对指令限幅 | 每轴 ±14.709975 N | 等效于原 ±1500 g，无扩大 |
| 可选位置参考速度 | 0.017 m/s | 保留，与力变化率分开 |

配置：`bone_ws/src/pus6_force_control/config/force_pid.yaml`。`ForceLimitsConfig` 及 ROS 参数的 `min_force_g/max_force_g` 已迁移为 `min_force_n/max_force_n`，数值乘以名义比例后迁移，不能只改参数名。旧 YAML/Child 配置中的 g 限幅键会报迁移错误，避免误将 1500 当成 1500 N。

UART Parent 已向既有 Child 传递 `direction`、`newtons_per_g`、`min_force_n/max_force_n`、`max_rate_g_s` 和负力支持参数；这不自动进入 Mode 3，也不赋予写入授权。若使用非默认标定/方向或调整限制，力控节点与 UART 驱动必须设置同一组参数。

电机额定机械力仍以 **N** 表示，不等同 int16 编码范围；软件默认限制没有因本次换算而放宽。装配状态硬件 Tare 后的零值不代表实际机械载荷消失；仅限制当前设备参考下的目标 N 不能证明包含既有预载的总机械力低于额定值。

## Tare、兼容性与历史数据

硬件 Tare 的 `0x09=1`、异步完成帧、单轴一次写入、清零读回、Mode 3 切换及同帧 Mode 0 重锚定逻辑保持。硬件 Tare 不属于单位转换，也不会由转换模块调用。设备已经 Tare 后，不再重复减去原装配基线；默认 `sensor_zero_g`、`actuator_bias_n` 都是零，重力/静载补偿不得重复计入。

正式驱动/Tare 接口的旧 `_gf` 兼容函数、属性、关键字和重复输出键已移除。历史原始数据未改；仅实验文件读取器保留显式历史字段迁移，将旧误标后缀按 g 读取，不增加 live API 别名。既有探索代码副本保留，不作为正式包的单位定义来源。

## 验证与 Stage 3C

测试覆盖标量/六轴正负换算、往返、每轴标定、int16 端点/溢出/舍入、N 限幅后 g/s 限速、真实协议编码、Jacobian→PID→分配→0x27 单位边界及历史 Tare 记录读取。反馈链回归包含已有录制帧/离线数据；新单位链测试使用合成输入/FakeBus，不是新真机结果。

本次单位修正消除了 Stage 3C 的软件单位衔接问题，可以继续小幅 Z 轴力跟踪实验开发。受载 PID 跟踪、增益与方向的整机表现仍需 Stage 3C 实机验证；本轮没有生成新的硬件性能结论。

本轮验证结果：

- 新增单位测试 **24 项**，与受影响驱动、PID 管线、Tare、Stage 3A 和部分模式切换恢复测试合跑：**80 passed**。
- 节点、Fixed-Lag/有效性/目标超时、协议反馈链与单位/管线测试合跑：**71 passed**。两组有重叠，不相加作总数。
- `pus6_msgs`、`pus6_research_msgs`、`pus6_motor_driver`、`pus6_force_control` 四包构建成功。清理了 Force 目录内沿用 Torque 路径的旧 CMake cache，重新生成源码已有的反馈有效性字段；没有修改 Torque。最终驱动/力控包再次构建成功。
- 本轮 `hardware_write_count = 0`；未连接真实串口，未提交或推送 Git。
