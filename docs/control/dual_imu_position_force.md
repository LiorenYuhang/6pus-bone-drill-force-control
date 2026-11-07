# 双 IMU 姿态与六轴位置力闭环

本页解释当前三进程入口，最终参数边界见[原生能力策略](native_capacity.md)，整体介绍见[README](../../README.md)。早期任务空间 Wrench/Jacobian 路径不参与这条主动稳定链。

## 姿态测量、同步和坐标

[dual_imu_attitude.py](../../bone_ws/src/pus6_force_control/pus6_force_control/dual_imu_attitude.py)接受 H30 已融合姿态 `[x,y,z,w]`；gyro-only 消息不刷新姿态龄期。各路保存有效四元数历史，在最新共同主机单调接收时间上 SLERP 插值，不外推；同一个共同样本不会重复推进观测器。各姿态最新龄期和插值间隔均受既有 100 ms 检查约束，ROS header 时间仅作诊断。

初始参考是实际编码器 FK 的姿态 `R0`，两路 IMU 各捕获 `qB0`、`qP0`。相对四元数 `q_rel=q0⁻¹⊗q`，机械变换 `C=Rz(+90°)`：

```text
RB   = C R(q_rel_base) Cᵀ
RP   = R0 C R(q_rel_platform) Cᵀ
eW   = Log(R0 RPᵀ)∨
Rff  = RBᵀ R0
Rcmd = RBᵀ Exp(uW) R0
```

`uW` 为复用 Position 的三轴 LADRC 角度残差，Base 正转产生反向前馈，Platform 正向世界误差产生反向修正。完整 RPY 模式不人为缩放角度。控制 XYZ 保持初始 FK 平移；姿态通过 SO(3) 合成，不直接相减 Euler 角。安装方向仍依赖原机械约定；配置里的最终外参测量待确认标记没有因软件运行而消失。

参数来自 [mode3_dual_imu.yaml](../../bone_ws/src/pus6_force_control/config/mode3_dual_imu.yaml)：

| 通道 | b0 / s⁻¹ | ωc / rad·s⁻¹ | ωo / rad·s⁻¹ | 原算法残差边界 |
|---|---:|---:|---:|---:|
| Roll | 20 | 0.8 | 2.5 | ±0.5° |
| Pitch | 10 | 1.0 | 4.0 | ±0.5° |
| Yaw | 6 | 0.25 | 1.2 | ±0.5° |

Classical P/PD 同时记录用于对照，不与 LADRC 输出相加。LESO 前一输入是上一轮送往姿态/IK 链的角度残差，并非实测力矩、实际执行器力或已实现姿态。

## IK 和真实起点

[DualImuPlan](../../bone_ws/src/pus6_force_control/pus6_force_control/mode3_dual_imu.py)读取本次实际六轴位置，FK 收敛后建立差分 IK：`q_ref=q_initial+IK(pose_ref)-IK(pose_initial)`，避免 FK 残差引发起点跳变。目标可达性与实际编码器位置分别验证，并记录目标、实际值、剩余行程和触发原因。当前不是起点 ±100 steps 包络；正常/绝对/设备边界取交集，常见为 200–1800 steps。目标不可达时不缩小 SO(3) 目标冒充完成。

## PI-D 的实际计算

[JointPositionForcePID](../../bone_ws/src/pus6_force_control/pus6_force_control/joint_position_force_pid.py)以各轴原始反馈接收时间计算速度；真实误差 `e=q_ref-q_encoder` 与预测控制误差分别记录。

```text
v_raw  = 最近 80 ms 编码器位置/原时间戳线性回归斜率
v      = 一阶滤波(v_raw)，τ=40 ms，使用各轴真实样本时间推进
q_pred = q_encoder + v × (feedback_age + 20 ms)
P      = Kp × deadband(q_ref-q_pred)
I_next = I + Ki × deadband(q_ref-q_encoder) × dt
D      = -Kd × v
ΔF     = P + I_next + D
F_cmd  = 实测初始保持偏置 + ΔF
```

Kp/Ki/Kd 各轴为 8000/6000/1000（SI），死区各 ±25 μm。速度回归不因小位移跨度突然清零；重复源样本不推进滤波。D 不对参考跳变求导。P 预测可反映反馈延迟，但不能制造真实编码器死区内的积分。初始保持与主动稳定使用同一个 PI-D 实例，不重置速度或积分。

三进程取消独立增量/积分实验限幅；`F_cmd` 含偏置一起钳位至 ±50 N，再量化为有符号整数 g，保证不超过 ±5098 g。`track_applied` 比较实际编码后的力增量与请求值；仅当饱和差异与真实积分误差方向一致、会进一步加深饱和时回退本轮积分，容忍 g 量化误差。这是条件抗积分饱和，不是“完全没有输出边界”。

正常 IK 位置参考直接使用，正常力指令不做变化率整形；5000 g/s 属于原允许的异常恢复路径。其余 Z/单轴/组合及原单进程入口仍可能采用旧边界，不能仅看共享类默认值或另一份 YAML 推断本入口参数。

## 验证解释

已记录的软件测试和固定历史输入回放见[原生能力策略](native_capacity.md)。固定历史反馈不随新指令改变，因此回放只能比较软件计算，不能判定新的实际受力、停止距离或稳定精度。实际目标与反馈力差异、动态滞后、耦合和 Yaw 响应须根据对应真机实验记录判断，不能用通信通过替代控制效果结论。
