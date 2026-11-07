> 历史归档：原路径 `docs/force_control_stage1.md`。正文保留当时的方案、参数及证据语境；当前原生 Mode 3 入口以[项目 README](../../../README.md)为准。历史命令不作为当前推荐启动命令。

# 6PUS Force Control 第一阶段

正式包：`bone_ws/src/pus6_force_control`。本阶段提供离线计算与协议帧构造，不打开串口、不进入 Mode 3、不发送电机命令。示例参数仅用于软件验证。

## 代码来源与结构

Force 已包含 Position 的基础包。本次逐文件比较确认 `pus6_msgs`、`pus6_kinematics`、`pus6_motor_driver` 中 Position 的现有文件已在 Force 中原样存在，因此直接复用本地基础包，不重复覆盖。几何 YAML、IK/FK、协议、UART、Motor 和 FakeBus 均由 Force 自己的源码构建。

| 文件 / 目录 | 本阶段职责 |
|---|---|
| `pus6_force_control/six_axis_force_drive.py` | 从 Force 同名探索模块移植，复用正式 Motor/协议，保留 Mode 3 帧、0x27 帧和六轴状态反馈读取；增加显式 monotonic 时间接口 |
| `pus6_force_control/wrench_allocation.py` | 从 Force 同名探索模块移植；分配和重建改用已有解析空间雅可比；保留 Euler 数值雅可比作校核；不默认加载历史 tare |
| `pus6_force_control/rate_limiter.py` | 提取 Torque `SixAxisPositionDrive.command()` 的 `old + clip(target-old, ±rate*dt)`；保留量化前浮点状态 |
| `pus6_force_control/task_space_force_pid.py` | Z 力 PID，参考限速、输出限幅、条件积分抗饱和、积分项限幅、测量微分与一阶微分滤波 |
| `pus6_force_control/force_pipeline.py` | 六轴力反馈 → 当前位姿 → 空间广义力 → PID → 六轴目标力；统一零点、方向和静态补偿；分配后驱动限制发生时抑制积分继续累积 |
| `pus6_force_control/offline_demo.py` | 安装入口 `force_pid_offline`；只使用 FakeBus 和 dry-run 帧 |
| `config/force_pid.yaml` | PID、力参考 N/s、传感器补偿、驱动限幅；保留 Torque 的位置速度 0.017 m/s、20 ms 周期、921600 baud 作为位置参考配置 |
| `test/conftest.py`, `test/test_force_core.py` | 本地包导入、限速、PID、非零姿态虚功、补偿、反馈 FK、协议帧与零硬件写入测试 |
| `package.xml`, `setup.py`, `setup.cfg`, `resource/`, `__init__.py` | ament_python 包声明、依赖、配置安装与入口 |

原 `bone_ws/scripts/research/force_position_control/` 探索代码和测试保持原样。现有 Position 三阶整形器及历史 bringup 仍作为继承代码保留，新 Force 包没有导入或调用它们。旧的系统 launch 不作为本阶段力控入口。

## 物理量与接口

坐标系为 `pus6_base`。位姿为 `[x,y,z,roll,pitch,yaw]`（m、rad），沿用现有几何原点与 `OPERATING_HOME=[1000]*6`，不另设零位。空间力矩关于动平台原点表达。

```text
q_dot = J_spatial [v_base; omega_base]
w_actuation = J_spatial.T f_slider
f_command = solve(J_spatial.T, w_command)
```

`f_slider` 是执行器沿导轨的轴向驱动力（N），不是斜连杆内力；`w` 为 `[Fx,Fy,Fz,Mx,My,Mz]`（N、N·m）。Euler 广义雅可比的后三项对应 Euler 广义力，不能直接当作空间力矩，尤其在非零姿态下。

`SixAxisForceDrive.read_feedback()` 沿用正式 `Motor.read_status()` 的 `0x30` 状态查询，从状态包解析实际力字段（对应 0x2C）、位置、ADC、电流及故障码。主机接收时间不是设备采样时间。轴序为电机 ID 1–6。

`TaskSpaceForcePID.update(target_force_n, measured_force_n, now_ns=...)` 返回限速后参考、误差、PID 分项、总输出及 dt。默认 `reference_plus_pid` 输出 `reference + P + I + D`；`pid_only` 输出 PID 本身，可作任务力指令或由调用者接入补偿。两种模式都对最终输出限幅。

`TaskSpaceForcePipeline.update(target_z_n, force_raw_g, pose, ...)` 接受当前位姿；`update_from_snapshot(...)` 则使用同一批六轴状态位置，按本地几何 step/m 映射后做 FK，不以固定 HOME 代替实测位置。输出除 Z 外的净任务力分量设为零；它们没有闭环控制，不能据此声称其他自由度稳定。

驱动端默认仍禁止负力，超出单轴范围或变化率的分配会被限制；`actuator_limited` 标记该情况。`applied_wrench` 是根据实际编码的整数 gf 目标预测的任务力，不是测得的执行结果。限制可能破坏原目标力矩；本阶段没有约束优化分配器。

## 速率与补偿

时间来自 `time.monotonic_ns()`，`dt` 为两次调用实际间隔。重复或倒退的时间戳被拒绝。初始化时显式设置起始参考，第一步同样受限；不采用固定每周期步长替代物理速度。

- 位置：`PositionReferenceRateLimiter`，m 与 m/s，保留 Torque 的 0.017 m/s。
- 任务 Z 力参考：`ForceReferenceRateLimiter`，N 与 N/s，示例为 1 N/s，独立配置。
- 驱动端：既有接口为 gf 与 gf/s，示例为 500 gf/s/轴，名义约 4.903325 N/s/轴。它与任务 Z 力参考速率不同。

反馈校正：`f_n = (raw_g - sensor_zero_g) * direction * newtons_per_g`。名义换算为 1 gf = 0.00980665 N；实际增益、方向、0x27/0x2C 的一致性仍需标定。

传感器零点只扣除反馈，不作为支撑力再加回目标。静态物理支撑偏置 `actuator_bias_n` 从校正后反馈扣除，并在分配后加回一次。也可逐周期传入姿态相关 `gravity_wrench`，从重建反馈扣除并在分配前加回一次；此时不能同时使用包含重力的静态支撑偏置。这里的 gravity_wrench 指**执行器需要提供的支撑广义力**，不是直接填入向下的重力外载。

重建量是执行器提供的广义力，扣除静态支撑后仍可能包含惯性、摩擦和模型误差。只有在相应准静态/动力学假设成立、载荷补偿充分且明确作用与反作用符号后，才可推断外部接触力。本阶段不把它命名为外部接触力真值。

## 构建与运行

只需要本项目和现有 ROS2 Jazzy。为避开复制工程中旧 build/install 的绝对路径，使用本项目生成目录内的 `force_stage1` 子目录。

```bash
cd /home/lyapunov/6pus_bone_drill_force_control/bone_ws
source /opt/ros/jazzy/setup.bash
colcon --log-base log/force_stage1 build --base-paths src \
  --build-base build/force_stage1 --install-base install/force_stage1
source install/force_stage1/local_setup.bash
ros2 run pus6_force_control force_pid_offline
```

入口只生成离线帧，即使读取配置中的 baudrate 也不会创建串口。输出 `source=MOCK`、`hardware_write_count=0`。配置不是 ROS 节点参数文件，而是数值组件与离线入口的 YAML 配置。

```bash
cd /home/lyapunov/6pus_bone_drill_force_control
python3 -m pytest -q bone_ws/src/pus6_force_control/test
```

本阶段完成全工作空间 8 包独立构建，并从 `/tmp` 验证已安装入口和三个核心包均来自 Force 安装目录。离线测试覆盖新核心、原 Force 驱动/分配器、继承 IK/几何及 Mock 驱动基础；这不构成真机性能结论。

## 下一阶段

接入正式 ROS2 力控节点，复用已有 `pus6_research_msgs` 消息与现有 UART 单所有者接口，保持回调单槽接收和通信执行分离。将实测力与当前位姿对齐，明确外部接触力的符号、零点和重力/支撑模型。随后实现 Mode 3 的实际进入、退出及通信故障处理，并在获准的硬件实验中调试 PID。驱动内保留的 `is_hardware_qualified` 仅是软件许可字段，不代表已完成硬件验证。
