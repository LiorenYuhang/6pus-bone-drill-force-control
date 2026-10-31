# 独立位置伺服外部力反馈入口

此入口不依赖 J14。一个 UART 工作线程同时完成六轴状态读取、`0x29` 位置写入及写入应答解析；复用现有 `Motor` / `UartBus`，运动写入不重试。`--select-servo` 显式允许在当前测量位置选择 `0x25=1`，不会回零、保存参数或写入 `0x27`。未指定时，非伺服状态不会自动切换。

控制链为：原双 IMU SO(3) 前馈和姿态 PID → 原 IK → 六轴长度误差 PID（N）→ 实际受力误差 → 现有动态映射 → 原轨迹整形 → 米到步数转换 → UART `0x29`。启动时以首个实际六轴位置为参考，保留后续 IK 相对于该初始姿态的变化；不会主动前往模型 HOME。位置和力来自同一组六轴状态帧，各轴保留独立主机接收时间。这不是设备同时采样。

## 构建

```bash
cd /home/lyapunov/6pus_bone_drill_torque_control/bone_ws
source /opt/ros/jazzy/setup.bash
colcon --log-base log/force_position build --base-paths src \
  --build-base build/force_position --install-base install/force_position
source install/force_position/setup.bash
```

研究脚本从本仓库源码运行，消息和驱动从上述独立安装目录加载。

## 当前默认：静止参考状态的相对受力清零

`calibrated_config.json` 已保存 2026-10-31 当前静止平放状态的六轴相对零点。连续采集约 8 秒，每轴 399 个实际 `0x2C` 样本，均值为 `[3520.067669, -277.994987, -6214.275689, 672.0, -1125.225564, -6885.784461]`。清零期间寄存器写入为 0。该状态包含原有重力和静载，定义为相对受力零点，不是物理无载状态。

后续启动自动读取已保存的配置，**不会自动重新清零**。重新清零前先停止本运行程序，使 UART 由清零工具单独使用。在机构重新静止于所需参考状态后执行：

```bash
python3 scripts/research/force_position_control/calibration.py --tare --duration 8 \
  --config scripts/research/force_position_control/calibrated_config.json \
  --output scripts/research/force_position_control/calibrated_config.json --replace
```

该命令保留已整定的相对 PID 参数和六轴方向/比例，只更新零点及采集信息。`calibration.raw_zero` 为六个独立偏置；`calibration.relative_scale` 为六个独立有符号比例，当前初值均为 1，保留协议报告的原始正负号。此比例可以逐轴调整，不宣称已完成物理方向或绝对力标定。

运行时采用 `delta_force_raw=raw-raw_zero`，`force_actual_relative=delta_force_raw*relative_scale`。PID 需求、受力反馈和力误差统一使用 `relative_raw` 单位。配置中的 `kp/ki/kd` 分别为 relative/m、relative/(m·s)、relative·s/m；`force_limit_relative` 为相对量限幅，`admittance_m_per_relative_s` 为 m/(relative·s)。六轴参数均独立。初始数值按手册名义 gf 比例从原参数换算，仅用于选择起始增益，不构成 N 标定。

相对模式不会发布 `/force_position/force_feedback` 的牛顿值。`uart_state.force_n` 为 `null`；实际相对反馈在 `/force_position/relative_force` 和控制诊断中的 `force_actual_relative`，期望及误差为 `force_cmd_relative`、`force_error_relative`。当前状态的采样均值为零，并不表示后续每个有噪声或载荷变化的读数都应强制为零。

## 可选：已知外力的牛顿标定

厂家将 `0x2C` 标为 g；若其含义是克力，名义关系为 `1 gf = 0.00980665 N`。本入口不据此猜测六轴已装配传感器的零点与正负方向，而是使用 `F_N=(raw-raw_zero)*newtons_per_raw`。`0x2D` ADC 同时读取和记录，不作为已标定力。

配置模板 `scripts/research/force_position_control/hardware_config.json` 的 `calibration` 为 `null`。当前默认使用已保存的相对零点，运行相对受力闭环；只有需要 N 单位时才需以下已知外力标定。

对每个轴使用可读出实际轴向力的测力仪或厂家该轴标定数据。选择两个稳定且不同的已知轴向受力状态，定义伸长方向的执行器推力为正，分别记录六轴的真实受力。安装状态下不能把平台上新增砝码的重量简单平均分给六轴，也不能把承重 HOME 的当前读数当成零受力。若不能独立获得各轴真实受力，需由现场人员完成这项物理测量。

分别在两个已知受力状态采集（只读、不改变电机模式）：

```bash
ros2 launch launch/force_position_hardware.launch.py mode:=read-only duration:=5 \
  log_dir:=/home/lyapunov/6pus_bone_drill_torque_control/results/force_position/calibration_first
ros2 launch launch/force_position_hardware.launch.py mode:=read-only duration:=5 \
  log_dir:=/home/lyapunov/6pus_bone_drill_torque_control/results/force_position/calibration_second
```

在两次采集之间由现场人员改变已知轴向受力。以下 `F0_1…F1_6` 必须替换为测力仪读数（N），不是示例标定数值：

```bash
python3 scripts/research/force_position_control/calibration.py \
  --config scripts/research/force_position_control/hardware_config.json \
  --first-run ../results/force_position/calibration_first \
  --second-run ../results/force_position/calibration_second \
  --first-force-n F0_1 F0_2 F0_3 F0_4 F0_5 F0_6 \
  --second-force-n F1_1 F1_2 F1_3 F1_4 F1_5 F1_6 \
  --output scripts/research/force_position_control/calibrated_config.json
```

工具逐轴拟合有符号比例和零点，并保留输入的已知力、原始均值与标准差。两点结果只支持仿射标定，不证明量程内精度。已知轴向力无法由目前的静态原始数据独立推断。

## 真机启动

使用本轮已保存的相对零点，完整路线 A 直接启动：

```bash
ros2 launch launch/force_position_hardware.launch.py mode:=active select_servo:=true
```

启动入口优先加载 `scripts/research/force_position_control/calibrated_config.json`，也可用 `config:=/absolute/path.json` 指定配置。`correction_limit_m` 单位 m、`correction_rate_m_s` 单位 m/s。映射为 `delta_q_dot=A*(F_cmd-F_actual)`，包含积分状态和修正幅值/速度限制。轨迹整形在力修正之后执行。已知外力 N 配置仍支持原 `force_limit_n`、`admittance_m_per_ns` 参数。

当前相对配置的需求限幅约 50.986 relative、修正限幅 0.05 mm、修正速度 0.025 mm/s；整个位置目标限制在启动读数附近 ±20 步。它们已用于本轮静止运行，不代表扰动抑制性能已经验证。`F_cmd` 为相对于已保存静载状态的增量需求，零位置误差对应零增量力，不将原有静载误认为不存在。

无需力标定的当前位置保持命令，仅验证真实六轴位置通信及停止，不运行力控制器：

```bash
ros2 launch launch/force_position_hardware.launch.py mode:=active hold_current:=true \
  select_servo:=true duration:=2
```

## 监控与停止

在另一终端加载相同环境后，分别查看：

```bash
ros2 topic echo --full-length /force_position/attitude
ros2 topic echo /force_position/position_feedback
ros2 topic echo /force_position/raw_force_feedback
ros2 topic echo /force_position/relative_force
ros2 topic echo /force_position/force_feedback
ros2 topic echo --full-length /research_force_position/diagnostics
ros2 topic echo --full-length /force_position/uart_state
```

`attitude` 包含基座/平台相对初始姿态的 RPY（rad）。当前相对模式的 `diagnostics` 包含 `force_cmd_relative`、`force_actual_relative`、`force_error_relative`、`correction_m` 和整形后的目标。已知外力标定的 N 模式才使用 `force_cmd_n/force_actual_n/force_error_n` 并发布 `force_feedback`。只读模式不运行力控制器。

停止正在运行的本入口：

```bash
ros2 topic pub --once -w 1 /force_position/stop std_msgs/msg/Empty '{}'
```

前台 Ctrl+C 同样退出，并向已开始写入的六轴各尝试一次 `0x1A=1` 暂停。通信失效时只能报告暂停未确认，软件命令不能保证断线后的物理停止。不要另外启动串口驱动来发送停止，本入口持有唯一串口。

日志自动保存在 `results/force_position/<时间>/`，含 `config.json`、`telemetry.jsonl` 和两只 IMU 的日志。正常和异常退出均记录；错误运行不覆盖成功运行。

## 本轮实际结果

- `20261031-170515`：真实传感器只读，160 组六轴反馈、192 条姿态记录，寄存器写入 0。
- `20261031-170532`：尝试当前位置保持，轴 1 的模式写入应答后，额外模式寄存器回读仍为 0 导致启动退出；六轴暂停均收到应答。随后参照已有 J7 的启动方式修复为依赖模式写入应答，不以该回读作为伺服使能条件。没有自动重试失败写入。
- `20261031-170601`：独立 UART 六轴当前位置保持完成，414 条周期 `0x29` 命令应答，另有 6 条当前目标预置、6 条模式选择和 6 条退出暂停；错误码均为 0。保持期间编码器范围分别为 `[1,1,0,2,2,1]` 步；这不等于外部末端精度或力控性能。
- `20261031-170655`：再次运行当前位置保持，通过 ROS 停止话题结束，六轴暂停均收到应答。
- `zero_20261031-180832`：按用户指定的静止平放状态采集相对零点，每轴 399 样本；保存六个独立均值、标准差、位置范围及原始数据。未进行执行器寄存器写入。
- `20261031-180912`：相对受力路线 A 实机运行约 5 秒，170 个有效控制计算周期，1020 条周期位置命令应答，六轴错误码均为 0，暂停均收到应答。
- `20261031-181044`：相对受力路线 A 实机运行约 13.8 秒，516 个有效控制计算周期，3138 条周期位置命令应答；通过 ROS 停止话题退出，暂停均收到应答。六轴最大力修正量分别约 `[3.622,1.295,2.628,4.183,11.085,12.050]` μm，均低于 50 μm 限幅。真实受力变化参与了控制计算，非当前位置保持分支。
- 上述相对闭环日志均明确 `force_unit=relative_raw`、`force_n=null`。绝对 N 标定、已知外部加载试验和基座手动扰动的姿态稳定性能尚未验证。
