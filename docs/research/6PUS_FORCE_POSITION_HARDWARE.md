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

## 实际力标定

厂家将 `0x2C` 标为 g；若其含义是克力，名义关系为 `1 gf = 0.00980665 N`。本入口不据此猜测六轴已装配传感器的零点与正负方向，而是使用 `F_N=(raw-raw_zero)*newtons_per_raw`。`0x2D` ADC 同时读取和记录，不作为已标定力。

配置模板 `scripts/research/force_position_control/hardware_config.json` 的 `calibration` 为 `null`，明确表示尚无实测标定。未标定时只发布原始力；不会发布零牛顿伪装实测力。实际力闭环需要填入标定结果。

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

标定文件存在且参数已根据现场响应设置后：

```bash
ros2 launch launch/force_position_hardware.launch.py mode:=active select_servo:=true \
  config:=/home/lyapunov/6pus_bone_drill_torque_control/bone_ws/scripts/research/force_position_control/calibrated_config.json
```

`controller` 参数每项有六个值：`kp` N/m、`ki` N/(m·s)、`kd` N·s/m；`force_limit_n` N；`admittance_m_per_ns` m/(N·s)，其符号对应位置变化引起的实际力变化方向；`correction_limit_m` m、`correction_rate_m_s` m/s。映射为 `delta_q_dot=A*(F_cmd-F_actual)`，包含积分状态和修正幅值/速度限制。轨迹整形在力修正之后执行。

模板参数是未整定的起始值：力命令限幅 0.5 N、力修正限幅 0.05 mm、修正速度 0.025 mm/s；整个位置目标限制在启动读数附近 ±20 步。它们不是已验证的稳定参数。`F_cmd` 为绝对轴向力时，`ki=0` 且位置误差为零会要求零力，不能据此声称承重平衡；现场整定须同时考虑实际静载、力限幅和积分项。本轮没有执行这项力闭环整定。

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
ros2 topic echo /force_position/force_feedback
ros2 topic echo --full-length /research_force_position/diagnostics
ros2 topic echo --full-length /force_position/uart_state
```

`attitude` 包含基座/平台相对初始姿态的 RPY（rad）。`diagnostics` 包含 `force_cmd_n`、`force_actual_n`、`force_error_n`、`correction_m` 和整形后的目标。未标定只读模式不发布牛顿力，也不运行力控制器，因此这两类话题不产生对应数据。

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
- 物理力标定、力闭环参数整定和基座扰动姿态稳定实验尚未执行；未伪造六轴已知外力或标定参数。
