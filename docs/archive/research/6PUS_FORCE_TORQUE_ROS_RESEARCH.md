> 历史归档：原路径 `docs/research/6PUS_FORCE_TORQUE_ROS_RESEARCH.md`。正文保留当时的方案、参数及证据语境；当前原生 Mode 3 入口以[项目 README](../../../README.md)为准。历史命令不作为当前推荐启动命令。

# 6PUS 双 IMU、力与力矩研究链

这是独立研究链，默认不连接真实执行器。`research_entry.py` 仅接受 `backend=mock`，无串口创建代码；样例配置仅供合成数据测试，不能用于真实设备。任何真实模式切换、使能和运动须单独获得授权并完成后续实机验证。

## 数据流

`research_imu_lengths` 订阅私有 `base_imu`、`platform_imu`（`sensor_msgs/Imu`），使用配置中明确给出的两个参考四元数。复用 J11 的相对四元数、+90° 机械坐标变换、三轴世界坐标 SO(3) PID 和前馈/残差合成，产生 `pus6_base` 内姿态；调用已有 `Analytical6PusKinematics`（米、弧度），再经 `StatefulJerkLimitedShaper` 产生六轴期望滑块位置（m）。模型 HOME 对应约 0.050 m、1000 steps；参考四元数不是自动校准结果。节点将输出发至 `/research_force_torque_mock/desired_lengths_m`。

`research_force_torque_mock` 接收研究输入 `position_feedback`（现有 `TimedJointFeedback`）及期望长度，PID_TORQUE 产生 `TorqueCommand`（N·m），ACTUATOR_FORCE_CONTROL 的位置 P 外环产生 `AxialForceCommand`（N）；外环增益为 N/m，内层使用电动缸手册描述的驱动器力闭环，不再增加力 PID。力模式也要求有效的 `ForceFeedback`，诊断包含力参考、测量、误差。统一入口选择 `DS_SCTDE` 时启动既有独立闭环仿真，而不是 ROS 执行器控制；其研究资产未改动。

IMU 同时检查 ROS header 时间与本机接收时钟的新鲜度；回放必须使用与消息相同的 ROS 时钟（例如正确配置 `use_sim_time`），并持续推进。header 不证明传感器内部采样时间。位置和力采用各轴主机 CLOCK_MONOTONIC 完成时间，只适合同一主机；它们不是硬件采样时钟。缺失、无效、过期或故障数据会阻止输出或锁存故障；上游 IMU 故障后目标不再更新，控制节点随后因目标超时锁存。研究节点内控制模式不可重复切换，不代表跨进程硬件独占。

## LAF 反馈和接口

手册描述 LAF 带力传感器，状态帧有力传感器值（g），寄存器 `0x2C` 为实际受力（g）、`0x2D` 为原始 ADC；驱动 `unpack_status_response` 解析状态帧中的有符号 `force_g`、ADC 和错误码。J14 UART/IPC/Feedback Bridge 未传递该力字段，原消息中的零值不是有效测量。

`ReadOnlyStatusSource` 可在**已有且唯一拥有总线**的进程中使用已有 bus 发起状态读取，返回原始帧与接收完成时间；它不创建或打开端口。`StatusSourcePublisher` 可由该 I/O 工作线程调用以发布采样，不在 ROS 订阅回调内阻塞。`RawMotorStatus` 携带轴 ID、来源（Mock 或硬件）、原始帧和时间；`research_force_status` 验证完整帧、ID、命令、顺序、来源及时间后，按轴发布 `ForceFeedback`；统一入口启用位置桥，将状态帧中的 `current_position` 依据既有几何配置转换为 `TimedJointFeedback` 米值并带各轴接收时间。该流没有 UART 原始周期号，`uart_cycle_sequence=0` 表示未提供，六轴可能来自不同采样时刻。默认 `gram_force_confirmed=false`，全部反馈无效；只有外部确认手册的 g 实际代表 gf 后才允许按 `1 gf = 0.00980665 N` 转换并标记有效。手册使用 g，但本次未找到明确的 gf 定义或真实量程、零点及正负方向，因此**不得将此假设视为已验证**。Mock 测试显式打开这一假设。来源标志由上游提供，软件无法据此独立证明传感器真实性。

尚未将 J14 已有 UART 所有者接入该新原始帧接口，也没有运行真实采集。使用未来真实数据时，必须在唯一 UART 所有者内接入状态帧发布并确认无竞争；不能并行开启第二个串口采集进程。

`LafForceAdapter` 是注入已拥有 bus 的候选软件接口：明确调用 `set_force_mode_explicit(initial_force)` 才按手册 3.5.6 写同一轴的 `0x25..0x27=[3,0,目标值]`，`write_force()` 写 `0x27`，并限制非负和配置量程。它不打开串口，不由研究入口实例化。手册给出 1000 g 的正数示例，负力命令编码和完整量程未确认，代码不假定负命令；故障不重试，最多一次请求各轴暂停。模式切换、命令、反馈、暂停和转化均未在真实设备验证。`PID_TORQUE` 仍只有独立 N·m Mock 适配器，未找到可确认的真实电机轴力矩寄存器，绝不写入 `0x29` 或用轴向力冒充力矩。

## 构建与一条命令启动

在研究仓库 `bone_ws` 中执行；仅使用 ROS Jazzy 与新建的独立安装目录：

```bash
source /opt/ros/jazzy/setup.bash
colcon --log-base /tmp/6pus_research_log build --base-paths src --build-base /tmp/6pus_research_build --install-base /tmp/6pus_research_install
source /tmp/6pus_research_install/setup.bash
ros2 launch launch/force_torque_research.launch.py
```

默认是 `ACTUATOR_FORCE_CONTROL`、`backend=mock`、合成输入，持续运行直到 Ctrl+C。入口按顺序创建控制器、IMU/IK、状态桥、日志和合成输入，使用单个 ROS executor；合成输入的 IMU、固定位置和固定力均不是硬件测量，也不是物理闭环动力学仿真。测试参数和参考四元数不能用于真机。输出 `LOG_DIR` 指向新建的 `results/research_control/session-*`，内含不可覆盖的配置快照与 `telemetry.jsonl`，记录 IMU/IK、每轴控制状态、命令、故障与退出事件。正常退出或故障会停止 Mock 后端；这不是实机停止保证。

选择 PID 力矩 Mock，并在 2 秒后退出：

```bash
ros2 launch launch/force_torque_research.launch.py mode:=PID_TORQUE duration:=2
```

选择轴向力 Mock：

```bash
ros2 launch launch/force_torque_research.launch.py mode:=ACTUATOR_FORCE_CONTROL
```

运行独立的 DS-SCTDE 既有闭环仿真（输出在该次 `LOG_DIR/ds_sctde`，不连接 ROS 执行器链）：

```bash
ros2 launch launch/force_torque_research.launch.py mode:=DS_SCTDE
```

`input:=external` 禁用合成源，可向研究话题提供来源明确的离线数据。默认状态桥限定 Mock 来源；可以设置 `input:=external status_source:=hardware` 接收由现有所有者发布、标记为硬件的状态帧，但本进程不创建串口或驱动，并仍只输出到 Mock。力模式需在配置中显式设置 `force_unit_confirmed=true`；该标志不能替代厂家确认。缺失输入时会超时并退出。`config:=/absolute/path.json` 可选择统一 JSON 配置，`log_dir:=/new/path` 指定不存在的新日志目录；默认配置在 `scripts/research/force_torque_control/example_mock_config.json`。Launch 参数 `mode` 和 `input` 覆盖 JSON 对应项。也可直接运行：

```bash
python3 scripts/research/force_torque_control/research_entry.py --config scripts/research/force_torque_control/example_mock_config.json --mode PID_TORQUE --duration 2
```

检查和复现测试：

```bash
python3 -m pytest -q scripts/research/force_torque_control
for d in joint_torque_pid ds_sctde ds_sctde_closed_loop ds_sctde_realistic; do python3 -m pytest -q "scripts/research/$d" || break; done
```

独立安装的是 ROS 包和消息；研究入口与 J11 算法仍从此研究仓库的源码树运行。未提供真实后端：任何非 `mock` 配置会在启动前拒绝，不能直接控制真机。将来需要确认厂家力单位、量程和正负方向，在唯一 UART 所有者中接入状态帧与控制适配、核实六轴映射及获得授权后，才能另外开发和验证实际设备入口。PID 电机轴力矩接口的协议仍需厂家提供；不能用 `0x29` 或轴向力寄存器代替。

## 真实传感器只读、输出仍为 Mock

在确认串口没有被其他进程使用后，可以显式启动以下模式，读取六轴状态和两只 H30 的数据并将它们送入 PID 研究链；所有 PID 命令只进入内存 Mock，**没有真实力矩或力命令**：

```bash
ros2 launch launch/force_torque_research.launch.py \
  input:=hardware-readonly status_source:=hardware mode:=PID_TORQUE duration:=4
```

此入口占用 `/dev/ttyUSB0`、基座 H30 的 `5B32028000` 和平台 H30 的 `5B0B029000`，故必须在已运行的 J14 或其他驱动停止使用这些端口后执行。电机 UART 经 pyserial 独占锁并启用 `UartBus(read_only=True)`，外层额外只允许 `0x30` 状态查询；H30 驱动只读打开。线程在后台顺序读取六轴；必须六轴各有新样本后才发布新一组位置，避免混用旧轴时间戳。读取与 ROS 消息回调分离。退出时关闭本进程打开的端口与 H30 子进程；`readonly_audit` 记录状态查询和寄存器写计数。

入口以首次接收到的有效 IMU 四元数作为**相对参考**，记录在日志中；它不是经过静止性验证的校准。HOME 一致性也只表示当前电机读数与软件参考接近，不证明物理安全或绝对位置。力传感器原始字段记录为 `force_g_raw` 和 ADC，未经厂家单位及传感器校准确认时，不标记为 SI 力反馈。

2026-10-28 的一次只读扫描取得 6 个轴状态和模式寄存器，模式均为 0，位置为 `[999, 998, 998, 1001, 1000, 998]` 步，错误码均为 0。随后 4 秒实际双 IMU 与六轴状态进入 PID 的 Mock 输出流程，记录 478 次状态查询、0 次寄存器写、76 条 Mock 力矩输出。首次尝试因位置桥发布混合新旧轴时间戳而报 `invalid feedback interval`，已修复为六轴全部更新后发布并重测。原始记录保存在本机 `results/research_control_readonly/`；真实力控、力矩执行、硬件停止及闭环性能均未验证。

## 2026-10-29：真实原始力反馈链

新增 `RawForceFeedback`：包含六轴 `force_g_raw`、力传感器 ADC、各轴主机接收时间、设备错误码、来源与有效位。有效位仅表示状态帧可解析、接收时间新鲜、设备未报告错误且 ADC 在手册范围内；**不表示单位、零点、方向、量程或物理测量精度已标定**。研究控制节点记录这份消息，PID 力矩路径不把原始数值转换成 N，也不把它代入力控制器。需要物理参数确认后才可将校准的 `ForceFeedback`（N）用于力控。

只读预检显示六轴 `0x25=0`、`0x27=0`，`0x1A` 读值为 1；寄存器读值本身不证明独立急停或实际停止功能。力传感器原始数值存在明显的正负差异。厂家手册指出力控模式会主动调整位置以接近目标力；因此，若直接以现有目标零值切换力控，很可能产生非预期移动。不得依据该读数直接执行模式切换。型号的证据仍来自工程文档，六轴实体铭牌/固件、g 定义、符号、标定和受力状态仍未确认。

2026-10-29 的 3 秒只读验证使用真实双 IMU 与六轴状态，控制器诊断中 55 个周期携带全部六轴新鲜的原始力与 ADC；360 次 `0x30` 状态查询，寄存器写入为 0。控制输出仍进入 Mock；日志在 `results/research_control_readonly/raw-force-20261029/`。验证没有执行真实模式切换、力命令或停止测试。

## 单轴位置验证候选工具

新增 `position_probe_cli.py`，作为独立的、单轴一次性位置测试入口。它使用独占串口和现有 `Motor`/`UartBus` 协议，先读取所选轴位置、故障、模式及驱动上下限，不做回零或自动模式切换；模式必须为 0。目标按最新读取的位置加上明确给定的步数偏移计算，再次读取确认起点没有改变，目标不能超过用户明确提供的位移范围及设备寄存器限位。仅在明确指定 `--execute` 后打开真实串口并尝试发送一次 `0x29`，不重试；发生错误也最多尝试一次 `0x1A` 暂停。之后记录连续三个状态帧。

这条路径的输入是 **位置 steps**，不接收力矩或轴向力；它不改变 PID 的 N·m 输出语义，也不是姿态稳定闭环。串口应由本进程独占，不能与 J14 或其他驱动同时运行。停机应有现场独立急停或断电手段，命令方向、实际允许运动范围、环境和操作者均须在测试前确认。设备寄存器限位、串口应答和读取到的稳定步数不能证明物理停止或排除碰撞。**本次尚未运行真实位置探针，也没有验证该工具的真实停止行为。**

`--help` 可查看参数。实际执行必须按现场确认的轴号、步数范围、允许观察时间和容差填写参数；`--log-dir` 必须指向不存在的新目录。每次运行记录命令尝试、应答、反馈、暂停尝试和结果；它不自动返回 HOME。Mock 验证了类型隔离、步数范围检查、单次发送、异常不重试及暂停尝试，并不构成硬件验证。
