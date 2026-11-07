> 历史归档：原路径 `docs/research/6PUS_FORCE_POSITION_HARDWARE.md`。正文保留当时的方案、参数及证据语境；当前原生 Mode 3 入口以[项目 README](../../../README.md)为准。历史命令不作为当前推荐启动命令。

# 独立位置伺服外部力反馈入口

此入口不依赖 J14。一个 UART 工作线程同时完成六轴状态读取、`0x29` 位置写入及写入应答解析；复用现有 `Motor` / `UartBus`，运动写入不重试。`--select-servo` 显式允许在当前测量位置选择 `0x25=1`，不会回零、保存参数或写入 `0x27`。未指定时，非伺服状态不会自动切换。

控制链为：原双 IMU SO(3) 前馈和姿态 PID → 原 IK → 六轴长度误差 PID → 实际受力误差 → 现有动态映射 → 可选轨迹整形 → UART 时间限速与米到步数转换 → `0x29`。当前默认力单位为 `relative_raw`，已完成外力标定的配置才使用 N。启动时以首个实际六轴位置为参考，保留后续 IK 相对于该初始姿态的变化；不会主动前往模型 HOME。位置和力来自同一组六轴状态帧，各轴保留独立主机接收时间。这不是设备同时采样。

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

## 真实受力方向只读诊断

`direction_diagnostic.py` 直接复用 `SixAxisPositionDrive.read()` 和已保存的相对零点，只发送六轴状态查询。UART 使用 `read_only=True`，不会发送位置、模式、暂停或原生力控命令，不启动 IMU/ROS 控制节点，也不会修改配置。它必须单独占用 UART，不会自动停止其他进程。已有电机保持状态不由本工具改变。

加载上文环境后，先记录静止基准：

```bash
python3 scripts/research/force_position_control/direction_diagnostic.py --duration 6
```

现场能够对实际轴施加方向已知的轻微轴向载荷时，运行逐轴交互诊断：

```bash
python3 scripts/research/force_position_control/direction_diagnostic.py --interactive --duration 3
```

按提示逐次完成轴 1–6 的静止基准、正向加载、释放、反向加载、释放。每段采集前用回车确认；`positive/negative` 必须对应操作者确认的该轴实际轴向受力正/负方向，不能把手向上推平台简单当作六轴正向受力。不能确定时按 Ctrl+C 结束，勿猜测标签。可用 `--axes 2 --directions positive negative` 仅记录某一轴。Ctrl+C 关闭读取并保留已写数据，不给电机发送停止或其他命令。

输出位于 `results/force_position/direction_<时间>/`：`samples.csv` 为每轴原始力、ADC、保存零点后的 `relative_raw`、位置、主机接收时间及现有控制器计算出的 PID/力误差/修正量；`summary.json` 包含各段均值/标准差/范围、加载增量、释放残差、其他通道响应和逐轴方向分析。计算器每段独立初始化，以该段首个实际位置为期望位置，**计算量不下发、不代表正在运行主动稳定闭环**。释放后的传感器恢复应查看原始/相对读数，不能把分段重置后的计算修正量误当作物理恢复。

响应筛选使用 `max(1 raw, 3×基准标准差)` 作描述性噪声参考，不是标定精度或运动许可。其他通道也响应可能来自并联机构真实载荷重分配，不能直接判为错线。只有正反向已知加载的符号一致时才列出相对比例符号建议，工具始终不修改 `relative_scale`。静止数据不能确认六轴物理正负号；手动加载结果也不能单独证明位置变化到受力变化的反馈符号，完整补偿方向仍需有依据的力—位置物理关系。

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
cd /home/lyapunov/6pus_bone_drill_torque_control
bash bone_ws/scripts/run_force_position_feedback_pid.sh
```

该入口沿用 J14 脚本的使用方式，自动加载 Jazzy 和本仓库 `install/force_position/local_setup.bash`，固定使用 `/usr/bin/python3` 运行独立 UART/双 IMU/力反馈位置补偿程序。默认加载已保存的 `calibrated_config.json`，允许在当前读数处选择位置伺服模式，启动不回 HOME、不增加启动运动。可附加 `--duration 10`、`--log-dir /absolute/path` 等现有运行参数；正常退出时先返回 HOME，再暂停。`--help` 可验证启动环境和参数入口，不连接硬件。

已手动加载隔离环境时，也可使用原 launch 入口：

```bash
cd /home/lyapunov/6pus_bone_drill_torque_control/bone_ws
ros2 launch launch/force_position_hardware.launch.py mode:=active select_servo:=true
```

启动入口优先加载 `scripts/research/force_position_control/calibrated_config.json`，也可用 `config:=/absolute/path.json` 指定配置。`correction_limit_m` 为 `null` 时，不设置独立的对称修正幅值上限，而根据最终位置的可用范围限制积分状态；显式六轴数组仍可用于有明确目的的单次实验。`correction_rate_m_s` 单位 m/s。映射为 `delta_q_dot=A*(F_cmd-F_actual)`，轨迹整形在力修正之后执行。已知外力 N 配置仍支持原 `force_limit_n`、`admittance_m_per_ns` 参数。

当前相对配置的需求限幅约 50.986 relative；速度配置见下文。`F_cmd` 为相对于已保存静载状态的增量需求，零位置误差对应零增量力，不将原有静载误认为不存在。

### 六轴最快参考响应配置

工程硬件资料记载六台型号为 LAF50-024D。[厂家型号表](https://www.inspire-robots.com/actuator/laf-series/)给出空载速度 17 mm/s、满载速度 8 mm/s；本次没有重新核对实体铭牌，也没有测得装配机构的最大加速度。配置采用六轴相同的 **17 mm/s = 0.017 m/s = 680 steps/s** 参考速度上限，实际可达速度依赖载荷、供电和原驱动保护，不将空载参数宣称为装配机构的实测极限。

- 两份配置中的 `controller.correction_rate_m_s` 从 0.025 mm/s 提高到六轴各 17 mm/s，只改变修正速度限幅；`delta_q_dot=A*e_F` 的映射系数、PID 增益、相对零点、比例和方向不变。因此，小力误差通过原映射产生的速度仍可能很小，不会为了达到峰值速度而放大力误差。
- `shaper.enabled=false` 绕过额外的三阶目标滤波，`shaped_command_m` 此时等于完整的 `position_command_m`。保持旧配置默认启用滤波的兼容行为，不改变原 J14 控制链。
- `max_cycle_delta_steps=null` 取消旧的每周期 2 步试验限制。唯一 UART 所有者在最终写入前，按实际单调时钟间隔及六轴速度上限连续限制目标变化，保留浮点余量再编码，避免每周期四舍五入造成速度长期累积超差。`position_acknowledged` 同时记录期望 `requested_steps` 和真正发送的 `targets_steps`。
- `max_accel_m_per_sec2` 和 `max_jerk_m_per_sec3` 为 `null`，表示不叠加这两个应用层限幅，不是无限物理加速度，也不是已确认的厂家加速度数值。驱动器原有位置插补、输出/电流/温度保护保持原样；没有写入保护阈值。若重新启用三阶滤波，`null` 同样只取消显式加速度/jerk 截断，保留该滤波器原有带宽。
- 控制目标周期仍为 20 ms；六轴 UART 事务和实际反馈决定有效更新率，不通过盲目缩短周期假装提高硬件带宽。此时参考速度为每名义周期 13.6 步，已不再受原 2 步限制。原始目标与最终编码目标均检查 0–2000 steps、行程模型和实际驱动限位；通信故障和退出暂停逻辑不变。

手册规定 `0x28` 目标速度只在速度模式有效，本入口仍保持 `0x25=1`、写 `0x29`，不会用其他模式或最大电压编码冒充伺服提速。上述速度链、六轴独立限速及可变周期下的最终位置协议输出通过软件测试；尚未进行此次配置的真机最快运动测试。

### 修正幅值与原机构位置范围

此前两个配置中的 `correction_limit_m=[5e-05]*6` 来自首次独立 UART 接入时的未整定小幅试验配置（`b7a9d83`），不是机构参数或机械约束；它把动态映射的积分状态 `correction_m` 截断在 ±50 μm。另一个 `max_start_delta_steps=20` 会限制相对启动位置的总位移至 ±0.5 mm。两个默认值现均设为 `null`，不再以试验幅度限制正常运动范围。

**路线 A 的位置范围依据为原位置控制工程的 0–2000 steps。** 原 `safety.validate_target_vector()` 使用绝对步数范围校验；几何配置中的旧 `normal_motion_limits_steps=200–1800` 和 `software_command_limits_steps=0–1800` 是另外的规划配置，不再用于缩小本研究链的补偿边界。代码读取 `absolute_raw_limits_steps`，与原行程模型以及实际读取的 `0x24/0x23` 下/上限取交集；不写入或放宽实际驱动限位，不改变 IK 可达性约束。

单位换算已对照原工程 `safety.py`、`j14_feedback_bridge.py` 和 `6pus_geometry.yaml`：1 mm = 40 steps，1 step = 0.025 mm = 0.000025 m。必须区分两个量：相对原始 0 步的行程位移为 `s=steps*0.000025 m`，所以 0–2000 步对应 0–50 mm 位移；控制器使用的是既有 IK/反馈模型坐标 `q=0.05+(steps-1000)*0.000025 m`，所以其位置范围为 0.025–0.075 m，HOME 1000 步对应 `q=0.05 m`。这里的 25–75 mm 是带模型偏置的位置坐标，不是重新测量得到的电缸本体绝对长度。反向编码为 `steps=round(1000+(q-0.05)/0.000025)`，偏置不能遗漏或重复添加。

每轴实时可用修正范围为 `q_min[i]-q_d[i] <= correction_m[i] <= q_max[i]-q_d[i]`。例如 `q_d=0.05 m` 且实际驱动限位不更窄时，可修正 −25 至 +25 mm；当目标靠近某端时，相应方向的余量随之减小。积分状态在这一范围内截断，避免超过边界后继续积累。该执行器范围不表示任意六轴组合都具有相同的末端姿态工作空间；原 IK 可达性判断保持不变。

传递顺序为 `correction_m → position_command_m=q_d+correction_m → shaped_command_m → UART 时间限速 → HOME 偏置换算/四舍五入 → 0x29`。当前 `shaper.enabled=false` 不再增加额外滤波滞后；最终驱动检查位置范围并按速度限制连续推进目标，不添加 50 μm 幅值上限。一个位置步为 25 μm，因此最终目标存在最多半步的量化误差。软件测试验证了超过 50 μm 的补偿传递，以及包含 0 和 2000 步两个端点的完整范围逐周期协议写入；未执行本次修改后的真机运动测试，历史小幅运行记录保持原样。

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

### 初始化、闭环计算与反馈符号

旧字段 `primed` 表示“本次调用仅完成首帧初始化”，不是“控制器是否已就绪”：首个有效六轴样本为 `true`，后续正常计算均为 `false`。旧 ROS 节点在首帧直接返回、没有发布该帧诊断，所以历史运行日志只记录到 `primed=false`；不能据此判断闭环未激活。现在首帧也发布诊断：`initialized=true, control_active=false`，且不发布位置命令；第二组有效同步反馈起为 `initialized=true, control_active=true`。故障后 `control_active=false` 并停止发布位置命令。保留 `primed` 的旧语义，避免改变历史数据含义。`control_active` 仅表示本控制器正在计算，不代表电机已使能或 UART 已应答，实际发送状态仍查看 `uart_state` 和 `position_acknowledged` 日志。

各轴依次执行 `raw-zero → signed scale → F_actual`，`F_cmd=PID(q_d-q)`，`e_F=F_cmd-F_actual`，再由动态映射积分得到 `correction_m`。因此在位置未变化时，单独改变受力会立即改变力误差和位置修正，**不会直接改变以位置误差为输入的 PID 输出**；执行器位置反馈变化后，位置 PID 才随之响应。逐轴测试覆盖两个原始受力变化方向、有符号比例、通道独立性，以及实际位置反馈返回 PID 的路径。

正映射系数下，正的实际受力偏差使修正朝负位置方向变化；负比例或负映射系数按配置反向处理。当前保存的六轴比例均为 +1，本次未更改方向或增益。软件符号和索引验证不等于每轴物理受力方向已标定，不能仅凭静止日志推断安装后的负反馈方向。启用轨迹整形时存在动态滞后；默认最快参考配置关闭这层滤波，但仍保留 UART 时间限速。小于半个位置步的修正可能暂时不改变 `0x29` 整数目标，这不是反馈被丢弃。

停止正在运行的本入口：

```bash
ros2 topic pub --once -w 1 /force_position/stop std_msgs/msg/Empty '{}'
```

前台 Ctrl+C、上述 stop 话题和 `--duration` 正常到期，都会停止接收控制目标，读取六轴实际位置，由同一个 UART 线程生成共享进度的五次平滑轨迹，通过 `0x29` 协调返回几何配置中的 `operating_home_raw_steps`（当前六轴均为 1000 steps）。轨迹沿用已配置速度限制，不改变 PID、映射、标定或行程。返回期间继续发布六轴反馈；最终发送 HOME 目标后，需两次连续的实际状态查询确认全部进入现有 HOME 反馈容差（±5 steps），才逐轴发送一次 `0x1A=1` 并检查六轴应答。HOME 和暂停完成前不会关闭 UART、日志及 ROS 节点。

日志中依次记录 `home_started`、`home_confirmed`、`pause_complete`、`uart_stopped`；暂停未确认或 HOME 超时均以错误退出。只读模式不返回 HOME、不写寄存器。通信故障、位置反馈失效或驱动故障退出时跳过 HOME，沿用暂停路径；返回过程中遇到同类故障立即中止返回并尝试暂停，不重试位置写入。通信失效时只能报告暂停未确认，软件命令不能保证断线后的物理停止。不要另外启动串口驱动来发送停止，本入口持有唯一串口。

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
