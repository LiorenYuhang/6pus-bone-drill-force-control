# 6PUS 双 IMU、力与力矩研究链

这是独立研究链，默认不连接真实执行器。`research_entry.py` 仅接受 `backend=mock`，无串口创建代码；样例配置仅供合成数据测试，不能用于真实设备。任何真实模式切换、使能和运动须单独获得授权并完成后续实机验证。

## 数据流

`research_imu_lengths` 订阅私有 `base_imu`、`platform_imu`（`sensor_msgs/Imu`），使用配置中明确给出的两个参考四元数。复用 J11 的相对四元数、+90° 机械坐标变换、三轴世界坐标 SO(3) PID 和前馈/残差合成，产生 `pus6_base` 内姿态；调用已有 `Analytical6PusKinematics`（米、弧度），再经 `StatefulJerkLimitedShaper` 产生六轴期望滑块位置（m）。模型 HOME 对应约 0.050 m、1000 steps；参考四元数不是自动校准结果。节点将输出发至 `/research_force_torque_mock/desired_lengths_m`。

`research_force_torque_mock` 接收研究输入 `position_feedback`（现有 `TimedJointFeedback`）及期望长度，PID_TORQUE 产生 `TorqueCommand`（N·m），ACTUATOR_FORCE_CONTROL 的位置 P 外环产生 `AxialForceCommand`（N）；外环增益为 N/m，内层使用电动缸手册描述的驱动器力闭环，不再增加力 PID。力模式也要求有效的 `ForceFeedback`，诊断包含力参考、测量、误差。`DS_SCTDE` 仍是单独的既有研究实现，在本入口不可选；其研究资产未改动。

IMU 同时检查 ROS header 时间与本机接收时钟的新鲜度；回放必须使用与消息相同的 ROS 时钟（例如正确配置 `use_sim_time`），并持续推进。header 不证明传感器内部采样时间。位置和力采用各轴主机 CLOCK_MONOTONIC 完成时间，只适合同一主机；它们不是硬件采样时钟。缺失、无效、过期或故障数据会阻止输出或锁存故障；上游 IMU 故障后目标不再更新，控制节点随后因目标超时锁存。研究节点内控制模式不可重复切换，不代表跨进程硬件独占。

## LAF 反馈和接口

手册描述 LAF 带力传感器，状态帧有力传感器值（g），寄存器 `0x2C` 为实际受力（g）、`0x2D` 为原始 ADC；驱动 `unpack_status_response` 解析状态帧中的有符号 `force_g`、ADC 和错误码。J14 UART/IPC/Feedback Bridge 未传递该力字段，原消息中的零值不是有效测量。

`ReadOnlyStatusSource` 可在**已有且唯一拥有总线**的进程中使用已有 bus 发起状态读取，返回原始帧与接收完成时间；它不创建或打开端口。`StatusSourcePublisher` 可由该 I/O 工作线程调用以发布采样，不在 ROS 订阅回调内阻塞。`RawMotorStatus` 携带轴 ID、来源（Mock 或硬件）、原始帧和时间；`research_force_status` 验证完整帧、ID、命令、顺序、来源及时间后，按轴发布 `ForceFeedback`。默认 `gram_force_confirmed=false`，全部反馈无效；只有外部确认手册的 g 实际代表 gf 后才允许按 `1 gf = 0.00980665 N` 转换并标记有效。手册使用 g，但本次未找到明确的 gf 定义或真实量程、零点及正负方向，因此**不得将此假设视为已验证**。Mock 测试显式打开这一假设。来源标志由上游提供，软件无法据此独立证明传感器真实性。

尚未将 J14 已有 UART 所有者接入该新原始帧接口，也没有运行真实采集。使用未来真实数据时，必须在唯一 UART 所有者内接入状态帧发布并确认无竞争；不能并行开启第二个串口采集进程。

`LafForceAdapter` 是注入已拥有 bus 的候选软件接口：明确调用 `set_force_mode_explicit()` 才写 `0x25=3`，`write_force()` 写 `0x27`，并限制非负和配置量程。它不打开串口，不由研究入口实例化。手册给出 1000 g 的正数示例，负力命令编码和完整量程未确认，代码不假定负命令；故障不重试，最多一次请求各轴暂停。模式切换、命令、反馈、暂停和转化均未在真实设备验证。`PID_TORQUE` 仍只有独立 N·m Mock 适配器，未找到可确认的真实电机轴力矩寄存器，绝不写入 `0x29` 或用轴向力冒充力矩。

## 离线运行

在研究仓库 `bone_ws`：

```bash
source /opt/ros/jazzy/setup.bash
colcon --log-base /tmp/6pus_research_log build --base-paths src --build-base /tmp/6pus_research_build --install-base /tmp/6pus_research_install
source /tmp/6pus_research_install/setup.bash
python3 scripts/research/force_torque_control/research_entry.py --config scripts/research/force_torque_control/example_mock_config.json
```

这是源码树研究入口，构建安装 ROS 包和消息，不依赖位置项目的 `install`。配置默认是用于 Mock 的示例；输入须由合成数据或回放发布到上述私有 IMU 话题、位置反馈话题和 `/research_force_status/raw_status`。该入口不启动生产驱动或传感器驱动。未提供输入时会报告缺失或超时，并不会移动设备。输入话题应通过 ROS remapping 接到正确的研究数据源，且只允许一个期望长度生产者。
