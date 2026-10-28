# 6PUS 关节位置误差到电机力矩：离线研究状态

状态：**仅离线原型，真实力矩执行链尚未实现，不可用于真机使能或运动**。2026-10-28。未访问电机串口，hardware_write_count = 0。

## 实际基础与论文

NUC 仓库 `/home/lyapunov/bone_drill` 的 `main` 为 `117384d`，原工作树有未提交修改；本工作没有更改这些文件。实际环境显示 ROS_DISTRO=jazzy、AMENT_PREFIX_PATH=/opt/ros/jazzy，但这不证明当前活动机器人进程所使用的 overlay。未取得 Windows 工作副本，无法逐文件比较。

现有链路为姿态控制和前馈 → 位姿 → 逆运动学 → 轨迹整形 → 位置步数 → J14 Parent/UART Child → Servo mode 1、0x29 位置目标。反馈桥的 `TimedJointFeedback` 包含逐轴接收完成时间、位置、步数、电流与故障字段；时间是主机接收完成时间，不是电机采样时间。桥当前的 `positions_meters = 0.050 + (steps - 1000)*0.000025`，M1–M6 对应 ID1–ID6。不得把其他坐标原点的目标直接和它相减。不能用当前生产位置目标话题传力矩。

论文式 (11) 为 e=q_d-q。式 (13) 把电机动力学、传动矩阵及由 J 的转置伪逆获得的主动关节广义力结合。式 (28) 的 τ_m=τ_SC+Nτ_a，τ_SC 含同步误差、参考加速度及摩擦，任务空间项含 TDE；这不等同普通 PID。式 (36) 为 APD 对比控制器，包含加速度和摩擦前馈，也不是 DS-SCTDE。论文机器人为四驱动、三任务自由度冗余机构，其同步项和伪逆不能未经重建地套用六轴 6PUS。

对线性电缸，令 x 为长度（m）、θ 为电机轴角（rad），局部传动 `n=dx/dθ`（m/rad）。理想无损虚功关系 `τ_motor=n F_axial`；真实传动中的效率、摩擦、方向、动态和负载依赖需要实测或厂家模型。研究 PID 直接定义 `τ_i=s_i(Kp_i e_i+Ki_i ∫e_i dt+Kd_i de_i/dt)`，其中 e 单位 m、τ 单位 N·m，s 为轴向伸长误差对应的电机力矩符号。Kp、Ki、Kd 分别为 N·m/m、N·m/(m·s)、N·m·s/m；它们不是轴向力 PID 参数，不能复用姿态 PD 参数。这里没有把该输出转换为电缸力命令。

## 后端缺口

本地 LAF V2.0.4 手册 3.4/3.5.6 将 0x25 mode 3 定义为力控、0x27 定义为以 g 表示的力目标，并说明通过动态调整位置保持受力；0x26 是电压，0x2B 电流是只读反馈。已检查的 `motor.py`、`protocol.py`、`uart_bus.py` 无电机轴力矩命令接口。以上是资料范围内的发现，不否认电机能力，也不证明厂家没有其他接口。必须取得**当前具体型号及固件适用的正式电机轴力矩控制接口定义**：模式切换及独占机制、帧格式/符号/单位/缩放/范围、更新周期、反馈和通信失败时的安全停止语义；还需轴向/电机方向、标定、实际可用限幅和控制整定依据。不能以轴向力 0x27 或电压 0x26 冒充轴力矩，更不能向 0x29 写 τ。

## 已实现和边界

`bone_ws/scripts/research/joint_torque_pid/controller.py` 是六轴纯软件 PID：逐轴参数、幅值/变化率限制、考虑两种饱和的条件积分、带一阶滤波的误差差分、逐轴时间戳检查、非有限值和反馈故障锁存；首次反馈只初始化且输出零。`state` 保留 P/I/D 与输出。速度差分是估计量，参考值阶跃也会进入微分。参数需明确提供，禁止把测试值作为真实整定值。故障时软件输出清零并不等于设备被停止。

`offline_ros_node.py` 是独立 ROS2 研究节点，订阅私有 `~/desired_positions_m`（Float64MultiArray，长度 6）和 `~/synthetic_feedback`（TimedJointFeedback），只发布私有 `~/diagnostics` JSON；没有生产命令发布者、串口访问、模式切换或电机停止。默认 `enabled=false`；若启用，还需显式设置每轴 gains、非零 `torque_limit_nm`、`slew_limit_nm_s`、`direction` 和超时。该节点输入必须是合成测试数据，diagnostics 的 N·m 为模拟值。故障锁存需重启研究进程才能复位；没有实现设备层急停。因为根本没有力矩写入后端，位置与力矩路径在此原型中没有竞争；这不是跨进程实际控制权互斥实现。

节点使用的时间戳必须同源于本机 `CLOCK_MONOTONIC`，六轴每次更新均须递增且在设定的间隔内，均非零、不过期；这可能不适用于异步低速逐轴反馈，实际部署需重新设计逐轴采样与调度并验证。目标通过消息接收时间检查陈旧；生产目标的有效性、轨迹来源和模式交接尚未实现。

## 本机无硬件验证与回滚

在仓库中运行：

```bash
cd /home/lyapunov/bone_drill/bone_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 -m pytest -q scripts/research/joint_torque_pid/test_controller.py scripts/research/joint_torque_pid/test_offline_ros_node.py
python3 -m pytest -q src/pus6_motor_driver/test/test_j7_c5_six_axis_servo_transport.py src/pus6_motor_driver/test/test_j7_c1_servo_mock.py
```

隔离构建，不覆盖生产安装：

```bash
source /opt/ros/jazzy/setup.bash
colcon --log-base /tmp/6pus-torque-log build --base-paths src --packages-up-to pus6_control pus6_motor_driver --build-base /tmp/6pus-torque-build --install-base /tmp/6pus-torque-install
```

上述构建验证现有四个包，不安装此独立研究脚本。2026-10-28 PID 与 ROS 消息回调合成测试通过，位置 Servo FakeBus 回归通过，隔离 colcon 构建通过；这不验证 DDS 端到端、编码、通信失败、物理停止或真机性能。脚本是未跟踪新增文件，可逐个审阅；回滚只需移除本研究目录和本文，不需要更改现有位置链或已有未提交工作。

## 真机验收尚未具备

取得正式接口定义后，先实现独立的单一硬件所有者、模式切换互斥、命令编码/反馈及可靠停止，并建立假总线通信失败/超时测试；核对坐标、轴序、符号、力矩范围、传动与各轴可追溯整定。确认无活动位置驱动持有串口后，再按厂家过程准备实机；真实使能和运动均需用户另行明确批准。当前没有可执行的真机力矩启动命令，也不能验证实际互斥或设备停止。不要在这些条件缺失时将本研究节点用于驱动。
