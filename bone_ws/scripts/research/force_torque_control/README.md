# 六轴力矩 / 轴向力研究链（仅 Mock）

这套代码不控制真实设备。`offline_node.py` 默认关闭，只能构造 `MockTorqueBackend` 和 `MockForceBus`；`hardware_write_count = 0`。不要将此状态当成真机准入或控制性能结论。现有 J14、PID、DS-SCTDE、TDE 和位置控制链没有被替换。

## 两条控制路径

* `PID_TORQUE`：输入 `q_d` 和 `q` 均为 m，误差 `q_d - q`。沿用既有 `JointTorquePID`，输出 `TorqueNm`，发布 `TorqueCommand.torque_nm`（N·m）。每轴 `kp` 的单位为 N·m/m，`ki` 为 N·m/(m·s)，`kd` 为 N·m·s/m；`torque_limit` 为 N·m，`slew_limit` 为 N·m/s，`direction` 是每轴符号。控制器使用每轴反馈接收时间计算实际 dt，`min_dt`、`max_dt`、`max_age` 和 `derivative_tau` 的单位为 s。积分对幅值及变化率饱和进行条件抗饱和。首帧只初始化，不输出。现有 PID 原本已按 N·m 定义，并无证据表明此前将它送入位置命令；新增强类型命令与 Mock 链路防止这种误接。未发现可确认的电机轴力矩命令协议，真实力矩适配器**不可用**。
* `ACTUATOR_FORCE_CONTROL`：位置 P 外环将 m 误差映射为期望轴向力 N，增益 `kp_n_per_m` 单位 N/m；`max_force_n`、`slew_n_s` 分别为 N、N/s，`direction` 为每轴符号。外环限幅到非负值；记录真实来源应提供的 `F_meas`、`F_ref` 与 `e_F = F_ref - F_meas`，单位均为 N。内环为手册描述的电动缸驱动器内置力闭环，研究代码不把测量力假装为控制命令。本节点的协议执行适配仅接受 Mock；另有不打开端口、尚未接入真机运行入口的总线注入候选适配器：`0x25=3` 设置模式，`0x27` 写以 g 标示的力目标；状态帧通过原有 `Motor.read_status()` 解析有符号 `force_g`。这里使用 1 g-force = 0.00980665 N 的工程换算假设，六轴方向、量程、零点、接线、型号/固件及换算仍需设备确认；手册当前仅提供正数示例，不假设负力命令编码。上述协议行为只通过 FakeBus 测试，不代表实机已经接通或可用。
* `DS_SCTDE`：既有算法及仿真保留；不接入该会话选择器，选择会明确拒绝。独立算法结果不等同真实驱动已就绪。

三种模式在本会话中互斥、不可无条件切换；这是进程内研究选择器，**不是跨进程物理设备独占机制**。任何故障锁存后停止 Mock；Mock 的 `0x1A` 操作并不能证明真实停机成功。所有操作不得作为真机安全机制。

## ROS 接口

新建 `pus6_research_msgs`，命令使用不同消息类型 `TorqueCommand` 和 `AxialForceCommand`；`ForceFeedback` 单独提供 N、每轴主机单调时钟接收时间、有效标志与故障码。来源必须显式标注；默认零值且 `valid=false` 的消息不构成力测量。`TimedJointFeedback` 原来的 `forces_g_raw` 未由 J14 桥填充，本节点不读取它。`ForceProtocolMock.read_feedback()` 演示通过现有协议解析器读取 Mock 状态；新增原始状态帧桥可将明确来源的协议反馈转换为 `ForceFeedback`；真实传感器生产者和 J14 UART 桥接目前**没有接通**。不能拿 Mock 作为真实测量。

节点私有输入 `~/desired_lengths_m`（Float64MultiArray）和 `~/position_feedback`（TimedJointFeedback），力模式还需 `~/force_feedback`；输出 `~/torque_command`、`~/force_command` 和 JSON `~/diagnostics`。输入长度可由独立研究入口的双 IMU 姿态修正、IK 和轨迹整形生成；详见 `docs/research/6PUS_FORCE_TORQUE_ROS_RESEARCH.md`。位置及力接收时间为同机 CLOCK_MONOTONIC，不是设备采样时间；期望长度以节点接收时间检查新鲜度。节点等待输入，超时或无效时锁存 FAULT；测试中观测的目标、反馈、误差、力或力矩输出及时间记录在诊断消息中。每次诊断显示的饱和标志包含限幅或变化率限制。

## 隔离构建与离线验证

从 `bone_ws` 开始，只 source ROS Jazzy，不 source 原项目安装目录：

```bash
source /opt/ros/jazzy/setup.bash
colcon --log-base /tmp/6pus_ft_log build --base-paths src --build-base /tmp/6pus_ft_build --install-base /tmp/6pus_ft_install
source /tmp/6pus_ft_install/setup.bash
python3 -m pytest -q scripts/research/force_torque_control
```

节点需要明确设置 `enabled=true`、`mode` 和 `config_json`，JSON 包含 `pid`（字段对应 `joint_torque_pid/controller.py` 的 `Config`）和 `force`（字段对应 `core.py` 的 `ForceOuterLoop`）。**不得把测试参数用作真实设备参数**。测试在独立 ROS domain 中用合成消息；ROS 消息传递、Mock 协议和算法不构成真实驱动或实机验证。
