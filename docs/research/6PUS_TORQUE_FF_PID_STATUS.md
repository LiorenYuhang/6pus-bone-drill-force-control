# 电机力矩前馈 + PID：离线加法原型及部署缺口

本目录的现有研究入口不具备真实电机轴力矩输出。本次新增 `scripts/research/joint_torque_ff_pid`，是独立离线加法原型，没有接入 ROS 启动入口或串口，也不是 Simulink 模型的复现。

每轴误差为 `e=q_d-q`（m）。调用方必须直接提供带时间戳的电机轴前馈力矩 `tau_ff`（N·m）；算法在电机轴坐标中计算 `tau_raw=tau_ff+direction*(kp*e+ki*integral(e)+kd*filtered_de_dt)`，再对 **合成结果** 执行力矩幅值及变化率约束，积分抗饱和也考虑前馈。`kp`、`ki`、`kd` 的单位分别是 N·m/m、N·m/(m·s)、N·m·s/m；前馈本身已经在电机轴坐标，不再乘 `direction`。首样本仅初始化，故障锁存只改变软件状态，不能停止实际电机。

该原型**不生成前馈**。搜索本机工程中的 `.slx`、`.mdl`、MATLAB 文件和已有参考包，未找到用户所指 Simulink 模型；需要模型文件或框图、各信号单位、采样方式与参数才能建立对应关系。论文中的 DS-SCTDE 式 (28) 是任务空间 TDE 与关节同步控制，式 (36) 是含动力学项的 APD 对照，不能凭此认定它们就是缺失模型中的关节 PID 前馈。基座 IMU 姿态前馈也不是电机轴力矩前馈。

现有 `PID_TORQUE` 研究链仍输出到 `MockTorqueBackend`，`research_entry.py` 拒绝非 Mock 后端。现有位置伺服采用 `0x29` 绝对位置步数；查得资料中未找到当前型号/固件可在位置模式接收 N·m 轴力矩的协议、缩放、限值和反馈定义。因此无法提供真实力矩实验启动命令：需要厂家针对实际型号/固件提供对应接口和停止/反馈语义，且先完成离线和硬件接口验证。不得将位置步数或轴向力指令冒充轴力矩。相关未跟踪的 `real_drive_interface.py` 中 `JointTorqueDriverInterface.write_torque` 只追加内存记录，不会发送力矩，也不应视为真实后端。

仅验证软件：

```bash
cd /home/lyapunov/6pus_bone_drill_torque_control/bone_ws
python3 -m pytest -q scripts/research/joint_torque_ff_pid scripts/research/joint_torque_pid/test_controller.py
```

本次不打开串口、不使能、不发送执行器命令；`hardware_write_count = 0`。已有运动位置模式和原有 PID、DS-SCTDE 均未修改。
