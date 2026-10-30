# 6PUS 三轴姿态轨迹离线扫描

`bone_ws/scripts/research/pose_trajectory_offline_v1/scan.py` 仅使用现有几何 IK 和六轴 jerk-limited 参考整形器，生成 Roll、Pitch、Yaw、双轴组合和三轴组合的周期目标。它不创建 ROS 节点、不打开串口、不发送命令，也不模拟电机或测量值。

默认每种工况为正弦参考，幅值 0.25°、周期 5 s、2 个周期、步长 0.02 s；参考在软件几何 HOME 处开始并回到该参考。每轴 CSV 包含姿态参考、IK 长度、整形后长度及模型相对 HOME 步数；JSON 中的 IK 与整形结果差异是**参考整形差异**，不是实际跟踪误差。没有真实姿态、真实驱动反馈、动态响应或实验重复性数据。

在力矩研究仓库中可复现；输出目录必须不存在：

```bash
cd /home/lyapunov/6pus_bone_drill_torque_control/bone_ws
source /opt/ros/jazzy/setup.bash
colcon --log-base /tmp/6pus_research_log build --base-paths src --build-base /tmp/6pus_research_build --install-base /tmp/6pus_research_install
source /tmp/6pus_research_install/setup.bash
python3 scripts/research/pose_trajectory_offline_v1/scan.py --output /tmp/6pus_pose_scan_new
python3 -m pytest -q scripts/research/pose_trajectory_offline_v1
```

本次输出在 `results/pose_trajectory_offline_v1/scan-20261030/`，包括七个 CSV、`summary.json` 和 `offline_ik_references.png`。结果标记 `EVALUATED_OFFLINE`、`real_motion=NOT_EXECUTED`、`hardware_write_count=0`。数值仅由几何模型和参考整形产生，不能用于批准真实运动或推断停止距离。

本轮环境明确没有人工现场急停按钮且操作人员不在现场。现有 UART 软件暂停依赖通信、驱动及进程本身，不能在这些路径失效时提供独立的停止手段；因此本轮没有使能或发送真实运动命令。现有位置生产路径及 PID、DS-SCTDE 研究代码均未被本扫描调用或修改。
