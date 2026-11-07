# 构建与软件检查

使用现有 Ubuntu / ROS 2 Jazzy / Bash 环境，保留工程边界；Position 和 Torque 无需被本工程运行时导入。Python 依赖和 ROS 包依赖以 [package.xml](../../bone_ws/src/pus6_force_control/package.xml) 为准，包含 NumPy、SciPy、PyYAML、Matplotlib、rclpy、pyserial/H30 等现有组件。

在本工程工作空间构建所需依赖链：

```bash
cd /home/lyapunov/6pus_bone_drill_force_control/bone_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to pus6_force_control
source install/setup.bash
```

该包依赖 `pus6_msgs`、`pus6_research_msgs`、`pus6_kinematics`、`pus6_motor_driver`、`pus6_control`、H30 等组件。已有安装完整时，局部修改可选择受影响包构建；不要把另一个工程的 install 环境当成本工程安装。参数由安装目录加载，因此仅改源码 YAML 而未构建可能使运行参数与阅读的文件不同。

启动脚本本身的无硬件检查：

```bash
/home/lyapunov/6pus_bone_drill_force_control/bone_ws/scripts/run_force_dual_imu_native_pid.sh --software-check
```

它加载环境、检查入口及 CLI 帮助，不启动 ROS 节点、IMU 或 UART。需要查看选项时可对该脚本使用 `--help`。有限时三进程 Mock 运行是另一种验证，会启动 Mock 控制进程；它不等于上述仅检查脚本，更不等于真机动态验证。此处不增加每次开机必须完成的测试门槛。

当前核心源码入口：

- [双 IMU CLI 与 Home 交接](../../bone_ws/src/pus6_force_control/pus6_force_control/mode3_dual_imu.py)
- [Parent](../../bone_ws/src/pus6_force_control/pus6_force_control/mode3_process_runtime.py)、[UART Child](../../bone_ws/src/pus6_force_control/pus6_force_control/mode3_process_child.py)、[Bridge](../../bone_ws/src/pus6_force_control/pus6_force_control/mode3_process_bridge.py)
- [六轴位置 PI-D](../../bone_ws/src/pus6_force_control/pus6_force_control/joint_position_force_pid.py)、[三进程输出策略](../../bone_ws/src/pus6_force_control/pus6_force_control/mode3_process_policy.py)
- [双 IMU 姿态/SO(3)/LADRC 适配](../../bone_ws/src/pus6_force_control/pus6_force_control/dual_imu_attitude.py)、[原力控驱动](../../bone_ws/src/pus6_force_control/pus6_force_control/six_axis_force_drive.py)

正式保留标签 `mode3-native-capacity-20261107` 的 commit 为 `46081f59b1a01705e9fb9780fd647e64a2352177`。当前未提交的长期运行/Home 和其他工作区修改不得被标签重置覆盖。文档整理不执行 checkout、reset、stash、commit 或 push。

已有构建和针对性测试结果引用[原生能力策略](../control/native_capacity.md)、[三进程迁移记录](../archive/force_history/three_process_migration_20261107.md)。本轮只做 Markdown 内容与链接检查，没有重跑 39 项或完整测试套件。
