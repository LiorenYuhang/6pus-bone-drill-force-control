# 实验进展与证据索引

真机记录、Mock、固定历史输入回放和几何离线计算分开解读。`results/` 目录和原 Markdown 结果文件保持不变；下面仅提供阅读入口。历史阶段文档顶部的“未执行/下一步”描述指当时阶段，后续状态以对应实际结果为准。

## 已有位姿运动

| 实验 | 原始记录 | 文档及结论边界 |
|---|---|---|
| Z 上升 10 mm 初次保持失败 | [13:06 数据](../../results/z_position_motion/20261106_130621_472451) | [保持浮点边界分析](motion/mode3_z_position_hold_analysis_20261106.md)；当次未返回 |
| Z 往返成功基线 | [13:31 数据](../../results/z_position_motion/20261106_133122_655194) | [早期实现记录](motion/mode3_z_position_motion.md)；100 s 单程往返与 Mode 0 记录 |
| 五自由度正反运动 | [单轴数据集](../../results/mode3_single_axis/20261106_134611_962821) | [六次运行与 Yaw 续做说明](motion/mode3_five_axis_results_20261106.md)；不能称五轴全部一次完成 |
| 六自由度同时轨迹 | [14:36 数据](../../results/mode3_combined_pose/run_20261106_143627_158172) | [原始自动结果](../../results/mode3_combined_pose/run_20261106_143627_158172/summary.md)、[开发记录](motion/mode3_combined_pose_motion.md)；完成轨迹并 Mode 0，仍有滞后/误差 |
| 单电机原生力 | [首次记录](native_motor/single_motor_force_control_experiment.md)、[V2](native_motor/single_motor_force_control_experiment_v2.md) | 真机单位、Tare、正反力与退出依据；不替代整机受力标定 |

六自由度组合轨迹原自动结果的运动 RMSE / 最大绝对误差如下；FK 来自顺序编码器反馈，不是外部绝对精度测量：

| 自由度 | RMSE | 最大绝对误差 |
|---|---:|---:|
| X / mm | 0.219305 | 0.800599 |
| Y / mm | 0.183584 | 0.689269 |
| Z / mm | 0.023014 | 0.079253 |
| Roll / ° | 0.077489 | 0.374961 |
| Pitch / ° | 0.090150 | 0.376866 |
| Yaw / ° | 0.318811 | 1.098997 |

该实验采用当时轨迹入口参数，不是当前三进程原生能力策略的动态性能验收。

## 双 IMU 故障与演进

| 记录 | 定位/语境 |
|---|---|
| [11-06 15:32](../../results/mode3_dual_imu/run_20261106_153217_939698) | 早期低速参考阶段；原目录保留，不把其参数归到当前入口 |
| [11-06 16:15](../../results/mode3_dual_imu/run_20261106_161531_093605) | [当时故障分析](faults/mode3_dual_imu_fault_20261106_161531.md) |
| [11-06 17:58](../../results/mode3_dual_imu/run_20261106_175818_903202) | 后续[速度/D 与饱和改进依据](../archive/force_history/mode3_dual_imu_pid_dynamics_20261106.md) |
| [11-07 10:51](../../results/mode3_dual_imu/run_20261107_105159_566908) | 停在初始保持，后改固定 1 秒并修正积分死区；见[接入历史](../archive/force_history/mode3_dual_imu.md) |
| [11-07 11:20](../../results/mode3_dual_imu/run_20261107_112026_767316) | [故障分析和写入核对](faults/mode3_dual_imu_fault_20261107_112026.md) |
| [11-07 13:15](../../results/mode3_dual_imu/run_20261107_131549_798122) | Mode 3 交接与参考时效竞态；18 次写入，无正常 PI-D 周期；见[修复记录](../archive/force_history/three_process_migration_20261107.md) |
| [11-07 13:28](../../results/mode3_dual_imu/run_20261107_132820_919946) | M1 约 −100.407 steps 跟踪误差触发旧边界；位置反馈回撤不足与旧输出削顶有直接证据，不等于已确定机械故障 |

## 软件、回放与后续真实运行

| 类型 | 记录 | 可支持内容 |
|---|---|---|
| 三进程 Mock | [30 s Mock](../../results/mode3_dual_imu/dry_run_20261107_123706_268110) | 1755 周期、Mode 0；非实机时延 |
| 启动修复 Mock | [短时完整阶段](../../results/mode3_dual_imu/dry_run_20261107_132346_864393) | 新鲜参考交接与阶段运行，硬件写入 0 |
| 原生能力回放 | [固定历史输入回放](../../results/mode3_dual_imu/native_capacity_replay_20261107_142929_683560) | 600 帧；原 M1 请求 −12.4287 N 在只去增量限幅时由 −630 g 变为 −1489 g；连续重算另见[详细策略](../control/native_capacity.md) |
| 原生能力软件测试 | [记录](../control/native_capacity.md) | 39 项已记录通过；本轮未重跑，不属于真机成绩 |
| 后续真机 | [14:40](../../results/mode3_dual_imu/run_20261107_144031_951372)、[15:24](../../results/mode3_dual_imu/run_20261107_152400_172980)、[16:15](../../results/mode3_dual_imu/run_20261107_161516_844033) | 均 `operator_stopped`，Mode 0 确认与串口关闭；分别记录 1840/1056/704 周期，不是指定长时轨迹完成 |
| 自动 Home Mock/真机 | [证据与边界](../operations/home.md) | 当前仅据已有记录报告，未新增现场测试 |

16:15 自动分析记录：主动阶段约 13.045 s，Base 三轴峰值约 `[10.884,13.289,3.117]°`，世界姿态误差 RMS `[1.813,1.541,1.215]°`、最大 `[6.204,4.667,3.761]°`；电流各轴峰值 `[183,274,221,185,220,232] mA`；无 IK 不可达记录、无最终绝对力饱和记录，自动稳定性结论为 `NOT_ESTABLISHED`。原报告见[analysis.json](../../results/mode3_dual_imu/run_20261107_161516_844033/analysis/analysis.json)。

这些记录说明新策略已经有真实运行，不应继续写“尚未有任何真机链路”；但主动稳定精度、持续大幅组合扰动、长期通信和所有故障退出情形仍不能据短时正常停止一并宣称通过。故障和未完成实验保留原始结论，不删去失败记录。
