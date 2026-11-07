> 历史归档：原路径 `docs/research/aux_translation_offline_v1/README.md`。正文保留当时的方案、参数及证据语境；当前原生 Mode 3 入口以[项目 README](../../../../README.md)为准。历史命令不作为当前推荐启动命令。

# 辅助平移激励：独立离线工具 v1

用途：在固定姿态目标下生成 `pus6_base` 基座系 XYZ 单轴平移，并检查纯数学 IK、轨迹整形、步数量化和配置范围。三个方向输出到三个 CSV，从不同时激励。此工具没有 ROS 节点、串口、驱动或运动发布入口，也不实现补偿器。

**`EVALUATED_OFFLINE` / `numerical_inputs_accepted=true` 只代表离线数值检查完成。物理行程、HOME 标定与 XYZ 安全边界始终标为 `UNVERIFIED`，不授权真机运动。** 示例的 0.00025 m 幅值仅用于数值演示，不是安全幅值。

## 运行

工具、配置、测试在独立目录：

`/home/lyapunov/bone_drill/bone_ws/scripts/research/aux_translation_offline_v1/`

```bash
python3 -B /home/lyapunov/bone_drill/bone_ws/scripts/research/aux_translation_offline_v1/offline_experiment.py \
  --config /home/lyapunov/bone_drill/bone_ws/scripts/research/aux_translation_offline_v1/example_config.json
```

默认在 `/home/lyapunov/bone_drill/results/aux_translation_offline_v1/` 下独占创建带 UTC 时间标签的新目录。也可用 `--output /absolute/new/directory` 指定尚不存在的目录；存在即拒绝，绝不追加或覆盖。

无需 source ROS 工作空间。依赖现有 Python 3 和 PyYAML；测试另需 pytest。拒绝与源码几何文件不同的 `PUS6_GEOMETRY_CONFIG` 环境覆盖。

```bash
python3 -B -m pytest -q -p no:cacheprovider \
  /home/lyapunov/bone_drill/bone_ws/scripts/research/aux_translation_offline_v1/test_offline_experiment.py
```

配置无隐式幅值默认值；必填字段见 `example_config.json`。复制配置到新的研究配置文件后再调整。`planning_xyz_offset_limits_m` 是使用者指定的离线拒绝范围，物理安全性仍为 `UNVERIFIED`。只接受 `frame_id=pus6_base`、`length_unit=m`；拒绝 world、动平台坐标和 mm，避免隐式坐标转换。四元数采用 xyzw，必须单位范数，不会自动归一化。

## 轨迹、坐标与初始状态

每次重复：静止基线 → 正弦（五次多项式包络平滑起停）→ 零平移基线。`cycles/frequency_hz` 是包含两个 ramp 的激励时长。时长和 ramp 必须与采样网格对齐；频率必须低于 Nyquist。所有方向分别重新开始，`time_s` 是单次重复的局部时间，联用 `axis/repeat_id/sample_index` 定位，不能当作实测时间戳。

- 位置：`[0,0,z_nom] + [dx,dy,dz]`，偏移沿基座轴，不随姿态旋转。
- 姿态：每一行原样复制固定四元数；仅为 IK 按现有适配器的 ZYX 约定转换 RPY。
- `ik_q_m`：几何滑块位置；`ik_delta_home_m = IK(目标)-IK(中性HOME)`。
- raw 目标：`HOME_steps + delta_home*40000`，随后复用原函数取整；不能直接把几何 q 乘 40000 当作 raw 目标。
- 每次重复假设已静止在固定姿态的基线位姿，速度/加速度为零。非零姿态的就位运动没有模拟；初始化不是物理 HOME 标定，也不代表机器人实际处于该状态。

## 输出与检查语义

每次生成 `x.csv`、`y.csv`、`z.csv`、`config.json`、`model.json`、`summary.json`。其中 `summary.json` 是该次运行的简短机读报告；`model.json` 保存几何 YAML 内容、参数来源、实际使用的整形参数、坐标及证据边界。本次示例的简短人工报告见 [preparation_20261025_01.md](/home/lyapunov/bone_drill/docs/research/aux_translation_offline_v1/preparation_20261025_01.md)。

CSV 以 ID1–ID6 的后缀 `_1`…`_6` 分列，记录时间、重复编号、XYZ、固定四元数，以及：

| 字段 | 含义 |
| --- | --- |
| `ik_q_m_* / ik_delta_home_m_*` | 几何 IK 目标 / 相对中性 HOME 位移，m |
| `ik_raw_steps_float_* / ik_raw_steps_*` | 量化前 / 取整后的 IK raw 目标 |
| `ik_delta_steps_* / shaped_delta_steps_*` | 同一次重复内，相邻采样的取整步数变化；首行记 0 |
| `shaped_q_m_* / shaped_delta_home_m_* / shaped_raw_steps_*` | 整形后几何位置 / HOME 相对位移 / 离线 raw 目标 |
| `target_fd_velocity_m_s_* / shaped_fd_velocity_m_s_*` | 对连续位置的相邻差分速度；首行约定 0 |
| `shaped_velocity_m_s_* / shaped_accel_m_s2_* / shaped_jerk_m_s3_*` | 原整形器的连续状态 |
| `target_speed_limit_exceeded` | 未整形 IK 位移差分速度是否超过整形器限速，不是电机超速 |
| `*_at_limit_end_* / shaped_at_limit_end` | 本次 update 末端状态是否在速度、加速度或 jerk 限值上 |
| `shaper_any_substep_clipped` | 固定为 `UNOBSERVED`；原函数不公开子步裁剪事件，不能用末端标志否认中途裁剪 |
| `target_*_violation / shaped_*_violation` | 对几何配置中的行程、绝对 raw、软件命令及普通运动范围分别检查 |
| `status / rejection_reason / exception / ik_feasible` | 离线计算状态与异常，不是硬件准入状态 |

范围检查采用几何配置的参考值：行程 0.025–0.075 m、绝对 raw 0–2000、软件命令 0–1800、普通运动 200–1800 step。这是本工具的规划拒绝策略，不声称当前 maintenance 生产路径启用了相同限制。纯 IK 的几何可达不保证逐轴行程，故本工具额外检查；不会对越界输入静默裁剪。

XYZ 规划边界、IK 失败或逐轴范围冲突会拒绝该行、终止该次重复的后续整形，并输出空缺值和 `SKIPPED_AFTER_REJECTION`，不会伪造保持命令。整形后若越界，也记录拒绝；数值证据不被改写。退出码 0=输入通过本工具数值检查，1=输出含拒绝/计算错误，2=配置或输出目录错误。

## 模型边界

整形器只读复用原 `JointTrajectoryController`；参数从现有 PD 配置读取（v1 的固定来源），不修改配置。步数是离线候选命令，不是 UART 最终发送值。未模拟 UART 最后变化率限制、实时调度、反馈延迟、电机内部伺服、机械误差或基座运动；`current_state` 只是上一数值输出。PD/LADRC 和基座前馈既未加载也未运行。没有姿态残差或补偿有效性结果。

仍需独立确认实际几何、轴向、HOME/行程标定、负载/碰撞/关节约束、组合姿态下工作空间及记录时间关联，之后才可能设计实机准入。不要仅因计算通过或幅值小就授权运动。
