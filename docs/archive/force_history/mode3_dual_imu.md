> 历史归档：原路径 `docs/mode3_dual_imu.md`。正文保留当时的方案、参数及证据语境；当前原生 Mode 3 入口以[项目 README](../../../README.md)为准。历史命令不作为当前推荐启动命令。

# 双 IMU → SO(3)/LADRC → 原生 Mode 3 位置闭环

本入口为 Force 工程独立入口，复用已经真机运行的 `z_position_motion.run`、IK、六轴位置 PI-D、保持力交接和退出恢复。Position/Torque 工程只读，本轮无真实电机写入、无真机运动。

## 算法来源与完整控制链

Position 的 `bone_ws/scripts/j14_demo_controller.py::controller_spec('feedforward_ladrc')` 使用 `controller_mode='3axis_active'`：Base SO(3) 逆向前馈与三轴 LADRC 角度残差组成姿态指令。Classical P/PD 同时计算供对照；不会将 Classical 和 LADRC 相加。虽然源 YAML/类名带 `shadow`，J14 入口实际选择的是三轴 LADRC 主动控制。

Force 中：

```
/pus6/imu_base/data + /pus6/imu_platform/data
  → 有效四元数历史、共同主机接收时间上的 SLERP 对齐
  → 各自捕获零姿态、Rz(+90°) 机械坐标变换
  → Base SO(3) 逆向前馈 + Platform 世界姿态误差
  → 原 Position 三轴 LADRC（Classical P/PD 仅记录对照）
  → SO(3) 合成目标姿态，XYZ 保持本次实测 FK 起点
  → 差分 IK，以实际编码器位置锚定
  → 最新完整可达 IK 目标直接进入原六轴位置 PI-D、主动制动和积分抗饱和
  → 本次实时保持力偏置 + PID 力增量
  → 原绝对力限幅与量化（正常运行无斜率整形）→ 原 UART owner → Mode 3 / 0x27
  → 新真实位置反馈 → 下一轮 PI-D；FK 只作位姿重建与记录
```

`position_ladrc.py` 是 Position `h30_imu/scripts/j13_ladrc.py` 的逐字节副本，不依赖 Position 工程路径。数学辅助函数及 Classical PID 复用 Force 已有的 `position_attitude_math.py`、`position_attitude_pid.py`。新 YAML 只保留 Position 姿态参数，不导入其电机配置、Position 驱动或位置发布节点。

| 通道 | b0 / s⁻¹ | ωc / rad·s⁻¹ | ωo / rad·s⁻¹ | 残差限幅 |
|---|---:|---:|---:|---:|
| Roll | 20 | 0.8 | 2.5 | ±0.5° |
| Pitch | 10 | 1.0 | 4.0 | ±0.5° |
| Yaw | 6 | 0.25 | 1.2 | ±0.5° |

Classical 对照参数为 Kp `[0.05,0.05,0.025]`、Ki=0、Kd `[0,0.0015,0.0015]`、导数 LPF 5 Hz，与 Position YAML 一致。姿态控制标称 50 Hz，dt 范围 1–100 ms，双 IMU 最新姿态龄期上限各 100 ms。

## 姿态方向、起点和同步

使用四元数 `[x,y,z,w]`，`q_rel=q0*⁻¹⊗q`，机械变换 `C=Rz(+90°)`。对于捕获的实际起始姿态 `R0`：

```
RB = C R(q_rel_base) Cᵀ
RP = R0 C R(q_rel_platform) Cᵀ
eW = Log(R0 RPᵀ)∨
Rff = RBᵀ R0
Rcmd = RBᵀ Exp(uW) R0
```

这沿用 Position 的误差方向和 SO(3) 合成，并采用 Force 已有适配方式支持非 HOME 起姿。Base 正向转动对应反向前馈，Platform 正向误差对应反向残差；不对 Euler 角简单相减。双 IMU 的安装方向必须保持 Position 已验证的机械安装约定。

输入使用现有 H30 已融合四元数；本入口不新增原始加速度/陀螺仪融合算法。只在 `orientation_covariance[0]>=0` 时更新姿态历史，gyro-only 数据不会刷新姿态龄期。两路选择最新共同的单调接收时间，SLERP 插值、不外推；同一个样本不会重复推进 LESO。ROS header 时间另行保存作诊断，不能当作硬件同步时钟。

位置 PI-D 继续用最新六轴实际反馈和各轴原时间戳；没有把 IMU 插值结果冒充电机反馈，也没有将最新位置环换成延迟 FK 环。

LESO 前一输入沿用 Position 定义：上一轮送入姿态/IK 链的角度残差。它不表示已经实现的物理姿态，不表示实际力或力矩；目标空间饱和与实际机构响应对该近似的影响需在 Force 实验中观察。

## 2026-11-07：固定启动保持与编码器积分死区

针对 `run_20261107_105159_566908`，双 IMU 入口的 `initial_hold` 改为固定 1 秒：六轴 Mode 3 / 保持力读回确认后启动阶段时钟，每轮继续验证双 IMU 数据，保持本次捕获位姿；到时直接进入 `stabilize`。位置误差、速度和 FK 的连续收敛条件不再阻止主动稳定启动。50 Hz 循环在首次到时周期完成后切换，时间分辨率受实际循环影响。终点 `final_hold` 沿用现有收敛和保持逻辑。

双 IMU PI-D 启用 `joint_integral_from_measured_error=True`：积分输入使用 `q_ref-q_encoder`，在原有 ±1 step（25 μm）死区内为零；死区外按真实误差积分。预测位置仍供 P 项使用，80 ms 连续速度回归及 40 ms D 项滤波保持不变。抗饱和回退也使用同一真实积分误差。进入主动稳定时复用同一 PID 实例，保留积分、速度历史和上一轮实际施加力状态，不清零或重新初始化；LADRC 在初始保持阶段不更新观测器，但持续对齐 IMU 时间，避免切换时累积 1 秒 dt。

未修改增益、LADRC 或保护边界：Kp/Ki/Kd 仍为 8000/6000/1000（SI 单位），积分 ±2.5 N，PID 增量 ±4 N，绝对目标力 ±1500 g，跟踪误差 2.5 mm；电流沿用当前共享基线 1500 mA。实际/目标行程仍为几何及设备边界交集（通常 200–1800 steps），IMU、反馈时效、通信、故障码、Mode 3 监控、暂停和 Mode 0 恢复保留。正常位置参考及力指令无速度/斜率整形，5000 g/s 仅用于原异常恢复路径。其他运动入口不启用本次积分策略。

验证：`colcon build --packages-select pus6_force_control --symlink-install` 成功；36 项针对性测试通过，覆盖编码器死区边缘、预测误差反号、积分限幅/抗饱和、无法满足旧收敛条件时仍在 1 秒后启动、非零积分状态交接、原保护故障恢复和既有动态优化。回放初始化改为恢复日志中真实的 PID 初始化反馈后，重跑 2 项受影响回放测试也通过。

本次历史数据离线回放（历史目录未改写）：

- 保留原阶段标签以隔离积分改动：[`replay_20261107_110857_728138`](../../../results/mode3_dual_imu/replay_20261107_110857_728138/replay_report.json)，955/955 帧完成。旧算法目标力与历史记录最大差值 0 g；新旧 P/D 最大差值均为 0。真实误差位于死区内、但预测引发积分输入的轴帧数从 `[3,24,0,10,164,83]` 变为全零。前 10 秒 M5 积分从旧算法 −0.354052 N 变为 0，六轴均无虚假积分累积。
- 使用新阶段时序：[`replay_20261107_110912_229752`](../../../results/mode3_dual_imu/replay_20261107_110912_229752/replay_report.json)，50 帧初始保持后，在起始时间后 1.039165 秒进入 `stabilize`，观测器开始更新，PID dt=21.235 ms，积分和速度状态连续。随后完成 492 帧主动稳定计算；11.384754 秒、M4 目标 1107.275 steps 对历史实际 1006 steps，误差 101.275 steps（2.53188 mm），原有 2.5 mm 跟踪保护停止回放，IK 目标可达。旧实验没有进入主动稳定，历史位置没有响应这些新目标；该停止不能解释为修改后真机发生了同样故障，也不能用回放证明 60 秒实机稳定。

两次回放均 `hardware_write_count=0`，不连接 ROS IMU 或串口。新增 `--replay-runtime-schedule` 仅可配合 `--replay-history` 使用，用于新阶段时序回放。

## NUC Dry-run

```bash
bash /home/lyapunov/6pus_bone_drill_force_control/bone_ws/scripts/run_mode3_dual_imu.sh --dry-run
```

默认合成三轴 IMU 输入 + 静态 `MockServoBus`，无需真实 IMU，不打开真实电机或 IMU 串口。流程为：原有起点保持约 1 秒 → 6 秒合成扰动输入 → 锁定最后姿态目标并按原条件保持约 4 秒 → Mock Mode 0 恢复。合成姿态幅值约 0.2°，仅为软件输入例子；Mock 不模拟实际机构动力学。

首次接入阶段的合成 IMU / Mock 运行结果目录（使用当时的参考限速设置，保留原记录）：

`results/mode3_dual_imu/dry_run_20261106_152331_262067/`

结果 `motion_complete`，546 个控制周期，`hardware_write_count=0`，Mock Mode 0 确认、串口 owner 已关闭。六轴均生成非零正反向目标力，范围约：

| 电机 | 合成输入下的目标力范围 / g |
|---|---:|
| 1 | −48～20 |
| 2 | −50～130 |
| 3 | −44～130 |
| 4 | −20～48 |
| 5 | −130～50 |
| 6 | −129～44 |

另用 1.5° 独立 Base 扰动进行三轴方向测试，确保包括 Yaw 在内的 IK 位移超过原一 step 死区后，原 PI-D/目标力增量方向正确。没有为让微小合成 Yaw 产生力而修改死区或增益。

## 下一步现场 Roll/Pitch 实验命令（本轮未执行）

```bash
bash /home/lyapunov/6pus_bone_drill_force_control/bone_ws/scripts/run_mode3_dual_imu.sh --hardware --hardware-qualified --start-imus --axes roll,pitch,yaw --duration-seconds 60
```

读取实际电机位置作为起点，保持 XYZ，Roll/Pitch/Yaw 三轴同时主动补偿。真实入口强制使用 ROS 双 IMU，禁止合成输入驱动真电机。`--start-imus` 会复用已有唯一发布者；缺失时才启动 Force 安装目录中的 H30 只读节点，端口、460800 波特率、话题和 frame_id 沿用 Position J14。不会启动 Position 控制节点或重复打开已占用的传感器串口。双 IMU 输入就绪之后才创建电机 owner。

现场先保持基座稳定完成参考捕获，再做三轴组合大幅扰动；幅值无需人工精确复现，实际输入以 Base IMU 记录为准。观察平台、电缆、连杆间隙与电流，不能把几何可达当成已验证的稳定能力。60 秒主动稳定结束后锁定最后送出的相对姿态目标，按原保持判据收敛后退出；不强制 HOME，不保证继续抵消结束后的新基座扰动。Ctrl+C 或真实故障走已有恢复流程。

主动稳定期间全部电机保持 Mode 3，以独立 0x27 指令输出目标力，没有独立 0x29 位置运动指令。**终止阶段仍保留既有 Mode 0 实测位置锚定恢复**（原有 0x25 块写与确认），不把主动控制改成 Position 模式。

## 限幅、记录与验证边界

双 IMU 六轴位置环现为 Kp=8000 N/m、Ki=6000 N/(m·s)、Kd=1000 N·s/m（此前 Kd=5000），积分 ±2.5 N、增量 ±4 N。速度采用 80 ms 时间窗回归，不再按 1.5-step 跨度强制归零；在各轴反馈时间上启用 τ=40 ms 的一阶滤波，供 D 项和位置预测使用。静止速度平滑衰减；重复反馈不推进滤波。积分在运动中也按条件积分/实际输出反馈防饱和更新，不再一检测到运动就完全冻结。

双 IMU 的 `joint_max_reference_speed_m_s=None`（JSON 为 `null`），`joint_reference_m` 仍直接等于当轮完整可达 IK 目标。没有位置时间斜坡，也没有给总目标力增加滤波或正常斜率。Z、单轴、组合轨迹入口保留原 Kd=5000、原速度估计/积分和参考限速。参数依据、历史对照及局限见 [PI-D 动态优化](mode3_dual_imu_pid_dynamics_20261106.md)。

实际保持偏置实时读取；最终力 ±1500 g。双 IMU 专用入口的正常 `ForceLimitsConfig.max_rate_g_s=None`，取消原 500 g/s 变化率限制，反向或阻尼制动方向也不再自动套用 5000 g/s 正常斜率。正常指令直接为 `quantize(clip(holding_bias + PID_delta, ±1500 g))`，不存在大数代替限速的做法。

双 IMU 电流保护对齐 Position/Torque 共享驱动的 1500 mA；保留机械/设备行程、2.5 mm 跟踪误差、通信/故障、Mode 3 和力读回检查。2.5 mm 保护直接检查有界 IK 目标减实测位置，不能靠缓慢推进参考减小该误差。取消软件参考/力斜率限制不保证某个实际速度，力、电流与机构自身响应仍决定实际运动。

PID 的 Kp/Ki/Kd、±4 N 增量、±2.5 N 积分保护、D 阻尼、速度估计及反馈龄期+20 ms 预测均保留。移除正常路径中原来依赖 5000 g/s 的 `abs(last_applied)/(5000*N_per_g)` 额外预测时延项：正常输出已没有该斜坡，不能继续按不存在的斜坡计算预测时间。配置报告 `force_policy.pid_slew_delay_compensation=false`，不修改 PID 增益。其余实验入口仍使用原有预测时延和斜率配置。

5000 g/s 现在只用于**显式异常退出/安全恢复**：IMU 输入故障或操作员中止时，若电机反馈新鲜、Mode 3 确认、位置/电流/故障/力反馈正常，则按5000 g/s将最后的PID增量回退到捕获的保持偏置，随后执行原有 Pause/Mode 0 锚定与观测。不会斜坡到零力。过流、机械/实际位置越界、跟踪超限、Mode丢失、通信故障等紧急情况跳过此斜坡，直接走既有停机恢复；恢复写入失败不重试。回偏置本身不作为机械停稳证明，最终仍以原Mode 0确认流程为准。

双 IMU 不使用起点 ±100 steps 包络，也不使用 SO(3) 角度缩放。按 Position/Torque 相同几何配置的正常行程与设备行程交集，目标与实际位置检查均使用通常的 200–1800 steps；删除此前额外叠加的 300–1700 实际内框及 350–1650 目标内框。共享配置的绝对 raw 范围为 0–2000；本入口仍不允许正常目标或实际反馈离开 200–1800/设备交集。设备边界更窄时直接服从设备边界，不再额外内缩或增加角度限制。

每帧记录 `target_reachable`、`ik_saturated`、完整目标、六轴目标/实际的剩余机械行程。可达目标原样进入 PI-D；不可达目标先保存 `IK_TARGET_WORKSPACE_BOUND` / `IK_TARGET_KINEMATIC_UNREACHABLE`，在写力指令前故障退出，绝不缩小目标后报告完成。实际反馈使用 `ACTUAL_POSITION_MECHANICAL_BOUND`，不再重复叠加一个更小的实际位置包络。运行结束的 `workspace_result` 区分完整目标执行与未评估的稳定性能。

三轴最大范围、历史工况、停止余量依据及离线结果见 [三轴大幅实验准备](mode3_dual_imu_three_axis_workspace_20261106.md)。电机及 IMU 关闭后自动生成 `analysis/analysis.json`、姿态/电流、六轴行程和力反馈曲线。最大 Base 角度、世界 SO(3) 误差、正负瞬态峰值、剩余行程及故障原因均由实测数据计算；不把人工随机扰动解释为标准阶跃百分比超调。

`attitude.target_constraint` 保留原始目标及可达性。可达时 `rotation_scale=1`；不可达时为 `null`、无替代目标并显式退出。`IK_TARGET_INVALID` 单独表示输入无效，不混用实际反馈越界原因。

每次创建独立 `results/mode3_dual_imu/{dry_run|run}_<时间戳>/`，不覆盖旧数据。`dual_imu_report.json` 保存起点、参考双 IMU、姿态/位置 PID 参数及退出确认；`tracking.jsonl` 保留原位置、PID、力、电流、模式、故障、FK 数据，并增加 `attitude`：双 IMU 原始四元数、共同时间/插值区间/龄期、坐标变换、误差、LADRC/Classic 对照残差、LESO 状态和限幅。另保留原 `feedback.jsonl` 与硬件运行串口记录。自行启动的 H30 日志为 `base_imu.log` / `platform_imu.log`。

报告中的 `force_policy.normal_max_rate_g_s=null`、`exceptional_recovery_rate_g_s=[5000,...]` 区分正常直通与恢复斜坡。若执行恢复斜坡，`recovery_force.jsonl` 单独记录每次目标力、实际位置、电流、力反馈和时间戳；`force_recovery` 说明是否执行/跳过/失败及恢复周期数。正常控制周期数不包含恢复周期，串口实际写入总数包含两者。

发生故障时报告增加精确单调时间 `fault_timestamp_ns`，并保存 `fault_snapshot.json`：失败阶段、待处理目标、原始/有界目标请求、最新实际反馈、姿态误差，以及最后完整控制帧（含 PID 输出）。若本轮 PID 在计算前失败，待处理帧的 PID 输出为 `null`，上一完整帧另行保留，不伪造新的输出。快照数据在恢复前捕获，文件在停机恢复和关闭串口后写入。

## 正常参考及力斜率取消后的软件验证与历史回放

此前取消正常力斜率阶段：Force 包定向构建成功，82项针对性测试通过（以下为当时记录）。覆盖正常大幅力跳变/反向不整形、±1500 g限幅、恢复阶段5000 g/s、过流立即停机而不走斜坡、恢复通信失败零重试、取消虚构斜坡预测时延、直接位置参考、连续姿态缩放/原边界、三轴方向、IMU对齐、故障现场和Mode 0恢复；Z/单轴/组合入口及既有力限制/主动制动测试均通过。PID 增益、保持力和物理硬件配置未改变。

只读回放 2026-11-06 的 419 条成功记录：

```bash
bash /home/lyapunov/6pus_bone_drill_force_control/bone_ws/scripts/run_mode3_dual_imu.sh --replay-history /home/lyapunov/6pus_bone_drill_force_control/results/mode3_dual_imu/run_20261106_153217_939698
```

取消正常力斜率阶段的历史回放输出：`results/mode3_dual_imu/replay_20261106_160926_689167/`。全部419帧完成，`hardware_write_count=0`。`joint_reference_m` 与当轮有界 IK 目标最大差值为0，参考峰值2.509 mm/s；最终目标力与“保持偏置+PID、绝对限幅及g量化”的计算结果最大差值为0，证明正常力路径没有额外斜率整形。按历史指令时间戳计算的最大目标力变化率约40072.65 g/s，超过500和5000 g/s；这只是离线指令变化率，不是实测力响应。反事实PID削顶50帧，最终目标力范围−734～1266 g，仍受既有力界限保护。旧的仅取消位置限速回放 `replay_20261106_155346_375105/` 保留不变。

历史日志只有越界前的419帧，没有被拒绝的 IMU/目标帧，所以回放中空间饱和为0。此前采用最后位姿扩大3%的合成延伸检查空间缩放；该旧行为现已被绝对工作空间和显式不可达退出替代，旧记录保留不变。

回放使用旧 IMU、旧实际位置和旧时间戳计算新指令，未模拟新的物理轨迹，不能证明取消限速后真实平台的运动、电流或稳定性能。此次未连接真实IMU或电机、未写入硬件，Position/Torque工程与旧实验记录未修改。

对实际双IMU调用链的检查表明：它使用 `writer.send_force_cycle` 直接发送，并不经过 `ServoCore.run_cycle` 的旧 `dispatch_force_slew` 或 Position 轨迹整形器，因此不存在第二层正常力斜率限制。LADRC/LESO带宽、速度估计噪声处理和积分动态属于保留的控制算法，不是新增速度或力斜率整形。UART帧间通信间隔及物理驱动自身限制未改动。
