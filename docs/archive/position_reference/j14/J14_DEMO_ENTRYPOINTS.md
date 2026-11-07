> 历史归档：原路径 `docs/j14/J14_DEMO_ENTRYPOINTS.md`。正文保留当时的方案、参数及证据语境；当前原生 Mode 3 入口以[项目 README](../../../../README.md)为准。历史命令不作为当前推荐启动命令。

# J14 三种主动姿态防抖 Demo 正式操作指南

**运行架构来源**：J14-A 冻结基线 (`v1.0-j14-a-three-process-runtime-reliability-qualified`)；此 tag 标识 runtime 基线，Demo 包装脚本的后续演进应按对应版本单独核验。
**验证状态**：**HUMAN HARDWARE CONFIRMATION = PASS**（真实 6PUS 并联平台实机确认通过）
**启动顺序**：Driver ready → Feedback ready → HOME confirmation → IMU preflight → ACTIVE
**正常退出**：操作员 Ctrl+C → 返回 OPERATING_HOME → 6-axis Pause ACK → 串口释放与进程清理
**故障退出**：Fault detection → Pause → cleanup；跳过 HOME return，不在故障后继续运动。

---

## 1. 快速启动命令

在确保机构处于 `OPERATING_HOME` 附近、设备硬件与双 IMU 就绪且无其他控制进程占用的终端中，运行对应模式脚本（脚本会自动加载 ROS 2 Jazzy 与工作空间环境变量，支持纯净新终端直接执行）：

主动姿态稳定 Demo 默认以 `OPERATING_HOME = [1000]*6`（各轴约 1000 steps）为运行参考，进入 ACTIVE 前必须完成 Startup HOME 确认。这是运行参考确认与返回 HOME，不是机械回零。`STORAGE_PARK = [0]*6` 用于存储/断电，不作为 ACTIVE 姿态稳定的起始参考。

```bash
# 1. Base SO(3) Feedforward (Platform IMU 仅观察，无 residual 反馈)
cd /home/lyapunov/bone_drill
bash bone_ws/scripts/run_j14_demo_feedforward.sh

# 2. Feedforward + Classical P/PD (Roll=P, Pitch/Yaw=PD, Ki=0)
cd /home/lyapunov/bone_drill
bash bone_ws/scripts/run_j14_demo_feedforward_pd.sh

# 3. Feedforward + First-order LADRC (三轴 LESO 扰动观测与残差补偿)
cd /home/lyapunov/bone_drill
bash bone_ws/scripts/run_j14_demo_feedforward_ladrc.sh
```

---

## 2. 模式参数与冻结基准对齐表

| 模式 | 运行节点实现 | 配置文件来源 | 核心参数基准 |
| :--- | :--- | :--- | :--- |
| **Base SO(3) Feedforward** | `J11BaseFeedforwardRunner` | `bone_ws/src/h30_imu/config/j11_feedforward_params.yaml` | 纯几何逆解前馈补偿，残差反馈为 0 |
| **Feedforward + Classical P/PD** | `J11M2PidResidualRunner` | `bone_ws/src/h30_imu/config/j11_m2_pid_params.yaml` | $K_p=[0.05, 0.05, 0.025]$, $K_i=0$, $K_d=[0, 0.0015, 0.0015]\,\text{s}$, 滤波 $f_c=5\,\text{Hz}$, 限幅 $\pm 0.5^\circ$ |
| **Feedforward + First-order LADRC** | `J13LadrcShadowRunner` | `bone_ws/src/h30_imu/config/j13_ladrc_params.yaml` | $b_0=[20,10,6]$, $\omega_c=[0.8,1.0,0.25]$, $\omega_o=[2.5,4.0,1.2]$, 限幅 $\pm 0.5^\circ$ |

> **架构说明**：Demo 统一启动生产级 `pus6_system.launch.py`，底层运行 Parent（`SingleThreadedExecutor`）+ 50 Hz UART Child（独占串口）+ Feedback Bridge（独占反馈 IPC）三进程解耦架构。兼容话题由独立 `j14_a2_q3_feedback_relay.py` 转换，零侵入生产驱动。

---

## 3. 标准操作生命周期与安全规程

1. **启动阶段 (Startup)**：
   - 驱动自检与 Feedback Bridge 就绪；
   - 检查电机初始反馈：验证 6 轴反馈有效性（无超时、过流、硬件故障且处于允许物理区间）；若已处于运行参考测量带（全 6 轴处于 [995, 1005] steps 容差窗口且连续 5 帧确认），直接放行；若存在偏差，则以反馈锚定的平滑轨迹返回 `OPERATING_HOME`（目标 `[1000]*6`），并在全 6 轴连续 5 帧稳定于 [995, 1005] steps 后确认完成；
   - 双 IMU 预检：检测 Base 与 Platform IMU 数据流延时（门限 $< 100\text{ ms}$）；
   - 控制器状态迁入 `ACTIVE`，开始实时动态姿态补偿。
2. **运行阶段 (Active Motion)**：
   - 系统持续运行，操作员可平稳晃动动基座观察平台的主动姿态对冲效果；
   - 终端实时显示当前姿态跟踪误差、控制输出与观测器状态。
3. **停机阶段 (Controlled Shutdown)**：
   - **无故障的 ACTIVE 运行中，操作员按下 `Ctrl+C` 发起正常退出**；
   - 系统拦截中断信号，首先通过轨迹整形器平稳引导动平台回归 `OPERATING_HOME` (`[1000]*6`)，防止急停造成机械失步；
   - 随后底层驱动下发六轴硬件 Pause（必须获得 6/6 ACK 确认）；
   - 串口资源被操作系统安全释放（`serial_release: free = true`）；
   - 所有伴随进程依次通过 SIGINT 平稳退出，残留进程数确认为 **0**。
   - 若检测到驱动/电机故障、反馈超时或其他故障退出条件，跳过返回 HOME，执行暂停与资源清理；若返回 HOME 过程中发生故障或再次中断，则终止 HOME 运动并执行暂停与清理。

---

## 4. 历史硬件链路事件说明 (F3 Base IMU USB Incident)

在实机联调中，曾记录过一次 Base IMU 外置 USB 集线器 Port 3 硬件禁用事件（内核日志：`disabled by hub (EMI?), re-enabling...`，设备短时重枚举为 `ttyACM2`）。
- **安全响应验证**：系统 100 ms IMU Watchdog 严格按设计触发保护停机（`Base IMU age timeout (>100ms)`），6 轴硬件 Pause 瞬间生效，未发生失步或机械碰撞。
- **现状与建议**：后续三个 Demo 实机复测均正常通过（HUMAN HARDWARE CONFIRMATION = PASS）。该事件属于外置 USB 拓扑与线缆振动的物理层可靠性事件（非软件/控制算法缺陷），后续建议将 Base IMU 直连工控机原生 USB Root 端口以获得更高级别的物理抗扰度。
