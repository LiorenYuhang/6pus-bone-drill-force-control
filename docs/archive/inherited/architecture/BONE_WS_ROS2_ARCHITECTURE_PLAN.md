> 历史归档：原路径 `agent_docs/architecture/BONE_WS_ROS2_ARCHITECTURE_PLAN.md`。正文保留当时的方案、参数及证据语境；当前原生 Mode 3 入口以[项目 README](../../../../README.md)为准。历史命令不作为当前推荐启动命令。

# Bone Drill / 6PUS ROS2 正式工程目录与迁移实施方案

> **状态覆盖说明（2026-08-20）**：本文是架构规划/迁移历史文档。文中早期“已装配验证 PASS”或类似表述不覆盖当前物理状态；当前完整机构尚未总装，两个 IMU 仍在同一块测试板上。当前阶段以 `agent_docs/decisions/CURRENT_STAGE.md` 为准：J4 控制算法/安全策略冻结且 UART transport 已应用 batch-atomicity hotfix，J5-C1/C2 软件与实时软件均 PASS/FROZEN，实机 FK/最终外参/融合尚未开始。

> 本文件作为后续 Agent 执行 ROS2 工程整理与初始化的权威任务说明。
>
> **当前阶段目标：完成 UART bring-up 阶段收口，并初始化正式 ROS2 工作空间。**
>
> 本轮只做目录规划、代码整理、ROS2 包初始化、代码迁移/封装设计与无硬件测试。
>
> **禁止执行任何新的电机实机运动。**

---

## 1. 当前工程根目录

项目根目录：

```text
/home/lyapunov/bone_drill
```

当前一级目录：

```text
bone_ws/
hardware/
STM32/
docs/
up/
```

其中：

```text
/home/lyapunov/bone_drill/bone_ws
```

作为后续 **6PUS 正式 ROS2 主工作空间**。

当前仅有：

```text
bone_ws/
└── src/
```

尚未建立正式 ROS2 packages，因此现在正适合统一初始化。

---

# 2. 总体目录职责

后续整个项目固定按以下职责划分：

```text
/home/lyapunov/bone_drill/
│
├── hardware/              # 厂商资料、协议资料、硬件说明书
├── STM32/                 # STM32 / MCU 固件工程
├── docs/                 # 教学、实验或其他已有内容，暂不调整
├── up/                    # Hardware bring-up / 单模块实机验证区
└── bone_ws/               # 正式 ROS2 机器人系统工作空间
```

必须保持这个层级边界。

---

# 3. `up/` 的定位：硬件 bring-up 与权威实机验证区

`up/` 不是正式 ROS2 主工程。

它用于：

- 单硬件模块 bring-up；
- 协议调试；
- 实机验证；
- 硬件诊断；
- 原始实验日志；
- 硬件问题追溯。

---

## 3.1 六电机 UART 工程保持原位并冻结

当前路径：

```text
/home/lyapunov/bone_drill/up/motor/uart_multi
```

这是已经完成的 **六电机 UART 权威实机验证工程**。

已经完成的递进验证链：

```text
单机验证
→ M2
→ M4
→ M6-A
→ M6-B5
→ M6-B6
→ M6-C
→ COMPLETE / PASS
```

最新 M6-C Retry 权威日志：

```text
/home/lyapunov/bone_drill/up/motor/uart_multi/logs/multi_motor/m6/20260817_184931_m6c_retry_joint_six_motors.log
```

当前全仓无硬件回归：

```text
306 passed
```

当前六机 M6-C Retry 最终位置：

```text
ID1 = 67
ID2 = 75
ID3 = 31
ID4 = 63
ID5 = 23
ID6 = 17
```

### 冻结原则

以下底层文件不得无协议证据修改：

```text
protocol.py
uart_bus.py
motor.py
```

当前已知冻结 SHA-256：

```text
protocol.py
d9064bb1d8bf05b4688bfa74243e2be93e69f2c54fd4c59c3a40c91294e61354

uart_bus.py
512a26db5b107e44e1347cb23612b12d82eeba5c220faa03a66071e1537e3a26

motor.py
8d2db8e3d1b5199f8e644248ea20d76df0b15b0c8903530c9b0639b9ea0f8780
```

Agent 应重新核对当前 SHA-256，并记录最终 freeze manifest。

### 重要原则

不要：

```text
把 uart_multi 整体搬进 bone_ws/src
```

不要让正式 ROS2 节点通过：

```python
sys.path.append(...)
```

跨目录直接 import：

```text
up/motor/uart_multi
```

更不要让正式 ROS2 系统直接依赖：

```text
safe_m6*.py
diagnose_id*.py
probe_*.py
```

这些脚本属于实机验证工具，不属于正式运行接口。

---

# 4. H30 IMU 工程的处理方式

当前路径：

```text
/home/lyapunov/bone_drill/up/sensor/IMU
```

它本质上已经是一个独立 ROS2 workspace：

```text
up/sensor/IMU/
├── build/
├── install/
├── log/
└── src/
    └── h30_imu/
```

真正需要进入正式 ROS2 主工作空间的是：

```text
up/sensor/IMU/src/h30_imu
```

而不是整个 `up/sensor/IMU`。

禁止迁移：

```text
build/
install/
log/
.venv/
```

推荐策略：

1. 保留 `up/sensor/IMU` 原位；
2. 将其中已验证的 ROS2 package `h30_imu` 复制/迁移到：

```text
/home/lyapunov/bone_drill/bone_ws/src/h30_imu
```

3. 在 `bone_ws` 中重新执行：

```bash
colcon build
```

4. 在正式工作空间验证通过后：
   - `up/sensor/IMU` 继续作为历史 bring-up 工程；
   - 正式机器人系统使用 `bone_ws/src/h30_imu`。

---

# 5. `bone_ws` 的正式目标结构

推荐最终形成：

```text
/home/lyapunov/bone_drill/bone_ws/
├── src/
│   ├── h30_imu/
│   ├── pus6_motor_driver/
│   ├── pus6_kinematics/
│   ├── pus6_control/
│   ├── pus6_bringup/
│   └── pus6_description/      # 后续需要 URDF / TF / RViz 时加入
│
├── build/
├── install/
└── log/
```

第一阶段暂时只要求：

```text
h30_imu
pus6_motor_driver
pus6_kinematics
pus6_control
pus6_bringup
```

`pus6_description` 可以后续再建立。

---

# 6. ROS2 package 职责划分

## 6.1 `pus6_motor_driver`

路径：

```text
bone_ws/src/pus6_motor_driver
```

职责：

- 六台因时电缸 UART 通信；
- ID1~ID6 固定映射；
- 六机状态读取；
- 目标位置下发；
- current / fault 安全检查；
- 位置范围检查；
- pause-all；
- UART 异常处理；
- 写操作审计；
- 六轴统一硬件接口。

正式 ROS2 驱动不得直接依赖 `safe_m6*.py`。

建议初始结构：

```text
pus6_motor_driver/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│
├── pus6_motor_driver/
│   ├── __init__.py
│   ├── protocol.py
│   ├── uart_bus.py
│   ├── motor.py
│   ├── six_motor_controller.py
│   ├── safety.py
│   └── motor_driver_node.py
│
├── config/
│   └── motors.yaml
│
├── launch/
│   └── motor_driver.launch.py
│
└── test/
```

其中：

```text
protocol.py
uart_bus.py
motor.py
```

可以从 `uart_multi` 的冻结版本提取一份初始生产版本。

但必须明确：

```text
uart_multi = 权威 bring-up / 验证源
pus6_motor_driver = 正式 ROS2 运行代码
```

---

## 6.2 `six_motor_controller.py`

这是下一阶段最重要的正式硬件抽象层。

建议提供：

```python
connect()

read_positions()

read_states()

configure_all()

command_positions(q_targets)

pause_all()

close()
```

其中：

```python
command_positions(q_targets)
```

要求：

```text
q_targets 必须是长度为 6 的目标位置向量
对应 ID1~ID6
```

上层不得直接操作 UART 寄存器。

推荐调用关系：

```text
ROS2 Node
   ↓
SixMotorController
   ↓
Motor × 6
   ↓
UartBus
   ↓
USB-UART
```

必须复用 M6 阶段已经验证出的安全规则，而不是重新发明一套。

---

## 6.3 `pus6_kinematics`

路径：

```text
bone_ws/src/pus6_kinematics
```

职责：

```text
[x, y, z, roll, pitch, yaw]
              ↓
         6PUS IK
              ↓
[q1, q2, q3, q4, q5, q6]
```

后续需要 FK 时也放在这里。

本包只负责运动学和必要的坐标/单位转换，不直接操作 UART。

---

## 6.4 `pus6_control`

路径：

```text
bone_ws/src/pus6_control
```

后续逐步承载：

- 关节空间控制；
- 轨迹控制；
- PID；
- ADRC；
- 强化学习控制接口；
- 状态反馈接口。

当前阶段先建立 package 骨架，不要一次性实现 PID / ADRC / SAC。

---

## 6.5 `pus6_bringup`

路径：

```text
bone_ws/src/pus6_bringup
```

职责：

- launch；
- yaml 参数；
- 系统统一启动；
- 节点组合；
- 串口路径；
- 电机 ID；
- 控制参数；
- IMU 参数。

最终目标类似：

```bash
ros2 launch pus6_bringup pus6_system.launch.py
```

统一启动：

```text
H30 IMU
+
六电机驱动
+
运动学
+
控制器
```

---

# 7. 整体 ROS2 软件层级

最终逻辑：

```text
──────────────────────────────
        Algorithm Layer
──────────────────────────────

        SAC / ADRC
            ↓
       pus6_control
            ↓

──────────────────────────────
        Kinematics Layer
──────────────────────────────

      pus6_kinematics
            ↓
     q1 q2 q3 q4 q5 q6

──────────────────────────────
         Driver Layer
──────────────────────────────

     pus6_motor_driver
            ↓
    SixMotorController
            ↓
      Motor × 6 / UART

──────────────────────────────
          Hardware
──────────────────────────────

      6 × 因时电缸
```

IMU 独立旁路：

```text
H30
 ↓
h30_imu
 ↓
sensor_msgs/Imu
 ↓
状态估计 / ADRC / SAC
```

---

# 8. 当前阶段具体任务

当前只执行 **J0 + J1**。

---

## J0：UART 阶段正式冻结

Agent 应完成：

1. 核对 `uart_multi` 当前文件；
2. 核对三份冻结底层 SHA-256；
3. 对以下关键文件额外生成 SHA-256：

```text
tools/safe_m6b_one_motor.py
tools/safe_m6c_joint_six_motors.py
AGENTS.md
UART实机运行记录.md
UART六电机编号与测试方案.md
```

4. 建立 freeze manifest；
5. 记录：
   - M6-C COMPLETE / PASS；
   - 权威日志路径；
   - 306 passed；
   - UART 参数 `921600 8N1`；
   - USB 串口：

```text
/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0
```

6. 所有历史 STOP / FAIL / ABORTED 日志继续永久保留；
7. 不修改底层协议代码；
8. 不执行新实机运动。

---

## J1：正式 ROS2 工程初始化

在：

```text
/home/lyapunov/bone_drill/bone_ws/src
```

进行正式 ROS2 软件架构建设。

当前建议：

```text
h30_imu
pus6_motor_driver
pus6_kinematics
pus6_control
pus6_bringup
```

本阶段要求：

### A. H30 IMU

检查：

```text
/home/lyapunov/bone_drill/up/sensor/IMU/src/h30_imu
```

迁移/复制到：

```text
/home/lyapunov/bone_drill/bone_ws/src/h30_imu
```

不要复制 build/install/log/.venv。

### B. `pus6_motor_driver`

建立正式 ROS2 Python package。

实现第一版：

```text
protocol.py
uart_bus.py
motor.py
six_motor_controller.py
safety.py
motor_driver_node.py
motors.yaml
launch
test
```

第一版不允许真实电机运动。

要求所有硬件调用均支持 mock / fake backend 测试。

### C. `pus6_kinematics`

先建立 package 骨架和 API，不要求当前立即完成所有真实 6PUS IK 数学实现。

### D. `pus6_control`

只建立 package 骨架、接口和 README。

暂不实现 PID / ADRC / SAC。

### E. `pus6_bringup`

建立基础 launch/config 组织。

当前不得通过 launch 自动发送任何电机运动目标。

---

# 9. 第一版 SixMotorController 的安全边界

必须考虑：

- ID 固定为 1~6；
- q_targets 长度必须等于 6；
- 所有目标先完成全量合法性检查，再开始写入；
- 任一目标非法时 0 写入；
- 当前协议位置合法区间；
- current / fault 检查；
- pause-all；
- UART 异常；
- 写审计；
- 禁止隐式 clamp；
- 禁止自动 +2 step 补偿；
- 保留静态 1-step / 已证明的有限量化反馈抖动认识；
- 不把 M6-C 叫作严格同步控制。

---

# 10. 当前阶段禁止事项

本轮 Agent 不得：

1. 执行任何电机实机运动；
2. 自动发送 0x29 新目标；
3. 执行新的 M7 / M8 等 UART 小测试；
4. 修改冻结 `protocol.py / uart_bus.py / motor.py`；
5. 删除或覆盖历史日志；
6. 把 `uart_multi` 整体搬入 `bone_ws`；
7. 把旧 `build/install/log/.venv` 搬入 `bone_ws`；
8. 开始实现 ADRC 或 SAC；
9. 随意调整六机安全门禁；
10. 将实验脚本作为正式 ROS2 runtime dependency。

---

# 11. J2 / J3 后续路线

本轮只设计，不执行。

## J2：正式 ROS2 Driver 实机三级推进方案

本轮只设计，不执行。

### J2-A：正式 ROS2 Driver 六机纯只读与遥测审计 (0 Writes) — [COMPLETE / PASS]
- 权威日志：`/home/lyapunov/bone_drill/bone_ws/log/j2a_readonly_20260817_211917.json`
- **运行版本 SHA 说明**：J2-A 记录的 SHA 清单为该阶段实机运行的**权威快照（Authoritative Run Snapshot）**，用于数据溯源与复现，**并非禁止后续驱动代码改进的永久冻结**。
- 运行结果审计（严格口径）：
  - 持续时间：60.00 s (1672 个完整六轴采样周期)
  - 硬件写操作：**全生命周期 0 writes (Startup: 0, Runtime: 0, Shutdown: 0)**
  - 实际持续采集速率 (Actual Sustained Acquisition Rate)：**$27.87\text{ Hz}$** ($60.00\text{ s} / 1672\text{ cycles}$)
  - ROS2 话题实际发布速率 (Actual Publish Rate)：**$27.85\text{ Hz}$** ($1671\text{ msgs} / 60.00\text{ s}$)
  - 单轮纯 UART 扫描服务时间 (Active Scan Service Time)：**$\bar{T}_{cycle} = 17.86\text{ ms}$** (P50=$17.76\text{ ms}$, P95=$19.16\text{ ms}$, P99=$19.76\text{ ms}$, Max=$21.81\text{ ms}$)
  - 理论背靠背总线扫描容量 (Theoretical Back-to-Back Capacity)：$1/\bar{T}_{cycle} \approx 55.99\text{ Hz}$
  - 6 轴顺序采样跨度：$\bar{t}_{ID6} - \bar{t}_{ID1} = 14.90\text{ ms}$
  - ROS2 话题接口：`/pus6/joint_states` (SI m 单位换算 100% 正确，effort/velocity 严格为空 `[]`)
  - 6 机静态位置统计（分布与众数）：
    - ID1: `[66, 67]`, mode=`66` (93.8%)
    - ID2: `[74, 75]`, mode=`74` (51.5%)
    - ID3: `[30, 31]`, mode=`30` (70.0%)
    - ID4: `[63, 63]`, mode=`63` (100.0%)
    - ID5: `[23, 24]`, mode=`23` (99.9%)
    - ID6: `[16, 17]`, mode=`17` (59.4%)
  - 故障与电流：全机 fault 恒为 `0x00`，电流恒为 `0 mA`。

### J2-F0：ROS2 + UART 六机 READ_ONLY 实时通信能力表征 — [PLANNING / NOT EXECUTED]
- 方案规范：`/home/lyapunov/bone_drill/agent_docs/stages/J2/J2-F0_ROS2_READONLY_COMMUNICATION_PERFORMANCE_PLAN.md` (Rev 2.1.1)
- 核心目标：解耦端到端 Python 驱动基准、协议帧长影响与 Driver-Side ROS 发布及端到端传输开销，实测确定 1~20Hz 防抖通信带宽基线。
- 4 个实验组 / 6 个逻辑子测试 / $N$ 个计时窗口（各 30s 采样窗口，0-write 红线，全量采用 `time.perf_counter_ns()` 单调纳秒计时与独立 `/pus6/benchmark_stamp` 插桩）：
  - **Group A (F0-A)**: Full-state / No ROS (全状态端到端背靠背基准，0x30 帧，168B 总线线时 1.823ms)
  - **Group B (F0-B)**: Position-only / No ROS (仅读位置端到端背靠背基准，0x31 帧，114B 总线线时 1.237ms，实测扫描耗时缩减量对比 0.586ms 理论线时缩减量，Pre/Post 0x30 安全快照)
  - **Group C**: Full-state / ROS Publish (标准 ROS 2 驱动管线，包含 F0-C1 50Hz 稳态验证与 F0-C2 基于 Timer 的自适应扫频测试，双进程拓扑，端到端延迟 $L_{\text{e2e}}$ 纳秒测量，统计 `Auditor Delivery Ratio` 与 `deadline_miss_count`)
  - **Group D**: Position-only / ROS Publish (精简 ROS 2 驱动管线，包含 F0-D1 50Hz 稳态验证与 F0-D2 最大可持续 ROS 速率判定测试，Pre/Post 0x30 安全快照)

### J2-B：正式 ROS2 Driver 单机极微位移验证 — [COMPLETE / PASS]
- 权威方案：`/home/lyapunov/bone_drill/agent_docs/stages/J2/J2-B_ROS2_SINGLE_MOTOR_REAL_HARDWARE_PLAN.md`；
- 权威日志：`/home/lyapunov/bone_drill/bone_ws/log/j2b2_tracking_motor4_20260818_134547.json`；
- ID4 `+10 step`、`50 step/s`、`3.0 s`，reference=`63`，target=`73`，final=`71`，final error=`-2 step`；
- first positive motion=`288.217 ms`，first error `<=2`=`1306.487 ms`，peak current=`33 mA`；
- 三帧写入严格为 ID4 `0x25 block -> 0x29 target -> 0x1A pause`，ID1/2/3/5/6 为 0 write；
- UART `223` transactions，retry/timeout/checksum/protocol/failure 全部为 0。

### J2-C：正式 ROS2 Driver 六轴协调/同步运动特性验证 — [COMPLETE / PASS]
- 方案：`/home/lyapunov/bone_drill/agent_docs/stages/J2/J2-C_ROS2_SIX_AXIS_VECTOR_REAL_HARDWARE_PLAN.md`；
- 权威日志：`/home/lyapunov/bone_drill/bone_ws/log/j2c_vector_20260818_161601.json`；
- 模式：`hardware_mode:=active`；
- 增量与速度：$\Delta q = [+10, +10, +10, +10, +10, +10]\text{ step}$，$\text{speed} = 50\text{ step/s}$；
- 运行结果：
  - 六轴 reference：`[66, 78, 37, 80, 28, 16]` step；
  - 六轴 target：`[76, 88, 47, 90, 38, 26]` step；
  - 六轴 final：`[74, 86, 45, 88, 37, 24]` step；
  - 位移与到位误差：全机位移 $+8\sim+9\text{ step}$，到位误差 ID1~ID4=-2, ID5=-1, ID6=-2（全机 $\le 2\text{ step}$，`all_axes_settled = True`）；
  - 六轴首次确认运动时延：ID3@638ms, ID4@705ms, ID5@773ms, ID6@773ms, ID1@907ms, ID2@1041ms（`start_skew = 402.57 ms`）；
  - 六轴首次到位时延（$\le 2\text{ step}$）：ID5@1108ms, ID4@1241ms, ID2@1376ms, ID3@1376ms, ID1@1444ms, ID6@1444ms（`arrival_skew = 335.32 ms`）；
  - 电流与故障：峰值电流 ID1~ID6 分别为 62, 53, 33, 32, 42, 41 mA（全机 $\le 500\text{ mA}$ 安全阈值），故障码恒为 0x00；
  - 目标下发时跨（Target dispatch spread）：$18.17\text{ ms}$；
  - 写入与通信审计：严格 18 次写入（6 配置 + 6 目标 + 6 暂停），总计 456 次 UART 交互，retry/timeout/checksum/protocol 均为 0；
  - 停止确认：`pause_all` 六机全部成功，两次采样确认全机物理静止且电流/故障安全。

J2-A/B/C 各子阶段实机验证全部圆满通过。

### J2-D：正式 ROS2 Driver 六轴 5 周期往返重复协调运动 — [PRE-HW SOFTWARE READY / NOT EXECUTED]
- runner：`/home/lyapunov/bone_drill/bone_ws/scripts/run_j2_c_vector.py --cycles 5`；
- 目标：验证正式六轴驱动连续执行 5 个往返周期（$+10\text{ step} \to \text{reference}$），统计正反向时延、重复性、累积漂移与总线时跨；
- 模式：`hardware_mode:=active`（实机运行需单独人工授权）；
- 参数：
  ```text
  cycles = 5
  Δq = [+10, +10, +10, +10, +10, +10] step
  speed = 50 step/s
  timeout_per_phase = 5.0 s
  ```
- 严格遵循 Safety STOP / Performance Result 分离原则；
- 统计指标：正向/反向每周期每轴 confirmed motion latency、start_skew、arrival_skew、实际位移、周期间标准差、累积漂移；
- 写入审计：严格 $6 + 5 \times 12 + 6 = 72$ 帧写入（6 配置 + 60 目标 + 6 暂停），零重试；
- Mock 测试 `22 passed`，全仓 `105 passed`，底层 `306 passed`，未执行真实 UART。

---

## J3：接入 6PUS IK

目标链路：

```text
期望动平台位姿
[x, y, z, α, β, γ]
        ↓
6PUS inverse kinematics
        ↓
[q1, q2, q3, q4, q5, q6]
        ↓
单位 / 零点 / 行程映射
        ↓
SixMotorController
        ↓
六电缸
```

完成 J3 后，才进入更高级的实时轨迹、PID、ADRC 和 SAC。

### J-HOME：六轴 Home Pose 规划（未来，仅规划）

后续 Home 方案只允许在独立安全评审和人工批准后实现，规划原则如下：

- 定义经过验证的安全六轴 Home Pose；
- 使用软件 `zero_offset` 保存零点偏置；
- 统一定义 `q_relative = q_raw - q_zero`；
- raw diagnostics 继续保留原始 step 值，不覆盖或归一化原始反馈；
- 不默认把六轴机械最短端作为 Home；
- 当前不实现 Homing，不发送任何 Homing 或运动指令。

---

# 12. Agent 完成本轮后必须报告

完成 J0 + J1 后，仅报告：

1. 最终目录树；
2. 哪些内容保持在 `up/`；
3. 哪些内容进入 `bone_ws/src`；
4. freeze manifest；
5. `pus6_motor_driver` API；
6. ROS2 topics / services / actions 初步设计；
7. H30 IMU 迁移结果；
8. `colcon build` 结果；
9. 单元测试结果；
10. 是否发生任何硬件写入；
11. J2 的后续实机方案，但不得执行。

---

# 13. 核心原则

最终必须保持：

```text
hardware / STM32
    = 资料和嵌入式层

up
    = bring-up / 实机验证层

bone_ws
    = 正式 ROS2 机器人系统
```

这是后续 6PUS 软件工程的固定架构边界。

---

# 14. 阶段里程碑记录

* **J0 / J1**: 目录划分、代码迁移、ROS2 package 骨架与单元测试 $\implies$ **COMPLETE / PASS**
* **J2-A ~ J2-D**: 单电机与六电机 UART 协调运动 Bring-Up $\implies$ **COMPLETE / PASS**
* **J3 PRE-ASSEMBLY FULL MOTION VALIDATION**: 16 项 Cartesian IK 驱动控制链实机批次验证 (M1~M16) $\implies$ **COMPLETE / PASS**
  *(验证范围：Cartesian Pose $\to$ 6PUS IK $\to$ 六轴目标 $\to$ S 曲线 $\to$ 真实电机执行链；未装配状态下验证)*
* **J3 ASSEMBLED 6PUS CARTESIAN-TO-IK JOINT-SPACE EXECUTION VALIDATION**: 真实装配 6PUS 机构全空间 6DoF 实机验证 $\implies$ **COMPLETE / PASS**
  *(覆盖：Homing Initialization、Pure-Z $+10\text{ mm}$、Pitch $\pm 0.05\text{ rad}$、Roll $\pm 0.05\text{ rad}$、Yaw $\pm 0.05\text{ rad}$、Surge $\pm 5\text{ mm}$、Sway $\pm 5\text{ mm}$、M13 6DoF 空间强耦合复合姿态，全流程 0 故障、0 异常过流、0 丢包)*
* **J4 ROS2 CONTINUOUS CARTESIAN CONTROL PIPELINE**: 连续笛卡尔到关节控制管线设计与 Mock 验证 $\implies$ **PLANNING / MOCK PHASE**


