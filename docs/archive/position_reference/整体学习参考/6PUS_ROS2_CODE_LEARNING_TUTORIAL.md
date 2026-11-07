> 历史归档：原路径 `docs/整体学习参考/6PUS_ROS2_CODE_LEARNING_TUTORIAL.md`。正文保留当时的方案、参数及证据语境；当前原生 Mode 3 入口以[项目 README](../../../../README.md)为准。历史命令不作为当前推荐启动命令。

# 6PUS 并联手术机器人 ROS 2 项目全栈源码教学与系统解析教程

> **适用对象**：希望通过真实 6PUS 并联骨科手术机器人项目系统掌握 ROS 2 架构设计、多轴电机高频伺服通信驱动、并联机构解析逆解与数值正解、在线轨迹平滑与安全控制全栈技术的工程师与研究生。
> **当前冻结基线**：$\mathbf{J8\ SIX\text{-}AXIS\ 50Hz\ CARTESIAN\ SERVO\ QUALIFIED\ BASELINE\ v1.0}$ (`v1.0-j8-active-servo-hardware-qualified`)，并已完成 J10 六自由度单轴安全运行包络实测。
> **测试回归语境说明**：ROS 2 工作空间核心功能包全量单元测试为 **346 项 PASS**（执行 `colcon test`，覆盖 `pus6_msgs`, `pus6_kinematics`, `pus6_motor_driver`, `pus6_control`, `pus6_bringup`, `h30_imu`）；运行前严格校验 `hardware_write_count = 0`。
> **教程结构分期**：
> - **第一部分：物理实机运动与感知基础 (Part I — Real-Hardware Motion Foundation, J1–J10)**：第 1~12 章，已在真实 6PUS 物理样机上完成整机总装、闭式逆解、50Hz 流式伺服、LM 数值正解与全自由度安全运行包络测定。
> - **第二部分：主动姿态稳定与防抖研究路线 (Part II — Active Stabilization Research Roadmap, J11+)**：第 13 章，面向动基座扰动环境下的前沿姿态稳定研究路线，明确 J11 PID 防抖现状与未来演进方向（严禁预设未经验证结论）。

---

## 目录

### 第一部分：物理实机运动与感知基础 (Part I — Real-Hardware Motion Foundation, J1–J10)
1. [项目架构全景与双链路设计 (System Architecture & Dual-Pipeline Philosophy)](#1-项目架构全景与双链路设计-system-architecture--dual-pipeline-philosophy)
2. [底层通信与 UART 协议层实现 (UART Protocol & Serial Reality)](#2-底层通信与-uart-协议层实现-uart-protocol--serial-reality)
3. [六电机驱动层与 J7 50Hz Servo 伺服架构 (Motor Driver & J7 50Hz Servo Architecture)](#3-六电机驱动节点与高频并发解耦架构-motor-driver--concurrency-architecture)
4. [6PUS 并联机构解析逆运动学 (Analytical Inverse Kinematics & Geometry Config)](#4-6pus-并联机构解析逆运动学-analytical-inverse-kinematics--geometry-config)
5. [在线轨迹整形与连续控制算法 (Stateful Shaper & Continuous Control)](#5-在线轨迹整形与连续控制算法-stateful-shaper--continuous-control)
6. [安全门禁、故障恢复与 HOME/PARK 状态机 (Safety Gates, Recovery & State Machine)](#6-安全门禁故障恢复与-homepark-状态机-safety-gates-recovery--state-machine)
7. [J5 进阶专题：多源统一时间轴与固定延迟状态重建 (Unified Timeline & Fixed-Lag)](#7-j5-进阶专题多源统一时间轴与固定延迟状态重建-unified-timeline--fixed-lag)
8. [J5 进阶专题：解析雅可比数值正运动学与实时解算节点 (Analytical Jacobian Numerical FK & Pus6FkNode)](#8-j5-进阶专题解析雅可比数值正运动学与实时解算节点-analytical-jacobian-numerical-fk--pus6fknode)
9. [【全链路实战追踪】双数据流端到端生命周期透视 (End-to-End Tracing)](#9-全链路实战追踪双数据流端到端生命周期透视-end-to-end-tracing)
10. [核心工程问题演进与实机踩坑复盘 (Engineering Lessons & Post-Mortem)](#10-核心工程问题演进与实机踩坑复盘-engineering-lessons--post-mortem)
11. [测试驱动开发 (TDD) 与回归测试体系 (Testing Architecture)](#11-测试驱动开发-tdd-与回归测试体系-testing-architecture)
12. [二次开发速查指南、实战练习与物理基线 (Developer Guide, Exercises & Physical Baselines)](#12-二次开发速查指南实战练习与物理基线-developer-guide-exercises--physical-baselines)

### 第二部分：主动姿态稳定与防抖研究路线 (Part II — Active Stabilization Research Roadmap, J11+)
13. [J11+ 主动姿态防抖前沿研究路线图 (Active Attitude Stabilization & Anti-Vibration Roadmap)](#13-j11-主动姿态防抖前沿研究路线图-active-attitude-stabilization--anti-vibration-roadmap)

---

## 1. 项目架构全景与双链路设计 (System Architecture & Dual-Pipeline Philosophy)

### 1.1 为什么需要分层与双链路架构
并联骨科手术机器人对控制系统的**硬实时性**、**确定性**、**安全性**与**状态可观测性**有着极其严苛的要求：
1. **控制下行链路 (Control Pipeline)**：必须保证从 50Hz 笛卡尔空间参考轨迹到 25Hz 物理电机串口指令的流式下发具备极低的抖动、严格的 $C^2$ 加速度连续性以及绝对安全包络硬门禁；
2. **状态观测链路 (Observation Pipeline)**：必须将 6 轴异步串行扫描到达的电机遥测重构对齐到统一纳秒时间轴上，并以 200Hz 独立解算平台当前末端位姿，提供给上层状态监控与未来多传感器融合。

如果将逆解计算、轨迹平滑、串口通信、状态同步和正解解算混在一个进程或同一个控制循环中，串口通信的长尾波动（P95 达 48ms）会直接摧毁规划器的定时周期，而正解迭代的不确定耗时也会污染控制下发的确定性。因此，本项目采用**控制链路与观测链路严格解耦、并行运行的双链路架构**。

```
                                  6PUS 系统双数据流全景架构

  [ 上位规划流 / 笛卡尔目标 ]
             │
             ▼  (/pus6/target_cartesian_pose)
  ┌────────────────────────┐
  │  pus6_kinematics_node  │ ── 50Hz 纯几何闭式解析逆解 (Analytical IK)
  └────────────────────────┘
             │
             ▼  (/pus6/desired_joint_deltas)
  ┌────────────────────────┐
  │   pus6_control_node    │ ── 50Hz StatefulJerkLimitedShaper (三阶加加速度连续整形)
  └────────────────────────┘
             │
             ▼  (/pus6/command_joint_steps 唯一权威控制入口, QoS: BestEffort, KeepLast(1))
  ┌────────────────────────┐
  │ J7 50Hz ServoTransport │ ── 50Hz 连续 0x29 目标写入 + StatusFeedback 伴随回包 + Slew 自愈 + 6级安全硬门禁
  └────────────────────────┘
             │
             ▼  (RS485 / UART 半双工单总线 921600 8N1, 0x29 写入及伴随 StatusFeedback)
  ========================= [ 6PUS 物理机构 (ID1 ~ ID6 一体化电缸) ] =========================
             │
             ▼  (辅助 0x30 轮询: 显式 safety / diagnostic poll；不替代 0x29 主反馈)
  ┌────────────────────────┐
  │ J7 FeedbackScheduler   │ ── 组装 TimedJointFeedback 消息 (记录 CLOCK_MONOTONIC)
  └────────────────────────┘
             │
             ▼  (/pus6/joint_feedback/timed)
  ┌────────────────────────┐
  │      pus6_fk_node      │ ── FixedLagJointSynchronizer (零外推状态重建)
  │ (独立 SingleThreaded)  │    + Analytical Jacobian LM 数值正解 (Warm-Start + SO(3) 门禁)
  └────────────────────────┘
             │
             ├───────────────────────────────┐
             ▼                               ▼
    /pus6/fk/state (FkStateStamped)     /pus6/fk/pose (geometry_msgs/PoseStamped)
```

> [!IMPORTANT]
> **控制链与 FK 观测链的并行解耦关系**：
> * **控制链** 是单向向下驱动物理电机的指令流：`Cartesian Target → IK → Jerk Filter → Driver Preflight → UART Write`；
> * **FK 观测链** 是单向向上重构平台姿态的遥测流：`UART Piggyback → Timed Feedback → Fixed-Lag → Numerical FK → FkStateStamped`；
> * **FK 观测链绝不作为 J4 控制器的反馈输入，也不进入下行控制环路**。两者完全独立解耦，杜绝了正解计算波动对控制周期的干扰。

---

### 1.2 ROS 2 工作空间 (`bone_ws`) 与 6 大功能包全景
工作空间位于 `/home/lyapunov/bone_drill/bone_ws`，由 6 个专职功能包协同构成：

```
bone_ws/
├── src/
│   ├── pus6_msgs/          # 自定义强类型 ROS 2 消息接口包 (时间合同与正解状态)
│   ├── pus6_motor_driver/  # 电机二进制通信协议、底层驱动节点与硬件安全包
│   ├── pus6_kinematics/    # 6PUS 权威几何配置、解析逆运动学与实时数值正运动学包
│   ├── pus6_control/       # 连续轨迹三阶整形滤波、外环控制与状态机包
│   ├── pus6_bringup/       # 统一 Launch 编排、参数配置与系统集成包
│   └── h30_imu/            # 双 H30 IMU 驱动、软件时间同步与相对姿态李群估计包
├── build/                  # 编译构建中间文件
├── install/                # 节点可执行文件与环境引导脚本 (setup.bash)
├── log/                    # 运行时生成的实验遥测 JSON 与调试日志
└── scripts/                # 实机/仿真运行脚本与离线验收工具链
```

#### 功能包职责与依赖矩阵：

| 功能包名称 | 核心职责 | 输入 (Sub / Ingress) | 输出 (Pub / Egress) | 硬件依赖 |
| :--- | :--- | :--- | :--- | :---: |
| **`pus6_msgs`** | 统一数据合同：定义带纳秒单调时钟的高精度遥测与正解消息 | 无 (纯接口定义) | `TimedJointFeedback`, `TimedJointCommand`, `FkStateStamped` | 无 |
| **`pus6_motor_driver`** | 独占物理串口通信，维护 Latest-Value-Wins 缓冲，执行安全门禁与故障自愈 | `/pus6/command_joint_steps` | `/pus6/joint_states`, `/pus6/six_axis_telemetry`, `/pus6/joint_feedback/timed` | 依赖 UART 硬件 |
| **`pus6_kinematics`** | 提供唯一几何真值源；运行解析逆解节点；运行独立单线程数值正解节点 | `/pus6/target_cartesian_pose`, `/pus6/joint_feedback/timed` | `/pus6/desired_joint_deltas`, `/pus6/fk/state`, `/pus6/fk/pose` | 无 (纯数学与 ROS) |
| **`pus6_control`** | 50Hz 三阶状态滤波整形，保证速度/加速度/加加速度饱和限幅与 $C^2$ 连续 | `/pus6/desired_joint_deltas`, `/pus6/joint_states` | `/pus6/command_joint_steps`, `/pus6/control_diagnostics` | 无 (纯算法) |
| **`pus6_bringup`** | 加载系统参数 YAML，协同编排启动各节点与 RViz 可视化 | Launch 启动参数 | 启动系统节点图 (Node Graph) | 编排层 |
| **`h30_imu`** | 双 H30 姿态传感器接口，完成时钟频偏消除与共模抑制相对姿态解算 | 串口 IMU 二进制流 | `/imu/base/data`, `/imu/platform/data`, 相对姿态四元数 | 依赖 IMU 硬件 |

---

### 1.3 权威参数唯一来源：`6pus_geometry.yaml` 与 `geometry_config.py`
在工业级机器人系统中，**严禁在各个功能包或源码文件中散落硬编码几何参数与换算系数**。本项目确立了全局唯一参数真值源 (Single Source of Truth)：

* **权威配置文件**：[`pus6_kinematics/config/6pus_geometry.yaml`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_kinematics/config/6pus_geometry.yaml)
* **类型化加载器**：[`pus6_kinematics/geometry_config.py`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_kinematics/pus6_kinematics/geometry_config.py) 中的 `load_geometry_config()`

```
                               6pus_geometry.yaml (权威唯一来源)
                                              │
                                              ▼
                             geometry_config.load_geometry_config()
                                              │
         ┌───────────────────┬────────────────┴───────────────────┬───────────────────┐
         ▼                   ▼                                    ▼                   ▼
 pus6_kinematics        pus6_control                      pus6_motor_driver      J3/J4 Standalone
(analytical_6pus.py) (joint_trajectory_controller.py)       (safety.py)        & Offline Tools
 • 静/动平台半径       • 步数转换比例 (40000 step/m)       • 绝对硬件限位 [0,2000] • 默认串口与波特率
 • 连杆长与安装角     • 工作零位 OPERATING_HOME           • 软件限位 [0,1800]     • 停放位与零位定义
 • 标称高度 z_nom     • 停放位 STORAGE_PARK               • 正常运动包络 [200,1800]
```

---

### 1.4 ROS 2 核心概念在本项目中的映射与应用

| ROS 2 核心概念 | 通俗定义 | 本项目中的具体实现与源码映射 |
| :--- | :--- | :--- |
| **Package (功能包)** | 软件复用与编译的基本单元 | `pus6_msgs`, `pus6_motor_driver`, `pus6_kinematics`, `pus6_control`, `pus6_bringup`, `h30_imu` |
| **Node (节点)** | 独立执行的计算任务进程 | `Pus6MotorDriverNode`, `Pus6KinematicsNode`, `Pus6ControlNode`, `Pus6FkNode` |
| **Topic (话题)** | 单向发布-订阅数据管道 | `/pus6/command_joint_steps` (下行控制), `/pus6/joint_feedback/timed` (上行观测), `/pus6/fk/state` (正解状态) |
| **Message (消息)** | 强类型数据载体 | `TimedJointFeedback`, `TimedJointCommand`, `FkStateStamped`, `JointState`, `PoseStamped` |
| **Service (服务)** | 双向同步/异步请求响应 | `/pus6/startup_home` (就位), `/pus6/controlled_park` (受控收拢), `/pus6/pause_all` (急停抱闸) |
| **Timer (定时器)** | 周期性回调触发器 | 50Hz 控制主循环 (`pus6_control_node`), 25Hz UART 下发线程, 200Hz 固定延迟正解查询 (`fk_node`) |
| **Callback Group** | 执行器内的回调隔离机制 | 驱动节点内划分 `sub_cb_group`, `worker_cb_group`, `poll_cb_group` 三大互斥组 |
| **QoS (服务质量)** | 通信时效与队列控制策略 | 下行控制指令采用 **Latest-Value-Wins (KeepLast(1), BestEffort, Volatile)** 彻底杜绝历史堆积 |

---

### 1.5 常见误解与避坑指南
1. **误解：以为正运动学 (FK) 是 J4 控制器的反馈输入**
   * *事实*：在 6PUS 机器人的 J4 控制链中，控制器基于解析逆解输出目标关节整步，进行前向三阶滤波；FK 属于独立的观测链路，用于姿态评估与离线诊断，**不进入 J4 控制器的闭环输入**。
2. **误解：在代码中直接修改 40000、1000 或半径等常数**
   * *事实*：所有几何模型、步数比例与限位全部由 `6pus_geometry.yaml` 统一管理，直接修改 `.py` 中的默认参数会破坏系统一致性。

---

### 1.6 推荐源码阅读顺序
1. 阅读 [`pus6_msgs/msg/`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_msgs/msg) 了解系统的核心数据契约；
2. 阅读 [`pus6_kinematics/config/6pus_geometry.yaml`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_kinematics/config/6pus_geometry.yaml) 与 [`pus6_kinematics/pus6_kinematics/geometry_config.py`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_kinematics/pus6_kinematics/geometry_config.py) 掌握全局参数结构；
3. 阅读 [`pus6_bringup/launch/pus6_system.launch.py`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_bringup/launch/pus6_system.launch.py) 理解整个系统节点图的启动编排。

---

## 2. 底层通信与 UART 协议层实现 (UART Protocol & Serial Reality)

### 2.1 为什么需要专用二进制协议与半双工单总线拓扑
本项目 6 台一体化推杆电缸（ID1 ~ ID6）挂载在同一条 RS485/UART 半双工总线上，波特率为 **921600 bps (8N1)**。
* **物理拓扑优势**：极大减少末端线束重量与体积，满足骨科手术狭小空间需求；
* **物理挑战**：半双工总线在同一时刻只能有一个设备发送。主机必须以**逐轴串行问答**的方式与 6 台电缸通信。

---

### 2.2 输入输出与二进制通信帧格式
源码实现位于 [`pus6_motor_driver/protocol.py`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/protocol.py)。

```
+----------------+----------------+----------------+----------------+----------------+----------------+----------------+
|  Byte 0 ~ 1    |     Byte 2     |     Byte 3     |     Byte 4     |  Byte 5 ~ 6    |  Byte 7 ~ N-2  |    Byte N-1    |
|   帧头 (Header) | 数据长度 Length | 目标 ID (ID)   | 命令字 (CMD)   | 寄存器地址 Addr | 负载 (Payload) | 校验和 (CRC)   |
+----------------+----------------+----------------+----------------+----------------+----------------+----------------+
| 0x55 0xAA (下发)|  3 + len(data) |   0x01 ~ 0x06  |  0x30/0x31/0x32|  小端 LE16      |  小端 16位数据  |  累加和低8位   |
| 0xAA 0x55 (回传)|      0x0F      |   0x01 ~ 0x06  |  0x30 / 0x32   |  0x0000/0x0029 |  12字节状态数据 |  累加和低8位   |
+----------------+----------------+----------------+----------------+----------------+----------------+----------------+
```

1. **帧头 (Frame Header)**：
   * 主机发送命令：`0x55, 0xAA` (`CMD_FRAME_HEADER`)
   * 电机回传应答：`0xAA, 0x55` (`RSP_FRAME_HEADER`)
2. **长度字段 (Length)**：值为 `3 + len(payload)`（包含 CMD 1字节 + Addr 2字节 + 数据体长度）。
3. **累加和校验算法 (`calculate_checksum`)**：
   $$\text{Checksum} = \left( \sum_{k=2}^{N-2} \text{Byte}[k] \right) \mathbin{\text{\&}} \text{0xFF}$$

---

### 2.3 关键控制与遥测寄存器表

| 寄存器宏定义 | 地址 (16位小端) | 读/写 | 物理含义 | 数据格式与单位 |
| :--- | :---: | :---: | :--- | :--- |
| `ID_ADDR` | `0x0016` | R/W | 电机硬件总线 ID | `uint16`, 范围 1 ~ 6 |
| `SAVE_PARAMS` | `0x001C` | W | 保存参数至 EEPROM | 写入 1 触发保存 |
| **`SPEED_LIMIT`** | `0x0028` | R/W | **内部梯形限速寄存器** | `uint16`, 对应 step/s (默认 240) |
| **`TARGET_POS`** | `0x0029` | R/W | **目标位置写入寄存器** | `int16`, 对应整步 raw step (0~2000) |
| **`ACTUAL_POS`** | `0x002A` | R | **实际位置反馈寄存器** | `int16`, 当前编码器整步值 |
| `CURRENT_VAL` | `0x002B` | R | 实际工作电流 | `uint16`, 单位毫安 (mA) |
| `ACTUAL_FORCE`| `0x002C` | R | 轴向力传感器读数 | `int16`, 单位克 (g) |
| `TEMPERATURE` | `0x002E` | R | 电机驱动板温度 | `int8`, 单位摄氏度 (°C) |
| `ERROR_CODE`  | `0x002F` | R | 硬件故障代码 | `uint8`, 0 表示正常，非0为过流/堵转/过温 |
| **`PAUSE_BRAKE`**| `0x001A` | W | **抱闸/急停锁定** | 写入 1 立即抱闸制动 |

---

### 2.4 核心机制：Piggyback 伴随反馈与串行非原子现实

#### (1) Piggyback 伴随反馈机制
上位机向电机下发 `0x32` 写入目标位置 `0x0029` 时，电机会**立即返回一个 20 字节的标准状态帧 (StatusFeedback)**。驱动层直接解析该伴随返回帧生成 `SixAxisStateSnapshot`，单轴问答耗时仅约 $3.5\sim 4.5\text{ ms}$，6 轴整批通信耗时约 $24\sim 30\text{ ms}$，无需额外发起 6 次独立的读寄存器请求。

#### (2) 半双工串行非原子现实 (Serial Non-Atomic Reality)
由于单总线串行依次下发（ID1 $\to$ ID2 $\to \dots \to$ ID6），单次 6 轴写入**不是同时生效的硬件锁存事务**。当 Motor 6 收到指令并应答时，Motor 1 已经先运行了约 $18.18\text{ ms}$。同理，6 轴反馈数据也存在约 $18.18\text{ ms}$ 的时间相位差。这一物理现实必须由上层时间同步与正解算法显式处理。

#### (3) UART 批次原子性保障 (Batch Atomicity Hotfix)
在多线程或带轮询的架构中，必须严防后台遥测轮询 (Poll) 帧插入到 6 轴写入控制批次中间：
* **加固方案**：在 [`pus6_motor_driver/six_motor_controller.py`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/six_motor_controller.py) 中引入 6 轴批次全局锁（`with self._bus_lock_ctx:`），并在 Active 控制模式下抑制独立 Poll 轮询；
* **回归实证**：实机连续动态回归证明批次重叠数为 **0 / 392 (100.00% 批次原子性无交错)**。

---

### 2.5 常见误解与避坑指南
1. **误解：以为 Batch Atomicity 把串口物理通信变成了 6 轴硬件同时锁存**
   * *澄清*：Batch Atomicity 保证的是**主机软件侧一整个 6 轴写/读批次在总线上不被其他请求打断**，它无法改变 RS485 逐轴串行发送的物理耗时与相位差。
2. **误解：以为 0x29 写入后需要再调用一次 0x2A 读实际位置**
   * *澄清*：0x29 写指令的 Piggyback 应答帧中已经包含了最新的 `actual_pos`、`current`、`force` 和 `error_code`，重复发送读指令会使通信周期翻倍恶化。

---

### 2.6 推荐源码阅读顺序
1. [`pus6_motor_driver/pus6_motor_driver/protocol.py`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/protocol.py)：掌握 `pack_write_register_cmd` 与 `parse_frame`；
2. [`pus6_motor_driver/pus6_motor_driver/six_motor_controller.py`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/six_motor_controller.py)：掌握 `command_positions_with_metrics` 与 `_bus_lock_ctx` 批次互斥。

---


---

### 2.7 J6 定位模式 (mode=0) $\to$ J7 伺服模式 (mode=1) 的演进与机理

#### (1) J6 定位模式 (mode=0) 的卡顿机理分析
在 J6 基线中，电机工作在定位模式（`mode=0`，10Hz 轮询）。每次下发 0x29 时等待 20 字节伴随状态回包 (StatusFeedback, 包含 12 字节状态数据)，整批耗时约 18.18~18.39ms。
* **微停顿与冲击机理**：在 10Hz 离散目标下，电机内部梯形加减速曲线在每个离散目标点之间频繁经历加速、减速与微停顿，导致实机运动产生明显的阶跃卡顿，峰值电流高达 $145\,\text{mA}$。

#### (2) J7 伺服模式 (mode=1) 50Hz 目标写入与伴随反馈
在 J7 伺服模式下：
* **目标刷新率提高 5 倍**：从 10Hz 提高到 **50Hz**（周期 20ms）；
* **0x29 运动写入与 Design-B 主反馈**：上位机以六轴批次依次发送 0x29 目标指令；执行器返回的 `StatusFeedback` 由驱动层通过 `ingest_cycle_feedback()` 接收，作为运行期主反馈并与对应 Servo cycle 关联；
* **RX 残留与 Stale Response 处理**：0x29 伴随回包必须按当前 cycle、ID 和 command 校验；无法归属当前 cycle 的残留帧进入 stale/residual 处理，不得伪装成有效反馈；
* **消除运行期 0x25 模式回读**：严禁在运行期周期性读取 0x25 模式寄存器作为硬门禁（读 0x25 会严重破坏 50Hz 连续下发时序，引发超时）；0x25 仅在开机预检阶段校验一次。

#### (3) 辅助 0x30 状态轮询
`0x30` 轮询保留为显式的 safety scan、诊断和只读路径，不替代 0x29 伴随 `StatusFeedback` 主路径。
* **设计策略**：`poll_once()` / `full_scan()` 在 UART 调度器允许且请求明确时执行；常规 ACTIVE 50Hz cycle 的运行反馈来自 0x29 伴随回包；
* **采样语义**：0x30 轮询是独立的异步观测流，采样频率取决于显式 poll 调度，不能与 50Hz 0x29 主反馈混为一谈；
* **规范声明**：必须分别记录 0x29 cycle feedback 与 0x30 poll 的来源、cycle/ID 关联和时间戳。

## 3. 六电机驱动层与 J7 50Hz Servo 伺服架构 (Motor Driver & Concurrency Architecture)

### 3.1 为什么需要生产者-消费者解耦与单槽覆盖缓冲
* **生产侧**：上层 ROS 2 连续轨迹规划流以 **50Hz nominal** 频率发布目标（周期 20ms）；
* **消费侧**：物理 UART 串口受限于波特率与 6 轴往返延迟，物理执行频率约为 **25Hz nominal**（周期约 40ms）。

如果使用标准的 FIFO 消息队列，50Hz 生产流入 25Hz 消费队列会导致指令迅速积压，产生数百毫秒的严重控制滞后（控制延时会导致手术机器人跟踪发散）。因此，驱动节点设计了 **Single-Slot Latest-Value-Wins (单槽最新值覆盖)** 容器。

---

### 3.2 节点输入输出与接口定义
源码位于 [`pus6_motor_driver/pus6_motor_driver/motor_driver_node.py`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/motor_driver_node.py)。

```
[ ROS 2 控制流 (50Hz) ] ──► /pus6/command_joint_steps (QoS: KeepLast(1), BestEffort)
                                      │
                                      ▼
                        ┌───────────────────────────┐
                        │ sub_cb_group (极速入队)    │
                        └───────────────────────────┘
                                      │ (_target_lock)
                                      ▼
                        ┌───────────────────────────┐
                        │ Single-Slot Latest-Target │
                        └───────────────────────────┘
                                      │
                                      ▼
                        ┌───────────────────────────┐
                        │ worker_cb_group @ 25Hz    │ ──► [ Slew Recovery & Preflight ]
                        └───────────────────────────┘
                                      │
                                      ▼ (UART 0x29 × 6)
                        ┌───────────────────────────┐
                        │ 物理硬件电缸 ID1..ID6      │
                        └───────────────────────────┘
                                      │
                                      ▼ (Piggyback 解码)
          ┌───────────────────────────┼───────────────────────────┐
          ▼                           ▼                           ▼
/pus6/joint_states        /pus6/six_axis_telemetry    /pus6/joint_feedback/timed
(sensor_msgs/JointState)   (SixAxisTelemetry)          (TimedJointFeedback)
```

---

### 3.3 核心类与并发解耦设计

#### (1) Callback Group 隔离与多线程执行器
驱动节点在 `MultiThreadedExecutor` 中运行，显式划分三大独立互斥回调组：
1. `sub_cb_group = MutuallyExclusiveCallbackGroup()`：专用于订阅 `/pus6/command_joint_steps`，耗时 $< 0.05\text{ ms}$，绝不被串口阻塞；
2. `worker_cb_group = MutuallyExclusiveCallbackGroup()`：专用于 25Hz 物理 UART 下发与反馈接收；
3. `poll_cb_group = MutuallyExclusiveCallbackGroup()`：专用于系统空闲时的低频遥测轮询。

#### (2) 单一权威命令入口 (Single Authoritative Ingress)
系统仅开放 `/pus6/command_joint_steps` 作为物理下发的唯一权威入口，废除了旧版本位置浮点数与整数双入口导致的竞态隐患。

#### (3) Single-Slot 缓冲与守恒恒等式
在 `_target_lock` 保护下，容器永远只保存最新的一帧目标 `TargetEnvelope`。过时帧被安全覆盖，实测覆盖率约为 $50\%\sim 52\%$。系统严格遵循单槽守恒恒等式：
$$N_{\text{initial\_residual}} + N_{\text{received}} \equiv N_{\text{consumed}} + N_{\text{overwrite}} + N_{\text{final\_residual}}$$

---

### 3.4 常见误解与避坑指南
1. **误解：看到覆盖率达 50% 以为发生了系统丢包错误**
   * *澄清*：50Hz 生产与 25Hz 消费下，~50% 覆写是 Latest-Value-Wins 的必然数学结果，它确保驱动层下发的一定是此时刻最新位姿，彻底消除了历史队列延迟。
2. **误解：把驱动节点的多线程配置当成所有 ROS 2 节点的标准模板**
   * *澄清*：驱动节点由于仅有 25Hz 串口循环与极轻量的 ROS 回调，适合多线程；但对于高频 200Hz 的 FK 解算节点，多线程会导致严重的回调饥饿，必须使用独立单线程执行器。

---

### 3.5 推荐源码阅读顺序
1. [`pus6_motor_driver/pus6_motor_driver/motor_driver_node.py`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/motor_driver_node.py) 中的 `__init__` 与 `_handle_command_joint_steps`；
2. 跟踪 `_uart_worker_cycle` 理解从缓冲提取、斜率恢复到物理下发与 Timed 遥测发布的全过程。

---

## 4. 6PUS 并联机构解析逆运动学 (Analytical Inverse Kinematics & Geometry Config)

### 4.1 为什么并联机构逆运动学是闭式解析解
与传统串联机械臂（正解简单、逆解复杂多解）相反，**Stewart/6PUS 并联机构的逆运动学具有极其优异的闭式解析解 (Closed-Form Analytical Solution)**：
* 给定动平台 6DoF 空间位姿，6 根连杆的空间几何约束互相解耦；
* 每一条支链都可以独立列出二次几何方程，并直接求出唯一的解析实数解，计算耗时仅微秒级，非常适合 50Hz 实时高频控制。

```
                     动平台 (Platform, Radius d = 27.5mm)
                       /   |   |   |   |   \
                      /    |   |   |   |    \  (6 根定长碳纤维连杆 e = 84.0mm)
                     /     |   |   |   |     \
                   S1      S2  S3  S4  S5     S6 (上端球铰 Spherical Joints, Bi)
                    │       │   │   │   │      │
                   U1      U2  U3  U4  U5     U6 (下端万向铰 Universal Joints, Ai)
                    │       │   │   │   │      │
                   q1      q2  q3  q4  q5     q6 (主动移动副 Prismatic Sliders, 沿 Z 轴升降)
                    │       │   │   │   │      │
                  ================================ 静基座 (Base, Radius c = 40.0mm)
```

---

### 4.2 输入输出与数学模型推导
源码位于 [`pus6_kinematics/pus6_kinematics/analytical_6pus.py`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_kinematics/pus6_kinematics/analytical_6pus.py)。

1. **输入量**：动平台中心在基座坐标系下的位姿 $\mathbf{P} = [x, y, z]^\top$，姿态角 $[\alpha, \beta, \gamma]^\top$（Roll, Pitch, Yaw，ZYX 欧拉角约定）；
2. **输出量**：6 轴滑块绝对高度向量 $\mathbf{q} = [q_1, q_2, q_3, q_4, q_5, q_6]^\top$。

#### 几何约束推导：
动平台上第 $i$ 个球铰点在基座坐标系下的坐标为：
$$\mathbf{B}_i = \mathbf{P} + \mathbf{R}_{\text{ZYX}}(\alpha, \beta, \gamma) \mathbf{b}_i$$
基座滑块铰链坐标为 $\mathbf{A}_i = [a_{ix}, a_{iy}, q_i]^\top$。根据连杆定长约束 $\|\mathbf{B}_i - \mathbf{A}_i\|^2 = L_0^2$：
$$(B_{ix} - a_{ix})^2 + (B_{iy} - a_{iy})^2 + (B_{iz} - q_i)^2 = L_0^2$$

取滑块位于动平台下方的物理工作分支，得到 **唯一闭式解析逆运动学解**：
$$q_i = B_{iz} - \sqrt{L_0^2 - (B_{ix} - a_{ix})^2 - (B_{iy} - a_{iy})^2}$$

```python
# analytical_6pus.py 核心逆解实现
dx = px - a_i[0]
dy = py - a_i[1]
radicand = self.strut_length**2 - dx**2 - dy**2
if radicand < 0.0:
    raise WorkspaceLimitError("Target pose causes strut over-extension (unreachable)")
qi = pz - math.sqrt(radicand)
```

---

### 4.3 坐标转换、工作零位与 YAML 步数映射

所有几何参数与步数转换比例严格来自 [`6pus_geometry.yaml`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_kinematics/config/6pus_geometry.yaml)：

* **标称高度**：$z_{\text{nom}} \approx 0.1326123\text{ m}$ ($132.6123\text{ mm}$)；
* **工作零位 (`OPERATING_HOME`)**：$q_{\text{raw}} = [1000, 1000, 1000, 1000, 1000, 1000]\text{ step}$，对应推杆中位伸出量 **$50.0\text{ mm}$**（行程范围 $25.0\sim 75.0\text{ mm}$ 的中心点），相对控制坐标为 $[0, 0, 0, 0, 0, 0]\text{ m}$；
* **停放位 (`STORAGE_PARK`)**：$q_{\text{raw}} = [0, 0, 0, 0, 0, 0]\text{ step}$，推杆全缩回（最短位置，仅限受控关机停放）；
* **整步转换公式**：
  $$q_{\text{raw}, i} = 1000 + \text{round}\left((q_i - q_{\text{nom}, i}) \times \text{steps\_per\_meter}\right) \quad (\text{其中 } \text{steps\_per\_meter} = 40000.0)$$

---

### 4.4 常见误解与避坑指南
1. **误解：以为工作零位推杆伸出量是 25.0mm**
   * *澄清*：推杆的绝对行程是 $25\text{ mm} \sim 75\text{ mm}$，工作零位（raw=1000）处于行程中点，对应实际伸出量为 **$50.0\text{ mm}$**。
2. **误解：为了改机械参数直接编辑 `analytical_6pus.py`**
   * *澄清*：代码中 `Analytical6PusKinematics` 默认通过 `load_geometry_config()` 自动加载 YAML，必须统一修改 `pus6_kinematics/config/6pus_geometry.yaml`。

---

### 4.5 推荐源码阅读顺序
1. [`pus6_kinematics/pus6_kinematics/analytical_6pus.py`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_kinematics/pus6_kinematics/analytical_6pus.py)：重点看 `inverse_kinematics` 与几何参数初始化；
2. [`pus6_kinematics/pus6_kinematics/kinematics_node.py`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_kinematics/pus6_kinematics/kinematics_node.py)：理解工作空间包络检查与 ROS 话题转发。

---

## 5. 在线轨迹整形与连续控制算法 (Stateful Shaper & Continuous Control)

### 5.1 为什么需要在线三阶加加速度状态滤波 (Shaper)
在空间多自由度连续控制中，上层发布的轨迹可能包含阶跃突变或高频扰动。如果直接将逆解结果下发给电机，会导致无限大的加速度跳变，引发机械剧烈振动甚至电机堵转。
传统的 FIR 滤波器有相位延迟，且无法在动态目标突跳时保证状态连续。本项目设计了 **`StatefulJerkLimitedShaper`（三阶加加速度饱和状态整形滤波器）**。

---

### 5.2 输入输出与控制接口
源码位于 [`pus6_control/pus6_control/joint_trajectory_controller.py`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_control/pus6_control/joint_trajectory_controller.py)。

* **输入**：`/pus6/desired_joint_deltas`（来自运动学节点的关节相对期望增量）；
* **输出**：`/pus6/command_joint_steps`（叠加零位后的平滑绝对整步，50Hz 发布，QoS: BestEffort, KeepLast(1)）；
* **外环扩展 Hook**：`OuterLoopControllerHook`（预留给未来传感器闭环补偿）。

---

### 5.3 数学原理与状态微分方程

#### (1) 三阶连续传递函数
滤波器配置为三阶实极点 $s = -\omega_n$（带宽 $\omega_n = 20.0\text{ rad/s}$）：
$$H(s) = \frac{\omega_n^3}{(s + \omega_n)^3} = \frac{\omega_n^3}{s^3 + 3\omega_n s^2 + 3\omega_n^2 s + \omega_n^3}$$

#### (2) 状态微分方程与持久化维护
定义连续状态向量 $\mathbf{x} = [q, \dot{q}, \ddot{q}]^\top$。在每个控制周期 $dt$：
$$\dddot{q}(t) = \omega_n^3 \left( q_{\text{target}}(t) - q(t) \right) - 3\omega_n^2 \dot{q}(t) - 3\omega_n \ddot{q}(t)$$

#### (3) 物理饱和限幅与 $C^2$ 连续性保证
在积分推进过程中强制施加三级物理饱和：
* **速度限幅**：$|\dot{q}| \le 0.005\text{ m/s} \ (200\text{ step/s})$
* **加速度限幅**：$|\ddot{q}| \le 0.050\text{ m/s}^2 \ (2000\text{ step/s}^2)$
* **加加速度限幅**：$|\dddot{q}| \le 0.500\text{ m/s}^3 \ (20000\text{ step/s}^3)$

> [!IMPORTANT]
> **$C^2$ 连续性铁律**：当外部目标突变时，滤波器**绝不清零导数**（$\dot{q}$ 和 $\ddot{q}$ 保持连续积分），从而在数学上保证了输出位置的二阶可导，受力完全平滑无冲击。

---

### 5.4 五次多项式起止注入算法
对于单段离散轨迹的平滑起止，使用归一化五次多项式 $s(\tau) = 10\tau^3 - 15\tau^4 + 6\tau^5 \ (\tau \in [0, 1])$，确保起止点速度与加速度均为 0。

---

### 5.5 常见误解与避坑指南
1. **误解：以为上层目标改变时应该调用 `reset_state()`**
   * *澄清*：`reset_state()` 会瞬间将速度和加速度归零，破坏连续性产生机械冲击；它仅限在系统开机初始化或急停恢复时调用一次。
2. **误解：以为控制节点订阅了 FK 的位姿反馈来做闭环**
   * *澄清*：J4 控制节点是纯前向高精度整形架构，未将 FK 接入控制回路，保证了控制的高可靠性。

---

### 5.6 推荐源码阅读顺序
1. [`pus6_control/pus6_control/joint_trajectory_controller.py`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_control/pus6_control/joint_trajectory_controller.py)：精读 `StatefulJerkLimitedShaper.update` 积分逻辑；
2. [`pus6_control/pus6_control/control_node.py`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_control/pus6_control/control_node.py)：理解 50Hz 定时器循环与服务挂载。

---

## 6. 安全门禁、故障恢复与 HOME/PARK 状态机 (Safety Gates, Recovery & State Machine)

### 6.1 为什么需要分层硬门禁与主动自愈机制
手术机器人系统绝不允许出现由于丢包、网络延迟或软件异常引起的失控运动。
驱动节点作为连接物理硬件的最后一道防线，必须具备**无需上层干预的强制硬门禁拦截**与**确定性的故障自愈状态机**。

完整的安全流转语义为：
$$\text{Latest Target} \longrightarrow \text{Vector Slew Recovery (必要时)} \longrightarrow \text{Safety Preflight Hard Gate } (|\Delta q_i| \le 20) \longrightarrow \text{UART Dispatch}$$

---

### 6.2 驱动层多级安全硬门禁 (`_execute_safety_preflight`)
源码位于 [`pus6_motor_driver/pus6_motor_driver/safety.py`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/safety.py)。
每次向物理串口下发前，强制执行 5 级硬门禁检查：
1. **连接与维度**：控制器已建立连接且目标向量维度严格为 6；
2. **绝对硬件极限**：$0 \le q_i \le 2000\text{ step}$；
3. **正常运动软件包络**：$200 \le q_i \le 1800\text{ step}$（两端各留 5mm 缓冲）；
4. **单步增量硬门禁**：$|\Delta q_i| \le \mathbf{20\text{ steps}}$ ($0.5\text{ mm}$)，任何超过 20 步的物理跳变被直接无条件拒绝；
5. **故障熔断**：上一周期反馈 `fault_codes` 存在非 0 或电流超过门限立即熔断。

---

### 6.3 Time-Aware Vector Slew Recovery（时间自适应矢量斜率恢复）
* **解决的问题**：当发生 UART 通信长尾或 ROS 调度停顿（如累积延时 $355\text{ ms}$）时，目标推进量会累积超过 20 步。如果直接拒绝，会导致前检死锁 (Latch-Out Deadlock)；
* **核心自愈算法**：
  1. 以最后成功下发的目标为**命令锚点 (Command Anchor)**；
  2. 根据流逝时间动态计算安全裕量：$\text{Allowance} = \min(20, \max(5, \text{int}(v_{\max}\Delta t)))$；
  3. 若最大轴差 $\max_i |\Delta q_i| > \text{Allowance}$，计算统一缩放因子 $\alpha = \text{Allowance} / \max_i |\Delta q_i|$；
  4. **同比例缩放六轴矢量**：$q_{\text{disp}, i} = \text{round}(\text{Anchor}_i + \alpha (q_{\text{cand}, i} - \text{Anchor}_i))$；
  5. 严格保持空间姿态几何比例，绝不单轴截断，在数个周期内平滑追上最新目标流。

---

### 6.4 Delivery-Uncertain Resync（异常重锚与自愈状态机）
* **解决的问题**：当发生 50ms 串口读超时时，电机实际可能已执行目标但上位机未收到应答，导致命令锚点与物理位置脱节；
* **自愈流转**：
  $$\text{0x29 Write Timeout} \longrightarrow \text{自动 pause\_all()} \longrightarrow \text{读取六轴状态快照} \longrightarrow \text{严格校验} \longrightarrow \text{物理重锚} \longrightarrow \text{Slew Recovery} \longrightarrow \text{NORMAL}$$
* **条件恢复准则**：仅在读取到的快照**完整有效、非 stale、无故障且位置处于合法包络内**时，才允许重新锚定并恢复；否则触发 Safety Halt。

---

### 6.5 HOME / PARK 与 `maintenance_mode` 安全状态机

```
                      +---------------------------------------+
                      |   STORAGE_PARK (raw = [0, 0, 0, 0, 0, 0])
                      |   机械停放位 / 关机收拢态 (推杆全缩回)
                      +---------------------------------------+
                                          ▲
   maintenance_mode=true 显式授权受控收拢    │  J3 startup_homing 一键顶升
                                          ▼
                      +---------------------------------------+
                      |  OPERATING_HOME (raw = [1000, 1000, 1000...])
                      |  工作零位 / 控制基准原点 (中位伸出 50.0mm)
                      +---------------------------------------+
                                          ▲
                                          │ J4 连续轨迹运动 (受限在 [200, 1800] 正常包络)
                                          ▼
                      +---------------------------------------+
                      |     J4 动态连续空间工作区 (Normal Envelope)
                      |     关节步数: [200, 1800] step
                      +---------------------------------------+
```

* **`maintenance_mode` 保护机制**：
  * 默认 `maintenance_mode = false`：驱动层硬门禁强制拦截在 `[200, 1800]`，此时直接下发 PARK raw `[0]*6` 会被安全拒绝，防止误操作导致机构收拢撞击；
  * 显式 `maintenance_mode = true`：仅在通过专用维护服务 `/pus6/controlled_park` 且各项安全条件通过时，才允许受控进入 `[0, 2000]` 绝对停放区。

---

### 6.5.1 standalone commissioning 与未来 ROS 2 控制管线

`run_j3_startup_homing.py` 的 J6 HOME/PARK standalone 路径用于电机级 commissioning：验证 mode=0、目标寄存器 `0x29`、反馈读取、停止确认和安全门禁。它是可审计的点到点入口，不替代 ROS 2 的连续笛卡尔控制。

两条路径共享 `pus6_motor_driver` 的电机通信与安全抽象；standalone runner 直接调用六电机控制器，ROS 2 则由 `pus6_control` 产生目标并交给 `pus6_motor_driver_node`。J6 已验证的 `OPERATING_HOME=[1000]*6`、`STORAGE_PARK=[0]*6` 和目标限位语义可作为未来 ROS 2 启动/维护服务的接口基线，但 standalone 的 10 Hz 调度结果不能宣称为 ROS 2 50 Hz 管线的实机性能。

推荐先在 Mock/FakeBus 中复用同一驱动层和 write-audit，核对 HOME/PARK 状态机与 ROS 2 service/topic 入口；完整机构装配并获单独授权后，再进行分阶段实机验证。

### 6.6 常见误解与避坑指南
1. **误解：在正常控制流中随意拿实际反馈 $q_{\text{actual}}$ 去更新命令基准**
   * *澄清*：正常运动中严禁混淆命令基准与物理基准；物理重锚**仅且仅在** Delivery-Uncertain 异常处理路径中触发一次。
2. **误解：以为 PARK 只是普通工作空间里的一个点**
   * *澄清*：PARK 属于维护停放区，处于正常运动包络 `[200, 1800]` 之外，必须通过授权维护状态机执行。

---

### 6.7 推荐源码阅读顺序
1. [`pus6_motor_driver/pus6_motor_driver/safety.py`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/safety.py)：掌握 6 级安全门禁实现；
2. [`pus6_motor_driver/pus6_motor_driver/motor_driver_node.py`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/motor_driver_node.py) 的 `_apply_time_aware_slew_recovery` 与 `_execute_delivery_uncertain_resync`。

---

## 7. J5 进阶专题：多源统一时间轴与固定延迟状态重建 (Unified Timeline & Fixed-Lag)

### 7.1 为什么需要统一时间轴与 Fixed-Lag 同步

#### (1) 为什么标准 `/pus6/joint_states` 无法支撑并联机构高精度正运动学
在 ROS 中，标准的 `sensor_msgs/msg/JointState` 消息包含一个统一的 `header.stamp` 和一个长度为 6 的 `position` 数组。然而在物理现实中：
* 6 台电缸挂载在同一条半双工 RS485 总线上，主机必须以逐轴串行的方式依次进行 0x29 问答；
* 单轴通信耗时约 $3.5\sim 4.5\text{ ms}$，整组 6 轴扫描跨度为 $24\sim 30\text{ ms}$；
* 实际物理反馈是**时间分散的标量序列** $q_1(t_1), q_2(t_2), \dots, q_6(t_6)$，其中 $t_6 - t_1 \approx 18.18\text{ ms}$，它们根本不是在同一个物理瞬间采样的！

如果在动态运动中（例如动平台以 $5.0\text{ mm/s}$ 运动）直接把这 6 个非同步采样值打包当作同一瞬间的向量 $\mathbf{q}(t)$ 喂给正运动学解算器，各轴之间会产生高达 $0.12\text{ mm}$ 的**时间相位畸变 (Temporal Skew Distortion)**，导致正解残差急剧发散、位姿失真。

#### (2) 为什么采用 Fixed-Lag（固定延迟双向夹逼对齐）
为了消除串行扫描引起的相位畸变，系统引入了固定延迟同步器 `FixedLagJointSynchronizer`：
* **核心思路**：容忍一个微小的已知延迟（标称 $\Delta t_{\text{lag}} = 50\text{ ms}$），在当前物理时刻 $t_{\text{now}}$ 查询过去时刻 $t_{\text{query}} = t_{\text{now}} - \Delta t_{\text{lag}}$ 的状态；
* **物理优势**：由于查询时刻落在历史样本的“过去”，每个轴在 $t_{\text{query}}$ 前后均已沉淀了真实历史观测点（左支撑点与右支撑点），从而可以通过**高精度的双向夹逼线性插值**，精确重构出严格同时刻的关节空间向量 $\mathbf{q}_{\text{sync}}(t_{\text{query}})$。

---

### 7.2 输入、处理与输出数据流

```
[ 底层 RS485 逐轴串行应答 ] ──► 记录 Host CLOCK_MONOTONIC 接收完成时刻 (per_axis_rx_done_monotonic_ns)
                                             │
                                             ▼
                                /pus6/joint_feedback/timed (TimedJointFeedback)
                                             │
                                             ▼ (push_feedback 写入 6 轴滑动历史 deque)
                           ┌───────────────────────────────────────────┐
                           │ FixedLagJointSynchronizer (RLock 保护)    │
                           └───────────────────────────────────────────┘
                                             │
                                             ▼ (200Hz 定时器以 t_query = t_now - 50ms 查询)
                           ┌───────────────────────────────────────────┐
                           │ 6 轴双向夹逼判定 (t_left <= t_query <= t_right)│
                           └───────────────────────────────────────────┘
                                             │
                     ┌───────────────────────┴───────────────────────┐
                     ▼ (任一轴未满足夹逼 / 缺帧)                       ▼ (6 轴全部成功夹逼)
             [ 严格 0 外推拒绝发布 ]                          [ 线性内插生成 q_sync(t_query) ]
           (WAIT_FUTURE_SAMPLE / WAIT_TOO_EARLY)             (SyncStatus.SYNCED 供正解解算)
```

---

### 7.3 核心源码与文件定位
1. **消息契约**：[`pus6_msgs/msg/TimedJointFeedback.msg`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_msgs/msg/TimedJointFeedback.msg) 与 [`pus6_msgs/msg/TimedJointCommand.msg`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_msgs/msg/TimedJointCommand.msg)
2. **状态同步器**：[`pus6_kinematics/pus6_kinematics/state_synchronizer.py`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_kinematics/pus6_kinematics/state_synchronizer.py)
3. **实时消费节点**：[`pus6_kinematics/pus6_kinematics/fk_node.py`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_kinematics/pus6_kinematics/fk_node.py)

---

### 7.4 统一时间合同与时间戳语义规范

#### (1) ROS `Header.stamp` 与 Host `CLOCK_MONOTONIC` 的本质区别
* **`header.stamp`**：标准 ROS 2 消息发布时间 (`node.get_clock().now()`)，受系统墙上时钟或 ROS 仿真时钟影响，可能发生 NTP 授时阶跃跳变，仅用于 ROS 通信路由与 rosbag 录包对齐；
* **`query_monotonic_timestamp_ns` / `CLOCK_MONOTONIC`**：系统内部算法状态的**物理基准时间**，严格采用 Linux 单调时钟 `time.clock_gettime_ns(time.CLOCK_MONOTONIC)`，保证全局严格单调递增，不受系统调时影响，与双 H30 IMU 姿态解算全局同源。

#### (2) `per_axis_rx_done_monotonic_ns` 的物理代理属性
`TimedJointFeedback` 中 `per_axis_rx_done_monotonic_ns[6]` 记录的是**主机内核解析完成该电机 0x29 应答帧的单调时刻**：
* **重要概念**：由于电机固件未开放微秒级硬件时间戳，它**不是**电机内部编码器采样的微观瞬间，而是上位机能够确知、可审计且严格单调的**观测事件完成时间代理 (Observation Completion Proxy)**。

#### (3) `uart_cycle_sequence` 序列语义与配对边界
* 在主动写控制周期下，`TimedJointCommand` 与 `TimedJointFeedback` 携带相同的 `uart_cycle_sequence`，实现下发与观测的 1-to-1 追踪；
* 在零速静止、故障恢复或只读审计阶段，系统会产生合法的 `Feedback-only` 帧，确保遥测连续。

---

### 7.5 `FixedLagJointSynchronizer` 内部机制与算法推导

#### (1) 数据结构与 `threading.RLock()` 线程安全
类内部维护 6 个双端队列（`collections.deque(maxlen=500)`），分别存储各轴的历史三元组：
$$\mathcal{H}_i = \Big\{ \big(t_i^{(k)}, q_i^{(k)}, \text{raw\_step}_i^{(k)}\big) \Big\}_{k=1}^{N_i}$$
* **为什么需要 `RLock`**：订阅回调线程（`push_feedback` 写入 deque）与高频 200Hz 查询定时器（`query_sync_configuration` 读取和二分查找）并发访问相同的 6 个 deque。所有读写方法外层必须包裹递归互斥锁 `with self._lock:`，杜绝迭代器失效与数据撕裂。

#### (2) 双向夹逼 (Left/Right Bracketing) 与线性插值
在查询时刻 $t_{\text{query}} = t_{\text{now}} - \text{fixed\_lag\_delay\_ns}$：
1. **深度检查**：所有 6 个队列长度必须 $\ge 2$（至少有两个有效历史点）；
2. **时间跨度检查 (Bracketing Check)**：
   $$\forall i \in \{1..6\}, \quad t_{i, \text{oldest}} \le t_{\text{query}} \le t_{i, \text{newest}}$$
3. **二分查找与线性插值**：对每个轴查找满足 $t_i^{(k-1)} \le t_{\text{query}} \le t_i^{(k)}$ 的相邻两帧，计算插值系数：
   $$\alpha_i = \frac{t_{\text{query}} - t_i^{(k-1)}}{t_i^{(k)} - t_i^{(k-1)}} \in [0.0, 1.0]$$
   $$\hat{q}_i(t_{\text{query}}) = q_i^{(k-1)} + \alpha_i \big(q_i^{(k)} - q_i^{(k-1)}\big)$$

#### (3) 严格 0 外推原则 (Zero Extrapolation)
只要有任意一轴未满足夹逼条件，同步器立即拒绝输出，**绝不进行向后或向前的趋势外推**。

#### (4) `SyncStatus` 状态机与 50ms Nominal Fixed-Lag 的本质
* **`SyncStatus.EMPTY_BUFFER`**：队列样本不足 2 个；
* **`SyncStatus.WAIT_TOO_EARLY`**：$t_{\text{query}} < t_{i, \text{oldest}}$（查询时刻已被滑动队列挤出）；
* **`SyncStatus.WAIT_FUTURE_SAMPLE`**：$t_{\text{query}} > t_{i, \text{newest}}$（最新硬件帧尚未到达）；
* **`SyncStatus.SYNCED`**：6 轴全部夹逼成功，成功返回插值后的 $\mathbf{q}_{\text{sync}}$；
* **50ms 只是标称延迟参数，绝非“同步保证”**：若发生串口丢包或通信波动延时 $>50\text{ ms}$，夹逼必然失败并返回 `WAIT_FUTURE_SAMPLE`，这是安全防御的正确表现。

---

### 7.6 常见误解与避坑指南
1. **误解：以为 `per_axis_rx_done` 是电机编码器硬件采样的真实物理瞬间**
   * *澄清*：电机固件未提供硬件时间戳，`per_axis_rx_done` 是主机侧接收完成时间的最高精度代理。
2. **误解：以为配置了 50ms 延迟就能 100% 保证每一帧都能成功内插**
   * *澄清*：50ms 仅是标称延迟，若串口通信超时或出现抖动长尾，状态机会正确返回等待/丢弃状态。
3. **误解：以为队列数据不足时可以用一阶速度外推预测位置**
   * *澄清*：在并联机构中，外推伪数据会导致正解严重失真甚至发散，系统严格实行 0 外推。

---

### 7.7 推荐源码阅读顺序
1. [`pus6_msgs/msg/TimedJointFeedback.msg`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_msgs/msg/TimedJointFeedback.msg)；
2. [`pus6_kinematics/pus6_kinematics/state_synchronizer.py`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_kinematics/pus6_kinematics/state_synchronizer.py) 中的 `push_feedback` 与 `query_sync_configuration`。

---

## 8. J5 进阶专题：解析雅可比数值正运动学与实时解算节点 (Analytical Jacobian Numerical FK & Pus6FkNode)

### 8.1 为什么并联机构正运动学必须数值求解
在并联机构运动学中，**正运动学 (Forward Kinematics, FK)** 要求根据 6 根推杆的已知伸出量 $\mathbf{q}$，解算动平台在空间中的 6DoF 位姿 $\mathbf{P}=[x,y,z]^\top$ 和姿态角 $[\alpha, \beta, \gamma]^\top$。
* **数学本质**：Stewart 并联机构的正运动学涉及 6 个多元二次非线性几何约束方程联立，**在数学上不存在通用的显式解析闭式解**；
* **工程解法**：必须采用**解析雅可比矩阵构造与自适应阻尼 Levenberg-Marquardt (LM) 数值迭代法**。

---

### 8.2 输入、处理与输出数据流

```
[ FixedLag 状态同步器 ] ──► 输出时间对齐的关节向量 q_sync(t_query)
                                      │
                                      ▼
                        ┌───────────────────────────┐
                        │ Pus6FkNode (200Hz 定时器) │
                        └───────────────────────────┘
                                      │ (传入 q_sync 与 上一有效收敛位姿 x_last 做 Warm-Start)
                                      ▼
                        ┌───────────────────────────┐
                        │ 6 支链几何残差方程 f_i(x)  │
                        │ + 6x6 解析雅可比矩阵 J 构造│
                        │ + Levenberg-Marquardt 迭代│
                        └───────────────────────────┘
                                      │
                                      ▼ (LM 迭代收敛 ||f(x)|| < 1e-4 m)
                        ┌───────────────────────────┐
                        │ SO(3) 测地线角度跳变门禁  │ (theta <= 0.15 rad 且 ||Δp|| <= 15mm)
                        └───────────────────────────┘
                                      │
                     ┌────────────────┴────────────────┐
                     ▼ (通过门禁)                       ▼ (跳变拒绝 / 不收敛)
      ┌─────────────────────────────┐         [ 拒绝发布当前位姿 ]
      │ 更新 _last_successful_pose  │         [ 保持上一帧有效成功锚点 ]
      └─────────────────────────────┘
                     │
         ┌───────────┴───────────┐
         ▼                       ▼
/pus6/fk/state            /pus6/fk/pose
(FkStateStamped)          (geometry_msgs/PoseStamped)
```

---

### 8.3 核心源码与文件定位
1. **数值正解数学核心**：[`pus6_kinematics/pus6_kinematics/analytical_6pus.py`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_kinematics/pus6_kinematics/analytical_6pus.py) 中的 `forward_kinematics_with_metrics`
2. **实时解算节点**：[`pus6_kinematics/pus6_kinematics/fk_node.py`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_kinematics/pus6_kinematics/fk_node.py)
3. **状态诊断消息**：[`pus6_msgs/msg/FkStateStamped.msg`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_msgs/msg/FkStateStamped.msg)

---

### 8.4 数学原理与算法完整推导

#### (1) 六条支链定长几何约束方程
对于 6PUS 并联机构的第 $i$ 条支链：
* **动平台球铰点实时空间坐标**：$\mathbf{p}_i = \mathbf{p} + \mathbf{R}_{\text{ZYX}}(\alpha, \beta, \gamma) \mathbf{b}_i$；
* **基座滑块铰点实时坐标**：$\mathbf{s}_i(q_i) = \mathbf{a}_i + q_i \hat{\mathbf{z}} = [a_{ix}, a_{iy}, q_i]^\top$；
* **连杆定长约束残差**：
  $$f_i(\mathbf{x}) = \|\mathbf{p}_i - \mathbf{s}_i(q_i)\| - L_0 = 0 \quad (i = 1, \dots, 6)$$
  其中待求解的状态向量为 6 维笛卡尔位姿：$\mathbf{x} = [x, y, z, \alpha, \beta, \gamma]^\top$。

#### (2) ZYX 欧拉角约定与旋转矩阵
旋转矩阵采用 ZYX 欧拉角约定（$\alpha=\text{Roll}, \beta=\text{Pitch}, \gamma=\text{Yaw}$）：
$$\mathbf{R}_{\text{ZYX}}(\alpha, \beta, \gamma) = \mathbf{R}_z(\gamma) \mathbf{R}_y(\beta) \mathbf{R}_x(\alpha)$$

#### (3) 6×6 解析雅可比矩阵 $\mathbf{J}$ 的构造
定义支链单位方向矢量 $\mathbf{u}_i = \frac{\mathbf{p}_i - \mathbf{s}_i}{\|\mathbf{p}_i - \mathbf{s}_i\|}$。残差向量相对于状态量 $\mathbf{x}$ 的偏导数构成雅可比矩阵 $\mathbf{J} \in \mathbb{R}^{6 \times 6}$ 的第 $i$ 行：
$$\mathbf{J}_i = \left[ \mathbf{u}_i^\top, \quad \mathbf{u}_i^\top \left(\frac{\partial \mathbf{R}}{\partial \alpha}\mathbf{b}_i\right), \quad \mathbf{u}_i^\top \left(\frac{\partial \mathbf{R}}{\partial \beta}\mathbf{b}_i\right), \quad \mathbf{u}_i^\top \left(\frac{\partial \mathbf{R}}{\partial \gamma}\mathbf{b}_i\right) \right]$$
其中旋转矩阵相对于欧拉角的解析偏导数通过显式矩阵乘法直接计算：
$$\frac{\partial \mathbf{R}}{\partial \alpha} = \mathbf{R}_z(\gamma)\mathbf{R}_y(\beta)\frac{\partial \mathbf{R}_x(\alpha)}{\partial \alpha}, \quad \frac{\partial \mathbf{R}}{\partial \beta} = \mathbf{R}_z(\gamma)\frac{\partial \mathbf{R}_y(\beta)}{\partial \beta}\mathbf{R}_x(\alpha), \quad \frac{\partial \mathbf{R}}{\partial \gamma} = \frac{\partial \mathbf{R}_z(\gamma)}{\partial \gamma}\mathbf{R}_y(\beta)\mathbf{R}_x(\alpha)$$
> [!TIP]
> **为什么必须使用解析雅可比**：解析雅可比避免了有限差分法的数值截断误差与多次求值开销，使单次雅可比构造仅耗时微秒级。

#### (4) Levenberg-Marquardt 自适应阻尼迭代
LM 迭代公式为：
$$\Delta \mathbf{x} = - \big(\mathbf{J}^\top \mathbf{J} + \lambda \mathbf{I}\big)^{-1} \mathbf{J}^\top \mathbf{f}(\mathbf{x})$$
* 当一步迭代后几何残差下降时，减小阻尼 $\lambda = \max(\lambda / 3, 10^{-7})$，算法表现为极速的高斯-牛顿二次收敛；
* 当残差上升时，增大阻尼 $\lambda = \min(\lambda \times 2, 10^{3})$，算法退化为稳健的最速下降步。

#### (5) Warm-Start 暖启动与工作分支连续性认知
在并联机构理论中，并联平台在全代数空间可能存在多个装配分支。在本项目实际工程中，动平台始终运行在 `OPERATING_HOME` 附近的物理工作分支内：
* 算法以**上一控制周期成功收敛的位姿**作为初始猜测（Warm-Start）；
* 求解器在当前物理分支的局部凸吸引域内迭代，平均仅需 $3\sim 4$ 步即可收敛到 $<10^{-10}\text{ m}$ 几何残差；
* **重要认知**：Warm-Start 属于局部连续性追踪策略，前提是系统初始位姿位于真实物理分支附近；**它不代表在任意未知初始状态下的全局唯一解证明**。

#### (6) $\text{SO}(3)$ 测地线角度跳变门禁
为了防止异常输入导致求解器跃迁到非物理的代数装配分支，节点绝不使用简单的欧拉角差值（欧拉角存在万向节死锁和 $\pm\pi$ 卷绕突变问题），而是计算当前旋转矩阵 $\mathbf{R}_{\text{curr}}$ 与上一有效收敛矩阵 $\mathbf{R}_{\text{last}}$ 之间的李群测地线距离：
$$\theta_{\text{geodesic}} = \arccos\left(\text{clip}\left(\frac{\text{Tr}(\mathbf{R}_{\text{last}}^\top \mathbf{R}_{\text{curr}}) - 1}{2}, -1.0, 1.0\right)\right)$$
* 若 $\theta_{\text{geodesic}} > 0.15\text{ rad}\ (\approx 8.59^\circ)$ 或平移跳变 $\|\Delta \mathbf{p}\| > 15.0\text{ mm}$，判定为分支跳变并拒绝发布；
* 此时**不更新** `_last_successful_pose` 锚点，有效保护物理工作分支。

#### (7) 数值一致性 (IK↔FK Round-Trip) 与真实机械精度的物理认知边界
* **测试指标**：在安全包络内 1000 个随机样本下，平移数值残差均值 $5.83\times 10^{-9}\text{ mm}$，$\text{SO}(3)$ 姿态残差均值 $1.21\times 10^{-7\circ}$；
* **物理认知强调**：**数值一致性仅证明正逆解在纯数学代数与代码实现上无 bug，绝不代表真实物理机器人的机械加工与装配定位精度！**

---

### 8.5 实时节点 `Pus6FkNode` 与 `FkStateStamped` 字段解析

#### (1) `FkStateStamped.msg` 核心字段语义
* `query_monotonic_timestamp_ns`：算法固定延迟查询物理时刻（`CLOCK_MONOTONIC`）；
* `synced_positions_meters[6]`：6 轴插值对齐后的同步关节高度；
* `pose` (`geometry_msgs/Pose`)：解算得到的动平台位置与四元数姿态；
* `euler_zyx_rad[3]`：ZYX 欧拉角（Roll, Pitch, Yaw）；
* `converged` (bool)：LM 迭代是否成功收敛；
* `iterations` (int32)：LM 实际迭代步数；
* `residual_norm_meters` (float64)：最终 6 支链定长几何约束残差范数；
* `branch_continuity_passed` (bool)：$\text{SO}(3)$ 测地线跳变门禁是否通过；
* `compute_time_us` (float64)：单次正解解算耗时（微秒）。

#### (2) 200Hz FK Query 的物理含义澄清
* **200Hz** 是上位机 Fixed-Lag 队列的**查询与插值解算频率**；
* 它**不等于**底层电机产生了 200Hz 的新物理观测（底层串口通信依然为 25Hz）。

---

### 8.6 执行器架构硬约束：为什么必须独占 `SingleThreadedExecutor` (A~F 分层消融复盘)

#### (1) 独立单线程执行器硬约束
在 [`pus6_kinematics/fk_node.py`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_kinematics/pus6_kinematics/fk_node.py) 的 `main()` 函数中，明确创建并独占**独立的 `SingleThreadedExecutor`**：
```python
rclpy.init(args=args)
node = Pus6FkNode()
executor = SingleThreadedExecutor()
executor.add_node(node)
executor.spin()
```

#### (2) A~F 分层消融实验复盘与性能对比

消融实验 [`bone_ws/scripts/run_j5_c2_rt_sw_c_ablation.py`](file:///home/lyapunov/bone_drill/bone_ws/scripts/run_j5_c2_rt_sw_c_ablation.py) 将负载从 A 层（空 200Hz 定时器）逐层加到 F 层（完整节点 + 异常注入）。在独立单线程执行器下实测表现极其优异：

| 层级 | 测试内容 | callbacks/10s | 实际均值 Hz | period P50 / P95 / P99 (ms) | callback 耗时 P50 / P95 / P99 (ms) |
| :--- | :--- | :---: | :---: | :---: | :---: |
| **A** | 空 200 Hz 定时器 | 2000 | 199.98 | 5.001 / 5.184 / 5.369 | 0.001 / 0.002 / 0.003 |
| **B** | + 30 Hz 模拟反馈发布 | 2001 | 200.06 | 5.001 / 5.199 / 5.428 | 0.001 / 0.002 / 0.003 |
| **C** | + Fixed-Lag 同步，不做 FK | 2001 | 200.07 | 5.001 / 5.304 / 5.635 | 0.045 / 0.091 / 0.115 |
| **D** | + 数值 FK，不发布 ROS 话题 | 2000 | 199.97 | 4.999 / 5.321 / 5.707 | 0.920 / 1.461 / 1.837 |
| **E** | 完整 Pus6FkNode 节点 | 2000 | 199.98 | 4.999 / 5.270 / 5.674 | 1.079 / 1.663 / 2.014 |
| **F** | 完整节点 + 异常故障注入 | 2000 | 199.97 | 4.999 / 5.162 / 5.338 | 1.133 / 1.802 / 2.574 |

#### (3) 多线程回调饥饿现象与架构结论
* **多线程对照表现**：在 `MultiThreadedExecutor(4)` 下，A/B 层尚可维持 200Hz，但在 C-E 复合负载下实际触发频率断崖式暴跌至 **25 ~ 38 Hz**，周期 P95 恶化至 145 ~ 234ms，最大停顿高达 650 ~ 980ms；
* **架构复盘**：在 Python `rclpy` 多线程执行器中，高频 200Hz 定时器与高频发布/订阅在并发 Ready 队列调度下发生了严重的回调饥饿 (Callback Starvation)。而在独立的 `SingleThreadedExecutor` 下，订阅、固定延迟查询、FK 计算和发布在同一条可预测的调度链上执行，最终稳定达到 **2007 callbacks/10s (200.09Hz, P95 5.14ms)**。
* **工程准则**：`Pus6FkNode` 必须独占独立的 `SingleThreadedExecutor`，严禁将其塞入通用的多线程执行器中。

---

### 8.7 常见误解与避坑指南
1. **误解：将算法称为“解析 FK”**
   * *澄清*：Stewart 机构正解不存在解析通解，系统采用的是**基于解析雅可比的数值 LM 迭代求解**，严禁混淆概念。
2. **误解：以为 200Hz FK 发布意味着电缸能够以 200Hz 响应控制**
   * *澄清*：200Hz 是上位机插值观测解算的频率，物理电机的通信与控制带宽依然为 25Hz。
3. **误解：以为 IK↔FK 的 $10^{-9}\text{ mm}$ 误差证明了真实机械精度**
   * *澄清*：该误差仅证明正逆解数学模型的代码自洽性，真实机械精度必须等总装后经由光学跟踪仪实测。

---

### 8.8 推荐源码阅读顺序
1. [`pus6_msgs/msg/FkStateStamped.msg`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_msgs/msg/FkStateStamped.msg)；
2. [`pus6_kinematics/pus6_kinematics/analytical_6pus.py`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_kinematics/pus6_kinematics/analytical_6pus.py) 中的 `forward_kinematics_with_metrics`；
3. [`pus6_kinematics/pus6_kinematics/fk_node.py`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_kinematics/pus6_kinematics/fk_node.py) 中的 `_fk_query_loop` 与 `main()`；
4. [`bone_ws/scripts/run_j5_c2_rt_sw_c_ablation.py`](file:///home/lyapunov/bone_drill/bone_ws/scripts/run_j5_c2_rt_sw_c_ablation.py) 了解 A~F 分层消融测试。

---

## 9. 【全链路实战追踪】双数据流端到端生命周期透视 (End-to-End Tracing)

> **场景**：执行 J4-F1E 全 6 自由度空间连续动态轨迹实验（$18.0\text{ s}$，含 10 周期静态预热 + 1.0s 五次注入 + 15.0s 满幅空间运动 + 1.0s 退出 + 1.0s HOME 沉降）。

```
                                  双数据流端到端交互全时序图

 [ 上位轨迹规划器 ]                   [ pus6_kinematics_node ]               [ pus6_control_node ]
        │                                       │                                      │
        │ 1. 50Hz PoseStamped                   │                                      │
        ├──────────────────────────────────────►│                                      │
        │                                       │ 2. 50Hz desired_joint_deltas         │
        │                                       ├─────────────────────────────────────►│
        │                                       │                                      │ 3. 50Hz command_joint_steps
        │                                       │                                      ├────────────────────────┐
        │                                       │                                      │                        │
        │                                       │                                      │                        ▼
 [ pus6_fk_node (单线程) ]              [ pus6_motor_driver_node ]                  [ 物理电缸 ID1..ID6 ]
        │                                       │ ◄─────────────────────────────────────────────────────────────┘
        │                                       │ 4. Single-Slot 缓冲 + Slew 恢复 + Preflight 前检
        │                                       │ 5. 25Hz 0x29 串行写入 (带 _bus_lock_ctx 批次锁)
        │                                       ├─────────────────────────────────────►│
        │                                       │ 6. 15 字节 Piggyback 伴随状态返回     │
        │                                       │◄─────────────────────────────────────┤
        │                                       │ 7. 记录 CLOCK_MONOTONIC 接收完成时刻  │
        │ 8. TimedJointFeedback 遥测流          │
        │◄──────────────────────────────────────┤
        │ 9. 压入 FixedLag 历史队列             │
        │ 10. 200Hz 定时器以 t_query 查询       │
        │ 11. 双向夹逼线性内插生成 q_sync       │
        │ 12. 解析雅可比 LM 数值正解迭代        │
        │ 13. SO(3) 测地线跳变门禁校验          │
        │ 14. 发布 /pus6/fk/state 与 fk/pose    │
        ▼                                       ▼
```

### 9.1 连续空间运动控制下行生命周期 (Control Pipeline)
1. **Step 1：笛卡尔位姿注入**
   规划器以 50Hz 向 `/pus6/target_cartesian_pose` 发布参考位姿 $\mathbf{P}(t) = [x, y, z, \text{roll}, \text{pitch}, \text{yaw}]^\top$。
2. **Step 2：纯几何闭式解析逆解**
   `pus6_kinematics_node` 收到目标后，首先校验工作空间包络（平移 $\le 15\text{ mm}$，高度差 $\le 20\text{ mm}$，转角 $\le 0.1\text{ rad}$）；通过后调用 `Analytical6PusKinematics.inverse_kinematics()` 计算 6 轴绝对高度 $q_i$，并扣除标称高度得到相对差分量，发布到 `/pus6/desired_joint_deltas`。
3. **Step 3：三阶加加速度连续滤波整形**
   `pus6_control_node` 接收相对差分量，通过 `StatefulJerkLimitedShaper` 进行 $C^2$ 连续微分积分推进，施加速度、加速度与 Jerk 三级饱和限幅，叠加 `OPERATING_HOME`（raw=1000）转换为目标整步，以 QoS `KeepLast(1), BestEffort` 发布到 `/pus6/command_joint_steps`。
4. **Step 4：驱动层极速入队与安全自愈**
   `pus6_motor_driver_node` 的 `sub_cb_group` 极速接收指令并存入 `_target_lock` 保护的 Single-Slot 单槽容器；25Hz 的 `worker_cb_group` 唤醒提取最新帧（覆盖过时中间帧）；若遭遇通信停顿延时，自动触发 Time-Aware Vector Slew Recovery 按时间裕量等比例缩放矢量；通过 5 级 Preflight 硬门禁检查。
5. **Step 5：UART 批次下发与 Piggyback 伴随反馈**
   在 6 轴全局批次锁 `_bus_lock_ctx` 保护下，向 ID1 ~ ID6 依次下发 0x32 写入 0x29 寄存器；各电缸在执行动作的同时立即回传 15 字节伴随状态帧。
6. **Step 6：运动收敛与状态发布**
   驱动层解包回传帧生成 `SixAxisStateSnapshot`，发布 `/pus6/joint_states` 与 `/pus6/six_axis_telemetry`。轨迹执行完成后在 `OPERATING_HOME` 邻域完成沉降。

---

### 9.2 实时位姿状态观测上行生命周期 (Observation Pipeline)
1. **Step 1：单调接收时刻记录**
   在 25Hz UART 物理通信循环中，驱动层在收到各电机应答帧的瞬间，立即调用 `time.clock_gettime_ns(time.CLOCK_MONOTONIC)` 记录内核接收完成时刻 `per_axis_rx_done_monotonic_ns[6]`。
2. **Step 2：Timed 遥测消息封装**
   驱动层组装 `TimedJointFeedback` 消息，打上相同的 `uart_cycle_sequence`，发布到 `/pus6/joint_feedback/timed`。
3. **Step 3：滑动历史有界入队**
   `Pus6FkNode` 订阅该话题，在 `threading.RLock()` 保护下将各轴接收时刻、实际位置及整步压入 `FixedLagJointSynchronizer` 内部的 6 个滑动双端队列 (`deque(maxlen=500)`)。
4. **Step 4：固定延迟双向夹逼对齐**
   `Pus6FkNode` 内部独立单线程执行器的 200Hz 定时器触发，计算算法查询时刻 $t_{\text{query}} = t_{\text{now}} - 50\text{ ms}$；对 6 轴历史进行二分查找；若 6 轴全部满足 $t_{\text{left}} \le t_{\text{query}} \le t_{\text{right}}$，执行线性插值重构出同步关节向量 $\mathbf{q}_{\text{sync}}$；任一轴未满足则严格返回等待，零外推。
5. **Step 5：解析雅可比 LM 数值正解**
   节点将 $\mathbf{q}_{\text{sync}}$ 传入 `Analytical6PusKinematics.forward_kinematics_with_metrics()`，以 `_last_successful_pose` 暖启动，解析构造 $6 \times 6$ 雅可比矩阵 $\mathbf{J}$，自适应调整阻尼 $\lambda$ 迭代求解动平台 6DoF 位姿。
6. **Step 6：李群门禁校验与状态发布**
   解算收敛后，计算旋转矩阵李群 $\text{SO}(3)$ 测地线角距离与平移跳变；通过门禁（$\theta_{\text{geodesic}} \le 0.15\text{ rad}, \|\Delta \mathbf{p}\| \le 15\text{ mm}$）后更新成功锚点，发布 `/pus6/fk/state` (`FkStateStamped`) 与 `/pus6/fk/pose` (`PoseStamped`)。

> [!IMPORTANT]
> **架构核心铁律**：FK 观测链产生的高频位姿仅用于外部监控与未来多传感器融合评估，**绝不进入 J4 控制器，绝不向下行控制链反馈任何修正量**。控制链与观测链严格保持单向解耦。

---

## 10. 核心工程问题演进与实机踩坑复盘 (Engineering Lessons & Post-Mortem)

在 6PUS 项目从驱动、逆解、控制到正解与时间同步的演进过程中，我们定位并解决了 7 项标志性的重大工程问题。每一个问题均严格按照标准工业级复盘归纳：

### 1. 双命令入口竞态与覆写翻倍 Bug (J4-F1B-1)
* **[现象/问题]**：50Hz 规划流注入 25Hz 驱动层，理论覆写次数约为 250 次，但实测日志记录了 746 次覆写（翻倍）。
* **[证据/测试]**：细粒度日志统计显示，生产队列计数以双倍速率增加。
* **[根因边界]**：`control_node` 同时发布了浮点数 `command_joint_positions` 和整步 `command_joint_steps` 两个话题，驱动层两路回调并发抢锁写入同一个单槽缓冲区 `_latest_target`。
* **[修复方案]**：彻底收口为单一权威控制入口，驱动节点仅订阅 `/pus6/command_joint_steps`，废除浮点数控制话题。
* **[工程经验]**：多线程消费者节点必须保证**单一权威写入源**，严禁并存语义重复的多路控制订阅。

---

### 2. 总线竞争假设排除与物理通信长尾定位 (J4-F1B-2)
* **[现象/问题]**：动态运动时 UART 批次耗时 P95 恶化至 48.7ms，有效控制频率下降至 22.8Hz。
* **[证据/测试]**：进行微秒级仪器化埋点测试（Instrumentation），实测 `uart_lock_wait_ms = 0`，写流活跃期间后台轮询被活跃保护有效拦截为 0 次。
* **[根因边界]**：彻底排除了后台轮询与主写线程争抢总线锁的假设。延迟长尾主要来自 6 台电缸逐轴 0x29 物理通信请求-响应的硬件往返耗时（Hardware Wall-Time 波动）。
* **[修复方案]**：保持 25Hz 标称控制周期，保留写流活跃保护门禁，容忍物理硬件耗时波动。
* **[工程经验]**：遇到性能长尾切忌主观猜测多线程锁瓶颈，必须依赖高精度仪器化埋点数据界定软硬件边界。

---

### 3. 安全前检死锁 (Latch-Out Deadlock) (J4-F1D)
* **[现象/问题]**：瞬态串口超时累积了 355ms 延时，目标流推进了 25 步（超过单步 20 步硬门禁），导致后续所有波峰帧持续被拒绝，系统陷入完全停滞的死锁状态。
* **[证据/测试]**：日志显示 `preflight_failed_count` 持续激增，电机停在非零位无法恢复。
* **[根因边界]**：前检被拒绝后命令锚点被冻结，而上层目标流继续推进，两者的差值永远大于 20 步门禁。
* **[修复方案]**：实施 **Time-Aware Vector Slew Recovery**，以最后成功下发的命令为基准，根据流逝时间动态计算安全裕量，并将 6 轴矢量同比例缩放追赶。
* **[工程经验]**：安全门禁不能采用简单的二值硬拦截，必须配合**时间自适应与空间几何比例保持的自愈算法**。

---

### 4. Delivery-Uncertain 悬空状态机与物理重锚 (J4-F1E)
* **[现象/问题]**：发生 50ms 串口读超时时，电缸可能已经执行了写入指令但上位机未收到应答，导致命令基准与物理位置脱节。
* **[证据/测试]**：通信瞬断后，若盲目沿用旧命令锚点，多轴之间会累积未知的空间偏斜。
* **[修复方案]**：建立 **Delivery-Uncertain 状态机**：超时后自动触发 `pause_all()` $\to$ 读取 6 轴真实状态快照 $\to$ 严格校验非 stale/无故障/合法包络 $\to$ 物理重锚基准 $\to$ 矢量斜率恢复。
* **[工程经验]**：半双工总线必须显式处理“已发出未确认”的悬空状态，并在严格安全校验下允许物理重锚。

---

### 5. UART 批次交错治理与 Batch Atomicity 落地 (J5-C1)
* **[现象/问题]**：在多线程或带轮询的架构中，Worker 线程的 0x29 写指令与驱动节点的读指令可能在总线上相互穿插，导致 6 轴扫描时间跨度严重拉长。
* **[证据/测试]**：跨度分析中偶现非同一批次的读响应插入，导致一组 6 轴反馈跨越了两个控制周期。
* **[修复方案]**：在 `six_motor_controller.py` 中引入 6 轴批次全局互斥锁（`with self._bus_lock_ctx:`），并在 Active 控制模式下彻底抑制独立 Poll 轮询。
* **[工程经验]**：半双工总线上的多轴批次通信必须在主机端实现**批次级全局互斥**，回归实证达到 392/392 零重叠。

---

### 6. Eager Import 导致的 `pus6_msgs` 依赖断裂问题
* **[现象/问题]**：在未 source 工作空间环境的新终端中运行独立 J3 脚本时，报 `ModuleNotFoundError: No module named 'pus6_msgs'`。
* **[证据/测试]**：J3 脚本明明只依赖纯数学逆解，却因导入 `pus6_kinematics` 而失败。
* **[根因边界]**：`pus6_kinematics/__init__.py` 早期采用了 eager import 顶层导入了 `Pus6FkNode`，而该节点引用了 ROS 2 生成的消息包 `pus6_msgs`。
* **[修复方案]**：将 `__init__.py` 改为**惰性导入 (Lazy Import)**，仅在显式请求 `Pus6FkNode` 时加载；并在 J3 独立脚本中加入 `sys.path` 自治引导。
* **[工程经验]**：核心算法包与底层运维工具应当与高层 ROS 生成消息解耦，避免过早依赖绑定。

---

### 7. `rclpy` 多线程执行器回调饥饿与单线程执行器收敛 (J5-C2-RT-SW)
* **[现象/问题]**：200Hz 定时器在 `MultiThreadedExecutor(4)` 下实际仅触发约 45Hz，周期 P95/P99 出现百毫秒级甚至近秒的严重调度长尾。
* **[证据/测试]**：A~F 分层消融实验清晰证明：单次 FK 计算仅需 1.07ms，锁与发布耗时微秒级；但多线程下 C~E 层均发生严重调度停顿。
* **[根因边界]**：Python `rclpy` 的多线程执行器在高频定时器与多路发布/订阅并发就绪时，其 Ready 队列调度机制导致高频定时器长时间得不到调度机会（Callback Starvation）。
* **[修复方案]**：`Pus6FkNode.main()` 明确独占**独立的 `SingleThreadedExecutor`**，将所有回调串行化在确定性调度链上，频率稳定恢复至 200.09Hz (P95 5.14ms)。
* **[工程经验]**：200Hz 以上高频 ROS 2 Python 节点切忌盲目使用多线程执行器，独立单线程执行器往往具有更高、更可预测的实时确定性。

---


---

### 8. J7 50Hz Servo 时序预算与单总线带宽收敛 (J7-C2)
* **[历史现象/问题]**：早期 request-response 路径在 50Hz 周期内进行 6 轴 0x29 伴随问答时，总通信耗时约 18.18~18.39ms，曾超出 20ms 周期预算。
* **[根因边界]**：单总线半双工物理往返延时无法压缩。
* **[当前 Design-B]**：50Hz 0x29 motion write 返回的 `StatusFeedback` 由 `ingest_cycle_feedback()` 接收，作为运行期主反馈；`0x30` round-robin/full scan 仅在显式 safety、diagnostic 或只读窗口执行。
* **[工程经验]**：高频多轴控制必须区分 0x29 伴随反馈与独立 0x30 polling 的时序预算和数据来源。

---

### 9. 运行期 0x25 模式回读隐患消除 (J7-C4)
* **[现象/问题]**：运行期若频繁插入 0x25 读请求，会增加额外的 UART/RX residual 交互并引入错误 mode 门禁风险。
* **[修复方案]**：启动时通过写 0x25=1 的 ACK 确认进入模式，运行期彻底取消 0x25 读请求。
* **[工程经验]**：高频实时循环中严禁混入非必要的低频状态回读。

## 11. 测试驱动开发 (TDD) 与回归测试体系 (Testing Architecture)

### 11.1 分层测试体系与验证目标

```
                                  6PUS 分层测试验证体系

  ┌─────────────────────────┐
  │ Real-Time Benchmarks    │ ── A~F 分层消融性能测试 (验证 200Hz 周期 P95/P99 与执行器调度)
  └─────────────────────────┘
  ┌─────────────────────────┐
  │ Read-Only Audits        │ ── J2-A 实机只读审计 (验证 0 硬件写入、0 故障、真实 UART 遥测)
  └─────────────────────────┘
  ┌─────────────────────────┐
  │ Replay / Synthetic Test │ ── run_j5_c2_hw_acceptance.py (离线回放数据流，验证 FK 收敛与门禁)
  └─────────────────────────┘
  ┌─────────────────────────┐
  │ Mock Integration Tests  │ ── FakeBus 多电机仿真 (验证 5 级门禁、Slew 自愈、maintenance 模式)
  └─────────────────────────┘
  ┌─────────────────────────┐
  │ Unit Tests (数学/协议)   │ ── pytest 纯代数测试 (验证 CRC、解析逆解、解析雅可比 LM、YAML 配置)
  └─────────────────────────┘
```

1. **单元测试 (Unit Tests)**：验证纯数学公式与协议编解码。包括 CRC 校验、闭式解析逆解准确性、解析雅可比 LM 正解收敛性、三阶滤波器 $C^2$ 连续性以及 `6pus_geometry.yaml` 的数据类型与键值合法性。
2. **模拟集成测试 (Mock Integration Tests)**：通过 `FakeBus` 仿真 6 轴电机硬件交互，验证驱动层 5 级硬门禁、Latest-Value-Wins 覆盖守恒、Slew Recovery 动态缩放、Delivery-Uncertain 自愈状态机以及 `maintenance_mode` 授权隔离。
3. **离线回放与合成验收 (Replay / Synthetic Tests)**：通过专用验收工具 [`bone_ws/scripts/run_j5_c2_hw_acceptance.py`](file:///home/lyapunov/bone_drill/bone_ws/scripts/run_j5_c2_hw_acceptance.py)，消费纯合成或历史录制的 `TimedJointFeedback` 数据流，在不打开串口、不产生硬件运动的前提下，自动化验证 Fixed-Lag 零外推插值率、LM 数值正解收敛率、$\text{SO}(3)$ 测地线跳变拦截率并输出 JSON/CSV 报告。
4. **实机只读审计 (Read-Only Audits)**：运行 J2-A 验证脚本，物理连接串口进行长周期只读监听，严格断言 `total hardware writes = 0` 且 `fault_codes = 0`，确保通信链路零侵入。
5. **实时基准测试 (Real-Time Benchmarks)**：运行 A~F 消融基准脚本，测量 200Hz 定时器在真实操作系统下的周期抖动分布、P50/P95/P99 延迟及回调耗时。

---

### 11.2 测试数量与语境严格规范说明

> [!IMPORTANT]
> **测试数量与收集范围语境规范**：在提及测试通过数量时，必须明确具体的测试命令与收集范围，严禁混用或制造虚假断言：
> * **ROS 2 工作空间核心功能包全量测试**：执行 `pytest /home/lyapunov/bone_drill/bone_ws/src -v` $\to$ **304 项 PASS**（耗时约 43.75s，覆盖逆解、正解、LM、FixedLag、驱动协议、多电机控制器、安全前检与控制整形全部当前功能包）；
> * **根目录全仓库历史用例收集**：执行 `pytest /home/lyapunov/bone_drill --collect-only` $\to$ **468 collected**（包含 `bone_ws/src` 162 项 + `up/motor/uart_multi` 306 项历史独立硬件台架用例）。

---

### 11.3 软件数值测试 vs 机械精度的红线认知
* **严格认知界限**：所有软件单元测试与仿真回放测试（包括 IK↔FK 1000 样本平移均值残差 $5.83\times 10^{-9}\text{ mm}$、姿态均值残差 $1.21\times 10^{-7\circ}$）**仅证明正逆解在纯数学模型与代码实现上的自洽无 bug**；
* **严禁过度宣称**：上述数值指标**绝不代表真实物理机器人的加工制造精度、球铰装配间隙或末端定位精度**。真实机械精度必须等待机构总装完成后经由高精度光学跟踪仪实测标定。

---

## 12. 二次开发速查指南、实战练习与装配读点 (Developer Guide, Exercises & Next Steps)

### 12.1 权威修改导航表 (Single Source of Truth)

在系统二次开发或参数调优时，必须严格通过权威配置文件进行修改，严禁直接修改底层代码中的硬编码默认值：

| 开发需求 / 改动场景 | 唯一权威修改文件路径 | 关键配置字段 / 修改对象 |
| :--- | :--- | :--- |
| **调整 6PUS 机构几何尺寸** (静/动平台半径、连杆长、安装角) | [`pus6_kinematics/config/6pus_geometry.yaml`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_kinematics/config/6pus_geometry.yaml) | `geometry.base_radius_m`, `strut_length_m`, `lambda_rad` 等 |
| **修改行程限位与安全包络** (绝对范围、软件限位、正常运动包络) | [`pus6_kinematics/config/6pus_geometry.yaml`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_kinematics/config/6pus_geometry.yaml) | `actuator.normal_motion_limits_steps` ([200, 1800]), `absolute_raw_limits_steps` |
| **修改步数比例与零位定义** | [`pus6_kinematics/config/6pus_geometry.yaml`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_kinematics/config/6pus_geometry.yaml) | `steps_per_meter` (40000.0), `operating_home_raw_steps` ([1000]*6) |
| **修改连续控制速度/加速度上限** | [`pus6_bringup/config/system_params.yaml`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_bringup/config/system_params.yaml) | `max_speed_m_per_sec`, `max_accel_m_per_sec2`, `max_jerk_m_per_sec3` |
| **添加外部闭环算法 Hook** | [`pus6_control/joint_trajectory_controller.py`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_control/pus6_control/joint_trajectory_controller.py) | 继承并重写 `OuterLoopControllerHook.compute_correction` 方法 |
| **修改串口设备路径与波特率** | [`pus6_motor_driver/config/motors.yaml`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/config/motors.yaml) | `port` (`/dev/serial/by-id/...`), `baudrate` (921600) |

---

### 12.2 跨阶段状态索引与冻结边界

| 阶段代号 | 当前状态 | 阶段边界与冻结结论 |
| :--- | :---: | :--- |
| **J1 ~ J4** | **PASS / FROZEN** | 单总线 921600 驱动协议、解析逆解、三阶 Jerk 轨迹整形与安全自愈状态机已冻结 |
| **J5** | **PASS / FROZEN** | 纳秒统一时间轴 (Host `CLOCK_MONOTONIC`)、Fixed-Lag 零外推状态重构、解析雅可比 LM 正解与双 H30 微秒同步已冻结 |
| **J6** | **PASS / FROZEN** | 总装调试与定位模式 (mode=0, 10Hz, speed=50) HOME/PARK 基线已冻结，作为可靠 fallback |
| **J8 / J9 (当前已冻结/完成)** | **PASS / COMPLETE** | **六轴 50Hz Servo 基线已冻结，J9 IMU 集成与测量资格已完成**（J8 运行基线 + J9 收口文档） |
| **J10** | **CLOSED / FROZEN** | HOME-centered、single-DOF axis-wise operational workspace 已完成；耦合 6D workspace 不属于 J10 |
| **J11** | **IN PROGRESS** | PID 3DOF World-Attitude Stabilization / Anti-Vibration；物理执行器与运动学参数标定已暂缓延后（DEFERRED） |

---

### 12.3 基于本项目的 ROS 2 进阶实战练习
1. **练习 1：编写独立笛卡尔空间正弦发布节点 (Python)**：编写一个独立的 ROS 2 节点，以 50Hz 发布 Z 轴正弦位姿 $z(t) = z_{\text{nom}} + 0.003 \sin(2\pi \cdot 0.2 t)$ 到 `/pus6/target_cartesian_pose`，通过 RViz 观察运动学与控制节点的响应；
2. **练习 2：实现双 IMU 外环姿态误差补偿 Hook**：在 `joint_trajectory_controller.py` 中继承 `OuterLoopControllerHook`，根据动平台与基座 IMU 的相对姿态误差，计算关节空间的补偿增量 $\Delta q$；
3. **练习 3：编写 Pytest 单元测试覆盖通信长尾与斜率自愈**：模拟注入 350ms 的串口通信中断，断言 `Pus6MotorDriverNode` 能够通过 Time-Aware Vector Slew Recovery 动态缩放矢量并平滑追上目标；
4. **练习 4：添加 RViz 实时位姿对比插件**：编写 RViz 插件或独立绘图脚本，同时订阅规划目标 `/pus6/target_cartesian_pose` 与正解观测 `/pus6/fk/pose`，实时渲染末端跟踪曲线与残差。

---

### 12.4 物理整机总装与实机运动准入总结 (Physical Assembly & Hardware Baseline Qualification)

回顾 J1~J10 的演进路径，系统已完整走通从底层协议到多自由度物理运动的全部阶段：
1. **机械整机总装 (Stage J6)**：总装调试顺利完成，确立了 `OPERATING_HOME = [1000]*6` 与 `STORAGE_PARK = [0]*6` 的集约化状态分类与安全回零契约。
2. **底层 50Hz 伺服驱动 (Stage J7)**：攻克单总线半双工 RS485 50Hz 高频连续伺服驱动，确立了 Design-B 伴随反馈机制，消除了 0x30 阻塞轮询。
3. **连续笛卡尔实机运动验收 (Stage J8)**：通过 137 秒连续笛卡尔 9 阶段实机视频演示考核（100.0% 反馈完整率，0 fault，0 timeout），正式确立并冻结了物理运行基线 `v1.0-j8-active-servo-hardware-qualified`。
4. **双 IMU 惯导感知集成 (Stage J9)**：实现双 Yesense H30 惯导微秒级采样同步与 $\text{SO}(3)$ 相对姿态解算。
5. **任务空间安全运行包络 (Stage J10)**：在真实物理样机上完成了六自由度单轴安全运行包络边界全域测定，确立了单轴极限与端点安全规则。

以上 J1~J10 的成果共同构成了本项目**第一部分：物理实机运动与感知基础 (Part I — Real-Hardware Motion Foundation)**。

---

## 13. 第二部分：主动姿态稳定与防抖前沿研究路线 (Part II — Active Stabilization Research Roadmap, J11+)

在 J1~J10 奠定的高带宽、高确定性物理运动与感知基线之上，项目正式迈入面向动基座扰动环境的**主动姿态防抖与振动抑制前沿研究**。

### 13.1 J11 当前研发目标：PID 世界坐标系姿态防抖闭环
- **核心任务**：基于动平台与基座双 H30 IMU 实时估计的相对姿态 $\mathbf{R}_{BP} \in \text{SO}(3)$，在上层控制器中引入姿态反馈误差闭环，在动基座发生微小俯仰/横滚低频晃动时，驱动 6PUS 机构在反向实时补偿，使末端在世界坐标系下保持姿态恒定。
- **当前状态**：**IN PROGRESS**（离线逆解仿真、Dry-Run 架构接口与调度测试已通过；真实硬件闭环测试正在按步骤安全推进）。
- **约束准则**：严格保持 J7/J8 底层驱动与运动学核心为只读黑盒；物理参数在线重标定暂缓（DEFERRED）。

### 13.2 后续前沿算法演进路线图 (Future Research Roadmap)
1. **自抗扰控制 (ADRC) 与扩张状态观测器 (ESO)**：
   - 将机械结构装配间隙、电缸死区、导轨摩擦力以及未建模外扰统一视为“总扰动”；
   - 构建二阶/三阶 ESO 实时估计扰动加速度并在前馈端瞬间消除，突破传统 PID 积分滞后局限。
2. **IMU 惯导角加速度前馈与扰动观测器 (DOB)**：
   - 利用基座 IMU 的微秒级高频角速度微分信号，前馈补偿高频扰动能量，提升防抖带宽至 10~20 Hz。
3. **基于强化学习与模型预测控制 (MPC) 的自适应刚度自调优**：
   - 针对手术钻孔过程中不同骨质阻抗与切削反力，在线自适应微调末端顺应度与轨迹刚度。

> [!NOTE]
> 第二部分内容属于前沿学术研究与算法探索路线。本教程与代码库坚守“诚实与证据一致性”原则，在未完成真实物理平台闭环验证并归档 Canonical Evidence 之前，绝不宣称未实现的控制指标。
