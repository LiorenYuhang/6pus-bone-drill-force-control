> 历史归档：原路径 `docs/整体学习参考/6PUS_SERVO_OPERATION_GUIDE.md`。正文保留当时的方案、参数及证据语境；当前原生 Mode 3 入口以[项目 README](../../../../README.md)为准。历史命令不作为当前推荐启动命令。

# 6PUS 并联机器人日常实机操作指南 (Servo Operation Guide)

> **权威性声明**：本文档为 **6PUS 并联骨科手术机器人平台当前唯一日常实机操作指南**。  
> 基于已通过实机验证并冻结的 **J8 连续笛卡尔伺服与伴随反馈硬件准入基线**（`v1.0-j8-active-servo-hardware-qualified`）编写，面向实验人员日常开机、HOME 初始化、实验后受控 PARK 收拢与应急安全停机。

---

## 1. 新终端标准初始化

所有实机操作前，必须在新终端中执行环境初始化：

```bash
source /opt/ros/jazzy/setup.bash
source /home/lyapunov/bone_drill/bone_ws/install/setup.bash
cd /home/lyapunov/bone_drill
```

---

## 2. 标准实验全流程概览

```text
POWER ON (开启 24V 动力电源)
   ↓
READ_ONLY PREFLIGHT (0 写入状态与暂定 PARK 测量带 [-5, 5] 步检测)
   ↓
J7/J8 SERVO HOME (执行一键开机就位，规划种子 [0]*6，指令 >= 0)
   ↓
OPERATING_HOME (处于工作零位 [1000]*6, z_nom ≈ 132.61mm)
   ↓
ROS 2 / Cartesian / Anti-Vibration Operation (开展 50Hz 笛卡尔运动与控制实验)
   ↓
Experiment Finish (实验完成)
   ↓
J7/J8 SERVO PARK (执行一键受控收拢，目标位 [0]*6)
   ↓
STORAGE_PARK (处于停放收拢位，读数落入暂定 PARK 测量带 [-5, 5] 步)
   ↓
0x1A STOP (抱闸停机确认)
   ↓
POWER OFF (关闭 24V 动力电源)
```

---

## 3. 开机就位：HOME 初始化

在 24V 动力电源开启后，执行生产级伺服 HOME 就位命令：

```bash
python3 /home/lyapunov/bone_drill/bone_ws/scripts/run_j7_servo_home_park.py \
  --action home \
  --speed 100 \
  --rate 50 \
  --real-hardware \
  --confirm-motion
```

* **执行机理**：
  ```text
  1. 0x30 多样本预检读取六轴当前实际位置 (q_current)
  2. 集中分类器判定状态 (VALID_PARK: 全轴位于 [-5, 5] 步且稳定)
  3. 确立安全规划种子 [0, 0, 0, 0, 0, 0]，保证第 0 周期指令严格 >= 0
  4. 配置 Servo 模式并验证 ACK
  5. 50Hz 纯写连续目标流 (20ms 周期，五次多项式平滑 S 曲线)
  6. 到达 OPERATING_HOME ([1000]*6, 行程中位伸出 50.0mm, z_nom ≈ 132.61mm)
  7. 驻留保持 2.0s 并执行最终全状态校验
  8. 0x1A 干净停机，退出程序
  ```

---

## 4. 正常实验与连续笛卡尔控制

* 当 HOME 命令成功退出且终端报告 `Result: PASSED` 后，机构已处于绝对工作基准原点 `OPERATING_HOME`；
* 此时可按实验需求启动上位规划、逆解控制节点或演示脚本：
  ```bash
  # 运行完整的 9 阶段实机展示轨迹：
  python3 /home/lyapunov/bone_drill/bone_ws/scripts/run_j8_final_video_demonstration.py
  ```

---

## 5. 实验结束：PARK 安全收拢

实验结束或准备关机维护前，必须执行生产级伺服 PARK 收拢命令：

```bash
python3 /home/lyapunov/bone_drill/bone_ws/scripts/run_j7_servo_home_park.py \
  --action park \
  --speed 100 \
  --rate 50 \
  --real-hardware \
  --confirm-motion
```

* **执行机理**：
  ```text
  1. 0x30 多样本预检读取六轴当前实际位置 (q_current)
  2. 验证 VALID_HOME 状态并设置规划种子 [1000]*6
  3. 50Hz 连续目标流平滑过渡至 STORAGE_PARK ([0]*6, 6 根推杆完全缩回至 25.0mm 机械极限)
  4. 驻留保持 2.0s 并执行最终全状态校验
  5. 0x1A 干净停机，退出程序
  ```
* **关机断电**：终端提示 `Result: PASSED` 且 6 轴均安全停靠在收拢位后，可安全关闭 24V 动力电源。

---

## 6. 应急与故障处置

* 若运行中发生异常，系统两级看门狗将自动触发：
  - **Tier 1 (连续 4 周期缺帧)** $\to$ 自动触发 0x1A 安全停机；
  - **Tier 2 (调度延迟超 263.324ms)** $\to$ 自动触发 0x1A 截止期保护。
* 任何情况下均可直接断开 24V 硬件急停/主电源开关。
