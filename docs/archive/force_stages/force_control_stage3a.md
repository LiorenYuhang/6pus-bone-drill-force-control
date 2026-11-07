> 历史归档：原路径 `docs/force_control_stage3a.md`。正文保留当时的方案、参数及证据语境；当前原生 Mode 3 入口以[项目 README](../../../README.md)为准。历史命令不作为当前推荐启动命令。

# Stage 3A：电机原生力控执行链（Mode 3 / 0x27 目标力写入）

本阶段完成了从 ROS2 `/pus6/command_axial_forces` 到电机底层寄存器 `0x27` 的完整原生力控软件执行链。
遵循严苛安全边界：**只修改 Force 工程**，Position 和 Torque 工程严格只读未改动；没有连接物理串口，没有接通真实电机硬件电源；`hardware_write_count = 0`。所有验证均在 `FakeBus`、`MockServoBus` 及离线测试套件中完成。

---

## 一、系统架构与数据路径

沿用既有成熟的 Parent / UART Child / Feedback Bridge 三进程架构，UART Child 保持为操作系统中唯一的物理串口所有者：

```
[ROS2 Force Controller Node]
       │  (publishes /pus6/command_axial_forces, msg: AxialForceCommand)
       ▼
[Parent Process (j14_process_driver.py)]
       │  - Subscribes to /pus6/command_axial_forces
       │  - Validates sequence, finite float checks
       │  - Non-blocking update_force(forces_n, publish_ns, seq)
       ▼ (AF_UNIX Socket IPC: state packet with mode='force')
[UART Child Process (j14_process_uart.py)]
       │  - CommandState: latest-value-wins, rejects partial/malformed/jump
       │  - Watchdog: command timeout -> drop to zero-force safe hold
       │  - Slew Rate Limiter: dispatch_force_slew() bounds dF/dt (gf/s) & limits
       │  - Decoupled Safety Gates: dry_run / hardware_write_enabled / is_hardware_qualified
       │  - Unit & Direction Conversion: N -> gf -> signed int16 little-endian
       ▼ (J7SixAxisServoTransport)
[Six-Axis Native Force Transport (j7_six_axis_servo_transport.py)]
       │  - Mode 3 Lifecycle: 0x25=3, 0x26=0, 0x27=initial_force
       │  - 50Hz Batch Write: 0x27 target force (ID1..ID6)
       │  - Fail-safe: Any single-axis write timeout/error -> trips 0x1A=1 Pause to ALL axes
       ▼
[RS485 Bus (FakeBus / MockServoBus)]
```

---

## 二、关键技术实现与模块改动

### 1. 协议与安全常量定义 (`pus6_motor_driver/safety.py`)
- **寄存器地址**:
  - `CONTROL_MODE_ADDR = 0x25` (模式寄存器: 1=伺服位置模式, 3=原生力控模式)
  - `TARGET_FORCE_ADDR = 0x27` (目标受力寄存器, 单位 gf, 有符号 16 位整数)
  - `ACTUAL_FORCE_ADDR = 0x2C` (实测受力寄存器, 单位 gf, 有符号 16 位整数)
  - `FORCE_ADC_ADDR = 0x2D` (受力传感器原始 ADC 读数, 0~4095)
- **物理与工程单位换算**:
  - 标称重力加速度 $g = 9.80665\text{ m/s}^2$，即 $1\text{ gf} = 0.00980665\text{ N}$。
  - 考虑每轴安装方向 $d_i \in \{+1.0, -1.0\}$：
    $$F_{\text{gf}, i} = \frac{F_{\text{N}, i}}{d_i \times 0.00980665}$$
    $$F_{\text{N}, i} = F_{\text{gf}, i} \times d_i \times 0.00980665$$
  - 提供 `encode_signed_i16_to_u16(val)` 与 `decode_u16_to_signed_i16(val)`，实现标准 16 位补码与小端双字节的严格无损编解码。
  - 提供 `validate_force_vector(forces_n, limits_n)`，保证六轴全量性与有限值约束。

### 2. 模拟总线支持 (`pus6_motor_driver/fake_bus.py`)
- `FakeBus`:
  - 维护 `self.target_forces = {mid: 0 for mid in MOTOR_IDS}`。
  - 在 `CMD_WR_REGISTER` 写入 `TARGET_FORCE_ADDR (0x27)` 时，解析有符号 16 位整数并记录目标力。
  - 在 `0x30` / `0x32` 状态回执中正确携带 `force_g = self.target_forces[mid]`。
- `MockServoBus`:
  - 同样记录 `0x27` 写入，并在 `modes` 字典中跟踪各轴 `CONTROL_MODE_ADDR (0x25)` 的模式切换。

### 3. 串口适配器应答判定 (`pus6_motor_driver/j7_real_uart_adapter.py`)
- 扩展 `J7RxOwner._is_motion_write_status()`：
  - 原仅识别 `TARGET_POSITION_ADDR (0x29)`。
  - 现同时识别 `TARGET_POSITION_ADDR (0x29)` 与 `TARGET_FORCE_ADDR (0x27)`。
  - 确保 50Hz 周期内写 0x27 产生的 `0x32` 状态回执被 `take_motion_write_feedbacks()` 正常收割和排空，不会产生残余 RX 污染。

### 4. 六轴传输层 Mode 3 生命周期与周期写入 (`pus6_motor_driver/j7_six_axis_servo_transport.py`)
- `configure_force_mode(initial_force_g=0)`:
  - 依次向 ID1..ID6 下发连续写命令：`0x25=3` (Mode 3)、`0x26=0` (速度/滤波)、`0x27=initial_force_g`。
  - 严格检查各轴响应与 `error_code == 0`。
  - 任一轴配置失败即置 `_delivery_uncertain = True`，立即调用 `stop()` 并抛出 `ServoTransportError`。
- `send_force_cycle(target_forces_g, cycle_id, target_timestamp_ns)`:
  - 检验六轴目标力全量性。
  - 排空前序 RX 残余 (`_prepare_motion_rx_locked()`)。
  - 依序构造 `pack_write_register_cmd(mid, TARGET_FORCE_ADDR, [encoded_i16])`。
  - 逐轴写入总线并记录 TX 时间戳。
  - 调用 `take_motion_write_feedbacks()` 收集六轴 0x32 写入状态回执。
  - **全有全无安全机制 (All-or-Nothing)**：任何一轴发生 `J7AdapterWriteTimeout`、`SerialException` 或协议错误，立即置 `_delivery_uncertain = True`，调用 `self.stop()` 触发 `pause_callback`（即向全部 6 轴广播 `0x1A=1` Pause），并抛出异常，绝不容许个别轴单独维持力输出。

### 5. UART Child 核心力控逻辑与防护 (`pus6_motor_driver/j14_process_uart.py`)
- `CommandState`:
  - 同时支持 `mode='static'`（位置模式，6 个整型 step）与 `mode='force'`（力控模式，6 个有限浮点数 N）。
  - 全量性校验：必须为严格 6 维，拒绝部分轴或包含 NaN/Inf 的数据。
  - `candidate_force(anchor_force, warning, now_ns)`：
    - 检查指令新鲜度：`now_ns - command_ns > timeout_ns`。
    - 若指令超时，**明确退化至 `[0.0] * 6` 安全零力保持**，消除断联或崩溃后的失控风险。
    - 指令正常时仅消费一次，未到新指令时保持上次力值（或零力）。
- `dispatch_force_slew()`:
  - 动态计算周期间隔：`dt = max(0.010, min(0.040, (now_ns - last_write_ns) / 1e9))`。
  - 对目标力实施逐轴一阶变化率限制（Slew Rate Limiter, 默认 500 gf/s）与硬件量程截断（默认 `[-1500, 1500] gf`）。
  - 将 `dt` 上限截断为 0.040s，确保即便在调度突发延迟或恢复瞬间，单周期增量也受到物理保护。
- `ServoCore`:
  - 提供 `enter_force_mode()` 与 `exit_force_mode()` 接口。
  - 严格三级安全门控：
    ```python
    if not is_dry_run and self.config['hardware_mode'] != 'mock':
        if not self.config.get('hardware_write_enabled', False):
            raise RuntimeError("hardware writes blocked: hardware_write_enabled=False")
        if not self.config.get('is_hardware_qualified', False):
            raise RuntimeError("hardware writes blocked: is_hardware_qualified=False")
    ```
  - 负力保护门控：若未显式启用 `allow_negative_forces`，当下发负力时立即阻断并抛出异常。
  - 反馈包扩展：在 `feedback` 字典中添加 `control_mode=3`、`dispatched_forces_gf` 和 `dispatched_forces_n`。
- `ProcessClient`:
  - 扩展 `update_force(forces_n, publish_ns, seq)`。
  - 在 `send_state()` 中根据当前指令类型自动组装 `mode='force'` 的 IPC 状态包发送至 Child。

### 6. ROS2 驱动接入 (`pus6_motor_driver/j14_process_driver.py`)
- 导入 `pus6_msgs.msg.AxialForceCommand`。
- 创建订阅者：
  - 话题：`/pus6/command_axial_forces`
  - 回调：`_handle_command_axial_forces(msg)`
  - 解析 `msg.forces` 与 `msg.seq`，直接通过 `self.client.update_force(...)` 转入 IPC 通道。

---

## 三、修改文件清单

| 文件（相对 `bone_ws/src`） | 修改说明 |
|---|---|
| `pus6_motor_driver/.../safety.py` | 增加 Mode 3/0x27/0x2C/0x2D 常量、16位有符号双字节编解码、N与gf双向换算及六轴力向量校验 |
| `pus6_motor_driver/.../fake_bus.py` | `FakeBus` 与 `MockServoBus` 支持 0x27 目标力写入与 0x30/0x32 状态回执 |
| `pus6_motor_driver/.../j7_real_uart_adapter.py` | `_is_motion_write_status` 识别 0x27 目标力写入回执，确保 50Hz 周期内 RX 正常排空 |
| `pus6_motor_driver/.../j7_six_axis_servo_transport.py` | 新增 `configure_force_mode` 与 `send_force_cycle`，实现六轴全有全无 Pause 联锁 |
| `pus6_motor_driver/.../j14_process_uart.py` | `CommandState` 支持力控模式与超时零力、`dispatch_force_slew` 变化率限制、`ServoCore` 门控与周期执行、`ProcessClient` IPC 转发 |
| `pus6_motor_driver/.../j14_process_driver.py` | 订阅 `/pus6/command_axial_forces` 并转发至 Child 进程 |
| `pus6_motor_driver/test/test_stage3a_force_execution.py` | 新增 Stage 3A 原生力控软件执行链专项单元与集成测试（16 项测试） |

---

## 四、离线测试与验证矩阵

所有测试均在本地虚拟环境中运行，无物理硬件介入。

### 1. Stage 3A 专项测试套件 (`test_stage3a_force_execution.py`)

| 测试用例 | 验证目标 | 结果 |
|---|---|---|
| `test_mode3_entry_six_axes` | 六轴 0x25=3 Mode 3 初始化写入与状态确认 | **PASSED** |
| `test_mode3_entry_failure_aborts_and_pauses` | Mode 3 配置失败立即终止并向全 6 轴触发 0x1A=1 Pause | **PASSED** |
| `test_target_force_positive_write_0x27` | 正向目标力 (+4.903N -> +500gf) 小端编解码与 0x27 写入 | **PASSED** |
| `test_target_force_negative_write_0x27` | 负向目标力 (-4.903N -> -500gf -> 0xFDDC) 二进制补码编码 | **PASSED** |
| `test_target_force_bidirectional_reversal_slew` | 正负力动态突变反向过程中的 Slew Rate 渐进限幅保护 | **PASSED** |
| `test_force_unit_conversion_precision` | 1 gf = 0.00980665 N 双向浮点换算精度与边界值校验 | **PASSED** |
| `test_axis_direction_handling` | 逐轴物理方向向量 (`direction=[1, -1, ...]`) 正负映射处理 | **PASSED** |
| `test_single_axis_write_failure_trips_all_axes_pause[1]` | 轴 1 写入超时中断，立即向全部 6 轴下发 Pause 保护 | **PASSED** |
| `test_single_axis_write_failure_trips_all_axes_pause[3]` | 轴 3 写入超时中断，立即向全部 6 轴下发 Pause 保护 | **PASSED** |
| `test_single_axis_write_failure_trips_all_axes_pause[6]` | 轴 6 写入超时中断，立即向全部 6 轴下发 Pause 保护 | **PASSED** |
| `test_command_timeout_triggers_safe_state` | ROS2 目标力指令超时（>100ms）后自动退化为安全零力保持 | **PASSED** |
| `test_safe_mode3_exit` | 主动退出 Mode 3，安全切换并广播 Pause 停机 | **PASSED** |
| `test_decoupled_safety_flags_guard` | `hardware_write_enabled` / `is_hardware_qualified` 未授权时阻断硬件写入 | **PASSED** |
| `test_disallow_negative_forces_guard` | `allow_negative_forces=False` 时下发负力抛出异常拒绝执行 | **PASSED** |
| `test_parent_child_ipc_force_command_stream` | Parent-Child AF_UNIX 跨进程 IPC 力控指令流端到端通信 | **PASSED** |
| `test_position_mode_compatibility_regression` | 现有伺服位置控制模式 (0x29) 零回归、零破坏 | **PASSED** |

**小计：16 / 16 测试通过。**

### 2. 回归测试验证
- **`pus6_motor_driver/test/test_j14_a1_9_process_uart.py`**: **24 / 24 PASSED**（原位置控制 IPC 与 UART 行为完全兼容）。
- **`pus6_force_control/test`**: **49 / 49 PASSED**（Stage 1 & Stage 2.1 力控算法、Fixed-Lag 时间对齐、PID、雅可比与节点行为完全兼容）。
- **全工作空间受影响模块总计 89 项测试全部通过**。

---

## 五、构建与运行指南

```bash
cd /home/lyapunov/6pus_bone_drill_force_control/bone_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --install-base install/force_stage1
source install/force_stage1/local_setup.bash

# 运行 Stage 3A 专项测试
pytest -v src/pus6_motor_driver/test/test_stage3a_force_execution.py

# 运行原有驱动与力控节点回归测试
pytest -v src/pus6_motor_driver/test/test_j14_a1_9_process_uart.py
pytest -v src/pus6_force_control/test
```

---

## 六、安全边界与 Stage 3B 展望

本阶段仅在软件模拟层完成力控执行链的打通与测试：
1. **严格禁止实机直连**：在进入实机之前，必须保持 `dry_run = True` 或 `hardware_write_enabled = False`。
2. **待实机验证事项 (Stage 3B)**：
   - 实机 Mode 3 真实电机的响应特性、内部死区与启动摩擦力；
   - 负向目标力（拉力）在真实推杆电机驱动板固件中的真实响应情况；
   - 物理传感器零点漂移与重力静态补偿。
