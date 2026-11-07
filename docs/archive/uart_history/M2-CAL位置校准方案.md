> 历史归档：原路径 `up/motor/uart_multi/M2-CAL位置校准方案.md`。正文保留当时的方案、参数及证据语境；当前原生 Mode 3 入口以[项目 README](../../../README.md)为准。历史命令不作为当前推荐启动命令。

# M2-CAL 位置校准验证方案

## 状态

- 阶段：`M2-CAL`
- 状态：`PREPARED / NOT EXECUTED`
- 前置：M2-A、M2-B、M2-C `PASS`；M2-D `FAIL / NOT PASSED`
- M4/M6：`LOCKED / NOT PREPARED`
- 本方案不修改 `protocol.py`、`uart_bus.py`、`motor.py`，不重新执行 M2-D，不写任何补偿量。

## 目标

验证 `target_position` 与 `actual_position` 的系统偏差规律，区分固定位置偏差、反馈量化和 pause 后目标寄存器语义变化。校准只记录偏差，不实施补偿。

固定绝对目标点为：`10 / 20 / 30 / 40`。ID1、ID2 使用同一 `UartBus`，但严格按电机串行执行：完成 ID1 四个点后再执行 ID2 四个点，不做双电机联合运动。

## 固定边界

- 目标点固定为 `10/20/30/40`，不得临时添加、删除或改写目标点。
- 速度固定 `50 step/s`，单点最大监控窗口 `2s`，电流硬限制 `500mA`，仅使用现有 Type-D 速度模式。
- 运动前读取每台设备限位和状态；目标必须在 `[lower_limit, min(upper_limit, 1800)]` 内，且相对初始位置的最大偏移不超过 `40 step`，否则在任何写入前 `ABORTED / NOT PASSED`。
- 每个点只允许现有控制寄存器 `0x25`、目标位置寄存器 `0x29` 和 pause 寄存器 `0x1A`；禁止 ID 写入、Flash 保存、广播写和任何补偿寄存器写入。
- 每个点按“point_before -> setup -> target -> motion poll -> pause_before -> pause -> pause_after/stop confirmation”记录；写操作不自动重试，pause 不自动重试。
- 任一异常时 UART 保持打开，只对当前运动电机发送一次 pause；pause 或停止确认失败必须输出 `HARD STOP REQUIRED`。
- 每次运行使用新的独立日志，禁止覆盖历史日志。

## 工具与审计

工具：`tools/safe_m2cal_position.py`。

- `CalibrationAuditedBus` 在进入冻结 `UartBus` 前解析所有帧。
- 目标 ID 只允许 `1`、`2`；写寄存器只允许 `0x25/0x29/0x1A`；任何其他 ID（包括广播）或寄存器立即拒绝。
- `Motor(1)` 和 `Motor(2)` 共享一个 `UartBus`，每个请求带 `expected_id`，日志输出逐帧 `ACCESS_AUDIT`。
- 结束输出 `ACCESS_SUMMARY`，必须证明 `other_id_write_count=0`、`compensation_write_count=0`。
- 工具只做偏差数据采集，不修改现有控制逻辑。

## 记录格式

每个采样和每个目标点必须包含：

```text
CAL_SAMPLE motor_id=<1|2> phase=<...> target_position=<n> actual_position=<n> error=<signed> current_mA=<n> force_g=<n> fault=0x<nn>
CAL_POINT motor_id=<1|2> target_position=<n> actual_position=<n> error=<signed> current_mA=<n> force_g=<n> fault=0x<nn> pause_success=<bool> stop_confirmation=<bool>
```

同时保留原始 UART TX/RX、`ACCESS_AUDIT`、`CAL_STOP_CONFIRMATION`、两台的 pause 前后状态和最终 `ACCESS_SUMMARY`。

## PASS / FAIL

`M2-CAL PASS` 只表示校准数据和安全链完整，不表示位置精度已通过，也不产生补偿参数。

PASS 必须同时满足：

- ID1、ID2 的四个目标点全部完成，共 8 个 `CAL_POINT`；每点均有 `target_position`、`actual_position`、有符号 `error`、`current`、`force`、`fault`。
- 全部采样无通信、帧解析、位置越界、`fault != 0` 或 `current > 500mA`。
- 每个点的 pause 成功，pause 后只读停止确认通过，并记录 pause 前后状态。
- 所有写帧 destination 仅为对应 ID1/ID2，寄存器仅为 `0x25/0x29/0x1A`；无广播、ID、Flash 或补偿写入。
- 日志独立、完整、不可覆盖，`compensation_write_count=0`。

FAIL 条件：任一点通信或帧错误、fault、超电流、越界、缺失必需字段、错误 destination/寄存器、pause 失败、停止确认失败或出现 `HARD STOP REQUIRED`。任何写入前因人工确认、限位或最大偏移检查失败，记为 `ABORTED / NOT PASSED`。

各点 `error` 的大小和符号是校准输出，不作为本阶段自动补偿依据；即使重复出现 `2 step`，也不得修改底层或安全阈值。

## 无硬件测试

- `test_safe_m2cal_position.py`：`3 passed`。
- 测试覆盖：广播/越权写拒绝、ID1/ID2 四点串行写审计、必需采样字段日志、人工未授权时零写入。
- 无硬件测试不等于实机 PASS；本阶段当前仍为 `PREPARED / NOT EXECUTED`。

## 人工批准

实机执行前必须重新获得明确批准。工具只接受精确确认词：

`M2-CAL MOVE ID1 ID2 TARGETS 10 20 30 40`

未输入确认词、设备未按要求固定、目标范围或急停条件不满足时，禁止任何写入。

## 唯一实机命令（仅准备，未执行）

```bash
cd /home/lyapunov/bone_drill/up/motor/uart_multi

python3 tools/safe_m2cal_position.py \
  --port /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
  --timeout 0.05 \
  --retries 2 \
  --log "logs/multi_motor/m2cal/$(date +%Y%m%d_%H%M%S)_m2cal_position.log"
```

该命令只准备一个新的独立日志路径；本轮不得执行。M2-CAL 完成并重新评审前，M4/M6 继续锁定。
