> 历史归档：原路径 `up/motor/uart_multi/ID4上电恢复初始化方案.md`。正文保留当时的方案、参数及证据语境；当前原生 Mode 3 入口以[项目 README](../../../README.md)为准。历史命令不作为当前推荐启动命令。

# ID4 近零端边界恢复与保护方案

状态：`COMPLETE / PASS`

实机已执行（原始日志：`logs/multi_motor/id4_init/20260817_161543_id4_poweron_init.log`，2026-08-17 16:15:43），100/100 采样 target/actual 均稳定在 39 step、current=0mA、fault=0x00，判定为 `PASS / ALREADY_IN_RANGE_NO_ACTION`，`write_count=0`，`global_gate_restored=true`。

## 1. 目的与边界修正

本次实机结果修正此前关于 ID4 状态的认识：
1. **非固定上电默认状态**：ID4 的 raw=-2 并非每次上电必然触发的底层固化状态，而是**近零端特殊边界状态**；
2. **合法区间保持**：实测确认 ID4 在约 39 step 掉电重启后仍读取 target=39、actual=39，保持在合法位置区间；底层掉电保持机制未由官方资料确认，不归因于绝对位置传感器；
3. **禁止补偿与主动触发**：严禁 `+2 step` 或任何形式的位置补偿；**严禁为了触发 -2 而主动将 ID4 移回零端**；
4. **工具定位转换**：`tools/safe_id4_poweron_initialize.py` 从一次性上电流程转为**常态化近零端边界恢复与安全保护工具**；若 ID4 因外力或端点触碰再次出现 raw=-2，可按本方案安全脱困。
5. 全局 `[0,2000]` 门禁、`50 step/s`、`2.0s`、`500mA` 均不修改，`protocol.py`、`uart_bus.py`、`motor.py` 保持冻结。

## 2. 上电状态分类

工具先核对 ID4 行程寄存器必须严格为 `[0,2000]`，再连续 100 轮读取 `0x29 target_position`、`0x2A actual_position` 和状态帧：

1. `RECOVERY_REQUIRED`：100/100 的寄存器 target/actual 与状态 target/actual 均严格为 raw `0xFFFE/-2`，同时 `current=0mA, fault=0`。只有此分类允许一次恢复写入；
2. `ALREADY_IN_RANGE_NO_ACTION`：100/100 的寄存器与状态 target/actual 均在 `[0,2000]`，同时 `current=0mA, fault=0`。直接 `PASS`，`write_count=0`，不得重复运动；
3. 任何混合、抖动越界、位置字段不一致、非零 current 或 fault：`FAIL / NOT INITIALIZED`，在零写入状态下拒绝。

## 3. 唯一受限恢复

仅当分类为 `RECOVERY_REQUIRED` 且人工输入固定确认文本后：

1. 发送唯一运动写帧到 destination=4：`0x25=[mode=2, voltage=0, force=0, speed=50, target=10]`；
2. 立即验证 pause 前 `0x29 readback_target_position == 10`；
3. raw=-2 过渡期 actual 只能位于 `[-2,10]`，不得向负方向越过 -2 或越过 target；
4. actual 首次进入 `>=0` 后立即恢复全局 `[0,2000]` 门禁；
5. 在原有最大 2 秒内确认正向运动，并进入 target 10 的普通残差带 `[7,10]`；
6. 记录 pause 前 raw target/actual、actual/error/current/force/fault，随后只发送一次 destination=4 的 pause；
7. 进行 0.5 秒只读停止确认，要求 current 全为 0、fault 全为 0、位置范围不超过 1 step 且稳定；
8. pause 后 raw target 和 raw actual 必须均在 `[0,2000]`。pause 后 target 同步到停止 actual 仅作既知语义记录，不与 commanded target 强制比较。

完整写序列只能为 `ID4 0x25`、`ID4 0x1A`。任意额外写、错误 destination、广播、超流、fault、超目标、目标读回不符、pause 或停止确认失败均判 `FAIL`；pause/停止确认不可完成时输出 `HARD STOP REQUIRED`。

## 4. PASS 与 M4-B1 解锁条件

- `PASS / ID4_POWERON_RECOVERY_COMPLETE`：严格 raw=-2 入口、受限恢复、目标读回、pause、停止确认和 pause 后双 raw 位置合法全部通过；
- `PASS / ALREADY_IN_RANGE_NO_ACTION`：连续 100 轮已经合法，零写入结束；
- 其他情况均不得执行 M4-B1。

只有初始化 PASS 后，且 ID1~ID4 已按 M4 拓扑连接并保持供电、不对 ID4 再次断电或重新插拔，才允许重新批准 M4-B1 Retry。M4-B1 自身仍须从原有四电机 preflight/baseline 开始，参数和门禁完全不变。初始化后若 ID4 再次掉电，先前授权立即失效，必须重新执行本初始化门禁。首次 M4-B1 的 `ABORTED / NOT PASSED` 历史永久保留，不追溯改判。

## 5. 无硬件测试

`test_safe_id4_poweron_initialize.py` 覆盖：严格 raw=-2 恢复、已合法零写入、入口位置/current/fault 拒绝、target readback mismatch、超目标、超 500mA、ID4-only/无广播写审计、固定参数以及人工拒绝零写入。

## 6. 唯一实机命令（仅准备，未执行）

在 ID1~ID4 最终 M4 接线已经完成、所有电机固定、ID4 正方向 `-2~10 step` 路径安全且急停可用后，保持整套设备供电连续性：

```bash
cd /home/lyapunov/bone_drill/up/motor/uart_multi

python3 tools/safe_id4_poweron_initialize.py \
  --port /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
  --timeout 0.05 \
  --retries 2 \
  --log "logs/multi_motor/id4_init/$(date +%Y%m%d_%H%M%S)_id4_poweron_init.log"
```

该命令仍须重新人工批准；本次未执行实机。
