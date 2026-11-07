> 历史归档：原路径 `up/motor/uart_multi/ID4-OFFSET固定零偏表征方案.md`。正文保留当时的方案、参数及证据语境；当前原生 Mode 3 入口以[项目 README](../../../README.md)为准。历史命令不作为当前推荐启动命令。

# ID4-OFFSET 固定零偏表征方案

状态：`COMPLETE / PASS`

实机已执行（日志：`logs/multi_motor/id4_offset/20260817_151126_id4_offset.log`），判定为 `PASS / ORDINARY_0_TO_3_STEP_RESIDUAL_SUPPORTED`（四点误差 `[-1, -1, -2, -1]`）。实机证实 ID4 不存在固定 -2 step 零参考偏移，严禁执行 `+2 step` 或任何形式的位置补偿；ID4 进入正位置区间后表现正常。本方案已执行完毕，历史保留。

## 1. 固定边界

- 仅连接并固定 ID4，正方向 `0~40 step` 区域必须无机械干涉，硬件急停可用；
- 固定目标：`10/20/30/40 step`；固定速度：`50 step/s`；每点固定观察 `2.0s`；电流上限保持 `500mA`；
- `protocol.py`、`uart_bus.py`、`motor.py` 不修改；
- 原始 `0x29 target_position`、`0x2A actual_position` 均以 `raw_u16/raw_hex/signed` 记录，不 clamp；
- 所有写帧 destination 必须为 4，不允许广播、其他 ID、ID/Flash/零点/校准/补偿写入；
- 正常或异常结束只向 ID4 发送一次 pause，并在 UART 关闭前完成只读停止确认；
- M4-B1 不重跑，M4-B2/M4-C/M6 继续锁定。

## 2. raw=-2 独立受限入口

该入口不修改全局 `[0,2000]` 门禁，仅存在于 `tools/safe_id4_offset_characterization.py`：

1. 读取并要求硬件行程上下限严格为 `[0,2000]`；
2. 连续 100 轮读取寄存器 target、寄存器 actual 和状态帧；每轮四个位置字段必须全部精确为 `0xFFFE/-2`，同时 `current=0mA, fault=0`；
3. 任意不一致均在零写入状态下拒绝；
4. 人工输入固定确认文本后，仅允许一个原子写帧到 ID4：`0x25=[mode=2, voltage=0, force=0, speed=50, target=10]`；该帧是从 raw=-2 启动的唯一例外；
5. 过渡期只允许 actual 位于 `[-2, commanded_target]`，不得向负方向越过 -2，不得越过当前目标；
6. actual 首次进入 `[0,2000]` 后立即恢复不变的全局门禁，后续不再接受负位置；
7. 首点未确认正向运动并进入合法区间、目标读回不等于 10，或出现任何非零 fault、超过 500mA、通信/地址异常，立即 pause 并停止。

## 3. 多点流程与日志

每个目标点按升序独立执行：

1. 写入目标；首点使用上述原子入口，后续点仅写 `0x29`；
2. pause 前验证 `readback_target_position == commanded_target`；
3. 在固定 2 秒窗口内记录状态，确认正方向运动，无负向异常和超目标；
4. pause 前读取原始 `0x29/0x2A` 和状态帧，记录：
   `commanded_target, readback_target_position, actual_position, error=actual-commanded, current, force, fault`；
5. 单次 pause，记录完整 `pause_response`；
6. 进行 0.5 秒只读停止确认，要求 current 全为 0、fault 全为 0、位置范围不超过 1 step 且尾部稳定或仅有双向量化抖动；
7. pause 后再次记录原始 target/actual 与状态 target/actual。pause 后 target 同步到停止 actual 是既知设备语义，仅记录，不与原 commanded target 比较；
8. 单点 pause 前误差不在 `[-3,0] step` 时判 FAIL，阻止后续点。

日志的关键行固定为 `ENTRY_SAMPLE`、`TARGET_WRITE_READBACK`、`POINT_PAUSE_BEFORE`、`PAUSE_RESPONSE`、`POINT_PAUSE_AFTER`、`STOP_CONFIRMATION`、`ID4_OFFSET_RESULT` 和 `ACCESS_SUMMARY`。日志使用独立时间戳文件且不覆盖。

## 4. 判定标准

完成四点、每点地址/电流/fault/pause/停止确认均安全，且 pause 前读回目标等于命令目标后，再按四个 pause 前误差分类：

- `PASS / FIXED_MINUS2_ZERO_REFERENCE_OFFSET_SUPPORTED`：四点误差严格为 `[-2,-2,-2,-2]`；支持固定 `-2 step` 零参考偏移假设，但仍禁止直接补偿；
- `PASS / ZERO_ENDPOINT_ONLY_SUPPORTED`：四点误差严格为 `[0,0,0,0]`；支持 `-2` 仅为零端状态；
- `PASS / ORDINARY_0_TO_3_STEP_RESIDUAL_SUPPORTED`：四点误差均在 `[-3,0]` 且至少有两个不同误差；不支持固定 `-2`，更符合既知普通到位残差；
- `INCONCLUSIVE / CONSTANT_NON_MINUS2_ERROR`：四点均为相同的 `-1` 或 `-3`，不足以归入上述三类；
- `INCONCLUSIVE / INCOMPLETE_POINTS`：通信等非安全数据原因导致点数不足，不能判断偏移规律；
- `FAIL`：目标读回不符、误差超出 `[-3,0]`、负向运动、超目标、fault 非零、电流超过 500mA、写地址/顺序违规、pause 失败或停止确认失败。

任何分类都不授权位置补偿、clamp 或 M4 恢复；实机结果须先记录并重新审核。

## 5. 无硬件测试

- 专项：`pytest -q test_safe_id4_offset_characterization.py`；
- 覆盖 raw=-2 严格入口、非 -2 零写入拒绝、ID4-only/无广播写审计、固定写序列、pause 后 target 同步语义、三类 PASS、FAIL 和 INCONCLUSIVE 分类，以及不可从 CLI 扩大参数。

## 6. 唯一实机命令（仅准备，未执行）

```bash
cd /home/lyapunov/bone_drill/up/motor/uart_multi

python3 tools/safe_id4_offset_characterization.py \
  --port /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
  --timeout 0.05 \
  --retries 2 \
  --log "logs/multi_motor/id4_offset/$(date +%Y%m%d_%H%M%S)_id4_offset.log"
```

执行仍需重新人工批准；本次仅完成准备，不执行该命令。
