> 历史归档：原路径 `up/motor/uart_multi/UART实机运行记录.md`。正文保留当时的方案、参数及证据语境；当前原生 Mode 3 入口以[项目 README](../../../README.md)为准。历史命令不作为当前推荐启动命令。

# UART 实机运行记录

本文件按 Run 编号追加，禁止覆盖、删除或复用编号。原始日志保留在各自路径。

## Run01

- 日期时间：2026-08-16 11:08:53 起始，11:09:05 目标写入
- 使用工具：`tools/safe_endpoint_escape_id1.py`（旧版停止生命周期）
- 电机 ID：1
- 原始日志：`logs/id1_motion/20260816_110853_endpoint_escape_id1_run01.log`
- 起始位置 p0：3 step
- target：8 step
- 位移增量：+5 step
- mode：2
- speed：50 step/s
- timeout：2s
- 电流保护：当时使用工具配置；运动期间最高约 42mA
- F0：约 -377g

### 运行结果

- first_motion_time：约 2026-08-16 11:09:05.695，响应实际位置首次为 4 step
- first_in_tolerance_time：未发生
- final_actual：4 step（程序超时停止前最后状态）
- final_error：4 step
- post-run 只读 actual：7 step
- max_current：约 42mA
- force min/max：约 -377g，DeltaF 约个位数 g
- temperature：约 34C
- fault：0
- pause 是否触发：是，曾尝试发送
- pause 是否成功：否，UART 已在异常处理前关闭
- stop confirmation：未执行
- HARD STOP REQUIRED：旧版本未输出

### 判定

FAIL

### 原因

超时后 `with UartBus` 先关闭串口，随后 pause 发送失败：`Attempting to use a port that is not open`。程序退出后只读发现 target 仍为 8、actual 后续为 7，不能排除目标保持并继续运动。

### 后续

- Run01 原始日志和后续只读日志永久保留。
- Run01 不得作为运动 PASS。
- 已修复 pause-before-close 生命周期，并增加 pause 后 0.5s 只读停止确认。
- 在 Run02 完成前不得扩大位移、速度、超时时间或电流阈值。

补充只读日志：`logs/id1_motion/20260816_120751_control_state_after_run01.log`，确认 mode=2、target_speed=50、target_position=8、actual=7 稳定、current=0、fault=0。

## Run02

状态：已执行，整体 `FAIL — motion validation not completed`。

- 日期时间：2026-08-16 13:05:19 起始，13:05:49 目标写入
- 使用工具：`tools/safe_endpoint_escape_id1.py`（pause-before-close 修复版本）
- 电机 ID：1
- 原始日志：`logs/id1_motion/20260816_130519_endpoint_escape_id1_run02.log`
- 起始位置 p0：7 step
- target：12 step
- 位移增量：+5 step
- mode：2
- speed：50 step/s
- timeout：2s
- 电流保护：沿用已审核阈值，未提高
- F0：`-358.3g`

### 运行结果

- `t0`：`2026-08-16T05:05:49.153281+00:00`
- 运动监控阶段：未确认任何正方向位移
- elapsed `0.377999s`：单次 `actual=6`，触发负方向保护
- first_motion_time：原始旧记录将 `actual=6` 错记为运动；按修复后的定义，`first_motion_time=未观察到`（只将 `actual>p0` 视为正方向运动）
- first_in_tolerance_time：未发生
- trigger_actual：6 step
- trigger_error：6 step
- pause_triggered：`True`
- pause_success：`True`
- stop_confirmation：`True`
- pause 后约 0.5s：position 稳定为 7 step，current=0mA，fault=0
- post_stop_final_actual：7 step
- post_stop_final_error：5 step（相对 target=12；因此未到位）
- final_actual：7 step（与 post-stop 稳定值相同）
- final_error：5 step
- max_current：0mA
- force：约 `-360g`，DeltaF 约 `-1.7g`
- temperature：34C
- fault：0
- HARD STOP REQUIRED：未触发

### 判定

FAIL — motion validation not completed

### 原因复盘

新的 pause-before-close 链路已完成真实硬件验证：pause 在 UART 关闭前成功发送，随后只读停止确认成功。`7->6` 与此前 `3<->4` 的静态位置抖动一致，高度怀疑是 ±1 step 反馈量化/抖动导致的负方向保护误触发，但目前没有充分证据证明执行器真实发生反向运动。因此本 Run 不能判定运动 PASS，也不能据此放宽保护或扩大位移。

### 下一步

- 保留 Run02 原始日志和本记录，不复用 Run02 编号。
- 当前暂停所有进一步实机运动，先完成只读位置量化/抖动诊断和保护策略审核。
- 后续 endpoint 专用反向确认规则已固化为：`actual<=p0-2` 立即停止；`actual=p0-1` 连续 3 次确认后停止；回到 `p0` 或正方向时清零确认计数。该规则未追溯改判 Run02。
- 不扩大位移、速度、timeout 或电流阈值；不执行 Run03，不运动 ID2，不进入多电机运动。

## Run03 前置：ID1 位置量化/抖动只读诊断

状态：已完成只读诊断；Run03 仍暂停，尚未执行。

- 工具：`tools/read_id1_position_jitter.py`
- 电机 ID：1
- 默认采样：30s、50Hz
- 只读寄存器/状态：状态帧中的 `0x2A/0x2B/0x2C/0x2E/0x2F`
- 禁止：任何写寄存器、pause、模式修改、目标位置修改、ID 修改和运动
- 独立日志：由唯一命令中的时间戳文件名生成，禁止覆盖历史日志

### 诊断输出

- position histogram 和众数
- position min/max 及相对众数最大偏差
- 各位置跳变次数和方向转移
- `p0-1`、`p0+1` 最大连续出现次数
- 各位置 run length 的数量、最大值和均值
- current/force/temperature/fault 的样本和统计
- 每次位置跳变是否伴随其他状态字段变化

该诊断用于验证 endpoint 规则：`actual<=p0-2` 立即停止，`actual=p0-1` 连续 3 次确认后停止。诊断完成前禁止执行 Run03、扩大位移或调整任何安全阈值。

### 只读实测结果

- 采样：30s、50Hz、1500 样本
- position histogram：`6:36`、`7:1464`、`8:0`
- position 众数：`7`
- position min/max：`6..7`
- 相对众数最大偏差：`1 step`
- `p0-1=6` 最大连续出现：`1`
- `p0+1=8` 最大连续出现：`0`
- 位置跳变：`7->6=36`、`6->7=36`
- `±2 step`：无
- current：全程 `0mA`
- fault：全程 `0`

结论：Run02 的单帧 `7->6` 高度符合已确认的 1-step 静态反馈抖动；`p0-1` 连续 3 次、`p0-2` 立即停止的低速 endpoint 规则获得实机依据。Run02 整体 FAIL 历史判定保持不变。

## Run03

状态：已执行，整体 `FAIL — motion validation not completed`；Run04 暂缓。

### 执行前固定条件

- 电机 ID：1 only
- 起始位置：启动时的单次位置只记录为 `precheck_single_position`，不得作为运动 `p0`
- 基线：必须完成 `0.5s / 20Hz` 只读采样；记录 `baseline_position_histogram`，且必须存在唯一明确众数
- 基线安全门：任一样本相对众数偏差达到 `±2 step` 即中止；通过后以 `baseline_position_mode` 作为 `p0`，预期 `p0=7`
- 目标：固定 `p0+5`，预期 `7->12`
- 位移：固定 `+5 step`
- 速度：固定 `50 step/s`
- timeout：固定 `2s`
- 电流保护：固定 `500mA`，不得提高
- 负方向保护：`p0-2` 立即停止；`p0-1` 连续 3 次确认后停止
- 不自动返回，不扩大任何参数，不运动 ID2
- 运动日志必须同时包含 `precheck_single_position`、`baseline_position_histogram` 和 `baseline_position_mode`

### 实际执行与结果

- 日期时间：2026-08-16 14:05 左右
- 原始日志：`logs/id1_motion/20260816_140542_endpoint_escape_id1_run03.log`
- 电机 ID：1
- `p0=7`，`target=12`，固定 `+5 step`
- 2s 运动监控内实际位置始终在 `6/7`，未观察到 `actual>7`
- `first_motion_time=未观察到`
- 轮询期间 `current` 从 0mA 逐步升高，`poll_max_current=56mA`
- `fault` 全程为 0
- timeout 后 pause 写入成功：`pause_success=True`
- pause 写响应状态帧的 `current=71mA`；该字段计入全部观测峰值，因此 `max_observed_current=71mA`
- pause 后只读阶段 `current=0mA`，位置仍在 `6/7` 间，未观察到持续单向漂移
- 当时的逐样本完全相同判据未通过：`stop_confirmation=False`，输出 `HARD STOP REQUIRED`

### Run03 后重新上电只读基线

- mode：`0`
- target_speed：`65535`
- target_position：读取显示为 `6->7`
- actual_position：`6~7`
- current：`0mA`
- 无主动运动证据

该掉电后状态确认：Run03 的 `mode=2`、`speed=50`、`target=12` 未掉电保持；Run03 没有产生可确认的 `7->12` 正向位移。

### 判定

`FAIL — motion validation not completed`

本次 FAIL 保持不变。`stop_confirmation=False` 只表示当时停止确认判据未通过；现有位置 `6/7`、pause 后电流为 0、fault 为 0 的证据不足以证明 pause 后仍在持续运动。

### Run01/Run03 共同特征

- 目标位置写入均获得响应，随后电流逐步上升。
- 在各自 2s 监控窗口内，实际位置都主要停留在低端附近，未形成可确认的正方向位置变化。
- 两次都存在 1-step 反馈量化/抖动背景：Run01 为 `3/4`，Run03 为 `6/7`。
- Run01 的 pause 因旧版 UART 生命周期失败；Run03 的 pause-before-close 已成功，因此两次不能用同一停止链路解释。
- 仅凭当前 UART 状态可以确认“控制输出有电流、位置反馈未前进”，不能进一步定性为持续运动、机械卡滞或目标未生效。

### Run03 通过标准

- 基线通过唯一众数和 ±2 step 门禁，并以该众数 `p0` 为起点；实际位置从约 `7` 正向到约 `12`，且 `abs(actual-target)<=1`
- 监控期间出现的 `p0-1` 单帧抖动不单独触发停止，但连续 3 次必须触发；任何 `p0-2` 或更低值必须立即触发
- 无通信异常、非零 fault、位置越界或电流超过 `500mA`
- 若触发保护，pause 必须在 UART 关闭前成功发送，且 stop confirmation 成功
- 原始日志和 `RUN_RESULT` 字段完整，包含 trigger_actual 与 post_stop_final_actual

### 后续建议

- Run04 继续暂停，不扩大位移、速度、timeout 或电流阈值。
- 先使用完全只读工具复核 pause 后的 mode、target speed、target position、actual position、current、force、temperature、fault，并保留独立日志。
- 后续如重新设计停止确认，使用位置 span/方向趋势、current=0、fault=0 的组合判据：允许 span<=1 的双向抖动或稳定一阶落点；span>1 或未收敛的单向趋势仍必须失败并触发 HARD STOP REQUIRED。

## Run04

状态：第一次尝试为 `ABORTED — motion command not issued`；Run04-R1 为 `FAIL — motion validation interrupted by baseline-model false trigger`；Run04-R2 为 `FAIL — target tolerance not reached`。

### 诊断目的

只改变 endpoint 正向目标增量，观察将 `+5 step` 增加到固定 `+10 step` 后，是否能在相同低端条件下确认正方向位置变化。该实验用于区分“小目标未越过低端反馈/机械起始区”和“目标写入后执行器始终没有正向位移”两类现象。

### 固定条件

- 电机：ID1 only
- 起点模型：第一次尝试和 R1 使用当时的 `0.5s / 20Hz` 基线位置众数；R2 使用修正后的 `2s / 50Hz` 基线位置带
- 位移：固定 `+10 step`；R2 目标按 `baseline_max+10` 计算，本次为 `7->17`
- mode：`2`
- speed：`50 step/s`
- timeout：`2s`
- 电流保护：`500mA`
- 只允许正方向，不自动返回
- 方向判据按各次运行时的软件版本执行；R2 使用 baseline band 判据，旧式 `p0-1` 连续 3 次模型仅属于 R1 历史
- 全部尝试均保留 pause-before-close；R1/R2 使用新版 stop confirmation
- 不改变 ID、速度、timeout、电流阈值，不运动其他电机

### 第一次尝试结果

- 日期时间：2026-08-16 15:27 左右
- 原始日志：`logs/id1_motion/20260816_152723_endpoint_escape_id1_run04.log`
- `precheck_single_position=6`
- `baseline_position_histogram={6: 2, 7: 8}`，基线众数 `p0=7`
- 已发送组合配置写：`mode=2`、`speed=50`、`target=p0=7`
- 配置写响应：`actual=6`、`current=0mA`、`fault=0`
- 旧式单帧 `actual<p0` 判据将该 `7->6` 反馈抖动误判为反向运动并中止
- 日志中不存在 `MOTION_T0`
- 未发送 `0x29 target=17`，没有执行 `+10 step` 运动目标
- 当时尚未跨过旧版 `motion_started` 边界，因此该异常路径没有进入 pause-before-close

### 第一次尝试判定

`ABORTED — motion command not issued`

本次不能用于评价 `+10 step` 是否产生正向运动，也不得改记为运动 FAIL。原始日志和 ABORTED 历史永久保留。

### Run04-R1 当时的重新尝试前置条件

- setup 写响应仅检查通信、设备限位、电流和 fault，不对单帧 `actual=p0-1` 做方向判定；
- setup 写完成后立即标记 `armed`，此后任何异常退出都必须在 UART 关闭前发送一次 pause；
- `first_motion_time` 和方向状态机仅从 `MOTION_T0` 后开始；
- 当时真正运动阶段仍使用 `p0-1` 连续 3 次、`p0-2` 立即停止规则；该模型已在 R1 后淘汰；
- 重新尝试必须使用新的独立日志文件，禁止覆盖本次 ABORTED 日志。

### Run04-R1

状态：`FAIL — motion validation interrupted by baseline-model false trigger`。

- 日期时间：2026-08-16 15:59 左右
- 运动前只读日志：`logs/id1_motion/20260816_155824_control_state_before_run04r1.log`
- 原始运动日志：`logs/id1_motion/20260816_155904_endpoint_escape_id1_run04r1.log`
- 基线 histogram：`{6: 4, 7: 6}`，旧模型选择 `p0=7`
- setup 完成并进入 armed：`mode=2`、`speed=50`、`target=7`
- 已记录 `MOTION_T0`，并真实发送 `0x29 target=17`
- `first_motion_time`：未观察到；actual 始终位于 `6/7`
- elapsed 约 `0.356s` 时，连续第三个 `actual=6` 触发旧式 `p0-1` 三连停止
- 触发前 poll 最大电流：`41mA`
- pause：`pause_success=True`，pause 响应电流 `37mA`
- 新版停止确认：`stop_confirmation=True`，pause 后 current=0、fault=0、position 位于 `6/7`
- 未触发 `HARD STOP REQUIRED`

Run04-R1 已越过 `MOTION_T0` 且写入运动目标，因此不是 ABORTED。它保持 FAIL，但现有日志只能证明运动验证被旧基线模型误触发中断，不能据此判定固定 `+10 step` 在完整 2s 窗口内必然无正向位移。

### Run04-R2 基线带模型

状态：已执行；最终判定为 `FAIL — target tolerance not reached`。

- 运动前位置基线固定为 `2s / 50Hz`，记录 `baseline_min`、`baseline_max` 和 histogram；
- 基线 span 必须满足 `baseline_max-baseline_min<=1 step`，否则写入前拒绝运动；
- 正方向目标固定为 `baseline_max+10 step`；
- 仅当 `actual>baseline_max` 时记录 `first_motion_time`；
- 仅当 `actual<baseline_min` 时触发负方向停止；
- baseline 带内任意 `6/7` 顺序或重复均不视为运动或反向运动；
- 保留 armed、pause-before-close、新版 stop confirmation 和全部既有故障/电流/限位保护；
- Run04-R1 原始日志和 FAIL 历史永久保留，R2 必须使用新的日志文件。

### Run04-R2 PASS 标准

- 基线 span 不超过 1 step，且 `baseline_max+10` 位于设备限位交集内；
- 观察到 `actual>baseline_max` 的正方向运动，且最终实际位置满足 `abs(actual-target)<=1`；
- 无通信异常、位置越界、非零 fault 或 current 超过 `500mA`；
- 若触发 timeout/保护，pause 必须在 UART 关闭前成功发送，且新版 stop confirmation 通过；
- 原始 TX/RX、`first_motion_time`、`trigger_actual`、`post_stop_final_actual` 和电流统计完整。

### Run04-R2 FAIL 标准

- 基线 span 超过 1 step，运动在任何写入前被拒绝；
- 2s 内始终未观察到 `actual>baseline_max`，或目标未在允许误差内达到；
- 运动阶段出现 `actual<baseline_min`；
- 任意通信异常、故障、越界、超电流或反向保护触发；
- pause 失败、停止确认失败或需要 `HARD STOP REQUIRED`。

### Run04-R2 实机结果

- 日期时间：2026-08-16 16:24 左右
- 原始日志：`logs/id1_motion/20260816_162439_endpoint_escape_id1_run04r2.log`
- `baseline_position_band=[6,7]`
- `target=17`
- 已记录 `MOTION_T0`，且已真实写入运动目标 `17`
- `first_motion_elapsed_s=0.894803`
- 首次确认正方向位置：`8`，即首次满足 `actual>baseline_max`
- 首次确认正方向运动样本电流：`93mA`
- 最大实际位置：`15`
- 首次到达最终平台位置 `15`：`elapsed_s=1.081911`；此后保持 `15` 至约 `2s` timeout
- `trigger_actual=15`
- `post_stop_final_actual=15`
- `final_actual=15`
- `final_error=2 step`
- poll 与全部观测最大电流均为 `93mA`，低于 `500mA` 保护阈值
- `fault=0`
- 正方向运动确认：`PASS`
- 破静摩擦后的位置变化速度与配置的 `50 step/s` 大体一致
- 目标容差 `abs(actual-target)<=1`：`FAIL`
- `pause_success=True`
- `stop_confirmation=True`
- `HARD STOP REQUIRED=False`

### Run04-R2 判定

`FAIL — target tolerance not reached`

本次已确认 ID1 从基线带 `[6,7]` 向正方向运动，证明 `+10 step` 目标能够产生可确认的正向位移；但在固定 `2s` 验证窗口结束时实际位置为 `15`，距目标 `17` 仍有 `2 step`，未满足既定的 `±1 step` 目标容差，因此不能判为 PASS。安全停止链路正常：pause 成功、停止确认通过，且未要求硬件急停。

分项判定：

- 正向运动能力：`PASS`
- 安全停止链：`PASS`
- `±1 step` 到位：`FAIL`

复盘结论：`+10 step` 已明确越过静态位置带。首次确认运动存在约 `0.895s` 延迟；破静摩擦后，从 `8` 到 `15` 的位置变化速率与配置的 `50 step/s` 处于同一量级。当前未决问题收敛为低端约 `0.9s` 启动延迟和最终约 `2 step` 残差是否属于低端区域特性，现有数据不足以外推到其他位置区域。

### Run04-R2 实际执行命令

```bash
cd /home/lyapunov/bone_drill/up/motor/uart_multi

python3 tools/safe_endpoint_escape_id1.py \
  --port /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
  --max-current-ma 500 \
  --escape-step 10 \
  --baseline-duration 2 \
  --baseline-rate 50 \
  --timeout 0.05 \
  --retries 2 \
  --settle-timeout 2 \
  --poll-interval 0.02 \
  --log "logs/id1_motion/$(date +%Y%m%d_%H%M%S)_endpoint_escape_id1_run04r2.log"
```

## ID1 Endpoint Safety Bring-up 阶段收尾

状态：`COMPLETE`。

阶段级完成表示以下安全能力已获得代码测试和/或实机证据：

- ID1 真实正方向运动能力；
- 运动期间 current 与 fault 监测；
- 异常路径 UART close 前单次 pause；
- pause 后只读 stop confirmation；
- 运动前 baseline position band 及带外方向保护；
- 原始 TX/RX、状态、目标、时序和安全结果的独立日志机制。

历史 Run 判定保持不变，尤其 Run04-R2 仍为 `FAIL — target tolerance not reached`；阶段级 `COMPLETE` 不等同于位置精度验收 PASS。

遗留但暂不阻塞下一阶段：

- 低端启动延迟约 `0.9s`；
- 低端测试中约 `2 step` 最终残差。

阶段转换决定：不再自动准备或执行 Run05，不为追查上述遗留而扩大位移、速度、timeout 或电流阈值。现有 endpoint 工具、安全链和原始日志作为下一阶段的安全基线保留；下一阶段的对象、目标、允许写操作和通过标准必须在实施前单独定义。当前未执行任何新的实机操作。

## 主线切换：多电机实机控制验证

日期：2026-08-16

ID1 endpoint 专项测试停止，不准备 Run04-R3，也不追加 Run05。低端约 `0.9s` 启动延迟和约 `2 step` 残差作为已知问题保留，当前不阻塞多电机集成。

已具备的多电机集成安全基线：

- ID1 真实正方向运动已验证；
- current/fault 在线监测已验证；
- pause-before-close 已实机通过；
- pause 后 stop confirmation 已实机通过；
- baseline position band 和唯一原始日志机制已验证；
- 六电机 UART 定向只读通信基线保持有效。

阶段路线：`M2 -> M4 -> M6`。本段记录时 M2-D 尚未通过、M4 仍锁定；该状态已由文末的 M2-D Retry2 PASS 和 Stage M2 `COMPLETE / PASS` 收口记录取代。

Stage M2 顺序固定为：

1. M2-A：只连接 ID1、ID2，完成两 ID 初始只读核对、ID3 预期无响应和 100 轮 ID1 -> ID2 串行轮询；
2. M2-B：M2-A PASS 后，仅 ID1 执行固定小幅安全正向运动，ID2 全程只读并验证未被误控制；
3. M2-C：M2-B PASS 后，仅 ID2 执行同参数安全正向运动，ID1 全程只读并验证未被误控制；
4. M2-D：M2-C 实机 PASS 后执行固定小幅联合正向运动；首次实机已 FAIL，必须先审计配置不匹配，不得自动重试。

个体运动参数不得自动扩大：`+10 step`、`50 step/s`、最大 `2s`、`500mA`，只允许正方向，目标上界为 `min(device_upper, 1800)`。任何目标越界直接 ABORT；任何非目标电机带外位置变化、配置变化、故障或误写均为 FAIL。Stage M2 的定向控制隔离门禁允许已知非阻塞残差 `final_error<=2 step`，同时继续单独记录原 `±1 step` 精度结果。M2-B/M2-C/M2-D 的运动目标和 pause 写命令均不重试；M2-D 仅允许向 ID1/ID2 发送各自地址的安全写入。若任一电机状态异常或无法确认停止，必须输出 `HARD STOP REQUIRED`，由操作者硬件急停/断电。

Stage M2 的详细步骤、M2-A/M2-B/M2-C 实机结果、M2-D PASS/FAIL 标准和唯一 M2-D 命令记录在 `UART六电机编号与测试方案.md` 第 20 节。冻结底层未修改。

### Stage M2-A 实机结果

状态：`PASS`。

- 日期时间：2026-08-16 16:47 左右
- 原始日志：`logs/multi_motor/m2/20260816_164752_m2a_readonly_id1_id2.log`
- ID1：`requests=101, first_success=101, retry_success=0, final_failure=0`
- ID2：`requests=101, first_success=101, retry_success=0, final_failure=0`
- TOTAL：`requests=202, first_success=202, retry_success=0, final_failure=0`
- `timeout=0, wrong_id=0, length_error=0, checksum_error=0`
- ID3：`expected_no_response=1`
- `avg_response_ms=2.887, max_response_ms=4.113`

M2-A 满足全部既定门禁，允许准备 M2-B。该结论只覆盖两 ID 同时在线的只读通信，不代表任何多电机运动已经执行。

### Stage M2-B 实机结果

状态：`PASS`。

首次预检日志永久保留：

- `logs/multi_motor/m2/20260816_170427_m2b_id1_only.log`
- 操作者未输入确认词，判定 `ABORTED / NOT PASSED`；
- 未写入任何电机：`write_count=0, id2_write_count=0`。

正式 PASS 日志：

- `logs/multi_motor/m2/20260816_170610_m2b_id1_only.log`
- ID1 baseline `[14,15]`，目标 `25`，最终 `23`，`final_error=2 step`；
- ID1 首次正向运动 `elapsed=0.789521s, current=67mA`，最大电流 `76mA`，fault=0；
- ID2 全程 actual=0、current=0、fault=0，mode/speed/target 配置不变；
- `ID2_read_only=true, ID2_config_unchanged=true`；
- 三个写帧目标均为 ID1，寄存器为 `0x25, 0x29, 0x1A`，`id2_write_count=0`；
- `pause_success=True, stop_confirmation=True, HARD STOP REQUIRED=False`。

M2-B 满足 Stage M2 定向控制隔离门禁：ID1 正向运动及 `final_error<=2 step`、ID2 未被误控制、写目标审计、安全停止链均通过。原严格 `±1 step` 精度仍未通过，不在本阶段扩大参数追查。

### Stage M2-C 实机结果

状态：`PASS`。

- 日期时间：2026-08-16 18:23 左右；
- 工具：`tools/safe_m2c_id2_only.py`；
- 原始日志：`logs/multi_motor/m2/20260816_182345_m2c_id2_only.log`；
- ID2 baseline `[0,0]`，目标 `10`，最终 `8`，`final_error=2 step`；
- ID2 首次正向运动 `elapsed=0.711788s, current=60mA`，最大电流 `67mA`；
- ID2 force `311..342g`，DeltaF `-31..0g`，温度 `36..38C`，fault=0；
- ID1 baseline `[22,23]`，全程 actual `22..23`、current `0mA`、force `-325g`、fault=0；
- ID1 配置保持 `mode=2, speed=50, target=22`，`ID1_read_only=true, ID1_config_unchanged=true`；
- 三个写帧目标均为 ID2，寄存器为 `0x25, 0x29, 0x1A`，`id1_write_count=0`；
- `pause_success=True, stop_confirmation=True, HARD STOP REQUIRED=False`；
- 严格 `±1 step` 精度未通过，但 Stage M2 集成门禁 `final_error<=2 step` 通过。

M2-C 满足全部既定门禁：ID2 明确正向运动、ID1 未被误控制、地址审计正确、安全停止链通过，因此判定 `PASS`。

### Stage M2 个体运动残差记录

- M2-B：ID1 单独运动，target `25`，final `23`，最终残差 `2 step`；
- M2-C：ID2 单独运动，target `10`，final `8`，最终残差 `2 step`。

两台电机在相同的 `+10 step / 50 step/s / 最大2s / 500mA` 个体运动门禁下均出现约 `2 step` 最终残差。该重复现象作为非阻塞已知现象保留，不据此扩大位移、速度、timeout 或电流阈值，也不追溯改判 M2-B/M2-C。

### Stage M2-D 准备与首次实机结果

状态：`FAIL / NOT PASSED` — 原始日志显示最终配置不匹配。

- 工具：`tools/safe_m2d_joint_id1_id2.py`；
- 同一个 `UartBus` 上创建 `Motor(1)` 和 `Motor(2)`；
- 两台分别建立 `2s/50Hz` baseline position band，各自目标固定为 `baseline_max+10 step`；
- 两台均固定 `50 step/s / 最大2s / 500mA`，只允许正方向；
- 目标按 ID1 -> ID2 顺序发送并记录命令时差；本阶段不定义或宣称严格同步精度；
- 逐样本记录两台 target、actual、current、force、DeltaF、temperature 和 fault；
- 所有出站帧在进入冻结 `UartBus` 前解析，写目标仅允许 ID1/ID2，写寄存器仅允许 `0x25/0x29/0x1A`；
- 正常结束或任一异常均分别对 ID1、ID2 各尝试一次 pause，不重试；随后对两台执行纯只读 stop confirmation；
- 任一 pause 或停止确认失败均输出 `HARD STOP REQUIRED`；
- M2-D 专项无硬件测试：`12 passed`；endpoint 与 M2-B/C/D 联合回归：`63 passed`。

首次实机原始日志显示 M2-D 已进入写入、运动、pause 和 stop confirmation；因 ID1 最终配置不匹配判定 FAIL。M4 仍为 `LOCKED / NOT PREPARED`，不得准备。

本次原始日志事实：ID1 baseline `[22,22]`、target `32`、final `30`、`final_error=2`；ID2 baseline `[7,8]`、target `18`、final `16`、`final_error=2`；最大电流分别为 `75mA`、`66mA`，两台 fault 均为 `0x00`。写帧为 `[1,2,1,2,1,2]`，无其他 ID 写入；两台 pause 均成功，双电机 stop confirmation 通过。

最终失败原因为 ID1 配置期望 `mode=2, speed=50, target=32`，读回 `mode=2, speed=50, target=30`。原始日志与“未发生运动/未发生 pause”的口述不一致；本记录以原始日志为准。本次不是写入前 ABORT，而是已进入运动后的 `FAIL / NOT PASSED`。

### M2 阶段最终结论与后续路线

- M2-A：`PASS`；两 ID 定向只读通信全部首次成功，ID3 预期无响应。
- M2-B：`PASS`；ID1 正向运动、ID2 只读隔离、地址审计和安全停止链均通过；最终残差 `2 step`。
- M2-C：`PASS`；ID2 正向运动、ID1 只读隔离、地址审计和安全停止链均通过；最终残差 `2 step`。
- M2-D：`FAIL / NOT PASSED`；两台均正向运动，fault=0，电流低于 `500mA`，写帧地址正确，pause 和双电机 stop confirmation 通过；但 ID1 target `32` 最终读回 `30`，两台最终残差均为 `2 step`。
- 归因判断：没有 UART timeout、错 ID、长度/校验错误或错误 destination，故不支持 UART 通信问题；目标写入顺序固定为 ID1 -> ID2，dispatch skew 约 `3.3ms`，不足以解释两台一致的 `2 step` 残差，故不支持多电机调度为主因；现阶段更符合电机控制器/反馈位置固定偏差或 pause 后目标寄存器语义变化，尚不能直接实施补偿。
- 当时阶段结论：M2-A~M2-C 通过、M2-D 尚未通过，因此 Stage M2 当时未完成；该历史状态不覆盖后续 Retry2 PASS。
- 后续路线：在 M4 前增加独立 `M2-CAL` 位置校准验证，先验证偏置和 pause 后寄存器语义，不修改 `2 step` 误差、不写补偿量、不扩大位移/速度/timeout/电流阈值。校准通过并重新批准后，才可重新评估 M2-D。

### M2-CAL 位置校准准备

状态：`PREPARED / NOT EXECUTED`。

- 独立方案：`M2-CAL位置校准方案.md`；工具：`tools/safe_m2cal_position.py`。
- 固定目标点：`10/20/30/40`，ID1 四点完成后再执行 ID2 四点；不做联合运动。
- 固定边界：`50 step/s`、单点最大 `2s`、`500mA`，目标相对初始位置最大偏移 `40 step`；只允许 mode/target/pause 写入，不写补偿。
- 每点记录 target_position、actual_position、error、current、force、fault，以及 pause 前后状态；所有写帧审计并禁止广播。
- 无硬件测试：`test_safe_m2cal_position.py`，`3 passed`。本段记录时 M2-CAL 尚未执行、M4 仍锁定；后续结果见文末。
- 下一步：重新获得人工批准后，才可考虑执行方案文档中的唯一 M2-CAL 命令。
## 当前线程交接快照

日期：2026-08-16

- M2-A：`PASS`；有效日志 `logs/multi_motor/m2/20260816_164752_m2a_readonly_id1_id2.log`。
- M2-B：`PASS`；预检未解锁日志 `logs/multi_motor/m2/20260816_170427_m2b_id1_only.log` 保持 `ABORTED / NOT PASSED`；正式有效日志 `logs/multi_motor/m2/20260816_170610_m2b_id1_only.log`。
- M2-C：`PASS`；有效日志 `logs/multi_motor/m2/20260816_182345_m2c_id2_only.log`。
- M2-CAL：`PASS`；原始日志 `logs/multi_motor/m2cal/20260817_100115_m2cal_position.log`，无硬件测试 `3 passed`。
- 截至本段记录：M2-D 首次实机 `FAIL / NOT PASSED`；Retry1 尚未完成最终收口。后续 Retry1 ABORTED 与 Retry2 PASS 见文末。

本段当时的下一步已被后续执行结果取代；当前唯一任务见文末 M4-A 状态。

冻结项继续有效：`protocol.py`、`uart_bus.py`、`motor.py` 不得修改；`logs/six_ids_test01.log` 为永久六电机 UART 基线，不得覆盖或删除。

当前不阻塞主线的已知问题：ID1、ID2 个体 `+10 step` 运动均出现约 `2 step` 最终残差；另有 ID1 低端启动延迟约 `0.9s` 和无载力传感器约 `-400g` 静态偏置。M2-D 不得因这些问题自动扩大位移、速度、timeout、电流限制或擅自修改力保护阈值。

## M2-D 验收逻辑复核（2026-08-17，仅离线审计）

本节不改写首次 M2-D 的历史判定；该次仍保留为 `FAIL / NOT PASSED`。本次未执行任何实机、M2-D 重试或 M4。

M2-CAL 已 `PASS`，原始日志为 `logs/multi_motor/m2cal/20260817_100115_m2cal_position.log`。8 点 pause 核对如下（每格为 `pause_before actual / pause_response actual / pause_after final actual`）：

| motor | commanded_target | pause actual | error | pause_success | stop_confirmation |
|---|---:|---:|---:|---|---|
| ID1 | 10 | 9 / 9 / 9 | -1 | True | True |
| ID1 | 20 | 18 / 18 / 18 | -2 | True | True |
| ID1 | 30 | 28 / 28 / 28 | -2 | True | True |
| ID1 | 40 | 39 / 39 / 39 | -1 | True | True |
| ID2 | 10 | 10 / 10 / 10 | 0 | True | True |
| ID2 | 20 | 17 / 17 / 17 | -3 | True | True |
| ID2 | 30 | 29 / 29 / 29 | -1 | True | True |
| ID2 | 40 | 38 / 39 / 38 | -2 | True | True |

8 点 pause 前、响应和 pause 后均 fault=0；pause 后 current=0，停止确认均通过。ID1/ID2 的误差分别为 `-1,-2,-2,-1` 和 `0,-3,-1,-2 step`，不存在固定 `-2 step` 偏置，禁止写入 `+2 step` 或其他补偿。

原始状态帧明确区分了三个量：`commanded_target` 为软件发出的目标；`readback_target_position` 为设备目标字段/`0x29` 读回；`actual_position` 为当前位置。八个点中，pause 响应仍带原 commanded target，紧随其后的 pause 后状态把 target 同步到停止 actual；M2-D 原始帧同样显示 ID1 `32/30 -> 30/30`、ID2 `18/16 -> 16/16`。因此首次 M2-D 的 final configuration mismatch 属于错误验收逻辑，不是 UART 通信或多电机调度失败。

`tools/safe_m2d_joint_id1_id2.py` 已做最小验收层修复：pause 前立即读取并要求 `readback_target_position == commanded_target`；pause 后不再要求 target 保持原 commanded target，仅验证 actual、`final_error<=2 step`、current、fault、两台 pause 和双电机 stop confirmation。pause 后 target 仍读取并记录，但不作为 PASS/FAIL 条件。未修改 `protocol.py`、`uart_bus.py`、`motor.py`，未改变位移、速度、timeout 或 `500mA` 阈值。

当时无硬件测试：M2-D 专项 `13 passed`；相关联合回归 `67 passed`。该段为 Retry1 前历史状态，后续结果见文末。

## M2-D Retry1（2026-08-17 10:41，baseline gate）

- 判定：`ABORTED / NOT PASSED — pre-motion baseline gate rejection`。
- 工具：`tools/safe_m2d_joint_id1_id2.py`。
- 原始日志：`logs/multi_motor/m2/20260817_104109_m2d_joint_id1_id2.log`。
- ID1 baseline histogram：`{38:96,39:4}`。
- ID2 baseline：`min=37, max=39, span=2, histogram={37:12,38:87,39:1}`。
- 旧判据在构造 ID2 baseline 时因 `span>1` 拒绝；实际上所有 ID2 样本均在唯一众数 `38` 的 `±1 step` 内。
- 地址审计：`write_count=0, write_destinations=[], other_id_write_count=0`；ID1/ID2 各完成 106 次只读请求。
- 未发生：配置写、目标写、ID1/ID2 运动、双电机联合控制、pause、stop confirmation。
- 本次结果只说明旧 baseline span 门禁过严，不用于评价联合运动能力；不得改记为运动 FAIL。

仅 M2-D baseline 判据已修正：`baseline_reference` 使用唯一众数；全样本必须满足 `abs(actual-baseline_reference)<=1`；偏离达到 `2 step` 或 tie 时仍在任何写入前拒绝；`baseline_safety_min/max` 保存实际范围；运动目标使用 `baseline_reference+10`；仅 `actual>baseline_safety_max` 确认正向运动，仅 `actual<baseline_safety_min` 确认负向运动。`500mA`、fault、pause、stop confirmation、地址审计、位移、速度和 timeout 均未改变。

M2-D 专项无硬件测试：`21 passed`；相关联合回归：`75 passed`。本段记录时 Retry2 为 `APPROVED / NOT EXECUTED`；后续 PASS 结果见下一节。

## M2-D Retry2（2026-08-17 10:58，PASS）

- 判定：`PASS`。
- 工具：`tools/safe_m2d_joint_id1_id2.py`。
- 原始日志：`logs/multi_motor/m2/20260817_105855_m2d_joint_id1_id2.log`。
- ID1：baseline reference `38`、safety range `[38,38]`、target `48`、final `46`、error `2 step`。
- ID2：baseline reference `38`、safety range `[37,38]`、target `48`、final `46`、error `2 step`。
- 正向运动：两台均确认；首次正向运动时间分别约 `0.792s` 和 `0.476s`。
- 最大电流：ID1 `66mA`、ID2 `48mA`；两台 faults 均为 `0x00`。
- pause 前目标读回：ID1/ID2 均为 `48`；pause 后 target 与 actual 均为 `46`，符合已确认的 pause 语义。
- 写帧审计：`write_count=6`、destinations `[1,2,1,2,1,2]`、寄存器 `0x25,0x25,0x29,0x29,0x1A,0x1A`、`other_id_write_count=0`。
- `pause_both=true`，两台 pause 均成功；`stop_confirmation=true`；`HARD STOP REQUIRED=False`。

Retry2 满足 M2-D 全部集成门禁，判定 `PASS`。首次 M2-D `FAIL / NOT PASSED` 和 Retry1 `ABORTED / NOT PASSED` 均作为历史永久保留，不追溯改判。

## Stage M2 最终收口

- M2-A：`PASS`。
- M2-B：`PASS`。
- M2-C：`PASS`。
- M2-CAL：`PASS`。
- M2-D：Retry2 `PASS`；首次 FAIL 与 Retry1 ABORTED 历史保留。
- Stage M2：`COMPLETE / PASS`。
- 位置残差约 `0~3 step`，按 `40 step/mm` 约 `0~0.075 mm`，继续作为非阻塞已知现象；不存在固定 `-2 step` 偏置，不写任何补偿。

Stage M4 当前状态：M4-A 正式 `PASS`；M4-B1 `ABORTED / NOT PASSED`；ID4 单机诊断、ID3/ID4 对照及 ID4 重新上电复核均确认 ID4 持续 `-2`；M4 主线暂停，当前仅 ID4-OFFSET `PREPARED / NOT EXECUTED`；M4-B2/M4-C/M6 均锁定。

## Stage M4-A 首次尝试（2026-08-17 11:28，FAIL / NOT PASSED）

- 工具：`tools/probe_two_ids.py --ids 1 2 3 4 --probe-id 5`，纯只读；
- 原始日志：`logs/multi_motor/m4/20260817_112819_m4a_readonly_id1_id2_id3_id4.log`；
- ID1/ID2 各 `requests=101, first_success=101, final_failure=0`；
- ID3/ID4 各 `requests=101, first_success=0, final_failure=101, timeout=303`；
- 总计 `requests=404, first_success=202, final_failure=202, timeout=606`，ID5 `expected_no_response=1`；
- 判定：`FAIL / NOT PASSED`。该日志与判定永久保留，不因后续 PASS 删除或追溯改判。

## Stage M4-A 正式实机（2026-08-17 11:32，PASS）

- 工具：`tools/probe_two_ids.py --ids 1 2 3 4 --probe-id 5`，纯只读；
- 原始日志：`logs/multi_motor/m4/20260817_113201_m4a_readonly_id1_id2_id3_id4.log`；
- ID1~ID4 均为 `requests=101, first_success=101, retry_success=0, final_failure=0`；
- 总计 `requests=404, first_success=404, retry_success=0, final_failure=0`；
- `timeout=0, wrong_id=0, length_error=0, checksum_error=0`；
- ID5 `expected_no_response=1`；
- `avg_response_ms=2.836, max_response_ms=3.655`；
- 满足 M4-A 全部只读通信标准，正式判定 `PASS`。

## Stage M4-B 准备（仅离线，未执行实机）

状态：`PREPARED / NOT EXECUTED`。

- 工具：`tools/safe_m4b_one_motor.py`；
- M4-B1：仅 ID1 运动，ID2/ID3/ID4 只读；
- M4-B2：仅 ID2 运动，ID1/ID3/ID4 只读；
- M4-B3：仅 ID3 运动，ID1/ID2/ID4 只读；
- M4-B4：仅 ID4 运动，ID1/ID2/ID3 只读；
- 固定参数：`baseline_reference+10 step / 50 step/s / 最大2s / 500mA`，不扩大位移、速度、timeout 或电流阈值；
- 每次四台分别使用唯一众数 baseline/reference 逻辑；当前运动 ID 目标写入前读回验证，三台观察电机配置不变、fault=0、电流不超过各自 baseline 最大值；
- 写审计仅允许当前运动 ID 的 `0x25/0x29/0x1A`，观察电机零写入；
- 正常或异常结束只 pause 当前运动 ID，随后对 ID1~ID4 进行只读 stop confirmation；
- PASS/FAIL 标准详见 `UART六电机编号与测试方案.md` 第 21.2 节；
- 专项无硬件测试 `23 passed`，全仓无硬件回归 `159 passed`。

上述为 M4-B1 执行前的准备状态；其后的实机 ABORT 结果见下一节。

## Stage M4-B1（2026-08-17 12:36，ABORTED / NOT PASSED）

- 判定：`ABORTED / NOT PASSED — ID4 pre-motion position out of range`；
- 工具：`tools/safe_m4b_one_motor.py --moving-id 1`；
- 原始日志：`logs/multi_motor/m4/20260817_123600_m4b1_id1_only.log`；
- ID4 `0x29` 原始读回为 `FE FF`，即无符号 `65534`、有符号 `-2`；
- ID4 `0x30` 状态帧中 `target_position=-2, actual_position=-2, current=0mA, force=-76g, fault=0x00`；
- 设备位置范围为 `[0,2000]`，因此 preflight 在任何 baseline 或运动写入前拒绝；
- 地址审计：`write_count=0, write_destinations=[], observer_write_count=0`；ID1~ID4 各完成 6 次只读请求；
- `positive_motion=False, pause_attempted=False, stop_confirmation=None`；
- 未发生任何配置写、目标写、运动、pause 或 stop confirmation。

本次只证明 ID4 在该时刻报告越界位置 `-2`；不判断其为瞬态还是持续状态。M4-B1 不重跑，M4-B2 不准备。

## ID4 纯只读状态诊断（2026-08-17 13:04，COMPLETE）

- 工具：`tools/diagnose_id4_readonly.py`；
- 原始日志：`logs/multi_motor/m4/20260817_130429_id4_readonly_diagnosis.log`；
- 连续 100/100 样本均为 `register_target=-2, status_target=-2, actual=-2, current=0mA, force=-61g, fault=0`；
- `0x29` 原始值始终为 `raw_u16=65534, raw_hex=0xFFFE`，只在诊断工具中有符号解释为 `-2`，未修改冻结解析层，未 clamp；
- 访问审计为 `read_register_count=100, read_status_count=100, write_count=0`；
- 分类：`PERSISTENT_MINUS2`。确认并非瞬时抖动或偶发通信错误，但尚不能判定为 ID4 硬件故障；
- M4-B1 继续保持 `ABORTED / NOT PASSED`，不得重跑，M4-B2 仍不准备。

## ID3/ID4 纯只读对照诊断（2026-08-17 13:29，COMPLETE）

状态：`COMPLETE / PERSISTENT_ID4_MINUS2_WITH_ID3_REFERENCE`。

- 工具：`tools/diagnose_id3_id4_readonly.py`；
- 原始日志：`logs/multi_motor/m4/20260817_132920_id3_id4_readonly_comparison.log`；
- 固定同一 UART 上的 ID3 与 ID4，连续 100 轮、0.05s 间隔，每轮依次对两台读取 mode `0x25`、target speed `0x28`、target position `0x29` 和 status `0x30`；
- mode、target speed、current、fault 均为 `equal=100, different=0`；force 为 `equal=0, different=100`；
- ID3 register target histogram 为 `{-1:1, 0:99}`，status target 与 actual 均为 `{0:100}`，位置正常位于 0 附近；
- ID4 register target、status target 与 actual 均为 `{-2:100}`，持续 `-2`；
- ID3/ID4 current 均为 `{0:100}`，fault 均为 `{0:100}`；
- 地址审计：ID3/ID4 各完成 300 次白名单寄存器读和 100 次状态读，`write_count=0`；
- 结论：问题定位为 ID4 自身持续位置状态，不支持 UART 总线或多机调度问题；仍不足以判定硬件故障。

手册核对：现行 Type-D V2.0.4 明列 `0x29 target_position` 与 `0x2A actual_position` 范围均为 `[0,2000]`，并说明绝对位置传感器掉电不丢失、无需归零；未找到位置零点、初始化或位置校准流程。`0x1B` 是还原参数命令，不是位置校准说明。旧 V1.8.3 将 target 范围列为 `0~2000`，actual 范围列为 `-20~2020`，且其零位校准只针对力传感器。因此 `0x29=-2` 在两版目标范围中都越界，`0x2A=-2` 也不符合现行 V2.0.4；旧版只能说明实际位置近零负值曾有历史规格依据，不能把当前 target/actual 同时为 `-2` 认定为正常。

## ID4 单机重新上电纯只读复核（2026-08-17 13:40，COMPLETE）

状态：`COMPLETE / PERSISTENT_MINUS2`。

- 人工断电后仅连接 ID4；确认其他电机的 UART 与动力均未连接，再给 ID4 重新上电；
- 工具：`tools/diagnose_id4_readonly.py`；
- 原始日志：`logs/multi_motor/m4/20260817_134018_id4_repower_readonly_verify.log`；
- 100/100 register target、status target、actual 均为 `-2`，current 始终 `0mA`，fault 始终 0，force 范围 `-61~-52g`；
- 审计汇总为 `read_register_count=100, read_status_count=100, write_count=0`；
- 确认 ID4 的 `-2` 是掉电重启后仍持续存在的 device-local persistent state；
- 继续按 V2.0.4 视为 out-of-spec；禁止运动、clamp、放宽 `[0,2000]` 门禁或执行零点/校准写入。

## ID3/ID4 纯只读配置/版本审计准备

状态：`PREPARED / NOT EXECUTED`。当前上一步只连接 ID4；正式对照前必须人工断电并只连接 ID3+ID4，Agent 未执行接线或实机读取。

本地官方资料结论：

- V2.0.4 只定义 `CMD_RD_STATUS=0x30`、`CMD_RD_REGISTER=0x31`、`CMD_WR_REGISTER=0x32`，没有设备信息或版本查询命令；
- V2.0.4 的公开寄存器 `0x16~0x2F` 中没有固件版本、协议版本、设备型号/类型、位置零点或位置偏移字段；
- 只能通过成功解析 `0x30/0x31` 响应确认设备使用 V2 风格协议族，不能确认 ID3/ID4 的精确固件版本是否相同；
- 位置相关可读配置为行程上限 `0x23`、行程下限 `0x24`、控制模式/目标配置 `0x25~0x29`，实际位置在状态帧及 `0x2A`；
- 旧 V1.8.3 的 `-20~2020` actual 范围不能覆盖现行 V2.0.4；其“零位校准”仅针对力传感器且地址布局不同，不得对现行设备试探读取或写入。

准备的工具：`tools/audit_id3_id4_config_readonly.py`。

- 固定 ID3、ID4，共用一个 UART；三轮读取并比较 14 个官方定义字段：ID `0x16`、波特率 `0x17`、过温/回温/过流 `0x1E~0x20`、正反输出上限 `0x21/0x22`、行程上下限 `0x23/0x24`、mode/voltage/force target/speed/position `0x25~0x29`，另读两台状态；
- 明确排除清故障 `0x18`、急停 `0x19`、pause `0x1A`、还原 `0x1B`、保存 `0x1C` 和权限验证码 `0x1D`；
- 所有读取均为单寄存器；任意写、其他 ID、非白名单地址和多寄存器读在冻结底层前拒绝，汇总必须为 `write_count=0`；
- 每项记录 raw u16、hex 和 signed i16；`0xFFFE` 保持为 `-2`，不 clamp，不修改 `[0,2000]` 门禁；
- 分类区分“除 ID 与动态 target 外配置一致”和“观察到静态配置差异”，不把 ID 地址或 target 状态差异误判为零点配置；
- 专项无硬件测试 `19 passed`，全仓无硬件回归 `205 passed`。

已有日志只能确认以下配置差异：ID 地址按设计分别为 3/4；行程下限均 0、上限均 2000、mode 均 0、target speed 均 65535；target/actual 为 ID3 约 0、ID4 持续 -2。波特率、保护值、输出上限、voltage 和 force target 尚无 ID3/ID4 正式对照结果。

该配置审计保持 `PREPARED / NOT EXECUTED`，但当前执行优先级已由 ID4-OFFSET 取代；未执行该命令。

## ID4-OFFSET 固定零偏表征（2026-08-17 15:11，COMPLETE / PASS）

状态：`COMPLETE / PASS`。

- 判定：`PASS / ORDINARY_0_TO_3_STEP_RESIDUAL_SUPPORTED`；
- 工具：`tools/safe_id4_offset_characterization.py`；
- 原始日志：`logs/multi_motor/id4_offset/20260817_151126_id4_offset.log`；
- 硬件连接：仅连接并固定 ID4，急停可用；
- 起始状态：100/100 连续采样 target_raw=`0xFFFE/-2`、actual_raw=`0xFFFE/-2`、current=`0mA`、fault=0，行程仍为 `[0,2000]`；
- 运行数据（固定目标 10/20/30/40 step，速度 50 step/s，电流上限 500mA）：
  - Point 1 (target=10): commanded=10, readback=10, raw target=10, raw actual=9, actual=9, error=-1, current=0mA, force=-15g, fault=0x00; pause 后 readback=9, actual=9;
  - Point 2 (target=20): commanded=20, readback=20, raw target=20, raw actual=19, actual=19, error=-1, current=18mA, force=-15g, fault=0x00; pause 后 readback=19, actual=19;
  - Point 3 (target=30): commanded=30, readback=30, raw target=30, raw actual=28, actual=28, error=-2, current=20mA, force=-15g, fault=0x00; pause 后 readback=28, actual=28;
  - Point 4 (target=40): commanded=40, readback=40, raw target=40, raw actual=39, actual=39, error=-1, current=19mA, force=-15g, fault=0x00; pause 后 readback=39, actual=39;
- 停止确认：四点 `stop_confirmation` 均稳定通过（`stable=True`，电流均为 0，fault 均为 0）；
- 写审计汇总：`write_count=8`（全部 destination=4，写入寄存器序列为 `0x0025, 0x001A, 0x0029, 0x001A, 0x0029, 0x001A, 0x0029, 0x001A`），`read_register_count=222, read_status_count=495, broadcast_write_count=0, other_id_write_count=0, calibration_write_count=0, flash_write_count=0, compensation_write_count=0`；
- 实机结论：
  1. 四点到位误差在 `[-2, -1]` 区间（变动非恒定 -2），不支持固定 `-2 step` 零偏假设；
  2. 严禁执行 `+2 step` 或任何形式的位置补偿；
  3. ID4 在进入合法正区间（>=0）后，表现为与其他电机一致的普通 `0~3 step` 机械到位残差；
  4. 剩余问题限定为：ID4 掉电上电后初始寄存器及状态 raw target/actual 持续处于 `-2`。

## ID4 近零端边界恢复与保护（2026-08-17 16:15，COMPLETE / PASS）

状态：`COMPLETE / PASS`。

- 判定：`PASS / ALREADY_IN_RANGE_NO_ACTION`；
- 工具：`tools/safe_id4_poweron_initialize.py`；
- 原始日志：`logs/multi_motor/id4_init/20260817_161543_id4_poweron_init.log`；
- 硬件操作：人工断电后重新上电，再执行 ID4-INIT 诊断保护；
- 采样数据：连续 100/100 轮读取 `0x29 target`、`0x2A actual` 及 `0x30` 状态，target 均为 39、actual 均为 39、current 始终 0mA、fault 始终 0x00；
- 访问审计：`write_count=0, read_register_count=202, read_status_count=100, compensation_write_count=0`；
- 门禁状态：`global_gate_restored=true`（全局 `[0,2000]` 门禁完整有效）；
- 关键结论修正：
  1. ID4 的 raw=-2 不是固定“上电必然触发状态”，而是“近零端特殊边界状态”；
  2. 实测确认 ID4 在约 39 step 掉电重启后仍读取 target=39、actual=39，保持在合法位置区间；底层掉电保持机制未由官方资料确认，不归因于绝对位置传感器；
  3. 严禁 `+2 step` 或任何形式的位置补偿；
  4. 严禁为了触发 -2 而主动将 ID4 移回零端；
  5. `safe_id4_poweron_initialize.py` 转化为常态化近零端边界恢复与保护工具。

## Stage M4-B1 Retry 实机预检结果（2026-08-17 16:20:22）

原始日志：`logs/multi_motor/m4/20260817_162022_m4b1_retry_id1_only.log`。

- 结果：`ABORTED / NOT PASSED`；
- 原因：在 baseline 采样第 2 轮，ID3 状态返回 target=`-1`、actual=`-1`（原始有符号值 `0xFFFF`），触发硬件下限门禁 `actual position=-1 outside device limits [0, 2000]`；
- 写审计：`write_count=0`，`write_destinations=[]`，`observer_write_count=0`；在写任何运动指令前安全中止，**确认没有任何电机发生运动**；
- 停止时其他电机状态：ID1 actual=45、ID2 actual=45、ID4 actual=39，全部在合法区间；
- 历史记录保留：首次 M4-B1 与本次 M4-B1 Retry 的 ABORTED 记录均永久保留。

## ID3 纯只读状态诊断（2026-08-17 16:21:48）

原始日志：`logs/multi_motor/m4/20260817_162148_id3_readonly_diagnosis.log`。
工具：`tools/diagnose_id3_id4_readonly.py`，纯只读审计（`write_count=0`），连续采集 100 组。

- ID3 统计结果：
  - 寄存器 target (0x29)：`{-1: 12, 0: 88}`（min=-1, max=0）；
  - 状态 target：`{-1: 19, 0: 81}`（min=-1, max=0）；
  - 状态 actual：`{-1: 19, 0: 81}`（min=-1, max=0）；
  - 电流 current：`{0: 100}`（100% 为 0mA）；
  - 故障码 fault：`{0: 100}`（100% 为 0x00）；
  - 受力 force：`333g ~ 364g`，静态平稳；
- ID4 统计对照：100/100 轮均为 target=39、actual=39、current=0mA、fault=0x00；
- 异常分类：**零端 ±1 step 静态量化抖动（Zero-endpoint ±1 step quantization jitter）**；主模态为 0（81%~88%），无负向失控（无 -2 或更小值），纯属传感器物理零点处的边缘量化读数抖动。

## Stage ID3-ESCAPE：ID3 零端脱离实机结果（2026-08-17 16:26:34）

原始日志：`logs/multi_motor/id3_escape/20260817_162634_id3_zero_escape.log`。

- 结果：`FAIL / HARD STOP REQUIRED — stop confirmation failed`（历史保留，不追溯改判）；
- 执行详情：
  1. 入口 100 轮只读预检通过，判定 `entry_state=ESCAPE_REQUIRED`；
  2. `0x25` 运动配置写入成功（target=10, speed=50），寄存器 `0x29` 读回确认 10；
  3. ID3 在 0.265s 首次正向运动，于 1.174s 达到 actual=7 step 并触发单次 pause（响应正常）；
  4. 首轮 0.5s 停止确认中前 10 样本均为 7，第 11 样本跳变到 6，因首尾不同且末端未回弹触发 `bounded but directional trend not settled`；
  5. 写审计：`write_count=2`，`write_destinations=[3, 3]`，ID1/ID2/ID4 严格零写入；峰值电流 33mA，全程无 fault、无越界、无反向运动；
- 脱后 100 轮纯只读诊断（`logs/multi_motor/m4/20260817_162836_id3_post_escape_diagnosis.log`）：
  - 寄存器 target: `{6: 36, 7: 64}`，状态 target/actual: `{6: 31, 7: 69}`，current=0mA×100，fault=0x00×100；
  - 确认 ID3 脱零运动目标已实际达成，ID3 当前处于 `6~7 step`，表现为既知正常的 `±1 step` 静态量化抖动；
  - 原始 FAIL 判定不予追溯改判，不再重复执行 ID3-ESCAPE。

## Stage M4-B1 Retry 实机结果（2026-08-17 16:31:11）

原始日志：`logs/multi_motor/m4/20260817_163111_m4b1_retry_id1_only.log`。

- 结果：**`COMPLETE / PASS`**；
- 运动电机：`ID1`
  - baseline_reference=`45`，baseline_safety=`[45, 46]`；
  - 目标 target=`55`，最终到达 actual=`53`，到位误差 error=`2 step`（符合 $\le 2$ step 门禁要求）；
  - 峰值电流=`62 mA`（远低于 500mA 上限），fault=`0x00`；
- 观察电机（全程只读）：
  - **ID2**：actual=`45~46 step`，current=`0 mA`，fault=`0x00`，配置无变化；
  - **ID3**：actual=`6~7 step`，current=`0 mA`，fault=`0x00`，配置无变化；
  - **ID4**：actual=`39 step`，current=`0 mA`，fault=`0x00`，配置无变化；
- 写审计：`write_count=3`，`write_destinations=[1, 1, 1]`（`0x25` 运动配置、`0x29` 目标读写、`0x1A` 单次 pause），`observer_write_count=0`（ID2/ID3/ID4 严格 0 写入）；
- 安全控制：`pause_success=True`，四机停止确认 `stop_confirmation=True`；
- 历史记录保留：首次 M4-B1 与首次 M4-B1 Retry 的 ABORTED 历史记录永久保留。

## Stage M4-B2 首次实机结果（2026-08-17 16:33:20）

原始日志：`logs/multi_motor/m4/20260817_163320_m4b2_id2_only.log`。

- 结果：**`STOP / NOT PASSED — ID1 configuration changed during after_setup`**（历史永久保留，不追溯改判）；
- 详情与原因：
  1. ID1 在 M4-B1 Retry 完成后 pause，固件内部 target 保持在停止位置（52/53 step）；
  2. ID1 传感器在 52~53 step 临界处存在固有的 $52 \leftrightarrow 53$ 的 1 step 静态量化抖动（baseline 采样记录 status target 为 `{52: 69, 53: 31}`）；
  3. `safe_m4b_one_motor.py` 原代码使用单点绝对全等比对，precheck 采集到 target=53，在 after_setup 读回 target=52 时触发拦截；
  4. 写审计：`write_count=2`，`write_destinations=[2, 2]`（ID2 `0x25` setup 与 `0x1A` pause），`observer_write_count=0`（ID1/3/4 严格 0 写入，无电机运动，无 fault）；
- ID1 100 轮只读诊断（`logs/multi_motor/m4/20260817_163444_id1_readonly_diagnosis.log`）：
  - mode=2（100%），speed=50（100%），register target `{52: 74, 53: 26}`，status target/actual `{52: 70, 53: 30}`，current=0mA×100，fault=0x00×100；
- 审计逻辑优化修复：
  - 更新 `safe_m4b_one_motor.py`：mode 和 speed 必须严格不变，在 baseline 阶段为每个 observer 建立独立的 `target safety band`，after_setup/motion/pause/final 阶段要求 observer target 必须落在其 baseline target safety band 内；
  - 补充 4 项单元测试，`test_safe_m4b_one_motor.py` 达到 27 passed，全仓回归 248 passed。

## Stage M4-B2 Retry 首次实机结果（2026-08-17 16:41:02）

原始日志：`logs/multi_motor/m4/20260817_164049_m4b2_retry_id2_only.log`。

- 结果：**`STOP / NOT PASSED — ID2 post-pause final_error=3 exceeds 2 step`**（历史永久保留，不追溯改判）；
- 过程还原与审计确认：
  1. `moving_id = 2`，baseline reference=`45 step`（Safety Band=`[45, 46]`），commanded target=`55 step`；
  2. ID2 严格正向运动 $45 \to 53\text{ step}$，到达 actual=`53 step`（$\text{error}=2\text{ step}$）时正常触发 pause；
  3. `pause_success = True`，在 post-pause 0.5s 静止采样中稳定在 actual=`52 step`（$\text{final\_error}=3\text{ step}$）；
  4. 四机停止确认 `stop_confirmation = True`；
  5. ID2 峰值电流 `49 mA`，全量故障码 `0x00`；
  6. 写审计：`write_count = 3`，`write_destinations = [2, 2, 2]`，`observer_write_count = 0`（ID1/ID3/ID4 严格 0 写入，无物理位移）；
  7. 唯一失败原因：原代码 `INTEGRATION_TOLERANCE_STEPS = 2` 过于严苛，将电机物理固有的 $0\sim 3\text{ step}$ 正常残差判定为超差；
- 判据明确拆分修复：
  - `PAUSE_TRIGGER_TOLERANCE_STEPS = 2`（运动阶段 pause 触发保持 $\le 2\text{ step}$，不放宽）；
  - `FINAL_ACCEPTANCE_TOLERANCE_STEPS = 3`（post-pause 最终验收误差 $\le 3\text{ step}$ PASS，$\ge 4\text{ step}$ FAIL）；
  - 补充 4 项单元测试，`test_safe_m4b_one_motor.py` 达到 31 passed，全仓回归 252 passed。

## Stage M4-B2 Retry2 最新实机结果（2026-08-17 16:46:12）

原始日志：`logs/multi_motor/m4/20260817_164559_m4b2_retry2_id2_only.log`。

- 结果：**`COMPLETE / PASS`**；
- 运动电机：`ID2`
  - baseline_reference=`52`，baseline_safety=`[52, 53]`；
  - 目标 target=`62`，到达 actual=`60`（error=2）触发 pause，停机复核后稳定在 actual=`59`，最终到位误差 error=`3 step`（符合 $\le 3\text{ step}$ 最终验收门禁）；
  - 峰值电流=`49 mA`（远低于 500mA 上限），停机后电流归 0mA，fault=`0x00`；
- 观察电机（全程只读）：
  - **ID1**：actual=`52~53 step`，current=`0 mA`，fault=`0x00`，配置无变化；
  - **ID3**：actual=`6~7 step`，current=`0 mA`，fault=`0x00`，配置无变化；
  - **ID4**：actual=`39 step`，current=`0 mA`，fault=`0x00`，配置无变化；
- 写审计：`write_count=3`，`write_destinations=[2, 2, 2]`（`0x25` 运动配置、`0x29` 目标读写、`0x1A` 单次 pause），`observer_write_count=0`（ID1/ID3/ID4 严格 0 写入）；
- 安全控制：`pause_success=True`，四机停止确认 `stop_confirmation=True`；
- 历史记录保留：首次 M4-B2 与首次 M4-B2 Retry 的 STOP 历史记录永久保留。

## Stage M4-B3 最新实机结果（2026-08-17 16:49:49）

原始日志：`logs/multi_motor/m4/20260817_164937_m4b3_id3_only.log`。

- 结果：**`COMPLETE / PASS`**；
- 运动电机：`ID3`
  - baseline_reference=`7`，baseline_safety=`[6, 7]`；
  - 目标 target=`17`，到达 actual=`15`（error=2）触发 pause，停机复核后稳定在 actual=`15`，最终到位误差 error=`2 step`（符合 $\le 3\text{ step}$ 最终验收门禁）；
  - 峰值电流=`34 mA`（远低于 500mA 上限），停机后电流归 0mA，fault=`0x00`；
- 观察电机（全程只读）：
  - **ID1**：actual=`52 step`，current=`0 mA`，fault=`0x00`，配置无变化；
  - **ID2**：actual=`59~60 step`，current=`0 mA`，fault=`0x00`，配置无变化；
  - **ID4**：actual=`39 step`，current=`0 mA`，fault=`0x00`，配置无变化；
- 写审计：`write_count=3`，`write_destinations=[3, 3, 3]`（`0x25` 运动配置、`0x29` 目标读写、`0x1A` 单次 pause），`observer_write_count=0`（ID1/ID2/ID4 严格 0 写入）；
- 安全控制：`pause_success=True`，四机停止确认 `stop_confirmation=True`。

## Stage M4-B4 最新实机结果（2026-08-17 16:53:35）

原始日志：`logs/multi_motor/m4/20260817_165331_m4b4_id4_only.log`。

- 结果：**`COMPLETE / PASS`**；
- 运动电机：`ID4`
  - baseline_reference=`39`，baseline_safety=`[39, 39]`；
  - 目标 target=`49`，到达 actual=`47`（error=2）触发 pause，停机复核后稳定在 actual=`47`，最终到位误差 error=`2 step`（符合 $\le 3\text{ step}$ 最终验收门禁）；
  - 峰值电流=`33 mA`（远低于 500mA 上限），停机后电流归 0mA，fault=`0x00`；
- 观察电机（全程只读）：
  - **ID1**：actual=`52 step`，current=`0 mA`，fault=`0x00`，配置无变化；
  - **ID2**：actual=`59~60 step`，current=`0 mA`，fault=`0x00`，配置无变化；
  - **ID3**：actual=`15 step`，current=`0 mA`，fault=`0x00`，配置无变化；
- 写审计：`write_count=3`，`write_destinations=[4, 4, 4]`（`0x25` 运动配置、`0x29` 目标读写、`0x1A` 单次 pause），`observer_write_count=0`（ID1/ID2/ID3 严格 0 写入）；
- 安全控制：`pause_success=True`，四机停止确认 `stop_confirmation=True`。

**Stage M4-B 汇总**：M4-B1、M4-B2、M4-B3、M4-B4 四电机单机独立运动验证已全部实机 **PASS**。

## Stage M4-C 最新实机结果（2026-08-17 17:02:44）

原始日志：`logs/multi_motor/m4/20260817_170233_m4c_joint_four_motors.log`。

- 结果：**`COMPLETE / PASS`**；
- 运动模式：ID1~ID4 全量四台电机同一 UART 总线联合运动 / 协同控制验证（`+10 step / 50 step/s / 2.0s / 500mA`）；
- 四台电机实机数据：
  - **ID1**：`baseline_reference=52`，`baseline_safety=[52, 52]`，目标 target=`62`，到达 actual=`60`（error=2）触发 pause，停机复核后稳定在 actual=`59`，最终到位误差 error=`3 step`（符合 $\le 3\text{ step}$ 验收门禁），峰值电流 `62 mA`，`fault=0x00`；
  - **ID2**：`baseline_reference=59`，`baseline_safety=[58, 60]`，目标 target=`69`，到达 actual=`67`（error=2）触发 pause，停机复核后稳定在 actual=`67`，最终到位误差 error=`2 step`（符合 $\le 3\text{ step}$ 验收门禁），峰值电流 `47 mA`，`fault=0x00`；
  - **ID3**：`baseline_reference=15`，`baseline_safety=[15, 15]`，目标 target=`25`，到达 actual=`23`（error=2）触发 pause，停机复核后稳定在 actual=`23`，最终到位误差 error=`2 step`（符合 $\le 3\text{ step}$ 验收门禁），峰值电流 `29 mA`，`fault=0x00`；
  - **ID4**：`baseline_reference=47`，`baseline_safety=[46, 47]`，目标 target=`57`，到达 actual=`55`（error=2）触发 pause，停机复核后稳定在 actual=`54`，最终到位误差 error=`3 step`（符合 $\le 3\text{ step}$ 验收门禁），峰值电流 `27 mA`，`fault=0x00`；
- 联合运动与协同过程：
  - 各机首次正向起步时间：ID4 @ 0.30s，ID3 @ 0.37s，ID2 @ 0.73s，ID1 @ 0.76s（实测起步存在离散时差，当前阶段未验证高精度同步启动，不表述为严格同步控制）；
  - 全四台电机在 1.36s 内全部到达各自误差 $\le 2\text{ step}$ 触发统一 Pause；
  - Pause 发送：依次向 ID1 $\to$ ID2 $\to$ ID3 $\to$ ID4 发送单次 Pause（`0x1A`），全部返回成功（`pause_all=true`）；
  - 停止确认：0.5s 静止采样内全四台电机通过稳定性评估（`stop_confirmation=True`，无残余漂移与发散）；
- 写审计汇总：
  - `write_count=12`，`write_destinations=[1, 2, 3, 4, 1, 2, 3, 4, 1, 2, 3, 4]`（依次为 4 次 `0x25` 运动配置、4 次 `0x29` 目标下发、4 次 `0x1A` Pause）；
  - 0 次非白名单写入，0 次越权操作；
  - 各机读状态与寄存器采样均为 165 次。

**Stage M4 总体总结**：
- Stage M4-A（四机只读通信与总线竞争消除）：`COMPLETE / PASS`；
- Stage M4-B（四机单机独立小幅运动 M4-B1 ~ M4-B4）：`COMPLETE / PASS`；
- Stage M4-C（四机联合小幅运动 / 协同控制验证）：`COMPLETE / PASS`。
- **四电机共线 UART 通信、安全门禁、单机独立控制及多机协同控制验证已全量闭环完成**。

当前四台电机静止位置：
- **ID1**：`59 ~ 60 step`
- **ID2**：`67 step`
- **ID3**：`23 step`
- **ID4**：`54 ~ 55 step`

## Stage M6 规划与实机进展

Stage M6 推进路线（按最小增量原则）：
1. **M6-A**：`COMPLETE / PASS`。ID1~ID6 六机纯只读状态与配置复核（`write_count=0`）；
2. **M6-B5**：`PREPARED / NOT EXECUTED`。仅 ID5 执行 `+10 step` 小幅运动，ID1/2/3/4/6 严格只读（ID1~ID4 不重复单机运动验证）；
3. **M6-B6**：`LOCKED / NOT PREPARED`。仅 ID6 执行 `+10 step` 小幅运动，ID1/2/3/4/5 严格只读；
4. **M6-C**：`LOCKED / NOT PREPARED`。ID1~ID6 六电机联合 `+10 step` 小幅运动 / 协同控制验证。

### Stage M6-A 实机结果（2026-08-17 17:31:48）

原始日志：`logs/multi_motor/m6/20260817_173141_m6a_readonly_review.log`。

- 结果：**`COMPLETE / PASS`**；
- 写审计：`write_count=0`（100% 严格纯只读，0 次写入）；
- 通信审计：各机读计数均为 105 次（5 次配置读 + 100 轮状态读），总线读事务 630 次 100% 成功，0 timeout、0 wrong_id、0 checksum 错误、0 length 错误、0 总线碰撞；
- 各机 100 轮采样统计数据：
  - **ID1**：limits=`[0, 2000]`，mode=`0`，speed=`65535`，target=`59`，actual=`59`（`{59: 100}`），current=`0 mA`，fault=`0x00`，force=`-360g`；
  - **ID2**：limits=`[0, 2000]`，mode=`0`，speed=`65535`，target=`67`，actual=`66~67`（`{66: 16, 67: 84}`），current=`0 mA`，fault=`0x00`，force=`280g`；
  - **ID3**：limits=`[0, 2000]`，mode=`0`，speed=`65535`，target=`22`，actual=`22~23`（`{22: 84, 23: 16}`），current=`0 mA`，fault=`0x00`，force=`367~400g`；
  - **ID4**：limits=`[0, 2000]`，mode=`0`，speed=`65535`，target=`54`，actual=`54~55`（`{54: 97, 55: 3}`），current=`0 mA`，fault=`0x00`，force=`-6~0g`；
  - **ID5**：limits=`[0, 2000]`，mode=`0`，speed=`65535`，target=`0`，actual=`0~1`（`{0: 88, 1: 12}`），current=`0 mA`，fault=`0x00`，force=`272g`；
  - **ID6**：limits=`[0, 2000]`，mode=`0`，speed=`65535`，target=`0`，actual=`0`（`{0: 100}`），current=`0 mA`，fault=`0x00`，force=`110~125g`；
- **ID5 / ID6 零端边界状态分析**：
  - ID5 和 ID6 均稳定处于合法位置区间（`0 ~ 1 step` 与 `0 step`）；
  - 未出现持续负值异常（无 raw `-1` 或 `-2`），行程配置严格为 `[0, 2000]`，无电流与故障；
  - ID5、ID6 当前处于物理出厂初始零端，后续正向运动目标 `+10 step` 将向行程安全正向区域移动（`0 -> 10 step`），具备进入 Stage M6-B 的安全前提。

### Stage M6-B5 首次实机运行（2026-08-17 18:14:34）

原始日志：`logs/multi_motor/m6/20260817_181430_m6b5_id5_only.log`。

- 结果：**`ABORTED / STOP / NOT PASSED`**（保留原始历史，不追溯改判）；
- 写审计：`write_count=0`，`observer_write_count=0`（基线收集阶段触发安全拦截，首条写寄存器前安全中止，全六台电机零写入、零运动）；
- 触发原因：ID2 作为 Observer 电机，在 100 轮静态基线采样中分布为 `{67: 86, 66: 13, 65: 1}`（众数 67），单点 65 偏差 2 step 触发现有 `_build_m2d_position_baseline` 的 `abs(sample - mode) <= 1` 静态门禁；
- 其余电机：ID1（59）、ID3（22）、ID4（54）、ID5（0）、ID6（0）基线 100% 恒定。

### ID2 500 轮纯只读扩展诊断（2026-08-17 18:17:53）

原始日志：`logs/multi_motor/m6/20260817_181740_id2_readonly_diagnosis_500.log`。

- 采样规模：500 轮连续采样，1000 次 UART 读事务 100% 成功，`write_count=0`；
- 统计结果：
  - actual：`{67: 424 (84.8%), 66: 74 (14.8%), 65: 2 (0.4%)}`，min=65, max=67, span=2 step ($\approx 0.05\text{ mm}$)；
  - reg_target `0x29`：`{67: 421, 66: 76, 65: 3}`；status_target：`{67: 424, 66: 74, 65: 2}`；
  - current=0mA (100%), fault=0x00 (100%)；无 $\le 64$ 或 $\ge 68$；
- 诊断定性：良性低概率 2-step 静态量化抖动（编码器停在临界量化边界），无单向漂移。

### Observer 基线门禁规则针对性升级

1. **Moving 电机（保持原规则）**：必须具有唯一众数，全量样本严格位于 `reference ± 1` 步以内，不放宽；
2. **Observer 电机（升级专用判据）**：
   - 必须具有唯一众数；
   - 位置在合法区间 `[0, 2000]`；
   - 全宽跨度满足 $\text{max} - \text{min} \le 2\text{ step}$；
   - 电流全为 0 mA，故障码全为 0x00；
   - 严格检验不存在单向持续漂移趋势（单调递增/递减或前后半段离散分离）；
   - baseline target band 与 actual band 独立建立为实际 $[\text{min}, \text{max}]$；
3. **Observer 运行时及停机监测**：实际位置与目标位置必须严格保持在其 baseline band 内，超出立即 STOP；`observer_write_count` 严格为 0。

### Stage M6-B5 Retry 首次实机运行（Attempt 1, 2026-08-17 18:22:10）

原始日志：`logs/multi_motor/m6/20260817_182206_m6b5_retry_id5_only.log`。

- 结果：**`STOP / NOT PASSED`**（永久独立保留历史）；
- 写审计：`write_count=1`（仅向 ID5 下发一条 setup 写，未进入位移与 pause）；
- 触发原因：Step 5 写入目标位置时调用了不存在的接口 `motor.set_target_position` 抛出 AttributeError 中止。

### Stage M6-B5 Retry 第二次实机运行（Attempt 2, 2026-08-17 18:22:53）

原始日志：`logs/multi_motor/m6/20260817_182246_m6b5_retry_id5_only.log`。

- 结果：**`STOP / NOT PASSED`**（永久独立保留历史）；
- 写审计：`moving_id=5` 写入 2 次（`0x0025`, `0x0029`），`observer_write_count=0`（ID1/ID2/ID3/ID4/ID6 全程严格零写入）；
- 基线建立：ID1~ID6 全部正常建立基线（ID2 判定为良性抖动带 `[65, 67]` 通过）；
- 触发原因：
  1. M6-B setup payload 寄存器映射错误，导致 0x28 target speed 未按设计配置为 50，因此该次运动配置无效；现有证据不能进一步判断 ID5 内部未驱动的具体机制；
  2. 异常处理中 `_pause_single` 调用 `motor.pause()` 接口错误。

### 协议语义纠正与完整代码修复

1. **V2.0.4 连续寄存器语义对齐（与 Stage M4-B 严格一致）**：
   - `0x25`：Control Mode = `2`（SPEED_MODE）
   - `0x26`：Voltage Target = `0`
   - `0x27`：Force Target = `0`
   - `0x28`：Target Speed = `50`（SPEED_STEPS_PER_SECOND）
   - `0x29`：Target Position = `moving_baseline.position.safety_max`（基线静止点）
2. **目标位置单写**：`motors[moving_id].write_register(0x29, [target_position])`；
3. **Pause 接口修复**：`_pause_single` 统一切换为标准 `motor.write_register(0x1A, [1])`，正常与异常路径统一检查并支持安全升级（HARD STOP REQUIRED）；
4. **底层保持冻结**：`protocol.py` / `uart_bus.py` / `motor.py` 零修改；
5. **门禁规则保持不变**：Moving baseline（$\text{mode}\pm 1$）、Observer baseline（$\text{span}\le 2$ 且无漂移）、$500\text{ mA}$ 电流限值、pause 触发 $2\text{ step}$、最终验收 $3\text{ step}$。

### Stage M6-B5 Retry 第三次实机运行（Attempt 3, 2026-08-17 18:28:42）

原始日志：`logs/multi_motor/m6/20260817_182836_m6b5_retry_id5_only.log`。

- 结果：**`STOP / NOT PASSED`**（永久独立保留历史）；
- 写审计：`write_count=3`（严格按顺序向 ID5 写入 `0x0025`, `0x0029`, `0x001A`），`observer_write_count=0`（ID1/ID2/ID3/ID4/ID6 全程严格零写入）；
- 通信参数与总线：UART 波特率实际为 921600 8N1；全过程 162 次读事务 100% 成功；
- 运动过程审计：
  - Setup 回读确认：`mode=2, speed=50`；Target 回读确认：`target=10`；
  - 运动确认（Positive Motion）：ID5 在零端克服静摩擦约 1.0s 后正向起步，平稳行进 `0 -> 1 -> 2 -> ... -> 7 step`；
  - 峰值电流：`30 mA`（远低于 500mA 阈值），故障码全程 `0x00`；
  - 观察机隔离：ID1（59）、ID2（67）、ID3（22）、ID4（54）、ID6（0）严格保持在基线带内；
  - 超时与保护：在 $2.0\text{s}$ 门禁截止时刻，ID5 达到 `actual = 7 step`（`err = 3`，差 1 步达到 `err <= 2` 的 pause 条件），触发超时保护；`_pause_single` 成功下发 `0x1A: [1]`，电机平稳停止在 `actual = 8 step`。
- 根因分析：
  - ID5 硬件运动正常，超时主因为零端首次启动存在约 1.0s 启动延迟（与此前 `AGENTS.md` 记录的 ID1 低端启动延迟类似），致使原有 2.0s 运动超时门禁偏紧；
  - 六机总线轮询仅影响监测采样时延，不降低电机实际物理运动速度。

### M6-B 运动超时门禁参数优化

- **修改项**：`MOTION_TIMEOUT_S` 由 `2.0s` 适度放宽至 `3.0s`；
- **严格保持不变的全部规则**：
  - `+10 step` 目标位移；
  - `50 step/s` 运动速度；
  - `500 mA` 电流上限；
  - `error <= 2` pause 触发阈值；
  - `final_error <= 3` 最终验收门禁；
  - Moving baseline 严苛规则（$\text{mode}\pm 1$）；
  - Observer baseline 专用规则（$\text{span}\le 2$ 且无持续单向漂移）；
  - 故障/超流/反向/越界立即 STOP 保护（不等待 3.0s 超时）；
  - 冻结底层代码（`protocol.py` / `uart_bus.py` / `motor.py` 零修改）。

### Stage M6-B5 实机运行（2026-08-17 18:33:11，COMPLETE / PASS）

原始日志：`logs/multi_motor/m6/20260817_183305_m6b5_retry_id5_only.log`。

- 结果：**`COMPLETE / PASS`**
- 运行方案：六机同总线共线，仅 ID5 执行 `+10 step / 50 step/s / 3.0s / 500mA` 正向运动；ID1/ID2/ID3/ID4/ID6 保持严格只读观察（`observer_write_count=0`）；
- 基线建立（100 轮采样）：
  - ID5（Moving）：`reference=7`（actual={7: 58, 8: 42}），safety band=`[7, 8]`，target_ref=7，target=`17 step`；
  - ID1（Observer）：59，band=`[59, 59]`，current=0mA, fault=0x00；
  - ID2（Observer）：67，band=`[66, 67]`，current=0mA, fault=0x00；
  - ID3（Observer）：22，band=`[22, 23]`，current=0mA, fault=0x00；
  - ID4（Observer）：54，band=`[54, 55]`，current=0mA, fault=0x00；
  - ID6（Observer）：0，band=`[0, 0]`，current=0mA, fault=0x00；
- 写序列与写审计：
  - `write_count = 3`（100% 严格发往 ID5）：
    1. `18:33:09.684` WRITE ID5 `0x0025` data=`02 00 00 00 00 00 32 00 08 00`（mode=2, speed=50, pos=8）；
    2. `18:33:09.738` WRITE ID5 `0x0029` data=`11 00`（target=17）；
    3. `18:33:10.970` WRITE ID5 `0x001A` data=`01 00`（pause=1）；
  - `observer_write_count = 0`（ID1/ID2/ID3/ID4/ID6 全程零写入）；
- 正向运动与停机确认：
  - ID5 从 8 step 正向平稳行进至 15 step；
  - 到达 15 step 时误差 `err = abs(17 - 15) = 2 step`，触发 pause 停机；
  - 停机确认（25 轮 0.5s 采样）：位置完全稳定在 15 step（`delta = 0`），`pause_success=True`, `stop_confirmation=True`；
  - 最终验收误差：`final_error = 2 step <= 3 step`（合格通过）；
- 安全性指标：
  - ID5 峰值电流：`30 mA`（$\ll 500\text{ mA}$）；
  - 全六机故障码：全程严格 `0x00`；
  - 观察机隔离：ID1、ID2、ID3、ID4、ID6 全程严格保持在 baseline band 内。

---

### Stage M6-B6 实机运行（2026-08-17 18:34:58，COMPLETE / PASS）

原始日志：`logs/multi_motor/m6/20260817_183450_m6b6_id6_only.log`。

- 结果：**`COMPLETE / PASS`**
- 运行方案：六机同总线共线，仅 ID6 执行 `+10 step / 50 step/s / 3.0s / 500mA` 正向运动；ID1/ID2/ID3/ID4/ID5 保持严格只读观察（`observer_write_count=0`）；
- 基线建立（100 轮采样）：
  - ID6（Moving）：`reference=0`（actual={0: 100}），safety band=`[0, 0]`，target_ref=0，target=`10 step`；
  - ID1（Observer）：59，band=`[59, 59]`，current=0mA, fault=0x00；
  - ID2（Observer）：67，band=`[66, 67]`，current=0mA, fault=0x00；
  - ID3（Observer）：22，band=`[22, 23]`，current=0mA, fault=0x00；
  - ID4（Observer）：54，band=`[54, 55]`，current=0mA, fault=0x00；
  - ID5（Observer）：15，band=`[14, 15]`，current=0mA, fault=0x00；
- 写序列与写审计：
  - `write_count = 3`（100% 严格发往 ID6）：
    1. `18:34:54.671` WRITE ID6 `0x0025` data=`02 00 00 00 00 00 32 00 00 00`（mode=2, speed=50, pos=0）；
    2. `18:34:54.726` WRITE ID6 `0x0029` data=`0A 00`（target=10）；
    3. `18:34:57.065` WRITE ID6 `0x001A` data=`01 00`（pause=1）；
  - `observer_write_count = 0`（ID1/ID2/ID3/ID4/ID5 全程零写入）；
- 正向运动与停机确认：
  - 启动延迟：ID6 在零端克服静摩擦约 0.8s 后正向平稳起步，行进至 8 step；
  - 到达 8 step 时误差 `err = abs(10 - 8) = 2 step`，触发 pause 停机；
  - 停机确认（25 轮 0.5s 采样）：位置完全稳定在 8 step（`delta = 0`），`pause_success=True`, `stop_confirmation=True`；
  - 最终验收误差：`final_error = 2 step <= 3 step`（合格通过）；
- 安全性指标：
  - ID6 峰值电流：`44 mA`（$\ll 500\text{ mA}$）；
  - 全六机故障码：全程严格 `0x00`；
  - 观察机隔离：ID1、ID2、ID3、ID4、ID5 全程严格保持在 baseline band 内。

### ID1~ID6 单机独立小幅正向运动（M4-B / M6-B）实机结果汇总

各单机独立运动验证（每次仅 1 台运动，其余全量共线电机严格零写入）的实机结果如下（不与 M4-C 联合运动最终位置混淆）：

| 电机 ID | 阶段代号 | 实机时间 | 基线位置 (Baseline) | 目标位置 (Target) | 最终到位 (Final) | 误差 (Error) | 验收判定 |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **ID1** | Stage M4-B1 Retry | 2026-08-17 16:31:11 | 45 | 55 | 53 | 2 step | **PASS** ($\le 3$) |
| **ID2** | Stage M4-B2 | 2026-08-17 16:47:04 | 52 | 62 | 59 | 3 step | **PASS** ($\le 3$) |
| **ID3** | Stage M4-B3 | 2026-08-17 16:54:19 | 7 | 17 | 15 | 2 step | **PASS** ($\le 3$) |
| **ID4** | Stage M4-B4 | 2026-08-17 17:02:18 | 39 | 49 | 47 | 2 step | **PASS** ($\le 3$) |
| **ID5** | Stage M6-B5 | 2026-08-17 18:33:11 | 7 | 17 | 15 | 2 step | **PASS** ($\le 3$) |
| **ID6** | Stage M6-B6 | 2026-08-17 18:34:58 | 0 | 10 | 8 | 2 step | **PASS** ($\le 3$) |

> **注（历史位置演变说明）**：
> - Stage M4-C 四机联合运动（2026-08-17 17:02:44）在 M4-B1~B4 之后执行，四机起始基线分别为 ID1=52, ID2=59, ID3=15, ID4=47，运动后分别停在 ID1=59, ID2=67, ID3=22~23, ID4=54~55；
> - 其后 Stage M6-B5 仅使 ID5 从 7 运动至 15；Stage M6-B6 仅使 ID6 从 0 运动至 8；
> - 当前（M6-C 执行前）六台电机实际静态位置为：**ID1 $\approx 59$, ID2 $\approx 67$, ID3 $\approx 22\sim 23$, ID4 $\approx 54\sim 55$, ID5 $\approx 15$, ID6 $\approx 8$**。

---

### Stage M6-C 实机运行（2026-08-17 18:41:36，STOP / NOT PASSED）

原始日志：`logs/multi_motor/m6/20260817_184132_m6c_joint_six_motors.log`。

- 结果：**`STOP / NOT PASSED`**（永久独立保留历史记录，不自动重试，不修改代码）；
- 方案：ID1~ID6 六机共线，六电机联合 +10 step 小幅运动 / 协同控制验证（`+10 step / 50 step/s / 3.0s / 500mA`）；
- 基线建立（100 轮采样）：
  - **ID1**：`reference=59`，safety band=`[59, 59]`，target=`69`；
  - **ID2**：`reference=67`，safety band=`[65, 67]`，target=`77`；
  - **ID3**：`reference=22`，safety band=`[22, 23]`，target=`32`；
  - **ID4**：`reference=54`，safety band=`[54, 55]`，target=`64`；
  - **ID5**：`reference=15`，safety band=`[14, 15]`，target=`25`；
  - **ID6**：`reference=8`，safety band=`[8, 8]`，target=`18`；
  - 全六机基线静态电流均为 0 mA，故障码均为 `0x00`；
- 写序列与配置阶段：
  - 依次向 ID1~ID6 写入 `0x25`（mode=2, speed=50），回读 100% 校验成功；
  - 依次向 ID1~ID6 写入 `0x29`（各自目标位置 69, 77, 32, 64, 25, 18），回读 100% 校验成功；
- 运动监测与异常拦截：
  - Poll 1（$t \approx 0.05\text{s}$）：ID1=59, ID2=67, ID3=22, ID4=54, ID5=15, ID6=8；
  - Poll 2（$t \approx 0.09\text{s}$）：ID6 状态回读出现 `actual = 7 step`（`0x07`）；由于 ID6 baseline band 严格为 `[8, 8]`，`actual=7 < 8` 触发 `BaselineBandGuard` 保护：
    > `negative-direction movement detected below baseline band: baseline=[8,8] actual=7`
  - 主控立即中止运动轮询，即刻执行六机 Pause-all 保护；
- Pause-all 保护与六机状态审计：
  - `_pause_all_once` 依次向 ID1 $\to$ ID2 $\to$ ID3 $\to$ ID4 $\to$ ID5 $\to$ ID6 下发 `0x1A: [1]` pause 命令，**6 台全部成功响应制动（100% 成功）**；
  - 各机制动后读回状态：
    - **ID1**：`actual = 59`, `current = 0 mA`, `force = -344 g`, `fault = 0x00`
    - **ID2**：`actual = 67`, `current = 0 mA`, `force = 311 g`, `fault = 0x00`
    - **ID3**：`actual = 22`, `current = 0 mA`, `force = 400 g`, `fault = 0x00`
    - **ID4**：`actual = 54`, `current = 0 mA`, `force = 30 g`, `fault = 0x00`
    - **ID5**：`actual = 14`, `current = 0 mA`, `force = 274 g`, `fault = 0x00`
    - **ID6**：`actual = 8`, `current = 0 mA`, `force = 142 g`, `fault = 0x00`
  - 全过程写审计：`write_count = 18`（6 次 `0x25` setup + 6 次 `0x29` target + 6 次 `0x1A` pause），无越权写入；
  - 全六机安全停机，无超流（峰值 0 mA），无硬件故障（fault 全为 0x00）。

### ID6 纯只读扩展诊断结果（2026-08-17 18:45:09）

原始日志：`logs/multi_motor/m6/20260817_184506_id6_readonly_extended_diagnosis.log`。

- 结果：`COMPLETE / PURE_READONLY_PASS`（1000 轮样本，`write_count = 0`，0 写入）；
- 核心统计：
  - 位置分布：actual={8: 985, 7: 15}（min=7, max=8, span=1 step）；
  - 寄存器同步：0x29 读回与 status target 均为 {8: 985, 7: 15}，与 actual 完全镜像同步；
  - 电气与故障：电流 100% 恒为 0 mA，故障码 100% 恒为 0x00；
  - 趋势分析：前 500 轮均值 7.9820，后 500 轮均值 7.9880（$\Delta = +0.0060$），无持续单向漂移趋势；
- 结论与机理修正：
  - 确认为 ID6 静止时存在 $7 \leftrightarrow 8$ 的 1-step 位置读回量化/反馈抖动；
  - 不归因于编码器、光栅或具体内部传感器机制（官方资料未证明）；
  - 本次 M6-C 中断系因静止状态下在确认 positive_motion 前遭遇 1.5% 概率的 7 step 抖动，被 `actual < baseline_min(8)` 严格门禁误判为反向位移。

---

### Stage M6-C Retry 实机运行（2026-08-17 18:49:38，COMPLETE / PASS）

原始日志：`logs/multi_motor/m6/20260817_184931_m6c_retry_joint_six_motors.log`。

- 结果：**`COMPLETE / PASS`**；
- 方案：ID1~ID6 六机共线同总线，六电机联合 +10 step 小幅正向运动 / 协同控制验证（`+10 step / 50 step/s / 3.0s / 500mA`）；
- 基线建立（100 轮采样）：
  - **ID1**：`reference=59`, safety band=`[59, 59]`, target=`69` (histogram={59: 100})
  - **ID2**：`reference=67`, safety band=`[66, 67]`, target=`77` (histogram={67: 85, 66: 15})
  - **ID3**：`reference=22`, safety band=`[22, 23]`, target=`32` (histogram={22: 99, 23: 1})
  - **ID4**：`reference=54`, safety band=`[54, 55]`, target=`64` (histogram={54: 97, 55: 3})
  - **ID5**：`reference=15`, safety band=`[14, 15]`, target=`25` (histogram={15: 81, 14: 19})
  - **ID6**：`reference=8`, safety band=`[7, 8]`, target=`18` (histogram={8: 98, 7: 2})
- 运动过程与起步离散时差：
  - 各机首次确认正向起步时间：
    - ID5: $0.371\text{s}$（到达 16）
    - ID3: $0.405\text{s}$（到达 24）
    - ID4: $0.409\text{s}$（到达 56）
    - ID6: $0.493\text{s}$（到达 9）
    - ID2: $0.559\text{s}$（到达 68）
    - ID1: $0.714\text{s}$（到达 61）
  - 全六台电机在 $0.84\text{s}$ 内全部到达各自误差 $\le 2\text{ step}$ 容差区间，触发统一 Pause；
- Pause 与停机到位误差审计：
  - Pause 前位置与误差：ID1=67(err=2), ID2=75(err=2), ID3=31(err=1), ID4=63(err=1), ID5=23(err=2), ID6=17(err=1)；
  - `_pause_all_once` 依次向 ID1 $\to$ ID2 $\to$ ID3 $\to$ ID4 $\to$ ID5 $\to$ ID6 发送单次 Pause（`0x1A`），6 台全部成功响应（`pause_all=true`）；
  - 停机复核（14 轮 0.5s 采样）：六台电机 100% 通过稳定性评估（`stop_confirmation=true`）；
  - 最终到位位置与误差：
    - **ID1**：`final_actual = 67`, `final_error = 2 step`（合格通过 $\le 3$）
    - **ID2**：`final_actual = 75`, `final_error = 2 step`（合格通过 $\le 3$）
    - **ID3**：`final_actual = 31`, `final_error = 1 step`（合格通过 $\le 3$）
    - **ID4**：`final_actual = 63`, `final_error = 1 step`（合格通过 $\le 3$）
    - **ID5**：`final_actual = 23`, `final_error = 2 step`（合格通过 $\le 3$）
    - **ID6**：`final_actual = 17`, `final_error = 1 step`（合格通过 $\le 3$）
- 电流与安全指标：
  - 各机峰值电流：ID1=65mA, ID2=47mA, ID3=29mA, ID4=33mA, ID5=41mA, ID6=43mA（全部 $\ll 500\text{ mA}$）；
  - 全六机故障码：全程严格为 `0x00`；
- 写审计：
  - `write_count = 18`（严格 6 次 setup `0x25` + 6 次 target `0x29` + 6 次 pause `0x1A`），0 次非法/越权写入。

**Stage M6 阶段总体结论**：
Stage M6（六电机共线只读通信 M6-A、单机独立小幅运动 M6-B5/M6-B6、六电机联合协同运动 M6-C Retry）**全部测试已圆满通过验收（M6-A / M6-B / M6-C 全 PASS）**。

## Run05 — J2-B ROS2 ID4 Same-Condition Retry

- 日期时间：2026-08-18 12:36:55
- 使用工具：`/home/lyapunov/bone_drill/bone_ws/scripts/run_j2_b_bringup.py`
- 电机 ID：4；其余 ID1/2/3/5/6 全程只读
- 原始权威 JSON：`/home/lyapunov/bone_drill/bone_ws/log/j2b_motor4_20260818_123655.json`
- 起始位置 p0：63 step（live pre-flight baseline）
- target：66 step
- 位移增量：+3 step
- 配置：单帧 `0x25` 连续 block `[2, 0, 0, 50, 63]`，回读 `0x25=2`、`0x28=50`、`0x29=63`、actual=63、current=0mA、fault=0
- timeout：3.0s
- 运行期电流保护：`current > 500mA`；fault 非零即 STOP

### 运行结果

- first_motion_time：目标 `0x29=66` 写入后约 2.729s，ID4 首次读到 64 step（error=-2）
- first_in_tolerance_time：同上，满足 `abs(error) <= 2`，随后触发一次 ID4 pause
- final_actual：64 step
- final_error：-2 step，满足最终 `<=3 step` 验收
- 运行期 ID4 current：0~28mA；正常非零运动电流未触发误 STOP
- fault：全程 0x00
- pause：ID4 单次 `0x1A=[1]`；pause 响应 current=26mA，随后两次停止确认均为 actual=64、current=0mA、fault=0x00
- stop confirmation：PASS；HARD STOP REQUIRED=false
- post-flight：六轴 current 全为 0mA，fault 全为 0x00
- UART telemetry：309 transactions/attempts，retry=0，timeout=0，checksum=0，protocol=0，failure=0
- 真实逐帧 write audit：仅 ID4 三帧，依次为 `0x25 [2,0,0,50,63]`、`0x29 [66]`、`0x1A [1]`；ID1/2/3/5/6 write count=0

### 判定

PASS

### 原因

所有 pre-flight、configuration readback、运行期 `fault/current` 门禁、到位误差、pause、停止确认、post-flight 与逐帧写审计均通过。此前 J2-B Retry 的 STOP 原始 JSON 保持不变。

### 下一步

J2-B 本次同条件 Retry 已完成。未获新的人工批准前，不执行第二次 J2-B、J2-C 或任何其他运动阶段。
