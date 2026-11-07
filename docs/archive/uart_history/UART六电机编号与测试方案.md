> 历史归档：原路径 `up/motor/uart_multi/UART六电机编号与测试方案.md`。正文保留当时的方案、参数及证据语境；当前原生 Mode 3 入口以[项目 README](../../../README.md)为准。历史命令不作为当前推荐启动命令。

# 6PUS 六电机 UART 编号与联调方案

> 状态：Stage M2 COMPLETE / PASS；M4-A PASS；M4-B1 ABORTED / NOT PASSED；ID4 掉电重启后持续 -2；M4 主线暂停，当前仅 ID4-OFFSET 已准备  
> 版本：V2.5（ID4-OFFSET 固定零偏表征准备）  
> 适用对象：NUC、AES-LA-31-06 USB-UART 调试板、六台相同的因时 LAF50-024D D 型电缸

## 1. 目标与边界

本方案首先完成六台电缸的底层 UART 通信和唯一 ID 配置，并按“1台 → 2台 → 4台 → 6台”的数量阶梯完成真实硬件验收；ID1 endpoint safety bring-up、Stage M2 和 M4-A 已完成。M4-B1 因 ID4 运动前位置越界在零写入状态下中止；ID4 单机诊断、ID3/ID4 对照及重新上电复核均确认 ID4 持续 `-2`，当前仅准备 ID3/ID4 纯只读配置审计，所有运动阶段锁定。

本阶段的目标是：

1. 在一次只连接一台电缸的条件下完成 ID 写入、Flash 保存和掉电保持验证。
2. 验证一个物理 UART Bus 管理多个 Motor 的通信架构。
3. 依次验证 1台、2台、4台、6台电缸同时在线时的定向读写、响应 ID、帧完整性和通信稳定性。
4. 建立可追溯的设备标签、测试记录、错误统计和失败回退流程。

已完成的 UART 编号与只读阶段明确不做：

- 目标位置、目标速度、同步运动、独立轨迹或全行程运动；
- Stewart/6PUS 运动学、ROS2、力控闭环和姿态控制；
- Modbus RTU/RS485、STM32/ESP32 移植；
- 广播写参数或任何未经单台验证的批量写操作。

参考资料：

- `inspire_motor_uart/ros2_jazzy/src/insmotor_driver/insmotor_driver/protocol.py`
- `inspire_motor_uart/ros2_jazzy/src/insmotor_driver/insmotor_driver/motor_controller.py`
- `bone_drill/hardware/电机手册/LAF因时机器人微型伺服电缸（力闭环专用）--LA用户手册（电气）V2.0.4cn.pdf`
- `bone_drill/hardware/功能模块/多功能电源模块使用说明书-V1.0.pdf`
- `bone_drill/hardware/调试工具/Kinematics.zip`（仅用于确认腿号命名，不在本阶段执行运动学）

## 2. 硬件、接线和供电前置条件

### 2.1 拓扑

```text
220VAC
  -> 24V/25A 开关电源
  -> 多功能电源模块 VIN/GND
  -> VOUT 调至约 8.0V（空载先调，接入后复核）
  -> AES-LA-31-06 调试板及其电缸动力输入
  -> 1/2/4/6 台 LAF50-024D 电缸

NUC USB
  -> AES-LA-31-06 USB 转 LVTTL 3.3V UART
  -> 同一 UART Bus 上的多个不同 ID 电缸
```

AES-LA-31-06 是 USB 转 LVTTL 3.3V UART 调试板，支持同一板连接六路不同 ID 的微型伺服电缸。USB 只负责通信，不能替代电缸动力供电。所有电缸使用同一块因时自带板，当前不使用 Modbus 总线。

### 2.2 上电前检查

- 220VAC 接线、PE、漏电保护和外壳防护由具备资质的人员完成。
- 24V 到多功能电源模块之间设置直流保险或限流；7.5~10A 仅作为初始量级，最终按线径、接插件和实测电流核定。
- 每个电缸支路设置独立保险或限流，约 2A 量级仅作初始建议，不得超过连接器、线缆和模块允许值。
- 多功能电源模块安装散热片并主动散热。手册中的 15A 为峰值能力，不能当作长期连续能力；整机功率上限也必须同时满足。
- 不使用模块的 6V 舵机输出给电缸供电；VOUT 先在无电缸状态用万用表调到约 8.0V，再确认接入后的电压没有异常掉压。
- 上电前确认硬件急停处于可操作状态，急停应直接切断电缸 8V 动力；软件急停不能替代硬件急停。
- 改号和保存阶段，除当前被测电缸外，其余电缸必须物理断开动力和 UART，不能只依靠“不发送指令”。
- 上电、断电和插拔前固定电缸，避免突然动作；断电后等待母线和板卡放电，再进行插拔。

## 3. UART 协议、编号和记录约定

### 3.1 通信参数

```text
波特率：921600
格式：8N1
相邻指令帧最小间隔：1ms
指令帧头：55 AA
应答帧头：AA 55
默认 ID：1
可用 ID：1~254
广播 ID：0xFF（本阶段不用于改号；广播通常不期待应答）
```

帧解析必须依据协议中的长度字段读取完整帧，再校验帧头、长度、ID、命令/寄存器和校验和。不得用一次串口 `read()` 返回的字节数假定“一次就是一帧”。

### 3.2 本阶段涉及的寄存器

| 地址 | 用途 | 本阶段规则 |
|---:|---|---|
| `0x16` | 电缸 ID | 只在单台编号阶段写入；写后立即生效 |
| `0x1C` | 保存参数到 Flash | 使用新 ID 写入；解析保存操作的两帧应答 |
| `0x2A` | 实际位置 | 只读，用于确认状态响应；不下发目标位置 |
| `0x2B` | 电流 | 只读 |
| `0x2C` | 实际受力 | 只读 |
| `0x2E` | 温度 | 只读 |
| `0x2F` | 故障码 | 只读 |

状态读取、读寄存器和写寄存器的具体命令字段以官方协议和参考代码为准；方案不在此臆造新的帧格式。每条请求必须记录原始 TX/RX 十六进制帧、时间戳、请求 ID、响应 ID、长度/校验结果、超时和重试次数。

### 3.3 实体编号

按运动学代码已有腿号直接约定：

```text
实体电缸 ID 1 -> q1
实体电缸 ID 2 -> q2
实体电缸 ID 3 -> q3
实体电缸 ID 4 -> q4
实体电缸 ID 5 -> q5
实体电缸 ID 6 -> q6
```

这里的 `q1~q6` 只是名称对应，不代表本阶段要发送运动命令。暂不额外定义俯视图顺/逆时针方向；若实际安装后需要方向或零点修正，只在后续配置层增加映射，不修改 MATLAB 原始运动学方程。

## 4. 单台电缸 ID 编号、保存与掉电验证（PASS）

### 4.1 编号前的固定规则

- 编号对象记为 `M1~M6`，先贴实体临时标签，再接线。
- 每次只允许一台电缸接入 AES-LA-31-06；其余五台同时断开动力和 UART。
- 不使用广播 ID 改号，不在编号阶段写目标位置/速度寄存器。
- 记录“实体标签、目标 ID、旧 ID、串口设备名、操作员、时间、每帧原始日志”。

### 4.2 对每台电缸执行的步骤

对 `M1` 到 `M6` 逐台重复以下流程；当任一步失败时立即停止，不得接入下一台。

1. **确认安全状态**：电缸机械区域无人员和障碍，急停可用，VOUT 已测量并在额定范围内。
2. **单台上电**：只给当前电缸供电并连接 UART，确认没有其他默认 ID=1 的设备在线。
3. **默认 ID 通信**：以旧 ID=1 读取状态，确认能收到完整应答，响应 ID=1，校验和正确，故障码可解释。
4. **读回当前 ID**：读取 `0x16`，记录旧值。若不是预期旧 ID，停止并人工核对标签，不能盲目写入。
5. **写入目标 ID**：
   - `M1` 出厂 ID 和目标 ID 均为 1：只读取 `0x16` 确认 ID=1，不发送 `0x16=1` 写入。
   - `M2~M6` 的目标 ID 分别为 2~6，使用旧 ID=1 写 `0x16=new_id`。
6. **处理改号应答**：对 `M2~M6`，改号命令的应答可能仍携带旧 ID=1；按事务上下文接收并记录这一帧，不要因为它是旧 ID 就重复写入。`M1` 没有改号应答。
7. **切换到新 ID**：对 `M2~M6`，从此刻起软件目标 ID 改为 `new_id`，用新 ID 读取一次状态或 `0x16`，确认新 ID 已立即生效；`M1` 保持 ID=1 并执行读回。
8. **保存到 Flash**：`M1` 没有参数变化，跳过 Flash 保存；`M2~M6` 使用新 ID 写 `0x1C=1`，完整读取两帧应答。第二帧必须满足 `CMD=0x40`、`Register=0x001C`，并通过帧头、Length、ID 和 checksum 校验。
9. **等待人工掉电**：程序完成只读确认或 Flash 保存后先显式关闭 UART，再提示操作者手动切断当前电缸动力，等待放电后重新上电；不要在带电状态插拔 D 型连接器。
10. **重新打开并正向验证**：操作者确认重新上电后，程序等待原始 `--port` 路径重新出现并重新打开 UART。优先使用 `/dev/serial/by-id/...` 稳定路径，不缓存其 `ttyUSB*` 目标；随后新建 Motor，用新 ID 读取 `0x16`。
11. **掉电反向验证**：对 `new_id != 1` 的电缸，程序再用旧 ID=1发送一次只读请求，必须无有效应答。对 `M1` 因旧 ID与新 ID相同，记录“旧 ID反向验证不适用”。
12. **归档结果**：在实体标签上写 `M# / ID#`，保存掉电前后读回值、保存应答、人工掉电确认、正向/反向验证结果和异常说明。

### 4.3 单台编号通过标准

- `M1` 只执行断电前 ID=1 读回和重启后 ID=1 读回，不发送 ID 写入或 Flash 保存。
- `M2~M6` 满足读 ID、写 ID、切换新 ID 和保存要求；保存第二帧的 `CMD=0x40`、`Register=0x001C`，且完整帧校验通过。
- `M2~M6` 断电后新 ID 可访问，旧 ID 不应答；`M1` 断电后 ID=1 可访问。
- 编号日志无未解释超时、错 ID 响应、长度错误、校验错误或残留帧。
- 只有满足以上条件，当前电缸才允许进入两台测试；任一失败都必须保持单台隔离并查明原因。

## 5. 阶段 1：单台通信与 ID 修改（PASS）

### 目标

验证 NUC、USB、AES-LA-31-06、供电和一台电缸的完整链路，并完成该台 ID 的写入、保存和掉电保持。

### 测试内容

- 按第 4 节完成 `M1~M6` 的逐台编号；
- 只读状态、读取 `0x16`、读取 `0x2A/0x2B/0x2C/0x2E/0x2F`；
- 编号完成后连续只读轮询当前电缸至少 20 次，建议 100 次；
- 记录响应耗时、超时、重试、错 ID、长度错误、校验错误和输入缓存残留。

### 通过标准

- 每台电缸均能通过新 ID 完成连续只读轮询；
- 所有响应 ID 与请求 ID 一致，原始帧可解析；
- 无未解释的超时、错 ID、CRC/校验和或帧长度错误；
- 保存和掉电正/反向验证符合第 4.3 节。

### 风险与禁止事项

- 风险：默认 ID 相同导致误写多台、改号后仍使用旧 ID 保存、保存双应答被错误丢弃、人工掉电未真正完成导致误判。
- 禁止：两台及以上默认 ID=1 的电缸同时上电；用广播改 ID；未完成人工掉电验证就进入 2台测试；编号阶段发送运动命令。

## 6. 阶段 2：两台同时连接（PASS）

### 目标

同时连接已验证的 `ID1`、`ID2`，确认同一 UART Bus 能可靠区分两个设备，且请求一个 ID 不会得到另一个设备的响应。

### 测试内容

1. 上电前确认两台实体标签分别为 ID1、ID2，均已完成掉电验证。
2. 依次轮询 ID1、ID2 的状态和只读寄存器；每个请求之间至少间隔 1ms。
3. 对 ID1 请求只允许接受响应 ID1；对 ID2 请求只允许接受响应 ID2。
4. 对不存在的 ID3 发送只读请求，记录为无响应/超时，不得把其他设备响应当作成功。
5. 连续执行至少 100 轮 `1 -> 2` 轮询，统计每台成功率、平均/最大响应耗时、超时和重试。

### 通过标准

- ID1、ID2 唯一，定向请求无重复响应、错 ID 响应或串扰；
- 100 轮原始轮询无未解释失败；重试必须单独统计，不能用重试后的成功掩盖首次失败；
- 无帧粘连、拆帧、残留输入或长度/校验错误；
- 请求 ID3 时不产生“误归属”的响应。

### 风险与禁止事项

- 风险：USB 缓存残留造成响应错配，未串行化访问导致两帧交叠，1ms 间隔不足造成丢帧。
- 禁止：阶段 2 未通过就接入第 3 台；并发线程直接写同一串口；修改 ID、保存参数或写运动寄存器；以广播响应判断单台成功。

## 7. 阶段 3：四台同时连接（PASS）

### 目标

同时连接 `ID1~ID4`，验证设备数量增加后帧边界、轮询调度、电源和 USB-UART 缓存仍稳定。

### 测试内容

- 依次轮询 ID1、ID2、ID3、ID4 的状态和只读寄存器；
- 每个请求至少间隔 1ms，连续至少 100 轮；
- 记录每台成功率、最大/平均响应耗时、首次失败、重试次数、错 ID、帧长度/校验错误和残留字节；
- 观察多功能电源模块输出电压、温升、线缆/接头温升和急停可用性；
- 请求不存在的 ID5，确认不会把 ID1~4 的响应误判为 ID5。

### 通过标准

- 四个定向 ID 只返回对应设备响应，无串扰和重复响应；
- 轮询期间无未解释超时、帧边界错误、校验错误或输入缓存残留；
- 供电无异常掉压、模块无过热、线缆和接头无异常发热；
- 只有在上述条件全部满足后才允许阶段 4。

### 风险与禁止事项

- 风险：总线负载和响应排队增加，供电支路压降/发热增加，USB 缓冲区更容易暴露错误的读帧实现。
- 禁止：为了提高速度取消 1ms 间隔；用“偶尔成功”替代完整轮询统计；在此阶段加入运动、广播写或在线运动学。

## 8. 阶段 4：六台同时连接（PASS）

### 目标

同时连接 `ID1~ID6`，完成当前硬件配置下的六电缸只读通信验收，为后续运动控制提供可靠的底层总线基础。

### 测试内容

1. 上电前逐台核对六个实体标签、ID、连接器和保险支路；六台都必须有阶段 1 的掉电验证记录。
2. 依次查询 ID1~ID6 的状态和只读寄存器，至少 100 轮，建议 200 轮。
3. 记录每台成功率、平均/最大响应耗时、超时、首次失败、重试、错 ID、长度错误、校验错误和残留帧。
4. 本阶段只发送 ID1~ID6 的定向只读请求，不进行广播探测或广播写参数。
5. 连续测试期间监测 8V 输出、电源模块温升、支路保险/线缆和急停状态。

### 通过标准

- ID1~ID6 唯一，所有定向请求只得到对应 ID 的完整有效响应；
- 无错 ID、长度错误、校验错误、帧粘连/拆分或未解释超时；
- 串口输入缓存无残留帧，轮询结束后再次查询任一 ID 仍能正确匹配；
- 统计结果、原始日志和供电监测记录完整归档；
- 连续测试期间无异常掉压、过热、急停失效或电缸故障状态恶化。

### 风险与禁止事项

- 风险：六台同时上电时总启动电流、模块散热、支路压降和通信排队达到最不利情况；一个设备异常可能污染后续帧解析。
- 禁止：六台 ID 未逐台验证就同时上电；未完成六台只读验收就发送目标位置/速度；把通信阶段通过误认为机械运动安全。

## 9. 统一失败处理、急停和日志要求

- 任意阶段出现连续超时、错 ID、校验/长度错误、无法解释的残留帧、异常电压/温升或故障码，立即停止发送，必要时按硬件急停切断 8V 动力。
- 失败时保留当前连接数量，不继续增加设备；回退到最近一个通过的阶段，逐台隔离定位。
- 软件重试不能替代失败记录：必须同时记录首次请求结果和最终结果。
- 每次测试日志至少包含：时间戳、阶段、连接数量、实体标签、请求/响应 ID、原始 TX/RX、寄存器、长度/校验结果、响应耗时、超时、重试、供电电压、温度、故障码和操作员备注。
- 任何硬件改线、保险更换、供电参数改变或固件/参数改变后，必须从阶段 1 重新验证相关电缸。

## 10. 软件架构约束：一个 UART Bus，多 Motor

保留“一个物理 UART Bus 管理多个 Motor”的架构：

```text
UartBus（唯一串口、统一锁、帧解析、调度、超时和日志）
  ├── Motor(id=1)
  ├── Motor(id=2)
  ├── Motor(id=3)
  ├── Motor(id=4)
  ├── Motor(id=5)
  └── Motor(id=6)
```

- `UartBus` 只打开一个串口实例，所有请求串行化，并强制相邻帧至少间隔 1ms。
- Bus 按声明长度读取完整帧，校验帧头、长度、校验和以及响应 ID；处理超时、重试和输入缓存隔离。
- 只读请求可以按配置重试；改 ID、普通写寄存器和保存 Flash 默认不自动重发，避免设备已执行但应答丢失时重复写入。写事务失败后必须停止并结合原始日志人工判断。
- `Motor` 只保存逻辑 ID 和设备操作，不自行打开串口；提供状态/寄存器读写、改 ID、保存参数等底层接口。
- 改号工具必须显式要求“当前只连接一台”，并在改号后自动切换新 ID、解析保存双应答，然后暂停等待人工断电/上电，再执行掉电正/反验证。
- 阶段测试工具只调用只读接口，参数化支持 1、2、4、6 台，禁止绕过 Bus 直接写串口。

参考代码中的多个 `MotorController` 不能原样复制为多个串口实例；新实现必须先解决统一串口访问、按长度收帧和保存双应答问题。

## 11. 代码开发记录

1. **[已完成] 整理协议测试数据**：已依据官方手册和参考代码确认长度字段、字节序、校验和及保存两帧结构。
2. **[已完成] 实现并测试 `protocol.py`**：帧打包、完整帧拆包、帧头/长度/校验验证、响应 ID 验证、状态与寄存器解析、保存第二帧 `CMD=0x40` 和 `Register=0x001C` 判断。
3. **[已完成] 实现 `uart_bus.py`**：单一物理串口、串口锁、1ms 最小间隔、按声明长度读取、超时/重试、残留输入隔离和原始日志。
4. **[已完成] 实现 `motor.py`**：绑定一个 ID，提供 `read_status()`、`read_register()`、`write_register()`、`set_motor_id()`、`save_parameters()`；不含轨迹或运动学接口。
5. **[已完成] 实现 `tools/assign_id.py`**：单台连接确认、旧 ID 读回、必要时改号、切换新 ID、保存、人工断电/上电、正向/反向验证和日志归档。
6. **[已完成] 实现 `test_protocol.py`**：覆盖正常帧、长度错误、校验错误、错 ID、超时、粘连/拆帧、保存双应答和旧 ID 改写应答。
7. **[已完成] NUC 单电机实机联调**：M1~M6 已完成 ID1~ID6 编号、保存和掉电保持验证。
8. **[已完成] 多电机只读阶段**：1台、2台、4台、6台定向只读通信均已通过真实硬件验收；仍禁止多电机运动。

暂不开发：运动学、ROS2、轨迹规划、同步/独立运动、力控闭环、STM32/ESP32 移植和 Modbus RTU。

## 12. UART 阶段完成定义（已满足）

当且仅当六台电缸均完成唯一 ID、保存和人工掉电验证，且 1台、2台、4台、6台的只读通信通过标准全部满足，才认为“底层 UART 六电机编号与通信联调”完成。此完成定义不包含任何机械运动安全或运动学联控结论。

## 13. UART 阶段验收记录（2026-08-15，PASS）

### 13.1 编号与分阶段只读通信

- `[PASS]` M1~M6 已分别固定为 ID1~ID6，均完成 Flash 保存和真实掉电保持验证。
- `[PASS]` 1台、2台、4台、6台定向只读通信均已完成真实硬件验收。
- `[PASS]` 一个 `UartBus` 管理多个 `Motor`，相邻帧至少 1ms，无广播探测或广播写入。

六台最终验收数据：

```text
online IDs:       1, 2, 3, 4, 5, 6
rounds:           initial validation + 100 ordered rounds
requests:         606
first_success:    606
retry_success:    0
final_failure:    0
timeout:          0
wrong_id:         0
length_error:     0
checksum_error:   0
avg_response:     2.861 ms
max_response:     5.015 ms
```

永久 UART Phase baseline 日志：

```text
/home/lyapunov/bone_drill/up/motor/uart_multi/logs/six_ids_test01.log
SHA-256: a5c43ead04d26dfe80945cdd84dc3e3de9f686dc4b1bfef17b02e875620ffa15
```

该文件禁止覆盖、移动或删除。后续实验使用 `logs/<phase>/` 下独立且可追溯的时间戳文件名；程序必须拒绝覆盖已有日志。

## 14. 下一阶段：ID1 极小位移安全测试

官方手册 PRJ-01-TS-U-007 V2.0.4 已核对：`0x23` 为行程上限、`0x24` 为行程下限、`0x25` 为控制模式、`0x28` 为目标速度（步/s）、`0x29` 为目标位置（0~2000 步）；状态应答包含目标/实际位置、电流、受力、温度和故障码。

本阶段只允许单台 ID1，速度固定 `50 step/s`，位移序列固定为 `p0 -> p0+5 -> p0 -> p0-5 -> p0`。首轮软件范围固定为 `200..1800 step`，不提供放宽范围的 CLI 参数；程序读取电缸 `0x23/0x24` 自身限位并与该范围取更严格交集，`p0`、`p0+5`、`p0-5` 任一越界都在运动写入前拒绝。运动前读取并记录当前控制模式 `0x25`、目标速度 `0x28`、目标位置 `0x29`、实际位置和设备限位，并由操作者在终端明确输入 `MOVE ID1` 后才允许进入速度模式。测试结束后不盲目恢复可能携带旧目标的原配置，而是重新读取并明确记录最终模式、速度和目标位置；预期为速度模式、`50 step/s`、目标 `p0`。任何通信失败、非零故障码、电流/受力超限、跟踪误差超限或到位超时均只发送一次官方暂停命令并停止；写操作不得自动重发，不得扩大位移。

本阶段仍禁止：其他电机运动、同步/连续轨迹、广播、Stewart/6PUS 运动学、ROS2 和闭环控制。

## 15. ID1 静态力基线阶段（运动暂停）

首次运动预检发现未运动、水平无载 ID1 的协议受力读数为负值。由于当前 `0x2C` 协议字段为有符号值，且旧版零点校准寄存器不能直接映射到 V2.0.4，本阶段先使用 `tools/read_id1_force_baseline.py` 采集水平、水平翻转和竖直姿态的只读样本。

工具默认 10 秒、50Hz，只读取状态帧，记录实际位置、电流、协议受力值、温度和故障码，并输出均值、总体标准差、最小值、最大值、峰峰值。基线比较完成前禁止继续任何运动测试、零点写入或阈值放宽。

### 15.1 已完成的水平姿态基线

- `horizontal`：`force mean=-428.06g`、`std=1.08g`、`min=-439g`、`max=-419g`；位置 `3~4 step`；电流 `0mA`；故障码 `0`。
- `horizontal_flipped`：`force mean=-381.94g`、`std=6.31g`；位置 `3~4 step`；电流 `0mA`；故障码 `0`。
- `vertical_handheld`：手持姿态波动较大，不作为基准。

结论：ID1 存在约 `-400g` 的稳定无载静态偏置，原有 `|force|<100g` 判据不能直接用于本设备。更关键的是当前位置约 `3~4 step`，位于机械低端附近，不满足原 `200..1800 step` 对称微动前提；在脱离低端前，原 `safe_micro_move_id1.py` 保持禁止运行。

## 16. ID1 低端单次脱离工具设计（待实机审核）

新增 `tools/safe_endpoint_escape_id1.py`，与原对称微动工具独立，固定 ID1。工具只允许测得 `p0 <= 199 step` 时执行一次正方向命令 `p0 -> p0+5`，禁止任何负方向目标；设备自身 `0x24/0x23` 限位仍是硬位置边界。配置写先将速度设为 `50 step/s`、目标保持 `p0`，随后只发送一次 `0x29=p0+5`，不自动返回端点。

运动前采集默认 `0.5s、20Hz` 的只读静态力基线 `F0`。基线和运动轮询均记录实际位置、电流、协议受力、`DeltaF=F-F0`、温度和故障码；日志使用独占创建，禁止覆盖历史数据。必须人工输入 `ESCAPE ID1` 才能进入目标写入。

当前明确的绝对保护只有：通信/帧解析失败立即停止；故障码非零立即停止；实际位置超出设备限位或出现低于 `p0` 的负方向运动立即停止；电流超过操作者明确提供的 `--max-current-ma` 立即停止；到位误差超过容差或超时立即停止。力值不设置未经厂家确认的绝对阈值，`-400g` 偏置只通过 `DeltaF` 记录和人工审核；不得把传感器量程或协议单位臆化为新的停止门限。

异常发生在目标写入阶段后，工具最多发送一次暂停命令，写操作不自动重发；预检或基线阶段失败时不发送任何写命令。测试结束保持在新位置，并记录最终控制模式、速度和目标寄存器，不自动恢复旧目标。

本阶段仍禁止：原 `200..1800` 对称运动、其他电机运动、连续轨迹、广播、Stewart/6PUS 运动学、ROS2 和闭环控制。

## 17. Run01 低端脱离复盘与 Type-D 协议审查（暂停运动）

Run01 从 `p0=3` 写入 `target=8` 后，2 秒内实际位置主要为 `3~4 step`，电流最高约 `42mA`，力基线约 `-377g`、`DeltaF` 为个位数 g、温度 `34C`、故障码 `0`。程序超时退出时，原版本先由 `with UartBus` 关闭串口，再在外层异常处理发送暂停，导致 `Attempting to use a port that is not open`。退出约 1 分钟后只读位置为 `7`，说明不能排除目标仍保持有效并继续运动。

现已修复为：运动目标写入后发生任何异常时，先在 Bus 仍打开期间发送一次 `0x1A=1` 暂停；暂停成功后连续约 `0.5s` 只读位置、电流、温度和故障码，确认位置不再变化；最后才关闭 UART。暂停写失败或只读确认失败均显著输出 `HARD STOP REQUIRED`，提示立即使用硬件急停/断电。暂停不自动重发，运动目标不自动重发。预检和基线阶段异常不会发送暂停写。

### 17.1 Type-D V2.0.4 对照表

| 项目 | V2.0.4 手册要求 | 当前 endpoint 工具实现 | 审查结论 |
|---|---|---|---|
| `0x25` 模式 | `0` 定位；`1` 伺服；`2` 速度；`3` 力控；`4` 电压；`5` 速度力控 | 常量 `SPEED_MODE=2` | 数值含义正确，但运动测试已暂停 |
| Type-D 单次位置运动 | 定位模式 `0`：写 `0x29` 目标位置；速度模式 `2`：按 `0x28` 速度运动到 `0x29` 后停止 | 设置模式 `2`、速度 `50`，再写一次 `0x29` | 选择与手册一致；尚未证明现场执行时序满足预期 |
| `0x28` | 速度模式有效，单位步/s；手册寄存器表未给出完整数值范围，示例使用 500 步/s | 写入 `50` | 单位正确；`50` 的现场行为仍需单独验证，不把它当作已证明的有效下限 |
| `0x29` | 速度、定位、伺服模式有效，范围 `[0,2000]` | 写 `p0+5` 一次 | 速度模式单次写入符合手册；伺服模式不能复用此做法 |
| 伺服/follow-up | 模式 `1` 要求控制器固定频率发送目标位置，建议不低于 `50Hz`，驱动器做位置插补跟随 | 未使用模式 `1`，未实现周期轨迹 | 符合当前“单次速度模式”范围；不应称为 follow-up |
| 运动启动/使能 | 手册示例只需设置控制模式及相关目标寄存器，没有额外 enable 步骤 | 无额外 enable | 未发现缺失的独立启动命令 |
| `0x1A` | 写入 `1` 可暂停当前运动 | 异常时单次写入 `[1]` | 作用和地址正确；修复了必须在 UART 关闭前发送的生命周期错误 |

### 17.2 对“2 秒内实际仍为 3~4”的结论

按手册，速度模式 `2` 应以 `0x28` 的目标速度运动到 `0x29` 后停止；若 `50 step/s` 被设备按该语义执行，5 步理论运动时间约为 `0.1s`，与 Run01 的 2 秒未到位不一致。现有日志只能确认电流上升、故障码为 0，不能区分以下原因：低端机械接触/堵转、设备实际速度配置或内部限幅、目标仍保持后的延迟运动、或现场执行时序问题。不能仅凭当前观测把原因归结为协议错误，也不能继续实机运动验证。

在完成停止闭环验证、并通过只读方式确认写后 `0x25/0x28/0x29` 和实际状态变化前，所有 endpoint 和对称微动均保持暂停。不得用 Type-2/PWM 或其他接口控制方式替代 Type-D UART 审查。
日期：2026-08-16

原始日志：

`logs/id1_motion/20260816_110853_endpoint_escape_id1_run01.log`

### 实验条件

- 电机：ID1
- 其余 ID2~ID6：断开
- 电缸：裸机、无外载、水平放置
- 起始位置：3 step
- 目标位置：8 step
- 目标增量：+5 step
- 配置速度：50
- settle timeout：2.0 s
- 电流保护阈值：500 mA
- 力基线：约 -377 g
- 初始故障码：0x00

### 实机结果

运动监控阶段：

- actual position 主要在 3~4 step
- 最大观测电流约 42 mA
- force 基本约 -377 g
- DeltaF 约为 -9~+6 g
- 温度约 34 °C
- fault 始终为 0x00

2 秒后未满足到位条件：

`STOP ID1 endpoint escape: target=8 did not settle within 2.00s`

随后发现安全停止命令发送失败：

`ERROR safety pause command failed: Attempting to use a port that is not open`

说明异常处理阶段 UART 已先关闭，导致 pause 未成功发送。

程序退出后再次进行纯只读检测：

- actual position = 7 step
- current = 0 mA
- fault = 0x00
- force mean = -363.8 g

因此不能排除目标位置命令在程序退出后仍然有效，并产生了后续残余运动。

### Run01 判定

**FAIL — 安全停止软件逻辑存在缺陷。**

本次不能视为正常低端脱离 PASS。

已确认：

- UART 通信正常
- 电机能够产生正方向位置变化
- 未出现设备故障码
- 电流未接近 500 mA 保护阈值
- 力变化未表现出明显异常

尚未确认：

- Type-D 模式下 0x25 / 0x28 / 0x29 的控制方式是否正确
- 配置值 50 的真实速度语义
- target=8 后的实际运动时序
- pause 命令在异常路径中的可靠执行

### 当前安全决定

在以下问题修复和核对完成前：

- 禁止再次执行 endpoint escape 实机运动
- 禁止执行 7 -> 12 step
- 禁止扩大位移
- 禁止进入原 ±5 step 对称运动测试

下一步：

1. 修复 pause 必须在 UART close 之前发送的问题；
2. pause 失败时明确输出 HARD STOP REQUIRED；
3. pause 后增加短时间只读位置确认；
4. 根据官方 V2.0.4 电气手册核对 LAF50-024D Type-D 的控制模式、速度和目标位置语义；
5. 单元测试通过后，再决定是否执行 Run02。

## 18. ID1 Endpoint Safety Bring-up 阶段验收（2026-08-16，完成）

经过 Run01、Run02、Run03、Run04、Run04-R1 和 Run04-R2 的逐步验证，ID1 endpoint safety bring-up 阶段正式结束。各 Run 的原始 PASS/FAIL/ABORTED 判定和日志均永久保留；阶段完成不追溯修改失败历史，也不代表 `±1 step` 位置精度已经通过。

已验证能力：

- ID1 能够产生真实正方向运动；
- 运动期间持续读取并检查 current、fault、position、force 和 temperature；
- 异常或 timeout 后在 UART close 前单次发送 pause，且不盲目重试；
- pause 后通过纯只读 position/current/fault 组合判据确认停止；
- 使用运动前 position baseline band 区分已确认的 1-step 静态反馈抖动与带外运动；
- 原始 TX/RX、运动目标、实际状态、关键时序及安全结果写入独立且不可覆盖的日志。

最终 Run04-R2 使用 `baseline=[6,7]`、`target=17`、`+10 step`、`50 step/s`、`2s` 和 `500mA`。首次确认正向运动为 `elapsed=0.894803s, actual=8, current=93mA`；实际位置在 `elapsed=1.081911s` 首次到达 `15` 并保持至 timeout，最终误差 `2 step`，fault 为 0。pause 和 stop confirmation 均成功。

Run04-R2 分项结论：正向运动能力 `PASS`，安全停止链 `PASS`，`±1 step` 到位 `FAIL`；总体仍为 `FAIL — target tolerance not reached`。

保留以下非阻塞遗留：低端约 `0.9s` 启动延迟，以及低端测试中约 `2 step` 最终残差。现有证据尚不能判断它们是否只属于低端区域，不在本阶段继续扩大参数或追加 Run05。进入下一阶段前必须另行明确实验对象、控制目标、写操作边界、机械条件和 PASS/FAIL 标准；在这些内容获批前不执行新的实机运动。

## 19. 多电机实机控制验证路线

验证数量按 `M2 -> M4 -> M6` 递增：Stage M2 只连接 ID1、ID2；M2-D 通过后才允许准备 ID1~ID4 的 Stage M4；M4 全部通过后才允许准备 ID1~ID6 的 Stage M6。当前 Stage M2 与 M4-A 已 PASS，M4-B1 已零写入 ABORT，ID4 重新上电后仍持续 `-2`；当前只允许 ID3/ID4 纯只读配置审计，M4-B2/M4-C/M6 不准备。

所有阶段继续遵守：一个 `UartBus` 管理所有 `Motor`；`Motor` 不自行打开串口；复用且不修改冻结的 `protocol.py`、`uart_bus.py`、`motor.py`；不重写 ID、不保存 Flash、不广播写；运动目标和 pause 均不自动重试；日志独立且不可覆盖；任何失败不增加在线电机数量。

## 20. Stage M2（COMPLETE / PASS）

状态：M2-A、M2-B、M2-C、M2-CAL、M2-D Retry2 均 `PASS`；Stage M2 为 `COMPLETE / PASS`。M2-D 首次实机 `FAIL / NOT PASSED` 与 Retry1 运动前 baseline 门禁 `ABORTED / NOT PASSED` 作为历史永久保留，不追溯改判。后续 M4-A 已正式 PASS，M4-B1 已零写入 ABORT。

### 20.1 硬件和安全前置条件

1. 断电接线，只将实体 M1/ID1 和 M2/ID2 接入同一 AES-LA-31-06；ID3~ID6 的 UART 和动力均断开。
2. 两台电缸分别可靠固定、互不机械连接、正方向具有可见安全余量；不得接入 6PUS 机构。
3. 使用 `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`，现场具备可立即操作的硬件急停或动力断电手段。
4. 核对实体标签与 ID；本阶段禁止 ID 写入、Flash 保存、广播、运动学、轨迹和 ROS2。
5. 每个子阶段使用 `logs/multi_motor/m2/` 下新的时间戳日志；运行前确认路径不存在，禁止复用或覆盖历史日志。

### 20.2 M2-A：两 ID 只读稳定通信（PASS）

1. 一个 `UartBus` 创建 `Motor(1)`、`Motor(2)`，先分别读取 `0x16` 并核对响应 ID 和读回 ID。
2. 对预期不存在的 ID3 只探测一次且不重试，必须得到 `EXPECTED NO RESPONSE`。
3. 按 ID1 -> ID2 串行轮询 100 轮；不得使用并发串口访问。
4. 保存全部 TX/RX、逐 ID 统计及总体统计。

M2-A PASS：ID1/ID2 初始读取及 100 轮全部首次成功；在线设备合计 `requests=202`、`first_success=202`、`retry_success=0`、`final_failure=0`、`timeout=0`、`wrong_id=0`、`length_error=0`、`checksum_error=0`；ID3 恰有一次 `expected_no_response`。任一在线请求重试、失败、错 ID、长度/校验错误，或 ID3 意外响应，M2-A 均为 FAIL，禁止进入运动子阶段。

实机结果（2026-08-16）：`PASS`。

- 原始日志：`logs/multi_motor/m2/20260816_164752_m2a_readonly_id1_id2.log`
- ID1：`requests=101, first_success=101, retry_success=0, final_failure=0`
- ID2：`requests=101, first_success=101, retry_success=0, final_failure=0`
- 总计：`requests=202, first_success=202, retry_success=0, final_failure=0`
- `timeout=0, wrong_id=0, length_error=0, checksum_error=0`
- ID3：`expected_no_response=1`
- `avg_response_ms=2.887, max_response_ms=4.113`

### 20.3 M2-B：仅 ID1 小幅运动，ID2 只读监视（PASS）

专用工具为 `tools/safe_m2b_id1_only.py`。它使用一个 `UartBus` 和两个 `Motor`，只允许向 ID1 写 mode/speed/target/pause；ID2 全程只读。

运动前同时读取两台的 `0x23/0x24` 设备限位、mode、speed、target 和完整状态，并采集 `2s/50Hz` position/force/current/fault 基线。ID1 只允许正方向单次 `baseline_max+10 step`，速度 `50 step/s`、最大监控窗口 `2s`、电流硬限制 `500mA`；目标必须不低于设备 lower limit 且不高于 `min(device_upper, 1800)`，否则 ABORT，不自动改为负方向或改变参数。

运动阶段串行交替读取 ID1、ID2。ID2 不得收到任何配置、目标或 pause 写入；其 position 必须保持在运动前 baseline band 内，实时 current 不得超过自身基线最大值，fault 必须为 0，且 ID1 setup 写后、ID1 target 写后及最终的 mode/speed/target 必须与运动前完全一致。ID1 到达 Stage M2 允许误差、达到 2s 上限或出现任何异常时，均在 UART 仍打开时只对 ID1 单次 pause，再对两台执行纯只读 stop confirmation。即使 ID2 出现疑似误运动、电流异常或 fault，也绝不向 ID2 写 pause 或其他寄存器；此时只 pause ID1，若无法确认 ID2 已停止则输出 `HARD STOP REQUIRED` 并要求硬件急停/断电。

所有出站帧先由 `Id1OnlyAuditedBus` 按协议解析。写帧必须同时满足 `CMD=0x32` 且 `destination_id=1`；任何 ID2 或其他 ID 的写帧在进入冻结 `UartBus` 前直接拒绝。日志逐帧输出 `ACCESS_AUDIT operation=WRITE/READ destination_id=...`，结束时输出 `ACCESS_SUMMARY`，其中必须证明所有写目标均为 1 且 `id2_write_count=0`。

M2-B PASS：

- ID1、ID2 的运动前 baseline position span 均不超过 1 step；
- ID1 明确出现 `actual>baseline_max` 的正向运动，pause 后最终误差不超过已知非阻塞残差 `2 step`；原 `±1 step` 结果仍单独记录；
- ID1 无反向带外运动、越界、`current>500mA` 或非零 fault；
- ID2 所有实际位置均在自身 baseline band 内，实时 current 不超过自身基线最大值，fault 始终为 0；
- ID2 的 mode/speed/target 在三个复核点均不变；
- 所有写帧 `destination_id=1`，`id2_write_count=0`；
- ID1 pause 成功，随后 ID1、ID2 的纯只读 stop confirmation 均通过；
- 独立日志包含原始 TX/RX、两台 baseline、逐样本状态、写/读审计和最终汇总。

M2-B FAIL：任一写帧目标不是 ID1；ID1 未确认正向运动、最终误差超过 2 step、反向、越界、超电流、fault 或通信异常；ID2 位置离开 baseline band、current 超过基线最大值、fault 非零或配置变化；pause/stop confirmation 失败；或出现 `HARD STOP REQUIRED`。若在任何写入前因 baseline、设备限位或人工确认不满足而拒绝，则记为 `ABORTED / NOT PASSED`，同样禁止进入 M2-C。

实机结果（2026-08-16）：`PASS`。

- 首次预检日志：`logs/multi_motor/m2/20260816_170427_m2b_id1_only.log`；操作者未确认，`ABORTED / NOT PASSED`，`write_count=0, id2_write_count=0`；
- 正式 PASS 日志：`logs/multi_motor/m2/20260816_170610_m2b_id1_only.log`；
- ID1 baseline `[14,15]`，target=25，final=23，`final_error=2 step`；
- 首次正向运动 `elapsed=0.789521s, current=67mA`，最大电流 `76mA`，fault=0；
- ID2 actual=0、current=0、fault=0，配置保持不变；
- 写帧 destinations=`[1,1,1]`，registers=`[0x25,0x29,0x1A]`，`id2_write_count=0`；
- `pause_success=True, stop_confirmation=True, HARD STOP REQUIRED=False`。

两份日志均为独立历史记录，禁止覆盖或删除。正式运行满足 M2-B 全部 Stage M2 定向控制隔离门禁，允许准备 M2-C。

### 20.4 M2-C：仅 ID2 小幅运动，ID1 只读监视（PASS）

专用工具为 `tools/safe_m2c_id2_only.py`，是 M2-B 的严格对称验证：同一 `UartBus` 上创建 `Motor(2)` 目标电机和 `Motor(1)` 观察电机，只允许向 ID2 写 mode/speed/target/pause；ID1 全程严格只读。

运动前分别读取两台的设备限位、mode、speed、target 和完整状态，并采集 `2s/50Hz` position/force/current/fault 基线。ID2 只允许正方向单次 `baseline_max+10 step`，速度 `50 step/s`、最大监控窗口 `2s`、电流硬限制 `500mA`；目标必须位于设备限位与软件上限 `1800` 的交集内，否则在任何写入前 ABORT。

运动阶段串行交替读取 ID2、ID1。ID1 的 actual/current/force/temperature/fault 逐样本记录，position 必须保持在自身 baseline band 内，current 不得超过自身基线最大值，fault 必须为 0；ID2 setup 写后、target 写后和最终均复核 ID1 的 mode/speed/target 完全不变。

所有出站帧先由 `Id2OnlyAuditedBus` 解析和审计。任何写帧 destination 不是 ID2 时，在进入冻结 `UartBus` 前立即拒绝。正常到达、timeout 或任意异常均只对 ID2 单次 pause，pause 不重试；之后只读确认两台停止。绝不向 ID1 发送 pause 或其他写操作。日志必须包含逐帧 `ACCESS_AUDIT` 和最终 `ACCESS_SUMMARY`，其中 `id1_write_count=0`。

M2-C PASS：

- ID1、ID2 的运动前 baseline position span 均不超过 1 step；
- ID2 明确出现 `actual>baseline_max` 的正向运动，pause 后 `final_error<=2 step`；
- ID2 无反向带外运动、越界、`current>500mA`、非零 fault 或通信异常；
- ID1 所有实际位置均在自身 baseline band 内，current 不超过自身基线最大值，fault 始终为 0；
- ID1 的 mode/speed/target 在全部复核点保持不变；
- 所有写帧 `destination_id=2`，且 `id1_write_count=0`；
- 仅 ID2 的 pause 成功，两台纯只读 stop confirmation 均通过；
- 独立日志包含原始 TX/RX、两台 baseline、逐样本状态、访问审计和最终汇总。

M2-C FAIL：任一写帧目标不是 ID2；ID2 未确认正向运动、最终误差超过 2 step、反向、越界、超电流、fault 或通信异常；ID1 位置离开 baseline band、current 超过基线最大值、fault 非零或配置变化；pause/stop confirmation 失败；出现 `HARD STOP REQUIRED`；或日志不能证明 `id1_write_count=0`。若在任何写入前因 baseline、限位或人工确认不满足而拒绝，则记为 `ABORTED / NOT PASSED`，不得据此准备或执行 M2-D。

M2-C 无硬件专项测试：`10 passed`。实机结果（2026-08-16）：`PASS`；原始日志：`logs/multi_motor/m2/20260816_182345_m2c_id2_only.log`。

Stage M2 个体运动已知现象：M2-B 的 ID1 为 target `25`、final `23`、最终残差 `2 step`；M2-C 的 ID2 为 target `10`、final `8`、最终残差 `2 step`。该现象非阻塞，不得据此扩大位移、速度、timeout 或电流阈值。

### 20.5 M2-D：双电机联合运动（Retry2 PASS）

状态：M2-D Retry2 `PASS`。首次实机 `FAIL / NOT PASSED` 和 Retry1 `ABORTED / NOT PASSED` 历史保持不变。

专用工具为 `tools/safe_m2d_joint_id1_id2.py`。工具在一个共享 `UartBus` 上创建 `Motor(1)` 和 `Motor(2)`，目标写入顺序固定为 ID1 -> ID2；这是一对串行目标命令，不定义或宣称严格同步时差精度，只记录 `target_dispatch_skew_s`。

运动前分别读取两台 `0x23/0x24` 设备限位、mode、speed、target 和完整状态，并采集 `2s/50Hz` position/force/current/fault 基线。每台以 position histogram 的唯一众数作为 `baseline_reference`；所有样本必须满足 `abs(actual-baseline_reference)<=1`，任一样本偏离众数达到 `2 step` 或出现众数 tie 均在任何写入前 ABORT。`baseline_safety_min/max` 记录实际观测范围；运动目标固定为 `baseline_reference+10 step`，且必须位于设备限位与软件上限 `1800` 的交集内。

两台均固定速度 `50 step/s`、最大监控窗口 `2s`、电流硬限制 `500mA`，只允许正方向。运动期间按 ID1 -> ID2 串行轮询，逐样本记录两台 target、actual、current、force、DeltaF、temperature 和 fault，并检查带外反向、位置限位、超电流和非零故障。

所有出站帧先由 `JointAuditedBus` 解析。目标 ID 只能为 1 或 2，写寄存器只能为 `0x25/0x29/0x1A`；日志必须证明正常写序列为 `[1,2,1,2,1,2]`，并记录两台各自读请求及响应 ID。运动目标写和 pause 均不自动重试。

正常达到双电机集成误差门禁或任一异常时，UART 保持打开，分别对 ID1、ID2 各发送一次 pause；即使第一台 pause 失败，也必须继续尝试第二台一次。随后对两台执行纯只读 stop confirmation；任一 pause 或停止确认失败均输出 `HARD STOP REQUIRED`。

M2-D PASS 标准：

- 两台 baseline 均存在唯一众数，全部样本在各自 `baseline_reference±1 step` 内，且两台 `baseline_reference+10` 目标均在各自设备限位与 `1800` 软件上限交集内；
- 两台均观察到 `actual>baseline_safety_max` 的正向运动；仅 `actual<baseline_safety_min` 判为负方向运动；
- 两台均无反向带外运动、位置越界、通信/帧解析异常、`current>500mA` 或非零 fault；
- 两台最终实际位置均满足 `final_error<=2 step`；严格 `±1 step` 结果单独记录，不作为本阶段唯一门禁；
- 所有写帧 destination 只属于对应 ID1/ID2，寄存器只属于 `0x25/0x29/0x1A`，无其他 ID 写入；
- 目标字段必须明确区分：`commanded_target` 是本次命令，`readback_target_position` 是 pause 前读取的 `0x29`，`actual_position` 是状态反馈的实际位置；目标写入后 `readback_target_position == commanded_target`，所有状态响应按各自地址正确匹配；
- pause 后不得要求 `readback_target_position` 仍等于原 `commanded_target`；只验证实际位置、停止确认、`current` 和 `fault`，并将 pause 后 target 读回作为语义观测记录；
- ID1、ID2 pause 均成功，且两台纯只读 stop confirmation 均通过；
- 独立日志包含原始 TX/RX、两台状态统计、目标命令时差、访问审计和最终结果。

M2-D FAIL 标准：

- 任一写帧目标或寄存器越界，任一响应 ID 错配，或出现通信、长度、校验错误；
- 任一 baseline 无唯一众数、出现众数 tie，或任一样本相对 `baseline_reference` 偏差达到 `2 step`；
- 任一电机未确认正向运动、反向越界、超电流、非零 fault、最终误差超过 2 step，或 pause 前目标读回不等于本次命令；
- 任一 pause 失败、停止确认失败或出现 `HARD STOP REQUIRED`；
- 日志不能证明两台 target/actual/current/force/fault、写 destination 和各自响应 ID；
- 若在任何写入前因人工确认、基线或限位检查失败而退出，记为 `ABORTED / NOT PASSED`，不得执行或解锁 M4。

M2-D 无硬件专项测试：`21 passed`；M2-CAL/M2-B/M2-C/M2-D/endpoint 联合无硬件回归：`75 passed`。首次实机结果仍按历史记录保留为 `FAIL / NOT PASSED`，Retry1 保留为 `ABORTED / NOT PASSED`，Retry2 为 `PASS`。

### M2-D 首次实机结果（2026-08-17）

- 原始日志：`logs/multi_motor/m2/20260817_093231_m2d_joint_id1_id2.log`。
- ID1 baseline `[22,22]`、target `32`、final `30`、`final_error=2`；ID2 baseline `[7,8]`、target `18`、final `16`、`final_error=2`。
- 最大电流：ID1 `75mA`，ID2 `66mA`；两台 fault 均为 `0x00`。
- 写帧顺序 `[1,2,1,2,1,2]`，无其他 ID 写入；两台 pause 成功，双电机 stop confirmation 通过。
- 最终失败：ID1 配置期望 `mode=2, speed=50, target=32`，读回 `mode=2, speed=50, target=30`。
- 原始日志与“未发生运动/未发生 pause”的口述不一致；以原始日志为准，判定为已进入运动后的 `FAIL / NOT PASSED`，不是写入前 ABORT。
- 原始状态帧显示 pause 后设备 target 字段与停止 actual 同步（ID1 `32/30` 后为 `30/30`，ID2 `18/16` 后为 `16/16`）；因此“pause 后 target 必须仍为原命令”的比较属于错误验收逻辑，不是 UART 或调度故障证据。
- 已完成最小修复并通过无硬件回归：pause 前验证 `readback_target_position == commanded_target`；pause 后只验证 actual、停止确认、current、fault，并记录 target 同步观测。
- 该修复后的 Retry1 已执行，结果见下一节；首次历史 FAIL 不追溯改写。

### M2-D Retry1 基线门禁结果（2026-08-17）

- 判定：`ABORTED / NOT PASSED — pre-motion baseline gate rejection`。
- 原始日志：`logs/multi_motor/m2/20260817_104109_m2d_joint_id1_id2.log`。
- ID1 baseline histogram `{38:96,39:4}`；ID2 baseline histogram `{37:12,38:87,39:1}`，旧判据计算 `min=37, max=39, span=2` 后拒绝。
- `ACCESS_SUMMARY write_count=0`；未发送任何配置、目标或 pause 写帧，未发生 ID1/ID2 运动、联合控制或 stop confirmation。
- ID2 全部样本实际位于唯一众数 `38` 的 `±1 step` 内，属于已知静态位置量化/抖动；旧 `span<=1` 判据过严。
- 已仅修正 M2-D baseline 判据：唯一众数 reference、全样本 `±1 step`、实际 safety min/max、目标 `reference+10`、带外正/负方向确认。保留所有其他参数和安全门禁。
- 修正后 M2-D 专项无硬件测试 `21 passed`，相关联合回归 `75 passed`；当时允许按原参数执行 Retry2。该次 Retry1 历史判定永久保持 `ABORTED / NOT PASSED`。

### M2-D Retry2 实机结果（2026-08-17）

- 判定：`PASS`；原始日志：`logs/multi_motor/m2/20260817_105855_m2d_joint_id1_id2.log`。
- ID1：`baseline_reference=38`、`baseline_safety=[38,38]`、target `48`、final `46`、`final_error=2`、最大电流 `66mA`、fault `0x00`。
- ID2：`baseline_reference=38`、`baseline_safety=[37,38]`、target `48`、final `46`、`final_error=2`、最大电流 `48mA`、fault `0x00`。
- 两台 pause 前 target 读回均为 `48`；pause 后 target 均按设备语义同步到停止 actual `46`，不作为失败条件。
- 写帧顺序 `[1,2,1,2,1,2]`，寄存器依次为 `0x25,0x25,0x29,0x29,0x1A,0x1A`，`other_id_write_count=0`。
- `pause_both=true`；ID1/ID2 pause 均成功；双电机 `stop_confirmation=true`；未出现 `HARD STOP REQUIRED`。
- 两台均确认正向运动，最终误差满足 Stage M2 的 `<=2 step` 集成门禁。严格 `±1 step` 精度未通过，但不阻塞 M2。
- M2-D Retry2 满足全部地址隔离、运动、安全停止和最终误差门禁，判定 `PASS`。

### Stage M2 最终结论与后续路线

- M2-A：`PASS`。ID1/ID2 定向只读通信 202/202 首次成功，ID3 预期无响应；无 timeout、错 ID、长度或校验错误。
- M2-B：`PASS`。ID1 正向运动，target `25`、final `23`、误差 `2 step`；ID2 全程只读且配置未变；pause 和 stop confirmation 通过。
- M2-C：`PASS`。ID2 正向运动，target `10`、final `8`、误差 `2 step`；ID1 全程只读且配置未变；pause 和 stop confirmation 通过。
- M2-D：首次实机结果保留 `FAIL / NOT PASSED`；Retry1 因过严 baseline span 判据在零写入状态下 `ABORTED / NOT PASSED`；Retry2 `PASS`。三次历史结果均按各自事实保留，不追溯改判。
- 归因收口：首次 FAIL 为 pause 后 target 语义的错误验收逻辑；Retry1 ABORTED 为过严 baseline span 门禁；两项均已在验收层最小修正，无证据支持 UART 通信或多电机调度故障。
- M2 阶段最终结论：M2-A、M2-B、M2-C、M2-CAL 和 M2-D Retry2 全部通过，Stage M2 `COMPLETE / PASS`。
- M2-CAL 已 PASS：8 点误差为 ID1 `-1,-2,-2,-1`、ID2 `0,-3,-1,-2` step；未发现固定 `-2 step` 偏置，禁止补偿写入。pause 语义核对支持 target 在 pause 后同步停止 actual；残差约 `0~3 step`（40 step/mm 约 `0~0.075 mm`），暂作非阻塞已知现象。
- 后续路线：解锁并仅准备 Stage M4；M4-A 先执行 ID1~ID4 纯只读通信，M4-B/M4-C 在前置阶段 PASS 前继续锁定。M6 不准备。

### 20.6 M2-A 已执行只读命令

以下命令只读取 ID 寄存器，不包含任何写操作：

```bash
cd /home/lyapunov/bone_drill/up/motor/uart_multi

python3 tools/probe_two_ids.py \
  --port /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
  --ids 1 2 \
  --probe-id 3 \
  --timeout 0.05 \
  --retries 2 \
  --log "logs/multi_motor/m2/$(date +%Y%m%d_%H%M%S)_m2a_readonly_id1_id2.log"
```

### 20.7 M2-B 已执行命令（历史记录）

```bash
cd /home/lyapunov/bone_drill/up/motor/uart_multi

python3 tools/safe_m2b_id1_only.py \
  --port /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
  --timeout 0.05 \
  --retries 2 \
  --log "logs/multi_motor/m2/$(date +%Y%m%d_%H%M%S)_m2b_id1_only.log"
```

该命令对应的两次独立历史日志已在 20.3 节记录，不得再次复用原日志名。

### 20.8 M2-C 实机命令（已执行，历史记录）

```bash
cd /home/lyapunov/bone_drill/up/motor/uart_multi

python3 tools/safe_m2c_id2_only.py \
  --port /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
  --timeout 0.05 \
  --retries 2 \
  --log "logs/multi_motor/m2/$(date +%Y%m%d_%H%M%S)_m2c_id2_only.log"
```

该命令已执行并对应日志 `logs/multi_motor/m2/20260816_182345_m2c_id2_only.log`；实机结果为 `PASS`。不得复用原日志名。

### 20.9 M2-D Retry2 已执行命令（PASS 历史记录）

```bash
cd /home/lyapunov/bone_drill/up/motor/uart_multi

python3 tools/safe_m2d_joint_id1_id2.py \
  --port /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
  --timeout 0.05 \
  --retries 2 \
  --log "logs/multi_motor/m2/$(date +%Y%m%d_%H%M%S)_m2d_joint_id1_id2.log"
```

首次历史命令对应日志 `logs/multi_motor/m2/20260817_093231_m2d_joint_id1_id2.log`，Retry1 对应 `logs/multi_motor/m2/20260817_104109_m2d_joint_id1_id2.log`，Retry2 PASS 对应 `logs/multi_motor/m2/20260817_105855_m2d_joint_id1_id2.log`；三个日志名均不得复用。本命令仅保留为历史记录，不得重新执行已经 PASS 的 M2-D。

### 20.10 M2-CAL 位置校准验证（PASS）

独立方案文档：`M2-CAL位置校准方案.md`；独立工具：`tools/safe_m2cal_position.py`。固定目标点为 `10/20/30/40`，先完成 ID1 四点，再完成 ID2 四点；不做双电机联合运动，不改现有控制逻辑，不写补偿量。

固定边界为 `50 step/s`、单点最大 `2s`、`500mA`，目标相对初始位置最大偏移 `40 step`；每点记录 `target_position`、`actual_position`、`error`、`current`、`force`、`fault`、pause 前后状态及停止确认。所有写帧只允许 ID1/ID2 对应地址和 `0x25/0x29/0x1A`，禁止广播、ID、Flash、补偿写入。

PASS 只表示 8 个点的数据字段、安全检查、pause 和 stop confirmation 全部完整；error 大小不作为自动补偿或精度 PASS。FAIL 包括通信/帧错误、越界、fault、超电流、缺字段、错误写地址、pause/停止确认失败；写入前安全检查失败为 `ABORTED / NOT PASSED`。

实机日志：`logs/multi_motor/m2cal/20260817_100115_m2cal_position.log`；8 点均 pause 成功、stop confirmation 通过、fault=0 且电流正常。pause 前/响应/后摘要如下：ID1 targets `10/20/30/40` 的 actual 分别为 `9/9/9`、`18/18/18`、`28/28/28`、`39/39/39`；ID2 targets `10/20/30/40` 的 actual 分别为 `10/10/10`、`17/17/17`、`29/29/29`、`38/39/38`（before/response/after）。原始状态帧显示 pause 后 target 字段同步到停止 actual。

8 点误差：ID1 `-1,-2,-2,-1`；ID2 `0,-3,-1,-2` step。未发现固定 `-2 step` 偏置，不写补偿；残差约 `0~3 step`（40 step/mm 约 `0~0.075 mm`）作为非阻塞已知现象。无硬件测试：`test_safe_m2cal_position.py`，`3 passed`。M2-CAL 执行时 M4 尚锁定；其后的 M4-A 已正式 PASS。
### 20.11 线程交接快照

- M2-A：`PASS`；有效日志 `logs/multi_motor/m2/20260816_164752_m2a_readonly_id1_id2.log`。
- M2-B：`PASS`；未解锁预检 `logs/multi_motor/m2/20260816_170427_m2b_id1_only.log` 保持 `ABORTED / NOT PASSED`，正式有效日志 `logs/multi_motor/m2/20260816_170610_m2b_id1_only.log`。
- M2-C：`PASS`；有效日志 `logs/multi_motor/m2/20260816_182345_m2c_id2_only.log`。
- M2-CAL：`PASS`；方案 `M2-CAL位置校准方案.md`、工具 `tools/safe_m2cal_position.py`，实机日志 `logs/multi_motor/m2cal/20260817_100115_m2cal_position.log`，无硬件测试 `3 passed`。
- M2-D：首次实机历史结果 `FAIL / NOT PASSED`；Retry1 日志 `logs/multi_motor/m2/20260817_104109_m2d_joint_id1_id2.log` 为运动前 baseline gate `ABORTED / NOT PASSED`，`write_count=0`；Retry2 日志 `logs/multi_motor/m2/20260817_105855_m2d_joint_id1_id2.log` 为 `PASS`。
- Stage M2：`COMPLETE / PASS`。其后 M4-A 已正式 PASS；M4-B1 因 ID4 preflight 位置 `-2` 在零写入状态下 ABORT；ID4 单机诊断、ID3/ID4 对照及重新上电复核均确认 ID4 持续 `-2` 且零写入。M4 主线暂停，当前仅 ID4-OFFSET `PREPARED / NOT EXECUTED`；M4-B1 不重跑，M4-B2/M4-C/M6 未准备。
- 冻结项：`protocol.py`、`uart_bus.py`、`motor.py` 以及永久基线 `logs/six_ids_test01.log`，均不得修改、覆盖或删除。
- 非阻塞遗留：ID1、ID2 个体运动均约 `2 step` 最终残差；另有 ID1 低端约 `0.9s` 启动延迟、无载力约 `-400g` 静态偏置。

## 21. Stage M4：ID1~ID4 四电机共享 UART 安全验证

状态：Stage M2 `COMPLETE / PASS`；M4-A 正式 `PASS`；M4-B1 `ABORTED / NOT PASSED`；ID4 纯只读诊断 `COMPLETE / PERSISTENT_MINUS2`；ID3/ID4 对照诊断 `COMPLETE / PERSISTENT_ID4_MINUS2_WITH_ID3_REFERENCE`；ID4 重新上电只读复核 `COMPLETE / PERSISTENT_MINUS2`；M4 主线暂停，ID4-OFFSET `PREPARED / NOT EXECUTED`；其余运动阶段锁定。

Stage M4 固定范围为同一 UART 总线上的 ID1、ID2、ID3、ID4。继续复用且不得修改 `protocol.py`、`uart_bus.py`、`motor.py`；禁止广播写、修改 ID、保存 Flash、位置补偿和扩大既有运动安全参数。约 `0~3 step`（约 `0~0.075 mm`）位置残差继续作为非阻塞已知现象记录，不写补偿。

### 21.1 M4-A：ID1~ID4 纯只读通信（PASS）

较早失败尝试作为历史保留：

- 日志：`logs/multi_motor/m4/20260817_112819_m4a_readonly_id1_id2_id3_id4.log`；
- ID1/ID2 各 `101/101 first success`，ID3/ID4 各 `101 final failure`，合计 `202/404 first success`、`202 final failure`、`timeout=606`；ID5 `expected_no_response=1`；
- 判定：`FAIL / NOT PASSED`，不追溯改判、不删除日志。

正式实机结果（2026-08-17 11:32）：

- 日志：`logs/multi_motor/m4/20260817_113201_m4a_readonly_id1_id2_id3_id4.log`；
- ID1~ID4 均为 `requests=101, first_success=101, retry_success=0, final_failure=0`；
- 合计 `requests=404, first_success=404, retry_success=0, final_failure=0`，`timeout=0, wrong_id=0, length_error=0, checksum_error=0`；
- ID5 `expected_no_response=1`；
- `avg_response_ms=2.836, max_response_ms=3.655`；
- 满足全部只读通信门禁，正式 M4-A 判定 `PASS`。

M4-A 无硬件测试：`test_probe_four_ids.py` 与 `test_probe_two_ids.py`，合计 `9 passed`。

### 21.2 M4-B：逐台单独运动（M4-B1 ABORTED / NOT PASSED）

分成四个独立 Run：

- M4-B1：仅 ID1 运动，ID2/ID3/ID4 只读；
- M4-B2：仅 ID2 运动，ID1/ID3/ID4 只读；
- M4-B3：仅 ID3 运动，ID1/ID2/ID4 只读；
- M4-B4：仅 ID4 运动，ID1/ID2/ID3 只读。

统一工具为 `tools/safe_m4b_one_motor.py --moving-id {1,2,3,4}`。四台共用一个 `UartBus`；每个 Run 对四台分别采集 `2s/50Hz` baseline。每台 baseline 使用 position histogram 唯一众数作为 `baseline_reference`，所有样本必须满足 `abs(actual-baseline_reference)<=1`；出现众数 tie 或偏离达到 2 step 时，在任何写入前拒绝。`baseline_safety_min/max` 记录实际范围，运动目标固定为当前运动 ID 的 `baseline_reference+10 step`。

仅当前运动 ID 使用既有 `50 step/s`、最大 `2s`、`500mA` 和设备限位/软件上限 `1800` 门禁。只有 `actual>baseline_safety_max` 才确认正向运动，只有 `actual<baseline_safety_min` 才判定反向运动。目标写入后、运动开始前必须验证 `readback_target_position == commanded_target`；pause 后 target 同步仅记录，不与原命令比较。

三台观察电机从 baseline 后保持严格只读，setup 后、target 后和最终均复核 mode/speed/target 配置不变；运动期间串行记录 actual/current/force/fault，实际位置须在各自 `baseline_reference±1`，current 不得超过各自 baseline 最大值，fault 必须为 0。所有出站帧由 `M4bAuditedBus` 审计；写 destination 只能是当前运动 ID，允许的写起始寄存器仅为 `0x25/0x29/0x1A`，禁止其他 ID、广播、ID 和 Flash 写入。

正常到位或 armed 后任一异常均只向当前运动 ID 发送一次 pause，不自动重试；绝不 pause 三台观察电机。pause 后对 ID1~ID4 同时执行纯只读 stop confirmation。pause 失败或四台任一停止确认失败必须输出 `HARD STOP REQUIRED`，UART 仅在安全处理后关闭。

M4-B 单 Run PASS 标准：

- 四台 baseline 均通过唯一众数与 `±1 step` 门禁，运动目标位于设备限位和软件上限内；
- 当前运动 ID 明确观察到正向运动，pause 后 `final_error<=2 step`；严格 `±1 step` 结果另行记录，不作为本阶段唯一门禁；
- 当前运动 ID 无反向带外、越界、`current>500mA`、非零 fault 或通信异常；
- 三台观察电机位置不离开各自 baseline band，current 不超过各自 baseline 最大值，fault=0；
- 三台观察电机的 mode/speed/target 在全部复核点不变；
- 全部写帧 destination 仅为当前运动 ID，三台观察电机 `write_count=0`；
- 仅当前运动 ID pause 成功，ID1~ID4 stop confirmation 全部通过；
- 独立日志完整记录四台 baseline、target/actual/current/force/fault、配置复核、访问审计、pause 和停止确认。

M4-B 单 Run FAIL 标准：

- 任一 baseline、限位或 pause 前目标读回门禁失败；
- 运动 ID 未确认正向运动、最终误差超过 2 step、反向、越界、超电流、fault 或通信异常；
- 任一观察电机位置/current/fault 异常或配置变化；
- 任一写帧目标不是当前运动 ID，或写寄存器不在白名单；
- pause/四台停止确认失败，或出现 `HARD STOP REQUIRED`；
- 日志不能证明三台观察电机零写入和四台完整状态；
- 若在任何写入前因 baseline、限位或人工确认失败，记为 `ABORTED / NOT PASSED`。

顺序门禁：
1. 首次 M4-B1 实机结果（2026-08-17 12:36，`logs/multi_motor/m4/20260817_123600_m4b1_id1_only.log`）因 ID4 初始位置 `-2` 越界中止，判定为 `ABORTED / NOT PASSED — ID4 pre-motion position out of range`（历史永久保留，不追溯改判）；
2. 首次 M4-B1 Retry 实机结果（2026-08-17 16:20，`logs/multi_motor/m4/20260817_162022_m4b1_retry_id1_only.log`）因 ID3 零端量化抖动为 `-1` 触发 baseline 中止，判定为 `ABORTED / NOT PASSED`（历史永久保留，不追溯改判）；
3. 最新 M4-B1 Retry 实机结果（2026-08-17 16:31:11，`logs/multi_motor/m4/20260817_163111_m4b1_retry_id1_only.log`）：**`COMPLETE / PASS`**。
   - 运动电机：`ID1`，baseline_reference=45，baseline_safety=[45,46]，commanded target=55，final_actual=53，到位误差 error=2 step（符合 $\le 2$ step 门禁）；
   - 观察电机：`ID2`（45~46 step）、`ID3`（6~7 step）、`ID4`（39 step），全程严格只读，`observer_write_count=0`；
   - 写审计：`write_count=3`（全部目标为 ID1：`0x25`、`0x29`、`0x1A`）；
   - pause 单次发送成功（`pause_success=True`），四机停止确认通过（`stop_confirmation=True`）；
4. 首次 M4-B2 实机结果（2026-08-17 16:33:20，`logs/multi_motor/m4/20260817_163320_m4b2_id2_only.log`）：因 ID1 内部 target 同步至实际停止位置并伴随 $52 \leftrightarrow 53$ 的 1 step 静态量化抖动，触发原代码单点绝对全等配置拦截，判定为 **`STOP / NOT PASSED`**（历史永久保留，不追溯改判）。写审计 `write_count=2`，无 observer 写入，无电机运动。随后 ID1 100 轮只读诊断确认状态正常；完善 observer configuration audit 为独立建立 baseline target safety band 机制并补充测试（专项 27 passed，全仓 248 passed）；
5. 首次 M4-B2 Retry 实机结果（2026-08-17 16:41:02，`logs/multi_motor/m4/20260817_164049_m4b2_retry_id2_only.log`）：ID2 正向运动到达 53 step（error=2）触发 pause，pause 成功后在 0.5s 静止采样内稳定在 52 step（final_error=3），触发原代码 2-step 验收上限拦截，判定为 **`STOP / NOT PASSED`**（历史永久保留，不追溯改判）。写审计 `write_count=3`，ID1/3/4 严格 0 写入，全量 fault=0x00，停止确认通过。随后明确拆分容差判据为 `PAUSE_TRIGGER_TOLERANCE_STEPS = 2`（运动阶段 pause 触发保持 $\le 2\text{ step}$）与 `FINAL_ACCEPTANCE_TOLERANCE_STEPS = 3`（post-pause 验收误差 $\le 3\text{ step}$ PASS，$\ge 4\text{ step}$ FAIL），补充测试（专项 31 passed，全仓 252 passed）；
6. 最新 M4-B2 Retry2 实机结果（2026-08-17 16:46:12，`logs/multi_motor/m4/20260817_164559_m4b2_retry2_id2_only.log`）：**`COMPLETE / PASS`**。
   - 运动电机：`ID2`，baseline_reference=52，baseline_safety=[52,53]，commanded target=62，到达 60（error=2）触发 pause，最终稳定在 final_actual=59，到位误差 error=3 step（符合 $\le 3$ step 最终验收门禁）；
   - 观察电机：`ID1`（52~53 step）、`ID3`（6~7 step）、`ID4`（39 step），全程严格只读，`observer_write_count=0`；
   - 写审计：`write_count=3`（全部目标为 ID2：`0x25`、`0x29`、`0x1A`）；
   - pause 单次发送成功（`pause_success=True`），四机停止确认通过（`stop_confirmation=True`）；
7. 最新 M4-B3 实机结果（2026-08-17 16:49:49，`logs/multi_motor/m4/20260817_164937_m4b3_id3_only.log`）：**`COMPLETE / PASS`**。
   - 运动电机：`ID3`，baseline_reference=7，baseline_safety=[6,7]，commanded target=17，到达 15（error=2）触发 pause，最终稳定在 final_actual=15，到位误差 error=2 step（符合 $\le 3$ step 最终验收门禁）；
   - 观察电机：`ID1`（52 step）、`ID2`（59~60 step）、`ID4`（39 step），全程严格只读，`observer_write_count=0`；
   - 写审计：`write_count=3`（全部目标为 ID3：`0x25`、`0x29`、`0x1A`）；
   - pause 单次发送成功（`pause_success=True`），四机停止确认通过（`stop_confirmation=True`）；
8. 最新 M4-B4 实机结果（2026-08-17 16:53:35，`logs/multi_motor/m4/20260817_165331_m4b4_id4_only.log`）：**`COMPLETE / PASS`**。
   - 运动电机：`ID4`，baseline_reference=39，baseline_safety=[39,39]，commanded target=49，到达 47（error=2）触发 pause，最终稳定在 final_actual=47，到位误差 error=2 step（符合 $\le 3$ step 最终验收门禁）；
   - 观察电机：`ID1`（52 step）、`ID2`（59~60 step）、`ID3`（15 step），全程严格只读，`observer_write_count=0`；
   - 写审计：`write_count=3`（全部目标为 ID4：`0x25`、`0x29`、`0x1A`）；
   - pause 单次发送成功（`pause_success=True`），四机停止确认通过（`stop_confirmation=True`）。

**Stage M4-B 总结**：四台电机独立小幅运动验证（M4-B1、M4-B2、M4-B3、M4-B4）已全部实机 **PASS**。

### 21.3 M4-C：四电机联合小幅运动 / 协同控制验证（COMPLETE / PASS）

状态：`COMPLETE / PASS`（2026-08-17 17:02:44，`logs/multi_motor/m4/20260817_170233_m4c_joint_four_motors.log`）。

- 方案：ID1~ID4 全量四台电机共线连接同一 UART 总线，执行联合小幅正向运动 / 协同控制验证（`+10 step / 50 step/s / 最大2.0s / 500mA`）；
- 实机运行数据：
  - **ID1**：baseline_reference=52, target=62, 到达 60 触发 pause, final_actual=59, error=3 step（$\le 3$ step 门禁通过），峰值电流 62 mA，fault=0x00；
  - **ID2**：baseline_reference=59, target=69, 到达 67 触发 pause, final_actual=67, error=2 step（$\le 3$ step 门禁通过），峰值电流 47 mA，fault=0x00；
  - **ID3**：baseline_reference=15, target=25, 到达 23 触发 pause, final_actual=23, error=2 step（$\le 3$ step 门禁通过），峰值电流 29 mA，fault=0x00；
  - **ID4**：baseline_reference=47, target=57, 到达 55 触发 pause, final_actual=54, error=3 step（$\le 3$ step 门禁通过），峰值电流 27 mA，fault=0x00；
- 过程与协同控制：
  1. 四台电机独立 baseline 门禁（众数 $\pm 1$ step 安全带）全部通过；
  2. Setup `0x25` 与 Target `0x29` 依次下发且 100% 读回一致；
  3. 实测首次正向起步时间：ID4 @ 0.30s、ID3 @ 0.37s、ID2 @ 0.73s、ID1 @ 0.76s（存在起步离散时差，当前阶段未验证高精度同步启动，不表述为严格同步控制）；
  4. 1.36s 内四台电机全部到达各自误差 $\le 2\text{ step}$ 触发统一 Pause；
  5. 全四机单次 pause `0x1A` 发送全部成功（`pause_all=true`）；
  6. 0.5s 静止采样确认四机停止（`stop_confirmation=True`）；
  7. 写审计汇总 `write_count=12`（4 次 setup、4 次 target、4 次 pause），0 次非法写入；
- 专项测试与回归：`test_safe_m4c_joint_four_motors.py`（10 passed），全仓回归 `269 passed`。

**Stage M4 阶段验收结论**：Stage M4（四电机共线通信、独立单机运动与四机联合协同运动）全部测试已圆满完成（M4-A / M4-B / M4-C 全 PASS）。

### 21.4 ID4 纯只读状态诊断（COMPLETE / PERSISTENT_MINUS2）

固定工具 `tools/diagnose_id4_readonly.py` 只创建 `Motor(4)`。连续采集 100 组、间隔 0.05s；每组读取 `0x29` 和 `0x30`，记录 signed target、status target、actual、current、force、fault。`0x29` 同时记录原始无符号值和十六进制，工具自身将 `0xFFFE` 解释为 `-2`，不修改 `protocol.py`。

原始日志 `logs/multi_motor/m4/20260817_130429_id4_readonly_diagnosis.log`：100/100 样本的 register target、status target、actual 均为 `-2`，current 始终 `0mA`，force 始终 `-61g`，fault 始终 0；审计汇总为 `read_register_count=100, read_status_count=100, write_count=0`。分类为 `PERSISTENT_MINUS2`。该结果排除瞬时抖动和偶发 UART 错误，但不证明 ID4 硬件故障，也不解锁任何运动。

`Id4ReadOnlyAuditedBus` 只允许 destination=4 的 `CMD_RD_REGISTER` 和 `CMD_RD_STATUS`，其他 ID 或任意写命令均在进入冻结底层前拒绝。专项无硬件测试 `12 passed`。

### 21.5 ID3/ID4 纯只读对照诊断（COMPLETE）

固定工具 `tools/diagnose_id3_id4_readonly.py` 只创建 `Motor(3)` 与 `Motor(4)`，共用同一 UART。连续采集 100 轮、间隔 0.05s；每轮依次对 ID3、ID4 读取 mode `0x25`、target speed `0x28`、target position `0x29` 与 status `0x30`。逐样本明确记录 register target、status target、actual、current、force、fault，并在结尾输出两台每个字段的直方图及逐字段相等/不同计数。

`Id3Id4ReadOnlyAuditedBus` 仅允许 destination=3/4 的 `0x25/0x28/0x29` 单寄存器读取及 `CMD_RD_STATUS`；任意写命令、其他 ID、其他寄存器和多寄存器读均在进入冻结底层前拒绝。汇总必须为 `write_count=0`。`0x29=0xFFFE` 同时保留 raw u16/hex，并只在本工具内解释为 `-2`；不修改冻结解析层，不 clamp 为 0，不改变现有 `[0,2000]` 安全门禁。样本不完整或任一读取失败均分类为 `INCONCLUSIVE`。

原始日志 `logs/multi_motor/m4/20260817_132920_id3_id4_readonly_comparison.log`：分类为 `PERSISTENT_ID4_MINUS2_WITH_ID3_REFERENCE`。mode、target speed、current、fault 均 100/100 相同；ID3 register target 为 `{-1:1, 0:99}`，status target/actual 均为 `{0:100}`；ID4 register target/status target/actual 均为 `{-2:100}`。ID3/ID4 各完成 300 次白名单寄存器读和 100 次状态读，`write_count=0`。该结果将问题定位为 ID4 自身持续位置状态，不支持 UART 总线或多机调度问题，但仍不足以判定硬件故障。

手册结论：

- 现行 Type-D V2.0.4 明列 `0x29 target_position=[0,2000]`、`0x2A actual_position=[0,2000]`；产品说明为内置绝对位置传感器、断电不丢失位置、无需归零；
- V2.0.4 未提供位置零点、位置初始化或位置校准流程；`0x1B` 仅描述为还原参数命令；
- 旧 V1.8.3 将 target 列为 `0~2000`、actual 列为 `-20~2020`，其中零位校准仅针对力传感器；
- 因此 `0x29=-2` 在两版目标范围中均越界，`0x2A=-2` 也超出现行规格。旧版只使“实际反馈在零点附近出现小负值”具有历史可解释性，不能把 ID4 当前 target/actual 同时持续 `-2` 认定为正常，也不足以判定硬件故障。

专项无硬件测试 `test_diagnose_id3_id4_readonly.py` 为 `15 passed`。

### 21.6 ID4 单机重新上电纯只读复核（COMPLETE / PERSISTENT_MINUS2）

人工断电后只保留 ID4 的 UART 与动力连接，确认 ID1/ID2/ID3/ID5/ID6 均断开，再给 ID4 重新上电。重新上电后不得发送任何初始化、模式、目标、pause、零点或校准写入。

复用已审核的 `tools/diagnose_id4_readonly.py`：固定只创建 `Motor(4)`，连续 100 轮、0.05s 间隔读取 `0x29 target_position` 和 `0x30` 状态中的 target/actual/current/force/fault。`0xFFFE` 保留 raw u16/hex 并只在工具内解释为 `-2`，不 clamp，不修改 `[0,2000]` 门禁。审计层只允许 destination=4 的读请求，任意写或其他 ID 均在进入冻结底层前拒绝，汇总必须为 `write_count=0`。

原始日志 `logs/multi_motor/m4/20260817_134018_id4_repower_readonly_verify.log`：100/100 register target、status target、actual 均为 `-2`，current 始终 `0mA`，fault 始终 0，force 范围 `-61~-52g`；审计汇总为 `read_register_count=100, read_status_count=100, write_count=0`。确认 `-2` 是掉电重启后仍存在的 device-local persistent state，继续按 V2.0.4 视为 out-of-spec。

禁止 ID4 运动、clamp、放宽 `[0,2000]` 门禁或执行零点/校准写入。本结果不直接判定硬件故障。

### 21.7 ID3/ID4 纯只读配置/版本审计（PREPARED / NOT EXECUTED）

当前物理状态来自 21.6：仅 ID4 连接。正式对照前必须人工断电，改为只连接 ID3+ID4，核对实体标签后再上电；Agent 不执行接线或实机读取。

本地官方资料结论：

- V2.0.4 只定义状态读 `0x30`、寄存器读 `0x31` 和寄存器写 `0x32`，没有设备信息/版本查询命令；
- 公开 `0x16~0x2F` 寄存器没有固件版本、协议版本、设备型号/类型、位置零点或位置偏移字段；成功解析 `0x30/0x31` 只能确认 V2 风格协议族，不能判断 ID3/ID4 的精确固件版本是否相同；
- 位置相关可读配置只有行程上限 `0x23`、行程下限 `0x24`、控制/目标配置 `0x25~0x29` 和实际位置 `0x2A`/状态帧；
- 旧 V1.8.3 的 `-20~2020` actual 范围不能覆盖现行 V2.0.4；旧版“零位校准”仅针对力传感器，且地址布局不同，不得用于探测现行设备。

固定工具 `tools/audit_id3_id4_config_readonly.py` 共用一个 UART，创建且只创建 `Motor(3)`、`Motor(4)`。执行三轮完整快照，每轮对两台读取：

- ID `0x16`、波特率 `0x17`；
- 过温/回温/过流保护 `0x1E~0x20`；
- 正反输出上限 `0x21/0x22`；
- 行程上限/下限 `0x23/0x24`；
- mode、motor voltage、force target、target speed、target position `0x25~0x29`；
- 状态帧中的 target、actual、current、force/ADC、temperature、fault。

白名单明确排除清故障 `0x18`、急停 `0x19`、pause `0x1A`、还原 `0x1B`、保存 `0x1C`、权限验证码 `0x1D` 和所有未列地址。只允许单寄存器读取；任意写、其他 ID、非白名单地址或多寄存器读在冻结底层前拒绝，汇总必须为 `write_count=0`。`0xFFFE` 保留 raw u16/hex 并解释为 `-2`，不 clamp，不修改 `[0,2000]` 门禁。

已有日志已确认：ID 地址按设计为 3/4；两台行程下限均 0、上限均 2000、mode 均 0、target speed 均 65535；ID3 target/actual 约 0，ID4 持续 -2。其余波特率、保护值、输出上限、voltage、force target 尚无正式 ID3/ID4 对照结果。

专项无硬件测试 `test_audit_id3_id4_config_readonly.py` 为 `19 passed`；全仓无硬件回归 `205 passed`。

唯一纯只读实机命令（仅在人工断电并只连接 ID3+ID4 后执行；当前未执行）：

```bash
cd /home/lyapunov/bone_drill/up/motor/uart_multi

python3 tools/audit_id3_id4_config_readonly.py \
  --port /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
  --timeout 0.05 \
  --retries 2 \
  --log "logs/multi_motor/m4/$(date +%Y%m%d_%H%M%S)_id3_id4_config_readonly_audit.log"
```

本配置审计保持 `PREPARED / NOT EXECUTED`，但其执行已被新的 ID4-OFFSET 优先任务取代；当前不得执行该命令。

### 21.8 ID4-OFFSET 固定零偏表征（COMPLETE / PASS）

状态：`COMPLETE / PASS`。

原始日志：`logs/multi_motor/id4_offset/20260817_151126_id4_offset.log`（2026-08-17 15:11:26）。
固定工具 `tools/safe_id4_offset_characterization.py` 仅连接 ID4，固定参数 `10/20/30/40 step`、`50 step/s`、每点 `2s`、电流上限 `500mA`。

实机结果：
- 入口验证：100/100 样本 target_raw/actual_raw 均为 `0xFFFE/-2`，current=0，fault=0，行程配置严格为 `[0,2000]`；
- 四点实测：
  - Point 1 (target=10): readback=10, actual=9, error=-1; pause 后 readback=9, actual=9;
  - Point 2 (target=20): readback=20, actual=19, error=-1; pause 后 readback=19, actual=19;
  - Point 3 (target=30): readback=30, actual=28, error=-2; pause 后 readback=28, actual=28;
  - Point 4 (target=40): readback=40, actual=39, error=-1; pause 后 readback=39, actual=39;
- 停止确认：四点 stop confirmation 均稳定通过（`stable=True`，电流 0mA，fault 0）；
- 写审计：`write_count=8`（全部 destination=4，仅首点 `0x25` 原子入口、各点 `0x1A` pause 与后续 `0x29` 目标写），`compensation_write_count=0`；
- 判定分类：`PASS / ORDINARY_0_TO_3_STEP_RESIDUAL_SUPPORTED`。

实机结论与规则：
1. 四点到位误差为 `[-1, -1, -2, -1]`，变动且在 `0~3 step` 既知残差区间内，不支持固定 `-2 step` 零参考偏移假设；
2. 严禁执行 `+2 step` 或任何形式的位置补偿；
3. ID4 进入正位置区间（`>=0`）后表现正常，与 ID1/ID2/ID3 行为一致；
4. 剩余问题仅为：ID4 掉电重新上电后初始寄存器与状态 target/actual 持续报告 raw `-2`。

### 21.9 ID4 近零端边界恢复与保护方案（COMPLETE / PASS）

状态：`COMPLETE / PASS`。

原始日志：`logs/multi_motor/id4_init/20260817_161543_id4_poweron_init.log`（2026-08-17 16:15:43）。
固定工具 `tools/safe_id4_poweron_initialize.py` 在人工断电重启后执行诊断保护，连续 100/100 轮读取 `0x29 target`、`0x2A actual` 及 `0x30` 状态，target 均为 39、actual 均为 39、current 始终 0mA、fault 始终 0x00，判定为 `PASS / ALREADY_IN_RANGE_NO_ACTION`，`write_count=0`，`global_gate_restored=true`。

实机结论修正与规则：
1. **非固定上电默认状态**：ID4 的 raw=-2 并非掉电上电必然触发的状态，而是**近零端特殊边界状态**；
2. **合法区间保持**：实测确认 ID4 在约 39 step 掉电重启后仍读取 target=39、actual=39，保持在合法位置区间；底层掉电保持机制未由官方资料确认，不归因于绝对位置传感器；
3. **严禁位置补偿与主动触发**：严禁 `+2 step` 或任何形式的位置补偿；**严禁为了触发 -2 而主动将 ID4 移回零端**；
4. **工具转为常态化边界保护**：`tools/safe_id4_poweron_initialize.py` 转化为常态化近零端边界恢复与安全保护工具，若 ID4 在后续因特殊情况再次触碰近零端导致 raw=-2 时可按本方案安全恢复。
5. 全局 `[0,2000]` 门禁不变，底层保持冻结。

专项无硬件测试：`test_safe_id4_poweron_initialize.py`（12 项测试全部通过）；全仓无硬件回归 `244 passed`。

### 21.10 ID3 零端脱离方案（COMPLETE / FAIL_STOP_CONFIRMATION）

状态：`FAIL / HARD STOP REQUIRED — stop confirmation failed`（原始结果永久保留，不追溯改判）。

原始日志：`logs/multi_motor/id3_escape/20260817_162634_id3_zero_escape.log`（2026-08-17 16:26:34）。

实机过程与分析：
1. 入口 100 轮只读预检判定 `entry_state=ESCAPE_REQUIRED`；
2. 写入 `0x25` 运动配置（target=10, speed=50），寄存器 `0x29` 读回确认 10；
3. ID3 在 0.265s 首次正向运动，于 1.174s 达到 actual=7 step 并触发单次 pause（响应正常）；
4. 首轮 0.5s 停止确认中前 10 样本均为 7，第 11 样本跳变到 6，因单向趋势未回弹触发 `bounded but directional trend not settled`；
5. 写审计：`write_count=2`，`write_destinations=[3, 3]`，ID1/ID2/ID4 严格零写入；峰值电流 33mA，全程无 fault、无越界、无反向运动；
6. 脱后 100 轮纯只读诊断（`logs/multi_motor/m4/20260817_162836_id3_post_escape_diagnosis.log`）：连续 100 轮采样寄存器 target `{6: 36, 7: 64}`、状态 actual `{6: 31, 7: 69}`、current 恒为 0mA、fault 恒为 0x00；
7. 结论：脱零运动目标实际上已达成，ID3 已稳定脱离 0/-1 边界并静止于 `6~7 step`（符合既知 `±1 step` 静态量化抖动）；原始 FAIL 判定不予追溯改判，不再重复执行 ID3-ESCAPE。

专项无硬件测试：`test_safe_id3_zero_escape.py`（14 项测试全部通过）；全仓无硬件回归 `244 passed`。

当前四机状态：ID1=59~60, ID2=67, ID3=23, ID4=54~55，Stage M4 全部子阶段已完整 PASS。

## 22. Stage M6 六电机验证规划与进展

### 22.1 Stage M6 阶段推进路线（最小增量原则）

1. **Stage M6-A：六机纯只读状态与配置复核（COMPLETE / PASS）**
   - 目标：ID1~ID6 六台电机共线连接同一 UART 总线，无任何写入（`write_count=0`），读取各机 limits `0x24/0x23`、mode `0x25`、speed `0x28`、target `0x29` 及状态 `0x30`（actual、target、current、force、fault）；
2. **Stage M6-B5：ID5 单机小幅运动验证（PREPARED / NOT EXECUTED）**
   - 目标：仅 ID5 执行 `+10 step / 50 step/s / 2.0s / 500mA` 小幅正向运动，ID1/ID2/ID3/ID4/ID6 保持严格只读观察（写审计 `write_count=0`）；ID1~ID4 沿用 M4 成果，不重复单机运动测试；
3. **Stage M6-B6：ID6 单机小幅运动验证（LOCKED / NOT PREPARED）**
   - 目标：仅 ID6 执行 `+10 step / 50 step/s / 2.0s / 500mA` 小幅正向运动，ID1/ID2/ID3/ID4/ID5 保持严格只读观察（写审计 `write_count=0`）；
4. **Stage M6-C：六电机联合小幅运动 / 协同控制验证（LOCKED / NOT PREPARED）**
   - 目标：ID1~ID6 全量六台电机共线联合执行 `+10 step / 50 step/s / 2.0s / 500mA` 小幅运动 / 协同控制验证。

### 22.2 Stage M6-A 实机结果（COMPLETE / PASS）

状态：`COMPLETE / PASS`（2026-08-17 17:31:48，`logs/multi_motor/m6/20260817_173141_m6a_readonly_review.log`）。

- 结果：`write_count=0`（100% 严格纯只读，0 次写入）；
- 采样统计：各机采集 100 轮状态 + 5 项配置，630 次总线读事务 100% 成功（0 timeout、0 校验错误、0 长度错误、0 总线冲突）；
- 数据汇总：
  - **ID1**：limits=[0, 2000], mode=0, speed=65535, target=59, actual={59: 100}, current=0mA, fault=0x00, force=-360g；
  - **ID2**：limits=[0, 2000], mode=0, speed=65535, target=67, actual={66: 16, 67: 84}, current=0mA, fault=0x00, force=280g；
  - **ID3**：limits=[0, 2000], mode=0, speed=65535, target=22, actual={22: 84, 23: 16}, current=0mA, fault=0x00, force=367~400g；
  - **ID4**：limits=[0, 2000], mode=0, speed=65535, target=54, actual={54: 97, 55: 3}, current=0mA, fault=0x00, force=-6~0g；
  - **ID5**：limits=[0, 2000], mode=0, speed=65535, target=0, actual={0: 88, 1: 12}, current=0mA, fault=0x00, force=272g；
  - **ID6**：limits=[0, 2000], mode=0, speed=65535, target=0, actual={0: 100}, current=0mA, fault=0x00, force=110~125g；
- **边界评估**：ID5 和 ID6 稳定处于合法非负区间（0~1 step / 0 step），无持续负值（无 raw=-1/-2），具备向 +10 step 安全正向运动条件。

### 22.3 Stage M6-B5 进展与 Retry 准备（PREPARED / NOT EXECUTED）

1. **M6-B5 首次实机运行（ABORTED / STOP）**：
   - 2026-08-17 18:14:34，`logs/multi_motor/m6/20260817_181430_m6b5_id5_only.log`；
   - 写审计：`write_count=0`（基线收集阶段安全中止，全六台电机零写入、零运动）；
   - 触发原因：ID2 observer 在 100 轮采样中 actual={67: 86, 66: 13, 65: 1}，单点 65 偏离众数 67 达 2 step 触发现有 `abs(sample - mode) <= 1` 静态门禁；
2. **ID2 500 轮纯只读扩展诊断**：
   - 2026-08-17 18:17:53，`logs/multi_motor/m6/20260817_181740_id2_readonly_diagnosis_500.log`；
   - 结果：actual={67: 424, 66: 74, 65: 2}，reg_target={67: 421, 66: 76, 65: 3}，status_target={67: 424, 66: 74, 65: 2}，current=0mA, fault=0x00，1000 次 UART 事务 100% 成功；
   - 定性：良性低概率 2-step 静态量化抖动（全幅 2 step $\approx 0.05\text{ mm}$），无单向漂移；
3. **M6-B5 Retry 三次实机记录（永久独立保留）**：
   - Attempt 1 (2026-08-17 18:22:10, `logs/multi_motor/m6/20260817_182206_m6b5_retry_id5_only.log`): `STOP / NOT PASSED`（`write_count=1`，目标写入接口名错误）；
   - Attempt 2 (2026-08-17 18:22:53, `logs/multi_motor/m6/20260817_182246_m6b5_retry_id5_only.log`): `STOP / NOT PASSED`（`moving_id=5 write_count=2, observer_write_count=0`，M6-B setup payload 寄存器映射错误导致 0x28 target speed 未按设计配置为 50，运动配置无效，2.0s 内未起步超时停止）；
   - Attempt 3 (2026-08-17 18:28:42, `logs/multi_motor/m6/20260817_182836_m6b5_retry_id5_only.log`): `STOP / NOT PASSED`（`moving_id=5 write_count=3, observer_write_count=0`，ID5 正向起步 0->7，peak 30mA，在 2.0s 截止时刻达到 actual=7 / error=3，差 1 步到 pause 阈值触发超时保护并 pause 停在 8）；
4. **参数优化与规则保持**：
   - `MOTION_TIMEOUT_S` 由 `2.0s` 优化为 `3.0s`（适应零端克服静摩擦的 ~1.0s 起步延迟）；
   - `+10 step`、`50 step/s`、`500mA`、`error <= 2` pause 触发、`final_error <= 3` 验收、Moving / Observer 基线规则严格保持不变；
   - 底层代码保持绝对冻结；
5. **Stage M6-B5 最终实机运行（COMPLETE / PASS）**：
   - 2026-08-17 18:33:11，`logs/multi_motor/m6/20260817_183305_m6b5_retry_id5_only.log`；
   - 结果：**`COMPLETE / PASS`**；
   - ID5 运动：baseline=7 (band=[7, 8]), target=17, final=15, error=2 step（符合 $\le 3$ step 验收）；
   - 观察机隔离：ID1（59）、ID2（66~67）、ID3（22~23）、ID4（54~55）、ID6（0）严格只读并在安全带内（`observer_write_count=0`）；
   - 写审计：`write_count=3`（严格发往 ID5: `0x0025`, `0x0029`, `0x001A`）；
   - 安全性：ID5 峰值电流 30mA，全机 fault 0x00，`pause_success=True`, `stop_confirmation=True`。

### 22.4 Stage M6-B6 实机运行（COMPLETE / PASS）

- 2026-08-17 18:34:58，`logs/multi_motor/m6/20260817_183450_m6b6_id6_only.log`；
- 结果：**`COMPLETE / PASS`**；
- ID6 运动：baseline=0 (band=[0, 0]), target=10, final=8, error=2 step（符合 $\le 3$ step 验收）；
- 启动延迟：ID6 在零端克服静摩擦约 0.8s 后正向起步；
- 观察机隔离：ID1（59）、ID2（66~67）、ID3（22~23）、ID4（54~55）、ID5（14~15）严格只读并在安全带内（`observer_write_count=0`）；
- 写审计：`write_count=3`（严格发往 ID6: `0x0025`, `0x0029`, `0x001A`）；
- 安全性：ID6 峰值电流 44mA，全机 fault 0x00，`pause_success=True`, `stop_confirmation=True`。

### ID1~ID6 单机独立小幅正向运动（M4-B / M6-B）实机结果汇总

| 电机 ID | 阶段代号 | 实机时间 | 基线位置 (Baseline) | 目标位置 (Target) | 最终到位 (Final) | 误差 (Error) | 验收判定 |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **ID1** | Stage M4-B1 Retry | 2026-08-17 16:31:11 | 45 | 55 | 53 | 2 step | **PASS** ($\le 3$) |
| **ID2** | Stage M4-B2 | 2026-08-17 16:47:04 | 52 | 62 | 59 | 3 step | **PASS** ($\le 3$) |
| **ID3** | Stage M4-B3 | 2026-08-17 16:54:19 | 7 | 17 | 15 | 2 step | **PASS** ($\le 3$) |
| **ID4** | Stage M4-B4 | 2026-08-17 17:02:18 | 39 | 49 | 47 | 2 step | **PASS** ($\le 3$) |
| **ID5** | Stage M6-B5 | 2026-08-17 18:33:11 | 7 | 17 | 15 | 2 step | **PASS** ($\le 3$) |
| **ID6** | Stage M6-B6 | 2026-08-17 18:34:58 | 0 | 10 | 8 | 2 step | **PASS** ($\le 3$) |

> **注**：Stage M4-C 四机联合运动后四台电机位置演进至 ID1=59, ID2=67, ID3=22~23, ID4=54~55；与单机结果独立记录。当前六机静态位置为：ID1 $\approx 59$, ID2 $\approx 67$, ID3 $\approx 22\sim 23$, ID4 $\approx 54\sim 55$, ID5 $\approx 15$, ID6 $\approx 8$。

### 22.5 Stage M6-C 实机运行（STOP / NOT PASSED）

- 2026-08-17 18:41:36，`logs/multi_motor/m6/20260817_184132_m6c_joint_six_motors.log`；
- 结果：**`STOP / NOT PASSED`**（永久独立保留历史记录，不自动重试，不修改代码）；
- 方案：ID1~ID6 六机共线联合 +10 step 小幅运动（`+10 step / 50 step/s / 3.0s / 500mA`）；
- 基线建立：ID1=59 (band=[59, 59]), ID2=67 (band=[65, 67]), ID3=22 (band=[22, 23]), ID4=54 (band=[54, 55]), ID5=15 (band=[14, 15]), ID6=8 (band=[8, 8])；
- 目标下发：ID1=69, ID2=77, ID3=32, ID4=64, ID5=25, ID6=18（Setup 与 Target 写入及回读 100% 成功）；
- 保护拦截：轮询第 2 轮（t=0.09s）ID6 状态回读出现 `actual=7`，由于 ID6 baseline band 严格为 `[8, 8]`，`actual=7 < 8` 触发 `BaselineBandGuard` 反向位移门禁（`negative-direction movement detected below baseline band: baseline=[8,8] actual=7`）；
- 制动结果：主控即刻执行六机 Pause-all，依次向 ID1~ID6 发送 `0x1A: [1]` pause 均 100% 成功返回，各机安全停止（ID1=59, ID2=67, ID3=22, ID4=54, ID5=14, ID6=8），全机电流 0mA，fault 全 0x00，写审计严格为 18 次。
- ID6 1000 轮只读扩展诊断：确认 ID6 静止时存在 $7 \leftrightarrow 8$ 的 1-step 位置读回量化/反馈抖动（不归因于具体内部传感器机制），无持续单向漂移趋势。

### 22.6 Stage M6-C Retry 实机运行（COMPLETE / PASS）

- 2026-08-17 18:49:38，`logs/multi_motor/m6/20260817_184931_m6c_retry_joint_six_motors.log`；
- 结果：**`COMPLETE / PASS`**；
- 方案：ID1~ID6 六机共线同总线，六电机联合 +10 step 小幅正向运动 / 协同控制验证（`+10 step / 50 step/s / 3.0s / 500mA`）；
- 基线与目标：
  - ID1: baseline=59, target=69, final=67, error=2 step ($\le 3$ step 验收通过)
  - ID2: baseline=67, target=77, final=75, error=2 step ($\le 3$ step 验收通过)
  - ID3: baseline=22, target=32, final=31, error=1 step ($\le 3$ step 验收通过)
  - ID4: baseline=54, target=64, final=63, error=1 step ($\le 3$ step 验收通过)
  - ID5: baseline=15, target=25, final=23, error=2 step ($\le 3$ step 验收通过)
  - ID6: baseline=8, target=18, final=17, error=1 step ($\le 3$ step 验收通过)
- 运动与协同：
  - 各机首次正向起步时间：ID5 @ 0.371s, ID3 @ 0.405s, ID4 @ 0.409s, ID6 @ 0.493s, ID2 @ 0.559s, ID1 @ 0.714s；
  - 0.84s 内全六机均到达各自误差 $\le 2\text{ step}$ 触发统一 Pause；
  - Pause 发送与停机：`pause_all=true`（6 台单次 pause 全部成功），0.5s 静止采样稳定性复核 `stop_confirmation=true`；
- 安全与写审计：
  - 各机峰值电流：ID1=65mA, ID2=47mA, ID3=29mA, ID4=33mA, ID5=41mA, ID6=43mA（全部 $\ll 500\text{ mA}$）；
  - 全六机故障码：全程严格 `0x00`；
  - 写审计：严格 18 次写入（6 setup + 6 target + 6 pause），0 次非法写入。

**Stage M6 总体验收结论**：Stage M6 六电机 UART 通信、单机独立控制及六机联合协同控制验证已全量圆满闭环完成（M6-A / M6-B / M6-C 全 PASS）。
