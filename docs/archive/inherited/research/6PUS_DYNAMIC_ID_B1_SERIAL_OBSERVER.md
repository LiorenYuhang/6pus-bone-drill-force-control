> 历史归档：原路径 `agent_docs/research/6PUS_DYNAMIC_ID_B1_SERIAL_OBSERVER.md`。正文保留当时的方案、参数及证据语境；当前原生 Mode 3 入口以[项目 README](../../../../README.md)为准。历史命令不作为当前推荐启动命令。

# 第四阶段 B1：透明串口观测层（离线）

日期：2026-10-19。范围仅新增诊断模块与 FakeSerial 测试；没有 J14 启动接入、
ROS 节点、真实串口、运动指令或协议解析新实现。`hardware_write_count = 0`。

## 实际接口与实现边界

源码核查：`j7_real_uart_adapter.py` 调用 serial `read(size)`、`write(data)`，
读取 `in_waiting/out_waiting/timeout/write_timeout/is_open`；`j14_process_uart.py`
另外创建 serial 对象、重置输入/输出缓冲区并关闭对象。

新增 `bone_ws/src/pus6_motor_driver/pus6_motor_driver/diagnostic_serial_observer.py`：

- `ObservedSerial(existing_serial, recorder, enabled=True, clock_ns=None)` 只观测
  read/write。构造不读取 serial 属性；属性与原有生命周期方法按需委托，属性赋值也
  转发。包装器不拥有串口，不创建串口、不增加读取、轮询、等待、重试或复位。
- `SerialEventRecorder(new_directory, run_id=..., capacity=4096)` 使用有界队列和
  单独写盘线程。控制路径做时间戳、必要的字节快照及 `put_nowait`，后台转换 hex/JSON。
  Python 队列与计数使用短锁，不是无锁实现或硬实时保证；实际观测开销尚未真机测量。
- 只覆盖当前实际调用的接口。不是完整 PySerial 替代品；例如上下文管理、迭代等特殊
  方法不在本阶段范围内。生产 J7/J8、协议、控制算法及 J14 入口没有修改。

依赖：新增模块仅用 Python 标准库；离线兼容测试使用现有 pytest、pyserial 的异常
类、FakeBus 与 J7 adapter/transport，不安装依赖、不调用 `serial.Serial`。

## 事件与时间语义

每行 JSONL：`schema_version/run_id/process_instance/recorder_instance/pid/event_seq`、
`clock/timestamp_semantics`、`operation/start_ns/end_ns/requested_length`、
`byte_encoding/input_hex/output_hex/returned_length/return_type/exception_type`。

- 默认时钟 `time.monotonic_ns()`，即本机 CLOCK_MONOTONIC，单位整数 ns。
  `clock_ns` 注入仅供离线确定性测试；真实接入必须使用同一单调基准。
- `process_instance` 在同一解释器进程中稳定，包含 PID 与进程 token；
  `recorder_instance` 区分同进程的多个记录器。event seq 在尝试入队时分配，满队列
  丢弃仍占序号。事件文件顺序是观测提交顺序；并发时不把它解释为操作开始顺序。
- read 保存原调用返回的字节，write 保存传入全部字节。hex 可用 `bytes.fromhex`
  完全恢复；空字节为 `""`，未知/不适用为 null，不能混为一谈。
- 返回原始对象/值；短写不补写，None 返回长度未知；发生异常时 bare raise 原异常
  实例，事件保存异常类型，不在控制路径调用异常的自定义字符串格式化。
- write 抛异常时，已发出多少字节未知。方法起止不是 UART 线上发送结束、设备接收
  或设备实际采样时刻。分段、多帧合并、重复、错误、缺失字节均原样记录，不补齐、
  去重、重新解析或强行进行命令—反馈配对。

## 完整性与生命周期

新输出目录独占创建，禁止复用已有目录。线程先写 `status.json` 的非完整状态，
事件写入 `events.jsonl`。`status()` 随时给出内存状态；在**串口所有者停止所有调用后**
显式 `close(timeout=5)` 排空队列并刷新最终状态。包装器 `.close()` 只转发串口关闭，
不会隐式等待诊断线程。两者由未来诊断入口分别管理。

统计区分：attempted、queued、written、queue_full、after_close、after_writer_failure、
discarded_on_writer_failure，以及 observation/writer/status errors。
`written_events` 指后台 write 返回完整字符数，不等于掉电持久化；flush/close 失败
可能使实际文件内容未知，即使 written 计数非零，`complete` 也必须为 false。

- 队列满只丢诊断事件并计数；不抛入串口调用、不改变控制/通信/安全策略。
- 写盘失败后拒绝新事件并计数，未写队列事件单独计数，串口仍保持原有行为。
- `close` 超时、任何事件缺失或观测/文件/状态错误均不能得到 `complete=true`。
- 完整性只有在显式关闭、事件数量一致、事件流 flush/close 成功且最终状态保存成功时
  为 true。disabled 包装不产生事件，零事件状态不能解释为完成了动态数据采集。
- 若最终状态也无法落盘，调用者必须保留 `close()` 返回的内存失败状态；初始状态
  不是成功证据。进程崩溃/掉电不能保证最终状态或尾部事件保存，不作掉电持久化声明。
- 关闭不得与新的串口调用并发；关闭后的调用仍原样执行，但记录为 rejected，
  内存完整性立即失效，需再次 close 更新磁盘状态。本接口不负责控制串口生命周期。

## 离线验证

测试文件：`bone_ws/src/pus6_motor_driver/test/test_diagnostic_serial_observer.py`。
通过 FakeSerial、FakeBus、假时钟、阻塞/失败写盘夹具验证：

1. 正常/空读、短写、None 返回、字节对象身份、mutable buffer 快照、原异常实例传播。
2. 分段、多帧合并、重复、坏校验字节、空缺和截断片段的原始字节追溯。
3. 属性/赋值/生命周期委托，无额外 serial 属性读取或读写调用。
4. 队列满、慢写盘、关闭超时、open/write/flush/close/短文件写入和状态保存失败。
5. disabled/direct/enabled 操作顺序、输入输出、异常与重试行为对照。
6. 复用现有 J7 adapter 和六轴 transport，对照正常、缺失响应、短写、超时和错误帧。
   adapter 超时异常文本包含实测耗时，因此该逻辑对照注入确定性计时；不是零开销证明。

安全约束：测试自动禁止 `serial.Serial` 和 `serial_for_url`，使用 FakeSerial/FakeBus；
测试输出只写 pytest 临时目录。命令关闭字节码与 pytest 缓存：

```bash
PYTHONDONTWRITEBYTECODE=1 \
PYTHONPATH=/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver \
python -m pytest -q -p no:cacheprovider \
  bone_ws/src/pus6_motor_driver/test/test_diagnostic_serial_observer.py \
  bone_ws/src/pus6_motor_driver/test/test_j7_c9_real_uart_adapter.py
```

本轮实际结果：新增测试与原有 J7 C9 测试合计 **35 passed (0.24 s)**。
新增 Python 文件通过内存 AST 语法检查，新增文件无行尾空白；已有 tracked diff
保持原来的 3 个 J14 Demo 文件、28 行新增，本轮只新增上述模块、测试和本说明。
该结果仅为离线软件验证，不是实时开销、真实 UART 或真机动态辨识资格。

## 下一步最小接口（尚未接入）

诊断版 J14 在单独入口创建唯一串口所有者，将**同一个现有 serial 对象**包为
ObservedSerial 后传给现有 J7UartAdapter；保持参数、读写顺序及安全处理不变。
入口负责新 run 目录、run_id、记录器健康状态汇总，串口全部调用完成后 close 记录器。
不允许第二个进程打开同一串口来采集。

本阶段事件没有 cycle/axis 标识与帧关联。后续另加诊断层事件，关联 Child cycle、
Parent/Child command seq、RX 字节区间和实际反馈采用/复用；现有协议不回显事务序号，
关联有歧义必须保留。任何 read-return 时间仍只是主机可见上界代理，不是设备采样时间。
