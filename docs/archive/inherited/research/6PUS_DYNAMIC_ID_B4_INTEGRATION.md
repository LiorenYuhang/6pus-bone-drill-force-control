> 历史归档：原路径 `agent_docs/research/6PUS_DYNAMIC_ID_B4_INTEGRATION.md`。正文保留当时的方案、参数及证据语境；当前原生 Mode 3 入口以[项目 README](../../../../README.md)为准。历史命令不作为当前推荐启动命令。

# 第四阶段 B4：诊断组件集成与软件压力验证

2026-10-20。新增 `diagnostic_j14_integration.py`、对应离线测试和本说明；复用
B1/B2/B3，不修改生产源码、协议、算法、安全决策、配置或历史结果，不提交 Git。
未启动 ROS/J14/子进程，未打开真实串口，`hardware_write_count = 0`。

## 集成边界与接口

`DiagnosticJ14Integration(config, existing_serial, parent_client, child_socket,
trace_directory=NEW_DIRECTORY, run_id=..., enabled=True, capacity=4096,
stream_factories=None)` 是独立的**注入式集成模块**，不是部署启动器。

- 只接受一个现有 serial，独占所有权转交给 B2；同一对象经 ObservedSerial →
  J7UartAdapter → TracedServoCore。没有串口创建或 fallback。
- Parent 必须尚未被工作线程共享。实例级包装原 lock/sock/update；Child 包装原
  recvmsg 端点。原 IPC 端点由调用者拥有，本模块不调用 ProcessClient.close。
- `wrap_publisher(existing_publisher, sequence=None)` 观察现有 publish 边界；无 getter
  时 source_sequence=null，不生成 Shaper 身份。离线 publisher 只是模拟发布边界。
- `initialize()`、`cycle(...)` 委托 B2。调用者显式执行原 parent.send_state、
  receive_packets、core.command.ingest，再调用 cycle；模块不复制 Child 的握手、
  lease 检查、停止条件或调度循环，不改变它们的决策。
- `enabled=False` 省略全部事件钩子、队列、写盘线程和诊断文件，仅保留 B2 的同一
  所有权包装（串口观测 bypass）。`recording_complete=null`，不是零事件采集成功。
- `status()/close()` 返回各流、串口生命周期及整体完整性；`merge()` 复用 B3，另
  将内存中的最终化失败纳入结果。无法落盘的失败状态需由调用者保留。

当前生产 ProcessClient 固定启动原 Child，child_run 固定构造 ServoCore；本轮没有
替换这些入口，也没有运行真实三进程握手。完整软件生命周期指上述显式注入链，
**不表示已部署完整生产启动链**。若下一阶段要求真实入口自动接线，需先另行审定
独立诊断 launcher/Child 的对象注入接口；不能把本模块直接作为生产启动器使用。

## 已关联与仍未知的量

成功验证的链路：模拟 publish → 原 Parent update/锁内覆盖 → 原 send_state/实际
datagram → 原 receive_packets → 原 CommandState 的 ingest/candidate → 实际最终
target_steps/cycle → 原 serial read/write → 原解析帧对象/读取字节跨度 → transport
实际选取的 StatusFeedback 对象 → 六轴缓存采用及旧值复用。

继承 B3 的 run/host/boot/process/recorder/event 标识与 CLOCK_MONOTONIC ns；每个
角色独立文件。相同 read 中的多个帧共享真实 read 返回边界，不制造逐帧接收时间。
软件测试将这些时钟替换为同一模拟单调域，不能解释为物理测量时间。

继续明确未知：Shaper 生成时间与跨消息源序号、Driver 自身 envelope 内部覆盖、
设备测量时间/事务号、物理命令—反馈配对、UART 线上完成时刻、未暴露的 stale 与
backlog 细分原因。没有新增 FK/控制器 tick 记录。批量 scheduler 时间仍是主机采用
代理。原 parser 抛错而未返回的内部局部帧仍不可见，不重新解析补填。

## 实际离线验证

B4 新增 **25 项**；与 B1/B2/B3、原 J7 C9 合并执行：**103 passed in 7.19 s**。

- 初始化成功、reset/初始化读失败、运行抛错、运动写超时返回原 fault packet、原
  serial.close 失败及主异常同时伴随关闭失败；原主异常保留。
- Parent 覆盖、持值、backlog 保持、最终 slew 目标；逐轴采用帧 raw_steps 与缓存
  相等，并沿一个或多个 read 字节跨度重建原始帧。缺轴、重复帧、迟到残留与旧值复用。
- 关闭记录/开启记录对照：正常、读错、写超时、IPC 满、关闭错、诊断队列满、写盘错、
  重复帧及缺轴迟到。比较底层调用顺序/参数、返回、原异常、IPC 字节及最终目标/位置；
  未新增通信或重试。B2 原生产构造分支的 FakeSerial 对照也在本次回归内。
- 阻塞后台文件后生产者仍完成原初始化和 cycle；溢出显式计数。另验证 writer 失败、
  recorder.close 异常、排空超时后再次完成，以及合并时计数缺口/悬空引用。
  丢失用独立派生测试文件模拟，没有删除原测试日志或历史数据。
- 关闭顺序：全部 core stop/pause 调用 → 唯一 raw serial.close 尝试 → Child recorder
  最终化；其余角色收尾并恢复借用的 lock/socket。清理前须先停止所有调用者。
  重复清理不重复 stop/pause/底层 close；只允许未完成记录器继续排空。

测试禁止 serial.Serial、serial_for_url、socket/socketpair 和 subprocess.Popen；
断言未导入 rclpy。所有数据由 FakeSerial/FakeBus、内存 IPC 提供，每个资源检查
`hardware_write_count == 0`。证据 JSON 记录 `forbidden_endpoint_calls=0`。
这与硬件写计数的断言一起构成无实机访问证据，不是硬件资格证明。

## 本次软件开销样本

机器可读结果：`results/6pus_dynamic_id/b4_software_20261020/software_evidence.json`。
原始测试日志在 pytest 临时目录。结果 JSON 为新增数据，没有修改旧结果。

相同六轴输入、相同模拟决策时钟，按 off/on/on/off 顺序各跑 80 个无节拍 cycle，
各模式合计 160 个样本。诊断时间读取不推进假时钟，原 transport/scheduler 读取与
显式场景推进才改变它；主机 perf_counter_ns 实测下列开销。包含背景线程竞争，
不模拟这些开销对真实 deadline 的反馈。没有固定 CPU 或作实时资格测试。

| 操作 | 关闭 p50 / p95 (μs) | 开启 p50 / p95 (μs) |
|---|---:|---:|
| publish（含原 Parent update） | 3.756 / 11.410 | 29.784 / 56.765 |
| send_state + receive + ingest | 31.060 / 92.520 | 76.314 / 162.798 |
| 原 core cycle 及诊断钩子 | 384.360 / 1007.884 | 714.929 / 1963.136 |

开启组的上述 max 分别为 2256.730、11770.654、6093.811 μs，单次主机尾部波动
不能从此归因为 UART/USB。开销明显非零，不能只用中位数推导实时保证。

正常开启的两轮各尝试/写入 2633 事件，均零丢失、最终完整性为 true；Child 队列
峰值分别为 1877/2095（容量 16384）。端到端写入吞吐约 13.14k/10.68k events/s，
分母含初始化和最终排空，是该短跑的平均值，不是磁盘持续吞吐资格。

| 压力场景（各 20 cycle） | 尝试 / 写入 / 丢失 | Child 峰值/容量 | 最终完整性 |
|---|---:|---:|---|
| 每次 Child 文件 write 延迟 0.5 ms | 713 / 248 / 465 | 128/128 | false |
| 阻塞 Child 写盘至生产者完成 | 713 / 35 / 678 | 8/8 | false |
| Child 文件写入抛 OSError | 713 / 120 / 593 | 128/128 | false |

压力场景的所有角色合计丢失数如上；容量应用于每个角色，阻塞场景 Parent/Shaper 也
达到 8。三者底层调用轨迹均与相同 20-cycle 关闭记录基线完全一致；实际计数具有
线程调度依赖，不应当作为固定预期值。慢盘例的生产者约 18.52 ms 完成，最终排空
另耗约 153.41 ms，等待发生在收尾。关闭记录的对应值约 9.91/3.14 ms。

QueueMeasuredTrace 在返回任何生产者前，为该 B1 Queue 实例的原 `_put` 增加峰值
计数，仍使用原 Queue mutex，没有更换队列或新增锁；峰值不含后台正在写的一条。
这是私有接口依赖，和 B3 parser/RX queue 依赖一起须在 Python/生产版本变化时重审。
普通快照、短锁、时间戳、入队和后台 GIL/写盘竞争都有成本，本轮不声称零开销。

复现（默认结果只写新的 pytest 临时目录）：

```bash
PYTHONDONTWRITEBYTECODE=1 \
PYTHONPATH=/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver \
python -m pytest -q -s -p no:cacheprovider \
  bone_ws/src/pus6_motor_driver/test/test_diagnostic_j14_integration.py \
  bone_ws/src/pus6_motor_driver/test/test_diagnostic_j14_event_chain.py \
  bone_ws/src/pus6_motor_driver/test/test_diagnostic_j14_serial_factory.py \
  bone_ws/src/pus6_motor_driver/test/test_diagnostic_serial_observer.py \
  bone_ws/src/pus6_motor_driver/test/test_j7_c9_real_uart_adapter.py
```

可选 B4_EVIDENCE_DIRECTORY 必须指定新目录，存在即拒绝写入，勿复用本次结果目录。
未覆盖：真实 UART/USB/ROS 和真实多进程启动握手、强退/掉电持久化、硬实时尾延迟、
真实存储负载下长期运行及诊断开销对通信 deadline 的影响。本轮结果仅为软件级证据。
