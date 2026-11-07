> 历史归档：原路径 `agent_docs/research/6PUS_DYNAMIC_ID_B2_J14_SERIAL_FACTORY.md`。正文保留当时的方案、参数及证据语境；当前原生 Mode 3 入口以[项目 README](../../../../README.md)为准。历史命令不作为当前推荐启动命令。

# 第四阶段 B2：J14 串口观测独立工厂

日期：2026-10-20。仅新增工厂、离线测试和本说明；不修改 B1 或 J14/J7/J8
生产文件，不接入部署环境，不启动节点/子进程，不打开串口、不提交 Git。

## 构造依据与接入边界

现有启动链为 Demo launcher → node host → production driver →
J14ProcessMotorDriverNode → ProcessClient → j14_uart_child_main → child_run →
ServoCore。ProcessClient 固定使用原 Child 入口，本阶段不替换它、不运行这条启动链。

`ServoCore(config, bus=...)` 已能接受现有 adapter，因此无需修改生产源码：

```text
调用者提供的同一个 existing_serial（独占所有权转交）
  → ObservedSerial 子类（仅增加 close-once 所有权处理）
  → 原 J7UartAdapter
  → 原 ServoCore(config, bus=adapter)
```

新增模块：`bone_ws/src/pus6_motor_driver/pus6_motor_driver/diagnostic_j14_serial_factory.py`。
调用接口为 `DiagnosticJ14Session(config, existing_serial,
recording_directory=NEW_DIRECTORY, run_id=RUN_ID)`，配合 context manager 或显式
close。没有串口创建工厂、CLI、自动启动或 fallback；不附着到正在运行的串口所有者。

保留原 active 分支的两次缓冲区 reset，各一次；既有 serial 应在此转交初始化所有权，
调用者不重复 reset。adapter 保留 baudrate、10 ms request timeout、0.1 ms 原有帧间隔
及原有 tx_queue_diagnostics 选择；原 read observer 在第一次 initialize 前连接到
core.rec.record_single_read。ServoCore、transport、scheduler、控制/安全逻辑原样复用。
reset 不属于新增通信；测试与原生产构造分支逐调用对照。

## 生命周期与失败语义

单一 UART 所有者线程使用；无并发关闭或跨线程接管。正常顺序：

1. 构造记录器 → 包装 existing_serial → 原有 reset → adapter → 注入 core。
2. initialize/cycle 委托原 core；不增加查询、轮询、重试、控制周期或等待。
3. close 调用原 core.close：原有一次性 stop/pause 的读写仍保持记录。
4. 原始 serial.close 仅尝试一次，结束后才关闭记录器、收集最终状态。

构造失败时关闭已转交的 serial；已存在 core 则先执行其原清理路径。
initialize/cycle 抛错时清理并保留原异常；已有 scheduler 对底层异常的包装与 cause
不变。core.cycle 返回原有 fault packet 时保持原行为，由调用者/context 收尾。
serial.close 或 core.close 抛错也会完成记录器关闭；原始关闭不重试，关闭结果不明
明确记为失败，不能将其解释为底层设备已释放。

`status()` 区分 `operation_error`、`cleanup_errors/cleanup_complete` 和 B1
`recording.complete`。清理失败使诊断记录标记不完整；记录器溢出或写盘错误不会
替换原串口/core 异常，也不成为运动安全决策。完整记录一次失败操作不等于操作成功。
若记录器排空超时，close 返回不完整；之后可再 close 完成排空，绝不再次关闭 core/serial。
记录器自身关闭异常提供内存失败状态；调用者应保留该状态，不假定磁盘已有最终结果。

每次使用新的 recording_directory（建议与新的 core 输出目录分开）。原有 J14
日志由 core 原样产生；本轮只在 pytest 临时目录产生测试输出，没有改历史实验结果。

## 实际离线验证

测试：`bone_ws/src/pus6_motor_driver/test/test_diagnostic_j14_serial_factory.py`。
新增 B2 **24 项**，连同 B1 和原 J7 C9 合计 **59 passed（1.53 s）**。

- FakeSerial/FakeBus 验证对象身份、构造顺序、配置、原有 read observer 及关闭顺序。
- 原生产构造分支的 Serial 构造器仅在测试局部替换为 FakeSerial，对照 direct /
  disabled / observed 的实际底层调用轨迹、参数、结果和 pause；每组仅创建一个 FakeSerial。
- 初始化异常及原 cause、运行异常、motion timeout/fault、一次 pause 且无 motion 重试。
- recorder/reset/adapter/core 构造失败，serial/core stop/core dump 关闭异常，以及主异常
  与清理异常同时发生。串口只关闭一次，记录器在全部串口调用后关闭。
- 队列溢出、写盘失败、记录器关闭失败、排空超时与二次完成均保留不完整状态或正确终态。

证据：测试默认禁止 serial.Serial、serial_for_url 和 subprocess.Popen；唯一例外是
上述**测试局部 FakeSerial 构造函数**，没有物理设备调用。所有传输均由 FakeBus
响应，每个 FakeSerial 的 `hardware_write_count == 0`，未运行 ROS 节点或 J14 进程。
原日志记录器自带的 atexit dump 也仅写测试临时目录，不做串口通信。

```bash
PYTHONDONTWRITEBYTECODE=1 \
PYTHONPATH=/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver \
python -m pytest -q -p no:cacheprovider \
  bone_ws/src/pus6_motor_driver/test/test_diagnostic_j14_serial_factory.py \
  bone_ws/src/pus6_motor_driver/test/test_diagnostic_serial_observer.py \
  bone_ws/src/pus6_motor_driver/test/test_j7_c9_real_uart_adapter.py
```

## 未覆盖与后续接口

未验证真实 UART/USB、进程信号强退、掉电持久化、实际运行开销或时序资格；单线程
所有权契约下通过，不支持活动串口并发接管。本阶段没有 Shaper、Parent/Child、逐轴
反馈事件扩展，也没有设备采样时刻或命令—反馈配对。

下一阶段可由**独立诊断 Child 入口**提供已有 serial 和相同配置，使用本工厂管理
core/serial/recorder，并汇总 close 返回状态；调度与 IPC 握手仍需后续单独离线核对。
本轮不宣称完成整条 J14 启动链替换或真机接入。`hardware_write_count = 0`。
