> 历史归档：原路径 `agent_docs/research/6PUS_DYNAMIC_ID_B3_EVENT_CHAIN.md`。正文保留当时的方案、参数及证据语境；当前原生 Mode 3 入口以[项目 README](../../../../README.md)为准。历史命令不作为当前推荐启动命令。

# 第四阶段 B3：命令—串口—反馈事件关联（离线）

日期：2026-10-20。只新增两个诊断模块、一个测试文件和本说明；B1/B2、J7/J8/J14
生产文件、消息、配置、历史结果及冻结资产未修改。不部署、不启动 ROS/进程、不打开
串口、不运行 MATLAB、不提交 Git。`hardware_write_count = 0`。

## 独立组件与使用边界

- `diagnostic_event_trace.py`：EventTrace 继承 B1 队列、后台写盘与完整性处理，增加
  run/host/boot/process/recorder/event 标识、统一 CLOCK_MONOTONIC ns 和只读合并器。
  B1 ObservedSerial 的原始 SerialCall 保留原字段与 hex 编码，不另做串口读取。
- `diagnostic_j14_event_chain.py`：通过 B2 的 recorder/adapter/core 注入工厂复用
  原 ServoCore、J7UartAdapter、J7RxOwner、CommandState 的计算与通信决策。
  其他组件是实例级 PublisherObserver、SocketObserver、ParentClientObserver、ParentLockObserver；没有
  全局 monkey patch、生产入口替换或第二个串口所有者。
- `traced_session(config, existing_serial, trace_directory=..., run_id=...)` 只构造
  独立诊断 session，不启动 Child loop。资源顺序仍由 B2 管理，停止/暂停通信及底层
  close 尝试结束后才关闭记录器。当前没有自动接入 Parent/Shaper 的部署入口。

未来独立诊断入口可在对象被工作线程共享前，显式包装 Shaper 的 cmd_steps_pub，
序号 getter 读取 node._seq；包装 ProcessClient 的 sock 和原 lock；Child 使用本
session，并将其原 IPC socket 包装后交给原 receive_packets。启动前已发生的事件
不在覆盖范围内，不能把实例中途挂接描述为完整记录。

## 已建立的关联与字段

| 事件 | 真实来源与关联 |
|---|---|
| shaper_publish | 原 publish 的六轴输入、可用本地 _seq、主机调用开始/返回边界；生成时间未知，序号不在原消息中传输 |
| parent_command_receive / parent_update_return | 原 ProcessClient.update 调用边界、传入序号/时间、原校验异常；正常返回本身不证明采用，通过 call event ID 关联锁内结果 |
| parent_latest_transition | 在同一个原 ProcessClient 锁内观察 latest 的前后值，记录 pending 覆盖/清空；没有增加锁获取次数 |
| ipc_send / ipc_receive | 原 send/recvmsg 调用的原始 datagram、时间、返回字节数/flags/异常；BlockingIOError 发送失败单独保存，不发起额外重试 |
| child_command_ingest | 原 ingest 输入批、接收代理时间、前后 seq/consumed/rejected、实际保留序号及 pending 替换；重复同序号输入无法唯一归属时标记 ambiguous |
| child_candidate / dispatch | 原 candidate 返回向量、warning-or-backlog/consumed 实参、实际 send_cycle 的最终 targets 和 cycle；记录持值来源及最终 slew 是否改变候选值 |
| serial | 原 B1 read/write 起止、hex 字节、返回长度与异常，附当前观察 cycle；设备事务对应未知 |
| rx_frame | 原 J7RxOwner.ingest 返回的每个帧对象、轴号/寄存器/raw frame、已有解码器旁路解码的 raw_steps、涉及的 read event IDs 与字节范围 |
| rx_extraction / transport_feedback | 原 J7 每批首次每轴选取规则及重复帧；通过实际返回 StatusFeedback 对象身份连接最终 transport audit.feedbacks，保留跨 drain 覆盖后的真实对象 |
| feedback_adoption | 原 core.update 完成后的六轴缓存、实际采用帧 ID、新反馈是否存在、缺轴、旧缓存复用、缓存是否有已观察来源、批量 ingest 时间 |
| rx_drained / cycle_result | 原路径丢弃的残留帧、cycle 是否实际到达 update；提前 fault 不会被记录为完成反馈采用 |

帧来源使用原解析器保留的 partial buffer 加本次真实 read 字节，按实际返回 frame.raw
定位字节跨度；不创建新解析器或改变原解析结果。重复帧的首次选择映射依赖当前原
J7 的已核实规则，并核对返回 status 内容；采用过程另外按对象身份关联，不能只靠
位置值相等判断。涉及原私有 parser buffer/RX queue，后续版本变更必须重审与重测。

一个 read 中的多个帧保留共同 read 返回时刻；一次 ingest 的多个帧也保留共同的 batch
返回边界，没有人为分配逐轴采样时间。分段帧可以引用多个 read；串口返回时间仍只是
主机可见代理，不是线上最后一位抵达时刻。ACTIVE 的 full_scan 字段同样为 true，
不能用它识别 0x30 查询；本组件使用实际 initialize/cycle 执行范围区分来源。

## 明确停止扩展的缺口

1. Shaper 的 Int32MultiArray 不携带源序号/单调时间，不能唯一证明某次 Parent 接收
   来自哪次 Shaper publish。保留本地 seq 及发布边界，source_generated_ns=null；不按
   相同目标或最近时间强配。最小后续改动是独立诊断消息/接收链保留源身份及时间。
2. MotorDriver 的 _latest_target/_target_overwrite_count 与 ProcessClient.latest 是
   两个不同层。当前只观察后者；不把其计数冒充 Driver envelope 的拒绝/覆盖。若要
   补齐，需要在独立诊断 Driver 的 _target_lock 内提供接收/分支结果事件观测点。
3. 设备没有测量时间或回显 UART cycle sequence。帧只关联到主机读取与实际采用；
   device_measurement_ns/device_transaction_sequence=null，物理命令—反馈对应为
   UNKNOWN_OR_AMBIGUOUS。迟到响应在当前 cycle 被读到不证明由当前命令触发。
4. 原 FrameStreamParser.feed 批内抛错可能不返回此前已经解出的局部帧；本组件只保存
   原始 read、parse_error，并标记观测覆盖不完整。若必须逐一获取这些内部局部结果，
   最小必要改动是诊断专用解析器观测回调；本轮没有修改原解析器或重新解析补填。
5. warning 实参合并了 stale/backlog，未暴露的细分局部原因不猜测；基于实际候选与
   最终目标比较得到的 slew_changed 是诊断计算标志，不是设备限速/饱和测量。

## 多进程合并与完整性

每个 recorder 使用 role+唯一 ID 的独立新目录，写自己的 events.jsonl/status.json。
合并器要求相同 run_id、host_id、boot_id；不同域拒绝时间比较，不以 UTC 校正。
按单调时间排序只是确定性展示，不代表同时间事件的因果先后。

merge_traces 检查最终状态、文件计数、event_seq 缺口、重复 ID、悬空引用及可选的
expected_roles。IPC 仅给出同 channel、相同原始 datagram、成功 send 的观察候选；
多次相同 datagram 保留歧义，不按“最近一次”强配。unique_observed_payload 也仅在
所查记录范围内唯一，不是设备事务对应证明。

complete 只表示声明的日志存储/引用检查通过，不表示上述未知量变成可观测，或整条
生产启动链已覆盖。队列满、后台文件错误、解析内部覆盖缺口使完整性明确失败；记录
失败不作为控制/通信/安全条件。原始字节仍尽量保存，最终状态无法写盘时保留内存状态。

## 实际离线验证

`test_diagnostic_j14_event_chain.py` 新增 **19 项**，与 B1/B2 和原 J7 C9 合并执行，
实际结果 **78 passed（2.53 s）**。测试输出只在 pytest 临时目录。

- 分段、多帧合并、不同内容的同轴重复、错误帧、缺轴缓存复用、迟到残留清理。
- 模拟 Shaper→原 ProcessClient.update/send_state→原 receive_packets→原
  CommandState→原 ServoCore→FakeSerial/FakeBus→实际缓存采用；逐轴核对 source frame
  raw_steps 与最终采用值，并沿字节跨度恢复原始帧。
- 模拟跨进程身份/独立目录合并（没有启动真实进程）、IPC 满/重复 datagram、不同
  boot、悬空引用、缺少声明角色以及多个生产线程共享原 Parent 锁的覆盖计数。
- B2/B3 正常、缺轴、读异常、写超时的底层调用轨迹、参数、结果、异常 cause、关闭
  及无新增重试对照。缺轴对照注入确定性 transport 时钟；不是零开销或真机时序证明。
- 队列满、慢写盘、写盘失败及原 publish/socket 异常传播。

测试禁止 serial.Serial、serial_for_url、subprocess.Popen；B2 原生产构造分支对照
仅局部使用 FakeSerial 替代构造器。所有通信来自 FakeSerial/FakeBus，硬件写计数
断言为零。未启动 ROS、J14 进程或物理串口。未验证真实 CPU/调度开销、USB 缓冲、
进程强退、掉电持久化、或完整诊断启动器；不能据此声称真机动态辨识完成。
