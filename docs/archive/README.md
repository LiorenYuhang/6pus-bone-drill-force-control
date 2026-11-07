# 历史归档导航

本文集中说明历史资料的用途，当前原生力控以[项目 README](../../README.md)、[控制链](../control/dual_imu_position_force.md)和[运行说明](../operations/long_running.md)为准。归档正文保留原日期、参数、方案、验证结果和当时局限；其中旧命令不是当前推荐启动入口。

| 目录 | 历史内容 |
|---|---|
| [force_stages](force_stages) | Stage 1–5：反馈链、Mode 3 交接、Tare、任务力探索、位置反馈执行层与失败分析 |
| [force_history](force_history) | 双 IMU 初次集成、速度/D 优化、工作空间、三进程迁移与启动竞态、手动恢复演进 |
| [route_a](route_a) | 整理前中英文首页全文；包含原未提交 Markdown 内容，保留 Force/Route A 迁移语境 |
| [position_reference](position_reference) | 继承的 Position J11/J14 指南和教程；Servo 控制语义不能套用到 Mode 3 |
| [inherited](inherited) | 迁入的非冻结架构、只读研究、延迟/辨识分析和旧操作方案 |
| [research](research) | Torque、TDE、合成模型、离线姿态与辅助平移研究；不是当前主控制链 |
| [uart_history](uart_history) | 六机编号、M2-CAL、ID4 偏置/上电恢复和 UART 真实运行记录 |
| [hardware_reference](hardware_reference) | 历史 Modbus/USB-TTL 教程；不替代当前厂家协议和真实驱动 |

当前历史双 IMU/三进程说明有较多重复。现在分别集中到 [architecture/three_process.md](../architecture/three_process.md)、[control/dual_imu_position_force.md](../control/dual_imu_position_force.md)、[operations/long_running.md](../operations/long_running.md)；旧全文作为决策与验证依据保留，不继续维护成另一套当前指南。旧首页和 `README_CN.md` 的当前说明已统一到根 README。

## 保留原位的历史材料

- [冻结资产](../../agent_docs/frozen)：J4/J5 清单与报告，禁止移动改写。
- [J-stage 历史](../../agent_docs/stages)、[设计决策](../../agent_docs/decisions)、[已验证结果报告](../../agent_docs/reports)、[J6 实机记录](../../agent_docs/j6)：保留冻结/验证上下文和旧引用。
- [J14 长期运行总结](../../agent_docs/stages/J14/J14_FINAL_SUMMARY.md)与[正式化记录](../../agent_docs/stages/J14/J14_A2_P1_PRODUCTIONIZATION.md)：Position 架构依据，不证明 Force 的相同负载表现。
- [TDE 参考包](../../agent_docs/stewart_TDE_reference_package)：保留包结构；其中“当前架构”指包生成时的 Position 研究状态。
- [实验结果总入口](../../results/README.md)、[J14 博客资料包](../../results/j14_blog_prep/README.md)：原始输出及包内复制文件保持不变，不为文档去重破坏交付内容。
- STM32、H30、底层模块及供应商说明保留各自目录，见[逐文件分类清单](../development/markdown_catalog.md)。

本轮移动 76 份正文，另保留整理前两个首页到历史目录。对被程序或只读证据引用的 13 个旧位置保留简短兼容页；不会删除原实验文件或冻结资产。所有正文新旧路径见[迁移对照](../development/documentation_map.md)。
