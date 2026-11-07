# 文档导航

当前项目为六轴 **Mode 3 / 0x27 原生力控**。先读[项目 README](../README.md)，再按主题阅读下表。旧 Position/Route A、任务空间力分配和阶段调参文件均有明确历史语境，不能用其命令和参数替代当前三进程入口。

| 主题 | 当前入口 | 内容 |
|---|---|---|
| 系统学习与讲解 | [Force 完整中文技术指南](technical_guide/README_CN.md) | 从控制原理、实际函数与参数，到三进程数据流、硬件执行和 Home 的完整阅读路线 |
| 系统架构 | [Parent / UART Child / Feedback Bridge](architecture/three_process.md) | UART 独占、IPC、时序、反馈与异步记录、停止边界 |
| 控制 | [双 IMU → IK → 六轴 PI-D](control/dual_imu_position_force.md) | 坐标、时间对齐、SO(3)/LADRC、位置反馈与力输出 |
| 输出策略 | [原生能力策略与回放](control/native_capacity.md) | ±50 N 依据、取消的实验限幅、39 项软件测试与历史输入回放 |
| 硬件 | [电机、力传感器及寄存器](hardware/native_force.md) | g、N、产品规格与固件保护的区别 |
| 日常运行 | [长期启动、日志和停止](operations/long_running.md) | 环境、持续运行、正常/故障退出 |
| Home | [Home 来源与退出返回](operations/home.md) | 配置、J7 Servo runner、到位判定、实际证据及局限 |
| 人工恢复 | [人支撑柔顺恢复记录](operations/manual_recovery.md) | 独立历史恢复入口；不是自动 Home |
| 退出历史 | [Force 停止逻辑修复](operations/force_shutdown_history.md) | 已发现的停止错误及当时修正依据 |
| 开发 | [编译和软件检查](development/build.md) | 工作空间依赖、入口与检查范围 |
| 实验 | [运动、故障与验证索引](experiments/README.md) | 真机与 Mock/回放分开，原结果目录保留 |
| 历史 | [归档导航](archive/README.md) | Force 早期阶段、Position/Torque 参考、UART 与研究资料 |
| 全量清单 | [全部 Markdown 分类](development/markdown_catalog.md) | 整理前 402 份非生成 Markdown 的逐文件归属 |
| 路径变化 | [迁移对照](development/documentation_map.md) | 旧位置、新正文和必要兼容页 |

## 目录职责

```text
docs/
├── technical_guide/        从控制原理到源码执行的完整中文讲解
├── architecture/           当前三进程架构
├── control/                当前控制链与原生能力输出策略
├── hardware/               厂家资料和寄存器解释
├── operations/             长期运行、Home、恢复
├── development/            编译、文档分类及迁移映射
├── experiments/
│   ├── motion/             Z、单轴、组合轨迹记录
│   ├── native_motor/       单电机和拆卸归零记录
│   └── faults/             双 IMU 故障分析
└── archive/
    ├── force_stages/       Stage 1–5 方案与结果
    ├── force_history/      双 IMU、动态优化、工作空间及恢复演进
    ├── route_a/            整理前两个首页全文
    ├── position_reference/ 继承的 Position 操作说明和教程
    ├── inherited/          继承的非冻结架构、研究和审查材料
    ├── research/           TDE、辅助平移、Torque/Route A 研究
    ├── uart_history/       独立 UART 验证历史
    └── hardware_reference/ 历史协议教学资料
```

## 原位保留的资料

`results/` 中 186 份 Markdown 属于实验输出、迁移记录、原始摘录和交付包；包括包内重复副本，保留内容和路径，不进行去重覆盖。`agent_docs/` 中冻结/J-stage/验证报告、参考包及其导航保留；可迁移的 11 份非冻结架构/研究文档已进入 `archive/inherited/`。`AGENTS.md`、模块 README、供应商知识库和日志 README 保留原位置。它们都在全量分类清单中，不因扩展名相同而搬离原始上下文。

旧文件名若被控制脚本或冻结文档引用，留下简短兼容入口；正文只维护一份。旧首页的 Route A 内容完整归档，根目录 `README.md` / `README_CN.md` 现分别提供完整英文/中文项目介绍。当前说明和归档说明发生差异时，首先比较入口、日期和证据等级，而不是把旧数值强行更新到历史正文中。

本轮没有修改任何程序、参数、配置、测试或历史运行数据，没有修改正式保留 Tag。具体链接检查范围和遗留情况见[文档检查说明](development/documentation_checks.md)。
