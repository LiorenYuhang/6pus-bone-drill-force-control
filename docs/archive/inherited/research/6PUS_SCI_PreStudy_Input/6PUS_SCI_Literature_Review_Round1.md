> 历史归档：原路径 `agent_docs/research/6PUS_SCI_PreStudy_Input/6PUS_SCI_Literature_Review_Round1.md`。正文保留当时的方案、参数及证据语境；当前原生 Mode 3 入口以[项目 README](../../../../../README.md)为准。历史命令不作为当前推荐启动命令。

# 6PUS博士课题SCI论文：第一轮文献检索与创新性调查

**检索日期：2026年10月16日（北京时间）**  
**研究对象：移动基座6PUS并联机器人，优先考虑手持骨钻的姿态稳定任务。**  
**本轮性质：问题筛选与创新性冲突调查，不确定新算法，不开展MATLAB、Simulink或ROS2研发。**

## 阅读结论

**不建议以“移动基座Stewart平台＋姿态优先＋辅助平移＋约束QP”这一宽泛组合直接立项或撰写创新点。** 其主要组成部分已有明确先例，且2025—2026年的工作比初始线索更接近候选问题：

- Khan等已明确提出六驱动Stewart平台三维姿态任务下的剩余自由度利用与机械约束优化。[R01]
- Mohammed与Li的2016年论文已把Stewart任务冗余写成约束QP；其作者学位论文提供了可核对的优化公式。[R03，S02]
- Lin等的2025年论文直接研究**浮动基座6-PUS稳定机构**，公开预览还描述了基座与上平台的双IMU配置。[R13]
- 船载双环MPC、饱和约束预测规划、Stewart执行器速度/加速度约束规划均已有研究。[R19—R23]
- 手持并联手术工具的基座扰动前馈、工具尖端稳定与受限工作空间也不是新应用概念。[R35—R38]

**本轮没有确认某一篇论文完整覆盖“具体6PUS结构＋实际位置伺服＋移动基座＋严格任务优先＋钻尖/进给约束＋持续可行性”全部条件。这个“未确认”不能转写为研究空白。** 目前最值得讨论的是：真实钻削任务还留下多少辅助运动，以及在持续基座扰动和有限执行能力下，何时必须允许姿态误差或请求人工重新定位。

---

## 第一部分：检索范围、检索方法与关键词

### 1.1 研究边界与证据标签

现有模型、解析IK/FK/Jacobian、双IMU、ROS2硬件、约50 Hz位置指令以及PD/PID、LADRC等工程条件，均按本次用户提供的背景处理；**本轮没有重新审计工程或复核硬件指标**。既有理想广义力TDE仿真不作为位置伺服实机可行性的证据。

时间优先覆盖2021年至检索日已公开的论文，并保留与任务冗余、任务层级、参考整形、手持手术工具直接相关的经典文献。没有以期刊分区作为纳入标准；本轮未查询JCR或中科院分区，不能据此承诺SCI二区/三区录用前景。

证据等级用于说明本轮实际读取程度，不评价论文质量：

| 标签 | 本轮掌握的材料 | 可以支持的判断 |
|---|---|---|
| F | 已取得并读取相关方法、约束和验证章节的全文/作者稿 | 可讨论已读章节、公式与实验条件；作者稿不等同最终排版版 |
| T | 期刊正文未取得，但取得作者学位论文的对应章节 | 公式和页码只归属于学位论文，不假定与期刊逐字相同 |
| A | 出版社/作者机构的摘要、引言或章节预览 | 只确认公开明确陈述的研究内容；证明假设和实验细节仍待全文 |
| M | 书目已核实，方法未充分审读 | 不据标题补写控制输入、约束、硬件实验或理论结论 |

正文另外区分：**论文事实**、**作者声明**、**本报告推论**、**待验证假设**。A等级论文的“有理论分析”“有实验”首先是作者公开描述，并不表示本轮已审查证明或实验原始数据。表中“未确认”不是“论文没有”。

### 1.2 实际使用的来源和访问限制

实际使用了：Crossref REST元数据、OpenAlex引用图、出版社网页/公开预览，以及PolyU、CUHK、TU Wien、TU Delft、CMU、HAL和日本学位论文仓储等来源。技术判断优先回到论文或作者机构材料。ResearchGate和其他聚合页仅用于寻找标题、DOI或可获取版本，未把其推荐阅读区的其他论文内容归给目标论文。

本环境没有可调用的专门academic-search MCP、Scopus或Web of Science订阅检索工具；也没有将普通网页搜索冒充这些数据库的完整检索。Crossref和OpenAlex通过公开API访问；IEEE、Wiley、Elsevier部分正文受访问限制或返回403，Elsevier API对部分请求仅返回书目。没有购买全文，也没有绕过机构权限。

**检索性质：可追溯的第一轮范围调查，尚不是可声明穷尽性的系统综述。** 搜索引擎排序、数据库收录滞后、未取得全文和未完整遍历所有引用，是明确限制。

### 1.3 检索式与扩展过程

以下英文检索式代表本轮实际使用的组合及后续扩展；网页搜索与数据库API对布尔语法的解释不同，复检时应按各数据库语法调整。

| 路线 | 核心英文检索组合 | 主要目的 |
|---|---|---|
| A1 | `"Stewart" "orientation" "redundancy"`；`"Stewart" "orientation-only" control constraints` | 姿态任务冗余及其命名 |
| A2 | `"Stewart" "task-priority" stabilization translation`；`Stewart platform orientation task priority constrained optimization translation moving base` | 姿态优先与辅助平移 |
| A3 | `"joint-constrained" "Gough-Stewart"`；`"Stewart" "actuator constraints"` | 关节约束、QP及约束管理 |
| A4 | `"6-PUS" "orientation" "optimization" control`；`"6-PUS" "floating base"` | 避免只检索传统6-UPS |
| B1 | `"moving-base" "Stewart"`；`"base motion compensators"` | 移动坐标系、惯性稳定 |
| B2 | `"Stewart" "wave compensation" "constraints" predictive` | 船载约束补偿和MPC |
| B3 | `"Stewart" "motion compensation" "workspace" "optimization"`；`"Stewart" "motion compensation" "feasible"` | 工作空间、参考可行性 |
| B4 | `"Stewart" "wave compensation" "predictive" "2025"`；`"Stewart" "moving-base" "2026" control` | 补充近两年工作 |
| C1 | `"Stewart" "dual IMU" estimation`；`"Improved Stewart platform state estimation"` | IMU与执行器测量融合 |
| C2 | `"parallel" robot "asynchronous" "inertial" estimation`；`"Online Temporal Calibration for Monocular Visual-Inertial Systems"` | 异步、时间偏移及通用先例 |
| D1 | `"handheld" "bone" "drill" "Stewart"`；`"handheld" "drill" "parallel robot" control stabilization` | 骨钻、移动手柄与并联结构 |
| D2 | `"Stewart" "tool-tip" "workspace" control`；`"handheld" "drilling" "robot" "6-PUS"` | 钻尖、RCM及可用辅助运动 |
| 核验 | 初始五条线索的完整英文标题、DOI精确检索 | 核实作者、出版年份与实际内容 |

扩展遵循结果而不是预设创新点：从R03/R02扩展到离散神经动力学；从Ono扩展到直接6-PUS浮动基座研究；从船载MPC扩展到饱和预测规划和在线轨迹缩放；从手持稳定扩展到Micron与ITD。

### 1.4 引用追踪、去重和纳入记录

以DOI小写标准化去重，同一论文的在线发表年与卷期年不重复计数；期刊论文、早期会议和作者学位论文分别标注。

通过OpenAlex对五个种子做了带截止日期`to_publication_date:2026-10-16`的前向追踪：

| 种子 | OpenAlex ID | 接口返回命中数 | 本轮实际取回 |
|---|---|---:|---:|
| R01 Khan | W4411349426 | 5 | 5 |
| R02 Li | W4388145478 | 17 | 17 |
| R03 Mohammed | W949575934 | 91 | 50（首批，未穷尽） |
| R14 Vu | W4412561122 | 4 | 4 |
| R11 Ono I | W3214773014 | 27 | 27 |

上述共取回103条记录，按OpenAlex ID去重为101条。**这只是引用追踪子集的计数，不是全部搜索的PRISMA筛选流量，也不是最终纳入篇数。** 股票预测、多智能体等明显非相关引用未因引用关系而纳入。R02/R03的追踪确认了R01与R05；R11的追踪发现R13、R22及P01/P02。又利用R13和R14公开参考文献回查船载速度前馈、延迟补偿、6-PUS结构优化等研究。

最终保留**40篇正式文献R01—R40**，另列2篇新近书目线索P01/P02、2份勘误及2份作者学位论文。40篇中，8篇取得并读取相关章节全文（F），2篇以作者学位论文对应章节补充（T）；其余按A/M保留。**没有把8—12篇全部重点论文伪报为已获全文。** 第四部分分析12组重点问题，合计涉及13篇重点论文，另外加入已获全文的方法核查。

纳入理由是对任务冗余、约束补偿、状态估计或手持任务边界有直接帮助；没有为凑数系统收集所有PID、滑模、神经网络调参论文。2026年10月16日之后的正式卷期日期，不自动证明该文在截止日前已全文公开；P01/P02因此保留单独状态。

## 第二部分：相关研究总体进展

### 2.1 主线A：任务冗余与约束优化已有成熟数学工具

R03的期刊摘要明确是约束QP；S02第二章可见低维任务约束、执行器速度映射及线性不等式。R01进一步明确针对三维姿态任务利用三个剩余自由度。R02则明确在冗余Gough–Stewart机器人上处理关节约束，但不能在未读全文时认定其主任务一定是姿态。[R01—R03，S02]

HQP允许按严格优先级处理相冲突的等式和不等式；SNS包含硬关节限制和任务不可行时的缩放。[R08，R09] 因而，“给姿态较高权重”“加入关节行程限制”“用零空间做次任务”“主任务不可行时加松弛量”都不能独立构成新科学机制。**有限大权重通常不等同严格任务优先。**

需要区分三类“冗余”：

1. **任务冗余**：六自由度机构只执行低维任务，本轮候选方向属于这一类。
2. **机构运动学冗余**：通过额外运动单元获得更多内部构型选择，如R06的新机构族。
3. **驱动冗余**：同一完整任务的独立驱动数超过所需数量；不能用它描述普通六驱动六自由度6PUS。

R10的6-PUS多目标结构优化与R40的Hexafloat设计说明，几何、工作空间和执行器能力联合考虑也有先例；**离线改变结构尺寸**与**在线在既定机构中分配辅助运动**仍须分开。[R10，R40]

### 2.2 主线B：移动基座惯性稳定、受限规划与伺服动态不能分割看待

Ono I以移动坐标系建立运动补偿关系，并有IK＋PID缩比实机；Ono II研究逆动力学并报告实机比较。二者已经覆盖“基座运动时保持上平台世界系稳定”这个基本目标。[R11，R12]

更直接的R13使用6-PUS结构、浮动基座动力学和多源扰动抑制。其公开材料提到上平台和基座IMU，不能再声称“6PUS＋移动基座＋双IMU”本身是空白。[R13]

R19明确将执行器行程放入外环运动规划，并建模液压内环；R20也采用外层预测规划、内层执行器控制。R21研究饱和约束下预测QP参考规划。R23已在电缸Stewart平台上研究速度、加速度限制和构型相关能力调节。[R19—R21，R23] 这些先例意味着：约束必须在参考生成阶段处理、执行器不是理想运动源，均已有明确研究基础。

但不能反向夸大：知道某论文包含MPC或CBF，不代表它已经保证任意扰动下递归可行；知道轨迹满足速度、加速度界，也不代表满足电机力矩、电流、接触力和真实跟踪误差要求。具体保证必须核对模型、初始集、扰动界和输入接口。

### 2.3 辅助方向C：融合与延迟本身已有先例，具体可观测性仍需查清

R32已用平台IMU与六路执行器位置做紧耦合UKF，并以SIMONA真实数据实时验证。其作者稿明确把测量时间不同步、安装误差等列为尚未纳入的问题；R33的2025年论文进一步把闭链几何直接加入ESKF更新。[R32，R33]

因此“IMU＋编码器融合”“闭链约束融合”“换成ESKF”均不宜作为独立创新。异步测量和在线时间标定也已有跨机器人领域的方法先例。[R34] 真正需要进一步调查的是**移动基座、两套姿态流、执行器反馈在不同测量时刻到达时，哪些状态/延迟可辨识，以及估计误差怎样影响受限补偿**。本轮尚未确认这组具体问题的已有完整解。

### 2.4 辅助方向D：姿态稳定不等于钻尖任务稳定

ITD在2007年已研究手持骨加工并联机器人的基座扰动前馈；Micron已把工具尖端、RCM、工作空间和有限驱动力联系起来；Hung等2021年报告了手持骨科机器人及猪脊柱钻孔实验。[R35—R38] 应用改成骨钻不是方法新颖性的充分理由。

不能直接把眼科RCM要求搬到骨钻：是否约束进入点、钻轴、沿轴进给、滚转或接触力，应由实际任务定义。眼科Micron提供的是“工具任务决定可用自由度”的已有实例，不是本课题临床指标的来源。

## 第三部分：核心文献清单

作者、正式题目、期刊/会议、卷期页码与DOI以Crossref及出版社/作者机构记录交叉核实。DOI中出现的年份不等同最终卷期年。下面的主题说明是本轮筛选用途；M级不据标题扩写方法。

### A：任务冗余、任务层级与6PUS结构

<a id="r01"></a>**[R01]** Ameer Hamza Khan; Xinwei Cao; Shuai Li (2025). **Recurrent Neural Network–Based Nonlinear Orientation Control of Redundant Stewart Platform**. *Journal of Field Robotics*, 42(7): 3952-3967. [DOI: 10.1002/rob.22605](https://doi.org/10.1002/rob.22605)；[已读取材料/内容入口](https://research.polyu.edu.hk/en/publications/recurrent-neural-networkbased-nonlinear-orientation-control-of-re/)。**A｜姿态任务冗余、非线性约束优化。**

<a id="r02"></a>**[R02]** Weibing Li; Yanying Zou; Xin Ma; Binbin Qiu; Dongsheng Guo (2024). **Novel Neural Controllers for Kinematic Redundancy Resolution of Joint-Constrained Gough–Stewart Robot**. *IEEE Transactions on Industrial Informatics*, 20(3): 4559-4570. [DOI: 10.1109/tii.2023.3326553](https://doi.org/10.1109/tii.2023.3326553)；[已读取材料/内容入口](https://research.cuhk.edu.hk/en/publications/novel-neural-controllers-for-kinematic-redundancy-resolution-of-j-2/)。**A｜关节受限冗余控制；正文待取得。**

<a id="r03"></a>**[R03]** Aquil Mirza Mohammed; Shuai Li (2016). **Dynamic Neural Networks for Kinematic Redundancy Resolution of Parallel Stewart Platforms**. *IEEE Transactions on Cybernetics*, 46(7): 1538-1550. [DOI: 10.1109/tcyb.2015.2451213](https://doi.org/10.1109/tcyb.2015.2451213)；[已读取材料/内容入口](https://pubmed.ncbi.nlm.nih.gov/26219101/)。**T｜约束QP；公式以S02对应章节补充。**

<a id="r04"></a>**[R04]** Mohammed Aquil Mirza; Shuai Li; Long Jin (2017). **Simultaneous learning and control of parallel Stewart platforms with unknown parameters**. *Neurocomputing*, 266: 114-122. [DOI: 10.1016/j.neucom.2017.05.026](https://doi.org/10.1016/j.neucom.2017.05.026)；[已读取材料/内容入口](https://www.sciencedirect.com/science/article/pii/S0925231217308639)。**A｜未知参数下学习与任务冗余控制。**

<a id="r05"></a>**[R05]** Yang Shi; Wangrong Sheng; Jie Wang; Long Jin; Bin Li; Xiaobing Sun (2024). **Real-Time Tracking Control and Efficiency Analyses for Stewart Platform Based on Discrete-Time Recurrent Neural Network**. *IEEE Transactions on Systems, Man, and Cybernetics: Systems*, 54(8): 5099-5111. [DOI: 10.1109/tsmc.2024.3392848](https://doi.org/10.1109/tsmc.2024.3392848)。**M｜由R02/R03引用追踪发现；方法和实验未充分核验。**

<a id="r06"></a>**[R06]** Clement Gosselin; Louis-Thomas Schreiber (2016). **Kinematically Redundant Spatial Parallel Mechanisms for Singularity Avoidance and Large Orientational Workspace**. *IEEE Transactions on Robotics*, 32(2): 286-300. [DOI: 10.1109/tro.2016.2516025](https://doi.org/10.1109/tro.2016.2516025)。**M｜结构运动学冗余的对照条目；不能与本机任务冗余混称。**

<a id="r07"></a>**[R07]** Oussama Kanoun; Florent Lamiraux; Pierre-Brice Wieber (2011). **Kinematic Control of Redundant Manipulators: Generalizing the Task-Priority Framework to Inequality Task**. *IEEE Transactions on Robotics*, 27(4): 785-792. [DOI: 10.1109/tro.2011.2142450](https://doi.org/10.1109/tro.2011.2142450)。**M｜不等式任务优先框架；详细能力以R08/R09全文为依据。**

<a id="r08"></a>**[R08]** Adrien Escande; Nicolas Mansard; Pierre-Brice Wieber (2014). **Hierarchical quadratic programming: Fast online humanoid-robot motion generation**. *The International Journal of Robotics Research*, 33(7): 1006-1028. [DOI: 10.1177/0278364914521306](https://doi.org/10.1177/0278364914521306)；[已读取材料/内容入口](https://hal.science/hal-00751924/document)。**F｜取得HAL较早两部分作者稿；最终期刊版未逐页对照。**

<a id="r09"></a>**[R09]** Fabrizio Flacco; Alessandro De Luca; Oussama Khatib (2015). **Control of Redundant Robots Under Hard Joint Constraints: Saturation in the Null Space**. *IEEE Transactions on Robotics*, 31(3): 637-654. [DOI: 10.1109/tro.2015.2418582](https://doi.org/10.1109/tro.2015.2418582)；[已读取材料/内容入口](https://iris.uniroma1.it/retrieve/e383532a-4954-15e8-e053-a505fe0a3de9/Flacco_Postprint_Control-of-Redundant_2015.pdf)。**F｜SNS硬限制、任务缩放及实机。**

<a id="r10"></a>**[R10]** S. Nader Nabavi; Morteza Shariatee; Javad Enferadi; Alireza Akbarzadeh (2020). **Parametric design and multi-objective optimization of a general 6-PUS parallel manipulator**. *Mechanism and Machine Theory*, 152: 103913. [DOI: 10.1016/j.mechmachtheory.2020.103913](https://doi.org/10.1016/j.mechmachtheory.2020.103913)；[已读取材料/内容入口](https://www.sciencedirect.com/science/article/abs/pii/S0094114X20301348)。**A｜6PUS与6UPS结构/工作空间性能比较。**

### B：移动基座、有限执行能力与参考可行性

<a id="r11"></a>**[R11]** Takeyuki Ono; Ryosuke Eto; Junya Yamakawa; Hidenori Murakami (2021). **Analysis and control of a Stewart platform as base motion compensators - Part I: Kinematics using moving frames**. *Nonlinear Dynamics*, 107(1): 51-76. [DOI: 10.1007/s11071-021-06767-8](https://doi.org/10.1007/s11071-021-06767-8)；[已读取材料/内容入口](https://link.springer.com/article/10.1007/s11071-021-06767-8)。**F｜移动基座运动学、IK/PID硬件；另见C01勘误。**

<a id="r12"></a>**[R12]** Takeyuki Ono; Ryosuke Eto; Junya Yamakawa; Hidenori Murakami (2021). **Analysis and control of a Stewart platform as base motion compensators—part II: dynamics**. *Nonlinear Dynamics*, 106(4): 3161-3182. [DOI: 10.1007/s11071-021-06749-w](https://doi.org/10.1007/s11071-021-06749-w)；[已读取材料/内容入口](https://link.springer.com/article/10.1007/s11071-021-06749-w)。**T｜期刊摘要＋S01相关章节；另见C02勘误。**

<a id="r13"></a>**[R13]** Chaoxiong Lin; Baogeng Xin; Shuojie Wang; Songlin Zhou; Weixing Chen; Feng Gao (2025). **Dynamic characteristics analysis and multi-source disturbances rejection control of a 6-PUS parallel stabilization mechanism on floating base**. *Mechanism and Machine Theory*, 205: 105888. [DOI: 10.1016/j.mechmachtheory.2024.105888](https://doi.org/10.1016/j.mechmachtheory.2024.105888)；[已读取材料/内容入口](https://www.sciencedirect.com/science/article/abs/pii/S0094114X2400315X)。**A｜直接6PUS浮动基座、双IMU预览与仿真。**

<a id="r14"></a>**[R14]** Duc Cuong Vu; Danh Huy Nguyen; Minh Nhat Vu; Tung Lam Nguyen (2025). **Lagrangian-based modeling and safety-critical controls for Stewart platforms under marine operations**. *Ocean Engineering*, 340: 122142. [DOI: 10.1016/j.oceaneng.2025.122142](https://doi.org/10.1016/j.oceaneng.2025.122142)；[已读取材料/内容入口](https://repositum.tuwien.at/handle/20.500.12708/226136)。**A｜船载CLF/CBF/ECBF-QP；仓储无全文。**

<a id="r15"></a>**[R15]** Yunfei Cai; Shutao Zheng; Weitian Liu; Zhiyong Qu; Junwei Han (2021). **Model Analysis and Modified Control Method of Ship-Mounted Stewart Platforms for Wave Compensation**. *IEEE Access*, 9: 4505-4517. [DOI: 10.1109/access.2020.3047063](https://doi.org/10.1109/access.2020.3047063)；[已读取材料/内容入口](https://ieeexplore.ieee.org/document/9306754/)。**A｜船载波浪补偿；已读公开摘要。**

<a id="r16"></a>**[R16]** Yunfei Cai; Shutao Zheng; Weitian Liu; Zhiyong Qu; Jiyue Zhu; Junwei Han (2021). **Adaptive robust dual-loop control scheme of ship-mounted Stewart platforms for wave compensation**. *Mechanism and Machine Theory*, 164: 104406. [DOI: 10.1016/j.mechmachtheory.2021.104406](https://doi.org/10.1016/j.mechmachtheory.2021.104406)；[已读取材料/内容入口](https://www.sciencedirect.com/science/article/abs/pii/S0094114X21001646)。**A｜船载速度前馈与自适应双环；公开为仿真。**

<a id="r17"></a>**[R17]** Yunfei Cai; Shutao Zheng; Weitian Liu; Zhiyong Qu; Jiyue Zhu; Junwei Han (2021). **Sliding-mode control of ship-mounted Stewart platforms for wave compensation using velocity feedforward**. *Ocean Engineering*, 236: 109477. [DOI: 10.1016/j.oceaneng.2021.109477](https://doi.org/10.1016/j.oceaneng.2021.109477)。**M｜船载速度前馈滑模研究；正文待取得。**

<a id="r18"></a>**[R18]** Rongji Yin; Wei Xie; Yu Wen; Chenming Zhang; Weixing Chen; Weidong Zhang (2023). **Robust wave compensation controller design for an active hexapod platform with time-varying input delays**. *Ocean Engineering*, 274: 114084. [DOI: 10.1016/j.oceaneng.2023.114084](https://doi.org/10.1016/j.oceaneng.2023.114084)；[已读取材料/内容入口](https://www.sciencedirect.com/science/article/pii/S0029801823004687)。**A｜时变输入延迟与UUB；证明条件待全文。**

<a id="r19"></a>**[R19]** Xiafei Chen; Yiyang Jiao; Xiaoming Yuan; Zhaoliang Meng; Lijie Zhang; Xiaoping Liu (2025). **An online dual-loop AMPC strategy for wave compensation of an electro-hydraulic servo Stewart platform**. *Control Engineering Practice*, 165: 106540. [DOI: 10.1016/j.conengprac.2025.106540](https://doi.org/10.1016/j.conengprac.2025.106540)；[已读取材料/内容入口](https://www.sciencedirect.com/science/article/pii/S0967066125003028)。**A｜液压双环AMPC、行程约束与实验声明。**

<a id="r20"></a>**[R20]** Xiafei Chen; Hai Zeng; Qing Gu; Zening Chen; Zhaoliang Meng; Lijie Zhang (2025). **Dual-layer adaptive predictive control for wave compensation of electro-hydraulic Stewart platforms**. *Ocean Engineering*, 340: 122293. [DOI: 10.1016/j.oceaneng.2025.122293](https://doi.org/10.1016/j.oceaneng.2025.122293)；[已读取材料/内容入口](https://www.sciencedirect.com/science/article/pii/S0029801825019778)。**A｜双层LPV-MPC与MRAC、实验声明。**

<a id="r21"></a>**[R21]** Dezhao Pang; Xuan Yun; Jingtao Li; Chen Zhang; Tieshi Zhao (2026). **Predictive wave compensation under saturation constraints via residual attention LSTM and quadratic programming**. *Ocean Engineering*, 358: 125678. [DOI: 10.1016/j.oceaneng.2026.125678](https://doi.org/10.1016/j.oceaneng.2026.125678)；[已读取材料/内容入口](https://www.sciencedirect.com/science/article/abs/pii/S002980182601512X)。**A｜饱和边界拟合与预测QP；联合仿真。**

<a id="r22"></a>**[R22]** Xiaokai Cui; Wenxuan Wang; Sheng Gao; Peng Xu; Bing Li (2026). **Disturbance rejection control Strategy of Stewart-gangway hybrid system for active wave compensation**. *Ocean Engineering*, 353: 124571. [DOI: 10.1016/j.oceaneng.2026.124571](https://doi.org/10.1016/j.oceaneng.2026.124571)；[已读取材料/内容入口](https://www.sciencedirect.com/science/article/pii/S0029801826004051)。**A｜九自由度Stewart—舷梯任务分配；不能视作普通6PUS。**

<a id="r23"></a>**[R23]** Xue Jiang; Chao Wang; Hai Zeng; Binghao Zhang; Lijie Zhang (2026). **Online Trajectory Planning for Stewart Parallel Mechanisms Based on Improved Trajectory Scaling and Condition-Number-Aware Velocity Bound Regulation**. *Machines*, 14(7): 794. [DOI: 10.3390/machines14070794](https://doi.org/10.3390/machines14070794)；[已读取材料/内容入口](https://mdpi-res.com/d_attachment/machines/machines-14-00794/article_deploy/machines-14-00794.pdf)。**F｜电缸Stewart在线轨迹缩放、速度/加速度与实验。**

<a id="r24"></a>**[R24]** Yash Raj Khusro; Yanggu Zheng; Marco Grottoli; Barys Shyrokau (2020). **MPC-Based Motion-Cueing Algorithm for a 6-DOF Driving Simulator with Actuator Constraints**. *Vehicles*, 2(4): 625-647. [DOI: 10.3390/vehicles2040036](https://doi.org/10.3390/vehicles2040036)；[已读取材料/内容入口](https://mdpi-res.com/d_attachment/vehicles/vehicles-02-00036/article_deploy/vehicles-02-00036-v2.pdf)。**F｜Stewart NMPC执行器行程限制；仿真。**

<a id="r25"></a>**[R25]** Emanuele Garone; Stefano Di Cairano; Ilya Kolmanovsky (2017). **Reference and command governors for systems with constraints: A survey on theory and applications**. *Automatica*, 75: 306-328. [DOI: 10.1016/j.automatica.2016.08.013](https://doi.org/10.1016/j.automatica.2016.08.013)；[已读取材料/内容入口](https://www.sciencedirect.com/science/article/abs/pii/S0005109816303715)。**A｜受约束闭环系统参考整形综述。**

<a id="r26"></a>**[R26]** Aaron D. Ames; Xiangru Xu; Jessy W. Grizzle; Paulo Tabuada (2017). **Control Barrier Function Based Quadratic Programs for Safety Critical Systems**. *IEEE Transactions on Automatic Control*, 62(8): 3861-3876. [DOI: 10.1109/tac.2016.2638961](https://doi.org/10.1109/tac.2016.2638961)。**M｜CBF-QP通用基础；本轮未逐式分析原文。**

<a id="r27"></a>**[R27]** Hung Pham; Quang-Cuong Pham (2018). **A New Approach to Time-Optimal Path Parameterization Based on Reachability Analysis**. *IEEE Transactions on Robotics*, 34(3): 645-659. [DOI: 10.1109/tro.2018.2819195](https://doi.org/10.1109/tro.2018.2819195)；[已读取材料/内容入口](https://arxiv.org/abs/1707.07239)。**A｜路径参数化与可达性基础；非移动基座专用。**

<a id="r28"></a>**[R28]** Lars Berscheid; Torsten Kroeger (2021). **Jerk-limited Real-time Trajectory Generation with Arbitrary Target States**. *Robotics: Science and Systems XVII*. [DOI: 10.15607/rss.2021.xvii.015](https://doi.org/10.15607/rss.2021.xvii.015)；[已读取材料/内容入口](https://arxiv.org/abs/2105.04830)。**A｜带速度/加速度/jerk界的在线轨迹生成；非本题完整解。**

<a id="r29"></a>**[R29]** Wenxuan Wang; Yinghao Ning; Yang Zhang; Peng Xu; Bing Li (2025). **Linear active disturbance rejection control with linear quadratic regulator for Stewart platform in active wave compensation system**. *Applied Ocean Research*, 156: 104469. [DOI: 10.1016/j.apor.2025.104469](https://doi.org/10.1016/j.apor.2025.104469)；[已读取材料/内容入口](https://www.sciencedirect.com/science/article/pii/S0141118725000574)。**A｜电缸位置环LADRC/LQR、偏心负载背景；公开为仿真。**

<a id="r30"></a>**[R30]** Zongyu Zhang; Jingwei Li; Jingjin Xie; Hui Zhang; Longfang Zhang; Jian Zhou (2026). **Research on Stewart Platform Control Method for Wave Compensation Based on BiLSTM Prediction and ADRC**. *Actuators*, 15(3): 140. [DOI: 10.3390/act15030140](https://doi.org/10.3390/act15030140)；[已读取材料/内容入口](https://www.mdpi.com/2076-0825/15/3/140)。**A｜预测加ADRC的船载姿态补偿；公开为仿真。**

<a id="r31"></a>**[R31]** Hongyan Mu; Ting Zhou; Binbin Li; Kun Liu (2026). **Simulation and Experimental Study of Vessel-Borne Active Motion Compensated Gangway for Offshore Wind Operation and Maintenance**. *Journal of Marine Science and Engineering*, 14(2): 187. [DOI: 10.3390/jmse14020187](https://doi.org/10.3390/jmse14020187)；[已读取材料/内容入口](https://www.mdpi.com/2077-1312/14/2/187)。**A｜船载运动补偿舷梯的仿真与实验声明。**

### C：状态估计与时间标定

<a id="r32"></a>**[R32]** I. Miletović; D.M. Pool; O. Stroosma; M.M. van Paassen; Q.P. Chu (2017). **Improved Stewart platform state estimation using inertial and actuator position measurements**. *Control Engineering Practice*, 62: 102-115. [DOI: 10.1016/j.conengprac.2017.03.006](https://doi.org/10.1016/j.conengprac.2017.03.006)；[已读取材料/内容入口](https://repository.tudelft.nl/file/File_95cfc3f0-4a5e-43c2-9f91-c5141a292f64)。**F｜取得TU Delft作者稿；IMU/腿位置UKF与实机数据。**

<a id="r33"></a>**[R33]** Changyu Yue; Jinzheng Liu; Liwen Guan (2025). **Accurate and robust state estimation for Stewart platform via error state Kalman filter with IMU and leg length fusion**. *Mechanism and Machine Theory*, 218: 106297. [DOI: 10.1016/j.mechmachtheory.2025.106297](https://doi.org/10.1016/j.mechmachtheory.2025.106297)；[已读取材料/内容入口](https://www.sciencedirect.com/science/article/abs/pii/S0094114X25003866)。**A｜ESKF、IMU/腿长直接闭链更新；数值仿真。**

<a id="r34"></a>**[R34]** Tong Qin; Shaojie Shen (2018). **Online Temporal Calibration for Monocular Visual-Inertial Systems**. *2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, : 3662-3669. [DOI: 10.1109/iros.2018.8593603](https://doi.org/10.1109/iros.2018.8593603)；[已读取材料/内容入口](https://arxiv.org/abs/1808.00692)。**A｜时间偏移联合估计的通用先例；视觉惯性机构不同。**

### D：手持工具与应用任务边界

<a id="r35"></a>**[R35]** Achim Wagner; Matthias Nübel; Essam Badreddin; Peter P. Pott; Markus L. Schwarz (2007). **DISTURBANCE FEED FORWARD CONTROL OF A HANDHELD PARALLEL ROBOT**. *Proceedings of the Fourth International Conference on Informatics in Control, Automation and Robotics*, : 44-51. [DOI: 10.5220/0001626500440051](https://doi.org/10.5220/0001626500440051)；[已读取材料/内容入口](https://www.scitepress.org/Papers/2007/16265/16265.pdf)。**F｜手持骨加工并联机器人、前馈及延迟；仿真。**

<a id="r36"></a>**[R36]** Sungwook Yang; Robert A. MacLachlan; Cameron N. Riviere (2015). **Manipulator Design and Operation of a Six-Degree-of-Freedom Handheld Tremor-Canceling Microsurgical Instrument**. *IEEE/ASME Transactions on Mechatronics*, 20(2): 761-772. [DOI: 10.1109/tmech.2014.2320858](https://doi.org/10.1109/tmech.2014.2320858)；[已读取材料/内容入口](https://publications.ri.cmu.edu/storage/publications/pub_files/2015/4/TMECH_Yang_accepted.pdf)。**F｜Micron手持尖端/RCM/载荷与硬件；2014在线、2015卷期。**

<a id="r37"></a>**[R37]** Sungwook Yang; Trent S. Wells; Robert A. MacLachlan; Cameron N. Riviere (2013). **Performance of a 6-degree-of-freedom active microsurgical manipulator in handheld tasks**. *2013 35th Annual International Conference of the IEEE Engineering in Medicine and Biology Society (EMBC)*, : 5670-5673. [DOI: 10.1109/embc.2013.6610837](https://doi.org/10.1109/embc.2013.6610837)；[已读取材料/内容入口](https://pmc.ncbi.nlm.nih.gov/articles/PMC3801264/)。**A｜Micron早期手持硬件研究；不同于R36完整期刊论文。**

<a id="r38"></a>**[R38]** Shuo‐Suei Hung; Alex Shao‐Feng Hsu; Tsung‐Han Ho; Chau‐Hwa Chi; Ping‐Lang Yen (2021). **A robotized handheld smart tool for orthopedic surgery**. *The International Journal of Medical Robotics and Computer Assisted Surgery*, 17(5): e2289. [DOI: 10.1002/rcs.2289](https://doi.org/10.1002/rcs.2289)；[已读取材料/内容入口](https://tcu.elsevierpure.com/en/publications/a-robotized-handheld-smart-tool-for-orthopedic-surgery-2/)。**A｜手持骨科工具、共享控制和猪脊柱实验声明。**

<a id="r39"></a>**[R39]** David E. Usevitch; Rachel S. Bronheim; Miguel A. Cartagena-Reyes; Carlos Ortiz-Babilonia; Adam Margalit; Amit Jain; Mehran Armand (2023). **Review of Enhanced Handheld Surgical Drills**. *Critical Reviews in Biomedical Engineering*, 51(6): 29-50. [DOI: 10.1615/critrevbiomedeng.2023049106](https://doi.org/10.1615/critrevbiomedeng.2023049106)；[已读取材料/内容入口](https://pmc.ncbi.nlm.nih.gov/articles/PMC10874117/)。**A｜骨钻领域综述，作任务背景；非方法性能证据。**

<a id="r40"></a>**[R40]** Hermes Giberti; Francesco La Mura; Gabriele Resmini; Marco Parmeggiani (2018). **Fully Mechatronical Design of an HIL System for Floating Devices**. *Robotics*, 7(3): 39. [DOI: 10.3390/robotics7030039](https://doi.org/10.3390/robotics7030039)；[已读取材料/内容入口](https://www.mdpi.com/2218-6581/7/3/39)。**A｜Hexafloat/6PUS相关HIL与机电设计；非本题稳定控制解。**


### 3.1 初始线索核验中的更正与保留

- **Ono I**：在线日期2021-11-19，正式卷期为2022年107卷51—76页；**Ono II**正式为2021年106卷3161—3182页。不能都笼统写成2022。
- **Khan**：作者为Ameer Hamza Khan、Xinwei Cao、Shuai Li；DOI为10.1002/rob.22605。在线2025-06-16，卷期为2025年42(7)。其公开摘要使用“experiments”并描述Simscape与椅载场景，**本轮没有取得足够材料将其认定为真实硬件闭环实验**。
- **Li 2024**：第一作者是Weibing Li，并非凭姓氏认定为Shuai Li；正式2024年20(3)，DOI含2023。本轮未取得正文。
- **Mohammed与Li 2016**：期刊作者记录是Aquil Mirza Mohammed、Shuai Li；其学位论文及2017年论文采用Mohammed Aquil Mirza的姓名排列。按各来源原始署名保留，避免生成不同作者身份的推断。
- **Vu 2025**：题目、四名作者和DOI已确认；TU Wien书目记录在线日期为2025-07-22，卷期为2025年340卷。其仓储明确标记没有全文。
- **Micron R36**：正式题目是“Manipulator Design and Operation **of** …”，正式卷期年为2015，在线发表于2014；不能因PMC收录日期或作者稿文件名而错记卷期年。

### 3.2 补充来源与勘误

- **C01**：Ono等，2022，*Correction to: Analysis and control … Part I*，Nonlinear Dynamics 108:2853—2855，[DOI 10.1007/s11071-022-07320-x](https://doi.org/10.1007/s11071-022-07320-x)。已读公开内容；包含公式上标、式号及排版等修正，后续若重用公式应对照勘误。
- **C02**：Ono等，2022，*Correction to: Analysis and control … part II*，Nonlinear Dynamics 108:2857—2859，[DOI 10.1007/s11071-022-07319-4](https://doi.org/10.1007/s11071-022-07319-4)。已核实勘误存在；未对受限期刊正文逐式复算。
- **S01**：Takeyuki Ono，2021，*Modeling and Control of a Stewart Platform Mounted on a Moving Body*，作者博士学位论文。[机构全文](https://niad.repo.nii.ac.jp/record/786/files/501_%E8%AB%96%E6%96%87.pdf)。取得142页PDF；用于补充R12的动力学、执行器接口和实验细节，论文印刷页码与PDF页码相差1页。它不是R12最终期刊版。
- **S02**：Mohammed Aquil Mirza，2019，*Dynamic Neural Networks for Parallel Stewart Platforms*，Hong Kong Polytechnic University博士学位论文。[机构全文](https://theses.lib.polyu.edu.hk/bitstream/200/10133/1/991022270855503411.pdf)。取得116页PDF；第二章用于核查R03相关QP框架，第三章对应2017年研究脉络。印刷页27—30对应PDF第41—44页。

### 3.3 截止日前出现的新近书目线索：不得冒充已审读全文

<a id="p01"></a>**[P01]** Guangrui Luan; Hongshuo Wang; Yong Liu; Donghong Ning; Xingjian Jing; Haiping Du (2026). **Prediction-enhanced kinematics-informed model-free finite-time saturated control for wave compensation under input saturation and uncertainty constraints**. *Ocean Engineering*, 367: 127986. [DOI: 10.1016/j.oceaneng.2026.127986](https://doi.org/10.1016/j.oceaneng.2026.127986)；[已读取材料/内容入口](https://api.crossref.org/works/10.1016/j.oceaneng.2026.127986)。**M｜2026-09-07已有DOI注册/OpenAlex记录；出版社API卷期日期2026-10-30；仅核实书目。**

<a id="p02"></a>**[P02]** Haifeng Zhang; Tian Zeng; Shousen Li; Zhongjie Zhang (2026). **Active wave compensation for dual-stage Stewart platforms using VAR-MPC**. *Ocean Engineering*, 368: 128336. [DOI: 10.1016/j.oceaneng.2026.128336](https://doi.org/10.1016/j.oceaneng.2026.128336)；[已读取材料/内容入口](https://api.crossref.org/works/10.1016/j.oceaneng.2026.128336)。**M｜2026-09-23已有DOI注册/OpenAlex记录；出版社API卷期日期2026-11-15；仅核实书目。**


P01/P02的DOI注册及OpenAlex记录早于检索日；出版社API能返回正式题目与期刊信息，但本轮没有取得出版社摘要或正文。P01卷期日期为2026-10-30，P02为2026-11-15，均晚于本轮检索日。**它们列为提前公开记录线索，不计入40篇的主要技术证据，不采用聚合站对其算法、证明和实验的扩写。** 下一轮必须核实实际online-first记录、版本与全文。

## 第四部分：重点文献技术分析

以下不是每篇都已全文分析的伪清单。F/T/A等级、支持位置和未知项均单列；缺少全文的论文不补造公式编号、硬件指标或作者局限。

### 4.1 Khan等2025：姿态任务冗余的最直接冲突［R01，A］

**已核实内容**：六个移动副驱动的六自由度Stewart平台；仅指定三维姿态时保留三个任务冗余自由度；以受机械不等式约束的优化处理姿态与执行器使用。作者公开摘要将BAORNN描述为免Jacobian、免梯度的求解机制，并声明有稳定性/收敛性分析和Simscape Multibody验证。

**证据位置**：Wiley与PolyU机构页摘要。正文未取得，不能核查优化变量究竟是关节位置、角度参数还是其他变量，也不能重构约束或定理。基座是否施加外部运动、是否直接优化辅助平移、行程/速度/被动角各自如何限制、任务冲突怎么松弛，均未确认。椅载场景不能单独证明硬件实验。

**对本课题的判断**：它已经覆盖“六驱动机构因姿态任务而具有冗余，并在约束下利用剩余运动”的概念。即使它没有移动基座，把已知坐标变换加到同一优化问题中也未必有实质新颖性。另一方面，不能据摘要断言它已解决钻尖容差、外部运动持续推动的可行性或50 Hz位置伺服。**作者明确不足：本轮未从正文取得。**

### 4.2 Li等2024：关节受限冗余控制已是明确主题［R02，A］

**已核实内容**：在运动学冗余Gough–Stewart机器人上比较三种ZNN控制器；用非线性互补问题函数处理关节约束，作者声明给出收敛分析、离散化控制器与两种路径跟踪任务。

**证据位置**：CUHK作者机构页摘要及正式书目，TII 20(3):4559—4570。未取得正文，不能认定三维姿态就是其主任务，不能把“joint constraints”扩成行程、速度、被动角、碰撞与力限制的全集；实际输入层、固定/移动基座和真实机器人实验均待确认。

**对本课题的判断**：该文足以否定“Stewart冗余控制首次考虑关节限制”类表述；目前尚不足以判定其是否完整覆盖姿态优先辅助平移。若下一轮发现其问题只是某个已知QP的求解器改进，我们的差异必须来自任务定义或可行性机制，而不能只是另一种求解器。**不能把“未看到硬件证据”写成作者没有硬件。** 作者明确局限和定理适用范围待全文。

### 4.3 Mohammed与Li 2016：候选问题的QP骨架已经存在［R03，T］

**证据位置**：期刊摘要确认约束QP与全局收敛声明；S02第二章§2.4—2.5，印刷页27—30、式(2.20)—(2.33)提供具体框架。以下符号属于S02，不冒充期刊式号。

S02用六维平台运动变量与腿速度的映射，施加低维任务等式和执行器线性不等式；式(2.23)的结构可概括为：

\[
\min_{\dot\pi,\tau}\tfrac12\dot\pi^T\Lambda_1\dot\pi+
\tfrac12\tau^T\Lambda_2\tau,
\quad \tau=A_1\dot\pi,\quad A_2\dot\pi=\alpha,\quad B\tau\le b.
\]

**这里的\(\tau\)定义为腿速度，不是力矩。** 用KKT条件构建动态神经网络，论文摘要报告仿真。S02明确说明未直接给平台运动变量另加约束，因为规划阶段通常已选择可行参考。这是该对应章节中可追溯的适用前提，不是从标题猜测的缺陷。

**本报告推论**：将低维任务选为姿态、剩余变量用于平移，已落在这一类优化框架中。6PUS需要替换实际闭链映射；不能照搬伸缩杆Stewart的腿长公式。此处优化解收敛也不等于未建模位置伺服闭环收敛，更不等于对不可行参考仍能完成原任务。期刊正文、主动约束的具体选取和硬件验证仍待取得；S02后续的建机章节不能倒推为2016年期刊实机实验。

### 4.4 Ono Part I/II：移动基座惯性稳定与真实输入接口［R11 F；R12 T］

**R11全文证据**：§2与Fig.1明确是基座U—P—S伸缩腿结构；§4推导速度关系；§5、Fig.8—9、Table 1为世界系静止目标、IK前馈和腿长PID实验。Appendix A式(A11)给出设计工作空间约束。实验步长0.01 s，低频运动由下方运动发生器产生。它不是6PUS导轨滑块结构。

**R12与补充证据**：出版社摘要报告逆动力学和实机比较；S01§3.4—3.5、印刷页72—80，式(3.61)—(3.64)说明先求执行器轴向力，再通过电机/丝杠模型形成电压/PWM信号；§3.5.1说明双惯性传感器与执行器位移测量。该接口并非只向一个封装位置伺服发送位置目标。

**约束与证明边界**：R11的工作空间分析不是在线约束优化，也没有在§5中把辅助平移作为可调次任务。S01/R12相关动力学推导不自动给出饱和冲突时的恢复保证。R12最终正文未取得，不能对其完整定理和式号做期刊级核验；两篇还应对照C01/C02勘误。

**迁移判断**：世界/基座/平台坐标分离、IK＋伺服和动力学前馈是明确先例。R11是合适工程基线；R12可作动态影响参考，但必须保留“轴向力→执行器模型”与本平台“位置命令→现有伺服”的差别。作者学位论文还讨论了位移积分漂移和模型不确定性，不能把双IMU当成世界系位置的无漂移测量。

### 4.5 Lin等2025：最接近6PUS移动基座工程对象的先例［R13，A］

**已核实内容与位置**：出版社摘要、引言、§2—§5预览。研究浮动基座6-PUS稳定机构，包含肢链惯性和基座运动；预览说明P—U—S次序及导轨/滑块实现。§4预览明确在平台和船甲板分别设置IMU；§5描述MATLAB/Simulink R2023b多体模型。作者提出逆动力学背景下的NTSMC与等效输入扰动补偿，并声明稳定性分析和仿真。

**重要区分**：文中的“12-DOF”模型把基座六维运动与平台六维运动合并描述，不能据此称平台有12个可用于姿态分配的自由度；双丝杠驱动共同滑块也不能直接推导为本课题所需的任务冗余。

**尚未确认**：控制最终注入电机哪一层、真实位置伺服限制、是否同时考虑行程/速度/被动角、是否有姿态主任务与辅助平移分配、是否有实机实验，正文未取得，不能填“无”。公开可读验证部分是仿真，不能称其为硬件结果。作者完整局限待全文。

**迁移与冲突**：该文是机构/移动基座/多源扰动/双IMU层面最需面对的论文；换成骨钻或将NTSMC换成TDE/LADRC都不足以绕开它。具体导轨方向、固定杆长、参数和伺服接口仍不能假定与现有6PUS完全相同。

### 4.6 Vu等2025：船载Stewart已有CLF/CBF/ECBF-QP［R14，A］

**已核实内容**：作者机构摘要明确使用Lagrangian建模、简化控制模型、CLF以及含CBF/ECBF的QP，并配合高增益扰动观测器；通过Simscape Multibody对照验证模型。

**证据位置**：TU Wien作者记录的摘要；完整记录标记`no Fulltext`。因此，本轮不能提供障碍函数、相对阶、约束种类、松弛变量、输入边界、可行初始集或递归可行性证明的公式。也不能将Simscape模型对照称为真实硬件验证。

**本报告推论**：若拟议贡献只是“船载Stewart稳定中加入QP安全约束”，冲突已明确。是否已覆盖姿态优先、钻尖容差和辅助平移，尚未确认。即便CLF可以放松，硬CBF与输入界冲突仍须有可行性条件；QP名称本身不提供这种条件。其动力学模型输入能否通过现有位置伺服实现，必须待全文判断。作者不足与证明假设同样待全文。

### 4.7 Chen等2025双环AMPC：不能将受限补偿简化成理想外环［R19，A］

**已核实内容与位置**：出版社摘要、Highlights与章节预览。内环针对阀控非对称液压缸建模并在线辨识；外环使用LPV模型进行运动规划/姿态调节，明确纳入执行器行程；还声明UKF–Newton融合估计和海况4条件下实验。

**未取得正文的边界**：实验是怎样的海况模拟台、外环/内环采样周期、实时求解时间含义、速度/加速度与被动角是否约束、是否有姿态优先、任务冲突松弛和递归可行性，均未确认。不能由“有海况4实验”推断真实海试，更不能把模型计算量数字换算成本课题实时性。

**迁移判断**：外环生成可执行参考、内环负责真实执行器动态，已经是直接先例。其液压模型不可照搬到6PUS电机位置伺服，但“换成电缸/位置伺服”本身通常是常规建模适配。R20是另一篇不同作者组合的双层LPV-MPC＋MRAC研究，不能与R19混为同一论文或重复计实验。

### 4.8 Pang等2026：参考不可行与饱和QP不是空白［R21，A］

**已核实内容与位置**：出版社摘要、Highlights。研究浮动基座重载补偿系统的执行器/工作空间饱和；用PCA拟合饱和边界，再进行预测QP运动规划。作者报告联合仿真，强调越界需求下参考平滑与驱动载荷减小。

**尚未确认**：具体机构是否Stewart、控制输出层、饱和边界是哪类变量的精确约束、是否包含姿态次序和辅助平移、拟合边界是否保守、是否有正式递归可行证明。摘要中的luffing/telescopic驱动描述也提醒我们不能把它直接标成六腿6PUS。不能把联合仿真改写为实机实验。

**对候选问题的影响**：它已经覆盖“外部运动导致补偿需求超过能力→先规划受限参考→再执行”的基本思路。我们的潜在差异只能是尚待证明的具体任务约束、可行域或误差保证，不能是“首次使用预测QP处理饱和”。作者明确不足和全部数学约束仍待全文。

### 4.9 Jiang等2026：电缸Stewart在线速度/加速度可行性［R23，F］

**全文位置**：§2.2式(5)—(14)将轨迹映射到执行器位移、速度和加速度；§2.3处理特殊加速度区间；§2.4式(55)—(58)使用归一化Jacobian条件数；§3.2为电缸平台实验，规划周期2 ms；§4—5讨论局限。

**论文事实**：沿既定几何路径调整时间律，结合工作空间分区和条件数调节速度界；包括单轴测试及真实平台验证。作者明确把动力学约束、载荷变化和预测优化留给后续工作。这里的加速度限制属于运动学能力约束，不能因此声称完整驱动力可行。

**本报告推论**：该文使“考虑工作空间和速度/加速度后生成平滑可行参考”不能作为宽泛创新。它保留几何路径、改变时间律，并未把辅助平移作为姿态任务零空间搜索。对于不可控制的基座扰动，减慢自身参考不等于让外部扰动减慢，因此还需比较同步补偿误差。原文有方法推导，但本轮没有发现可据以声称“任意外扰下持续稳定”的完整保证；不能把未见保证当成作者错误。

### 4.10 Miletović等2017：已有实机闭链融合，也有明确时间问题［R32，F］

**全文位置**：作者稿§2—3为运动学/UKF；§5为SIMONA实时数据验证；§5.2、PDF第19页说明安装位置与未建模时间误差；§5.3、PDF第20页报告IMU相对执行器位置测量存在10—50 ms、随轴和时间变化的延迟。该数值只属于其系统。

**论文事实**：平台惯性传感器与六路执行器位置紧耦合，用真实运动数据检验估计；不是移动基座双IMU补偿控制实验。作者明确说时间延迟、安装误差等处理超出该文范围。

**迁移与研究风险**：R33已进一步研究ESKF与直接闭链几何更新，故“去掉迭代FK”不能据R32局限重新宣称新颖。可继续调查的是移动基座多流测量的时刻一致性、可辨识条件与稳定控制影响。估计更平滑不是更准确的充分证据；应需要独立姿态/位移参考。仅有IMU与内部闭链约束，不能自动观测整机共同的世界平移；绝对航向也取决于是否有外部航向观测。

### 4.11 Yang等Micron 2015：工具尖端、RCM和有限驱动力必须同时考虑［R36，F］

**全文位置**：作者稿§II.A讨论尖端及RCM运动学；§II.B—C、式(15)—(18)分析速度/载荷传递与几何优化；§IV与Fig.7是IK转换腿长后PID控制；§V为侧向载荷、指向和轨迹实验；§VI列出执行非线性与实验边界。

**论文事实**：手柄可动、微型Gough–Stewart执行器移动工具，采用光学跟踪；工具尖端工作空间、RCM侧载和有限推力影响补偿能力；有真实手持实验。其机械几何优化主要属于设计阶段，不能冒充在线姿态冗余分配。作者同时说明实验条件与实际手术验证仍有距离。

**本报告推论**：它直接否定“只看平台姿态误差即可代表手术稳定效果”。6PUS与其微型结构、驱动和测量均不同；可迁移的是任务端指标与载荷约束的建模思路，不能迁移微米级误差、频率、负载或临床结论。RCM下某些平移由姿态变化唯一决定，不是任意可用的冗余。

### 4.12 Wagner等2007：手持骨加工和前馈延迟问题已有经典先例［R35，F］

**全文位置**：§2、Fig.1—2描述六个基座固定直线驱动与轻杆；连接使用球关节，不能凭外观改称标准6PUS。§4、Fig.3给出局部轴伺服＋全局逆动力学扰动前馈；§5、Fig.8研究传感延迟；§6讨论理想模型、摩擦忽略、传感误差及基座动力学假设。

**论文事实**：目标是手持钻削/铣削工具相对骨组织稳定；MATLAB/Simulink与SimMechanics验证属于仿真，不是该文硬件性能证明。文中明确外部基座运动不由机器人控制动作直接改变的近似。

**对本课题的判断**：应用、扰动前馈和延迟敏感性都有早期先例。现有50 Hz位置命令链能否实现有效补偿，必须以本系统闭环动态证明；不能把该文1 ms仿真步长或控制参数当作本系统参数。其理想模型条件是作者自述边界，而不是我们人为制造的缺陷。

### 4.13 已获全文的通用方法核查：避免把常规组合写成新机制

| 文献 | 核对位置与实际能力 | 对本课题的直接约束 |
|---|---|---|
| R08 HQP | HAL作者稿是较早两部分版本，51页；Part 1 §2、§4含严格层级、等式/不等式及slack，Part 2为实现与实验。式号只按该版本理解 | 严格任务优先与冲突松弛已有算法基础；没有理由把“姿态第一层”本身写成新方法。最终期刊版23页，不能混用页码 |
| R09 SNS | 作者稿§III—VI、式(12)、(21)—(24)，§IX为KUKA LWR实机；考虑硬关节界、任务缩放及最优性条件 | 主任务不可行时缩放并非新思路；但原任务缩放不自动抵消持续基座扰动，需处理外生运动项 |
| R24 Stewart NMPC | §3.2、式(25)—(29)为执行器映射；§4.2式(40)—(42)明确施加倾斜角速度和执行器行程限制；§6仍将物理六足实时测试列为未来工作 | 建模了腿速度不等于对其施加了全部硬限制；该文的运动感觉目标也不是惯性稳定。可借用约束规划基础，不能导入其任务代价 |

这些方法中，优化器收敛、瞬时运动学约束满足、路径保持、物理伺服稳定和持续外扰下可恢复性是不同命题，后续论文必须分别说明。

## 第五部分：高相关方法对比矩阵

“U”表示本轮材料未确认；“概念可迁移”不等于现有控制代码可直接使用。A/T级的理论/实验只按公开记录或对应学位章节表述。

| 研究 | 机构/基座 | 任务与输入层 | 约束、辅助运动及冲突 | 理论与验证证据 | 对6PUS迁移的主要障碍 |
|---|---|---|---|---|---|
| R01 Khan | 六移动副Stewart；外部移动基座U | 三维姿态；优化变量与驱动接口U | 明确任务冗余、机械不等式；具体辅助平移/松弛U | 作者声明收敛/稳定分析；Simscape；硬件U | 正文、具体约束和伺服层均待核 |
| R02 Li | 冗余Gough–Stewart；基座U | 两类路径；输入层U | 关节约束；主任务类型及失配处理U | 作者声明理论与离散实现；硬件U | 不能先假定与姿态/移动基座完全等价 |
| R03＋S02 | 常规Stewart固定基座模型 | 低维运动任务；腿速度 | 线性执行器约束；额外运动变量；规划参考假设可行 | 期刊摘要理论＋仿真；公式由学位论文补充 | 6PUS运动映射、位置伺服和外生基座项 |
| R11 Ono I | 6-UPS、移动基座 | 世界系位姿静止；IK腿长参考＋PID | 工作空间设计约束；未在控制实验中优化辅助平移 | 运动学推导＋缩比硬件 | 可作基线；几何与测量不能照搬 |
| R12＋S01 | 同类移动基座平台 | 位姿稳定；轴向力经执行器模型到电压/PWM | 在线辅助平移/饱和恢复U | 期刊称实机；学位论文给出实现细节 | 不能直接放到封装位置伺服外层 |
| R13 Lin | **6-PUS、浮动基座** | 位姿/多源扰动；逆动力学背景，最终接口U | 任务冗余与显式限幅约束U | 作者声明稳定性；公开验证为仿真 | 最接近机构先例；完整控制输入与约束待核 |
| R14 Vu | 船载Stewart | 稳定；简化动力学输入，具体物理接口U | CBF/ECBF-QP；具体h、松弛及辅助平移U | 作者声明CLF/安全约束；Simscape对照；硬件U | 位置伺服实现与可行初始域U |
| R19 Chen | 船载液压Stewart | 外环LPV-AMPC运动规划、内环液压控制 | 明确行程；速度/加速度/优先级U | 作者声明海况4实验；证明细节U | 液压内环不能替代电机位置伺服 |
| R21 Pang | 浮动基座重载补偿机构，非确认6PUS | 预测QP生成参考；驱动接口U | 拟合饱和边界；姿态优先/平移自由度U | 公开材料为联合仿真；理论细节U | 约束变量、边界保守性和机构须核实 |
| R22 Cui | 九自由度Stewart—舷梯混合系统 | 多任务轨迹分配＋关节动态控制 | 公开预览明确零空间有界QP；具体任务层级U | 预览称稳定性分析，正文结构为仿真 | 系统九自由度冗余不同于6PUS姿态任务冗余 |
| R23 Jiang | 固定基座电缸Stewart | 沿几何路径调整时间律；运动参考 | 速度/加速度、区域与条件数；不搜索姿态零空间平移 | 方法推导＋电缸平台实机 | 时间缩放不等于同步抵消外部基座运动 |
| R08/R09 | 通用冗余机器人 | 微分运动学；关节速度 | 严格优先、硬界、slack或缩放 | 数学性质分析，含机器人实机 | 替换闭链Jacobian并纳入外生漂移和伺服动态 |
| R32/R33 | Stewart状态估计；移动基座扩展U | IMU＋腿位置/长度，输出状态而非控制 | 闭链测量；双IMU异步处理U | R32真机数据实时；R33公开摘要为数值仿真 | 传感时刻、可观测性、真实PUS约束 |
| R35/R36/R38 | 手持并联工具 | 工具位姿/尖端，含局部伺服或共享控制 | RCM、工作空间、有限载荷或人工导航 | R35仿真；R36硬件；R38公开称猪脊柱实验 | 各手术任务、执行器、尺度、测量不同 |

### 5.1 传统工具能直接解决什么

| 子问题 | 成熟方法的适用性 | 不能自动得到的结论 |
|---|---|---|
| 给定基座姿态生成相对姿态参考 | 坐标变换＋IK已有直接先例R11 | 参考始终在工作空间内 |
| 姿态任务下分配未指定运动 | 运动学QP/HQP/零空间框架R03、R08、R09 | 工具实际允许这些运动 |
| 行程与瞬时速度限制 | IK约束或局部线性不等式、SNS、QP | 非线性步进后仍安全、未来仍可行 |
| 约束冲突时牺牲较低任务 | HQP、slack、任务缩放已有方法 | 物理硬约束互相矛盾时仍存在解 |
| 带内环动态的可执行参考 | MPC与reference governor已有基础R19、R25 | 未辨识伺服也有正确预测 |
| 位移/姿态估计 | IMU＋执行器测量融合已有R32、R33 | 绝对世界位置、全部延迟均可观测 |

## 第六部分：当前候选方向的创新性冲突分析

### 6.1 先用最小数学定义检验问题是否真实

以下是本报告为澄清研究问题采用的刚体关系与可行性定义，**不是提出新算法，也不是声称已经验证的6PUS结果**。

设世界、基座、平台坐标系分别为W、B、P，工具尖端在平台系的固定偏置为\(r_{tip}^P\)。

\[
R_{WP}=R_{WB}R_{BP},\qquad
p_{tip}^W=p_P^W+R_{WP}r_{tip}^P.
\]

若完整姿态\(R_{WP}\)和钻尖位置\(p_{tip}^W\)都严格指定，则

\[
p_P^W=p_{tip,d}^W-R_{WP,d}r_{tip}^P.
\]

因此在一般满秩构型下，平台三维平移已被确定；**没有三个自由的辅助平移维度**。只控制姿态时通常有三维任务冗余；加入钻尖容差、直线/平面约束、进给或接触后，实际余量要重新计算，不能继续固定称为“三自由度冗余”。

小扰动下，以世界系小转角\(\delta\theta\)表示：

\[
\delta p_{tip}^W\approx\delta p_P^W-
[R_{WP}r_{tip}^P]_\times\delta\theta.
\]

这说明姿态与尖端误差耦合，但不产生可任意使用的新自由度。若只要求钻轴方向而不要求绕轴滚转，方向任务通常是\(S^2\)上的两维任务；这与完整\(SO(3)\)三维姿态任务不同。能否释放这一维要由具体钻具、夹持和工艺需求确认。

对既定基座状态与期望姿态，可定义满足机构约束和工具任务容差的平移集合\(\mathcal P(R_d,b,\mathcal T)\)。固定平移与允许辅助平移相比的潜在收益，首先是比较这一集合是否非空及约束余量，而不是立即比较复杂控制器。若它为空，再好的求解器也不能同时完成全部硬任务。

**6PUS几何必须独立建模。** 普通伸缩腿Stewart的执行器变量常为连接点距离；导轨式PUS通常满足固定杆长闭链约束，滑块坐标才是驱动变量。例如在一种常见记号下：

\[
\|p_{BP}+R_{BP}b_i-a_i-q_i u_i\|=L_i.
\]

这只是示意约束：\(u_i\)为导轨方向，\(L_i\)为固定杆长；具体本机结构须以现有IK为准。把R11/R23的伸缩腿长度直接替换成滑块行程会改变可行域和Jacobian。

### 6.2 几何、运动学、执行动态和持续可行性是四层问题

| 层次 | 真正需要成立的命题 | 不能混淆的情况 |
|---|---|---|
| 几何可行 | 存在某一平台构型满足IK、行程、被动角、碰撞及任务约束 | 当前点能达到，不代表能及时到达 |
| 运动学可行 | 连续/离散轨迹满足关节速度、加速度等限制 | 速度、加速度满足，不代表所需驱动力足够 |
| 执行动态可行 | 实际位置伺服、负载和延迟条件下能在容差内实现 | 理想力仿真或运动学积分不能替代此证明 |
| 持续/可恢复 | 在规定基座扰动集合内，后续仍有满足要求的动作；越界后有可解释退让 | 本步QP有解，不代表下一步有解；有限扰动也不代表无限时域可补偿 |

在基座不可控制的假设下，世界角速度含外生项。若以基座系姿态Jacobian\(J_\omega^B(q)\)表示，则

\[
\omega_{WP}^W=\omega_{WB}^W+R_{WB}J_\omega^B(q)\dot q.
\]

理想姿态静止要求右侧为零。这通常仍可进入常规带仿射项QP；**出现外生项本身不是新方法**。但把补偿关节运动整体缩放会留下基座角速度残差，所以不能直接把“参考放慢后路径不变”解释成“世界姿态继续完全稳定”。这是需要检验的适用差异，尚不是已成立的文献空白。

约50 Hz只表示约20 ms指令更新间隔。它既不是已测得的闭环带宽，也不证明25 Hz或任何指定频段可稳定；还取决于内部插补、通信与测量时延、负载、执行器动态及扰动谱。本轮没有测这些量。

### 6.3 对A—J问题的直接回答

| 问题 | 本轮回答 | 分类与证据 |
|---|---|---|
| A. 是否已有相同问题？ | 各主要组成部分已被明确研究；完全相同的组合与保证尚未确认 | **已有文献明确解决/部分解决**：R01、R03、R11、R13、R19、R21、R23；**文献中尚未确认**完整交集 |
| B. 哪篇最近？ | 不能只给一个“最近”：机构与移动基座最接近R13，姿态任务冗余最接近R01，受限执行参考最接近R19/R21/R23，手持骨钻最接近R38及R35 | 多维最近邻；不能用某篇不含骨钻就排除其方法先例 |
| C. 哪些可直接解决？ | 常规QP/HQP可处理局部姿态任务、辅助变量、线性化界和层级；坐标变换＋IK能生成补偿参考 | 直接可用的是数学工具；仍须本机映射和约束数据 |
| D. 哪些属于常规组合？ | 世界系误差＋IK＋关节限制QP；姿态第一层、回中第二层；MPC预测伺服；延迟补偿叠加稳定控制 | 没有额外定理、性能边界或可复现反例时，高概率属于常规适配 |
| E. 是否有未充分研究问题？ | 有值得核查的具体问题候选，但目前没有确认可宣称的空白 | **文献中尚未确认**：工具容差下余量是否有效、持续外扰下可恢复边界、多流异步误差与可行性耦合 |
| F. 来自何种限制？ | 可用任务自由度、外部扰动不可减速、伺服并非理想输入、状态与时刻不完全已知 | **我们提出的待验证假设**；其中可行参考前提见S02、动态/载荷未来工作见R23、时间问题见R32 |
| G. 是否可能只需传统方法？ | **很可能。应以正确建模的传统QP/HQP/MPC/reference governor优先判断。** | 不是创新失败的坏结果；若已足够，就应避免另造控制器 |
| H. 值得研究新机制吗？ | 目前尚不值得直接设计新机制；先找出传统方法在公平约束和可测条件下确有不足的实例 | 不把降低基线质量作为创新依据 |
| I. 哪些需本机验证？ | 几何容差可行集合、实际任务Jacobian秩、约束激活顺序、伺服/测量动态、可实现误差和负载方向影响 | 本轮未运行模型，所有本机性能收益仍未知 |
| J. 是否放弃？ | **放弃宽泛组合式创新表述；暂不放弃平台和问题调查。**若真实任务无有用辅助运动，或常规方法已满足需求，应退出当前辅助平移方向 | 条件式继续，不预设一定产出新算法或SCI论文 |

### 6.4 新颖性风险排序

1. **最高：把六自由度平台的三维姿态任务冗余当新发现。** R01/R03直接冲突。
2. **最高：把移动基座6PUS、双IMU及抗扰控制当空白。** R13直接冲突；R11/R12和R35更早提供相关基础。
3. **高：以QP/MPC或执行器限制作为主要创新。** R08/R09/R19/R21/R23已覆盖相当部分，R22还覆盖混合系统多任务规划。
4. **高：把低频船载成果换成手持场景即认定创新。** 时域、负载和感知不同可能重要，但需要独立定义和证据。
5. **高：在理想力模型上证明，再用位置伺服实验“展示”而不说明输入间关系。** 这是证据链缺口，不是新颖性来源。
6. **中高：把双IMU＋编码器异步融合视为自然空白。** R32/R33/R34及一般延迟融合先例必须进一步比较。
7. **尚待判断：真实工具容差与外生扰动下的可恢复域/误差下界。** 有研究意义不等于已有创新，需要全文查重和本机证据两道检验。

## 第七部分：其他值得考虑的研究问题

以下仅给问题与证据要求，不设计控制器。难度为基于用户现有工程背景的估计，不是已经验证的工时或性能承诺。

### 候选P1：真实工具约束下，辅助运动究竟能增加多少稳定能力？

- **具体问题**：在给定钻尖横向容差、允许进给和钻轴方向要求下，6PUS能否通过任务允许的运动扩大姿态补偿可行集合？增益在哪些构型存在、在哪些约束下消失？
- **最近文献**：R01/R03的低维任务优化，R36的尖端/RCM/负载考虑，R38的手持骨科任务，R10的6PUS性能优化。
- **实质差异尚待证明**：从“自由平移”变为实际工具容差定义，并给出相应可行域和不能改善的条件。该问题不同于换求解器，但集合计算本身仍可能是常规运动学优化。
- **可能突破口**：可解释的任务自由度分析、任务容差与稳定余量的关系、适用/不可改善条件；不是先叠加新控制器。
- **复用与难度**：可复用IK/Jacobian和多体模型；MATLAB难度低—中；硬件难度中—高，取决于能否独立测量工具相对目标的位置。
- **最大风险**：完整姿态与尖端位置均硬约束时没有辅助平移；允许的容差太小、收益不足；或传统约束优化已足够。
- **建议**：最先讨论，用来决定当前候选方向是否值得保留。

### 候选P2：持续基座扰动和有限位置伺服下，何时还能保持任务、何时必须退让？

- **具体问题**：在已知的执行器行程/速率、实际伺服动态和有限时延条件下，哪些基座运动与初始状态允许将姿态、钻尖误差保持在规定范围？何时只有限制姿态精度、停止进给或人工回中才可恢复？
- **最近文献**：R19/R20的执行器动态预测控制，R21的饱和参考规划，R23的时间律规划，R18的输入延迟，R25的reference governor；P01/P02是必须补查的新近冲突线索。
- **实质差异尚待证明**：外部扰动不可时间缩放，封装位置伺服只能跟踪有限参考；需要具体的可维持/可恢复条件或误差边界，而不是一般饱和控制器。
- **可能突破口**：将现实执行能力与任务容差连接为可验证边界，解释几何可行却跟不上、瞬时有解却后续无解的情况。
- **复用与难度**：可复用仿真与硬件；MATLAB难度中—高，需可信伺服模型；ROS2实机难度高，受时序、真实反馈和独立测量限制。
- **最大风险**：传统MPC/RG或近期船载控制已覆盖；若只增加延迟、速度界和有限预测域，仍是常规组合。
- **建议**：在P1确认有实际任务余量后优先继续全文调查；也可能在无辅助平移时单独成立，但需另行重定义问题。

### 候选P3：移动基座双IMU与执行器反馈的时间一致性、可辨识性和控制后果

- **具体问题**：两路惯性姿态与关节反馈在不同采样/到达时刻出现时，哪些相对姿态、偏置和时间偏移能被闭链约束区分？剩余估计误差是否成为补偿能力的主要限制？
- **最近文献**：R32的实机UKF及其明确时间问题、R33的ESKF直接闭链更新、R34的在线时间标定；R13说明双IMU在移动6PUS中已有先例。
- **实质差异尚待证明**：移动基座两套惯性流与特定PUS闭链的联合可观测性/时间模型，以及误差如何影响稳定任务。仅将单IMU扩为双IMU不够。
- **可能突破口**：识别延迟与偏置混淆的条件、设计能检验这些条件的可观测证据、给出对补偿的误差影响分析；本轮不指定滤波器结构。
- **复用与难度**：可利用双IMU、编码器及ROS2记录；MATLAB难度中—高；真机难度高，需要明确传感器测量时刻与通信到达时刻，且需要外部参考。
- **最大风险**：现有异步融合/时间标定方法已足够；当前硬件只有接收时间、没有可信采样时间，可能无法支持想证明的命题；任务误差可能主要来自伺服而非估计。
- **建议**：作为独立备选调查，不预设与P2必须绑定。

### 7.1 暂不建议单独立项的表述

- “TDE/滑模/神经网络/LADRC替换PID后误差更小”：缺少新问题，且R13、R16—R18、R29/R30已有大量近邻。
- “偏心负载下再加一个扰动观测器”：R13/R29及Micron侧载研究已经构成明显先例。只有任务相关能力边界、可辨识限制等实质问题才值得重审。
- “首次将QP用于骨钻6PUS”：应用和机构标签不能代替方法差异。
- “允许任意平移即可扩大姿态工作空间”：与钻尖任务可能矛盾，必须先给出允许集合。

## 第八部分：建议进一步调查的方向

**建议讨论顺序为P1 → P2，P3作为独立备选。** 这不是研发任务书，也没有确认算法路线。

首先明确一类可验证任务：是完整平台姿态，还是钻轴方向；尖端需固定点、容差管道还是只限制横向偏差；是否允许沿轴进给；目标相对世界还是相对骨组织。若骨组织移动，世界姿态稳定与骨组织相对稳定是不同目标。

其次补齐直接冲突文献。优先顺序：R01和R02确认任务变量/约束；R13确认6PUS输入接口与双IMU；R14确认CBF约束、slack和可行条件；R19/R21确认执行器限制与不可行参考策略；P01/P02确认最新论文的版本及具体保证；R38确认手持骨钻工作空间和控制任务。不要先用这些论文的“未知项”写论文创新点。

再判断是否存在值得投入的机制问题。一个可以继续的依据是：在同样任务约束、同样可用感知与真实执行条件下，既有方法仍出现可解释、可重复且有实际意义的局限，而该局限不能由正确配置现有方法消除。若只需常规QP/MPC就能达到指标，应如实定位为工程应用研究，或另找问题；不应为了保留题目增加算法复杂度。

本轮只能支持“宽泛组合创新风险很高”和“有三个需继续核查的问题候选”，尚不能支持“已经找到可发表新机制”。

## 第九部分：仍然缺少的关键证据

### 9.1 文献证据缺口

| 优先级 | 缺口 | 它可能改变的判断 |
|---|---|---|
| 最高 | R01正文及补充材料 | 姿态任务、辅助运动和约束是否已高度覆盖本候选问题；真实硬件状态 |
| 最高 | R02正文 | 冗余来自哪类任务，关节界和冲突恢复具体形式；不能仅凭“joint-constrained”推断 |
| 最高 | R13正文 | 6PUS控制输入、双IMU使用、机构约束、实际伺服以及是否有硬件证据 |
| 最高 | R14正文 | CBF/ECBF的具体安全变量、输入界、CLF松弛、初始安全集和QP可行条件 |
| 高 | R19/R20/R21正文 | 速度/加速度/行程如何处理，参考越界时如何退让，实验平台与求解周期 |
| 高 | P01/P02的online-first、全文和版本 | 2026年最新饱和/延迟/预测补偿是否覆盖P2候选问题 |
| 高 | R38正文；R36后续任务研究 | 手持骨钻实际自由度、人工回中、钻尖及进给约束是否已有解决 |
| 中 | R03/R12最终期刊版与勘误逐式对照 | 学位论文补充细节是否与最终发表式号、模型和假设完全一致 |
| 中 | R33全文及公开实现说明 | 是否已包含多IMU、时延、偏置及可观测性扩展 |
| 中 | 更完整前向追踪及WoS/Scopus复检 | 本轮未取回的引用、数据库新收录和同义词漏检 |

### 9.2 本机与任务事实缺口

这些是后续是否值得开发的证据需求，不表示本轮已获得开发授权。

1. **真实任务允许集合**：钻尖横向/轴向容差、轴方向误差、是否约束滚转、允许进给、接触阶段与人体/骨组织参考系。
2. **约束来源**：行程、速度、加速度、被动关节角、碰撞与奇异区界，分别是图纸值、驱动器配置、仿真值还是测量值；不能混用。
3. **从命令到运动的链路**：50 Hz位置目标之后的插补、限速、跟踪误差、时延及负载依赖；不能用更新频率代替带宽。
4. **测量时间**：双IMU的采样时刻、输出延迟、ROS2接收时刻、关节反馈时刻、时钟关系以及丢包情况。
5. **独立真值**：工具尖端与姿态的外部测量，特别是相对骨组织的位置。用同一IK/FK模型相互校验只能说明内部一致性。
6. **扰动与负载范围**：基座运动幅值、频率、持续时间、偏心质量与钻削反作用的实际范围；不能把海况数据直接当手持数据。
7. **公平比较指标**：姿态/钻轴误差、尖端误差、约束违反量、连续可行时长、必须退让的时间比例、求解/通信耗时及恢复行为，不能只报未饱和段的RMSE。
8. **传统方法是否已足够**：固定平移IK、正确建模的任务QP/HQP、含执行器预测的传统方案等，在相同约束和信息条件下的结果。没有这些证据，不能认定需要新机制。

### 9.3 可以和不可以用于下一轮讨论的结论

**可以使用**：任务冗余、约束QP、移动基座补偿、6PUS浮动基座双IMU建模/抗扰、部分受限参考规划和手持工具稳定均有真实先例；宽泛组合式创新不稳妥。

**仍然未知**：实际骨钻任务是否留下有用辅助运动；完整候选交集是否已被某篇正文覆盖；位置伺服下可维持的误差/扰动边界；双IMU异步问题是不是主要瓶颈。

**本报告提出但未验证**：P1—P3可能形成清楚且可检验的问题。它们不是已确认创新点，更不是本机性能结论。

本轮产物仅为本报告；没有运行MATLAB、创建Simulink模型、修改ROS2工程或启动下一阶段研发。
