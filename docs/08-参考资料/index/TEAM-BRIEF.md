# 导航算法研究组 · 团队简报（TEAM-BRIEF）

> 版本：2026-10-21 · 维护：Lead
> 所有成员先读本简报，再读各自任务卡。**不要修改固件源码**；研究/笔记/仿真之外不改文件。

---

## 1. 项目与任务

**平台**：纵列双发正交单轴矢量推力 + 差速偏航的 VTOL 概念飞行器。
悬停构型：上摆=滚转、下摆=俯仰、差速=偏航；NED/FRD，四元数 RK4。
飞控固件：`TandemVec_FCS`（STM32H743，200 Hz 导航任务）。
当前阶段：实机首飞通过，进入调参优化；仿真参数仍多为 MODEL-DEFAULT，需要实机数据标定。

**本轮目标**：
1. 建立项目级文献资料管理体系（PDF/转换/笔记/注册表/索引/校验）。
2. 精读秦永元《惯性导航（第三版）》与《卡尔曼滤波与组合导航原理（第3版）》的相关章节，
   以及 EKF/组合导航/捷联惯导的专业文献。
3. 对照现有 15 状态 ESKF 导航代码，回答“还能不能进一步优化升级”，给出**有证据、分优先级、
   可落地**的升级路线图（含仿真验证与风险）。

---

## 2. 现有导航系统（事实基线）

### 2.1 架构

- 核心：`TandemVec_FCS/lib/navigation-main/src/ekf_15_state.h`（约 4058 行）
  - 15 状态误差状态 EKF：位置(3)、NED 速度(3)、体系姿态误差 δβ^b(3)、加计零偏(3)、陀螺零偏(3)。
  - 双子样 IMU 增量、圆锥/划摇补偿、WGS-84 地球模型、完整 F 矩阵（`FULL_F_MATRIX=1`）。
  - GNSS 6 维位置/速度联合更新，NIS 门控 + 失联膨胀 + 低增益重捕获 + 延迟回放重传播。
  - ZUPT / Gravity / StaticGyro / Attitude / Yaw / Baro 等量测更新；Joseph 形式；协方差稳定化。
  - 延迟回放：64 格环形快照，15 ms 典型延迟。
- 任务层：`TandemVec_FCS/src/navigation_task.cpp`（约 1962 行）
  - 200 Hz 主循环；IMU 双子样组装；静止检测（`include/ins_static_detector.h`）；
  - GNSS 质量分级（全质量/降级）、动态 R（`include/ins_gnss_dynamic_weight.h`）、
    时间映射（`include/ins_gnss_epoch_timing.h`）、静止辅助分级（`include/ins_static_aid_profile.h`）、
    气压高度（`BFS_EKF_BARO_ALTITUDE_UPDATE`）、光流速度辅助/重锚、双矢量航向。
- 传感器：ICM45686 IMU；UBX GNSS（PVT，10 Hz）；DPS310 气压计；IST8310 磁力计（**已接入驱动/
  标定链路，但磁力计尚未馈入 EKF/Madgwick**，`mag_ut` 仍为伪磁场 `(30,0,40)`）；光流+测距
  （UART7 自动识别，MSP/MAVLink）；无空速管。

### 2.2 已知机制性风险（来自既有报告，必须接着做，不要重复发现）

- **F1 误静止巡航**：水平恒速飞行与静止在 IMU 上不可区分；静止误确认 → GNSS 速度强制 0 →
  6 维联合更新 NIS 爆炸 → 位置一并被拒 → 纯惯导漂移。已修：GNSS 速度 >1.5 m/s 强制退出静止。
- **F2 航向弱可观踢动**：`F(v,β)=−C^n_b[f_b]×`；P0 航向 π 时过渡/转弯的大比力水平分量把
  GNSS 速度噪声放大为 ±10–25°/epoch 的航向踢动；航向误差 >~9° 后双矢量量测 NIS 超敏全拒。
  已修：P0 航向 π→0.2 rad；姿态误差 55–170°→5–17°，但过渡期航向仍漂（SC4 18.7°）。
- **F3 转弯段速度失控锁死（F2 残余）**：P0 修复后仍 1/8 种子偶发；航向踢动 → 速度被踢 3–9 m/s →
  GNSS 6 维联合更新整体拒绝 → 速度态永久失锁 → 位置 ~30 m/s 发散。15 ms 回放是共因非根因。
  **开放项：速度通道鲁棒恢复 / 航向可观测性门控。**
- **P1** ZUPT NIS 1000→16.3 已修；**P2** GNSS R 地板不可收紧；**P3** Q 加计 0.25→0.05 有收益但
  缺实机震动数据，暂不动；**P4** 气压辅助是失联期垂直通道唯一硬约束，维持。
- 审查报告已修：H1（降级路径 AHRS 航向闭环）、H2（标量航向倾斜补偿）、MED-0b/1/2/3/6/8/10/11/14
  等；剩余 MED-5/7/9/15 与 LOW 项属防御/一致性，按需处理。
- 2026-08-21 光流：双新鲜度通道、室内静止出口、速度失锁重锚
  （`MeasurementUpdateAidingVelocityDetailed` / `ResetVelocityToExternal`，连续 50 次拒绝自动重锚）。
- 2026-08-15 磁力计：IST8310 驱动/椭球标定/Flash 存取已就绪；`kMagFeedMadgwick=false`，
  **上板轴映射与椭球标定未验证前不得直接启用**。

### 2.3 关键代码入口（差距分析必须引用行号/函数名）

| 主题 | 入口 |
|---|---|
| EKF 状态/传播 | `ekf_15_state.h`：`PropagateStateFromIncrements()`、`TimeUpdateTwoSample()`、`ComputeEarthModelTerms()` |
| 误差注入/协方差 | `InjectErrorState()`、`StabilizeCovariance()`、`StabilizeCovarianceAfterPropagation()` |
| GNSS 更新/门控 | `MeasurementUpdateDetailed()`、`ComputeGnssInnovationStats()`、`BuildConservativeGnssReacquisitionNoise()`、`GrowGnssOutageCovariance()` |
| 延迟回放 | `GetDelayedSnapshotIndexByAge()`、`RestoreSnapshot()`、`RepropagateFromDelayedSnapshot()` |
| 速度/ZUPT | `ApplyVelocityUpdateDetailed()`、`MeasurementUpdateAidingVelocityDetailed()`、`ResetVelocityToExternal()` |
| 姿态/重力/航向 | `ApplyAttitudeUpdateDetailed()`、`ApplyGravityUpdateDetailed()`、`ApplyYawUpdateDetailed()`（H2 倾斜补偿） |
| 任务层 GNSS | `navigation_task.cpp`：`gnssEpochPassesFullQuality()`、`applyDowngradedGnssUpdate()`、`handleDualVectorYawFusion()`、`handleIndoorOpticalFlowVelocityFusion()` |
| 静止/辅助 | `ins_static_detector.h`、`ins_static_aid_profile.h`、`ins_gnss_dynamic_weight.h`、`ins_gnss_epoch_timing.h` |
| 仿真 | `TandemVec-EKF-Report/sim/opt/opt_ekf.py`、`run_baseline.py`、`run_variants.py`；`sim/hifi/` |

---

## 3. 文献与本地资料

- 文献地图与题录：`docs/08-参考资料/index/bibliography.md`（先读）。
- 两本教材：
  - `books/REF-120-Qin-2020-inertial-navigation-3e/`（印刷页+8=PDF 页；重点 Ch7–Ch9）
  - `books/REF-121-Qin-2015-kalman-filter-integrated-navigation-3e/`（扫描 OCR；重点 Ch3、Ch4、Ch6、Ch7、Ch8）
- 现有内部报告：
  - `TandemVec-EKF-Report/审查报告-固件EKF导航全面审查.md`
  - `TandemVec-EKF-Report/优化研究-算法与参数优化研究报告.md`
  - `TandemVec-EKF-Report/高保真固件闭环仿真报告.md`
  - `TandemVec-EKF-Report/main.pdf`（80 页级理论与算法报告，含目录/章节）
- 论文库：`docs/08-参考资料/papers/REF-xxx/`（`paper.pdf`/`paper.md`/`notes.md`）。
- 新文献由 `ref-librarian` 处理；**不要自己改注册表/编号**。

---

## 4. 角色与写作范围（严格不重叠）

| 角色 | 任务 | 只写这些路径 |
|---|---|---|
| Lead | 基线、代码差距、综合、路线图、最终报告 | `docs/08-参考资料/index/`、`notes/00-*`、`README.md`、`参考资料注册表.md`、`TandemVec-EKF-Report/导航算法升级评估-*.md` |
| ref-librarian | 文献库基础设施、下载/转换/质量/目录/校验 | `docs/08-参考资料/papers/REF-*`、`index/catalog.json`、`scripts/check-refs.py`、`books/*/notes.md` 之外的元数据 |
| sins-theory | 捷联惯导理论（Ch7–9 + Savage/Ignagni） | `docs/08-参考资料/notes/01-捷联惯导理论.md` |
| kf-theory | 卡尔曼滤波/ESKF/可观性/一致性（Ch2–4/6 + Solá/Markley/Hesch/Barrau） | `docs/08-参考资料/notes/02-卡尔曼滤波理论.md` |
| gnss-robust | 组合导航/容错/自适应/FDI（Ch7–8 + Mohamed/Mehra/Liu/Brown） | `docs/08-参考资料/notes/03-组合导航与鲁棒性.md` |
| heading-mag | 航向/磁辅助/初始对准（Ch8.3/9.9 + Klein/Barczyk/Gebre/VQF） | `docs/08-参考资料/notes/04-航向与磁辅助.md` |
| sim-analyst | 候选方案仿真 A/B（磁航向、GNSS 故障隔离、自适应 Q） | `TandemVec-EKF-Report/sim/opt/upgrade_*.py`、`TandemVec-EKF-Report/sim-data/opt/upgrade_*` |

---

## 5. 输出格式（每份笔记必须包含）

```markdown
# <主题> — 理论精读与项目映射

## 0. 结论摘要（5 条以内，先给判断）
## 1. 理论要点（带公式/页码）
- 每条：结论 + 公式 + 出处（[REF-120 §9.3 p.267] / DOI）
## 2. 与现有实现的逐项对照
| 理论要求 | 固件现状（文件:行号/函数） | 偏差 | 影响量级 | 证据 |
## 3. 可升级项（按 P0/P1/P2）
- 每项：动机、文献依据、实现方案（具体到函数/状态/矩阵）、风险、验证方法、预期收益
## 4. 不建议/不可行的方案（含理由）
## 5. 开放问题与需要实机数据验证的点
## 6. 参考（REF 编号 + 章节/页码）
```

**质量红线**：
- 公式必须能追溯到文献页码/章节；代码结论必须能追溯到文件:行号/函数名。
- 区分「文献结论」「项目证据」「推断/假设」；推断必须标注。
- 不编造文献、页码、DOI、实验数据；不确定就写“未核实”。
- 不把地面车辆/AUV/视觉 VIO 的结论直接外推到本 VTOL；必须写平台差异。
- 量化优先：能用现有仿真数据/报告数字就用；没有就给出量级估计与验证方法。

---

## 6. 当前最需要回答的 6 个问题

1. 现有双子样圆锥/划摇、误差模型、地球模型相对秦永元 Ch7–9 / Savage 是否还有**正确性**或
   **精度**缺口？（不是“再加阶数”，而是先确认符号/坐标系/补偿系数）
2. F2/F3 的航向弱可观，文献中的 OC-EKF / FEJ / InEKF / 磁辅助，哪条路**最适合本机**
   （MEMS、无空速、有 IST8310、过渡飞行俯仰 0–90°）？各自代价/风险？
3. GNSS 6 维联合 NIS 拒绝导致“位置陪葬、速度永久失锁”，文献中的**故障检测/隔离**（RAIM、
   序贯更新、子集检验、CUSUM/GLR、联邦/协方差交叉）哪条能最小改动落地？
4. 现有 Q/R 是经验值；文献中的自适应 Q/R、Allan 方差标定、振动谱在线估计，哪些适合 H743
   200 Hz、且不会引入新的失稳？
5. 气压/光流/静止辅助的**状态建模**是否不足（baro bias、flow scale/bias、ZUPT 检测器）？
   升级的边界收益是多少？
6. 哪些“更高级”的方法（UKF/CKF/粒子/因子图/紧组合）在本项目**不值得**做？为什么？

---

## 7. 协作约定

- 每个成员先 `team_task_get` 读任务卡，`team_task_update action=claim` 领取，完成后
  `action=complete` 并在消息里给 5 行摘要。
- 只写自己 scope 内的文件；需要改别人的文件先在消息里说明。
- 引用文献用 `[REF-xxx]`；引用代码用 `文件:行号 函数名`。
- 发现与既有报告冲突，先记录证据，不要静默覆盖。
- 禁止修改：`TandemVec_FCS/src/**`、`TandemVec_FCS/include/**`、`TandemVec_FCS/lib/navigation-main/**`、
  `models/aircraft-model.json`、`parameters.mjs`。
- 书籍 PDF 为个人研究副本，不提交到公开仓库；转换后的 Markdown/笔记可入库。
