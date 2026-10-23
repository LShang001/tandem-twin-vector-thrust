# 文献笔记：On-Manifold Preintegration for Real-Time Visual-Inertial Odometry

> 元数据级笔记（ref-librarian）。深度理论解读见 `docs/08-参考资料/notes/02-卡尔曼滤波理论.md`（kf-theory 负责）。

## 0. 题录

| 项 | 内容 |
|---|---|
| 编号 | REF-169 |
| 作者 | Christian Forster, Luca Carlone, Frank Dellaert, Davide Scaramuzza |
| 年份 | 2017（期刊版；arXiv v3 为 2016-10-30） |
| 标题 | On-Manifold Preintegration for Real-Time Visual-Inertial Odometry |
| 类型 | 期刊论文（IEEE Transactions on Robotics） |
| 出处 | *IEEE Trans. Robotics*, 33(1), 2017, 1–21；DOI: 10.1109/TRO.2016.2597321 |
| 预印本 | arXiv:1512.02363v3（21 页，本库下载版本；Unpaywall 记为 Green OA） |
| 获取方式 | arXiv 开放获取，2026-10-21 下载 |
| 本地副本 | `paper.pdf`（3,816,161 B，21 页）、`paper.md`（150,662 字符，MinerU v4 / vlm / en） |
| 质量状态 | `ok`（无告警；图片引用数由 quality_report 记录） |
| 关联项目模块 | 延迟回放/OOSM 的时间处理、IMU 高频数据压缩、因子图/优化路线评估（TEAM-BRIEF 问题 6） |

## 1. 摘要（3-5 句）

1. VIO 用非线性优化可获得高精度，但轨迹变长后实时优化不可行，而 IMU 高频测量让变量数快速膨胀；本文的核心手段是**把关键帧之间的惯性测量预积分（preintegration）成单个相对运动约束**（Abstract、§I）。
2. 第一项贡献是**正确处理旋转群流形结构**的预积分理论：给出生成式量测模型、旋转噪声的性质，并推导 MAP 估计器的表达式；所有雅可比（残差、噪声传播、**事后零偏校正**）都有解析形式（Abstract、§VI）。
3. 第二项贡献是把预积分 IMU 模型无缝嵌入**因子图**框架，从而可用增量平滑（iSAM2），并配合"无结构（structureless）视觉量测"避免优化三维点（Abstract、§IV、§VII）。
4. 在真实与仿真数据集上评估单目 VIO 管线，实现**100 Hz 的全平滑**并优于对比方法（Abstract、§VIII）。
5. 论文同时具有教程价值：§III 给出流形上不确定度表示与雅可比推导的精简综述，附录给出全部公式（§III、Appendix）。

## 2. 与本项目的关系（5 条）

1. **"把高频 IMU 压缩成相对约束"正好对应本项目的延迟回放问题**：本项目用 64 格环形快照 + 15 ms 延迟回放（`GetDelayedSnapshotIndexByAge()`、`RepropagateFromDelayedSnapshot()`）处理量测时间戳滞后；预积分给出了另一种等价视角——把"从 t₁ 到 t₂ 的 IMU 增量"一次性合成，避免逐帧回放。**注意：预积分是为优化器设计的，不是为 EKF 设计的**，直接搬到 ESKF 不成立，但可用于理解"回放误差如何随积分长度增长"。
2. **"事后零偏校正"（§VI-C）对本项目有直接借鉴**：ESKF 中零偏状态更新后，历史传播的增量会失配；预积分给出的"零偏变化时如何一阶修正预积分量及其雅可比"的公式，可用于评估本项目"零偏更新后是否需要重传播"的取舍。
3. **TEAM-BRIEF 问题 6（因子图/优化是否值得上机）的判据来源**：本文报告 100 Hz 全平滑的算力需求，是在桌面/嵌入式计算平台上实现的优化器（GTSAM）；对 H743（无 FPU 大矩阵优化、无 OS 内存管理）而言，**因子图路线不可行**，这为"不走优化路线"提供了文献依据。
4. **流形上不确定度表示**（§III-B）与 REF-136 的 ESKF 教程互补：一个从"优化/因子图"角度，一个从"滤波"角度；两者都强调旋转不能按欧氏向量处理。
5. **无结构视觉量测**（§VII）不适用本项目（无相机），但其"边缘化掉高维辅助状态"的思想可类比到"光流尺度/气压零偏等辅助状态是否值得进状态向量"的决策。

## 3. 可借鉴点

- **§VI-A 预积分量的定义**（ΔR, Δv, Δp 及其与零偏的关系）：是理解"IMU 增量如何组合"的清晰参考。
- **§VI-B 噪声传播**：给出从连续噪声到预积分量协方差的传播方式，可对照本项目 Q 的离散化实现。
- **§VI-C 零偏更新的一阶修正**：`∂ΔR/∂b_g` 等雅可比的解析式，工程上可直接用于"零偏变化后修正增量"。
- **§III-C 流形上的 Gauss-Newton**：如果未来确实要做批量优化（如离线标定、轨迹平滑），这是起点。

## 4. 不适用 / 需谨慎外推

- **视觉惯性系统**：本文量测是相机特征（重投影误差），本项目无相机；光流测速与视觉特征跟踪在数学形式与噪声特性上不同。
- **优化 ≠ 滤波**：预积分的"相对约束"服务于 MAP 优化；ESKF 是递推滤波，二者的误差传播、协方差含义不同，不能混用公式。
- **算力假设**：100 Hz 全平滑依赖桌面级/强嵌入式平台与成熟优化库；H743 上不现实。
- **平台差异**：手持/无人机室内 VIO 场景，无 GNSS、无长航时地球模型项；本项目为室外长航时 VTOL。
- **尺度与重力**：单目 VIO 的尺度可观性由 IMU 提供，本文部分讨论围绕此展开，与本项目（GNSS 提供绝对尺度）不同。

## 5. 关键摘录（便于复核的位置）

- 问题与两条贡献：Abstract、§I Introduction。
- 流形基础（不确定度、GN 迭代）：§III（A Riemannian notions / B Uncertainty on SO(3) / C Gauss-Newton on manifold）。
- MAP 与因子图：§IV（A 状态 / B 量测 / C 因子图与 MAP）。
- IMU 模型与运动积分：§V。
- **预积分主体**：§VI（A Preintegrated IMU Measurements / B Noise Propagation / C Incorporating Bias Updates / D Preintegrated IMU Factors / E Bias Model）。
- 无结构视觉因子：§VII；实验：§VIII（A 仿真…）。

## 6. 元数据与转换记录

| 项 | 值 |
|---|---|
| 下载时间 | 2026-10-21 |
| 下载源 | `https://arxiv.org/pdf/1512.02363`（HTTP 200，`%PDF-` 头校验通过；Unpaywall 对该 DOI 同时给出 arXiv 与 ZORA 两个 green OA 位置） |
| 转换命令 | `mineru_convert.py paper.pdf -o <dir> --model-version vlm --language en` |
| 字符数 | 150,662；标题数由 quality_report 记录 |
| 转换告警 | 无 |
| SHA256 | 见 `docs/08-参考资料/index/catalog.json` 的 `pdf_sha256` |

## 7. 引用建议

- 在"延迟回放/时间处理"章节引用：`[REF-169] §VI-A、§VI-C`（作为"增量合成 vs 逐帧回放"的对照观点），并与 `[REF-151] Bar-Shalom OOSM`、`[REF-159] Skog 时间同步` 并列。
- 在"为何不上因子图/优化"章节引用：`[REF-169] §VIII`（算力与平台前提），作为 TEAM-BRIEF 问题 6 的文献依据。
- 引用时明确"本文是优化框架下的预积分，不直接适用于 EKF 递推"。
