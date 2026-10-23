# 文献笔记：Observability-Constrained Resampling-Free Cubature Kalman Filter for GNSS/INS with Measurement Outliers

> 元数据级笔记（ref-librarian）。深度理论解读见 `docs/08-参考资料/notes/02-卡尔曼滤波理论.md`（kf-theory）与 `notes/03-组合导航与鲁棒性.md`（gnss-robust）。

## 0. 题录

| 项 | 内容 |
|---|---|
| 编号 | REF-144 |
| 作者 | Bingbo Cui, Wu Chen, Duojie Weng, Xinhua Wei, Zeyu Sun, Yan Zhao, Yufei Liu |
| 年份 | 2023 |
| 标题 | Observability-Constrained Resampling-Free Cubature Kalman Filter for GNSS/INS with Measurement Outliers |
| 类型 | 期刊论文（MDPI Remote Sensing，开放获取 CC BY 4.0） |
| 出处 | *Remote Sensing*, 15(18), 2023, 4591；DOI: 10.3390/rs15184591 |
| 获取方式 | MDPI 出版社 OA（CC BY），2026-10-21 从 MDPI 内容 CDN 下载 |
| 本地副本 | `paper.pdf`（4,185,355 B，21 页）、`paper.md`（77,196 字符，MinerU v4 / vlm / en） |
| 质量状态 | `warn`：3 处表格以 HTML `<table>` 输出；图片引用 22 处无缺失 |
| 关联项目模块 | GNSS/INS 量测更新与野值（F3）、NIS 门控之外的鲁棒方案、可观性约束、CKF 可行性评估 |

## 1. 摘要（3-5 句）

1. GNSS 受遮挡/干扰时 GNSS/INS 会出现**量测野值**，导致性能退化；已有结论是"先验不确定度较大时 sigma 点类 KF 优于 EKF"（Abstract）。
2. 作者在其前期工作"免重采样 sigma 点更新框架（SUF）→ RCKF"基础上，提出 **ROCRCKF**（robust observability-constrained RCKF）：用**伯努利变量对野值发生建模，并与状态一起估计**（Abstract）。
3. 同时引入**自适应量测噪声协方差估计**与**野值检测**，并施加**可观性约束**以保持不可观子空间（§2.3、§3）。
4. 用**车 Mounted GNSS/INS** 实验验证：相比 RCKF，航向误差由 1.96° 降至 0.27°，位置平均 RMSE 由 6.38 m 降至 5.95 m（Abstract、§4.2）。
5. 作者结论：该算法对量测野值与时变模型不确定性鲁棒，适合 GNSS 受限环境的实时 GNSS/INS（§5–§6）。

## 2. 与本项目的关系（5 条）

1. **"野值=状态"的思路值得借鉴**：把野值发生建模为伯努利变量、与状态联合估计，等价于让滤波器**自适应地学习"这次量测可信度"**，而不是像本项目 `ComputeGnssInnovationStats()` 那样用 NIS 阈值硬拒绝。硬拒绝在 F3 中的问题是"野值把整个 6 维更新一起否掉"，导致位置陪葬；软加权/概率模型是本项目值得评估的方向。
2. **可观性约束与 REF-138 同族**：§2.3 的 OC 处理与 Hesch 2014 的零空间保持思路一致；本文把它与 sigma 点滤波结合。对本项目而言，说明"OC 思想不限于 EKF"。
3. **自适应 R 与本项目的地板值**：本项目已实测"GNSS R 地板（2/3/0.15/0.25）不可收紧"（TEAM-BRIEF P2），本文的自适应 R 估计给出了一条"让 R 随环境变化"的替代路径，但需注意：自适应 R 在观测质量差时可能把 R 调得更大 → 更依赖惯导，需与失锁重锚逻辑配合。
4. **CKF 上机可行性**：CKF 需要 2n 个 sigma 点传播（15 状态 → 30 次传播/更新），在 H743 上 200 Hz 主循环内代价过高；本文也承认实时性依赖实现优化。**结论倾向：CKF 不作为本项目首选**（与 TEAM-BRIEF 问题 6 一致），但可借其鲁棒机制。
5. **坐标系差异需注意**：本文状态写在 **ECEF（e-frame）**，体轴为 right-forward-up；本项目用 NED/FRD。引用其误差方程时必须做轴系换算，不能直接抄公式。

## 3. 可借鉴点

- **野值检测 + 自适应 R + OC 三件套的组合方式**（§2.3、§3）：可作为"在不换滤波器主体的前提下增强鲁棒性"的设计参考。
- **Bernoulli 野值模型**：若不想扩状态，可退化为"用新息统计量在线估计野值概率，再做加权"，计算量小。
- **对比指标**（航向误差、位置 RMSE）与实验设计（车载跑车）可直接对应本项目的评估口径，便于横向比较。
- **免重采样 sigma 点更新**（§2.2）：若未来确实要上非线性滤波，这是降低计算量的一个方向。

## 4. 不适用 / 需谨慎外推

- **车载平台**：低速、二维运动为主、GNSS 观测条件与 VTOL 差异大（本项目有悬停、过渡大俯仰、差速偏航）。
- **e-frame 编排**：与固件 NED 编排不同，误差方程与 F 矩阵不可直接复用。
- **CKF 计算量**：见上，H743 上 200 Hz 不现实（除非降频/降维）。
- **"航向误差 1.96°→0.27°"不可当作本项目预期收益**：这是车载 GNSS 良好条件下的姿态误差，本项目航向受弱可观与磁未接入双重限制。
- **开放获取版本为出版社排版**：公式编号与页码以 DOI 版本为准。

## 5. 关键摘录（便于复核的位置）

- 问题背景与野值影响：Abstract、§1 Introduction。
- RCKF/SUF 基础：§2.1 Cubature Kalman Filter、§2.2 Resampling-Free Sigma-Point Update Framework。
- 可观性约束的免重采样 CKF：§2.3 Observability-Constrained Resampling-Free CKF。
- 方法主体（野值建模、自适应 R、检测）：§3 Methodology。
- GNSS/INS 滤波模型（e-frame、状态 `[δΨ, δv, δp, b_a, b_g]`、误差方程 (50)–(52)）：§4.1。
- 实验结果与讨论：§4.2、§5 Discussion、§6 Conclusions。

## 6. 元数据与转换记录

| 项 | 值 |
|---|---|
| 下载时间 | 2026-10-21 |
| 下载源 | `https://mdpi-res.com/d_attachment/remotesensing/remotesensing-15-04591/article_deploy/remotesensing-15-04591.pdf`（HTTP 200，`%PDF-` 头校验通过；`www.mdpi.com/.../pdf` 直链被 Cloudflare 403，改用出版社内容 CDN） |
| 转换命令 | `mineru_convert.py paper.pdf -o <dir> --model-version vlm --language en` |
| 字符数 | 77,196；标题 13 个 |
| 转换告警 | 3 处 HTML 表 |
| SHA256 | 见 `docs/08-参考资料/index/catalog.json` 的 `pdf_sha256` |

## 7. 引用建议

- 在"GNSS 野值/鲁棒更新"章节引用：`[REF-144] §2.3、§3、§4.2`。
- 与 `[REF-153] Liu 2019`（新息序列欺骗检测）、`[REF-154] Liu 2023`（EHF+保护级）、`[REF-152] Brown 1992`（RAIM）并列，讨论"硬门控 vs 软加权 vs 完整性监测"的取舍。
- 引用其量化结果时必须标注"车载、e-frame、良好 GNSS 条件"。
