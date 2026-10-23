# 文献笔记：Robust State and Protection-Level Estimation within Tightly Coupled GNSS/INS Navigation System

> 元数据级笔记（ref-librarian）。深度理论解读见 `docs/08-参考资料/notes/03-组合导航与鲁棒性.md`（gnss-robust 负责）。

## 0. 题录

| 项 | 内容 |
|---|---|
| 编号 | REF-154 |
| 作者 | Shuchen Liu, Kaizheng Wang, Dirk Abel |
| 年份 | 2023 |
| 标题 | Robust state and protection-level estimation within tightly coupled GNSS/INS navigation system |
| 类型 | 期刊论文（GPS Solutions，开放获取 CC BY 4.0） |
| 出处 | *GPS Solutions*, 27(3), 2023, 111；DOI: 10.1007/s10291-023-01447-z |
| 获取方式 | Springer OA（CC BY），2026-10-21 下载 |
| 本地副本 | `paper.pdf`（3,385,649 B，17 页）、`paper.md`（75,512 字符，MinerU v4 / vlm / en） |
| 质量状态 | `warn`：7 处表格以 HTML `<table>` 输出；图片引用 16 处无缺失 |
| 关联项目模块 | GNSS 鲁棒性（F3）、故障检测/隔离（FDI）、完好性/保护级、H∞ 滤波可行性 |

## 1. 摘要（3-5 句）

1. 城市环境里卫星信号受扰、观测参数化不当，会让 GNSS/INS 的状态估计与其**保护级（protection level, PL）**同时失效；作者提出用**扩展 H∞ 滤波（EHF）**做鲁棒状态估计、用**zonotope（全对称多面体）**做保护级生成（Abstract）。
2. 系统是**紧组合** GNSS/INS：以惯导为核心，GPS/Galileo 双星座的**伪距与伪距率（pseudorange / deltarange）**作为观测（Abstract、§Methodology）。
3. 实验用真实车载数据做后处理，**网格扫描 GNSS 量测噪声参数空间**，比较 EHF 与常规 EKF 的精度/鲁棒性；并用 sigma-ε 模型做同参数对比（§Experimental evaluation）。
4. 结论：EHF 在各种 GNSS 观测参数化与环境条件下都优于 EKF；zonotope 保护级计算**有效、计算量可接受、可实时实现**（Abstract、§Conclusion）。
5. 保护级用 Stanford-ESA Integrity Diagram 验证"误差界与真值的一致性"（§Validation of PL generation using zonotope）。

## 2. 与本项目的关系（5 条）

1. **提供了"不用 NIS 硬门控"的另一条鲁棒路线**：本项目 F3 的失效链是"野值/失配 → 6 维联合新息 NIS 爆炸 → 整体拒绝 → 位置陪葬"。H∞ 滤波不假设噪声统计精确已知，而是对**有界扰动**做最坏情况抑制，理论上不会因 R 参数失配而整体拒测。可作为"EKF + NIS"之外的对照方案评估。
2. **完好性/保护级是本项目完全缺失的能力**：目前固件只报位置估计，不报"这个位置可信到什么程度"。zonotope PL 只需"误差有界"假设（不要求高斯），与本项目 MEMS + 消费级 GNSS 的现实更匹配；实现上是集合的线性变换（生成元矩阵乘法），计算量远小于粒子滤波。
3. **紧组合 vs 本项目现状**：本项目是**松组合**（u-blox PVT 位置/速度，10 Hz）。本文用伪距/伪距率紧组合，能拿到卫星级残差做 FDE，但需要原始观测（RAWX）链路与卫星几何处理，工程量与风险都大。**建议：短期只借 PL/鲁棒思想，不动紧组合架构**。
4. **FDE 与滤波解耦的分层设计**：本文在 EHF 之前先做 GNSS 故障检测与剔除（FDE），再把"无故障观测集"送进滤波器。这与本项目"更新前 NIS 门控"是同一层的不同实现；分层（FDE 层 + 滤波层）比"滤波器内做所有拒绝"更清晰，值得在架构上借鉴。
5. **保护级可作为调参验收指标**：本项目目前以 RMSE/最大误差评估；若引入 PL，则可用"误差是否落在 PL 内"（完整性风险）作为鲁棒性验收指标，比单一 RMSE 更能反映 F3 这类偶发失效。

## 3. 可借鉴点

- **EHF 的滤波器结构**（Appendix: Theory of extended H∞ filter）：与 EKF 结构几乎相同，只改增益中的 Riccati 递推项，**改造成本低**（可视为 EKF 的"鲁棒版增益"），适合作为 F3 的候选 A/B 变体。
- **zonotope 误差界传播**（Appendix: Theory of zonotope）：用生成元矩阵线性传播误差界，实时性好；可用于位置/速度的在线可信区间。
- **参数敏感性网格实验**（§Sensitivity analysis of estimation error vs. c_ρ and c_d）：用两个标度参数扫描，画出 EKF/EHF 的精度曲面——本项目可直接复用该实验设计来评估 R 参数敏感性（对应 P2 的"R 地板不可收紧"结论）。
- **Stanford-ESA Integrity Diagram**（§Validation of PL generation）：验证误差界一致性的标准图，可作为 PL 上线前的验收工具。

## 4. 不适用 / 需谨慎外推

- **后处理实验**：本文结果是后处理（post-processing）得到的，**未验证嵌入式实时性**；本项目 H743 上 200 Hz 的算力约束更紧。
- **车载 + 双星座紧组合**：本项目为消费级单频 GNSS 模块（PVT 输出），无伪距/伪距率原始观测链路的现成实现，紧组合改造属于"架构级"变更。
- **H∞ 的前提**：需要对扰动/噪声的界有合理估计（`c_ρ`、`c_d` 等参数），界设得过松会牺牲精度、过紧会失去鲁棒性；文中给出了扫描方法，但没有"免调参"的结论。
- **没有 VTOL 场景**：车辆运动学约束、低速、地面遮挡模式与飞行器差异明显。
- **zonotope 的保守性**：集合方法天然保守（overestimation），文中也评估了 overestimation 效应；对本项目而言 PL 过大会失去实用价值。

## 5. 关键摘录（便于复核的位置）

- 问题与方案总述：Abstract、Introduction（"a robust approach against environmental disturbances is crucial"）。
- 系统总体结构与 FDE 分层：§Methodology（Fig. 1 概览；"GNSS fault detection and exclusion (FDE) ... is performed after the correction of GNSS observables"）。
- EHF 状态估计：§State estimation applying extended H∞ filter；量测模型：§Measurement model。
- 误差界（zonotope）：§Error-bound estimation using zonotope；保护级生成与完整性风险：§Protection-level generation and integrity risk；实现伪代码汇总：§Summary of the implementation scheme。
- 实验：§Experiment setup、§Experiment overview、§Sensitivity analysis、§Validation of PL generation using zonotope。
- 理论附录：§Appendix → Theory of extended H∞ filter、Theory of zonotope。

## 6. 元数据与转换记录

| 项 | 值 |
|---|---|
| 下载时间 | 2026-10-21 |
| 下载源 | `https://link.springer.com/content/pdf/10.1007/s10291-023-01447-z.pdf`（HTTP 200，`%PDF-` 头校验通过；Unpaywall 记该 DOI 为 OA） |
| 转换命令 | `mineru_convert.py paper.pdf -o <dir> --model-version vlm --language en` |
| 字符数 | 75,512；标题 20 个 |
| 转换告警 | 7 处 HTML 表（本文表格较多，如需管道表请跑 `mineru_postprocess.py`） |
| SHA256 | 见 `docs/08-参考资料/index/catalog.json` 的 `pdf_sha256` |

## 7. 引用建议

- 在"鲁棒滤波/完好性"章节引用：`[REF-154] §State estimation applying extended H∞ filter、§Error-bound estimation using zonotope`。
- 在"F3 速度通道失锁的替代方案"段落与 `[REF-144] ROCRCKF`、`[REF-153] 欺骗检测`、`[REF-152] RAIM` 并列，讨论"硬门控 / 软加权 / H∞ / 保护级"四类方案。
- 引用量化结论时标注"后处理、车载、双星座紧组合"。
