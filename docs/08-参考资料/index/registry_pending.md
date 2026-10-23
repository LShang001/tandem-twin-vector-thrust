# 待登记文献行（registry_pending）

> 生成：ref-librarian（task-1），2026-10-21。
> **状态：已于 2026-10-21 由 Lead 合并入主注册表 `../参考资料注册表.md`（REF-122～174，共 53 行）。**
> 本文件保留作合并记录与失败项说明，不再新增；后续新文献直接改主注册表并更新 `catalog.json`。
> **本文件不是主注册表**：`../参考资料注册表.md` 由 Lead 维护。
> 编号规则：主线文献从 **REF-122** 起，与 `index/bibliography.md` §1–§6 完全对齐（119 之后无空号）。
> 状态口径：`已就位` = 本地有 `paper.pdf` + `paper.md` + `notes.md`；`仅题录` = 付费墙或无合法开放副本，只有题录，**不下载**。

---

## 0. 合并摘要

| 项 | 数量 | 说明 |
|---|---|---|
| 待追加行总数 | 48 | REF-122 ~ REF-169（bibliography §1–§6 全部条目） |
| 其中 `已就位` | 8 | REF-136 / 138 / 140 / 144 / 145 / 154 / 163 / 169（PDF + MD + notes 全齐） |
| 其中 `仅题录` | 40 | 付费墙、或开放获取副本被出版社反爬阻断（见 §2 失败项） |
| 本轮新增本地目录 | 8 | `papers/REF-136-Sola-2017-quaternion-eskf/` 等（见 index/catalog.json） |
| 需一并更正 | 2 处 | REF-145 年份 2025→**2026**；REF-138 本地副本为作者主页期刊版（非 UMN 学位论文） |

---

## 1. 待并入主注册表的行（REF-122 ~ REF-169）

### 1.1 专著与教材（§1，全部仅题录）

| 编号 | 标题 | 类型 | 位置/链接 | 用途 | 状态 |
|------|------|------|-----------|------|------|
| REF-122 | Strapdown Inertial Navigation Technology (2nd ed.)（Titterton & Weston, IET 2004） | 专著 | DOI: 10.1049/PBRA017E | 英文 SINS 标准教材；与秦永元书互为交叉校验 | 仅题录 |
| REF-123 | Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems (2nd ed.)（Groves, Artech House 2013） | 专著 | 纸质书（无 DOI） | 组合导航系统级框架、误差预算、辅助源建模 | 仅题录 |
| REF-124 | Aided Navigation: GPS with High Rate Sensors（Farrell, McGraw-Hill 2008） | 专著 | 纸质书（无 DOI） | 高率 GNSS/INS 辅助导航、工程实现视角 | 仅题录 |
| REF-125 | Strapdown Analytics (2nd ed.)（Savage, Strapdown Associates 2007） | 专著 | 出版方直售 | 圆锥/划摇/涡卷的解析推导与系数表 | 仅题录 |
| REF-126 | Stochastic Models, Estimation, and Control, Vol. 1（Maybeck, Academic Press 1979） | 专著 | 纸质书（无 DOI） | 卡尔曼滤波、故障检测与多模型估计理论基础 | 仅题录 |
| REF-127 | Factorization Methods for Discrete Sequential Estimation（Bierman, Academic Press 1977） | 专著 | 纸质书（无 DOI） | UD/平方根滤波；协方差数值稳定性 | 仅题录 |
| REF-128 | Estimation with Applications to Tracking and Navigation（Bar-Shalom et al., Wiley 2001） | 专著 | 纸质书（无 DOI） | 序贯处理、OOSM、IMM、量测门控 | 仅题录 |
| REF-129 | Kalman Filtering: Theory and Practice with MATLAB (4th ed.)（Grewal & Andrews, Wiley 2015） | 专著 | 纸质书（无 DOI） | 工程 KF 实现与数值检查 | 仅题录 |

### 1.2 捷联惯导编排、圆锥/划摇补偿（§2）

| 编号 | 标题 | 类型 | 位置/链接 | 用途 | 状态 |
|------|------|------|-----------|------|------|
| REF-130 | Strapdown Inertial Navigation Integration Algorithm Design Part 1: Attitude Algorithms（Savage, JGCD 21(1) 1998, 19–28） | 期刊论文 | DOI: 10.2514/2.4228 | 多子样旋转矢量/圆锥补偿经典设计；对照现有双子样 | 仅题录 |
| REF-131 | Strapdown Inertial Navigation Integration Algorithm Design Part 2: Velocity and Position Algorithms（Savage, JGCD 21(2) 1998, 208–221） | 期刊论文 | DOI: 10.2514/2.4242 | 划摇/涡卷补偿与速度/位置积分；对照双矢量增量 | 仅题录 |
| REF-132 | Optimal Strapdown Attitude Integration Algorithms（Ignagni, JGCD 13(2) 1990） | 期刊论文 | DOI: 10.2514/3.20558 | 圆锥最优系数；与 Savage 结果互相验证 | 仅题录 |
| REF-133 | Duality of Optimal Strapdown Sculling and Coning Compensation Algorithms（Ignagni, Navigation 45(2) 1998） | 期刊论文 | DOI: 10.1002/j.2161-4296.1998.tb02373.x | 划摇与圆锥算法对偶性；检查现有补偿系数 | 仅题录 |
| REF-134 | Equivalency Between Strapdown Inertial Navigation Coning and Sculling Integrals/Algorithms（Roscoe, JGCD 24(1) 2001） | 期刊论文 | DOI: 10.2514/2.4718 | 圆锥/划摇积分等价性；算法审查参考 | 仅题录 |
| REF-135 | Attitude Error Representations for Kalman Filtering（Markley, JGCD 26(2) 2003, 311–317） | 期刊论文 | DOI: 10.2514/2.5048；OA 副本：NASA NTRS 20020060647 | 姿态误差参数化（乘性/加性、体轴/导航轴）；对照 δβ^b 定义 | 仅题录 |
| REF-136 | Quaternion Kinematics for the Error-State Kalman Filter（Solá, arXiv:1711.02508, 2017） | 预印本/技术报告 | `papers/REF-136-Sola-2017-quaternion-eskf/` | ESKF 权威教程；误差注入/reset 雅可比、RK4、局部 vs 全局误差 | 已就位 |

### 1.3 可观性、一致性与不变 EKF（§3）

| 编号 | 标题 | 类型 | 位置/链接 | 用途 | 状态 |
|------|------|------|-----------|------|------|
| REF-137 | Observability Analysis of Piece-Wise Constant Systems. I. Theory（Goshen-Meskin & Bar-Itzhack, IEEE TAES 28(4) 1992） | 期刊论文 | DOI: 10.1109/7.165367 | 分段常值系统可观性分析；INS/GNSS 不可观子空间 | 仅题录 |
| REF-138 | Consistency Analysis and Improvement of Vision-aided Inertial Navigation（Hesch et al., IEEE T-RO 30(1) 2014, 158–176） | 期刊论文 | `papers/REF-138-Hesch-2014-consistency-vision-aided-ins/` | OC-VINS：保持不可观子空间、抑制虚假信息增益；F2 同构 | 已就位 |
| REF-139 | Observability-Based Rules for Designing Consistent EKF SLAM Estimators（Huang et al., IJRR 29(5) 2010） | 期刊论文 | DOI: 10.1177/0278364909353640 | 可观性约束的 EKF 设计规则；与航向弱可观同构 | 仅题录 |
| REF-140 | The Invariant Extended Kalman Filter as a Stable Observer（Barrau & Bonnabel, IEEE TAC 62(4) 2017, 1797–1812） | 期刊论文 | `papers/REF-140-Barrau-2017-inekf-stable-observer/` | InEKF 稳定性/误差自治；左/右不变观测；InEKF 路线判据 | 已就位 |
| REF-141 | Invariant Extended Kalman Filter Design for a Magnetometer-plus-GPS Aided Inertial Navigation System（Barczyk & Lynch, IEEE CDC 2011） | 会议论文 | DOI: 10.1109/CDC.2011.6160733 | 磁+GPS 辅助 INS 的 InEKF；与本项目传感器组合高度相关 | 仅题录 |
| REF-142 | Invariant EKF Design for Scan Matching-Aided Localization（Barczyk et al., IEEE TCST 23(6) 2015） | 期刊论文 | DOI: 10.1109/TCST.2015.2413933；OA 副本：HAL hal-01256770 | InEKF 在辅助导航中的工程化 | 仅题录 |
| REF-143 | Observability Analysis of Heading Aided INS for a Maneuvering AUV（Klein & Diamant, Navigation 65(1) 2018） | 期刊论文 | DOI: 10.1002/navi.222 | 机动中航向辅助可观测性；F2/F3 直接理论参照 | 仅题录 |
| REF-144 | Observability-Constrained Resampling-Free Cubature Kalman Filter for GNSS/INS with Measurement Outliers（Cui et al., Remote Sens. 15(18) 2023, 4591） | 期刊论文 | `papers/REF-144-Cui-2023-observability-ckf-gnss-ins/` | 野值伯努利建模 + 自适应 R + 可观性约束；F3 软加权路线 | 已就位 |
| REF-145 | MF-IEKF: A Multiplicative Federated Invariant Extended Kalman Filter for INS/GNSS（Zhao et al., Sensors 26(1) **2026**, 127） | 期刊论文 | `papers/REF-145-Zhao-2025-mf-iekf-federated/` | 左/右不变误差组合、联邦 IEKF；GNSS 位置/姿态解耦 | 已就位 |
| REF-146 | GNSS/INS Navigation System Based on Lie Group Right Invariant Kalman Filter（Ye et al., IEEE ICUS 2023） | 会议论文 | DOI: 10.1109/ICUS58632.2023.10318318 | 右不变 KF 的 GNSS/INS 实现 | 仅题录 |
| REF-147 | A Land Vehicle's INS/GNSS Integrated Navigation System Using Left Invariant Extended Kalman Filter（Ibrahim et al., J. Phys. Conf. Ser. 2616(1) 2023, 012023） | 会议论文 | DOI: 10.1088/1742-6596/2616/1/012023（CC BY，OA 但下载被 IOP/Radware 反爬阻断） | 左不变 EKF 的陆用对照 | 仅题录 |

### 1.4 GNSS/INS 组合、鲁棒/自适应、故障检测（§4）

| 编号 | 标题 | 类型 | 位置/链接 | 用途 | 状态 |
|------|------|------|-----------|------|------|
| REF-148 | Adaptive Kalman Filtering for INS/GPS（Mohamed & Schwarz, J. Geodesy 73, 1999, 193–203） | 期刊论文 | DOI: 10.1007/s001900050236 | 新息/残差自适应 R/Q 的经典工程方法 | 仅题录 |
| REF-149 | On the Identification of Variances and Adaptive Kalman Filtering（Mehra, IEEE TAC 15(2) 1970） | 期刊论文 | DOI: 10.1109/TAC.1970.1099422 | 自适应滤波统计基础；避免拍脑袋调 Q/R | 仅题录 |
| REF-150 | The Interacting Multiple Model Algorithm for Systems with Markovian Switching Coefficients（Blom & Bar-Shalom, IEEE TAC 33(8) 1988） | 期刊论文 | DOI: 10.1109/9.1299 | IMM；GNSS 质量/机动模式切换候选 | 仅题录 |
| REF-151 | Update with Out-of-Sequence Measurements in Tracking: Exact Solution（Bar-Shalom, IEEE TAES 38(3) 2002） | 期刊论文 | DOI: 10.1109/TAES.2002.1039398 | OOSM 精确更新；对照现有延迟回放 | 仅题录 |
| REF-152 | A Baseline GPS RAIM Scheme and a Note on the Equivalence of Three RAIM Methods（Brown, Navigation 39(3) 1992） | 期刊论文 | DOI: 10.1002/j.2161-4296.1992.tb02278.x | RAIM 基础；GNSS 故障检测/隔离 | 仅题录 |
| REF-153 | Analysis of Kalman Filter Innovation-Based GNSS Spoofing Detection Method for INS/GNSS（Liu et al., IEEE Sensors J. 19(13) 2019） | 期刊论文 | DOI: 10.1109/JSEN.2019.2902178 | 新息序列欺骗检测；慢漂/多历元监测 | 仅题录 |
| REF-154 | Robust State and Protection-Level Estimation within Tightly Coupled GNSS/INS Navigation System（Liu, Wang & Abel, GPS Solutions 27(3) 2023, 111） | 期刊论文 | `papers/REF-154-Liu-2023-robust-protection-level-gnss-ins/` | 扩展 H∞ 鲁棒估计 + zonotope 保护级；完好性监测 | 已就位 |
| REF-155 | Federated Square Root Filter for Decentralized Parallel Processors（Carlson, IEEE TAES 26(3) 1990） | 期刊论文 | DOI: 10.1109/7.106130 | 联邦滤波；多源容错/故障隔离架构 | 仅题录 |
| REF-156 | A Non-Divergent Estimation Algorithm in the Presence of Unknown Correlations（Julier & Uhlmann, ACC 1997） | 会议论文 | DOI: 10.1109/ACC.1997.609105 | 协方差交叉；分散传感器融合 | 仅题录 |
| REF-157 | A Constrained Total Extended Kalman Filter for Integrated Navigation（Mahboub & Mohammadi, J. Navigation 71(4) 2018） | 期刊论文 | DOI: 10.1017/S0373463318000012 | 状态约束在组合导航中的应用 | 仅题录 |
| REF-158 | An Adaptive Kalman Filter Estimating Process Noise Covariance（Wang et al., Neurocomputing 223, 2017, 12–17） | 期刊论文 | DOI: 10.1016/j.neucom.2016.10.026 | 过程噪声在线估计；振动自适应 Q 候选 | 仅题录 |
| REF-159 | Effects of Time Synchronization Errors in GNSS-Aided INS（Skog & Handel, IEEE/ION PLANS 2008） | 会议论文 | DOI: 10.1109/PLANS.2008.4570010 | 时间同步误差量化；延迟回放/时间戳设计 | 仅题录 |

### 1.5 辅助源、约束与传感器（§5）

| 编号 | 标题 | 类型 | 位置/链接 | 用途 | 状态 |
|------|------|------|-----------|------|------|
| REF-160 | Zero-Velocity Detection—An Algorithm Evaluation（Skog et al., IEEE TBME 57(11) 2010） | 期刊论文 | DOI: 10.1109/TBME.2010.2060723 | ZUPT 检测器比较；静止检测阈值/迟滞 | 仅题录 |
| REF-161 | The Aiding of a Low-Cost Strapdown IMU Using Vehicle Model Constraints for Land Vehicle Applications（Dissanayake et al., IEEE T-RA 17(5) 2001） | 期刊论文 | DOI: 10.1109/70.964672 | 非完整约束（NHC）；巡航零侧滑约束参照 | 仅题录 |
| REF-162 | Calibration of Strapdown Magnetometers in Magnetic Field Domain（Gebre-Egziabher et al., J. Aerospace Eng. 19(2) 2006） | 期刊论文 | DOI: 10.1061/(ASCE)0893-1321(2006)19:2(87) | 椭球/硬铁/软铁标定；磁航向工程基础 | 仅题录 |
| REF-163 | VQF: Highly Accurate IMU Orientation Estimation with Bias Estimation and Magnetic Disturbance Rejection（Laidig & Seel, Information Fusion 91, 2023, 187–204） | 期刊论文 | `papers/REF-163-Laidig-2023-vqf/` | 磁干扰三段式门控、陀螺零偏估计；IST8310 接入参考 | 已就位 |
| REF-164 | On-Board Velocity Estimation and Closed-Loop Control of a Quadrotor UAV Based on Optical Flow（Grabe et al., IEEE ICRA 2012） | 会议论文 | DOI: 10.1109/ICRA.2012.6225328 | 光流测速/尺度；室内无 GNSS 速度辅助 | 仅题录 |
| REF-165 | A Comparison of Scale Estimation Schemes for a Quadrotor UAV Based on Optical Flow and IMU（Grabe et al., IEEE/RSJ IROS 2013） | 会议论文 | DOI: 10.1109/IROS.2013.6697107 | 光流尺度在线估计；现有 flow/range 方案升级参考 | 仅题录 |
| REF-166 | Analysis and Modeling of Inertial Sensors Using Allan Variance（El-Sheimy et al., IEEE TIM 57(1) 2008） | 期刊论文 | DOI: 10.1109/TIM.2007.908635 | IMU 噪声/零偏标定；Q 参数实机标定方法 | 仅题录 |

### 1.6 非线性滤波与优化（§6，评估用）

| 编号 | 标题 | 类型 | 位置/链接 | 用途 | 状态 |
|------|------|------|-----------|------|------|
| REF-167 | A New Method for the Nonlinear Transformation of Means and Covariances in Filters and Estimators（Julier et al., IEEE TAC 45(3) 2000） | 期刊论文 | DOI: 10.1109/9.847726 | UKF 原始论文；与 EKF 的精度/算力权衡 | 仅题录 |
| REF-168 | Cubature Kalman Filters（Arasaratnam & Haykin, IEEE TAC 54(6) 2009） | 期刊论文 | DOI: 10.1109/TAC.2009.2019800 | CKF；高维非线性滤波替代方案 | 仅题录 |
| REF-169 | On-Manifold Preintegration for Real-Time Visual-Inertial Odometry（Forster et al., IEEE T-RO 33(1) 2017, 1–21） | 期刊论文 | `papers/REF-169-Forster-2017-on-manifold-preintegration/` | 流形预积分/因子图；延迟回放与"是否上优化"判据 | 已就位 |

---

## 2. 本轮下载/转换结果与失败项

### 2.1 已完成（8 篇：PDF + MinerU MD + notes.md）

| REF | 目录 | 页数 | paper.md 字符 | MinerU 质量 | 来源（均已过 `%PDF-` 头校验） |
|---|---|---|---|---|---|
| REF-136 | `papers/REF-136-Sola-2017-quaternion-eskf/` | 95 | 258,932 | warn（5 HTML 表） | arXiv:1711.02508 |
| REF-138 | `papers/REF-138-Hesch-2014-consistency-vision-aided-ins/` | 19 | 118,190 | ok | 作者主页（UMN）期刊版 PDF |
| REF-140 | `papers/REF-140-Barrau-2017-inekf-stable-observer/` | 36 | 129,734 | ok | arXiv:1410.1465 |
| REF-144 | `papers/REF-144-Cui-2023-observability-ckf-gnss-ins/` | 21 | 77,058 | warn（3 HTML 表） | MDPI 内容 CDN（CC BY） |
| REF-145 | `papers/REF-145-Zhao-2025-mf-iekf-federated/` | 31 | 112,521 | warn（2 HTML 表） | MDPI 内容 CDN（CC BY） |
| REF-154 | `papers/REF-154-Liu-2023-robust-protection-level-gnss-ins/` | 17 | 75,199 | warn（7 HTML 表） | Springer OA（CC BY） |
| REF-163 | `papers/REF-163-Laidig-2023-vqf/` | 22 | 105,491 | warn（3 HTML 表） | arXiv:2203.17024（Green OA） |
| REF-169 | `papers/REF-169-Forster-2017-on-manifold-preintegration/` | 21 | 150,471 | ok | arXiv:1512.02363 |

> 质量 audit 工具：`mineru_quality.py`；全部 8 篇 `failed=0`，告警仅为"表格以 HTML 输出"，不影响正文与公式。
> 转换命令：`mineru_convert.py <pdf> -o <dir> --model-version vlm --language en`；`full.md` → `paper.md`。

### 2.2 失败 / 未完成（明确列出，非静默跳过）

| REF | 论文 | 原因 | 处置 |
|---|---|---|---|
| REF-147 | Ibrahim 2023（IOP JPCS 2616 012023） | 开放获取（CC BY），但 IOP 站点由 Radware bot manager 拦截，curl/Python 直连均返回 302 → 校验页（JS 挑战）；本机浏览器 provider 不可用，无法完成验证 | 仅题录；PDF **未下载** |
| REF-135 | Markley 2003（NASA NTRS 20020060647） | Unpaywall 判定 OA（green，NTRS），但 NTRS API/归档路径对本机 IP 全部返回 403 Forbidden（含浏览器式请求头） | 仅题录；PDF **未下载** |
| REF-142 | Barczyk 2015（HAL hal-01256770） | HAL 部署 Anubis proof-of-work 反爬（返回 "Making sure you're not a bot!"），未绕过 | 仅题录；PDF **未下载** |
| REF-141 / 143 / 159 / 160 / 162 / 164 / 165 / 166 等 | 见 §1 表 | Unpaywall/Semantic Scholar 判定 `is_oa=false`（付费墙） | 仅题录，按约定不下载 |
| REF-146 | Ye 2023（IEEE ICUS） | 付费墙 | 仅题录 |

### 2.3 题录更正（合并注册表时请一并改）

1. **REF-145 年份**：bibliography 写 `Sensors, 26(1), 2025, 127`；PDF 首页为 `Sensors 2026, 26, 127` → 注册表按 **2026** 登记（目录名保留 `REF-145-Zhao-2025-...` 以维持既有命名，避免二次改号）。
2. **REF-138 版本**：UMN 机构库（conservancy.umn.edu）的 GREEN OA 链接实际指向同作者 **2016 博士论文（190 页）**；本库采用作者主页的 **期刊版 PDF（19 页，T-RO 30(1) 158–176）**，与题录一致。若需论文完整推导，可另取该学位论文，但**不作为 REF-138 的替换**。
3. `bibliography.md` §1 中 REF-136 标注为"仅题录（可开放获取）"——本轮已下载并转换，请改为"已入库"。

---

## 3. 合并建议（给 Lead）

1. 直接复制 §1.1–§1.6 六张表的数据行，按 REF-122→REF-169 顺序追加到 `参考资料注册表.md` 表格末尾（`## 说明` 之前）。
2. 同步把上文 §2.3 的三处题录更正应用到 `index/bibliography.md`（该文件属 Lead 写范围）。
3. `index/bibliography.md` §7 的内部资料（REF-170~REF-174）**不在本 pending 内**：它们是仓库内报告/代码，不属于本轮新增外部文献，建议由 Lead 按需单独登记。
4. 合并后建议删除本文件，避免与主注册表重复（`scripts/check-refs.py` 检测到同一 REF 同时出现在两处时只会给 warning，不会阻断）。
5. 校验入口：`py -3.12 docs/08-参考资料/scripts/check-refs.py`（0 error 通过）；重建索引：加 `--update-catalog`。
