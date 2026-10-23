# 导航算法专业文献地图（EKF / 组合导航 / 捷联惯导）

> 版本：2026-10-21
> 用途：为 `TandemVec_FCS` 导航子系统（15 状态 ESKF + 捷联惯导 + GNSS/气压/光流/静止辅助）
> 的优化升级提供可追溯的专业文献依据。
> 引用约定：正文用 `[REF-xxx]` 或 `作者+年份`；本表只登记**可核验**的题录。
> 状态：`已入库` = 本地有 PDF/Markdown；`仅题录` = 付费墙，仅登记元数据；`待补` = 尚未获取。

---

## 0. 阅读优先级（与本项目关系）

| 优先级 | 主题 | 代表文献 | 解决的问题 |
|---|---|---|---|
| P0 | 捷联惯导编排/圆锥划摇/误差模型 | 秦永元 2020 Ch7-9；Savage 1998a/b；Ignagni 1990 | 现有双子样、比力方程、误差状态线性化是否还有遗漏 |
| P0 | 卡尔曼滤波技术处理/容错组合导航 | 秦永元 2015 Ch3/Ch7/Ch8 | 量测序贯处理、故障检测/隔离、容错组合导航架构 |
| P0 | 航向可观测性/磁辅助/初始对准 | 秦永元 2020 §8.3/§9.9；Klein & Diamant 2018；Barczyk & Lynch 2011；Gebre-Egziabher 2006；Laidig & Seel 2023 | F2/F3 航向弱可观踢动的根治方向 |
| P1 | ESKF/误差状态定义/一致性 | Solá 2017；Markley 2003；Hesch 2014；Huang 2010；Barrau & Bonnabel 2017 | 误差定义、可观性约束、协方差一致性 |
| P1 | GNSS/INS 鲁棒与自适应 | Mohamed & Schwarz 1999；Mehra 1970；Liu 2019；Liu/Wang/Abel 2023；Brown 1992 | NIS 门控之外的慢漂/欺骗/多历元故障检测 |
| P1 | 约束/辅助/光流/气压 | Dissanayake 2001；Skog 2010；Grabe 2012/2013 | 巡航 NHC、ZUPT、光流尺度与偏置 |
| P2 | 非线性滤波/因子图 | Julier 2000；Arasaratnam 2009；Forster 2017 | 评估 UKF/CKF/因子图是否值得上机 |
| P3 | 时间同步/OOSM | Bar-Shalom 2002；Skog & Handel 2008 | 延迟回放与量测时间戳的边界 |

---

## 1. 专著与教材（Books）

| 编号 | 文献 | 出版信息 | 本地状态 | 本项目用途 |
|---|---|---|---|---|
| REF-120 | 秦永元. **惯性导航（第三版）** | 北京：科学出版社，2020.11. ISBN 978-7-03-066408-2 | 已入库：`books/REF-120-Qin-2020-inertial-navigation-3e/` | 捷联惯导编排、四元数/旋转矢量、圆锥/划摇/涡卷补偿、误差模型、自主对准与传递对准；本项目的**第一理论基准** |
| REF-121 | 秦永元，张洪钺，汪叔华. **卡尔曼滤波与组合导航原理（第3版）** | 西安：西北工业大学出版社，2015.6. ISBN 978-7-5612-4350-3 | 已入库：`books/REF-121-Qin-2015-kalman-filter-integrated-navigation-3e/` | 卡尔曼滤波技术处理、稳定性、广义滤波、**容错组合导航设计**、INS 对准与组合导航应用；本项目的**第二理论基准** |
| REF-122 | Titterton, D. H., & Weston, J. L. **Strapdown Inertial Navigation Technology** (2nd ed.) | IET, 2004. DOI: 10.1049/PBRA017E | 仅题录 | 英文 SINS 标准教材；与秦永元书互为交叉校验 |
| REF-123 | Groves, P. D. **Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems** (2nd ed.) | Artech House, 2013 | 仅题录 | 组合导航系统级框架、误差预算、辅助源建模 |
| REF-124 | Farrell, J. A. **Aided Navigation: GPS with High Rate Sensors** | McGraw-Hill, 2008 | 仅题录 | 高率 GNSS/INS 辅助导航、工程实现视角 |
| REF-125 | Savage, P. G. **Strapdown Analytics** (2nd ed.) | Strapdown Associates, 2007 | 仅题录 | 圆锥/划摇/涡卷的解析推导与系数表 |
| REF-126 | Maybeck, P. S. **Stochastic Models, Estimation, and Control, Vol. 1** | Academic Press, 1979 | 仅题录 | 卡尔曼滤波、故障检测与多模型估计的理论基础 |
| REF-127 | Bierman, G. J. **Factorization Methods for Discrete Sequential Estimation** | Academic Press, 1977 | 仅题录 | UD/平方根滤波；协方差数值稳定性 |
| REF-128 | Bar-Shalom, Y., Li, X.-R., & Kirubarajan, T. **Estimation with Applications to Tracking and Navigation** | Wiley, 2001 | 仅题录 | 序贯处理、OOSM、IMM、量测门控 |
| REF-129 | Grewal, M. S., & Andrews, A. P. **Kalman Filtering: Theory and Practice with MATLAB** (4th ed.) | Wiley, 2015 | 仅题录 | 工程 KF 实现与数值检查 |

---

## 2. 捷联惯导编排、圆锥/划摇补偿

| 编号 | 文献 | 出处 | 本地状态 | 要点/用途 |
|---|---|---|---|---|
| REF-130 | Savage, P. G. “Strapdown Inertial Navigation Integration Algorithm Design Part 1: Attitude Algorithms” | *J. Guidance, Control, and Dynamics*, 21(1), 1998, 19–28. DOI: 10.2514/2.4228 | 仅题录 | 多子样旋转矢量/圆锥补偿的经典设计；对照现有双子样 |
| REF-131 | Savage, P. G. “Strapdown Inertial Navigation Integration Algorithm Design Part 2: Velocity and Position Algorithms” | *J. Guidance, Control, and Dynamics*, 21(2), 1998, 208–221. DOI: 10.2514/2.4242 | 仅题录 | 划摇/涡卷补偿与速度/位置积分；对照现有双矢量增量 |
| REF-132 | Ignagni, M. B. “Optimal Strapdown Attitude Integration Algorithms” | *J. Guidance, Control, and Dynamics*, 13(2), 1990. DOI: 10.2514/3.20558 | 仅题录 | 圆锥最优系数；与 Savage 结果互相验证 |
| REF-133 | Ignagni, M. B. “Duality of Optimal Strapdown Sculling and Coning Compensation Algorithms” | *Navigation*, 45(2), 1998. DOI: 10.1002/j.2161-4296.1998.tb02373.x | 仅题录 | 划摇与圆锥算法对偶性；检查现有补偿系数 |
| REF-134 | Roscoe, K. M. “Equivalency Between Strapdown Inertial Navigation Coning and Sculling Integrals/Algorithms” | *J. Guidance, Control, and Dynamics*, 24(1), 2001. DOI: 10.2514/2.4718 | 仅题录 | 圆锥/划摇积分等价性；算法审查参考 |
| REF-135 | Markley, F. L. “Attitude Error Representations for Kalman Filtering” | *J. Guidance, Control, and Dynamics*, 26(2), 2003, 311–317. DOI: 10.2514/2.5048 | 仅题录 | 姿态误差参数化（乘性/加性、体轴/导航轴）；现有 δβ^b 定义对照 |
| REF-136 | Solá, J. “Quaternion Kinematics for the Error-State Kalman Filter” | arXiv:1711.02508, 2017 | 已入库：`papers/REF-136-Sola-2017-quaternion-eskf/` | ESKF 的权威教程；误差状态注入、四元数更新、扰动定义 |

---

## 3. 可观性、一致性与不变 EKF

| 编号 | 文献 | 出处 | 本地状态 | 要点/用途 |
|---|---|---|---|---|
| REF-137 | Goshen-Meskin, D., & Bar-Itzhack, I. Y. “Observability Analysis of Piece-Wise Constant Systems. I. Theory” | *IEEE Trans. Aerospace and Electronic Systems*, 28(4), 1992, 1056–1067. DOI: 10.1109/7.165367 | 仅题录 | 分段常值系统的可观性分析；INS/GNSS 不可观子空间 |
| REF-138 | Hesch, J. A., Kottas, D. G., Bowman, S. L., & Roumeliotis, S. I. “Consistency Analysis and Improvement of Vision-Aided Inertial Navigation” | *IEEE Trans. Robotics*, 30(1), 2014, 158–176. DOI: 10.1109/TRO.2013.2277549 | 已入库：`papers/REF-138-Hesch-2014-consistency-vision-aided-ins/` | OC-EKF/First-Estimates Jacobian；协方差一致性 |
| REF-139 | Huang, G. P., Mourikis, A. I., & Roumeliotis, S. I. “Observability-Based Rules for Designing Consistent EKF SLAM Estimators” | *Int. J. Robotics Research*, 29(5), 2010, 502–528. DOI: 10.1177/0278364909353640 | 仅题录 | 可观性约束的 EKF 设计规则；与航向弱可观同构 |
| REF-140 | Barrau, A., & Bonnabel, S. “The Invariant Extended Kalman Filter as a Stable Observer” | *IEEE Trans. Automatic Control*, 62(4), 2017, 1797–1812. DOI: 10.1109/TAC.2016.2594085 | 已入库：`papers/REF-140-Barrau-2017-inekf-stable-observer/` | InEKF 的稳定性/一致性理论；INS 群仿射结构 |
| REF-141 | Barczyk, M., & Lynch, A. F. “Invariant Extended Kalman Filter Design for a Magnetometer-plus-GPS Aided Inertial Navigation System” | *Proc. IEEE CDC*, 2011, 5389–5394. DOI: 10.1109/CDC.2011.6160733 | 仅题录 | **磁+GPS 辅助 INS 的 InEKF**；与本项目传感器组合高度相关 |
| REF-142 | Barczyk, M., Bonnabel, S., Deschaud, J.-E., & Goulette, F. “Invariant EKF Design for Scan Matching-Aided Localization” | *IEEE Trans. Control Systems Technology*, 23(6), 2015, 2440–2448. DOI: 10.1109/TCST.2015.2413933 | 仅题录 | InEKF 在辅助导航中的工程化 |
| REF-143 | Klein, I., & Diamant, R. “Observability Analysis of Heading Aided INS for a Maneuvering AUV” | *Navigation*, 65(1), 2018, 73–82. DOI: 10.1002/navi.222 | 仅题录 | **机动中航向辅助的可观性**；F2/F3 的直接理论参照 |
| REF-144 | Cui, B., Chen, W., Weng, D., Wei, X., Sun, Z., & Zhao, Y. “Observability-Constrained Resampling-Free Cubature Kalman Filter for GNSS/INS with Measurement Outliers” | *Remote Sensing*, 15(18), 2023, 4591. DOI: 10.3390/rs15184591 | 已入库：`papers/REF-144-Cui-2023-observability-ckf-gnss-ins/` | 可观性约束 + 鲁棒 CKF；GNSS/INS 野值场景 |
| REF-145 | Zhao, L., Chen, T., Yuan, P., Li, X., & Luo, Y. “MF-IEKF: A Multiplicative Federated Invariant Extended Kalman Filter for INS/GNSS” | *Sensors*, 26(1), 2026, 127. DOI: 10.3390/s26010127 | 已入库：`papers/REF-145-Zhao-2025-mf-iekf-federated/` | 联邦 + 不变 EKF；多源容错组合导航 |
| REF-146 | Ye, X., Xu, Y., & Yan, H. “GNSS/INS Navigation System Based on Lie Group Right Invariant Kalman Filter” | *Proc. IEEE ICUS*, 2023, 1329–1333. DOI: 10.1109/ICUS58632.2023.10318318 | 仅题录 | 右不变 KF 的 GNSS/INS 实现 |
| REF-147 | Ibrahim, A., Azouz, A., & Abosekeen, A. “A Land Vehicle’s INS/GNSS Integrated Navigation System Using Left Invariant Extended Kalman Filter” | *J. Physics: Conf. Series*, 2616(1), 2023, 012023. DOI: 10.1088/1742-6596/2616/1/012023 | 仅题录（开放获取） | 左不变 EKF 的陆用对照 |

---

## 4. GNSS/INS 组合、鲁棒/自适应、故障检测

| 编号 | 文献 | 出处 | 本地状态 | 要点/用途 |
|---|---|---|---|---|
| REF-148 | Mohamed, A. H., & Schwarz, K. P. “Adaptive Kalman Filtering for INS/GPS” | *Journal of Geodesy*, 73, 1999, 193–203. DOI: 10.1007/s001900050236 | 仅题录 | 新息/残差自适应 R/Q 的经典工程方法 |
| REF-149 | Mehra, R. K. “On the Identification of Variances and Adaptive Kalman Filtering” | *IEEE Trans. Automatic Control*, 15(2), 1970. DOI: 10.1109/TAC.1970.1099422 | 仅题录 | 自适应滤波的统计基础；避免“拍脑袋调 Q/R” |
| REF-150 | Blom, H. A. P., & Bar-Shalom, Y. “The Interacting Multiple Model Algorithm for Systems with Markovian Switching Coefficients” | *IEEE Trans. Automatic Control*, 33(8), 1988. DOI: 10.1109/9.1299 | 仅题录 | IMM；GNSS 质量模式/机动模式切换的候选 |
| REF-151 | Bar-Shalom, Y. “Update with Out-of-Sequence Measurements in Tracking: Exact Solution” | *IEEE Trans. Aerospace and Electronic Systems*, 38(3), 2002, 769–777. DOI: 10.1109/TAES.2002.1039398 | 仅题录 | OOSM 精确更新；对照现有延迟回放 |
| REF-152 | Brown, R. G. “A Baseline GPS RAIM Scheme and a Note on the Equivalence of Three RAIM Methods” | *Navigation*, 39(3), 1992, 301–316. DOI: 10.1002/j.2161-4296.1992.tb02278.x | 仅题录 | RAIM 基础；GNSS 故障检测/隔离 |
| REF-153 | Liu, Y., Li, S., Fu, Q., Liu, Z., & Zhou, Q. “Analysis of Kalman Filter Innovation-Based GNSS Spoofing Detection Method for INS/GNSS Integrated Navigation System” | *IEEE Sensors Journal*, 19(13), 2019, 5167–5178. DOI: 10.1109/JSEN.2019.2902178 | 仅题录 | 新息序列的欺骗检测；慢漂/多历元监测 |
| REF-154 | Liu, S., Wang, K., & Abel, D. “Robust State and Protection-Level Estimation within Tightly Coupled GNSS/INS Navigation System” | *GPS Solutions*, 27(3), 2023. DOI: 10.1007/s10291-023-01447-z | 已入库：`papers/REF-154-Liu-2023-robust-protection-level-gnss-ins/` | 鲁棒状态 + 保护级；完好性设计 |
| REF-155 | Carlson, N. A. “Federated Square Root Filter for Decentralized Parallel Processors” | *IEEE Trans. Aerospace and Electronic Systems*, 26(3), 1990, 517–525. DOI: 10.1109/7.106130 | 仅题录 | 联邦滤波；多源容错/故障隔离架构 |
| REF-156 | Julier, S. J., & Uhlmann, J. K. “A Non-Divergent Estimation Algorithm in the Presence of Unknown Correlations” | *Proc. American Control Conference*, 1997, 2369–2373. DOI: 10.1109/ACC.1997.609105 | 仅题录 | 协方差交叉；分散传感器融合 |
| REF-157 | Mahboub, V., & Mohammadi, D. “A Constrained Total Extended Kalman Filter for Integrated Navigation” | *Journal of Navigation*, 71(4), 2018, 971–988. DOI: 10.1017/S0373463318000012 | 仅题录 | 状态约束（位置/速度/姿态）在组合导航中的应用 |
| REF-158 | Wang, H., Deng, Z., Feng, B., Ma, H., & Xia, Y. “An Adaptive Kalman Filter Estimating Process Noise Covariance” | *Neurocomputing*, 223, 2017, 12–17. DOI: 10.1016/j.neucom.2016.10.026 | 仅题录 | 过程噪声在线估计；振动自适应 Q 的候选 |
| REF-159 | Skog, I., & Handel, P. “Effects of Time Synchronization Errors in GNSS-Aided INS” | *Proc. IEEE/ION PLANS*, 2008. DOI: 10.1109/PLANS.2008.4570010 | 仅题录 | 时间同步误差量化；延迟回放/时间戳设计 |

---

## 5. 辅助源、约束与传感器

| 编号 | 文献 | 出处 | 本地状态 | 要点/用途 |
|---|---|---|---|---|
| REF-160 | Skog, I., Handel, P., Nilsson, J. O., & Rantakokko, J. “Zero-Velocity Detection—An Algorithm Evaluation” | *IEEE Trans. Biomedical Engineering*, 57(11), 2010, 2657–2666. DOI: 10.1109/TBME.2010.2060723 | 仅题录 | ZUPT 检测器比较；静止检测阈值/迟滞设计 |
| REF-161 | Dissanayake, G., Sukkarieh, S., Nebot, E., & Durrant-Whyte, H. “The Aiding of a Low-Cost Strapdown Inertial Measurement Unit Using Vehicle Model Constraints for Land Vehicle Applications” | *IEEE Trans. Robotics and Automation*, 17(5), 2001, 731–747. DOI: 10.1109/70.964672 | 仅题录 | 非完整约束（NHC）；巡航零侧滑约束的理论参照 |
| REF-162 | Gebre-Egziabher, D., Elkaim, G. H., Powell, J. D., & Parkinson, B. W. “Calibration of Strapdown Magnetometers in Magnetic Field Domain” | *J. Aerospace Engineering*, 19(2), 2006, 87–102. DOI: 10.1061/(ASCE)0893-1321(2006)19:2(87) | 仅题录 | 椭球/硬铁/软铁标定；磁航向工程基础 |
| REF-163 | Laidig, D., & Seel, T. “VQF: Highly Accurate IMU Orientation Estimation with Bias Estimation and Magnetic Disturbance Rejection” | *Information Fusion*, 91, 2023, 187–204. DOI: 10.1016/j.inffus.2022.10.014 | 已入库：`papers/REF-163-Laidig-2023-vqf/` | 磁干扰检测/拒绝；航向辅助的门控范式 |
| REF-164 | Grabe, V., Bulthoff, H. H., & Giordano, P. R. “On-Board Velocity Estimation and Closed-Loop Control of a Quadrotor UAV Based on Optical Flow” | *Proc. IEEE ICRA*, 2012, 491–497. DOI: 10.1109/ICRA.2012.6225328 | 仅题录 | 光流测速/尺度；室内无 GNSS 速度辅助 |
| REF-165 | Grabe, V., Bulthoff, H. H., & Giordano, P. R. “A Comparison of Scale Estimation Schemes for a Quadrotor UAV Based on Optical Flow and IMU Measurements” | *Proc. IEEE/RSJ IROS*, 2013, 5193–5200. DOI: 10.1109/IROS.2013.6697107 | 仅题录 | 光流尺度在线估计；现有 flow/range 方案的升级参考 |
| REF-166 | El-Sheimy, N., Hou, H., & Niu, X. “Analysis and Modeling of Inertial Sensors Using Allan Variance” | *IEEE Trans. Instrumentation and Measurement*, 57(1), 2008, 140–149. DOI: 10.1109/TIM.2007.908635 | 仅题录 | IMU 噪声/零偏标定；Q 参数实机标定方法 |

---

## 6. 非线性滤波与优化（评估用，不作为首选上机方案）

| 编号 | 文献 | 出处 | 本地状态 | 要点/用途 |
|---|---|---|---|---|
| REF-167 | Julier, S., Uhlmann, J., & Durrant-Whyte, H. F. “A New Method for the Nonlinear Transformation of Means and Covariances in Filters and Estimators” | *IEEE Trans. Automatic Control*, 45(3), 2000, 477–482. DOI: 10.1109/9.847726 | 仅题录 | UKF 原始论文；与 EKF 的精度/算力权衡 |
| REF-168 | Arasaratnam, I., & Haykin, S. “Cubature Kalman Filters” | *IEEE Trans. Automatic Control*, 54(6), 2009, 1254–1269. DOI: 10.1109/TAC.2009.2019800 | 仅题录 | CKF；高维非线性滤波的替代方案 |
| REF-169 | Forster, C., Carlone, L., Dellaert, F., & Scaramuzza, D. “On-Manifold Preintegration for Real-Time Visual–Inertial Odometry” | *IEEE Trans. Robotics*, 33(1), 2017, 1–21. DOI: 10.1109/TRO.2016.2597321 | 已入库：`papers/REF-169-Forster-2017-on-manifold-preintegration/` | 流形预积分/因子图；机载实时性评估的参照 |

---

## 7. 与本项目直接相关的既有内部资料

| 编号 | 资料 | 位置 | 用途 |
|---|---|---|---|
| REF-170 | 固件 EKF 组合导航理论与算法分析设计报告（LaTeX，80 页级） | `TandemVec-EKF-Report/main.pdf`、`main.tex`、`ch/*.tex` | 现有算法的逐行推导与实现对照；本书单的“基线” |
| REF-171 | 固件 EKF 导航全面审查报告 | `TandemVec-EKF-Report/审查报告-固件EKF导航全面审查.md` | H1/H2、MED/LOW 问题清单与已修复项 |
| REF-172 | 算法与参数优化研究报告 | `TandemVec-EKF-Report/优化研究-算法与参数优化研究报告.md` | F1/F2/F3 机制、参数扫描、变体 A/B、已实施修改 |
| REF-173 | 高保真固件闭环仿真报告 | `TandemVec-EKF-Report/高保真固件闭环仿真报告.md` | 闭环验证方法与场景 |
| REF-174 | 导航算法代码 | `TandemVec_FCS/lib/navigation-main/src/ekf_15_state.h`、`TandemVec_FCS/src/navigation_task.cpp` | 差距分析的唯一代码事实源 |

---

## 8. 备注

- 付费墙文献只登记题录，PDF 不入库；开放获取文献由 `ref-librarian` 负责下载、MinerU 转换、质量审计与登记。
- 中文教材页码与 PDF 页偏移：`REF-120` 印刷页 + 8 = PDF 页；`REF-121` 为纯扫描件，按 OCR 后的章节标题定位，不依赖固定偏移。
- 所有“文献结论”与“本项目推断”必须分开标注；不得把文献中的平台/传感器结论直接外推到本机。
