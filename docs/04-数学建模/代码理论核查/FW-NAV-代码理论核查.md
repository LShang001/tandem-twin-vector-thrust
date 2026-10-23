# 代码理论核查记录 — fw-nav

> 核查日期：2026-10-23 ｜ 执行：fw-nav（Agent Teams task-3）｜ 规范：[GOV-004](../../00-项目治理/GOV-004-代码注释与理论对照规范.md)
> 代码基线：`6784221`（2026-09-11）；工作区另含 2026-10 的 P0-0/P0-1/P0-3/P1-5 未提交改动（EKF 旋转效应、ZUPT 门限、GNSS 降级/子集更新、振动自适应等），本记录以**工作区实际代码**为核查对象。
> 理论来源：REF-120（秦永元《惯性导航》第三版）§7.2/§7.4/§7.5/§9.2/§9.6/§9.8/§9.9；REF-121（《卡尔曼滤波与组合导航原理》第三版）§2.2.1/§2.2.4/§7.3.1/§8.2.3；MOD-002 §2/§3/§5；notes/01、04、06；GOV-004 附录 A。
> 方法：逐文件读代码 → 对照公式与式号 → 实际打开 REF-120/REF-121 章节核对式号与原文表述 → 核对 notes/01/04/06 的页码与结论 → 补 L0/L1/L2 与五段式 → `check-code-comments.py` 复检 → 宿主 g++ 回归。

---

## 1 范围与覆盖率

| 文件 | 行数 | L0 文件头 | L1 符号文档 | `@ref` 引用 | 备注 |
|------|------|-----------|-------------|-------------|------|
| `src/navigation_task.cpp` | 2412 | 有 | 有（主状态机/双矢量/光流/降级/重锚/三个 KF + 5 段五段式） | 28 | R07 理论关键文件 |
| `src/navigation_task.h` | 156 | 有 | 有（全部导出函数） | 4 | 修正“15 状态=四元数4+…”等 3 处腐烂注释 |
| `src/sensor_imu.cpp` | 547 | 有 | 有（init/read/handle） | 3 | 轴映射红线 |
| `src/sensor_imu.h` | 75 | 有 | 有 | 2 | 修正“写 AHRS_Packet”错误描述 |
| `src/math_utils.h` | 280 | 有 | 部分（原有工具函数注释） | 2 | R07 理论关键文件 |
| `include/QuaternionMath.h` | 540 | 有 | 有（全部公开函数） | 4 | 修正 `theta=-asin(R13)` 等价说明 |
| `include/Quat4f.h` | 54 | 有 | 有 | 2 | 原 `//` 头改为 `/** */` L0 |
| `include/VerticalKF.h` | 389 | 有 | 有（类 + 成员） | 2 | 3 状态垂直 KF |
| `include/VerticalKF_2State.h` | 330 | 有 | 有 | 2 | 修正 25=5σ 的错误注释 |
| `include/HorizontalKF.h` | 265 | 有 | 有（类 + 成员） | 2 | 速度观测型水平 KF |
| `include/ComplementaryFilter.h` | 97 | 有 | 有（原有类注释） | 1 | |
| `include/IMU_AutoCalibrator.h` | 260 | 有 | 有 | 2 | 方差检测校准 |
| `include/GeoDisplacement.h` | 134 | 有 | 有（原有函数注释） | 2 | ENU/NED 混用风险已注明 |
| `include/ins_altitude_reference.h` | 64 | 有 | 有 | 1 | |
| `include/ins_bias_jump_monitor.h` | 208 | 有 | 有 | 1 | |
| `include/ins_bias_temp_observer.h` | 115 | 有 | 有 | 1 | |
| `include/ins_bias_temp_table.h` | 121 | 有 | 有 | 1 | |
| `include/ins_gnss_delay_auto.h` | 284 | 有 | 有 | 1 | |
| `include/ins_gnss_dynamic_weight.h` | 137 | 有 | 有 | 1 | |
| `include/ins_gnss_epoch_timing.h` | 122 | 有 | 有（原有函数语义） | 1 | |
| `include/ins_gnss_pps_latency.h` | 147 | 有 | 有 | 1 | |
| `include/ins_inertial_reference.h` | 66 | 有 | 有 | 2 | |
| `include/ins_static_aid_profile.h` | 271 | 有 | 有（原有调度说明） | 2 | |
| `include/ins_static_detector.h` | 280 | 有 | 有 | 2 | |
| `include/ins_vibration_adaptive.h` | 121 | 有 | 有（原 `//` 块保留为说明） | 1 | |

合计：25 文件 / 7475 行（核查前 6887 行，本轮净增 588 行注释与文档）；L0 覆盖 25/25；`@ref` 共 71 处（本轮新增 62 处）。
本轮 diff 中新增注释/文档约 680 行，无新增/删除可执行语句（唯一非注释变更行是 `VerticalKF_2State` 某常量行的行尾注释文字，代码值不变）。

---

## 2 核对矩阵

结论枚举：`一致` / `等价变换后一致` / `近似（可接受）` / `偏差（已登记）` / `不一致（待修）`。

### 2.1 姿态更新与姿态数学（A）

| ID | 模块/文件 | 理论要素或公式 | 文献出处 | 代码位置（符号名） | 结论 | 证据 | 处置 |
|----|-----------|----------------|----------|--------------------|------|------|------|
| A-01 | `QuaternionMath.h` | Hamilton 乘积 `q1⊗q2`、共轭 `q*=w−xi−yj−zk` | REF-120 §9.2.1 (9.2.2)(9.2.5) p.253 | `quaternionMultiply()`、`quaternionConjugate()` | 一致 | 分量展开逐项对照；`Quat4f.h` float 版逐字同构 | 补 `@theory`/`@ref` |
| A-02 | `QuaternionMath.h` | 四元数规范化 `q / norm(q)`（模长过小回退单位四元数） | REF-120 §9.2.5 p.264 | `normalizeQuaternion()` | 一致 | 阈值 1e-9 为工程保护，不影响归一化数学 | 补 `@ref` |
| A-03 | `QuaternionMath.h` | 3-2-1 欧拉角提取 `theta=-asin(R13)` | MOD-002 §3；REF-120 §9.2.2 p.256 | `quaternionToEuler()` | 等价变换后一致 | `R13=2(xz+wy) ⇒ −asin(R13)=asin(2(wy−xz))`，与实现 `sin_pitch=2(w*y−x*z)` 逐字一致；`sin_pitch` 钳位 [−1,1] | 已补 `@formula`/@note，标注**不可改符号**（红线） |
| A-04 | `QuaternionMath.h` | 欧拉角→四元数 Z-Y′-X″ | MOD-002 §3 | `eulerToQuaternion()` | 一致 | 半角公式四式与 ZYX 顺序一致；输出保范 | 补 `@ref` |
| A-05 | `QuaternionMath.h` | 坐标变换 `v_F2=q⊗v_F1⊗q*` | REF-120 §9.2.2 p.256 | `rotateVector()` | 一致 | 纯四元数旋转定义 | 已有说明 |
| A-06 | `QuaternionMath.h` | 最短对齐旋转（半角公式 + 反向特判） | REF-120 §9.2.1 p.253（构造式） | `computeAlignQuaternion()` | 近似（可接受） | 夹角→π 时旋转轴不唯一，代码用垂直轴兜底；对航向无影响 | 保留 |
| A-07 | `QuaternionMath.h` | 纯倾斜目标姿态（机体 −z 对齐推力方向、无绕 z 旋转） | 项目控制构造（无教材式） | `constructTiltTargetQuaternion()` | 近似（可接受） | 四元数 `(1−d_D, d_E, −d_N, 0)` 归一化；水平分量超限时归一化 | 登记 P2-7（缺项目推导文档引用） |
| A-08 | `navigation_task.cpp` | 姿态右乘更新 `q_{k+1}=q_k⊗Exp(ω^b Δt)` | REF-120 §9.2 (9.2.14) p.268；MOD-002 §3.4 | `TimeUpdateTwoSample()` / `TimeUpdate()` 调用点 | 一致（库内实现由 fw-ekf 核查） | 本区域只负责双子样拆分与调用；库内每步保范由 fw-ekf 记录 | 补五段式与 `@ref` |
| A-09 | `navigation_task.cpp` | 双子样速度增量：旋转项 `½Δθ×Δv`、划摇项 `(2/3)(Δθ₁×Δv₂+Δv₁×Δθ₂)` | REF-120 §9.6 (9.6.9)(9.6.17)(9.6.19) p.298–299、(9.6.25) p.301 | `TimeUpdateTwoSample()` 调用点 | 一致（P0-0 已修“旋转效应双计”） | `test_host/test_ekf_15state.cpp` T7：残差 0.000125005 vs 期望 0.000124999 | 补五段式 |
| A-10 | `sensor_imu.cpp` | 四元数链式修正 `q_FRD_from_NED=q_FRD_from_FLU⊗q_FLU_from_NWU⊗q_NWU_from_NED` | REF-120 §9.2.2 p.256；MOD-002 §3.3 | `readIMUData()` 步骤 8 | 等价变换后一致 | 两次 180° 旋转（FLU→FRD、NWU→NED）；轴映射与实飞存档版逐字一致 | 补 `@ref`/红线说明 |
| A-11 | `sensor_imu.cpp` | Madgwick 梯度下降姿态滤波（非秦书算法） | 工程实现（Madgwick 2010） | `madgwick.updateIMU()` | 近似（可接受） | 只作后台 AHRS/无 GNSS 姿态量测源；yaw 无磁时纯陀螺积分 | 文件头 `@theory` 注明“非秦书算法” |
| A-12 | `math_utils.h` | 平面旋转 `[x_b,y_b]=R(−ψ)[x_n,y_n]` | MOD-002 §2 | `nedToBody()`、`bodyToNed()` | 一致 | 与 DCM 3-2-1 的 ψ 分量一致 | 补 `@ref` |

### 2.2 初始对准与航向源（B）

| ID | 模块/文件 | 理论要素或公式 | 文献出处 | 代码位置（符号名） | 结论 | 证据 | 处置 |
|----|-----------|----------------|----------|--------------------|------|------|------|
| B-01 | `navigation_task.cpp` | 伪磁初值 `mag_ut=(30,0,40)` 的语义 | REF-120 §9.9.1 p.323（静基座 `f≈−g`）；notes/04 §1.3/§2 | `handleNavigationSystem()` 初始化前的 `mag_ut` 赋值 | 偏差（已登记） | 该矢量不随姿态/磁北变化，只提供“确定性”航向；TiltCompass 解算结果非真北 | 代码已注明“伪磁/非罗经对准”；报告中明确语义 |
| B-02 | `navigation_task.cpp` | TiltCompass 双矢量定姿（第二把尺子） | REF-120 §9.9.1 p.323；notes/06 §2 | `nav_ekf.Initialize(..., mag_ut, ...)` | 等价变换后一致 | 秦书用 `g^b`+`ω_ie^b`；本机用 `g^b`+伪磁，MEMS 陀螺无法分辨 `ω_ie cosL≈13.25°/h`（长沙 28.2°N，notes/04 §1） | 保留，注明替换基准 |
| B-03 | `navigation_task.cpp` | 双矢量航向 `ψ=atan2(v_E,v_N)−atan2(v_y^b,v_x^b)` | REF-120 §9.8.14 p.320–322；notes/06 §2/§4 | `handleDualVectorYawFusion()` | 近似（可接受） | 门控：v≥1 m/s、`\|ω_z\|`≤0.35 rad/s、尺度比 ±35%；静止跳过 | 补 L1 + 五段式 + `@deviation` |
| B-04 | `navigation_task.cpp` | `P0(8,8)=0.2 rad` 的语义 | notes/06 §1.2；AGENTS.md“EKF 三个机制性风险” | `init_heading_err_std_rad(0.2f)` | 一致（语义=伪磁确定性 1σ，非真实精度） | 2026-08-13 仿真：π→2.5–6.7 km 闭环崩溃；0.2 rad 后姿态误差 5–17° | 保留并注释 |
| B-05 | `navigation_task.cpp` | 航向弱可观：`K(8,v)∝f_x·P(8,8)/Q_a`；consider 门控 | REF-121 §8.2.3 p.291；notes/06 §6 | `set_gnss_consider_heading()` 调用点 | 一致 | P1-5：`\|pitch\|/\|roll\|>0.5 rad` 且 `\|v\|>3 m/s` 时置位，仅本次更新生效 | 补 `@ref` |
| B-06 | `navigation_task.cpp` | 静止时航向不可观、双矢量观测退化 | notes/06 §1/§4 | `handleDualVectorYawFusion()` 的 `static_det.confirmed_static` 早退 | 一致 | 悬停/匀速时航迹角退化，§9.8.14 的 φ_U 不进入速度观测 | 已有 L2 |

### 2.3 速度/位置编排、零速修正与静止检测（C）

| ID | 模块/文件 | 理论要素或公式 | 文献出处 | 代码位置（符号名） | 结论 | 证据 | 处置 |
|----|-----------|----------------|----------|--------------------|------|------|------|
| C-01 | `navigation_task.cpp` | 比力方程 `V̇ⁿ=Cⁿ_b f^b−(2ω_ie+ω_en)×Vⁿ+gⁿ` | REF-120 §7.4 (7.4.6) p.189 | `TimeUpdate()`/`TimeUpdateTwoSample()` 调用 | 一致（库内实现） | 库内 `PropagateStateFromIncrements()`（fw-ekf 记录） | 补 `@ref` |
| C-02 | `navigation_task.cpp` | 位置编排 `L̇=V_N/(R_m+h)`、`ḣ=−V_D`；高度通道不稳定 | REF-120 §7.2/§8.1、§7.5 (7.5.2) p.190–191 | 输出桥 `relative_down`、气压高度更新 | 一致 | 高度必有气压反馈；纯惯导高度发散见 §7.5 | 补 `@ref` |
| C-03 | `ins_static_detector.h` | 静止判据 `\|f^b\|≈g、\|ω^b\|≈0、短时变化小` + 迟滞 | REF-120 §9.9.3 p.336（静基座假设）；REF-121 §8.2.3 p.291 | `Icm45686StaticDetectorUpdate()` | 一致 | 阈值 9.5–10.1 m/s²、0.01 rad/s、Δa 0.5、Δω 0.05；进 20 帧/出 5 帧 | 补 L0/L1 |
| C-04 | `navigation_task.cpp` | 静止误判的物理边界：恒速平飞与静止在 IMU 上不可区分 | REF-121 §8.2.3 p.291（须外来信息）；§7.3.1 p.255 | `handleNavigationSystem()` 静止检测段 | 一致 | 仿真 120 s 漂 330 m（修复前）；GNSS 速度>0.8 强制退出后消失 | 已有 L2，补 `@ref` |
| C-05 | `navigation_task.cpp` | ZUPT：`z=0−v̂`，三轴速度观测 | REF-121 §8.2.3 p.291；REF-120 §9.9.3 p.336 | `MeasurementUpdateVelocityDetailed()` 调用点 | 一致 | `test_host/test_ekf_15state.cpp` T4/T5；ZUPT NIS 16.3 修正 | 补五段式 |
| C-06 | `ins_static_aid_profile.h` | ZUPT 残差门限与自适应噪声（>1.5 m/s 跳过、分段放大到 6×） | REF-121 §7.3.1 p.255（FDI 门限） | `Icm45686AdaptZuptNoiseForResidual()` | 近似（可接受） | 分段系数为工程折中，无教材对应；残差大时跳过有物理依据 | 补 `@ref` + `@note` |
| C-07 | `ins_static_aid_profile.h` | 外部参照门控：无参照 >0.75 m/s 拒绝、3 s 封底 0.30、30 s 停用 | REF-121 §8.2.3 p.291 | `Icm45686UpdateZuptExtRefGuard()`、`Icm45686ZuptExternalRefAllowsFusion()` | 一致 | 仿真 SC9b：末端 51.64 m→11.21 m；`Icm45686StaticAidPhasesCanCollide()` 用中国剩余定理防饿死 | 补 L0；结构化说明 |
| C-08 | `ins_static_aid_profile.h` | 三档强度与分频调度（弱约束→稳态→长静止） | 工程调度（无教材式） | `Icm45686SelectStaticAidProfile()`、`Icm45686SelectStaticAidAction()` | 近似（可接受） | 档位噪声 0.18/0.10/0.03 m/s（水平），置信度/帧数与协方差健康共同门控 | 补 `@note` |
| C-09 | `navigation_task.cpp` | GNSS 6 维联合更新 + NIS/子集 FDI | REF-121 §2.2.1 p.33–34、§7.3.1 p.255–259 | `MeasurementUpdateDetailed()` 调用点 | 一致（库内） | T9–T11 覆盖速度坏/位置坏/双坏；χ²(3) 25.90/16.27 | 补五段式 |
| C-10 | `navigation_task.cpp` | 降级 GNSS：位置 R=1e4 等效关闭、速度 R 用 spd_acc | REF-121 §2.2.1 p.33 | `applyDowngradedGnssUpdate()` | 一致 | 位置量测用 epoch 真实 LLA（2026-08-12 修复自差语义） | 补 L1 |
| C-11 | `navigation_task.cpp` | 首次真实 GNSS 硬重锚（位置/速度重置、恢复起飞点） | 工程（无教材式） | `reanchorNavigationToGnss()` | 一致 | 量测年龄 ≤0.15 s；姿态/零偏保持连续 | 补 L1 |

### 2.4 辅助源：气压、光流、磁、零偏、延时、振动（D）

| ID | 模块/文件 | 理论要素或公式 | 文献出处 | 代码位置（符号名） | 结论 | 证据 | 处置 |
|----|-----------|----------------|----------|--------------------|------|------|------|
| D-01 | `ins_altitude_reference.h` | 绝对高度 = 起飞点绝对高度 + 相对高度（向上为正） | REF-120 §7.2（大地/高度基准）；MOD-002 §5.1 | `InsComputeAbsoluteBaroAltitude()`、`InsRelativeHeightUpFromNedDown()` | 一致 | `test_host/test_altitude_reference.cpp` PASS | 补 L0/L1 |
| D-02 | `navigation_task.cpp` | 气压高度标量量测（25 Hz 降采样，H 仅位置 D 分量） | REF-121 §2.2.1 p.33；REF-120 §7.5 p.190–191 | `MeasurementUpdateBaroAltitudeDetailed()` 调用点 | 一致 | NIS 门限 16≈χ²(1) 99.99%；`baro_ekf_*` 诊断 | 已有 L2 |
| D-03 | `navigation_task.cpp` | 光流速度 `v^n=C_b^n·[v_x^b,v_y^b,0]`，垂直通道 R=1e3 关闭 | REF-120 §7.4 (7.4.6) p.189；REF-121 §8.2.3 p.291 | `handleIndoorOpticalFlowVelocityFusion()` | 一致 | 用 `quat2dcm(q)^T` 全姿态旋转；z 取 0 的误差 ≈v_z·sin(tilt) | 补 L1 + 五段式 |
| D-04 | `navigation_task.cpp` | 光流速度/测距双新鲜度通道：测距不得刷新 `last_update_ms` | AGENTS.md 2026-08-21 踩坑记录 | `updateEstimatedVerticalVelocity()`、`handleHorizontalEstimation()`、`handleIndoorOpticalFlowVelocityFusion()` | 一致 | 测距分支用 `last_range_update_ms`；速度通道用 `last_update_ms` | L2 已注明（原注释保留并强化） |
| D-05 | `navigation_task.cpp` | 光流速度失锁重锚（连续 50 次拒绝 → 重置速度 + 抬 P） | AGENTS.md 2026-08-21 | `ResetVelocityToExternal()`、`s_consecutive_rejects` | 一致 | `test_host/test_ekf_aiding_velocity.cpp` 16 断言（fw-tests） | 补 `@deviation` |
| D-06 | `sensor_imu.cpp` / `lib/MagCal` | 椭球标定（9 参数、≥30 样本、残差 RMS≤0.15、Flash 持久化） | notes/04 §1.3/§4.2（Gebre-Egziabher 2006） | `initMagnetometer()`、`magcal::storeLoad()` | 一致 | 质量门对航向偏松（notes/04 建议 ≤0.03），当前未馈入 EKF | 登记 P1-3（不在本区域改） |
| D-07 | `navigation_task.cpp` | 加计/陀螺零偏观测（Gauss-Markov、静止陀螺分轴权重） | REF-121 §2.2.1 p.33 | `MeasurementUpdateStaticGyroDetailed()` 调用点 | 一致 | Z 轴噪声 5e-4、X/Y 1.0；`throttlePercent<3%` 才融合 | 已有 L2 |
| D-08 | `ins_bias_jump_monitor.h`/`ins_bias_temp_*.h` | 零偏阶跃/慢漂/温漂监控与温度表（只诊断） | REF-121 §7.3.1 p.255（FDI） | `InsBiasJumpMonitor`、`InsBiasTempObserver`、`InsBiasTempTable` | 一致 | 宿主回归 bjm/bto/btt 全部 rc=0 | 补 L0 |
| D-09 | `navigation_task.cpp` | GNSS 量测年龄/延时回放（默认 15 ms 编译常量） | REF-121 §8.2.3 p.291 | `BFS_NAVIGATION_GNSS_DELAY_S`、`InsGnssMapEpochToLocalTime()` | 一致 | 无 PPS 时 15 ms 是常量；实机需标定 | 补 `@ref` |
| D-10 | `ins_gnss_delay_auto.h` | 无 PPS 延时批处理估计（9 参数最小二乘 + 0–150 ms 网格） | REF-121 §8.2.3 p.291 | `InsGnssDelayAutoEstimator::TryEstimate()` | 一致 | 机动窗口估计 45 ms/真实 50 ms；匀速不可观 | 补 L0 |
| D-11 | `ins_gnss_pps_latency.h` | PPS 链路延时（中位数 + MAD） | REF-121 §2.2.1 p.33（统计量建模） | `InsGnssEstimatePpsLatency()` | 一致 | 25/50 ms 回归 8/8；硬件捕获未接入 | 补 L0 |
| D-12 | `ins_gnss_epoch_timing.h` | GPS 周内时→本地 micros 映射与年龄（模运算、半周折叠） | REF-121 §8.2.3 p.291 | `InsGnssMapEpochToLocalTime()`、`InsGnssEpochAgeSeconds()` | 一致 | micros 回绕/负映射回归用例 | 补 L0 |
| D-13 | `ins_vibration_adaptive.h` | 振动 RMS → EKF 噪声建议（0.5 Hz 高通 + 0.5 s EMA） | REF-121 §2.2.1 p.33 | `InsVibrationEstimator::Update()` | 近似（可接受） | Policy C：Q 仅允许 0.15–0.25，延时/失锁/低精度回退 0.25 | 补 `@note` |
| D-14 | `VerticalKF.h`/`VerticalKF_2State.h` | KF 五方程 + Joseph 形式 + 残差门限 | REF-121 §2.2.1 (2.2.4a–e) p.33–34、§2.2.4 p.42–43；REF-120 §7.5 p.190–191 | `predict()`、`update()` | 一致 | `test_host/test_vertical_kf.cpp` PASS；门限按标准化残差平方比较 | L0 已补；修正 25=5σ 注释 |
| D-15 | `HorizontalKF.h` | 速度观测型 KF（H=[0,1]） | 同上 | `HorizontalKF_1Axis_VelMeas` | 近似（可接受） | 位置无绝对观测→随机游走；只用于短时/后备 | L0 已补；登记 P1-1 |
| D-16 | `IMU_AutoCalibrator.h` | 静基座零偏标定（方差检测 + 3 s + 4000 样本） | REF-120 §9.9.1 p.323（静基座基准） | `IMU_AutoCalibrator::update()` | 一致 | 方差阈值 2e-5、陀螺 0.02 rad/s；解锁即中止 | 补 L0 |
| D-17 | `GeoDisplacement.h` | WGS84 LLA→ECEF→ENU | REF-120 §7.2.1/§7.2.2 p.174–178；MOD-002 §1 | `lla_to_ecef()`、`calculate_ne_displacement()` | 一致 | 椭球常量与公式逐项对照；ENU 与 NED 轴向差异已 `@note` | L0 已补 |

### 2.5 AGENTS.md 重点坑位复核（E）

| ID | 坑位 | 代码证据 | 结论 |
|----|------|----------|------|
| E-01 | 光流速度/测距双新鲜度通道，测距帧不得刷新 `last_update_ms` | `updateEstimatedVerticalVelocity()` 用 `last_range_update_ms`；速度消费者用 `last_update_ms` | 一致，注释已强化 |
| E-02 | static gyro 在 `throttlePercent≥3%` 跳过融合 | `handleNavigationSystem()` 静止辅助的 `StaticGyro` 分支 | 一致（`throttlePercent < 3.0f` 条件） |
| E-03 | 航向弱可观、P0 航向与 consider 门控 | `init_heading_err_std_rad(0.2f)`、`set_gnss_consider_heading()` | 一致，按相位启用 |
| E-04 | ZUPT 门限/盲区（1.5→0.8 m/s、外部参照门控） | `handleNavigationSystem()` GNSS 静止退出段、`ins_static_aid_profile.h` | 一致 |
| E-05 | 四元数分量顺序 `[w,x,y,z]`（不是 xyzw） | `Quaternion` 结构体首成员 `w`；`Quat4f` 注释 | 一致，L0 已写明 |
| E-06 | 传感器轴映射 `bX=-sZ、bY=+sX、bZ=-sY` | `readIMUData()` 步骤 5 | 一致，标为 `@redline` |
| E-07 | `DISABLE_GNSS_DELAY_REPLAY=1` 会拒绝全部 GNSS（纯惯导测试宏） | 本区域未直接消费该宏（在 `ekf_15_state.h`） | 记录在报告，不在本区域改 |

---

## 3 发现清单

| 严重度 | 位置（符号） | 问题 | 影响 | 建议 |
|--------|--------------|------|------|------|
| P0 | — | 本轮未发现符号/量纲/定理层面的错误（姿态 `theta=-asin(R13)`、双子样系数、ZUPT/门限均与文献或已修记录一致） | — | — |
| P1-1 | `handleHorizontalEstimation()` / `HorizontalKF` | 光流速度只用航向 ψ 做二维旋转（`bodyToNed`），忽略俯仰/滚转；EKF 路径用全姿态四元数 | 倾斜时速度投影有 cos(tilt) 尺度误差；只影响 `fused_*` 遥测/后备，不影响控制（控制消费 EKF） | 后续改为全姿态旋转，或保持现状但明确标注适用范围（已在 L1 `@deviation` 标注） |
| P1-2 | `handleNavigationSystem()` 的 `mag_ut` | 磁力计未馈入（`kMagFeedMadgwick=false`），航向初始化为伪磁常量；无绝对航向物理源 | 悬停/过渡航向只能靠双矢量（有速度时）与短时陀螺；与 notes/04 结论一致，属设计缺口 | 按 notes/04 G0–G3 验收后启用磁矢量更新；本轮只登记 |
| P1-3 | `initMagnetometer()` / `lib/MagCal` | 椭球标定质量门 residual RMS≤0.15 对航向偏松（notes/04 建议 ≤0.03） | 启用磁航向时方向误差可达约 8.6° | 属 `lib/MagCal`（非本区域），提交 fw-ekf/磁专项处理 |
| P1-4 | `handleVerticalEstimation()` | 激光高度分支 `if (0)` 永久禁用，垂直 KF 只剩气压高度更新 | 气压失效时无备份高度源（EKF 侧仍有 GNSS 高度） | 明确恢复条件或删除死代码；本轮只加注说明 |
| P2-1 | `VerticalKF_2State` 构造函数 | 门限 25.0 的注释写“3-sigma (3*3=9)”，实际 25=5² | 误读门限强度 | 已修正为“5σ 的平方（非 3σ）” |
| P2-2 | `sensor_imu.cpp` 磁偏角段 | 旧常量 −4.133° 与 notes/04（长沙约 −2.2°，待 WMM2025 核实）不一致，且该段已被注释 | 读者可能误用旧值 | 已改写为不含具体数值的说明 |
| P2-3 | `navigation_task.cpp` 多处 | 注释中残留 `本文件 1116 行 / 1183 行 / 694 行 / 1118-1123 行` 行号引用，违反 GOV-004 §6.2 | 行号随重构漂移 | 已改为符号名引用 |
| P2-4 | `navigation_task.h` | 旧注释：EKF“15 状态=四元数(4)+速度+位置+零偏”，实际误差状态 15 维、四元数单独维护；静止检测“50 帧”、垂直 KF “激光优先”、水平 KF“覆盖 relative_*”均已过期 | 误导读者 | 已按实现修正（见 §1 备注） |
| P2-5 | `sensor_imu.h` | 旧注释称 `readIMUData()` 打包到 `AHRS_Packet`，实际只写 `backup_ahrs_*` | 误导读者 | 已修正 |
| P2-6 | `GeoDisplacement.h` | 输出为 ENU（东-北-天），与飞控 NED 不同；函数名 `calculate_ne_displacement` 易误解为 NED | 换轴错误风险 | 已在 L0 `@note` 明确“north=North、east=East、down=−Up” |
| P2-7 | `constructTiltTargetQuaternion()` | 工程构造无项目推导文档引用（MOD/CTL/THY） | 可追溯性弱 | 后续在 CTL-001/THY 补一节或加 `@deviation` |
| P2-8 | `ins_gnss_dynamic_weight.h` | 原“使用方式”代码块被 R04 判为注释代码 | 规范不合规 | 已改写为散文说明（语义不变） |
| P2-9 | notes/01 引用 (9.9.43c) | REF-120 MinerU 版仅有 43a/43b/43d（OCR 缺项） | 引用式号无法逐字定位 | 本轮代码引用避开 43c；notes 由作者维护 |

---

## 4 未决问题

1. **航向绝对基准**：磁力计未接入、伪磁初值非真北；双矢量航向只在有速度时有效。航向弱可观的根治（磁矢量更新、速度通道重锚、可观测性门控）仍为开放项（与 notes/04、AGENTS.md 2026-08-13 一致）。
2. **水平 KF 的旋转近似**（P1-1）：是否统一到全姿态旋转，需评估 CPU 与遥测口径影响；本轮不改行为。
3. **激光高度分支**（P1-4）：`if (0)` 是临时禁用还是永久策略，代码无 owner/日期，需项目确认后删除或恢复。
4. **GNSS 延时标定**：15 ms 为编译期常量；PPS 硬件捕获未接入，无 PPS 自动估计只在机动段可用且默认有条件应用。
5. **磁偏角**：代码旧值 −4.133° 与 notes/04 的 −2.2° 不一致，需以 WMM2025/地面标定定值。
6. **REF-121 页码**：本轮沿用的 `§8.2.3 p.291`、`§7.3.1 p.255` 来自 AGENTS.md/既有核查记录；MinerU 版无印刷页码，未能独立复核页码，只核对了章节与原文表述（§2.2.1 p.33–34、§2.2.4 p.42–43 页码来自 GOV-004 附录 A）。
7. **`test/ekf_host_regression.cpp`** 在 HEAD 基线即挂 3743 断言（project 场景），非本区域改动引入，仍待单独定位（fw-tests 负责）。

---

## 5 门禁与测试

### 5.1 注释规范门禁

```text
核查前：py -3.12 tools/check-code-comments.py --area fw-nav --check
        25 文件 | 平均分 69.4 | error 33 | warning 21（R01=25 R04=14 R06=2 R07=8 R09=5）→ 退出码 1

核查后：py -3.12 tools/check-code-comments.py --area fw-nav --check
        25 文件 | 平均分 100.0 | error 0 | warning 0 | 全绿 25 → 退出码 0
```

### 5.2 宿主回归（g++ 13.2.0，`-std=c++17`，`-ITandemVec_FCS/include -ITandemVec_FCS/lib/eigen/src`）

命令模板（仓库根目录；MinGW g++ 是原生 Windows 程序，`-o` 必须用 `C:/...` 或相对路径，MSYS 的 `/c/...` 会报 cannot open output file）：

```bash
cd TandemVec_FCS
BIN="C:/Users/<user>/AppData/Local/Temp/tv_fcs_nav_bin"; mkdir -p "$BIN"
g++ -std=c++17 -O1 -Iinclude -Ilib/eigen/src test_host/test_static_detector.cpp -o "$BIN/sd.exe" && "$BIN/sd.exe"
# test_mag_cal 追加 -Ilib/MagCal；test_vertical_kf 追加 -Itest_host/stub
```

| 测试 | 结果 | 结论 |
|------|------|------|
| `test_gnss_dynamic_weight.cpp` | rc=0，全部通过 | PASS |
| `test_static_aid_profile.cpp` | rc=0，全部通过 | PASS |
| `test_static_detector.cpp` | rc=0，全部通过 | PASS |
| `test_altitude_reference.cpp` | rc=0，[PASS] | PASS |
| `test_gnss_epoch_timing.cpp` | rc=0，通过 | PASS |
| `test_gnss_pps_latency.cpp` | rc=0，8 passed / 0 failed | PASS |
| `test_gnss_delay_auto.cpp` | rc=0，[PASS] | PASS |
| `test_ins_vibration_adaptive.cpp` | rc=0，[PASS] | PASS |
| `test_bias_jump_monitor.cpp` | rc=0 | PASS |
| `test_bias_temp_observer.cpp` | rc=0 | PASS |
| `test_bias_temp_table.cpp` | rc=0 | PASS |
| `test_mag_cal.cpp` | rc=0，全部通过 | PASS |
| `test_vertical_kf.cpp` | rc=0，全部通过 | PASS |

合计 13/13 通过（0 失败）。

### 5.3 固件构建与语法检查

- `pio run`（TandemVec_FCS）：**SUCCESS**（TandemVec_FCS env 67.8 s、test env 63.2 s，RAM 35.7%、Flash 16.6%），在本轮导航注释修改后执行；编译警告均为基线既有项（如 `sensor_imu.cpp` 的 `gyrFrd[3]` double→float 收窄、`communication.cpp` 未用变量），非本轮引入。
- 独立头文件语法检查：`QuaternionMath.h`、`Quat4f.h`、`VerticalKF*.h`、`HorizontalKF.h`、`ComplementaryFilter.h`、`ins_*.h` 共 18/20 通过 `g++ -fsyntax-only`；`GeoDisplacement.h` 需 `-D_USE_MATH_DEFINES`（Windows 的 `M_PI` 定义）、`IMU_AutoCalibrator.h` 需真实 `Arduino.h` 的 `millis()`（`test_host/stub/Arduino.h` 明确不提供时间基准），二者均为既有环境限制，非本轮改动。

### 5.4 行为不变自查

- `git diff` 逐文件扫描：本轮触及的 25 个文件中，新增可执行语句 0 行、删除可执行语句 0 行；除 `VerticalKF_2State.h` 某常量行的行尾注释文字（值未变）外，全部新增行均为注释/文档。
- 工作区中 `navigation_task.cpp`、`sensor_imu.cpp`、`ins_static_aid_profile.h` 另含任务开始前已存在的 2026-10 功能改动（P0-3/P1-5、振动、温度等），非本轮引入，已在代码基线中注明。任务开始时记下的基线 `git diff --stat`：`navigation_task.cpp` +240/−7、`sensor_imu.cpp` +9/−0、`ins_static_aid_profile.h` +76/−0，与本轮扫描到的“非注释 diff 行”逐一对应；本轮对这三文件的净变更均为注释。
