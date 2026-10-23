# 代码理论核查记录 — fw-ekf（15 状态 ESKF 导航库）

> 核查日期：2026-10-23 ｜ 执行：fw-ekf（Agent） ｜ 规范：[GOV-004](../../00-项目治理/GOV-004-代码注释与理论对照规范.md)
> 代码基线：`TandemVec_FCS/lib/navigation-main/src/**` 工作区状态（任务开始时已含未提交的
> P0-1 子集 FDI / 航向 consider / 光流重锚逻辑改动，见 §3 P2-2；本轮不改变任何代码 token）
> 理论来源：REF-120（秦永元《惯性导航》第3版）§7.2/§7.4/§9.2–9.8；REF-121（秦永元《卡尔曼滤波与
> 组合导航原理》第3版）§2.2/§3.3/§6.2.3/§7.3.1/§8.2.3；MOD-002；notes/01、notes/02、notes/07
> 方法：逐文件读代码 → 回书核对公式文本 → 以 notes 的印刷页锚点定位页码 → 五段式补注释 →
> 宿主回归 + 语法检查 + lint 复检 → 去注释后与基线逐行比对确认无代码变化

---

## 1 范围与覆盖率

| 文件 | 行数（改后） | L0 文件头 | L1 符号文档 | @ref 处数 | @formula | 备注 |
|------|------|-----------|-------------|-----------|----------|------|
| `ekf_15_state.h` | 4584 | 有 | 有（传播内核/各量测入口/回放/稳定化） | 45 | 25 | 本轮重点；状态与误差方向约定单列 |
| `transforms.h` | 516 | 有 | 有（姿态互转/LLA-ECEF-NED/Olson） | 17 | 12 | 删除 5 处“公式样式伪代码”注释块 |
| `earth_model.h` | 290 | 有 | 有（曲率/重力/LLA 率/自转/运输率） | 15 | 9 | Somigliana 偏差登记 |
| `aid_source_status.h` | 298 | 有 | 部分（观测源/结果结构/各记账接口） | 2 | 0 | 与 EKF 解耦，理论点少 |
| `navigation.h` | 116 | 有 | 聚合头（模块职责/宏/调用序列） | 4 | 0 | 重写文件头，消除示例代码块 |
| `tilt_compass.h` | 84 | 有 | 有（调平/定向公式与奇异） | 3 | 1 | — |
| `constants.h` | 84 | 有 | 有（两组常量分组说明） | 4 | 0 | 两组取值来源区分 |
| `utils.h` | 105 | 有 | 有（Wrap/Skew） | 5 | 3 | 归一化边界说明 |

合计：8 文件 / 6077 行；L0 覆盖 8/8；新增 `@ref` 95 处、`@formula` 50 处。
基线 lint：13 error / 20 warning（平均分 55.0）→ 核查后 0 error / 0 warning（平均分 100.0）。

---

## 2 核对矩阵

结论枚举：`一致` / `等价变换后一致` / `近似（可接受）` / `偏差（已登记）` / `不一致（待修）`。
页码为 REF-120/121 印刷页；页码与式号均以本地书稿 OCR 全文回书核对，页码锚点取自 notes/01、notes/02。

### 2.1 状态定义、误差方向与姿态数学

| ID | 模块/文件 | 理论要素或公式 | 文献出处 | 代码位置（符号名） | 结论 | 证据 | 处置 |
|----|-----------|----------------|----------|--------------------|------|------|------|
| A-01 | `ekf_15_state.h` | 15 维误差状态顺序 [δp, δv, δβ, δb_a, δb_g]；δ = 真值−估计 | MOD-002 §3；GOV-004 §6.3 | 类前“状态与误差定义”注释、`InjectErrorState()` | 一致 | `test_ekf_15state` T1/T4/T6 | 补注释 |
| A-02 | `ekf_15_state.h` | 四元数右乘误差 `q_true = q_est⊗Exp(δβ)`；指数映射 | REF-120 §9.2.4 p.262 | `BuildDeltaQuatFromRotationVector()`、`InjectErrorState()` | 一致 | T4/T6；ω=2 rad/s 下与一阶近似差 O(δβ³) | 补注释 |
| A-03 | `ekf_15_state.h` | 秦书导航系失准角 `φ = C^n_b·δβ`（一阶等价） | REF-120 §9.8.2 (9.8.12)(9.8.13) | 类前注释、F 矩阵注释 | 等价变换后一致 | 推导见 §2.4 E-1 | 补注释 |
| A-04 | `ekf_15_state.h` | δ 方向取反：秦书 δV=计算−真值，固件 δv=真值−估计 | GOV-004 §6.3 | 文件头 `@note`、F(V,β)/F(β,V) 注释 | 等价变换后一致 | 推导见 §2.4 E-2 | 补注释 |
| A-05 | `tilt_compass.h` | 重力调平 `pitch=asin(a_x)`、`roll=asin(−a_y/cosθ)`；磁矢量倾斜补偿 atan2 | REF-120 §9.9.1 p.323–324 | `TiltCompass()` | 一致 | T1（水平静止 roll/pitch<1e-3 rad） | 补注释 |
| A-06 | `transforms.h` | `theta = −asin(R13)`；四元数↔DCM 等价 | MOD-002 §3；REF-120 §9.2.2 p.256–258 | `quat2angle()`、`dcm2angle()`、`quat2dcm()` | 一致 | T1（欧拉角基准） | 补注释 |

### 2.2 捷联惯导名义传播（速度/姿态/位置）

| ID | 模块/文件 | 理论要素或公式 | 文献出处 | 代码位置（符号名） | 结论 | 证据 | 处置 |
|----|-----------|----------------|----------|--------------------|------|------|------|
| A-07 | `ekf_15_state.h` | 双子样圆锥补偿 `Φ=Δθ₁+Δθ₂+(2/3)Δθ₁×Δθ₂` | REF-120 §9.3.3 (9.3.30) p.274 | `PropagateStateFromIncrements()` 的 `coning_delta_theta_rad` | 一致 | notes/01 §2 复核：10 Hz 划桨 5 ms 系数残差 0.12% | 五段式注释 |
| A-08 | `ekf_15_state.h` | 双子样划摇 `ΔV_scul=(2/3)[ΔV₁×Δθ₂−ΔV₂×Δθ₁]` | REF-120 §9.6.2 (9.6.25) p.301；最优系数 §9.6.3 p.303 | `sculling_delta_v_body_mps` | 等价变换后一致 | 恒等式 `a×b=−b×a`，两式恒等 | 五段式注释写明 |
| A-09 | `ekf_15_state.h` | 旋转效应 `ΔV_rot=½Δθ×ΔV` 必须配合**区间起始姿态** | REF-120 §9.6.1 (9.6.17) p.298、(9.6.19) p.299 | `delta_v_ned`（中点姿态 `t_b2ned_mid`） | 一致（历史 P0-0 已修） | T7：0.000250008 → 0.000125005（期望 0.000124999） | 五段式写明“当前实现与历史缺陷” |
| A-10 | `ekf_15_state.h` | 速度更新 `V_m=V_{m−1}+C^n_b(m−1)ΔV_sf+ΔV_g/cor` | REF-120 §9.6.1 (9.6.9) p.298 | `delta_v_ned`、`ins_ned_vel_mps_` | 等价变换后一致（改用中点姿态，二阶精度） | T2/T3/T7 | 五段式注释 |
| A-11 | `ekf_15_state.h` | 比力方程 `V̇^n=C^n_b f^b−(2ω_ie+ω_en)×V^n+g^n` | REF-120 §7.4 (7.4.6) p.189 | `coriolis_accel_mid_ned`、`gravity_ned_mid` | 一致 | T2（静止速度保持 <0.02 m/s） | 补注释 |
| A-12 | `ekf_15_state.h` | 姿态推进 `q_k=Exp(−(ω_ie+ω_en)Δt)⊗q_{k−1}⊗Exp(Φ)` | REF-120 §9.2.4 p.262；§9.8.2 (9.8.13) | `quat_`、`nav_frame_delta_theta_rad` | 等价变换后一致 | T2（静止 100 步姿态保持） | 五段式注释 |
| A-13 | `ekf_15_state.h` | 位置积分 `L̇=V_N/(Rm+h)`、`λ̇=V_E/((Rn+h)cosL)`、`ḣ=−V_D` | REF-120 §9.7 p.304–313；§9.8.1 (9.8.5a–c) p.317–322 | `ComputeLlaRateFromTerms()` | 一致 | T4（GNSS 位置残差收敛） | 补注释 |
| A-14 | `ekf_15_state.h` | 地球自转率 `ω_ie^n=[ω_e cosφ,0,−ω_e sinφ]`；运输率 `ω_en^n` | REF-120 §7.4 (7.4.6) p.189 | `ComputeEarthModelTerms()`、`ComputeNavRateFromTerms()` | 一致 | T1（初始零偏扣除含 ω_ie+ω_en） | 补注释 |

### 2.3 误差状态 F 矩阵、离散化与过程噪声

| ID | 模块/文件 | 理论要素或公式 | 文献出处 | 代码位置（符号名） | 结论 | 证据 | 处置 |
|----|-----------|----------------|----------|--------------------|------|------|------|
| A-15 | `ekf_15_state.h` | 速度误差方程 `δV̇=−φ^n×f^n+…`；F(V,β)=−C^n_b[f^b]× | REF-120 §9.8.1 (9.8.3) p.317–322 | `fs_.block(3,6)` | 等价变换后一致 | 推导见 §2.4 E-2/E-3 | 五段式注释 |
| A-16 | `ekf_15_state.h` | 姿态误差方程 `φ̇=φ×ω_in^n+δω_in^n−C^n_b ε^b`；F(β,V) | REF-120 §9.8.2 (9.8.13) | `fs_.block(6,3)`（M 为 NED 下 ∂ω_en/∂v） | 等价变换后一致 | 推导见 §2.4 E-4；与 KF-GINS M 定义一致 | 五段式注释（含 M 的完整矩阵） |
| A-17 | `ekf_15_state.h` | F(V,V) 哥氏/传输率 8 项；F(P,P) 5 项 | REF-120 §9.8.1 (9.8.3)(9.8.4) p.317–322 | `fs_` 的 `F(V,V)`/`F(P,P)` 块 | 一致 | 逐项与 (9.8.4) 分量式对照 | 五段式注释 |
| A-18 | `ekf_15_state.h` | F(V,b_a)=−C^n_b；F(V,D)=2g/R̄；F(β,β)=−[ω^b]×；F(β,b_g)=−I₃；Markov 零偏块 | REF-120 §9.8.1/§9.8.2；REF-121 §2.2.1 p.33 | `fs_.block(3,9)`、`fs_(5,2)`、`fs_.block(6,6)`、`fs_.block(6,12)`、`RefreshStateTransitionStaticBlocks()` | 一致 | T2/T3（P 增长有界）+ 量纲核对 | 五段式注释 |
| A-19 | `ekf_15_state.h` | 离散化 `Φ≈I+FΔt+½(FΔt)²`（快路径一阶） | REF-121 §2.2.1 (2.2.4d) p.33 | `phi_` | 近似（可接受） | 5 ms 下二阶项量级 O((FΔt)²)；一阶路径仅 bring-up | `@approx` 标注 |
| A-20 | `ekf_15_state.h` | 过程噪声 `Qd=∫ΦG Rw G^T Φ^T dτ`（Simpson 三点） | REF-121 §2.2.1 (2.2.4d) p.33 | `q_`、`Icm45686StructuredContinuousQc()` | 一致 | T2/T3（200 步 P 有界/对称） | 五段式注释 |
| A-21 | `ekf_15_state.h` | Q 的结构化等价：`C^n_b(σ²I)C^b_n=σ²I` | 矩阵正交性（自证） | 快速路径 4 个对角块、`Icm45686FirstOrderCovarianceDeltaFromBlocks()` | 等价变换后一致 | 与稠密 `Gs·Rw·Gs^T` 数学等价 | 注释写明前提（非对角相关项出现须回退） |

### 2.4 等价变换推导（单列）

**E-1 φ 与 δβ 的关系**：秦书 `Ĉ^n_b=(I−[φ]×)C^n_b`（Ĉ 为计算值）；固件 `C_true=C_est·Exp(δβ)`（右乘）。一阶展开
`C_est=C_true(I−[δβ]×)`，与 `(I−[φ]×)C_true=C_true(I−[C^b_nφ]×)` 比较得 `δβ=C^b_nφ`，即 `φ=C^n_b·δβ`。
**E-2 δ 方向取反**：秦书速度误差 `δV=计算−真值`，固件 `δv=真值−估计`，故 `δv=−δV`；姿态误差 φ 由 E-1 与 δβ 同向。
速度通道：`δV̇=−φ×f^n+…` ⇒ `δv̇=+φ×f^n=−C^n_b[f^b]×δβ`，对应 `F(V,β)=−C^n_b[f^b]×`（代码实现）。
**E-3 F(V,β) 的坐标旋转**：`−[f^n]×=−[C^n_b f^b]×=−C^n_b[f^b]×C^b_n`，左乘/右乘 φ 与 δβ 的转换后与代码一致。
**E-4 F(β,V) 的符号与矩阵**：`φ̇⊃δω_in^n`，其中 `δω_en^n=∂ω_en/∂v·δV`，NED 下
`∂ω_en/∂v=[[0,1/(Rn+h),0],[−1/(Rm+h),0,0],[0,−tanφ/(Rn+h),0]]`（代码 `transport_jacobian`）。
由 `δβ̇=C^b_nφ̇+Ċ^b_nφ` 得 `δβ̇⊃C^b_n·∂ω_en/∂v·δV`；再用 E-2 的 `δv=−δV` 取反，得
`F(β,V)=−C^b_n·M`，其中 `M=∂ω_en/∂v`——与代码 `fs_.block<3,3>(6,3) = -t_b2ned_mid.transpose() * transport_jacobian` 一致。
**E-5 划摇两式恒等**：`(2/3)[ΔV₁×Δθ₂−ΔV₂×Δθ₁] = (2/3)(Δθ₁×ΔV₂+ΔV₁×Δθ₂)`（叉积反交换）。
**E-6 位置误差→LLA 比例尺**：`δL=δp_N/(Rm+h)`、`δλ=δp_E/((Rn+h)cosL)`、`δh=−δp_D`，代码 `InjectErrorState()` 用更新前纬度/高度统一比例尺。

### 2.5 量测更新、门控与鲁棒性

| ID | 模块/文件 | 理论要素或公式 | 文献出处 | 代码位置（符号名） | 结论 | 证据 | 处置 |
|----|-----------|----------------|----------|--------------------|------|------|------|
| A-22 | `ekf_15_state.h` | KF 五方程 (2.2.4a–e) | REF-121 §2.2.1 p.33–34 | `MeasurementUpdateDetailed()` | 一致 | T4/T9–T11 | 五段式注释 |
| A-23 | `ekf_15_state.h` | Joseph 形式 (2.2.4e)；简单式数值退化风险 | REF-121 §2.2.4 p.42–43 | 全部量测更新（`p_=(I−KH)P(I−KH)^T+KRK^T`） | 一致 | T2/T3（P 对称/正定） | 补注释 |
| A-24 | `ekf_15_state.h` | NIS `r^T S^-1 r ~ χ²(m)`；门限由误警率 α 定 | REF-121 §7.3.1 p.255–259（表 7.3.1） | 门限常量区、各 `Apply*UpdateDetailed()` | 一致（门限偏保守） | T6/T9–T11；ZUPT 门限 1000→16.3 | 注释列全部门限与分位数 |
| A-25 | `ekf_15_state.h` | 序贯/子集更新等价性 (3.3.2a–c) | REF-121 §3.3 p.84–86 | `ApplyGnssSubsetFallback()` | 等价变换后一致（3+3 子集是序贯特例） | T9–T11（速度坏/位置坏/双坏） | 补注释 |
| A-26 | `ekf_15_state.h` | 重力方向观测 `H=+[h]×`（体系误差）；扣加计零偏后取方向 | REF-120 §9.8.2 (9.8.13) | `ApplyGravityUpdateDetailed()` | 一致 | `test_ekf_bias_static`（静止重力约束） | 补注释（含历史漏乘 `t_b2ned` 修复说明） |
| A-27 | `ekf_15_state.h` | 航向 H = 真航向轴在体系的分量 = `C^b_n` 第三行 | REF-120 §9.8.2 (9.8.13)；本项目推导 | `ApplyYawUpdateDetailed()` 的 `h_yaw` | 一致 | T8（consider 对比） | 注释写明 2026-08-12 修复 |
| A-28 | `ekf_15_state.h` | bias-only 静止陀螺更新（不改姿态） | REF-121 §8.2.3 p.291；§7.3.1 p.255–259 | `ApplyStaticGyroUpdateDetailed()` | 一致（工程决策，见注释） | `test_ekf_bias_static` | 补注释（含 throttle 门控） |
| A-29 | `ekf_15_state.h` | 气压高度与 GNSS-D 同构的标量更新 | REF-121 (2.2.4b–e) p.33–34；REF-120 (9.8.5c) p.321 | `ApplyBaroAltitudeUpdateDetailed()` | 一致 | `test_ekf_bias_static` | 补注释（绝对椭球高前提） |
| A-30 | `ekf_15_state.h` | GNSS 延迟回放：量测作用于其对应时刻 | REF-121 §2.2.1 p.33；AGENTS.md 踩坑 | `GetDelayedSnapshotIndexByAge()`、`RestoreSnapshot()`、`RepropagateFromDelayedSnapshot()` | 一致（工程实现，非教材内容） | `test_ekf_delay_highspeed.cpp`（延迟 50 ms RMSE 1.458→0.164 m） | 补注释 |
| A-31 | `ekf_15_state.h` | 失联膨胀、重捕获低增益融合与连续一致 GNSS 硬重锚 | REF-121 §3.5 p.87–93（P 过度自信）；REF-121 §7.3.1 p.255–259（残差门控） | `GrowGnssOutageCovariance()`、`BuildConservativeGnssReacquisitionNoise()`、`TryResetAfterPersistentImplausibleGnss()` | 近似（工程参数；硬重锚为恢复路径，非教材算法） | T13/T13b；注释标注上限 4 倍与 3 帧一致性条件 | 补注释 |
| A-32 | `ekf_15_state.h` | 光流/持续测速辅助：残差门放宽、NIS 主门控 | REF-121 §7.3.1 p.255–259 | `MeasurementUpdateAidingVelocityDetailed()`、`ResetVelocityToExternal()` | 一致（工程决策） | `test_ekf_aiding_velocity` 16/16 | 补注释 |
| A-33 | `ekf_15_state.h` | P0-1 航向 consider：K 的航向行清零（Schmidt 近似） | REF-121 §8.2.3 p.291 | `set_gnss_consider_heading()`、`k_.row(8).setZero()` | 近似（可接受，按相位启用） | T8（1.724°→0.000°） | 补注释 |

### 2.6 地球模型、坐标变换与工具

| ID | 模块/文件 | 理论要素或公式 | 文献出处 | 代码位置（符号名） | 结论 | 证据 | 处置 |
|----|-----------|----------------|----------|--------------------|------|------|------|
| A-34 | `earth_model.h` | 正常重力 Somigliana (7.2.17) + 高度修正 (7.2.18) | REF-120 §7.2.4 (7.2.17)(7.2.18) p.178 | `normalgravity_mps2()`、`ComputeEarthModelTerms()` | 偏差（已登记） | 常数项差 8.7e-6、sin²2φ 项差 ~1e-6，合成 ~1e-5 m/s² | `@deviation` + 本表 |
| A-35 | `earth_model.h` | 卯酉圈/子午圈曲率半径 | REF-120 §7.2.1 表 7.2.1 | `earthrad_transverse_m()`、`earthrad_meridonal_m()` | 一致 | T2/T3（经 Rn/Rm 积分的位置有界） | 补注释 |
| A-36 | `earth_model.h` | 地球自转率/运输率/LLA 变化率 | REF-120 §7.4 (7.4.6) p.189；§9.7 p.304–313 | `earthrate()`、`navrate()`、`llarate()` | 一致 | 与 EKF 内部缓存实现同式 | 补注释 |
| A-37 | `transforms.h` | WGS-84 LLA↔ECEF（解析式） | REF-120 §7.2.1 表 7.2.1 | `lla2ecef()` | 一致 | T4（位置换算链路） | 补注释 |
| A-38 | `transforms.h` | ECEF→LLA Olson 一次修正 | REF-120 §7.2.1/§7.2.2（椭球几何） | `ecef2lla()` | 近似（可接受） | 亚米级标称精度；|h| 数十 km 时退化 | `@deviation` 标注 |
| A-39 | `transforms.h` | ECEF↔NED 切平面旋转阵 R | REF-120 §9.8.1 (9.8.1) p.317–318 | `ecef2ned()`、`ned2ecef()` | 一致 | R 正交，逆=转置 | 补注释 |
| A-40 | `utils.h` | 角度归一化与斜对称矩阵 | REF-121 §2.2.1 p.33；REF-120 §9.2.3 (9.2.47) p.261–262 | `WrapTo2Pi()`、`WrapToPi()`、`Skew()` | 一致（WrapToPi 边界见 P1-2） | T2（新息/姿态） | 补注释 |
| A-41 | `constants.h` | WGS-84 椭球/自转/引力常量（两套取值） | REF-120 §7.2.1 表 7.2.1；ICD-GPS-200 | `SEMI_MAJOR_AXIS_LENGTH_M` 等 | 一致 | 与 (7.2.17) 使用的 a/e² 一致 | 补注释 |
| A-42 | `tilt_compass.h` | 无磁时伪磁向量的确定性航向语义 | notes/06（P0 航向修复） | `TiltCompass()` + `Initialize()` | 一致（工程语义） | T1；AGENTS.md P0 航向 π→0.2 rad | 注释写明 |

---

## 3 发现清单

### P0（影响飞行安全或定理成立条件）

本轮**未发现 P0**。特别说明：核对中重点复核了“F(β,V) 与 F(V,β) 的符号是否与秦书相反”这一
易误判点，按 §2.4 E-1～E-4 的等价变换换算后与 (9.8.3)/(9.8.13) 逐块一致，属**等价变换**而非缺陷。

### P1（精度/鲁棒性/可维护性，不直接危及安全）

| 严重度 | 位置（符号） | 问题 | 影响 | 建议 |
|--------|--------------|------|------|------|
| P1-1 | `transforms.h::dcm2quat()` | `w=½√(1+tr C)` + `/(4w)` 在 tr(C)→−1（180° 旋转）时数值退化 | 当前库内无调用点；启用后会得坏四元数 | 改为“选 trace 最大主分量”分支法并补 180° 用例；已加 `@deviation` |
| P1-2 | `utils.h::WrapToPi()` | 只做一次 2π 归位，仅保证 (−3π,3π) 输入落回主值 | 外部传大角度会返回非主值；EKF 内部另用 `WrapAngleRad()`（fmod）不受影响 | 已在 `@note` 写明边界；若外部启用需改为 fmod 版 |
| P1-3 | `earth_model.h::normalgravity_mps2()`、`ComputeEarthModelTerms()` | 级数式与 (7.2.17) Somigliana 闭式差 ~1e-5 m/s² | 远小于 MEMS 加计零偏（~1e-2），高度通道无系统偏差 | 保留（`@deviation` 登记）；如换高精度 IMU 再评估 |
| P1-4 | `ekf_15_state.h` 延迟回放 + `BFS_NAVIGATION_EMBEDDED_DISABLE_GNSS_DELAY_REPLAY` | 该宏置 1 时缓冲缩为 1 格，带年龄的 GNSS 更新会被 `require_delayed_snapshot` 路径全部拒绝 | 误当“禁用回放”使用会导致纯惯导 | 文件头 `@note` 已写明；建议在 platformio.ini 注释同步 |
| P1-5 | `ekf_15_state.h::ApplyVelocityUpdateDetailed()` | ZUPT 残差门使用 EKF 自身速度（自指判据），无外部参照 | 长时间失联期可能自洽错误 | 已引 REF-121 §8.2.3 p.291；根治需外部速度参照（上层门控，非本文件改动） |

### P2（注释/引用/工具链）

| 严重度 | 位置（符号） | 问题 | 影响 | 建议 |
|--------|--------------|------|------|------|
| P2-1 | `tools/check-code-comments.py::CODE_HINT` | 任务期间该文件的第三分支 `^\s*[\w.\[\]]+\s*=\s*[^=]` 被外部删除（文件变为未跟踪状态），R04 判定范围缩小 | 注释掉的赋值型代码不再被检出；本区域仍全绿，但全仓 R04 基线会漂移 | 由 Lead 确认该工具改动的归属，恢复第三分支后重跑全仓 R04 |
| P2-2 | `ekf_15_state.h`、`aid_source_status.h` | 任务开始时工作区已含未提交的 P0-1 子集 FDI / 航向 consider / 光流重锚逻辑改动；任务期间另有并发改动给 `delta_theta_rad`/`delta_v_body_mps` 加了两处 `const` | 以 `git HEAD` 比对会看到代码差异，易误判为本轮引入 | 本轮以“任务开始时备份”+ 去注释比对确认：除上述两处并发 `const` 外无任何代码 token 变化（见 §5） |
| P2-3 | 注释与核查记录 | 历史 notes 使用行号引用（如 `:2275-2280`） | 重构后失效 | 本轮新增注释一律用符号名；核查记录同 |
| P2-4 | `transforms.h`、`earth_model.h`、`ekf_15_state.h` | 基线 12 处“公式样式伪代码”注释块（`Rn = a/√(...)` 等）触发 R04 误报 | lint warning | 已改写为 Doxygen `@formula` 形式，warning 清零；同时保留原工程说明 |

---

## 4 未决问题

1. **φ=C^n_b·δβ 的直接数值回归缺失**：当前靠推导 + T7/T8 间接覆盖。建议增加一个 host 断言：
   构造已知小角度姿态误差，检查 `F(β,V)` 与 `F(V,β)` 作用方向与手算一致（可作为 fw-tests 的后续任务）。
2. **磁偏角**：`TiltCompass()`/`MeasurementUpdateYaw()` 的航向量测以磁北为基准，磁偏角未建模；
   与 GNSS 航向/双矢量航向混用时会产生常值偏置（不在本轮范围）。
3. **GNSS 延迟常量的静态假设**：`BFS_NAVIGATION_GNSS_DELAY_S=0.015` 是编译期估计；PPS 标定与
   在线估计（`ins_gnss_delay_auto.h`）的接入点不在本文件，需在导航任务层核对。
4. **`dcm2quat()` 与 `WrapToPi()` 的边界用例**：当前无调用点/无测试；若后续启用，需补回归。

---

## 5 门禁与测试

### 5.1 GOV-004 lint

```bash
py -3.12 tools/check-code-comments.py --area fw-ekf --check
```

- 核查前：8 文件，平均分 55.0，**error 13 / warning 20**（R01=8、R04=12、R07=5、R09=8）
- 核查后：8 文件，平均分 **100.0**，**error 0 / warning 0**，退出码 **0**

### 5.2 语法检查（编译阻塞回归）

```bash
cd TandemVec_FCS
g++ -std=c++17 -fsyntax-only -D_USE_MATH_DEFINES -DEKF_HOST_REGRESSION \
    -Iinclude -Ilib/eigen/src -Isrc -Ilib/navigation-main/src -Ilib/units/src \
    test_host/test_ekf_15state.cpp
# 同样命令对 test_ekf_aiding_velocity.cpp / test_ekf_bias_static.cpp
```

结果：三者均 **无输出、退出码 0**。
> 任务中期曾出现 `stray '@' in program`：`MeasurementUpdateDetailed()` 的 `@return` 行写作
> `subset_*/NIS`，其中 `*/` 提前闭合了 Doxygen 块。已改为 `subset_* 与 NIS`，并对全区域
> 检索了注释内 `*/`/`/*` 片段（仅此一处异常）。

### 5.3 宿主回归

```bash
cd TandemVec_FCS
g++ -std=c++17 -O2 -DEKF_HOST_REGRESSION -D_USE_MATH_DEFINES \
    -Iinclude -Ilib/eigen/src -Isrc -Ilib/navigation-main/src -Ilib/units/src \
    test_host/test_ekf_15state.cpp lib/units/src/convang.cpp \
    lib/units/src/convangacc.cpp lib/units/src/convangvel.cpp -o /tmp/tv_bin2/ekf_15state.exe
./ekf_15state.exe
```

| 测试 | 结果 |
|------|------|
| `test_ekf_15state.cpp` | **36 通过 / 0 失败**（T1–T11） |
| `test_ekf_aiding_velocity.cpp` | **16 pass / 0 fail** |
| `test_ekf_bias_static.cpp` | 退出码 0（seed=1 mode=1 输出正常） |

### 5.4 代码 token 不变性

方法：自写 C++ 注释剥离器（处理 `//`、`/* */`、字符串/字符字面量），比较“去注释后的代码行序列”。

- 6 个文件（`constants.h`、`earth_model.h`、`navigation.h`、`tilt_compass.h`、`transforms.h`、`utils.h`）：
  **与 git HEAD 去注释后完全一致**。
- `ekf_15_state.h`：**与任务开始时备份去注释后 2962 行代码完全一致**，仅差异为并发改动新增的两处
  `const`（`delta_theta_rad`、`delta_v_body_mps`），非本 Agent 修改。
- `aid_source_status.h`：`git diff` 中唯一的代码差异是任务前已有的 P0-1 字段
  （`subset_position`/`subset_velocity`/`nis_pos3`/`nis_vel3`），本 Agent 只改文件头与
  `MarkAvailable()` 的注释块。

结论：本轮为**纯注释/文档改动**，未修改任何逻辑、宏值、阈值、状态维数或矩阵索引。

---

## 6 重新生成命令

```bash
# 门禁
py -3.12 tools/check-code-comments.py --area fw-ekf --check

# 宿主回归（全部）
cd TandemVec_FCS && BIN_DIR=/c/Users/<user>/AppData/Local/Temp/tv_bin bash test_host/run_all.sh
```
