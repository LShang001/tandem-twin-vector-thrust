# 代码理论核查记录 — fw-control

> 核查日期：2026-10-23 ｜ 执行：fw-control（Agent Teams task-4）｜ 规范：[GOV-004](../../00-项目治理/GOV-004-代码注释与理论对照规范.md)
> 代码基线：`6784221`（2026-09-11）；工作区另含任务开始前已存在的未提交改动（`ActuatorFaultMonitor.h` 新文件、`flight_control.cpp` 的故障诊断接线，约 49 行），本记录以**工作区实际代码**为核查对象。
> 理论来源：[CTL-001](../../05-控制与分配/CTL-001-控制架构与SAS.md) §2–§8.6、[CTL-002](../../05-控制与分配/CTL-002-可控性与失效模式.md) §1–§3、[THY-002](../../03-理论推导/THY-002-推进系统的物理建模.md) §2–§6、[THY-003](../../03-理论推导/THY-003-刚体动力学控制分配与模态分析.md) §1.2–§3、[PROP-001](../../06-推进与执行机构/PROP-001-推进与摆座模型.md) §2–§6、[MOD-002](../../04-数学建模/MOD-002-坐标系与符号约定.md) §1.2/§3、`TandemVec-Paper/ch/05-allocation.tex`（eq:diff_def / eq:differential_roll / eq:effectiveness_matrix / eq:Btrue_allocation / eq:direct_mapping）、`ch07-control.tex`（eq:sas_* / eq:rate_command / eq:rc_curve / eq:max_clamp）、`ch16-adrc-onlineid.tex`（eq:eso2_* / eq:adrc_law / eq:rls_update / eq:excitation / eq:gain_scheduling）、`AGENTS.md` 踩坑记录（轴映射、差速增益调度、速率环量纲、FPV 曲线）。
> 方法：逐文件读代码 → 对照公式/式号与参数现值 → 核对 AGENTS 踩坑记录与红线条目 → 补 L0/L1/L2 与五段式公式块 → `check-code-comments.py` 复检 → 宿主 g++ 回归 + 真实源码 SIL 编译/运行。

---

## 1 范围与覆盖率

| 文件 | 行数 | L0 文件头 | L1 符号文档 | `@ref` | 备注 |
|------|------|-----------|-------------|--------|------|
| `src/flight_control.cpp` | 1890 | 有 | 有（每层段落标题 + 8 个函数文档块 + 4 段五段式） | 24 | 轴映射/速率量纲/差速调度三条红线 |
| `src/flight_control.h` | 77 | 有 | 有（3 个导出函数） | 2 | 原头缺 `@module/@details/@frame/@units` |
| `include/FlightCtrlParams.h` | 170 | 有（原 `//` 块改 `/** */`） | 有（两个结构体逐字段单位） | 2 | 清理重复 `#pragma once`；R05 长行重排 |
| `include/ControlUnits.h` | 51 | 有 | 有（2 个换算函数） | 3 | 量纲唯一换算点 |
| `include/IncrementalPID.h` | 144 | 有 | 有（类 + compute） | 0 | 历史工具类，无调用点（见 §2.6） |
| `include/PositionPID.h` | 395 | 有 | 有（主文件已有，补积分/微分约定） | 2 | R04 注释块改写 |
| `include/RcCurve.h` | 87 | 有 | 有（3 个函数） | 3 | 补 `@module/@details/@frame/@units` |
| `include/InertiaDecoupling.h` | 103 | 有 | 有（2 个函数） | 2 | R04 大块改写；坐标系问题见 P0-2 |
| `include/AirframeModel.h` | 164 | 有 | 有（结构体 + 2 个合成器） | 1 | 数值差分 B |
| `include/TandemVec_Config.h` | 139 | 有（原 `//` 块改 `/** */`） | 有（参数逐项单位） | 1 | dMax 15°/25° 双口径已注明 |
| `include/TandemVec_ControlAllocation.h` | 261 | 有 | 有（输入/输出/主函数） | 4 | R04 两处策略注释块改写 |
| `include/TandemVec_Propulsion.h` | 243 | 有 | 有（正向映射/差速/效能矩阵） | 2 | R04/R06 修（`/* Fx */` 假阳性） |
| `include/TandemVec_OnlineID.h` | 244 | 有 | 有（RLS + 管理器 step/adapt） | 1 | R04 FFRLS 注释块改写 |
| `include/TandemVec_ADRC.h` | 125 | 有 | 有（ESO2 + ADRC） | 1 | 未接线，已注明 |
| `include/TVC_Control_3rdOrder_Poly.h` | 75 | 有 | 有（1 个函数） | 1 | 已弃用，仅 main.cpp 包含、无调用 |
| `include/TVC_Control_Geometric.h` | 78 | 有 | 有（1 个函数） | 2 | 已弃用，同上 |
| `include/TandemVec_ServoModel.h` | 108 | 有（原 `//` 块改 `/** */`） | 有（2 个函数） | 1 | 仿真用；25° 口径已注明 |
| `include/ActuatorFaultMonitor.h` | 151 | 有 | 有（类 + update/action） | 1 | F3/F4 判据与边界 |

合计：18 文件 / 4512 行（核查前 3970 行，本轮净增 542 行注释与文档）；L0 覆盖 18/18；`@ref` 共 53 处。

---

## 2 核对矩阵

结论枚举：`一致` / `等价变换后一致` / `近似（可接受）` / `偏差（已登记）` / `不一致（待修）`。

### 2.1 执行器轴映射与构型语义（A）

| ID | 理论要素 | 文献出处 | 代码位置（符号名） | 结论 | 证据 / 处置 |
|----|----------|----------|--------------------|------|-------------|
| A-01 | 通道→执行器：滚转→上摆 δf、俯仰→下摆 δt、偏航→差速 Δω | AGENTS.md 踩坑记录（2026-08-07 轴置换）；CTL-001 §8.6 | `mix_and_output_commands` 的 `Mx/My/Mz` 三条逆解赋值 → `allocateMoments(BTRUE)` → `lower_gimbal_deg=ao.delta_t`、`upper_gimbal_deg=ao.delta_f` → `SetServoPos(...,TVC_PITCH/TVC_ROLL_SERVO_PIN)` | 一致 | 实码即锚点：`alpha_roll→Mz'→δf`、`alpha_pitch→My'→δt`、`alpha_yaw→Mx'→Δω`；`test_flight_control_axis` 断言 `M_pos_dt[1]<0`、`M_pos_df[2]>0`、`M_pos_dw[0]<0` 与分配层符号一致 |
| A-02 | 悬停构型语义：机头朝天、前电机最高=上摆、尾电机最低=下摆、差速绕推力轴=世界航向 | AGENTS.md 执行器分配段；MOD-002 §1.2 | 同上 + `TandemVec_Config`（Ix=轴向惯量）+ `TandemVec_Propulsion::computeWrench` | 等价变换后一致 | 固件存档系（z_b=推力轴）与论文/MOD-002 模型系（x'=推力轴）是同一机械布局的纯旋转置换 `x'=−z_b、y'=+y_b、z'=+x_b`（det=+1，`TandemVec_FCS/tools/verify_frame_map.py` 输出）；**同一物理、两套轴名** |
| A-03 | 巡航读法：前摆 δf→Mz（偏航）、差速反扭→Mx（滚转） | MOD-002 §1.2；CTL-001 §3 表；论文 ch05 eq:moment_linearized | `computeWrench`（Mz=a·Tf·sδf、Mx=−Qf·cf+Qt·ct） | 等价变换后一致 | 该读法属**仿真模型系**（x_b=机身纵轴/推力轴）；固件把同一 Mz'/Mx' 通道命名为滚转/偏航。论文 ch05 已注明"经轴置换后差速对应偏航通道"；CTL-001 §3 表未标口径 → P2-3 |
| A-04 | 轴置换符号：`Mx'=−Ix·α_yaw`、`My'=+Iy·α_pitch`、`Mz'=−Iz·α_roll` | AGENTS.md `verify_mix_axes.py` 结论；CTL-001 §4/§5 | `mix_and_output_commands` 层1 | 等价变换后一致 | `tools/verify_mix_axes.py` 以实机陀螺直通锚点反解：`tail: My'=+I·α_pitch`、`front: Mz'=−I·α_roll`、`dw: Mx'=−I·α_yaw`，与实码逐字一致 |
| A-05 | 手动 TVC 直通方向须与自控链最终摆角一致（roll 取反） | AGENTS.md 踩坑记录；`mix_and_output_commands` 锁定/手动分支 | `upper_gimbal_deg = mapFloat(roll_raw, 988, 2012, MAX_CORRECTION, −MAX_CORRECTION)` | 等价变换后一致 | 注释写明取反理由（`Mz=−Iz·α_roll` + 分配器正系数 ⇒ front 摆角与 α_roll 反号）；2026-08-08 已实机核对 |
| A-06 | 固件注释不得再写"x_b=机身纵轴=推力轴" | AGENTS.md 踩坑记录（写轴注释前先查 mix 层） | `execute_attitude_controller` 原注释 | 不一致（注释）→ 已修 | 原注释把模型系与存档系混读，已改正为"滚转=绕 x_b、z_b=推力轴"；同类旧注释"步骤7 → 上摆角 δ_f"、「ATTITUDE_MODE 走 q_err.z 姿态外环」一并废止（P2-1、P2-4） |
| A-07 | `GYRO_DIRECT_TEST` 调试直通路径不参与正式飞行 | `TandemVec_Config.h` 调试开关（`#if` 判断） | `mix_and_output_commands` 的 `#if GYRO_DIRECT_TEST` 块 | 偏差（已登记） | 宏为 0，整块不编译；块内注释"上摆 −K·ω_z"与实码 `+K·ω_z` 不一致、轴名沿用历史模型系 → P2-8，已在块内加警示 |

**口径注（必须与上述映射同时阅读）**：悬停构型下机身竖立、机头朝天，推力轴（体轴 z_b）竖直指地；上摆=前电机（最高点）主控滚转，下摆=尾电机（最低点）主控俯仰，差速绕推力轴=主控航向。巡航构型下同一对摆即"前摆/尾摆"；旧描述"前摆=偏航、差速反扭=滚转"是巡航系读法，已废止。两种读法数值上通过固定置换对应，**不得跨文档混用轴名**。

### 2.2 速率环量纲与串级单位（B）

| ID | 理论要素 | 文献出处 | 代码位置（符号名） | 结论 | 证据 / 处置 |
|----|----------|----------|--------------------|------|-------------|
| B-01 | 控制器输入 deg/s、输出 deg/s²，仅在写 `ControlOutputs_t` 时 ×π/180 | AGENTS.md"速率环量纲已统一（2026-08-11）"；CTL-001 §8.6 | `ControlUnits::dps2ToRadps2` + `execute_attitude_controller` / `execute_yaw_controller` 两处赋值 | 一致 | 全仓仅 `ControlUnits.h` 做换算；`dps2ToRadps2` 调用点只有上述两条链路与 `update_online_identification` 的 i_term；`test_control_units` 11 断言 PASS |
| B-02 | kp/ki 现值与旧混合域 0.28/0.20/0.10 严格等价 | 同上 | `kFlightCtrlParamsDefaults.rate_roll/rate_pitch/rate_yaw` | 一致 | 复算：0.28×57.29578=16.042818 ✓、0.20×57.29578=11.459156 ✓、0.10×57.29578=5.729578 ✓；yaw ki 2026-08-12 下调 25%：5.729578×0.75=4.297184 ✓（与文件现值逐位一致） |
| B-03 | PositionPID 的 D 项为"每拍差分（未除 dt）" | GOV-004 §8；AGENTS.md 速率环量纲条 | `PositionPID::computePID2DOF` 的 dOut 一行 | 一致 | kd 现役全 0，故当前无物理影响；非零时必须连同 ×180/π 一起迁移（注释已写入调用点） |
| B-04 | 旧混合域 ×57.3 换算警示必须写在控制链旁 | AGENTS.md | `execute_attitude_controller` 的"角速率内环量纲"红线块 | 一致 | 注释给出"外部参数写旧值会让物理增益低 57.3 倍"的可操作警示 |
| B-05 | 0x21/CAN/旧 `alpha_ref_*` 仍 rad/s²，`adps2_x/y/z` 才是角度域 | AGENTS.md AnoVars 条 | `gnc_tel.alpha_ref_radps2` / `alpha_ref_dps2` | 一致 | 遥测双字段并存，注释写明边界换算唯一出口 |
| B-06 | 姿态外环输出限幅 ±80 deg/s，与 RATE_MODE ±600 deg/s 解耦 | AGENTS.md FPV 曲线条；论文 ch07 §rc_curve | `kMaxTargetRate=80`、`MAX_MANUAL_*RATE=600` | 一致 | 复算满杆：0.5×200/(1−0.7)=333.3 deg/s、yaw 0.5×200/(1−0.55)=222.2 deg/s，均 <600 |

### 2.3 差速分配与增益调度（C）

| ID | 理论要素 | 文献出处 | 代码位置（符号名） | 结论 | 证据 / 处置 |
|----|----------|----------|--------------------|------|-------------|
| C-01 | 差速开方分配 ωf=ω0√(1+Δω)、ωt=ω0√(1−Δω) | CTL-001 §7.1；论文 ch05 eq:diff_def | `TandemVec_Propulsion::allocateDifferential` | 一致 | 公式与红线条目逐字一致；`max(0,·)` 兜底 + wMax 钳位 + 饱和标记；`test_tandemvec_allocation` 覆盖 |
| C-02 | 未钳位时 ωf²+ωt²=2ω0²（总推力一阶不变），钳位破坏该性质 | CTL-001 §7.2；论文 ch07 eq:max_clamp | 同上 | 一致 | 注释写明"钳位后总推力低于预期，调用方应提示操纵手" |
| C-03 | 低油门 1/w0² 使 Δω 指令爆炸 → 调度 (w0/wh)² | AGENTS.md 差速增益调度坑①（2026-08-07） | `mix_and_output_commands` 层2 的 `yaw_gain_sched` | 一致 | `w_hover=√(0.5·m·g/kT)` 复算 ≈574 rad/s；`w0_floor=0.6·w_hover≈344 rad/s ≈30% wMax`（注释数值正确） |
| C-04 | 物理力矩仍 ∝w0² → 稳态有效增益随油门放大 → 封顶 1.0 | AGENTS.md 坑②（2026-08-09） | `yaw_gain_sched = fminf((w0_eff/w_hover)², 1.0f)` | 一致 | 仅乘 Mx（差速）通道，摆座通道不扩范围 |
| C-05 | τm 一阶观测器 `w_est += (w_cmd−w_est)·dt/τm`（dt=0.005、τm=0.28） | AGENTS.md 坑③（2026-08-09 抖油门震荡根因）；THY-002 §4；PROP-001 §3 | `g_wf_est/g_wt_est` 更新两行 | 一致 | 每拍系数 0.005/0.28=0.017857；观测器输入是 `prev_prop_state`（上一拍分配输出），与"指令经 τm 滞后"一致；开环状态估计无稳定性问题 |
| C-06 | 有效回路增益 = kp·(w0_actual/w0_used)²，瞬态 0.25→0.44 越震荡点 0.35 | AGENTS.md 坑③ | 层2 长注释块 | 一致 | 注释完整复述三层根因与"约 160 ms"量级，并给出 `tools/verify_*.py` 验证入口 |
| C-07 | B 矩阵工作点/调度/current_state 全用观测转速；`allocateDifferential` 必须用真实指令 w0 | AGENTS.md 坑③修复；CTL-001 §8.6 | `ai.w0=w0` vs `w0_eff`、`ai.current_state.wf=fmaxf(g_wf_est,w0_floor)` | 一致 | 两套转速的用途在注释中逐条说明；零油门门控（throttle<5%）补回"零油门→执行器归中" |
| C-08 | 观测器用相对步长（无 min 截断），与 PROP-001 的 `min(dt/τ,1)` | PROP-001 §3.1/§3.2 | 同上 | 近似（可接受） | dt/τ=0.0179≪1，min 永不触发；两式在本工况数值等价 |

### 2.4 控制分配 B 矩阵（D）

| ID | 理论要素 | 文献出处 | 代码位置（符号名） | 结论 | 证据 / 处置 |
|----|----------|----------|--------------------|------|-------------|
| D-01 | 解析 B_true 九元素与论文 eq:Btrue_allocation 逐项一致 | 论文 ch05 eq:Btrue_allocation | `TandemVec_Propulsion::computeEffectMatrix` | 一致 | 逐项：`[−kQw0²(cf+ct), −Qt·st, Qf·sf; b·kTw0²st−kQw0²sf, −b·Tt·ct, −Qf·cf; a·kTw0²sf+kQw0²st, −Qt·ct, a·Tf·cf]` 与论文矩阵逐元素相同 |
| D-02 | 零摆角退化 B_full = diag(−2τ0, −bT0, aT0) + 交叉项 | CTL-002 §1.1；论文 ch05 eq:effectiveness_matrix | 同上（注释末段） | 一致 | 注释给出退化矩阵并注明 `T0=Tt=Tf、τ0=Qt=Qf` 条件 |
| D-03 | 数值差分 B（中心差分 h=1e-4 相对、摆轴 Rodrigues 旋转）与解析式等价 | THY-003 §2.2（摆角雅可比 = 摆轴×推力轴） | `AirframeModel::computeJacobianNumeric` + `computeEffectMatrixDataDriven` | 一致 | `test_tandemvec_allocation` T11：相对 <5e-3、绝对 <5e-4；零摆角耦合项为 0 两实现一致；`test_airframe_generic` 14 断言 PASS |
| D-04 | 本机 3 输入×3 力矩通道为方阵：用 B⁻¹（精确逆）而非 B⁺ 伪逆 | THY-003 §3.2；CTL-001 §2 | `allocateMoments` BTRUE 分支（Cramer） | 等价变换后一致 | 方阵且非奇异时 B⁺=B⁻¹；伪逆只在冗余执行器场景需要（CTL-002 §5 登记为未来方向） |
| D-05 | 近奇异保护：|det|<1e-8 退降 FULL_B；零推力 T0<T0_MIN 直接全零 | THY-003 §3.3（奇异性）；CTL-002 §2 | `allocateMoments` 的 `DET_MIN` / `T0_MIN` | 一致 | 注释说明退降条件（零油门/极端摆角）；工作点 floor 使 det 恒 ≈1.1e-2（`tools/verify_floor_consistency.py`） |
| D-06 | 列序 [Δω, δ_t, δ_f] ↔ 行序 [Mx', My', Mz']；输出限幅 dMax/dwMax | 论文 ch05 eq:Btrue_allocation；CTL-001 §8.6 | `AllocationOutput` / `EffectMatrix` / 末尾 clampf | 一致 | `sat_*` 标记 = 限幅前后是否不等；固件 dMax=15°、dwMax=0.7 |
| D-07 | `roll_authority = (2kQw0²)/(2kQwMax²)` | 论文 ch05 §耦合比 | `out.roll_authority = T0/T0_max` | 等价变换后一致 | 代码用 kT 比值、注释用 kQ 比值，二者是同一 `(w0/wMax)²`，结论相同 |

### 2.5 惯量逆解与交叉耦合（E）

| ID | 理论要素 | 文献出处 | 代码位置（符号名） | 结论 | 证据 / 处置 |
|----|----------|----------|--------------------|------|-------------|
| E-01 | 欧拉方程 `I·ω̇ = M − ω×(Iω) − ω×h`，前馈取 `M_ff=ω×(Iω)+ω×h` | THY-003 §1.2、§5；THY-002 §5 | `InertiaDecoupling::computeInertiaCompensation` | 一致 | 对角惯量展开三项 + 叉乘三项；符号以仿真 `dynamics.mjs` 为锚（g += ω×h，M_ff=+g） |
| E-02 | 转子角动量 h 沿推力轴，`h=Jp(ωf·cf−ωt·ct)`（前后反向） | THY-002 §5（h=Jp·ω·n）；论文 ch03 | `mix_and_output_commands` 的 `h_rotor` | 近似（可接受） | 用上一拍摆角余弦投影（小角度 cos≈1），忽略摆座摆动附加进动项 `−δ̇·Jp·ω·(e_摆轴×n)`（THY-002 §5 末条），量级 Jp·ω·δ̇ |
| E-03 | 使能掩码 bit0=陀螺耦合、bit1=转子陀螺，可在线 A/B | AGENTS.md（`inertia_comp_mask`）；`FlightCtrlParams.h` | `inertiaCompEnabled` / 掩码分支 | 一致 | 默认 0x03；host `test_inertia_decoupling` 15 断言对照刚体模型（7e-7） |
| E-04 | 前馈的输入必须与 ω、I、h 同坐标系 | MOD-002 §2（DCM 语义）；AGENTS.md 轴映射 | 同上调用点 | **不一致（待修）** | 固件传入 fw 系陀螺（x=滚转、z=推力轴），却用模型系惯量顺序（Ix=轴向 0.0021 放在第 1 位）与 `h=(h,0,0)`（h 实际沿推力轴=z）；随后又按 fw→模型置换映射结果 → 见 **P0-2** |

### 2.6 在线辨识 / ADRC / INDI 定位（F）

| ID | 理论要素 | 文献出处 | 代码位置（符号名） | 结论 | 证据 / 处置 |
|----|----------|----------|--------------------|------|-------------|
| F-01 | FFRLS：`y=φᵀθ+ε`，`φ=[α_cmd,1]ᵀ`，`θ=[b,d]ᵀ`，标准递推 + 遗忘因子 | 论文 ch16 eq:rls_model/eq:rls_update；Åström & Wittenmark (1995)、Ljung (1999) | `RLS_2Param::update` | 一致 | 逐式核对：S=Pφ、K=S/(λ+φᵀS)、θ+=Kε、P=(I−Kφᵀ)P/λ；`test_online_id` 25 断言 PASS |
| F-02 | 激励门控 `σ²=0.95σ²+0.05α_cmd²`，σ²≤4 跳过更新 | 论文 ch16 eq:excitation | `OnlineID::step` | 一致 | 与论文式逐项一致（RMS<2 rad/s² 不更新）；另有 5%/拍变化率限制与 b∈[0.3,3.0] 硬约束 |
| F-03 | 辨识输入角速率用 deg/s（内部转 rad/s 再微分） | 注释 2026-08-11 修复记录 | `OnlineID::step` 的 `DEG2RAD` | 一致 | 若直接传 deg/s 会把 ω̇ 放大 57.3 倍并把 b 顶到约束边界（注释已写） |
| F-04 | CG 偏移 = `i_term[1]·Iy / (2·kT·w0²)`，悬停门控（30–70% 油门、三轴 |ω|<15 deg/s） | 论文 ch16 eq:cg_moment/eq:cg_offset | `OnlineID::step` 尾段 | 一致 | 用实际油门反推 w0，不再硬编码 40% |
| F-05 | 辨识只观测、不写回增益 | AGENTS.md"第一步只观测不闭环" | `update_online_identification` 的导出段 | 一致 | 只写 `id_*`；建议 Kp 仅供地面站（`adaptKpR`） |
| F-06 | ADRC（二阶 ESO）为候选模块，未接入生产链 | 论文 ch16 §ESO/§ADRC 稳定性 | `TandemVec_ADRC.h` | 一致 | `grep #include "TandemVec_ADRC.h"` = 0 处；论文亦注明"ADRC 尚未接入生产控制链" |
| F-07 | 固件不是 INDI：BTRUE 是"上一拍工作点 Jacobian + 绝对指令"（准线性/增益调度） | AGENTS.md（2026-08-11 术语修正）；CTL-001 §8.5 | `mix_and_output_commands` 首段注释 | 一致 | 无增量累加、不用实测角加速度；注释已写明"不是 INDI" |
| F-08 | `TandemVec_ServoModel.h` 为仿真/分析模型，固件不调用 | PROP-001 §7（理想化声明） | 全仓 include 计数 = 0 | 一致 | 已在 L0 注明，并标明 25° 为仿真口径 |

### 2.7 FPV 摇杆曲线与舵机模型（G）

| ID | 理论要素 | 文献出处 | 代码位置（符号名） | 结论 | 证据 / 处置 |
|----|----------|----------|--------------------|------|-------------|
| G-01 | 三参数模型 `ω=200·rcRate·rcCmd/(1−|rcCmd|·super)`，`rcCmd=rc(expo·rc²+1−expo)` | 论文 ch07 eq:rc_curve；AGENTS.md FPV 条 | `rcRateCurve` / `rcExpoCurve` | 一致 | 与论文式逐项一致；`test_rc_curve` ALL PASSED |
| G-02 | 满杆值 = rcRate×200/(1−super)；死区 0.02；极点保护 denom≥0.1 | 论文 ch07 §rc_curve | 同上 | 一致 | 复算 333.3/222.2 deg/s；死区 0.02×512≈10 µs |
| G-03 | 限幅必须独立于姿态外环（600 而非 80） | AGENTS.md（2026-08-11 解耦）；论文 ch07 §实机限幅解耦教训 | `MAX_MANUAL_rollRATE/pitchRATE/yawRATE` 调用点 | 一致 | 注释写明"80 会在半杆削平曲线"的成因 |
| G-04 | 舵机动态：速率限制 + 死区 + 一阶滞后（摆座端↔舵机端换齿轮比） | PROP-001 §3（一阶滞后离散化） | `stepServo` | 近似（可接受） | 代码用 `α=dt/(τ+dt)`（近似 ZOH），PROP-001 用 `min(dt/τ,1)`（前向欧拉），差 O((dt/τ)²)≈3e-4；注释已标"仿真口径" |
| G-05 | 摆角→PWM：中位 + 半行程归一 + 齿轮比 + 方向 | `TandemVec_Config.h` §ServoConfig；`flight_control.cpp` 层3 | `gimbalToServoPWM`（仿真）/ mix 层（固件） | 等价变换后一致 | 固件用 `SC.zero_*_pct`、`SC.half_travel_deg`、`gear=40/30`、`dir_*=±1`；`gimbalToServoPWM` 的 `gimbal_max` 形参未使用（P2-6） |

### 2.8 执行器故障监测 F3/F4（H）

| ID | 理论要素 | 文献出处 | 代码位置（符号名） | 结论 | 证据 / 处置 |
|----|----------|----------|--------------------|------|-------------|
| H-01 | F4 判据：|姿态误差|>25° 且 |摆角指令|≥90% dMax 持续 0.5 s | notes/08-F4 单舵机卡死能力边界 | `ActuatorFaultMonitor` 默认配置 + `update` | 一致 | 阈值与 notes/08"25°/0.5 s 只是仿真值"一一对应；`test_actuator_fault_monitor` 9/0 PASS |
| H-02 | F4 只能"检测 + 有界下降"，前摆是滚转唯一主控，无第二条通道 | notes/08 结论；AGENTS.md 低速悬停闭环口径 | `action()`→ControlledDescent；`g_actuator_controlled_descent_request` | 一致 | 只写标志，无执行路径；notes/08 明确"不是可恢复故障" |
| H-03 | F3 判据：油门>75% 且垂直速度误差<−1.0 m/s 持续 1.0 s | notes/08；FULL-ENVELOPE-STATUS F3 行 | `ActuatorFaultMonitor` 配置 + 调用点 | **不一致（待修）** | 监测器约定"下沉为负"（测试 T4 用 −2.0 m/s），但调用点传 `target_vertical_velocity − current_v_up`（下沉为正）→ 见 **P0-1** |
| H-04 | F3 动作建议 ThrustBoost，定高环可补偿 ~30% 推力损失 | FULL-ENVELOPE-STATUS F3 行 | `action()` | 一致 | 注释区分"高度环补偿"与"故障重构"，不外推 |

### 2.9 SAS / 摆角 / 差速限幅口径（J）

| ID | 理论要素 | 文献出处 | 代码位置（符号名） | 结论 | 证据 / 处置 |
|----|----------|----------|--------------------|------|-------------|
| J-01 | 仿真 SAS 限幅 ±25°/±0.7 是**仿真值**，固件实机摆角 ±15° | AGENTS.md 行为红线；CTL-001 §4.3/§8.6；`TandemVec_Config.h` | `kDefaultTandemVecParams.dMax=0.2617994`（15°）、`dwMax=0.7` | 等价变换后一致（不同口径） | **不得混写**：仿真 `P.dMax`=25°（`aircraft-model.json` 未回写）、固件 15°；dwMax 两边都是 0.7。注释与文档均以"不同口径"表述 |
| J-02 | 固件姿态外环输出 ±80 deg/s（`kMaxTargetRate`）；手动姿态角 ±30° | CTL-001 §8.6；`state_data.cpp` | `MAX_TARGET_RATE`、`MAX_ANGLE_COMMAND` | 一致 | `max_angle` 需小于 90°−dMax 保留执行器余量（注释） |
| J-03 | 位置环最大倾角 15°（`POS_CTRL_MAX_TILT_ANGLE_RAD`）与刹车预测 `MAX_TILT_DEG` 一致 | CTL-001 §8.4 | `handlePositionControl` 的 `MAX_TILT_DEG` | 一致 | 均 15°；a_max=g·tan15°≈2.62 m/s² 注释复算正确 |

### 2.10 垂直/水平通道（K）

| ID | 理论要素 | 文献出处 | 代码位置（符号名） | 结论 | 证据 / 处置 |
|----|----------|----------|--------------------|------|-------------|
| K-01 | `F_z=m(g+a_z)`；倾角补偿 `F_total=F_z/|R33|`（钳 0.5）；`thr%=F/MAX_THRUST` | PROP-001 §5；MOD-002 §1.2；CTL-001 §8.4 | `compute_thrust_control` 段3 | 一致 | `MAX_THRUST=27.5 N` 与 2·kT·wMax²=27.5 一致；R33 用法注释含 2026-08-07 的 R13 事故 |
| K-02 | 刹车点 `s=v|v|/(2a_max)·1.4`，限幅 ±5 m（水平）/±3 m（垂直） | CTL-001 §8.4（物理制导优化） | `handlePositionControl`/`compute_thrust_control` 状态机 | 一致 | 注释给出 a_max=g·tan15° 来源与安全系数含义 |
| K-03 | 水平加速度圆限幅（非方形） | THY-003 §3（可达集） | `handlePositionControl` 段5 / `generate_attitude_target` GUIDED 分支 | 一致 | 圆形限幅避免斜向超限；注释已说明 |
| K-04 | 推力方向 → 目标姿态：`q_tilt ⊗ Rz(Heading)`（定点/制导）、`Rz(Heading) ⊗ Rxy`（姿态模式） | MOD-002 §3（右乘约定）；CTL-001 §8.6 | `generate_attitude_target` | 等价变换后一致 | 存档系 z_b=推力轴 → 悬停 roll=pitch=0，无需悬停基态补偿；注释含 2026-08-07 的错误映射事故 |

---

## 3 发现清单

| 严重度 | 位置（符号） | 问题 | 影响 | 建议 |
|--------|--------------|------|------|------|
| **P0-1** | `updateActuatorFaultDiagnostics` → `ActuatorFaultMonitor::update` 的 `vertical_vel_err` | 调用方传 `target_vertical_velocity − current_v_up`（下沉为正），而监测器/测试的约定是"下沉为负"（`thrust_vel_err_mps=-1.0`，`test_actuator_fault_monitor` T4 传 −2.0 m/s） | F3 单发推力损失**漏报**（真实下沉时 `err` 为正，永不满足 `<−1.0`）；反之爬升超目标 >1 m/s 且油门 >75% 持续 1 s 会**误报** ThrustBoost。当前无自动执行路径，不影响操纵，但 F3 检测实际失效 | **已修复（2026-10-23，DEC-002）**：调用方改走 `bfs::thrustLossVerticalVelocityError(measured, target)`，F3 仅在 AUTO_ALTITUDE/AUTO_POSITION/GUIDED 启用；SIL 发现 75% 门限在未标定模型下不可达，但**实飞基线优先，`thrust_throttle` 保持 0.75**（是否参数化/下调待实机日志）；`test_actuator_fault_monitor` 18/18（T4b 75% 边界、T8–T10） |
| **P0-2** | `mix_and_output_commands` 的惯量前馈段（`omega_body`/`h_rotor`/`Ix,Iy,Iz`） | 输入不在同一坐标系：ω 是固件存档系（x=滚转、y=俯仰、z=推力轴），但惯量按模型系顺序传（Ix=轴向 0.0021 排第 1 位，实际应排 z），`h_rotor=(h,0,0)` 也放在第 1 位（h 沿推力轴=z）；结果随后又按"存档系→模型系"置换映射 | 转子陀螺前馈落到错误通道：物理上应由"俯仰/滚转速率"产生"滚转/俯仰"力矩，实码变成由"偏航/俯仰速率"产生"差速/俯仰"力矩；同时 `(Iz−Iy)ωyωz` 项因 Iy=Iz 恒 0，而正确项应为 `(I_z^fw−I_y^fw)=(0.0021−0.022)≠0`。悬停 ω≈0 时≈0，机动时注入 ~0.05–0.17 N·m 的错误通道力矩（差速权限 0.74 N·m 同量级） | **已修复（2026-10-23，DEC-002）**：新增 `computeInertiaCompensationFromArchive()`，一次性完成 ω′=(−ωz,ωy,ωx)、h′=(h,0,0)、I′=(I_axial,I_pitch,I_roll) 置换，输出模型系直接相加；`test_inertia_decoupling` 全通过（T6 复现旧缺陷、T7 封装=显式置换），`pio run` 双 env SUCCESS |
| P1-1 | `execute_attitude_controller` 的 RATE_MODE 分支 | 每拍执行 `rollRatePID.reset(); pitchRatePID.reset();`（连同外环 PID） | RATE_MODE 下速率环积分每拍清零 → 等效纯 P；与论文 ch07"固件 Rate 模式采用 PI 并输出角加速度指令"不一致；若依赖积分消除配平误差，RATE_MODE 下不存在 | 文档/注释写明该口径（本轮已在函数 L0 注明"RATE_MODE 复位内环 PID"）；是否改为保留积分属行为变更，需评审 |
| P1-2 | `update_online_identification` 的 `alpha_cmd[0..2]` 符号链 | 辨识命令量经 `s_yaw_gain_sched` 修正，但三通道与惯量/轴置换的符号未在同一测试中对齐；首测 `b≈0.35/0.36` 贴近硬约束下限 0.3 | 若符号反，RLS 会收敛到负值并被 `clamp(0.3,3.0)` 截断，读数"稳定"但无物理意义 | 用 `inertia_comp_mask=0` + 已知 α 方波做一次开环合成数据回归（host 可做），确认 b 的符号与量级 |
| P2-1 | `TandemVec_FCS/AGENTS.md` §坐标系统一约定 | "x_b = 机身纵轴（…）；★ 电机推力轴 = z_b" 同一段自相矛盾（x_b 与 z_b 都被称纵轴/推力轴） | 写轴注释时最易再踩；本次核查即在源码中发现两处混读 | 改为"存档系：z_b=推力轴=机身纵轴（悬停竖直指地）；x_b/y_b 为两条横向轴（悬停水平）" |
| P2-2 | 根 `AGENTS.md` 速率环量纲条 | 写 "yaw ki=5.729578"，实际 `FlightCtrlParams.h` 已是 4.297184（2026-08-12 下调 25%） | 参数抄写漂移 | 同步为 4.297184 并注明下调日期 |
| P2-3 | `CTL-001 §3` 三通道表 | 表中 δf→Mz 称"偏航"、Δω→Mx 称"滚转"，未标"仿真模型系" | 与固件通道名相反，首次阅读者会误判代码轴错 | 表头加"仿真模型系（x_b=推力轴）；固件通道名见 §8.6" |
| P2-4 | `execute_yaw_controller` 文档/形参 | 原注释"ATTITUDE_MODE 走 q_err.z 姿态外环"与实码（ratchet hold）不符；形参 `q_target` 未使用 | 注释腐烂（已在本轮改写为实码语义） | 保留形参（与同签名），注释已说明 |
| P2-5 | `TandemVec_ServoModel.h` 的 `gimbalToServoPWM` | 形参 `gimbal_max` 未使用（-Wunused-parameter）；该文件 25° 为仿真口径 | 误读为固件限幅 | L0 已注明"仿真/分析模型"与 15°/25° 双口径；形参保留待后续清理 |
| P2-6 | `TVC_Control_Geometric.h` | `r²<0` 时静默退回零位长度；`M_PI` 依赖 `_USE_MATH_DEFINES` 且必须在任何 `<cmath>` 之前包含 | 标定时若某角度恒返回 50 难以发现；头文件顺序敏感 | 已在 L0 注明兜底语义；建议后续改为显式错误返回（非本区域行为变更） |
| P2-7 | `IncrementalPID.h` | 无任何 `@ref`；Ki/Kd 隐含采样周期，且无调用点 | 历史工具类，误用会与 PositionPID 的连续域 Ki 混淆 | L0 已注明"离散增益、换频率必须重整定、无调用点" |
| P2-8 | `mix_and_output_commands` 的 `GYRO_DIRECT_TEST` 调试块 | 块内注释写"上摆: δf = −K·ω_z"，实码为 `upper_gimbal_deg = +GYRO_K * icm_gyro_z`；轴名沿用历史模型系（上摆"绕 z_b"），与固件通道命名（上摆=滚转=体轴 x）不同；该块在 `GYRO_DIRECT_TEST=0` 时不参与编译 | 调试路径的注释自相矛盾；若将来重启直通测试，可能按注释改错符号 | 本轮已在块内加"调试死路径/符号以代码为准/勿与 mix 层通道命名混读"警示；是否修正符号需按实机直通重新标定后决定 |

**P0 说明**：两条 P0 均属"符号/坐标系错误"，但都作用在**前馈/诊断层**（不是闭环稳定性的基础）：F3 无自动执行路径；惯量前馈可通过 `inertia_comp_mask=0` 在线关闭，悬停 ω≈0 时影响≈0。实机首飞已通过说明二者未造成失控，但会掩盖/污染调参与故障诊断，应按 P0 跟踪。**2026-10-23 已按 DEC-002 修复并回归**（只改这两处，未改增益/限幅/阈值）。

---

## 4 未决问题

1. **P0-1 已修复**：调用方符号统一走 `thrustLossVerticalVelocityError()`，F3 仅自动垂直模式启用；后续建议在实机日志中确认 F3 标志与真实掉高事件一致（当前无自动执行路径）。
2. **P0-2 已修复**：存档系→模型系封装已通过 host 置换等价回归；实机建议仍做一次 `inertia_comp_mask=0/0x03` A/B（机动段对比姿态超调/振动），确认修正后的前馈方向正确。
3. **P1-2 在线辨识符号链**：需一次开环/合成数据回归；当前 `b_est` 首测值（0.35/0.36/0.90）与 0.3 下限过近，存在被硬约束截断的可能。
4. **轴名术语统一**：`MOD-002 §1.2`、`TandemVec_FCS/AGENTS.md`、`CTL-001 §3`、`TandemVec_Propulsion.h` 用的是"模型系（x=推力轴）"，固件源码与 `TandemVec_Config.h` 用"存档系（z=推力轴）"。建议在 GOV-004 §7.1 增一行"两套轴名的固定置换"，避免每轮核查重复解释。
5. **`TVC_Control_*` 两个弃用头文件**仍被 `src/main.cpp` 包含（无调用点）——属 fw-infra 的 include 清理项，不在本区域改动。
6. **`fw_sil --manual` 口径**：默认（定高）场景本轮复跑仍收敛到 10.00 m；`--manual` 复跑末尾高度持续下降（−458 m/40 s）且 roll/pitch≈0，与 AGENTS"roll +5.86°/pitch −5.85°"的记录不一致，疑为 fw-tests 侧 harness 更新中，留给该区域确认（不影响本区域结论）。

---

## 5 门禁与测试

### 5.1 注释规范门禁

```text
核查前：py -3.12 tools/check-code-comments.py --area fw-control --check
        18 文件 | 平均分 58.6 | error 30 | warning 32
        （R01=18 R03=1 R04=19 R05=8 R06=2 R07=11 R09=3）→ 退出码 1

核查后：py -3.12 tools/check-code-comments.py --area fw-control --check
        18 文件 | 平均分 100.0 | error 0 | warning 0 | 全绿 18 → 退出码 0
```

### 5.2 宿主回归（g++ 13.2.0，`-std=c++17 -O2`，仓库根目录）

命令模板（MinGW g++ 是原生 Windows 程序，`-o` 必须用 `C:/...` 或相对路径，MSYS 的 `/c/...` 会报 cannot open output file）：

```bash
cd TandemVec_FCS/..     # 仓库根
BIN="C:/Users/<user>/AppData/Local/Temp/fwctrl"; mkdir -p "$BIN"
INC="-ITandemVec_FCS/include -ITandemVec_FCS/lib/eigen/src -ITandemVec_FCS/test_host/stub"
for t in test_flight_control_axis test_tandemvec_allocation test_position_pid \
         test_airframe_generic test_rc_curve test_inertia_decoupling \
         test_online_id test_control_units test_actuator_fault_monitor; do
  g++ -std=c++17 -O2 $INC TandemVec_FCS/test_host/$t.cpp -o "$BIN/$t.exe" && "$BIN/$t.exe"
done
```

| 测试 | 结果（末尾输出） | 结论 |
|------|------------------|------|
| `test_flight_control_axis.cpp` | `=== 全部通过 ===`（41 条 PASS 行） | PASS |
| `test_tandemvec_allocation.cpp` | `=== 全部通过 ===`（72 条 PASS 行） | PASS |
| `test_position_pid.cpp` | `=== 全部通过 ===`（53 条 PASS 行） | PASS |
| `test_airframe_generic.cpp` | `=== 全部通过（通用框架跨机型验证）===`（14） | PASS |
| `test_rc_curve.cpp` | `test_rc_curve: ALL PASSED` | PASS |
| `test_inertia_decoupling.cpp` | `全部通过（失败 0）`（含 T6/T7 跨系回归） | PASS |
| `test_online_id.cpp` | `=== 全部通过 ===`（25） | PASS |
| `test_control_units.cpp` | `test_control_units: ALL PASSED`（11） | PASS |
| `test_actuator_fault_monitor.cpp` | `=== 结果: 16 通过, 0 失败 ===`（含 T8–T10） | PASS |

合计 9/9 套件通过，0 失败（与核查前基线一致）。

### 5.3 固件源码编译与闭环检查

- **真实源码 SIL**（`test_host/fw_sil/build_and_run.sh`，链接 `src/flight_control.cpp` + `src/state_data.cpp`）：
  - 编译 **成功**（无 error；`g++ … -DTANDEMVEC_FIRMWARE`，输出到临时目录）；
  - 默认场景运行 40 s：高度收敛到 **10.00 m**（t=36 s 起），roll/pitch/yaw ≈0，舵机 50/50、电机 49.9/49.9 —— 与 AGENTS 记录的"定高到 10.00 m"一致，说明注释改动未影响控制链数值。
  - 首次尝试时该脚本因 `lib/navigation-main/src/ekf_15_state.h` 缺 5 参重载而中断（fw-ekf 区域正在改，非本轮引入）；稍后复跑已恢复并成功。
- **独立头文件语法检查**：14 个被改头文件合并成一个 TU 做 `g++ -fsyntax-only` 通过；`TVC_Control_Geometric.h` 单独编译通过（其 `M_PI` 依赖 `_USE_MATH_DEFINES` 先于 `<cmath>`，属既有顺序敏感性，见 P2-6）。
- **`pio run`**：2026-10-23 P0 修复后统一复跑，`TandemVec_FCS` 与 `test` 两个 env 均 **SUCCESS**（约 29 s / 32 s，2/2 succeeded；RAM 35.7%、Flash 16.6%）。

### 5.4 行为不变自查

- `git diff` 逐文件扫描（18 个文件）：新增可执行语句 **0** 行、删除可执行语句 **0** 行。非注释 diff 行仅三类，均可逐条解释：
  1. **L0 头插入导致的 include/`#pragma once` 位置变化**（`AirframeModel.h`、`InertiaDecoupling.h`：`#pragma once`+`#include` 移到顶部注释块之后）；
  2. **`FlightCtrlParams.h` 初始化行的空白对齐**（att_*/pos_n/pos_e 行去掉了多列对齐空格、pos_n/pos_e 拆成两行，**数值逐字未变**；另删除重复的 `#pragma once`）；
  3. **任务开始前已存在的未提交改动**（`flight_control.cpp` 的 `ActuatorFaultMonitor` include/静态对象/`updateActuatorFaultDiagnostics`/调用点，以及新文件 `ActuatorFaultMonitor.h` 本身），非本轮引入，已计入代码基线说明。
- 无任何增益/限幅/阈值/逻辑改动；`tools/verify_*.py` 未需要重跑（公式未动）。
- 本轮只改本区域 18 个文件 + 本记录；未提交 git（遵守纪律）。

### 5.5 P0 修复复验（2026-10-23，DEC-002）

| 项 | 改动 | 回归 |
|----|------|------|
| C-01 F3 符号 | `ActuatorFaultMonitor::update()` 增加 `thrust_detection_enabled`；新增 `thrustLossVerticalVelocityError()`；`flight_control.cpp` 改用该函数并按自动垂直模式门控 | `test_actuator_fault_monitor` 18/18（T8 符号契约、T9 使能开关、T10 旧反向符号漏报回归） |
| C-02 惯量前馈跨系 | `InertiaDecoupling.h` 新增 `computeInertiaCompensationFromArchive()`；mix 层改用它，输出模型系直接相加（删除手写置换）；h 的模型系/存档系符号（x′=−z_b）写入文档 | `test_inertia_decoupling` 全通过（T6 复现旧直调恒 0、新封装 −0.0199；T7 封装=显式置换；T8 存档系直接叉乘等价），`pio run` 双 env SUCCESS |
