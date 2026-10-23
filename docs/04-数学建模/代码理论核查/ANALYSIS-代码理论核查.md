# 代码理论核查记录 — analysis（高保真仿真 / EKF 研究仿真 / 脚本与工具）

> 核查日期：2026-10-23 ｜ 执行：analysis Agent（多 Agent 注释与理论核对轮）｜ 规范：[GOV-004](../../00-项目治理/GOV-004-代码注释与理论对照规范.md)
> 代码基线：`main` @ `6784221`（工作区含未提交改动；`sim/hifi/run_full_envelope.py`、`run_nav_envelope.py`、`run_firmware_suite.py`、`sim/opt/upgrade_candidates.py`、`upgrade_diag.py`、`p15_consider_probe.py` 在工作区为**未跟踪文件**）
> 理论来源：REF-120 §7.2/§7.4/§8.1/§9.2/§9.3/§9.6、REF-121 §2.2.1/§2.2.4/§3.5/§3.10/§6.1.3/§7.3.1、MOD-001/002/003、CTL-001、THY-001/002/003、`notes/02`、`notes/03`、`notes/05`、`notes/07`、`notes/08`、`TandemVec-Paper/ch10、ch12、ch14、ch15、ch16、ch17`、`TandemVec-EKF-Report/` 各报告
> 方法：逐文件读代码 → 对照公式/固件实现 → 数值复算（配平、线性化特征值、EKF 冒烟）→ 补 L0/L1 注释 → lint 复检 → AST 比对确认无行为变化

---

## 1 范围与覆盖率

| 组 | 路径 | 文件 | 行数 | L0 文件头 | 新增/改写 @ref | 备注 |
|----|------|-----:|-----:|-----------|----------------|------|
| 高保真分析 | `simulations/high-fidelity-analysis/**` | 18 | 2 678 | 18/18 | ~95 | core/controllers/estimators 为理论关键文件 |
| 浏览器实验台 | `simulations/py-web-lab/**`（排除 `web/vendor`） | 24 | 3 266 | 24/24 | ~55 | 服务端 + 前端 + 测试 |
| EKF 研究仿真 | `TandemVec-EKF-Report/sim/**` | 20 | 5 099 | 20/20 | ~90 | ekf_core/hifi_core/opt_ekf 为理论关键文件 |
| 脚本 | `scripts/**` | 11 | 1 751 | 11/11 | ~50 | MPC/自适应/辨识/配平 |
| 工具 | `tools/sync-params.py`、`tools/build-standalone.py` | 2 | 301 | 2/2 | 8 | `build-docs.py`/`check-links.py`/`check-code-comments.py` 由 Lead 负责 |
| 渲染 | `blender-test/**` | 2 | 1 126 | 2/2 | 4 | 渲染层，`@theory N/A` |

合计：**80 文件**（检查器口径，含 Lead 负责的 3 个 `tools/` 文件）/ 约 14.6 k 行；本轮实改 **73 文件**（64 `.py` + 9 `.mjs`）。L0 覆盖率 80/80，理论关键文件 6/6 均有 `@ref REF-/MOD-/CTL-/THY-`。

> 未改动的 7 个 = 3 个 Lead 负责文件 + 4 个在核查前已合规的未跟踪文件（`run_full_envelope.py` 等实际已由本轮补齐，见 git `??` 状态）。

---

## 2 核对矩阵

结论枚举：`一致` / `等价变换后一致` / `近似（可接受）` / `偏差（已登记）` / `不一致（待修）`。

### 2.1 高保真六自由度模型（`simulations/high-fidelity-analysis`）

| ID | 模块/符号 | 理论要素 | 文献出处 | 代码位置（符号名） | 结论 | 证据 |
|----|-----------|----------|----------|--------------------|------|------|
| A-01 | 推力矢量六维映射 | `Fx=Tf·cf+Tt·ct`、`Fy=Tf·sf`、`Fz=−Tt·st`；`Mx=−Qf·cf+Qt·ct`、`My=−b·Tt·st−Qf·sf`、`Mz=a·Tf·sf−Qt·st` | MOD-001 §2、§4 | `Propulsion.forces()` | 一致 | 六式与 MOD-001 §4 逐项对应；前摆绕 z_b、尾摆绕 y_b、反扭随机座投影一致 |
| A-02 | 推力/反扭 | `T=k_Tω²`、`τ=k_Qω²+J_pω̇` | THY-002 §2、MOD-001 §3.1 | `Propulsion.forces()` | 一致 | `Qf=kQ·wf²+Jp·dwf`，`dwf` 由 `update()` 缓存（注释记录"重算恒为 0"的坑） |
| A-03 | 差速分配 | `ωf=ω0√(1+Δω)`、`ωt=ω0√(1−Δω)` | CTL-001 §5（行为红线） | `Propulsion.update()` | 一致 | 与 `propulsion.mjs` 同式；另加 `wMax` 钳制（`√(1+Δω)>1` 时的边界） |
| A-04 | 转子角动量 | `h=Jp·ωf·[cf,sf,0]ᵀ−Jp·ωt·[ct,0,−st]ᵀ` | MOD-001 §3.4 | `Propulsion.forces()` 的 `self.hv` | 一致 | 代码 `hv=[Jp(wf·cf−wt·ct), Jp·wf·sf, Jp·wt·st]`，展开与 §3.4 相同 |
| A-05 | 平动方程 | `m·v̇=F+F_a+m·g_b−m(ω×v)` | MOD-001 §5.1、REF-120 §7.4 (7.4.6) p.189 | `dynamics_derivatives()` | 一致 | `vdot=(F+a)/m+g_b−ω×v`，`g_b=q⊗[0,0,g]⊗q*`（NED z 向下为正） |
| A-06 | 转动方程 | `I·ω̇=M+M_a−ω×(Iω)−ω×h` | MOD-001 §5.2 | `dynamics_derivatives()` | 一致 | `gx=(Iz−Iy)q r+(q hz−r hy)` 等三项展开正确 |
| A-07 | 姿态运动学 | `q̇=½·q⊗[0,ω_b]`（右乘） | REF-120 §9.2 (9.2.14) p.268、MOD-002 §3.4 | `dynamics_derivatives()`、`rk4_step()` | 一致 | 右乘；RK4 每阶段 `quat_norm` 保范（红线） |
| A-08 | 气动模型 | `L=q̄SC_Lα`、`D=q̄S(CD0+CDkC_L²)`、`M=q̄S·c̄(Cm0+Cmα·α+Cmq·q̂)` | MOD-001 §6 | `aero_forces()` | 一致 | 风轴→机体投影 `aX=L sinα−D cosα`、`aZ=−L cosα−D sinα` 与 MOD-001 §6 相同；`V<0.5 m/s` 钳位（红线） |
| A-09 | 控制效率矩阵 | `∂Mx/∂Δω≈−2k_Qω0²`、`∂My/∂δt=−b·Tt·cosδt`、`∂Mz/∂δf=+a·Tf·cosδf` | MOD-001 §4、THY-003 §3 | `control_effectiveness()` | 一致 | 手算复核：`Mx=−tau0(1+Δω)cf+tau0(1−Δω)ct` ⇒ `∂/∂Δω=−tau0(cf+ct)`，δ=0 时 =−2tau0 ✔；非对角项（反扭随摆座投影）亦正确 |
| A-10 | 欧拉角约定 | `φ=atan2(R23,R33)`、`θ=−asin(R13)`、`ψ=atan2(R12,R11)` | MOD-002 §4.1（锁定行为） | `euler_from_quat()` | 一致 | 与 `math.mjs:eulerFromQuat` 同式；注释写明"绕 +y 转 +a 读数 θ=−a" |
| A-11 | 欧拉域 SAS | `δt=δt0+kq q−kθΔθ−ki∫Δθ`、`Δω=kp p−kφφ−kiφ∫φ` | MOD-001 §8 | `simulate.SASController` | 一致 | 角速率项取正、姿态项取负，与"θ̇≈−q"成对；积分限幅 `intThMax/intPhiMax` |
| A-12 | 四元数 SAS | `τ=−K_q·ε−g(‖ε‖)K_w(ω−ω_ref)−K_i∫ε` | MOD-001 §8、`TandemVec-Paper/ch12` | `QuatSASController.update()` | 等价变换后一致 | ε≈φ/2 故 `K_q` 取 2 倍（0.8/0.6/0.8 对 0.4/0.3/0.4）；阻尼调度 `1+3·min(‖ε‖/0.15,1)` |
| A-13 | INDI 增量律 | `Δu=B⁻¹·I·(ν−ω̇_est)` | `TandemVec-Paper/ch10`、MOD-001 §4 | `QuatINDIController.update()`、`simulate.INDIController` | 一致 | `M_cmd_delta=I·(ν−ω̇)` 后 `solve(B,·)`；注释记录"缺 I 时滚转通道放大 ~10 倍" |
| A-14 | LQR | 悬停点线性化 `A=[0,½I;0,0]`、`B=I⁻¹B_true`，CARE + Bryson 权重 | `TandemVec-Paper/ch14` | `QuatLQRController` | 一致 | `x_max=[0.5,0.5,0.5,1,1,1]`、`u_max=[dwMax,dMax,dMax]`，Q/R 按 1/x²、1/u² |
| A-15 | 三方程配平 | `Fx=0,Fz=0,My=0`（δf=Δω=β=0） | MOD-003 §2.1（三方程版） | `trim_analysis.trim_longitudinal()` | 一致 | 残差按力/力矩平衡装配；Newton + 数值 Jacobian，限幅 `|α|≤0.3`、`ω0∈[50,wMax]` |
| A-16 | 四方程配平 | 另加 `Mz=a·Tf·sinδf−Q·sinδt=0` | MOD-003 §2.1（四方程版） | `scripts/trim_solve.residuals()` | 一致 | 与 MOD-003 §2.1 四式逐项相同；坐标下降 + 各变量单调二分，残差 <1e-12 |
| A-17 | 数值线性化 | 小扰动 A 矩阵；纵向 `[u,w,q,θ]`、横航向 `[v,p,r,φ]` | MOD-003 §4 | `trim_analysis.linearize_at_trim()` | 等价变换后一致 | θ 以 `q2` 作状态（θ≈−2q2）；实测两种坐标下特征值完全相同（相似变换不变量）：`−3.3647±5.5087j`、`−0.1703`、`+0.0177`（28 m/s） |
| A-18 | 模态映射 | `ω_n=|λ|`、`ζ=−σ/|λ|`、`T=2π/ω_d` | MOD-003 §4.2 | `analyze_modes()`、`export_latex.modes_table()` | 一致 | 共轭对去重后输出频率/阻尼/时间常数 |

### 2.2 15 态 EKF 研究仿真（`TandemVec-EKF-Report/sim`）

| ID | 模块/符号 | 理论要素 | 文献出处 | 代码位置（符号名） | 结论 | 证据 |
|----|-----------|----------|----------|--------------------|------|------|
| A-19 | 状态顺序 | `x=[δp(3),δv(3),δβ(3),δba(3),δbg(3)]` | 固件 `ekf_15_state.h` 头注释 0–14 | `ekf_core.Ekf15` | 一致 | 与固件索引顺序逐项相同；`P` 分块 0:3/3:6/6:9/9:12/12:15 对应 |
| A-20 | 误差姿态定义 | 体轴右乘 `q=q_nom⊗Exp(δβ^b)` | REF-121 §2.2.1 p.33、`notes/02` 附录 B | `Ekf15.inject()` | 一致 | `quat=quat_mult(quat,rotvec_to_quat(dx[6:9]))` 后归一化并 `w>0`；与固件 `InjectErrorState()` 的 `quat_=quat_*delta_quat_` 同构 |
| A-21 | 误差注入 | `δlat=δp_N/(R_m+h)`、`δlon=δp_E/((R_n+h)cosL)`、`δh=−δp_D` | REF-120 §8.1（位置速率） | `Ekf15.inject()` | 一致 | 与固件同式（同一 `Rns/Rew` 语义） |
| A-22 | 双子样圆锥/划摇 | `Δθ+⅔Δθ₁×Δθ₂`；`ΔV+(2/3)(Δθ₁×ΔV₂+ΔV₁×Δθ₂)` | REF-120 §9.3 (9.3.30) p.274、§9.6 (9.6.25) p.301 | `Ekf15.time_update()` | **偏差（已登记）** | 圆锥项与固件一致；划摇项**多加了 `0.5·Δθ×ΔV`**。固件 2026-10 P0-0 已删除该项（中点姿态 `t_b2ned_mid` 已含旋转效应，(9.6.17) 的 ½Δθ×ΔV 只配起始姿态用，见固件 `[公式 3]` 注释）→ 见 **P0-4** |
| A-23 | KF 五方程/Φ 离散 | `P⁺=ΦPΦᵀ+Q_d`，Φ=Ⅰ+F·dt+½(F·dt)² | REF-121 §2.2.1 (2.2.4d) p.33 | `Ekf15.time_update()` | 近似（可接受） | 二阶 Φ 与 `notes/02` §2 记录的固件飞行构建（FAST_PROPAGATION 走 ZOH Q_d）不同；量级 O(F·dt)≈1%（dt=5 ms） |
| A-24 | Q_d 离散 | Simpson：`(dt/6)(Q_c+4Φ½Q_cΦ½ᵀ+ΦQ_cΦᵀ)` | REF-121 §2.2.1 (2.2.4d) | `Ekf15.time_update()` | 一致 | `rw` 为 12 维驱动噪声谱密度（加计/陀螺白噪 + Markov 零偏 `2σ²/τ`），`Gs` 装配符号与 F 一致 |
| A-25 | Joseph 更新 | `P=(Ⅰ−KH)P⁻(Ⅰ−KH)ᵀ+KRKᵀ` | REF-121 §2.2.4 p.42–43 | `measurement_update_gnss/velocity/gravity/yaw()` | 一致 | 四条更新路径均用 Joseph；`static_gyro` 只更新零偏子块（对照固件） |
| A-26 | NIS 门控 | `λ=rᵀS⁻¹r ~ χ²(m)`，超门限拒绝 | REF-121 §7.3.1 (7.3.22) p.255–259 | 各 `measurement_update_*` | 一致（阈值见 A-28） | 6 维 GNSS NIS、3 维 ZUPT/重力、标量航向分别实现；门限常量与固件逐个核对 |
| A-27 | 地球模型/正常重力 | `R_n,R_m`、Somigliana + 高度修正、`ω_ie^n`、`ω_en^n`、`ḣ=−V_D` | REF-120 §7.2.4 (7.2.17)(7.2.18) p.178、§8.1 | `EarthModel` | 一致 | `g0=9.780318(1+5.3024e-3sin²L−5.898e-6sin²2L)`，`g=g0/(1+h/√(RnRm))²`；`@deviation` 已注明与完整级数差 ~1e-5 m/s² |
| A-28 | NIS 阈值 | GNSS 30 / 速度 16.3 / 重力 18 / 静止陀螺 24 / 姿态 18 / 航向 12 / 气压 16 | 固件 `ekf_15_state.h` 常量 | `Ekf15.__init__` 的 `nis_*` | 偏差（已登记） | 仿真 `nis_zupt` 默认 **1000**（旧值），固件 `VELOCITY_NIS_REJECT_THRESHOLD=16.3`；`opt_ekf` 候选基线用 `CUR_CFG={"nis_zupt":16.3}` 覆盖。其余 7 项逐一相符 |
| A-29 | 失联膨胀/重捕获 | 失联后协方差膨胀、重捕获窗口内膨胀 R 低增益融合 | `notes/02` §1.13（一致性）、AGENTS 2026-08-13 | `grow_outage_cov()`、`measurement_update_gnss()` | 一致 | `scale=min(t/2,cap)`、`reacq_nis2=80`、`reacq_window=6 s` 与固件常量同值 |
| A-30 | 静止辅助分频 | ZUPT 4 帧 / 重力 8 帧 / 静止陀螺 8 帧（200 Hz 基） | 固件 `navigation_task.cpp` | `ZUPT_DIV/GRAV_DIV/SG_DIV` | 一致 | 常量 4/8/8；由调用方 `run_scenarios` 按帧号取模 |
| A-31 | 静止检测四条件 | `‖a‖≈g`、`‖ω‖<ε`、`‖Δa‖<ε`、`‖Δω‖<ε` + 迟滞 | 固件 `ins_static_detector.h` | `opt_ekf.StaticDetector` | 一致 | `sd_a_lo=9.5/sd_a_hi=10.1`、`sd_g=0.01`、进入 20 帧/退出 5 帧；置信度=min(四子分, dwell) |
| A-32 | 三档静止辅助 | 档位由帧数与置信度决定，档越高噪声越小/分频越稀 | 固件 `ins_static_aid_profile.h` | `opt_ekf.StaticAid` | 一致 | tier1 门槛 36 帧、tier2 100 帧且 conf≥0.80；divs (4,8,8)/(8,16,16)/(16,32,32) |
| A-33 | 双矢量航向 H | 标量航向应投影到失准角行（6..8），倾斜段需 `C_b^n` 第三行 | `notes/04`、`notes/05` §2 | `opt_ekf`/`upgrade_candidates` 的 `yaw_h2_fix` | 偏差（已登记） | 旧实现 `H[0,7:10]` 错位（注入 +10° 观测航向变化 0.000°、Δba_x=+0.173）；修正版 H=6..8 得 +9.975°。AGENTS 2026-10-21 ③ 已记录 |
| A-34 | 候选基线口径 | 固件现值 = 静态门控 + P0 航向 0.2 rad + ZUPT NIS 16.3 | `notes/05` §1.1 | `upgrade_candidates.CUR_SPEC/CUR_CFG` | 一致 | `CUR_SPEC={"gnss":{"static_gate":True},"p0_hdg":0.2}`、`CUR_CFG={"nis_zupt":16.3}`；`--selftest` 要求与 baseline 差 <1e-9 |
| A-35 | 6DOF 闭环动力学 | 同 A-01…A-06 | MOD-001 §2–§5 | `hifi_core.Dynamics.step()` | 一致（积分器见 A-36） | `F_body/M_body/hv` 与 A-01/A-04/A-06 同式；悬停构型语义（上摆=roll、下摆=pitch、差速=yaw）注释与固件 mix 一致 |
| A-36 | 闭环积分格式 | 期望二阶，实为一阶半隐式欧拉 | MOD-001 §9（RK4 红线针对仿真 core） | `hifi_core.Dynamics.step()` | 偏差（已登记） | 代码中 `dt2=dt/2`、`dq` 为未使用的遗留变量；真值积分是 `v+=a·dt; ω+=ω̇·dt; q←q⊗Exp(ω·dt)`。dt=5 ms + τm=0.28 s 下稳定，但与 `core.py` 的 RK4 不是同一精度等级 |
| A-37 | 固件控制律复刻 | 误差四元数外环 + 角速率内环 + 惯量逆解 + B 对角分配 + 差速 √(1+Δω) | CTL-001 §4–§5、`FlightCtrlParams.h` | `hifi_core.FirmwareControl.step()` | 近似（可接受） | 分配只取 B 对角线（不含 BTRUE 非对角/τm 观测器），故闭环行为与固件同构但不等价；轴重排 `[ωz,ωy,ωx]`、`M_cmd←[yaw,pitch,roll]` 的 2026-10 修复已在代码中 |
| A-38 | pos-hold 体轴映射 | 北=pitch(y)、东=roll(z)（悬停构型） | 验收报告 §2b、AGENTS 2026-10-21 | `hifi/run_scenes_opt.SceneRunner` | 一致 | `rotvec=[0, sp·aN/g, sr·aE/g]` 后 `Q_HOVER⊗Exp(rotvec)`；注释明写"旧实现把东向塞进 x(偏航) 导致位置保持无效" |
| A-39 | 姿态/航向工具 | `ZYX` 欧拉↔四元数、`wrap_pi` | MOD-002 §4.1 | `quat_to_euler/quat_from_yp/wrap_pi`（ekf_core、hifi_core、opt_ekf） | 一致 | `pitch=−asin(C20)` 与固件 `quat2angle` 同式；`wrap_pi` 用 fmod 后再折回 (−π,π] |

### 2.3 脚本与工具（`scripts`、`tools`、`blender-test`、`py-web-lab`）

| ID | 模块/符号 | 理论要素 | 文献出处 | 代码位置（符号名） | 结论 | 证据 |
|----|-----------|----------|----------|--------------------|------|------|
| A-40 | MPC 原型 | 悬停点线性化 + 滚动时域 + 显式约束；Offset-free 扰动增广；NMPC 非线性预测 | `TandemVec-Paper/ch17`、MOD-003 §4 | `mpc_proto.LMPCAttitude`、`mpc_adv.OffsetFreeLMPC/NMPC` | 近似（可接受） | 预测模型用 `[ε,ω]` 线性化（小角度假设）；60° 扰动场景超线性化区，退化是预期对比项，不作实现缺陷 |
| A-41 | 在线辨识 B_eff | 单轴脉冲使 `g=ω×(Iω)+ω×h=0`，则 `ω̇=I⁻¹M` 可干净回归 | MOD-001 §4–§5、THY-003 §3 | `ident_probe`、`adaptive_mpc.RLS` | 近似（可接受） | 依赖"单轴转动"假设；差速通道用长脉冲避开 Jp 瞬态。实机需先做轴向解耦验证 |
| A-42 | 物理参数辨识 | 用实测 `wf/wt` 免去 τm 建模，残差只含 [kT,kQ,Ix,Iy,Iz] | THY-002 §2、MOD-001 §5.2、`ch16` | `ident_phys.resid()` | 一致 | 回归式 `ω̇=I⁻¹[M(wf,wt)−ω×(Iω)−ω×h]`；注释记录"用指令转速会把 τm 吸收进 kT/kQ"的失败模式 |
| A-43 | 配平落盘策略 | 配平结果不得自动写回参数源 | GOV-002/AGENTS 边界 | `scripts/trim_solve.main()` | 一致 | 只打印、不写文件（`grep write_text/open/dump` 无命中）；回写须人工评审 |
| A-44 | 参数一致性 | `parameters.mjs` 由 JSON 生成、禁止手改 | AGENTS 关键路径 | `tools/sync-params.render()`、`--check` | 一致 | `--check` 有漂移即退出码 1；当前工作区漂移来自 `models/aircraft-model.json` 的未提交改动（非本区域） |
| A-45 | 单文件构建 | ESM→tiny-AMD 依赖闭包内联，file:// 可直开 | N/A（构建工具） | `tools/build-standalone.build_bundle/build_html` | 一致 | 不支持的语法形式直接报错，不静默降级 |
| A-46 | 实验台控制律 | SAS 逐位对齐 `control.mjs`；INDI/LQR/ADRC 移植 | MOD-001 §8、`ch10/ch14/ch16` | `server/controllers.py` | 一致（1 项回归失败见 P1-4） | 交叉验证门槛 <1e-6；`tests/test_algorithms.py` 5/6 通过 |
| A-47 | 子步/帧步长 | 帧 delta≤0.05 s、子步≤0.004 s（红线） | 行为保持红线清单 | `server/sim.py`、`core.rk4_step` | 一致 | `n=max(1,ceil(dt/maxStep))`；`simulate.py/controllers.py` 默认 dt=0.004 |
| A-48 | 渲染≠物理 | 渲染力臂 ±1.78 m ≠ 物理 0.315 m | MOD-002 §6 | `browser/aircraft-view.mjs`、`blender-test/anim.py` | 一致 | 两处 L0 均写明"示意性非尺度模型"，禁止据图估算物理量 |

---

## 2.5 仿真口径 vs 固件口径（单列对照）

> 用途：防止把仿真结论直接外推实机。**左侧是代码里的现值，右侧是固件事实源**；凡不一致处均已在对应 L0/L1 注释里标注。

| # | 项目 | 高保真分析 / EKF 研究仿真 | 固件实机（事实源） | 差异影响 | 处置 |
|---|------|---------------------------|--------------------|----------|------|
| D-01 | 摆角限幅 `dMax` | 25° = 0.4363 rad（`models/aircraft-model.json`）；`hifi_core.P["dMax"]=0.2618`（15°） | ±15°（`TandemVec_Config.h`、`state_data.cpp` MAX_CORRECTION，2026-08-07 收紧） | 25° 口径下配平 `dtTrim=−18.702°` 看似有 6.3° 裕度；按 15° 重算则**超限 24.7%**，28 m/s 巡航配平点不可达 | 已登记（MOD-003 §3.2、ASM-017）；注释中点明"不可外推" |
| D-02 | 差速限幅 `dwMax` | ±0.7 | ±0.7（同值） | 无 | 一致 |
| D-03 | 重力加速度 | 动力学 `P["g"]=9.79`（模型 JSON MODEL-DEFAULT）；EKF `G_MPS2=9.80665` | 固件用 Somigliana 计算（`ComputeEarthModelTerms`） | 闭环内动力学与 EKF 的重力差 **0.17%**；悬停配平转速偏差 <0.1% | 已登记（A-27、hifi_core L0） |
| D-04 | P0 航向初值 | `ekf_core.cfg_p0["hdg"]=0.2`；`opt_ekf.FW["p0_hdg"]=0.2`（2026-10-23 对齐；旧 π 仅显式覆盖） | 0.2 rad（伪磁确定性初始化语义，2026-10-21 P0 修复） | 无；默认即固件口径 | 已修复（A-02） |
| D-05 | ZUPT/速度 NIS 门限 | `nis_zupt=1000`（统计失效：χ²(3) 99.9%≈16.3） | `VELOCITY_NIS_REJECT_THRESHOLD=16.3` | 门限 1000 使 ZUPT 几乎不拒，误静止时把速度强拉为零 | 已登记；`CUR_CFG={"nis_zupt":16.3}` |
| D-06 | 其余 NIS 门限 | GNSS 30 / 重力 18 / 静止陀螺 24 / 航向 12 / 气压 16 | 同值 | 无 | 一致 |
| D-07 | 气压更新率/相位 | 需 `--baro-hz 2 --baro-phase 2` 才稳（A2 定高） | 固件 baro 实际速率不同（BMP/DPS310 配置） | 仿真"稳定"依赖这两项口径；直接照搬实机会错配相位 | 已登记（AGENTS 2026-10-21） |
| D-08 | 高度环积分 `alt_vel.ki` | `hifi_core.CTRL["alt_vel_ki"]=1.25`（=固件现值）；候选 `--a2-alt-ki 0.5` | 1.25（`FlightCtrlParams.h`） | ki=1.25 时仿真 A2 seed2/3 出现 74.8/818.5 m 发散；0.5 仅仿真候选，**未回写** | 已登记（验收报告 §3.1） |
| D-09 | 水平位置环 | `--pos-hold` 是**仿真 P-D**（北=pitch、东=roll(z)）；默认闭环**无**水平位置环 | 固件位置环未完成/未验证 | 无 `--pos-hold` 时 A2/A3/A5 的 `pos_ne` 不能单独当失败判据 | 已登记；`run_full_envelope` L0 明写 |
| D-10 | 磁/光流辅助 | `--mag-vector`（理想三轴磁，NIS 10）、`--flow-aiding`（理想光流速度） | 真实磁需标定/门控；光流需尺度/延迟/失效处理 | 理想传感器结论不能宣称实机可用（磁不标定不上机：SC3 受扰磁 647 m） | 已登记（验收报告 §4、`notes/05` §7.1） |
| D-11 | 静止辅助陀螺门控 | 无油门门控（`StaticDetector` 只看四条件） | 2026-10 追加：`throttlePercent>=3%` 时跳过 static gyro 融合 | 仿真会把电机出力/缓慢漂移当成零角速率估零偏 | 已登记（AGENTS 2026-10-21 ⑥），仿真侧未建模 |
| D-12 | 闭环验收结论 | `run_full_envelope`（EKF 进控制）**10/10 FAIL**；`run_nav_envelope`（真值稳控）仅评导航链 | C++ 固件由 `test_host` + `pio run` 验证 | 两类结论不可互换：truth-stabilized 通过 ≠ 固件闭环通过 | 已在 L0 分别声明 |
| D-13 | 仿真积分器 | `core.py` RK4（红线）；`hifi_core` 一阶半隐式欧拉；`ekf_core` 二阶 Φ + Simpson Q_d | 固件：双子样 + 二阶 Φ + ZOH Q_d（FAST_PROPAGATION 构建） | 三套积分精度不同；跨文件比数值时必须先声明 | A-22/A-36 已标注 |
| D-14 | 速率环量纲 | `hifi_core.CTRL` 与固件同值（`rate_kp_rp=16.042818` deg/s→deg/s²） | 同（2026-08-11 统一量纲） | 研究脚本（`QuatSASController`）用无量纲 K_q/K_w，属不同口径 | A-12 已标注 |
| D-15 | 场景种子 | `run_scenarios`/`hifi` 固定 seed=1..N；`upgrade_candidates` 5 种子 | 实机单次 | 5 种子不足以定论发散概率（SC3 无磁 ±49 m）；关键结论需 ≥20 种子 + 实机数据 | `notes/05` §7.1 第 7 条 |
| D-16 | A8（yaw90）场景 | 目标定义不成立，单列不计分 | — | 该场景"通过/失败"均无意义 | 验收报告 §2、AGENTS 2026-10-21 |
| D-17 | F4 单前摆卡死 | 仿真 failsafe 把发散压成 72 m/102 m 有界下降 | `ActuatorFaultMonitor` 只读，控制响应未接入 | 滚转唯一主控失效是硬件单点，不可恢复 | `notes/08` |
| D-18 | 测试环境依赖 | `check_ctrl/prop/shutdown/spring/pw_verify` 硬编码本机 Playwright 路径 + 8090 端口 | — | 换机即失败，不能进 CI | P2-6 |

---

## 3 发现清单

| 严重度 | 位置（符号） | 问题 | 影响 | 建议 |
|--------|--------------|------|------|------|
| **P0-1** | `ekf_core.cfg_p0["hdg"]` / `opt_ekf.FW["p0_hdg"]` | 仿真默认 P0 航向仍为 **π**，与固件已修复的 0.2 rad 不一致 | 直接复现"航向弱可观踢动→双矢量 NIS 全拒→失锁"的旧失效链；任何用默认配置得出的航向/位置结论都偏悲观且不可外推 | **已修复（2026-10-23，A-02）**：两处默认值改为 0.2，π 仅保留为显式覆盖；回归 `verify_p0_fixes.py` |
| **P0-2** | `ekf_core.Ekf15.measurement_update_gnss()` | 依赖 `self.home_lla`，但 `initialize()` 不设置它 | 直接调用 GNSS 更新会 `AttributeError`（各 `run_*.py` 都手工赋值才不炸） | **已修复（2026-10-23，A-03）**：`initialize()` 用初始 LLA 复制到 `home_lla`，外部仍可覆盖；回归 `verify_p0_fixes.py` |
| **P0-3** | `trim_analysis.trim_longitudinal()`（25° 口径）与 `MOD-003 §3.2` | 三方程配平未闭合 `Mz`，且 `dtTrim` 在 15° 固件限幅下不可达 | 若把该配平点当巡航工作点直接用于实机/固件，摆角超限 | **已处置（2026-10-23，A-04）**：不改 dMax/vTrim；新增 `trim_solve.py --scan --dmax-deg 15`，给出 15° 可行下限约 29.8 m/s（30–40 m/s 可行），写入 MOD-003 §3.2 / ASM-017 |
| **P0-4** | `ekf_core.Ekf15.time_update()`（`opt_ekf` 继承） | 划摇增量里保留 `0.5·Δθ×ΔV`，而速度又用**中点姿态** `Cnb_mid` 旋转 → 旋转效应**双计**（固件 2026-10 P0-0 已删除两处 ½Δθ×ΔV） | 每步多算 ~½\|ω×f\|·dt（2 rad/s、10 m/s²、5 ms 时 1e-2–5e-2 m/s²）；动态段速度估计有方向性偏置。凡用本仿真论证"与固件逐式一致"的结论都需重新评估（`notes/05`、优化研究报告的速度/位置数字） | **已修复（2026-10-23，A-01）**：删除 `0.5·Δθ×ΔV`，统一走 `two_sample_delta_v()`；回归 `verify_p0_fixes.py` 量化单步多算量 ½\|ω×f\|dt² |
| **P1-1** | `ekf_core.Ekf15.time_update()` | `F[6:9,6:9]` 先赋 `−skew(Cnb·0+bg)` 后又被 `−skew(g_body)` 覆盖；`a_est` 为占位死代码（`self.vel*0`） | 读者易误判姿态自旋项来源；死代码掩盖真实线性化点 | 删除死代码/占位（属行为无变化的清理，建议单独提交）；注释已记录正确语义 |
| **P1-2** | `hifi_core.Dynamics.step()` | 注释称 "RK2 半步"，实为一阶半隐式欧拉；`dt2`/`dq` 为未使用遗留 | 与 `core.py`（RK4）比精度时会被误当作同阶方法 | 已在 `@deviation` 中登记；建议后续改注释或删遗留变量（不改数值需单独评审） |
| **P1-3** | `hifi_core.Dynamics.imu_measure()` | `self._dt_prev = 0.005` 在用过之后被硬写，加速度差分始终按 5 ms 计 | 若调用方用 200 Hz 以外步长，比力被按比例缩放（20% 量级误差） | 建议改为传入实际 dt；当前 200 Hz 路径无影响 |
| **P1-4** | `py-web-lab/tests/test_algorithms.py` | `[FAIL] INDI 俯仰姿态指令 5°：末点 ‖ω‖max=0.1297 rad/s`（门限 0.05） | 5/6 通过；INDI 在姿态指令模式下残差超阈 | 属既存失败（本轮仅改注释）。需定位 INDI 外环在 vtol 模式的目标速率限幅/积分分离 |
| **P1-5** | `run_full_envelope` 默认闭环 | 默认无水平位置环，真值稳控下 10 s 仍北漂 77 m（本轮冒烟实测） | 不启用 `--pos-hold` 时的位置指标无意义 | 已在 L0 声明；建议把默认行为或用例名显式化（如 `A2-nopos`） |
| **P1-6** | `upgrade_candidates` 单次套件 | 5 种子/格 | SC3 无磁种子方差 ±49 m，发散概率无法定论 | 已在 L0 与 `notes/05` §7.1 声明"关键结论需 ≥20 种子" |
| **P2-1** | `simulations/high-fidelity-analysis/export_latex.py`、`cascade_extended.py` | 8 处 `SyntaxWarning: invalid escape sequence`（`'\,'`、`'\o'`），位于 f-string/标题 | 不影响运行，但 Python 3.12 编译噪声大；`'\o'` 属笔误 | 建议改 raw string（`r"$\,$"`）；本轮未改（避免行为/输出文本变化） |
| **P2-2** | `trim_analysis.linearize_at_trim()` docstring | 原注释称 A(12×12)，实际 13×13、状态 0–12 | 读者对照易错 | 已在本轮 L0 更正并写明状态编号 |
| **P2-3** | `hifi_core.imu_measure()` / `opt_ekf` 注释 | 悬挂 `# 0.005` 等实现细节无单位说明 | 可维护性 | 已补 `@units` |
| **P2-4** | 各文件原 docstring 缺失/无 `@file` | 79 文件无 L0（本轮前 85 error） | 门禁不通过、可读性差 | 本轮补齐（见 §5） |
| **P2-5** | `blender-test/anim.py` | 宏 `bpy` 依赖，无法在 CPython 下 import（无离线校验路径） | 渲染脚本只能人工跑 | L0 已注明"不能在 CPython import" |
| **P2-6** | `py-web-lab/tests/check_*.py`、`pw_verify.py` | 硬编码本机 Playwright Chromium 绝对路径 + 8090 端口 | 换机/CI 直接失败 | 建议抽 `EXE` 到环境变量或 `--exe` 参数 |

---

## 4 未决问题

1. **EKF 进控制闭环 10/10 FAIL**（`run_full_envelope`）尚未定位到单一根因；`run_nav_envelope` 的通过只能说明导航链本身可接受。需要把"导航噪声→控制振荡"与"控制器本身"分离（已有 `truth_rate`/`yaw_free` 等诊断开关，尚未系统跑矩阵）。
2. **`alt_vel.ki=1.25` 在仿真低速段不稳定**，而固件现值就是 1.25：是仿真口径问题（baro 相位/高度源）还是固件真实风险，需实机定高数据判定。
3. **静止检测的油门门控**（固件 2026-10 追加）在仿真侧完全缺失；若要在仿真里评估该项收益，需在 `StaticDetector` 增加 throttle 输入（会改变现有结果，须单独评审）。
4. **`F[6:9,6:9]` 的正确形式**：当前 `−skew(g_body)`（用去偏角增量）与固件 `−[ω×]` 的一致性只在"去偏后 ω≈真值"时成立；误差状态方程里严格应为 `−[ω̂×]`。属理论细节，尚未与固件逐式复核。
5. **`hifi_core` 控制分配只用 B 对角**：与固件 `AirframeModel` 数据驱动 B（含非对角）不等价；若要复现固件调参结论，需换成完整 B。
6. **未跟踪文件**：`sim/hifi/run_full_envelope.py` 等 6 个文件尚未纳入 git，`git diff` 看不到其历史；建议由 Lead 统一提交。
7. **`models/aircraft-model.json` 处于未提交改动状态**，`tools/sync-params.py --check` 因此报漂移；`standalone.html` 同步报漂移。两者都不属于本区域改动，需由参数/构建负责人重新生成后再跑门禁。

---

## 5 门禁与测试

### 5.1 注释门禁（GOV-004）

```text
# 核查前（本轮开始，--area analysis）
范围: 80 文件 | 平均分: 65.4 | error: 85 | warning: 214
文件: 有 error 79 | 仅 warning 0 | 全绿 1
规则命中: R01=79  R07=6  R08=152  R09=62

# 核查后
范围: 80 文件 | 平均分: 97.2 | error: 0 | warning: 45
文件: 有 error 0 | 仅 warning 23 | 全绿 57
规则命中: R08=45
```

- **error 85 → 0**（R01 缺 L0 头 79 项、R07 理论文件无 `@ref` 6 项全部清零），R09（缺 `@details/@frame/@units`）62 → 0。
- warning 214 → 45（−79%）：R08（公共符号缺 docstring）152 → 45。剩余 45 项集中在 `tools/build-docs.py`、`tools/check-links.py`（Lead 文件，7 项）与长脚本的 `main()`/绘图辅助函数。
- 命令：`py -3.12 tools/check-code-comments.py --area analysis --check` → **退出码 0**。

### 5.2 语法与冒烟

| 检查 | 命令 | 结果 |
|------|------|------|
| 语法（全部改动 .py） | `py -3.12 -m py_compile <64 文件>` | 全部通过（0 失败） |
| 前端语法 | `node --check web/src/main.mjs`、`browser/hud.mjs` | 通过 |
| 高保真时域仿真 | `core.load_params` + `trim_longitudinal(28)` + `simulate(SAS, 0.5 s)` | 配平 `α=0.071955 rad`、`ω0=268.595 rad/s`、`δt=−0.221088 rad`；125 步，末点 `θ=−0.074041 rad`（无异常） |
| EKF 传播 | `Ekf15(cfg).initialize(...)` + 400 步 `time_update` | `trace(P)=2.387e+01`，`nis_gnss=30.0`，`lla.alt` 收敛正常，无 NaN |
| EKF 优化框架 | `opt_ekf.StaticDetector/StaticAid/build_p0` | 静止 30 帧确认（`confirmed=True`，tier1，divs (4,8,8)）；`build_p0()["hdg"]=π`、`build_cfg()["nis_gnss"]=30.0` |
| 固件语义闭环 | `hifi_core.Dynamics+FirmwareControl`（真值导航，10 s，dt=5 ms） | 高度 9.91 m（定高 10 m）、姿态保持 90°，北向漂 77.6 m（无水平位置环，符合 P1-5） |
| 实验台算法回归 | `cd simulations/py-web-lab && py -3.12 tests/test_algorithms.py` | **5/6 通过**，失败项见 P1-4（既存） |
| 参数同步 | `py -3.12 tools/sync-params.py --check` | **通过**（2026-10-23 重建后） |
| 单文件构建 | `py -3.12 tools/build-standalone.py --check` | **通过**（2026-10-23 重建后） |

### 5.3 行为保持自查

- `git diff` 全量为**注释/文档字符串增改**；逐文件核对 diff 只含 `"""…"""`、`@tag` 行与空行，无逻辑行增删。
- AST 比对（剥离 docstring 后 `ast.dump`）：61 个改动 `.py` 中 52 个与 HEAD **完全等价**；8 个差异文件的差异**不含任何注释/docstring**，均为**本轮之前就存在的未提交改动**（`MAG_NED_UT`/`self.fault`/`gravity_fast`/`use_static_gyro`/`OUT` 从 `docs/03-理论推导/THY-004` 迁到 `TandemVec-Paper`），与本次核查无关；`tests/repro_exit.py` 的差异是**移除文件头 BOM**（HEAD 侧带 BOM 无法被 `ast.parse` 解析），属空白字符修正。
- 未修改 `models/aircraft-model.json`、`parameters.mjs`、`standalone.html`、`tools/check-code-comments.py`、`tools/build-docs.py`、`tools/check-links.py`、`docs/` 下任何 Markdown（本文除外）。
- 未执行任何 `git commit/add/checkout/reset/stash/clean`。
- 随机种子、参数、算法、数值结果未改（配平/特征值/冒烟数值与核查前一致）。

---

## 6 备注：本轮新增的关键注释点（便于复查）

1. 每个文件头统一 `@file/@module/@brief/@details/@theory/@frame/@units/@note/@see`，理论文件另加 `@redline`。
2. `core.py`、`controllers.py`、`estimators.py`、`ekf_core.py`、`hifi_core.py`、`opt_ekf.py` 的每个公共符号补 L1（含 `@formula`/`@ref`/`@note`/`@deviation`）。
3. `ekf_core` 的 L0 明确列出固件参数块（IMU std、R 地板、NIS 门限、分频、Φ/Q_d、Joseph、重捕获）并标注 P0 航向与固件不一致。
4. `hifi_core` 的 L0 明确 15° vs 25° 摆角口径、g=9.79 vs 9.80665、以及"Python 固件语义闭环 ≠ C++ HIL"。
5. `run_full_envelope`/`run_nav_envelope`/`run_firmware_suite` 三者 L0 互相区分"EKF 进控制闭环 / 导航在环 / 控制器故障矩阵"，防止结论串用。
6. `trim_solve` 的 L0 记录"15° 限幅下 28 m/s 配平不可达"，`trim_analysis` 的 L0 记录"三方程参考点、非全配平"。
7. `ekf_core.time_update()` 的 `@deviation` 记录划摇旋转效应双计（P0-4），并写明"改数值属行为变更"，让后续读者不会误以为仿真与固件逐式一致。

---

## 7 P0 修复与复验（2026-10-23 追加，决策见 DEC-002）

| 项 | 修复内容 | 验证 |
|----|----------|------|
| A-01 | 删除 `0.5·Δθ×ΔV`，新增 `two_sample_delta_v()` 统一双子样增量；`time_update()` 的 `@deviation` 改为对齐说明 | `verify_p0_fixes.py`：新实现=ΔV、旧实现多算量=½\|ω×f\|dt²（1e-12 量级吻合） |
| A-02 | `cfg_p0["hdg"]` 与 `opt_ekf.FW["p0_hdg"]` 改为 0.2 rad；π 仅显式覆盖 | `verify_p0_fixes.py` 断言两处均 0.2；20 s 场景冒烟 `pos_err≈0.10/0.05 m`、NIS≈4.63 |
| A-03 | `initialize()` 设置 `home_lla = lla.copy()`，外部仍可覆盖 | `verify_p0_fixes.py`：初始化后零残差 GNSS 更新可执行、NIS≈0 |
| A-04 | `trim_solve.py` 新增 `--dmax-deg`/`--scan`；MOD-003 §3.2 与 ASM-017 补 15° 可行包线 | `py -3.12 scripts/trim_solve.py --scan --dmax-deg 15` → 可行区间 30.00–40.00 m/s，阈值约 29.8 m/s |

> 影响说明：A-01/A-02 会改变研究仿真的默认数值口径（此前用 π 或双计旋转项跑出的速度/位置结论需按新口径复核）；A-03 只补默认值，不改变显式设置 `home_lla` 的脚本；A-04 只改分析工具与文档，不动 `aircraft-model.json` / `vTrim` / `dMax`。
