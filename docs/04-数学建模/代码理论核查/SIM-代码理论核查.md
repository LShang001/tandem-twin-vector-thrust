# 代码理论核查记录 — sim

> 核查日期：2026-10-23 ｜ 执行：sim-core（Agent Teams task-1）｜ 规范：[GOV-004](../../00-项目治理/GOV-004-代码注释与理论对照规范.md)
> 代码基线：`6784221`（工作区含未提交改动；sim 区域核查前在 git 中是**干净**的，本记录全部改动均为本轮新增注释/文档）
> 理论来源：MOD-001 §2–§9；MOD-002 §2–§6；MOD-003 §2；THY-001 §6–§9；THY-002 §3–§6；THY-003 §1–§3；CTL-001 §3–§8；`TandemVec-Paper/ch02-propulsion`–`ch05-allocation`；AGENTS.md（执行器轴映射、速率环量纲、LMI 外推红线）
> 方法：逐文件读代码 → 对照公式与符号 → Node 数值复算（四元数等价、重力变换、B 矩阵差分、收敛阶）→ 补 L0/L1/L2 与五段式 → `check-code-comments.py --area sim --check` 复检 → `node --test` 全量回归。

---

## 1 范围与覆盖率

范围：`simulations/vector-thrust-lab/**` 下全部自研 `.mjs/.js/.py`（排除 `vendor/three-r170/**`、生成物 `parameters.mjs`、构建产物 `docs/*.html`），共 **38 文件**。

| 文件 | 行数 | L0 文件头 | L1 符号文档 | `@ref` 引用 | 备注 |
|------|------|-----------|-------------|-------------|------|
| `src/core/math.mjs` | 140 | 有 | 有（8 个导出全部） | 13 | R07 理论关键；四元数/欧拉角五段式 |
| `src/core/state.mjs` | 208 | 有 | 有（6 个导出） | 8 | R07；轴语义双读法、三种复位语义 |
| `src/core/control.mjs` | 224 | 有 | 有（SAS + 悬停律） | 15 | R07；SAS 五段式、定高/B_true 分支注释 |
| `src/core/dynamics.mjs` | 269 | 有 | 有（调度/单步/导数/RK4） | 16 | R07；牛顿–欧拉与 RK4 五段式 |
| `src/core/propulsion.mjs` | 70 | 有 | 有（stepPropulsion） | 9 | R07；差速分配 + 六维映射五段式 |
| `src/core/aerodynamics.mjs` | 80 | 有 | 有（2 个导出） | 8 | R07；风轴→机体五段式 |
| `src/core/control-allocation.mjs` | 75 | 有 | 有（2 个导出） | 9 | R07；B 矩阵五段式 |
| `src/core/telemetry.mjs` | 42 | 有 | 有 | 1 | R07 豁免后仍补 `@ref MOD-002 §8` |
| `src/core/parameters.mjs` | 82 | — | — | 0 | **生成物，只读核对**（见 P1-1），未改动 |
| `src/browser/aircraft-view.mjs` | 315 | 有 | 有（2 个导出） | 3 | 修正"力臂 0.62 m"腐烂注释（P2-8） |
| `src/browser/controls-ui.mjs` | 215 | 有 | 有 | 2 | 滑块语义/量纲切换说明 |
| `src/browser/effects.mjs` | 134 | 有 | 有 | 1 | 载机跟随系渲染口径 |
| `src/browser/hud.mjs` | 91 | 有 | 有 | 1 | 显示契约（键名/量程） |
| `src/browser/scene.mjs` | 75 | 有 | 有 | 0 | 相机 up=−z 的原因 |
| `src/browser/scope.mjs` | 145 | 有 | 有 | 1 | 采样/绘制节流的理由 |
| `src/browser/theme.mjs` | 59 | 有 | 有 | 0 | 主题只改渲染参数 |
| `src/browser/demo.mjs` | 70 | 有 | 有 | 1 | 演示激励幅值口径 |
| `src/main.js` | 72 | 有 | 装配层 | 1 | 帧截断红线、首帧执行器初值 |
| `tests/math.test.mjs` | 78 | 有 | 覆盖表 + 容差依据 | 2 | |
| `tests/control.test.mjs` | 190 | 有 | 覆盖表 | 2 | |
| `tests/control-allocation.test.mjs` | 124 | 有 | 覆盖表 + 容差依据 | 3 | |
| `tests/dynamics.test.mjs` | 136 | 有 | 覆盖表 + 容差依据 | 4 | |
| `tests/aerodynamics.test.mjs` | 81 | 有 | 覆盖表 | 1 | |
| `tests/propulsion.test.mjs` | 81 | 有 | 覆盖表 + 容差依据 | 3 | |
| `tests/state.test.mjs` | 103 | 有 | 覆盖表 | 2 | |
| `tests/telemetry.test.mjs` | 60 | 有 | 覆盖表 | 1 | |
| `tests/regression.test.mjs` | 46 | 有 | 容差 1e-9 依据 | 2 | |
| `tests/parameters.test.mjs` | 70 | 有 | 覆盖表 | 2 | |
| `tests/vtol.test.mjs` | 499 | 有 | 覆盖表 + 容差依据 | 3 | 31 用例，五个分组 |
| `tests/scenarios.mjs` | 69 | 有 | 运行器语义 | 0 | 非 test 文件 |
| `tests/fixtures/generate.mjs` | 35 | 有 | 红线流程 | 1 | 基线生成器 |
| `scripts/cascade_closedform.py` | 262 | 有 | 8 个函数 docstring | 2 | 符号/数值闭式 |
| `scripts/phase_margin_analysis.py` | 409 | 有 | 5 个公共符号 | 6 | R08 三处已修 |
| `scripts/pm_sensitivity.py` | 131 | 有 | 2 个公共符号 | 2 | 三方带宽口径 |
| `scripts/pm_observer_compensation.py` | 119 | 有 | 1 个公共符号 | 3 | 观测器补偿量化 |
| `scripts/pm_2d_sweep.py` | 171 | 有 | 2 个公共符号 | 3 | 二维扫描 |
| `scripts/robust_lmi_analysis.py` | 291 | 有 | 6 个公共符号 | 2 | Petersen LMI |
| `scripts/robust_montecarlo.py` | 224 | 有 | 4 个公共符号 | 1 | 蒙特卡洛验证 |
| `scripts/run_all.py` | 121 | 有 | 入口 docstring | 0 | 编排脚本 |

合计：**38 文件 / 5666 行**（核查前 4824 行，本轮净增 842 行注释与文档）；L0 覆盖 38/38；L1 覆盖全部 JS 导出符号（32 个）与 Python 公共函数/入口（机检 R08 命中项已全部清零）；`@ref` 共 **134 处**（本轮新增 110 行引用）。

本轮 diff：38 文件 / +1057 −215 行；其中注释与 docstring 增行 1008 行，其余 49 行为 docstring 散文行与 2 处 Python 多行签名单行化（`augmented_A`/`discrete_poles`，仅为满足机检 R08 的 docstring 3 行窗口，**参数、默认值、语句顺序与行为均未变**）。

---

## 2 核对矩阵

结论枚举：`一致` / `等价变换后一致` / `近似（可接受）` / `偏差（已登记）` / `不一致（待修）`。

### 2.1 姿态数学（A）

| ID | 模块/文件 | 理论要素或公式 | 文献出处 | 代码位置（符号名） | 结论 | 证据 | 处置 |
|----|-----------|----------------|----------|--------------------|------|------|------|
| A-01 | `math.mjs` | 四元数 Hamilton 乘 `a⊗b` 分量式 | THY-001 §6 | `quatMultiply()` | 一致 | 与 THY-001 §6 展开式逐项相同；复合顺序数值验证 1.60e-15 | 补 `@formula`/`@ref` |
| A-02 | `math.mjs` | 三明治积 `v'=q⊗ṽ⊗q*` | THY-001 §6 | `rotateVecByQuat()` | 等价变换后一致 | 2000 随机 `(v,q)`：与 `quatMultiply(q, v, q*)` 最大差 **3.11e-15**；与罗德里格斯式同源 | 五段式写明 `t=2(q×v)` 展开 |
| A-03 | `math.mjs` | 单位四元数逆 = 共轭 | THY-001 §6 | `quatInvert()` | 一致 | `q⊗q⁻¹=1` 单测 1e-9 | 补 `@pre`（仅单位范数成立） |
| A-04 | `math.mjs` | 归一化/保范投影、零四元数回退 | THY-001 §9 | `quatNormalize()` | 一致 | 单测：非零范数=1、零输入回退 `(0,0,0,1)` | 补 `@note` 零回退理由 |
| A-05 | `math.mjs` | 3-2-1 欧拉角 `phi=atan2(R23,R33)`、`theta=−asin(R13)`、`psi=atan2(R12,R11)` | MOD-002 §4.1；THY-001 §8 | `eulerFromQuat()` | 一致（公式层） | 2000 样本与"标准 ZYX 分解作用于 `R^T`"最大差 **1.8e-14** | 补公式与 clamp 说明 |
| A-06 | `math.mjs` | 三角度符号语义（"显示角"） | MOD-002 §4.2 | `eulerFromQuat()` | 偏差（已登记 P2-1） | 单轴正转 a → φ/θ/ψ 全为 −a；复合 `(φ,θ,ψ)=(0.2,0.3,−0.5)` → 代码读数 `(−0.3228,−0.1596,0.5564)`，**不是**逐分量取负 | 不改代码（行为红线），登记文档缺口 |
| A-07 | `math.mjs` | `q` 为机体→惯性系、右乘运动学 | MOD-002 §3.2/§3.4；THY-001 §7 | `dynamics.mjs:stateDerivatives()` 的 `qDot` | 一致 | `quatMultiply(q, {0.5ω,0})`，与 THY-001 §7 右乘式一致 | 补五段式 |

### 2.2 六自由度动力学与积分（B）

| ID | 模块/文件 | 理论要素或公式 | 文献出处 | 代码位置（符号名） | 结论 | 证据 | 处置 |
|----|-----------|----------------|----------|--------------------|------|------|------|
| B-01 | `dynamics.mjs` | 重力变换 `g_b=R(q)ᵀ·(0,0,g)` | MOD-001 §5.1；THY-003 §1.1 | `stateDerivatives()` 的 `gb` | 一致 | 2000 样本与 `Rᵀg` 最大差 **4.44e-15**；`|g_b|=g` | 补 `@note`（z 向下为正） |
| B-02 | `dynamics.mjs` | 平动 `m·v̇=F_thrust+F_aero+m·g_b−m(ω×v)` | MOD-001 §5.1；THY-003 §1.1 | `stateDerivatives()` 的 `vDot` | 一致 | 逐项对照；`−ω×v` 为动系绝对导数必然项 | 已补五段式 |
| B-03 | `dynamics.mjs` | 转动 `I·ω̇=M−ω×(I·ω)−ω×h` | MOD-001 §5.2；THY-003 §1.2 | `stateDerivatives()` 的 `wDot` | 一致 | 主轴展开式 `(Iz−Iy)qr` 等逐项对应；`h` 见 B-04 | 已补五段式 |
| B-04 | `dynamics.mjs` | 双转子角动量 `h=[Jp(ωf·cf−ωt·ct), Jp·ωf·sf, Jp·ωt·st]` | MOD-001 §3.4；THY-002 §5 | `physicsStep()` 的 `hv` | 一致 | 与 MOD-001 §3.4 矢量式逐分量相同（含尾桨反号） | 补 `@ref` |
| B-05 | `dynamics.mjs` | 子步调度 `n=max(1,ceil(dt/maxStep))` | MOD-001 §9 | `stepPhysics()` | 一致 | 单测：与手动 n 子步 `deepEqual` 完全一致；保证 `h≤4 ms` | 补五段式 |
| B-06 | `dynamics.mjs` | 经典 RK4 权重与每阶段保范 | THY-001 §9 | `rk4Step()` | 一致 | 权重 `1/6,2/6,2/6,1/6`；每阶段 `quatNormalize`；600 帧机动后 `|q|−1<1e-9` | 已补 `@note` |
| B-07 | `dynamics.mjs` | 整体离散精度阶 | 数值积分理论 | `physicsStep()`（RK4 + 显式欧拉混用） | 近似（可接受，P2-2） | 端到端收敛阶估测 **1.53 / 1.38 / 1.74**（T=0.2 s，n=4/8/16/32 vs n=6400 参考）：被电机转速一阶滞后与 SAS 积分器的显式欧拉限制为一阶 | 注释写明"RK4 仅作用于 [v,ω,q]" |
| B-08 | `dynamics.mjs` | 地面约束 `pos.z>groundZ 且 vWorld.z>0` | MOD-002 §5.2 | `physicsStep()` 尾部 | 一致 | 单测：触地钳位+清下降速度（机体系速度残差 <1e-5，来自步内气动微扰）；上升不拦截 | 补 `@note` 触发条件 |
| B-09 | `dynamics.mjs` | 位置积分用末速度、`lockXY` 语义 | MOD-001 §9（载机跟随系） | `physicsStep()` 的 `vw`/`F.pos` | 一致（工程约定） | 单测：lockXY 水平速度/位置不变、垂向照常 | 补 `@note`（非悬停模型） |

### 2.3 推进系统（C）

| ID | 模块/文件 | 理论要素或公式 | 文献出处 | 代码位置（符号名） | 结论 | 证据 | 处置 |
|----|-----------|----------------|----------|--------------------|------|------|------|
| C-01 | `propulsion.mjs` | 差速开方分配 `ωf=ω0√(1+Δω)`、`ωt=ω0√(1−Δω)` | CTL-001 §7.1；MOD-001 §3.2 | `stepPropulsion()` 的 `wfT/wtT` | 一致 | `ωf²+ωt²=2ω0²` 数值精确（952200 vs 952200）；`max(0,·)` 兜底 | 红线注释 |
| C-02 | `propulsion.mjs` | 推力/反扭矩 `T=kT·ω²`、`τ=kQ·ω²+Jp·ω̇` | THY-002 §3/§6 | `stepPropulsion()` 的 `Tf/Tt/Qf/Qt` | 一致 | 单测稳态 T=kT·ω²（1e-6）；反扭矩含 `Jp·ω̇` 项，与 THY-002"机体承受电磁扭矩"结论一致 | 五段式写明 |
| C-03 | `propulsion.mjs` | 一阶滞后 `τm·ω̇+ω=ω_cmd` | THY-002 §4 | `stepPropulsion()` 的 `wfT→S.wf` | 近似（可接受，P2-3） | 100 个 τm/100 小步后达 63.2%（0.5%·wMax 内）；显式欧拉截断 O((dt/τm)²)≈7e-3 @4 ms | `@approx` 标注 |
| C-04 | `propulsion.mjs` | 六维映射 `Fx/Fy/Fz、Mx/My/Mz` | MOD-001 §4 | `stepPropulsion()` 尾部 | 一致 | 与 MOD-001 §4 六式逐项相同；`df>0→Fy>0,Mz>0`、`dt>0→Fz<0,My<0` 单测锁定；零摆角等转速 Mx≈0（1e-6） | 已补 `@formula` |
| C-05 | `propulsion.mjs` | 差速对总推力一阶不变 | CTL-001 §7.2–7.3 | `stepPropulsion()` 的 `Fx` | 等价变换后一致 | 由 `Tf=T0(1+Δω)` 精确线性（√ 分配平方后）⇒ 小摆角下 `Fx` 一阶不变；钳位 `wMax` 后失效（文档已注明） | 补 `@note` |
| C-06 | `propulsion.mjs` | 目标转速用指令油门、力矩用实际转速 | 固件对照（AGENTS 差速增益调度） | `stepPropulsion()` | 一致（仿真口径） | 仿真内 B 用指令油门（P1-2），力矩用实际转速——与固件"修复前"口径相同 | 见 P1-2 |

### 2.4 气动（D）

| ID | 模块/文件 | 理论要素或公式 | 文献出处 | 代码位置（符号名） | 结论 | 证据 | 处置 |
|----|-----------|----------------|----------|--------------------|------|------|------|
| D-01 | `aerodynamics.mjs` | 动压 `q̄=½ρV²`、`L=q̄S·CLa·α`、`D=q̄S(CD0+CDk·CL²)`、`Y=q̄S·CYb·β` | MOD-001 §6 | `computeAeroState()` | 一致 | 单测 aX 与解析式差 <1e-9；α>0 时 aZ<0（升力沿 −z_b） | 五段式 |
| D-02 | `aerodynamics.mjs` | 气动矩 `Clb/Clp`、`Cm0/Cma/Cmq`、`Cnb/Cnr` + 无量纲角速率 | MOD-001 §6 | `computeAeroState()` 的 `Mx/My/Mz` | 一致 | 与 MOD-001 §6 三式逐项相同；Cmq<0 阻尼、Cnb>0/Clb<0 符号单测 | 补 `@ref` |
| D-03 | `aerodynamics.mjs` | 风轴→机体投影 `aX=L·sinα−D·cosα`、`aZ=−L·cosα−D·sinα` | THY-003 §1 | `computeAeroState()` | 一致 | 解析式比对 1e-9；α→0 时退化 `aX≈−D, aZ≈−L` | 五段式 |
| D-04 | `aerodynamics.mjs` | 空速下限 `vMin=0.5 m/s` 仅作分母保护 | MOD-001 §6 注；行为红线 | `computeAeroState()` 的 `vSafe` | 一致 | 单测：V=0 时 `q̄=0`、力/力矩全零、`qbar` 有限——保护速度不造力 | 已补 `@note` |
| D-05 | `aerodynamics.mjs` | 迎角 `α=atan2(w,u)` 无低速保护 | MOD-001 §11（线性小迎角模型边界） | `computeAeroState()` 的 `al` | 近似（可接受，P2-5） | `u→0` 时 α→±90°，线性 `CL=CLa·α` 失效；悬停默认 `aero=false` 规避 | 注释写明适用边界 |

### 2.5 控制分配（E）

| ID | 模块/文件 | 理论要素或公式 | 文献出处 | 代码位置（符号名） | 结论 | 证据 | 处置 |
|----|-----------|----------------|----------|--------------------|------|------|------|
| E-01 | `control-allocation.mjs` | 力矩映射 `M=Σ[r×F+s·τ·n]`、摆角雅可比 | THY-003 §2.1–2.2 | `computeBTrue()` | 一致 | 解析偏导与中心差分最大差 **2.38e-9**（测试阈值 1e-6） | 五段式 |
| E-02 | `control-allocation.mjs` | 行/列序 `[Mx,My,Mz]`×`[Δdw,Δδt,Δδf]` | CTL-001 §8.5；固件三端同构 | `computeBTrue()` | 一致 | 与 `TandemVec_ControlAllocation.h`、Python `control_effectiveness` 同构；测试抽查 3 元素 1e-12 | 红线注释 |
| E-03 | `control-allocation.mjs` | `∂Mx/∂Δω=−2·kQ·ω0²`（零摆角） | CTL-001 §7.4 | `computeBTrue()[0][0]` | 等价变换后一致 | `B[0][0]=−tau0(cf+ct)`，零摆角退化为 `−2kQω0²` | 补 `@note` |
| E-04 | `control-allocation.mjs` | 增量分配 `Δu=B⁻¹(M_des−M_cur)`、奇异回退 | THY-003 §3.2；CTL-001 §8.5 | `inv3()`、`applyVtolHover()` | 一致 | `B⁻¹B=I`（1e-9）；低油门 `det≈0` 返回 null；ΔM 复现 1e-4 | 补 `@note` |
| E-05 | `control-allocation.mjs` | B 为准稳态（忽略 `Jp·ω̇`、气动、陀螺） | CTL-001 §8.5 口径 | `computeBTrue()` | 近似（可接受，P2-4） | `M_cur` 含 `Jp·ω̇` 而 B 不含 ⇒ 分配层存在一阶模型失配；小增量下可接受 | 注释写明忽略项 |
| E-06 | `control.mjs` | B 工作点用 `S.thr·wMax`（指令） | 固件对照 | `applyVtolHover()` 的 `computeBTrue` 调用 | 偏差（已登记 P1-2） | 固件自 2026-08-09 起用 τm 观测器 `w_est`；仿真仍用指令油门 ⇒ 油门瞬态有效增益偏差 ∝(w_act/w_cmd)²（峰值 ~1.33 → ~76%） | 不改行为，登记 |

### 2.6 SAS 与悬停控制（F）

| ID | 模块/文件 | 理论要素或公式 | 文献出处 | 代码位置（符号名） | 结论 | 证据 | 处置 |
|----|-----------|----------------|----------|--------------------|------|------|------|
| F-01 | `control.mjs` | SAS 三通道律（阻尼+比例+积分） | CTL-001 §4.1 | `applySas()` | 一致 | 与 CTL-001 §4.1 源码摘录逐项相同（限幅位置不同但等价：统一在出口 clamp） | 五段式 |
| F-02 | `control.mjs` | 反馈极性：`θ̇≈−q`、`φ̇≈−p` ⇒ 速率项正、姿态项负 | CTL-001 §5 | `applySas()` | 一致 | 12 条单测：俯仰/偏航/滚转极性、方向断言 | 补 `@note` 为何不能只看 ∂M/∂δ |
| F-03 | `control.mjs` | 积分器与指令限幅 `±intThMax/±intPhiMax/±dMax/±dwMax` | CTL-001 §4.3 | `applySas()` | 一致 | 单测积分限幅收敛到 `intThMax`；摆角/差速限幅断言 | 补 `@note` |
| F-04 | `control.mjs` | 模式切换清积分器 | 工程约定（历史瞬态事故） | `applySas()` 的 `_prevSasMode` 分支 | 一致（工程约定） | 单测：切模式积分归零、不切则继续累积 | 补 `@note` |
| F-05 | `control.mjs` | 仿真摆角限幅 ±25° vs 固件 ±15° | 行为红线/AGENTS | `P.dMax`（参数） | 偏差（已登记 P1-3） | `dMax=0.4363 rad`（25°）；固件 2026-08-07 收紧 ±15°，仿真未回写 | 已 `@redline` 标注；回写需重采基线 |
| F-06 | `param` | 角速度闭环（mode 3）滑块=ω_ref | CTL-001 §8.2 | `applySas()` 的 `sasMode===3` 分支 | 一致 | 单测三通道：误差为零保留配平偏置、超调反向修正 | 已补 |
| F-07 | `control.mjs` | 悬停 `qCmd=Q_HOVER⊗Rx(ψ)⊗Ry(θ)⊗Rz(φ)` | CTL-001 §8.2 | `applyVtolHover()` | 一致 | 与 §8.2 公式同序；ψ_est 由 `y_b` 水平投影提取（ψ=0/90/180 用例） | 五段式 |
| F-08 | `control.mjs` | 机体系误差四元数 `qe=qCmd⁻¹⊗q`、短弧取反 | CTL-001 §8.2 | `applyVtolHover()` | 一致 | 200° 误差走 +160° 短弧用例 | 已补 |
| F-09 | `control.mjs` | 悬停内环三通道效率符号 | CTL-001 §8.3 | `applyVtolHover()` | 一致 | 三通道符号链单测（δt→My<0、Δω→Mx<0、δf→Mz<0） | 已补 |
| F-10 | `control.mjs` | 定高串级 + `1/√cosγ` 倾角补偿 | CTL-001 §8.4 | `applyVtolHover()` 的 `altHold` 段 | 一致 | 倾斜 10° 稳态油门 ≈ `thrHover/√cos10°`（±0.02）；爬升/下降/直通/积分限幅用例 | 已补 |
| F-11 | `state.mjs` | `Q_HOVER`、`hoverThrottle` | CTL-001 §8.1 | `Q_HOVER`、`hoverThrottle()` | 一致 | `x_b→(0,0,−1)`；双发推力=重力（1e-9） | 已补 `@formula` |
| F-12 | `state.mjs` | 三种复位语义 | CTL-001 §8.1；UI 契约 | `resetSimulationState`/`resetPoseOnly`/`resetVtolHoverState` | 一致 | 3 条复位测试（保留模式/输入、模式标志对称） | 已补 `@note` |
| F-13 | `control.mjs` | 执行器轴映射（体轴 vs 悬停构型） | AGENTS.md「执行器轴映射」；CTL-001 §8.6 | `applySas`/`applyVtolHover`/`propulsion.mjs` | 一致 | 体轴 δf→Mz/δt→My/Δω→Mx 与 MOD-001 §4 相同；悬停构型语义（上摆=滚转、下摆=俯仰、差速=偏航）为同一符号在不同姿态下的读法，不构成代码分叉 | L0 与 L1 同时写两套读法 |

### 2.7 状态、参数、遥测与浏览器层（G）

| ID | 模块/文件 | 理论要素或公式 | 文献出处 | 代码位置（符号名） | 结论 | 证据 | 处置 |
|----|-----------|----------------|----------|--------------------|------|------|------|
| G-01 | `parameters.mjs` | 59 参数与 `aircraft-model.json` 一致 | GOV-002；AGENTS | 生成物 | 偏差（已登记 P1-1） | `sync-params.py --check` **FAIL**；逐字比对差异仅为 `dMax` 的 description 文本（数值相同） | 不手改生成物；由参数 owner 重生成 |
| G-02 | `parameters.mjs` | 配平常数（aTrim/dtTrim/dfTrim/thrTrim） | MOD-003 §2.2 | `P.aTrim` 等 | 一致 | `parameters.test.mjs` 锁定到 1e-15；与 MOD-001 §7.2 数值一致 | 已补测试说明 |
| G-03 | `telemetry.mjs` | 遥测分组/快照隔离 | MOD-002 §8 | `getTelemetry()` | 一致 | 单测键集合、改快照不影响 sim | 已补 |
| G-04 | `aircraft-view.mjs` | 渲染力臂 ±1.78/−1.82 m ≠ 物理 0.315 m | MOD-002 §6 | `createAircraftView()` | 一致（注释已修正） | 原注释"a=b=0.62 m"错误（P2-8），已改 0.315 m；几何位置与 MOD-002 §6 表一致 | 已修 |
| G-05 | `browser/*.mjs` | 渲染约定（相机 up=−z、载机跟随系） | MOD-001 §9 | `createScene()`、`effects.update()` | 一致（工程约定） | 注释说明；无物理量参与 | 已补 |
| G-06 | `tests/*` | 91 项测试性质与容差 | 各模块文献 | `tests/**` | 一致 | 见 §5；容差依据写在各文件头 | 已补 |

---

## 3 发现清单

> 严重度定义见 [README](README.md) §3。**P0 = 无**（未发现符号/量纲/稳定性结论错误；四元数、重力、牛顿–欧拉、六维映射、B 矩阵与文档逐项一致）。

| 严重度 | 位置（符号） | 问题 | 影响 | 建议 |
|--------|--------------|------|------|------|
| P1-1 | 生成物 `parameters.mjs` ↔ `models/aircraft-model.json` | `py -3.12 tools/sync-params.py --check` 失败：两者文本不一致（逐字比对确认**仅 `dMax` 的 description 文本**不同，数值 `0.4363323129985824` 相同；另含行尾差异） | 参数漂移门禁红灯；后续改参数时无法用该检查守护 | 由参数 owner 走 GOV-002 重生成（不改数值、不手改生成物）；本轮只读核对并登记 |
| P1-2 | `applyVtolHover()` → `computeBTrue(S.thr·wMax, …)` | B_true 工作点用**指令油门**换算 `ω0`，而 `M_cur` 用**实际转速** `S.wf/S.wt`（含 τm 滞后）。固件已于 2026-08-09 改为 τm 观测器 `w_est`，仿真未同步 | 油门瞬态时分配增益失配：有效增益 ∝(w_act/w_cmd)²，峰值比 ≈1.33 时偏差 ~76%，可能放大抖油门振荡 | 登记待决策；若同步固件需改行为并重采回归基线（属红线流程），本轮不改 |
| P1-3 | `P.dMax` / `applySas()` | 仿真摆角限幅 ±25°（`dMax=0.4363`），固件实机 2026-08-07 已收紧 ±15°，仿真未回写 | 仿真控制权限高于实机；±15°–25° 段的仿真结论不能外推实机 | 已按红线在 L0 `@redline` 与参数注释标注；回写需重采基线，登记不擅自改 |
| P2-1 | `eulerFromQuat()` | 返回值实为"对 `Rᵀ` 做标准 ZYX 分解"的角，**φ/θ/ψ 三者皆为逆旋转角**；MOD-002 §4.2 只强调 theta 的负号 | 读者可能误以为 φ/ψ 是标准角；复合姿态下不是简单取负（数值例见 A-06） | 不改代码（行为红线）；建议 MOD-002 §4.2 补一句全角说明（文档 owner） |
| P2-2 | `physicsStep()` + `stepPropulsion()` + `applySas()` | 耦合系统端到端为一阶精度：RK4 仅作用于 `[v,ω,q]`，电机转速与 SAS 积分器为显式欧拉 | 4 ms 子步下可接受；提高子步精度需同步改电机/积分器离散化（会改基线） | 注释写明（B-07）；不改行为 |
| P2-3 | `stepPropulsion()` | 一阶滞后用显式欧拉 `ω+=(ω_cmd−ω)·min(dt/τm,1)`，非 `1−e^(−dt/τm)` | 相对误差 ~7e-3（dt=4 ms/τm=0.28 s） | 已 `@approx` 标注；可接受 |
| P2-4 | `computeBTrue()` | B 为准稳态（不含 `Jp·ω̇`），而 `M_cur` 含该瞬态项 | 分配层一阶模型失配（小增量可接受） | 已 `@note`；不改 |
| P2-5 | `computeAeroState()` 的 `al` | α 用 `atan2(w,u)`，未像 β 那样做低速分母保护；`u→0` 时线性小迎角模型失效 | 悬停开气动时升力/力矩无物理意义（悬停默认 `aero=false`） | 注释写明适用边界；建议 UI 层保持"悬停默认关气动" |
| P2-6 | `theme.mjs` | `import * as THREE` 未使用（死导入） | 无功能影响；静态检查噪声 | 建议后续删除（非注释改动，本轮不动） |
| P2-7 | MOD-002 §3.4/§5.2、CTL-001 §4.1/§7.1 等 | 文档用行号引用源码（`dynamics.mjs:physicsStep 第 62–63 行`、`propulsion.mjs:12–14`、`control.mjs:16/17/18/19`），本轮补注释后行号进一步漂移 | 引用失效（GOV-004 §6.2 明令禁止行号） | 文档 owner 改为符号名；本轮核查记录一律用符号名 |
| P2-8 | `aircraft-view.mjs` 文件头 | 原注释"与动力学力臂（a=b=0.62m）无关"数值错误（实际 0.315 m） | 误导读者（渲染/物理力臂红线） | **已修正**为 0.315 m，并补 MOD-002 §6 引用 |
| P2-9 | `tests/**`（91 条 test） | 逐 test 块注释采用"文件头覆盖表（性质/输入/期望/容差依据）+ 关键容差块内注释"，未为每条 test 再加一行 | 满足"为什么这个容差"的可追溯性，但非逐块行内注释 | 列为遗留；如需严格逐块，可在后续轮次补 91 行（不改行为） |

---

## 4 未决问题

1. **P1-2 的口径决策**：仿真是否跟进固件的 τm 观测器（用 `w_est` 做 B 工作点）。跟进会改变悬停分配行为 → 需重采回归基线，应由飞控/仿真 owner 共同决策。
2. **P1-3 的仿真回写**：仿真 `dMax` 是否收紧到 ±15°。影响巡航配平窗口（`dtTrim=−18.7°` 已超出 ±15°，回写会直接使巡航配平不可达）——这不是简单改参数，须先讨论固件配平逻辑。
3. **P2-1 的文档措辞**：MOD-002 §4.2 是否补充"φ/ψ 同为逆旋转角"以及复合姿态示例。
4. 本次核查以代码与文档公式对照 + 数值复算为准，**未使用实机飞行数据**；SAS 增益与气动参数仍为 MODEL-DEFAULT 未标定（MOD-001 §10），本文"一致"仅指实现与文档公式一致，不表示与真实飞机一致。

---

## 5 门禁与测试

### 5.1 注释门禁（GOV-004）

| 阶段 | 文件数 | 平均分 | error | warning | 全绿 | 规则命中 |
|------|--------|--------|-------|---------|------|----------|
| 核查前 | 38 | 69.3 | **46** | 49 | 0 | R01=38 R04=4 R07=8 R08=36 R09=8 R10=1 |
| 核查后 | 38 | **100.0** | **0** | **0** | **38** | 无 |

命令：`py -3.12 tools/check-code-comments.py --area sim --check` → **退出码 0**。

说明：R04 的 4 处（`control.mjs` 模式表/定高公式/B_true 说明、`math.mjs` 列主序式）在人工判断后确认是**公式注释**被启发式误判（`0 = 关闭`、`h = −pos.z` 等行匹配"变量赋值"模式）；本轮改写为 `公式：…`/`→` 形式，语义不变。R08 的 3 处 Python 误报由多行签名导致（docstring 落在机检 3 行窗口外），通过签名单行化消除——参数、默认值与语句完全不变。

### 5.2 单元/回归测试

- `node --test "simulations/vector-thrust-lab/tests/*.test.mjs"` → **91/91 通过，0 失败**（约 0.2 s）。
- 全量测试覆盖：math 8、control 12、control-allocation 5、dynamics 10、aerodynamics 5、propulsion 6、state 3、telemetry 3、parameters 6、regression 4、vtol 31。
- 注释改动未改变任何数值断言：`tests/fixtures/regression-baseline.json` **未改动**。

### 5.3 静态与语法

- `py -3.12 -m py_compile simulations/vector-thrust-lab/scripts/*.py` → 通过（8/8）。
- `node --check` 覆盖 `src/core/*.mjs`、`src/browser/*.mjs`、`src/main.js` → 全部通过。
- 冒烟运行：`pm_sensitivity.py`、`cascade_closedform.py` 退出码 0（依赖 numpy/scipy/sympy 可用）。

### 5.4 数值复算（可复跑）

以下结论由一次性 Node 脚本对 `src/core/` 复算得到（不入库，命令可复现）：

| 核对项 | 方法 | 结果 |
|--------|------|------|
| `rotateVecByQuat` ≡ `q⊗ṽ⊗q*` | 2000 组随机 `(v,q)` | 最大差 **3.11e-15** |
| 四元数复合顺序 `q_a⊗q_b`（先 b 后 a） | 2000 组随机 | 最大差 **1.60e-15** |
| 重力变换 `g_b=Rᵀ·(0,0,g)` | 2000 组随机姿态 | 最大差 **4.44e-15** |
| `eulerFromQuat` ≡ 标准 ZYX(Rᵀ) | 2000 组随机姿态 | 最大差 **1.8e-14** |
| `computeBTrue` 解析 vs 中心差分 | `h=1e-7`，悬停工作点 | 最大差 **2.38e-9**（测试阈值 1e-6） |
| 耦合系统收敛阶 | T=0.2 s，n=4/8/16/32 vs n=6400 参考 | **1.53 / 1.38 / 1.74**（一阶，见 P2-2） |

---

## 6 本轮改动摘要（仅注释/文档）

- 38 文件全部补 L0（`@file/@module/@brief/@details/@theory/@frame/@units`，触及红线处加 `@redline`）。
- 32 个 JS 导出符号与 8 个 Python 公共函数补 L1 文档块（`@brief/@param 带单位/@returns/@formula/@ref/@note/@approx/@test`）。
- 核心 6 个理论文件（math/dynamics/control/propulsion/aerodynamics/control-allocation）各补"五段式"理论对照块（公式/符号/来源/实现/等价变换与数值证据）。
- L2：单位与符号翻转（NED z 向下）、边界保护（vMin/奇异/null 回退）、红线常量（dMax/dwMax/tauM/groundZ）、时序假设（applySas 先于 stepPropulsion）、历史事故理由（模式切换清积分、qCmd 跟航向）。
- 修正腐烂注释 1 处（渲染力臂 0.62 m → 0.315 m），消除 docstring/注释中的行号式源码引用 2 处（`control.mjs:142-147` → 符号名 `applyVtolHover`）；另有 1 处同类引用位于 `pm_sensitivity.py` 的运行输出字符串中（改文本会改变程序输出，留给脚本 owner，见 P2-7）。
- 未改动：任何可执行语句的语义/数值/顺序、`parameters.mjs`（生成物）、`regression-baseline.json`（基线）、`docs/*.html`（构建产物）。
