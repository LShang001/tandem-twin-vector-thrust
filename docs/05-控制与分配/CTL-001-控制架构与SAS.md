# CTL-001 控制架构与 SAS

> 本文定位「纵列双发矢量推力飞行器」整机控制链的工程实现侧，覆盖从 UI 操纵输入到六维力/力矩输出的信号流、SAS 三通道增稳律、反馈极性整定依据及关键限幅。对应源码实现见 `simulations/vector-thrust-lab/src/core/control.mjs` 与 `propulsion.mjs`，理论背景见 [THY-003](../03-理论推导/THY-003-刚体动力学控制分配与模态分析.md)。

---

## 1 控制链总览

整机控制链路如下图所示。操纵输入经 UI 限幅后进入 SAS（可选），三通道律输出执行指令，再由推进分配映射为六维力/力矩，最后经刚体动力学积分得到状态反馈。

```mermaid
flowchart LR
    accTitle 整机控制链路
    accDescr 从 UI 操纵输入到动力学状态反馈的完整信号流，标注关键限幅点。

    UI["UI 滑块输入\nthr / dt / df / dw"]
    UI_LIM["UI 限幅\n|dt|,|df|≤dMax\n|dw|≤dwUiMax"]
    SAS_SW{"SAS 开关\nsas=true/false"}
    SAS_LAW["SAS 三通道律\n俯仰/偏航/滚转"]
    ACT_LIM["执行限幅\n|dtAct|,|dfAct|≤dMax\n|dwAct|≤dwMax"]
    PROP["推进分配\n差速开方 + 六维映射"]
    DYN["六自由度动力学\n牛顿–欧拉积分"]
    STATE["状态反馈\nφ,θ,ψ / p,q,r"]

    UI --> UI_LIM --> SAS_SW
    SAS_SW -->|开启| SAS_LAW --> ACT_LIM
    SAS_SW -->|关闭| ACT_LIM
    ACT_LIM --> PROP --> DYN --> STATE
    STATE -.-> SAS_LAW
```

链路说明：

| 节点 | 位置/文件 | 关键行为 |
|---|---|---|
| UI 滑块 | `main.js` / 浏览器事件 | 输出归一化 thr∈[0,1]、dt/df∈[-dMax,dMax]、dw∈[-dwUiMax,dwUiMax] |
| SAS 模式 | `state.mjs` 字段 `S.sasMode` | 0=直通，1=全 SAS，2=仅角速率，3=角速度闭环；尾摆均叠加 `dtTrim` |
| 三通道律 | `control.mjs:applySas` | 比例/积分/角速率反馈，输出限幅至 ±dMax / ±dwMax |
| 推进分配 | `propulsion.mjs:stepPropulsion` | 差速开方 → 电机一阶滞后 → 六维力/力矩 |
| 状态反馈 | `dynamics.mjs:physicsStep` | `eulerFromQuat` 提取 φ/θ/ψ，`S.omega` 直接可用 |

---

## 2 架构声明：直接映射而非 B⁺ 伪逆优化分配

当前实现采用**直接映射 + SAS 反馈增稳**，未使用控制分配理论中的 Moore–Penrose 伪逆或加权伪逆。原因如下：

1. **执行变量与受控力矩近似一一对应**：尾摆角 δ_t 主控俯仰力矩 M_y，前摆角 δ_f 主控偏航力矩 M_z，差速 Δω 主控滚转力矩 M_x。三通道主控效率远大于通道间耦合效率（量级由反扭-推力比 τ/(T·力臂) 决定，见 [THY-003](../03-理论推导/THY-003-刚体动力学控制分配与模态分析.md) §7）。
2. **无执行器冗余**：每个力矩通道仅由一个执行变量主控，不存在过驱动系统的自由度优化问题。
3. **实现简单、可解释性强**：直接映射使 SAS 律可按 SISO 通道独立整定与调试，符号错误可在单测中逐条锁定。

当未来增加气动舵面或第三台推进器形成冗余时，再考虑引入 B⁺ / WLS 在线分配。当前仅登记该方向，不实施（见 [CTL-002](CTL-002-可控性与失效模式.md) §5）。

---

## 3 三通道控制机理

| 通道 | 执行变量 | 主控力矩 | 效率符号（小角度） | 反馈符号 | 备注 |
|---|---|---|---|---|---|
| 俯仰 | δ_t（尾摆角，绕 y_b） | M_y ≈ −b·T_t·δ_t | ∂M_y/∂δ_t < 0 | 速率项 `+q`；显示姿态项 `−Δθ` | `theta_dot≈−q`，两类反馈符号不能合并判断 |
| 偏航 | δ_f（前摆角，绕 z_b） | M_z ≈ +a·T_f·δ_f | ∂M_z/∂δ_f > 0 | SAS 取负号 | 前推力侧向分量产生偏航力矩 |
| 滚转 | Δω（差速） | M_x ≈ −2·kQ·ω0²·Δω | ∂M_x/∂Δω < 0 | 速率项 `+p`；显示姿态项 `−φ` | `phi_dot≈−p`，反扭矩差驱动滚转 |

表中效率符号由 `propulsion.mjs` 六维映射逐项导出，见 [PROP-001](../06-推进与执行机构/PROP-001-推进与摆座模型.md) §4。

---

## 4 SAS 控制律

### 4.1 源码公式

`control.mjs:applySas` 的完整实现如下（直接摘录）：

```javascript
thetaError = theta + P.aTrim;
S.intTh  = clamp(S.intTh  + thetaError * dt, -P.intThMax,  P.intThMax);
S.intPhi = clamp(S.intPhi + phi   * dt, -P.intPhiMax, P.intPhiMax);

dtC = clamp(P.dtTrim + dt + P.sasQ * S.omega.y - P.sasTh * thetaError - P.sasI * S.intTh, -P.dMax, P.dMax);
dfC = clamp(df - P.sasR * S.omega.z,                                      -P.dMax, P.dMax);
dwC = clamp(dw + P.sasP * S.omega.x - P.sasPhi * phi - P.sasIPhi * S.intPhi, -P.dwMax, P.dwMax);
```

用符号表达：

```
δ_t,cmd = clamp( dtTrim + δ_t + sasQ·q − sasTh·Δθ − sasI·∫Δθ, ±dMax )
δ_f,cmd = clamp( δ_f − sasR·r,                   ±dMax )
Δω_cmd  = clamp( Δω  + sasP·p − sasPhi·φ − sasIPhi·∫φ, ±dwMax )

Δθ = θ − θ_ref = θ + aTrim,  θ_ref = −aTrim
∫θ  ← clamp( ∫θ + Δθ·dt, ±intThMax )
∫φ  ← clamp( ∫φ + φ·dt, ±intPhiMax )
```

### 4.2 增益表

全部增益来自 `models/aircraft-model.json`，状态 MODEL-DEFAULT，为手工整定值，无系统辨识。

| 参数 | 值 | 单位 | 含义 | 来源 |
|---|---|---|---|---|
| sasQ | 0.14 | s | 俯仰角速率反馈增益 | MODEL-DEFAULT |
| sasR | 0.14 | s | 偏航角速率反馈增益 | MODEL-DEFAULT |
| sasP | 0.18 | s | 滚转角速率反馈增益 | MODEL-DEFAULT |
| sasTh | 0.30 | − | 俯仰角比例反馈增益 | MODEL-DEFAULT |
| sasPhi | 0.40 | − | 滚转角比例反馈增益 | MODEL-DEFAULT |
| sasI | 0.10 | − | 俯仰积分反馈增益 | MODEL-DEFAULT |
| sasIPhi | 0.15 | − | 滚转积分反馈增益 | MODEL-DEFAULT |

### 4.3 积分器与指令限幅

| 限幅项 | 值 | 单位 | 代码位置 | 说明 |
|---|---|---|---|---|
| 俯仰积分限幅 intThMax | 0.5 | rad | `control.mjs:16` | 抑制俯仰积分饱和 |
| 滚转积分限幅 intPhiMax | 0.3 | rad | `control.mjs:17` | 抑制滚转积分饱和 |
| 摆角指令限幅 dMax | 0.4363323129985824 | rad（25°） | `control.mjs:18,19` | 尾/前摆角同时限幅 |
| SAS 差速指令限幅 dwMax | 0.7 | − | `control.mjs:19` | SAS 输出差速上限 |
| UI 差速幅值 dwUiMax | 0.55 | − | UI 输入层 | 用户可操作范围 |

注意：当前 SAS 律仅含「积分与指令限幅」，不含抗饱和（anti-windup）逻辑或摆角速率限制。积分器持续饱和时表现为固定偏置输出，详见 [CTL-002](CTL-002-可控性与失效模式.md) §3。

---

## 5 反馈极性整定逻辑

SAS 增益符号必须按各通道控制效率符号整定，否则构成正反馈。推导如下：

1. **俯仰通道**：`propulsion.mjs` 给出 `M_y = −b·T_t·sinδ_t − Q_f·sinδ_f`，因此 `q>0` 时用 `+sasQ·q` 产生负俯仰矩，速率阻尼符号为正。但当前显示约定满足小角度 `theta_dot≈−q`；当显示角误差 `Δtheta>0` 时，需要增大 `q` 使 `theta` 回落，因此姿态项必须是 `−sasTh·Δtheta`，积分项同号。不能只看 `∂M_y/∂δ_t` 就把速率项与姿态项设成相同符号。

2. **偏航通道**：`propulsion.mjs:33` 给出 `M_z = a·T_f·sinδ_f − Q_t·sinδ_t`。小角度下主项 `+a·T_f·δ_f`，效率 ∂M_z/∂δ_f > 0。当 `r>0`（右偏航）时，需减小 δ_f 使 M_z 减小（左偏航恢复），故 SAS 律取**负号**：`dfC = df − sasR·r`。

3. **滚转通道**：差速效率 `∂M_x/∂Δω < 0`，所以 `p>0` 时 `+sasP·p` 提供角速率阻尼。由于当前显示角满足 `phi_dot≈−p`，姿态恢复项为 `−sasPhi·phi`，积分项同号。

调试历史上曾因符号错误导致发散；当前极性由 `control.test.mjs` 三条极性测试锁定。

---

## 6 欧拉角符号约定依赖

姿态提取函数 `math.mjs:eulerFromQuat` 采用约定：

```
θ = −asin(R13)
```

其中 `R13` 为旋转矩阵第 1 行第 3 列（机体系 x 轴在惯性系 z 轴的投影）。该约定导致：绕机体 +y_b 轴正转 +a0（抬头）读出的俯仰角为 −a0。`tests/math.test.mjs` 第 47 行测试明确锁定此行为，注释指出「SAS 增益即按此约定整定」。

因此：

- 物理上抬头 θ_phys = +a0 时，SAS 见到的 `theta = −a0`；
- 俯仰参考为 `theta_ref=-aTrim`，控制器使用 `thetaError=theta+aTrim`；姿态项取负号，角速率项取正号；
- 若将 `eulerFromQuat` 改为 `θ = +asin(R13)`，则所有俯仰/偏航/滚转增益符号需同步翻转，否则闭环失稳。

坐标系与欧拉角提取的完整约定见 [MOD-002](../04-数学建模/MOD-002-坐标系与符号约定.md)。

---

## 7 差速开方分配

### 7.1 分配律

`propulsion.mjs:12–14`：

```
ω0 = thr · wMax
ω_f,target = ω0 · √(max(0, 1 + Δω_cmd))
ω_t,target = ω0 · √(max(0, 1 − Δω_cmd))
```

### 7.2 平方和不变

```
ω_f² + ω_t² = ω0²(1+Δω) + ω0²(1−Δω) = 2·ω0²
```

因此在两侧目标转速均未触及 `wMax` 时，推力幅值之和为常数。高油门大差速使高转速侧被钳位后，该等式不再成立；例如油门 1.0、差速 0.55 时当前模型总推力下降 27.5%。

### 7.3 总推力一阶不变

机体 x 轴推力：

```
F_x = T_f·cosδ_f + T_t·cosδ_t
```

当 δ_f、δ_t 为小量且 Δω 独立变化时，`cosδ ≈ 1`，`F_x` 一阶近似不变。该性质由 `propulsion.test.mjs` 第 27 行测试锁定。

### 7.4 滚转力矩线性度

零摆角、稳态（`ω̇_f = ω̇_t = 0`）时：

```
M_x = −Q_f + Q_t
    = −kQ·ω_f² + kQ·ω_t²
    = −kQ·ω0²(1+Δω) + kQ·ω0²(1−Δω)
    = −2·kQ·ω0²·Δω
```

故滚转通道在小 Δω 范围内具有理想线性控制效率 `∂M_x/∂Δω = −2·kQ·ω0²`。该效率随油门平方衰减：低油门时 ω0 小，滚转控制能力二次退化，见 [CTL-002](CTL-002-可控性与失效模式.md) §2。

---

## 8 VTOL 悬停控制模式（机头朝天，无翼）

> 对应源码：`control.mjs:applyVtolHover`、`state.mjs:Q_HOVER/resetVtolHoverState/hoverThrottle`；固件同构实现见 `TandemVec_FCS/src/flight_control.cpp`（`q_hover ⊗ Rx(-Heading)` 悬停链）。UI 按钮「悬停模式」切换 `S.vtolMode`。

### 8.1 构型与初始化

- **悬停姿态**：`Q_HOVER = (w, y) = (√2/2, √2/2)`（绕 NED y 转 +90°），机头朝天（x_b → NED −z）。显示约定下 `theta = −asin(R₁₃) = −90°`——欧拉角在 θ≈−90° 奇异，**固定翼欧拉反馈律不可用**，必须换四元数误差控制。
- **无翼构型**：进入悬停时 `S.aero = false`（气动力忽略 = 关掉机翼），可手动再开；低速下 α/β 定义失真、无翼机身无升力，气动关闭是物理合理的默认。
- **悬停配平油门**（物理派生量，非独立参数）：双发推力平衡重力 `2·kT·(thr·wMax)² = m·g` → `thrHover = √(m·g / (2·kT·wMax²)) ≈ 0.499`。
- **摆角基准 0**：悬停不叠加巡航配平偏置（`dtTrim/dfTrim` 是固定翼配平值）。
- 悬停模式只做**姿态稳定**，无位置/高度闭环；倾斜指令产生持续水平漂移（无翼无阻尼），为模型固有行为。

### 8.2 控制律：误差四元数级联

```
目标姿态:  qCmd = Q_HOVER ⊗ Rx(ψ)·Ry(θ)·Rz(φ)     （滑块小角度指令，四轴式语义）
机体系误差: qe = qCmd⁻¹ ⊗ q                        （⚠ 勿用 q ⊗ qCmd⁻¹：那是 NED 系表达，
                                                    悬停时 x/y/z 轴错位，会张冠李戴）
目标角速度: ωdes = −2·vtolAttKp · qe.xyz            （小角度 qe.xyz ≈ 误差/2）
内环（按通道效率符号，与 §4 角速度闭环一致）:
  尾摆 δt（∂My/∂δt<0）: dtC = rateKq·(ω.y − ωdes.y)
  前摆 δf（∂Mz/∂δf>0）: dfC = rateKr·(ωdes.z − ω.z)
  差速 Δω（∂Mx/∂Δω<0）: dwC = rateKp·(ω.x − ωdes.x)
```

- **滑块语义切换**（悬停模式）：dw → **航向角速度指令 ψ̇**（°/s，绕 x_b，rate 模式：松手回中停转、停在当前航向，无姿态回中；与固件 RATE_MODE 偏航摇杆一致）；dt → 俯仰倾斜目标角（绕 y_b，尾摆）；df → 侧倾目标角（绕 z_b，前摆）。**悬停自稳模式下三滑块均为弹簧回中**（dt/df 松手归零 → 姿态回竖直，dw 松手停转）；**qCmd 跟随当前航向**（ψ_est 由 y_b 水平投影提取）——否则航向自由旋转会使 qe 的 y/z 分量被 cosψ 调制（ψ=90° 倾斜失效、180° 反向，2026-08-05 复现并修复）。
- **自稳开关**（`S.sasMode`）：悬停下仅区分 0/非 0——0 时直通滑块摆角/差速（无自稳，可对比演示"无自稳必倾倒"）；非 0 时四元数自稳。
- **增益**：姿态外环 `vtolAttKp = 2.5`（`models/aircraft-model.json` §VTOL 悬停控制，MODEL-DEFAULT）；内环复用 `rateKq/rateKr/rateKp`。
- **180° 翻转防御**：`qe.w < 0` 时取反（等价误差），保证走最短旋转路径。

### 8.3 符号数值验证（三通道）

| 误差方向 | 控制输出 | 力矩符号链 | 收敛 |
|---|---|---|---|
| qe.y > 0（绕 y_b 超转） | dtAct > 0 | δt>0 → My=−b·Tt·sinδt < 0 → q̇<0 | ✓ |
| qe.x > 0（绕 x_b 超转=航向偏移） | dwAct > 0 | Δω>0 → Mx=−2·kQ·ω0²·Δω < 0 → ṗ<0 | ✓ |
| qe.z > 0（绕 z_b 超转=侧倾） | dfAct < 0 | δf<0 → Mz=a·Tf·sinδf < 0 → ṙ<0 | ✓ |

闭环收敛、指令追踪与无自稳对比见 `vtol.test.mjs`（13 用例）。

### 8.4 自动定高（高度保持，可选）

> 对应源码：`control.mjs:applyVtolHover` 末尾高度环；UI「定高」按钮 + 参考高度滑块（仅悬停模式显示）。固件对应链：`altitudePositionPController`（高度→目标垂直速度，±1.0 m/s）+ `altitudeVelocityPIDController`（垂直速度→油门）。

**串级结构**（高度外环 P+I → 目标垂直速度 → 油门内环 P）：

```
h    = −pos.z          （NED z 向下，高度向上为正）
vZ   = −vWorld.z       （垂直速度，向上为正）
vZref = clamp(altKpH·h_err + altKpI·∫h_err, ±altVZMax)      h_err = altRef − h
thr  = thrHover/√cosγ + altKpV·(vZref − vZ)                 cosγ = x̂_b·(−ẑ_NED)
```

- **倾角补偿**：倾斜损失竖直推力分量，且 T∝thr² → 补偿因子 **1/√cosγ**（非 1/cosγ）；稳态 `T·cosγ = m·g` 精确成立（数值验证：倾斜 10° 稳态 thr = 0.503 ≈ thrHover/√cos10° = 0.503）
- **参考高度**：`S.altRef`（UI 滑块 0–20 m，**拖动实时更新参考高度**，定高开启时生效）；开关切换时清积分器防瞬态
- **增益**：`altKpH=0.5`、`altKpI=0.15`、`altKpV=0.1`、`altVZMax=1.0`（`models/aircraft-model.json` §VTOL 高度保持，MODEL-DEFAULT）；积分限幅 ±1.5 m·s（派生常量）
- **边界**：定高与姿态指令可同时使用（倾斜平移时高度保持）；无位置闭环，水平漂移仍为模型固有行为。仿真无传感器模型，高度/垂直速度用真值

---

## 9 测试覆盖指针

| 测试用例 | 文件/行 | 对应本文条目 |
|---|---|---|
| SAS 关闭时指令直通 | `control.test.mjs:10` | §1 控制链、§4.1 |
| 俯仰姿态反馈与 `theta_dot=-q` 一致 | `control.test.mjs:21` | §5、§6 |
| 偏航反馈极性（正 r 减小 dfAct） | `control.test.mjs:29` | §5 |
| 滚转姿态反馈与 `phi_dot=-p` 一致 | `control.test.mjs:36` | §5、§6 |
| 执行限幅 ±dMax / ±dwMax | `control.test.mjs:44` | §4.3 |
| 积分器限幅并随时间累积 | `control.test.mjs:53` | §4.1、§4.3 |
| 差速分配保持平方转速和不变 | `propulsion.test.mjs:27` | §7.2 |
| 前/尾摆角符号 | `propulsion.test.mjs:34,42` | §3、§5 |
| 零摆角等转速反扭对消 | `propulsion.test.mjs:50` | §7.4 |
| eulerFromQuat 约定 θ=−a0 | `math.test.mjs:47` | §6 |
| 悬停配平平衡 / Q_HOVER 方向 | `vtol.test.mjs` | §8.1 |
| 三通道控制符号 / 无配平偏置 | `vtol.test.mjs` | §8.2、§8.3 |
| 悬停收敛 / 扰动恢复 / 指令追踪 / 无自稳对比 | `vtol.test.mjs` | §8.2、§8.3 |

---

## 10 成熟度声明

- 当前 SAS 为**概念级**直接反馈增稳律，增益为手工整定，未经过系统辨识或飞行验证；
- 所列工况下仿真响应有界（见 [MOD-001](../04-数学建模/MOD-001-六自由度仿真模型规范.md) §7.3），不表示全包线稳定；
- 限幅策略为「积分与指令限幅」，不含抗饱和、速率限制或执行器位置动态。
