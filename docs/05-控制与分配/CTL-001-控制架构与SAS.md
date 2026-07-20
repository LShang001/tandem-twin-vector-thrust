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
| SAS 开关 | `state.mjs` 字段 `S.sas` | 关闭时 `dtAct=dt`、`dfAct=df`、`dwAct=dw` 直通 |
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
| 俯仰 | δ_t（尾摆角，绕 y_b） | M_y ≈ −b·T_t·δ_t | ∂M_y/∂δ_t < 0 | SAS 取正号 | 尾推力摆离 x_b 产生低头力矩 |
| 偏航 | δ_f（前摆角，绕 z_b） | M_z ≈ +a·T_f·δ_f | ∂M_z/∂δ_f > 0 | SAS 取负号 | 前推力侧向分量产生偏航力矩 |
| 滚转 | Δω（差速） | M_x ≈ −2·kQ·ω0²·Δω | ∂M_x/∂Δω < 0 | SAS 取正号 | 反扭矩差驱动滚转 |

表中效率符号由 `propulsion.mjs` 六维映射逐项导出，见 [PROP-001](../06-推进与执行机构/PROP-001-推进与摆座模型.md) §4。

---

## 4 SAS 控制律

### 4.1 源码公式

`control.mjs:applySas` 的完整实现如下（直接摘录）：

```javascript
S.intTh  = clamp(S.intTh  + theta * dt, -P.intThMax,  P.intThMax);
S.intPhi = clamp(S.intPhi + phi   * dt, -P.intPhiMax, P.intPhiMax);

dtC = clamp(dt + P.sasQ * S.omega.y + P.sasTh * theta + P.sasI * S.intTh,  -P.dMax, P.dMax);
dfC = clamp(df - P.sasR * S.omega.z,                                      -P.dMax, P.dMax);
dwC = clamp(dw + P.sasP * S.omega.x + P.sasPhi * phi   + P.sasIPhi * S.intPhi, -P.dwMax, P.dwMax);
```

用符号表达：

```
δ_t,cmd = clamp( δ_t + sasQ·q + sasTh·θ + sasI·∫θ, ±dMax )
δ_f,cmd = clamp( δ_f − sasR·r,                   ±dMax )
Δω_cmd  = clamp( Δω  + sasP·p + sasPhi·φ + sasIPhi·∫φ, ±dwMax )

∫θ  ← clamp( ∫θ + θ·dt, ±intThMax  )
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

1. **俯仰通道**：`propulsion.mjs:32` 给出 `M_y = −b·T_t·sinδ_t − Q_f·sinδ_f`。小角度下主项 `−b·T_t·δ_t`，效率 ∂M_y/∂δ_t < 0。当 `q>0` 或 `θ>0`（抬头）时，需增大 δ_t 使 M_y 更负（低头恢复），故 SAS 律取**正号**：`dtC = dt + sasQ·q + ...`。

2. **偏航通道**：`propulsion.mjs:33` 给出 `M_z = a·T_f·sinδ_f − Q_t·sinδ_t`。小角度下主项 `+a·T_f·δ_f`，效率 ∂M_z/∂δ_f > 0。当 `r>0`（右偏航）时，需减小 δ_f 使 M_z 减小（左偏航恢复），故 SAS 律取**负号**：`dfC = df − sasR·r`。

3. **滚转通道**：差速使 `ω_f = ω0·√(1+Δω)`、`ω_t = ω0·√(1−Δω)`。零摆角稳态下 `M_x = −Q_f + Q_t = −kQ·ω_f² + kQ·ω_t² ≈ −2·kQ·ω0²·Δω`，效率 ∂M_x/∂Δω < 0。当 `p>0` 或 `φ>0`（右滚）时，需增大 Δω 使 M_x 更负（左滚恢复），故 SAS 律取**正号**：`dwC = dw + sasP·p + ...`。

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
- SAS 俯仰律中 `sasTh·theta` 为负，使 `dtC` 减小，M_y 向正方向变化，产生低头恢复力矩；
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

因此两电机推力幅值之和 `T_f + T_t = kT(ω_f²+ω_t²) = 2·kT·ω0²` 为常数，不随 Δω 变化。

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

## 8 测试覆盖指针

| 测试用例 | 文件/行 | 对应本文条目 |
|---|---|---|
| SAS 关闭时指令直通 | `control.test.mjs:10` | §1 控制链、§4.1 |
| 俯仰反馈极性（正 θ/q 增大 dtAct） | `control.test.mjs:21` | §5 |
| 偏航反馈极性（正 r 减小 dfAct） | `control.test.mjs:29` | §5 |
| 滚转反馈极性（正 φ/p 增大 dwAct） | `control.test.mjs:36` | §5 |
| 执行限幅 ±dMax / ±dwMax | `control.test.mjs:44` | §4.3 |
| 积分器限幅并随时间累积 | `control.test.mjs:53` | §4.1、§4.3 |
| 差速分配保持平方转速和不变 | `propulsion.test.mjs:27` | §7.2 |
| 前/尾摆角符号 | `propulsion.test.mjs:34,42` | §3、§5 |
| 零摆角等转速反扭对消 | `propulsion.test.mjs:50` | §7.4 |
| eulerFromQuat 约定 θ=−a0 | `math.test.mjs:47` | §6 |

---

## 9 成熟度声明

- 当前 SAS 为**概念级**直接反馈增稳律，增益为手工整定，未经过系统辨识或飞行验证；
- 所列工况下仿真响应有界（见 [MOD-001](../04-数学建模/MOD-001-六自由度仿真模型规范.md) §7.3），不表示全包线稳定；
- 限幅策略为「积分与指令限幅」，不含抗饱和、速率限制或执行器位置动态。
