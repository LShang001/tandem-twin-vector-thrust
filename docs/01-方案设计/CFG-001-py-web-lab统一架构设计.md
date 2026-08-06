# CFG-001 py-web-lab 统一架构设计（Python 仿真引擎 + Web 交互可视化）

> 状态：草案待评审
> 域：01-方案设计
> 关联：[CFG-000](CFG-000-概念构型基线C0.md)、[GOV-002](../00-项目治理/GOV-002-信息与配置管理.md)、`simulations/high-fidelity-analysis/core.py`、`simulations/vector-thrust-lab/`
> 所有参数值来源: `models/aircraft-model.json`（唯一事实源）

## 1 背景与目标

### 1.1 问题

现有双仿真架构为**同构双写**：Web（`vector-thrust-lab/src/core/*.mjs`）与 Python（`high-fidelity-analysis/core.py`）独立实现同一组方程，靠交叉验证（当前偏差 1e-14）维持一致性。代价：

- 新增/修改算法必须在 JS 与 Python 各实现一次（当前 Web 侧只有 SAS/悬停级联/定高/B_true；INDI/MPC/级联/辨识等先进算法只存在于 Python 侧，Web 无法交互演示）
- 论文数据（Python 侧 `run_all.py`）与交互演示（JS 侧）天然分离，无单一实现可同时服务两者

### 1.2 目标

1. **统一实现**：模型/动力学/控制算法全部由 Python 实现一份，同时服务：交互 Web 可视化、论文仿真数据/图、批处理研究
2. **可交互演示先进算法**：INDI/MPC/级联等控制器只写 Python，Web 前端零改动即可演示
3. **保留现有资产**：`standalone.html` 离线版完全不动，双版本共存
4. **参数仍同源**：`models/aircraft-model.json` 唯一事实源

### 1.3 非目标（第一版不做）

- 多用户/鉴权/部署到服务器（本地单用户工具）
- 前端驱动的 step 模式（服务端自主仿真）
- 硬件在环 / 固件对接

## 2 设计原则与约束

| 原则/约束 | 说明 |
|---|---|
| **Python 是唯一模型实现** | `sim.py` 直接复用 `high-fidelity-analysis/core.py`（动力学/推进/气动已与 Web/固件三端对齐到浮点极限） |
| **控制律 Python 版必须逐位对齐 JS 版** | 悬停级联/定高/B_true 的 Python 实现与 `vector-thrust-lab/src/core/control.mjs` 行为一致（交叉测试守护，见 §8.2） |
| **前端协议不变** | 先进算法加入只改 `controllers.py`，WebSocket 协议稳定 |
| **子步/帧红线** | 子步 ≤0.004s、广播 ≤30Hz、电机 τm=0.28s 等行为红线全部继承 |
| **standalone 零改动** | 现有构建链/产物不受影响 |

## 3 总体架构

```
浏览器 (py-web-lab/web)                  Python 服务 (py-web-lab/server)
┌─────────────────────────┐  WebSocket  ┌────────────────────────────────┐
│ index.html              │  (JSON 消息) │ main.py  FastAPI 应用           │
│ ├─ main.mjs（新写）      │ ◀──遥测 30Hz │  ├─ sim.py       仿真引擎        │
│ │  WebSocket 客户端      │             │  │   └─ 复用 high-fidelity-    │
│ │  遥测→sim 结构映射     │             │  │      analysis/core.py       │
│ ├─ controls.mjs（新写）  │ ───指令────▶ │  ├─ controllers.py 控制律       │
│ │  滑块/按钮→协议指令    │             │  │   （SAS/悬停级联/定高/B_true， │
│ └─ browser/*（复用渲染） │             │  │    后续 INDI/MPC 只加这里）    │
│    scene/aircraft-view/ │             │  ├─ protocol.py  消息协议        │
│    hud/scope/effects/   │             │  └─ static/ 托管前端             │
└─────────────────────────┘             └────────────────────────────────┘
```

**仿真节奏**：服务端自主 100Hz 主循环（每 tick 25 子步 × 0.004s）→ 每 3 tick（30Hz）广播遥测快照。前端只发指令、收状态，不参与物理计算。

## 4 目录结构

```
simulations/py-web-lab/
├── server/
│   ├── main.py          # FastAPI 入口：/ws WebSocket + static 托管
│   ├── sim.py           # Simulation 类：状态 S/F/dyn/aero + 主循环（复用 core.py 推进/气动/动力学）
│   ├── controllers.py   # 控制律：apply_sas / apply_vtol_hover / apply_alt_hold / b_true 分配
│   ├── protocol.py      # 消息编解码 + 指令分派（协议版本号）
│   └── run.py           # 启动脚本（uvicorn 包装，端口 8001）
├── web/
│   ├── index.html       # 布局（沿用 vector-thrust-lab 样式，控制面板 WebSocket 版）
│   └── src/
│       ├── main.mjs     # 连接/遥测映射/渲染循环（新写）
│       ├── controls.mjs # 控制面板（新写，弹簧回中逻辑本地）
│       └── browser/     # 从 vector-thrust-lab 复制的纯渲染模块
├── tests/
│   ├── test_engine.py       # pytest：引擎单测（悬停收敛/定高/符号）
│   ├── test_controllers.py  # 控制律逐位对比 JS（加载 node 输出 JSON 基线）
│   └── cross_verify.mjs     # 交叉验证：py-web-lab vs vector-thrust-lab 同场景
├── docs/
│   └── protocol.md      # 协议细则（版本化）
└── README.md
```

**与现有项目关系**：不修改 `vector-thrust-lab/`、`high-fidelity-analysis/`、固件；`core.py` 只读复用（import 不拷贝）。

## 5 通信协议（第一版）

WebSocket 端点 `/ws`，消息 JSON。协议版本字段 `v: 1`。

### 5.1 客户端 → 服务端（指令）

| type | payload | 说明 |
|---|---|---|
| `set` | `{key, value}` | 连续量：`thr`(0..1)、`dt`/`df`(rad 摆角或姿态指令)、`dw`(rad/s 航向角速度指令)、`altRef`(m) |
| `toggle` | `{key}` | 开关：`vtolMode`、`altHold`、`useBtrue`、`aero`、`lockXY`、`sasMode`(循环) |
| `reset` | `{mode: 'pose'\|'full'}` | 轻量复位（对齐 resetPoseOnly）或全复位 |
| `ping` | `{}` | 存活/延迟探测 |

### 5.2 服务端 → 客户端（遥测 30Hz）

```json
{
  "v": 1, "type": "telemetry", "t": 12.34,
  "S": {"thr":0.5,"dt":0,"df":0,"dw":0,"dtAct":0,"dfAct":0,"dwAct":0,
        "sasMode":1,"aero":false,"lockXY":false,"vtolMode":true,
        "useBtrue":false,"altHold":true,"altRef":5,"intAlt":0,
        "wf":574.1,"wt":574.1,"omega":[0,0,0],"quat":[0,0.707,0,0.707],"time":12.34},
  "F": {"pos":[0,0,-5],"vel":[0,0,0],"vWorld":[0,0,0],"euler":[0,-1.5708,0]},
  "dyn": {"Fx":6.85,"Fy":0,"Fz":0,"Mx":0,"My":0,"Mz":0,"Tf":3.43,"Tt":3.43,"Qf":0.1,"Qt":0.1},
  "aero": {"V":0.1,"qbar":0,"alpha":0,"beta":0,"Mx":0,"My":0,"Mz":0}
}
```

字段名与 `vector-thrust-lab` 的 `S/F/dyn/aero` 完全一致（前端渲染层零适配直接读）。指令 `dt/df/dw` 语义按 `vtolMode` 分支与现有版一致（巡航=摆角增量/差速百分比；悬停=姿态角/角速度指令）。

### 5.3 时序

- 客户端连接 → 服务端发 `{type:'hello', t, config:{frameHz:100, dt:0.004}}`
- 指令 → 立即 `{type:'ack', ok:true|false, msg}`；无效指令不中断仿真
- 断线 → 仿真暂停（单会话）；重连 → 继续或 `reset full`

## 6 服务端设计

### 6.1 sim.py

```python
class Simulation:
    """仿真引擎：状态 + 主循环（对齐 vector-thrust-lab 的 sim 结构）"""
    S: dict   # 控制/执行状态（字段见协议 §5.2）
    F: dict   # 飞行状态（vel/vWorld/pos/euler/quat 分量）
    def step(self, dt):          # 单子步：apply_sas → 推进(Propulsion.update/forces) → 气动(aero_forces) → rk4_step
    def run_loop(self, hz=100):  # 主循环线程：每 tick 25 子步，30Hz 广播
```

- 复用 `core.py`：`Propulsion`、`aero_forces`、`dynamics_derivatives`、`rk4_step`、`quat_*`、`control_effectiveness`（B_true）
- 姿态四元数约定 `(w,x,y,z)` 与 `core.py` 一致；遥测输出时转 `(x,y,z,w)` 与前端一致

### 6.2 controllers.py

| 控制器 | 对齐源 | 说明 |
|---|---|---|
| `apply_sas`（固定翼 4 模式） | `simulate.py:SASController` + `vector-thrust-lab/control.mjs` 直通/阻尼/闭环 | 复用 SASController，补充 mode 0/2/3 语义 |
| `apply_vtol_hover` | `vector-thrust-lab/control.mjs:applyVtolHover` | **逐位对齐**：qCmd 跟随航向（ψ_est）、qe=qCmd⁻¹⊗q、ωdes=−2·vtolAttKp·qe、内环通道效率符号 |
| `apply_alt_hold` | `control.mjs` 高度环 | P+I 外环 + 油门内环 + 1/√cosγ 倾角补偿 |
| `apply_b_true` | `control.mjs` B_true 分支 | 内环 M_des=diag(I)·btrueK·(ωdes−ω)，分配 Δu=B⁻¹·(M_des−M_cur)，M_cur 当前状态重算 |

**★ 对齐策略**：每个控制器提供 `log_io()` 输出（输入状态/输出执行器指令），`test_controllers.py` 用 vector-thrust-lab 的 Node 测试场景 JSON 逐位比对（容差 1e-9），防两实现漂移。

### 6.3 protocol.py

- 指令白名单校验（非法 key/value 拒绝并 ack false）
- 遥测序列化（numpy → list）
- 版本字段 `v`，未来不兼容变更升版本

## 7 前端设计

### 7.1 复用（从 vector-thrust-lab 拷贝，不改逻辑）

| 模块 | 用途 |
|---|---|
| `browser/scene.mjs`、`effects.mjs`、`theme.mjs` | 场景/特效/主题 |
| `browser/aircraft-view.mjs` | 飞行器渲染（读 sim.S.quat/omega） |
| `browser/hud.mjs` | HUD 数值（读 telemetry 字段） |
| `browser/scope.mjs` | 示波器（接收遥测流） |

### 7.2 新写

| 模块 | 职责 |
|---|---|
| `main.mjs` | WebSocket 连接/重连、遥测 JSON → sim 结构映射（S/F/dyn/aero）、rAF 渲染循环、指令发送 |
| `controls.mjs` | 滑块/按钮面板：语义按 vtolMode 切换（同现有版）、dw 范围 ±80°/s、弹簧回中（本地动画）、按钮文案状态机 |

### 7.3 行为对齐清单（与 standalone 版一致）

悬停：进入默认无翼（aero=false）、thrHover 配平、三滑块弹簧回中（dw 角速度指令 ±80°/s、dt/df 姿态角）、定高（参考滑块实时）、B_true 开关、复位轻量/全量。固定翼：SAS 4 模式、差速 ±30%。

## 8 验证方案

### 8.1 单元测试（pytest）

- 引擎：悬停配平平衡、RK4 保范、定高收敛（复用 vtol.test.mjs 场景数值）
- 控制律符号：三通道修正方向（与 JS 用例同断言）

### 8.2 交叉验证（核心门槛）

`tests/cross_verify.mjs`：同场景（巡航扰动 / 悬停扰动 / 定高 / B_true）分别跑
① `vector-thrust-lab` web core（node）② `py-web-lab` 引擎（node spawn python）→ 轨迹逐点对比，**最大偏差 < 1e-6**（控制律与动力学双重对齐）；失败即 CI 阻断。

### 8.3 浏览器实测（Playwright）

模式切换/滑块/定高/B_true/复位，控制台零错误（复用既有 `output/playwright/` 脚本思路）。

### 8.4 论文复用验证

`py-web-lab` 引擎跑 `run_all.py` 同一场景（SAS 阶跃/INDI 对比），输出与 `high-fidelity-analysis` 一致（同一 core.py，控制律复用 controllers.py——论文与交互版同源）。

## 9 里程碑

| 阶段 | 内容 | 验收 |
|---|---|---|
| M1 服务端 | server/（sim/controllers/protocol）+ pytest + 交叉验证脚本 | 引擎与 web core 轨迹 <1e-6 |
| M2 前端 | web/ 渲染复用 + 控制面板 + 浏览器实测 | 功能对齐清单全过、控制台零错误 |
| M3 收尾 | README/协议文档、启动脚本、与 standalone 共存说明 | 评审通过、提交 |

## 10 风险与开放问题

| 风险 | 缓解 |
|---|---|
| Python 控制律与 JS 逐位漂移 | §8.2 交叉验证作为强制门槛，任何一方改动须通过 |
| 100Hz 主循环 + numpy 每子步开销 | 单用户本地无压力（numpy 向量化，25 子步/帧 < 5ms）；若超载可降 50Hz 步进或降子步数 |
| 前端与 standalone 渲染层漂移 | 渲染模块拷贝后冻结（不随 standalone 演进自动同步，变更走评审） |
| fastapi 版本依赖 | 已装（0.133.1）；requirements.txt 锁定 |
| 双版本并存的分流 | 定位明确：standalone=离线演示；py-web-lab=研究/扩展/论文同源 |

**开放问题**（评审点）：端口选择（默认 8001）；是否需要 `docker` 打包（暂不需要）；先进算法（INDI/MPC）加入顺序。
