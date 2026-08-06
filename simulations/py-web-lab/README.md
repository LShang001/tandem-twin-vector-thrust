# py-web-lab — Python 引擎 + Web 可视化仿真

> 统一架构研究版：**模型与控制律全部在 Python（复用 `simulations/high-fidelity-analysis/core.py`）**，
> 浏览器只做指令发送与渲染。与 `vector-thrust-lab`（standalone 离线版）并存：
> standalone 保留零依赖双击演示，py-web-lab 面向算法扩展与论文同源研究。

## 架构

```
┌────────────────────────────┐   WebSocket (JSON)   ┌─────────────────────────────┐
│ 浏览器（web/）              │ ◀──────────────────▶ │ Python 服务端（server/）      │
│  Three.js 渲染（零计算）     │   init/reset/set/     │  FastAPI + uvicorn           │
│  指令透传：滑块/按钮 → 服务端  │   pulse/step 消息      │  Simulation（core.py 引擎）  │
│  状态渲染：step 响应 → 渲染    │                     │  controllers.py（控制律）    │
└────────────────────────────┘                     └─────────────────────────────┘
```

- **引擎** `server/sim.py`：六自由度动力学 + 推进 + 气动（复用 `high-fidelity-analysis/core.py`，参数同源 `models/aircraft-model.json`）
- **控制律** `server/controllers.py`：固定翼 SAS（4 模式）+ VTOL 悬停（四元数级联 + 定高 + B_true 分配），逐位对齐 `vector-thrust-lab/src/core/control.mjs`；每步填充 `S.dbg` 调试中间量（qe/ωdes/Δu，CLI 打印用）。**可插拔控制律**（`S.ctrl`，UI「控制律」按钮切换）：
  - `sas`：默认（巡航 SAS + 悬停级联）
  - `indi`：巡航 INDI 增量动态逆（移植 `high-fidelity-analysis/simulate.py:INDIController`，论文 ch/11；外环姿态+内环 K_rate=3+混合角加速度+在线 Jacobian B⁻¹）
  - `lqr`：悬停 LQR（移植 `controllers.py:QuatLQRController`，论文 ch/14；CARE+Bryson 权重，航向通道追踪角速度指令）
  - `adrc`：悬停内环 ADRC（移植固件 `TandemVec_ADRC.h`：ESO2+PD，ωc=6/ωo=8/b0 取通道名义增益，论文 ch/16；外环 qe→ω_ref 与级联同构）
  - 切换自动规范化：进悬停时 indi→sas、回巡航时 lqr/adrc→sas；B_true 仅 sas 模式可用
- **协议** `server/protocol.py`：JSON 消息（见 §协议），白名单字段带有限性/范围/布尔严格校验（NaN/Inf/越界/未知字段一律拒绝）
- **CLI** `server/cli.py`：无浏览器调试接口（见 §CLI）
- **前端** `web/`：渲染模块复用 `vector-thrust-lab/src/browser/`（scene/effects/aircraft-view/hud/scope/theme，零 core 依赖）

## 快速开始

```bash
# 依赖（首次）
pip install -r requirements.txt

# 启动（Windows）
server\start.bat
# 或：cd server && python -m uvicorn main:app --host 127.0.0.1 --port 8090

# 浏览器打开
http://127.0.0.1:8090/
```

## CLI（调试 / 测试 / 分析，不依赖浏览器）

```bash
cd server

python cli.py state --name vtol                    # 初始状态快照（JSON）
python cli.py scene --name vtol_btrue --secs 8 --out t.json --pulse 0,0.3,0.2
                                                   # 跑场景 → 轨迹 JSON
python cli.py step --n 100 --name vtol_btrue --pulse 0,0.3,0.2 --print-every 10
                                                   # 逐步推进，打印 qe/ωdes/Δu 控制律中间量
python cli.py compare a.json b.json [--tolerance 1e-6]
                                                   # 轨迹对比（字段级最大偏差）
```

场景名：`vtol / vtol_pert / vtol_alt / vtol_btrue / vtol_dw / cruise_trim / cruise_sas`；
`--S 'thr=0.5,dt=0.1,dw=0.3,sasMode=3'` 直接注入状态字段（含 aero/altHold/paused 等布尔）。

## 验证

| 层 | 方法 | 命令 |
|---|---|---|
| 引擎+控制律 与 web core 交叉一致 | 6 场景同轨迹对比（门槛 <1e-6，实际 4.6e-13） | `cd tests/cross && python run_py.py && node run_js.mjs && node compare.mjs` |
| 协议链路 | 悬停/定高/扰动/复位冒烟 | `python tests/smoke_ws.py` |
| 浏览器端到端 | Playwright 6 步流程 + 视觉审核 | `python tests/pw_verify.py` |

## 协议

client → server：

| 消息 | 作用 |
|---|---|
| `{"cmd":"init"}` | 全量状态 + 参数快照（`params` 43 个） |
| `{"cmd":"reset","mode":"cruise"\|"vtol"\|"pose"}` | 全复位 / 进入悬停 / 轻量复位（保留模式输入） |
| `{"cmd":"set","S":{"thr":…,"dt":…,"sasMode":…,"paused":…}}` | 白名单字段写入（thr/dt/df/dw/sasMode/aero/lockXY/vtolMode/useBtrue/altHold/altRef/intTh/intPhi/intAlt/paused；NaN/Inf/越界/未知字段拒绝） |
| `{"cmd":"pulse","omega":{"x":…,"y":…,"z":…}}` | 角速度扰动注入 |
| `{"cmd":"step","dt":0.016}` | 推进一帧（服务端按 `maxStep` 分子步；`paused=true` 时不推进仅回状态） |

## 前端功能清单（与 standalone 纯 web 版对齐）

| 功能 | 说明 |
|---|---|
| 演示按钮 ×4 | 俯仰/偏航/滚转/综合演示（指令由前端按时间序列透传，滑块或模式切换自动停止） |
| 控制律切换 | 「控制律」按钮：巡航 SAS⇄INDI、悬停 SAS⇄LQR⇄ADRC（回归见 `tests/test_algorithms.py`，6/6 通过） |
| 弹簧回中 | 悬停自稳（sasMode≠0）与固定翼角速度闭环（sasMode=3）下 dt/df/dw 滑块松手自动回 0（摇杆式）；**「回中」按钮可关闭**（松手停留） |
| 暂停/继续 | `paused` 字段；暂停时仿真冻结且渲染动画（螺旋桨/箭头）同步停 |
| 悬停模式 | 机头朝天 + 无翼 + 悬停配平；滑块语义切换（姿态指令/航向角速度） |
| 定高 / B_true | 悬停模式可用（B_true 需自稳开） |
| 水平约束 | `lockXY`：水平速度不积分（服务端语义与 JS 版一致） |
| 复位 RESET | 轻量复位：只复位位置/姿态/速度，保留模式与输入 |

server → client：`{"type":"state","S":{…},"F":{…},"dyn":{…},"aero":{…},"t":…}`（init 附加 `params`）、`{"type":"error","msg":…}`。

**步进模式**：客户端 rAF 驱动请求-响应（服务端无自主循环，无状态并发问题）；响应未达时跳过该帧渲染（自适应帧率）。新增算法只需扩展服务端 `controllers.py`，前端零改动。

## 目录

```
server/        sim.py（引擎）· controllers.py（控制律）· protocol.py（协议）· main.py（FastAPI）· start.bat
web/           index.html · src/main.mjs（WS 客户端+渲染装配）· src/browser/（复用渲染模块）· vendor/（three-r170）· css/
tests/         cross/（交叉验证三件套）· smoke_ws.py（协议冒烟）· pw_verify.py（浏览器实测）
```
