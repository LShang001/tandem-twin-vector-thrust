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
- **控制律** `server/controllers.py`：固定翼 SAS（4 模式）+ VTOL 悬停（四元数级联 + 定高 + B_true 分配），逐位对齐 `vector-thrust-lab/src/core/control.mjs`；每步填充 `S.dbg` 调试中间量（qe/ωdes/Δu，CLI 打印用）
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

server → client：`{"type":"state","S":{…},"F":{…},"dyn":{…},"aero":{…},"t":…}`（init 附加 `params`）、`{"type":"error","msg":…}`。

**步进模式**：客户端 rAF 驱动请求-响应（服务端无自主循环，无状态并发问题）；响应未达时跳过该帧渲染（自适应帧率）。新增算法只需扩展服务端 `controllers.py`，前端零改动。

## 目录

```
server/        sim.py（引擎）· controllers.py（控制律）· protocol.py（协议）· main.py（FastAPI）· start.bat
web/           index.html · src/main.mjs（WS 客户端+渲染装配）· src/browser/（复用渲染模块）· vendor/（three-r170）· css/
tests/         cross/（交叉验证三件套）· smoke_ws.py（协议冒烟）· pw_verify.py（浏览器实测）
```
