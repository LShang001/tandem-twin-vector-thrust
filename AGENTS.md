# 纵列双发矢量推力飞行器 — 项目级 Claude Code 指令

## 项目概述

纵列双发、正交单轴摆座、差速反扭滚转的固定翼飞行器概念项目。前电机（拉力式 CW）绕 zb 摆→偏航主控，尾电机（推进式 CCW）绕 yb 摆→俯仰主控，滚转由差速反扭矩差驱动。NED 右手系（x 前/y 右/z 下），四元数显式欧拉右乘积分。

> **成熟度**：概念级仿真，全部参数 `MODEL-DEFAULT`，未台架标定/飞行验证。

## 仓库地图

| 路径 | 内容 |
|------|------|
| `models/aircraft-model.json` | ★ 参数单一事实源（46 参数，含来源/置信度） |
| `simulations/vector-thrust-lab/` | Web 6-DOF 仿真（core 纯计算 + browser 渲染） |
| `docs/00-项目治理/` | GOV-001 范围术语、GOV-002 信息配置 |
| `docs/01-方案设计/` | CFG-000 构型基线、CONOPS-001 运行概念 |
| `docs/03-理论推导/` | THY-001~004 理论文档（旋转数学/推进/动力学/构型原理） |
| `docs/04-数学建模/` | MOD-001~003 模型规范/坐标系/配平稳定性 |
| `docs/05-控制与分配/` | CTL-001~002 控制架构/可控性 |
| `docs/06-推进与执行机构/` | PROP-001 推进摆座模型 |
| `docs/07-验证与确认/` | VER-001/SAFE-001 验证安全 |
| `docs/registers/` | 参数数据手册/假设日志/追溯矩阵/需求/验证/危害注册表 |
| `tools/` | sync-params.py / build-docs.py / build-standalone.py / check-links.py |
| `.claude/agents/` | Agent 类型定义 |

## 关键行为红线（修改必重采基线）

| 红线 | 值 | 位置 |
|------|-----|------|
| 帧 delta 上限 | 0.05 s | `frameCap` |
| 积分子步上限 | 0.004 s | `maxStep` |
| 电机时间常数 | 0.28 s | `tauM` |
| 差速分配 | `ωf=ω0·√(1+Δω)`, `ωt=ω0·√(1−Δω)` | `propulsion.mjs` |
| SAS 摆角限幅 | ±0.4363 rad (25°) | `dMax` |
| SAS 差速限幅 | ±0.7 | `dwMax` |
| 积分限幅 | 俯仰±0.5 rad / 滚转±0.3 rad | `intThMax`/`intPhiMax` |
| 四元数积分 | 显式欧拉右乘 + 每步归一化 | `dynamics.mjs` |
| 欧拉角提取 | `theta = -asin(R13)` | `math.mjs` |
| 渲染/物理分离 | 渲染 ±1.78m ≠ 物理力臂 0.62m | `aircraft-view.mjs` |

## 工作约定

### 代码修改
1. **先读后改**：改参数前读 `models/aircraft-model.json`，改理论前读对应 THY/MOD 文档
2. **参数修改流程**：JSON → `py -3.12 tools/sync-params.py` → `node --test tests/` → 提交
3. **仿真源码**：core/ 层零 Three.js/DOM 依赖，可在 Node 纯计算环境测试
4. **standalone.html**：构建产物（`tools/build-standalone.py` 生成），不手改
5. **编译 LaTeX**：`docs/03-理论推导/THY-004/` 下运行 `build.bat`（XeLaTeX ×3）

### 文档修改
1. **Markdown 源为唯一编辑格式**（非 HTML）
2. **参数在 JSON 中定义**，文档引用参数值时标注 `来源: models/aircraft-model.json`
3. **交叉引用**格式：`见 CTL-001 §3`、`式\ref{eq:six_component}`
4. **新增文档**按 GOV-002 编号规则：`<域前缀>-<三位序号>-<标题>.md`

### Git 约定
- Commit 用中文，尾部 `Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>`
- 按阶段拆分提交，避免超大 diff
- 主分支 `main`，远程 `origin` (GitHub: `LShang001/tandem-twin-vector-thrust`)

## 常用操作

```bash
# 参数漂移检测
py -3.12 tools/sync-params.py --check

# 全量测试
node --test simulations/vector-thrust-lab/tests/

# 仿真启动
cd simulations/vector-thrust-lab && python -m http.server 8080

# LaTeX 编译
cd docs/03-理论推导/THY-004 && build.bat

# 文档构建 + 链接检查
py -3.12 tools/build-docs.py && py -3.12 tools/check-links.py
```

## 子项目

### vector-thrust-lab（Web 仿真）

核心模块（`src/core/`）：
- `parameters.mjs` — sync-params.py 生成，勿手改
- `math.mjs` — vec3/quat/rotateVecByQuat/eulerFromQuat
- `state.mjs` — createSimulationState / reset
- `control.mjs` — applySas（SAS 4 模式 + 差速分配）
- `propulsion.mjs` — stepPropulsion（电机滞后/六维力映射）
- `aerodynamics.mjs` — computeAero（气动模型）
- `dynamics.mjs` — stepPhysics/physicsStep（6-DOF 积分）
- `telemetry.mjs` — 遥测快照

### THY-004（LaTeX 技术文档）

模块化工程：`main.tex` 通过 `\input` 组装 `preamble.tex` + `ch/` (9章) + `fig/` (6 TikZ图) + `ref.bib.tex`。
与仿真代码的交叉验证记录见 `THY-004/README.md §勘误记录`。
