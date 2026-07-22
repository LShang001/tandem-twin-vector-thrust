# AGENTS.md — 纵列双发矢量推力飞行器

> **AI Agent 入口文件**。平台无关，适用于 Claude Code、Codex、Cursor、Windsurf 等任何 AI 编程工具。
> Agent 角色定义、协作工作流、交接协议详见 `docs/00-项目治理/GOV-003-多Agent协作规范.md`。

---

## 项目概要

纵列双发、正交单轴摆座、差速反扭滚转的固定翼飞行器概念项目。
- 前电机（拉力式 CW）绕 zb 摆→偏航主控
- 尾电机（推进式 CCW）绕 yb 摆→俯仰主控
- 滚转由差速反扭矩差驱动
- NED 右手系（x 前 / y 右 / z 下），四元数显式欧拉右乘积分

> **成熟度**：概念级仿真，全部参数 `MODEL-DEFAULT`，未台架标定/飞行验证。

## 仓库地图

| 路径 | 内容 |
|------|------|
| `models/aircraft-model.json` | ★ 参数单一事实源（46 参数，含来源/置信度） |
| `simulations/vector-thrust-lab/` | Web 6-DOF 仿真（`src/core/` 纯计算 + `src/browser/` 渲染） |
| `docs/00-项目治理/` | GOV-001 范围术语、GOV-002 信息配置、GOV-003 多Agent协作 |
| `docs/01-方案设计/` | CFG-000 构型基线、CONOPS-001 运行概念 |
| `docs/03-理论推导/` | THY-001~004 理论文档 |
| `docs/04-数学建模/` | MOD-001~003 模型规范/坐标系/配平 |
| `docs/05-控制与分配/` | CTL-001~002 控制架构/可控性 |
| `docs/06-推进与执行机构/` | PROP-001 推进摆座模型 |
| `docs/07-验证与确认/` | VER-001/SAFE-001 验证安全 |
| `docs/registers/` | 参数数据手册/假设日志/追溯矩阵/需求/验证/危害注册表 |
| `tools/` | sync-params.py / build-docs.py / build-standalone.py / check-links.py |
| `scripts/output/agent_handoff/` | Agent 交接记录 |

## 行为保持红线

修改以下任何一项**必须**重采回归基线、文档同步、并在提交中说明原因：

| 红线 | 值 | 所在位置 |
|------|-----|----------|
| 帧 delta 上限 | 0.05 s | `frameCap` |
| 积分子步上限 | 0.004 s | `maxStep` |
| 电机时间常数 | 0.28 s | `tauM` |
| 差速分配 | `ωf=ω0√(1+Δω)`, `ωt=ω0√(1−Δω)` | `propulsion.mjs` |
| SAS 摆角限幅 | ±0.4363 rad (25°) | `dMax` |
| SAS 差速限幅 | ±0.7 | `dwMax` |
| 积分限幅 | 俯仰 ±0.5 rad / 滚转 ±0.3 rad | `intThMax`/`intPhiMax` |
| 四元数积分 | 显式欧拉右乘 + 每步归一化 | `dynamics.mjs` |
| 欧拉角提取 | `theta = -asin(R13)` | `math.mjs` |
| 渲染/物理分离 | 渲染几何 ±1.78m ≠ 物理力臂 0.62m | `aircraft-view.mjs` |
| 空速下限 | 0.5 m/s | `vMin` |
| 地面约束 | `pos.z > 6.2` 且 `vWorld.z > 0` | `groundZ` |

## 工作约定

### 代码
- **先读后改**：改参数前读 `models/aircraft-model.json`，改逻辑前读对应 THY/MOD 文档
- **参数修改流程**：JSON → `py -3.12 tools/sync-params.py` → `node --test tests/` → 提交
- **core/ 层零依赖**：`src/core/` 不 import Three.js / DOM，可在 Node 纯计算环境测试
- **standalone.html**：`tools/build-standalone.py` 生成的构建产物，不可手改
- **LaTeX**：`docs/03-理论推导/THY-004/build.bat`（XeLaTeX ×3）

### 文档
- **Markdown 为唯一编辑源**，HTML 由 `tools/build-docs.py` 生成
- **参数溯源**：文档中所有参数值标注 `来源: models/aircraft-model.json`
- **编号规则**：`<域前缀>-<三位序号>-<标题>.md`（详见 GOV-002）
- **交叉引用**：`见 CTL-001 §3`，不写死文件名

### Git
- Commit 用中文，尾部 `Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>`
- 按阶段拆分，避免超大 diff
- 主分支 `main`，远程 `origin`（`LShang001/tandem-twin-vector-thrust`）

## 常用命令

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

## Agent 体系

| 文档 | 内容 |
|------|------|
| **GOV-003** | Agent 角色定义（Codex/Hermes/Reasonix/Kuhn）、协作工作流、交接协议 |
| `scripts/output/agent_handoff/` | Agent 间交接记录（JSON 格式） |
