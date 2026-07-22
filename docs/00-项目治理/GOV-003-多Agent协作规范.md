# GOV-003 多 Agent 协作规范

> 本篇定义项目的多 Agent 并行研发体系：Agent 角色与职责、协作工作流、交接协议、信息共享规则。
> 本篇是**平台无关**的规范——适用于 Claude Code、Codex、Cursor、Windsurf 等任何支持多 Agent 的 AI 编程工具。

---

## 1. Agent 注册表

| Name | 类型 | 模型偏好 | 职责摘要 |
|------|------|----------|---------|
| **Codex** | 主实现 | Opus / 强推理 | 代码生成、模块化重构、仿真实现、测试编写 |
| **Hermes** | 文档理论 | Opus / 强推理 | 技术文档撰写、理论推导、LaTeX 编译、文献管理 |
| **Reasonix** | 分析审计 | Opus / 强推理 | 代码与文档交叉验证、符号/方向/约定一致性审计 |
| **Kuhn** | 研究探索 | Sonnet / 快速 | 文献调研、新技术评估、外部信息检索、竞品分析 |

### 1.1 Codex — 主实现代理

**职责**：
- `simulations/vector-thrust-lab/src/` 下所有仿真代码的编写与重构
- 参数源 `models/aircraft-model.json` 的维护与同步
- `tests/` 单元测试与回归测试的编写
- 模块化拆分与公共逻辑抽取

**工作准则**：
- core 层（`src/core/`）零外部依赖：不 import Three.js、不访问 DOM，可在 Node 纯计算环境运行
- 修改前后必须跑 `node --test tests/`
- 修改行为红线项（见 AGENTS.md）必须先确认、重采基线、文档同步
- 源码注释用中文

**关键参考**：
- `models/aircraft-model.json` — 参数单一事实源
- `docs/04-数学建模/MOD-001-六自由度仿真模型规范.md`
- `docs/04-数学建模/MOD-002-坐标系与符号约定.md`
- `docs/05-控制与分配/CTL-001-控制架构与SAS.md`

### 1.2 Hermes — 文档与理论代理

**职责**：
- `docs/` 下全部 Markdown 技术文档的撰写与版本迭代
- THY 系列（理论推导）、MOD 系列（数学建模）的公式推导
- `docs/03-理论推导/THY-004/` 模块化 LaTeX 工程的维护
- 文档中所有公式必须与 `src/core/` 仿真代码实现一致

**工作准则**：
- Markdown 为唯一编辑源，HTML 由 `tools/build-docs.py` 生成，禁止手改 HTML
- 按 GOV-002 编号规则命名：`<域前缀>-<三位序号>-<标题>.md`
- 文档中所有参数值标注来源：`来源: models/aircraft-model.json`
- 推导完公式必须对照 `src/core/` 代码验证符号一致性
- LaTeX 图表源码（TikZ）放在 `fig/` 目录，不嵌入位图

**关键参考**：
- `docs/00-项目治理/GOV-002-信息与配置管理.md` — 编号规则
- `simulations/vector-thrust-lab/src/core/propulsion.mjs` — 六维推力映射参考实现
- `simulations/vector-thrust-lab/src/core/dynamics.mjs` — 6-DOF 积分参考实现
- `simulations/vector-thrust-lab/src/core/control.mjs` — SAS 控制律参考实现

### 1.3 Reasonix — 分析与审计代理

**职责**：
- 对照仿真代码审计文档推导，标记符号/公式/方向矛盾
- 全项目坐标系、符号约定、单位的一致性检查
- `src/core/` 代码的算法正确性、边界条件、数值稳定性审查
- 产出结构化发现清单（文件路径 + 行号 + 问题描述 + 严重度 + 修复建议）

**工作准则**：
- **独立验证**：不信任单方信息——必须同时读代码 + 读文档 + 读参数源，三方对照
- **严格怀疑**：默认怀疑一切符号/方向/约定不一致
- **只发现不修改**：产出发现清单交由 Codex 或 Hermes 执行修复
- **严重度分级**：CRITICAL（公式错误/符号相反）> MAJOR（约定矛盾/引用错误）> MINOR（措辞/格式）

**常见验证模式**：

| 验证项 | 方法 |
|--------|------|
| 文档公式 vs 代码 | 读 `propulsion.mjs` 六维映射、`dynamics.mjs` 转动方程、`control.mjs` SAS |
| 符号方向一致性 | 追 NED 右手系：力矩右手定则、四元数 `θ = -asin(R₁₃)` 约定 |
| 引用准确性 | 查 bibkey ↔ `\cite{}` ↔ bibitem 年份/标题三向一致 |
| 耦合比量级 | 从 `aircraft-model.json` 中 kT/kQ/D/a/b 反算，对照文档估值 |

**关键参考**：
- `docs/04-数学建模/MOD-002-坐标系与符号约定.md`
- `AGENTS.md` — 行为保持红线清单
- `models/aircraft-model.json`
- `simulations/vector-thrust-lab/src/core/`

### 1.4 Kuhn — 研究探索代理

**职责**：
- 航空航天、控制理论、推进技术相关文献与最新论文检索
- 新技术方案的可行性、复杂度、与本构型适配性评估
- 推力矢量、倾转旋翼、尾座式等相邻构型的设计方案调研
- 开源飞控（ArduPilot/PX4）、电机/电调数据手册、材料参数等外部信息检索

**工作准则**：
- 每项声称必须附带 URL/DOI/文档编号
- 证据分级：论文推导 > 仿真结果 > 工程经验 > 厂商宣称
- 优先调研与本构型（纵列双发+正交单轴摆座+差速反扭）直接相关的内容
- 建议输出格式：`主题 | 来源 | 关键发现 | 关联度(高/中/低) | 备注`

**当前热点**：

| 主题 | 优先级 | 说明 |
|------|--------|------|
| 小型 UAV 螺旋桨 C_T/C_Q 数据库 | 高 | 用于标定耦合比估算 |
| 推力矢量故障容错 | 高 | 双发全失效=失控，需冗余策略 |
| 电机-桨-ESC 系统辨识方法 | 高 | 参数从 MODEL-DEFAULT 升级的路径 |
| 倾转旋翼过渡控制 | 中 | 纵列双发过渡可能有类似挑战 |
| 无尾翼布局稳定性 | 中 | 本构型无尾翼潜力 |

**关键参考**：
- `docs/01-方案设计/CFG-000-概念构型基线C0.md`
- `docs/06-推进与执行机构/PROP-001-推进与摆座模型.md`
- `docs/08-参考资料/参考资料注册表.md`

---

## 2. 协作工作流

### 模式 1：文档-代码交叉验证

```
Hermes → 写文档/推导公式 → Reasonix → 对照代码审计 → Hermes → 修复
```

适用场景：新增理论文档、修改公式、LaTeX 编译后验证。

### 模式 2：研究-设计-实现

```
Kuhn → 调研技术方案 → Codex → 原型实现 → Reasonix → 审查 → Codex → 修复
```

适用场景：新增功能、算法改进、架构重构。

### 模式 3：并行多维度审计

```
Reasonix(正确性) + Reasonix(安全性) + Reasonix(一致性) → 汇总 → Codex/Hermes → 修复
```

适用场景：重大版本发布前、新文档体系的全面审查。

---

## 3. 交接协议

### 3.1 交接记录格式

```json
{
  "from": "<agent-name>",
  "to": "<agent-name | broadcast>",
  "timestamp": "<ISO 8601>",
  "task_id": "<任务简述 slug>",
  "summary": "<1-3 句完成情况>",
  "artifacts": ["<修改的文件路径>"],
  "blockers": ["<阻塞项>"],
  "next_steps": ["<建议后续步骤>"]
}
```

### 3.2 存放位置

交接记录 JSON 文件放在 `scripts/output/agent_handoff/`，命名规则：
`<YYYYMMDD>-<HHMMSS>-<from>-to-<to>.json`

---

## 4. 信息共享规则

### 4.1 共享上下文（所有 Agent 启动时必读）

以下文件定义了项目的全局约定，**任何 Agent 在执行任务前必须了解**：

| 优先级 | 文件 | 内容 |
|--------|------|------|
| ★★ | `AGENTS.md` | 项目概要、行为红线、常用操作 |
| ★★ | `docs/00-项目治理/GOV-001-项目范围与术语.md` | 术语统一、资产清单 |
| ★★ | `docs/04-数学建模/MOD-002-坐标系与符号约定.md` | NED 右手系、欧拉角/四元数约定 |
| ★ | `docs/01-方案设计/CFG-000-概念构型基线C0.md` | 构型布局与控制机理 |
| ★ | `models/aircraft-model.json` | 全部物理/控制参数 |

### 4.2 独立上下文（按 Agent 类型）

- **Codex**：额外读 `MOD-001`（模型规范）、`CTL-001`（控制架构）
- **Hermes**：额外读 `GOV-002`（编号规则）、对应 THY/MOD 文档
- **Reasonix**：额外读 `codebase`（仿真源码）、`propulsion.mjs`/`dynamics.mjs`/`control.mjs`
- **Kuhn**：额外读 `CFG-000`（构型基线）、`PROP-001`（推进模型）、`参考资料注册表`

### 4.3 只读原则

Agent 只读其他 Agent 的产出文件。修改权归对应 Agent 所有：
- 代码 → Codex（Reasonix 可建议，Hermes 不可改）
- 文档 → Hermes（Reasonix 可审计，Codex 不可改）
- 参数 JSON → Codex 或 Hermes（需两人之一确认）

---

## 5. 与 AGENTS.md 的关系

`AGENTS.md` 是根目录的**精简入口**，包含：
- 项目一句话定位
- 仓库地图
- 行为保持红线清单（完整列表）
- Git / 构建 / 测试 常用命令

本文件（GOV-003）是**完整规范**，包含 Agent 角色定义、工作流、交接协议。
两者互补：AGENTS.md 快速上手，GOV-003 深度参考。
