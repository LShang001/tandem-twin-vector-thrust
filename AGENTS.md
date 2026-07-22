# AGENTS.md — 纵列双发矢量推力飞行器

> AI Agent 入口文件。平台无关（Claude Code / Codex / Cursor / Copilot / Windsurf）。
> Agent 角色定义、协作工作流、交接协议 → `docs/00-项目治理/GOV-003-多Agent协作规范.md`。

---

## 项目

纵列双发正交单轴矢量推力 + 差速反扭滚转的固定翼概念飞行器。
前电机（拉力 CW）绕 zb 摆 → 偏航主控；尾电机（推进 CCW）绕 yb 摆 → 俯仰主控。
NED 右手系（x 前 / y 右 / z 下），四元数显式欧拉右乘积分。
**成熟度：概念级仿真，全部参数 MODEL-DEFAULT，未经标定。**

## 原则

1. **先读后改** — 改参数前读 `models/aircraft-model.json`；改推进/动力学/控制逻辑前读 `simulations/vector-thrust-lab/src/core/` 对应模块
2. **复用优于新建** — 检查 core 层是否有现成函数（math/state/control/propulsion/aerodynamics/dynamics），不要重复实现
3. **外科手术式修改** — 只改目标代码，不顺手重构、不扩范围
4. **修改后必须跑测试** — `node --test simulations/vector-thrust-lab/tests/`，全绿再提交

## 命令

```bash
# 参数同步（修改 aircraft-model.json 后必须执行）
py -3.12 tools/sync-params.py          # 生成 parameters.mjs
py -3.12 tools/sync-params.py --check  # 漂移检测

# 测试
node --test simulations/vector-thrust-lab/tests/                    # 全量
node --test simulations/vector-thrust-lab/tests/propulsion.test.mjs # 单模块

# 仿真
cd simulations/vector-thrust-lab && python -m http.server 8080

# LaTeX（修改 ch/*.tex 后编译验证）
cd docs/03-理论推导/THY-004 && build.bat

# 文档构建
py -3.12 tools/build-docs.py && py -3.12 tools/check-links.py
```

## 关键路径（AI 无法自行推断的）

| 路径 | 为什么必须知道 |
|------|---------------|
| `models/aircraft-model.json` | ★ 唯一参数源，46 参数。所有参数值以此为准，不可在代码中硬编码 |
| `simulations/vector-thrust-lab/src/core/` | 纯计算层，零 Three.js/DOM 依赖，Node 可单独测试 |
| `simulations/vector-thrust-lab/src/core/parameters.mjs` | sync-params.py 生成，**禁止手改** |
| `simulations/vector-thrust-lab/standalone.html` | 构建产物（build-standalone.py 生成），**禁止手改** |
| `docs/04-数学建模/MOD-002-坐标系与符号约定.md` | NED 右手系、theta=-asin(R13)、渲染≠物理力臂 |
| `docs/03-理论推导/THY-004/` | 模块化 LaTeX 工程，编译需要 XeLaTeX ×3 |

## 行为红线

修改以下任一项，必须：重采回归基线 `tests/fixtures/regression-baseline.json`、同步更新文档、提交说明原因。

- 帧 delta ≤ 0.05 s / 子步 ≤ 0.004 s / 电机 τm = 0.28 s
- 差速分配公式 `ωf=ω0√(1+Δω), ωt=ω0√(1−Δω)`（保持不变）
- 四元数积分显式欧拉右乘 + 每步归一化
- theta = −asin(R₁₃)（欧拉角提取约定）
- 渲染几何电机位置 ±1.78m ≠ 物理力臂 0.62m（分离设计）
- SAS 限幅：摆角 ±25°、差速 ±0.7、积分 俯仰 ±0.5 rad / 滚转 ±0.3 rad
- 空速下限 0.5 m/s、地面 pos.z > 6.2

完整红线清单与变更流程 → `docs/registers/行为保持红线清单.md`。

## 边界

**绝不修改**：
- `standalone.html` — 构建产物，改源码后必须用 `build-standalone.py` 重新生成
- `parameters.mjs` — sync-params.py 生成，参数修改必须改 `aircraft-model.json` 再同步
- `simulations/vector-thrust-lab/docs/*.html` — 文档构建产物，源码在 `docs/` Markdown

**修改前必须确认**：
- 行为红线项 → 逐条对照红线清单
- `docs/` 中所有参数值必须标注 `来源: models/aircraft-model.json`

## 约定

- **参数修改流程**：`aircraft-model.json` → `sync-params.py` → `node --test tests/` → commit
- **文档**：Markdown 为唯一编辑源；编号规则 `域前缀-三位序号-标题.md`；交叉引用 `见 CTL-001 §3`
- **Git**：中文 commit，尾部 `Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>`；main 分支；按阶段拆分
- **LaTeX**：TikZ 源码存 `fig/`，公式推导后对照 `src/core/` 验证符号一致性

## 踩坑记录

- `dyn.Fx = Tf*cf + Tt*ct`（尾推正贡献，非负）
- `dyn.Fz = -Tt*st`（δt>0→受力向上→Fz<0）
- 前后转子角动量反向：前 +x_b（CW），尾 -x_b（CCW 反转）
- 陀螺力矩俯仰→偏航耦合轴为 z_b（非 y_b）
