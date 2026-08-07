# AGENTS.md — 纵列双发矢量推力飞行器

> 项目背景见 `README.md`——不在这里重复。
> 多 Agent 协作问题或新 Agent 入场时，读 `docs/00-项目治理/GOV-003-多Agent协作规范.md`。

---

## 项目

纵列双发正交单轴矢量推力 + 差速反扭滚转的固定翼概念飞行器。
前电机（拉力 CW）绕 zb 摆 → 偏航主控；尾电机（推进 CCW）绕 yb 摆 → 俯仰主控。
NED 右手系（x 前 / y 右 / z 下），四元数RK4积分。
**成熟度：实机首飞验证通过（2026-08），进入调参优化阶段；仿真参数仍 MODEL-DEFAULT 未标定，固件参数实机逐轮调整中。**

## 命令

```bash
# 参数同步（修改 aircraft-model.json 后必须执行）
py -3.12 tools/sync-params.py          # 生成 parameters.mjs
py -3.12 tools/sync-params.py --check  # 漂移检测

# 测试
node --test "simulations/vector-thrust-lab/tests/*.test.mjs"         # 全量
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
| `models/aircraft-model.json` | ★ 唯一参数源，52 参数。所有参数值以此为准，不可在代码中硬编码 |
| `simulations/vector-thrust-lab/src/core/` | 纯计算层，零 Three.js/DOM 依赖，Node 可单独测试 |
| `simulations/vector-thrust-lab/src/core/parameters.mjs` | sync-params.py 生成，**禁止手改** |
| `simulations/vector-thrust-lab/standalone.html` | 构建产物（build-standalone.py 生成），**禁止手改** |
| `docs/04-数学建模/MOD-002-坐标系与符号约定.md` | NED 右手系、theta=-asin(R13)、渲染≠物理力臂 |
| `docs/03-理论推导/THY-004/` | 模块化 LaTeX 工程，编译需要 XeLaTeX ×3 |

## 原则

1. **先读后改** — 改参数前读 `models/aircraft-model.json`；改推进/动力学/控制逻辑前读 `simulations/vector-thrust-lab/src/core/` 对应模块
2. **复用优于新建** — 检查 core 层是否有现成函数（math/state/control/propulsion/aerodynamics/dynamics），不要重复实现
3. **外科手术式修改** — 只改目标代码，不顺手重构、不扩范围
4. **修改后必须跑测试** — `node --test simulations/vector-thrust-lab/tests/`，全绿再提交

## 行为红线

修改以下任一项，必须：重采回归基线 `tests/fixtures/regression-baseline.json`、同步更新文档、提交说明原因。完整清单及变更审核流程见 `docs/registers/行为保持红线清单.md`。

核心红线（逐条对照）：
- 帧 delta≤0.05s / 子步≤0.004s / 电机 τm=0.28s
- 差速公式 ωf=ω0√(1+Δω)（保持不变）
- 四元数RK4积分 + 每阶段保范投影
- theta=−asin(R₁₃)
- 力臂分离（渲染±1.78m ≠ 物理0.315m）
- SAS 限幅 ±25°/±0.7（积分俯仰±0.5rad / 滚转±0.3rad）
- 空速下限 0.5m/s、地面 pos.z>6.2

## 边界

**绝不修改**：
- `standalone.html` — 构建产物，改源码后必须用 `build-standalone.py` 重新生成
- `parameters.mjs` — sync-params.py 生成，参数修改必须改 `aircraft-model.json` 再同步
- `simulations/vector-thrust-lab/docs/*.html` — 文档构建产物，源码在 `docs/` Markdown

**修改前必须确认**：
- 行为红线项 → 逐条对照红线清单
- `docs/` 中所有参数值必须标注 `来源: models/aircraft-model.json`

## 约定

- **文档**：Markdown 为唯一编辑源；编号规则 `域前缀-三位序号-标题.md`；交叉引用 `见 CTL-001 §3`
- **Git**：中文 commit，尾部 `Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>`；main 分支；按阶段拆分
- **LaTeX**：TikZ 源码存 `fig/`，公式推导后对照 `src/core/` 验证符号一致性
- **Skill**：创建/修改项目 Skill 时走 `skill-creator-plus` 流程（草稿→清单审查），不裸调 `install_skill`。Skill 存 `.agents/skills/`（Agentskills.io 标准），不存 `.reasonix/skills/`

## 踩坑记录

- `dyn.Fx = Tf*cf + Tt*ct`（尾推正贡献，非负）
- `dyn.Fz = -Tt*st`（δt>0→受力向上→Fz<0）
- 前后转子角动量反向：前 +x_b（CW），尾 -x_b（CCW 反转）
- 陀螺力矩俯仰→偏航耦合轴为 z_b（非 y_b）
- Python 仿真循环内禁止用 `dt` 作尾摆角指令变量名——会遮蔽时间步 `dt`，导致积分冻结、图全空（`controllers.py:simulate_vtol` 已改 `dt_cmd`，来源：2026-07-22）
- `theta=-asin(R₁₃)` 的负号使显示角运动学为 θ̇≈−q、φ̇≈−p（**非**标准 3-2-1 的 +q/+p）——SAS 姿态环取负号正依赖此约定。判断 SAS/控制律符号正确性前必须先在项目约定下数值推导运动学关系；直接套教科书标准关系会得出"姿态环正反馈"的错误结论（来源：2026-07-23，曾误报第 7 章后由三层数值验证撤销）
- 四元数误差**表达系陷阱**：`qe = q ⊗ qCmd⁻¹` 的旋转轴在 NED 系（相似变换），大姿态（悬停机头朝天）下 x/y/z 轴错位、控制张冠李戴；机体系误差必须用 `qe = qCmd⁻¹ ⊗ q`（来源：2026-08-04 VTOL 悬停模式，航向指令 19° 不收敛调试出）。**补充（2026-08-05）**：`qCmd⁻¹ ⊗ q` 严格表达在 **qCmd 系**，仅当 qCmd≈q（误差小）时一阶等价机体系——若某通道不做姿态回中（如悬停航向=纯角速度指令），qCmd 与机体系绕该轴自由错位，其余通道误差分量被 cosψ 调制（ψ=90° 失效、180° 反向）。**修复范式：让 qCmd 跟随被释放的自由度**（悬停航向 ψ_est = π/2 − atan2(yb.y, yb.x)，yb 为机体 y 轴 NED 表达），保证 qe 恒小
- 四元数**双覆盖/最短路径**：`Ry(200°)⁻¹ ≡ Ry(+160°)`，qe.w<0 取反后沿短弧修正；测试断言方向前必须先做 S³ 几何推导，勿直觉判断（来源：2026-08-04，断言曾写反）
- 悬停构型下 3-2-1 欧拉显示退化：机头朝天（θ≈−90°）时绕 x 转 20° 显示为 θ=−70°/φ=90°（奇异重新分配），**勿用欧拉显示判断悬停姿态**，以四元数为准（来源：2026-08-04 视觉验证）
- `quat(x,y,z,w)` 分量顺序：绕 x 转 = `(sin,0,0,cos)`、绕 z = `(0,0,sin,cos)`——x/z 分量写反是隐蔽笔误，测试三通道符号用例可防（来源：2026-08-04）

## 知识沉淀协议

> 本协议在对话中自动生效。不是规章制度——是给你未来会话的自己的**记忆外挂**。
> 遵守这些规则 = 帮未来的自己。1 分钟的记录 = 下次会话省 10 分钟重学。
> 如果此轮触发检查没有值得记录的内容，沉默——不作声比废话强。

| 规则 | 触发条件 | 行为 |
|------|---------|------|
| **P1** | 用户说 "记住"/"记下来"/"沉淀" | 写 `docs/memory/YYYY-MM-DD-<摘要>.md`（3-5 要点），追问是否同步到 AGENTS.md |
| **P2** | 用户纠正错误（"不对"/"错了"/"应该是"） | 如学到项目特定知识 → 追加到 §踩坑记录：`<错误> — <正确>（来源：<日期>）` |
| **P3** | 自主发现非显然约定 | 追加到 AGENTS.md 最相关节，简短告知用户 |
| **P4** | 同一流程被指导 ≥3 次 | 向用户提议用 **skill-creator-plus** 封装为项目 Skill |
| **P5** | 任务完成 / 用户说 "好了" | 自问"有什么值得记录？"——**答案为空则沉默**，不要为仪式感编造内容 |
