# AGENTS.md — 纵列双发矢量推力飞行器

> 项目背景见 `README.md`——不在这里重复。
> 多 Agent 协作问题或新 Agent 入场时，读 `docs/00-项目治理/GOV-003-多Agent协作规范.md`。

---

## 项目

纵列双发正交单轴矢量推力 + 差速偏航的固定翼概念飞行器。
★执行器分配（2026-08-07 轴置换后，悬停构型语义）：**上摆=滚转主控 / 下摆=俯仰主控 / 差速 Δω=偏航主控**
（旧描述"前摆=偏航、差速反扭滚转"为巡航读法，已废止；锚点=手动模式摇杆直通实机验证）。
悬停（机头朝天）时前电机在最高点=上摆、尾电机在最低点=下摆；巡航构型下同两摆即前摆/尾摆。
前电机（拉力 CW）→ 上摆 δf（滚转）；尾电机（推进 CCW）→ 下摆 δt（俯仰）；差速 → 偏航。
NED 右手系（x 前 / y 右 / z 下），四元数RK4积分。
**成熟度：实机首飞验证通过（2026-08），进入调参优化阶段；仿真参数仍 MODEL-DEFAULT 未标定，固件参数实机逐轮调整中。**

## 命令

```bash
# 参数同步（修改 aircraft-model.json 后必须执行）
py -3.12 tools/sync-params.py          # 生成 parameters.mjs
py -3.12 tools/sync-params.py --check  # 漂移检测

# 测试（日常按改动范围只跑相关子测试；全量 run_all.sh 与 SIL 矩阵留到里程碑/跨模块改动/用户要求）
node --test "simulations/vector-thrust-lab/tests/*.test.mjs"         # 全量
node --test simulations/vector-thrust-lab/tests/propulsion.test.mjs # 单模块
bash TandemVec_FCS/test_host/run_all.sh                              # 宿主全量回归（耗时）

# 固件算法直调（仿真脚本调固件 EKF/控制，不在 sim/ 复刻算法）
bash TandemVec_FCS/test_host/fw_algo/build_fw_algo.sh   # 共享库（4 组 ABI，输出 tmp/tv_fw_algo）
py -3.12 TandemVec-EKF-Report/sim/run_fw_gnc_demo.py    # 控制链冒烟
py -3.12 TandemVec-EKF-Report/sim/run_fw_nav_demo.py    # 导航步进冒烟
py -3.12 TandemVec-EKF-Report/sim/run_fw_gnc_parity_suite.py  # SIL↔直调库逐拍对拍

# 上位机 GCS（AnoCom 协议；桌面窗口/纯后端/无硬件测试）
cd TandemVec_FCS/GCS && py -3.12 app.py
cd TandemVec_FCS/GCS && py -3.12 -m uvicorn server.main:app --port 8091
cd TandemVec_FCS/GCS && py -3.12 -m pytest tests/ -q

# 串口调试统一入口（全部散脚本已并入，见 TandemVec_FCS/tools/README.md）
cd TandemVec_FCS/GCS && py -3.12 server/cli.py --port COM10 param get|set <name> [value]
cd TandemVec_FCS/GCS && py -3.12 server/cli.py --port COM10 link|sniff euler|stick|baudscan

# 仿真 / LaTeX / 文档 / 注释门禁
cd simulations/vector-thrust-lab && python -m http.server 8080
cd TandemVec-Paper && build.bat
py -3.12 tools/build-docs.py && py -3.12 tools/check-links.py
py -3.12 tools/check-code-comments.py --check     # GOV-004 门禁；--area fw-nav 单区域
```

## 关键路径（AI 无法自行推断的）

| 路径 | 为什么必须知道 |
|------|---------------|
| `models/aircraft-model.json` | ★ 唯一参数源（59 参数），所有参数值以此为准，不可硬编码 |
| `TandemVec_FCS/include/FlightCtrlParams.h` | ★ 固件参数唯一事实源（`kFlightCtrlParams`），固件与宿主测试共用 |
| `TandemVec_FCS/test_host/fw_algo/` | ★ 固件直调接口（C ABI + Python `sim/fwapi`）：`build_fw_algo.sh` 的 `CORE_SOURCES` 是唯一固件源清单；**禁止在 `sim/` 复刻算法** |
| `docs/08-参考资料/` | ★ 理论资料第一入口：`books/REF-120/121`（秦永元两本教材 MinerU 全文+分章）、`notes/00–08` 主题深读、`参考资料注册表.md`；引用格式 `[REF-120 §9.9.2 (9.9.43c) p.334]` |
| `docs/04-数学建模/MOD-002-坐标系与符号约定.md` | NED 右手系、`theta=-asin(R13)`、渲染力臂≠物理力臂 |
| `TandemVec_FCS/GCS/` + `TandemVec_FCS/docs/PROTOCOL-001-AnoCom官方通信协议.md` | 上位机协议唯一实现（`anocom.py`）+ 官方协议手册与差异对照表 |

其它：`TandemVec-Paper/`（论文工程，XeLaTeX×5 构建）、`simulations/vector-thrust-lab/src/core/`（仿真纯计算层，Node 可单测）、`TandemVec_FCS/include/AirframeModel.h`（机型数据驱动执行器模型）、`docs/00-项目治理/GOV-004-代码注释与理论对照规范.md`（注释五段式 + `@ref` 格式，改公式性代码前读）。

## 原则

1. **先读后改** — 改参数前读 `models/aircraft-model.json`；改推进/动力学/控制逻辑前读 `simulations/vector-thrust-lab/src/core/` 对应模块
2. **复用优于新建** — 检查 core 层是否有现成函数（math/state/control/propulsion/aerodynamics/dynamics），不要重复实现；**仿真侧算法一律经 `fwapi` 直调固件实现，不要在 `sim/` 另写一份**；**仿真积分统一四阶 Runge-Kutta，不用欧拉积分**（宿主模型、SIL `Plant::step`、宿主刚体测试一致；2026-10-31 用户要求）
3. **外科手术式修改** — 只改目标代码，不顺手重构、不扩范围
4. **修改后必须跑测试** — `node --test simulations/vector-thrust-lab/tests/`，全绿再提交
5. **改代码必改注释（GOV-004）** — 公式性实现必须带 `@formula` + `@ref`（章节+式号+页码）；新文件必须有 L0 文件头；提交前跑 `py -3.12 tools/check-code-comments.py --check`
6. **查理论先读本地资料库** — `docs/08-参考资料/`（`notes/` 主题深读 + `books/REF-120/121` 章节）优先于网络检索，网络只用于补缺（2026-11-01 教训）
8. **算法测试两阶段：先高精度理想模型验正确性，再真实传感器模型验工程效果** — 新增/修改估计、导航、控制算法时，先在高精度理想模型（零噪声/零偏/延迟，WGS84 正常重力、标准 ω_ie、解析或高精度数值真值）下断言算法自洽与收敛（例：理想 IMU 1 h 漂移 <2 m、参数辨识误差与理论值一致、TRIAD 全姿态误差 <1e-3°）；通过后再注入真实传感器噪声/零偏/振动/延迟/失锁评估工程效果。**不得跳过理想阶段**——否则无法区分“算法错”“传感器限制”“测试构造错”（2026-11-01 曾把重力口径差伪装的 24 km 漂移误判为机械编排缺陷）。理想阶段失败必须修算法；真实阶段不达标先归因再决定是否改算法。
7. **改固件/宏/模块后同步文档** — 新增或删除宏、模块、测试后，同一提交内更新：`TandemVec_FCS/AGENTS.md` 导航宏一览、`docs/04-数学建模/代码理论核查/上板标定与验证清单` §7 宏状态表、`卫惯组合导航优化总结` §29 模块表、`代码理论核查/README.md` §0 研究文档索引；过时结论在原文标注“已并入/已废止”而非直接删（2026-11-01 用户要求）

## 行为红线

修改以下任一项，必须：重采回归基线 `tests/fixtures/regression-baseline.json`、同步更新文档、提交说明原因。完整清单及变更审核流程见 `docs/registers/行为保持红线清单.md`。

核心红线（逐条对照）：
- 帧 delta≤0.05s / 子步≤0.004s / 电机 τm=0.28s
- 差速公式 ωf=ω0√(1+Δω)（保持不变）
- 四元数RK4积分 + 每阶段保范投影
- theta=−asin(R₁₃)
- 力臂分离（渲染±1.78m ≠ 物理0.315m）
- SAS 限幅 ±25°/±0.7（**模型概念窗口，仿真域**；固件实机包线摆角 ±15°=0.2618 rad，两套口径不可互相外推，见 DEC-002 A-04——积分俯仰±0.5rad / 滚转±0.3rad）
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
- **Git**：中文 commit，尾部 `Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>`；main 分支（**不另开实验/升级分支**）；按阶段拆分；**阶段完成即提交并主动 `git push origin main`**（GitHub 需代理：`git -c http.proxy=http://127.0.0.1:7897 push`）
- **LaTeX**：TikZ 源码存 `fig/`，公式推导后对照 `src/core/` 验证符号一致性
- **Skill**：创建/修改项目 Skill 时走 `skill-creator-plus` 流程（草稿→清单审查），不裸调 `install_skill`。Skill 存 `.agents/skills/`（Agentskills.io 标准），不存 `.reasonix/skills/`
- **仿真可视化**：每组仿真结果至少配时序 / 3D 轨迹 / 分布（箱线或小提琴）/ CDF / 散点或热图；脚本统一 `sim/plot_*.py`，生成后逐张目视审查再写结论（图内文字、坐标尺度、NaN/空图都要查）

## 踩坑记录（完整记录见 docs/registers/踩坑记录-AGENTS.md——改控制/通信/调试代码前先检索）

> 完整条目（130+ 条，按时间累积）已外移到 `docs/registers/踩坑记录-AGENTS.md`
> （触发条件：改控制分配/差速调度/滤波/通信/GNSS/构建/灯效前检索）。这里只留最高频的 5 条。

- **执行器轴映射（悬停构型语义）**：上摆 δf=滚转、下摆 δt=俯仰、差速 Δω=偏航；锚点=`src/flight_control.cpp` mix 层 1202–1252 行，写任何轴/执行器注释前先查实码，勿凭概念名或旧注释（2026-08-07 轴置换）。
- **控制参数唯一事实源**：`include/FlightCtrlParams.h` `kFlightCtrlParams`（static constexpr，固件与宿主测试共用）；速率环输入 deg/s、输出 deg/s²，`kp` 单位 s⁻¹、`ki` s⁻²，旧参数 0.28/0.20/0.10 不得导入。
- **改传感器驱动前先查手册寄存器**：ICM-42688-P 的 LPF 在 GYRO_ACCEL_CONFIG0(0x52)（陀螺 bit3:0、加计 bit7:4），旧实现写进 ODR 字段导致 LPF 从未生效；45686 用 DS-000577。改驱动先 grep 手册寄存器名，不凭经验假设。
- **仿真 ≠ 实机与 trace 副作用**：仿真参数按设计意图（LPF 生效前）与实机不一致；运行期 `printf` 会改变浮点/栈路径（同一 seed 从 0.3° 发散到 170°）——trace 一律内存收集、结束一次性写文件；判"仿真发散"前先排除日志路径。
- **理论/文献任务先读 `docs/08-参考资料/`**（2026-11-01 教训）：`books/REF-120/121` 是秦永元两本教材 MinerU 全文+分章，`notes/00–08` 已有对准可观测性/F 矩阵对照/P0-P2 清单；引用用 `[REF-120 §9.9.2 (9.9.43c) p.334]`，网络检索只补缺。

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
