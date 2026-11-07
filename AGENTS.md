# AGENTS.md — UniGNC 通用飞控平台

> 项目背景见 `README.md`——不在这里重复。
> 多 Agent 协作问题或新 Agent 入场时，读 `docs/00-项目治理/GOV-003-多Agent协作规范.md`。

---

## 项目

**UniGNC —— 机型无关的通用飞控平台。** 一套 GNC 内核（串级姿态环 + 15 状态组合导航 + 200 Hz 调度）
配套可插拔的机型分配层；控制律只输出广义控制量，由分配层按惯量、控制效率矩阵、舵效与执行器限幅
解算到任意执行机构。**差异止于执行层，其上是同一份代码。**

已完成实飞验证的四类飞行器（用户 2026-11-05 确认）：**纵列双发矢量推力（2026-08，基型）**、
**共轴双桨 VTVL**、**单涵道四燃气舵**（2026-11-05，出厂默认控制参数首飞即稳）、
**VTVL 液体火箭**。其中纵列与单涵道有独立记录，共轴以用户确认为准（固件文档 2026-11-04 的
"尚无共轴实飞数据"是当时时点状态，待补记录）。

同一块板可烧多种机型，**烧录前必须先确认这次飞哪一型**；env 与机型对应见
`TandemVec_FCS/AGENTS.md` §当前实机机型与构建。
仿真参数仍 MODEL-DEFAULT 未标定。

**基型（纵列双发）的执行器语义**（2026-08-07 轴置换后，悬停构型语义）：
★ 上摆=滚转主控 / 下摆=俯仰主控 / 差速 Δω=偏航主控（锚点=手动模式摇杆直通实机验证）。
悬停（机头朝天）时前电机在最高点=上摆、尾电机在最低点=下摆；巡航构型下同两摆即前摆/尾摆。
前电机（拉力 CW）→ 上摆 δf（滚转）；尾电机（推进 CCW）→ 下摆 δt（俯仰）；差速 → 偏航。
NED 右手系（x 前 / y 右 / z 下），四元数RK4积分。**新增机型只补分配层，不改其上控制与导航。**
**实机 GNSS：u-blox NEO-F10N（REF-175；多星座 10 Hz、CEP 1 m；单星座 20 Hz、CEP 1.5 m；速度 0.05 m/s）。仿真/参数默认按 F10N 档建模。**

## 命令

```bash
# ★ 总守卫（改完代码先跑这条，~2 s；7 项全绿再提交）
py -3.12 tools/verify-all.py                 # 参数/推进/分配/坐标系/注释/仿真域/坐标系实现
py -3.12 tools/verify-all.py --include-sil   # 追加三机型 SIL 跑批（慢档 ~10–20 min）

# 参数同步（修改 aircraft-model.json 后必须执行）
py -3.12 tools/sync-params.py          # 生成 parameters.mjs
py -3.12 tools/sync-params.py --check  # 漂移检测
py -3.12 tools/check-param-parity.py   # 仿真 JSON ⇄ 固件 TandemVec_Config.h 机型参数逐项核对（除白名单 dMax 外必须一致）
py -3.12 tools/check-propulsion-parity.py  # 仿真 propulsion.mjs ⇄ 固件 computeWrench 六维对拍（三端同构守卫）
py -3.12 tools/check-allocation-parity.py  # 仿真 computeBTrue ⇄ 固件 computeEffectMatrix 3×3 控制效能矩阵对拍
py -3.12 tools/check-frame-refs.py         # 坐标系规范落地：全仓 @frame 是否都标注了所属坐标系（MOD-002 §0.5 第 1 条）
py -3.12 tools/check-sil-coverage.py       # SIL 覆盖审计：每个 src/*.cpp 要么进 SIL 核心源清单、要么登记豁免原因

# 测试（日常按改动范围只跑相关子测试；全量 run_all.sh 与 SIL 矩阵留到里程碑/跨模块改动/用户要求）
node --test "simulations/vector-thrust-lab/tests/*.test.mjs"         # 全量
node --test simulations/vector-thrust-lab/tests/propulsion.test.mjs # 单模块
bash TandemVec_FCS/test_host/run_all.sh                              # 宿主全量回归（耗时）
bash TandemVec_FCS/test_host/fw_sil/run_all_airframes.sh             # ★ 三机型 SIL 统一跑批（纵列/共轴/单涵道，逐项判据+汇总）

# 固件算法直调（仿真脚本调固件 EKF/控制，不在 sim/ 复刻算法）
# ★ 输出盘符必须是 ASCII：MinGW 的 ld/g++ 都处理不了含中文的路径。
#   用 subst 把项目 tmp 映射成盘符即可（本机已有 T: -> tmp）：
#     subst T: "D:\纵列双发矢量推力飞行器\tmp"
#     BIN_DIR=T:/tv_fw_algo bash TandemVec_FCS/test_host/fw_algo/build_fw_algo.sh
#     FW_ALGO_BIN_DIR=T:/tv_fw_algo py -3.12 TandemVec-EKF-Report/sim/run_fw_gnc_demo.py
bash TandemVec_FCS/test_host/fw_algo/build_fw_algo.sh   # 共享库（4 组 ABI；BIN_DIR 指向 ASCII 盘符）
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

# ★ 实机烧录后自检（只读：不解锁/不给油/不写参数，8 项判据）
py -3.12 tools/verify_board.py COM6 SingleEdf4Vane   # 参数数量/机型/IMU/CPU/ring
#   判据：参数表=169（版本锚点）、机型匹配、ISR 1800–2200 Hz、cal=1、drop=0、
#   ring_drop=0、CPU≤85%、IMU 真实速率走 vars watch（非 DBG 路径）
#   磁力计只报告不判 FAIL（装不装看机型配置）

# 仿真 / LaTeX / 文档 / 注释门禁
cd simulations/vector-thrust-lab && python -m http.server 8080
cd TandemVec-Paper && build.bat
py -3.12 tools/build-docs.py && py -3.12 tools/check-links.py
py -3.12 tools/check-code-comments.py --check     # GOV-004 门禁；--area fw-nav 单区域
py -3.12 -m pytest tools/tests/ -q                # 工具脚本回归（注释识别判据）
```

## 关键路径（AI 无法自行推断的）

| 路径 | 为什么必须知道 |
|------|---------------|
| `models/aircraft-model.json` | ★ 唯一参数源（59 参数），所有参数值以此为准，不可硬编码 |
| `TandemVec_FCS/include/FlightCtrlParams.h` | ★ 固件参数唯一事实源（`kFlightCtrlParams`），固件与宿主测试共用 |
| `TandemVec_FCS/test_host/fw_algo/` | ★ 固件直调接口（C ABI + Python `sim/fwapi`）：`build_fw_algo.sh` 的 `CORE_SOURCES` 是唯一固件源清单；**禁止在 `sim/` 复刻算法** |
| `docs/08-参考资料/` | ★ 理论/器件资料第一入口：`books/REF-120/121`（秦永元教材 MinerU 全文+分章）、`notes/00–08`、`参考资料注册表.md`、**`器件手册与驱动资产索引.md`（ICM42688/45686/DPS310/F10N 手册+驱动+审计）**；引用格式 `[REF-120 §9.9.2 (9.9.43c) p.334]` |
| `docs/04-数学建模/MOD-002-坐标系与符号约定.md` **§0** | ★★ **全仓坐标系唯一正本**：三套系（存档系/模型系/NED）定义、`{M}↔{A}` 转换表、逐量转换、分端使用表、禁止事项。写任何轴名、做任何轴变换、或读到"roll/推力轴"歧义前**必读**；代码正本＝`TandemVec_FCS/include/FrameConventions.h` |
| `TandemVec_FCS/GCS/` + `TandemVec_FCS/docs/PROTOCOL-001-AnoCom官方通信协议.md` | 上位机协议唯一实现（`anocom.py`）+ 官方协议手册与差异对照表 |

其它：`TandemVec-Paper/`（论文工程，XeLaTeX×5 构建）、`simulations/vector-thrust-lab/src/core/`（仿真纯计算层，Node 可单测）、`TandemVec_FCS/include/AirframeModel.h`（机型数据驱动执行器模型）、`docs/00-项目治理/GOV-004-代码注释与理论对照规范.md`（注释五段式 + `@ref` 格式，改公式性代码前读）。

## 原则

1. **先读后改** — 改参数前读 `models/aircraft-model.json`；改推进/动力学/控制逻辑前读 `simulations/vector-thrust-lab/src/core/` 对应模块
2. **复用优于新建** — 检查 core 层是否有现成函数（math/state/control/propulsion/aerodynamics/dynamics），不要重复实现；**仿真侧算法一律经 `fwapi` 直调固件实现，不要在 `sim/` 另写一份**；**仿真积分统一四阶 Runge-Kutta，不用欧拉积分**（宿主模型、SIL `Plant::step`、宿主刚体测试一致；2026-10-31 用户要求）
3. **外科手术式修改** — 只改目标代码，不顺手重构、不扩范围
4. **修改后必须跑测试** — `node --test simulations/vector-thrust-lab/tests/`，全绿再提交
5. **改代码必改注释（GOV-004）** — 公式性实现必须带 `@formula` + `@ref`（章节+式号+页码）；新文件必须有 L0 文件头；提交前跑 `py -3.12 tools/check-code-comments.py --check`
6. **查理论/器件先读本地资料库** — `docs/08-参考资料/`（`notes/` + `books/REF-120/121` + **`器件手册与驱动资产索引.md`**）优先于网络检索，网络只用于补缺（2026-11-01/11-03 教训）
7. **改固件/宏/模块后同步文档** — 新增或删除宏、模块、测试后，同一提交内更新：`docs/04-数学建模/代码理论核查/上板标定与验证清单-2026-11-01.md` §7 宏状态表（**宏清单正本**，含 18 个导航宏的默认值/作用/开启前提）、`卫惯组合导航优化总结` §29 模块表、`代码理论核查/README.md` §0 研究文档索引。**确认错误或过时的结论直接删除或就地改对**，不复述错误内容、不留"曾经错现在改"的对照说明（2026-11-07 用户指令，取代此前"标注已废止"的口径）；文档只保留当前正确的唯一口径，担心误用就改写正确表述本身使其可判定，不靠"某旧说法已废止"来提醒
8. **算法测试两阶段：先高精度理想模型验正确性，再真实传感器模型验工程效果** — 新增/修改估计、导航、控制算法时，先在高精度理想模型（零噪声/零偏/延迟，WGS84 正常重力、标准 ω_ie、解析或高精度数值真值）下断言算法自洽与收敛（例：理想 IMU 1 h 漂移 <2 m、参数辨识误差与理论值一致、TRIAD 全姿态误差 <1e-3°）；通过后再注入真实传感器噪声/零偏/振动/延迟/失锁评估工程效果。**不得跳过理想阶段**——否则无法区分“算法错”“传感器限制”“测试构造错”（2026-11-01 曾把重力口径差伪装的 24 km 漂移误判为机械编排缺陷）。理想阶段失败必须修算法；真实阶段不达标先归因再决定是否改算法。
9. **架构迁移的验收基准是"响应一致"，不是"参数不变"** — 换控制架构（例：速率环输出由舵角/力矩域改为目标角加速度 α-native，交给机型分配层）时，必须用仿真证明新架构在相同惯量假设下的控制响应与**实飞验证过的旧架构**相当，才能上机；旧架构验证通过后删除，不长期并存。姿态模式＝串级（外环姿态→内环角速度），角速率模式跳过外环；输出限幅保留直观的执行机构限幅（2026-10-31 用户口径）。
10. **临时文件与一次性脚本写进项目 `tmp/`** — 不写系统 `%TEMP%`、不新建盘符根目录；长期复用的脚本按域归入 `tools/`；跑完即清（2026-10-31 用户要求，同日因 `T:\` 目录被质问）。

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
- **PDF 必须先经 MinerU 转换**（2026-11-01 用户要求）：所有 PDF（器件手册、论文、标准）入库或阅读前统一 `mineru parse <pdf> --tier standard`，转换产物（paper.md）入 `docs/08-参考资料/papers/REF-xxx/` 并在 manifest 注明 `converter=MinerU`；禁止用 pypdf/pdftotext 直读作为正式入库文本（临时 grep 可，但不得作为交付/引用依据）。注意首次需 `mineru server start` 并等引擎预热（engine_unavailable 时等 15–30 s 重试）；**扫描件/影印书/低质量 PDF 用 `--tier advanced`**（普通电子版 PDF 用 standard），并在 notes/manifest 注明所选 tier。

## 多会话并发与变更联动规范

> revo 与 magicbook 各有一个 agent 会话共用本仓库。完整规范（含 2026-11-06 案例、测试口径、跨机同步细则）见 `docs/00-项目治理/GOV-003-多Agent协作规范.md` §6。

**开工三步**（动仓库前必做）：
```bash
git -c http.proxy=http://127.0.0.1:7897 fetch origin
git status --short                      # 有未提交改动先确认归属
git log --oneline HEAD..origin/main     # 远端领先的提交，逐个 git show 看过再合
```
远端领先时**禁止** `git reset --hard origin/main` 与 `git push --force`。

**收工三步**：
```bash
git status --short        # 只允许预期文件
git diff --check          # 空白/行尾问题
git -c http.proxy=http://127.0.0.1:7897 push origin main
```

**参数与默认值变更的联动清单**（改一处默认值往往牵动六处）：

| 变更类型 | 必须同步的位置 |
|---|---|
| 机型默认值 | `FlightCtrlParams.h` 默认值 + `static_assert` + **宿主测试里的默认值断言** |
| 参数数量 | `ano_params.h` `ANO_PARAMS_COUNT`；`FlightCtrlParams.h` 参数表宏与「N 项」注释；GCS `params.py` 名字表与包线；GCS 测试数量断言；CTL 文档；本文件。**三重守护已就位**：①编译期 `ano_params.cpp` 的 `static_assert(ANO_PARAMS_COUNT == 133 + BFS_COAX_PARAM_COUNT + BFS_SINGLE_EDF_PARAM_COUNT)`；②GCS 测试断言；③`check-doc-stats.py` 比对文档数字。改表时三道都会报 |
| 新增参数 | 注册表插入位置会移动其后所有参数 ID → 必须同步 GCS 测试里的 ID 断言 |
| 协议帧格式 | 固件发送侧 + GCS `anocom.py` 解析 + 匿名上位机实测抓帧（三方一致才算改完） |

**跨机同步走 bundle**（magicbook 访问不了 GitHub）：起点必须用**对端当前 HEAD**（先 `ssh magicbook "git rev-parse --short HEAD"`），`pull` 后必须核对 `git rev-parse --short HEAD` 与预期一致才能继续。**烧录后必须用参数读回验证新行为**——`Verified OK` 只证明写进了 Flash，不证明固件是新的。

## 踩坑记录（完整记录见 docs/registers/踩坑记录-AGENTS.md——改控制/通信/调试代码前先检索）

> 完整条目（130+ 条，按时间累积）已外移到 `docs/registers/踩坑记录-AGENTS.md`
> （触发条件：改控制分配/差速调度/滤波/通信/GNSS/构建/灯效前检索）。这里只留最高频的 5 条。

- **执行器轴映射（悬停构型语义）**：上摆 δf=滚转、下摆 δt=俯仰、差速 Δω=偏航；锚点=`src/flight_control.cpp` mix 层 1202–1252 行，写任何轴/执行器注释前先查实码，勿凭概念名或旧注释（2026-08-07 轴置换）。
- **控制参数唯一事实源**：`include/FlightCtrlParams.h` `kFlightCtrlParams`（static constexpr，固件与宿主测试共用）；旧参数 0.28/0.20/0.10 不得导入。
  **内环契约统一**：三种机型速率环都输出**目标角加速度 α [deg/s²]**（`kp` 单位 s⁻¹、`ki` 单位 s⁻²），
  机型差异由分配层按（惯量 I、舵效 k、满舵角、矩阵、逐舵方向）换算——单涵道也不例外。
  **限幅口径由机型参数换算**：单涵道用户只设机械摆角 `sedf.ctrl_vane_deg`（默认 20°），
  `applyFlightCtrlParams()` 内部换算成 PID 输出限幅 `out_max = ctrl_vane_deg·(4k/I)·180/π`
  （20° → roll/pitch 4583.66、yaw 3055.78 deg/s²）；I 或 k 标定更新后限幅自动跟随。
  改增益前先确认机型：α 域参数不能与舵角域数值互抄（两者相差 (4k/I)·180/π 倍）。
- **改传感器驱动前先查手册寄存器**：ICM-42688-P 的 LPF 在 GYRO_ACCEL_CONFIG0(0x52)（陀螺 bit3:0、加计 bit7:4），旧实现写进 ODR 字段导致 LPF 从未生效；45686 用 DS-000577。改驱动先 grep 手册寄存器名，不凭经验假设。
- **仿真 ≠ 实机与 trace 副作用**：仿真参数按设计意图（LPF 生效前）与实机不一致；运行期 `printf` 会改变浮点/栈路径（同一 seed 从 0.3° 发散到 170°）——trace 一律内存收集、结束一次性写文件；判"仿真发散"前先排除日志路径。
- **理论/文献任务先读 `docs/08-参考资料/`**（2026-11-01 教训）：`books/REF-120/121` 是秦永元两本教材 MinerU 全文+分章，`notes/00–08` 已有对准可观测性/F 矩阵对照/P0-P2 清单；引用用 `[REF-120 §9.9.2 (9.9.43c) p.334]`，网络检索只补缺。

## 攻关沉淀（docs/memory/）

> 按日期的攻坚记录。动手改对应模块前先按关键词扫一遍——比翻代码快，也比重新踩坑快。

| 文件 | 内容 | 何时读 |
|------|------|--------|
| `2026-11-05-单涵道四燃气舵首飞.md` | 单涵道四燃气舵首飞成功（默认参数即稳定）、舵面映射三轮定案、速率环 α 域（源机型增益等价换算）、匿名助手协议对齐、解锁门放宽；4 项待补 | 改单涵道机型 / 调 `sedf.*` 参数 / 匿名助手链路 |
| `2026-11-04-IMU-2kHz攻关.md` | 测量方法论（禁用 DBG）、有效/无效优化、P1 架构、三个隐蔽 bug | 调 IMU 采样率 / 丢帧 / 时序 |
| `2026-11-03-导航调度通信基线.md` | F10N 配置口径、UART4 优先级、调度丢执行机理、假原点判据、「任务频率/芯片率/消费率」三件事 | 调 GNSS / 调度 / 判 EKF 是否发散 |
| `2026-11-01-固件模块与SIL进展.md` | 模块化架构、SIL 真实源码链接、GNSS/INS 延时补偿与振动自适应 | 改 SIL / 导航 / EKF |
| `2026-10-24-PIO20升级-ArduinoCore3迁移.md` | PlatformIO 20 / Arduino Core 3.0 迁移踩坑 | 动工具链 |
| `2026-08-11-D1-级联架构LMI鲁棒界证明.md` | 级联架构严格 Lyapunov + LMI 鲁棒界证明 | 改控制律 / 写论文 ch15 |
| `2026-08-11-相位裕度与量纲分析.md` | 相位裕度推导与量纲一致性检查 | 改控制增益 / 审量纲 |

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
