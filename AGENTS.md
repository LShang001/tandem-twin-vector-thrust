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

# 测试
node --test "simulations/vector-thrust-lab/tests/*.test.mjs"         # 全量
node --test simulations/vector-thrust-lab/tests/propulsion.test.mjs # 单模块

# 上位机 GCS（Python 后端 + Web 前端 + pywebview 桌面窗口，AnoCom 协议）
cd TandemVec_FCS/GCS && py -3.12 app.py               # 桌面窗口模式（后端自动拉起，关窗即停；--browser 回退浏览器）
cd TandemVec_FCS/GCS && py -3.12 -m uvicorn server.main:app --port 8091  # 无窗口纯后端，前端 http://127.0.0.1:8091/
cd TandemVec_FCS/GCS && py -3.12 -m pytest tests/ -q                     # 无硬件可跑

# 串口调试统一入口（所有散脚本已并入，见 TandemVec_FCS/tools/README.md）
cd TandemVec_FCS/GCS && py -3.12 server/cli.py --port COM10 param get att_yaw.kp  # 在线读参数
cd TandemVec_FCS/GCS && py -3.12 server/cli.py --port COM10 param set att_yaw.kp 1.1  # 写参数（写后读回验证，确认帧假阴性兜底）
cd TandemVec_FCS/GCS && py -3.12 server/cli.py --port COM10 link          # 链路健康检查
cd TandemVec_FCS/GCS && py -3.12 server/cli.py --port COM10 sniff euler   # 帧监控主题 rc/att/axes/euler/raw/all
cd TandemVec_FCS/GCS && py -3.12 server/cli.py --port COM10 stick         # 打杆诊断（模式/通道/控制输出）
cd TandemVec_FCS/GCS && py -3.12 server/cli.py baudscan --port COM10      # 波特率扫描（跳过预连接）

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
| `simulations/vector-thrust-lab/src/core/` | 纯计算层，零 Three.js/DOM 依赖，Node 可单独测试；`parameters.mjs` 由 sync-params.py 生成**禁止手改** |
| `docs/04-数学建模/MOD-002-坐标系与符号约定.md` | NED 右手系、theta=-asin(R13)、渲染≠物理力臂 |
| `docs/03-理论推导/THY-004/` | 模块化 LaTeX 工程，编译需要 XeLaTeX ×3 |
| `TandemVec_FCS/GCS/` | 上位机。协议/字段缩放以 `GCS/server/anocom.py` 为 PC 侧唯一实现（与固件逐字节对齐）；**参数链路 = 固件 `src/ano_params.cpp` 注册表（ID 顺序变更须同步 `GCS/server/params.py::expected_names()` 校验）** |
| `TandemVec_FCS/docs/PROTOCOL-001-AnoCom官方通信协议.md` | 官方协议手册（MinerU 转换 26 页）+ **本工程差异对照表**（0x40/0x20/0x0D/0x30 占用约定；0xE0-0xE3 参数链路已验证逐项一致） |
| `TandemVec_FCS/include/FlightCtrlParams.h` | 固件参数唯一事实源：`kFlightCtrlParamsDefaults`（出厂默认，static constexpr）+ 固件分支 `kFlightCtrlParams`（可变实例，0xE1 写入） |
| `TandemVec_FCS/include/AirframeModel.h` | 机型数据驱动执行器模型（通用层）：换机型=填几何数据表；`computeEffectMatrixDataDriven` 数值差分生成 B（ta T11 等价回归锁定） |

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

## 踩坑记录（完整 19 条见 docs/registers/踩坑记录-AGENTS.md——改控制/通信/调试代码前先检索）

- **执行器轴映射（2026-08-07 轴置换）**：固件实际 **上摆=滚转 / 下摆=俯仰 / 差速=偏航**（alpha_roll 体轴x→δf、alpha_pitch 体轴y→δt、alpha_yaw 体轴z→Δω；悬停构型语义，机头朝天时前电机=最高点=上摆）；旧描述"前摆=偏航、差速反扭滚转"为**巡航读法**，2026-08-09 已统一文档。锚点=手动模式摇杆直通实机验证（roll_raw→上摆、pitch_raw→下摆）。**写任何轴/执行器注释前先查 mix 层 1202-1204 行，勿凭概念名或旧注释**（来源：2026-08-09）
- **差速增益调度（flight_control.cpp 层2）**：有效回路增益 = kp·(w0_actual/w0_used)²。三层坑：①Δω 指令 1/w0² 低油门爆炸（2026-08-07 调度 (w0/wh)² 修复）；②物理力矩仍 ∝w0² → 稳态有效增益随油门放大（2026-08-09 封顶 1.0 修复）；③**瞬态失配才是抖油门震荡根因**——分配器用指令油门而力矩用实际转速（τm 滞后），油门释放瞬间 w0_actual/w0_cmd≈1.33 → 有效增益瞬态 0.25→0.44 越震荡点 0.35 约 160ms。**修复：τm 一阶观测器（w_est += (w_cmd−w_est)·dt/τm），B 矩阵工作点/调度/current_state 全部用观测值**。调差速相关增益前先算有效增益随油门/瞬态的变化（来源：2026-08-09 抖油门震荡三层定位）
- **遥测双环结构（2026-08-10 COMM-001，communication.cpp handleAnoCom）**：200Hz tick 分档——**快环 100Hz**（IMU 0x01/欧拉 0x03/目标姿态 0x0A/控制量 0x21/执行器 0xF1，奇数 tick）+ 慢环（位置 50Hz t%4==2 / RC 25Hz t%8==4 / 显示帧 25Hz t%8==6 / GPS·氧压 10Hz t%20==10，偶数 tick 余数错开永不碰撞）。快变字段每 tick 采集（陈旧度 ≤5ms）。**组模式**（`AnoComProtocol::beginGroup/endGroup`）：同 tick 多帧合并单次 write（原 5 帧 5 次 write → 1 次）。**SERIAL6_BAUDRATE=921600**（2M 在 VCP 丢帧 ~15% → 921600 位时间翻倍显著降丢帧，带宽余量仍 87%）。改频率先看帧分布：`cli.py sniff all`（快环 ~100Hz 计数）。**官方上位机兼容**：帧格式/缩放零改动，只变频率（2026-08-10 实机解析 226 帧零坏帧验证）。★ 旧描述"4 组 50Hz 轮发"已废止
- **改 GCS 后端后必须杀掉旧进程再启动**：8091 旧 uvicorn 残留 + 前端 no-store = "界面全新、后端逻辑全旧"，用户看到的就是没数据且所有新修复无效。曾因此误诊半天——固件 2093 帧/3s 全有效、问题 100% 在残留进程。防护：`app.py` 启动时检测端口占用→查 `/api/status` 版本不符自动 taskkill 重启；WS `hello` 握手前端比对版本（`BACKEND_VERSION` 与 `main.py app.version` 必须同步改）（来源：2026-08-08 三轮"没数据"终定位）
- **AnoVars 通用变量上报（2026-08-10，0xF2/0xF3 + MAVLink + DBG vars）**：平台无关注册表（`include/AnoVars.h`）+ 多协议输出（0xF2 值帧/0xF3 清单/MAVLink NAMED_VALUE_FLOAT+DEBUG_VECT/DBG vars）。**加变量 = `src/ano_vars.cpp` kAnoVars 加一行宏**（预注册 46 个：gnc_tel 全字段/w_est 观测器/EKF/辨识）。CLI：`vars list`/`vars watch <name>... -t 5 --json`。**★ 三坑**：①注册表 const 数组定义必须 `extern` 且**不带大小**（否则 sizeof 虚高、遍历空项 segfault）；②volatile 变量（bat_voltage_mv）fptr 需 `const volatile float*`；③0xF3 应答 20B。**MAVLink**：DBG `proto mavlink|anocom` 运行时切换（不重烧），AnoCom 与 MAVLink 互斥由该标志保证（来源：2026-08-10，host segfault 三层定位）
- **上电无信号时 `raw_rc_values` 全 0 → 任何 `raw_rc[x] < 阈值` 的开关判定误触发低档分支**：CH8=0<1200 → 手动 TVC 旁路 → 舵机钳到满偏 ±15°，直到首帧通道到达。修复范式：**开关/模式判定一律以 `isLinkUp` 门控兜底**（`is_manual_tvc = isLinkUp && ...`，来源：2026-08-08 灯效扩展时发现并修复）
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
