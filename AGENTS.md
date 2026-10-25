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
cd TandemVec-Paper && build.bat

# 文档构建
py -3.12 tools/build-docs.py && py -3.12 tools/check-links.py

# 代码注释与理论对照门禁（GOV-004）
py -3.12 tools/check-code-comments.py                 # 全量扫描 + 评分
py -3.12 tools/check-code-comments.py --check         # 有 error 即退出码 1
py -3.12 tools/check-code-comments.py --area fw-nav   # 单区域：sim/fw-ekf/fw-nav/fw-control/fw-infra/fw-tests/gcs/analysis
```

## 关键路径（AI 无法自行推断的）

| 路径 | 为什么必须知道 |
|------|---------------|
| `models/aircraft-model.json` | ★ 唯一参数源，59 参数。所有参数值以此为准，不可在代码中硬编码 |
| `simulations/vector-thrust-lab/src/core/` | 纯计算层，零 Three.js/DOM 依赖，Node 可单独测试；`parameters.mjs` 由 sync-params.py 生成**禁止手改** |
| `docs/04-数学建模/MOD-002-坐标系与符号约定.md` | NED 右手系、theta=-asin(R13)、渲染≠物理力臂 |
| `TandemVec-Paper/` | 独立 LaTeX 论文工程（纵列双发矢量推力飞行器：构型原理、动力学建模与控制特性分析），编译需要 XeLaTeX ×5（build.bat） |
| `TandemVec_FCS/GCS/` | 上位机。协议/字段缩放以 `GCS/server/anocom.py` 为 PC 侧唯一实现（与固件逐字节对齐）；**参数链路 = 固件 `src/ano_params.cpp` 注册表（ID 顺序变更须同步 `GCS/server/params.py::expected_names()` 校验）** |
| `TandemVec_FCS/docs/PROTOCOL-001-AnoCom官方通信协议.md` | 官方协议手册（MinerU 转换 26 页）+ **本工程差异对照表**（0x40/0x20/0x0D/0x30 占用约定；0xE0-0xE3 参数链路已验证逐项一致） |
| `TandemVec_FCS/include/FlightCtrlParams.h` | 固件参数唯一事实源：`kFlightCtrlParamsDefaults`（出厂默认，static constexpr）+ 固件分支 `kFlightCtrlParams`（可变实例，0xE1 写入） |
| `TandemVec_FCS/include/AirframeModel.h` | 机型数据驱动执行器模型（通用层）：换机型=填几何数据表；`computeEffectMatrixDataDriven` 数值差分生成 B（ta T11 等价回归锁定） |
| `docs/08-参考资料/` | 文献库：两本秦永元教材（`books/REF-120/121`，MinerU 全文/章节）、论文、主题笔记 `notes/00–05`、索引/校验脚本；导航算法升级评估报告在 `TandemVec-EKF-Report/导航算法升级评估-专业文献调研-2026-10.md` |
| `docs/00-项目治理/GOV-004-代码注释与理论对照规范.md` | ★ 注释唯一标准：L0 文件头 / L1 符号文档块 / 理论对照五段式 / `@ref` 引用格式 / 机检规则 R01–R10；配套 `tools/check-code-comments.py` 与 `docs/04-数学建模/代码理论核查/` |

## 原则

1. **先读后改** — 改参数前读 `models/aircraft-model.json`；改推进/动力学/控制逻辑前读 `simulations/vector-thrust-lab/src/core/` 对应模块
2. **复用优于新建** — 检查 core 层是否有现成函数（math/state/control/propulsion/aerodynamics/dynamics），不要重复实现
3. **外科手术式修改** — 只改目标代码，不顺手重构、不扩范围
4. **修改后必须跑测试** — `node --test simulations/vector-thrust-lab/tests/`，全绿再提交
5. **改代码必改注释（GOV-004）** — 公式性实现必须带 `@formula` + `@ref`（章节+式号+页码）；新文件必须有 L0 文件头；提交前跑 `py -3.12 tools/check-code-comments.py --check`

## 行为红线

修改以下任一项，必须：重采回归基线 `tests/fixtures/regression-baseline.json`、同步更新文档、提交说明原因。完整清单及变更审核流程见 `docs/registers/行为保持红线清单.md`。

核心红线（逐条对照）：
- 帧 delta≤0.05s / 子步≤0.004s / 电机 τm=0.28s
- 差速公式 ωf=ω0√(1+Δω)（保持不变）
- 四元数RK4积分 + 每阶段保范投影
- theta=−asin(R₁₃)
- 力臂分离（渲染±1.78m ≠ 物理0.315m）
- SAS 限幅 ±25°/±0.7（仿真值；固件实机摆角 2026-08-07 已收紧 ±15°，仿真未回写——积分俯仰±0.5rad / 滚转±0.3rad）
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
- **AnoVars 通用变量上报（2026-08-10，0xF2/0xF3 + MAVLink + DBG vars）**：平台无关注册表（`include/AnoVars.h`）+ 多协议输出（0xF2 值帧/0xF3 清单/MAVLink NAMED_VALUE_FLOAT+DEBUG_VECT/DBG vars）。**加变量 = `src/ano_vars.cpp` kAnoVars 加一行宏**（预注册 52 个：gnc_tel 全字段/w_est 观测器/EKF/辨识/执行器故障 afm_*；角度域角加速度名为 `adps2_x/y/z`，保证 MAVLink 9 字符截断后仍唯一）。CLI：`vars list`/`vars watch <name>... -t 5 --json`。**★ 三坑**：①注册表 const 数组定义必须 `extern` 且**不带大小**（否则 sizeof 虚高、遍历空项 segfault）；②volatile 变量（bat_voltage_mv）fptr 需 `const volatile float*`；③0xF3 应答 20B。**MAVLink**：DBG `proto mavlink|anocom` 运行时切换（不重烧），AnoCom 与 MAVLink 互斥由该标志保证（来源：2026-08-10，host segfault 三层定位）
- **FPV 摇杆曲线（2026-08-11，RATE_MODE）**：Betaflight 三参数模型（`include/RcCurve.h` 平台无关）——`rc_rate` 全局灵敏度、`rc_expo[3]` 输入中心曲线、`rc_super[3]` 输出边缘双曲曲线；默认 0.5/0.25/0.7·0.7·0.55（满杆 roll/pitch 333°/s、yaw 222°/s）。**调参走 CLI `param set rc_rate 0.6`**（在线，不重烧）；RATE_MODE 限幅 MAX_MANUAL_*RATE=600（2026-08-11 由 80 解耦——旧 80 复用姿态外环限幅会把曲线在 ~半杆削平）。曲线图 `rc-curve.png`（项目根）。**实机注意：RATE_MODE 满杆从存档 80°/s 变 333°/s，先低杆量试**（来源：2026-08-11）
- **编码统一（2026-08-11）**：批处理（.cmd/.bat）**内容必须 ASCII**——cmd 用进程启动代码页解析文件，运行时 `chcp 65001` 只影响输出编码、不救文件解析（UTF-8 中文/emoji 注释会凑出特殊字节崩行，实测）；首行 `chcp 65001 >nul` 保留（对调用的 Python 输出有益）；**中文输出走 Python + `PYTHONUTF8=1`**（tvc-cli.cmd 已设，console/管道一致）。`启动.bat` 等已 ASCII 化（来源：2026-08-11 编码统一实测）

- **上电无信号时 `raw_rc_values` 全 0 → 任何 `raw_rc[x] < 阈值` 的开关判定误触发低档分支**：CH8=0<1200 → 手动 TVC 旁路 → 舵机钳到满偏 ±15°，直到首帧通道到达。修复范式：**开关/模式判定一律以 `isLinkUp` 门控兜底**（`is_manual_tvc = isLinkUp && ...`，来源：2026-08-08 灯效扩展时发现并修复）
- **单通道 LMI 不得外推为实机增益裕度**：Petersen LMI（cvxpy/CLARABEL）在无积分、无滤波、无执行器动态、无饱和的标量乘性摄动模型中得到 $\bar\gamma=0.968$（仿真比例模型）和 $0.897$（固件比例项等效口径）。该模型不覆盖 $\tau_m$ 瞬态工作点失配、BTRUE 非对角耦合和离散PI闭环，不得据此解释实机调参或给出实机裕度（来源：2026-08-11 论文证据审计）
- **速率环量纲已统一（2026-08-11，旧“rate.kp 是混单位增益”描述已废止）**：控制器输入 `deg/s`、输出 `deg/s²`，仅在写入 `ControlOutputs_t` 的惯量逆解边界乘 `π/180` 转为 `rad/s²`。现值 roll/pitch `kp=16.042818 s⁻¹`、yaw `kp=11.459156 s⁻¹`、`ki=5.729578 s⁻²`（**2026-08-12 偏航 Ki 首轮保守下调 25% → 4.297184 s⁻²，以 `FlightCtrlParams.h` 为准**），分别与迁移前混合域 `0.28/0.20/0.10` 行为严格等价；外部参数文件若继续写旧值会使物理增益降低 57.3 倍。`kd` 当前为 0，PositionPID 仍按每拍差分（未除 dt）实现，非零值迁移也须乘 57.3。0x21/CAN/旧 `alpha_ref_*` 继续保持 `rad/s²` 兼容，新增 `adps2_x/y/z` 提供角度域观测（来源：2026-08-11 用户要求统一单位并完成行为保持迁移；2026-08-12 偏航 Ki 调整；2026-10-23 核查同步）
- **论文修订必须保留完整长稿**：`TandemVec-Paper/` 的17章、原有推导、图表和历史研究过程属于论文资产；优化时以证据分层、勘误和重写为主，不得用压缩稿覆盖或大规模删减。历史结果若与当前固件不一致，应保留并明确标注“历史原型/待复核”，而不是直接删除（来源：2026-08-11 用户纠正）
- **EKF 导航三个机制性风险（2026-08-13 优化研究，已修 2 项）**：①**误静止巡航**——水平恒速飞行与静止在 IMU 上不可区分（|a|≈g、|ω|≈0、Δa≈0），静止检测误确认→GNSS 速度强制 0（navigation_task.cpp:1148 语义）→6 维联合更新 NIS 爆炸→**位置一并被拒**→纯惯导漂移（仿真 120s 漂 330m）。修复=GNSS 速度>0.8 m/s 强制退出静止（仅退出不进入，非对称防 GNSS 噪声漏触发）。②**航向弱可观踢动**——F(v,β)=−C^n_b[f_b]× 使 P(8,v) 按 f_xy·P(8,8) 增长，K(8,v)∝f_x·P(8,8)/Q_a；P0 航向=π 时 VTOL 过渡（f_xy 峰值 7.8 m/s²）/转弯把 GNSS 速度噪声放大为 ±10-25°/epoch 航向踢动→双矢量量测 NIS 超敏全拒→失锁（闭环悬停 5 场景全崩 2.5-6.7km）。修复=P0 航向 π→0.2 rad（伪磁确定性初始化语义，原 LOW-7）。③**转弯段速度失控锁死（航向踢动残余）**——P0 修复后仍偶发（1/8 种子）：航向踢动使速度估计被踢 3-9 m/s→GNSS 6 维联合更新 NIS 爆炸→速度态永久失锁→位置 ~30 m/s 发散。15ms 回放只是改变踢动落点的共因**非独立根因**（无门控时 8 种子无失控）；且固件 `DISABLE_GNSS_DELAY_REPLAY=1` 会经 require_delayed_snapshot=true 路径**拒绝全部 GNSS**（纯惯导测试宏），不可用于"禁用回放"——回放维持现状，根治方向=速度通道失锁重锚/航向可观测性门控（开放项）。**另**：ZUPT NIS 门限 1000 统计失效（χ²(3)≈16.3），已修；GNSS R 地板（2/3/0.15/0.25）**不可收紧**（实测收紧接受率暴跌）；Q 加计 0.25→0.05 有明确增益但无实机震动数据前维持 0.25 不动。报告：TandemVec-EKF-Report/优化研究-算法与参数优化研究报告.md，仿真框架 sim/opt/（来源：2026-08-13）
- **光流速度/测距是两条独立新鲜度通道（2026-08-21，UART7 自动识别配套）**：MSP/MAVLink 的测距帧与光流帧相互独立，测距帧**不得**刷新 `flow_data.last_update_ms`（速度通道专属）——否则光流停流后冻结速度会被 EKF 当新测量反复融合（导航侧按 last_update_ms 判重，时间戳在走就每次都融）；测距通道用 `flow_data.last_range_update_ms`，`handleOpticalFlowSensor` 每 tick 按通道清扫 is_flow_valid/is_range_valid，重识别判据 = 双通道取新。**室内静止出口**：无 GNSS 时静止检测器原有唯一出口（GNSS 速度>0.8）失效，已在检测器更新点加"光流速度>1m/s 持续按住复位"（只出不进）；**光流速度失锁重锚**：EKF 新增 `MeasurementUpdateAidingVelocityDetailed`（豁免 ZUPT 零速守卫，残差门 30m/s，ZUPT 路径行为不变）+ `ResetVelocityToExternal`（速度重置+P 抬升+清回放历史），导航侧连续 50 次拒绝自动重锚（DBG flow 看 g_flow_fusion_reanchors）；宿主回归 test_host/test_ekf_aiding_velocity.cpp 16 断言。★ test/ekf_host_regression.cpp 大回归在 HEAD 基线即挂 3743 断言（project 场景），非本次改动引入，待单独定位（来源：2026-08-21 光流审查修复轮）
- **低速/悬停闭环仿真口径（2026-10-21）**：hifi `run_full_envelope.py` 默认闭环仅姿态/定高、无水平位置环，A2/A3/A5 的 `pos_ne` 不能单独当失败判据。加 `--pos-hold --mag-vector --flow-aiding --a1-tilt-limit 5 --a2-alt-ki 0.5 --nis-mag 10` 后，3 种子、5 种子、10 种子有效场景均 9/9（A8 目标定义不成立，单列）。位置保持必须用体轴映射“北=pitch(y)、东=roll(z)”（旧实现把东向塞进 x=偏航，A9 因此漂）；A2 还受气压更新率/相位耦合影响，仿真用 `--baro-hz 2 --baro-phase 2` 才稳，固件 baro 实际速率不同，不能直接照搬。`alt_vel.ki` 固件仍 1.25，0.5 只是仿真候选。A8 yaw90 目标定义不成立，不计入。F4 单前摆=滚转唯一主控，卡死是硬件单点，只能检测+有界下降（F4b 72 m/102 m），不是可恢复故障；详见 notes/08。
- **EKF P0-0/P0-1/P0-3/P1-5 修复（2026-10-21）**：①速度增量曾同时用 `t_b2ned_mid` 与 `0.5Δθ×Δv`，旋转效应双计；已删两处（REF-120 §9.6.1 (9.6.9)(9.6.17)(9.6.19) p.298–299），host T7 由 0.000250008 修正到 0.000125005（期望 0.000124999），`test_ekf_15state` 36/36。②ZUPT 1 m/s 盲区：GNSS 静止退出门 1.5→0.8 m/s + 外部参照门控（无参照且 `|v|>0.75` 立即拒绝、3 s 封底 0.30 m/s、30 s 停用；REF-121 §8.2.3 p.291 / §7.3.1 p.255）；仿真 SC9b 末端 51.64→11.21 m，**只加 guard 无效**。③仿真双矢量 H 索引 `7:10`→`6:9`。④P1-5 航向 consider：`set_gnss_consider_heading()` 把 GNSS 更新 K 的航向行清零，`navigation_task.cpp` 按 `|pitch|/|roll|>0.5 rad` 且 `|v|>3 m/s` 置位；host T8 无 consider 1.724°→consider 0.000°；5 种子 SC4 过渡 kick 3.0→0.09°、SC3 无收益，**按相位启用，不全局开**。⑤P0-1 子集 FDI：联合 NIS 拒绝后 3D 位置/3D 速度子集 χ²（25.90/16.27），只更新通过子集；速度连续 10 次被拒 → `InflateVelocityCovariance(2.0)` 软重锚；host T9–T11 覆盖速度坏/位置坏/双坏，36/36。`run_all.sh` 全绿、`pio run` SUCCESS；实机/录包回放待验证。报告：`TandemVec-EKF-Report/导航算法升级评估-专业文献调研-2026-10.md` §5.1/§5.10/§9.6。⑥2026-10 追加：static gyro 在 `throttlePercent>=3%` 时跳过融合（`navigation_task.cpp`），防止把电机出力/缓慢漂移当成零角速率估零偏；另发现 hifi 闭环控制路径曾二次减 gyro_bias、真值控制器曾二次减零偏，均已修，但 EKF 进控制闭环仍 10/10 FAIL。

- **EKF 进控制闭环 10/10 FAIL 根因（2026-10-24，hifi 消融）**：不是单一 EKF 数学 bug。悬停时航向/姿态不可观；闭环速率抖动使 `|ω|<0.01 rad/s` 静止门永不满足（A0/A2/A9 静止确认 0%），ZUPT/重力/静止陀螺全部关闭，EKF 姿态误差 1:1 进入控制回路，真值被“估计误差”带着漂。消融：控制用真值姿态时 5 seeds × A0/A2/A9 全部 0.7–2.4°、位置 0.5–1.7 m；用 EKF 姿态则 A0 3.5–38°、A9 可达 68°。修法是绝对姿态源（机体系磁矢量 `H=[m_pred×]`）+ 位置保持/高度门控，不是调速率环增益；hifi 默认无 `--pos-hold/--mag-vector`，A2/A5/A3 的 `pos_ne` 不能单独当控制失败判据。另：`run_scenes_opt.py` 默认 `thr=50%` 与 hifi `49.912%` 悬停配平不一致（60 s 爬 60 m），已修；仿真静止陀螺在 50% 油门仍融合，与固件 `throttlePercent>=3%` 门控不一致，已加门控。**2026-10-24 real-nav SIL 复测**：无磁 `clean/nominal/degraded` 40 s 位置保持约 23 s 后姿态/航向发散；已加 `HeadingAidingGate`（只认磁矢量成功融合，>2 s 失效即降级，恢复需连续 3 s）——`runGNCExecutive` 把 AUTO_POSITION/GUIDED 强制降 AUTO_ALTITUDE，`execute_yaw_controller` 同时关闭 heading hold。无磁 `nomag_degraded` 3 seeds PASS（定高 9.17–9.29 m、max_roll 0、`pos_degraded=1`）；有磁 `mag_clean/mag_nominal` 6/6 PASS，`mag_disturbed`（含 GNSS 失联）只验收门控拒绝/姿态有界。位置保持验收必须用磁 profile；无磁只用于观测性/失效研究；无磁发散在加门控前 HEAD 与新二进制逐行一致，不是垂直推力映射引入（来源：2026-10-24 EKF-in-loop 定位 + 磁失效门控）
- **参数不确定性 MC（2026-10-24，hifi L1）**：`sim/hifi/run_param_mc.py` 17 参数 LHS。正常范围 1024 例 0 fail（pos P95 0.123 m、att P95 3.04°）；压力尾 256 例 22 fail（8.6%），边界集中在低 `att_kp`+低 `rate_kp_rp`+高 `tauM`+低 `kT`+高质量（控制权限不足），以及高 `rate_kp_rp`+低 `att_kp`（振荡）。参数范围/敏感度见 `sim-data/campaign-2026-10-24/param-mc-*`。**这是 L1 模型 + 理想磁/光流；参数为 ESTIMATED/MODEL-DEFAULT，不能外推实机**（来源：2026-10-24 参数 MC）
- **最坏情况 DE 搜索（2026-10-24，hifi L1）**：`sim/hifi/run_worst_case_search.py` 16×9 代 144 次评估找到 pos=2034 m/att=104°/alt=219 m 的最坏组合（m=0.45、kT=1.5e-5、tauM=0.097、att_kp=6.0、att_kp_yaw=1.0、rate_kp=8.2、rate_ki=9.5、alt_vel_ki=0.1），比随机压力尾最坏 pos 204 m 差 10×。说明随机 MC 不能替代主动边界搜索；参数已超出 ESTIMATED 正常范围，L1 模型结论不能外推实机（来源：2026-10-24 DE 搜索）
- **长航时 GNSS 慢漂累积（2026-10-24，hifi L1）**：30 min×2 seeds，600 s 起注入 GNSS 0.02/0.01 m/s、气压 0.005 m/s 慢漂，1200–1260 s 失联。pos_ne 分段 0.1→3.2→9.3→16.0→22.7 m（两 seed 一致），姿态 0.35–0.85° 稳定，高度除失联段 max 4.5 m 外 <0.2 m。0.02 m/s 每帧仅 2 mm，30 m 野值门/ NIS 对慢漂无效；当前无 GNSS 慢漂/跨源一致性检测。**这是已确认失效路径，不是通过项**；L1 无电池/温漂/老化模型（来源：2026-10-24 长航时仿真）
- **GNSS spoof 小于野值门时不可检测（2026-10-24，real-nav SIL）**：`--gnss-spoof` 注入 h_acc 正常的 20 m 位置台阶/0.5 m/s 慢偏；30 m 野值门不触发（outlier_rejects=0），EKF 融合后位置保持被拖走 20–32 m。需要 GNSS 与惯导/气压/光流交叉一致性或外部基准；当前没有。**这是已确认失效路径，不是通过项**（来源：2026-10-24 GNSS spoof 注入）
- **IMU NaN/冻结故障路径（2026-10-24，real-nav SIL）**：SIL 注入 1 s NaN / 2 s 冻结。修复前 NaN 直入控制混控（`servo=nan`、位置/姿态 NaN）；现 `navigation_task` 跳过非有限 IMU 帧、`flight_control` 陀螺保持上一有效值，**不再输出 NaN**，但 1 s NaN/2 s 冻结仍导致失稳——**没有 IMU 故障检测/failsafe**，需后续加方差/一致性监测与可控下降。不要把这轮“无 NaN 输出”写成故障恢复通过（来源：2026-10-24 IMU 故障注入）
- **GNSS 长失联后永久拒绝 → 自动硬重锚（2026-10-24，延时闭环 5k 修复）**：高振动（1.1–1.8 g）+ 5–15 s 失锁时，纯惯导在失锁段可漂 30–200 m；GNSS 恢复后 `IsGnssReacquisitionResidualPlausible()` 的 25 m / 6 m/s 物理残差门持续拒绝，`time_since_gnss_update_s_` 不再清零 → 协方差继续膨胀 → 位置发散到 0.7–4.3 km（5k 中 4 例：1439/2840/3082/3727，fused 仅 54–202/600）。`post_outage_boost`/`bias_reanchor`/`soft_velocity` 均无效；`reset_on_recovery` 依靠仿真真值才有效，不能作为固件方案。修复：`Ekf15State::TryResetAfterPersistentImplausibleGnss()`——失联 ≥2 s 后**连续 3 帧**残差不可信且相邻帧 GNSS 速度差 ≤3 m/s，才把位置/速度重锚到当前 GNSS（按量测年龄用 GNSS 速度前推），位置/速度协方差抬到 8/12 m 与 1.0/1.5 m/s，清空回放历史；姿态/零偏不动。单帧/双帧跳点仍按离群值拒绝，`MeasurementUpdateResult::reacquired` 供上层诊断，`navigation_task.cpp` 记为 `RecordReset`。验证：T13/T13b 新增（一致 3 帧触发、矛盾跳点串不触发），`test_ekf_15state` 51/51；四例 pos3d → 14.35/0.74/1.32/0.44 m；全量 5k 重跑 mean 0.908 m / P95 2.419 m / max 24.078 m，`>50 m`/`>100 m`/`>500 m` 全为 0，37 例触发重锚；加抗混叠模型后 mean 0.903 m / max 4.931 m、`>10 m` 为 0；SIL 3/3、导航验收 4/4；`pio run` 两 env SUCCESS。**边界**：本层只做 GNSS 帧间速度一致性，位置跳点/多径仍依赖上游 fix 质量门控；失联期间仍必须执行飞控失效保护，不能把该恢复当作“失联可正常飞”的许可（来源：2026-10-24 延时闭环 5k）
- **振动 MC 近 Nyquist 混叠（2026-10-24，5k 尾部定位）**：200 Hz 采样下，199.9 Hz 单频振动会混叠成 ~0.1 Hz 准直流（幅值可达整个 `vib_g`），EKF 把它当零偏，5k 中 6 个无失锁 `>10 m` 尾部全部落在 `vib_hz≈199–200 Hz`；**这不是滤波器 bug**。固定 case 3493 扫频：20/40/60/80/100/150 Hz 为 0.22–1.19 m，198/199.9/199.96 Hz 为 8.68/14.96/21.98 m，恰好 200 Hz 为 0.28 m（采样点落在 sin 零点）。修复=仿真台加可选 `anti_alias`（8 阶 Butterworth fc=80 Hz，按连续频率先衰减再采样；数字滤波对已采样的假直流无效），199.96 Hz 时 21.98→0.275 m，20–60 Hz 通带不变，80 Hz 0.329 m。**测振动必须同时记录频率与采样率，不能只报 g 值**（来源：2026-10-24 延时闭环 5k 尾部）
- **PlatformIO 20 / Arduino Core 3.0 升级（2026-10-24）**：① `pio upgrade` 在 Windows 可能因 `pio.exe` 被占用中断（WinError 32），恢复用 `C:\Users\12631\.platformio\penv\Scripts\python.exe -m pip install -U platformio==6.2.0`；② PlatformIO 包下载必须走代理，当前 shell 用 `eval "$(proxy env --shell bash)"` 注入 7897 代理，长命令后台跑；不要并行开多个 `pio pkg install`；③ Core 3.0 具体串口类改名 `Uart`，`HardwareSerial` 成为抽象基类；`availableForWrite` 只在 `Uart`，所以 `opticalFlowSerial`、`debugFlashCommand`、`handleDebugConsole` 等必须用 `Uart&`；④ `Print` 进入 `arduino::` 命名空间，`task_scheduler.h` 的全局 `class Print;` 会产生二义性，改用 `arduino::Print&`；⑤ 宿主 SIL 替身需在 `test_host/fw_sil/stub/Arduino.h` 增加 `using Uart = HardwareSerial;` 与 `namespace arduino { using ::Print; }`；完整清单见 `docs/memory/2026-10-24-PIO20升级-ArduinoCore3迁移.md`（来源：2026-10-24 升级实施）

- **Host 仿真脚本执行（2026-10-24）**：`.sh` 必须 LF；CRLF 会让 Git Bash 把 `set -eu`/`for` 解析成非法命令（症状：`set: -: invalid option`）。`.gitattributes` 已固定 `*.sh text eol=lf`。直接 `bash script.sh` 可能解析到 WSL bash（症状：WSL localhost 警告后停在编译前），SIL/矩阵统一用 `/bin/bash`（Git Bash）执行；修复后 SIL 验收 3/3 PASS（来源：2026-10-24 超大规模仿真）

- **自动延时闭环低延时发散（2026-10-24，已修复）**：`run_delay_closed_loop_mc.py` 5k 原结果 est τ mean 28.3 ms / P95 91.1 ms、pos max 4.3 km；根因是 `Ekf15State::GetDelayedSnapshotIndexByAge()` 对 `delay < 0.5*dt` 时 `best_idx=-1` 返回 false，`require_delayed_snapshot=true` 的 GNSS 更新被整帧拒绝，comp EKF 变纯惯导（`auto_delay=0` 同样复现）。修复：初始化 `best_idx=latest_idx`（最新快照为 sub-frame 最近邻），补 T12 回归；修复后 case54 fused=600/rejected=0、pos3d 0.076 m，500 例低延时定向 499/500 fused≥500、P95 0.594 m。**5k 全量尚未复跑，4 个高延时离群案例另查；复跑前实机不默认依赖自动延时补偿**（来源：2026-10-24 超大规模仿真 + 根因定位）

- **磁标定输出单位球≠µT（2026-10 磁链路前置修复）**：`MagCalCollector::fit()` 原 `scale=0`，`applyCal()` 输出 |c|≈1 的单位球，但 `MagSample` 文档写 µT；直接接 EKF 会被模长门控全拒。已改为质量门通过后 `scale=stats.fieldMagUt`，`paramsSanity()` 要求有效 scale 5–200 µT，旧 Flash scale=0 标定 fail-closed（需重新 `magcal fit/save`）。改磁标定/融合前先确认这个量纲契约（来源：2026-10-24 磁生产链接入）
- **磁参考场必须 NED z-down 且含磁偏角（2026-10）**：IGRF-14 2026.0 @28.2N/112.9E → N/E/D=35.075/-2.523/+34.079 µT，D>0、dec=-4.114°。hifi 旧 `MAG_NED_UT=[22,0,-42]` 的 D 为负，与 `MagMath` 的北半球 z>0 判据矛盾；已统一到 `include/mag_reference.h`。真实磁初始化必须给 EKF 加磁偏角 offset，否则 TiltCompass 得到磁北航向；参考场离开 home >50 km 应失效（来源：2026-10-24 磁链路）
- **40 s 位置保持的横滚/航向摆动是既有控制边界，不是磁更新引入（2026-10-24）**：real-nav SIL clean/nominal 40 s `--pos-hold` 无磁也会 max_roll≈12–13°，磁 clean 9.9°；去掉 `--pos-hold` 后两者 max_roll=0。磁更新 `mag_kick` 仅约 0.6°/`mag_krp` 0.04°，不是根因。磁/故障 profile 的姿态门限只能做“不发散”宽松门（20–25°），不能当飞行品质或磁融合质量指标；根因仍是位置保持-偏航/姿态耦合（来源：2026-10-24 磁 SIL 调试）
- **IMU 健康监测只做检测+隔离，不做安全降落（2026-10）**：`ins_imu_health.h` 检测非有限/无样本/低样本/精确冻结/饱和/高振动/零偏跳变；Failed 时导航停止积分并禁止 ZUPT/Gravity/StaticGyro。单 IMU 无冗余，stuck/nan/drop 故障窗内均能 `imu_detected=1`；1 s 窗口结束后样本恢复、健康回到 Ok，但**持续故障最终会失稳/坠落**，不要写成故障恢复或安全降落。冻结判据只覆盖精确重复样本，不覆盖缓慢变化/带噪声假数据（来源：2026-10-24 IMU 健康 SIL）
- **GNSS 一致性默认 warn-only，0.02 m/s 慢漂不可检测（2026-10）**：`ins_gnss_consistency.h` 用 1 s 窗口 IMU 速度增量/位置自洽/光流/气压交叉；爬升/位置保持动态下 1 s 加速度残差可被姿态误差推到 ~0.9 m/s²，故基线默认只告警（`g_gnss_consistency_reject_enabled=false`），避免误拒 GNSS。`--gnss-consistency-reject` + 0.5 m/s ramp 可触发拒绝，但无外部基准时拒绝后仍漂移；0.02 m/s 慢漂/恒定偏置+一致速度欺骗不承诺检测（来源：2026-10-24 GNSS 一致性接入）

- **多机型通用化分层（2026-10-24）**：现有 `AirframeModel.h` 是“执行器几何+输入映射+数值 B”的 L1 通用层；新增 `VehicleProfile.h`（机型/能力位/输出表）、`ActuatorOutputMap.h`（归一化输出→舵机/电调/阀门/点火/RCS）、`TandemVecAllocator.h`（α_cmd/油门→惯量逆解/BTRUE/差速）、`ThrustAllocation.h`（期望天向比力+cos_tilt+质量/最大推力→总推力/油门）、`HeadingAidingGate.h`（磁航向失效→位置保持/heading hold 降级）、`CoaxialVtvControl.h`（DirectTvcRate：姿态 P→速率 PI→±10° 修正角/PWM 差速）和 `LiquidRocketModel.h`（L1 点火/阀门/比冲变质量/喷管几何）以及 `GenericControlAllocation.h`（3×N 加权最小范数+限幅/秩亏）。`ControlOutputs_t` 现为“期望天向比力 + 三轴角加速度”，自动模式油门由 mix 层机型推进映射回填，控制层不再读 `initial_mass/MAX_THRUST`；`test_vehicle_profile/test_tandemvec_allocator/test_thrust_allocation/test_heading_aiding_gate/test_coaxial_vtv_control/test_liquid_rocket_model/test_generic_allocation` 锁定旧公式等价与新模型边界。**垂直映射仍是 TandemVec 线性油门口径（不是 kT·ω² 物理逆）；共轴/火箭必须替换推力曲线/阀门映射。** 共轴 `kCoaxialVtvProfile`/`mapCoaxialVtvOutputs` 能复现 33.33–66.67% TVC、1000–2000µs 差速与 45° 锁电机，`CoaxialVtvControl` 已按实飞共轴工程的 PID/积分限幅/两级滤波/RATE_MODE 复位结构逐项对齐，并通过 `BFS_COAXIAL_VTVL`/`CoaxialVTVL` env 接入 `runGNCExecutive`；真实固件 + 近似共轴 6DOF plant 的 mag_clean/mag_nominal/nomag_degraded ×5 **15/15 PASS**；**仍无外部实飞日志回放/台架/HIL/实机，plant 惯量/力臂/差速力矩是 L1 假设**。液体火箭只做 L1 模型 + 垂直回收闭环 SIL（`test_liquid_rocket_sil`，20/30/40 m × 2.5/3/4 kg 共 9 组）；外部 `VTVL_LiquidRocket_FC` 的 CCAEGT 库已 vendor 到 `test_host/reference/ccaegt/` 仅作参考测试（30 m 回收接地 1.87 m/s），**未接入固件飞行模式**；无气动/RCS/多级，QuadVTVL/LiquidRocket 仍仅表结构示例。三机型兼容性判据与未证明项见 `docs/07-验证与确认/VER-002-三机型兼容性与虚拟验证.md`。旧 BTRUE、差速开方、τm、轴置换、零油门归中仍是红线。**扩展路线见 `docs/05-控制与分配/CTL-003-多机型与执行器抽象.md`（来源：2026-10-24 架构通用化）
- **三机型反演巨型 MC（2026-10-24）**：`simulations/three_vehicle_mc/` 用可飞行配置反演等效模型：TandemVec 直接回写实飞参数；共轴 m=0.84/Tmax=14/PID 来自实飞配置，惯量 0.05/0.05/0.08、力臂 0.15、差速力矩 0.0005 是低置信反演可行点；液体 M0/Tmax/Tmin/Isp 来自 CCAEGT，惯量/力臂为细长体假设。名义范围 50k/机型 **150k 全部通过**；宽范围 100k/机型（含 2% 推力损失/1% 舵机卡死）TandemVec 78.5%/3.16% 失败、共轴 84.7%/3.91%、液体 60.5%/23.5%，低 T/W 与故障主导。**fast_mc 是降阶 L1 包线工具，不是固件 SIL；反演不唯一，不能写实机概率/实机通过**（VER-003 §3.5/§3.6；完整 110MB CSV 未入库，可由脚本复现）。**同日修正**：thrust_loss 曾乘在油门上被二次曲线平方，TandemVec 故障桶旧数字作废；修正后 fixed/默认 osc/tuned 失败 1.90/1.17/1.02%，tuned 无故障失败 0.17%，但仅高 T/W 机型适用（共轴中性、液体净负），不可无条件启用。ML 层（失败代理/主动学习/失效聚类/Optuna TPE）见 VER-003 §3.8：TPE 1M 失败 0.93%，但更敏感，未改固件默认。**2026-10-24 晚修正**：瞬态基准发现旧“逐拍换向”检测器在悬停噪声下 98% 时间触发，实为常降增益；新选择性检测器（滤波/幅值/驾驶杆门控）悬停触发 <1%，但 100k 消融显示固定降增益 0.85/0.70 失败 1.01/0.83%，优于选择性 1.51%——`AdaptiveGainDamper` 暂不推荐上机，先验证固定增益/延迟裕度。跨机型 detune70：共轴近乎中性、液体略负；最坏风偏/延迟对抗组合仍 170°/−1526 m，固定降增益只是减轻、不是安全保证。1M 同种子 detune70：失败 0.87%、无故障 0.004%，oracle 延迟调度未进一步降低失败率。**SIL 复核（2026-10-24）**：6 例 detune85/detune70/70+ki 均 5/6（mag_clean seed3 hold 3.63/5.57/5.57 m，基线 1.10 m、门限 1.5 m）。扩到 10 seeds × 2 profiles：baseline 16/20、detune70 15/20；detune 修掉 baseline seed6/7 但新增 seed3/5/10（seed5 hold 39 m、seed10 13 m），净少 1 例。**hold_end 裕度很小；seed7 轨迹显示根因是磁扰下 yaw 极限环/航向翻转（yaw +42→+172→−82°，位置跑 132 m，NIS 仍正常），位置环按错误航向投递修正形成正反馈——不是速率环相位或位置 PID 增益。** 因此 AAF/lead/detune 都无效；下一步应做航向振荡/磁新息门控并降级定高，不要继续调速率/位置增益。已做 `YawOscillationDetector.h`（|r|>35°/s 连续 1 s 触发、<10°/s 连续 3 s 恢复）并 opt-in 接入 `HeadingAidingGate`（`BFS_YAW_OSC_GATE`，输入用航向估计跳变速率，不是陀螺 z）。SIL：10 seeds × 2 profiles 16/20（同 baseline），mag_nominal seed7 位置 132→81 m、roll 41→30.8°，`pos_degraded=1` 但仍超 20° 门限；门控 t≈31 s 才触发，此时已有 ~9 m/s 水平速度，切定高只能滑行。**只能减轻、不能修复；必须更早阻断（磁新息一致性/航向估计修复）。** 另试单次磁航向跳变限幅（EKF `mag_yaw_jump_limit_rad`，默认 0）：15° 反而更差（roll 176°）、30/45° 与 baseline 相当，正常 6 例不误伤。再做陀螺约束门控（`BFS_MAG_YAW_GYRO_GATE`，上限 2°+3·|gyro_z|·0.1s）：seed7 hold 104.7→11.9 m、roll 37→15.7° 但仍失败且 yaw 仍翻转；正常 6 例退化到 3/6。**单帧幅值门控无法区分正常收敛与磁扰翻转。** 多帧限幅应用（`BFS_MAG_YAW_MULTIFRAME_GATE`，2° 限幅 + 连续 5 帧超限停磁 3 s）同样失败：seed7 18.2 m、正常 1/6——2° 限幅本身破坏正常磁收敛。限幅/拒绝类不能再以修正幅值为判据。再试航向协方差门控（`BFS_YAW_SIGMA_GATE`，P_yaw 1σ>10° 0.5 s 降级）：seed7 54 m、正常 2/6，仍不行。**单阈值（幅值/速率/协方差）都区分不了正常磁收敛与失效；不要再加门控。** 因子隔离最终定位：seed7 失败的必要条件是 **GNSS 20–25 s 失联**（无失联 hold 5.37 m、无振动 1.51 m、无风 1.27 m，均通过），不是磁标定。修复：导航层 GNSS age>0.5 s 锁存失联、重捕获 age<0.2 s 置一次性 `g_pos_hold_reanchor`，位置保持把当前点设为目标并复位位置/速度环积分；seed7 hold 104.7→**1.28 m PASS**，10 seeds×2 profiles **17/20**（baseline 16/20），无失联行为不变。剩余 mag_clean seed6/8/9 是 3.67 m/pos_rms 1.06/1.03 的边际失败。失联 0.5/1/2/5/10 s × seeds 1/2/3/7：hold_end 最大 0.92/1.12/2.07/4.92/7.33 m、roll ≤18.7°，未发散；10 s seed7 7.33 m 近门限；10 s×10 seeds 有 1/10 边际失败（seed4 9.59 m），20 s×10 seeds 全 pass（max 5.91 m），**重锚只防大姿态、不保证原位保持，更长失联仍需失效保护**。仅缩 yaw kp 0.85：seed6 修好但 mag_clean seed3 新增失败，seed8/9 pos_rms 不变，10 seeds 仍 17/20——不要按种子调 yaw 增益。**SIL `pos_rms` 是 EKF 相对位置−真值（导航误差），不是保持误差；seed8/9 超门限仅 0.03–0.06 m、hold_end 0.60/0.72 m，属 1 m 级导航估计误差，不能用控制增益修。**时序：10–15 s（保持前）seed3 9.42 m 边际失败，25–30/30–35 s（保持中/后段）全 pass（max 4.67/1.67 m）——保持前失联仍会继承较差估计。MATLAB R2025b 频域复核（`matlab/analysis/rate_loop_margins.m`）：fixed roll PM 37.8°/附加延迟 49.4 ms，detune70 47.7°/82.8 ms，lead05 66.5°/72.4 ms，AAF08 162.6°/**13.3 ms**（yaw 甚至 −128°/−13.7 ms）——AAF 高相位裕度是高频穿越的假象，与 SIL 高增益失败一致。MATLAB 结果仍是 L1 线性近似，不含位置环/磁融合/振动。也不能用 6 例 6/6 当稳定证据。现代控制 ADRC/ESO 已做模块+宿主测试+100k：四组带宽失败 2.59–42.77%、delay 相关 0.71–0.80，远不如 PID/detune，不接飞控。速率预测 lead（ω+L·α）L1 最强：1M 失败 0.86%、无故障 0.00%，对抗最坏 11°/5.5 m；共轴/液体中性，TandemVec 特定。**但固件 SIL 否定**：真实固件源码 A/B，基线 6/6 PASS、`-DBFS_RATE_LEAD` 仅 2/6（mag_nominal 全挂，roll 27–76°/hold_end 90–224 m）；根因是 fast MC 用实测 `alpha_act`、固件用上一拍指令 `alpha_ref`，构成近似代数正反馈且 Ki 放大。`BFS_RATE_LEAD` 保持默认关闭；要再试必须改成实测角加速度/实测速率差外推并重跑 SIL。实测角加速度反馈 AAF（`RateAccelFeedback.h`，只反馈陀螺差分、不反馈指令）在 L1 更强：同种子 100k TandemVec 78.63/1.90%→80.22/0.85%，1M 独立种子 78.59/1.96%→80.18/0.86%（与 detune70 80.16/0.87、lead05 80.18/0.86 持平），救回 211/新增失败 0，无故障 206→0，延迟 90–200 ms 失败 80→6/104→3/35→0；60s 79.66/0.88、对抗 TPE 15°/7.9 m（fixed 280°/−2901 m）。**但固件 SIL 同样否定**：K=0.05 6/6（但相对基线无一致改善）、K=0.1 4/6、K=0.2 2/6、K=0.5/0.8 1/6；τ 降到 0.005 s 后 K=0.1–0.3 全 0/6。退化随增益单调、低 τ 更差，指向陀螺差分噪声与位置保持外环耦合。`BFS_RATE_ACCEL_FB` 默认关闭；fast MC 无水平位置环/Ki/真实噪声谱，L1 收益不能外推 SIL。同轮 phase lead（`RatePhaseLead.h`，`ω+K(ω−LPFω)`，不放大差分）L1 K=1.0 达 80.50/0.89，但 SIL K=0.5 3/6、K=1.0 0/6，同样否定；`BFS_RATE_PHASE_LEAD` 默认关闭。结论：现有位置保持外环下，抬升速率测量高频分量的方案都不安全，只有降增益/延迟裕度有效。`ml_optuna_worstcase.py` 旧 `tuned` 变体名未被 `fast_mc` 识别会落成 fixed（已改 label/variant 分离），重跑前不要引用该行 tuned 数据。INDI 增量动态逆 100k：失败 0.97–1.29%、P95 25.6–35.3°，低延迟回归/丢通过严重，不接固件。
- **回中航向锁速率限幅（2026-10-25，20 种子复核后撤回）**：10 seeds 时把回中航向锁速率钳到 14°/s 看似修好 seed6（3.67→1.36 m，17/20→18/20）；20 seeds 否定——基线（`att_yaw.int_limit` 默认 80°/s，不是 24）36/40，14°/s 仍 36/40 但 mag_nominal seed12 1.61→46.3 m、mag_clean seed13 55.3→196.7 m；小误差限幅（<20° 钳 14，大误差 80）20 seeds 仍 36/40，seed6 修好但 seed12 19.3 m 失败。限制航向速率削弱大磁修正后的恢复，属于按种子调参；固件已恢复原行为，20 种子基线 36/40、50 seeds 90/100（mag_clean 9/50、mag_nominal 1/50）；seed6 3.67 m 保留为已知边界（来源：2026-10-25 yaw hold 限幅 20 种子复核）。**光流辅助对照（2026-10-25，随机流未对齐，不作因果）**：`mag_clean` 继承 clean，无 GNSS 失联/风/振动；seed8/9 的 pos_rms 1.06/1.03 是 0.5 m/轴 GNSS 噪声 + EKF 本底，不是失联滑行。加 `--flow` 的 10 seeds 为 pos_rms 0.26–0.83、hold_end 3 例超门限（seed1 7.88/seed7 1.55/seed9 2.18 m），7/10；但 SIL 光流噪声消耗 RNG，启用 `--flow` 会改变整条传感器噪声序列，且 GNSS 有效时光流融合直接返回——不能归因于光流，只能说明配置变化改变了该种子边界。默认 mag_clean 继续无光流；因果 A/B 需固定 RNG 流或独立光流 RNG。新验收 profile 名必须在 `run_real_acceptance.py::BIN_PROFILE_ALIAS` 映射到 SIL 认识的 profile，否则回落 nominal（无磁）——测试台坑（来源：2026-10-25 光流对照）
- **200Hz 控制级追踪与姿态环尾部（2026-10-25）**：SIL 新增 `--trace-csv PATH`（200Hz：真值/EKF 姿态、陀螺、角速率指令、位置/目标、磁 kick/门控、模式/降级、舵机/电机）。seed13 vs 正常 seed14：最大机体 yaw rate 259.1 vs 41.5°/s、EKF pitch 误差峰值 13.9° vs 1.6°、最大磁 kick 21.6° vs 31.3°——**磁 kick 幅值不是判别量**；seed13 先出现 roll/pitch 高频振荡/EKF 倾角误差，随后磁修正与航向环互相追错，t=26.4 yaw>60°/s，现有航向门控 t=27.8 才降级。更快降级路线均否定：yaw rate 安全门使 seed13 55.3→75.7m、seed6 3.67→43.9m；yaw 振荡门控 seed13→76.4m；磁 yaw 限幅 5/10/15/20° 为 26.3/140.9/88.9/59.1m；detune85 50 seeds 93/100 vs 90/100，但 **100 seeds×2 = 181/200 vs 基线 182/200**，新增 seed79 499m 尾部；分离扫描 rate-only 无效果、att-only 复现 detune——敏感性在姿态外环。**50 种子 +3 是噪声，必须扩到 100。** SIL anti-alias 只作用于注入振动，未滤 plant 总角速率，200Hz CSV 又会混叠 >100Hz；下一步加 2kHz 短窗追踪并对总 IMU 信号 anti-alias，确认高频成分是 plant/舵机数值伪影还是真实模态，再决定姿态环滤波，不能据此改增益（来源：2026-10-25 200Hz 追踪）

- **无磁/磁仅初始对准（2026-10-25）**：新增 `g_heading_aid_mode`（0=磁连续默认/1=磁仅初始/2=无磁手动）与 `g_heading_align_deg`（编译期 `BFS_HEADING_AID_MODE`/`BFS_HEADING_ALIGN_DEG`，SIL 可 CLI 覆盖）。物理边界：纯悬停和纯平移（风推、无倾斜）航向不可观——SIL seed1 在 4 m/s² 侧风定高下真实 yaw=0、EKF yaw 漂到 −147°；不能把 GNSS 速度方向当航向。`HeadingAidingGate` 在模式 1/2 下允许 **20 s 内**的位置保持与 heading hold（yaw 1σ≤20°），窗口结束/σ超限立即降级定高；这是短时经验边界，不是长时定点能力。依据：SIL 20 s 窗口内 15 m 前飞 18 s 到达 ~16.5 m 并保持，21 s 后漂移；侧风窗口后漂 225 m。长时需转手动/定高或重新对准；磁连续模式不受窗口限制。10 seeds clean/no-wind t=19 s：10/10 窗口内，最大位置误差 1.52 m、max_roll 7.65°、|yaw 误差|≤13.9°；种子 1–6≈0 m、7–10 为 0.35–1.52 m。窗口后 4/10 种子 t=25 s 漂 1.3–8 m、t=40 s 29–127 m（3/10 >50 m），其余 6 个保持 0 m——20 s 到点必须立即降落/切手动。（来源：2026-10-25 无磁航向）
- **无磁悬停 GNSS 会把真实偏航率吸收成陀螺零偏 z（2026-10-25，SIL 消融定位）**：航向不可观时，GNSS 位置/速度更新经 `P(φ,v)` 耦合把机体真实 yaw rate 估成 `gyro_bias.z`——SIL 风扰 2 m/s² 下 `bias_z` 40 s 从 0.008 涨到 0.406 rad/s，陀螺积分 468° 而 EKF 航向只变 −20°（航向“冻结”、真值在转）；9–21 s 断 GNSS 时 `bias_z` 保持 0.008、EKF 航向误差 <2°，证明来源是 GNSS 更新而非陀螺。修复：无磁离地 `ClampGyroBiasZ(0.01 rad/s)`（`Ekf15State::ClampGyroBiasZ`，宿主 T17），默认磁连续模式 0 不变。**Schmidt `consider_heading` 全局禁 GNSS 修航向已试并撤回**：1 m/s² 风下窗口内位置 3.82→8.14 m、yaw 误差 43.8→61.6°，2 m/s² 下 yaw 30.1→57.4° 且 16.8–18.3 s 提前降级；剩余侧风误差是机体偏航/位置耦合与偏航权限，不是 GNSS 航向耦合，不要再加此类全局门控。20 s 无磁保持窗口同时改为**离地起算**（解锁且油门≥3%），地面等待/落地不消耗窗口（门控 T9b/T9c）。★ SIL 的 `--takeoff-at` 长地面等待场景无效：地面只约束 z、无摩擦，位置环会让机体积滑（30 s 等待实测滑 80 m、起飞前偏航），不能用它判窗口时序。试过“估计高度>1 m”判离地：SIL 中窗口仅 20.5→21.1 s 且地面高度信号会误判，已回退油门判据；窗口语义以单元测试为准。风包线（10 seeds/档，0.5 m/s² 另做 50 seeds）：无风 50 seeds 窗口内最大位置 1.54 m/roll 8.50°/yaw 14.97°、窗口末 2.18 m、AUTO_POSITION 占比 100%；0.5 m/s² 50 seeds 2.34 m/11.4°/21.6°；1.0 m/s² 3.82 m/13.2°/43.8°；1.5 m/s² 18.1 m/13.4°/42.5° 且部分种子 18.5–19.6 s 提前降级；2.0 m/s² 32.7 m/14.3°/30.1°（4/10 种子 20–41 m）。**无磁 20 s 定点只在 ≤0.5 m/s²（约 3° 配平倾角）可用；≤1 m/s² 只保证位置有界；≥1.5 m/s² 不要进入定点。** 无磁位置保持的**地面静止校准**门控/遥测已实现（`static_cal`/`static_cal_s`），但默认阈值 **0=关闭**：SIL 静置扫描 2/10/30/60 s×10 seeds（控制冻结）显示静置越久起飞后越差（yaw 中位 8.6/17.4/28.6/41.0°、60 s 位置最大 58 m），而零偏 z 约 1 s 已收敛；机制指向长静止压小姿态协方差、GNSS 新息经 P(φ,v) 把航向拉回 0。起飞协方差膨胀 A/B 已否定（10 seeds：T=10 s yaw 中位 17→47°、位置 17→28 m；60 s 位置最大 68 m），代码撤回；下一步转导航-only 台架做 NEES/NIS 一致性与可观测性分析，不再拍起飞瞬间参数。SIL `--calib-hold T` 做静置研究、`--static-calib-s` 调阈值。报告 `TandemVec-EKF-Report/无磁EKF收敛仿真-2026-10-25.md`（含图）。**导航-only 一致性（5 seeds，60 s）**：标称 pos/vel RMSE 0.50 m/0.13 m/s，NEES_pos 中位 1.27、NEES_vel 0.40、NIS 4.77（目标 1/6）；高噪声 1.28 m/0.26 m/s、NEES_vel 0.48、NIS 5.16；15 ms 延迟与 5 s 失联无持续发散。**速度协方差偏保守约 2–2.5×，位置尾部偏乐观；只调 Q_vel/P0_vel（R 地板不可收紧），以 NEES→1、NIS→6 为目标。** 导航-only Q 加速谱密度扫描（60 s/5 seeds）：q_accel_scale 1.0→0.05 时 NEES_vel 0.40→0.81、vel RMSE 0.130→0.079 m/s、NIS 4.77→5.30，位置 NEES 1.23 不变；0.1/0.05 在 15 ms 延迟、5 s 失联、高噪声下无发散，但高噪声位置 1.48 m、NEES_pos≈1.55 偏乐观。**10 seeds 复核 Q=0.05**：标称 pos 0.489 m、vel 0.079 m/s、NEES_pos 1.00、NEES_vel 0.79、NIS 5.32；高噪声 pos 1.177 m、vel 0.200、NEES_pos 0.96、NEES_vel 0.74、NIS 5.35（基线 1.28 m/0.26/NIS 5.16）——标称与高噪声同时改善，无鲁棒性退化。**0.05–0.1 仍仅为研究候选，未下固件。闭环 SIL 复核已否决**：10 seeds/工况，1 m/s² 风下 q=0.05/0.1 最大位置 43.75/43.18 m（基线 3.82 m，4/10、2/10 种子 37–54 m 双峰），0.5 m/s² 风 7.57/8.47 m（基线 2.30），无风无差异。导航-only 的低 Q 收益不能迁移到闭环 SIL；固件默认 Q=1.0，`--ekf-q-scale` 仅研究设施。高 Q 反向扫描：无风/0.5 风 q=2/4 略好，但 1 m/s² 风 q=2/4 最大位置 7.47/8.95 m（q=1 为 3.82）——**Q=1.0 是当前闭环最坏情况最优**，默认保留。 慢漂 10 seeds（z 线性 1e-4 rad/s²，T=2/30/60）：末端零偏误差中位 2.07e-4/2.83e-4/3.51e-4 rad/s、最大 9.37e-4，**随时长增大**（仍 <0.06°/s）；yaw 1σ 11.39/11.16/10.97° 随时长下降。慢漂模型下长静置不改善零偏跟踪、反而收紧协方差，方向与 SIL 长静置后更差一致；实机温漂谱未知，不能直接否定实机需求，但“几十秒必然更好”不成立。慢漂下 Q=0.05 与 1.0 末端零偏误差相同（3.51e-4/9.37e-4），yaw 1σ 10.72 vs 10.97°——低 Q 不损害静置零偏跟踪。位置 NEES 尾部（单步峰值 3–6）在 Q=1.0 与 0.05 几乎相同（seed0–2: 5.74/4.72/3.26 vs 5.70/4.73/3.26），是位置模型/GNSS 噪声本底，不是低 Q 代价；巡航段 3σ 略乐观，需单独查 P_pos/Q_pos。 完整 3×3 P 的 full NEES 与对角线近似一致（中位 Q=1.0 pos 0.64/vel 0.30；Q=0.05 pos 0.66/vel 0.64），说明 NEES 不是对角线伪影；Q=0.05 速度 NEES 0.30→0.64，位置中位不变。AnoVars 已加 `hdg_hold`/`hdg_win`（剩余秒数，-1=非无磁）/`hdg_mode`；`hdg_win≤3 s` 或 `pos_deg=1` 应触发降落/切手动。20 seeds 运动收敛矩阵：悬停 yaw 中位 1.9°/最大 10.8°、位置 1.54 m；前飞 15 m 中位 13.7°/最大 40.8°、位置 4.48 m；侧飞中位 6.8°/最大 22.6°、位置 1.9 m——**运动激励不改善航向，前飞反而最差**，不能当航向校准手段。手动对准端到端（SIL 真值 45°）：align45 初始 EKF 44.3°、15 m 段目标误差 ≤6.61 m；align0（45° 误差）初始 EKF 0.3°、位置 58–88 m、yaw 误差 170°——**对准角必须等于机体实际朝向**，45° 误差即可让 20 s 窗口失效。注册表已达 98 项，`ANO_VARS_MAX_COUNT` 96→112（0xF3 count 为 u16；watch 仍 16）。 强风剩余误差是机体偏航-位置耦合/偏航权限，不是估计器；不要再加全局航向门控（来源：2026-10-25 GNSS-bias 消融 + 风窗 SIL，L2）

- **MinGW O0 构建 Eigen SIL 失败（2026-10-25）**：`OPT=-O0` 编译 fw_real 时汇编器报 `too many sections (76084)` / `file too big`，构建无二进制；`build_and_run_real.sh` 默认 `-O2` 可正常构建。调试该 SIL 不要用 O0，需保留 O2 或用其他编译器；排查“构建卡住 3 分钟”时先看输出尾部是否有该汇编错误（来源：2026-10-25 前飞段测试构建）。

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
