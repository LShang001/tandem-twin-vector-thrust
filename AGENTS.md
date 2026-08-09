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
- **黑匣子帧格式**：**P 帧只含 21 个差分增量**（第 22 槽被帧尾 CRC 占用）——解析按 21 增量，末通道（p2）保持最近 I 帧参考值（曾按 22 增量致 p2 被 CRC 污染）；**S 帧 236B 定长**（9B 头 + 通道名表 ≤224B pad + 1B 预留 + CRC 2B）——导出切帧按定长 236B，勿按变长 `\0` 搜索（来源：2026-08-08）
- **AnoCom 0x0D 帧占用约定（2026-08-10 数据归位修订）**：`fc_voltage/fc_current` 承载氧压 P1/P2（非电压电流，官方上位机忽略扩展字节）；**前 2 字段 = 真实 ADC 采样电压/电流**（`bat_voltage_mv`/`bat_current_ca`，原占位常量 12.6/16.8 已废；电流无采样恒 0）；0x20 帧 = 手册语义 PWM 输出（ch3/ch4 电机 %×100 → 0-10000，官方上位机油门条）；**0x40 恢复手册遥控帧**（10×int16 us，RC 显示数据源）；**执行器帧迁 0xF1**（手册灵活格式帧，字段/解码名不变，前端零改动）（来源：2026-08-08 字段对齐；2026-08-10 手册审计数据归位）
- **AnoCom 0xF1 执行器帧（本工程自定义，12B，2026-08-10 起）**：前/尾摆角 int16 ×100→deg、前/尾电机 u16 ×10→%、差速 Δω int16 ×1000（归一化）、饱和标记 u8（bit0 δf / bit1 δt / bit2 Δω）+ 预留 u8；**mix 输出级统一捕获（`g_tvc_front_deg`/`g_tvc_rear_deg`/`ch3_output`/`ch4_output`/`gnc_tel.dw`/`alloc_sat`），锁定/手动/自动全模式有效**；摆角限幅 ±15°（MAX_CORRECTION/GYRO_K），上位机仪表量程即 ±15°。无硬件渲染验证：写 `GCS/output/*.csv`（含 0xF1 字段）→ WS 发 `replay_start` → IAB 浏览器视口加高截图 + PIL 像素采样（来源：2026-08-09 TVC 面板；2026-08-10 由 0x40 迁入）
- **执行器轴映射（2026-08-07 轴置换）**：固件实际 **上摆=滚转 / 下摆=俯仰 / 差速=偏航**（alpha_roll 体轴x→δf、alpha_pitch 体轴y→δt、alpha_yaw 体轴z→Δω；悬停构型语义，机头朝天时前电机=最高点=上摆）；旧描述"前摆=偏航、差速反扭滚转"为**巡航读法**（物理公式 Mz=a·Tf·sδf 在巡航系即偏航力矩，悬停系即侧倾力矩），2026-08-09 已统一文档。锚点=手动模式摇杆直通实机验证（roll_raw→上摆、pitch_raw→下摆）。**写任何轴/执行器注释前先查 mix 层 1202-1204 行，勿凭概念名或旧注释**（来源：2026-08-09 文档统一）
- **差速增益调度（flight_control.cpp 层2）**：有效回路增益 = kp·(w0_actual/w0_used)²。三层坑：①Δω 指令 1/w0² 低油门爆炸（2026-08-07 调度 (w0/wh)² 修复）；②物理力矩仍 ∝w0² → 稳态有效增益随油门放大（2026-08-09 封顶 1.0 修复）；③**瞬态失配才是抖油门震荡根因**——分配器用指令油门而力矩用实际转速（τm 滞后），油门释放瞬间 w0_actual/w0_cmd≈1.33 → 有效增益瞬态 0.25→0.44 越震荡点 0.35 约 160ms。**修复：τm 一阶观测器（w_est += (w_cmd−w_est)·dt/τm），B 矩阵工作点/调度/current_state 全部用观测值**。调差速相关增益前先算有效增益随油门/瞬态的变化（来源：2026-08-09 抖油门震荡三层定位）
- **黑匣子读取工具 `GCS/server/bb_tool.py`**：`--list`/`--latest`/`--seg N`/`--analyze`/`--pages N`（默认 512 页≈4 分钟飞行，全量 2048 页要 8-15 分钟）。驱动后端 DBG 导出链路。**2026-08-09 修复**：① `_on_findseg_text` 定义了但从未被接线（黑匣子页「列出飞行段」静默失效）→ `_on_dbg_rx` 补调用；② DbgSession 导出中断后卡 collecting → enter/exit 复位；③ **S 帧落点缺陷**：S 帧随解锁游标写入（远离段头），导出从段头页读不到通道名——修复=段头写入时把 S 帧拼进段起始页（`setSegmentNameProvider` 回调，header 16B+S 帧 236B 一次编程）；④ 解析器动态化：默认 13 通道（当前固件通道表）+ S 帧预扫描取权威通道数 + 零帧回退 22。**通道表 22→13**（去 accel/vel/位置/氧压/阀门，摆角改录指令值，新增 Δω/目标姿态）——改通道数时解析器无需再动（S 帧自描述）。**导出完整性影响差分参考链**（缺页 → 通道值漂移，曾把上摆 ±15° 读出 25°）——分析前用完整导出（来源：2026-08-09）
- **数字滤波相位滞后边界（2026-08-09 实测）**：本机飞控下有**物理减震底座** + 执行机构（伺服/电机 τm）天然低通——数字滤波的噪声收益接近于零，只加滞后。实测滞后 4.2°@1Hz（单级 α=0.3）稳、6.9°（级联 0.3+0.4）触发内环振铃。**最终配置：滤波近全关**——gyro 一级 α=0.4、二级 α=0.99（直通）、输出滤波 0.9；yaw 输出 0.12 为旧震荡时代产物（增益调度+观测器修复后过时）。教训：**动滤波前先确认振动是否已被物理隔离；滞后是实打实的裕度杀手，噪声收益要量化后才有资格换滞后**（来源：2026-08-09 滤波大审计）
- **GCS 串口链路两坑（2026-08-08）**：① **DBG 与遥测互斥**——发 `DBG\n` 后固件停遥测轮发（handleAnoCom 短路），`exit` 恢复；黑匣子流程必须先 DBG 再 `flash export`，export 输出含 `[DBG] export start=..` 文本行需剥离后按 2048B/页收集。② **遥测缓冲 O(n²) 卡死**——`find_frames` 全缓冲扫描+整体 clear 在垃圾流下无限膨胀；修复范式：`anocom.extract_frames` 返回已消费偏移只删前缀、半帧留尾、超 `TELE_BUF_CAP=8192` 按垃圾流裁尾；连接后自动发 `exit\n` 恢复卡死 DBG 态（遥测态下固件丢弃该行，无副作用）。
- **GCS 3D 视图 NED→Three 必须是相似变换 P·R·v**（groupBasis 固定 P：Three_x=NED_y / Three_y=−NED_z / Three_z=−NED_x，det=1）+ groupBody（q_ned 欧拉 'ZYX'）；旧实现 `M=P·R` 缺 Pᵀ 致轴向错乱。改姿态显示后必须 node + vendored three 数值验证（三轴 × 5 姿态，8/8）（来源：2026-08-08 view3d 重写）
- **改 GCS 后端后必须杀掉旧进程再启动**：8091 旧 uvicorn 残留 + 前端 no-store = "界面全新、后端逻辑全旧"，用户看到的就是没数据且所有新修复无效。曾因此误诊半天——固件 2093 帧/3s 全有效、问题 100% 在残留进程。防护：`app.py` 启动时检测端口占用→查 `/api/status` 版本不符自动 taskkill 重启；WS `hello` 握手前端比对版本（`BACKEND_VERSION` 与 `main.py app.version` 必须同步改）（来源：2026-08-08 三轮"没数据"终定位）
- **GCS 前端事件总线 = CustomEvent，载荷在 `e.detail`**：`bus.addEventListener('telemetry', onTelemetry)` 直接收事件对象，`s.roll_deg` 恒 undefined → 页面全 "--"。三个页面曾全中。同时两个启动坑：`'page'` 监听器必须先于初始 `switchPage` 注册（否则首屏 activate 永不执行）；es-module-shims 初始化前动态 `import()` 静默失败需重试+告警。验证前端修复必须过真实 WebView2 窗口（pywebview evaluate_js 采样 DOM），Python WS 客户端验证不到渲染层（来源：2026-08-08 页面不刷新终定位）
- **PLATFORMIO_BUILD_DIR 隔离目录必须用纯 ASCII 路径**：`D:/纵列双发矢量推力飞行器/.pio-build-nmea` 中中文路径让 ld.exe `cannot open output file firmware.elf`（对象编译正常、仅链接失败），换 `D:/pio-build-nmea` 即成功；默认 `.pio/build` 在中文路径下工作正常，勿据此误判（来源：2026-08-09 NMEA 集成构建）
- **GNSS 双协议已内聚进库（`lib/ublox-main`，2026-08-09 方案演进）**：`GpsProtocol`（kUbx/kNmea/kAuto）+ `SetProtocol/SwitchProtocol(协议,波特率)` 独立切换；UBX 泵按协议分派字节（kNmea 跳过 UBX 状态机、kUbx 跳过 NMEA、kAuto 双流并行），NMEA 快照合成 `UbxEpoch` 入同一队列，导航层零改动。早期"固件层 SetByteTap 镜像"方案已删除（字节镜像 tap 保留为通用扩展，仅 Pump 路径触发）。解析器用 **minmea（MIT）**——MicroNMEA 是 LGPL 2.1（静态链接商业分发有传染约束），故弃用。合成语义：秒键去重 + 800ms 节流（GGA/RMC 双句 1Hz 流 → 1Hz epoch，位置最新/速度滞后 ≤1s）；GGA+RMC+GSA 三句合成（**GSA→PDOP/VDOP**，GN 组合解优先、单星座兜底，激活动态权重 DOP 缩放）；kAuto 下 UBX 300ms 失效才兜底（防混合输出双通道重复融合）。合成 epoch `pvt_tow_ms==eoe_tow_ms`（UTC 时间-of-day 伪 tow）；下游 tow 去重/时间映射对常量 tow 偏移不敏感（±10s 内用参考映射否则退回本帧接收时刻）。回归测试：`TandemVec_FCS/tools/nmea-host-test`（g++ 66 项断言：三模式/分片/串扰重同步/混合流/跨日 tow 回绕/缺省字段/坏校验/超长句溢出恢复/非法波特率防护；非 Arduino 路径走 `core/core.h` shim）（来源：2026-08-09 库级双协议集成+打磨）
- **接收机侧 GNSS 配置（`ubxcfg` 扩展，2026-08-09）**：DBG `ubxcfg msg <句> <rate|off>`（CFG-MSG，NMEA 句 0xF0 + UBX 诊断消息 pvt/eoe/dop/sat/status/svin）、`ubxcfg core <on|off>`（GGA+RMC+GSA 一键）、`ubxcfg nav5 <dyn 2|4|fix 0|1|2|elev 0-90|pdop 0-100>`（CFG-NAV5 读改写）、`ubxcfg itfm <on|off>`（CW 干扰检测 0x06 0x39）、`ubxcfg ant`（天线短路/开路查询 0x06 0x13）、`ubxcfg nmver/nmtalker/nmfilter`（CFG-NMEA 0x06 0x17）、`ubxcfg proto <ubx|nmea|both>`（CFG-PRT outProtoMask）。**★ 2026-08-09 修复 `_cfg_nav5` 错位 bug：dynModel 在 payload[2]、mask bit0 在 payload[0]，旧实现把枚举值 6/7 写进 mask 字节致 Airborne 配置从未生效**（对照 SparkFun setDynamicModel 确认）。NMEA 消息 ID 表在 `ubx_config.h`；只改 RAM，固化 `ubxcfg save`。与 `gpsproto`（解析侧）配合 = 全链路双协议双保险（来源：2026-08-09）
- **minmea 的两个真实坑**：① u-blox 等真实接收机 RMC 尾部为 `,,A`（variation 空 + FAA mode 'A'），官方 `minmea_parse_rmc` 格式 `"tTcfdfdffDfd"` 会把 'A' 当 variation 方向字符而 parse_error（'A' 不在 N/E/S/W）→ 整句丢弃。修复范式：库内自实现，用 `minmea_scan(sentence, "tTcfdfdffD", ...)` 截断到 date（NMEA 2.3 后字段皆可选）；② 状态机内部 `micros()==0` 时 `!=0U` 判断失效（首帧/上电瞬间跳过节流/backoff）——用独立布尔标志（`nmea_epoch_seen_`/`nmea_ubx_seen_`）而非时刻值判空（来源：2026-08-09 库级双协议调试，测试先跑出 13 个失败定位）
- **AirframeModel 机型通用层（2026-08-10）**：`include/AirframeModel.h` 数据驱动执行器模型——执行器=物理电机（kRotor/kGimbal）+ 控制输入映射（u_spd 转速源/u_angle 摆角源/spd_sign 差速联动符号）；**差速 Δω 是两电机的联动转速输入（w=w0√(1±Δω)）不是独立执行器**——初版把它建成独立执行器双重计数，被 T11 等价测试抓出修正。`computeJacobianNumeric` 数值中心差分生成 B（机型无关），**相对步长 h·max(1,|u|) 对 ω² 大量级自适应**（固定小步长差值会被 float 精度吞没→全零，曾致四旋翼 demo 全零 B）。纵列双发 `computeEffectMatrixDataDriven` 与解析式逐元素等价（ta T11，绝对 <5e-4）。**换机型=填几何数据表**，四旋翼 demo `test_host/test_airframe_generic.cpp` 验证。解析 `computeEffectMatrix` 保留为运行时实现（实机已验证），切换时机=下次几何变更（来源：2026-08-10 机型通用化）
- **上电无信号时 `raw_rc_values` 全 0 → 任何 `raw_rc[x] < 阈值` 的开关判定误触发低档分支**：CH8=0<1200 → 手动 TVC 旁路 → 舵机钳到满偏 ±15°（PWM 72%/28%），直到首帧通道到达。修复范式：**开关/模式判定一律以 `isLinkUp` 门控兜底**（`is_manual_tvc = isLinkUp && ...`，来源：2026-08-08 灯效扩展时发现并修复）
- **ZCode agent shell PATH 被精简（2026-08-09 定位）**：Bash 工具默认 PATH 只有 `ZCode tools + %APPDATA%\npm` 三目录（连 System32 都没有），git/node/py 全不可用——**系统注册表 PATH（HKLM+HKCU）本身完整，勿去改系统环境变量**。修复：命令前缀 `call tools\shell-env.cmd && ...` 恢复完整 PATH（脚本在 `tools/shell-env.cmd`，ASCII 副本 `C:\Users\12631\.zcode\bin\`）。**★.cmd 批处理必须纯 ASCII**：cmd 按 GBK 代码页解析批处理，UTF-8 中文注释乱码使整个批处理解析失败（报错特征 `'的' is not recognized` + `& was unexpected`）——.cmd 注释一律英文（来源：2026-08-09 shell 环境诊断）

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
