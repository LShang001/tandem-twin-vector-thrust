# AGENTS.md — UniGNC 通用飞控平台

## 入口与范围

- 先读 [项目 README](README.md) 了解定位，再从 [文档总索引](docs/README.md) 按任务查正本。交接或判断结果是否仍有效时读 [GOV-005](docs/00-项目治理/GOV-005-项目全景与历史.md)；找文件、辨认生成物时读 [项目地图](docs/PROJECT-MAP.md)。本规则全仓适用；涉及固件时再读 [固件规则](TandemVec_FCS/AGENTS.md)。
- 编辑任何模块前，按关键词查本页沉淀索引和 [踩坑记录](docs/registers/踩坑记录-AGENTS.md)，命中触发条件就先读。查理论或器件先查 [本地参考资料](docs/08-参考资料/) 与 [器件资产索引](docs/08-参考资料/器件手册与驱动资产索引.md)，网络只补缺。
- 任何轴名、轴变换或执行器符号先对照 [MOD-002 §0](docs/04-数学建模/MOD-002-坐标系与符号约定.md) 和 [FrameConventions.h](TandemVec_FCS/include/FrameConventions.h)。纵列双发悬停语义：上摆主控滚转，下摆主控俯仰，差速主控偏航；不要把模型系的轴名直接套进固件。

## 命令

以下在仓库根执行。日常按改动面选最小验证集合；完整清单和分档见 [VER-009](docs/07-验证与确认/VER-009-SIL验收脚本判据表-2026-11-07.md)。

```powershell
py -3.12 tools/verify-all.py
node --test "simulations/vector-thrust-lab/tests/*.test.mjs"
py -3.12 tools/sync-params.py --check
py -3.12 tools/build-docs.py --check
py -3.12 tools/check-links.py
py -3.12 tools/check-code-comments.py --check
```

- 改代码先跑快档；行为可能变化（包括共享默认值、控制分配、估计）再跑 `--include-host`。改 SIL 公共代码必须跑 `--include-sil`，覆盖全部三套跑批机型。仅文档改动至少跑 `git diff --check` 和受影响的文档守卫。
- 改仿真参数后运行 `py -3.12 tools/sync-params.py`；站点 Markdown 改动后运行 `py -3.12 tools/build-docs.py` 同步产物。仿真、论文、地面站与固件直调的专用入口见 [项目地图](docs/PROJECT-MAP.md)，硬件操作另见 [固件规则](TandemVec_FCS/AGENTS.md)。

## 修改边界

- 仿真参数的正本是 [aircraft-model.json](models/aircraft-model.json)，文档引用其中的参数值时明确标注来源；固件控制参数以 [FlightCtrlParams.h](TandemVec_FCS/include/FlightCtrlParams.h) 为准。未经标定的仿真参数不外推实机。
- 不手改 `standalone.html`、`parameters.mjs` 或 `simulations/vector-thrust-lab/docs/*.html`；分别用 `build-standalone.py`、`sync-params.py` 和 `build-docs.py` 生成。源文件、生成物、参考快照的归属见 [项目地图](docs/PROJECT-MAP.md)。
- 接新机型只补机型层，不改共享控制和导航。改仿真先查对应 [计算层](simulations/vector-thrust-lab/src/core/)；需要固件控制或导航算法时走 [固件直调接口](TandemVec_FCS/test_host/fw_algo/)，`CORE_SOURCES` 是直调构建清单，不在 `sim/` 复刻算法。仿真积分统一 RK4。
- 新写或修改公式性代码前读 [GOV-004](docs/00-项目治理/GOV-004-代码注释与理论对照规范.md)，同步中文注释、`@formula`、`@ref` 和新文件 L0 头。改导航宏、模块、测试时同步 [宏状态表](docs/04-数学建模/代码理论核查/上板标定与验证清单-2026-11-01.md) 与 [核查索引](docs/04-数学建模/代码理论核查/README.md) 中受影响的条目。
- 估计、导航或控制算法先用理想传感器、WGS-84 正常重力和解析/高精度真值验证正确性，再加入真实噪声、振动、延迟和失锁评估工程效果。架构迁移按相同条件下与实飞旧基线的**响应一致性**验收，不以参数相同判通过。
- 临时文件只写仓库 `tmp/`；复用脚本进 `tools/`。Windows ASCII 盘符仅在编译工具确实需要时临时映射，使用后立即撤销。用户未要求时不碰硬件、第三方快照或无关修改。

## 行为红线

改行为前逐项读 [行为保持红线清单](docs/registers/行为保持红线清单.md)：时步、积分与保范、差速公式、角度符号、力臂、限幅和地面判定均受控。批准改动后重采 `simulations/vector-thrust-lab/tests/fixtures/regression-baseline.json`，同步文档并在提交里说明原因。仿真概念包线与固件实机包线不能互换。固件安全规则见 [固件指令](TandemVec_FCS/AGENTS.md)。

## ★ 动手前必查的沉淀

按触发条件点读，不把历史记录当当前配置。总索引门禁由 `tools/check-memory-index.py` 执行。

| 文件 | 何时读 |
|---|---|
| [`2026-11-24-EKF惰性表达式悬垂与全链路SIL口径.md`](docs/memory/2026-11-24-EKF惰性表达式悬垂与全链路SIL口径.md) | 写返回 Eigen 表达式的 lambda；排查 SIL 时好时坏、定高读数 10.52 m 或退出码 134；用 SIL 推断实飞协方差行为；用 WSL 跑 SIL |
| [`2026-11-12-IMU-ISR轮询频率选档.md`](docs/memory/2026-11-12-IMU-ISR轮询频率选档.md) | 改 ODR／ISR、增构建环境或排查环境编译失败 |
| [`2026-11-11-GNSS串口逐字节中断丢帧.md`](docs/memory/2026-11-11-GNSS串口逐字节中断丢帧.md) | 丢历元、校验失败、定位跳变或串口重配 |
| [`2026-11-11-IMU延迟根因-FIFO速率失配.md`](docs/memory/2026-11-11-IMU延迟根因-FIFO速率失配.md) | 改 IMU 取数或排空、延迟不断增长 |
| [`2026-11-10-openocd烧录Tcl路径转义.md`](docs/memory/2026-11-10-openocd烧录Tcl路径转义.md) | 跨机烧录、Tcl 路径或 PowerShell 多层传参 |
| [`2026-11-10-GCS仪表盘重做.md`](docs/memory/2026-11-10-GCS仪表盘重做.md) | 改仪表读数、布局、Canvas 或协议字段显示 |
| [`2026-11-10-GCS平板可点性验收.md`](docs/memory/2026-11-10-GCS平板可点性验收.md) | 改触控布局、控件尺寸或做平板验收 |
| [`2026-11-08-共轴roll-pitch不对称排查.md`](docs/memory/2026-11-08-共轴roll-pitch不对称排查.md) | 查共轴姿态不对称或准备动增益 |
| [`2026-11-05-单涵道四燃气舵首飞.md`](docs/memory/2026-11-05-单涵道四燃气舵首飞.md) | 查实飞证据、改该机型或上位机链路 |
| [`2026-11-09-ICM45686换型适配.md`](docs/memory/2026-11-09-ICM45686换型适配.md) | IMU 换型、改驱动或查数据链路和滤波异常 |
| [`2026-11-06-数传接入与遥测帧率排查.md`](docs/memory/2026-11-06-数传接入与遥测帧率排查.md) | 接数传、遥测频率异常或下行丢帧 |
| [`2026-11-04-IMU-2kHz攻关.md`](docs/memory/2026-11-04-IMU-2kHz攻关.md) | 测采样率、丢帧和时序，核对已排除路线 |
| [`2026-11-03-导航调度通信基线.md`](docs/memory/2026-11-03-导航调度通信基线.md) | 改 GNSS、调度或判断 EKF 状态 |
| [`2026-11-01-固件模块与SIL进展.md`](docs/memory/2026-11-01-固件模块与SIL进展.md) | 改 SIL、导航或 EKF 链接与验证 |
| [`2026-10-24-PIO20升级-ArduinoCore3迁移.md`](docs/memory/2026-10-24-PIO20升级-ArduinoCore3迁移.md) | 动构建工具链或框架 |
| [`2026-08-11-D1-级联架构LMI鲁棒界证明.md`](docs/memory/2026-08-11-D1-级联架构LMI鲁棒界证明.md) | 审控制架构论证或写论文对应章节 |
| [`2026-08-11-相位裕度与量纲分析.md`](docs/memory/2026-08-11-相位裕度与量纲分析.md) | 改控制增益或审量纲 |

## 多会话并发与变更联动规范

- revo 是唯一开发、编译和提交机；其他节点只读或在明确授权下接收产物。改动前获取 `origin/main`、看 `git status --short` 并逐个检查领先提交；不覆盖其他会话工作，不用 `reset --hard` 或 `push --force`。跨机同步按 [GOV-003 §6](docs/00-项目治理/GOV-003-多Agent协作规范.md)，先核对对端真实 HEAD 和未提交内容，再按快进流程同步并验回 HEAD。
- 同步前 `git diff --check`，仅提交任务涉及的路径；在 `main` 分段中文提交，尾注 `Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>`，完成后快进推送。GitHub 代理见 [GOV-003](docs/00-项目治理/GOV-003-多Agent协作规范.md) 与本机网络配置；已知 revo 本地代理为 `http://127.0.0.1:7897`。
- **联动清单**：改固件默认值，同步参数定义、静态断言和宿主默认值断言；改参数注册数量/顺序，同步 `ANO_PARAMS_COUNT`、GCS 表和 ID 断言、文档；改协议帧，同步固件/GCS/对端解析，并按实测抓帧验证。改默认值或参数数量必须跑完整宿主回归。

## 约定与知识沉淀协议

- 文档以 Markdown 为正本，编号 `域前缀-三位序号-标题.md`，章节引用写 `见 CTL-001 §3`。已核实的过时内容就地改对；不要留容易被当成当前规则的错误对照。
- PDF 正式阅读或入库先用 MinerU：电子版用 `standard`，扫描件与低质量 PDF 用 `advanced`。保存转换来源、档位和 manifest。仿真结果保留时序、轨迹、分布、CDF、散点/热图等适用视角，出结论前逐张目视核对；TikZ 源码放论文 `fig/`，公式与实现核对符号。
- **P1**：「记住/记下来/沉淀」或确证踩坑时，在 `docs/memory/` 写 3–5 个要点，首屏标「触发条件」，登记在本页索引并运行 `py -3.12 tools/check-memory-index.py`。
- **P2**：收到纠正后更新 [踩坑正本](docs/registers/踩坑记录-AGENTS.md) 或对应专题；**P3**：发现非显然约定先查重，稳定全仓规则才放本页。
- **P4**：同一流程被指导三次，提议封装技能；创建或改技能走 `skill-creator-plus`，项目技能统一放 `.agents/skills/`。**P5**：完工反思；没有值得沉淀的事实就不写。

## 记忆下沉归档（自动迁移）

- SIL 跑批与覆盖判据（2026-11-19 自记忆下沉）：`bash test_host/fw_sil/run_all_airframes.sh` 一键跑纵列/共轴/单涵道并逐项判据——纵列姿态 10.00m 阶跃 + roll 峰值 5.86°、共轴 24 cases 0 failed、单涵道 10.00m + 5.86°；纵列 run_acceptance.sh（真实固件+EKF、3 seeds 定高闭环、约 81s）已并入第 1/3 阶段。液体火箭无固件 env（fw_sil_env_main 零引用，用 LiquidRocketModel.h + 简化 6DOF plant），走宿主回归 run_all.sh 的 test_liquid_rocket_sil，故「三机型 SIL」准确说法须同时 --include-sil --include-host。**覆盖判据＝跑批最终汇总里有该项输出**，脚本存在且单独跑绿不等于已覆盖。**纵列手动模式油门直通不控高**：alt=-458m 是设计行为不是发散，判据只判姿态峰值+解锁，不加高度/发散断言。防重复造轮子：纵列已有 test_roundtrip（20% 容限）、共轴已有独立 dynamics 与控制测试。
- 推进层三端同构守卫（2026-11-19 自记忆下沉）：`tools/check-propulsion-parity.py`（仿真 control-allocation.mjs vs 固件 computeWrench，9 组工况×6 分量＝54 项对拍，退出码即结论）：容差用「绝对+相对」混合（纯相对会把 Mx≈-3.5e-4 类小分量的 4.9e-9 差值放大成 FAIL）。MinGW ld 写不了中文绝对路径→cwd 到仓库根用相对路径。覆盖仅稳态，Qf 的 Jp·dω/dt 瞬态项未对拍（已知空白非已证等价）。
- 参考固件 VTVL_ElectricDualRotor_FCS（2026-11-19 自记忆下沉）：位于 `D:\PIO_Projects\VTVL_ElectricDualRotor_FCS`（master，HEAD `c322f64`，2026-10-31），是「共轴双桨单机型完整固件」，被 TANDEMVEC 三机型平台在功能面完全覆盖；桌面已无对应 zip。其磁力计链路（MagSensor/MagCal 标定+Flash 持久化）已接硬件但未融合：`navigation_task.cpp:811-819` 真实磁路径整段注释，EKF 航向用伪磁矢量 (30,0,40) 初始化，飞行中靠 GNSS 航迹角−侧滑角，无可回收的连续融合策略。桌面 `TandemVec_FCS` 目录是当前工程 2026-11-06 旧快照（非 git 仓库），缺磁力计改接 I2C1、DTCM 可选 env、GNSS 统一入口修复、静止磁重对准、航向模式默认值等后续改动。
- 两份 AGENTS.md 分工（2026-11-20 自记忆下沉，判据）：根 `AGENTS.md` 只放跨域规则（命令入口、参数源、原则、红线、Git/LaTeX/Skill/PDF 约定、多会话并发、知识沉淀）；固件 `.agents/AGENTS.md` 放机型工程事实（三机型选择表、轴系、执行器分配、寄存器、串口、烧录验证）。判据：换成另一机型/模块仍成立 → 写根，否则写固件。速率环是例外：纵列/共轴走 α 域，单涵道走舵角域直通，两域量级差两个数量级，两边都要写并写明例外。
- 机型参数与调参结论（2026-11-21 自记忆下沉）：m＝0.7 kg、力臂 0.315 m、摆角 ±15°、单桨 13.75 N；2026-08 实机首飞通过进入调参。参数唯一源 `models/aircraft-model.json` → `sync-params.py`。结论＝INDI 不必要，增益调度 PID 即可（轴间耦合 <10%）。
- Ano 调参链路（固件参数在线读写的工程事实）：固件参数注册数以 `ANO_PARAMS_COUNT` 为准，命令 0xE0/0xE1/0xE2 逐字节对齐，写入下一拍生效。`param list` 兜底必须用「本地参数表长度兜底 + 截断固件上报值」，否则 count 未及时返回时尾部参数全被漏读（commit c2f9f07）。上板验收（断动力、CH6 关）：`server/cli.py --port COMx param get/set sedf.vane_max_deg` 写后读回、`param list | grep sedf` 列全；读写全超时先 exit 掉 DBG（DBG 模式暂停参数链路）。固有限制：0xE1 只在固件侧拒 NaN/Inf、不做范围校验 → GCS 按 meta min/max 拦截；参数不持久化，需 param dump + 上电 param load。源 Universal-EDF 缺陷勿照搬：大倾角联锁写 `|AHRS_Packet.Roll|>45.0f` 但字段单位是 rad（永不触发），移植改用 acos|R33|。
- 角加速度量级与舵机延迟定位（实测）：①PID 输出的 `alpha_ref_dps2` 范围约 −5.5~+4.4 平滑，fast MC 用的 `alpha_act`（未滤波差分）范围约 −24.9~+11.8 跳变——同名「角加速度」噪声量级差 3-5 倍，这是 fast MC 的 lead 参数搬到固件后失效的原因。②舵机是物理输出不经数传 ⇒ 延迟必在控制链内；抓 0x01 陀螺 vs 0x03 EKF 欧拉角互相关（COM14、90s），滞后几十上百 ms＝EKF 姿态环，几 ms＝输出/舵机驱动。
