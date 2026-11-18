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
