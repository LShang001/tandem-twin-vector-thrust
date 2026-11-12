# UniGNC

**机型无关的通用飞控平台** · Vehicle-Agnostic Flight Control Platform

一套 GNC 内核，四类飞行器实飞验证：VTVL 液体火箭 · 纵列双发矢量推力 · 共轴双桨 VTVL · 单涵道四燃气舵

---

## 为什么会有这个项目

常规飞控把"机型"写进控制律：执行器数量、混控矩阵、舵效、限幅散落在控制代码各处，
每接一种新构型都要动主干，改完还得重新验证整条链路。

UniGNC 反过来做——**让控制律看不见执行机构**。内环只输出广义控制量（三轴力矩需求 /
目标角加速度），由可插拔的机型分配层连同惯量、控制效率矩阵、舵效与执行器限幅一并解算。
接入新飞行器时，姿态环、15 状态组合导航、200 Hz 调度一行不改，只补一个分配层
（`*Allocator.h`）。

通用性来自数学上的归一：任何机型的控制映射都能写成「广义力矩到执行器偏量」的线性算子，
附带饱和与联合限幅约束；控制律只与算子的维数打交道，不关心算子背后装的是舵面、摆座、
燃气舵还是涵道推进器。**矩阵满秩是充分条件，联合限幅不破坏方向是必要条件。**

## 已验证的四类飞行器

| 机型 | 执行机构 | 状态 |
|---|---|---|
| VTVL 液体火箭 | 发动机推力调节 + 两轴摆座 | 实飞验证 |
| 纵列双发矢量推力（基型） | 2 电机 + 2 正交单轴摆座 + 差速 | 实飞验证（2026-08 首飞） |
| 共轴双桨 VTVL | 双桨差速 + TVC | 实飞验证 |
| 单涵道四燃气舵 | 1 涵道推进器 + 4 片燃气舵 | 实飞验证（2026-11-05，出厂默认参数首飞即稳） |

四者的执行机构形式截然不同，但差异止于执行层——其上的姿态环、导航与控制分配契约是同一份代码。

> ⚠️ **成熟度声明**：固件域已完成上述实飞验证，进入调参优化阶段。仿真参数
> （`models/aircraft-model.json`）仍为 `MODEL-DEFAULT`/`ESTIMATED`（未经台架标定）；
> 仿真结果不构成实际飞行性能或安全承诺。同一块板可烧多种机型，**烧录前必须先确认这次飞哪一型**。

## 基型（纵列双发）的构型

项目从这一构型起步，它仍是理解分配层设计意图的最好入口：**前后纵列双电机、正交单轴摆座、反向旋转螺旋桨**。

- **前电机**（拉力式，正转）绕机体 z 轴单轴摆动 → 悬停构型下的**滚转主控**
- **尾电机**（推进式，反转）绕机体 y 轴单轴摆动 → **俯仰主控**
- **滚转/偏航**由前后电机差速产生的反扭矩差驱动（共轴直升机扭矩差原理）
- 零摆角时两推力线均过质心，无静态推力力矩

> 注：巡航构型下同两摆读作前摆/尾摆，执行器语义以悬停（机头朝天）构型为准，
> 详见 `docs/04-数学建模/MOD-002-坐标系与符号约定.md` 与 `TandemVec_FCS/AGENTS.md`。

## 📁 仓库结构

| 目录 | 内容 |
|---|---|
| `docs/00-项目治理` | 项目范围、术语、信息与配置管理规则 |
| `docs/01-方案设计` | 概念构型基线、运行概念（ConOps） |
| `docs/02-需求与验证` | 需求基线与验证方法 |
| `docs/03-理论推导` | 与实现无关的一般理论（姿态数学、推进物理、刚体动力学） |
| `docs/04-数学建模` | 本构型六自由度模型规范、坐标系与符号、配平与稳定性 |
| `docs/05-控制与分配` | 控制架构、SAS 设计、可控性与失效模式 |
| `docs/06-推进与执行机构` | 推进与摆座模型、限制与标定计划 |
| `docs/07-验证与确认` | 验证确认方法、模型可信度、安全计划 |
| `docs/08-参考资料` | 参考资料注册表 |
| `docs/09-决策记录` | 架构决策记录（ADR） |
| `docs/registers` | 参数数据手册、假设日志、追溯矩阵等受控注册表 |
| `models/` | 模型参数单一事实源（JSON + Schema） |
| `TandemVec_FCS/` | STM32H743 飞控固件（纵列双发矢量推力 VTOL 实机版，EKF 组合导航 + TVC 控制分配） |
| `simulations/vector-thrust-lab` | Web 六自由度交互仿真子项目（standalone 离线版，双击直开） |
| `simulations/py-web-lab` | Python 引擎 + Web 可视化仿真子项目（统一架构研究版，可插拔控制律 INDI/LQR/ADRC） |
| `simulations/high-fidelity-analysis` | Python 高保真分析（论文仿真数据/图、先进控制算法研究） |
| `scripts/` | 控制算法研究脚本（MPC/INDI/在线辨识、配平求解） |
| `assets/` | 构型候选图与论文配图素材 |
| `blender-test/` | Blender 三维渲染实验脚本 |
| `tools/` | 文档构建、standalone 打包、参数同步、链接检查脚本 |

## 📚 文档地图

项目文档按域组织，**全部 192 篇的唯一入口是 [`docs/README.md`](docs/README.md)**——里面有分域清单、
每篇一句话摘要，以及「我要改 X → 先读哪几篇」的任务索引。

新会话（人或 agent）建议按这个顺序进场：

| 顺序 | 读什么 | 解决什么 |
|---|---|---|
| 1 | 本文件 | 项目是什么、验证到什么程度 |
| 2 | [`docs/00-项目治理/GOV-005-项目全景与历史.md`](docs/00-项目治理/GOV-005-项目全景与历史.md) | **至今做过什么、现在停在哪、哪些坑已踩过、哪些结论还不能当定论** |
| 3 | [`docs/README.md`](docs/README.md) | 全部文档的导航中枢 + 按任务索引 |
| 4 | [`docs/PROJECT-MAP.md`](docs/PROJECT-MAP.md) | 东西在哪、哪些能改、哪些是生成物 |
| 5 | [`AGENTS.md`](AGENTS.md) | 必跑命令、行为红线、动手前必查清单 |
| 6 | [`TandemVec_FCS/AGENTS.md`](TandemVec_FCS/AGENTS.md) | 固件侧：env 体系、IMU 采样、遥控通道语义、串口映射、烧录 |

三个「动手前先查」的正本：

- [`docs/registers/踩坑记录-AGENTS.md`](docs/registers/踩坑记录-AGENTS.md) — 130+ 条历史踩坑，按关键词检索
- [`docs/memory/`](docs/memory/) — 15 篇攻坚全过程，每篇首行标了「触发条件」
- [`docs/08-参考资料/器件手册与驱动资产索引.md`](docs/08-参考资料/器件手册与驱动资产索引.md) — ICM-42688/45686、DPS310、F10N 的手册与驱动资产

> ★ **文档是前置输入，不是事后记录。** 改任何模块前先按关键词扫上面三处，比翻代码快，也比重新踩坑快。

## 🚀 快速开始

零依赖直接运行：双击 `simulations/vector-thrust-lab/standalone.html`（单文件版，无需服务器）。

开发模式（需 Python 3）：

```bat
simulations\vector-thrust-lab\启动.bat
```

或手动：

```bash
cd simulations/vector-thrust-lab
python -m http.server 8080
# 浏览器打开 http://localhost:8080
```

详见 [仿真子项目 README](simulations/vector-thrust-lab/README.md)。

其他子项目：飞控固件见 [TandemVec_FCS/README.md](TandemVec_FCS/README.md)，Python+Web 仿真研究版见 [py-web-lab/README.md](simulations/py-web-lab/README.md)。

## 📐 工程约定

- **坐标系**：NED 惯性系（z 向下）；机体系 x 前 / y 右 / z 下，原点在质心
- **单位**：内部统一 SI；角度内部用弧度，界面可用度
- **文档**：Markdown 为唯一编辑源，HTML 由 `tools/build-docs.py` 生成，禁止手改
- **参数**：`models/aircraft-model.json` 为唯一事实源，仿真参数文件由 `tools/sync-params.py` 生成
- **版本**：文件名不含"最终版/v2"等状态词，版本由 Git 标签与基线编号管理
- **许可证**：本项目采用 [MIT 许可证](LICENSE)；`simulations/vector-thrust-lab/vendor/` 内第三方库遵循其各自许可证（Three.js：MIT）
