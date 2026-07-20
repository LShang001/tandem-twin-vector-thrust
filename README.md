# 纵列双发矢量推力飞行器

**Tandem Twin Vector-Thrust Aircraft** (`tandem-twin-vector-thrust`)

_纵列双发、正交单轴矢量推力、差速反扭滚转的固定翼飞行器概念项目 —— 方案设计 · 理论推导 · 数学建模 · 仿真验证_

---

本项目研究一种非常规布局的飞行器概念：**前后纵列双电机、正交单轴摆座、反向旋转螺旋桨**。

- **前电机**（拉力式，正转）绕机体 z 轴单轴摆动 → 主偏航通道
- **尾电机**（推进式，反转）绕机体 y 轴单轴摆动 → 主俯仰通道
- **滚转**由前后电机差速产生的反扭矩差驱动（共轴直升机扭矩差原理）
- 零摆角时两推力线均过质心，无静态推力力矩

> ⚠️ **成熟度声明**：项目当前处于**概念设计 — 集总参数建模 — 交互式仿真演示**阶段。仓库内所有参数均为 `MODEL-DEFAULT`（模型默认值，未经台架标定或飞行验证）；仿真结果不构成实际飞行性能或安全承诺。

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
| `simulations/vector-thrust-lab` | Web 六自由度交互仿真子项目 |
| `tools/` | 文档构建、参数同步、链接检查脚本 |

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

## 📐 工程约定

- **坐标系**：NED 惯性系（z 向下）；机体系 x 前 / y 右 / z 下，原点在质心
- **单位**：内部统一 SI；角度内部用弧度，界面可用度
- **文档**：Markdown 为唯一编辑源，HTML 由 `tools/build-docs.py` 生成，禁止手改
- **参数**：`models/aircraft-model.json` 为唯一事实源，仿真参数文件由 `tools/sync-params.py` 生成
- **版本**：文件名不含"最终版/v2"等状态词，版本由 Git 标签与基线编号管理
- **许可证**：本项目采用 [MIT 许可证](LICENSE)；`simulations/vector-thrust-lab/vendor/` 内第三方库遵循其各自许可证（Three.js：MIT）
