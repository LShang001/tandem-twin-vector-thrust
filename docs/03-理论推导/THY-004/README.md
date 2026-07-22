# 纵列双发矢量推力飞行器：构型原理、动力学建模与控制特性分析

> THY-004 ｜ LaTeX 技术文档工程

## 编译

```bash
# Windows（双击运行或命令行）
build.bat

# 或手动
xelatex main.tex    # ×3（交叉引用需要三次）
```

## 目录结构

```
THY-004/
├── main.tex                 # ★ 主入口文件（\input 组装所有部件）
├── preamble.tex             # 导言区（文档类、宏包、定理环境、TikZ 库）
├── build.bat                # Windows 一键编译脚本
├── README.md                # 本文件
│
├── ch/                      # 章节内容（独立 .tex，可单独修改）
│   ├── 01-intro.tex         # 第1章：引言（背景、构型分类、本文结构）
│   ├── 02-propulsion.tex    # 第2章：推进系统建模（动量理论、叶素理论、反扭矩、电机动态、陀螺效应）
│   ├── 03-mapping.tex       # 第3章：推力矢量映射（坐标系、摆座运动学、六维映射）
│   ├── 04-dynamics.tex      # 第4章：六自由度刚体动力学（牛-欧方程、四元数、积分策略）
│   ├── 05-allocation.tex    # 第5章：控制分配（效能矩阵、解耦机理、耦合评估）
│   ├── 06-stability.tex     # 第6章：稳定性模态分析（线性化、纵向/横航向模态、SAS影响）
│   ├── 07-control.tex       # 第7章：增稳控制律设计（SAS律、角速度闭环、饱和管理）
│   ├── 08-discussion.tex    # 第8章：讨论（优势/局限/对比/验证路径/扩展方向）
│   └── 09-conclusion.tex    # 第9章：结论
│
├── fig/                     # 图表源码与生成资产
│   ├── 01-configuration.tex     # 图1：LaTeX 包装与图注
│   ├── 01-configuration.html    # 图1：HTML/SVG 信息图源码
│   ├── 01-configuration.png     # 图1：1600×1000、约 300 dpi 构建产物
│   ├── 01-ai-selected-j.png     # 图1：经独立审查选中的 AI 概念底图（非尺度）
│   ├── 01-web-render.png        # 图1：Web 三维模型无界面渲染层
│   ├── 02-actuator-disk.tex     # 图2：桨盘动量理论控制体模型
│   ├── 03-thrust-mapping.tex    # 图3：推力矢量映射几何（双视图）
│   ├── 04-stability-effects.tex # 图4：SAS 增益定性因果图
│   ├── 05-control-arch.tex      # 图5：控制架构级联框图
│   ├── 06-config-compare.tex    # 图6：构型机制来源与定位图
│   └── 07-web-validation.tex    # 图7：Web core 非线性回归响应
│
└── ref.bib.tex              # 参考文献（thebibliography 环境，19 篇）
```

## 迭代指南

| 要改什么 | 改哪个文件 |
|----------|-----------|
| 修改某章内容 | `ch/XX-name.tex` |
| 修改图1构型信息图 | `fig/01-configuration.html`，重新截图生成同名 PNG；AI 候选与提示词见 `assets/ai-generated/aircraft-configuration/2026-07-23/` |
| 修改其他图的绘制 | `fig/XX-name.tex`（TikZ/PGF 源码） |
| 添加新章节 | `ch/` 下新建 `.tex`，在 `main.tex` 中 `\input` |
| 添加新图表 | `fig/` 下新建 `.tex`，在对应章节中 `\input{../fig/XX.tex}` |
| 添加新宏包 | `preamble.tex` |
| 添加新参考文献 | `ref.bib.tex` |
| 修改标题/作者 | `main.tex` |

## TikZ 图表单独预览

每个 `fig/*.tex` 文件可在独立文档中编译预览：

```latex
\documentclass{standalone}
\usepackage{tikz}
\usetikzlibrary{arrows.meta,shapes,positioning,calc,patterns,angles,quotes,3d,backgrounds,fit}
\usepackage{amsmath}
\begin{document}
\input{fig/01-configuration.tex}
\end{document}
```

## 技术栈

- **编译器**：XeLaTeX（UTF-8 + 中文 ctex）
- **图表**：TikZ/PGF（无外部图片依赖，纯矢量）
- **数学**：amsmath + mathtools + bm
- **超链接**：hyperref + cleveref
- **参考文献**：thebibliography 手动环境

## 勘误记录

### 2026-07-20 — 推力矢量映射符号修正（与仿真代码交叉验证）

以 `simulations/vector-thrust-lab/src/core/propulsion.mjs` 为参考，修正以下错误：

| 位置 | 原内容 | 修正 | 依据 |
|------|--------|------|------|
| §3 尾电机受力方向 | `[-cosδ_t, 0, sinδ_t]^T`（排气方向） | `[+cosδ_t, 0, -sinδ_t]^T`（牛顿第三定律：对机体作用力与排气反向） | `dyn.Fz = -Tt * st` |
| §3 尾电机角动量 | `[+cosδ_t, 0, -sinδ_t]^T` | `[-cosδ_t, 0, +sinδ_t]^T`（反转转子，h_t 沿机体受力反方向） | `hv.x = -Jp * wt * ct` |
| §3 F_x 公式 | `T_f cosδ_f - T_t cosδ_t` | `T_f cosδ_f + T_t cosδ_t` | `dyn.Fx = Tf*cf + Tt*ct` |
| §3 F_z 公式 | `T_t sinδ_t` | `-T_t sinδ_t`（δ_t>0 → 受力向上 → F_z<0） | `dyn.Fz = -Tt*st` |
| §2 陀螺轴线 | q 致前转子陀螺力矩沿 `+y_b` | `+z_b`（pitch→yaw coupling，非 pitch axis） | `M_gyro = -Ω×h` 推导 |
| §3,§5 耦合比 | `(D/b)·(δ_f/δ_t)`，估值 20%–60% | 增加 `C_Q/C_T` 因子，修正估值 3%–15% | `k_Q/k_T = (C_Q/C_T)·D` |
| §8 引用 | `ducard2009modeling`/`kim2004nonlinear`（年份不匹配） | `ducard2008modeling`/`kim2003nonlinear` | 原文发表年 |
| preamble | `subcaption`, `longtable`, `tabularx`, `listings` 等未引用包 | 移除 | 全文 `\cite` 扫描 |

完整推导交叉验证见 `propulsion.mjs:28–34`、`dynamics.mjs:48–52`、`control.mjs`。

## 旧版

原单体文件位于上级目录：
`../THY-004-纵列双发矢量推力飞行器构型原理与技术分析.tex`
（保留作为历史参考，后续迭代以本目录的分拆版本为准）
