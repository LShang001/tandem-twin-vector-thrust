# TandemVec-EKF-Report — 固件 EKF 组合导航理论与算法分析设计报告

本工程对纵列双发矢量推力飞行器固件（`TandemVec_FCS`）的 EKF 组合导航系统进行
逐行级代码分析，输出一份专业论文风格的 80 页级 LaTeX 分析与设计报告。

## 2026-10-24 超大规模仿真测试活动

- 活动记录与结论：`仿真测试活动-2026-10-24.md`
- 原始结果：`sim-data/campaign-2026-10-24/`
- 覆盖：固件 SIL、导航验收/振动矩阵、GNSS 延时 10k、延时闭环 5k、
  EKF P0 200 runs、全包线 A0–A9、高保真巡航/VTOL。

## 分析对象（固件代码）

| 文件 | 角色 |
|------|------|
| `lib/navigation-main/src/ekf_15_state.h` | 15 状态误差状态 EKF（4058 行，核心） |
| `src/navigation_task.cpp` | 200 Hz 导航任务：IMU 双子样组装、静止检测、辅助调度、GNSS 融合、输出桥接 |
| `include/VerticalKF_2State.h` | 2 状态垂直卡尔曼滤波器 |
| `include/HorizontalKF.h` | 2 轴水平卡尔曼滤波器（光流速度观测） |
| `include/ins_gnss_dynamic_weight.h` | GNSS 动态权重 R（clamp + pDOP 缩放） |
| `include/ins_gnss_epoch_timing.h` | GNSS iTOW→本地时刻映射与量测年龄 |
| `include/ins_static_detector.h` | 静止检测器（阈值 + 迟滞状态机 + 置信度） |
| `include/ins_static_aid_profile.h` | 静止辅助强度分级调度（ZUPT/Gravity/StaticGyro） |
| `include/ins_altitude_reference.h` | 气压高度基准换算 |

## 2026-10 专业文献调研与升级评估（新增）

在两本专业教材（秦永元《惯性导航（第三版）》《卡尔曼滤波与组合导航原理（第3版）》）
全文转换 + 40 篇文献题录/精读 + 6 个子代理并行研究的基础上，新增：

- **主报告**：`导航算法升级评估-专业文献调研-2026-10.md`
  （结论：现有 ESKF 骨架正确，P0-0/P0-1/P0-3/P1-5 已入固件并通过 host；升级重点在航向绝对基准、
  GNSS 子集 FDI、ZUPT 外部参照、噪声/辅助源标定；不建议换 UKF/CKF/粒子/因子图/联邦/紧组合）
- **文献库**：`../docs/08-参考资料/`（books/REF-120/121、papers、notes/00–05、index/、check-refs.py；OCR 校订记录见两本书的 `ocr-errata.md`）
- **团队笔记**：
  - `notes/01-捷联惯导理论.md`（含 P0-0 数值证明）
  - `notes/02-卡尔曼滤波理论.md`（ESKF/可观性/一致性/FEJ）
  - `notes/03-组合导航与鲁棒性.md`（FDI/ZUPT/CUSUM）
  - `notes/04-航向与磁辅助.md`（磁矢量/TRIAD/健康状态机）
  - `notes/05-仿真验证.md`（已完成：A/B/C/P0 四套件 + H 索引修正）
  - `notes/06-深读-对准与航向可观测性.md`（静基座航向不可观、MEMS 罗经、P1-5 Schmidt、P0-2 磁矢量 H）
  - `notes/07-深读-一致性与故障检测.md`（NEES/SNIS/CUSUM/GLR、状态 χ²、P1-3/P1-7）
- **新增仿真**：`sim/opt/upgrade_candidates.py` + `sim-data/opt/upgrade_candidates.json`
  （静态误确认、磁航向、GNSS FDI A/B；已记录负面/边界结果）；P0-3 修复对照见
  `sim-data/opt/upgrade_P0_guard.json`（SC9b 末端 51.6→11.2 m）。

> 重要：`notes/01` 发现速度旋转效应双计（P0-0），**2026-10-21 已修并验证**
> （PDF p.298–299 + host T7 19/19 + `run_all.sh` 全绿 + `pio run` SUCCESS）；
> `notes/05` 发现仿真双矢量 H 索引错位（`H[0,7:10]` vs 固件 `6..8`，旧 H2/闭环结论
> 需重读），**已在 `opt_ekf.py` 修正**，修正后：磁 A 套件在机动/失联段有条件增益
> （SC3 56.6→1.16 m）、FDI B 套件位置改善但航向退化（SC3 0.47→3.12°）、C 套件悬停
> 严重有害（yaw 0.26→17–24°）、**ZUPT 1 m/s 盲区已实施 0.8 门+外部参照门控，
> 51.6→11.2 m**。这些都在主报告 §6.5–6.9 如实记录，不能只看“升级项”标题。

## 仿真

`sim/` 下为独立数值仿真（Python + NumPy），按固件真实参数实现误差状态 EKF 核心，
生成报告第 10 章的图表：

```bash
cd sim && py -3.12 run_scenarios.py
```

输出：`sim-data/summary.json` + `fig/*.png`（6 个场景）。

### 优化研究（2026-08-13 新增，`sim/opt/`）

算法与参数优化研究框架（见《优化研究-算法与参数优化研究报告.md》）：

```bash
cd sim/opt
py -3.12 run_baseline.py --seeds 20   # 基线蒙特卡洛（8 场景 × N 种子）
py -3.12 run_sweep.py all             # 18 项参数扫描（分组：gnss/qnoise/static/outage）
py -3.12 run_variants.py              # 13 组算法变体 A/B
py -3.12 make_digest.py               # 生成数据摘要 sim-data/opt/digest.md

cd ../hifi && py -3.12 run_scenes_opt.py  # 高保真固件闭环变体验证（5 场景 × 9 配置）
```

研究产出：`sim-data/opt/{baseline_mc,sweep_*,variants,hifi_variants}.json`。
固件已实施修改：GNSS 速度静态门控、P0 航向 π→0.2、ZUPT NIS 1000→16.3。

## 编译

```bash
xelatex -interaction=nonstopmode main.tex   # 需 x2~3 次
# 或
latexmk -xelatex main.tex
```

## 文档约定

- 公式符号体系与固件注释一致：NED/FRD、乘性四元数、体系右乘误差角 δβ^b
- 所有"固件数值"均标注源码出处（文件:行号）
- 仿真结果为独立复现，用于演示算法行为，不替代实机标定
