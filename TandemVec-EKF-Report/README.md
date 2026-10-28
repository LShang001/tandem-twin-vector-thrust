# TandemVec-EKF-Report — 固件 EKF 组合导航理论与算法分析设计报告

本工程对纵列双发矢量推力飞行器固件（`TandemVec_FCS`）的 EKF 组合导航系统进行
逐行级代码分析，输出一份专业论文风格的 80 页级 LaTeX 分析与设计报告。

## 2026-10-24 超大规模仿真测试活动

- 活动记录与结论：`仿真测试活动-2026-10-24.md`
- 原始结果：`sim-data/campaign-2026-10-24/`
- 覆盖：固件 SIL、导航验收/振动矩阵、GNSS 延时 10k、延时闭环 5k、
  EKF P0 200 runs、全包线 A0–A9、高保真巡航/VTOL。

## 2026-10-26 大过载/大机动与 GNSS 杆臂补偿

- 报告：`高动态卫惯组合导航-大过载大机动-2026-10-26.md`
- 台架：`../TandemVec_FCS/test_host/test_ekf_high_dynamic.cpp`（固件 EKF，2 kHz 解析真值）
- 矩阵：`sim/run_high_dynamic_mc.py`（1420 例）、`sim/run_lever_ab.py`（150 例）
- 结论：2–8 g/180–1080°/s 位置 0.20–0.25 m；>8 g 饱和、≥330 ms 延迟为硬边界；GNSS 杆臂位置+速度补偿已入固件（参数 `gnss_lever_x/y/z`），8 g/2 m 杆臂 1.98→0.21 m。

## 2026-10-27 6DOF 导弹弹体 + 真实固件导航 SIL

- 报告：`导弹6DOF-导航SIL-2026-10-27.md`
- 台架：`../TandemVec_FCS/test_host/fw_sil/missile_plant.h` + `--missile`（6DOF/气动/自驾，只注入传感器；真实 `handleNavigationSystem()`）
- 矩阵：`sim/run_missile_sil_mc.py`（20/40 seeds/格，全精度 vs MCU 宏路径）；可视化套件 `sim/plot_missile_sil.py`、`plot_missile_envelope.py`、`plot_missile_visuals.py`、`plot_missile_sensor_fidelity.py`、`plot_vib_isolator.py`（时序/3D/箱线/小提琴/CDF/直方/散点/热图/条形/频响）
- **结论（2026-10-27 armed 口径）**：代表弹道 6.4 g/451 m/s/1987 m。SIL 现在在 t≥1 s 置 `unlocked=true/throttle=50%`（飞行武装），旧 unarmed 的 hz5 5.79/200 ms 7.72/冻结 3.11 作废。armed 40 seeds：ICM 0.68°/2.46 m/0.158 m/s，5 s 失联 0.65°/2.61 m，200 ms 3.80°/37.8 m，5 Hz 0.66°；失联 10/20/30 s 位置 8.87/13.07/20.47 m。过载 ±8 g：8.4 g 3.69 m 可用；10.1 g 触顶 622.7 m 不可用。armed 后 fresh gate 只剩振动作用（1 g 6.94→1.94°、2 g 8.32→7.97°），其余 19 格逐项相同；VTOL 100 seeds 默认 291/300、fresh 290/300，但 fresh 消掉 24 m 尾部（max hold 2.28 m）。高零偏对准 opt-in 20/20。**物理减震座**：推荐 fn≈20–25 Hz（含 80 Hz）/30 Hz（仅 120 Hz）、ζ≈0.10；2 g/120/8.4 g 7.97→0.64°、速度 0.63→0.24 m/s，1 g/80 1.93→0.98°，发射 2 g 8.73→0.66°；容差网格见 `figs/nav_vib_isolator_tolerance_2026-10-27.png`。**IMU 短时失效组合 fallback（opt-in，上限 2 s）**：NaN 1 s 121.75→3.15 m、冻结 1 s 35.24→2.33 m（0 拒绝/0 重锚）；2 s 为 12.67/65.5 m；33 s 长故障仍失导；10.1 g 饱和不受益。200 ms 延迟臂仍是“数据+iTOW 同移”的未建模延迟口径，见报告 §4.2/§4.5/§4.6/§4.9/§4.10。


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
