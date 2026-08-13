# TandemVec-EKF-Report — 固件 EKF 组合导航理论与算法分析设计报告

本工程对纵列双发矢量推力飞行器固件（`TandemVec_FCS`）的 EKF 组合导航系统进行
逐行级代码分析，输出一份专业论文风格的 80 页级 LaTeX 分析与设计报告。

## 分析对象（固件代码）

| 文件 | 角色 |
|------|------|
| `lib/navigation-main/src/ekf_15_state.h` | 15 状态误差状态 EKF（3968 行，核心） |
| `src/navigation_task.cpp` | 200 Hz 导航任务：IMU 双子样组装、静止检测、辅助调度、GNSS 融合、输出桥接 |
| `include/VerticalKF_2State.h` | 2 状态垂直卡尔曼滤波器 |
| `include/HorizontalKF.h` | 2 轴水平卡尔曼滤波器（光流速度观测） |
| `include/ins_gnss_dynamic_weight.h` | GNSS 动态权重 R（clamp + pDOP 缩放） |
| `include/ins_gnss_epoch_timing.h` | GNSS iTOW→本地时刻映射与量测年龄 |
| `include/ins_static_detector.h` | 静止检测器（阈值 + 迟滞状态机 + 置信度） |
| `include/ins_static_aid_profile.h` | 静止辅助强度分级调度（ZUPT/Gravity/StaticGyro） |
| `include/ins_altitude_reference.h` | 气压高度基准换算 |

## 仿真

`sim/` 下为独立数值仿真（Python + NumPy），按固件真实参数实现误差状态 EKF 核心，
生成报告第 10 章的图表：

```bash
cd sim && py -3.12 run_scenarios.py
```

输出：`sim-data/summary.json` + `fig/*.png`（6 个场景）。

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
