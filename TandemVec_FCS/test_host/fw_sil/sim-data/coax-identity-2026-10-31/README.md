# 共轴 α 适配器 identity 与 A/B 证据（2026-10-31 02:04 复跑）

目的：回答"新 α 架构的控制响应是否与实飞验证过的旧共轴路径一致"。

- 仓库状态：`c18e694` + 当时未提交的 identity/液体火箭改动（最终提交为 `ae39a55`）。
- 二进制：`T:/sil_final2/fw_coax.exe`（含 `--coax-alloc-mode` 与 `--rc-profile`）。
- 命令：`run_coaxial_ab.py --bin T:/sil_final2/fw_coax.exe --out T:/ab_identity --work T:/ab_work`
- 关键结论：RATE 12/12 逐位等价（上升时间/峰值差 <1e-9）；ATTITUDE 在 thr70 离地条件下 4/4 逐位等价，thr50 推力 7 N < 重量 8.24 N 为 NO_LIFT（无判别力）；惯量/推力失配 4 档×3 seeds 同档同 seed 逐位相同。
- 口径修正：`--trace-ctrl` 的 roll/pitch 为弧度、`alt_up` 为 cm；ATTITUDE 阶跃前末高 <2 m 记 NO_LIFT。
- `ab_summary.md` 为脚本原始输出；`ab_metrics.json` 为逐场景指标。
