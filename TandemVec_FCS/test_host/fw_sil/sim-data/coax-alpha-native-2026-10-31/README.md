# 共轴 α-native 证据（2026-10-31）

对应改动：`coax.alloc_mode=2`（SIL 参数 `--coax-alloc-mode 2`），速率内环直接输出
机体系角加速度 α，由 `CoaxialAllocator` 逆解为摆角/差速；`alloc_mode=0/1` 路径未改。

- 仓库基线：`190c0f1` + 本次 α-native 工作区（未提交）
- SIL 二进制：`T:/sil_alpha/fw_coax.exe`（`build_and_run_coaxial.sh --build-only`）
- 输入：同一二进制、同一 seed、同一 `--rc-profile`，只切换 `--coax-alloc-mode`

## 文件

| 文件 | 说明 |
|---|---|
| `ab_summary.md` | `run_coaxial_ab.py --alloc-modes 0,2` 自动摘要（含判定、表、provenance） |
| `ab_metrics.json` | 原始指标（48 组：16 场景 × 2 路径；含 thr30/50/70） |
| `acceptance_alloc0.log` | legacy 3×5 验收（8/9，唯一 FAIL `mag_nominal seed1` 22.069°） |
| `acceptance_alloc2.log` | α-native 3×5 验收（8/9，同一 FAIL 场景 22.764°） |
| `acceptance_alloc2_8x5.log` | α-native 8 profile×5 seeds 验收（**39/40**，唯一 FAIL 仍 `mag_nominal seed1` 22.764°） |
| `acc_inertia15_0.log` / `acc_inertia15_2.log` | 真实惯量×1.5：legacy 与 α-native 同一失败集（mag_clean/mag_nominal hold 超限）；α-native 未引入额外惯量敏感 |

## 自算对照（ab_metrics.json，峰值/上升/超调）

- thr70 离地 12 例：峰值差 ≤0.03%、上升时间差 0.00%、超调差 ≤0.04pp。
- RATE 全档（thr30/50/70）：峰值差 ≤0.10%、上升时间差 0.00%。
- thr30/50 地面档：峰值差 ≤0.33%，仅作一致性检查。

注：`run_coaxial_ab.py` 的 verdict 只统计 thr50（本改动后按 `final_alt<2 m` 标
`NO_LIFT`、不计 FAIL），因此显示 "PASS 0/0"；空中判别力以 thr70 行为与上表为准。
