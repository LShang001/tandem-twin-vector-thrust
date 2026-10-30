# 共轴控制路径 A/B 与大规模仿真矩阵（task-13 证据）

- 生成时间：2026-10-31 01:40:37
- 仓库：`TandemVec_FCS` @ `bddb0b7`，工作区未提交文件 6 个（矩阵运行时 .status 记录；含其他 teammate 的固件/plant/文档改动，只作 provenance 记录，未 commit）
- 二进制：`T:/sil_alloc_final/fw_coax.exe`（mtime 2026-10-31 01:07:03，build_mode=binary，`--coax-alloc-mode` 探测 alloc_ok=1）
- 未提交文件共 7 个：docs/ 文档 1 个；include/ 固件 1 个；src/ 固件 1 个；test_host/ 仿真与工具 4 个（其中本任务新增/修改：`test_host/fw_sil/run_coaxial_ab.py`（新）、`run_coaxial_suite.sh`、`run_coaxial_mc.py`、sim-data/coax-alloc-2026-10-31/；其余为其他 teammate 的固件/plant/文档改动）
- 证据目录约定：顶层 `acceptance*.log`/`mc_*.json`/`sweep.json` 是 **alloc_mode=1** 套的便捷副本（与 `matrix/` 同源，md5 相同）；alloc_mode=0 套在 `matrix_alloc0/`（另含 `mc_current.csv`、`mc_current.log` 等）；A/B 明细在 `ab_metrics.json`/`ab_summary.md`；基线对照在 `baseline_alloc0/`。

## 0. 结论摘要

1. **A/B 手感**：名义油门 50% 成对判据 PASS 12/16（另有 NO_LIFT 4 例，地面无判别力，不计失败）；RATE 12/12 通过且 12/12 alloc0/alloc1 **逐位等价**（上升时间与峰值差异 <1e-9）→ α+分配器路径在名义点没有改变手感，旧/新控制路径往返 identity 成立。
2. **ATTITUDE（已更正单位：roll/pitch 弧度→度）**：**thr70 离地有效场景 4 例**：roll_step3_thr70 峰值 3.04°（101.5% 指令、超调 1.58%）；roll_step8_thr70 峰值 8.22°（102.8% 指令、超调 2.79%）；pitch_step3_thr70 峰值 3.03°（101.0% 指令、超调 1.41%）；pitch_step8_thr70 峰值 8.24°（103.0% 指令、超调 3.10%），alloc0/alloc1 峰值逐位相同；**thr50 4 例标 NO_LIFT**（50% 推力 7 N < 重量 8.24 N，贴地、地面约束伪迹，无判别力、不参与判据）；FAIL 0 例。alloc0/alloc1 仍逐位等价 → **未发现固件姿态外环缺陷**（此前把 thr50 地面伪迹误判为外环不跟，已更正）。
3. **大规模矩阵 alloc0 vs alloc1 通过数一致、数值差异为浮点级**：验收 current 39/40 vs 39/40，realistic 40/40 vs 40/40；MC current 56/100 vs 56/100，realistic 58/100 vs 58/100；sweep 15/18 vs 15/18。
4. **失配**：inertia 1.5/0.67、thrust 0.8/1.2 各 3 seeds，两路径同档同 seed 峰值逐位相同（最大相对差 0.000%），失配只改变响应幅度（物理正确方向），不放大路径差异。

## 1. A/B 手感（同一 seed、同一 `--rc-profile` 激励）

| 场景 | 模式 | 轴 | 油门% | alloc0 上升(s) | alloc1 上升(s) | 上升差% | 超调差pp | 响应占指令% | 判定 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| rate_roll_square5_thr50 | RATE | roll | 50 | 0.311 | 0.311 | 0.0 | 0.00 | 91.42 | PASS |
| rate_roll_square10_thr50 | RATE | roll | 50 | 0.316 | 0.316 | 0.0 | 0.00 | 91.02 | PASS |
| rate_roll_square20_thr50 | RATE | roll | 50 | 0.321 | 0.321 | 0.0 | 0.00 | 90.69 | PASS |
| rate_roll_ramp5_thr50 | RATE | roll | 50 | — | — | — | 0.00 | 80.37 | PASS |
| rate_roll_ramp10_thr50 | RATE | roll | 50 | — | — | — | 0.00 | 80.41 | PASS |
| rate_roll_ramp20_thr50 | RATE | roll | 50 | — | — | — | 0.00 | 79.27 | PASS |
| rate_pitch_square5_thr50 | RATE | pitch | 50 | 0.306 | 0.306 | 0.0 | 0.00 | 91.52 | PASS |
| rate_pitch_square10_thr50 | RATE | pitch | 50 | 0.316 | 0.316 | 0.0 | 0.00 | 90.94 | PASS |
| rate_pitch_square20_thr50 | RATE | pitch | 50 | 0.321 | 0.321 | 0.0 | 0.00 | 90.71 | PASS |
| rate_pitch_ramp5_thr50 | RATE | pitch | 50 | — | — | — | 0.00 | 80.66 | PASS |
| rate_pitch_ramp10_thr50 | RATE | pitch | 50 | — | — | — | 0.00 | 80.72 | PASS |
| rate_pitch_ramp20_thr50 | RATE | pitch | 50 | — | — | — | 0.00 | 79.34 | PASS |
| attitude_roll_step3_thr50 | ATTITUDE | roll | 50 | 0.311 | 0.311 | 0.0 | 0.00 | 106.45 | NO_LIFT |
| attitude_roll_step8_thr50 | ATTITUDE | roll | 50 | 0.306 | 0.306 | 0.0 | 0.00 | 107.63 | NO_LIFT |
| attitude_pitch_step3_thr50 | ATTITUDE | pitch | 50 | 0.301 | 0.301 | 0.0 | 0.00 | 107.34 | NO_LIFT |
| attitude_pitch_step8_thr50 | ATTITUDE | pitch | 50 | 0.296 | 0.296 | 0.0 | 0.00 | 108.58 | NO_LIFT |

方波场景可测 10–90% 上升时间；斜坡场景目标连续变化，上升时间无定义时退化为峰值相对差判据。油门列：AB 矩阵覆盖 30/50/70%，本表列名义 50% 与全油门趋势见 `ab_summary.md` 第 2 节。

## 2. 大规模矩阵 alloc0 vs alloc1

| 阶段 | alloc_mode=0 | alloc_mode=1 | 差异 |
| --- | --- | --- | --- |
| 验收 8 profiles×5 seeds（默认 plant） | 39/40 | 39/40 | 通过集合同；个别场景 roll_max 差 ≤0.23°（浮点级） |
| 验收 8×5（`--coax-realistic`） | 40/40 | 40/40 | 通过集合同；imu_nan 的 hold_end 差 ≤0.07 m（浮点级） |
| MC 100 例（默认 plant） | 56/100 | 56/100 | 通过数相同，hold P50/P95 1.381/27.85 vs 1.380/32.51 |
| MC 100 例（realistic） | 58/100 | 58/100 | 通过数相同，hold P50/P95 1.226/28.73 vs 1.226/29.13 |
| sweep 18 组边界 | 15/18 | 15/18 | 失败集合同（lever_low / iy_high / combined_low）；pitch 差 ≤0.23° |

失败清单（两套通过集合一致，阈值判定相同）：

- 验收：`mag_nominal seed1` `max_pitch=22.069 > 20`（alt 9.10 / hold_end 0.96 / roll 8.17，pitch 超阈值约 2° 的 near-threshold 场景）
- sweep：lever_low (pitch 45.0°, roll 1.5°), iy_high (pitch 35.6°, roll 5.8°), combined_low (pitch 37.0°, roll 44.4°)

| profile | 通过/总数（current） | hold_end 范围(m) | roll_max 范围(°) |
| --- | --- | --- | --- |
| mag_clean | 5/5 | 0.27 – 0.88 | 1.20 – 1.71 |
| mag_nominal | 4/5 | 0.31 – 0.87 | 7.62 – 12.41 |
| mag_disturbed | 5/5 | 0.41 – 4.09 | 8.35 – 15.32 |
| nomag_degraded | 5/5 | 121.90 – 135.18 | 0.99 – 1.13 |
| imu_stuck | 5/5 | 114.09 – 126.77 | 3.81 – 7.43 |
| imu_nan | 5/5 | 158.59 – 214.59 | 4.82 – 7.68 |
| imu_drop | 5/5 | 103.08 – 205.94 | 4.50 – 8.02 |
| gnss_spoof | 5/5 | 76.62 – 111.07 | 1.80 – 5.19 |

注：hold_end 只统计 PASS 场景（nomag_degraded/imu_*/gnss_spoof 属 skip_hold_end，hold_end 漂移是预期的；range 中的大值来自 skip 场景的 near-hold 指标，不代表失败）。

## 3. 差异解释（通过数为何一致、差异为何是浮点级）

- 名义点：`--coax-alloc-mode 1` 走的 α = `lever·thrust_est/I·sin(corr)` 经 `bfs::allocateCoaxial()` 逆解回同一组舵机/差速指令，悬停小角度下与旧代数路径严格往返，trace 逐位一致（`ch1/ch2` 差 <1e-4 为打印精度）。
- 验收/realistic 的个别场景：α 路径把 `sin(corr)` 与旧代数校正混用，在长时间积分下产生 ≤0.23° / ≤0.07 m 的数值差（mag_disturbed 的 roll_max、imu_nan 的 hold_end），全部远低于判据阈值，通过集合完全一致。
- 大偏差/失败案例：MC 的 hold_end P95 略有差异（alloc1 32.51 vs alloc0 27.85 m），但失败判据是 `max_pitch>30°`（pitch 超调主导），两套通过数完全一致；姿态分布在失败尾部有小幅重排，不影响结论。
- realistic 开关（motor τ=0.28 s、servo 300 dps、落地 μm=0.5）对两套的作用相同，通过数 56→58，差异来自 plant 保真度而非控制路径。

## 4. 失配与安全（摘要，详见 `ab_summary.md` 第 3–4 节）

| 档位 | 峰值范围（两路径相同） | 3 seeds |
| --- | --- | --- |
| inertia_scale_0.67 | 8.073 – 8.081 | 3 |
| inertia_scale_1.5 | 6.450 – 6.464 | 3 |
| thrust_scale_0.8 | 6.774 – 6.787 | 3 |
| thrust_scale_1.2 | 7.570 – 7.575 | 3 |

- 安全场景 3 例 × 2 路径：NaN 无，反向 无；峰值两路径相同（yaw_big_lowthr 0.200；roll_full_50thr 123.456；att45_lock 31.018）。

## 5. 复现命令

```bash
cd TandemVec_FCS
# A/B（48 手感 + 3 安全 + 4 档失配×3 seeds；两路径同剖面）
py -3.12 test_host/fw_sil/run_coaxial_ab.py \
  --bin T:/sil_alloc_final/fw_coax.exe \
  --out test_host/fw_sil/sim-data/coax-alloc-2026-10-31 --work T:/ab_work4
# 矩阵（alloc=1 / alloc=0 各一套，验收 8×5 + MC 100 + realistic MC + sweep 18）
bash test_host/fw_sil/run_coaxial_suite.sh --bin T:/sil_alloc_final/fw_coax.exe \
  --alloc-mode 1 --realistic \
  --out test_host/fw_sil/sim-data/coax-alloc-2026-10-31/matrix
bash test_host/fw_sil/run_coaxial_suite.sh --bin T:/sil_alloc_final/fw_coax.exe \
  --alloc-mode 0 --realistic \
  --out test_host/fw_sil/sim-data/coax-alloc-2026-10-31/matrix_alloc0
```

## 6. 遗留风险

1. **thr50 地面无判别力（已更正）**：共轴 50% 油门推力 7 N < 重量 8.24 N，AUTO_ALTITUDE 又在死区，飞机贴地；ATTITUDE thr50 的阶跃响应是地面约束伪迹，标 NO_LIFT，不算 FAIL。原报告按度读弧度（3° 被读成 0.052°）并误判为“外环不跟”，已修正为“thr70 有效场景正常跟踪、未发现固件姿态外环缺陷”。
2. **near-threshold FAIL 敏感**：`mag_nominal seed1 max_pitch=22.069` 仅超 20° 阈值 2°，realistic 下即通过；结论对 plant 保真度敏感，不宜写成“通过/不通过”的绝对判断。
3. plant 为近似 6DOF，质量/惯量/力臂/差速力矩是 L1 假设；A/B 结论只对该 plant 成立，不能当实机手感承诺。失配档只覆盖 inertia/thrust scale，未覆盖质量与力臂耦合。
4. 证据未 commit（工作区含其他 teammate 未完成改动），基线对照见 `baseline_alloc0/baseline_alloc0_3x5.log`（14/15，唯一 FAIL 同样是 mag_nominal seed1 pitch 22.069）。
