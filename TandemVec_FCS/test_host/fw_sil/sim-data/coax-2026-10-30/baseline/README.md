# 共轴 VTVL 基线复证据（2026-10-30）

本目录是 task-7 要求「先跑基线复现 15/15、60/100、15/18」的原始证据。
与文档记录（VER-002 / CTL-003）不一致的数字**如实保留**，未改任何阈值。

## 构建与二进制

- 仓库：`TandemVec_FCS` @ `57be7ef`（docs(nav): 定位导航对拍 0.53 m 残差…）
- 工作区：19 个未提交文件，含
  - `test_host/fw_sil/build_and_run_coaxial.sh` 的 Madgwick 源补丁（task-5/lead）；
  - `test_host/fw_sil/fw_sil_env_main.cpp` 的共轴 plant 保真度开关
    `--coax-realistic`（task-1，**未冻结**，默认不开）。
- 二进制：`/c/Users/12631/AppData/Local/Temp/tv_coax_bin/fw_coax.exe`
  （2026-10-30 19:30 由 `build_and_run_coaxial.sh --build-only` 构建）。
- 构建开关：`-DTANDEMVEC_FIRMWARE -DFW_SIL_REAL_NAV -DFW_SIL_COAXIAL
  -DBFS_COAXIAL_VTVL -DBFS_EKF_BARO_ALTITUDE_UPDATE`。
- 注意：此时 plant/固件尚未收到 lead 的 "plant/firmware frozen" 通知，
  本基线用于复现历史数字，**不是最终验收矩阵**；最终矩阵以
  `../summary.md` 为准。

## 复跑命令

```bash
cd TandemVec_FCS
BIN=/c/Users/12631/AppData/Local/Temp/tv_coax_bin/fw_coax.exe
OUT=test_host/fw_sil/sim-data/coax-2026-10-30/baseline

# 1) 文档口径：3 profiles × 5 seeds
bash test_host/fw_sil/run_coaxial_acceptance.sh --bin "$BIN" \
    --profiles mag_clean,mag_nominal,nomag_degraded --seeds 1,2,3,4,5 \
    > "$OUT/acceptance_3x5.log" 2>&1

# 2) 8 profiles 单 seed 语义核验（4 个故障 profile 的判据位）
bash test_host/fw_sil/run_coaxial_acceptance.sh --bin "$BIN" \
    --profiles mag_clean,mag_nominal,mag_disturbed,nomag_degraded,imu_stuck,imu_nan,imu_drop,gnss_spoof \
    --seeds 1 > "$OUT/profile_semantics_1seed.log" 2>&1

# 3) 随机参数 MC 100 例
py -3.12 test_host/fw_sil/run_coaxial_mc.py --bin "$BIN" --cases 100 \
    --out "$OUT/mc100.json" > "$OUT/mc100.log" 2>&1

# 4) plant 参数/故障扫描 18 组
py -3.12 test_host/fw_sil/run_coaxial_sweep.py --bin "$BIN" \
    --out "$OUT/sweep18.json" > "$OUT/sweep18.log" 2>&1
```

## 结果（逐项）

### 1. 验收 3 profiles × 5 seeds = **14/15**

| 场景 | 通过 | 失败原因 |
| --- | --- | --- |
| mag_clean | 5/5 | — |
| mag_nominal | 4/5 | seed1 `max_pitch=22.108° > 20°`（roll 8.21°、hold 0.96 m 均合格） |
| nomag_degraded | 5/5 | —（`pos_degraded=1`，hold_end 121–136 m 漂移，判据 `skip_hold_end`） |

文档 VER-002 记录 15/15；本次唯一差异是 `mag_nominal seed1` 的俯仰超调，
属于真实回归/裕度不足，不是脚本或环境问题。

### 2. 8 profiles 单 seed 语义核验（`profile_semantics_1seed.log`）

| profile | 结果 | 关键证据位 |
| --- | --- | --- |
| mag_clean | PASS | alt 9.86、hold 0.87、roll 1.26、mag 391/1 |
| mag_nominal | FAIL | 仅 `max_pitch=22.108° > 20°`，其余合格 |
| mag_disturbed | PASS | mag 208/184、mag_gate 置位、roll 12.78 |
| nomag_degraded | PASS | `pos_degraded=1`、alt 9.87、roll 1.03（hold 不判） |
| imu_stuck | PASS | `imu_detected=1`、alt 9.12 |
| imu_nan | PASS | `imu_detected=1`、alt 9.10 |
| imu_drop | PASS | `imu_detected=1`、alt 9.10 |
| gnss_spoof | PASS | `gnss_crej>=1`（alt 0.03、hold 不判） |

结论：8 个 profile 在共轴二进制里都有真实语义，无“假覆盖”项。

### 3. 随机参数 MC 100 例 = **56/100**

| 指标 | 本次基线 | 文档 VER-002 |
| --- | --- | --- |
| 通过率 | 56/100 | 60/100 |
| hold_end P50 / P95 | 1.272 / 27.590 m | 1.26 / 64.93 m |
| max_roll P50 / P95 | 1.611 / 12.247° | — / 23.9° |
| max_pitch P50 / P95 | 28.119 / 58.126° | — / 55.8° |

最差 8 例全部是俯仰超调（pitch 49–70°），其中 5 例伴随位置保持失效：

| case | mass | T | L | Iy | Iz | yawk | seed | alt | hold_end | max_pitch |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 073 | 0.75 | 15.8 | 0.10 | 0.10 | 0.11 | 0.00064 | 5 | 10.21 | 135.80 | 69.52 |
| 014 | 0.73 | 14.9 | 0.11 | 0.09 | 0.10 | 0.00088 | 1 | 8.61 | 58.38 | 62.68 |
| 046 | 0.88 | 13.6 | 0.09 | 0.10 | 0.08 | 0.00044 | 1 | 9.64 | 57.10 | 54.79 |
| 005 | 0.81 | 13.3 | 0.09 | 0.09 | 0.07 | 0.00028 | 8 | 10.15 | 39.91 | 60.28 |
| 088 | 0.87 | 15.5 | 0.09 | 0.06 | 0.11 | 0.00092 | 5 | 9.79 | 28.10 | 49.25 |
| 083 | 0.82 | 15.5 | 0.13 | 0.09 | 0.09 | 0.00093 | 6 | 9.83 | 27.15 | 52.76 |
| 054 | 0.86 | 15.3 | 0.11 | 0.08 | 0.09 | 0.00118 | 5 | 9.76 | 27.56 | 48.64 |
| 013 | 0.79 | 13.5 | 0.13 | 0.09 | 0.09 | 0.00074 | 5 | 10.31 | 21.11 | 52.52 |

### 4. plant 扫描 18 组 = **15/18**

失败 3 组（与文档一致）：

| 场景 | alt | hold_end | max_roll | max_pitch | f4 | 原因 |
| --- | --- | --- | --- | --- | --- | --- |
| lever_low（L=0.08） | 9.63 | 5.52 | 1.43 | 45.03 | 1 | pitch 超调、F4 检测置位 |
| iy_high（Iy=0.08） | 9.86 | 4.59 | 5.38 | 35.39 | 1 | pitch 超调、F4 检测置位 |
| combined_low（m=0.70/T=12/L=0.08/Iy=0.03/Iz=0.04） | 10.17 | 2.28 | 34.75 | 35.66 | 1 | roll+pitch 双超调、F4 检测置位 |

- `mass_low`（mass=0.70）本次 **PASS**（pitch 27.86°），文档笼统写的
  “mass=0.70 边界失败”在本次构建上未复现；真正失败的是 `lever_low`、
  `iy_high`、`combined_low`。
- 故障检测：`both_loss_60` → f3=1（双发 60% 损失检测通过）；`servo_jam`
  → f4=1（舵机卡死检测通过）。

## 与文档数字的差异归因（待查项）

- HEAD 相对 VER-002/CTL-003 记录时已接入**生产 Madgwick AHRS** 与多处
  导航/传感器链改动（`M src/...`、`M fw_sil_env_main.cpp`）；当前构建的
  姿态响应与文档快照不是同一份代码。
- 失败集中在**俯仰超调/姿态边界**：`mag_nominal seed1` pitch 22.1°、
  MC pitch P95 58.1°、sweep 的 lever/iy/combined 低值边界 pitch 35–45°。
  MC 的 hold P95 反而从 64.93 m 降到 27.59 m，说明位置环没有整体变差，
  是姿态裕度单独下降。
- 与真实导航切换、磁航向融合链的关系需在 plant/固件定稿后复核；本任务
  不改阈值、不把 56/100 写成 60/100。

## 证据文件

| 文件 | 内容 |
| --- | --- |
| `acceptance_3x5.log` | 3 profiles × 5 seeds 逐例 PASS/FAIL（14/15） |
| `profile_semantics_1seed.log` | 8 profiles × 1 seed 语义核验（7/8） |
| `mc100.json` / `mc100.csv` | 100 例随机参数 MC 的逐例行、P50/P95、最差 8 例 |
| `mc100.log` | MC 控制台汇总 |
| `sweep18.json` | 18 组 plant/故障扫描逐例结果 |
| `sweep18.log` | sweep 控制台逐例 PASS/FAIL |
