# 共轴 VTVL 验证证据（coax-2026-10-30）

- 生成时间：2026-10-30 19:52:14
- 仓库：`TandemVec_FCS` @ `57be7ef`（工作区未提交文件 21 个）
- 二进制：`/c/Users/12631/AppData/Local/Temp/tv_coax_final_bin/fw_coax.exe`（构建时间 2026-10-30 19:49:58，大小 881202 bytes / 861 KB）
- `MadgwickAHRS.cpp` 构建补丁：已包含（lead 在冻结前补入，修复 HEAD 链接失败）
- 矩阵：profiles=`mag_clean,mag_nominal,mag_disturbed,nomag_degraded,imu_stuck,imu_nan,imu_drop,gnss_spoof`；seeds=`1,2,3,4,5`；seconds=40；双套（--coax-realistic）=是
- 验收口径：8 profiles × 5 seeds，current 40 场景 + realistic 40 场景 = **80 场景**（不使用旧文档的 3×5=15 口径）。
- 口径：真实固件（`BFS_COAXIAL_VTVL`）+ 近似 6DOF plant，不是台架/实机证据；plant 质量/惯量/力臂/差速力矩为 L1 假设。

## 阶段结果与退出码

| 阶段 | 退出码 | 判定 | 证据文件 |
| --- | --- | --- | --- |
| 构建 | 0 | PASS | `build.log` |
| 验收 8 profiles（current） | 1 | FAIL | `acceptance.log` |
| 验收 8 profiles（realistic） | 0 | PASS | `acceptance_realistic.log` |
| MC 100（current） | 1 | FAIL | `mc_current.json` |
| MC 100（realistic） | 1 | FAIL | `mc_realistic.json` |
| sweep 18 组（current） | 1 | FAIL | `sweep.json` |

**套件退出码：1**（0=全部通过；1=任一阶段失败。注意：历史基线 MC/sweep 未达 100%，默认运行即返回 1，是真实门禁。）

## final 运行命令（2026-10-30 19:49，plant/firmware 冻结后）

```bash
cd TandemVec_FCS
# 构建（ASCII BIN_DIR，避免中文路径问题）
BIN_DIR=/c/Users/12631/AppData/Local/Temp/tv_coax_final_bin \n  bash test_host/fw_sil/build_and_run_coaxial.sh --build-only
# 全矩阵：8 profiles × 5 seeds current + 同矩阵 realistic + MC 100 ×2 + sweep 18
BIN_DIR=/c/Users/12631/AppData/Local/Temp/tv_coax_final_bin \n  bash test_host/fw_sil/run_coaxial_suite.sh --realistic
```

各阶段独立命令（等价，便于单跑）：

```bash
BIN_DIR=... bash test_host/fw_sil/run_coaxial_acceptance.sh --full                        # current 8×5
BIN_DIR=... bash test_host/fw_sil/run_coaxial_acceptance.sh --full --append-args=--coax-realistic  # realistic 8×5
py -3.12 test_host/fw_sil/run_coaxial_mc.py --cases 100                                   # MC current
py -3.12 test_host/fw_sil/run_coaxial_mc.py --cases 100 --realistic                       # MC realistic
py -3.12 test_host/fw_sil/run_coaxial_sweep.py                                            # sweep 18
```

## 验收矩阵（8 profiles，5 seeds）

| 场景 | current 通过 | realistic 通过 | current hold_end [m] | realistic hold_end [m] | current max_roll [°] | realistic max_roll [°] | 说明 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| mag_clean | 5/5 | 5/5 | 0.27–0.87 | 0.27–0.88 | 1.17–1.68 | 0.75–1.19 | — |
| mag_nominal | 4/5 | 5/5 | 0.31–0.96 | 0.30–0.85 | 7.61–12.42 | 7.50–12.65 | max_pitch=22.108 > 20.0 |
| mag_disturbed | 5/5 | 5/5 | 0.41–4.08（不判） | 0.39–3.94（不判） | 8.35–15.32 | 8.37–14.51 | 位置保持不判（skip_hold_end） |
| nomag_degraded | 5/5 | 5/5 | 121.64–135.58（不判） | 109.30–123.20（不判） | 1.00–1.13 | 0.93–1.06 | 位置保持不判（skip_hold_end） |
| imu_stuck | 5/5 | 5/5 | — | — | — | — | 仅判故障检测（skip_motion） |
| imu_nan | 5/5 | 5/5 | — | — | — | — | 仅判故障检测（skip_motion） |
| imu_drop | 5/5 | 5/5 | — | — | — | — | 仅判故障检测（skip_motion） |
| gnss_spoof | 5/5 | 5/5 | — | — | — | — | 仅判故障检测（skip_motion） |
| **合计** | **39/40** | **40/40** | — | — | — | — | — |

### near-threshold 敏感性（禁止误读）

- current 唯一失败 `mag_nominal seed1` 的 `max_pitch=22.108°` 仅超 20° 阈值 **2.108°**；realistic 下同一 seed 通过（roll 8.94°，pitch 未超 20°）。
- 这说明该场景处在阈值边界：realistic 预设（电机 τ=0.28 s、舵机 300°/s+量化、地面 μ=0.5）带来的 plant 微差即可翻转结果。**不能**解读为“pitch 超调问题已解决”，也不能解读为 realistic 比 current 更安全。
- 两套 8×5 共 80 个场景（40 current + 40 realistic）中，只有 `mag_nominal seed1` 这 1 例出现 current/realistic 不一致，其余 79 次结果一致。

## 随机参数 MC（100 例）

| 组 | 通过数 | 通过率 | hold P50/P95 [m] | roll P50/P95 [°] | pitch P50/P95 [°] |
| --- | --- | --- | --- | --- | --- |
| current（默认 plant） | 56/100 | 56% | 1.27/27.59 | 1.61/12.25 | 28.12/58.13 |
| realistic（--coax-realistic） | 57/100 | 57% | 0.99/21.23 | 1.10/11.74 | 27.15/57.25 |

realistic 相对 current 通过数变化：+1 例。

### current ↔ realistic 差异归因

- 差异 +1 例（56→57）来自 plant 开关：realistic 打开电机一阶滞后（τ=0.28 s）、舵机速率/量化（300°/s、0.1% PWM）与落地后地面摩擦（μ=0.5）；默认 plant 无这些动态。
- 两套最差 8 例几乎同集合（current: 73/14/46/5/88/83/54/13；realistic: 73/14/23/89/54/5/59/83），全部是俯仰超调（`max_pitch` 47.8–72.3°），说明**俯仰裕度不足在默认 plant 就存在，不是 realistic 新开关引入的**。
- 统计量变化小：hold P95 27.59→21.23 m，roll P95 12.25→11.74°，pitch P95 58.13→57.25°；结论方向一致。

### MC 最差 8 例（current（默认 plant））

| 案例 | mass | T | L | Iy | Iz | yawk | seed | alt | hold_end | max_roll | max_pitch | 原因 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 73 | 0.75 | 15.8 | 0.10 | 0.10 | 0.11 | 0.00064 | 5 | 10.21 | 135.80 | 16.36 | 69.52 | hold_end=135.801>6.0; max_pitch=69.517>30.0 |
| 14 | 0.73 | 14.9 | 0.11 | 0.09 | 0.10 | 0.00088 | 1 | 8.61 | 58.38 | 18.23 | 62.68 | hold_end=58.381>6.0; max_pitch=62.678>30.0 |
| 46 | 0.88 | 13.6 | 0.09 | 0.10 | 0.08 | 0.00044 | 1 | 9.64 | 57.10 | 7.62 | 54.79 | hold_end=57.104>6.0; max_pitch=54.786>30.0 |
| 5 | 0.81 | 13.3 | 0.09 | 0.09 | 0.07 | 0.00028 | 8 | 10.15 | 39.91 | 2.11 | 60.28 | hold_end=39.914>6.0; max_pitch=60.280>30.0 |
| 88 | 0.87 | 15.5 | 0.09 | 0.06 | 0.11 | 0.00092 | 5 | 9.79 | 28.10 | 1.21 | 49.25 | hold_end=28.097>6.0; max_pitch=49.246>30.0 |
| 83 | 0.82 | 15.5 | 0.13 | 0.09 | 0.09 | 0.00093 | 6 | 9.83 | 27.15 | 7.77 | 52.76 | hold_end=27.150>6.0; max_pitch=52.761>30.0 |
| 54 | 0.86 | 15.3 | 0.11 | 0.08 | 0.09 | 0.00118 | 5 | 9.76 | 27.56 | 1.23 | 48.63 | hold_end=27.563>6.0; max_pitch=48.635>30.0 |
| 13 | 0.79 | 13.5 | 0.13 | 0.09 | 0.09 | 0.00074 | 5 | 10.31 | 21.11 | 1.29 | 52.52 | hold_end=21.115>6.0; max_pitch=52.522>30.0 |

### MC 最差 8 例（realistic（--coax-realistic））

| 案例 | mass | T | L | Iy | Iz | yawk | seed | alt | hold_end | max_roll | max_pitch | 原因 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 73 | 0.75 | 15.8 | 0.10 | 0.10 | 0.11 | 0.00064 | 5 | 5.32 | 91.63 | 10.54 | 72.31 | final_alt=5.324 越界[8.0,11.0]; hold_end=91.632>6.0; max_pitch=72.309>30.0 |
| 14 | 0.73 | 14.9 | 0.11 | 0.09 | 0.10 | 0.00088 | 1 | 10.44 | 59.77 | 17.79 | 62.71 | hold_end=59.774>6.0; max_pitch=62.706>30.0 |
| 23 | 0.77 | 15.1 | 0.08 | 0.07 | 0.12 | 0.00050 | 6 | 9.61 | 48.84 | 3.35 | 67.72 | hold_end=48.845>6.0; max_pitch=67.719>30.0 |
| 89 | 0.87 | 15.8 | 0.10 | 0.08 | 0.06 | 0.00114 | 4 | 10.25 | 30.30 | 9.96 | 51.54 | hold_end=30.304>6.0; max_pitch=51.544>30.0 |
| 54 | 0.86 | 15.3 | 0.11 | 0.08 | 0.09 | 0.00118 | 5 | 9.93 | 23.25 | 4.02 | 47.80 | hold_end=23.247>6.0; max_pitch=47.798>30.0 |
| 5 | 0.81 | 13.3 | 0.09 | 0.09 | 0.07 | 0.00028 | 8 | 10.29 | 21.13 | 16.13 | 57.37 | hold_end=21.128>6.0; max_pitch=57.368>30.0 |
| 59 | 0.75 | 15.5 | 0.14 | 0.09 | 0.06 | 0.00090 | 3 | 9.71 | 20.49 | 14.65 | 58.16 | hold_end=20.493>6.0; max_pitch=58.155>30.0 |
| 83 | 0.82 | 15.5 | 0.13 | 0.09 | 0.09 | 0.00093 | 6 | 9.99 | 19.21 | 8.02 | 53.15 | hold_end=19.212>6.0; max_pitch=53.146>30.0 |

## plant 参数/故障扫描（18 组）

通过 15/18，失败 3 组。

| 场景 | 结果 | alt [m] | hold_end [m] | max_roll [°] | max_pitch [°] | f3/f4 | 失败原因 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| nominal | PASS | 9.86 | 0.87 | 1.26 | 5.77 | 0/0 | — |
| mass_low | PASS | 9.97 | 1.29 | 5.01 | 27.86 | 0/0 | — |
| mass_high | PASS | 9.66 | 0.88 | 1.13 | 2.02 | 0/0 | — |
| thrust_low | PASS | 9.85 | 0.88 | 1.09 | 2.95 | 0/0 | — |
| thrust_high | PASS | 9.88 | 1.09 | 1.54 | 23.40 | 0/0 | — |
| lever_low | FAIL | 9.63 | 5.52 | 1.43 | 45.03 | 0/1 | max_pitch=45.031>30.0 |
| lever_high | PASS | 9.86 | 0.87 | 1.23 | 3.45 | 0/0 | — |
| iy_low | PASS | 9.86 | 0.87 | 1.26 | 3.31 | 0/0 | — |
| iy_high | FAIL | 9.86 | 4.59 | 5.38 | 35.39 | 0/1 | max_pitch=35.391>30.0 |
| iz_low | PASS | 9.86 | 0.87 | 1.26 | 5.76 | 0/0 | — |
| iz_high | PASS | 9.86 | 0.87 | 1.26 | 5.77 | 0/0 | — |
| yawk_low | PASS | 9.86 | 0.87 | 1.26 | 5.76 | 0/0 | — |
| yawk_high | PASS | 9.86 | 0.87 | 1.26 | 5.76 | 0/0 | — |
| combined_low | FAIL | 10.17 | 2.28 | 34.75 | 35.66 | 0/1 | max_roll=34.751>30.0; max_pitch=35.657>30.0 |
| combined_high | PASS | 9.71 | 0.88 | 1.31 | 5.66 | 0/0 | — |
| motor_loss_30 | PASS | 9.85 | 0.87 | 1.26 | 5.77 | 0/0 | — |
| both_loss_60 | DETECTED | -0.00 | 14.10 | 1.26 | 5.77 | 1/0 | — |
| servo_jam | DETECTED | -0.00 | 137.97 | 179.98 | 5.77 | 1/1 | — |

## 基线复现对照（历史文档 VER-002 / CTL-003 vs 本次）

（下表第一行是历史 3×5 口径，仅用于对照文档；本次验收口径见上方矩阵。）

| 项目 | 历史文档 | 本次结果 | 差异 |
| --- | --- | --- | --- |
| 验收（历史 3 profiles×5 seeds） | 15/15 | **14/15** | mag_nominal seed1 `max_pitch=22.108° > 20°` 失败 |
| 验收（本套件 8 profiles×5 seeds = 40 场景） | — | **39/40** | 仅 mag_nominal seed1 失败，与 3×5 基线同源 |
| 验收（realistic 8×5 = 40 场景） | — | **40/40** | 全过 |
| 随机 MC 100 例（current） | 60/100 | **56/100** | hold P50/P95 1.26/64.93 → 1.27/27.59 m；roll P95 23.9 → 12.25°；pitch P95 55.8 → 58.13° |
| sweep 18 组 | 15/18 | **15/18** | 一致（失败：lever_low、iy_high、combined_low） |

基线原始证据（3×5 验收、100 例 MC、18 组 sweep、8 profile 单 seed 语义）见 `baseline/`：`acceptance_3x5.log`、`mc100.json/.csv`、`sweep18.json`、`profile_semantics_1seed.log`。

差异归因（待查，未改阈值凑绿）：HEAD 相对 VER-002 记录时已接入生产 Madgwick AHRS 与多处导航/传感器链改动；失败集中在俯仰超调/姿态边界，与真实导航切换、磁航向融合链相关。`mag_nominal seed1` 的 pitch 超调与 MC 的 pitch P95 抬升同源，需在 plant/固件定稿后复核。

## 8 个 profile 在共轴二进制中的语义

| profile | 语义 | 共轴适用性 |
| --- | --- | --- |
| mag_clean / mag_nominal | 磁航向融合 + 位置保持 | 适用（主验收） |
| mag_disturbed | 磁干扰门控 + 高度/姿态有界（不判位置） | 适用，共用导航/磁门控层 |
| nomag_degraded | 无磁主动降级 AUTO_POSITION→AUTO_ALTITUDE | 适用，要求 `pos_degraded=1` |
| imu_stuck / imu_nan / imu_drop | IMU 故障/丢包检测隔离 | 适用，共用导航故障检测层（`imu_detected=1`） |
| gnss_spoof | GNSS 斜坡欺骗 + 一致性拒绝 | 适用，共用导航一致性监测（`gnss_crej>=1`） |

未纳入默认矩阵：`mag_clean_flow`（光流仅纵列诊断）、`mag_gate`（与 mag_disturbed 门控场景重叠）、无磁 clean/nominal/degraded（观测性诊断，不作验收）。

## 遗留风险

1. 验收失败 1 例（current 39/40；realistic 40/40，同一 `mag_nominal seed1` 属 near-threshold 翻转，见上）与 MC current 56/100：失败集中在俯仰裕度，需台架标定后重评。
2. plant 质量/惯量/力臂/差速力矩为 L1 假设，无外部实飞日志逐波形回放；本证据不能外推实机。
3. `--coax-realistic` 预设（电机 τ=0.28 s、舵机 300°/s+量化、地面 μ=0.5）已由 lead 于 2026-10-30 冻结，本次已两套对照；预设值仍是 L1 假设，需台架/实飞数据标定。
4. 本套件默认退出码为 1（MC/sweep 未全过），是真实门禁，不是脚本错误。

