# 共轴 VTVL 验证证据（coax-2026-10-30）

- 生成时间：2026-10-30 23:45:15
- 仓库：`TandemVec_FCS` @ `8ae8ba7`（工作区未提交文件 7 个）
- 二进制：`/c/Users/12631/AppData/Local/Temp/tv_fix_sil/fw_coax.exe`（构建时间 2026-10-30 23:44:01）
- `MadgwickAHRS.cpp` 构建补丁：已包含（lead 在冻结前补入，修复 HEAD 链接失败）
- 矩阵：profiles=`mag_clean,mag_nominal,mag_disturbed,nomag_degraded,imu_stuck,imu_nan,imu_drop,gnss_spoof`；seeds=`1,2,3,4,5`；seconds=40；双套（--coax-realistic）=否
- 验收口径：8 profiles × 5 seeds，current 40 场景（不使用旧文档的 3×5=15 口径）。
- 口径：真实固件（`BFS_COAXIAL_VTVL`）+ 近似 6DOF plant，不是台架/实机证据；plant 质量/惯量/力臂/差速力矩为 L1 假设。

## 阶段结果与退出码

| 阶段 | 退出码 | 判定 | 证据文件 |
| --- | --- | --- | --- |
| 构建 | 0 | PASS | `build.log` |
| 验收 8 profiles（current） | 1 | FAIL | `acceptance.log` |
| MC 100（current） | 1 | FAIL | `mc_current.json` |
| MC 100（realistic） | — | 未运行（未指定 --realistic） | `mc_realistic.json`（未生成） |
| sweep 18 组（current） | 1 | FAIL | `sweep.json` |

**套件退出码：1**（0=全部通过；1=任一阶段失败。注意：历史基线 MC/sweep 未达 100%，默认运行即返回 1，是真实门禁。）

## 验收矩阵（8 profiles，5 seeds）

| 场景 | current 通过 | realistic 通过 | hold_end 范围 [m] | max_roll 范围 [°] | 说明 |
| --- | --- | --- | --- | --- | --- |
| mag_clean | 5/5 | — | 0.27–0.88 | 1.20–1.71 | — |
| mag_nominal | 4/5 | — | 0.31–0.96 | 7.62–12.41 | max_pitch=22.069 > 20.0 |
| mag_disturbed | 5/5 | — | 0.41–4.09（不判） | 8.35–15.32 | 位置保持不判（skip_hold_end） |
| nomag_degraded | 5/5 | — | 121.90–135.18（不判） | 0.99–1.13 | 位置保持不判（skip_hold_end） |
| imu_stuck | 5/5 | — | — | — | 仅判故障检测（skip_motion） |
| imu_nan | 5/5 | — | — | — | 仅判故障检测（skip_motion） |
| imu_drop | 5/5 | — | — | — | 仅判故障检测（skip_motion） |
| gnss_spoof | 5/5 | — | — | — | 仅判故障检测（skip_motion） |
| **合计** | **39/40** | — | — | — | — |

## 随机参数 MC（100 例）

| 组 | 通过数 | 通过率 | hold P50/P95 [m] | roll P50/P95 [°] | pitch P50/P95 [°] |
| --- | --- | --- | --- | --- | --- |
| current（默认 plant） | 56/100 | 56% | 1.38/27.85 | 1.67/13.43 | 27.96/56.39 |

### MC 最差 8 例（current（默认 plant））

| 案例 | mass | T | L | Iy | Iz | yawk | seed | alt | hold_end | max_roll | max_pitch | 原因 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 73 | 0.75 | 15.8 | 0.10 | 0.10 | 0.11 | 0.00064 | 5 | 10.44 | 173.27 | 18.79 | 71.57 | hold_end=173.270>6.0; max_pitch=71.567>30.0 |
| 14 | 0.73 | 14.9 | 0.11 | 0.09 | 0.10 | 0.00088 | 1 | 10.39 | 91.67 | 22.94 | 64.75 | hold_end=91.667>6.0; max_pitch=64.747>30.0 |
| 23 | 0.77 | 15.1 | 0.08 | 0.07 | 0.12 | 0.00050 | 6 | 9.79 | 41.03 | 3.59 | 67.73 | hold_end=41.026>6.0; max_pitch=67.727>30.0 |
| 95 | 0.72 | 13.3 | 0.11 | 0.05 | 0.08 | 0.00093 | 5 | 9.78 | 37.90 | 4.99 | 42.16 | hold_end=37.899>6.0; max_pitch=42.156>30.0 |
| 5 | 0.81 | 13.3 | 0.09 | 0.09 | 0.07 | 0.00028 | 8 | 10.30 | 28.85 | 3.05 | 56.34 | hold_end=28.847>6.0; max_pitch=56.336>30.0 |
| 83 | 0.82 | 15.5 | 0.13 | 0.09 | 0.09 | 0.00093 | 6 | 9.82 | 27.14 | 8.30 | 52.72 | hold_end=27.135>6.0; max_pitch=52.718>30.0 |
| 54 | 0.86 | 15.3 | 0.11 | 0.08 | 0.09 | 0.00118 | 5 | 9.77 | 27.80 | 1.25 | 48.59 | hold_end=27.801>6.0; max_pitch=48.593>30.0 |
| 88 | 0.87 | 15.5 | 0.09 | 0.06 | 0.11 | 0.00092 | 5 | 9.80 | 27.49 | 1.23 | 49.25 | hold_end=27.490>6.0; max_pitch=49.246>30.0 |

## plant 参数/故障扫描（18 组）

通过 15/18，失败 3 组。

| 场景 | 结果 | alt [m] | hold_end [m] | max_roll [°] | max_pitch [°] | f3/f4 | 失败原因 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| nominal | PASS | 9.86 | 0.88 | 1.29 | 5.75 | 0/0 | — |
| mass_low | PASS | 9.95 | 1.32 | 5.45 | 27.74 | 0/0 | — |
| mass_high | PASS | 9.66 | 0.88 | 1.13 | 2.08 | 0/0 | — |
| thrust_low | PASS | 9.85 | 0.88 | 1.09 | 3.00 | 0/0 | — |
| thrust_high | PASS | 9.89 | 1.10 | 1.58 | 23.27 | 0/0 | — |
| lever_low | FAIL | 9.67 | 5.30 | 1.45 | 45.04 | 0/1 | max_pitch=45.039>30.0 |
| lever_high | PASS | 9.86 | 0.87 | 1.23 | 3.56 | 0/0 | — |
| iy_low | PASS | 9.86 | 0.87 | 1.28 | 3.41 | 0/0 | — |
| iy_high | FAIL | 9.83 | 5.04 | 5.52 | 35.37 | 0/1 | max_pitch=35.369>30.0 |
| iz_low | PASS | 9.86 | 0.88 | 1.29 | 5.77 | 0/0 | — |
| iz_high | PASS | 9.86 | 0.88 | 1.29 | 5.77 | 0/0 | — |
| yawk_low | PASS | 9.86 | 0.88 | 1.29 | 5.76 | 0/0 | — |
| yawk_high | PASS | 9.86 | 0.88 | 1.29 | 5.77 | 0/0 | — |
| combined_low | FAIL | 9.97 | 2.01 | 44.42 | 36.96 | 0/1 | max_roll=44.425>30.0; max_pitch=36.960>30.0 |
| combined_high | PASS | 9.71 | 0.88 | 1.31 | 5.70 | 0/0 | — |
| motor_loss_30 | PASS | 9.85 | 0.87 | 1.29 | 5.75 | 0/0 | — |
| both_loss_60 | DETECTED | -0.00 | 14.06 | 1.29 | 5.75 | 1/0 | — |
| servo_jam | DETECTED | -0.00 | 137.94 | 180.00 | 5.75 | 1/1 | — |

## 基线复现对照（历史文档 VER-002 / CTL-003 vs 本次）

（下表第一行是历史 3×5 口径，仅用于对照文档；本次验收口径见上方矩阵。）

| 项目 | 历史文档 | 本次结果 | 差异 |
| --- | --- | --- | --- |
| 验收（历史 3 profiles×5 seeds） | 15/15 | **14/15** | mag_nominal seed1 `max_pitch=22.108° > 20°` 失败 |
| 验收（本套件 8 profiles×5 seeds = 40 场景） | — | **39/40** | 仅 mag_nominal seed1 失败，与 3×5 基线同源 |
| 随机 MC 100 例（current） | 60/100 | **56/100** | hold P50/P95 1.26/64.93 → 1.27/27.59 m；roll P95 23.9 → 12.25°；pitch P95 55.8 → 58.13° |
| sweep 18 组 | 15/18 | **15/18** | 一致（失败：lever_low、iy_high、combined_low） |

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

1. 验收失败 1 例（current 39/40；realistic 0/0）与 MC current 56/100：失败集中在俯仰裕度，需台架标定后重评。
2. plant 质量/惯量/力臂/差速力矩为 L1 假设，无外部实飞日志逐波形回放；本证据不能外推实机。
3. `--coax-realistic`（电机 τ=0.28 s、舵机 300°/s、地面 μ=0.5）已由 task-1 加入源但尚未冻结；本次默认未加该开关，待 lead 通知 `plant/firmware frozen` 后用 `--realistic` 重跑覆盖。
4. 本套件默认退出码为 1（MC/sweep 未全过），是真实门禁，不是脚本错误。

