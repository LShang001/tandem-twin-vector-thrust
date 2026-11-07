# 共轴 VTVL 验证证据（coax-2026-11-08）

- 生成时间：2026-11-08 04:04:56
- 仓库：`TandemVec_FCS` @ `688bc0a`（工作区未提交文件 2 个）
- 二进制：`U:/suite1/fw_coax.exe`（构建时间 2026-11-08 03:59:34）
- `MadgwickAHRS.cpp` 构建补丁：已包含（lead 在冻结前补入，修复 HEAD 链接失败）
- 矩阵：profiles=`mag_clean,mag_nominal,mag_disturbed,nomag_degraded,imu_stuck,imu_nan,imu_drop,gnss_spoof`；seeds=`1,2,3,4,5`；seconds=40；双套（--coax-realistic）=否
- 验收口径：8 profiles × 5 seeds，current 40 场景（不使用旧文档的 3×5=15 口径）。
- 口径：真实固件（`BFS_COAXIAL_VTVL`）+ 近似 6DOF plant，不是台架/实机证据；plant 质量/惯量/力臂/差速力矩为 L1 假设。

## 阶段结果与退出码

| 阶段 | 退出码 | 判定 | 证据文件 |
| --- | --- | --- | --- |
| 构建 | 0 | PASS | `build.log` |
| 验收 8 profiles（current） | 0 | PASS | `acceptance.log` |
| MC 100（current） | 1 | FAIL | `mc_current.json` |
| MC 100（realistic） | — | 未运行（未指定 --realistic） | `mc_realistic.json`（未生成） |
| sweep 18 组（current） | 1 | FAIL | `sweep.json` |

**套件退出码：1**（0=全部通过；1=任一阶段失败。注意：历史基线 MC/sweep 未达 100%，默认运行即返回 1，是真实门禁。）

## 验收矩阵（8 profiles，5 seeds）

| 场景 | current 通过 | realistic 通过 | hold_end 范围 [m] | max_roll 范围 [°] | 说明 |
| --- | --- | --- | --- | --- | --- |
| mag_clean | 5/5 | — | 0.28–0.89 | 0.75–1.12 | — |
| mag_nominal | 5/5 | — | 0.61–1.44 | 7.03–9.39 | — |
| mag_disturbed | 5/5 | — | 5.27–6.92（不判） | 11.48–14.25 | 位置保持不判（skip_hold_end） |
| nomag_degraded | 5/5 | — | 41.82–44.88（不判） | 0.33–0.43 | 位置保持不判（skip_hold_end） |
| imu_stuck | 5/5 | — | — | — | 仅判故障检测（skip_motion） |
| imu_nan | 5/5 | — | — | — | 仅判故障检测（skip_motion） |
| imu_drop | 5/5 | — | — | — | 仅判故障检测（skip_motion） |
| gnss_spoof | 5/5 | — | — | — | 仅判故障检测（skip_motion） |
| **合计** | **40/40** | — | — | — | — |

## 随机参数 MC（100 例）

| 组 | 通过数 | 通过率 | hold P50/P95 [m] | roll P50/P95 [°] | pitch P50/P95 [°] |
| --- | --- | --- | --- | --- | --- |
| current（默认 plant） | 52/100 | 52% | 1.47/28.67 | 1.26/8.50 | 29.35/58.38 |

### MC 最差 8 例（current（默认 plant））

| 案例 | mass | T | L | Iy | Iz | yawk | seed | alt | hold_end | max_roll | max_pitch | 原因 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 73 | 0.75 | 15.8 | 0.10 | 0.10 | 0.11 | 0.00064 | 5 | 22.77 | 53.99 | 6.62 | 84.54 | final_alt=22.774 越界[8.0,11.0]; hold_end=53.986>6.0; max_pitch=84.542>30.0 |
| 46 | 0.88 | 13.6 | 0.09 | 0.10 | 0.08 | 0.00044 | 1 | 9.73 | 47.97 | 7.09 | 54.57 | hold_end=47.968>6.0; max_pitch=54.574>30.0 |
| 83 | 0.82 | 15.5 | 0.13 | 0.09 | 0.09 | 0.00093 | 6 | 9.57 | 44.63 | 8.48 | 54.56 | hold_end=44.634>6.0; max_pitch=54.557>30.0 |
| 5 | 0.81 | 13.3 | 0.09 | 0.09 | 0.07 | 0.00028 | 8 | 10.23 | 36.25 | 13.42 | 58.67 | hold_end=36.248>6.0; max_pitch=58.668>30.0 |
| 14 | 0.73 | 14.9 | 0.11 | 0.09 | 0.10 | 0.00088 | 1 | 10.55 | 28.66 | 6.27 | 66.71 | hold_end=28.660>6.0; max_pitch=66.706>30.0 |
| 89 | 0.87 | 15.8 | 0.10 | 0.08 | 0.06 | 0.00114 | 4 | 9.96 | 28.82 | 9.17 | 52.13 | hold_end=28.825>6.0; max_pitch=52.128>30.0 |
| 37 | 0.75 | 13.0 | 0.10 | 0.06 | 0.06 | 0.00061 | 7 | 10.06 | 26.46 | 1.27 | 49.95 | hold_end=26.463>6.0; max_pitch=49.950>30.0 |
| 95 | 0.72 | 13.3 | 0.11 | 0.05 | 0.08 | 0.00093 | 5 | 9.78 | 19.98 | 1.09 | 45.56 | hold_end=19.975>6.0; max_pitch=45.560>30.0 |

## plant 参数/故障扫描（18 组）

通过 15/18，失败 3 组。

| 场景 | 结果 | alt [m] | hold_end [m] | max_roll [°] | max_pitch [°] | f3/f4 | 失败原因 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| nominal | PASS | 9.88 | 0.99 | 1.18 | 23.36 | 0/0 | — |
| mass_low | PASS | 10.02 | 1.10 | 1.45 | 28.01 | 0/0 | — |
| mass_high | PASS | 9.65 | 0.88 | 1.16 | 2.54 | 0/0 | — |
| thrust_low | PASS | 9.80 | 0.92 | 1.18 | 20.81 | 0/0 | — |
| thrust_high | PASS | 9.87 | 1.06 | 1.25 | 24.39 | 0/0 | — |
| lever_low | FAIL | 9.86 | 4.95 | 1.07 | 46.83 | 0/1 | max_pitch=46.834>30.0 |
| lever_high | PASS | 9.86 | 0.88 | 1.39 | 3.06 | 0/0 | — |
| iy_low | PASS | 9.85 | 0.88 | 1.27 | 2.36 | 0/0 | — |
| iy_high | FAIL | 9.94 | 2.45 | 6.50 | 37.43 | 0/0 | max_pitch=37.429>30.0 |
| iz_low | PASS | 9.88 | 1.00 | 1.19 | 23.36 | 0/0 | — |
| iz_high | PASS | 9.88 | 0.99 | 1.19 | 23.37 | 0/0 | — |
| yawk_low | PASS | 9.88 | 0.99 | 1.18 | 23.36 | 0/0 | — |
| yawk_high | PASS | 9.88 | 1.00 | 1.18 | 23.36 | 0/0 | — |
| combined_low | FAIL | 10.04 | 2.58 | 45.41 | 36.54 | 0/1 | max_roll=45.412>30.0; max_pitch=36.539>30.0 |
| combined_high | PASS | 9.70 | 0.98 | 1.29 | 22.45 | 0/0 | — |
| motor_loss_30 | PASS | 9.87 | 0.90 | 1.18 | 23.11 | 0/0 | — |
| both_loss_60 | DETECTED | -0.00 | 4.29 | 1.18 | 23.11 | 1/0 | — |
| servo_jam | DETECTED | -0.00 | 361.19 | 179.97 | 52.19 | 1/1 | — |

## 基线复现对照（历史文档 VER-002 / CTL-003 vs 本次）

（下表第一行是历史 3×5 口径，仅用于对照文档；本次验收口径见上方矩阵。）

| 项目 | 历史文档 | 本次结果 | 差异 |
| --- | --- | --- | --- |
| 验收（历史 3 profiles×5 seeds） | 15/15 | **14/15** | mag_nominal seed1 `max_pitch=22.108° > 20°` 失败 |
| 验收（本套件 8 profiles×5 seeds = 40 场景） | — | **40/40** | 全过 |
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

1. 验收失败 0 例（current 40/40；realistic 0/0）与 MC current 52/100：失败集中在俯仰裕度，需台架标定后重评。
2. plant 质量/惯量/力臂/差速力矩为 L1 假设，无外部实飞日志逐波形回放；本证据不能外推实机。
3. `--coax-realistic`（电机 τ=0.28 s、舵机 300°/s、地面 μ=0.5）已由 task-1 加入源但尚未冻结；本次默认未加该开关，待 lead 通知 `plant/firmware frozen` 后用 `--realistic` 重跑覆盖。
4. 本套件默认退出码为 1（MC/sweep 未全过），是真实门禁，不是脚本错误。

