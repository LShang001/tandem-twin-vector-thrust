# 真实固件代码宿主机 SIL（`test_host/fw_sil`）

> 目标：**不是按源码重写控制律**，而是直接编译/链接真实固件源码
> `src/flight_control.cpp` 与 `src/state_data.cpp`，只替换 STM32/Arduino
> 外设、传感器和执行器反馈，构成可跑的虚拟闭环。

## 1. 运行

```bash
cd TandemVec_FCS
BIN_DIR=/c/Users/12631/AppData/Local/Temp/tv_fcs_fw_sil \
  bash test_host/fw_sil/build_and_run.sh          # 定高 10 m 闭环
BIN_DIR=/c/Users/12631/AppData/Local/Temp/tv_fcs_fw_sil \
  bash test_host/fw_sil/build_and_run.sh --manual # 手动 roll/pitch 阶跃
```

- 编译**真实源码**：`src/flight_control.cpp`、`src/state_data.cpp`；
- `test_host/fw_sil/fw_sil_main.cpp`：刚体 plant + 传感器注入 + 执行器解算；
- `test_host/fw_sil/fw_sil_stubs.cpp`：硬件对象/外设/MAVLink 等链接桩；
- `test_host/fw_sil/stub/`：Arduino/SPI/Wire/ubx 等宿主机头桩。

## 2. 已验证结果（2026-10-22）

### 定高闭环

真实固件 `runGNCExecutive()` 在 200 Hz 下：

| 时间 | 高度 | 电机 | 模式 |
|---|---|---|---|
| 2 s | -8.74 m（开始定高指令） | 56.7% | AUTO_ALTITUDE |
| 5 s | -10.30 m（先下沉） | 50.8% | AUTO_ALTITUDE |
| 20 s | 4.71 m | 49.9% | AUTO_ALTITUDE |
| 25 s | 9.62 m | 47.0% | AUTO_ALTITUDE |
| 30 s | 9.98 m | 49.9% | AUTO_ALTITUDE |
| 39 s | **10.00 m** | 49.9% | AUTO_ALTITUDE |

结论：真实固件定高环能把 plant 从 0 拉到 10 m 并稳定在 10.00 m；
`ch1/ch2` 舵机保持 50%，`ch3/ch4` 约 49.9%。

> 注：2–5 s 的负高度是 plant 解锁后先下沉、定高环随后拉回；
> 不是最终稳态；最终稳定在 10.00 m。

### 手动姿态阶跃

真实固件 `ATTITUDE_MODE`：

- 10–15 s 打 roll 1600：
  - plant roll 从 0 到 **5.86°**；
- 20–25 s 打 pitch 1600：
  - plant pitch 从 0 到 **-5.85°**。

说明真实 `execute_attitude_controller` / `mix_and_output_commands` /
伺服逆解链路在宿主机上跑通。

## 3. 这不是什么

- 不是 `pio run` 的固件二进制；
- 不是 QEMU/Renode 的全芯片模拟；
- 不是实机验证；
- 传感器是注入的（无 EKF/噪声/延迟）；执行器只做一阶电机滞后 + 理想舵机；
- 硬件对象构造/外设写是桩，`state_data.cpp` 的全局变量和真实控制律是实际代码。

## 3b. 环境/传感器 SIL（2026-10-22 追加）

`fw_sil_env_main.cpp` 在真实固件控制外，增加了：

- 6DOF 刚体（姿态 + NED 速度/位置，推力/力矩来自真实 `computeWrench`）；
- 电机一阶滞后 `τm=0.28 s`、舵机一阶 `τ=0.05 s`；
- IMU：陀螺零偏 `[0.003,-0.002,0.004] rad/s` + 白噪声，加计零偏/噪声；
- GNSS 10 Hz：位置 0.8 m、高度 1.2 m、速度 0.1–0.15 m/s 噪声；20–25 s 失锁；
- 气压计 25 Hz：0.25 m 噪声 + 0.02 m/s 漂移；
- 磁航向（真值+噪声）；
- 风扰：20–25 s 侧风 3 m/s；
- 地面约束：NED down=0 时位置/下沉速度钳位。

运行：

```bash
BIN_DIR=/c/Users/12631/AppData/Local/Temp/tv_fcs_fw_sil   bash test_host/fw_sil/build_and_run_env.sh --seed 1
```

结果（seed1/2）：

- 定高仍能拉到约 9.2 m（有传感器噪声/风扰时，目标 10 m）；
- 20–25 s GNSS 失锁 + 3 m/s 侧风后，水平位置漂到 180 m 量级；
- 这不是控制律失效，而是**当前 SIL 没有把原始 IMU/GNSS/baro 送进真实 EKF**，
  `relative_*` 直接由传感器注入，失锁后无水平观测。

### 故障注入

```bash
# F3 前发推力损失 30%（8 s 后）
bash test_host/fw_sil/build_and_run_env.sh --fault 3 --seed 1

# F4 上摆卡死 +8°（8 s 后）
bash test_host/fw_sil/build_and_run_env.sh --fault 4 --seed 1
```

seed1 结果：F3 在 38 s 已 roll -80°/pitch -29°、位置 1.3 km；
F4 出现 NaN/姿态 90° 崩。**这是故障注入下的真实控制输出，不是完整 EKF 验证；
F4 目前是 plant/控制发散，不代表实机结论。**

### `--flow`：光流速度辅助（2026-10-22）

```bash
bash test_host/fw_sil/build_and_run_env.sh --seed 1 --ekf --flow
```

- 光流 = 机体系水平速度 ×0.95 尺度 + 0.12 m/s 噪声，10 Hz；
- 用当前 EKF 姿态转 NED，走真实 `MeasurementUpdateAidingVelocityDetailed`；
- seed1（含舵机 300°/s + PWM 量化）：
  - 无光流：**151 m**；
  - 光流 `--flow-delay 0`：**139 m**；
  - 光流 `--flow-delay 1`（0.1 s）：**297 m**；
- 0.1 s 延迟比无光流更差；说明当前延迟/尺度/姿态链路仍会放大水平误差，
  不能只看“光流开没开”。
- seed2/3：最终高度 9.24/8.84 m，roll 最大 6.8°/8.3°；
- 仍不是实机光流（无尺度/延迟/失效/机体坐标到 NED 的完整链路）。

### 磁航向 `--mag`：默认关闭

`--mag` 会调用真实 `MeasurementUpdateYaw`（10 Hz，噪声 0.02 rad）。
3 seed 矩阵中，`--ekf` 本身稳定（9.17–9.23 m），但 `--mag` 在某些 seed
会 NaN/发散；因此**默认关闭**，只作为实验路径。结论：理想磁航向也不能
替代水平观测。

### 矩阵脚本

```bash
BIN_DIR=/c/Users/12631/AppData/Local/Temp/tv_fcs_fw_sil   bash test_host/fw_sil/run_matrix.sh
```

覆盖 direct/EKF/flow0/flow1/F3/F4 × seed1–3，用于回归。

### `--ekf`：真实 EKF 闭环（已验证高度，水平仍受限）

`fw_sil_env_main.cpp` 把 **真实 `lib/navigation-main/src/ekf_15_state.h`** 编译进来：

- IMU 传播 + GNSS 10 Hz + 气压 25 Hz；磁航向用 `--mag` 单独开启（默认关闭，见下）；
- EKF 姿态/位置/速度直接驱动真实 `flight_control.cpp`；
- **已验证**：seed1 在 39 s 时 plant 高度 9.16 m、EKF 高度 10.00 m，
  电机约 40–50%，说明定高链闭环；
- 但 20–25 s GNSS 失锁 + 3 m/s 侧风后，水平位置仍漂到 **309 m**：
  当前没有光流/速度辅助，EKF 失锁后水平不可观。

因此 `--ekf` 可用于**定高闭环验证**，不能用于“失锁水平保持”结论。
默认（不带 `--ekf`）仍是直接传感器注入，已证 9.2 m 高度和 264 m 失锁漂移。

## 4. 下一步

1. 把 `hifi`/EKF 作为传感器源接入，替换当前理想注入；
2. 加 PWM/舵机速率限幅；
3. 把 F3/F4 故障注入到 plant/执行器；
4. 保留此 SIL 作为真实固件行为的回归基线。


## 5. SIL 验收基线（2026-10-22）

### 构建

```bash
BIN_DIR=/c/Users/12631/AppData/Local/Temp/tv_fcs_fw_sil   bash test_host/fw_sil/build_and_run_env.sh --seed 1 --ekf
```

### 自动验收门（3 seeds）

```bash
BIN_DIR=/c/Users/12631/AppData/Local/Temp/tv_fcs_fw_sil   bash test_host/fw_sil/run_acceptance.sh
```

判据：真实固件 + 真实 EKF，seed1–3 最终高度必须在 **8–11 m**；
当前实测 **9.17 / 9.23 / 9.23 m，通过**。

### 已验证

| 项 | 结果 |
|---|---|
| 真实 `flight_control.cpp`/`state_data.cpp` | 直接链接 |
| 定高（EKF，seed1–3） | plant 9.17–9.23 m，EKF 9.93–10.04 m |
| 定高（直注，seed1–3） | plant 9.15–9.28 m |
| 无光流失锁+风 | 水平漂 151–287 m |
| 光流无延迟 | 115/268/457 m（不稳定） |
| 光流 0.1 s 延迟 | 更差，部分 seed NaN |
| F3/F4 | 仍 NaN/90°，无重构 |
| 磁航向 `--mag` | 实验，部分 seed NaN，默认关闭 |

**结论：真实固件 SIL 已可复现地验证“定高闭环”；水平失锁保持、光流鲁棒性、
F3/F4 故障重构仍未完成。**

## 6. 真实导航任务 SIL（`--real-nav`，2026-10-24）

旧 SIL 的 `--ekf` 只把传感器直接喂给 EKF；`--real-nav` 改为：

- **2 kHz IMU** 采样/聚合，真实 `navigation_task.cpp::handleNavigationSystem()` 在
  200 Hz 消费（含静止检测、双子样、GNSS 动态权重、epoch 时间映射、气压/光流、重捕获）；
- **UBX NAV-PVT/NAV-EOE 二进制帧**注入 Serial4，由真实 `lib/ublox-main` 解析；
- 虚拟 `millis()/micros()`（32 bit，与 STM32 对齐）；
- GNSS 延迟/噪声/失联/野值/spoof、气压滞后/漂移、光流噪声/延迟、RC 链路丢失、IMU 丢样/NaN/冻结；
- **磁力计生产链**：SIL 以 100 Hz 把站点参考场+噪声/干扰注入 `MagSensor`，导航任务按
  10 Hz 门控后走 `MeasurementUpdateMagVectorDetailed`；不再是绕过导航的 EKF 旁路；
- **IMU 健康监测**：非有限/无样本/冻结/饱和判定，Failed 时隔离静止辅助并保持最后有效输出；
- **GNSS 一致性**：1 s 窗口 IMU 速度增量/位置自洽/光流/气压交叉；默认只告警，
  `--gnss-consistency-reject` 才启用拒绝（见 §8）。

### 运行

```bash
# 构建并跑磁基线 mag_clean/mag_nominal/mag_disturbed × 3 seeds（40 s）
# 无磁 clean/nominal/degraded 是观测性诊断，不是位置保持验收基线（见 §6）
BIN_DIR=/tmp/tv_fcs_fw_sil_real bash test_host/fw_sil/run_real_acceptance.sh \
  --profiles mag_clean,mag_nominal,mag_disturbed

# 单例（例：nominal + 位置保持）
BIN_DIR=/tmp/tv_fcs_fw_sil_real bash test_host/fw_sil/build_and_run_real.sh \
  --real-nav --profile nominal --pos-hold --seconds 40 --seed 1
```

### 配置

| profile | 传感器/环境 | 用途 |
|---|---|---|
| clean | 低噪声、无失联/风、无振动 | 最小闭环基线 |
| nominal | 1 g/80 Hz 振动、20 ms 延迟、5 s 失联、1 m/s² 风、光流 | 常规退化验收 |
| degraded | 2 g/120 Hz、50 ms、2.5 m 噪声、5 s 失联、1.5 m/s² 风、2% 野值、光流、IMU 丢样 | 退化边界 |
| mag_clean / mag_nominal | 在 clean/nominal 上加 100 Hz 磁注入、0.3/0.6 µT 噪声 | 磁链路无干扰基线 |
| mag_disturbed | nominal + 12 µT NED 慢变干扰 | 磁门控拒绝/姿态有界；不验收位置保持 |
| nomag_degraded | clean 环境但不注入磁 | 验证 `HeadingAidingGate` 降级到定高、`pos_degraded=1` |
| `--rc-loss A:B` | 单独注入真实 `linkDownCallback` failsafe | 不能与传感器退化混在一起 |
| `imu_stuck` / `imu_nan` / `imu_drop` | 10–11 s 冻结/NaN/全丢样 | 只验收“检测/隔离”，不验收降落品质 |
| `gnss_spoof` | 15 s 起 0.5 m/s ramp + 一致性拒绝 | 只验收监测/拒绝路径，不保证位置保持 |

### 当前基线（L2 SIL，3 seeds/profile，2026-10-24 垂直推力重构后）

**无磁时位置保持已被安全门控降级，不再作为验收项。** `HeadingAidingGate`
只把“磁矢量最近成功融合 ≤2 s”当作绝对航向源；失效即把 AUTO_POSITION/GUIDED
强制降为 AUTO_ALTITUDE，并关闭 yaw heading hold（回中只给 ω_ref=0）。无磁
`nomag_degraded` ×3 seeds **3/3 PASS**：final_alt 9.17–9.29 m、max_roll=0、
`pos_degraded=1`。加门控前无磁 40 s 位置保持会在约 23 s 后姿态/航向发散；
该失败在重构前 HEAD 与重构后二进制上 1 Hz 轨迹逐行一致，不是垂直推力映射引入。
**无磁 profile 现在只验证“降级到定高且不追不可观航向”，不再验证位置保持。**

磁生产链 profile ×3 seeds **6/6 PASS**（`mag_clean` + `mag_nominal`）：

- `mag_clean`：final_alt 9.96–10.06 m，hold_end 0.71–1.10 m，pos_rms 0.48–0.89 m；
- `mag_nominal`：final_alt 9.15–9.24 m，hold_end 0.47–0.90 m，pos_rms 0.55–1.05 m。

`mag_disturbed`（nominal 的 GNSS 失联 + 12 µT 磁干扰）只验收“磁门控确实拒绝 +
高度/姿态有界”，不把位置保持当判据：3 seeds 通过，mag gate 179–251/392；
同时失去绝对航向与水平姿态辅助时，位置保持本身已不可保证。

生产传感器 + 故障路径验收矩阵（`mag_clean,mag_nominal,mag_disturbed,
nomag_degraded,imu_stuck,imu_nan,imu_drop,gnss_spoof` ×3 seeds）：
**24/24 PASS**。IMU 故障三类的 `imu_detected=1`、final_alt 9.16–9.29 m、
max_roll 0–5.94°；它们属于 `skip_motion` 路径，`hold_end` 不作为判据
（无磁时位置保持已被门控降级）。

无磁 `clean/nominal/degraded` 仅作诊断（不计验收）：clean 3/3 稳定定高
（hold=0、max_roll=0）；nominal/degraded 因位置保持被门控关闭，风致漂移
75.3/113.0 m，但 final_alt 分别 9.17–9.29/6.08–6.24 m、max_roll 0–8.8°，
未再出现加门控前的航向自旋。**这些 FAIL 是“主动放弃位置保持”的预期结果，
不能写成姿态发散或位置保持通过。**

```bash
# 磁基线（位置保持验收）
py -3.12 test_host/fw_sil/run_real_acceptance.py \
  --bin /tmp/tv_fcs_fw_sil_real/fw_real \
  --profiles mag_clean,mag_nominal,mag_disturbed --seeds 1,2,3 --seconds 40

# 无磁降级路径（定高 + pos_degraded）
py -3.12 test_host/fw_sil/run_real_acceptance.py \
  --bin /tmp/tv_fcs_fw_sil_real/fw_real \
  --profiles nomag_degraded --seeds 1,2,3 --seconds 40
```

RC 链路失效单独验证（`--profile nominal --rc-loss 30:31`）：真实 `linkDownCallback`
保持解锁、模式落 AUTO_ALTITUDE、摇杆归中；final_alt 8.18 m、hold_end 0.87 m，
无空中 disarm/坠落。该结果只证明 SIL 中 failsafe 策略路径，不是实机链路失效验证。

**边界**：传感器模型参数是 MODEL-DEFAULT，不是实机标定；这是 L2 SIL，不是 HIL，
不能证明真实气动/推进/传感器/执行器/调度行为。RC-loss 是独立故障路径，不能把
“命令 disarm 后坠落”写成传感器退化验收失败。

## 7. 故障注入（2026-10-24 首批）

| 注入 | 开关 | 当前行为（L2 SIL） |
|---|---|---|
| RC 链路丢失 | `--rc-loss A:B` | 真实 `linkDownCallback`：飞行中保持解锁+AUTO_ALTITUDE；nominal 1 s 丢失 final_alt 8.18 m、hold_end 0.87 m |
| IMU NaN | `--imu-fault nan --imu-fault-window A:B` | 健康监测立即 Failed；导航保持最后有效输出，`imu_detected=1`，但 1 s 缺失仍导致失稳 |
| IMU 冻结 | `--imu-fault stuck --imu-fault-window A:B` | 健康监测约 0.5 s 内 Failed/Frozen；隔离 ZUPT/Gravity/StaticGyro，不再把冻结样本积分；**无安全降落** |
| IMU 全丢样 | `--imu-dropout A:B` | 健康监测约 0.2 s 内 Failed/NoSamples；后续同冻结路径 |
| GNSS 野值 | `--gnss-outlier P`（profile degraded） | 显式 30 m/10 m/s 野值门 + NIS；SIL degraded 每 40 s 拒绝 5–7 帧 |
| GNSS 失联 | `--gnss-outage A:B` | 无 GNSS 时纯惯导；有光流可保持水平，无光流只能验收定高 |
| GNSS spoof | `--gnss-spoof ... --gnss-consistency-reject` | 一致性监测可在 ramp 后拒绝 GNSS（`gnss_crej>0`）；默认 warn-only；无外部基准时拒绝后仍可能漂移/坠落 |

**边界**：IMU 健康监测只提供“检测+隔离”，单 IMU 无冗余，不能写成故障恢复/安全降落；
GNSS 一致性默认只告警，0.02 m/s 级慢漂和“恒定偏置+一致速度”欺骗不在可检测范围。
RC loss 结果也不能外推到真实链路/接收机失效。

## 8. 磁/IMU/GNSS 健康链路（2026-10，L2）

### 磁生产链

- 标定输出已从“单位球”修成物理 µT；旧 Flash `scale=0` 标定会被 fail-closed 拒绝，
  需重新 `magcal fit/save`；
- 参考场来自 IGRF-14 2026.0 @ 28.2N/112.9E（`include/mag_reference.h`），NED、D 向下；
  导航层用 origin 与站点距离 >50 km 使参考场失效；
- 3 seeds × 40 s 结果：`mag_clean`/`mag_nominal` 约 380–392 fused、0–14 gate，
  `mag_disturbed` 141–206 fused/186–251 gate；三场景 final_alt 9.1–10.1 m。
  `mag_disturbed` 的拒绝期间不能用“仍然能飞”解释为磁融合成功；加入
  `HeadingAidingGate` 后它只验收门控拒绝和姿态有界，不再要求位置保持。

### IMU 健康

- 3 seeds × 40 s：`imu_stuck` / `imu_nan` / `imu_drop` 均在 10–11 s 故障窗内
  `imu_detected=1`；1 s 故障后样本恢复，健康状态回到 Ok，飞行保持定高（非故障恢复证明）；
- Failed 后导航不再积分冻结/缺失样本，并禁止 ZUPT/Gravity/StaticGyro；
- 没有冗余 IMU/独立姿态源，**不提供安全降落**；若故障持续/落地，验收只看检测与隔离，
  不能把 1 s 窗口内的定高保持写成“故障恢复”。

### GNSS 一致性

- 默认 `g_gnss_consistency_reject_enabled=false`，只写 `gnss_susp/gnss_crej/gnss_cres` 遥测；
  无磁 clean/nominal/degraded 现在会先经 `HeadingAidingGate` 降级为定高，不能再拿
  clean/nominal/degraded 的位置保持当验收；磁基线 `mag_clean/mag_nominal` 6/6 PASS（warn-only）；
- `--gnss-consistency-reject` 时 `gnss_spoof` profile（15 s 起 0.5 m/s ramp）出现
  `gnss_crej>0` 并跳过 GNSS；但无外部水平基准时拒绝后仍可能漂移，故该 profile
  `skip_motion`，只验收监测/拒绝路径；
- 0.02 m/s 慢漂/恒定偏置欺骗不承诺检测（单测 C6 明确锁定）。

```bash
# 磁与故障 profile（3 seeds，40 s）
py -3.12 test_host/fw_sil/run_real_acceptance.py \
  --bin /tmp/tv_fcs_fw_sil_real/fw_real \
  --profiles mag_clean,mag_nominal,mag_disturbed,nomag_degraded,imu_stuck,imu_nan,imu_drop,gnss_spoof \
  --seeds 1,2,3 --seconds 40
```

**证据边界**：以上全部为 L2 SIL；磁硬/软铁、电机磁干扰、真实 IMU 电气冻结、RF spoof、
HIL/实飞均未验证。
