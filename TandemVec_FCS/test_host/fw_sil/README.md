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

> 宿主 SIL 默认加 `-DEIGEN_DONT_VECTORIZE -DEIGEN_DONT_PARALLELIZE`：向量化路径会让
> 边际 seed 在混合核机器上按核出现双吸引子（core0 失败 / core2 通过），而固件
> Cortex-M7 无 NEON、本来就是标量 Eigen。可用 `EIGEN_SIL_FLAGS` 覆盖；增量构建用
> `obj/.build_flags` 检测选项变化并整目录重建，避免混合宏二进制。详见报告 §7.10。

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

### 协方差一致性遥测（2026-10-26）

默认 SIL/固件**不含** NEES 观测路径；单独构建：

```bash
BIN_DIR=/tmp/tv_fcs_nees EXTRA_CXXFLAGS=-DEKF_NAV_TELEMETRY \
  bin/bash test_host/fw_sil/build_incremental.sh --build-only
```

SIL 在 t>10 s、10 Hz 以 plant 真值计算 `NEES=e^T P^{-1}e/3`（位置/速度），JSON 增加
`nees_pos_{med,p95,max}`、`nees_vel_{med,p95,max}`、`nees_n`。真值基准在导航原点
变化时立即记录；零噪声对照（`--gnss-white --gnss-pos-std 0.01 --gnss-vel-std 0.01`）
应给出 NEES≈0，否则是参照系/采样口径错误。

- `--gnss-white`：关闭 SIL 的 20 s 相关 GNSS 误差分量（0.5/0.5/0.8 m），保留同一
  白噪声随机序列，用于同二进制隔离“有色误差导致协方差过度自信”。
- `--gnss-r-scale S`：只放大 UBX `h_acc/v_acc/s_acc`（EKF 的 R），不改实际噪声；
  S=1 为固件口径。R 放大改善 NEES 但降低位置保持跟随性（报告 §7.16.4），仅研究用。

结果：白噪声臂 NEES_pos 中位 1.05、默认有色臂 2.22（50 seeds，40 s，mag_clean +
flow）；零噪声 0.028。详见 `TandemVec-EKF-Report/无磁EKF收敛仿真-2026-10-25.md` §7.16。

### IMU 量程的复现构建（2026-10-27）

16 g / 32 g 过载包线研究此前靠**手改 `include/imu_range.h`** 产出二进制，产物不可复现、
也无法与 SIL 侧 `--imu-fs-g` 档对齐。现该文件把量程对改成编译期宏，一条命令即可重建：

```bash
# 8 g（生产默认，与历史逐位相同）
BIN_DIR=/tmp/tv_hf4 bin/bash test_host/fw_sil/build_incremental.sh --build-only
# 16 g
BIN_DIR=/tmp/tv_16g EXTRA_CXXFLAGS="-DBFS_ICM42688_ACCEL_FS_G=16.0f -DBFS_ICM42688_ACCEL_RANGE_MPS2=156.9f" \
  bin/bash test_host/fw_sil/build_incremental.sh --build-only
# 32 g
BIN_DIR=/tmp/tv_32g EXTRA_CXXFLAGS="-DBFS_ICM42688_ACCEL_FS_G=32.0f -DBFS_ICM42688_ACCEL_RANGE_MPS2=313.8f" \
  bin/bash test_host/fw_sil/build_incremental.sh --build-only
```

编译期守卫（实测 5 例，`-fsyntax-only`）：配对正确（8/78.45、16/156.9、32/313.8）通过；
**只给 FS 不给 m/s²** ⇒ `BFS_ICM42688_ACCEL_RANGE_MPS2` 未声明，编译直接失败；
**只改 m/s² 或两值不匹配**（32 g 配 78.45）⇒ `static_assert` 报"量程对不一致"。
容差 0.2%，覆盖 78.45/34.9066 这类历史取整值。
运行时必须同步 `--imu-fs-g` 与 `--imu-health-accel-range`，否则传感器按旧档削顶
（量程三处同步：传感器 FS、健康门限、EKF 合理值门限）。

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
| GNSS 垂直 spoof | `--gnss-spoof 0 --gnss-spoof-start 25 --gnss-spoof-z ±50` 或 `--gnss-spoof-z-ramp ±0.5` | 正值=报得更高；只改高度时须把水平 `--gnss-spoof` 设 0。默认配置下 ±50/100 m 台阶经重锚被接受，10/10 下降到地面或爬到 32 m（报告 §7.17） |
| 垂直重锚门控 | `--gnss-v-gate 0\|1`（需 `-DBFS_GNSS_VERTICAL_REANCHOR_GATE` 构建） | GNSS/气压高度差连续 3 帧 >25 m 时忽略 GNSS 垂直位置、禁止硬重锚，水平/速度仍融合；健康气压下 ±50/100 m 台阶 0/10 落地，无欺骗场景与关臂逐项相同。**气压快速漂移时会灌入错误气压，默认关闭** |
| 控制级追踪 | `--trace-csv PATH` | 200 Hz CSV：真值/EKF 姿态、陀螺、角速率指令、EKF 位置与保持目标、磁 kick/门控原因、飞行模式/降级/重锚、舵机与电机输出，以及真值/EKF 高度、目标高度、真值/控制输入的垂直速度、EKF 加计 z 零偏；仅测试设施，默认关闭。`vfk_vz` 列在 real-nav SIL 中未调度垂直 KF，恒为 0 |
| 落地/触地指标 | `SIL_REAL_JSON` | `land_t` 首次真值高度 <0.5 m、`land_hold` 触地水平位移、`land_vz_down` 触地垂直速度（NED 向下为正）、`max_desc_vz_down` 触地前最大下降速度、`max_roll_preland` 触地前最大滚转、`alt_at_outage` 失联起始真值高度；用于把飞行段与落地后无摩擦滑行分开，并避免把 `--alt` 目标高度误当实际失联高度 |
| EKF 气压垂速伪量测 | `--baro-vel-aid 0\|1` | 仅测试/研究：同一二进制内开关 `BFS_BARO_VERTICAL_VELOCITY_AID` 的 v_d 伪量测（5 Hz、σ=1.0 m/s、GNSS age>8 s）；JSON `baro_vel_attempts/fused/rejected` 计数。默认关闭；实机未验证 |
| IMU 饱和 Q 膨胀 | `--sat-q-inflate Q --sat-q-enter s --sat-q-release s`（需 `-DBFS_IMU_SATURATION_Q_INFLATE` 构建） | 加计削顶锁存期间把加计噪声标准差抬到 Q（默认 4.87 m/s²、enter 0.269 s、release 0.27 s），让 GNSS 主导速度/位置；10.1 g 位置 622.6→3.5 m、拒帧 20→0、重锚 1→0，8.4 g 姿态/航向代价约 +0.03°/+0.15°。默认宏关闭；实机未验证 |
| 健康门限量程 | `--imu-health-accel-range <m/s²>` | 研究设施：把 `ImuHealthConfig.accel_range_mps2` 改成与实际注入 FS 一致；16 g 实验应配 156.9，否则 8 g 门限会把未削顶误判为饱和并触发 Q（4.19 vs 4.35 m） |
| 导航在环 | `--missile-nav-loop` | 把 EKF 姿态/陀螺估计（减零偏）喂给弹体理想自驾，默认真值闭环；JSON 新增 `ms_final_n/e/d`，用于比较高过载下导航误差对弹道/稳定性的影响 |
| 2 kHz 短窗追踪 | `--trace-fast PATH --trace-fast-window A:B` | 记录窗口内 plant 角速率与 IMU 原始 gyro/accel（2 kHz），用于区分高频模态、混叠与控制器行为；仅测试设施，默认关闭 |
| 6DOF 积分子步 | `--plant-substeps N` | 诊断用：将 0.5 ms 的 plant 积分再细分 N 步（默认 1 保持旧行为），用于判断显式欧拉近 Nyquist 数值振荡 |
| 航向源模式 | `--heading-aid-mode 0\|1\|2` `--heading-align-deg X` | 仅测试用：0=磁连续（默认）；1=磁仅初始对准；2=无磁手动对准（机头真北角 X）。模式 1/2 允许 20 s 内的位置保持/heading hold（yaw 1σ≤20°），窗口从离地起算（地面等待不计时，落地复位），结束或超限降级定高；这是短时边界，不承诺长时定点 |
| 前飞段测试 | `--pos-leg N:E` | 仅测试用：进入 AUTO_POSITION 时把目标点叠加北/东偏移 [m]，用于“起飞→有倾斜前飞→转定点”时序；**trace 的 `hold_n/hold_e` 不含该偏移**，分析误差时须自行加上 N/E |
| 前飞→定点时序 | `--pos-hold-at T --fwd-stick A:B` | 仅测试用：T 秒前保持定高、A–B 秒压俯仰杆建立速度/倾斜，T 秒切 AUTO_POSITION；需配合 `--pos-leg` 验证无磁运动航向启动定点 |
| 风窗口 | `--wind-window A:B` | 仅测试用：覆盖默认 20–25 s 侧风时段，用于验证 20 s 无磁窗口内受风保持 |
| 地面静置校准 | `--calib-hold T` | 仅测试用：前 T 秒电机不出力、固件油门置 0，用于研究静止校准时长对 EKF 收敛的影响；配合 `--static-calib-s 0` 跳过门控 |
| 初始航向 | `--initial-yaw-deg D` | 仅测试用：plant 初始真值航向 D°，配合 `--heading-align-deg D` 验证起飞前手动对准 |
| 起飞时刻 | `--takeoff-at T` | 仅测试用：延迟起飞到 T 秒（默认 3 s）。★ 地面只约束 z、无摩擦，位置环会让机体积滑；T>10 s 的等待场景结果无效（SIL 台缺陷），窗口离地判据用门控单元测试 T9b/T9c 验证 |
| 增量构建 | `BIN_DIR=/tmp/inc bash test_host/fw_sil/build_incremental.sh` | 与 `build_and_run_real.sh` 同宏同源，按 TU 保留 `.o/.d`；首次全量，之后只重编改动文件（实测 1.4 s 无改动重链接） |
| Q 研究 | `--ekf-q-scale S` | 仅研究：缩放 EKF 加速度过程噪声；默认 1.0。闭环 SIL 已否决 0.05–0.1（风下 37–54 m 尾部） |
| 零偏限幅 | `--bias-z-max R` | 无磁离地时陀螺零偏 z 限幅 [rad/s]，默认 0.01；0=关闭（消融对比） |
| 增益分离扫描 | `--rate-gain-scale X` / `--att-gain-scale X` | 仅测试用：分别缩放速率内环/姿态外环 kp（覆盖 `--gain-scale` 的对应项）；用于区分高频振荡与位置/姿态慢响应 |

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

## 9 共轴双桨 VTVL 固件分支（2026-10-24）

`platformio.ini` 的 `CoaxialVTVL` env 定义 `BFS_COAXIAL_VTVL`；`runGNCExecutive`
仍复用输入/模式/定高/目标姿态层，但从姿态控制开始改走
`CoaxialVtvControl` + `mapCoaxialVtvOutputs`（±10° 修正、PWM 差速、45° 锁电机），
不再进入 TandemVec 的 BTRUE/差速开方。`CoaxialVtvControl` 的 PID 增益、
积分限幅、姿态目标/修正角两级滤波和 RATE_MODE 复位行为已按实飞共轴工程
逐项对齐（见 `CTL-003 §2.1`）。编译通过：

```bash
cd TandemVec_FCS
pio run -e CoaxialVTVL
```

SIL 使用同一套真实导航/传感器注入，只增加共轴近似 plant：

```bash
BIN_DIR=/tmp/tv_fcs_fw_sil_coax \
  bash test_host/fw_sil/run_coaxial_acceptance.sh
# 或单例
BIN_DIR=/tmp/tv_fcs_fw_sil_coax \
  bash test_host/fw_sil/build_and_run_coaxial.sh \
  --real-nav --profile mag_clean --pos-hold --seconds 40 --seed 1 --no-wind --mag
```

5 seeds 结果（L2，近似 plant）：

- `mag_clean`：final_alt 9.84–9.94 m、hold_end 0.29–0.88 m、max_roll 0.71–1.17°；
- `mag_nominal`：final_alt 9.04–9.12 m、hold_end 0.16–0.85 m、max_roll 8.59–14.23°；
- `nomag_degraded`：final_alt 9.84–9.92 m、`pos_degraded=1`，位置保持被门控关闭，
  hold 5.4–12.5 m 是风/残余倾斜漂移，max_roll 0.07–0.14°；
- 15/15 PASS。

**边界**：plant 的质量 0.84 kg、惯量 0.05/0.05/0.08、力臂 0.15 m、差速力矩
0.0005 N·m/norm 都是 L1 假设（外部工程只给了 0.84 kg/14 N，未给惯量/力臂）；
没有共轴台架、桨距/反扭标定、45° 锁电机故障注入或实机证据。该 SIL 证明
“固件分支能跑通、姿态/定高闭环有界”，不能证明真实共轴机飞行品质。

## 10 6DOF 导弹弹体（`--missile`，2026-10-27，L2）

`missile_plant.h` 提供 6DOF 刚体 + 线性气动 + 理想姿态自驾的导弹 plant；`--missile`
把 plant 真值接到现有 `--real-nav` 传感器注入链（IMU/UBX/气压/磁），**导航仍跑真实
`handleNavigationSystem()`**，跳过 `runGNCExecutive()`，导航结果不回灌弹道。

```bash
BIN_DIR=/tmp/tv_fcs_missile bash test_host/fw_sil/build_incremental.sh --build-only
/tmp/tv_fcs_missile/fw_real.exe --real-nav --missile --profile mag_clean \
  --seed 1 --seconds 45 --imu-grade icm --trace-csv /tmp/msl.csv
```

- `--imu-grade ideal|icm|low|adverse`：理想 / ICM 标称 / ×2 / ×5 零偏+噪声+尺度+失准。
- **生产传感器路径（默认）**：2 kHz 软件 LPF（传感器系陀螺 α=0.15、加计 x/y α=0.02、z α=0.05；轴映射后机体系 x=0.05、y=0.02、z=0.02）+ 真实 Madgwick（2 kHz，滤波后机体系→FLU/NWU）。A/B 开关：`--no-imu-lpf`（旧未滤波口径）、`--no-sil-ahrs`（旧 `backup_ahrs=0` 口径）、`--nav-raw-imu`（导航增量绕过 LPF，研究未采用；★ 该开关**只改导航增量源**，AHRS/控制仍用滤波值，
所以它不等价于"整机提带宽"，也不受固件任何门控保护——见报告 §4.24h 的冻结格代价）
  - **★ `--nav-raw-fault-fallback`（研究设施，必须与 `--nav-raw-imu` 同用）**：IMU 故障窗
    及其后 5 s 内把导航增量**退回滤波值**，用于检验"冻结格 +67.5% 代价来自未滤波台阶直接进
    导航积分"这一机理假设（报告 §4.24m）。**地位：机制验证中，不是已判定的交付方案**；
    单独给出（不带 `--nav-raw-imu`）完全惰性。自证据 = JSON 字段 `nrfb`（回退帧数，
    不带故障注入时应为 0）。
  - **`--nav-raw-fb-health`（与上条同用）**：把回退锁存键从"注入故障窗"换成
    **固件 IMU 健康状态 `Failed`**（导航侧镜像 `g_sil_nav_debug_imu_failed`）+ 5 s 后持，
    这是固件化时真正拿得到的触发源。用途：测误触发占空比（削顶/强振动会不会让它静默
    关掉高过载段需要的带宽收益）。判据 K1–K3 见报告 §4.24m；**结论未出，勿当已验证方案**。。
- **AHRS 策略**：方向门默认开；`BFS_AHRS_GNSS_FRESH_GATE`（GNSS 新鲜时不用 AHRS，opt-in）修 5 Hz/200 ms/1 g 振动；`BFS_DISABLE_AHRS_ATTITUDE_AID` 完全关 AHRS。
- 轨迹：50 m 起始、1 s 支撑、10 s 推力；最大 6.4 g、451 m/s、1987 m、27.8°/s。
- 指标：JSON 增加 `ms_vmax/ms_altmax/ms_gmax/ms_rate_max/ms_qbar_max/ms_impact_t` 与分轴姿态误差 `nav_roll/pitch/yaw_rmse,max`；`[ms]` 每秒打印弹体状态。
- 低/高零偏初始化：统一静止检测器 `|ω|<0.01 rad/s` 对 2×/5× 零偏档不成立；用
  `EXTRA_CXXFLAGS="-DBFS_PRELAUNCH_ALIGN_STABILITY_GATE"` 构建 opt-in 对准门。
- 矩阵：`TandemVec-EKF-Report/sim/run_missile_sil_mc.py`、作图 `plot_missile_sil.py` / `plot_missile_envelope.py` / `plot_missile_visuals.py` / `plot_missile_sensor_fidelity.py` / `plot_vib_isolator.py`；报告 `TandemVec-EKF-Report/导弹6DOF-导航SIL-2026-10-27.md`。
- 该 plant 无气动弹性/风/舵机动态，自驾是理想 PD；只用于导航算法 L2 评估，
  不是弹道/制导仿真，也不能外推实机。
- 振动：`--vib G`/`--vib-hz`；`--vib-start T` 用于先干净对准再起振（默认 0）。
  生产双路 1 g/6.4 g 姿态 6.95°、2 g/120 Hz/8.4 g 8.32°；fresh gate 1.94/7.97°；发射段 v2 门 20/20（1 g 7.01°/2 g 9.02°，fresh 1.98/8.73°）。
- **自动 GNSS 延时估计的应用侧门（同二进制 A/B）**：`--vib-veto 0|1`（`vib_high` 否决，默认随宏=0 关）、`--delay-deadband S`（应用死区）、`--delay-max-gap S`（拟合窗 GNSS 缺口门）、`--delay-max-resid MPS`（自报残差门，默认 150；`0`=关闭）、`--delay-grid-max-ms MS`（搜索网格上界，默认宏 150——上界以下的延时才可能被估出）、`--delay-cap-ms MS`（τ̂ 与编译期先验的最大偏差门限）；`-1` 一律表示"用编译期宏默认"。★ `--delay-grid-max-ms` 与 `--delay-cap-ms` **必须成对放宽**：只放宽网格，估出的 τ̂ 仍会被 CAP 门挡住（表现为 `ad_blk` 位 3 置位而 `ad_app` 不增）。诊断列 `ad_tau_ms/ad_sharp/ad_valid/ad_rej/ad_ng/ad_er/ad_app/ad_blk` 与 JSON `gnss_stale_peak_s`（失联注入自证据，**必须用它而不是末帧 `gnss_age`**）。历史开关 `--delay-disc-hold`（拟合窗断裂重锚保持）因六格 A/B 无可测收益已**连同枚举与接缝一并删除**，旧脚本再传它会由"未识别参数硬失败"拦下。
- **`vib_high` 通道选择**：`--vib-hf-source 0|1`（0=既有 0.5 Hz 中频 RMS，吃在软件 LPF **之后**，弹道上由确定性机动决定；1=`ins_vibration_hf.h` 的高频通道 `raw − LPF(raw)`，吃在 LPF **之前**，对应 80–120 Hz 机械振动）。编译期默认 `BFS_VIB_HIGH_SOURCE_HF=0`；trace 新增 `vib_hf_g` 列（无论选哪个通道都记录高频 RMS，便于直接看两个通道的可分性）。`--vib-hf-enter G` 运行时覆盖高频通道进入门限（默认宏 0.15 g，释放门限按 0.667 比例随之缩放），用于在同一次构建里扫"只挡强振动"的档位。
- **振动自适应 Q 边界**：`--gyro-q-cap RADPS` 在线改陀螺 Q 上界（默认 0.010=历史行为）。实测弹道上 `accel_rms_g` 由确定性机动主导（零振动 6.4→10.1 g 时 0.143→0.279 g），加计 Q 100% 卡在 0.25 端；该检测器对 80–120 Hz 振动灵敏度只有 10 Hz 的 8%~2%（输入在软件 LPF 之后）。
- **失联开关是 `--gnss-outage 起:止`**；`--loss F` 是推力保留比例（需 `--fault 3/5`），超出 `(0,1]` 或未识别参数现在都会**硬失败退出**（旧行为是静默忽略，曾产出"失联档与洁净档逐位相同"的假证据）。
- **物理减震座（SIL 研究）**：`--vib-iso-fn <Hz> --vib-iso-zeta <ζ>` 用单自由度传递率缩放振动；推荐 fn≈20–25 Hz（含 80 Hz 主谱）/30 Hz（仅 120 Hz）、ζ≈0.10；2 g/120/8.4 g 7.97°→0.66/0.64°（40 seeds），发射 2 g 8.73°→0.66°。模型只含幅值，无相位/转动传递率，装座前需台架。
- **飞行武装（2026-10-27 修正）**：导弹模式 t≥1 s 置 `unlocked=true/throttle=50%`，IMU 健康冻结检测才武装。旧 unarmed 的 5 Hz 5.79°/200 ms 7.72°/冻结 3.11 m 作废；armed 40 seeds 为 0.66/3.80°/35.2 m。
- **IMU 短时失效组合 fallback（opt-in `BFS_NAV_IMU_OUTAGE_FALLBACK`，默认关，上限 2 s）**：非有限→保持上一有效样本；有限但健康 Failed（冻结）→重力抵消比力+零角速率 coast；都保留 GNSS/磁/气压更新。armed 20 seeds：NaN 1 s 121.75→**3.15 m/0.58 m/s/0 拒绝/0 重锚**，冻结 1 s 35.24→**2.33 m/1.06 m/s/0/0**；2 s 12.67/65.5 m；33 s 长故障仍失导；10.1 g 饱和不受益。单独 hold-last/coast 宏保留作研究对照。
- **`--gnss-latency-ms` 口径**：该开关把数据与 UBX iTOW 一起后移，固件时间映射自洽，
  `gnss_age` 仍约 15 ms；因此它测的是“未建模接收机延迟/时间标签偏移”，不是 EKF
  回放缓冲。真延迟测试需保持 iTOW 为量测时刻（只延迟注入）或使用显式年龄接口。
- 包线扫描：`--ms-thrust N`、`--ms-mass kg`、`--ms-burn s`（默认 5000 N/120 kg/10 s →
  6.4 g/451 m/s）；8000 N → 8.41 g/696 m/s 仍可用（位置 3.94 m）；10000 N → 10.13 g/844 m/s
  在生产 ICM ±8 g 量程下触顶、导航失效（636 m）；`--imu-fs-g 16` 仅作研究对照。
  生产 `sensor_imu.cpp` 是 ±8 g，SIL 默认已对齐 8 g。

## 11 共轴 plant 可切保真度开关（2026-10-30，L1）

共轴 VTVL 的 L1 plant（`--real-nav` 下的 `Plant`/`computeCoaxialWrench`，以及
2 kHz 内环执行器动态）原本把电机 τ=0.28 s、舵机一阶 τ=0.05 s + 300 °/s、
0.1% PWM 量化（±15°/1000 计数 ≈ 0.03°/计数）**硬编码**，且地面无水平摩擦。
本轮把这些变成可切参数：**默认值等于原行为，默认开关下 15 场景输出逐位不变**
（详见 `sim-data/coax-plant-2026-10-30/default_equivalence.txt`）。

| 开关 | 物理含义 | 默认 | `--coax-realistic` |
| --- | --- | --- | --- |
| `--coax-motor-tau <s>` | 两电机推力一阶跟踪时间常数；`0`=理想瞬时 | 0.28（既有 L1 值） | 0.28 |
| `--coax-servo-rate <deg/s>` | 舵机角速率上限；`0`=无限速率 | 300（既有 L1 值） | 300 |
| `--coax-ground-mu <μ>` | 落地后水平库仑摩擦（μ·g 消减，过零置零不反冲） | 0（关闭） | 0.5 |
| `--coax-realistic` | 预设：τ=0.28 / 300 °/s / μ=0.5（argv 顺序生效，后续显式开关可覆盖） | 关闭 | — |
| `--coax-servojam roll\|pitch\|none` | 卡死对应舵机（锁存当时实际角并保持） | none | — |
| `--coax-servojam-at <s>` | 卡死时刻 | = `--fault-at` | — |
| `--coax-motor-loss1/2 <r>` | 电机 1/2 持久推力保留比例 [0,1]（区别于 `--fault 3/5` 的瞬时故障） | 1.0 | — |

- `--coax-realistic` 相对基线**真正新增的物理只有地面摩擦**：执行器两项的预设值
  就是原硬编码值。故 `--coax-motor-tau 0.28`/`--coax-servo-rate 300` 与默认逐位一致。
- **气动阻尼/风相对速度按用户指示不在本 L1 plant 范围内**（VTVL 低速段气动阻尼
  相对控制力矩很小）；现有 `--wind` 直接力语义保持不变。
- 默认开关验收仍为 **14 PASS + 1 FAIL**，失败项与改动前逐字相同
  （mag_nominal seed1 `max_pitch=22.108 > 20.0`）；`--coax-realistic` 15/15 PASS，
  但 mag_nominal seed1 的 22.108→19.915 是地面摩擦对起飞前地面段的扰动导致的
  混沌敏感性，**不是鲁棒性改善**，不应据此放宽门限或丢弃该 seed。
- 边界：地面接触仍是点质量近似（无法向反力/接触力矩/姿态支撑），舵机卡死是确定性
  离散事件（无卡滞前摩擦），`--coax-motor-loss1/2` 不含温升/电压动态；τ/速率/量化
  沿用既有 L1 假设，本轮未做频响/台架标定。正常定高下 `--coax-servo-rate` 通常
  不饱和（舵机一阶 τ=50 ms 主导），只在大指令误差的剧烈机动下起作用。
- 证据：`sim-data/coax-plant-2026-10-30/`（15 场景 JSON/原始 JSONL、验收日志、
  等价性对比、realistic 结果表与逐开关冒烟说明）。

### 11.1 分配器/失配激励与遥控激励表（2026-10-31，task-11）

为共轴「α+分配器」解耦改造的 A/B 与鲁棒性验证加的可配置激励。**默认值全部等于
改动前数值**（`×1.0f` 在 IEEE 下精确、不加舍入差），默认开关下 15 场景 stdout
与改动前逐字节相同（仅 `SIL_REAL_JSON` 末尾多 8 个追踪字段，既有字段与顺序不变）。

| 开关 | 物理含义 | 默认 | 范围 |
| --- | --- | --- | --- |
| `--coax-ix <IX>` | plant 绕机体 x 轴（滚转）惯量，独立于 `--coax-iy` | 0.05 kg·m²（既有） | (0,5] |
| `--coax-inertia-scale <F>` | 三轴惯量统一乘子（Ix/Iy/Iz × F），模拟载荷/惯量失配 | 1.0 | [0.25,4] |
| `--coax-thrust-scale <F>` | 推力效能失配：实际推力 = 指令 × F（只改 plant 力/力矩，不动固件） | 1.0 | [0.5,1.5] |
| `--coax-yaw-scale <F>` | 差速偏航力矩系数乘子（模拟 kQ/差速效率偏差） | 1.0 | [0.25,4] |
| `--coax-alloc-mode <0\|1>` | SIL 启动写 `kFlightCtrlParams.coax.alloc_mode`（0=legacy 1=α+分配器） | 0（不写、不打印） | {0,1} |
| `--rc-profile <CSV>` | 遥控激励表：50 Hz 节拍覆盖四根摇杆与 CH7/CH9 | 关闭（严格走原路径） | 见下 |

- 实际惯量 = `--coax-ix` × `--coax-inertia-scale`（三轴同乘）；JSON 同时记录 IX 与乘子，
  便于把「绝对惯量」与「相对失配」分开归因。
- `--coax-alloc-mode` 用编译期字段探测（`std::void_t` SFINAE）接线：固件字段
  `CoaxialCtrlParams::alloc_mode` 存在时直接写入（task-10 已落地则打印
  `[sil] 固件参数 coax.alloc_mode=1`）；字段不存在时只打印警告并照常运行，字段
  落地后自动接线、SIL 不需再改。默认 0 等于固件默认值，0 时不写也不打印，
  缺省 stdout 不受影响。
- **`--rc-profile` CSV**：首行表头固定
  `t,roll_us,pitch_us,yaw_us,throttle_us,mode_us,att_mode_us`；`t` 为 SIL 秒
  （非负、单调不减、≤1e6），其余列为 PWM 微秒；行数 ≤4096，允许空行。
  50 Hz 遥控节拍按行间线性插值取值，`t` 超过末行保持末行（静态时间基准、单调游标）。
  列范围：`mode_us`∈[1000,2000]，其余（含 `att_mode_us`）∈[988,2012]；
  表头不符/文件缺失/解析失败/非单调/越界/无数据行均 `rc=2`。
  语义：roll/pitch/yaw/throttle 覆盖原硬编码摇杆；`mode_us` 灌 CH7、`att_mode_us`
  灌 CH9；CH5 解锁（t≥0.5 s）、CH6=1800、CH8=1000 与链路丢失/failsafe 时序保持原逻辑。
  CH9 <1500 → `ATTITUDE_MODE`，≥1500 → `RATE_MODE`（`src/flight_control.cpp:208`）。
- SIL_REAL_JSON 末尾新增追踪字段：`coax_ix`、`coax_inertia_scale`、`coax_thrust_scale`、
  `coax_yaw_scale`、`coax_alloc_mode`、`rc_profile_rows`、`rc_profile_t_first`、
  `rc_profile_t_last`；既有字段逐项不变（默认 rows=0、t_first=t_last=0）。

证据（2026-10-31，`BIN_DIR=T:/sil_alloc`，干净基线同源码 HEAD）：
- 默认一致性：3 profiles×5 seeds 仍 **14 PASS + 1 FAIL**，验收日志与改动前
  **逐字节相同**（mag_nominal seed1 `max_pitch=22.069 > 20`，其余 14 项 PASS）；
  15 场景 stdout 除上述 8 个新键外全部相同（`cmp_task11.py` 判 ALL_IDENTICAL）。
- `--rc-profile` 6 行冒烟（10 s）：trace-ctrl `rc0`(roll) 按 1500→1600→1500 插值
  （t=4.5 s 为 1575）、`rc8`(CH9) 按 1000→1600 插值（同一时刻 trace 与 CSV 一致）；
  相同轨迹仅 CH9=1000 vs 1600（`ATTITUDE_MODE` vs `RATE_MODE`）时
  `max_roll` 1.381° vs 32.501°、`final_alt` 4.366 m vs 4.101 m，证明 CH9 真进了控制律。
- 单开关（mag_clean seed1，40 s）：默认 `final_alt=9.859` / `hold_end_rms=0.875` /
  `max_roll=1.292` / `max_pitch=5.754`；`--coax-inertia-scale 2.0` →
  `28.482 / 46.165`（2× 惯量下姿态环路明显变差）；`--coax-thrust-scale 0.8` →
  `max_pitch=2.603` / `max_roll=1.028`；`--coax-ix 0.1` → `max_roll=1.457`；
  `--coax-yaw-scale 2.0` 影响很小（定高段差速/偏航力矩很小，属预期）。
- 非法值全部 `rc=2`：`--coax-alloc-mode 2`、`--coax-ix 0|9`、
  `--coax-inertia-scale 0.1`、`--coax-thrust-scale 2.0`、`--coax-yaw-scale 9`，
  以及 CSV 表头错/非单调 t/列越界/文件缺失/空文件。
- 环境注记：本机 SIL 进程向 `%TEMP%`/用户根写 trace 会 `errno=13`（Permission
  denied，宿主策略），trace 证据改写到 `T:/traces`（项目内 tmp 盘）；`--rc-profile`
  用相对路径读取正常。
