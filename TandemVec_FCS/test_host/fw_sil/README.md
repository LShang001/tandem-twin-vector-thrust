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
