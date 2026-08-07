# 宿主机平台无关算法回归测试

本目录用于在不依赖 STM32 硬件的前提下，用 `g++` 编译运行 `include/` 下平台无关算法的回归测试。

## 覆盖范围

| 测试文件 | 被测算法 | 依赖 | 断言数 |
|---|---|---|---|
| `test_gnss_dynamic_weight.cpp` | `ins_gnss_dynamic_weight.h` — GNSS 动态权重 R 矩阵 | 无 | 28 |
| `test_static_aid_profile.cpp` | `ins_static_aid_profile.h` — 静止辅助强度调度（含帧边界精确值，并入自已删除的 static_aid_profile_test.cpp） | 无 | 46 |
| `test_static_detector.cpp` | `ins_static_detector.h` — 静止检测器状态机 | Eigen/Dense | 22 |
| `test_altitude_reference.cpp` | `ins_altitude_reference.h` — 气压/GNSS/控制高度参考系转换 | 无 | 8 |
| `test_gnss_epoch_timing.cpp` | `ins_gnss_epoch_timing.h` — epoch 消费门控、iTOW 到 MCU 时间映射与回绕 | 无 | 20 |
| `test_vertical_kf.cpp` | `VerticalKF.h` — 三状态垂直卡尔曼滤波器 | Arduino.h 桩 | 16 |
| `test_tandemvec_allocation.cpp` | `TandemVec_ControlAllocation.h` — 控制分配（DIRECT/FULL_B/FF/BTRUE、极性、饱和、往返一致、差速平方和） | 无 | 39 |
| `test_tandemvec_cascade.cpp` | `TandemVec_CascadeCtrl.h` — 级联控制律（四元数外环/内环/全链，未接入固件、仅供宿主机验证） | 无 | 20 |
| `test_tandemvec_sim.cpp` | 分配精度 + CascadeCtrl 闭环步响应 + 物理参数一致性 | 无 | 12 |
| `test_flight_control_axis.cpp` | `flight_control.cpp` 姿态环闭环仿真（直连轴序，与 thrustWrench 自洽）：roll←q_err.x/ω.x、pitch←q_err.y/ω.y、yaw←q_err.z/ω.z；**参数读自 include/FlightCtrlParams.h（kFlightCtrlParams，实机唯一事实源，防漂移）**；刚体角动力学 + 推进力矩闭环收敛、执行器符号与 propulsion.mjs 交叉一致、推力垂直投影 R13（cos_tilt）、目标姿态合成、四轴式摇杆映射、六自由度悬停全动力学（配平平衡/航向保持/饱和/电机滞后/陀螺耦合/航向指令跟踪/保范） | 无 | 38 |
| `test_position_pid.cpp` | `PositionPID.h` — v3 回归（P1–P12：基础响应/积分钳位/抗饱和/无扰切换/NaN 防护等） | 无 | 45 |
| `test_online_id.cpp` | `TandemVec_OnlineID.h` — 在线辨识（RLS + 自适应增益调度） | 无 | 17 |
| `test_advanced_theory.cpp` | 级联控制理论（Lyapunov/ADRC/辨识），名义参数分析 | Arduino.h 桩 | 12 |
| `test_robustness.cpp` | 大机动 + 系统误差鲁棒性（名义参数扫描） | Arduino.h 桩 | 8 |
| `test_qual_analysis.cpp` | 增益定性分析（Kp_a/Kp_r 扫描，固件语义对照） | Arduino.h 桩 | 16 |
| `test_comprehensive_sim.cpp` | 增益扫描/扰动抑制/模型偏差（研究工具，非实机回归） | Arduino.h 桩 | 10 |
| `test_ekf_15state.cpp` | `ekf_15_state.h` — 15 状态 EKF 主机回归（`-DEKF_HOST_REGRESSION`） | units/eigen | 15 |

> 注：`test_flight_control_axis.cpp` 的轴序为**直连序**（与 thrustWrench/allocateMoments 轴系自洽）。实机 `flight_control.cpp` mix 层存在存档系→模型系置换（悬停构型语义），两者对应关系涉及 README/flight_control/state_data 三处轴系注释矛盾（悬停 vs 巡航推力轴定义），**专项核对前不在测试中引入置换**（2026-08-08 审查结论）。

测试覆盖：正常输入输出、边界条件（阈值边界、floor/cap、迟滞帧数）、异常路径（NaN/Inf 防御、状态可恢复性）、状态机切换、以及 KF 预测/更新数学与数值稳定性。

`test_vertical_kf.cpp` 通过 `test_host/stub/Arduino.h`（极简桩，仅转发到 `<math.h>`/`<string.h>`）使依赖 `Arduino.h` 的 `VerticalKF.h` 可在宿主机编译；该桩仅在宿主机测试路径生效，不影响 PlatformIO 固件编译。

## 未收录文件（研究/专项工具，`run_all.sh` 不编译）

| 文件 | 用途 |
|---|---|
| `dps310_fifo_policy_test.cpp` / `dps_profile_parser_test.js` / `dps_profile_runner_test.js` | DPS310 采样策略/profile 工具链测试（配合 `tools/dps_profile_*.js`，需 node） |
| `test_integrator_compare.cpp` / `test_integrator_sweep.cpp` | 积分器数值研究（对拍/扫描） |
| `test_mpc_emb.cpp` / `mpc_emb.h` / `optimizer_runner.cpp` | 嵌入式 MPC 研究 |
| `test_optimal_gains.cpp` | 最优增益研究 |
| `task_scheduler_interval_test.cpp` | 任务调度器小数 tick 研究 |

> `static_aid_profile_test.cpp` 已于 2026-08-08 删除：其独有覆盖（帧边界精确值）已并入 `test_static_aid_profile.cpp`，Action 优先级与 ZUPT 噪声自适应已被收录版覆盖。

## 运行方式

在仓库根目录执行（需本机已安装 `g++`，支持 C++17）：

```bash
bash test_host/run_all.sh
```

退出码 `0` 表示全部通过，非 `0` 表示存在失败项。

也可单独编译运行某一个测试：

```bash
# GNSS 动态权重（不依赖 Eigen）
g++ -std=c++17 -Iinclude test_host/test_gnss_dynamic_weight.cpp -o test_host/bin/gdw && ./test_host/bin/gdw

# 静止辅助调度（不依赖 Eigen）
g++ -std=c++17 -Iinclude test_host/test_static_aid_profile.cpp -o test_host/bin/sap && ./test_host/bin/sap

# 静止检测器（依赖 Eigen，头文件位于 lib/eigen/src）
g++ -std=c++17 -Iinclude -Ilib/eigen/src test_host/test_static_detector.cpp -o test_host/bin/sd && ./test_host/bin/sd

# 垂直卡尔曼滤波器（依赖 Arduino.h，用 test_host/stub 桩）
g++ -std=c++17 -Itest_host/stub -Iinclude test_host/test_vertical_kf.cpp -o test_host/bin/vkf && ./test_host/bin/vkf
```

## 与 PlatformIO 测试环境的关系

本目录与 `[env:test]`（`pio test -e test`）相互独立：

- `[env:test]` 是 STM32 交叉编译环境，编译 `test/ekf_host_regression.cpp`，需要硬件运行。
- `test_host/` 不在 PlatformIO 测试收集范围内，不会被 `pio test` 编译或上传，纯宿主机验证。

修改 `include/ins_*.h` 后，先用本目录的测试快速验证算法正确性，再视情况走 `pio run` 完整编译。
