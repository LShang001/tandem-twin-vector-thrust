# 宿主机平台无关算法回归测试

本目录用于在不依赖 STM32 硬件的前提下，用 `g++` 编译运行 `include/` 下平台无关算法的回归测试。

## 覆盖范围

| 测试文件 | 被测算法 | 依赖 | 断言数 |
|---|---|---|---|
| `test_gnss_dynamic_weight.cpp` | `ins_gnss_dynamic_weight.h` — GNSS 动态权重 R 矩阵 | 无 | 28 |
| `test_static_aid_profile.cpp` | `ins_static_aid_profile.h` — 静止辅助强度调度 | 无 | 41 |
| `test_static_detector.cpp` | `ins_static_detector.h` — 静止检测器状态机 | Eigen/Dense | 22 |
| `test_altitude_reference.cpp` | `ins_altitude_reference.h` — 气压/GNSS/控制高度参考系转换 | 无 | 8 |
| `test_gnss_epoch_timing.cpp` | `ins_gnss_epoch_timing.h` — epoch 消费门控、iTOW 到 MCU 时间映射与回绕 | 无 | 20 |
| `test_vertical_kf.cpp` | `VerticalKF.h` — 三状态垂直卡尔曼滤波器 | Arduino.h 桩 | 16 |

测试覆盖：正常输入输出、边界条件（阈值边界、floor/cap、迟滞帧数）、异常路径（NaN/Inf 防御、状态可恢复性）、状态机切换、以及 KF 预测/更新数学与数值稳定性。

`test_vertical_kf.cpp` 通过 `test_host/stub/Arduino.h`（极简桩，仅转发到 `<math.h>`/`<string.h>`）使依赖 `Arduino.h` 的 `VerticalKF.h` 可在宿主机编译；该桩仅在宿主机测试路径生效，不影响 PlatformIO 固件编译。

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
