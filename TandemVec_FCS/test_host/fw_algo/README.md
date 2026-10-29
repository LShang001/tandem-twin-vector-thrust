# 固件算法宿主直调接口（fw_algo）

把固件里的算法**原样**编译成共享库，让仿真/调试脚本直接调用，不再在 Python 里
另写一份实现。

## 为什么需要

此前 `TandemVec-EKF-Report/sim/ekf_core.py`（835 行）按固件语义复刻了 15 态 ESKF，
`sim/hifi/hifi_core.py`（571 行）复刻了控制律。任何固件修复都要在两处各改一遍，
结论还可能不一致；仿真里验证过的参数也不等于固件里的行为。

`fw_algo` 的做法：源码就是 `lib/navigation-main/src/ekf_15_state.h`，本目录只加一层
薄封装 + C ABI。算法、门限、默认值全部透传，不复制公式。

## 组成

| 文件 | 作用 |
|---|---|
| `fw_algo_api.h` | 稳定 C ABI（初始化/传播/6 类量测/复位/门控/状态/诊断 JSON） |
| `fw_algo_api.cpp` | 薄封装 `bfs::Ekf15State`，返回 `MeasurementUpdateResult` 全字段 |
| `build_fw_algo.sh` | 生成 `fw_algo.dll` / `libfw_algo.so` |
| `../../test_host/test_fw_algo_api.cpp` | ABI 路径 vs 直接调用固件类，逐位一致性回归 |
| `TandemVec-EKF-Report/sim/fwapi/` | Python ctypes 封装（`fwapi.Ekf`，numpy 接口） |

## 快速开始

### Python（推荐）

```python
import sys
sys.path.insert(0, "TandemVec-EKF-Report/sim")
import fwapi, numpy as np

lla0 = np.array([28.2*np.pi/180, 112.9*np.pi/180, 50.0])
ekf = fwapi.Ekf()                       # 首次调用自动编译共享库（约 36 s），之后缓存
ekf.initialize(accel=(0, 0, -9.79), gyro=(0, 0, 0), lla=lla0)
ekf.set_velocity_subset(True, max_pos_resid_m=25.0, max_fires_per_epoch=1)

ekf.time_update_two_sample(dth1, dth2, dv1, dv2, 0.005)
r = ekf.gnss_update(vel_ned, lla, 0.005, measurement_age_s=0.02,
                    require_delayed_snapshot=True)
s = ekf.state()        # lla/vel/quat/ypr/ba/bg/cov_diag/计数器/残差
d = ekf.diag()         # 固件侧 JSON 诊断
```

## 构建与加载

```bash
bash TandemVec_FCS/test_host/fw_algo/build_fw_algo.sh          # 输出到 /tmp/tv_fw_algo
bash TandemVec_FCS/test_host/fw_algo/build_fw_algo.sh --print-path
```

- 默认输出到 `${TMPDIR:-/tmp}/tv_fw_algo`：MinGW 无法在中文路径下写目标文件。
- Windows 使用 `-static` 链接，DLL 自包含 MinGW 运行库（Python 进程 PATH 上没有
  `libstdc++-6.dll` / `libwinpthread-1.dll`，否则 `ctypes.CDLL` 报 “Could not find module”）。
- 固定 `-DEIGEN_DONT_VECTORIZE -DEIGEN_DONT_PARALLELIZE`：与 SIL/固件标量口径一致，
  避免多核向量化把边际样本变成双吸引子。
- `-DEKF_NAV_TELEMETRY` 只增加只读协方差访问器（全仓仅此一处守卫），不改滤波数学。
- Python 侧环境变量：`FW_ALGO_LIB` 指定现成库；`FW_ALGO_NO_BUILD=1` 禁止自动构建；
  `FW_ALGO_BIN_DIR`/`BIN_DIR` 指定输出目录。

## 控制/分配层（ABI v2，2026-10-29 追加）

| ABI 函数 | 固件实现 |
|---|---|
| `fw_ctrl_default_params` | `TandemVec_Config.h::kDefaultTandemVecParams`（14 个机型参数） |
| `fw_ctrl_state_hover` | 以 `w_hover=√(0.5·m·g/kT)` 初始化分配器状态 |
| `fw_tandemvec_alloc` | `TandemVecAllocator.h::allocateTandemVec`（惯量逆解/BTRUE/差速开方/τm 观测/饱和） |
| `fw_thrust_alloc` | `ThrustAllocation.h::allocateLinearThrustThrottle`（比力→总推力/线性油门） |

Python：`fwapi.TandemVecAllocator`（有状态，逐拍调用）与 `fwapi.thrust_alloc()`；
默认参数直接取固件常量，不在 Python 维护第二份。ABI v1→v2 为新增结构体，
旧 `fwapi` 会校验版本并自动重建共享库。

## 覆盖的固件路径

| ABI 函数 | 固件方法 |
|---|---|
| `fw_ekf_init` | `Initialize`（含磁偏置/手动对准角） |
| `fw_ekf_time_update` / `_two_sample` | `TimeUpdate` / `TimeUpdateTwoSample` |
| `fw_ekf_gnss_update` | `MeasurementUpdateDetailed`（延迟回放 + NIS + 子集 + 持久重锚） |
| `fw_ekf_mag_update` | `MeasurementUpdateMagVectorDetailed` |
| `fw_ekf_baro_update` | `MeasurementUpdateBaroAltitudeDetailed` |
| `fw_ekf_aiding_velocity_update` | `MeasurementUpdateAidingVelocityDetailed` |
| `fw_ekf_gravity_update` | `MeasurementUpdateGravityDetailed` |
| `fw_ekf_static_gyro_update` | `MeasurementUpdateStaticGyroDetailed` |
| `fw_ekf_yaw_update` | `MeasurementUpdateYawDetailed` |
| `fw_ekf_reset_position_velocity` / `_velocity` | `ResetPositionVelocityToGnss` / `ResetVelocityToExternal` |
| `fw_ekf_set_gnss_gates` | `set_gnss_consider_heading` / `set_gnss_reanchor_allowed` / 垂直重锚 |
| `fw_ekf_set_velocity_subset` | 速度子集软恢复开关/上限/每历元配额 |
| `fw_ekf_set_imu_limits` / `_q_accel_scale` / `_lever_arm` | 合理值门限 / Q 缩放 / GNSS 杆臂 |
| `fw_ekf_get_state` / `fw_ekf_diag_json` | 状态快照（含 P 对角）/ 单行 JSON 诊断 |

## 验证

1. **宿主一致性**：
   - `test_host/test_fw_algo_api.cpp`（`fwalgo`）：
   同一输入序列下经 ABI 与直接 `Ekf15State` 的 LLA/速度/四元数/零偏/P 对角/计数器
     **逐位相同**，当前 **468 项检查 0 失败**，覆盖 GNSS 融合与野值拒绝路径。
   - `test_host/test_fw_ctrl_api.cpp`（`fwctrl`）：分配器 11 319 例 + 推力分配 231 例
     与直接调用头文件逐位相同（含 next 状态递推），非法输入回零。
2. **与 Python 复刻版对拍**（`TandemVec-EKF-Report/sim/run_fw_ekf_parity.py`，20 s、
   10 Hz GNSS、静态+小幅正弦激励、无磁/无气压/无静止辅助）：
   水平位置 RMS **0.097 m**（max 0.687 m，出现在 t=0 的 P0 差异）、垂直 RMS **2.3 mm**、
   速度 RMS **0.017 m/s**、倾角 RMS **0.069°**（max 0.159°）；
   航向 RMS 2.42°（max 5.57°）——该场景无磁，航向不可观，两实现的零偏积分必然分叉，
   不是实现错误。结论：倾角/位置/速度高度一致，**Python 复刻版可以退出基线研究**，
   只保留 `opt/` 里的算法变体探索。

## 边界

- **不含 `navigation_task` 层**：自动延时估计、IMU 健康、气压/磁门控、失联降级、
  位置保持等仍在 SIL（`fw_sil`）里端到端验证；`fw_algo` 只到 EKF 算法本身。
- **不含 plant/传感器模型**：本接口是“喂什么算什么”，传感器与 6DOF 在 `fw_sil`。
- ABI 结构体布局变更必须同步 `FW_ALGO_ABI_VERSION` 与 `fwapi/library.py`；
  `fwapi` 加载时会校验版本，不匹配直接报错。
- 新增固件算法时优先在此加一个 ABI 函数 + `fwapi` 方法 + 一致性断言，
  不要再在 `sim/` 里写第二份实现。
