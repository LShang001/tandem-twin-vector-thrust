# Changelog

## Fork 2026-06-11：单位一致性修复
- **修复（关键）**：`ekf_15_state.h` GNSS 量测更新中 `lla2ned(gnss_lla, hist_lla)`
  未传单位参数，弧度输入被按默认 DEG 解释，GNSS 位置新息被缩小 π/180（约 1/57.3）
  倍且方向被扭曲。修复后位置新息恢复正确尺度——**实机 GNSS 位置修正力度将显著
  增强属预期行为**；若 NIS 拒绝率异常升高，优先复核 `GNSS_NIS_REJECT_THRESHOLD`
  与 GNSS 噪声参数，而不是回退本修复。
- **变更**：`transforms.h` 地理函数（lla2ecef/ecef2lla/ecef2ned/ned2ecef/
  lla2ned/ned2lla）与 `earth_model.h` 全部函数的 `AngPosUnit` 默认值由 DEG 统一
  改为 RAD，与姿态函数、EKF 内部及全仓调用约定一致；传度数需显式
  `AngPosUnit::DEG`。修复了 `llarate`（默认 RAD）与 `navrate`（原默认 DEG）
  孪生向量重载默认单位相反的隐患。
- **文档**：修正 README 中 `WE_RADPS` 笔误（7.292115e-11 → 7.292115e-5 rad/s）；
  补充比力符号约定（静止时 FRD 加速度计 z ≈ −9.81 m/s²）。
- **测试**：新增宿主回归测试 `test/navigation_lla2ned_units/`，锁定 lla2ned
  新息尺度、RAD 默认值与 navrate/llarate 默认一致性。
- 重写 Arduino / CMake 示例，所有地理函数调用显式标注单位。

## v4.0.2
- Typo / bug in EKF15 state didn't take the sqrt of the denominator in the radius of curvature calc

## v4.0.1
- Fixed inline-ness of functions

## v4.0.0
- Updated function signatures to match MATLAB
- Added README

## v3.0.0
- Updated to work with CMake and Arduino build systems

## v2.0.1
- Updated to eigen v2.0.0 and units v3.2.0

## v2.0.0
- Updated namespace to *bfs*

## v1.2.4
- Updated CONTRIBUTING
- Switched dependencies to GitHub source
- Moved navigation::constants to navigation

## v1.2.3
- Updated CONTRIBUTING
- Switched from ssh to https for *fetch_content*

## v1.2.2
- Updated to MIT license
- Specified versions of dependencies

## v1.2.1
- Fixed bugs with ConstrainPi and Constrain2Pi.

## v1.2.0
- Added ConstrainPi and Constrain2Pi, which can convert angles to +/-Pi or 0-2Pi respectively.

## v1.1.0

- Added Eigen::Vector3d output for lla

## v1.0.0

- Initial baseline
