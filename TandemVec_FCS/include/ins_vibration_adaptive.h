/**
 * @file    ins_vibration_adaptive.h
 * @module  固件/导航（振动自适应噪声）
 * @brief   由 IMU 高通残差的均方根估计振动量级，并给出 EKF 噪声建议值
 * @details 输入：机体系加速度/角速度、步长；处理：0.5 Hz 一阶低通得慢分量 →
 *          原始减慢分量得振动分量 → 0.5 s EMA 均方 → RMS → 线性映射为
 *          accel/gyro 量测标准差建议；输出：`InsVibrationEstimate`（含回差标志）。
 *          本类只产生建议值，写不写 EKF 由 `BFS_NAVIGATION_VIBRATION_ADAPTIVE`
 *          与上层安全策略（失锁/延时/低精度回退 0.25）决定。
 * @theory  振动环境下量测噪声方差随振动功率增大（R 自适应）；
 *          ACCEL_STD 为 EKF 加计噪声，属随机误差建模 @ref REF-121 §2.2.1 p.33
 * @frame   机体系 FRD 输入；输出为标量，无坐标系
 * @units   m/s²、m/s²/g（1 g 取 9.79）、rad/s、dps、s
 * @note    RMS 时间常数 0.5 s、高通 0.5 Hz 是工程折中（见 `kLpTau`/`kRmsTau`）；
 *          高振动回差 0.15 g 进 / 0.10 g 出，防止阈值附近抖动。
 * @see     `handleNavigationSystem()` 振动自适应段、`BFS_NAVIGATION_VIBRATION_ADAPTIVE`
 */

#pragma once

// ============================================================
//  在线振动估计与自适应噪声（纯平台无关）
//
//  - 一阶高通去掉重力/慢机动，保留振动频段；
//  - 对高通后的加速度/陀螺做 EMA 均方，得到 RMS；
//  - 由 RMS 给出建议的 accel_std / gyro_std；
//  - 仅提供建议值，是否写入 EKF 由上层开关决定。
//
//  默认不改变 EKF 参数；当前固件只在 BFS_NAVIGATION_VIBRATION_ADAPTIVE
//  打开时才消费建议值。
// ============================================================

#include <algorithm>
#include <cmath>
#include <Eigen/Dense>

#ifndef BFS_NAVIGATION_VIBRATION_ADAPTIVE
#define BFS_NAVIGATION_VIBRATION_ADAPTIVE 1
#endif

// 陀螺侧允许的保守抬升上界 [rad/s]；估计器建议值还会被它夹持。
// 默认 0.010 与历史行为一致（历史直接写入 suggest，其自身上限即 0.010）。
#ifndef BFS_NAVIGATION_VIBRATION_GYRO_STD_MAX
#define BFS_NAVIGATION_VIBRATION_GYRO_STD_MAX 0.010f
#endif

struct InsVibrationEstimate
{
  float accel_rms_mps2 = 0.0f;
  float accel_rms_g = 0.0f;
  float gyro_rms_radps = 0.0f;
  float gyro_rms_dps = 0.0f;
  float accel_std_suggest = 0.25f;
  float gyro_std_suggest = 0.0015f;
  bool high_vibration = false;
};

class InsVibrationEstimator
{
public:
  void Reset()
  {
    lp_accel_.setZero();
    lp_gyro_.setZero();
    accel_ms_ = 0.0f;
    gyro_ms_ = 0.0f;
    out_ = InsVibrationEstimate{};
  }

  InsVibrationEstimate Update(const Eigen::Vector3f &accel_mps2,
                              const Eigen::Vector3f &gyro_radps,
                              float dt_s)
  {
    if (!(dt_s > 1.0e-5f) || !accel_mps2.allFinite() || !gyro_radps.allFinite())
    {
      return out_;
    }
    dt_s = std::min(dt_s, 0.05f);

    // 0.5 Hz 一阶低通，高通 = 原始 - 低通。
    constexpr float kLpTau = 0.318f; // 1/(2π·0.5)
    const float a = dt_s / (kLpTau + dt_s);
    lp_accel_ += (accel_mps2 - lp_accel_) * a;
    lp_gyro_ += (gyro_radps - lp_gyro_) * a;
    const Eigen::Vector3f hp_accel = accel_mps2 - lp_accel_;
    const Eigen::Vector3f hp_gyro = gyro_radps - lp_gyro_;

    // 0.5 s 均方 EMA；sqrt 后得到 RMS。
    constexpr float kRmsTau = 0.5f;
    const float b = dt_s / (kRmsTau + dt_s);
    accel_ms_ += (hp_accel.squaredNorm() - accel_ms_) * b;
    gyro_ms_ += (hp_gyro.squaredNorm() - gyro_ms_) * b;
    accel_ms_ = std::max(0.0f, accel_ms_);
    gyro_ms_ = std::max(0.0f, gyro_ms_);

    out_.accel_rms_mps2 = std::sqrt(accel_ms_);
    out_.accel_rms_g = out_.accel_rms_mps2 / 9.79f;
    out_.gyro_rms_radps = std::sqrt(gyro_ms_);
    out_.gyro_rms_dps = out_.gyro_rms_radps * 57.2957795f;

    // 建议值：安静 ~0.05，1 g 振动 ~0.4；限幅防止极端值。
    out_.accel_std_suggest =
        std::clamp(0.05f + 1.50f * out_.accel_rms_g, 0.05f, 0.35f);
    out_.gyro_std_suggest =
        std::clamp(0.0015f + 0.0025f * out_.gyro_rms_dps, 0.0015f, 0.010f);

    // 50 mg 回差，避免阈值附近抖动。
    if (!out_.high_vibration && out_.accel_rms_g > 0.15f)
    {
      out_.high_vibration = true;
    }
    else if (out_.high_vibration && out_.accel_rms_g < 0.10f)
    {
      out_.high_vibration = false;
    }
    return out_;
  }

  const InsVibrationEstimate &last() const { return out_; }

private:
  Eigen::Vector3f lp_accel_ = Eigen::Vector3f::Zero();
  Eigen::Vector3f lp_gyro_ = Eigen::Vector3f::Zero();
  float accel_ms_ = 0.0f;
  float gyro_ms_ = 0.0f;
  InsVibrationEstimate out_{};
};

/**
 * @brief 陀螺侧建议值的安全夹持（上层策略，非估计器内部逻辑）
 * @details 本模块的高通只到 0.5 Hz 量级，且输入还经过 IMU 软件低通：实测弹道上
 *          `gyro_rms_dps` 主要由确定性机动贡献，`gyro_std_suggest` 会长期停在
 *          标称值的数倍处，而加计侧早有 `[0.15, 0.25]` 保守夹持、陀螺侧没有。
 *          这里把两侧对齐：不得低于标称 0.0015，不得高于给定的保守上界。
 * @param  suggest_radps 估计器建议的陀螺量测标准差 [rad/s]
 * @param  cap_radps     允许的最大抬升 [rad/s]；取 0.0015 等效关闭自适应
 * @return 夹持后的 gyro 量测标准差 [rad/s]
 * @units  rad/s
 * @see    `handleNavigationSystem()` 振动自适应段、`test_ins_vibration_adaptive.cpp` T5
 */
inline float InsClampGyroStd(const float suggest_radps, const float cap_radps)
{
  constexpr float kNominalGyroStdRadps = 0.0015f;
  const float cap = std::max(kNominalGyroStdRadps, cap_radps);
  if (!(suggest_radps > 0.0f))
  {
    return kNominalGyroStdRadps;
  }
  return std::clamp(suggest_radps, kNominalGyroStdRadps, cap);
}
