/**
 * @file    ins_ahrs_gravity_gate.h
 * @module  固件/导航（AHRS 姿态观测门）
 * @brief   用“实测比力方向 vs EKF 预测重力方向”的夹角判定加速度计是否
 *          仍可作为重力观测，防止无坡度转弯/持续推力把 AHRS 拉平。
 * @details AHRS（Madgwick）的 roll/pitch 由陀螺积分 + 加速度重力修正得到。
 *          只用 |f|≈g 作为门限时，无坡度转弯的升力也沿机体系 z，模长≈g，
 *          AHRS 会把 20° 俯仰误判成 0°，再作为姿态量测灌入 EKF。本模块
 *          要求实测比力方向与当前姿态预测的重力方向夹角小于门限，方向
 *          不符时拒绝该量测（EKF 继续用陀螺/GNSS/磁）。
 * @theory  静基座/无加速度时 f^b=-C_n^b g^n，方向即重力；有气动/推力时
 *          f^b 不再等于重力方向，加速度计不能作倾角观测
 *          @ref REF-120 §9.9.3 p.336；AHRS 重力修正的适用条件
 *          @ref REF-121 §8.2.3 p.291（外部信息失效时不能靠新息“造”观测）
 * @frame   机体系 FRD；方向向量单位化
 * @units   rad
 * @approx  以 EKF 当前姿态为参考；姿态明显错误时既可能误拒也可能误放，
 *          因此只在超出 VTOL 低倾角/低过载包线时启用，包线内保留原行为。
 *          门限 10°，大于常规振动/安装误差。
 */

#pragma once

#include "Eigen/Dense"
#include <cmath>

// 门控只在超出 VTOL 低倾角/低过载包线时启用：包线内保留原 AHRS 行为，
// 包线外加速度计不再可能是重力仪，必须做方向一致性检查。
inline bool InsAhrsGravityGateRequired(const float roll_rad,
                                       const float pitch_rad,
                                       const float load_g)
{
  return std::fabs(roll_rad) > 15.0f * 0.0174532925f ||
         std::fabs(pitch_rad) > 15.0f * 0.0174532925f || load_g > 1.5f;
}

inline bool InsAhrsGravityDirectionAllowed(
    const Eigen::Vector3f &expected_specific_force_dir_body,
    const Eigen::Vector3f &accel_body_mps2,
    const float max_angle_rad)
{
  const float accel_norm = accel_body_mps2.norm();
  const float expected_norm = expected_specific_force_dir_body.norm();
  if (!(accel_norm > 1.0e-3f) || !(expected_norm > 1.0e-6f) ||
      !std::isfinite(accel_norm) || !std::isfinite(expected_norm))
  {
    return false;
  }
  const float dot = accel_body_mps2.dot(expected_specific_force_dir_body) /
                    (accel_norm * expected_norm);
  const float angle = std::acos(std::fmax(-1.0f, std::fmin(1.0f, dot)));
  return angle < max_angle_rad;
}
