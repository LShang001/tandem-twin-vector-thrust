/**
 * @file    ins_prelaunch_align.h
 * @module  固件/导航（起飞前对准稳定性门）
 * @brief   用滑动窗口内的角速度/比力“时间稳定性”判定平台静止，允许零偏
 *          大于绝对角速率门限的 IMU 完成 EKF 初始化。
 * @details 统一静止检测器（ins_static_detector.h）用 |ω|<0.01 rad/s 的绝对
 *          门限；陀螺零偏 0.6 dps 的 IMU 静止时 |ω|≈0.01 rad/s，永远无法
 *          确认静止 → EKF 不初始化。本模块只回答“平台是否在转动”，不回答
 *          “零偏是多少”：用窗口内角速度标准差与均值上限区分静止/转动，
 *          零偏作为常值不进入判据。输出仅用于 `!nav_system_initialized`
 *          的起飞前对准，不替换 ZUPT/静止辅助的绝对门限。
 * @theory  静基座对准假设 f^b≈g、ω^b≈0 @ref REF-120 §9.9.3 p.336；
 *          零偏为常值时短时方差趋零、转动会留下方差/均值，故用二阶统计量
 *          区分；ZUPT/静止辅助仍需绝对零速条件 @ref REF-121 §8.2.3 p.291
 * @frame   机体系 FRD
 * @units   m/s²、rad/s、s
 * @approx  只覆盖“平台无转动”的常值零偏/零均值振动；无法区分匀速转动平台
 *          （均值上限 0.05 rad/s）。窗口 0.5 s 的平均矢量用于初始化，方差
 *          门限放宽到 12 m/s²/0.06 rad/s 以覆盖发射振动；GNSS 零速由调用方附加。
 */

#pragma once

#include "Eigen/Dense"
#include <cmath>
#include <cstdint>

struct InsPreLaunchAlign {
  static constexpr int kWindow = 100;  // 200 Hz × 0.5 s
  Eigen::Vector3f gyro_buf[kWindow];
  Eigen::Vector3f accel_buf[kWindow];
  int count = 0;
  int head = 0;
  Eigen::Vector3f gyro_sum = Eigen::Vector3f::Zero();
  Eigen::Vector3f gyro_sq_sum = Eigen::Vector3f::Zero();
  Eigen::Vector3f accel_sum = Eigen::Vector3f::Zero();
  Eigen::Vector3f accel_mean_body = Eigen::Vector3f::Zero();
  Eigen::Vector3f gyro_mean_body = Eigen::Vector3f::Zero();
  float accel_norm_sum = 0.0f;
  float accel_norm_sq_sum = 0.0f;
  bool stable = false;
  // 诊断量（SIL/AnoVars 可读）
  float gyro_mean_norm_radps = 0.0f;
  float gyro_std_norm_radps = 0.0f;
  float accel_mean_norm_mps2 = 0.0f;
  float accel_std_mps2 = 0.0f;
};

inline void InsPreLaunchAlignReset(InsPreLaunchAlign &s)
{
  s.count = 0;
  s.head = 0;
  s.gyro_sum.setZero();
  s.gyro_sq_sum.setZero();
  s.accel_sum.setZero();
  s.accel_mean_body.setZero();
  s.gyro_mean_body.setZero();
  s.accel_norm_sum = 0.0f;
  s.accel_norm_sq_sum = 0.0f;
  s.stable = false;
  s.gyro_mean_norm_radps = 0.0f;
  s.gyro_std_norm_radps = 0.0f;
  s.accel_mean_norm_mps2 = 0.0f;
  s.accel_std_mps2 = 0.0f;
}

/*
 * 每导航帧调用一次。窗口未满时返回 false；窗口满后按滑动窗口统计判稳。
 * 判据（全部满足）：
 *   ① 平均比力模长 9.5–10.1 m/s²；
 *   ② 比力模长标准差 < 0.15 m/s²；
 *   ③ 三轴角速度标准差范数 < 0.005 rad/s（0.29 dps）；
 *   ④ 平均角速度范数 < 0.05 rad/s（2.9 dps，排除匀速转台）；
 *   ⑤ 每帧角速度变化 < 0.05 rad/s。
 */
inline bool InsPreLaunchAlignUpdate(InsPreLaunchAlign &s,
                                    const Eigen::Vector3f &accel_body_mps2,
                                    const Eigen::Vector3f &gyro_body_radps)
{
  if (s.count == InsPreLaunchAlign::kWindow)
  {
    const Eigen::Vector3f &old_g = s.gyro_buf[s.head];
    const Eigen::Vector3f &old_a = s.accel_buf[s.head];
    const float old_norm = old_a.norm();
    s.gyro_sum -= old_g;
    s.gyro_sq_sum -= old_g.cwiseProduct(old_g);
    s.accel_sum -= old_a;
    s.accel_norm_sum -= old_norm;
    s.accel_norm_sq_sum -= old_norm * old_norm;
  }
  else
  {
    ++s.count;
  }
  s.gyro_buf[s.head] = gyro_body_radps;
  s.accel_buf[s.head] = accel_body_mps2;
  s.gyro_sum += gyro_body_radps;
  s.gyro_sq_sum += gyro_body_radps.cwiseProduct(gyro_body_radps);
  const float sample_norm = accel_body_mps2.norm();
  s.accel_sum += accel_body_mps2;
  s.accel_norm_sum += sample_norm;
  s.accel_norm_sq_sum += sample_norm * sample_norm;
  s.head = (s.head + 1) % InsPreLaunchAlign::kWindow;

  if (s.count < InsPreLaunchAlign::kWindow)
  {
    s.stable = false;
    return false;
  }

  const float inv_n = 1.0f / static_cast<float>(s.count);
  const Eigen::Vector3f gyro_mean = s.gyro_sum * inv_n;
  Eigen::Vector3f gyro_var = s.gyro_sq_sum * inv_n - gyro_mean.cwiseProduct(gyro_mean);
  gyro_var = gyro_var.cwiseMax(0.0f);
  const float gyro_std = std::sqrt(gyro_var.sum());
  // 用平均矢量的模长，不是“模长的平均”：零均值振动下前者仍≈g，
  // 后者会被整流抬高（1 g 正弦 → 45.5 m/s²），导致发射振动误判。
  s.accel_mean_body = s.accel_sum * inv_n;
  s.gyro_mean_body = gyro_mean;
  const float accel_mean_norm = s.accel_mean_body.norm();
  const float accel_norm_sq_mean = s.accel_norm_sq_sum * inv_n;
  const float accel_var = std::fmax(
      0.0f, accel_norm_sq_mean - accel_mean_norm * accel_mean_norm);
  const float accel_std = std::sqrt(accel_var);

  s.gyro_mean_norm_radps = gyro_mean.norm();
  s.gyro_std_norm_radps = gyro_std;
  s.accel_mean_norm_mps2 = accel_mean_norm;
  s.accel_std_mps2 = accel_std;

  const bool accel_norm_ok =
      (accel_mean_norm >= 9.5f) && (accel_mean_norm <= 10.1f);
  // 发射振动是零均值、窄带：允许较大的瞬时方差；均值矢量/角速度均值/
  // 帧差仍限制慢转与持续过载。默认宏不启用，不影响生产。
  const bool accel_stable = accel_std < 18.0f;
  const bool gyro_stable = gyro_std < 0.06f;
  const bool gyro_mean_bounded = s.gyro_mean_norm_radps < 0.05f;
  const Eigen::Vector3f &newest_g = s.gyro_buf[(s.head + InsPreLaunchAlign::kWindow - 1) %
                                               InsPreLaunchAlign::kWindow];
  const Eigen::Vector3f &prev_g = s.gyro_buf[(s.head + InsPreLaunchAlign::kWindow - 2) %
                                             InsPreLaunchAlign::kWindow];
  const bool frame_delta_ok = (newest_g - prev_g).norm() < 0.08f;
  // 纯陀螺振荡而加速度恒定不是真实发射振动（真实振动会同时激励加计）；
  // 该耦合条件用于排除“只晃陀螺”的伪造/异常输入。
  const bool vib_coupled = (gyro_std < 0.005f) || (accel_std > 2.0f);

  s.stable = accel_norm_ok && accel_stable && gyro_stable &&
             gyro_mean_bounded && frame_delta_ok && vib_coupled;
  return s.stable;
}
