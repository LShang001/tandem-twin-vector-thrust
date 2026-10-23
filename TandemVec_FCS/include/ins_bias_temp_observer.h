/**
 * @file    ins_bias_temp_observer.h
 * @module  固件/导航（IMU 温度-零偏诊断）
 * @brief   用在线最小二乘估计陀螺零偏随芯片温度的斜率（只诊断、不补偿）
 * @details 输入：芯片温度、EKF 当前陀螺零偏估计、静止/ GNSS 健康标志；
 *          处理：仅在静止且 GNSS 健康时累积 Σt、Σt²、Σb、Σtb，样本数 ≥200 后
 *          解一元线性回归斜率；输出：斜率 [rad/s/°C]、温度跨度、置信度。
 *          不变量：不写回 EKF，不改变任何控制量。
 * @theory  一元线性回归最小二乘：斜率 = (nΣtb-ΣtΣb)/(nΣt²-(Σt)²)
 *          @ref REF-121 §2.2.1 p.33（最小二乘/滤波估计的基本形式）
 * @frame   N/A（标量温度 × 三轴零偏）
 * @units   温度 °C，零偏 rad/s，斜率 rad/s/°C，时间 s
 * @note    置信度 = min(1, 温度跨度/5°C)·min(1, n/1000)，只有温度真正变化过
 *          斜率才可信；机动/失锁期间暂停采样，避免把真实零偏变化当温漂。
 * @see     `handleNavigationSystem()` 温度-零偏诊断段、`InsBiasTempTable`
 */

#pragma once

// 温度-零偏诊断观测器（只估计斜率，不直接补偿）
// 仅在静止且 GNSS 健康时采样，避免机动/失锁污染温漂回归。
#include <Eigen/Dense>
#include <cmath>
#include <cstdint>

class InsBiasTempObserver
{
public:
  struct Result
  {
    bool valid = false;
    float temp_span_c = 0.0f;
    float confidence = 0.0f;
    Eigen::Vector3f gyro_slope_radps_per_c = Eigen::Vector3f::Zero();
  };

  void Reset()
  {
    n_ = 0;
    sum_t_ = 0.0f;
    sum_tt_ = 0.0f;
    sum_b_ = Eigen::Vector3f::Zero();
    sum_tb_ = Eigen::Vector3f::Zero();
    tmin_ = 1e9f;
    tmax_ = -1e9f;
    valid_ = false;
    slope_ = Eigen::Vector3f::Zero();
  }

  Result Update(float temp_c, const Eigen::Vector3f &gyro_bias_radps,
                bool static_ok, bool gnss_ok, float dt_s)
  {
    Result r;
    if (!(dt_s > 1e-5f) || !std::isfinite(temp_c) ||
        !gyro_bias_radps.allFinite())
    {
      return Last();
    }
    if (!static_ok || !gnss_ok)
    {
      return Last();
    }
    // 丢弃明显异常温度。
    if (temp_c < -40.0f || temp_c > 125.0f)
    {
      return Last();
    }

    ++n_;
    sum_t_ += temp_c;
    sum_tt_ += temp_c * temp_c;
    sum_b_ += gyro_bias_radps;
    sum_tb_ += temp_c * gyro_bias_radps;
    tmin_ = std::fmin(tmin_, temp_c);
    tmax_ = std::fmax(tmax_, temp_c);

    if (n_ >= 200)
    {
      const float denom = n_ * sum_tt_ - sum_t_ * sum_t_;
      if (std::fabs(denom) > 1e-6f)
      {
        slope_ = (n_ * sum_tb_ - sum_t_ * sum_b_) / denom;
        valid_ = true;
      }
    }
    r.valid = valid_;
    r.temp_span_c = tmax_ - tmin_;
    r.confidence = std::fmin(1.0f, r.temp_span_c / 5.0f) *
                   std::fmin(1.0f, static_cast<float>(n_) / 1000.0f);
    r.gyro_slope_radps_per_c = slope_;
    return r;
  }

  Result Last() const
  {
    Result r;
    r.valid = valid_;
    r.temp_span_c = tmax_ - tmin_;
    r.confidence = std::fmin(1.0f, r.temp_span_c / 5.0f) *
                   std::fmin(1.0f, static_cast<float>(n_) / 1000.0f);
    r.gyro_slope_radps_per_c = slope_;
    return r;
  }

private:
  uint32_t n_ = 0;
  float sum_t_ = 0.0f;
  float sum_tt_ = 0.0f;
  Eigen::Vector3f sum_b_ = Eigen::Vector3f::Zero();
  Eigen::Vector3f sum_tb_ = Eigen::Vector3f::Zero();
  float tmin_ = 1e9f;
  float tmax_ = -1e9f;
  bool valid_ = false;
  Eigen::Vector3f slope_ = Eigen::Vector3f::Zero();
};
