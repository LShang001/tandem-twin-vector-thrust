/**
 * @file    ins_bias_temp_table.h
 * @module  固件/导航（IMU 温度-零偏标定数据面）
 * @brief   维护最多 16 个温度点的加计/陀螺零偏表并做线性插值
 * @details 输入：温度、零偏估计；处理：±0.25°C 内视为同一点覆盖旧值，否则插入
 *          并按温度升序排序；查询时端点外做钳位、区间内线性插值；
 *          输出：插值后的三轴零偏。仅存数据，是否补偿由上层决定。
 * @theory  分段线性插值（表驱动温补的一阶形式）@ref REF-121 §2.2.1 p.33
 *          （以先验/查表值作为量测或补偿量的基本做法）
 * @frame   N/A（温度 → 三轴零偏）
 * @units   温度 °C，零偏 rad/s 与 m/s²
 * @note    表容量满或温度越界（-40~125°C）时拒绝写入；本工程当前只采点、
 *          不参与补偿（见 `handleNavigationSystem()` 温度表段），避免未标定即补偿。
 * @see     `InsBiasTempObserver`
 */

#pragma once

// 温度-零偏校准表（最多 16 点，线性插值）
// 仅作为温补数据面：先标定/采点，再由上层决定是否启用补偿。
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <cstdint>

class InsBiasTempTable
{
public:
  static constexpr uint8_t kMaxPoints = 16;

  struct Point
  {
    float temp_c = 0.0f;
    Eigen::Vector3f gyro_bias_radps = Eigen::Vector3f::Zero();
    Eigen::Vector3f accel_bias_mps2 = Eigen::Vector3f::Zero();
  };

  void Reset() { count_ = 0; }

  bool AddPoint(float temp_c, const Eigen::Vector3f &gyro_bias_radps,
                const Eigen::Vector3f &accel_bias_mps2)
  {
    if (!std::isfinite(temp_c) || !gyro_bias_radps.allFinite() ||
        !accel_bias_mps2.allFinite() || temp_c < -40.0f || temp_c > 125.0f)
    {
      return false;
    }
    if (count_ >= kMaxPoints)
    {
      return false;
    }
    for (uint8_t i = 0; i < count_; ++i)
    {
      if (std::fabs(points_[i].temp_c - temp_c) < 0.25f)
      {
        points_[i].gyro_bias_radps = gyro_bias_radps;
        points_[i].accel_bias_mps2 = accel_bias_mps2;
        Sort();
        return true;
      }
    }
    points_[count_].temp_c = temp_c;
    points_[count_].gyro_bias_radps = gyro_bias_radps;
    points_[count_].accel_bias_mps2 = accel_bias_mps2;
    ++count_;
    Sort();
    return true;
  }

  bool Interpolate(float temp_c, Eigen::Vector3f *gyro_bias_radps,
                   Eigen::Vector3f *accel_bias_mps2) const
  {
    if (count_ < 2 || !std::isfinite(temp_c) ||
        gyro_bias_radps == nullptr || accel_bias_mps2 == nullptr)
    {
      return false;
    }
    if (temp_c <= points_[0].temp_c)
    {
      *gyro_bias_radps = points_[0].gyro_bias_radps; // 低于表头：钳位，不外推
      *accel_bias_mps2 = points_[0].accel_bias_mps2;
      return true;
    }
    if (temp_c >= points_[count_ - 1].temp_c)
    {
      *gyro_bias_radps = points_[count_ - 1].gyro_bias_radps; // 高于表尾：钳位，不外推
      *accel_bias_mps2 = points_[count_ - 1].accel_bias_mps2;
      return true;
    }
    for (uint8_t i = 1; i < count_; ++i)
    {
      if (temp_c <= points_[i].temp_c)
      {
        const Point &a = points_[i - 1];
        const Point &b = points_[i];
        const float span = b.temp_c - a.temp_c;
        const float f = (span > 1e-6f) ? (temp_c - a.temp_c) / span : 0.0f;
        *gyro_bias_radps = (1.0f - f) * a.gyro_bias_radps +
                           f * b.gyro_bias_radps;
        *accel_bias_mps2 = (1.0f - f) * a.accel_bias_mps2 +
                           f * b.accel_bias_mps2;
        return true;
      }
    }
    return false;
  }

  uint8_t size() const { return count_; }

private:
  void Sort()
  {
    std::sort(points_, points_ + count_,
              [](const Point &a, const Point &b) {
                return a.temp_c < b.temp_c;
              });
  }

  Point points_[kMaxPoints];
  uint8_t count_ = 0;
};
