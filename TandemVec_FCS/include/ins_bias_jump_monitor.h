/**
 * @file    ins_bias_jump_monitor.h
 * @module  固件/导航（IMU 零偏健康监控）
 * @brief   监控加计/陀螺零偏的阶跃与慢漂移，只告警不直接回滚
 * @details 输入：EKF 零偏估计、步长；处理：2 s 快基线比阶跃（连续 3 帧超门限）、
 *          10 s 慢基线比温漂、30 s 超慢基线比弱陀螺漂移；输出：跳变/漂移标志、
 *          变化量、跳变计数。发现跳变后接受新工作点，避免重复告警。
 * @theory  组合导航系统级故障检测（FDI）：对残差/状态做门限与 χ² 类检验，
 *          检测与隔离分开，避免一次坏更新污染零偏 @ref REF-121 §7.3.1 p.255
 * @frame   N/A（三轴零偏矢量模长）
 * @units   加计零偏 m/s²，陀螺零偏 rad/s，时间 s
 * @note    门限（加计 0.18 m/s²、陀螺 0.035 rad/s、加计漂移 0.003 m/s²/s、
 *          陀螺漂移 0.0003 rad/s/s、弱漂移 8e-5 rad/s/s）为台架/飞行观测折中，
 *          改动前先确认是否属于行为红线；本类不做任何状态写回。
 * @see     `handleNavigationSystem()` 零偏跳变监控段、`gnc_tel.bias_*`
 */

#pragma once

// 零偏跳变监控（只告警，不直接回滚）
// 用于发现加计/陀螺零偏阶跃、失锁恢复异常，避免把一次坏更新当成真实零偏。
#include <Eigen/Dense>
#include <cmath>
#include <cstdint>

class InsBiasJumpMonitor
{
public:
  struct Result
  {
    bool accel_jump = false;
    bool gyro_jump = false;
    bool accel_slow_drift = false;
    bool gyro_slow_drift = false;
    bool gyro_weak_drift = false;
    float accel_delta = 0.0f;
    float gyro_delta = 0.0f;
    float accel_drift_rate = 0.0f;
    float gyro_drift_rate = 0.0f;
    uint32_t jump_count = 0;
  };

  void Reset(const Eigen::Vector3f &accel_bias,
             const Eigen::Vector3f &gyro_bias)
  {
    ab_ = accel_bias;
    gb_ = gyro_bias;
    baseline_ab_ = accel_bias;
    baseline_gb_ = gyro_bias;
    slow_ab_ = accel_bias;
    slow_gb_ = gyro_bias;
    very_slow_gb_ = gyro_bias;
    elapsed_s_ = 0.0f;
    elapsed_very_slow_s_ = 0.0f;
    accel_slow_hold_ = 0;
    gyro_slow_hold_ = 0;
    gyro_weak_hold_ = 0;
    initialized_ = true;
    accel_hold_ = 0;
    gyro_hold_ = 0;
    count_ = 0;
  }

  Result Update(const Eigen::Vector3f &accel_bias,
                const Eigen::Vector3f &gyro_bias,
                float dt_s)
  {
    Result r;
    if (!initialized_)
    {
      Reset(accel_bias, gyro_bias);
      return r;
    }
    if (!(dt_s > 1e-5f) || !accel_bias.allFinite() || !gyro_bias.allFinite())
    {
      return r;
    }
    dt_s = std::fmin(dt_s, 0.05f);
    // 慢基线，约 2 s；阶跃会持续偏离基线，从而被计数。
    const float alpha = dt_s / (2.0f + dt_s);
    baseline_ab_ += alpha * (accel_bias - baseline_ab_);
    baseline_gb_ += alpha * (gyro_bias - baseline_gb_);

    const Eigen::Vector3f da = accel_bias - baseline_ab_;
    const Eigen::Vector3f dg = gyro_bias - baseline_gb_;
    r.accel_delta = da.norm();
    r.gyro_delta = dg.norm();

    // 慢漂移：10 s 基线，检测温漂等缓变；与阶跃分开告警。
    elapsed_s_ += dt_s;
    const float alpha_slow = dt_s / (10.0f + dt_s);
    slow_ab_ += alpha_slow * (accel_bias - slow_ab_);
    slow_gb_ += alpha_slow * (gyro_bias - slow_gb_);
    // 30 s 超慢基线：用于 0.01 dps/s 级弱陀螺漂移；噪声经长平均后更低。
    elapsed_very_slow_s_ += dt_s;
    const float alpha_very_slow = dt_s / (30.0f + dt_s);
    very_slow_gb_ += alpha_very_slow * (gyro_bias - very_slow_gb_);
    if (elapsed_very_slow_s_ > 30.0f)
    {
      const float weak_rate = (gyro_bias - very_slow_gb_).norm() / 30.0f;
      if (weak_rate > 0.000080f)
      {
        if (++gyro_weak_hold_ >= 3)
        {
          r.gyro_weak_drift = true;
          gyro_weak_hold_ = 0;
          very_slow_gb_ = gyro_bias;
        }
      }
      else
      {
        gyro_weak_hold_ = 0;
      }
    }

    if (elapsed_s_ > 10.0f)
    {
      const Eigen::Vector3f ra = (accel_bias - slow_ab_) / 10.0f;
      const Eigen::Vector3f rg = (gyro_bias - slow_gb_) / 10.0f;
      r.accel_drift_rate = ra.norm();
      r.gyro_drift_rate = rg.norm();

      if (r.accel_drift_rate > 0.0030f)
      {
        if (++accel_slow_hold_ >= 3)
        {
          r.accel_slow_drift = true;
          accel_slow_hold_ = 0;
          slow_ab_ = accel_bias;
        }
      }
      else
      {
        accel_slow_hold_ = 0;
      }

      if (r.gyro_drift_rate > 0.00030f)
      {
        if (++gyro_slow_hold_ >= 3)
        {
          r.gyro_slow_drift = true;
          gyro_slow_hold_ = 0;
          slow_gb_ = gyro_bias;
        }
      }
      else
      {
        gyro_slow_hold_ = 0;
      }
    }

    // 3 个连续周期超过阈值才判定，抑制单帧噪声。
    if (r.accel_delta > 0.18f)
    {
      if (++accel_hold_ >= 3)
      {
        r.accel_jump = true;
        accel_hold_ = 0;
        ++count_;
        baseline_ab_ = accel_bias; // 接受新工作点，避免重复告警
      }
    }
    else
    {
      accel_hold_ = 0;
    }

    if (r.gyro_delta > 0.035f)
    {
      if (++gyro_hold_ >= 3)
      {
        r.gyro_jump = true;
        gyro_hold_ = 0;
        ++count_;
        baseline_gb_ = gyro_bias;
      }
    }
    else
    {
      gyro_hold_ = 0;
    }

    r.jump_count = count_;
    ab_ = accel_bias;
    gb_ = gyro_bias;
    return r;
  }

  bool initialized() const { return initialized_; }

private:
  Eigen::Vector3f ab_ = Eigen::Vector3f::Zero();
  Eigen::Vector3f gb_ = Eigen::Vector3f::Zero();
  Eigen::Vector3f baseline_ab_ = Eigen::Vector3f::Zero();
  Eigen::Vector3f baseline_gb_ = Eigen::Vector3f::Zero();
  Eigen::Vector3f slow_ab_ = Eigen::Vector3f::Zero();
  Eigen::Vector3f slow_gb_ = Eigen::Vector3f::Zero();
  Eigen::Vector3f very_slow_gb_ = Eigen::Vector3f::Zero();
  float elapsed_s_ = 0.0f;
  float elapsed_very_slow_s_ = 0.0f;
  bool initialized_ = false;
  uint32_t accel_hold_ = 0;
  uint32_t gyro_hold_ = 0;
  uint32_t accel_slow_hold_ = 0;
  uint32_t gyro_slow_hold_ = 0;
  uint32_t gyro_weak_hold_ = 0;
  uint32_t count_ = 0;
};
