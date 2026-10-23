/**
 * @file    ins_gnss_delay_auto.h
 * @module  固件/导航（无 PPS 的 GNSS 延时估计）
 * @brief   在机动段用批处理对齐估计 GNSS 量测延时 τ，并给出锐度与残差
 * @details 输入：IMU 纯惯导参考轨迹（20 s 环形缓存）、GNSS 位置/速度样本；
 *          处理：粗差剔除（与参考差 >50 m / >20 m/s 的样本丢弃）→ 要求速度
 *          有明显变化（匀速/悬停不可观测）→ 0~150 ms 网格搜索 + 抛物线亚网格
 *          插值 → 每个 τ 解 9 参数线性最小二乘（位置/速度联合）取残差最小；
 *          输出：τ、残差、锐度、有效标志。默认只记录，`*_APPLY` 才参与年龄。
 * @theory  量测延时使残差模型退化为“参考轨迹在 t−τ 处的值”；τ 只在有加速度
 *          激励时可辨识（可观测性来自机动）@ref REF-121 §8.2.3 p.291
 *          （量测时标对齐是组合导航一致性的前提），FDI 思想同 §7.3.1 p.255
 * @frame   NED（参考轨迹与 GNSS 速度均在 NED）
 * @units   时间 s，位置 m，速度 m/s，残差 m/s，τ 网格 5 ms
 * @note    缓存用 float 省 RAM（4000 点 ≈112 KB），位置 0.1 m 级精度对
 *          1.5 m GNSS 噪声无影响；估计值只在 sharpness≥0.003、新鲜 ≤5 s、
 *          与 15 ms 先验差 ≤100 ms 且非高振动时替换固定延时（见 navigation_task）。
 * @see     `ins_inertial_reference.h`、`handleNavigationSystem()` 延时估计段
 */

#pragma once

// ============================================================
//  无 PPS GNSS 延时自动估计（记录/估计模块，默认不参与控制）
//
//  用途：
//   - 由导航任务缓存 IMU/导航参考轨迹和 GNSS 位置/速度；
//   - 在机动段调用批处理对齐估计延时；
//   - 结果默认只记录，不改变 gnss_measurement_age_s。
//
//  该模块不依赖 Arduino/STM32，可宿主机测试。
// ============================================================

#include <cmath>
#include <deque>
#include <vector>
#include <Eigen/Dense>

#ifndef BFS_NAVIGATION_GNSS_DELAY_AUTO_ESTIMATE
#define BFS_NAVIGATION_GNSS_DELAY_AUTO_ESTIMATE 1
#endif

// 次级开关：允许估计值参与 gnss_measurement_age_s。
// 默认 0；必须先只记录验证，再单独打开。
#ifndef BFS_NAVIGATION_GNSS_DELAY_AUTO_APPLY
#define BFS_NAVIGATION_GNSS_DELAY_AUTO_APPLY 1
#endif

struct InsGnssDelaySample
{
  // 片上缓存用 float：4000 点约 112 KB，避免 224 KB 级堆占用；
  // 位置精度在 0.1 m 量级，远小于 GNSS 1.5 m 噪声。
  float t_s = 0.0f;
  Eigen::Vector3f pos = Eigen::Vector3f::Zero();
  Eigen::Vector3f vel = Eigen::Vector3f::Zero();
};

struct InsGnssDelayAutoResult
{
  bool valid = false;
  float tau_s = 0.0f;
  Eigen::Vector3d bias_mps2 = Eigen::Vector3d::Zero();
  float residual_mps = 0.0f;
  float sharpness = 0.0f;
};

class InsGnssDelayAutoEstimator
{
public:
  void Reset()
  {
    ref_.clear();
    gnss_.clear();
    last_result_ = InsGnssDelayAutoResult{};
  }

  void PushReference(double t_s, const Eigen::Vector3d &pos,
                     const Eigen::Vector3d &vel)
  {
    if (!std::isfinite(t_s) || !pos.allFinite() || !vel.allFinite())
    {
      return;
    }
    ref_.push_back({static_cast<float>(t_s), pos.cast<float>(),
                    vel.cast<float>()});
    while (ref_.size() > kMaxRef)
    {
      ref_.pop_front();
    }
  }

  void PushGnss(double t_s, const Eigen::Vector3d &pos,
                const Eigen::Vector3d &vel)
  {
    if (!std::isfinite(t_s) || !pos.allFinite() || !vel.allFinite())
    {
      return;
    }
    gnss_.push_back({static_cast<float>(t_s), pos.cast<float>(),
                     vel.cast<float>()});
    while (gnss_.size() > kMaxGnss)
    {
      gnss_.pop_front();
    }
  }

  bool TryEstimate(int min_ref = 200, int min_gnss = 20)
  {
    if (static_cast<int>(ref_.size()) < min_ref ||
        static_cast<int>(gnss_.size()) < min_gnss)
    {
      return false;
    }
    // GNSS 失锁/数据陈旧：参考轨迹已更新但 GNSS 停在旧时刻，不能估计。
    if ((ref_.back().t_s - gnss_.back().t_s) > 0.5)
    {
      return false;
    }
    // 粗差剔除：与同时间参考轨迹偏差过大的 GNSS 样本不参与拟合。
    // 延时在 30 m/s、150 ms 下也只造成约 4.5 m 位置偏差，50 m/20 m/s 门限足够宽松。
    for (auto it = gnss_.begin(); it != gnss_.end();)
    {
      const Eigen::Vector3d p = InterpRefPos(it->t_s);
      const Eigen::Vector3d v = InterpRefVel(it->t_s);
      if ((it->pos.cast<double>() - p).norm() > 50.0 ||
          (it->vel.cast<double>() - v).norm() > 20.0)
      {
        it = gnss_.erase(it);
      }
      else
      {
        ++it;
      }
    }
    // 要求参考轨迹有明显机动激励（加速度方差/速度变化）
    double vmin = 1e9, vmax = -1e9;
    for (const auto &r : ref_)
    {
      const float vn = r.vel.norm();
      vmin = std::min(vmin, static_cast<double>(vn));
      vmax = std::max(vmax, static_cast<double>(vn));
    }
    if (vmax - vmin < 0.5)
    {
      return false; // 匀速/悬停不可观测
    }

    InsGnssDelayAutoResult out;
    double costs[31];
    int best_c = -1;
    double best_cost = 1e9;
    for (int c = 0; c <= 30; ++c)
    {
      const double tau = c * 0.005;
      const double cost = Fit(tau);
      costs[c] = cost;
      if (!std::isfinite(cost)) continue;
      if (cost < best_cost)
      {
        best_cost = cost;
        best_c = c;
      }
    }
    if (best_c >= 0)
    {
      double tau = best_c * 0.005;
      if (best_c > 0 && best_c < 30)
      {
        const double cm = costs[best_c - 1];
        const double c0 = costs[best_c];
        const double cp = costs[best_c + 1];
        const double denom = cm - 2.0 * c0 + cp;
        if (denom > 1e-9)
        {
          const double delta = 0.5 * (cm - cp) / denom;
          tau += std::max(-0.005, std::min(0.005, delta * 0.005));
        }
      }
      out.valid = true;
      out.tau_s = static_cast<float>(tau);
      out.residual_mps = static_cast<float>(best_cost);
      out.sharpness = Sharpness(static_cast<float>(tau));
      last_result_ = out;
    }
    return out.valid;
  }

  const InsGnssDelayAutoResult &last_result() const { return last_result_; }

  // 诊断用：候选代价（仅宿主调试，不参与控制）
  double DebugCost(double tau_s) const { return Fit(tau_s); }
  size_t DebugRefCount() const { return ref_.size(); }
  size_t DebugGnssCount() const { return gnss_.size(); }

private:
  static constexpr size_t kMaxRef = 4000;  // 20 s @200Hz
  static constexpr size_t kMaxGnss = 200;  // 20 s @10Hz

  Eigen::Vector3d InterpRefPos(double t_s) const
  {
    if (ref_.empty()) return Eigen::Vector3d::Zero();
    if (t_s <= ref_.front().t_s) return ref_.front().pos.cast<double>();
    if (t_s >= ref_.back().t_s) return ref_.back().pos.cast<double>();
    for (size_t i = 1; i < ref_.size(); ++i)
    {
      if (ref_[i].t_s >= t_s)
      {
        const auto &a = ref_[i - 1];
        const auto &b = ref_[i];
        const double f = (t_s - static_cast<double>(a.t_s)) /
                         std::max(1e-9, static_cast<double>(b.t_s - a.t_s));
        return ((1.0 - f) * a.pos.cast<double>() +
                f * b.pos.cast<double>());
      }
    }
    return ref_.back().pos.cast<double>();
  }

  Eigen::Vector3d InterpRefVel(double t_s) const
  {
    if (ref_.empty()) return Eigen::Vector3d::Zero();
    if (t_s <= ref_.front().t_s) return ref_.front().vel.cast<double>();
    if (t_s >= ref_.back().t_s) return ref_.back().vel.cast<double>();
    for (size_t i = 1; i < ref_.size(); ++i)
    {
      if (ref_[i].t_s >= t_s)
      {
        const auto &a = ref_[i - 1];
        const auto &b = ref_[i];
        const double f = (t_s - static_cast<double>(a.t_s)) /
                         std::max(1e-9, static_cast<double>(b.t_s - a.t_s));
        return ((1.0 - f) * a.vel.cast<double>() +
                f * b.vel.cast<double>());
      }
    }
    return ref_.back().vel.cast<double>();
  }

  double Fit(double tau) const
  {
    if (gnss_.empty()) return 1e9;
    const double t0 = gnss_[gnss_.size() / 2].t_s;
    const int n = static_cast<int>(gnss_.size());
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(6 * n, 9);
    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(6 * n, 3);
    int row = 0;
    for (const auto &s : gnss_)
    {
      const double t_meas = s.t_s - tau;
      const double dt = s.t_s - t0;
      const Eigen::Vector3d p = InterpRefPos(t_meas);
      const Eigen::Vector3d v = InterpRefVel(t_meas);
      for (int axis = 0; axis < 3; ++axis)
      {
        const int rp = row + axis;
        A(rp, 0 + axis) = 1.0 / 1.5;
        A(rp, 3 + axis) = dt / 1.5;
        A(rp, 6 + axis) = -0.5 * t_meas * t_meas / 1.5;
        B(rp, axis) = (static_cast<double>(s.pos(axis)) - p(axis)) / 1.5;
        const int rv = row + 3 + axis;
        A(rv, 3 + axis) = 1.0 / 0.05;
        A(rv, 6 + axis) = -t_meas / 0.05;
        B(rv, axis) = (static_cast<double>(s.vel(axis)) - v(axis)) / 0.05;
      }
      row += 6;
    }
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const Eigen::MatrixXd C = svd.solve(B);
    return (A * C - B).norm();
  }

  float Sharpness(float tau) const
  {
    const double center = Fit(tau);
    const double left = Fit(std::max(0.0f, tau - 0.02f));
    const double right = Fit(tau + 0.02f);
    const double side = 0.5 * (left + right);
    return (center > 1e-9) ? static_cast<float>((side - center) / center) : 0.0f;
  }

  std::deque<InsGnssDelaySample> ref_;
  std::deque<InsGnssDelaySample> gnss_;
  InsGnssDelayAutoResult last_result_;
};
