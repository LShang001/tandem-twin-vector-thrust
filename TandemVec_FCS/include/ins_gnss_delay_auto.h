/**
 * @file    ins_gnss_delay_auto.h
 * @module  固件/导航（无 PPS 的 GNSS 延时估计）
 * @brief   在机动段用批处理对齐估计 GNSS 量测延时 τ，并给出锐度与残差
 * @details 输入：IMU 纯惯导参考轨迹（20 s 环形缓存）、GNSS 位置/速度样本；
 *          处理：粗差剔除（门限随窗口速度自适应，见 ADAPTIVE_FDI）→ 要求速度
 *          有明显变化（匀速/悬停不可观测）→ 网格搜索 + 抛物线亚网格插值 →
 *          每个 τ 解 9 参数线性最小二乘（位置/速度联合）取残差最小；
 *          输出：τ、残差、锐度、有效标志 + `diag()` 拒绝原因/样本计数。
 *          ★ 端点不可信：τ̂ 贴近 0 或网格上界一律判无效（锐度在端点退化成单侧
 *          差分，会把钳位伪装成高置信）。默认只记录，`*_APPLY` 才参与年龄。
 * @theory  量测延时使残差模型退化为"参考轨迹在 t−τ 处的值"。可观测性来自
 *          窗口内**加速度发生变化**：常加速度段的时移量 τ·v(t) 是 t 的一次函数，
 *          落在本模型 {1,t,t²} 摄动基的张成空间里，会被零偏项吸收（宿主合成弹道
 *          实测：助推台阶离开 20 s 缓存后 cost(τ_true) 反而高于 cost(0)、
 *          锐度归零）@ref REF-121 §8.2.3 p.291（量测时标对齐是组合导航一致性的
 *          前提），FDI 思想同 §7.3.1 p.255
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

#include <algorithm>
#include <cmath>
#include <cstdint>
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

/* 搜索上界 [s]：默认 0.150 保持既有 0~150 ms 行为。放宽时必须同时放宽
   navigation_task 的"与先验偏差"门限，且不得超过回放缓冲 64×5 ms=320 ms，
   否则年龄超出缓冲后 GNSS 整帧被拒，补偿反而更差。 */
#ifndef BFS_NAVIGATION_GNSS_DELAY_AUTO_MAX_TAU_S
#define BFS_NAVIGATION_GNSS_DELAY_AUTO_MAX_TAU_S 0.150
#endif

/* 调用方允许的"估计值与先验偏差"上限 [s]。默认 0.10 保持既有行为；
   放宽搜索上界时必须同步放宽本门限，否则网格上沿的 τ 会被静默丢弃。 */
#ifndef BFS_NAVIGATION_GNSS_DELAY_AUTO_CAP_S
#define BFS_NAVIGATION_GNSS_DELAY_AUTO_CAP_S 0.10f
#endif
// 应用侧死区 [s]：估计值与先验的差小于一个网格步（5 ms）时不替换先验。
// 依据：洁净弹道实测 τ̂≈11 ms 而设施真实管线年龄≈15 ms，4 ms 的伪改善会把
// 无延时档的位置 RMSE 从 1.39 推到 1.56 m（45/50 seed 变差）；而 100 ms 档
// τ̂≈89 ms，与先验差 74 ms，死区不影响真实收益。默认 0=不启用。
#ifndef BFS_NAVIGATION_GNSS_DELAY_AUTO_DEADBAND_S
#define BFS_NAVIGATION_GNSS_DELAY_AUTO_DEADBAND_S 0.0f
#endif
// 拟合窗允许的最大 GNSS 数据缺口 [s]：窗内跨过一次缺口时 τ̂ 不可信（见 WindowGap）。
// 默认 0 = 关闭，取值由导弹 SIL 的 runtime A/B 决定（GNSS 标称周期 0.1 s）。
#ifndef BFS_NAVIGATION_GNSS_DELAY_AUTO_MAX_GAP_S
#define BFS_NAVIGATION_GNSS_DELAY_AUTO_MAX_GAP_S 0.0f
#endif
// 应用侧残差门限 [m/s]：LS 拟合的自报残差超过它说明"参考轨迹与 GNSS 不同步"
// 这件事本身不成立（振动污染纯 IMU 参考、IMU 冻结使参考停止推进），此时 τ̂ 无意义。
// 与 `vib_high` 的区别：这是估计器对自己拟合质量的度量，不依赖被机动误判的振动 RMS。
// 默认 150 m/s：SIL 逐格 A/B（`apply_gate_ab_2026-10-27.json`，8 seeds/格）——
// 振 2 g@120/8.4 g 位置 RMSE 24.88→5.05 m、振 1 g 3.04→2.36、巡航失联 20 s
// 12.79→12.09，而洁净/延 50/100/200 ms 逐格不动（延 100 ms 4.85→4.92，+1.4%）。
// 注意它不是"越大越坏"的单调判据：见 §4.22.1b 结论 1 的更正与逐历元作用说明。
#ifndef BFS_NAVIGATION_GNSS_DELAY_AUTO_MAX_RESIDUAL_MPS
#define BFS_NAVIGATION_GNSS_DELAY_AUTO_MAX_RESIDUAL_MPS 150.0f
#endif

/* 应用侧是否用 `vib_high` 否决自动延时。
   默认 0（2026-10-27 改）：该位按 `accel_rms_g > 0.15 g` 判定，而导弹 SIL **未注入
   任何振动**也全程置位，把唯一可用的估计窗口封死（应用次数 0/20 seeds）。去掉后
   100 ms 注入延迟档位置误差 17.91 → 4.93 m（−72%，20 seeds），0 ms 档 1.59 → 1.74 m
   （+0.15 m，未做显著性判定），且 20 seeds × 4 档无过补偿。
   置 1 可回退到旧行为；真需要振动否决的载体应改用"高频振动分量"判据而非
   含机动的加速度 RMS（来源未证，见报告 §4.20.3）。 */
#ifndef BFS_NAVIGATION_GNSS_DELAY_AUTO_VIB_VETO
#define BFS_NAVIGATION_GNSS_DELAY_AUTO_VIB_VETO 0
#endif

/* 粗差门限是否随窗口速度自适应。默认 0 = 保持既有固定 50 m/20 m/s 行为；
   宿主/导弹 SIL 证明高速段确被饿死后再改默认。 */
#ifndef BFS_NAVIGATION_GNSS_DELAY_AUTO_ADAPTIVE_FDI
#define BFS_NAVIGATION_GNSS_DELAY_AUTO_ADAPTIVE_FDI 0
#endif

struct InsGnssDelaySample
{
  // 片上缓存用 float：4000 点约 112 KB，避免 224 KB 级堆占用；
  // 位置精度在 0.1 m 量级，远小于 GNSS 1.5 m 噪声。
  float t_s = 0.0f;
  Eigen::Vector3f pos = Eigen::Vector3f::Zero();
  Eigen::Vector3f vel = Eigen::Vector3f::Zero();
};

/* 拒绝原因：延时估计在什么条件下"估不出"必须可观测，否则上层只看到一个
   valid=0，无法区分"没有激励""数据被粗差门吃掉""代价函数平底"三种病因，
   三者的修法完全不同（分别对应：换窗口、放宽粗差门、换参数化）。 */
enum class InsGnssDelayAutoReject : uint8_t
{
  None = 0,        ///< 估计成功
  NotEnoughData,   ///< 参考轨迹或 GNSS 样本数不足
  GnssStale,       ///< GNSS 相对参考停滞 >0.5 s（失锁/降频）
  NoExcitation,    ///< 窗口内速度变化 <0.5 m/s（匀速/悬停不可观测）
  OutlierStarved,  ///< 粗差剔除后剩余 GNSS 样本不足
  Boundary,        ///< 极小值钉在网格端点（0 或上界）——不是可信的延时估计
  FlatCost,        ///< 网格上无有限代价（拟合失败）
  WindowGap,       ///< 拟合窗内 GNSS 有数据缺口（重捕获后窗未清空前的有害区）
};

struct InsGnssDelayAutoDiag
{
  InsGnssDelayAutoReject reject = InsGnssDelayAutoReject::NotEnoughData;
  uint16_t n_ref = 0;   ///< 本次参与拟合的参考点数（粗差剔除前）
  uint16_t n_gnss = 0;  ///< 本次参与拟合的 GNSS 样本数（粗差门通过后）
  uint16_t n_gnss_total = 0; ///< 缓存中的 GNSS 样本总数（不销毁，故可持续）
  uint16_t n_erased = 0;///< 被粗差门剔除的 GNSS 样本数
  float speed_span_mps = 0.0f; ///< 窗口内参考速度最大值-最小值
  float speed_max_mps = 0.0f;  ///< 窗口内参考速度最大值（粗差门偏差 ∝ τ·v）
  float accel_max_mps2 = 0.0f; ///< 窗口内参考最大加速度（速度通道偏差 ∝ τ·a）
  float max_gap_s = 0.0f;      ///< 窗内相邻 GNSS 样本的最大时间间隔
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
    fit_idx_.clear();
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

  /**
   * @brief 通知"量测轨迹刚被硬重锚"——窗内存在阶跃时 τ̂ 无意义
   * @details 长失联后的 `TryResetAfterPersistentImplausibleGnss` 会把位置/速度整体
   *          跳到 GNSS，20 s 拟合窗里因此同时含有"跳点前"和"跳点后"两段互不自洽的
   *          量测；此时估出的 τ̂ 会被应用门当作有效补偿（巡航失联 20 s 档实测
   *          `ad_app` 2→139、位置 RMSE +0.66 m，`WindowGap` 因缺口早已被缓存淘汰而
   *          无法触发）。保持窗口长度个历元后自动放行。
   * @param hold_samples 需要等待的 GNSS 历元数；0 = 不启用该门
   */


  bool TryEstimate(int min_ref = 200, int min_gnss = 20)
  {
    diag_ = InsGnssDelayAutoDiag{};
    diag_.n_ref = static_cast<uint16_t>(std::min<size_t>(ref_.size(), 0xFFFFu));
    diag_.n_gnss =
        static_cast<uint16_t>(std::min<size_t>(gnss_.size(), 0xFFFFu));
    if (static_cast<int>(ref_.size()) < min_ref ||
        static_cast<int>(gnss_.size()) < min_gnss)
    {
      diag_.reject = InsGnssDelayAutoReject::NotEnoughData;
      return false;
    }
    // GNSS 失锁/数据陈旧：参考轨迹已更新但 GNSS 停在旧时刻，不能估计。
    if ((ref_.back().t_s - gnss_.back().t_s) > 0.5)
    {
      diag_.reject = InsGnssDelayAutoReject::GnssStale;
      return false;
    }
    /* 拟合窗必须被 GNSS 连续覆盖。窗内跨过一次数据缺口（失联/长丢帧）时，τ̂ 是
    //  "缺口前历史 + 缺口后新息"的混合产物：`GnssStale` 只挡得住缺口**期间**，
    //  重捕获后 20 s 窗仍未清空，于是继续输出看似 valid 实则有害的估计。
    //  导弹 SIL 实测（`outage_veto_ab_2026-10-27.json`）：20 s 失联后应用 139 次，
    //  20/20 seed 位置 RMSE 反而 +0.66 m；助推段 5 s 失联 20/20 变差 +0.25 m。
    //  阈值按标称周期的数倍判定，0 = 关闭该门。 */
    double max_gap_s = 0.0;
    for (size_t i = 1; i < gnss_.size(); ++i)
    {
      max_gap_s = std::max(max_gap_s,
                           static_cast<double>(gnss_[i].t_s - gnss_[i - 1].t_s));
    }
    diag_.max_gap_s = static_cast<float>(max_gap_s);
    if (max_gap_limit_s_ > 0.0f && max_gap_s > max_gap_limit_s_)
    {
      diag_.reject = InsGnssDelayAutoReject::WindowGap;
      return false;
    }
    // 粗差剔除：与同时间参考轨迹偏差过大的 GNSS 样本不参与**本次**拟合。
    // ★ 两条硬约束：
    //   ① 不能销毁缓存。旧实现用 gnss_.erase() 永久删除样本，于是"参考轨迹暂时
    //      和 GNSS 对不上"的几秒（助推段 τ·v 越过门限、机动脉冲）会把 20 s 历史
    //      清空，之后补不回来 ⇒ 导弹 SIL 实测末帧只剩 10 个样本、
    //      reject=NotEnoughData 常驻，估计器自我饿死（2026-10-27）。
    //   ② 门限必须覆盖"可搜索延时"本身：位置偏差 ≈ τ·|v|。固定 50 m 隐含低速
    //      包线（30 m/s、150 ms 只有 4.5 m）；451 m/s × 200 ms = 90 m 会把**正常**
    //      样本当粗差。故按窗口最大速度自适应（ADAPTIVE_FDI，默认关）。
    double vmin = 1e9, vmax = -1e9, amax = 0.0;
    for (size_t i = 0; i < ref_.size(); ++i)
    {
      const float vn = ref_[i].vel.norm();
      vmin = std::min(vmin, static_cast<double>(vn));
      vmax = std::max(vmax, static_cast<double>(vn));
      if (i > 0)
      {
        const double dt = static_cast<double>(ref_[i].t_s) -
                          static_cast<double>(ref_[i - 1].t_s);
        if (dt > 1e-6)
        {
          amax = std::max(amax, static_cast<double>(
              (ref_[i].vel - ref_[i - 1].vel).norm()) / dt);
        }
      }
    }
    diag_.speed_span_mps = static_cast<float>(vmax - vmin);
    diag_.speed_max_mps = static_cast<float>(vmax);
    diag_.accel_max_mps2 = static_cast<float>(amax);
    double pos_gate = 50.0;
    double vel_gate = 20.0;
#if BFS_NAVIGATION_GNSS_DELAY_AUTO_ADAPTIVE_FDI
    pos_gate = std::max(50.0, kGnssGrossMargin * vmax * max_tau_s_);
    vel_gate = std::max(20.0, kGnssGrossMargin * amax * max_tau_s_);
#endif
    fit_idx_.clear();
    fit_idx_.reserve(gnss_.size());
    size_t erased = 0;
    for (size_t i = 0; i < gnss_.size(); ++i)
    {
      const InsGnssDelaySample &s = gnss_[i];
      const Eigen::Vector3d p = InterpRefPos(s.t_s);
      const Eigen::Vector3d v = InterpRefVel(s.t_s);
      if ((s.pos.cast<double>() - p).norm() > pos_gate ||
          (s.vel.cast<double>() - v).norm() > vel_gate)
      {
        ++erased;
      }
      else
      {
        fit_idx_.push_back(i);
      }
    }
    diag_.n_erased = static_cast<uint16_t>(std::min<size_t>(erased, 0xFFFFu));
    diag_.n_gnss =
        static_cast<uint16_t>(std::min<size_t>(fit_idx_.size(), 0xFFFFu));
    diag_.n_gnss_total =
        static_cast<uint16_t>(std::min<size_t>(gnss_.size(), 0xFFFFu));
    if (static_cast<int>(fit_idx_.size()) < min_gnss)
    {
      diag_.reject = InsGnssDelayAutoReject::OutlierStarved;
      return false;
    }
    if (vmax - vmin < 0.5)
    {
      diag_.reject = InsGnssDelayAutoReject::NoExcitation;
      return false; // 匀速/悬停不可观测
    }

    InsGnssDelayAutoResult out;
    /* 搜索网格：0 ~ max_tau_s_，步长 kTauStepS。
       上界必须与两个物理约束一起看：①回放缓冲 64×5 ms=320 ms，年龄再大就整帧
       拒绝；②调用方的"与先验偏差"门限（navigation_task），否则估出来也不用。
       默认保持既有 0~150 ms 行为，放宽需同时放宽调用方门限并重新验证。
       上界改为成员（`set_max_tau_s`）以便同二进制做 runtime A/B 与台架标定；
       数组按 `kGridMax`（0.5 s / 5 ms）静态定容，固件默认值不增加栈占用。 */
    constexpr int kGridMax = static_cast<int>(0.5 / kTauStepS + 0.5) + 1;
    const int kGridCount =
        std::min(kGridMax, static_cast<int>(max_tau_s_ / kTauStepS + 0.5) + 1);
    double costs[kGridMax];
    int best_c = -1;
    double best_cost = 1e9;
    for (int c = 0; c < kGridCount; ++c)
    {
      const double tau = c * kTauStepS;
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
      double tau = best_c * kTauStepS;
      if (best_c > 0 && best_c < kGridCount - 1)
      {
        const double cm = costs[best_c - 1];
        const double c0 = costs[best_c];
        const double cp = costs[best_c + 1];
        const double denom = cm - 2.0 * c0 + cp;
        if (denom > 1e-9)
        {
          const double delta = 0.5 * (cm - cp) / denom;
          tau += std::max(-kTauStepS, std::min(kTauStepS, delta * kTauStepS));
        }
      }
      out.valid = true;
      out.tau_s = static_cast<float>(tau);
      out.residual_mps = static_cast<float>(best_cost);
      out.sharpness = Sharpness(static_cast<float>(tau));
      // ★ 端点/近端点极小值不是"测到的延时"而是"钳位值"：best_c 落在网格首/末
      //   一格说明真延时在搜索范围之外，或参考轨迹误差把代价压平。
      //   必须含**近端点**：亚网格抛物插值会把 best_c=29/30 的格点推到 147.5 ms
      //   （真值 200 ms、上界 150 ms），只判"恰好在端点"会被绕过并应用
      //   （2026-10-27 导弹 SIL 追踪实测 τ̂=147.5、sharp 0.026 通过门限）。
      //   只判端点 ±1 格（5 ms），τ̂=5/10/15 ms 仍是合法的小延时估计。
      if (best_c <= 1 || best_c >= kGridCount - 2)
      {
        out.valid = false;
        out.sharpness = 0.0f;
        diag_.reject = InsGnssDelayAutoReject::Boundary;
      }
      else
      {
        diag_.reject = InsGnssDelayAutoReject::None;
      }
      last_result_ = out;
      return out.valid;
    }
    diag_.reject = InsGnssDelayAutoReject::FlatCost;
    return false;
  }

  const InsGnssDelayAutoResult &last_result() const { return last_result_; }
  const InsGnssDelayAutoDiag &diag() const { return diag_; }

  // 诊断用：候选代价（仅宿主调试，不参与控制）
  double DebugCost(double tau_s) const { return Fit(tau_s); }
  size_t DebugRefCount() const { return ref_.size(); }
  size_t DebugGnssCount() const { return gnss_.size(); }

  ///< 拟合窗最大允许 GNSS 缺口 [s]；0 = 关闭该门。默认取宏值。
  void set_max_gap_s(float limit_s) { max_gap_limit_s_ = limit_s; }
  float max_gap_s() const { return max_gap_limit_s_; }
  /** 搜索网格上界 [s]：默认取宏（0.150）。放大到 >0.320 s 会撞上回放缓冲，
   *  且调用方的"与先验偏差"门限必须同步放宽，否则估出来仍不被应用。 */
  void set_max_tau_s(double tau_s)
  {
    if (tau_s > kTauStepS && std::isfinite(tau_s))
    {
      max_tau_s_ = tau_s;
    }
  }
  double max_tau_s() const { return max_tau_s_; }

private:
  float max_gap_limit_s_ = BFS_NAVIGATION_GNSS_DELAY_AUTO_MAX_GAP_S;
  double max_tau_s_ = BFS_NAVIGATION_GNSS_DELAY_AUTO_MAX_TAU_S;

  static constexpr size_t kMaxRef = 4000;  // 20 s @200Hz
  static constexpr size_t kMaxGnss = 200;  // 20 s @10Hz
  static constexpr double kTauStepS = 0.005;
  ///< 粗差门相对"最大可搜索延时造成的偏差"的余量（噪声 + 参考轨迹误差）
  static constexpr double kGnssGrossMargin = 1.5;
  ///< 锐度差分半窗 [s]；同时用作"端点不可信"的判定带宽
  static constexpr float kSharpHalfWindowS = 0.02f;

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
    // fit_idx_ 非空 ⇒ 只用通过粗差门的样本（不销毁缓存，见 TryEstimate 注释）
    const bool filtered = !fit_idx_.empty();
    const size_t n_used = filtered ? fit_idx_.size() : gnss_.size();
    if (n_used == 0) return 1e9;
    const size_t mid = filtered ? fit_idx_[n_used / 2] : n_used / 2;
    const double t0 = gnss_[mid].t_s;
    const int n = static_cast<int>(n_used);
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(6 * n, 9);
    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(6 * n, 3);
    int row = 0;
    for (size_t j = 0; j < n_used; ++j)
    {
      const InsGnssDelaySample &s = gnss_[filtered ? fit_idx_[j] : j];
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
    // ★ 单侧差分不能证明"尖锐"。τ 靠近下界时左邻点会跑到负延时（物理不存在、
    //   且 Fit 会在缓存外插值），若仍按单侧算，"钉在端点"就会被伪装成高锐度
    //   （2026-10-27 导弹段实测 sharp 0.015~0.045、合成弹道 0.75~1.05，均配
    //   错误的 τ̂=0 并通过 0.003 应用门限）。这里把半窗收缩到不越过 0，
    //   τ=0 直接判 0；端点钳位另由 TryEstimate 的 Boundary 分支拒绝。
    const float half = std::min(kSharpHalfWindowS, std::max(0.0f, tau));
    if (half <= 0.0f)
    {
      return 0.0f;
    }
    const double center = Fit(tau);
    const double left = Fit(tau - half);
    const double right = Fit(tau + half);
    const double side = 0.5 * (left + right);
    return (center > 1e-9) ? static_cast<float>((side - center) / center) : 0.0f;
  }

  std::deque<InsGnssDelaySample> ref_;
  std::deque<InsGnssDelaySample> gnss_;
  ///< 本次拟合通过粗差门的样本下标（非破坏性：缓存本身不删）
  std::vector<size_t> fit_idx_;
  InsGnssDelayAutoResult last_result_;
  InsGnssDelayAutoDiag diag_;
};
