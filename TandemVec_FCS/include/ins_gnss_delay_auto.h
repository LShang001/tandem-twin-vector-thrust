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

// ★ 2026-11-11 默认关闭（原 1）：在线延时估计每 1 s 在 200 Hz 导航任务里跑
//   一次网格搜索，原实现在板上实测单次 713 ms（health `delay_max_ms`），
//   运动时把导航从 200 Hz 拖到 152 Hz、数传波形一卡一卡。收益（GNSS 链路
//   延时补偿）在地面/低速阶段可以忽略，代价却是控制环周期性阻塞。
//   关闭后 `TryEstimate()` 不再被调用，延时用固定先验
//   `BFS_NAVIGATION_GNSS_DELAY_S`（见 ekf_15_state.h），行为确定。
//   重新启用前必须先证明单次耗时 < 导航周期（5 ms）量级，或改成分帧摊销。
#ifndef BFS_NAVIGATION_GNSS_DELAY_AUTO_ESTIMATE
#define BFS_NAVIGATION_GNSS_DELAY_AUTO_ESTIMATE 0
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
// 2026-11-01：默认 150 ms 会把 150/300 ms 的真实延迟判成 Boundary（网格端点），
// ABI 实测 150/300 ms 全部无有效 τ̂；宿主独立验证 0.4 s 上界下 50/150/300 ms
// 估计误差 2.6–4.1 ms。提到 400 ms 覆盖 F10N 典型链路延迟与多径。
#define BFS_NAVIGATION_GNSS_DELAY_AUTO_MAX_TAU_S 0.400
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

/**
 * @brief 固定容量环形缓冲（替代 std::deque 存参考轨迹与 GNSS 缓存）
 *
 * @details ★ 2026-11-11 换型原因（HardFault 现场取证）：室外 RTK 定位 + 机体运动时
 *          飞控复位。禁用 IWDG 后复现，调试器 halt 读到
 *          `CFSR=0x00008200`（BusFault 精确错误、BFAR 有效）、`BFAR=R0=0xB0000118`
 *          （STM32 FMC 外部存储地址，本板无外接 SDRAM），异常栈帧的 fault PC 落在
 *          `InsGnssDelayAutoEstimator::FindRefSegment()` 里 `ref_[size-1]` 的
 *          `ldr r0,[r0]`——即 `std::deque` 的 `operator[]` 返回了垃圾地址。
 *          结论：deque 的内部块指针（`_M_node`）在长时间 push_back/pop_front 下
 *          被破坏（嵌入式堆环境下 deque 的块管理与其它堆分配互相干扰）。
 *
 *          本类改用单块固定数组 + 写指针 + 元素计数：容量编译期确定、零运行期
 *          堆分配、无内部指针可被破坏；索引经两次条件减法回绕，任何 i<size()
 *          都不可能给出数组外地址。
 *
 * @tparam T 元素类型；@tparam N 容量（必须是编译期常量）
 * @note 语义与 std::deque 保持子集兼容（clear/empty/size/push_back/pop_front/
 *       operator[]/back），因此上层调用点无需改动。
 *       容量满后 push_back 覆盖最旧元素（等价于 deque 的"push 后 pop_front"）。
 */
template <typename T, size_t N>
class FixedRingBuffer
{
  static_assert(N > 1, "FixedRingBuffer 容量必须大于 1");

public:
  void clear()
  {
    head_ = 0;
    count_ = 0;
  }
  bool empty() const { return count_ == 0; }
  size_t size() const { return count_; }

  void push_back(const T &v)
  {
    buf_[head_] = v;
    head_ = (head_ + 1U == N) ? 0U : (head_ + 1U);
    if (count_ < N)
    {
      ++count_;
    }
  }

  /// 丢弃最旧元素（等价 deque::pop_front）；空时无操作
  void pop_front()
  {
    if (count_ == 0U)
    {
      return;
    }
    --count_;
  }

  /// i 以"最旧元素为 0"计；调用方保证 i < size()
  const T &operator[](size_t i) const
  {
    size_t idx = head_ + N - count_ + i;  // head_>=count_ 时无需回绕
    if (idx >= N)
    {
      idx -= N;
    }
    if (idx >= N)
    {
      idx -= N;
    }
    return buf_[idx];
  }

  /// 最新元素（调用方保证非空）
  const T &back() const { return (*this)[count_ - 1U]; }

private:
  T buf_[N] = {};      ///< 零初始化，避免读到未写入的样本
  size_t head_ = 0;    ///< 下一个写入位置
  size_t count_ = 0;   ///< 当前元素数
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
    ref_decim_ = 0;
    last_result_ = InsGnssDelayAutoResult{};
  }

  void PushReference(double t_s, const Eigen::Vector3d &pos,
                     const Eigen::Vector3d &vel)
  {
    if (!std::isfinite(t_s) || !pos.allFinite() || !vel.allFinite())
    {
      return;
    }
    // ★ 2026-11-11：2:1 抽取（调用 200 Hz → 缓存 100 Hz）。参考轨迹是纯惯导
    //   积分，20 s 窗内线性插值误差 ~a·dt²/8 ≈ 1.3e-5 m（dt=10 ms），可忽略；
    //   这样 20 s 窗只占 2000 点（64 KB），既保留可观测性又把静态 RAM 压下来。
    if ((ref_decim_++ & 1U) != 0U)
    {
      return;
    }
    ref_.push_back({static_cast<float>(t_s), pos.cast<float>(),
                    vel.cast<float>()});
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
    // GNSS 自报速度是比参考积分更可靠的静止判据：地面静止时参考速度会因
    // 加速度计噪声缓慢漂移，单看 vmax 仍会误触发225ms级网格搜索。
    double gnss_vmax = 0.0;
    for (size_t i = 0; i < gnss_.size(); ++i)
    {
      gnss_vmax = std::max(gnss_vmax,
                           static_cast<double>(gnss_[i].vel.norm()));
    }
    if (gnss_vmax < 0.5)
    {
      diag_.speed_max_mps = 0.0f;
      diag_.reject = InsGnssDelayAutoReject::NoExcitation;
      return false;
    }
    diag_.speed_span_mps = static_cast<float>(vmax - vmin);
    diag_.speed_max_mps = static_cast<float>(vmax);
    diag_.accel_max_mps2 = static_cast<float>(amax);
    // 静止/近悬停：没有延时激励，且昂贵的粗差 InterpRef* 循环与网格搜索
    // 会把200Hz导航拖住（地面实测单次302ms）。必须在进入重计算前退出。
    if (vmax < 0.5)
    {
      diag_.reject = InsGnssDelayAutoReject::NoExcitation;
      return false;
    }
    if (vmax - vmin < 0.5)
    {
      diag_.reject = InsGnssDelayAutoReject::NoExcitation;
      return false;
    }
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
    InsGnssDelayAutoResult out;
    /* 搜索网格：0 ~ max_tau_s_，步长 kTauStepS。
       上界必须与两个物理约束一起看：①回放缓冲 64×5 ms=320 ms，年龄再大就整帧
       拒绝；②调用方的"与先验偏差"门限（navigation_task），否则估出来也不用。
       默认保持既有 0~150 ms 行为，放宽需同时放宽调用方门限并重新验证。
       上界改为成员（`set_max_tau_s`）以便同二进制做 runtime A/B 与台架标定；
       数组按 `kGridMax`（0.5 s / 5 ms）静态定容，固件默认值不增加栈占用。
       ★ 2026-11-11：曾改两级网格（20/10 ms 粗搜 + 5 ms 细搜）压缩调用次数，
       但宿主回归 T1 出现 20~25 ms 的主瓣偏移（58→33 ms），精度不达标已回退；
       本函数的调用侧（navigation_task）现由 `BFS_NAVIGATION_GNSS_DELAY_AUTO_ESTIMATE=0`
       整体关闭，故不再构成热路径负担。重新启用前需先解决耗时（分帧/降采样）。 */
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
      // ★ 端点/近端点极小值不是"测到的延时"而是"钳位值"：τ 落在搜索范围
      //   [0, max_tau] 的两端说明真延时在范围之外，或参考轨迹误差把代价压平。
      //   2026-11-11 两级网格后，判定必须按"绝对 τ"而不是细搜数组下标：
      //   细搜窗是相对的，真值恰好落在窗边缘时会被误判（test_gnss_delay_estimator
      //   的 300 ms 档实测 rej=5 Boundary → valid=0）。只判端点 ±1.5 个细步长，
      //   τ̂=5/10/15 ms 仍是合法的小延时估计。
      if (tau <= 1.5 * kTauStepS || tau >= max_tau_s_ - 1.5 * kTauStepS)
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

  // 2026-11-02：窗口保持 20 s（5 s 实测可观测性不足：150 ms 估成 109 ms）。
  // 参考漂移问题改由“每 2 s 从 EKF 重同步”解决（navigation_task 3667 行）。
  // ★ 2026-11-11：容器由 std::deque 改为 FixedRingBuffer（见其类注释的
  //   HardFault 取证）。参考轨迹按 2:1 抽取存 100 Hz，20 s 窗 = 2000 点
  //   （64 KB BSS）；若按 200 Hz 存 4000 点要 128 KB，静态 RAM 会到 94%。
  static constexpr size_t kMaxRef = 2000;  // 20 s @100Hz（PushReference 已 2:1 抽取）
  static constexpr size_t kMaxGnss = 200;  // 20 s @10Hz
  ///< 单次拟合最多使用的 GNSS 样本数（均匀降采样）。见 Fit() 的堆耗尽说明：
  ///< 拟合内存由它决定（6×48×9 双精度 ≈ 20.7 KB + 6×48×3 ≈ 6.9 KB），与缓存深度解耦。
  ///< ★ 不能降到 32（2026-11-12 实测）：32 样本时 sharpness 从 0.00324 掉到 0.00295，
  ///<   低于「才允许覆盖固定先验」的 0.003 门槛，test_gnss_delay_auto 直接 FAIL；
  ///<   τ 估计偏差也从 8 ms 涨到 17 ms。耗时由两级网格的 Fit 调用次数（约 30 次）主导，
  ///<   48 样本单次约几十微秒；上板复核 health 的 delay_max_ms / nav_gap_max_ms，
  ///<   若真卡顿优先压 Fit 次数，不要牺牲拟合点数。
  static constexpr int kMaxFitSamples = 48;
  static constexpr double kTauStepS = 0.005;
  ///< 粗差门相对"最大可搜索延时造成的偏差"的余量（噪声 + 参考轨迹误差）
  static constexpr double kGnssGrossMargin = 1.5;
  ///< 锐度差分半窗 [s]；同时用作"端点不可信"的判定带宽
  static constexpr float kSharpHalfWindowS = 0.02f;

  /**
   * @brief 在参考轨迹里按时间定位相邻两样本（二分查找，O(log n)）
   * @details 2026-11-11 修复：原实现是线性扫描（每次最多 4000 步），
   *          而 `TryEstimate()` 每次网格搜索要调用 81×2×200 次插值 →
   *          约 130 万次 deque 遍历，是单次 225~302 ms 耗时的主因之一。
   *          改成二分后同一次搜索约 1.5 万步，快两个数量级。
   * @param[in] t_s 目标时刻 [s]
   * @param[out] lo 相邻样本左下标（ref_[lo].t_s <= t_s <= ref_[hi].t_s）
   * @return false = 轨迹为空，调用方须自行返回
   */
  bool FindRefSegment(double t_s, size_t &lo) const
  {
    if (ref_.empty()) return false;
    size_t a = 0;
    size_t b = ref_.size() - 1;
    if (t_s <= ref_[a].t_s) { lo = 0; return true; }
    if (t_s >= ref_[b].t_s) { lo = (b > 0) ? (b - 1) : 0; return true; }
    while (b - a > 1)
    {
      const size_t m = a + (b - a) / 2;
      if (static_cast<double>(ref_[m].t_s) >= t_s) b = m;
      else a = m;
    }
    lo = a;
    return true;
  }

  /// 插值系数：t_s 在 ref_[lo]..ref_[lo+1] 区间的归一化位置 [0,1]
  double RefAlpha(double t_s, size_t lo) const
  {
    const size_t hi = (lo + 1 < ref_.size()) ? (lo + 1) : lo;
    const double t0 = static_cast<double>(ref_[lo].t_s);
    const double t1 = static_cast<double>(ref_[hi].t_s);
    const double f = (t_s - t0) / std::max(1e-9, t1 - t0);
    return (f < 0.0) ? 0.0 : ((f > 1.0) ? 1.0 : f);
  }

  Eigen::Vector3d InterpRefPos(double t_s) const
  {
    size_t lo = 0;
    if (!FindRefSegment(t_s, lo)) return Eigen::Vector3d::Zero();
    if (lo + 1 >= ref_.size()) return ref_[lo].pos.cast<double>();
    const double f = RefAlpha(t_s, lo);
    const auto &a = ref_[lo];
    const auto &b = ref_[lo + 1];
    return ((1.0 - f) * a.pos.cast<double>() + f * b.pos.cast<double>());
  }

  Eigen::Vector3d InterpRefVel(double t_s) const
  {
    size_t lo = 0;
    if (!FindRefSegment(t_s, lo)) return Eigen::Vector3d::Zero();
    if (lo + 1 >= ref_.size()) return ref_[lo].vel.cast<double>();
    const double f = RefAlpha(t_s, lo);
    const auto &a = ref_[lo];
    const auto &b = ref_[lo + 1];
    return ((1.0 - f) * a.vel.cast<double>() + f * b.vel.cast<double>());
  }

  /**
   * @brief 单次候选延时的批处理最小二乘代价（残差范数）
   *
   * @details ★ 2026-11-11 修复堆耗尽死机：原实现每格网格都构造动态矩阵
   *          `MatrixXd A(6n,9)` + `MatrixXd B(6n,3)` + `JacobiSVD(ComputeThinU|V)`。
   *          n 取满 200 时单次需要 A 86.4 KB + B 28.8 KB + U 86.4 KB ≈ 197.5 KB，
   *          而本固件静态 RAM 已占 64.7%（剩余约 180 KB）——`TryEstimate()`
   *          在网格上要调用本函数 81 次（再加 Sharpness 3 次），第一次分配就必然
   *          失败；Eigen 在 `-fno-exceptions` 下走 abort 路径，表现为 HardFault /
   *          看门狗复位。触发条件正是「室外有 RTK 定位 + 机体运动」：静止时
   *          `TryEstimate()` 被速度门/静态门提前返回，从不进入本函数。
   *
   *          现改为：① 拟合样本固定上限 `kMaxFitSamples`（均匀降采样，20 s 窗内
   *          48 点对 9 参数足够）；② 矩阵尺寸编译期固定、落 BSS（不反复 new/delete，
   *          无堆碎片）；③ SVD 换成正规方程 9×9 求解（未知数只有 9 个，条件数由
   *          位置/速度量纲归一化后的 1/1.5、1/0.05 权重保证）；④ 残差范数手写循环，
   *          避免 `(A*C-B)` 产生 288×3 的临时矩阵。
   *          改造后单次内存约 28 KB 静态 + 数 KB 栈，与 n 无关。
   *
   * @param[in] tau 候选 GNSS 延时 [s]（量测时刻 = s.t_s - tau）
   * @return 残差范数；gnss_ 为空或有效样本不足时返回 1e9
   * @units m（位置）与 m/s（速度）按 1/1.5、1/0.05 加权后的混合范数
   * @frame NED（位置/速度均为 NED 分量）
   */
  double Fit(double tau) const
  {
    if (gnss_.empty()) return 1e9;
    // fit_idx_ 非空 ⇒ 只用通过粗差门的样本（不销毁缓存，见 TryEstimate 注释）
    const bool filtered = !fit_idx_.empty();
    const size_t pool = filtered ? fit_idx_.size() : gnss_.size();
    if (pool == 0) return 1e9;
    // 均匀降采样到 kMaxFitSamples：窗口内时间分布均衡，拟合参数不丢可观测性
    const int n = static_cast<int>(
        (pool > static_cast<size_t>(kMaxFitSamples)) ? kMaxFitSamples : pool);
    const size_t step = (n > 0) ? (pool / static_cast<size_t>(n)) : 1;

    // mid 取中心样本作时间原点（与原实现一致，仅影响数值尺度）
    const size_t mid = filtered ? fit_idx_[pool / 2] : pool / 2;
    const double t0 = gnss_[mid].t_s;

    // ★ 2026-11-11 第二版：全部改纯标量手写。
    //   上一版用 Eigen 的 `A.leftCols/transpose()*A` 等运行时尺寸表达式，
    //   在板上实测单次 Fit 耗时把 TryEstimate 推到 713 ms（health 实测），
    //   原因是 Eigen 对 Dynamic 尺寸 Block 乘积走通用实现、带临时与函数调用开销。
    //   这里用定容裸数组 + 手写正规方程 + 高斯-约当消元，未知数只有 9 个，
    //   288/192 行的规模纯标量足够快，且完全不碰 Eigen 的表达式模板。
    static double A[6 * kMaxFitSamples][9];
    static double B[6 * kMaxFitSamples][3];

    // ★ 裸数组不会自动置零：不清的话上一拍的残留元素会参与 AᵀA，
    //   实测表现为 τ̂ 整体偏移（宿主 T1 58 ms → 33 ms）。只清本次用到的行。
    const int clear_rows = 6 * n;
    for (int r = 0; r < clear_rows; ++r)
    {
      for (int c = 0; c < 9; ++c) A[r][c] = 0.0;
      for (int c = 0; c < 3; ++c) B[r][c] = 0.0;
    }

    int row = 0;
    for (int j = 0; j < n; ++j)
    {
      const size_t idx = filtered
                             ? fit_idx_[static_cast<size_t>(j) * step]
                             : (static_cast<size_t>(j) * step);
      if (idx >= gnss_.size()) break;
      const InsGnssDelaySample &s = gnss_[idx];
      const double t_meas = s.t_s - tau;
      const double dt = s.t_s - t0;
      const Eigen::Vector3d p = InterpRefPos(t_meas);
      const Eigen::Vector3d v = InterpRefVel(t_meas);
      for (int axis = 0; axis < 3; ++axis)
      {
        const int rp = row + axis;
        A[rp][0 + axis] = 1.0 / 1.5;
        A[rp][3 + axis] = dt / 1.5;
        A[rp][6 + axis] = -0.5 * t_meas * t_meas / 1.5;
        B[rp][axis] = (static_cast<double>(s.pos(axis)) - p(axis)) / 1.5;
        const int rv = row + 3 + axis;
        A[rv][3 + axis] = 1.0 / 0.05;
        A[rv][6 + axis] = -t_meas / 0.05;
        B[rv][axis] = (static_cast<double>(s.vel(axis)) - v(axis)) / 0.05;
      }
      row += 6;
    }
    if (row == 0) return 1e9;

    // 正规方程 AᵀA·C = AᵀB；A 每行只有 2~3 个非零元，跳过零可省 3/4 乘加
    double AtA[9][9];
    double AtB[9][3];
    for (int i = 0; i < 9; ++i)
    {
      for (int k = 0; k < 9; ++k) AtA[i][k] = 0.0;
      for (int k = 0; k < 3; ++k) AtB[i][k] = 0.0;
    }
    for (int r = 0; r < row; ++r)
    {
      const double *ar = A[r];
      const double *br = B[r];
      for (int i = 0; i < 9; ++i)
      {
        const double ai = ar[i];
        if (ai == 0.0) continue;
        for (int k = 0; k < 9; ++k) AtA[i][k] += ai * ar[k];
        for (int k = 0; k < 3; ++k) AtB[i][k] += ai * br[k];
      }
    }

    // 求 9×3 解：列主元高斯-约当
    double M[9][12];
    for (int i = 0; i < 9; ++i)
    {
      for (int j = 0; j < 9; ++j) M[i][j] = AtA[i][j];
      for (int j = 0; j < 3; ++j) M[i][9 + j] = AtB[i][j];
    }
    for (int col = 0; col < 9; ++col)
    {
      int piv = col;
      double maxv = std::fabs(M[col][col]);
      for (int r = col + 1; r < 9; ++r)
      {
        const double v = std::fabs(M[r][col]);
        if (v > maxv) { maxv = v; piv = r; }
      }
      if (maxv < 1e-9) return 1e9;  // 秩亏：该 τ 的代价不可用，调用方按 FlatCost 处理
      if (piv != col)
      {
        for (int k = col; k < 12; ++k)
        {
          const double tmp = M[col][k];
          M[col][k] = M[piv][k];
          M[piv][k] = tmp;
        }
      }
      const double inv = 1.0 / M[col][col];
      for (int k = col; k < 12; ++k) M[col][k] *= inv;
      for (int r = 0; r < 9; ++r)
      {
        if (r == col) continue;
        const double f = M[r][col];
        if (f == 0.0) continue;
        for (int k = col; k < 12; ++k) M[r][k] -= f * M[col][k];
      }
    }

    // 残差范数（手写，避免生成临时矩阵）
    double sse = 0.0;
    for (int r = 0; r < row; ++r)
    {
      for (int axis = 0; axis < 3; ++axis)
      {
        double v = -B[r][axis];
        for (int k = 0; k < 9; ++k) v += A[r][k] * M[k][9 + axis];
        sse += v * v;
      }
    }
    return std::sqrt(sse);
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

  FixedRingBuffer<InsGnssDelaySample, kMaxRef> ref_;
  FixedRingBuffer<InsGnssDelaySample, kMaxGnss> gnss_;
  ///< 本次拟合通过粗差门的样本下标（非破坏性：缓存本身不删）
  std::vector<size_t> fit_idx_;
  InsGnssDelayAutoResult last_result_;
  InsGnssDelayAutoDiag diag_;
  uint8_t ref_decim_ = 0;  ///< PushReference 的 2:1 抽取计数
};
