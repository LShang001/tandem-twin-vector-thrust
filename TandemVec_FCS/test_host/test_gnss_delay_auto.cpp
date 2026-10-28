/**
 * @file    test_gnss_delay_auto.cpp
 * @module  固件/宿主回归测试（fw-tests：无 PPS 的 GNSS 延时在线估计器）
 * @brief   机动窗口内验证延时估计收敛、野值剔除与 GNSS 失锁拒绝
 * @details 被测：`include/ins_gnss_delay_auto.h`（`InsGnssDelayAutoEstimator`：
 *          `PushGnss()`/`PushReference()`/`TryEstimate()`/`last_result()`）。
 *          输入：60 s、200 Hz 参考轨迹（真值 + 0.196 m/s² 加计零偏），
 *          GNSS 10 Hz、位置 1.5 m / 速度 0.05 m/s 噪声，真实延时默认 50 ms，
 *          机动段为 0–5 s 北向 +2、10–15 s 东向 +2、20–25 s 天向 -1、25–30 s 北向 -2；
 *          处理：只喂 0–30 s 机动窗口（之后匀速不可观测），TryEstimate(200, 20)。
 *          输出：估计有效、|τ̂−τ|<20 ms、sharpness≥0.003，且野值/失锁分支正确拒绝。
 *          运行：g++ -std=c++17 -Iinclude -Ilib/eigen/src test_host/test_gnss_delay_auto.cpp
 *          可选 argv：seed、true_delay、outlier_frac（默认 20261022/0.050/0.0）。
 * @theory  批处理对齐：对候选 τ 用“参考轨迹在 t−τ 的预测”拟合 GNSS 量测，
 *          残差最小者即延时；曲率（机动）是 τ 可观测的必要条件。
 *          @ref REF-120 §9.6 (9.6.9) p.298（速度/位置递推）、
 *          @ref REF-121 §2.2.1 p.33（量测时标假设）
 * @frame   NED（位置 m、速度 m/s），参考轨迹为纯 IMU 航位推算
 * @units   延时 s/ms，位置 m，速度 m/s，加计零偏 m/s²，sharpness 为残差曲率量
 * @note    20 ms 门限 = 候选网格 5 ms 的 4 倍；sharpness 门限 0.003 用于拒绝
 *          “平底残差”（匀速段所有 τ 残差相同，估计不可信）。
 *          历史事故：固件开关 `DISABLE_GNSS_DELAY_REPLAY` 会拒绝全部 GNSS，
 *          与本估计器不是同一路径，不能互相替代。
 * @test    T1 机动窗口估计收敛；T2 100 m/50 m/s 野值被剔除后仍有效；
 *          T3 GNSS 停在 20 s（失锁）时 TryEstimate 返回 false；
 *          T4 导弹级速度（404 m/s 弹道）真延时 100 ms 命中且可应用；
 *          T5 同窗口 400 m 真粗差在自适应门限下仍被剔除；
 *          T6 不变量：判为端点钳位(Boundary) 的结果不得 valid/不得带非零锐度；
 *          T7 非破坏性粗差门：45 s 高速段后缓存仍持有满窗样本
 *          T8 拟合窗缺口门：重捕获后窗内跨缺口必须 WindowGap 拒绝，缺口滑出后自动放行
 *          T10 运行时网格上界：真延时 200 ms 在默认 150 ms 网格上必被判端点，
 *             抬到 300 ms 后必须命中（锁 set_max_tau_s 真的改变搜索范围）
 *          T9 未建：硬重锚只移动 EKF 状态，估计器比较的是"纯 IMU 参考↔GNSS"，
 *          其输入不受重锚影响 ⇒ 该门在 SIL 中预期为 no-op，由 A/B 经验裁决后再决定去留
 * @see     TandemVec_FCS/include/ins_gnss_delay_auto.h、src/navigation_task.cpp（默认开关）
 */
// 无 PPS 延时估计模块（ins_gnss_delay_auto.h）宿主回归
#include <cstdio>
#include <cmath>
#include <random>
#include <vector>
#include <Eigen/Dense>
#include "ins_gnss_delay_auto.h"

// 导弹级参考轨迹：1 s 助推 6.4 g（沿 30° 仰角）+ 之后自由飞行（a = g）。
// 与固件 `InsInertialReference` 同语义（纯 IMU 航位推算，位置/速度 NED、z 下）。
static void HdTruth(double t, Eigen::Vector3d *p, Eigen::Vector3d *v)
{
  const double g = 9.80665;
  const double gamma0 = 30.0 * 3.14159265358979 / 180.0;
  const Eigen::Vector3d u(std::cos(gamma0), 0.0, -std::sin(gamma0));
  const Eigen::Vector3d ab = u * (6.4 * g);
  const Eigen::Vector3d ag(0.0, 0.0, g);
  const double tb = (t < 1.0) ? t : 1.0;
  const Eigen::Vector3d vb = ab * tb;
  const Eigen::Vector3d pb = 0.5 * ab * tb * tb + Eigen::Vector3d(0, 0, -50.0);
  if (t <= 1.0) { *p = pb; *v = vb; return; }
  const double s = t - 1.0;
  *p = pb + vb * s + 0.5 * ag * s * s;
  *v = vb + ag * s;
}

// 流式喂一个高动态窗口：200 Hz 参考 + 10 Hz GNSS（真延时 tau、可选在 t_outlier
// T10 用：>0 时覆盖搜索网格上界 [s]（等价 SIL 的 --delay-grid-max-ms）。
static double g_max_tau_override = 0.0;

// 注入 +400 m/+50 m/s 台阶），每 `cadence_s` 调一次 TryEstimate。
// cb(t, result, diag) 每次调用后回调；返回最后一次调用的 valid。
template <typename Fn>
static bool HdRun(double tau, double t_end, double t_outlier, double cadence,
                  unsigned seed, Fn cb)
{
  constexpr double dt = 0.005;
  std::mt19937 rng(seed);
  std::normal_distribution<double> npos(0.0, 1.5);
  std::normal_distribution<double> nvel(0.0, 0.05);
  InsGnssDelayAutoEstimator est;
  if (g_max_tau_override > 0.0)
  {
    est.set_max_tau_s(g_max_tau_override);
  }
  std::vector<Eigen::Vector3d> ps, vs;
  const int steps = static_cast<int>(t_end / dt);
  bool last_ok = false;
  for (int k = 0; k <= steps; ++k)
  {
    const double t = k * dt;
    Eigen::Vector3d p, v;
    HdTruth(t, &p, &v);
    ps.push_back(p);
    vs.push_back(v);
    est.PushReference(t, p, v);
    if ((k % 20) == 0 && t > 0.0)
    {
      const int ti = static_cast<int>(
          std::lround(std::max(0.0, t - tau) / dt));
      Eigen::Vector3d z = ps[ti] + Eigen::Vector3d(npos(rng), npos(rng), npos(rng));
      Eigen::Vector3d zv = vs[ti] + Eigen::Vector3d(nvel(rng), nvel(rng), nvel(rng));
      if (t_outlier > 0.0 && std::fabs(t - t_outlier) < 0.05)
      {
        z += Eigen::Vector3d(400.0, 400.0, 400.0);
        zv += Eigen::Vector3d(50.0, 50.0, 50.0);
      }
      est.PushGnss(t, z, zv);
    }
    if ((k % static_cast<int>(cadence / dt)) == 0 && t > 3.0)
    {
      last_ok = est.TryEstimate();
      cb(t, est.last_result(), est.diag());
    }
  }
  return last_ok;
}

// 同 HdRun，但在 [gap_start, gap_end) 内不喂 GNSS（模拟失联），并可设拟合窗缺口门。
// 用途：检验"重捕获后窗内仍跨着缺口"这一危害段是否被 `WindowGap` 挡住。
template <typename Fn>
static bool HdRunGap(double tau, double t_end, double gap_start,
                     double gap_end, double gap_limit, double cadence,
                     unsigned seed, Fn cb)
{
  constexpr double dt = 0.005;
  std::mt19937 rng(seed);
  std::normal_distribution<double> npos(0.0, 1.5);
  std::normal_distribution<double> nvel(0.0, 0.05);
  InsGnssDelayAutoEstimator est;
  est.set_max_gap_s(static_cast<float>(gap_limit));
  std::vector<Eigen::Vector3d> ps, vs;
  const int steps = static_cast<int>(t_end / dt);
  bool last_ok = false;
  for (int k = 0; k <= steps; ++k)
  {
    const double t = k * dt;
    Eigen::Vector3d p, v;
    HdTruth(t, &p, &v);
    ps.push_back(p);
    vs.push_back(v);
    est.PushReference(t, p, v);
    const bool in_gap = (t >= gap_start && t < gap_end);
    if ((k % 20) == 0 && t > 0.0 && !in_gap)
    {
      const int ti = static_cast<int>(
          std::lround(std::max(0.0, t - tau) / dt));
      Eigen::Vector3d z = ps[ti] + Eigen::Vector3d(npos(rng), npos(rng), npos(rng));
      Eigen::Vector3d zv = vs[ti] + Eigen::Vector3d(nvel(rng), nvel(rng), nvel(rng));
      est.PushGnss(t, z, zv);
    }
    if ((k % static_cast<int>(cadence / dt)) == 0 && t > 3.0)
    {
      last_ok = est.TryEstimate();
      cb(t, est.last_result(), est.diag());
    }
  }
  return last_ok;
}

int main(int argc, char **argv)
{
  constexpr double dt = 0.005;
  constexpr double T = 60.0;
  const unsigned seed = (argc > 1) ? static_cast<unsigned>(std::atoi(argv[1])) : 20261022;
  const double true_delay = (argc > 2) ? std::atof(argv[2]) : 0.050;
  const double outlier_frac = (argc > 3) ? std::atof(argv[3]) : 0.0;
  constexpr double bias = 0.196;
  const int steps = static_cast<int>(T / dt);
  std::mt19937 rng(seed);
  std::normal_distribution<double> npos(0.0, 1.5);
  std::normal_distribution<double> nvel(0.0, 0.05);
  std::uniform_real_distribution<double> unif(0.0, 1.0);

  std::vector<Eigen::Vector3d> p_true(steps + 1), v_true(steps + 1);
  std::vector<Eigen::Vector3d> p_ref(steps + 1), v_ref(steps + 1);
  Eigen::Vector3d p(0, 0, 0), v(30, 0, 0), pr = p, vr = v;

  for (int k = 0; k < steps; ++k)
  {
    const double t = k * dt;
    Eigen::Vector3d a(0, 0, 0);
    if (t < 5.0) a(0) = 2.0;
    else if (t >= 10.0 && t < 15.0) a(1) = 2.0;
    else if (t >= 20.0 && t < 25.0) a(2) = -1.0;
    else if (t >= 25.0 && t < 30.0) a(0) = -2.0;
    v += a * dt;
    p += v * dt;
    Eigen::Vector3d am = a + Eigen::Vector3d(bias, 0, 0);
    vr += am * dt;
    pr += vr * dt;
    p_true[k + 1] = p;
    v_true[k + 1] = v;
    p_ref[k + 1] = pr;
    v_ref[k + 1] = vr;
  }

  InsGnssDelayAutoEstimator est;
  const int every = 20; // 10 Hz
  for (int k = 0; k < steps; k += every)
  {
    const double t = k * dt;
    if (t > 30.0) break; // 只在机动窗口内估计；50–60 s 已匀速、不可观测
    const double tm = std::max(0.0, t - true_delay);
    const int ti = static_cast<int>(std::lround(tm / dt));
    Eigen::Vector3d z = p_true[ti] +
                        Eigen::Vector3d(npos(rng), npos(rng), npos(rng));
    Eigen::Vector3d zv = v_true[ti] +
                         Eigen::Vector3d(nvel(rng), nvel(rng), nvel(rng));
    if (unif(rng) < outlier_frac) {
      z += Eigen::Vector3d(100.0, 100.0, 100.0);
      zv += Eigen::Vector3d(50.0, 50.0, 50.0);
    }
    est.PushGnss(t, z, zv);
  }

  // 参考轨迹按 200 Hz 缓存（与固件钩子一致）
  for (int k = 0; k <= steps; ++k) {
    if (k * dt > 30.0) break;
    est.PushReference(k * dt, p_ref[k], v_ref[k]);
  }

  const bool ok_est = est.TryEstimate(200, 20);
  const auto &r = est.last_result();
  std::printf("valid=%d tau=%.0f ms residual=%.3f sharpness=%.3f\n",
              ok_est ? 1 : 0, r.tau_s * 1000.0f, r.residual_mps, r.sharpness);
  // 野值：在正常估计后注入一个 100 m / 50 m/s 的坏样本，应被剔除。
  InsGnssDelayAutoEstimator outlier_est = est;
  outlier_est.PushReference(30.05, p_ref[static_cast<int>(30.05 / dt)],
                            v_ref[static_cast<int>(30.05 / dt)]);
  outlier_est.PushGnss(30.05, p_true[static_cast<int>(30.05 / dt)] +
                                  Eigen::Vector3d(100, 100, 100),
                       v_true[static_cast<int>(30.05 / dt)] +
                           Eigen::Vector3d(50, 50, 50));
  const bool outlier_ok = outlier_est.TryEstimate(200, 20);
  std::printf("outlier_ok=%d\n", outlier_ok ? 1 : 0);

  // GNSS 失锁：参考继续、GNSS 停在 20 s，估计必须拒绝。
  InsGnssDelayAutoEstimator stale;
  for (int k = 0; k <= steps; ++k) {
    if (k * dt > 30.0) break;
    stale.PushReference(k * dt, p_ref[k], v_ref[k]);
  }
  for (int k = 0; k < steps; k += every) {
    const double t = k * dt;
    if (t > 20.0) break;
    const double tm = std::max(0.0, t - true_delay);
    const int ti = static_cast<int>(std::lround(tm / dt));
    stale.PushGnss(t, p_true[ti], v_true[ti]);
  }
  const bool stale_rejected = !stale.TryEstimate(200, 20);
  std::printf("stale_rejected=%d\n", stale_rejected ? 1 : 0);

  // ---- T4/T5/T6 高动态（导弹级速度）2026-10-27 ------------------------
  // T4：真延时 100 ms 落在默认网格内部、且助推台阶仍在 20 s 缓存内 →
  //     估计必须命中并可被应用（sharp≥0.003 才允许覆盖固定先验）。
  InsGnssDelayAutoResult t4r{};
  InsGnssDelayAutoDiag t4d{};
  HdRun(0.100, 15.0, -1.0, 1.0, 20261027u,
        [&](double, const InsGnssDelayAutoResult &r,
            const InsGnssDelayAutoDiag &d) { t4r = r; t4d = d; });
  const bool hd_est = t4r.valid && t4d.reject == InsGnssDelayAutoReject::None;
  std::printf("hd_t4: valid=%d tau=%.1f ms sharp=%.3f res=%.1f rej=%u ng=%u\n",
              t4r.valid ? 1 : 0, t4r.tau_s * 1000.0f, t4r.sharpness,
              t4r.residual_mps, static_cast<unsigned>(t4d.reject), t4d.n_gnss);

  // T5：同一窗口注入 +400 m/+50 m/s 台阶（真粗差）。门限取
  //     max(50, 1.5·v·τ_max)，400 m 在两档门限之外 → 必须仍被剔除且不污染 τ̂。
  InsGnssDelayAutoResult t5r{};
  InsGnssDelayAutoDiag t5d{};
  HdRun(0.100, 15.0, 14.9, 1.0, 20261027u,
        [&](double, const InsGnssDelayAutoResult &r,
            const InsGnssDelayAutoDiag &d) { t5r = r; t5d = d; });
  const bool hd_outlier_caught =
      (t5d.n_erased >= 1) && t5r.valid &&
      std::fabs(t5r.tau_s - 0.100) <= 0.03f;
  std::printf("hd_t5: erased=%u valid=%d tau=%.1f ms\n", t5d.n_erased,
              t5r.valid ? 1 : 0, t5r.tau_s * 1000.0f);

  // T6 不变量：被判为"端点钳位"(Boundary) 的结果不得 valid、不得带非零锐度。
  //     旧实现里 τ̂=0 的单侧差分给出 0.015~1.05 的假锐度并穿过 0.003 应用门限，
  //     等于把 15 ms 先验换成错误的 0 ms；而亚网格插值又能把 best_c=29/30 推到
  //     147.5 ms 绕过"恰好在端点"的判据 ⇒ 判据含端点 ±1 格。与编译期网格无关。
  bool boundary_safe = true;
  int boundary_seen = 0;
  HdRun(0.200, 45.0, -1.0, 5.0, 20261027u,
        [&](double, const InsGnssDelayAutoResult &r,
            const InsGnssDelayAutoDiag &d) {
          if (d.reject == InsGnssDelayAutoReject::Boundary)
          {
            ++boundary_seen;
            if (r.valid || r.sharpness > 0.0f) boundary_safe = false;
          }
        });
  std::printf("hd_t6: boundary_hits=%d safe=%d\n", boundary_seen,
              boundary_safe ? 1 : 0);

  // T7 非破坏性粗差门（2026-10-27）：粗差门只能"本次排除"，不能销毁缓存。
  //     旧实现在高速段每秒永久删 ~20 个样本，45 s 后缓存只剩 42 个、
  //     之后补不回来 ⇒ 估计器自我饿死。这里用固定 50 m 门（ADAPTIVE_FDI=0）
  //     跑完整 45 s，要求缓存仍持有满窗样本。剔除能力本身由 T5 锁定，
  //     本条只锁"不销毁历史"（自适应门下剔除数为 0 是正确行为，不能当失败）。
  InsGnssDelayAutoDiag t7d{};
  HdRun(0.200, 45.0, -1.0, 1.0, 20261027u,
        [&](double, const InsGnssDelayAutoResult &,
            const InsGnssDelayAutoDiag &d) { t7d = d; });
  const bool history_kept = (t7d.n_gnss_total >= 190);
  std::printf("hd_t7: total=%u kept=%u erased=%u rej=%u\n", t7d.n_gnss_total,
              t7d.n_gnss, t7d.n_erased, static_cast<unsigned>(t7d.reject));

  /* T8 拟合窗缺口门（2026-10-27）：`GnssStale` 只挡得住缺口**期间**；重捕获后
  //     20 s 窗仍跨着缺口，τ̂ 是"缺口前历史 + 缺口后新息"的混合产物。SIL 实测
  //     该段一旦被应用会让 20/20 seed 位置 RMSE 变差（纯失联 +0.25/+0.66 m，
  //     `outage_veto_ab_2026-10-27.json`）。本台架锁三件事：
  //     ① 门=0.5 s 时危害段必须 reject==WindowGap 且 !valid（门真的会 fire）；
  //     ② 缺口滑出 20 s 窗后必须自动放行（否则就是 M4 式自我饿死）；
  //     ③ 门关闭时同一时段本台架也不得 valid —— 探针实测该时段被 `Boundary`
  //        自拒（τ̂ 落在网格端点），说明本门在该台架**不额外剥夺**任何可用估计；
  //        它的实际价值由 SIL 证明（那里 τ̂ 落在内部、锐度达标、会被应用）。 */
  bool gap_blocked = false;
  bool gap_released = false;
  float gap_measured = 0.0f;
  HdRunGap(0.100, 50.0, 20.0, 25.0, 0.5, 1.0, 20261027u,
           [&](double t, const InsGnssDelayAutoResult &,
               const InsGnssDelayAutoDiag &d)
           {
             // 判据只看 `diag().reject`：`last_result()` 在拒绝时不刷新，
             // 拿它的 valid 当"本次是否成功"会读到缺口前的陈旧值。
             if (t > 26.0 && t < 30.0 &&
                 d.reject == InsGnssDelayAutoReject::WindowGap)
             {
               gap_blocked = true;
               gap_measured = d.max_gap_s;
             }
             if (t > 46.5 && d.reject == InsGnssDelayAutoReject::None)
             {
               gap_released = true;
             }
           });
  bool hazard_self_rejected = true;
  HdRunGap(0.100, 50.0, 20.0, 25.0, 0.0, 1.0, 20261027u,
           [&](double t, const InsGnssDelayAutoResult &,
               const InsGnssDelayAutoDiag &d)
           {
             if (t > 25.5 && t < 39.5 && d.reject == InsGnssDelayAutoReject::None)
             {
               hazard_self_rejected = false;
             }
           });
  bool gap_gate_ok =
      gap_blocked && gap_released && hazard_self_rejected &&
      std::fabs(gap_measured - 5.0f) < 0.5f;
  std::printf("hd_t8: blocked=%d gap=%.2f released=%d offarm_safe=%d -> %d\n",
              gap_blocked ? 1 : 0, gap_measured, gap_released ? 1 : 0,
              hazard_self_rejected ? 1 : 0, gap_gate_ok ? 1 : 0);

  /* T10 运行时网格上界（2026-10-28）：真延时 200 ms > 默认网格上界 150 ms 时，
  //     代价极小值落在端点 ⇒ 只能报 Boundary（T6 已锁"端点不得当测量用"）。
  //     把上界抬到 300 ms 后同一个真延时必须被命中。这条锁的是
  //     `set_max_tau_s()` 真的改变搜索范围（SIL `--delay-grid-max-ms` 的前置），
  //     并再次确认"大延迟估不出"首先是搜索范围问题、其次才是能不能应用。 */
  bool t10_clamped = false;
  g_max_tau_override = 0.0;
  HdRun(0.200, 45.0, -1.0, 5.0, 20261028u,
        [&](double, const InsGnssDelayAutoResult &,
            const InsGnssDelayAutoDiag &d) {
          if (d.reject == InsGnssDelayAutoReject::Boundary) t10_clamped = true;
        });
  bool t10_wide_hit = false;
  float t10_wide_tau = 0.0f;
  g_max_tau_override = 0.30;
  HdRun(0.200, 45.0, -1.0, 5.0, 20261028u,
        [&](double, const InsGnssDelayAutoResult &r,
            const InsGnssDelayAutoDiag &) {
          if (r.valid && std::fabs(r.tau_s - 0.200f) <= 0.03f)
          {
            t10_wide_hit = true;
            t10_wide_tau = r.tau_s;
          }
        });
  g_max_tau_override = 0.0;
  const bool grid_ok = t10_clamped && t10_wide_hit;
  std::printf("hd_t10: 默认网格钳位=%d  上界300ms 命中 tau=%.1f ms -> %d\n",
              t10_clamped ? 1 : 0, t10_wide_tau * 1000.0f, grid_ok ? 1 : 0);

  /* T11 已试并撤回（2026-10-28，报告 §4.24b）：应用侧从 τ̂ 常数剔除 IMU 群延迟
     （`InsGnssDelayApplyAgeSeconds`）经 20 seeds A/B 否证——拒帧 21→0/1 但位置
     RMSE 反而 16.85→24.1/24.6 m（12–14/20 seed 变差），而"完全不应用"是 6.74 m。
     代价曲线非单调 ⇒ 时变滞后不能被任何常数年龄吸收，故删除该接缝，不留死代码。 */

  const bool pass = ok_est && outlier_ok && stale_rejected &&
                    hd_est && std::fabs(t4r.tau_s - 0.100) <= 0.03f &&
                    hd_outlier_caught && boundary_safe && history_kept &&
                    gap_gate_ok && grid_ok &&
                    std::fabs(r.tau_s - true_delay) <= 0.02f &&
                    r.sharpness >= 0.003f;
  std::printf("[%s] ins_gnss_delay_auto\n", pass ? "PASS" : "FAIL");
  return pass ? 0 : 1;
}
