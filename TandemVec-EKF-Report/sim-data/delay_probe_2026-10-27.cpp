// 一次性诊断（不入库）：自动 GNSS 延时估计在导弹级速度下的行为
#include <cstdio>
#include <cmath>
#include <random>
#include <vector>
#include "ins_gnss_delay_auto.h"

static const char* RejectName(InsGnssDelayAutoReject r)
{
  switch (r)
  {
    case InsGnssDelayAutoReject::None: return "None";
    case InsGnssDelayAutoReject::NotEnoughData: return "NotEnoughData";
    case InsGnssDelayAutoReject::GnssStale: return "GnssStale";
    case InsGnssDelayAutoReject::NoExcitation: return "NoExcitation";
    case InsGnssDelayAutoReject::OutlierStarved: return "OutlierStarved";
    case InsGnssDelayAutoReject::FlatCost: return "FlatCost";
  }
  return "?";
}

int main(int argc, char **argv)
{
  const double tau_true = (argc > 1) ? std::atof(argv[1]) : 0.200;
  const unsigned seed = (argc > 2) ? (unsigned)std::atoi(argv[2]) : 7u;
  constexpr double dt = 0.005;      // 200 Hz 参考
  constexpr double T = 45.0;
  constexpr double g = 9.80665;
  const double boost_a = 6.4 * g;    // 比力
  const double boost_t = 1.0;
  const double gamma0 = 30.0 * M_PI / 180.0;  // 发射仰角

  std::mt19937 rng(seed);
  std::normal_distribution<double> npos(0.0, 1.5);
  std::normal_distribution<double> nvel(0.0, 0.05);

  InsGnssDelayAutoEstimator est;
  std::vector<double> ts;
  std::vector<Eigen::Vector3d> ps, vs;
  auto truth = [&](double t, Eigen::Vector3d *p, Eigen::Vector3d *v)
  {
    // 助推段：沿初始弹道方向 6.4g 比力；之后自由飞行（a = g 向下）
    Eigen::Vector3d u(std::cos(gamma0), 0.0, -std::sin(gamma0)); // NED, z 下
    Eigen::Vector3d ab = u * boost_a;
    Eigen::Vector3d ag(0.0, 0.0, g);
    double tb = std::min(t, boost_t);
    // 助推末速度/位置
    Eigen::Vector3d vb = ab * tb;
    Eigen::Vector3d pb = 0.5 * ab * tb * tb + Eigen::Vector3d(0, 0, -50.0);
    if (t <= boost_t) { *p = pb; *v = vb; return; }
    const double s = t - boost_t;
    *p = pb + vb * s + 0.5 * ag * s * s;
    *v = vb + ag * s;
  };

  const int steps = (int)(T / dt);
  for (int k = 0; k <= steps; ++k)
  {
    const double t = k * dt;
    Eigen::Vector3d p, v;
    truth(t, &p, &v);
    ts.push_back(t); ps.push_back(p); vs.push_back(v);
    est.PushReference(t, p, v);

    if ((k % 20) == 0 && t > 0.0)
    {
      const int ti = (int)std::lround(std::max(0.0, t - tau_true) / dt);
      Eigen::Vector3d z = ps[ti] + Eigen::Vector3d(npos(rng), npos(rng), npos(rng));
      Eigen::Vector3d zv = vs[ti] + Eigen::Vector3d(nvel(rng), nvel(rng), nvel(rng));
      est.PushGnss(t, z, zv);
    }

    if ((k % 200) == 0 && t > 3.0)
    {
      const bool ok = est.TryEstimate();
      const auto &r = est.last_result();
      const auto &d = est.diag();
      std::printf("t=%4.1f v=%6.1f ok=%d rej=%-13s nref=%4u ng=%3u er=%3u "
                  "span=%7.1f smax=%6.1f tau_hat=%6.1f ms (true %5.0f) "
                  "res=%9.3f sharp=%7.4f  cost(0)=%9.3f cost(true)=%9.3f\n",
                  t, v.norm(), ok ? 1 : 0, RejectName(d.reject),
                  d.n_ref, d.n_gnss, d.n_erased, d.speed_span_mps,
                  d.speed_max_mps, r.tau_s * 1000.0, tau_true * 1000.0,
                  r.residual_mps, r.sharpness,
                  est.DebugCost(0.0), est.DebugCost(tau_true));
    }
  }
  return 0;
}
