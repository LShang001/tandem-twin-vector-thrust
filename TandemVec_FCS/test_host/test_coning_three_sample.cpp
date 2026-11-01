/**
 * @file    test_coning_three_sample.cpp
 * @brief   纯锥运动下双子样 vs 三子样圆锥补偿精度对比。
 *
 * 经典补偿（旋转矢量）：
 *   双子样：δθ = (2/3)·(θ1×θ2)
 *   三子样：δθ = (9/20)·(θ1×θ3) + (27/40)·(θ2×(θ3−θ1))
 * 参考：REF-120 §9.3.30 p.274（双子样）、§9.4.15 p.283（三子样）、
 *       表 9.4.1/9.4.2 p.284（最优系数）；Savage JGCD 1998 / Ignagni 1990。
 *
 * 方法：解析锥运动 ω(t)=[a·sin(ω0 t), a·cos(ω0 t), 0]；
 *   真值用细步长（h/1000）旋转矢量积分；补偿算法用 N 子样/周期；
 *   比较 1 h 累计姿态误差（°）。
 */
#include <cmath>
#include <cstdio>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg2Rad = kPi / 180.0;

Eigen::Vector3d omegaAt(double t, double a, double w0)
{
  return Eigen::Vector3d(a * std::sin(w0 * t), a * std::cos(w0 * t), 0.0);
}

// 旋转矢量 → 四元数
Eigen::Quaterniond quatFromRotVec(const Eigen::Vector3d &rv)
{
  const double th = rv.norm();
  if (th < 1e-12)
  {
    return Eigen::Quaterniond::Identity();
  }
  return Eigen::Quaterniond(Eigen::AngleAxisd(th, rv / th));
}

// 真值：细步长积分
Eigen::Quaterniond integrateTruth(double t0, double t1, double a, double w0,
                                  int sub)
{
  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
  const double dt = (t1 - t0) / sub;
  for (int i = 0; i < sub; ++i)
  {
    const double t = t0 + dt * (i + 0.5);
    q = q * quatFromRotVec(omegaAt(t, a, w0) * dt);
  }
  return q.normalized();
}

// 双子样补偿：周期内 2 子样
Eigen::Quaterniond propagateTwoSample(double t0, double h, double a, double w0)
{
  const double dt = h / 2.0;
  const Eigen::Vector3d th1 = omegaAt(t0 + dt * 0.5, a, w0) * dt;
  const Eigen::Vector3d th2 = omegaAt(t0 + dt * 1.5, a, w0) * dt;
  const Eigen::Vector3d dth = th1 + th2;
  const Eigen::Vector3d coning = (2.0 / 3.0) * th1.cross(th2);
  return quatFromRotVec(dth + coning);
}

// 三子样补偿：周期内 3 子样
Eigen::Quaterniond propagateThreeSample(double t0, double h, double a,
                                        double w0)
{
  const double dt = h / 3.0;
  const Eigen::Vector3d th1 = omegaAt(t0 + dt * 0.5, a, w0) * dt;
  const Eigen::Vector3d th2 = omegaAt(t0 + dt * 1.5, a, w0) * dt;
  const Eigen::Vector3d th3 = omegaAt(t0 + dt * 2.5, a, w0) * dt;
  const Eigen::Vector3d dth = th1 + th2 + th3;
  const Eigen::Vector3d coning =
      (9.0 / 20.0) * th1.cross(th3) + (27.0 / 40.0) * th2.cross(th3 - th1);
  return quatFromRotVec(dth + coning);
}

double attErrDeg(const Eigen::Quaterniond &q_ref, const Eigen::Quaterniond &q)
{
  const Eigen::Quaterniond dq = q_ref.conjugate() * q;
  const double w = std::fmin(1.0, std::fabs(dq.w()));
  return 2.0 * std::acos(w) / kDeg2Rad;
}
} // namespace

int main()
{
  std::printf("== 纯锥运动：双子样 vs 三子样圆锥补偿（1 h 累计） ==\n");
  std::printf("%-8s %-8s %-8s %14s %14s\n", "锥半角", "锥频率", "周期",
              "双子样(°)", "三子样(°)");

  int fail = 0;
  for (double amp_deg : {0.5, 1.0})
  {
    for (double f_hz : {20.0, 50.0})
    {
      const double a = amp_deg * kDeg2Rad;
      const double w0 = 2.0 * kPi * f_hz;
      const double h = 0.005;  // 200 Hz 导航周期
      const int steps = static_cast<int>(3600.0 / h);

      Eigen::Quaterniond q_true = Eigen::Quaterniond::Identity();
      Eigen::Quaterniond q_two = Eigen::Quaterniond::Identity();
      Eigen::Quaterniond q_three = Eigen::Quaterniond::Identity();
      for (int k = 0; k < steps; ++k)
      {
        const double t0 = k * h;
        q_true = (q_true * integrateTruth(t0, t0 + h, a, w0, 200)).normalized();
        q_two = (q_two * propagateTwoSample(t0, h, a, w0)).normalized();
        q_three = (q_three * propagateThreeSample(t0, h, a, w0)).normalized();
      }
      const double e_two = attErrDeg(q_true, q_two);
      const double e_three = attErrDeg(q_true, q_three);
      std::printf("%-8.1f %-8.0f %-8.0f ms %14.6f %14.6f\n", amp_deg, f_hz,
                  h * 1000.0, e_two, e_three);
      if (!(e_three <= e_two * 1.05 + 1e-6))
      {
        std::printf("  [FAIL] 三子样未优于双子样\n");
        ++fail;
      }
    }
  }

  if (fail == 0)
  {
    std::printf("三子样圆锥补偿通过（各锥频下不劣于双子样）\n");
    return 0;
  }
  std::printf("失败 %d 项\n", fail);
  return 1;
}
