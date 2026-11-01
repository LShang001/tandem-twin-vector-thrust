/**
 * @file    test_ins_trajectory.cpp
 * @brief   真实运动轨迹下的纯惯导 vs GNSS 辅助对比（输出 CSV 供可视化）。
 *
 * 轨迹（60 s）：
 *   0–5 s    静止
 *   5–10 s   垂直爬升 0→50 m（10 m/s）
 *   10–40 s  盘旋：半径 80 m、速度 15 m/s、高度 50 m
 *   40–50 s  直线加速到 25 m/s（保持高度）
 *   50–60 s  继续直线并爬升到 120 m
 *
 * 姿态简化：机头沿速度方向（协调转弯），无 roll/pitch；
 * IMU = 比力 + 角速度 + 零偏 + 噪声（MEMS 5 mg / 0.4 °/s）。
 * 输出 CSV：t, true_n/e/d, ins_n/e/d, gnss_n/e/d, true_att, ins_att。
 */
#include <cmath>
#include "ins_test_gravity.h"
#include <cstdio>
#include <random>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "ekf_15_state.h"

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg2Rad = kPi / 180.0;
constexpr float kNavDt = 0.005f;
const float kGravity = ins_test::GravityWgs84();  // WGS84 正常重力（与 EKF 同口径）
const Eigen::Vector3d kLla(31.2304 * kDeg2Rad, 121.4737 * kDeg2Rad, 5.0);

const float kAccelBiasTrue = 0.005f * kGravity;  // 5 mg
const float kGyroBiasTrue = 0.4f * kDeg2Rad;     // 0.4 °/s
const float kAccelNoise = 0.002f * kGravity;
const float kGyroNoise = 0.02f * kDeg2Rad;

struct TrajState
{
  Eigen::Vector3d pos_ned = Eigen::Vector3d::Zero();
  Eigen::Vector3d vel_ned = Eigen::Vector3d::Zero();
  Eigen::Vector3d acc_ned = Eigen::Vector3d::Zero();
  double heading = 0.0;       // rad（机头方向）
  double heading_rate = 0.0;  // rad/s
};

TrajState traj(double t)
{
  TrajState s;
  const double r = 80.0;          // 盘旋半径
  const double v_turn = 15.0;     // 盘旋速度
  const double omega = v_turn / r;
  if (t < 5.0)
  {
    // 静止
  }
  else if (t < 10.0)
  {
    // 垂直爬升
    const double h = 10.0 * (t - 5.0);
    s.pos_ned(2) = -h;
    s.vel_ned(2) = -10.0;
  }
  else if (t < 40.0)
  {
    // 盘旋：起点 (0,0,-50)，切向沿 +e
    const double tau = t - 10.0;
    const double h = 50.0;
    s.pos_ned(0) = r * std::sin(omega * tau);
    s.pos_ned(1) = r * (1.0 - std::cos(omega * tau));
    s.pos_ned(2) = -h;
    s.vel_ned(0) = v_turn * std::cos(omega * tau);
    s.vel_ned(1) = v_turn * std::sin(omega * tau);
    s.vel_ned(2) = 0.0;
    s.acc_ned(0) = -r * omega * omega * std::sin(omega * tau);
    s.acc_ned(1) = r * omega * omega * std::cos(omega * tau);
    s.acc_ned(2) = 0.0;
    s.heading = omega * tau;
    s.heading_rate = omega;
  }
  else if (t < 50.0)
  {
    // 直线加速（从盘旋末端位置沿 heading 方向）
    const double tau = t - 40.0;
    const double h = 50.0;
    const double acc = 1.0;  // 1 m/s²
    const double v0 = v_turn;
    const double v = v0 + acc * tau;
    const double d = v0 * tau + 0.5 * acc * tau * tau;
    // 盘旋末端位置与航向
    const double hd = omega * (40.0 - 10.0);
    s.pos_ned(0) = r * std::sin(hd) + d * std::cos(hd);
    s.pos_ned(1) = r * (1.0 - std::cos(hd)) + d * std::sin(hd);
    s.pos_ned(2) = -h;
    s.vel_ned(0) = v * std::cos(hd);
    s.vel_ned(1) = v * std::sin(hd);
    s.vel_ned(2) = 0.0;
    s.acc_ned(0) = acc * std::cos(hd);
    s.acc_ned(1) = acc * std::sin(hd);
    s.acc_ned(2) = 0.0;
    s.heading = hd;
    s.heading_rate = 0.0;
  }
  else
  {
    // 直线 + 爬升到 120 m
    const double tau = t - 50.0;
    const double h = 50.0 + 7.0 * tau;  // 10 s 爬 70 m
    const double v0 = 25.0;
    const double acc = 0.0;
    const double d = v0 * tau;
    const double hd = omega * (40.0 - 10.0);
    s.pos_ned(0) = r * std::sin(hd) + d * std::cos(hd);
    s.pos_ned(1) = r * (1.0 - std::cos(hd)) + d * std::sin(hd);
    s.pos_ned(2) = -h;
    s.vel_ned(0) = v0 * std::cos(hd);
    s.vel_ned(1) = v0 * std::sin(hd);
    s.vel_ned(2) = -7.0;
    s.acc_ned << 0.0, 0.0, 0.0;
    s.heading = hd;
    s.heading_rate = 0.0;
  }
  return s;
}

Eigen::Matrix3d nedFromBody(double heading)
{
  const double c = std::cos(heading);
  const double s = std::sin(heading);
  Eigen::Matrix3d R;
  R << c, -s, 0.0,
       s,  c, 0.0,
       0.0, 0.0, 1.0;
  return R;
}
} // namespace

int main()
{
  std::mt19937 rng(20261101U);
  std::normal_distribution<double> n01(0.0, 1.0);

  const Eigen::Vector3f accel_bias(
      kAccelBiasTrue * static_cast<float>(n01(rng)),
      kAccelBiasTrue * static_cast<float>(n01(rng)),
      kAccelBiasTrue * static_cast<float>(n01(rng)));
  const Eigen::Vector3f gyro_bias(
      kGyroBiasTrue * static_cast<float>(n01(rng)),
      kGyroBiasTrue * static_cast<float>(n01(rng)),
      kGyroBiasTrue * static_cast<float>(n01(rng)));

  // EKF 初对准：用 t=0 的静止数据
  bfs::Ekf15State ekf_ins;
  ekf_ins.Initialize(Eigen::Vector3f(0.0f, 0.0f, -kGravity),
                     Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(),
                     Eigen::Vector3f::Zero(), kLla);
  bfs::Ekf15State ekf_gnss = ekf_ins;

  // 输出 CSV（仓库 tmp/ 下）
  std::FILE *f = std::fopen("../tmp/ins_trajectory.csv", "w");
  if (f == nullptr)
  {
    std::printf("无法打开 ../tmp/ins_trajectory.csv\n");
    return 1;
  }
  std::fprintf(f, "t,tn,te,td,en,ee,ed,gn,ge,gd,att_true,att_ins\n");

  const int steps = static_cast<int>(60.0 / kNavDt);
  int gnss_div = static_cast<int>(1.0 / (10.0 * kNavDt));  // 10 Hz
  for (int k = 0; k < steps; ++k)
  {
    const double t = k * kNavDt;
    const TrajState tr = traj(t);
    const Eigen::Matrix3d R = nedFromBody(tr.heading);
    // 比力：f_b = R^T (a_ned - g_ned)
    const Eigen::Vector3d g_ned(0.0, 0.0, kGravity);
    const Eigen::Vector3d f_body = R.transpose() * (tr.acc_ned - g_ned);
    const Eigen::Vector3d w_body(0.0, 0.0, tr.heading_rate);

    Eigen::Vector3f accel = f_body.cast<float>();
    Eigen::Vector3f gyro = w_body.cast<float>();
    for (int a = 0; a < 3; ++a)
    {
      accel(a) += accel_bias(a) + kAccelNoise * static_cast<float>(n01(rng));
      gyro(a) += gyro_bias(a) + kGyroNoise * static_cast<float>(n01(rng));
    }

    ekf_ins.TimeUpdate(accel, gyro, kNavDt);
    ekf_gnss.TimeUpdate(accel, gyro, kNavDt);

    // GNSS 10 Hz：理想位置 + 1.5 m 噪声
    if ((k % gnss_div) == 0)
    {
      const Eigen::Vector3d lla(
          kLla(0) + tr.pos_ned(0) / 6378137.0,
          kLla(1) + tr.pos_ned(1) / (6378137.0 * std::cos(kLla(0))),
          kLla(2) - tr.pos_ned(2));
      const Eigen::Vector3f vel = tr.vel_ned.cast<float>();
      ekf_gnss.gnss_pos_ne_std_m(1.5f);
      ekf_gnss.gnss_pos_d_std_m(2.5f);
      ekf_gnss.gnss_vel_ne_std_mps(0.1f);
      ekf_gnss.gnss_vel_d_std_mps(0.2f);
      (void)ekf_gnss.MeasurementUpdateDetailed(vel, lla, kNavDt);
    }

    // 每 0.1 s 记录一次
    if ((k % 20) == 0)
    {
      const double en = (ekf_ins.lat_rad() - kLla(0)) * 6378137.0;
      const double ee = (ekf_ins.lon_rad() - kLla(1)) * 6378137.0 *
                        std::cos(kLla(0));
      const double ed = -(ekf_ins.alt_m() - kLla(2));
      const double gn = (ekf_gnss.lat_rad() - kLla(0)) * 6378137.0;
      const double ge = (ekf_gnss.lon_rad() - kLla(1)) * 6378137.0 *
                        std::cos(kLla(0));
      const double gd = ekf_gnss.alt_m() - kLla(2);  // 向上为正
      const double att_true =
          std::atan2(tr.vel_ned(1), tr.vel_ned(0)) * 180.0 / kPi;
      const double att_ins =
          std::atan2(2.0 * (ekf_ins.quat().w() * ekf_ins.quat().z() +
                            ekf_ins.quat().x() * ekf_ins.quat().y()),
                     1.0 - 2.0 * (ekf_ins.quat().y() * ekf_ins.quat().y() +
                                  ekf_ins.quat().z() * ekf_ins.quat().z())) *
          180.0 / kPi;
      std::fprintf(f, "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f,%.1f\n",
                   t, tr.pos_ned(0), tr.pos_ned(1), -tr.pos_ned(2), en, ee, ed,
                   gn, ge, gd, att_true, att_ins);
    }
  }
  std::fclose(f);
  std::printf("轨迹 CSV 已写出：../tmp/ins_trajectory.csv\n");
  return 0;
}
