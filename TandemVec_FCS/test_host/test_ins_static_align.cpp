/**
 * @file    test_ins_static_align.cpp
 * @brief   静止校准（零偏估计）与补偿效果验证。
 *
 * 思路：
 *   1) 静止 60 s，用加速度计/陀螺的均值估计零偏：
 *        accel_bias = mean(accel) − (0,0,−g)
 *        gyro_bias  = mean(gyro)
 *   2) 用估计值补偿后跑 300 s 纯惯导，对比未补偿的位置漂移；
 *   3) 同时输出估计误差（相对注入的真零偏）。
 *
 * 物理边界：MEMS 零偏（5 mg / 0.4 °/s）下即使完美补偿，纯惯导位置
 * 漂移仍会增长（速度误差积分）；校准只能消除“常值零偏”这一项。
 *
 * 编译：g++ -std=c++17 -O2 -D_USE_MATH_DEFINES -Iinclude -Isrc \
 *       -Ilib/navigation-main/src -Ilib/eigen/src -Ilib/units/src \
 *       test_host/test_ins_static_align.cpp lib/units/src/convang.cpp \
 *       lib/units/src/convangacc.cpp lib/units/src/convangvel.cpp -o t.exe
 */
#include <cmath>
#include "ins_test_gravity.h"
#include <cstdio>
#include <random>
#include <Eigen/Core>

#include "ekf_15_state.h"

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg2Rad = kPi / 180.0;
constexpr float kNavDt = 0.005f;
const float kGravity = ins_test::GravityWgs84();  // WGS84 正常重力（与 EKF 同口径）
const Eigen::Vector3d kLla(31.2304 * kDeg2Rad, 121.4737 * kDeg2Rad, 5.0);

const float kAccelBiasTrue = 0.005f * kGravity;  // 5 mg
const float kGyroBiasTrue = 0.4f * kDeg2Rad;     // 0.4 °/s = 0.00698 rad/s

// 白噪声（静止段用于校准，噪声被平均抑制）
const float kAccelNoise = 0.002f * kGravity;  // 2 mg σ
const float kGyroNoise = 0.02f * kDeg2Rad;    // 0.02 °/s σ
} // namespace

int main()
{
  std::printf("== 静止校准（零偏估计）与补偿效果 ==\n");
  int fail = 0;

  for (int seed = 1; seed <= 3; ++seed)
  {
    std::mt19937 rng(static_cast<uint32_t>(seed) * 9973U);
    std::normal_distribution<double> n01(0.0, 1.0);

    // 随机化真零偏方向（各轴独立，量级固定）
    const Eigen::Vector3f accel_bias_true(
        kAccelBiasTrue * static_cast<float>(n01(rng)),
        kAccelBiasTrue * static_cast<float>(n01(rng)),
        kAccelBiasTrue * static_cast<float>(n01(rng)));
    const Eigen::Vector3f gyro_bias_true(
        kGyroBiasTrue * static_cast<float>(n01(rng)),
        kGyroBiasTrue * static_cast<float>(n01(rng)),
        kGyroBiasTrue * static_cast<float>(n01(rng)));

    // ---- 1) 静止 60 s 采样，估计零偏 ----
    const int align_steps = static_cast<int>(60.0 / kNavDt);
    Eigen::Vector3f accel_sum = Eigen::Vector3f::Zero();
    Eigen::Vector3f gyro_sum = Eigen::Vector3f::Zero();
    for (int k = 0; k < align_steps; ++k)
    {
      Eigen::Vector3f accel(0.0f, 0.0f, -kGravity);
      Eigen::Vector3f gyro = Eigen::Vector3f::Zero();
      for (int a = 0; a < 3; ++a)
      {
        accel(a) += accel_bias_true(a) +
                    kAccelNoise * static_cast<float>(n01(rng));
        gyro(a) += gyro_bias_true(a) +
                   kGyroNoise * static_cast<float>(n01(rng));
      }
      accel_sum += accel;
      gyro_sum += gyro;
    }
    const Eigen::Vector3f accel_bias_est =
        accel_sum / static_cast<float>(align_steps) -
        Eigen::Vector3f(0.0f, 0.0f, -kGravity);
    const Eigen::Vector3f gyro_bias_est =
        gyro_sum / static_cast<float>(align_steps);

    const float accel_err = (accel_bias_est - accel_bias_true).norm();
    const float gyro_err = (gyro_bias_est - gyro_bias_true).norm();

    // ---- 2) 补偿/不补偿各跑 300 s 纯惯导 ----
    struct RunOut
    {
      double pos_m;
      double att_deg;
    };
    auto run = [&](bool compensate) {
      bfs::Ekf15State ekf;
      ekf.Initialize(Eigen::Vector3f(0.0f, 0.0f, -kGravity),
                     Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(),
                     Eigen::Vector3f::Zero(), kLla);
      const int steps = static_cast<int>(300.0 / kNavDt);
      for (int k = 0; k < steps; ++k)
      {
        Eigen::Vector3f accel(0.0f, 0.0f, -kGravity);
        Eigen::Vector3f gyro = Eigen::Vector3f::Zero();
        for (int a = 0; a < 3; ++a)
        {
          accel(a) += accel_bias_true(a) +
                      kAccelNoise * static_cast<float>(n01(rng));
          gyro(a) += gyro_bias_true(a) +
                     kGyroNoise * static_cast<float>(n01(rng));
        }
        if (compensate)
        {
          accel -= accel_bias_est;
          gyro -= gyro_bias_est;
        }
        ekf.TimeUpdate(accel, gyro, kNavDt);
      }
      const double dn = (ekf.lat_rad() - kLla(0)) * 6378137.0;
      const double de = (ekf.lon_rad() - kLla(1)) * 6378137.0 *
                        std::cos(kLla(0));
      RunOut out;
      out.pos_m = std::hypot(dn, de);
      out.att_deg =
          std::fmax(std::fabs(ekf.roll_rad()), std::fabs(ekf.pitch_rad())) *
          180.0 / kPi;
      return out;
    };

    const RunOut raw = run(false);
    const RunOut comp = run(true);

    std::printf(
        "seed=%d  accel_err=%.4f mg  gyro_err=%.4f °/s | 位置 %.1f → %.1f m"
        "  姿态 %.3f → %.3f°\n",
        seed, accel_err / kGravity * 1000.0f, gyro_err / kDeg2Rad,
        raw.pos_m, comp.pos_m, raw.att_deg, comp.att_deg);

    if (accel_err > 0.5f * kAccelBiasTrue)
    {
      std::printf("  [FAIL] 加速度计零偏估计误差过大\n");
      ++fail;
    }
    if (gyro_err > 0.5f * kGyroBiasTrue)
    {
      std::printf("  [FAIL] 陀螺零偏估计误差过大\n");
      ++fail;
    }
    if (comp.pos_m > 0.8 * raw.pos_m)
    {
      std::printf("  [FAIL] 补偿后位置漂移未改善（%.1f vs %.1f m）\n",
                  comp.pos_m, raw.pos_m);
      ++fail;
    }
  }

  if (fail == 0)
  {
    std::printf("静止校准通过（零偏估计误差 < 真值 50%%，补偿后漂移下降 >20%%）\n");
    return 0;
  }
  std::printf("失败 %d 项\n", fail);
  return 1;
}
