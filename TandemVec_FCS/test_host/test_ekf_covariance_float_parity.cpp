/**
 * @file    test_ekf_covariance_float_parity.cpp
 * @module  固件/宿主回归测试（fw-tests：float/double 协方差传播数值一致性）
 * @brief   相同确定性 IMU 轨迹与 GNSS 量测下，对比 float 与 double 协方差中间量的输出
 * @details 被测：`lib/navigation-main/src/ekf_15_state.h` 的 `TimeUpdate` 与
 *          `MeasurementUpdateDetailed`。输入：水平静止比力叠加正弦机动、陀螺正弦
 *          角速度、每 50 拍一次 GNSS 位置/速度量测；处理：200 Hz 传播 2000 拍；
 *          输出：姿态/速度/位置及全部协方差对角线，供双可执行文件逐项 diff。
 * @theory  仅替换协方差传播中间量精度，不改变 Phi、Qd 或 Joseph 更新公式。
 * @frame   NED + FRD
 * @units   rad、m/s、m、m/s²、rad/s
 * @note    本测试不产生随机数；两种编译变体必须产生可比较的确定性轨迹。
 * @see     test_ekf_15state.cpp
 */
#include <cstdio>
#include <cmath>
#include <Eigen/Dense>
#include "ekf_15_state.h"

using bfs::Ekf15State;

int main()
{
  const Eigen::Vector3d lla(28.2 * M_PI / 180.0, 112.9 * M_PI / 180.0, 50.0);
  const Eigen::Vector3f gravity(0.0f, 0.0f, -9.79f);
  Ekf15State ekf;
  ekf.Initialize(gravity, Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(),
                 Eigen::Vector3f::Zero(), lla);

  for (int k = 0; k < 2000; ++k)
  {
    const float t = 0.005f * k;
    const Eigen::Vector3f accel(
        0.35f * std::sin(0.7f * t) + 0.05f * std::sin(3.1f * t),
        -0.20f * std::cos(0.9f * t),
        -9.79f + 0.45f * std::sin(1.3f * t));
    const Eigen::Vector3f gyro(0.03f * std::sin(1.1f * t),
                               0.04f * std::cos(0.8f * t),
                               0.02f * std::sin(1.7f * t));
    ekf.TimeUpdate(accel, gyro, 0.005f);

    if ((k % 50) == 0)
    {
      const Eigen::Vector3f gnss_vel(0.15f * std::sin(0.2f * t),
                                     -0.08f * std::cos(0.3f * t),
                                     0.04f * std::sin(0.5f * t));
      const Eigen::Vector3d gnss_lla = lla;
      (void)ekf.MeasurementUpdateDetailed(gnss_vel, gnss_lla, 0.005f);
    }
  }

  std::printf("roll,%.12g\n", ekf.roll_rad());
  std::printf("pitch,%.12g\n", ekf.pitch_rad());
  std::printf("yaw,%.12g\n", ekf.yaw_rad());
  const auto vel = ekf.ned_vel_mps();
  std::printf("vn,%.12g\n", vel(0));
  std::printf("ve,%.12g\n", vel(1));
  std::printf("vd,%.12g\n", vel(2));
  const auto pos = ekf.lla_rad_m();
  std::printf("lat,%.12g\n", pos(0));
  std::printf("lon,%.12g\n", pos(1));
  std::printf("alt,%.12g\n", pos(2));
  for (int r = 0; r < 15; ++r)
  {
    for (int c = r; c < 15; ++c)
    {
      std::printf("p%02d_%02d,%.12g\n", r, c, ekf.covariance_coeff_for_test(r, c));
    }
  }
  return 0;
}
