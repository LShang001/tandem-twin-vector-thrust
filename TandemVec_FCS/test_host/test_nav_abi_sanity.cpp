/**
 * @file    test_nav_abi_sanity.cpp
 * @brief   fw_nav_api 最小自检：静止 / AHRS / 运动 三阶段隔离 6° 姿态偏差来源。
 *
 * 用法：test_nav_abi_sanity <mode>
 *   mode=0 静止、无 AHRS、无 GNSS
 *   mode=1 静止 + AHRS 真值（水平）
 *   mode=2 运动（正弦加速度）+ AHRS 真值
 * 输出：每 5 s 的 att/roll/pitch 与最终误差。
 */
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "fw_nav_api.h"

namespace
{
constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0;
constexpr double kLat = 31.2304 * kDeg2Rad;
constexpr double kLon = 121.4737 * kDeg2Rad;
constexpr double kG = 9.7938;
constexpr double kDt = 0.005;
}  // namespace

int main(int argc, char **argv)
{
  const int mode = (argc > 1) ? std::atoi(argv[1]) : 0;
  fw_nav *nav = nullptr;
  if (fw_nav_create(&nav) != 0)
  {
    std::printf("create fail\n");
    return 1;
  }
  fw_nav_reset(nav);

  fw_nav_output_t out;
  std::memset(&out, 0, sizeof(out));
  out.struct_size = sizeof(out);

  const double T = 30.0;
  const int steps = static_cast<int>(T / kDt);
  const double omega = 2.0 * 3.141592653589793 / 15.0;  // 15 s 周期

  for (int k = 0; k < steps; ++k)
  {
    const double t = k * kDt;
    fw_nav_input_t in;
    std::memset(&in, 0, sizeof(in));
    in.struct_size = sizeof(in);
    in.abi_version = FW_NAV_ABI_VERSION;
    in.dt_s = static_cast<float>(kDt);
    in.sample_count = 1;

    double a = 0.0, v = 0.0;
    if (mode == 2 && t >= 1.0)
    {
      a = 2.0 * std::cos(omega * (t - 1.0));
      v = (2.0 / omega) * std::sin(omega * (t - 1.0));
    }
    in.accel_sum_body_mps2[0] = static_cast<float>(a);
    in.accel_sum_body_mps2[1] = 0.0f;
    in.accel_sum_body_mps2[2] = static_cast<float>(-kG);

    if (mode >= 1)
    {
      // AHRS：固定水平姿态（FRD←NED 单位四元数）
      in.ahrs_quat_wxyz[0] = 1.0f;
      in.ahrs_valid = 1;
    }
    in.baro_alt_up_m = 0.0f;
    in.baro_valid = 1;
    in.unlocked = 1;
    in.throttle_pct = 50.0f;

    if (fw_nav_step(nav, &in, &out) != 0)
    {
      std::printf("step fail k=%d\n", k);
      return 1;
    }
    if (k % 1000 == 0)
    {
      std::printf("t=%5.1f roll=%7.3f pitch=%7.3f yaw=%7.3f pos=(%6.2f,%6.2f,%6.2f) init=%d\n",
                  t, out.euler_rpy_deg[0], out.euler_rpy_deg[1],
                  out.euler_rpy_deg[2], out.pos_ned_m[0], out.pos_ned_m[1],
                  out.pos_ned_m[2], out.initialized);
    }
  }
  fw_nav_destroy(nav);
  return 0;
}
