/**
 * @file    test_nav_abi_sanity.cpp
 * @module   测试/宿主机
 * @brief   fw_nav_api 最小自检：静止 / AHRS / 运动 三阶段隔离 6° 姿态偏差来源。
 * @details 输入：argv[1]=mode（0 静止无 AHRS/无 GNSS、1 静止 + AHRS 真值、
 *          2 正弦加速度 + AHRS 真值），30 s；比力取 [a, 0, −9.7938]、气压高 0、
 *          油门 50%；处理：逐 5 ms 调 fw_nav_step 驱动真实固件导航，每 5 s
 *          打印一次；输出：roll/pitch/yaw（deg）与 pos_ned、initialized。
 *          判定：诊断用，无断言（隔离姿态偏差来源）。
 * @frame   机体系（FRD）比力输入；输出 euler_rpy_deg（deg）与 pos_ned_m（NED）；
 *          AHRS 四元数 wxyz 为 FRD←NED（C_n^b）。
 * @units   比力 m/s²（kG=9.7938 为站点近似重力）、姿态 deg、位置 m、时间 s。
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
#include "ins_test_gravity.h"

namespace
{
constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0;
constexpr double kLat = 31.2304 * kDeg2Rad;
constexpr double kLon = 121.4737 * kDeg2Rad;
const double kG = static_cast<double>(ins_test::GravityWgs84());  // WGS84 正常重力，与 EKF 同口径
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
