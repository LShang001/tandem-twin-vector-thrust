/**
 * @file test_fw_algo_api.cpp
 * @module fw-tests/EKF ABI 一致性
 * @brief 固件算法宿主直调接口（fw_algo）与直接调用 bfs::Ekf15State 的一致性回归。
 *
 * @details 目的：证明 C ABI 只是薄封装——同一输入序列下，经 ABI 的句柄与直接
 *          使用固件类的实例必须给出逐位相同的状态、协方差对角与计数器。
 *          覆盖路径：初始化、单子样传播、双子样传播、GNSS 融合、GNSS 野值拒绝、
 *          磁矢量、气压高度、重力方向、静止角速度、航向、位置/速度重置。
 * @frame  NED 导航系 / FRD 机体系（与固件一致）
 * @units  SI
 * @test   run_all.sh 中的 fwalgo 项；失败即 ABI 与固件语义发生漂移。
 * @see    fw_algo/fw_algo_api.h
 */
#include <cmath>
#include <cstdio>
#include <cstring>

#include "ekf_15_state.h"
#include "fw_algo/fw_algo_api.h"

namespace
{
int g_fail = 0;
int g_checked = 0;

void Check(bool ok, const char *what)
{
  ++g_checked;
  if (!ok)
  {
    ++g_fail;
    std::printf("[FAIL] %s\n", what);
  }
}

bool Eq(float a, float b) { return a == b; }
bool Eq(double a, double b) { return a == b; }

const Eigen::Vector3d kLla(31.2304 * M_PI / 180.0, 121.4737 * M_PI / 180.0, 5.0);
const Eigen::Vector3f kGravity(0.0f, 0.0f, -9.79f);

/** 汇总一次状态比对，返回是否逐位相同。 */
bool CompareState(bfs::Ekf15State &ref, fw_ekf *h, const char *tag)
{
  fw_nav_state s;
  std::memset(&s, 0, sizeof(s));
  fw_ekf_get_state(h, &s);

  const Eigen::Vector3d lla = ref.lla_rad_m();
  const Eigen::Vector3f v = ref.ned_vel_mps();
  const Eigen::Vector3f ba = ref.accel_bias_mps2();
  const Eigen::Vector3f bg = ref.gyro_bias_radps();
  const Eigen::Quaternionf q = ref.quat();

  bool same = Eq(s.lla_rad_m[0], lla(0)) && Eq(s.lla_rad_m[1], lla(1)) &&
              Eq(s.lla_rad_m[2], lla(2)) && Eq(s.vel_ned_mps[0], v(0)) &&
              Eq(s.vel_ned_mps[1], v(1)) && Eq(s.vel_ned_mps[2], v(2)) &&
              Eq(s.quat_wxyz[0], q.w()) && Eq(s.quat_wxyz[1], q.x()) &&
              Eq(s.quat_wxyz[2], q.y()) && Eq(s.quat_wxyz[3], q.z()) &&
              Eq(s.accel_bias_mps2[0], ba(0)) && Eq(s.accel_bias_mps2[1], ba(1)) &&
              Eq(s.accel_bias_mps2[2], ba(2)) && Eq(s.gyro_bias_radps[0], bg(0)) &&
              Eq(s.gyro_bias_radps[1], bg(1)) && Eq(s.gyro_bias_radps[2], bg(2)) &&
              Eq(s.euler_ypr_rad[0], ref.yaw_rad()) &&
              Eq(s.euler_ypr_rad[1], ref.pitch_rad()) &&
              Eq(s.euler_ypr_rad[2], ref.roll_rad());

  for (int i = 0; i < FW_EKF_STATE_DIM; ++i)
  {
    same = same && Eq(s.cov_diag[i], ref.covariance_coeff(i, i));
  }

  same = same && s.gnss_reacquire_reset_count == ref.gnss_reacquire_reset_count() &&
         s.gnss_velocity_reanchor_count == ref.gnss_velocity_reanchor_count() &&
         s.gnss_outlier_reject_count == ref.gnss_outlier_reject_count() &&
         s.mag_fused_count == ref.mag_vector_fused_count() &&
         s.mag_reject_count == ref.mag_vector_reject_count() &&
         s.vel_subset_count == ref.gnss_velocity_subset_recovery_count();

  if (!same)
  {
    std::printf("[FAIL] 状态不一致 @ %s\n", tag);
  }
  return same;
}

bool SameResult(fw_update_result a, bfs::MeasurementUpdateResult b)
{
  return a.input_valid == (b.input_valid ? 1 : 0) &&
         a.attempted == (b.attempted ? 1 : 0) &&
         a.fused == (b.fused ? 1 : 0) &&
         a.reacquired == (b.reacquired ? 1 : 0) &&
         a.outlier_rejected == (b.outlier_rejected ? 1 : 0) &&
         a.subset_position == (b.subset_position ? 1 : 0) &&
         a.subset_velocity == (b.subset_velocity ? 1 : 0) && Eq(a.nis_pos3, b.nis_pos3) &&
         Eq(a.nis_vel3, b.nis_vel3) && Eq(a.innovation_norm, b.innovation_norm) &&
         Eq(a.test_ratio, b.test_ratio) &&
         Eq(a.yaw_correction_rad, b.yaw_correction_rad) &&
         a.yaw_correction_clamped == (b.yaw_correction_clamped ? 1 : 0);
}
} // namespace

int main()
{
  Check(fw_algo_abi_version() == FW_ALGO_ABI_VERSION, "ABI 版本匹配");
  Check(fw_algo_build_info() != nullptr && std::strlen(fw_algo_build_info()) > 8,
        "构建信息可用");

  fw_ekf *h = fw_ekf_create();
  Check(h != nullptr, "创建句柄");
  if (h == nullptr)
  {
    return 1;
  }

  bfs::Ekf15State ref;
  fw_ekf_init_t init;
  std::memset(&init, 0, sizeof(init));
  init.accel_mps2[0] = kGravity(0);
  init.accel_mps2[1] = kGravity(1);
  init.accel_mps2[2] = kGravity(2);
  init.lla_rad_m[0] = kLla(0);
  init.lla_rad_m[1] = kLla(1);
  init.lla_rad_m[2] = kLla(2);

  ref.Initialize(kGravity, Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(),
                 Eigen::Vector3f::Zero(), kLla);
  Check(fw_ekf_init(h, &init) == 0, "ABI 初始化返回 0");
  Check(CompareState(ref, h, "init"), "初始化状态一致");

  // 用与固件相同的默认配置：默认子集关闭；此处打开，验证 setter 透传。
  ref.set_gnss_velocity_subset_recovery_enabled(true);
  ref.set_gnss_velocity_subset_max_pos_resid_m(25.0f);
  ref.set_gnss_velocity_subset_max_fires_per_epoch(1);
  fw_ekf_set_velocity_subset(h, 1, 25.0f, 1);

  const float dt = 0.005f;
  const Eigen::Vector3f mag_ned(22.0f, 0.0f, 34.0f);
  bool ok = true;

  for (int step = 0; step < 1200; ++step)
  {
    const float t = static_cast<float>(step) * dt;
    const Eigen::Vector3f accel(
        0.02f * std::sin(1.3f * t), 0.015f * std::cos(0.7f * t),
        -9.79f + 0.01f * std::sin(2.1f * t));
    const Eigen::Vector3f gyro(2.0e-3f * std::sin(0.9f * t),
                               1.5e-3f * std::cos(1.1f * t),
                               1.0e-3f * std::sin(0.4f * t));

    if (step % 2 == 0)
    {
      ref.TimeUpdate(accel, gyro, dt);
      fw_ekf_time_update(h, accel.data(), gyro.data(), dt);
    }
    else
    {
      const Eigen::Vector3f dth1 = gyro * (dt * 0.5f);
      const Eigen::Vector3f dth2 = gyro * (dt * 0.5f);
      const Eigen::Vector3f dv1 = accel * (dt * 0.5f);
      const Eigen::Vector3f dv2 = accel * (dt * 0.5f);
      ref.TimeUpdateTwoSample(dth1, dth2, dv1, dv2, dt);
      fw_ekf_time_update_two_sample(h, dth1.data(), dth2.data(), dv1.data(),
                                    dv2.data(), dt);
    }

    if (step % 10 == 0)
    {
      Eigen::Vector3d lla = kLla;
      lla(0) += 2.0e-7 * std::sin(0.05f * t);
      lla(1) += 2.0e-7 * std::cos(0.05f * t);
      Eigen::Vector3f vel(0.05f * std::sin(0.2f * t), 0.05f * std::cos(0.2f * t),
                          0.01f * std::sin(0.3f * t));
      if (step == 700)
      {
        lla(0) += 2.0e-3; // ≈222 m 跳点：应触发物理野值门
      }
      const bfs::MeasurementUpdateResult rr =
          ref.MeasurementUpdateDetailed(vel, lla, dt, 0.02f, true);
      const fw_update_result ra =
          fw_ekf_gnss_update(h, vel.data(), lla.data(), dt, 0.02f, 1);
      ok = ok && SameResult(ra, rr);
      Check(SameResult(ra, rr), "GNSS 更新结果一致");
    }

    if (step % 20 == 0)
    {
      const bfs::MeasurementUpdateResult rr =
          ref.MeasurementUpdateMagVectorDetailed(mag_ned, mag_ned, 3.0f);
      const fw_update_result ra =
          fw_ekf_mag_update(h, mag_ned.data(), mag_ned.data(), 3.0f);
      ok = ok && SameResult(ra, rr);
      Check(SameResult(ra, rr), "磁矢量更新结果一致");
    }

    if (step % 5 == 0)
    {
      const float baro = static_cast<float>(kLla(2)) + 0.1f * std::sin(0.2f * t);
      const bfs::MeasurementUpdateResult rr =
          ref.MeasurementUpdateBaroAltitudeDetailed(baro, 0.4f);
      const fw_update_result ra = fw_ekf_baro_update(h, baro, 0.4f);
      ok = ok && SameResult(ra, rr);
      Check(SameResult(ra, rr), "气压更新结果一致");
    }

    if (step % 40 == 0)
    {
      const bfs::MeasurementUpdateResult rr =
          ref.MeasurementUpdateGravityDetailed(accel, 0.05f);
      const fw_update_result ra = fw_ekf_gravity_update(h, accel.data(), 0.05f);
      ok = ok && SameResult(ra, rr);
      Check(SameResult(ra, rr), "重力更新结果一致");
    }

    if (step % 200 == 0)
    {
      const bfs::MeasurementUpdateResult rr =
          ref.MeasurementUpdateStaticGyroDetailed(gyro, 0.002f);
      const fw_update_result ra = fw_ekf_static_gyro_update(h, gyro.data(), 0.002f);
      ok = ok && SameResult(ra, rr);
      Check(SameResult(ra, rr), "静止陀螺结果一致");
    }

    if (step == 900)
    {
      const bfs::MeasurementUpdateResult rr = ref.MeasurementUpdateYawDetailed(0.01f, 0.05f);
      const fw_update_result ra = fw_ekf_yaw_update(h, 0.01f, 0.05f);
      ok = ok && SameResult(ra, rr);
      Check(SameResult(ra, rr), "航向更新结果一致");
    }

    ok = ok && CompareState(ref, h, "loop");
  }

  // 重置路径
  {
    Eigen::Vector3d lla = kLla;
    lla(0) += 1.0e-5;
    const Eigen::Vector3f vel(0.2f, -0.1f, 0.05f);
    ref.ResetPositionVelocityToGnss(vel, lla);
    fw_ekf_reset_position_velocity(h, vel.data(), lla.data());
    ok = ok && CompareState(ref, h, "reset_pos_vel");

    const Eigen::Vector3f vel2(0.1f, 0.1f, -0.02f);
    ref.ResetVelocityToExternal(vel2, 0.5f);
    fw_ekf_reset_velocity(h, vel2.data(), 0.5f);
    ok = ok && CompareState(ref, h, "reset_vel");
  }

  Check(ref.gnss_outlier_reject_count() >= 1, "注入的 GNSS 跳点确实被拒绝");
  Check(ref.gnss_outlier_reject_count() == 1, "GNSS 野值仅拒绝一次");

  char json[2048];
  const int n = fw_ekf_diag_json(h, json, static_cast<int32_t>(sizeof(json)));
  Check(n > 0, "诊断 JSON 可输出");
  char abi_expect[32];
  std::snprintf(abi_expect, sizeof(abi_expect), "\"abi\":%d", FW_ALGO_ABI_VERSION);
  Check(std::strstr(json, abi_expect) != nullptr, "诊断 JSON 含当前 ABI 版本");
  Check(std::strstr(json, "\"cov_diag\":[") != nullptr, "诊断 JSON 含协方差对角");

  fw_ekf_destroy(h);
  fw_ekf_destroy(nullptr);

  Check(ok, "全序列逐位一致");

  std::printf("fw_algo ABI 一致性：%d 项检查，%d 失败\n", g_checked, g_fail);
  return g_fail == 0 ? 0 : 1;
}
