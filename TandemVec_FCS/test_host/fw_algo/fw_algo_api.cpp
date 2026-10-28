/**
 * @file fw_algo_api.cpp
 * @module fw-nav/EKF 宿主直调 C ABI
 * @brief 固件算法宿主直调接口实现：薄封装 bfs::Ekf15State，不改滤波数学。
 *
 * @details 每个 fw_ekf 句柄持有一个独立的 bfs::Ekf15State。所有输入输出在 ABI 边界
 *          转换为 Eigen 定长类型后原样透传；返回值由固件 bfs::MeasurementUpdateResult
 *          逐字段映射。新增/修改算法只需改固件头文件，本文件不复制任何公式。
 *
 * @frame  NED 导航系 / FRD 机体系（透传固件约定）
 * @units  SI：m、m/s、m/s²、rad、rad/s、s
 * @note 编译时与固件/SIL 同用标量 Eigen（-DEIGEN_DONT_VECTORIZE /
 *       -DEIGEN_DONT_PARALLELIZE），避免宿主多核向量化造成边际样本双吸引子。
 * @see fw_algo_api.h
 */
#include "fw_algo_api.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <new>

#include "ekf_15_state.h"

struct fw_ekf
{
  bfs::Ekf15State ekf;
  bool initialized = false;
};

namespace
{
Eigen::Vector3f ToVec3(const float v[3])
{
  return Eigen::Vector3f(v[0], v[1], v[2]);
}

Eigen::Vector3d ToVec3d(const double v[3])
{
  return Eigen::Vector3d(v[0], v[1], v[2]);
}

fw_update_result ToAbi(const bfs::MeasurementUpdateResult &r)
{
  fw_update_result o;
  o.input_valid = r.input_valid ? 1 : 0;
  o.attempted = r.attempted ? 1 : 0;
  o.fused = r.fused ? 1 : 0;
  o.reacquired = r.reacquired ? 1 : 0;
  o.outlier_rejected = r.outlier_rejected ? 1 : 0;
  o.subset_position = r.subset_position ? 1 : 0;
  o.subset_velocity = r.subset_velocity ? 1 : 0;
  o.nis_pos3 = r.nis_pos3;
  o.nis_vel3 = r.nis_vel3;
  o.innovation_norm = r.innovation_norm;
  o.test_ratio = r.test_ratio;
  o.yaw_correction_rad = r.yaw_correction_rad;
  o.yaw_correction_clamped = r.yaw_correction_clamped ? 1 : 0;
  return o;
}

bfs::MeasurementUpdateResult EmptyResult()
{
  bfs::MeasurementUpdateResult r;
  return r;
}
} // namespace

extern "C" {

int32_t fw_algo_abi_version(void)
{
  return FW_ALGO_ABI_VERSION;
}

const char *fw_algo_build_info(void)
{
  static const char kInfo[] =
      "fw_algo abi=1 impl=bfs::Ekf15State(firmware) "
#if defined(EIGEN_DONT_VECTORIZE)
      "eigen=scalar "
#else
      "eigen=vectorized "
#endif
#if defined(_WIN32)
      "platform=win32 "
#else
      "platform=posix "
#endif
      "built=" __DATE__ " " __TIME__;
  return kInfo;
}

fw_ekf *fw_ekf_create(void)
{
  return new (std::nothrow) fw_ekf();
}

void fw_ekf_destroy(fw_ekf *h)
{
  delete h;
}

int32_t fw_ekf_init(fw_ekf *h, const fw_ekf_init_t *init)
{
  if (h == nullptr || init == nullptr)
  {
    return -1;
  }
  h->ekf.Initialize(ToVec3(init->accel_mps2), ToVec3(init->gyro_radps),
                    ToVec3(init->mag_ut), ToVec3(init->vel_ned_mps),
                    ToVec3d(init->lla_rad_m), init->yaw_offset_rad);
  h->initialized = true;
  return 0;
}

void fw_ekf_time_update(fw_ekf *h, const float accel_mps2[3],
                        const float gyro_radps[3], float dt_s)
{
  if (h == nullptr || accel_mps2 == nullptr || gyro_radps == nullptr)
  {
    return;
  }
  h->ekf.TimeUpdate(ToVec3(accel_mps2), ToVec3(gyro_radps), dt_s);
}

void fw_ekf_time_update_two_sample(fw_ekf *h, const float dth1[3],
                                   const float dth2[3], const float dv1[3],
                                   const float dv2[3], float dt_s)
{
  if (h == nullptr || dth1 == nullptr || dth2 == nullptr || dv1 == nullptr ||
      dv2 == nullptr)
  {
    return;
  }
  h->ekf.TimeUpdateTwoSample(ToVec3(dth1), ToVec3(dth2), ToVec3(dv1),
                             ToVec3(dv2), dt_s);
}

fw_update_result fw_ekf_gnss_update(fw_ekf *h, const float vel_ned_mps[3],
                                    const double lla_rad_m[3], float dt_avg_s,
                                    float measurement_age_s,
                                    int32_t require_delayed_snapshot)
{
  if (h == nullptr || vel_ned_mps == nullptr || lla_rad_m == nullptr)
  {
    return ToAbi(EmptyResult());
  }
  return ToAbi(h->ekf.MeasurementUpdateDetailed(
      ToVec3(vel_ned_mps), ToVec3d(lla_rad_m), dt_avg_s, measurement_age_s,
      require_delayed_snapshot != 0));
}

fw_update_result fw_ekf_mag_update(fw_ekf *h, const float mag_body_ut[3],
                                   const float mag_ned_ut[3], float noise_ut)
{
  if (h == nullptr || mag_body_ut == nullptr || mag_ned_ut == nullptr)
  {
    return ToAbi(EmptyResult());
  }
  return ToAbi(h->ekf.MeasurementUpdateMagVectorDetailed(
      ToVec3(mag_body_ut), ToVec3(mag_ned_ut), noise_ut));
}

fw_update_result fw_ekf_baro_update(fw_ekf *h, float altitude_m,
                                    float noise_std_m)
{
  if (h == nullptr)
  {
    return ToAbi(EmptyResult());
  }
  return ToAbi(h->ekf.MeasurementUpdateBaroAltitudeDetailed(altitude_m,
                                                            noise_std_m));
}

fw_update_result fw_ekf_aiding_velocity_update(fw_ekf *h,
                                               const float vel_ned_mps[3],
                                               float vel_ne_std_mps,
                                               float vel_d_std_mps)
{
  if (h == nullptr || vel_ned_mps == nullptr)
  {
    return ToAbi(EmptyResult());
  }
  return ToAbi(h->ekf.MeasurementUpdateAidingVelocityDetailed(
      ToVec3(vel_ned_mps), vel_ne_std_mps, vel_d_std_mps));
}

fw_update_result fw_ekf_gravity_update(fw_ekf *h, const float accel_mps2[3],
                                       float noise_rad)
{
  if (h == nullptr || accel_mps2 == nullptr)
  {
    return ToAbi(EmptyResult());
  }
  return ToAbi(h->ekf.MeasurementUpdateGravityDetailed(ToVec3(accel_mps2),
                                                       noise_rad));
}

fw_update_result fw_ekf_static_gyro_update(fw_ekf *h,
                                           const float gyro_radps[3],
                                           float noise_radps)
{
  if (h == nullptr || gyro_radps == nullptr)
  {
    return ToAbi(EmptyResult());
  }
  return ToAbi(h->ekf.MeasurementUpdateStaticGyroDetailed(ToVec3(gyro_radps),
                                                          noise_radps));
}

fw_update_result fw_ekf_yaw_update(fw_ekf *h, float yaw_rad, float noise_rad)
{
  if (h == nullptr)
  {
    return ToAbi(EmptyResult());
  }
  return ToAbi(h->ekf.MeasurementUpdateYawDetailed(yaw_rad, noise_rad));
}

void fw_ekf_reset_position_velocity(fw_ekf *h, const float vel_ned_mps[3],
                                    const double lla_rad_m[3])
{
  if (h == nullptr || vel_ned_mps == nullptr || lla_rad_m == nullptr)
  {
    return;
  }
  h->ekf.ResetPositionVelocityToGnss(ToVec3(vel_ned_mps), ToVec3d(lla_rad_m));
}

void fw_ekf_reset_velocity(fw_ekf *h, const float vel_ned_mps[3],
                           float vel_std_mps)
{
  if (h == nullptr || vel_ned_mps == nullptr)
  {
    return;
  }
  h->ekf.ResetVelocityToExternal(ToVec3(vel_ned_mps), vel_std_mps);
}

void fw_ekf_set_gnss_gates(fw_ekf *h, int32_t consider_heading,
                           int32_t reanchor_allowed,
                           int32_t vertical_reanchor_allowed)
{
  if (h == nullptr)
  {
    return;
  }
  h->ekf.set_gnss_consider_heading(consider_heading != 0);
  h->ekf.set_gnss_reanchor_allowed(reanchor_allowed != 0);
  h->ekf.set_gnss_vertical_reanchor_allowed(vertical_reanchor_allowed != 0);
}

void fw_ekf_set_velocity_subset(fw_ekf *h, int32_t enabled,
                                float max_pos_resid_m,
                                int32_t max_fires_per_epoch)
{
  if (h == nullptr)
  {
    return;
  }
  h->ekf.set_gnss_velocity_subset_recovery_enabled(enabled != 0);
  h->ekf.set_gnss_velocity_subset_max_pos_resid_m(max_pos_resid_m);
  const int fires = max_fires_per_epoch < 0 ? 0
                    : (max_fires_per_epoch > 255 ? 255 : max_fires_per_epoch);
  h->ekf.set_gnss_velocity_subset_max_fires_per_epoch(
      static_cast<uint8_t>(fires));
}

void fw_ekf_set_imu_limits(fw_ekf *h, float accel_mps2, float gyro_radps)
{
  if (h == nullptr)
  {
    return;
  }
  h->ekf.set_imu_plausibility_limits(accel_mps2, gyro_radps);
}

void fw_ekf_set_q_accel_scale(fw_ekf *h, float scale)
{
  if (h == nullptr)
  {
    return;
  }
  h->ekf.set_q_accel_scale(scale);
}

void fw_ekf_set_lever_arm(fw_ekf *h, const float lever_body_m[3])
{
  if (h == nullptr || lever_body_m == nullptr)
  {
    return;
  }
  h->ekf.gnss_lever_arm_body_m(ToVec3(lever_body_m));
}

void fw_ekf_get_state(fw_ekf *h, fw_nav_state *out)
{
  if (h == nullptr || out == nullptr)
  {
    return;
  }
  const bfs::Ekf15State &e = h->ekf;
  const Eigen::Vector3d lla = e.lla_rad_m();
  out->lla_rad_m[0] = lla(0);
  out->lla_rad_m[1] = lla(1);
  out->lla_rad_m[2] = lla(2);

  const Eigen::Vector3f v = e.ned_vel_mps();
  const Eigen::Vector3f ba = e.accel_bias_mps2();
  const Eigen::Vector3f bg = e.gyro_bias_radps();
  for (int i = 0; i < 3; ++i)
  {
    out->vel_ned_mps[i] = v(i);
    out->accel_bias_mps2[i] = ba(i);
    out->gyro_bias_radps[i] = bg(i);
  }

  const Eigen::Quaternionf q = e.quat();
  out->quat_wxyz[0] = q.w();
  out->quat_wxyz[1] = q.x();
  out->quat_wxyz[2] = q.y();
  out->quat_wxyz[3] = q.z();

  out->euler_ypr_rad[0] = e.yaw_rad();
  out->euler_ypr_rad[1] = e.pitch_rad();
  out->euler_ypr_rad[2] = e.roll_rad();

  out->yaw_sigma_rad = e.yaw_sigma_rad();
  out->tilt_sigma_rad = e.tilt_sigma_rad();
  out->gravity_mps2 = static_cast<float>(e.gravity_mps2());

  for (int i = 0; i < FW_EKF_STATE_DIM; ++i)
  {
    out->cov_diag[i] = e.covariance_coeff(i, i);
  }

  out->gnss_reacquire_reset_count = e.gnss_reacquire_reset_count();
  out->gnss_velocity_reanchor_count = e.gnss_velocity_reanchor_count();
  out->gnss_outlier_reject_count = e.gnss_outlier_reject_count();
  out->mag_fused_count = e.mag_vector_fused_count();
  out->mag_reject_count = e.mag_vector_reject_count();
  out->vel_subset_count = e.gnss_velocity_subset_recovery_count();

  out->last_gnss_resid_pos_ne_m = e.gnss_resid_pos_ne_m();
  out->last_gnss_resid_vel_ne_mps = e.gnss_resid_vel_ne_mps();
  out->last_gnss_gate_pos = e.gnss_gate_pos_outlier() ? 1 : 0;
  out->last_gnss_gate_vel = e.gnss_gate_vel_outlier() ? 1 : 0;
  out->time_since_gnss_s = e.time_since_gnss_update_s();
}

int32_t fw_ekf_diag_json(fw_ekf *h, char *buf, int32_t cap)
{
  if (h == nullptr || buf == nullptr || cap <= 0)
  {
    return -1;
  }
  buf[0] = '\0';
  fw_nav_state s;
  fw_ekf_get_state(h, &s);
  const int n = std::snprintf(
      buf, static_cast<size_t>(cap),
      "{\"abi\":%d,\"lla\":[%.9f,%.9f,%.4f],"
      "\"vel\":[%.6f,%.6f,%.6f],"
      "\"quat\":[%.7f,%.7f,%.7f,%.7f],"
      "\"ypr\":[%.7f,%.7f,%.7f],"
      "\"ba\":[%.6f,%.6f,%.6f],\"bg\":[%.7f,%.7f,%.7f],"
      "\"sig_yaw\":%.6f,\"sig_tilt\":%.6f,\"g\":%.6f,"
      "\"cov_diag\":[%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,"
      "%.6g,%.6g,%.6g,%.6g,%.6g],"
      "\"tsg\":%.3f,\"res_p\":%.3f,\"res_v\":%.3f,"
      "\"gate_pos\":%d,\"gate_vel\":%d,"
      "\"reacq\":%u,\"vreanch\":%u,\"outlier\":%u,"
      "\"mag_f\":%u,\"mag_r\":%u,\"vsub\":%u}",
      FW_ALGO_ABI_VERSION, s.lla_rad_m[0], s.lla_rad_m[1], s.lla_rad_m[2],
      s.vel_ned_mps[0], s.vel_ned_mps[1], s.vel_ned_mps[2], s.quat_wxyz[0],
      s.quat_wxyz[1], s.quat_wxyz[2], s.quat_wxyz[3], s.euler_ypr_rad[0],
      s.euler_ypr_rad[1], s.euler_ypr_rad[2], s.accel_bias_mps2[0],
      s.accel_bias_mps2[1], s.accel_bias_mps2[2], s.gyro_bias_radps[0],
      s.gyro_bias_radps[1], s.gyro_bias_radps[2], s.yaw_sigma_rad,
      s.tilt_sigma_rad, s.gravity_mps2, s.cov_diag[0], s.cov_diag[1],
      s.cov_diag[2], s.cov_diag[3], s.cov_diag[4], s.cov_diag[5],
      s.cov_diag[6], s.cov_diag[7], s.cov_diag[8], s.cov_diag[9],
      s.cov_diag[10], s.cov_diag[11], s.cov_diag[12], s.cov_diag[13],
      s.cov_diag[14], s.time_since_gnss_s, s.last_gnss_resid_pos_ne_m,
      s.last_gnss_resid_vel_ne_mps, s.last_gnss_gate_pos, s.last_gnss_gate_vel,
      s.gnss_reacquire_reset_count, s.gnss_velocity_reanchor_count,
      s.gnss_outlier_reject_count, s.mag_fused_count, s.mag_reject_count,
      s.vel_subset_count);
  if (n < 0 || n >= cap)
  {
    buf[0] = '\0';
    return -1;
  }
  return n;
}

} // extern "C"
