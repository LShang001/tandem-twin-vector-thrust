/**
 * @file    fw_nav_api.cpp
 * @module  fw-nav/固件核心宿主直调 C ABI（导航步进）
 * @brief   薄封装固件 `handleNavigationSystem()`：把宿主准备的每拍输入写进
 *          `state_data` 全局量（聚合 IMU / AHRS / 气压 / 磁 / 光流 / 解锁标志），
 *          UBX 历元经真实解析器入队，调用固件导航任务后读回 EKF 状态。
 *
 * @details 与 SIL 主循环（`fw_sil_env_main.cpp` 的 real-nav 分支）同一套全局量映射：
 *          `acc_sum_*`/`imu_delta_*`（2 kHz 聚合）、`backup_ahrs_*`、`baro_altitude`、
 *          `magSensor.silInjectFrdSample`、`flow_data`、`Serial4.feed + ubx.Pump`、
 *          `g_is_unlocked`/`throttlePercent`。本文件不复制任何导航/滤波算式：
 *          传感器层的 2 kHz 低通与 Madgwick 由调用方负责（见头文件边界说明）。
 * @frame  NED / FRD；四元数 FRD←NED，欧拉角 theta=−asin(R13)
 * @units  SI；`euler_rpy_deg`/`lla_deg` 在边界处转度
 * @note   单例语义（EKF/解析器/传感器对象都是全局量）；`reset()` 重建 EKF 并清计数。
 * @see     fw_nav_api.h、src/navigation_task.cpp、src/state_data.h、CTL-004
 */
#include "fw_nav_api.h"

#include <cmath>
#include <cstring>
#include <new>

#include "Arduino.h"
#include "QuaternionMath.h"
#include "math_utils.h"
#include "navigation_task.h"
#include "sil_sensor_model.h"
#include "state_data.h"
#include "ubx.h"

struct fw_nav
{
  uint32_t steps = 0;
  double sim_time_s = 0.0;
};

namespace
{
fw_nav *g_nav_instance = nullptr;

bool AllFinite(const float *v, int n)
{
  for (int i = 0; i < n; ++i)
  {
    if (!std::isfinite(v[i]))
    {
      return false;
    }
  }
  return true;
}

void ClearImuAccumulators()
{
  acc_sum_x = acc_sum_y = acc_sum_z = 0.0f;
  gyro_sum_x = gyro_sum_y = gyro_sum_z = 0.0f;
  imu_sample_count = 0;
  imu_delta_sample_count = 0;
  imu_delta_overflow = false;
  imu_delta_time_sum_s = 0.0f;
  for (int i = 0; i < NAV_IMU_DELTA_BUFFER_SIZE; ++i)
  {
    imu_delta_theta_x[i] = imu_delta_theta_y[i] = imu_delta_theta_z[i] = 0.0f;
    imu_delta_v_x[i] = imu_delta_v_y[i] = imu_delta_v_z[i] = 0.0f;
    imu_delta_dt_s[i] = 0.0f;
  }
}

void InjectImu(const fw_nav_input_t *in)
{
  // sample_count=0 ⇒ 本拍无 IMU 样本：原样传给固件（它会走"无样本"分支并推进健康计时），
  // 不要伪造成 1，否则与 SIL/生产在启动/丢样拍上分叉。
  const int count = (in->sample_count > 0) ? in->sample_count : 0;
  acc_sum_x = in->accel_sum_body_mps2[0];
  acc_sum_y = in->accel_sum_body_mps2[1];
  acc_sum_z = in->accel_sum_body_mps2[2];
  gyro_sum_x = in->gyro_sum_body_radps[0];
  gyro_sum_y = in->gyro_sum_body_radps[1];
  gyro_sum_z = in->gyro_sum_body_radps[2];
  imu_sample_count = count;

  int n = in->delta_count;
  if (n < 0)
  {
    n = 0;
  }
  if (n > FW_NAV_MAX_DELTA_SAMPLES)
  {
    n = FW_NAV_MAX_DELTA_SAMPLES;
  }
  float t_sum = 0.0f;
  for (int i = 0; i < n; ++i)
  {
    imu_delta_theta_x[i] = in->delta_theta_rad[i][0];
    imu_delta_theta_y[i] = in->delta_theta_rad[i][1];
    imu_delta_theta_z[i] = in->delta_theta_rad[i][2];
    imu_delta_v_x[i] = in->delta_vel_mps[i][0];
    imu_delta_v_y[i] = in->delta_vel_mps[i][1];
    imu_delta_v_z[i] = in->delta_vel_mps[i][2];
    imu_delta_dt_s[i] = in->delta_dt_s[i];
    t_sum += in->delta_dt_s[i];
  }
  imu_delta_sample_count = n;
  imu_delta_overflow = false;
  imu_delta_time_sum_s = t_sum;
}

void InjectAhrs(const fw_nav_input_t *in)
{
  if (!in->ahrs_valid)
  {
    return;
  }
  backup_ahrs_Qw = in->ahrs_quat_wxyz[0];
  backup_ahrs_Qx = in->ahrs_quat_wxyz[1];
  backup_ahrs_Qy = in->ahrs_quat_wxyz[2];
  backup_ahrs_Qz = in->ahrs_quat_wxyz[3];
  const Quaternion q{static_cast<double>(backup_ahrs_Qw),
                     static_cast<double>(backup_ahrs_Qx),
                     static_cast<double>(backup_ahrs_Qy),
                     static_cast<double>(backup_ahrs_Qz)};
  float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
  quaternionToEuler(q, roll, pitch, yaw);
  backup_ahrs_roll = roll;
  backup_ahrs_pitch = pitch;
  // 与 sensor_imu.cpp 同口径：yaw 已叠加航向校正量（这里校正量恒 0）。
  backup_ahrs_yaw = wrapAnglePi(yaw + ahrs_yaw_correction_rad);
}

void InjectSensors(const fw_nav_input_t *in)
{
  if (in->baro_valid)
  {
    baro_altitude_raw = in->baro_alt_up_m;
    baro_altitude = in->baro_alt_up_m;
  }
  if (in->mag_valid)
  {
    magSensor.silInjectFrdSample(in->mag_body_ut, micros(), true);
  }
  // 光流是**事件型**输入：`flow_valid=1` 表示本拍更新了光流状态（有新样本，
  // 或显式判失效——用 quality=0 表达），不是"状态保持"。不在事件拍上不要写
  // `flow_data`：否则会把 10 Hz 的样本刷新成 200 Hz（融合率、新鲜度窗口全变）。
  if (in->flow_valid)
  {
    flow_data.is_flow_valid = (in->flow_quality > 0);
    flow_data.is_range_valid = (in->range_quality > 0);
    if (flow_data.is_flow_valid)
    {
      flow_data.velocity_x_mps = in->flow_vx_mps;
      flow_data.velocity_y_mps = in->flow_vy_mps;
      flow_data.flow_quality = in->flow_quality;
      flow_data.timestamp_ms = millis();
      flow_data.last_update_ms = millis();
    }
    if (flow_data.is_range_valid)
    {
      flow_data.distance_m = in->flow_range_m;
      flow_data.range_quality = in->range_quality;
      flow_data.last_range_update_ms = millis();
    }
  }
  g_is_unlocked = (in->unlocked != 0);
  throttlePercent = in->throttle_pct;
}

void InjectGnss(const fw_nav_input_t *in)
{
  sil::UbxPvtFields f;
  f.itow = in->gnss_itow_ms;
  f.lat_1e7 = in->gnss_lat_1e7;
  f.lon_1e7 = in->gnss_lon_1e7;
  f.height_mm = in->gnss_alt_mm;
  f.h_msl_mm = f.height_mm;
  f.vel_n_mmps = in->gnss_vel_ned_mmps[0];
  f.vel_e_mmps = in->gnss_vel_ned_mmps[1];
  f.vel_d_mmps = in->gnss_vel_ned_mmps[2];
  f.g_speed_mmps = static_cast<int32_t>(std::hypot(
      static_cast<double>(in->gnss_vel_ned_mmps[0]),
      static_cast<double>(in->gnss_vel_ned_mmps[1])));
  f.fix_type = 3;
  f.num_sv = 14;
  f.h_acc_mm = in->gnss_h_acc_mm;
  f.v_acc_mm = in->gnss_v_acc_mm;
  f.s_acc_mmps = in->gnss_s_acc_mmps;
  f.p_dop_1e2 = 120U;
  uint8_t frame[128];
  size_t n = sil::BuildNavPvt(f, frame, sizeof(frame));
  Serial4.feed(frame, n);
  n = sil::BuildNavEoe(f.itow, frame, sizeof(frame));
  Serial4.feed(frame, n);
  ubx.Pump(512, micros());
}

void ReadOutputs(fw_nav_output_t *out)
{
  const Eigen::Quaternionf q = nav_ekf.quat();
  out->quat_wxyz[0] = q.w();
  out->quat_wxyz[1] = q.x();
  out->quat_wxyz[2] = q.y();
  out->quat_wxyz[3] = q.z();
  out->euler_rpy_deg[0] = nav_ekf.roll_rad() * 57.29577951308232f;
  out->euler_rpy_deg[1] = nav_ekf.pitch_rad() * 57.29577951308232f;
  out->euler_rpy_deg[2] = nav_ekf.yaw_rad() * 57.29577951308232f;
  const Eigen::Vector3d lla = nav_ekf.lla_rad_m();
  out->lla_deg[0] = lla(0) * 57.29577951308232;
  out->lla_deg[1] = lla(1) * 57.29577951308232;
  out->lla_deg[2] = lla(2);
  out->pos_ned_m[0] = relative_north;
  out->pos_ned_m[1] = relative_east;
  out->pos_ned_m[2] = relative_down;
  const Eigen::Vector3f v = nav_ekf.ned_vel_mps();
  out->vel_ned_mps[0] = v.x();
  out->vel_ned_mps[1] = v.y();
  out->vel_ned_mps[2] = v.z();
  out->alt_up_m = estimated_height;
  const Eigen::Vector3f ba = nav_ekf.accel_bias_mps2();
  const Eigen::Vector3f bg = nav_ekf.gyro_bias_radps();
  for (int i = 0; i < 3; ++i)
  {
    out->accel_bias_mps2[i] = ba(i);
    out->gyro_bias_radps[i] = bg(i);
  }
  out->yaw_sigma_deg = nav_ekf.yaw_sigma_rad() * 57.29577951308232f;
  out->tilt_sigma_deg = nav_ekf.tilt_sigma_rad() * 57.29577951308232f;
  out->initialized = nav_system_initialized ? 1 : 0;
  out->gnss_fix3d = (ubx.fix() >= bfs::Ubx::FIX_3D) ? 1 : 0;
  out->gnss_epoch_count = gnss_status.epoch_count;
  out->gnss_fix3d_count = gnss_status.fix3d_count;
  out->gnss_quality_fail = gnss_status.quality_fail_count;
  out->gnss_duplicate = gnss_status.duplicate_itow_count;
  out->gnss_age_s = gnss_status.last_measurement_age_s;
  out->baro_fused = baro_ekf_fused ? 1 : 0;
  out->baro_innov_m = baro_ekf_innovation_m;
  out->heading_aiding = g_heading_hold_allowed ? 1 : 0;
  out->pos_degraded = g_pos_hold_degraded ? 1 : 0;
  out->imu_health = static_cast<int32_t>(g_imu_health_state);
  out->imu_health_flags = g_imu_health_flags;
  out->gnss_queued = ubx.queued_epoch_count();
  out->gnss_pvt = ubx.nav_pvt_count();
  out->gnss_eoe = ubx.nav_eoe_count();
  out->gnss_overflow = ubx.queue_overflow_count();
  out->last_epoch_tow_ms = gnss_status.last_epoch_tow_ms;
  out->steps = g_nav_instance != nullptr ? g_nav_instance->steps : 0U;
}
}  // namespace

int32_t fw_nav_create(fw_nav **out)
{
  if (out == nullptr)
  {
    return 1;
  }
  if (g_nav_instance == nullptr)
  {
    g_nav_instance = new (std::nothrow) fw_nav();
    if (g_nav_instance == nullptr)
    {
      return 1;
    }
  }
  *out = g_nav_instance;
  return 0;
}

void fw_nav_destroy(fw_nav *handle)
{
  if (handle != nullptr && handle == g_nav_instance)
  {
    delete g_nav_instance;
    g_nav_instance = nullptr;
  }
}

int32_t fw_nav_reset(fw_nav *handle)
{
  if (handle == nullptr)
  {
    return 1;
  }
  handle->steps = 0;
  handle->sim_time_s = 0.0;
  ClearImuAccumulators();
  gnss_status = GnssRuntimeStatus{};
  // Ekf15State 含 const 成员（编译期常量），不可赋值 ⇒ 就地重建（全局对象，
  // 不涉及多态/资源）。这是"回到刚上电"语义，而不是保留旧协方差。
  nav_ekf.~Ekf15State();
  new (&nav_ekf) bfs::Ekf15State();
  nav_system_initialized = false;
  estimated_height = 0.0f;
  relative_north = relative_east = relative_down = 0.0f;
  ahrs_yaw_correction_rad = 0.0f;
  return 0;
}

int32_t fw_nav_step(fw_nav *handle, const fw_nav_input_t *in,
                    fw_nav_output_t *out)
{
  if (handle == nullptr || in == nullptr || out == nullptr)
  {
    return 1;
  }
  if (in->abi_version != FW_NAV_ABI_VERSION)
  {
    return 2;
  }
  if (in->struct_size < sizeof(fw_nav_input_t))
  {
    return 3;
  }
  if (out->struct_size < sizeof(fw_nav_output_t))
  {
    return 4;
  }
  if (!std::isfinite(in->dt_s) || in->dt_s <= 0.0f ||
      !AllFinite(in->accel_sum_body_mps2, 3) ||
      !AllFinite(in->gyro_sum_body_radps, 3) ||
      !AllFinite(in->ahrs_quat_wxyz, 4) ||
      !std::isfinite(in->baro_alt_up_m) || !std::isfinite(in->throttle_pct))
  {
    return 5;
  }

  handle->sim_time_s += static_cast<double>(in->dt_s);
  g_sil_time_us = static_cast<uint32_t>(handle->sim_time_s * 1.0e6);

  InjectImu(in);
  InjectAhrs(in);
  InjectSensors(in);
  if (in->gnss_valid)
  {
    InjectGnss(in);
  }
  handleNavigationSystem();

  const uint32_t out_size = out->struct_size;
  std::memset(out, 0, sizeof(fw_nav_output_t));
  out->struct_size = out_size;
  out->abi_version = FW_NAV_ABI_VERSION;
  ++handle->steps;
  ReadOutputs(out);
  return 0;
}
