/**
 * @file    fw_gnc_api.cpp
 * @module  fw-control/固件核心宿主直调 C ABI（控制链 + 能力/参数）
 * @brief   薄封装固件 `runGNCExecutive()` 与参数注册表：注入 state_data 全局量 →
 *          调用固件 → 读回输出；能力查询回报 ABI 版本与编译期特性位。
 *
 * @details 与 SIL 主循环 `feed_firmware()` + `runGNCExecutive()` 同一套全局量映射：
 *          AHRS_Packet（姿态）、INS_GNSS_Packet（速度）、relative_*（位置）、
 *          estimated_height、current_omega_dps_body_filtered、raw_rc_values、
 *          isLinkUp/failsafe_in_flight、target_altitude。本文件不复制任何控制律，
 *          参数读写直接复用 `ano_params.cpp` 的注册表（与 GCS/MAVLink 同源）。
 *
 * @frame  NED / FRD；四元数 FRD←NED，欧拉角 theta=−asin(R13)（与固件一致）
 * @units  SI + PWM µs + 摆角 deg（边界处换算）
 * @note   单例语义：控制层 PID/滤波器为全局或函数静态状态，一个进程一个实例。
 *         输入角速度必须已去偏（见头文件）。
 * @see     fw_gnc_api.h、src/flight_control.cpp、src/state_data.h、src/ano_params.cpp
 */
#include "fw_gnc_api.h"

#include <cmath>
#include <cstring>
#include <new>

#include "QuaternionMath.h"
#include "ano_params.h"
#include "flight_control.h"
#include "fw_algo_api.h"
#include "state_data.h"

#ifndef FW_BUILD_ID
#define FW_BUILD_ID "dev"
#endif

struct fw_gnc
{
  uint32_t steps = 0;
  double sim_time_s = 0.0;
};

namespace
{
/** 控制分配 ABI 版本（fw_ctrl_api.cpp 手写常量，这里做一致性提示用）。 */
constexpr uint32_t kCtrlAbiVersion = 2U;

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

bool InputValid(const fw_gnc_input_t *in)
{
  return AllFinite(in->quat_wxyz, 4) && AllFinite(in->omega_body_radps, 3) &&
         AllFinite(in->vel_ned_mps, 3) && AllFinite(in->pos_ned_m, 3) &&
         std::isfinite(in->alt_up_m) && std::isfinite(in->target_alt_m) &&
         std::isfinite(in->dt_s) && in->dt_s > 0.0f;
}

uint32_t CompileTimeFeatures()
{
  uint32_t bits = FW_FEAT_GNC_AVAILABLE;
#ifdef BFS_NAVIGATION_RAW_IMU_INCREMENT
  bits |= FW_FEAT_RAW_IMU_INCREMENT;
#endif
#ifdef BFS_GNSS_OUTAGE_VELOCITY_HOLD
  bits |= FW_FEAT_GNSS_OUTAGE_VEL_HOLD;
#endif
#ifdef BFS_GNSS_OUTAGE_ALT_HOLD
  bits |= FW_FEAT_GNSS_OUTAGE_ALT_HOLD;
#endif
#ifdef BFS_LONG_GNSS_OUTAGE_DESCENT
  bits |= FW_FEAT_LONG_OUTAGE_DESCENT;
#endif
#ifdef BFS_BARO_HEALTH_GATE
  bits |= FW_FEAT_BARO_HEALTH_GATE;
#endif
#ifdef BFS_GNSS_VERTICAL_REANCHOR_GATE
  bits |= FW_FEAT_GNSS_VERTICAL_GATE;
#endif
  return bits;
}

uint32_t CompileTimeVehicle()
{
#if defined(BFS_COAXIAL_VTVL)
  return FW_VEHICLE_COAXIAL_VTVL;
#elif defined(BFS_LIQUID_ROCKET)
  return FW_VEHICLE_LIQUID_ROCKET;
#else
  return FW_VEHICLE_TANDEMVEC;
#endif
}

void InjectState(const fw_gnc_input_t *in)
{
  // 姿态：固件 AHRS_Packet 存 FRD←NED 四元数与 rad 欧拉角
  AHRS_Packet.Qw = in->quat_wxyz[0];
  AHRS_Packet.Qx = in->quat_wxyz[1];
  AHRS_Packet.Qy = in->quat_wxyz[2];
  AHRS_Packet.Qz = in->quat_wxyz[3];
  const Quaternion q{static_cast<double>(in->quat_wxyz[0]),
                     static_cast<double>(in->quat_wxyz[1]),
                     static_cast<double>(in->quat_wxyz[2]),
                     static_cast<double>(in->quat_wxyz[3])};
  float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
  quaternionToEuler(q, roll, pitch, yaw);
  AHRS_Packet.Roll = roll;
  AHRS_Packet.Pitch = pitch;
  AHRS_Packet.Heading = yaw;

  // 速度/位置：与 SIL feed_firmware 同源
  INS_GNSS_Packet.location_north = in->pos_ned_m[0];
  INS_GNSS_Packet.location_east = in->pos_ned_m[1];
  INS_GNSS_Packet.location_down = in->pos_ned_m[2];
  INS_GNSS_Packet.velocity_north = in->vel_ned_mps[0];
  INS_GNSS_Packet.velocity_east = in->vel_ned_mps[1];
  INS_GNSS_Packet.velocity_down = in->vel_ned_mps[2];
  relative_north = in->pos_ned_m[0];
  relative_east = in->pos_ned_m[1];
  relative_down = -in->alt_up_m;
  estimated_height = in->alt_up_m;

  // 控制链吃 dps 域角速度（固件内部再乘 DEG_TO_RAD）
  current_omega_dps_body_filtered = Vector3(
      static_cast<double>(in->omega_body_radps[0]) * RAD_TO_DEG,
      static_cast<double>(in->omega_body_radps[1]) * RAD_TO_DEG,
      static_cast<double>(in->omega_body_radps[2]) * RAD_TO_DEG);

  for (int i = 0; i < 16; ++i)
  {
    raw_rc_values[i] = in->rc_pwm[i];
  }
  isLinkUp = (in->link_up != 0);
  failsafe_in_flight = (in->failsafe_in_flight != 0);
  target_altitude = in->target_alt_m;
}

void ReadOutputs(fw_gnc_output_t *out)
{
  out->ch1_pct = ch1_output;
  out->ch2_pct = ch2_output;
  out->ch3_pct = ch3_output;
  out->ch4_pct = ch4_output;
  out->flight_mode = static_cast<int32_t>(g_current_flight_mode);
  out->unlocked = g_is_unlocked ? 1 : 0;
  out->tvc_upper_deg = g_tvc_upper_deg;
  out->tvc_lower_deg = g_tvc_lower_deg;
  for (int i = 0; i < 3; ++i)
  {
    out->omega_ref_dps[i] = gnc_tel.omega_ref_dps[i];
    out->alpha_ref_dps2[i] = gnc_tel.alpha_ref_dps2[i];
    out->M_cmd[i] = gnc_tel.M_cmd[i];
  }
  out->delta_f_deg = gnc_tel.delta_f_deg;
  out->delta_t_deg = gnc_tel.delta_t_deg;
  out->dw = gnc_tel.dw;
  out->mode_degraded_heading = g_mode_degraded_heading ? 1 : 0;
  out->mode_degraded_gnss = g_mode_degraded_gnss ? 1 : 0;
  out->failsafe_in_flight = failsafe_in_flight ? 1 : 0;
  uint32_t faults = 0U;
  if (g_actuator_servo_jam) { faults |= 1U << 0; }
  if (g_actuator_thrust_loss) { faults |= 1U << 1; }
  if (g_actuator_controlled_descent_request) { faults |= 1U << 2; }
  out->actuator_fault_bits = faults;
}
} // namespace

void fw_core_caps(fw_core_caps_t *out)
{
  if (out == nullptr || out->struct_size == 0U)
  {
    return;
  }
  fw_core_caps_t caps{};
  caps.struct_size = sizeof(caps);
  caps.core_abi_version = FW_CORE_ABI_VERSION;
  caps.ekf_abi_version = FW_ALGO_ABI_VERSION;
  caps.ctrl_abi_version = kCtrlAbiVersion;
  caps.gnc_abi_version = FW_GNC_ABI_VERSION;
  caps.param_count = anoParamCount();
  caps.vehicle_profile = CompileTimeVehicle();
  caps.feature_bits = CompileTimeFeatures();
  std::strncpy(caps.build_id, FW_BUILD_ID, sizeof(caps.build_id) - 1U);
  const uint32_t n = (out->struct_size < sizeof(caps)) ? out->struct_size : sizeof(caps);
  std::memcpy(out, &caps, n);
}

fw_gnc *fw_gnc_create(void)
{
  static fw_gnc *instance = nullptr;
  if (instance == nullptr)
  {
    instance = new (std::nothrow) fw_gnc();
  }
  return instance;
}

void fw_gnc_destroy(fw_gnc *handle)
{
  delete handle;
}

int32_t fw_gnc_step(fw_gnc *handle, const fw_gnc_input_t *in, fw_gnc_output_t *out)
{
  if (handle == nullptr || in == nullptr || out == nullptr)
  {
    return 1;
  }
  if (in->abi_version != FW_GNC_ABI_VERSION)
  {
    return 2;
  }
  if (in->struct_size < sizeof(fw_gnc_input_t))
  {
    return 3;
  }
  if (out->struct_size < sizeof(fw_gnc_output_t))
  {
    return 4;
  }
  if (!InputValid(in))
  {
    return 5;
  }

  const uint32_t out_size = out->struct_size;
  std::memset(out, 0, sizeof(fw_gnc_output_t)); // 只清零本库已知长度
  out->struct_size = out_size;
  out->abi_version = FW_GNC_ABI_VERSION;

  // 虚拟时钟推进（固件用 millis()/micros() 做超时与调度判断）
  handle->sim_time_s += static_cast<double>(in->dt_s);
  g_sil_time_us = static_cast<uint32_t>(handle->sim_time_s * 1.0e6);

  InjectState(in);
  runGNCExecutive();
  ReadOutputs(out);
  ++handle->steps;
  return 0;
}

uint32_t fw_param_count(void)
{
  return anoParamCount();
}

const char *fw_param_name(uint32_t id)
{
  return (id < anoParamCount()) ? anoParamNameAt(static_cast<uint16_t>(id)) : nullptr;
}

int32_t fw_param_find(const char *name)
{
  return anoParamIdByName(name);
}

int32_t fw_param_get(uint32_t id, float *out)
{
  if (out == nullptr || id >= anoParamCount())
  {
    return 1;
  }
  return anoParamReadFloat(static_cast<uint16_t>(id), out) ? 0 : 1;
}

int32_t fw_param_set(uint32_t id, float value)
{
  if (id >= anoParamCount())
  {
    return 1;
  }
  return anoParamWriteFloat(static_cast<uint16_t>(id), value) ? 0 : 1;
}
