/**
 * @file fw_ctrl_api.cpp
 * @module fw-control/机型分配宿主直调
 * @brief 固件控制/分配层的 C ABI 薄封装：TandemVec 机型分配 + 垂直推力分配。
 *
 * @details 直接调用 include/TandemVecAllocator.h 与 include/ThrustAllocation.h 的
 *          内联实现，不复制任何公式。参数默认值取自 TandemVec_Config.h 的
 *          kDefaultTandemVecParams，避免 Python 侧再维护一份机型参数。
 * @frame  α/gyro 为存档系（x=滚转、y=俯仰、z=推力轴）；NED 天向用于推力分配。
 * @units  α rad/s²、ω rad/s、M N·m、摆角 rad、电机 %、推力 N。
 * @note  这是宿主/研究接口，不参与固件镜像编译。
 * @see    fw_algo_api.h、include/TandemVecAllocator.h、include/ThrustAllocation.h
 */
#include "fw_algo_api.h"

#include <cmath>

#include "TandemVecAllocator.h"
#include "TandemVec_Config.h"
#include "ThrustAllocation.h"

namespace
{
TandemVecParams ToParams(const fw_tv_params &p)
{
  TandemVecParams out;
  out.kT = p.kT;
  out.kQ = p.kQ;
  out.Jp = p.Jp;
  out.wMax = p.wMax;
  out.a = p.a;
  out.b = p.b;
  out.tauM = p.tauM;
  out.Ix = p.Ix;
  out.Iy = p.Iy;
  out.Iz = p.Iz;
  out.dMax = p.dMax;
  out.dwMax = p.dwMax;
  out.m = p.m;
  out.g = p.g;
  return out;
}

TandemVecAllocatorState ToState(const fw_tv_alloc_state &s)
{
  TandemVecAllocatorState out;
  out.prev.wf = s.prev.wf;
  out.prev.wt = s.prev.wt;
  out.prev.delta_f = s.prev.delta_f;
  out.prev.delta_t = s.prev.delta_t;
  out.wf_est = s.wf_est;
  out.wt_est = s.wt_est;
  return out;
}

fw_tv_alloc_state FromState(const TandemVecAllocatorState &s)
{
  fw_tv_alloc_state out;
  out.prev.wf = s.prev.wf;
  out.prev.wt = s.prev.wt;
  out.prev.delta_f = s.prev.delta_f;
  out.prev.delta_t = s.prev.delta_t;
  out.wf_est = s.wf_est;
  out.wt_est = s.wt_est;
  return out;
}
} // namespace

extern "C" {

void fw_ctrl_default_params(fw_tv_params *out)
{
  if (out == nullptr)
  {
    return;
  }
  const TandemVecParams &p = kDefaultTandemVecParams;
  out->kT = p.kT;
  out->kQ = p.kQ;
  out->Jp = p.Jp;
  out->wMax = p.wMax;
  out->a = p.a;
  out->b = p.b;
  out->tauM = p.tauM;
  out->Ix = p.Ix;
  out->Iy = p.Iy;
  out->Iz = p.Iz;
  out->dMax = p.dMax;
  out->dwMax = p.dwMax;
  out->m = p.m;
  out->g = p.g;
}

void fw_ctrl_state_hover(const fw_tv_params *p, fw_tv_alloc_state *out)
{
  if (p == nullptr || out == nullptr)
  {
    return;
  }
  const float w_hover =
      (p->kT > 0.0f) ? std::sqrt(0.5f * p->m * p->g / p->kT) : 0.0f;
  out->prev.wf = w_hover;
  out->prev.wt = w_hover;
  out->prev.delta_f = 0.0f;
  out->prev.delta_t = 0.0f;
  out->wf_est = w_hover;
  out->wt_est = w_hover;
}

fw_tv_alloc_output fw_tandemvec_alloc(const fw_tv_params *p,
                                      const fw_tv_alloc_state *state,
                                      const fw_tv_alloc_input *in)
{
  fw_tv_alloc_output o;
  o.valid = 0;
  o.delta_f_rad = 0.0f;
  o.delta_t_rad = 0.0f;
  o.motor1_pct = 0.0f;
  o.motor2_pct = 0.0f;
  o.dw = 0.0f;
  for (int i = 0; i < 3; ++i)
  {
    o.M_cmd[i] = 0.0f;
    o.M_ff[i] = 0.0f;
  }
  o.w0_eff = 0.0f;
  o.yaw_gain_sched = 0.0f;
  o.sat_delta_f = 0;
  o.sat_delta_t = 0;
  o.sat_dw = 0;
  o.next.prev.wf = 0.0f;
  o.next.prev.wt = 0.0f;
  o.next.prev.delta_f = 0.0f;
  o.next.prev.delta_t = 0.0f;
  o.next.wf_est = 0.0f;
  o.next.wt_est = 0.0f;
  if (p == nullptr || state == nullptr || in == nullptr)
  {
    return o;
  }

  TandemVecAllocatorInput ai;
  ai.alpha_roll_radps2 = in->alpha_roll_radps2;
  ai.alpha_pitch_radps2 = in->alpha_pitch_radps2;
  ai.alpha_yaw_radps2 = in->alpha_yaw_radps2;
  ai.throttle_pct = in->throttle_pct;
  for (int i = 0; i < 3; ++i)
  {
    ai.gyro_body_radps[i] = in->gyro_body_radps[i];
  }
  ai.dt_s = in->dt_s;
  ai.inertia_comp_mask = static_cast<uint8_t>(in->inertia_comp_mask);

  const TandemVecAllocatorOutput r =
      allocateTandemVec(ai, ToState(*state), ToParams(*p));

  o.valid = r.valid ? 1 : 0;
  o.delta_f_rad = r.delta_f_rad;
  o.delta_t_rad = r.delta_t_rad;
  o.motor1_pct = r.motor1_pct;
  o.motor2_pct = r.motor2_pct;
  o.dw = r.dw;
  for (int i = 0; i < 3; ++i)
  {
    o.M_cmd[i] = r.M_cmd[i];
    o.M_ff[i] = r.M_ff[i];
  }
  o.w0_eff = r.w0_eff;
  o.yaw_gain_sched = r.yaw_gain_sched;
  o.sat_delta_f = r.sat_delta_f ? 1 : 0;
  o.sat_delta_t = r.sat_delta_t ? 1 : 0;
  o.sat_dw = r.sat_dw ? 1 : 0;
  o.next = FromState(r.next);
  return o;
}

fw_thrust_alloc_output fw_thrust_alloc(const fw_thrust_alloc_input *in)
{
  fw_thrust_alloc_output o;
  o.valid = 0;
  o.total_thrust_n = 0.0f;
  o.throttle_pct = 0.0f;
  o.saturated_high = 0;
  o.saturated_low = 0;
  if (in == nullptr)
  {
    return o;
  }
  bfs::ThrustAllocationInput bi;
  bi.specific_force_up_mps2 = in->specific_force_up_mps2;
  bi.thrust_axis_up_cos = in->thrust_axis_up_cos;
  bi.mass_kg = in->mass_kg;
  bi.max_total_thrust_n = in->max_total_thrust_n;
  bi.min_cos_tilt = in->min_cos_tilt;
  const bfs::ThrustAllocationOutput r = bfs::allocateLinearThrustThrottle(bi);
  o.valid = r.valid ? 1 : 0;
  o.total_thrust_n = r.total_thrust_n;
  o.throttle_pct = r.throttle_pct;
  o.saturated_high = r.saturated_high ? 1 : 0;
  o.saturated_low = r.saturated_low ? 1 : 0;
  return o;
}

} // extern "C"
