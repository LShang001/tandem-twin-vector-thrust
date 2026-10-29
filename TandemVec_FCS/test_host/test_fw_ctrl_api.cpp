/**
 * @file test_fw_ctrl_api.cpp
 * @module fw-tests/控制分配 ABI 一致性
 * @brief 固件控制分配 C ABI 与直接调用头文件实现的逐位一致性回归。
 *
 * @details 覆盖：默认机型参数透传、悬停状态初始化、纵列双发分配器在
 *          α/油门/陀螺/前馈掩码网格上的输出（含 next 状态递推）、垂直推力分配
 *          的合法/非法输入。ABI 只是薄封装，任何字段差异都说明封装漂移。
 * @frame  α/ω 存档系；推力分配 NED 天向。
 * @units  SI。
 * @test   run_all.sh 的 fwctrl 项。
 * @see    fw_algo/fw_algo_api.h、include/TandemVecAllocator.h、include/ThrustAllocation.h
 */
#include <cmath>
#include <cstdio>
#include <cstring>

#include "TandemVecAllocator.h"
#include "TandemVec_Config.h"
#include "ThrustAllocation.h"
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

bool SameAlloc(const TandemVecAllocatorOutput &a, const fw_tv_alloc_output &b)
{
  bool ok = a.valid == (b.valid != 0) && Eq(a.delta_f_rad, b.delta_f_rad) &&
            Eq(a.delta_t_rad, b.delta_t_rad) && Eq(a.motor1_pct, b.motor1_pct) &&
            Eq(a.motor2_pct, b.motor2_pct) && Eq(a.dw, b.dw) &&
            Eq(a.w0_eff, b.w0_eff) && Eq(a.yaw_gain_sched, b.yaw_gain_sched) &&
            a.sat_delta_f == (b.sat_delta_f != 0) &&
            a.sat_delta_t == (b.sat_delta_t != 0) && a.sat_dw == (b.sat_dw != 0);
  for (int i = 0; i < 3; ++i)
  {
    ok = ok && Eq(a.M_cmd[i], b.M_cmd[i]) && Eq(a.M_ff[i], b.M_ff[i]);
  }
  ok = ok && Eq(a.next.prev.wf, b.next.prev.wf) &&
       Eq(a.next.prev.wt, b.next.prev.wt) &&
       Eq(a.next.prev.delta_f, b.next.prev.delta_f) &&
       Eq(a.next.prev.delta_t, b.next.prev.delta_t) &&
       Eq(a.next.wf_est, b.next.wf_est) && Eq(a.next.wt_est, b.next.wt_est);
  return ok;
}

bool SameThrust(const bfs::ThrustAllocationOutput &a, const fw_thrust_alloc_output &b)
{
  return a.valid == (b.valid != 0) && Eq(a.total_thrust_n, b.total_thrust_n) &&
         Eq(a.throttle_pct, b.throttle_pct) &&
         a.saturated_high == (b.saturated_high != 0) &&
         a.saturated_low == (b.saturated_low != 0);
}
} // namespace

int main()
{
  Check(fw_algo_abi_version() == FW_ALGO_ABI_VERSION, "ABI 版本匹配");

  // ---- 默认参数 ----
  fw_tv_params fp;
  std::memset(&fp, 0, sizeof(fp));
  fw_ctrl_default_params(&fp);
  const TandemVecParams &dp = kDefaultTandemVecParams;
  bool params_ok = Eq(fp.kT, dp.kT) && Eq(fp.kQ, dp.kQ) && Eq(fp.Jp, dp.Jp) &&
                   Eq(fp.wMax, dp.wMax) && Eq(fp.a, dp.a) && Eq(fp.b, dp.b) &&
                   Eq(fp.tauM, dp.tauM) && Eq(fp.Ix, dp.Ix) && Eq(fp.Iy, dp.Iy) &&
                   Eq(fp.Iz, dp.Iz) && Eq(fp.dMax, dp.dMax) &&
                   Eq(fp.dwMax, dp.dwMax) && Eq(fp.m, dp.m) && Eq(fp.g, dp.g);
  Check(params_ok, "默认机型参数逐字段一致");

  // ---- 悬停状态 ----
  fw_tv_alloc_state fstate;
  std::memset(&fstate, 0, sizeof(fstate));
  fw_ctrl_state_hover(&fp, &fstate);
  const float w_hover = std::sqrt(0.5f * dp.m * dp.g / dp.kT);
  Check(Eq(fstate.prev.wf, w_hover) && Eq(fstate.prev.wt, w_hover),
        "悬停状态初值 = w_hover");

  // ---- 分配器网格（含状态递推）----
  TandemVecAllocatorState ref_state;
  ref_state.prev.wf = w_hover;
  ref_state.prev.wt = w_hover;
  ref_state.prev.delta_f = 0.0f;
  ref_state.prev.delta_t = 0.0f;
  ref_state.wf_est = w_hover;
  ref_state.wt_est = w_hover;

  int cases = 0;
  bool alloc_ok = true;
  for (int ia = -3; ia <= 3; ++ia)
  {
    for (int ib = -3; ib <= 3; ++ib)
    {
      for (int ic = -3; ic <= 3; ++ic)
      {
        for (int it = 0; it <= 100; it += 10)
        {
          for (int ig = -1; ig <= 1; ++ig)
          {
            TandemVecAllocatorInput ri;
            ri.alpha_roll_radps2 = 0.37f * static_cast<float>(ia);
            ri.alpha_pitch_radps2 = -0.41f * static_cast<float>(ib);
            ri.alpha_yaw_radps2 = 0.29f * static_cast<float>(ic);
            ri.throttle_pct = static_cast<float>(it);
            ri.gyro_body_radps[0] = 0.21f * static_cast<float>(ig);
            ri.gyro_body_radps[1] = -0.13f * static_cast<float>(ig);
            ri.gyro_body_radps[2] = 0.07f * static_cast<float>(ig);
            ri.dt_s = 0.005f;
            ri.inertia_comp_mask = static_cast<uint8_t>((it / 30) % 4);

            fw_tv_alloc_input fi;
            std::memset(&fi, 0, sizeof(fi));
            fi.alpha_roll_radps2 = ri.alpha_roll_radps2;
            fi.alpha_pitch_radps2 = ri.alpha_pitch_radps2;
            fi.alpha_yaw_radps2 = ri.alpha_yaw_radps2;
            fi.throttle_pct = ri.throttle_pct;
            for (int k = 0; k < 3; ++k)
            {
              fi.gyro_body_radps[k] = ri.gyro_body_radps[k];
            }
            fi.dt_s = ri.dt_s;
            fi.inertia_comp_mask = ri.inertia_comp_mask;

            const TandemVecAllocatorOutput rr =
                allocateTandemVec(ri, ref_state, dp);
            fw_tv_alloc_state fs_in = fstate;
            const fw_tv_alloc_output ra = fw_tandemvec_alloc(&fp, &fs_in, &fi);

            alloc_ok = alloc_ok && SameAlloc(rr, ra);
            ref_state = rr.next;
            fstate = ra.next;
            ++cases;
          }
        }
      }
    }
  }
  Check(alloc_ok, "分配器 ABI 与直接调用逐位一致（含状态递推）");
  Check(cases == 7 * 7 * 7 * 11 * 3, "分配器网格覆盖数正确");
  Check(cases > 11000, "分配器样本量 > 11000");

  // ---- 非法输入 ----
  {
    fw_tv_alloc_input bad;
    std::memset(&bad, 0, sizeof(bad));
    bad.alpha_roll_radps2 = std::nanf("");
    bad.throttle_pct = 50.0f;
    bad.dt_s = 0.005f;
    const fw_tv_alloc_output r = fw_tandemvec_alloc(&fp, &fstate, &bad);
    Check(r.valid == 0, "非有限 α ⇒ valid=0");
    Check(r.delta_f_rad == 0.0f && r.motor1_pct == 0.0f, "非法输入输出回零");
  }

  // ---- 推力分配 ----
  bool thrust_ok = true;
  int thrust_cases = 0;
  for (int i = 0; i <= 20; ++i)
  {
    for (int j = 0; j <= 10; ++j)
    {
      bfs::ThrustAllocationInput ri;
      ri.specific_force_up_mps2 = 2.0f + 1.5f * static_cast<float>(i);
      ri.thrust_axis_up_cos = 0.2f + 0.08f * static_cast<float>(j);
      ri.mass_kg = 0.7f;
      ri.max_total_thrust_n = 13.73f;
      ri.min_cos_tilt = 0.5f;

      fw_thrust_alloc_input fi;
      fi.specific_force_up_mps2 = ri.specific_force_up_mps2;
      fi.thrust_axis_up_cos = ri.thrust_axis_up_cos;
      fi.mass_kg = ri.mass_kg;
      fi.max_total_thrust_n = ri.max_total_thrust_n;
      fi.min_cos_tilt = ri.min_cos_tilt;

      thrust_ok = thrust_ok && SameThrust(bfs::allocateLinearThrustThrottle(ri),
                                          fw_thrust_alloc(&fi));
      ++thrust_cases;
    }
  }
  Check(thrust_ok, "推力分配 ABI 与直接调用逐位一致");
  Check(thrust_cases == 231, "推力分配网格覆盖数正确");

  {
    fw_thrust_alloc_input bad;
    bad.specific_force_up_mps2 = std::nanf("");
    bad.thrust_axis_up_cos = 1.0f;
    bad.mass_kg = 0.7f;
    bad.max_total_thrust_n = 13.73f;
    bad.min_cos_tilt = 0.5f;
    const fw_thrust_alloc_output r = fw_thrust_alloc(&bad);
    Check(r.valid == 0 && r.throttle_pct == 0.0f, "推力分配非法输入回零");
  }

  std::printf("fw_ctrl ABI 一致性：%d 项检查，%d 失败（分配 %d 例）\n",
              g_checked, g_fail, cases);
  return g_fail == 0 ? 0 : 1;
}
