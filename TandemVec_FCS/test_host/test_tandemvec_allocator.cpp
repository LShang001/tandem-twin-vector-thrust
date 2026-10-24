/**
 * @file    test_tandemvec_allocator.cpp
 * @module  固件/宿主回归测试（fw-tests：纵列双发机型分配器）
 * @brief   验证 α_cmd/油门 → 摆角/电机 的机型逆解边界、零油门归中、
 *          饱和、τm 状态推进与非法输入回零
 * @details 被测：`include/TandemVecAllocator.h`。
 *          输入：默认 TandemVec 参数、合成 α/油门/陀螺/上一拍状态；
 *          处理：allocateTandemVec；
 *          输出：PASS/FAIL 计数，main 返回非零即失败。
 *          编译：g++ -std=c++17 -Iinclude test_host/test_tandemvec_allocator.cpp
 *                -o bin/tva
 * @theory  M = I·α + M_ff；BTRUE；ωf,t = w0√(1±Δω)
 *          @ref THY-003 §3；CTL-001 §8.6；CTL-003 §2
 * @frame   α/gyro 存档系；M 模型系
 * @units   rad/s²、N·m、rad、%、s
 * @note    本测试锁“接口/安全边界”，不复制旧 mix 的整段公式；旧公式等价由
 *          fw_sil 真实固件 30/30 与 test_vehicle_profile 共同覆盖。
 * @test    T1 正常；T2 零油门归中；T3 大指令饱和；T4 NaN 回零；
 *          T5 τm 状态推进；T6 前馈掩码关闭
 * @see     include/TandemVecAllocator.h、src/flight_control.cpp
 */
#include <cmath>
#include <cstdio>

#include "TandemVecAllocator.h"

static int g_fail = 0;
#define CHECK(cond, msg)                                                        \
  do                                                                            \
  {                                                                             \
    if (!(cond))                                                                \
    {                                                                           \
      std::printf("FAIL: %s (line %d)\n", msg, __LINE__);                       \
      g_fail++;                                                                 \
    }                                                                           \
  } while (0)

static TandemVecAllocatorState makeState()
{
  TandemVecAllocatorState s;
  s.prev = PropulsionState{800.0f, 800.0f, 0.0f, 0.0f};
  s.wf_est = 800.0f;
  s.wt_est = 800.0f;
  return s;
}

int main()
{
  const TandemVecParams &p = kDefaultTandemVecParams;

  // T1 正常：有限输出、在限幅内、状态被推进。
  {
    TandemVecAllocatorInput in;
    in.alpha_roll_radps2 = 0.1f;
    in.alpha_pitch_radps2 = -0.2f;
    in.alpha_yaw_radps2 = 0.05f;
    in.throttle_pct = 50.0f;
    in.gyro_body_radps[0] = 0.01f;
    in.gyro_body_radps[1] = -0.02f;
    in.gyro_body_radps[2] = 0.03f;
    const auto s = makeState();
    const auto out = allocateTandemVec(in, s, p);
    CHECK(out.valid, "T1 输出有效");
    CHECK(std::fabs(out.delta_f_rad) <= p.dMax + 1e-5f &&
              std::fabs(out.delta_t_rad) <= p.dMax + 1e-5f,
          "T1 摆角在 dMax 内");
    CHECK(out.motor1_pct >= 0.0f && out.motor1_pct <= 100.0f &&
              out.motor2_pct >= 0.0f && out.motor2_pct <= 100.0f,
          "T1 电机输出在 0–100%");
    CHECK(std::isfinite(out.M_cmd[0]) && std::isfinite(out.M_cmd[1]) &&
              std::isfinite(out.M_cmd[2]),
          "T1 M_cmd 有限");
    CHECK(out.next.prev.wf > 0.0f && out.next.prev.wt > 0.0f,
          "T1 τm 状态推进");
  }

  // T2 零油门：摆角/差速/电机全部归中。
  {
    TandemVecAllocatorInput in;
    in.alpha_roll_radps2 = 0.3f;
    in.alpha_pitch_radps2 = 0.3f;
    in.throttle_pct = 0.0f;
    const auto out = allocateTandemVec(in, makeState(), p);
    CHECK(out.valid, "T2 零油门有效");
    CHECK(std::fabs(out.delta_f_rad) < 1e-6f &&
              std::fabs(out.delta_t_rad) < 1e-6f &&
              std::fabs(out.dw) < 1e-6f,
          "T2 零油门执行器归中");
    CHECK(out.motor1_pct < 1e-4f && out.motor2_pct < 1e-4f,
          "T2 零油门电机 0%");
  }

  // T3 大指令：输出仍限幅并报告饱和。
  {
    TandemVecAllocatorInput in;
    in.alpha_roll_radps2 = 100.0f;
    in.alpha_pitch_radps2 = 100.0f;
    in.alpha_yaw_radps2 = 100.0f;
    in.throttle_pct = 50.0f;
    const auto out = allocateTandemVec(in, makeState(), p);
    CHECK(out.valid, "T3 大指令有效");
    CHECK(std::fabs(out.delta_f_rad) <= p.dMax + 1e-5f &&
              std::fabs(out.delta_t_rad) <= p.dMax + 1e-5f &&
              std::fabs(out.dw) <= p.dwMax + 1e-5f,
          "T3 输出被限幅");
    CHECK(out.sat_delta_f || out.sat_delta_t || out.sat_dw,
          "T3 至少一路饱和标记");
  }

  // T4 非有限 α：回零且 valid=false。
  {
    TandemVecAllocatorInput in;
    in.alpha_roll_radps2 = std::nanf("");
    in.throttle_pct = 50.0f;
    const auto out = allocateTandemVec(in, makeState(), p);
    CHECK(!out.valid, "T4 NaN 输入无效");
    CHECK(std::fabs(out.delta_f_rad) < 1e-6f &&
              std::fabs(out.motor1_pct) < 1e-6f,
          "T4 NaN 输入执行器回零");
  }

  // T5 τm 观测器：上一拍转速高于估计时，估计向上一拍靠近。
  {
    TandemVecAllocatorInput in;
    in.throttle_pct = 50.0f;
    auto state = makeState();
    state.wf_est = 400.0f;
    state.wt_est = 400.0f;
    state.prev.wf = 900.0f;
    state.prev.wt = 900.0f;
    const auto out = allocateTandemVec(in, state, p);
    CHECK(out.next.wf_est > state.wf_est &&
              out.next.wf_est < state.prev.wf,
          "T5 wf_est 向上一拍靠近");
    CHECK(out.next.wt_est > state.wt_est &&
              out.next.wt_est < state.prev.wt,
          "T5 wt_est 向上一拍靠近");
  }

  // T6 前馈掩码关闭：M_ff 为零。
  {
    TandemVecAllocatorInput in;
    in.alpha_roll_radps2 = 0.1f;
    in.alpha_pitch_radps2 = 0.1f;
    in.alpha_yaw_radps2 = 0.1f;
    in.throttle_pct = 50.0f;
    in.inertia_comp_mask = 0;
    in.gyro_body_radps[0] = 1.0f;
    in.gyro_body_radps[1] = 2.0f;
    in.gyro_body_radps[2] = 3.0f;
    const auto out = allocateTandemVec(in, makeState(), p);
    CHECK(std::fabs(out.M_ff[0]) < 1e-6f &&
              std::fabs(out.M_ff[1]) < 1e-6f &&
              std::fabs(out.M_ff[2]) < 1e-6f,
          "T6 掩码关闭 M_ff=0");
  }

  if (g_fail == 0)
  {
    std::printf("=== test_tandemvec_allocator: 全部通过 ===\n");
    return 0;
  }
  std::printf("=== test_tandemvec_allocator: %d 项失败 ===\n", g_fail);
  return 1;
}
