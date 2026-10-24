/**
 * @file    test_heading_aiding_gate.cpp
 * @module  固件/宿主回归测试（fw-tests：绝对航向辅助与位置保持降级）
 * @brief   验证磁/GNSS 双矢量航向新鲜度、失效立即降级、恢复 3 s 锁存、
 *          YawGsf 速度门限与非有限输入
 * @details 被测：`include/HeadingAidingGate.h`。
 *          输入：合成 Mag/YawGsf 年龄、水平速度与 dt；
 *          处理：HeadingAidingGate::update；
 *          输出：PASS/FAIL 计数，main 返回非零即失败。
 *          编译：g++ -std=c++17 -Iinclude test_host/test_heading_aiding_gate.cpp
 *                -o bin/hag
 * @theory  悬停航向不可观 → 无绝对航向源时禁止水平位置保持
 *          @ref REF-120 §9.8.14 p.320–322；AGENTS.md EKF-in-loop 定位
 * @frame   N/A
 * @units   s、m/s
 * @note    本测试只锁门控策略，不评估 EKF 航向精度；真实稳定性由 SIL 验收。
 * @test    T1 初始降级；T2 磁源连续 3 s 后恢复；T3 失效立即降级；
 *          T4 恢复未满 3 s 仍降级；T5 YawGsf 只诊断不解除；T6 非有限输入
 * @see     include/HeadingAidingGate.h、src/navigation_task.cpp、src/flight_control.cpp
 */
#include <cmath>
#include <cstdio>

#include "HeadingAidingGate.h"

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

static bfs::HeadingAidingInput input(float mag_age, float gsf_age = 1.0e9f,
                                     float speed = 0.0f, float dt = 0.005f)
{
  bfs::HeadingAidingInput in;
  in.mag_age_s = mag_age;
  in.yaw_gsf_age_s = gsf_age;
  in.horizontal_speed_mps = speed;
  in.dt_s = dt;
  return in;
}

int main()
{
  // T1 初始无源：降级。
  {
    bfs::HeadingAidingGate gate;
    const auto out = gate.update(input(1.0e9f));
    CHECK(out.position_hold_degraded, "T1 初始降级");
    CHECK(!out.source_fresh, "T1 无源");
    CHECK(out.valid_for_s == 0.0f, "T1 恢复计时为 0");
  }

  // T2 磁源连续 3 s：恢复；之前一直降级。
  {
    bfs::HeadingAidingGate gate;
    bool recovered_early = false;
    for (int i = 0; i < 599; ++i)
    {
      const auto out = gate.update(input(0.02f));
      if (!out.position_hold_degraded && i < 599)
      {
        recovered_early = true;
      }
    }
    CHECK(!recovered_early, "T2 未满 3 s 不恢复");
    const auto out = gate.update(input(0.02f));
    CHECK(!out.position_hold_degraded, "T2 满 3 s 恢复");
    CHECK(out.source_fresh, "T2 源新鲜");
  }

  // T3 失效（年龄 >2 s）立即降级，即使此前已恢复。
  {
    bfs::HeadingAidingGate gate;
    for (int i = 0; i < 700; ++i) (void)gate.update(input(0.02f));
    const auto out = gate.update(input(2.01f));
    CHECK(out.position_hold_degraded, "T3 失效立即降级");
    CHECK(!out.source_fresh, "T3 失效源不新鲜");
  }

  // T4 恢复未满 3 s 仍降级，满 3 s 才恢复。
  {
    bfs::HeadingAidingGate gate;
    for (int i = 0; i < 700; ++i) (void)gate.update(input(0.02f));
    (void)gate.update(input(5.0f));
    for (int i = 0; i < 599; ++i)
    {
      const auto out = gate.update(input(0.02f));
      CHECK(out.position_hold_degraded, "T4 恢复计时未满仍降级");
    }
    const auto out = gate.update(input(0.02f));
    CHECK(!out.position_hold_degraded, "T4 满 3 s 恢复");
  }

  // T5 YawGsf 只诊断、不解除门控：即使高速新鲜也不能替代磁航向。
  {
    bfs::HeadingAidingGate gate;
    auto out = gate.update(input(1.0e9f, 0.02f, 1.9f));
    CHECK(out.position_hold_degraded, "T5 低速 YawGsf 不使能");
    for (int i = 0; i < 700; ++i)
    {
      out = gate.update(input(1.0e9f, 0.02f, 2.5f));
    }
    CHECK(out.yaw_gsf_fresh, "T5 高速 YawGsf 诊断新鲜");
    CHECK(out.position_hold_degraded, "T5 YawGsf 不解除降级");
  }

  // T6 非有限输入：源不新鲜、保持降级、不产生 NaN。
  {
    bfs::HeadingAidingGate gate;
    const auto out = gate.update(input(std::nanf(""), std::nanf(""),
                                       std::nanf(""), std::nanf("")));
    CHECK(out.position_hold_degraded, "T6 非有限输入降级");
    CHECK(!out.source_fresh, "T6 非有限输入无源");
    CHECK(std::isfinite(out.valid_for_s), "T6 valid_for 有限");
  }

  // T7 偏航振荡：磁新鲜时仍因持续高偏航角速度降级；安静 3 s 恢复。
  {
    bfs::HeadingAidingGate gate;
    bfs::HeadingAidingInput in = input(0.1f);
    for (int i = 0; i < 700; ++i) (void)gate.update(in); // 先恢复位置保持
    in.monitor_yaw_oscillation = true;
    in.yaw_rate_dps = 80.0f;
    bool osc_seen = false;
    for (int i = 0; i < 300; ++i)
    {
      const auto out = gate.update(in);
      osc_seen = osc_seen || out.yaw_oscillating;
    }
    CHECK(osc_seen, "T7 高偏航率触发降级");
    in.yaw_rate_dps = 2.0f;
    for (int i = 0; i < 700; ++i) (void)gate.update(in);
    const auto out = gate.update(in);
    CHECK(!out.yaw_oscillating, "T7 安静后解除振荡");
    CHECK(!out.position_hold_degraded, "T7 磁新鲜+安静恢复位置保持");
  }

  // T8 航向协方差：P_yaw 持续 >10° 降级；<6° 持续 3 s 恢复。
  {
    bfs::HeadingAidingGate gate;
    bfs::HeadingAidingInput in = input(0.1f);
    for (int i = 0; i < 700; ++i) (void)gate.update(in);
    in.monitor_yaw_sigma = true;
    in.yaw_sigma_deg = 20.0f;
    bool sigma_seen = false;
    for (int i = 0; i < 300; ++i)
    {
      const auto out = gate.update(in);
      sigma_seen = sigma_seen || out.yaw_sigma_high;
    }
    CHECK(sigma_seen, "T8 高航向协方差降级");
    in.yaw_sigma_deg = 2.0f;
    for (int i = 0; i < 700; ++i) (void)gate.update(in);
    const auto out = gate.update(in);
    CHECK(!out.yaw_sigma_high, "T8 协方差收敛后解除");
    CHECK(!out.position_hold_degraded, "T8 磁新鲜+协方差小恢复位置保持");
  }

  // T9 无磁/磁仅初始对准：短时（≤20 s）允许位置保持/heading hold；
  // 时间窗结束或航向 1σ 过大时降级。
  {
    bfs::HeadingAidingGate gate;
    bfs::HeadingAidingInput in = input(1.0e9f);
    in.heading_mode = 2;
    in.yaw_sigma_deg = 5.0f;
    in.yaw_drift_dps = 0.2f;
    in.motion_excitation_valid = true;
    in.airborne = true;
    for (int i = 0; i < 700; ++i) (void)gate.update(in);
    auto out = gate.update(in);
    CHECK(!out.position_hold_degraded, "T9 短时窗口内允许位置保持");
    CHECK(out.heading_hold_allowed, "T9 短时窗口内允许 heading hold");
    CHECK(out.source_fresh, "T9 短时窗口内航向源可用");
    for (int i = 0; i < 4000; ++i) (void)gate.update(in);
    out = gate.update(in);
    CHECK(out.position_hold_degraded, "T9 超过 20 s 降级");
    CHECK(!out.heading_hold_allowed, "T9 超过 20 s 禁 heading hold");
  }

  // T9b 无磁窗口只从离地起算：地面等待 60 s 不消耗保持时间。
  {
    bfs::HeadingAidingGate gate;
    bfs::HeadingAidingInput in = input(1.0e9f);
    in.heading_mode = 2;
    in.yaw_sigma_deg = 5.0f;
    in.airborne = false;
    for (int i = 0; i < 12000; ++i) (void)gate.update(in);
    in.airborne = true;
    for (int i = 0; i < 700; ++i) (void)gate.update(in);
    const auto out = gate.update(in);
    CHECK(!out.position_hold_degraded, "T9b 地面时间不计入 20 s 窗口");
    CHECK(out.heading_hold_allowed, "T9b 离地后仍允许 heading hold");
  }

  // T9d 静止校准前置：未完成校准不得进入无磁位置保持。
  {
    bfs::HeadingAidingGate gate;
    bfs::HeadingAidingInput in = input(1.0e9f);
    in.heading_mode = 2;
    in.yaw_sigma_deg = 5.0f;
    in.airborne = true;
    in.calibrated = false;
    for (int i = 0; i < 100; ++i) (void)gate.update(in);
    auto out = gate.update(in);
    CHECK(out.position_hold_degraded, "T9d 未静止校准降级");
    CHECK(!out.heading_hold_allowed, "T9d 未静止校准禁 heading hold");
    in.calibrated = true;
    out = gate.update(in);
    CHECK(!out.position_hold_degraded, "T9d 校准完成后允许位置保持");
    CHECK(out.heading_hold_allowed, "T9d 校准完成后允许 heading hold");
  }

  // T9c 落地复位窗口：离地超时降级后，落地再起飞重新获得 20 s。
  {
    bfs::HeadingAidingGate gate;
    bfs::HeadingAidingInput in = input(1.0e9f);
    in.heading_mode = 2;
    in.yaw_sigma_deg = 5.0f;
    in.airborne = true;
    for (int i = 0; i < 6000; ++i) (void)gate.update(in);
    auto out = gate.update(in);
    CHECK(out.position_hold_degraded, "T9c 离地 30 s 后降级");
    in.airborne = false;
    (void)gate.update(in);
    in.airborne = true;
    out = gate.update(in);
    CHECK(!out.position_hold_degraded, "T9c 落地复位后重新允许短时保持");
  }

  if (g_fail == 0)
  {
    std::printf("=== test_heading_aiding_gate: 全部通过 ===\n");
    return 0;
  }
  std::printf("=== test_heading_aiding_gate: %d 项失败 ===\n", g_fail);
  return 1;
}
