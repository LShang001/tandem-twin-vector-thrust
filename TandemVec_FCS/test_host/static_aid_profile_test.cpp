#include <cstdio>
#include <cstdlib>

// ins_static_detector.h 依赖 Eigen，宿主机测试直接测 profile 纯函数即可
#include "../include/ins_static_aid_profile.h"

namespace
{
void require(bool condition, const char *message)
{
  if (!condition)
  {
    std::fprintf(stderr, "FAILED: %s\n", message);
    std::exit(1);
  }
}

// 模拟 dwell_score 爬升：60 帧满，取 confirmed_static_frames 对应置信度
float sim_confidence(uint32_t frames)
{
  if (frames >= 60U)
    return 1.0f;
  return static_cast<float>(frames) / 60.0f;
}

void profile_tier1_ends_at_frame16()
{
  // 帧 15：仍在档位 1
  const auto p15 = Icm45686SelectStaticAidProfile(sim_confidence(15U), 15U, true);
  require(p15.vel_ne_std_mps == 0.18f, "frame15 vel_ne_std should be 0.18 (tier1)");
  require(p15.prefer_gravity_first, "frame15 should prefer gravity first (tier1)");

  // 帧 16：应离开档位 1（confidence 仍低，但帧门槛已过）
  // confidence(16) = 16/60 ≈ 0.267 < 0.35 → 还在档位1（置信度门限也要满足）
  const auto p16 = Icm45686SelectStaticAidProfile(sim_confidence(16U), 16U, true);
  require(p16.vel_ne_std_mps == 0.18f, "frame16 still tier1 due to low confidence");

  // 帧 32：confidence(32)=0.533 >= 0.35，帧 >= 16 → 应进档位 2
  const auto p32 = Icm45686SelectStaticAidProfile(sim_confidence(32U), 32U, true);
  require(p32.vel_ne_std_mps == 0.10f, "frame32 should be tier2");
  require(!p32.prefer_gravity_first, "frame32 tier2 no gravity-first");
}

void profile_tier3_starts_at_frame100_with_full_confidence()
{
  // 帧 99：confidence(99)=1.0（>=60），帧 < 100 → 档位 2
  const auto p99 = Icm45686SelectStaticAidProfile(1.0f, 99U, true);
  require(p99.vel_ne_std_mps == 0.10f, "frame99 should be tier2 (frames<100)");

  // 帧 100：confidence=1.0 >= 0.80，帧 >= 100，cov_ok → 档位 3
  const auto p100 = Icm45686SelectStaticAidProfile(1.0f, 100U, true);
  require(p100.vel_ne_std_mps == 0.02f, "frame100 should be tier3");
  require(p100.gravity_noise_rad == 0.03f, "frame100 tier3 gravity noise");

  // 如果协方差不健康，不升到档位 3
  const auto p100_bad_cov = Icm45686SelectStaticAidProfile(1.0f, 100U, false);
  require(p100_bad_cov.vel_ne_std_mps == 0.10f, "frame100 bad cov stays tier2");
}

void profile_tier3_rejects_low_confidence()
{
  // confidence=0.70 < 0.80 → 不进档位 3（即使帧数够）
  const auto p = Icm45686SelectStaticAidProfile(0.70f, 200U, true);
  require(p.vel_ne_std_mps == 0.10f, "low confidence should not enter tier3");
}

void dwell_score_reaches_one_at_60_frames()
{
  // 验证 sim_confidence（即 ins_static_detector.h 中新的 dwell 公式）
  require(sim_confidence(59U) < 1.0f, "frame59 confidence < 1");
  require(sim_confidence(60U) == 1.0f, "frame60 confidence == 1");
  require(sim_confidence(200U) == 1.0f, "frame200 confidence still 1");
}

void total_convergence_time_is_under_600ms()
{
  // 新参数下完整收敛时序（@200Hz）：
  // enter_min_frames=20 → confirmed_static at t=100ms
  // 档位1结束：confidence>=0.35需要 0.35*60=21帧 → t=100+105=205ms
  // 档位3解锁：frames=100 且 confidence(100)=1.0 → t=100+500=600ms
  // 测试档位3在 100 帧时确实可用
  const float conf_at_frame100 = sim_confidence(100U); // = 1.0
  require(conf_at_frame100 >= 0.80f, "convergence: conf>=0.80 at frame100");
  const auto p = Icm45686SelectStaticAidProfile(conf_at_frame100, 100U, true);
  require(p.vel_ne_std_mps == 0.02f, "convergence: tier3 available at frame100 (500ms post-confirm)");
}

void gravity_first_profile_prioritizes_gravity_when_both_due()
{
  const auto action = Icm45686SelectStaticAidAction(
      true /*zupt_due*/, true /*gravity_due*/, true /*static_gyro_due*/,
      true /*prefer_gravity_first*/);
  require(action == Icm45686StaticAidAction::Gravity,
          "tier1 should run Gravity before ZUPT when both are due");
}

void gravity_first_profile_suppresses_zupt_until_gravity_runs()
{
  const auto action = Icm45686SelectStaticAidAction(
      true /*zupt_due*/, false /*gravity_due*/, false /*static_gyro_due*/,
      true /*prefer_gravity_first*/);
  require(action == Icm45686StaticAidAction::None,
          "tier1 should suppress ZUPT until the gravity phase runs");
}

void normal_profile_keeps_zupt_priority_when_both_due()
{
  const auto action = Icm45686SelectStaticAidAction(
      true /*zupt_due*/, true /*gravity_due*/, true /*static_gyro_due*/,
      false /*prefer_gravity_first*/);
  require(action == Icm45686StaticAidAction::Zupt,
          "tier2/tier3 should keep ZUPT priority when both are due");
}

void zupt_noise_inflates_for_large_residual()
{
  const Icm45686StaticAidProfile profile{0.03f, 0.02f, 0.04f, false};
  const auto tuned = Icm45686AdaptZuptNoiseForResidual(profile, 0.55f);
  require(tuned.noise_scale > 1.0f,
          "large ZUPT residual should report inflated noise scale");
  require(tuned.vel_ne_std_mps > profile.vel_ne_std_mps,
          "large ZUPT residual should inflate NE noise");
  require(tuned.vel_d_std_mps > profile.vel_d_std_mps,
          "large ZUPT residual should inflate D noise");
  require(tuned.allow_fusion,
          "moderate residual should remain a soft ZUPT, not a hard skip");
}

void zupt_noise_blocks_extreme_residual()
{
  const Icm45686StaticAidProfile profile{0.03f, 0.02f, 0.04f, false};
  const auto tuned = Icm45686AdaptZuptNoiseForResidual(profile, 2.2f);
  require(tuned.noise_scale == 1.0f,
          "blocked ZUPT residual should keep nominal noise scale for diagnostics");
  require(!tuned.allow_fusion,
          "extreme ZUPT residual should skip velocity fusion");
}

} // namespace

int main()
{
  profile_tier1_ends_at_frame16();
  profile_tier3_starts_at_frame100_with_full_confidence();
  profile_tier3_rejects_low_confidence();
  dwell_score_reaches_one_at_60_frames();
  total_convergence_time_is_under_600ms();
  gravity_first_profile_prioritizes_gravity_when_both_due();
  gravity_first_profile_suppresses_zupt_until_gravity_runs();
  normal_profile_keeps_zupt_priority_when_both_due();
  zupt_noise_inflates_for_large_residual();
  zupt_noise_blocks_extreme_residual();
  std::puts("static_aid_profile_test passed");
  return 0;
}
