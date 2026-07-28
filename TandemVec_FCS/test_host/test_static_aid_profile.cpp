// 宿主机回归测试：静止辅助强度调度纯函数
// 编译运行：g++ -std=c++17 -Iinclude test_host/test_static_aid_profile.cpp -o test_host/bin/sap && ./test_host/bin/sap
//
// 本文件不依赖 Arduino / STM32 HAL / Eigen，仅验证 include/ins_static_aid_profile.h 的 constexpr 逻辑。
#include "../include/ins_static_aid_profile.h"

#include <cmath>
#include <cstdio>
#include <string>

static int g_fail_count = 0;
static void check(bool cond, const std::string &name)
{
  std::printf(cond ? "[PASS] %s\n" : "[FAIL] %s\n", name.c_str());
  if (!cond) ++g_fail_count;
}
static bool approx(float a, float b, float rel = 1e-5f)
{
  return std::fabs(a - b) <= rel * std::fmax(std::fabs(a), std::fabs(b));
}

int main()
{
  // ---- Icm45686NormalizeStaticAidDivider ----
  check(Icm45686NormalizeStaticAidDivider(0U) == 1U, "divider=0 归一化为 1 (防除零)");
  check(Icm45686NormalizeStaticAidDivider(7U) == 7U, "divider=7 保持 7");

  // ---- Icm45686StaticAidGcd ----
  check(Icm45686StaticAidGcd(12U, 8U) == 4U, "GCD(12,8)=4");
  check(Icm45686StaticAidGcd(0U, 5U) == 5U, "GCD(0,5)=5");
  check(Icm45686StaticAidGcd(5U, 0U) == 5U, "GCD(5,0)=5");
  check(Icm45686StaticAidGcd(7U, 7U) == 7U, "GCD(7,7)=7");
  check(Icm45686StaticAidGcd(0U, 0U) == 1U, "GCD(0,0)=1 (双零保护)");

  // ---- Icm45686StaticAidPhasesCanCollide ----
  // zupt 周期 10、gravity 周期 6，GCD=2；同相位 0 必然碰撞
  check(Icm45686StaticAidPhasesCanCollide(10U, 0U, 6U, 0U), "同相位 (10,0)/(6,0) 必碰撞");
  // 相位差 1 不能被 GCD(2) 整除 → 不碰撞
  check(!Icm45686StaticAidPhasesCanCollide(10U, 1U, 6U, 0U), "相位差 1 mod GCD2 !=0 不碰撞");
  // 相位差 2 被 GCD(2) 整除 → 碰撞
  check(Icm45686StaticAidPhasesCanCollide(10U, 2U, 6U, 0U), "相位差 2 mod GCD2 ==0 碰撞");
  // divider=0 归一化为 1，周期 1 每帧到期，必然碰撞
  check(Icm45686StaticAidPhasesCanCollide(0U, 0U, 0U, 0U), "双零周期归一化为 1 必碰撞");

  // ---- Icm45686Clamp01 ----
  check(approx(Icm45686Clamp01(-0.5f), 0.0f), "Clamp01(-0.5)=0");
  check(approx(Icm45686Clamp01(0.5f), 0.5f), "Clamp01(0.5)=0.5");
  check(approx(Icm45686Clamp01(1.5f), 1.0f), "Clamp01(1.5)=1");
  check(approx(Icm45686Clamp01(0.0f), 0.0f), "Clamp01(0)=0");
  check(approx(Icm45686Clamp01(1.0f), 1.0f), "Clamp01(1)=1");

  // ---- Icm45686SelectStaticAidAction ----
  using A = Icm45686StaticAidAction;
  // prefer_gravity_first 模式：只看 gravity，忽略其余
  check(Icm45686SelectStaticAidAction(true, true, true, true) == A::Gravity,
        "prefer_gravity_first 且 gravity_due → Gravity");
  check(Icm45686SelectStaticAidAction(true, false, true, true) == A::None,
        "prefer_gravity_first 且 gravity 未到期 → None (即使 zupt 到期)");
  // 普通模式优先级：Zupt > Gravity > StaticGyro > None
  check(Icm45686SelectStaticAidAction(true, true, true, false) == A::Zupt,
        "zupt 优先于 gravity/static_gyro");
  check(Icm45686SelectStaticAidAction(false, true, true, false) == A::Gravity,
        "无 zupt 时 gravity 优先于 static_gyro");
  check(Icm45686SelectStaticAidAction(false, false, true, false) == A::StaticGyro,
        "仅 static_gyro 到期 → StaticGyro");
  check(Icm45686SelectStaticAidAction(false, false, false, false) == A::None,
        "全部未到期 → None");

  // ---- Icm45686AdaptZuptNoiseForResidual ----
  Icm45686StaticAidProfile base{0.10f, 0.10f, 0.14f, false};
  {
    auto z = Icm45686AdaptZuptNoiseForResidual(base, 0.10f);
    check(z.allow_fusion && approx(z.noise_scale, 1.0f), "残差<=0.20 scale=1.0 融合");
    check(approx(z.vel_ne_std_mps, base.vel_ne_std_mps), "scale=1 时 vel_ne 不放大");
  }
  {
    auto z = Icm45686AdaptZuptNoiseForResidual(base, 0.20f);
    check(approx(z.noise_scale, 1.0f), "残差=0.20 边界 scale=1.0");
  }
  {
    auto z = Icm45686AdaptZuptNoiseForResidual(base, 0.40f);
    // scale = 1 + (0.40-0.20)*5 = 2.0
    check(z.allow_fusion && approx(z.noise_scale, 2.0f), "残差=0.40 scale=2.0");
    check(approx(z.vel_ne_std_mps, base.vel_ne_std_mps * 2.0f), "vel_ne 按 scale 放大");
  }
  {
    auto z = Icm45686AdaptZuptNoiseForResidual(base, 0.60f);
    // scale = 1 + (0.60-0.20)*5 = 3.0 (边界 <=0.60)
    check(approx(z.noise_scale, 3.0f), "残差=0.60 边界 scale=3.0");
  }
  {
    auto z = Icm45686AdaptZuptNoiseForResidual(base, 1.00f);
    // scale = 3 + (1.00-0.60)*5 = 5.0
    check(z.allow_fusion && approx(z.noise_scale, 5.0f), "残差=1.00 scale=5.0");
  }
  {
    auto z = Icm45686AdaptZuptNoiseForResidual(base, 1.50f);
    // 1.50 不 > 1.5，走计算：scale=3+(1.5-0.6)*5=7.5 → cap 6.0
    check(z.allow_fusion && approx(z.noise_scale, 6.0f), "残差=1.50 scale 被 cap 到 6.0");
  }
  {
    auto z = Icm45686AdaptZuptNoiseForResidual(base, 1.60f);
    // >1.5 不融合
    check(!z.allow_fusion, "残差>1.5 不融合");
    check(approx(z.noise_scale, 1.0f), "不融合时 scale=1.0");
  }
  {
    auto z = Icm45686AdaptZuptNoiseForResidual(base, -1.0f);
    // 负残差归零，<=0.20 分支 scale=1.0 融合
    check(z.allow_fusion && approx(z.noise_scale, 1.0f), "负残差归零后 scale=1.0 融合");
  }

  // ---- Icm45686SelectStaticAidProfile 三档调度 ----
  {
    auto p = Icm45686SelectStaticAidProfile(0.10f, 100U, true);
    check(approx(p.gravity_noise_rad, 0.18f) && p.prefer_gravity_first,
          "档1: confidence<0.35");
  }
  {
    auto p = Icm45686SelectStaticAidProfile(0.50f, 10U, true);
    check(approx(p.gravity_noise_rad, 0.18f), "档1: frames<16");
  }
  {
    auto p = Icm45686SelectStaticAidProfile(0.50f, 50U, true);
    check(approx(p.gravity_noise_rad, 0.10f) && !p.prefer_gravity_first,
          "档2: 中置信度中帧数");
  }
  {
    auto p = Icm45686SelectStaticAidProfile(0.90f, 50U, true);
    check(approx(p.gravity_noise_rad, 0.10f), "档2: frames<100");
  }
  {
    auto p = Icm45686SelectStaticAidProfile(0.90f, 200U, false);
    check(approx(p.gravity_noise_rad, 0.10f), "档2: 协方差不健康降级");
  }
  {
    auto p = Icm45686SelectStaticAidProfile(0.90f, 200U, true);
    check(approx(p.gravity_noise_rad, 0.03f) && approx(p.vel_ne_std_mps, 0.02f),
          "档3: 高置信度长静止稳态");
  }
  {
    // 超界置信度应被 clamp 到 1.0 后进入档3
    auto p = Icm45686SelectStaticAidProfile(1.5f, 200U, true);
    check(approx(p.gravity_noise_rad, 0.03f), "置信度>1 被 clamp 后进档3");
  }
  {
    auto p = Icm45686SelectStaticAidProfile(-0.5f, 200U, true);
    check(approx(p.gravity_noise_rad, 0.18f), "置信度<0 被 clamp 后进档1");
  }

  if (g_fail_count == 0)
  {
    std::printf("\n=== 静止辅助调度测试全部通过 ===\n");
    return 0;
  }
  std::printf("\n=== 失败 %d 项 ===\n", g_fail_count);
  return 1;
}
