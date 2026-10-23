/**
 * @file    test_static_aid_profile.cpp
 * @module  固件/宿主回归测试（fw-tests：静止辅助强度调度纯函数）
 * @brief   验证 ZUPT/重力辅助的周期 GCD、相位碰撞、强度分级与帧边界精确值
 * @details 被测：`include/ins_static_aid_profile.h` 的 constexpr 纯函数：
 *          `Icm45686NormalizeStaticAidDivider()`、`Icm45686StaticAidGcd()`、
 *          `Icm45686StaticAidPhasesCanCollide()`、`Icm45686Clamp01()`、
 *          `Icm45686SelectStaticAidAction()`、`Icm45686AdaptZuptNoiseForResidual()`、
 *          `Icm45686SelectStaticAidProfile()` 与 ZUPT 外部参照门控。
 *          输入：周期/相位/残差/置信度等标量组合，含 0 与边界值；
 *          处理：逐条 check 断言返回值（含帧边界精确值）；
 *          输出：PASS/FAIL 计数，main 返回非零即失败。
 *          编译：g++ -std=c++17 -Iinclude test_host/test_static_aid_profile.cpp -o bin/sap
 * @theory  静止辅助按“每 N 帧做一次”分档；两个辅助若周期 GCD 与相位差同余，
 *          会同帧叠加，必须避免（否则同一帧执行两次 ZUPT/重力修正）。
 *          噪声自适应把残差量级映射成 ZUPT 观测噪声缩放（0.6–1.5 区间分段，
 *          cap 6.0），本质是把“残差大小”转成“信任程度”。
 *          @ref REF-121 §8.2.3 p.291（静止辅助/ZUPT）、@ref REF-121 §7.3.1 p.255（门控）
 * @frame   N/A（调度/标量逻辑；残差在 NED 速度域但本文件只传标量）
 * @units   周期/相位为帧数；残差 m/s；噪声缩放无量纲
 * @note    帧边界用例是从已删除的 `static_aid_profile_test.cpp` 并入的（2026-08-08），
 *          期望值全部是解析值（例如 1.50 不 > 1.5 走计算分支：
 *          scale=3+(1.5−0.6)×5=7.5 再 cap 到 6.0）。0 周期被归一化为 1 是防除零硬保护。
 * @test    S1 divider 归一化；S2 GCD 含双零；S3 相位碰撞判据；S4 Clamp01；
 *          S5 动作选择优先级；S6 ZUPT 噪声自适应分段；S7 三档 profile 调度；
 *          S8 帧边界精确值；S9 ZUPT 外部参照门控（P0-3）
 * @see     TandemVec_FCS/include/ins_static_aid_profile.h、
 *          TandemVec_FCS/include/ins_static_detector.h
 */
// 宿主机回归测试：静止辅助强度调度纯函数
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
  // prefer_gravity_first 模式：gravity 到期优先 Gravity；未到期回落 Zupt/StaticGyro，
  // 不再饿死其它辅助（2026-08-12 审查修复）。
  check(Icm45686SelectStaticAidAction(true, true, true, true) == A::Gravity,
        "prefer_gravity_first 且 gravity_due → Gravity");
  check(Icm45686SelectStaticAidAction(true, false, true, true) == A::Zupt,
        "prefer_gravity_first 且 gravity 未到期 → 回落 Zupt (不再饿死)");
  check(Icm45686SelectStaticAidAction(false, false, true, true) == A::StaticGyro,
        "prefer_gravity_first 且 gravity/zupt 未到期 → 回落 StaticGyro");
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
    check(approx(p.gravity_noise_rad, 0.18f), "档1: frames<36");
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
  // ---- 帧边界精确值（并入自 static_aid_profile_test.cpp，该文件已删除）----
  {
    // frame16：confidence=16/60≈0.267 < 0.35 → 置信度不足 → 仍档1
    auto p16 = Icm45686SelectStaticAidProfile(0.267f, 16U, true);
    check(approx(p16.gravity_noise_rad, 0.18f),
          "档1: frame16 低置信度不升级（置信度门槛未过）");
    // frame32：confidence=0.533 ≥ 0.35 但帧 32 < 36（2026-08-12 门槛 16→36）→ 仍档1
    auto p32 = Icm45686SelectStaticAidProfile(0.533f, 32U, true);
    check(approx(p32.gravity_noise_rad, 0.18f),
          "档1: frame32 帧门槛未过（36）仍档1");
    // frame36：confidence=0.533 ≥ 0.35 且帧 ≥36 → 档2（精确边界）
    auto p36 = Icm45686SelectStaticAidProfile(0.533f, 36U, true);
    check(approx(p36.gravity_noise_rad, 0.10f),
          "档2: frame36 置信度与帧门槛均满足");
    // frame99：帧 <100 → 即使满置信度仍档2
    auto p99 = Icm45686SelectStaticAidProfile(1.0f, 99U, true);
    check(approx(p99.gravity_noise_rad, 0.10f),
          "档2: frame99（帧<100）满置信度仍不升档3");
    // frame100：帧 ≥100 且满置信度 → 档3（精确边界）
    auto p100 = Icm45686SelectStaticAidProfile(1.0f, 100U, true);
    check(approx(p100.gravity_noise_rad, 0.03f) && approx(p100.vel_ne_std_mps, 0.02f),
          "档3: frame100 满置信度精确边界");
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

  // ---- ZUPT 外部参照门控（2026-10 P0-3）----
  // 理论依据：秦永元等《卡尔曼滤波与组合导航原理（第3版）》，西北工业大学
  //   出版社 2015，§8.2.3 p.291：间接法滤波的阻尼必须利用外来信息；
  //   §7.3.1 p.255：系统级 FDI 用 χ² 检验。
  {
    Icm45686ZuptExtRefGuard g;
    g = Icm45686UpdateZuptExtRefGuard(g, false, 2.9f);
    check(!g.cap_noise && !g.disable_zupt, "外部参照门控: <3s 不封顶");
    check(!Icm45686ZuptExternalRefAllowsFusion(g, 1.0f),
          "外部参照门控: 无外部参照立即拒绝 1.0m/s ZUPT");
    check(Icm45686ZuptExternalRefAllowsFusion(g, 0.5f),
          "外部参照门控: 无外部参照低速仍允许 ZUPT");
    g = Icm45686UpdateZuptExtRefGuard(g, false, 0.2f);
    check(g.cap_noise && !g.disable_zupt, "外部参照门控: ≥3s 封顶噪声");
    check(Icm45686ZuptExternalRefAllowsFusion(g, 0.5f),
          "外部参照门控: 封顶期低速仍允许 ZUPT");
    check(!Icm45686ZuptExternalRefAllowsFusion(g, 1.0f),
          "外部参照门控: 封顶期 1.0m/s 拒绝 ZUPT（慢速巡航盲区）");
    g = Icm45686UpdateZuptExtRefGuard(g, false, 27.1f);
    check(g.disable_zupt, "外部参照门控: ≥30s 停用 ZUPT");
    check(!Icm45686ZuptExternalRefAllowsFusion(g, 0.0f),
          "外部参照门控: 停用后即使零速也不融合");

    g = Icm45686UpdateZuptExtRefGuard(g, true, 0.005f);
    check(!g.cap_noise && !g.disable_zupt && g.no_ext_ref_s == 0.0f,
          "外部参照门控: 外部参照恢复后清零/解除");

    Icm45686StaticAidProfile p{0.03f, 0.02f, 0.04f, false};
    g = Icm45686UpdateZuptExtRefGuard(g, false, 3.1f);
    Icm45686ApplyZuptExtRefNoiseFloor(p, g);
    check(p.vel_ne_std_mps >= Icm45686ZuptExtRefNoiseFloorMps &&
              p.vel_d_std_mps >= Icm45686ZuptExtRefNoiseFloorMps,
          "外部参照门控: 封顶把 0.02/0.03 抬到 0.30");
  }

  if (g_fail_count == 0)
  {
    std::printf("\n=== 静止辅助调度测试全部通过 ===\n");
    return 0;
  }
  std::printf("\n=== 失败 %d 项 ===\n", g_fail_count);
  return 1;
}
