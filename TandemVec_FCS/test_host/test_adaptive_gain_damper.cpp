/**
 * @file    test_adaptive_gain_damper.cpp
 * @module  固件/宿主回归测试（fw-tests：振荡检测降增益）
 * @brief   验证高频换向降增益、安静恢复、下限/上限、非有限输入与复位
 * @details 被测：`include/AdaptiveGainDamper.h`。
 *          输入：合成速率误差序列/dt；处理：AdaptiveGainDamper::update；
 *          输出：PASS/FAIL 计数，main 返回非零即失败。
 *          编译：g++ -std=c++17 -Iinclude test_host/test_adaptive_gain_damper.cpp
 *                -o bin/agd
 * @theory  高频换向 → 相位裕度不足；降增益拉回稳定区，安静后缓慢恢复
 *          @ref CTL-001 §8.6；VER-003 §3.6
 * @frame   N/A
 * @units   s、无量纲
 * @note    默认参数 0.15 s/4/0.85/1.01/0.45，来自 TandemVec L1 MC；
 *          模块未接入默认飞控路径，只建议高 T/W 机型启用。
 * @test    A1 高频换向降增益；A2 安静恢复；A3 下限；A4 非有限输入；A5 复位
 * @see     include/AdaptiveGainDamper.h、simulations/three_vehicle_mc/fast_mc.py
 */
#include <cmath>
#include <cstdio>

#include "AdaptiveGainDamper.h"

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

int main()
{
  const float dt = 0.005f;

  // A1 高频换向：每拍换向，窗口后 gain < 1。
  {
    bfs::AdaptiveGainDamper d;
    float g = 1.0f;
    for (int i = 0; i < 60; ++i) g = d.update((i % 2) ? 1.0f : -1.0f, dt);
    CHECK(g < 1.0f, "A1 高频换向降增益");
    CHECK(g >= d.params().floor - 1e-6f, "A1 不低于 floor");
  }

  // A2 安静：恒定误差无换向，窗口后恢复到 1。
  {
    bfs::AdaptiveGainDamper d;
    for (int i = 0; i < 60; ++i) (void)d.update((i % 2) ? 1.0f : -1.0f, dt);
    const float low = d.gain();
    for (int i = 0; i < 4000; ++i) (void)d.update(1.0f, dt);
    CHECK(d.gain() > low, "A2 安静后恢复");
    CHECK(d.gain() <= 1.0f + 1e-6f, "A2 不超过 1");
  }

  // A3 持续振荡：增益停在下限。
  {
    bfs::AdaptiveGainDamper d;
    for (int i = 0; i < 5000; ++i) (void)d.update((i % 2) ? 1.0f : -1.0f, dt);
    CHECK(std::fabs(d.gain() - d.params().floor) < 1e-5f, "A3 振荡到 floor");
  }

  // A4 非有限输入：增益不变。
  {
    bfs::AdaptiveGainDamper d;
    const float g0 = d.gain();
    (void)d.update(std::nanf(""), dt);
    (void)d.update(0.1f, 0.0f);
    CHECK(std::fabs(d.gain() - g0) < 1e-6f, "A4 非有限输入不改变增益");
  }

  // A5 复位：回到 1。
  {
    bfs::AdaptiveGainDamper d;
    for (int i = 0; i < 200; ++i) (void)d.update((i % 2) ? 1.0f : -1.0f, dt);
    d.reset();
    CHECK(std::fabs(d.gain() - 1.0f) < 1e-6f, "A5 复位到 1");
  }

  if (g_fail == 0)
  {
    std::printf("=== test_adaptive_gain_damper: 全部通过 ===\n");
    return 0;
  }
  std::printf("=== test_adaptive_gain_damper: %d 项失败 ===\n", g_fail);
  return 1;
}
