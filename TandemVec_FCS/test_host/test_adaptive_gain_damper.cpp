/**
 * @file    test_adaptive_gain_damper.cpp
 * @module  固件/宿主回归测试（fw-tests：选择性振荡降增益）
 * @brief   验证滤波/最小幅值/门控后的真实振荡检测、噪声抑制、恢复、下限与复位
 * @details 被测：`include/AdaptiveGainDamper.h`。
 *          输入：合成速率误差序列/dt/allow_update；处理：滤波+换向计数；
 *          输出：PASS/FAIL 计数，main 返回非零即失败。
 *          编译：g++ -std=c++17 -Iinclude test_host/test_adaptive_gain_damper.cpp
 *                -o bin/agd
 * @theory  只有滤波后幅值足够、驾驶杆回中时的持续换向才降增益；
 *          噪声/小幅值/驾驶杆输入不得触发。
 *          @ref CTL-001 §8.6；VER-003 §3.8
 * @frame   N/A
 * @units   s、rad/s、无量纲
 * @note    默认 1.0 s/2/0.90/1.01/0.55，filter 0.05 s，min 0.035 rad/s；
 *          模块未接入默认飞控路径。
 * @test    A1 真实振荡降增益；A2 恢复；A3 下限；A4 非有限；A5 复位；
 *          A6 小噪声不触发；A7 门控不更新；A8 小幅值不触发
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

static float osc_at(float t_s, float amp)
{
  return amp * std::sin(2.0f * 3.14159265358979323846f * 3.0f * t_s);
}

int main()
{
  const float dt = 0.005f;

  // A1 3 Hz 真实振荡：幅值 1 rad/s，1 s 窗口内多次换向 → gain < 1。
  {
    bfs::AdaptiveGainDamper d;
    float g = 1.0f;
    for (int i = 0; i < 600; ++i) g = d.update(osc_at(i * dt, 1.0f), dt);
    CHECK(g < 1.0f, "A1 真实振荡降增益");
    CHECK(g >= d.params().floor - 1e-6f, "A1 不低于 floor");
  }

  // A2 安静：恒定 0 无换向，20 s 后恢复到 1。
  {
    bfs::AdaptiveGainDamper d;
    for (int i = 0; i < 600; ++i) (void)d.update(osc_at(i * dt, 1.0f), dt);
    const float low = d.gain();
    for (int i = 0; i < 4000; ++i) (void)d.update(0.0f, dt);
    CHECK(d.gain() > low, "A2 安静后恢复");
    CHECK(d.gain() <= 1.0f + 1e-6f, "A2 不超过 1");
  }

  // A3 持续振荡：增益停在下限。
  {
    bfs::AdaptiveGainDamper d;
    for (int i = 0; i < 6000; ++i) (void)d.update(osc_at(i * dt, 1.0f), dt);
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
    for (int i = 0; i < 1200; ++i) (void)d.update(osc_at(i * dt, 1.0f), dt);
    d.reset();
    CHECK(std::fabs(d.gain() - 1.0f) < 1e-6f, "A5 复位到 1");
  }

  // A6 小噪声：幅值 0.01 rad/s < 0.035，20 s 不触发。
  {
    bfs::AdaptiveGainDamper d;
    for (int i = 0; i < 4000; ++i)
      (void)d.update((i % 2) ? 0.01f : -0.01f, dt);
    CHECK(std::fabs(d.gain() - 1.0f) < 1e-6f, "A6 小噪声不触发");
  }

  // A7 驾驶杆活动门控：allow_update=false 时不更新；恢复后真实振荡触发。
  {
    bfs::AdaptiveGainDamper d;
    for (int i = 0; i < 1200; ++i)
      (void)d.update(osc_at(i * dt, 1.0f), dt, false);
    CHECK(std::fabs(d.gain() - 1.0f) < 1e-6f, "A7 门控关闭不降增益");
    for (int i = 0; i < 600; ++i) (void)d.update(osc_at(i * dt, 1.0f), dt, true);
    CHECK(d.gain() < 1.0f, "A7 门控恢复后振荡触发");
  }

  // A8 小幅值振荡：0.01 rad/s < 0.035，不触发。
  {
    bfs::AdaptiveGainDamper d;
    for (int i = 0; i < 1200; ++i) (void)d.update(osc_at(i * dt, 0.01f), dt);
    CHECK(std::fabs(d.gain() - 1.0f) < 1e-6f, "A8 小幅值不触发");
  }

  if (g_fail == 0)
  {
    std::printf("=== test_adaptive_gain_damper: 全部通过 ===\n");
    return 0;
  }
  std::printf("=== test_adaptive_gain_damper: %d 项失败 ===\n", g_fail);
  return 1;
}
