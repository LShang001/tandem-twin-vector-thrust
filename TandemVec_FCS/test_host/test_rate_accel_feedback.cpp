/**
 * @file    test_rate_accel_feedback.cpp
 * @module  固件/宿主回归测试（实测角加速度反馈 AAF）
 * @brief   验证稳态零修正、恒加速度收敛、符号、复位、非有限输入与限幅
 * @details 被测：`include/RateAccelFeedback.h`。
 *          编译：g++ -std=c++17 -Iinclude test_host/test_rate_accel_feedback.cpp
 *                -o bin/aaf
 * @theory  速率差分 = 角加速度；负反馈增加阻尼，不应在稳态产生偏置。
 *          @ref VER-003 §3.11
 * @frame   N/A
 * @units   deg/s、s、deg/s²
 * @note    默认 gain=0.8、tau=0.02 s；本模块未接入默认飞控路径。
 * @test    A1 恒定速率零修正；A2 恒加速度收敛到 -Kα；A3 符号；
 *          A4 复位；A5 非有限输入冻结；A6 限幅
 * @see     include/RateAccelFeedback.h
 */
#include <cmath>
#include <cstdio>

#include "RateAccelFeedback.h"

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

  // A1 恒定速率：差分 0，修正应保持 0。
  {
    bfs::RateAccelFeedback a;
    float corr = 0.0f;
    for (int i = 0; i < 200; ++i) corr = a.update(10.0f, dt);
    CHECK(std::fabs(corr) < 1e-5f, "A1 恒定速率零修正");
  }

  // A2 恒加速度 100 deg/s²：滤波收敛后修正 ≈ -0.8*100。
  {
    bfs::RateAccelFeedback a;
    float rate = 0.0f;
    float corr = 0.0f;
    for (int i = 0; i < 2000; ++i)
    {
      rate += 100.0f * dt;
      corr = a.update(rate, dt);
    }
    CHECK(std::fabs(corr + 80.0f) < 2.0f, "A2 恒加速度收敛到 -Kα");
  }

  // A3 符号：速率增大时修正为负，减小时为正。
  {
    bfs::RateAccelFeedback up;
    bfs::RateAccelFeedback down;
    for (int i = 0; i < 20; ++i)
    {
      (void)up.update(static_cast<float>(i) * 5.0f, dt);
      (void)down.update(100.0f - static_cast<float>(i) * 5.0f, dt);
    }
    CHECK(up.correctionDps2() < 0.0f, "A3 加速时负修正");
    CHECK(down.correctionDps2() > 0.0f, "A3 减速时正修正");
  }

  // A4 复位：修正与内部滤波归零。
  {
    bfs::RateAccelFeedback a;
    for (int i = 0; i < 100; ++i) (void)a.update(static_cast<float>(i), dt);
    a.reset();
    CHECK(std::fabs(a.correctionDps2()) < 1e-6f, "A4 复位修正为 0");
    CHECK(std::fabs(a.alphaFilteredDps2()) < 1e-6f, "A4 复位滤波为 0");
    const float corr = a.update(50.0f, dt);
    CHECK(std::fabs(corr) < 1e-6f, "A4 复位首拍不产生冲击");
  }

  // A5 非有限输入：保持上一拍修正。
  {
    bfs::RateAccelFeedback a;
    for (int i = 0; i < 50; ++i) (void)a.update(static_cast<float>(i), dt);
    const float before = a.correctionDps2();
    (void)a.update(std::nanf(""), dt);
    (void)a.update(1.0f, 0.0f);
    CHECK(std::fabs(a.correctionDps2() - before) < 1e-6f,
          "A5 非有限输入冻结");
  }

  // A6 限幅：极端阶跃不得超过 max_correction_dps2。
  {
    bfs::RateAccelFeedback a;
    bfs::RateAccelFeedbackParams p;
    p.max_correction_dps2 = 50.0f;
    a.setParams(p);
    float corr = 0.0f;
    for (int i = 0; i < 50; ++i)
      corr = a.update((i % 2) ? 1000.0f : 0.0f, dt);
    CHECK(std::fabs(corr) <= 50.0f + 1e-5f, "A6 修正限幅");
  }

  if (g_fail == 0)
  {
    std::printf("=== test_rate_accel_feedback: 全部通过 ===\n");
    return 0;
  }
  std::printf("=== test_rate_accel_feedback: %d 项失败 ===\n", g_fail);
  return 1;
}
