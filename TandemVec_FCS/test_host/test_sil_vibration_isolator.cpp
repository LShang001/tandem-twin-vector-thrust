/**
 * @file    test_sil_vibration_isolator.cpp
 * @module  固件/宿主测试（fw-tests：弹载减震座传递率）
 * @brief   验证 sil_vibration_isolator.h 的单自由度基础激励传递率：
 *          低频≈1、r=√2 穿越 1、共振放大、高频衰减、非法输入回退 1。
 * @details 只测纯标量公式，不涉及 SIL 主循环。
 * @frame   N/A
 * @units   Hz、无量纲
 */
#include "sil_vibration_isolator.h"

#include <cmath>
#include <cstdio>

static int g_fail = 0;
#define CHECK(cond, msg)                                                        \
  do                                                                            \
  {                                                                             \
    if (!(cond))                                                                \
    {                                                                           \
      std::printf("FAIL: %s\n", msg);                                            \
      ++g_fail;                                                                 \
    }                                                                           \
    else                                                                        \
    {                                                                           \
      std::printf("PASS: %s\n", msg);                                            \
    }                                                                           \
  } while (0)

int main()
{
  const float fn = 50.0f;
  const float zeta = 0.15f;
  {
    const float t = sil::VibrationTransmissibility(0.5f, fn, zeta);
    CHECK(std::fabs(t - 1.0f) < 0.01f, "T1 低频 f≪fn：传递率≈1");
  }
  {
    const float t = sil::VibrationTransmissibility(fn * std::sqrt(2.0f), fn,
                                                   zeta);
    CHECK(std::fabs(t - 1.0f) < 0.01f, "T2 r=√2：传递率穿越 1");
  }
  {
    const float t = sil::VibrationTransmissibility(fn, fn, 0.1f);
    CHECK(t > 4.5f && t < 5.5f, "T3 共振 r=1、ζ=0.1：放大≈5");
  }
  {
    const float lo = sil::VibrationTransmissibility(2.0f * fn, fn, zeta);
    const float hi = sil::VibrationTransmissibility(4.0f * fn, fn, zeta);
    CHECK(lo > hi && hi < 0.12f, "T4 高频：单调衰减，4fn 处 <0.12");
  }
  {
    const float t = sil::VibrationTransmissibility(80.0f, fn, zeta);
    CHECK(t > 0.5f && t < 0.8f, "T5 fn=50/ζ=0.15、80 Hz：传递率 0.5–0.8");
  }
  {
    CHECK(std::fabs(sil::VibrationTransmissibility(80.0f, 0.0f, zeta) - 1.0f) <
              1e-6f,
          "T6 fn=0（未装减震座）：回退 1");
    CHECK(std::fabs(sil::VibrationTransmissibility(0.0f, fn, zeta) - 1.0f) <
              1e-6f,
          "T7 f=0：回退 1");
  }

  if (g_fail == 0)
  {
    std::printf("=== test_sil_vibration_isolator: 全部通过 ===\n");
    return 0;
  }
  std::printf("=== test_sil_vibration_isolator: %d 项失败 ===\n", g_fail);
  return 1;
}
