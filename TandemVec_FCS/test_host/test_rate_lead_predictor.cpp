/**
 * @file    test_rate_lead_predictor.cpp
 * @module  固件/宿主回归测试（fw-tests：速率预测）
 * @brief   验证 ω_pred=ω+L·α、零 L、非有限输入与非法参数
 * @details 被测：`include/RateLeadPredictor.h`；纯标量数学，无 plant。
 * @theory  ω(t+L)≈ω(t)+α·L 一阶预测；L=0 时严格直通
 *          @ref VER-003 §3.11
 * @frame   N/A
 * @units   rad/s、rad/s²、s
 * @note    默认不接入飞控路径。
 * @test    A1 预测值；A2 零 L 直通；A3 非有限；A4 非法 L
 * @see     include/RateLeadPredictor.h、fast_mc.py
 */
#include <cmath>
#include <cstdio>

#include "RateLeadPredictor.h"

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
  bfs::RateLeadPredictor p;
  bfs::RateLeadParams q;
  q.lead_s = 0.05f;
  p.setParams(q);

  // A1 1 + 0.05*10 = 1.5
  CHECK(std::fabs(p.predict(1.0f, 10.0f) - 1.5f) < 1e-6f, "A1 预测值");

  // A2 L=0 直通
  q.lead_s = 0.0f;
  p.setParams(q);
  CHECK(std::fabs(p.predict(1.0f, 10.0f) - 1.0f) < 1e-6f, "A2 零 L 直通");

  // A3 非有限输入
  q.lead_s = 0.05f;
  p.setParams(q);
  CHECK(std::fabs(p.predict(std::nanf(""), 1.0f)) < 1e-6f, "A3 非有限 rate");
  CHECK(std::fabs(p.predict(1.0f, std::nanf("")) - 1.0f) < 1e-6f,
        "A3 非有限 alpha 回退 rate");

  // A4 非法 L 回退 rate
  q.lead_s = -1.0f;
  p.setParams(q);
  CHECK(std::fabs(p.predict(1.0f, 10.0f) - 1.0f) < 1e-6f, "A4 非法 L");

  if (g_fail == 0)
  {
    std::printf("=== test_rate_lead_predictor: 全部通过 ===\n");
    return 0;
  }
  std::printf("=== test_rate_lead_predictor: %d 项失败 ===\n", g_fail);
  return 1;
}
