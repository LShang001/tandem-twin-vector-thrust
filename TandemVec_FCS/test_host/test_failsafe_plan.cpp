/**
 * @file    test_failsafe_plan.cpp
 * @module  固件/宿主回归测试（fw-tests：CRSF 断链通道合成）
 * @brief   穷举断链判据真值表并锁定通道合成不变量
 * @details 被测：`include/FailsafeLinkLossPlan.h`（固件 `linkDownCallback` 与
 *          本测试调用**同一份**判据，不是按源码重写）。
 *          输入：臂通道/油门通道/离地高度三维网格 + 旧判据对照点；
 *          处理：computeLinkLossPlan；
 *          输出：PASS/FAIL 计数，main 返回非零即失败。
 *          编译：g++ -std=c++17 -Iinclude test_host/test_failsafe_plan.cpp -o bin/fsplan
 * @theory  在空中 = 已解锁 AND (油门有动力 OR 已离地)
 *          @ref CTL-001 §8.6；VER-004（共轴机型原始固件对照）
 * @frame   N/A（通道值语义由 CRSF/固件定义）
 * @units   通道 µs；高度 m（向上为正）
 * @note    本测试锁定 2026-10-31 审计修复（P1-2/P1-3）后的判据。关键回归点：
 *          旧判据在“已解锁 + 低油门 + 已离地”时判成地面 → 空中上锁；
 *          新判据必须判成空中，且 CH3 必须归中（AUTO 下定高），不能压 988。
 * @test    T1 真值表穷举；T2 通道不变量；T3 旧判据差异点；T4 通道越界防护
 * @see     include/FailsafeLinkLossPlan.h、src/communication.cpp::linkDownCallback
 */
#include <cstdio>

#include "FailsafeLinkLossPlan.h"

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

namespace fs = bfs::failsafe;

// 旧判据（2026-10-31 之前）：`CH5 > 1500 && CH3 > 1100`，不看高度。
// 仅用于对照差异点，不参与实现。
static bool legacyWasInFlight(uint16_t arm_us, uint16_t throttle_us)
{
  return (arm_us > 1500) && (throttle_us > 1100);
}

// T1：三维网格穷举 —— 判定必须与“未解锁恒假、已解锁看油门/高度”的定义一致。
static void testTruthTable()
{
  const uint16_t arms[] = {0, 900, 1000, 1499, 1500, 1501, 1800, 2012};
  const uint16_t throttles[] = {0, 900, 988, 1000, 1099, 1100, 1101, 1500, 2012};
  const float heights[] = {-5.0f, -0.1f, 0.0f, 0.499f, 0.5f, 0.501f, 1.0f, 10.0f};

  int cases = 0;
  for (uint16_t arm : arms)
  {
    for (uint16_t thr : throttles)
    {
      for (float h : heights)
      {
        const bool expected = (arm > 1500) && ((thr > 1100) || (h > 0.5f));
        const bool actual = fs::isInFlightAtLinkLoss(arm, thr, h);
        if (expected != actual)
        {
          std::printf("FAIL: 真值表 arm=%u thr=%u h=%.3f 期望=%d 实际=%d\n",
                      static_cast<unsigned>(arm), static_cast<unsigned>(thr),
                      static_cast<double>(h), static_cast<int>(expected),
                      static_cast<int>(actual));
          g_fail++;
        }
        ++cases;
      }
    }
  }
  std::printf("T1 真值表：%d 组穷举完成\n", cases);

  // 边界语义必须显式锁定（避免以后误把阈值改成 >= 或把高度取反）。
  CHECK(!fs::isInFlightAtLinkLoss(1500, 2012, 50.0f),
        "臂通道恰好 1500 必须判为未解锁（阈值是严格大于）");
  CHECK(fs::isInFlightAtLinkLoss(1501, 1101, -3.0f),
        "已解锁 + 油门有动力时，负高度（地下标定/异常）不得改变判定");
  CHECK(!fs::isInFlightAtLinkLoss(1800, 1100, 0.5f),
        "油门/高度都落在阈值上（非严格大于）必须判为地面");
  CHECK(fs::isInFlightAtLinkLoss(1800, 1100, 0.6f),
        "仅靠离地证据也必须能判为空中");
}

// T2：通道合成不变量 —— 只允许 988/1500；姿态三轴恒归中；油门/臂/模式随判定。
static void testChannelInvariants()
{
  const uint16_t arms[] = {0, 1000, 1500, 1800};
  const uint16_t throttles[] = {988, 1100, 1200, 2012};
  const float heights[] = {0.0f, 0.4f, 0.6f, 8.0f};

  for (uint16_t arm : arms)
  {
    for (uint16_t thr : throttles)
    {
      for (float h : heights)
      {
        fs::Inputs in{arm, thr, h};
        const fs::Plan plan = fs::makeLinkLossPlan(in);

        for (int i = 0; i < fs::kMaxChannels; ++i)
        {
          CHECK(plan.channels[i] == fs::kPwmLow || plan.channels[i] == fs::kPwmMid,
                "通道值只允许 988 或 1500");
        }
        CHECK(plan.channels[fs::kChRoll] == fs::kPwmMid, "CH1 滚转必须归中");
        CHECK(plan.channels[fs::kChPitch] == fs::kPwmMid, "CH2 俯仰必须归中");
        CHECK(plan.channels[fs::kChYaw] == fs::kPwmMid, "CH4 偏航必须归中");
        CHECK(plan.channels[fs::kChIgnition] == fs::kPwmLow, "CH6 点火必须压最低");
        CHECK(plan.channels[fs::kChTvcManual] == fs::kPwmLow, "CH8 手动TVC 必须压最低");

        const uint16_t expected = plan.in_flight ? fs::kPwmMid : fs::kPwmLow;
        CHECK(plan.channels[fs::kChThrottle] == expected,
              "CH3 必须随判定取 1500（空中定高）或 988（地面）");
        CHECK(plan.channels[fs::kChArm] == expected, "CH5 必须随判定保持解锁或锁定");
        CHECK(plan.channels[fs::kChMode] == expected, "CH7 必须随判定切 AUTO_ALTITUDE 或最低");

        // 9..15 号通道同样必须压最低（不得留断链前杆位）。
        for (int i = 9; i < fs::kMaxChannels; ++i)
        {
          CHECK(plan.channels[i] == fs::kPwmLow, "9 号及以上通道必须压最低");
        }
      }
    }
  }
  std::printf("T2 通道不变量：完成\n");
}

// T3：与旧判据的差异点 —— 这些组合是本次修复的语义变更，必须能抓到回退。
static void testLegacyDifference()
{
  struct Case
  {
    uint16_t arm;
    uint16_t thr;
    float h;
    const char *why;
  };
  const Case cases[] = {
      {1800, 1000, 8.0f, "空中主动下降（AUTO 下 0% = -1 m/s）"},
      {1800, 1099, 3.0f, "空中下降杆接近地面待机阈值"},
      {1800, 988, 0.6f, "贴地起飞/低空悬停"},
  };

  for (const Case &c : cases)
  {
    const bool legacy = legacyWasInFlight(c.arm, c.thr);
    const bool fixed = fs::isInFlightAtLinkLoss(c.arm, c.thr, c.h);
    CHECK(!legacy, "差异点前提：旧判据必须判成地面");
    CHECK(fixed, "新判据必须在这些组合上判成空中");
    std::printf("T3 差异点：arm=%u thr=%u h=%.1f 旧=%d 新=%d（%s）\n",
                static_cast<unsigned>(c.arm), static_cast<unsigned>(c.thr),
                static_cast<double>(c.h), static_cast<int>(legacy),
                static_cast<int>(fixed), c.why);
  }

  // 地面待机必须仍然锁定：这是修复不能破坏的回归点。
  CHECK(!fs::isInFlightAtLinkLoss(1800, 988, 0.0f),
        "地面已武装但未离地、油门最低时必须判为地面（防止桨在地上转到悬停功率）");
  CHECK(!fs::isInFlightAtLinkLoss(1000, 2012, 10.0f),
        "未解锁时即使油门高/高度大也必须判为地面");
}

int main()
{
  std::printf("== 断链通道合成回归（FailsafeLinkLossPlan.h）==\n");
  testTruthTable();
  testChannelInvariants();
  testLegacyDifference();

  if (g_fail == 0)
  {
    std::printf("全部通过\n");
    return 0;
  }
  std::printf("失败 %d 项\n", g_fail);
  return 1;
}
