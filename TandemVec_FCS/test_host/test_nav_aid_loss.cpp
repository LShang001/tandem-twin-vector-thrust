/**
 * @file    test_nav_aid_loss.cpp
 * @module  固件/宿主回归测试（GNSS+光流组合失效检测）
 * @brief   验证 NavAidLossDetector 的连续丢失、滞回恢复与非法 dt 处理
 * @details 被测：`include/NavAidLossDetector.h`；200 Hz 步进。
 * @frame  N/A
 * @units  s
 * @note   本测试只锁判定逻辑，不代表飞控降级/下降已接入。
 * @see     TandemVec_FCS/include/NavAidLossDetector.h
 */
#include <cmath>
#include <cstdio>

#include "NavAidLossDetector.h"

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

static bfs::NavAidLossDetector::Output run(bfs::NavAidLossDetector &det,
                                           bool gnss_stale, bool flow_invalid,
                                           float seconds)
{
  bfs::NavAidLossDetector::Output out;
  const int steps = static_cast<int>(seconds / 0.005f);
  for (int i = 0; i < steps; ++i)
  {
    const bfs::NavAidLossDetector::Output step =
        det.update(gnss_stale, flow_invalid, 0.005f);
    out.recovered = out.recovered || step.recovered;
    out.degraded = step.degraded;
    out.descent_requested = step.descent_requested;
  }
  return out;
}

int main()
{
  {
    bfs::NavAidLossDetector det;
    const auto out = run(det, true, true, 0.4f);
    CHECK(!out.degraded, "D1 0.4 s 不降级");
  }
  {
    bfs::NavAidLossDetector det;
    const auto out = run(det, true, true, 0.6f);
    CHECK(out.degraded, "D2 0.6 s 降级");
    CHECK(!out.descent_requested, "D2 0.6 s 不请求下降");
  }
  {
    bfs::NavAidLossDetector det;
    const auto out = run(det, true, true, 2.1f);
    CHECK(out.degraded, "D3 2.1 s 降级");
    CHECK(out.descent_requested, "D3 2.1 s 请求下降");
  }
  {
    bfs::NavAidLossDetector det;
    run(det, true, true, 2.1f);
    const auto out = run(det, false, false, 1.1f);
    CHECK(!out.degraded, "D4 恢复 1.1 s 清降级");
    CHECK(!out.descent_requested, "D4 恢复 1.1 s 清下降");
    CHECK(out.recovered, "D4 恢复事件只报一次");
  }
  {
    bfs::NavAidLossDetector det;
    run(det, true, true, 0.4f);
    run(det, false, true, 0.1f);
    const auto out = run(det, true, true, 0.4f);
    CHECK(!out.degraded, "D5 间断丢失不累计");
  }
  {
    bfs::NavAidLossDetector det;
    const auto out = det.update(true, true, std::nanf(""));
    CHECK(!out.degraded && !out.descent_requested, "D6 非法 dt 不推进");
  }
  {
    bfs::NavAidLossDetector det;
    const auto out = run(det, true, false, 2.1f);
    CHECK(!out.degraded, "D7 仅 GNSS 陈旧不触发（光流可用）");
  }
  {
    bfs::NavAidLossDetector det;
    const auto out = run(det, false, true, 2.1f);
    CHECK(!out.degraded, "D8 仅光流无效不触发（GNSS 可用）");
  }

  if (g_fail == 0)
  {
    std::printf("=== test_nav_aid_loss: 全部通过 ===\n");
    return 0;
  }
  std::printf("=== test_nav_aid_loss: %d 项失败 ===\n", g_fail);
  return 1;
}
