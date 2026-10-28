/**
 * @file    test_ins_baro_health.cpp
 * @module  固件/宿主回归测试（fw-tests：气压健康监测）
 * @brief   验证气压慢漂/台阶/尖峰/卡死检测与 GNSS 无效时暂停漂移判定
 * @details 被测：`include/ins_baro_health.h`（平台无关）。
 *          输入：合成气压/GNSS 高度、EKF 垂速、dt；处理：逐帧 `update()`；
 *          输出：PASS/FAIL 计数，main 返回非零即失败。
 *          编译：g++ -std=c++17 -Iinclude test_host/test_ins_baro_health.cpp -o bin/baroh
 * @theory  双源垂直一致性：GNSS 短期可信时气压慢漂表现为残差斜率
 *          @ref REF-121 §7.3.1 p.255–259
 * @frame   N/A（标量高度/垂速）
 * @units   m、m/s、s
 * @note    这是逻辑/边界测试；真实气压噪声谱、温度漂移与 GNSS 高度质量需实机。
 * @test    T1 标称不误报；T2 0.5 m/s 漂移在 3.5 s 内识别；T3 50 m 台阶；
 *          T4 单帧尖峰；T5 卡死（垂速非零）；T6 GNSS 无效不判漂移；T7 reset
 * @see     TandemVec_FCS/include/ins_baro_health.h
 */
#include <cmath>
#include <cstdio>

#include "ins_baro_health.h"

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

static const float kDt = 0.005f;

int main()
{
  // T1 标称：气压与 GNSS 差 5 m + 0.05 m 慢摆，5 s 不误报。
  {
    bfs::BaroHealthMonitor m;
    for (int i = 0; i < 1000; ++i)
    {
      const float t = static_cast<float>(i) * kDt;
      m.update(1005.0f + 0.05f * std::sin(0.2f * t), 1000.0f, 0.0f, true, kDt);
    }
    CHECK(!m.suspect(), "T1 nominal not suspect");
  }

  // T2 0.5 m/s 漂移：8 s 内应判 Suspect，速率约 0.5（2 m 残差门限后
  // 检测约 6 s；仍远早于垂直门控 25 m/50 s 的矛盾时刻）。
  {
    bfs::BaroHealthMonitor m;
    bool flagged = false;
    float rate_at_flag = 0.0f;
    for (int i = 0; i < 2000; ++i)  // 10 s
    {
      const float t = static_cast<float>(i) * kDt;
      const auto s = m.update(1005.0f + 0.5f * t, 1000.0f, 0.0f, true, kDt);
      if (!flagged && s.state == bfs::BaroHealthState::Suspect &&
          ((s.flags & bfs::baroHealthFlagBit(bfs::BaroHealthFlag::Drift)) != 0U) &&
          t <= 8.0f)
      {
        flagged = true;
        rate_at_flag = s.drift_rate_mps;
      }
    }
    CHECK(flagged, "T2 drift flagged within 8 s");
    CHECK(std::fabs(rate_at_flag - 0.5f) < 0.2f, "T2 drift rate ~0.5 m/s");
  }

  // T3 50 m 台阶：5 s 内判 Suspect。
  {
    bfs::BaroHealthMonitor m;
    bool flagged = false;
    for (int i = 0; i < 1000; ++i)
    {
      const float t = static_cast<float>(i) * kDt;
      const float baro = 1005.0f + ((t >= 2.0f) ? 50.0f : 0.0f);
      const auto s = m.update(baro, 1000.0f, 0.0f, true, kDt);
      if (s.state == bfs::BaroHealthState::Suspect) flagged = true;
    }
    CHECK(flagged, "T3 50 m step suspect");
  }

  // T4 单帧尖峰：5 m/5 ms = 1000 m/s，立刻 Spike。
  {
    bfs::BaroHealthMonitor m;
    m.update(1005.0f, 1000.0f, 0.0f, true, kDt);
    const auto s = m.update(1010.0f, 1000.0f, 0.0f, true, kDt);
    CHECK((s.flags & bfs::baroHealthFlagBit(bfs::BaroHealthFlag::Spike)) != 0U,
          "T4 spike flag");
  }

  // T5 卡死：气压不变、垂速 3 m/s，3.2 s 后 Stuck。
  {
    bfs::BaroHealthMonitor m;
    bool flagged = false;
    for (int i = 0; i < 700; ++i)
    {
      const auto s = m.update(1005.0f, 1000.0f, 3.0f, true, kDt);
      if ((s.flags & bfs::baroHealthFlagBit(bfs::BaroHealthFlag::Stuck)) != 0U)
      {
        flagged = true;
      }
    }
    CHECK(flagged, "T5 stuck flag");
  }

  // T6 GNSS 无效：同样的漂移序列不应判 Drift。
  {
    bfs::BaroHealthMonitor m;
    bool drift = false;
    for (int i = 0; i < 1400; ++i)
    {
      const float t = static_cast<float>(i) * kDt;
      const auto s = m.update(1005.0f + 0.5f * t, 1000.0f, 0.0f, false, kDt);
      if ((s.flags & bfs::baroHealthFlagBit(bfs::BaroHealthFlag::Drift)) != 0U)
      {
        drift = true;
      }
    }
    CHECK(!drift, "T6 no drift flag without GNSS");
  }

  // T7 Drift 迟滞：漂移停止后需连续 quiet 3 s 才释放，不能随噪声抖动。
  {
    bfs::BaroHealthMonitor m;
    float baro = 1005.0f;
    // 200 Hz 下需 ~4 s 累积 2 m 残差 + 2 s 确认；跑 10 s 保证锁存。
    for (int i = 0; i < 2000; ++i)
    {
      baro = 1005.0f + 0.5f * static_cast<float>(i) * kDt;
      m.update(baro, 1000.0f, 0.0f, true, kDt);
    }
    const float held = baro;
    bool still_latched = true;
    for (int i = 0; i < 200; ++i)  // 停止后 1 s 仍应判 Drift
    {
      const auto s = m.update(held, 1000.0f, 0.0f, true, kDt);
      still_latched =
          still_latched &&
          ((s.flags & bfs::baroHealthFlagBit(bfs::BaroHealthFlag::Drift)) != 0U);
    }
    bool cleared = false;
    for (int i = 0; i < 1600; ++i)  // 再 8 s 内应释放
    {
      const auto s = m.update(held, 1000.0f, 0.0f, true, kDt);
      if ((s.flags & bfs::baroHealthFlagBit(bfs::BaroHealthFlag::Drift)) == 0U)
      {
        cleared = true;
      }
    }
    CHECK(still_latched, "T7 drift latch holds 1 s after stop");
    CHECK(cleared, "T7 drift latch releases within 8 s quiet");
  }

  // T8 reset。
  {
    bfs::BaroHealthMonitor m;
    for (int i = 0; i < 1400; ++i)
    {
      const float t = static_cast<float>(i) * kDt;
      m.update(1005.0f + 0.5f * t, 1000.0f, 0.0f, true, kDt);
    }
    m.reset();
    CHECK(!m.suspect(), "T8 reset clears state");
    CHECK(m.status().drift_s == 0.0f, "T8 reset clears timers");
  }

  if (g_fail == 0)
  {
    std::printf("PASS: ins_baro_health (T1-T8)\n");
    return 0;
  }
  std::printf("FAILED: %d check(s)\n", g_fail);
  return 1;
}
