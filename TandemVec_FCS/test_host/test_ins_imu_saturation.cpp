/**
 * @file    test_ins_imu_saturation.cpp
 * @module  固件/宿主回归测试（fw-tests：IMU 饱和锁存 Q 建议）
 * @brief   验证饱和锁存的进入/保持/释放计时、短脉冲不锁存、标称值直通与配置边界
 * @details 被测：`include/ins_imu_saturation.h`（平台无关）。
 *          输入：逐帧“是否饱和”与 dt；处理：逐帧 `update()`；
 *          输出：PASS/FAIL 计数，main 返回非零即失败。
 *          编译：g++ -std=c++17 -Iinclude test_host/test_ins_imu_saturation.cpp -o bin/satq
 * @theory  削顶期间比力幅值不可信，应把误差计入过程噪声而非量测噪声
 *          @ref REF-121 §7.3.1 p.255–259；AGENTS.md（±8 g 生产量程）
 * @frame   N/A（标量幅值判定）
 * @units   s、m/s²
 * @note    这是逻辑/边界测试；真实削顶由 ICM 量程与安装决定，需上板确认饱和标志。
 * @test    T1 短脉冲不锁存；T2 连续饱和到 enter_s 锁存；T3 短干净窗不释放；
 *          T4 干净窗到 release_s 释放；T5 锁存时噪声建议取较大值；
 *          T6 未锁存直通标称值；T7 非有限/非正 dt 不改状态；T8 reset 清空
 * @see     TandemVec_FCS/include/ins_imu_saturation.h、src/navigation_task.cpp
 */
#include <cmath>
#include <cstdio>

#include "ins_imu_saturation.h"

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

static bfs::ImuSaturationQMonitor makeMonitor()
{
  bfs::ImuSaturationQConfig cfg;
  cfg.enter_s = 0.05f;
  cfg.release_s = 0.5f;
  cfg.inflate_accel_std_mps2 = 25.0f;
  return bfs::ImuSaturationQMonitor(cfg);
}

int main()
{
  const float dt = 0.005f;

  // T1 短脉冲（3 帧=15ms < enter 50ms）不锁存。
  {
    auto m = makeMonitor();
    for (int i = 0; i < 3; ++i)
    {
      CHECK(!m.update(true, dt), "T1 short burst must not latch");
    }
    CHECK(!m.active(), "T1 inactive after short burst");
  }

  // T2 连续饱和 10 帧（50ms）锁存。
  {
    auto m = makeMonitor();
    bool latched = false;
    for (int i = 0; i < 10; ++i)
    {
      latched = m.update(true, dt);
    }
    CHECK(latched, "T2 latch at enter_s");
    CHECK(m.active(), "T2 active");
  }

  // T3 锁存后短干净窗（0.2s < release 0.5s）不释放。
  {
    auto m = makeMonitor();
    for (int i = 0; i < 12; ++i)
    {
      m.update(true, dt);
    }
    for (int i = 0; i < 40; ++i)
    {
      m.update(false, dt);
    }
    CHECK(m.active(), "T3 hold through short clean window");
  }

  // T4 干净窗累计到 release_s 释放。
  {
    auto m = makeMonitor();
    for (int i = 0; i < 12; ++i)
    {
      m.update(true, dt);
    }
    for (int i = 0; i < 100; ++i)
    {
      m.update(false, dt);
    }
    CHECK(!m.active(), "T4 release at release_s");
  }

  // T4b 释放前再次饱和应清零干净窗。
  {
    auto m = makeMonitor();
    for (int i = 0; i < 12; ++i)
    {
      m.update(true, dt);
    }
    for (int i = 0; i < 90; ++i)
    {
      m.update(false, dt);
    }
    m.update(true, dt); // 重新饱和
    for (int i = 0; i < 90; ++i)
    {
      m.update(false, dt);
    }
    CHECK(m.active(), "T4b saturation resets clean timer");
  }

  // T5 锁存时噪声建议取较大值。
  {
    auto m = makeMonitor();
    for (int i = 0; i < 12; ++i)
    {
      m.update(true, dt);
    }
    CHECK(std::fabs(m.accelStdMps2(0.25f) - 25.0f) < 1.0e-6f,
          "T5 inflate nominal to 25");
    CHECK(std::fabs(m.accelStdMps2(40.0f) - 40.0f) < 1.0e-6f,
          "T5 keep larger nominal");
  }

  // T6 未锁存直通标称值（含 0）。
  {
    auto m = makeMonitor();
    CHECK(std::fabs(m.accelStdMps2(0.25f) - 0.25f) < 1.0e-6f,
          "T6 passthrough nominal");
    CHECK(std::fabs(m.accelStdMps2(0.0f) - 0.0f) < 1.0e-6f,
          "T6 passthrough zero");
  }

  // T7 非有限/非正 dt 不改状态。
  {
    auto m = makeMonitor();
    for (int i = 0; i < 12; ++i)
    {
      m.update(true, dt);
    }
    m.update(false, 0.0f);
    m.update(false, -1.0f);
    m.update(false, std::nanf(""));
    CHECK(m.active(), "T7 invalid dt keeps latch");
    CHECK(m.cleanDurationS() == 0.0f, "T7 invalid dt does not advance clean");
  }

  // T8 reset 清空。
  {
    auto m = makeMonitor();
    for (int i = 0; i < 12; ++i)
    {
      m.update(true, dt);
    }
    m.reset();
    CHECK(!m.active(), "T8 reset clears latch");
    CHECK(m.saturatedDurationS() == 0.0f, "T8 reset clears timers");
  }

  if (g_fail == 0)
  {
    std::printf("PASS: ins_imu_saturation (T1-T8)\n");
    return 0;
  }
  std::printf("FAILED: %d check(s)\n", g_fail);
  return 1;
}
