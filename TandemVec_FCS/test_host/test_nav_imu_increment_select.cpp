/**
 * @file    test_nav_imu_increment_select.cpp
 * @module  固件/宿主回归测试（fw-tests：导航 IMU 增量源选择与故障回退）
 * @brief   验证总开关关闭时完全惰性、Failed 当拍回退、后持递减与再触发、
 *          hold_ticks 归一、reset 清零，"确认延迟把切入推离故障前沿"这一设计不变量，
 *          以及轴映射与逐轴低通的可交换性
 * @details 被测：`include/NavImuIncrementSelect.h`。
 *          输入：合成 `imu_failed` 序列（2 kHz 节拍）；
 *          处理：`NavImuIncrementSelector::select()`；
 *          输出：PASS/FAIL 计数，main 返回非零即失败。
 *          编译：g++ -std=c++17 -Iinclude test_host/test_nav_imu_increment_select.cpp
 *                -o bin/navisel
 * @theory  控制链软件低通的时变群延迟会被自动 GNSS 延时估计吸收后回放，
 *          大过载段造成群体级退化；让导航增量绕过该低通可提带宽，
 *          但代价出现在惯导冻结故障上 ⇒ 回退必须按故障确认状态切入
 *          @ref TandemVec-EKF-Report/导弹6DOF-导航SIL-2026-10-27.md §4.24/§4.24h/§4.24k-终/§4.24m
 * @frame   N/A（只做"用哪一路增量"的布尔决策，无坐标系语义）
 * @units   拍（2 kHz 下 1 拍 = 0.5 ms；10 000 拍 = 5 s）
 * @note    本测试只锁选择器语义，不评估导航精度；精度与安全性由 SIL 矩阵验收
 * @test    T1 关闭恒滤波且计数不动；T2 Failed 当拍回退；T3 后持递减后恢复未滤波；
 *          T4 关闭时 Failed 也不产生任何计数；T5 连续 Failed 刷新锁存；
 *          T6 hold_ticks=0 归一为 1；T7 reset 清零；
 *          T8 确认延迟使切入落点离开故障前沿（§4.24m-W 组不变量）；
 *          T9 定常带符号置换与逐轴一阶低通可交换（raw 分支与 SIL 口径等价的前提）
 * @see     include/NavImuIncrementSelect.h、src/sensor_imu.cpp、include/ins_imu_health.h
 */
#include <cmath>
#include <cstdio>

#include "NavImuIncrementSelect.h"

using bfs::NavImuIncrementSelector;
using bfs::NavImuSource;

static int g_fail = 0;
#define CHECK(cond, msg)                                                        \
  do                                                                            \
  {                                                                             \
    if (!(cond))                                                                \
    {                                                                           \
      std::printf("FAIL: %s (line %d)\n", msg, __LINE__);                      \
      g_fail++;                                                                 \
    }                                                                           \
  } while (0)

/// 跑 n 拍，全部按 `failed` 喂给选择器，返回末拍源。
static NavImuSource pump(NavImuIncrementSelector& sel, uint32_t n, bool failed)
{
  NavImuSource src = NavImuSource::Filtered;
  for (uint32_t i = 0; i < n; ++i)
  {
    src = sel.select(failed);
  }
  return src;
}

int main()
{
  // T1：总开关关闭 ⇒ 恒 Filtered，且两个计数都不动（默认惰性的机检形式）
  {
    NavImuIncrementSelector sel(/*enabled=*/false);
    pump(sel, 100, /*failed=*/false);
    CHECK(sel.select(false) == NavImuSource::Filtered, "T1 disabled must stay Filtered");
    CHECK(sel.rawTicks() == 0U && sel.fallbackTicks() == 0U,
          "T1 disabled must not touch counters");
  }

  // T2：开启后 Failed 当拍即回退（锁存置满，本拍不递减）
  {
    NavImuIncrementSelector sel(true, 10U);
    CHECK(sel.select(false) == NavImuSource::Raw, "T2 pre-fault must be Raw");
    CHECK(sel.select(true) == NavImuSource::Filtered, "T2 fault tick must fall back");
    CHECK(sel.inFallback() && sel.holdRemainingTicks() == 10U,
          "T2 fault tick must set the latch full without decrementing");
    CHECK(pump(sel, 9, false) == NavImuSource::Filtered,
          "T2 held ticks must stay Filtered");
    CHECK(sel.holdRemainingTicks() == 1U, "T2 nine clean ticks leave one hold tick");
    CHECK(sel.select(false) == NavImuSource::Raw,
          "T2 drain to zero must return to Raw");
  }

  // T3：后持递减到 0 后恢复未滤波（恢复期被覆盖，之后重新提带宽）
  {
    NavImuIncrementSelector sel(true, 5U);
    sel.select(true);                       // 触发：hold=5，故障拍不递减
    CHECK(pump(sel, 4, false) == NavImuSource::Filtered, "T3 hold tail must stay Filtered");
    CHECK(sel.holdRemainingTicks() == 1U, "T3 four clean ticks leave one hold tick");
    CHECK(sel.select(false) == NavImuSource::Raw, "T3 after drain must return to Raw");
    CHECK(sel.holdRemainingTicks() == 0U, "T3 hold must drain to zero");
  }

  // T4：关闭时即便 Failed 持续，也不产生任何计数（不污染诊断、不改行为）
  {
    NavImuIncrementSelector sel(false, 100U);
    pump(sel, 50, true);
    CHECK(sel.rawTicks() == 0U && sel.fallbackTicks() == 0U,
          "T4 disabled must not count fallback ticks");
    CHECK(sel.select(true) == NavImuSource::Filtered, "T4 disabled output stays Filtered");
  }

  // T5：故障持续期间每个 Failed 拍都刷新锁存 ⇒ 后持从"最后一次 Failed"起算
  {
    NavImuIncrementSelector sel(true, 10U);
    for (uint32_t i = 0; i < 30U; ++i)
    {
      CHECK(sel.select(true) == NavImuSource::Filtered, "T5 sustained fault must fall back");
    }
    CHECK(sel.holdRemainingTicks() == 10U, "T5 latch must be refreshed each fault tick");
    CHECK(pump(sel, 9, false) == NavImuSource::Filtered,
          "T5 held ticks after fault must stay Filtered");
    CHECK(sel.holdRemainingTicks() == 1U, "T5 nine clean ticks leave one hold tick");
    CHECK(sel.select(false) == NavImuSource::Raw, "T5 raw resumes");
  }

  // T6：hold_ticks=0 归一为 1（不允许"零后持"导致除零/永不回退的歧义）
  {
    NavImuIncrementSelector sel(true, 0U);
    CHECK(sel.holdTicks() == 1U, "T6 hold_ticks=0 must normalize to 1");
    CHECK(sel.select(true) == NavImuSource::Filtered, "T6 fault tick still falls back");
    CHECK(sel.select(false) == NavImuSource::Raw, "T6 single-tick hold then raw");
  }

  // T7：reset 清零全部诊断量
  {
    NavImuIncrementSelector sel(true, 20U);
    pump(sel, 10, true);
    sel.reset();
    CHECK(sel.rawTicks() == 0U && sel.fallbackTicks() == 0U &&
              sel.holdRemainingTicks() == 0U && !sel.inFallback(),
          "T7 reset must clear counters and latch");
  }

  // T8：§4.24m-W 组不变量——回退切入必须落在"确认之后"，即不能在故障第一拍就切。
  //     这里用 2 kHz 模拟 1 s 故障（2000 拍），但健康状态需要 1000 拍才确认 Failed，
  //     断言回退覆盖的是后半段（≈1000+5000 拍），而不是从第 1 拍开始。
  {
    constexpr uint32_t kFaultTicks = 2000U;      // 注入的 1 s 冻结
    constexpr uint32_t kConfirmTicks = 1000U;    // 检测器确认所需拍数
    constexpr uint32_t kHold = 10000U;           // 5 s 后持
    NavImuIncrementSelector sel(true, kHold);
    uint32_t first_fallback = 0xFFFFFFFFU;
    for (uint32_t i = 0; i < 20000U; ++i)
    {
      const bool confirmed_failed = (i >= kFaultTicks - kConfirmTicks) &&
                                    (i < kFaultTicks);
      const NavImuSource src = sel.select(confirmed_failed);
      if (src == NavImuSource::Filtered && first_fallback == 0xFFFFFFFFU)
      {
        first_fallback = i;
      }
    }
    CHECK(first_fallback == kFaultTicks - kConfirmTicks,
          "T8 fallback must start at confirmation, not at fault onset");
    // 覆盖时长 = 确认后的故障剩余 + 后持 ⇒ 落在 §4.24m 的"后半段覆盖"一侧
    CHECK(sel.fallbackTicks() > 0U && sel.rawTicks() > 0U,
          "T8 both paths must be exercised");
    CHECK(sel.fallbackTicks() >= kFaultTicks - kConfirmTicks,
          "T8 must cover the post-front part of the fault");
  }

  // T9：定常带符号置换与逐轴一阶低通**可交换**——这是"固件在传感器系低通后取 raw
  //     分支"与"SIL 在机体系低通"逐轴等价的前提。若此性质不成立，§4.24m 的
  //     SIL→固件迁移就不成立，因此必须有机检而不是只在注释里断言。
  //     映射（与 sensor_imu.cpp 步骤 5 逐字一致）：bX=-sZ、bY=+sX、bZ=-sY
  {
    auto lpf = [](float prev, float x, float alpha) { return prev + alpha * (x - prev); };
    // 传感器系逐轴 α（生产口径：x/y=0.02、z=0.05；陀螺三轴 0.15）
    const float aS[3] = {0.02f, 0.02f, 0.05f};
    // 机体系对应 α：bX←sZ=0.05、bY←sX=0.02、bZ←sY=0.02
    const float aB[3] = {0.05f, 0.02f, 0.02f};

    auto mapBody = [](const float s[3], float b[3]) {
      b[0] = -s[2];
      b[1] = s[0];
      b[2] = -s[1];
    };

    float lp_then_map[3] = {0.0f, 0.0f, 0.0f}; // 先低通（传感器系）后映射
    float state_s[3] = {0.0f, 0.0f, 0.0f};
    float map_then_lp[3] = {0.0f, 0.0f, 0.0f}; // 先映射后低通（机体系）
    float state_b[3] = {0.0f, 0.0f, 0.0f};

    float worst = 0.0f;
    for (int i = 0; i < 4000; ++i)
    {
      // 确定性伪随机激励（不含 rand，跨平台逐位可复现）
      const float ph = 0.001f * static_cast<float>(i);
      float s[3] = {1.3f * static_cast<float>(std::sin(6.1 * ph)) + 0.2f,
                    -0.7f * static_cast<float>(std::cos(2.3 * ph)) + 9.8f,
                    0.9f * static_cast<float>(std::sin(0.7 * ph)) - 1.0f};
      float b_in[3];
      mapBody(s, b_in);
      for (int k = 0; k < 3; ++k)
      {
        state_s[k] = lpf(state_s[k], s[k], aS[k]);
        state_b[k] = lpf(state_b[k], b_in[k], aB[k]);
      }
      float mapped_s[3];
      mapBody(state_s, mapped_s);
      for (int k = 0; k < 3; ++k)
      {
        lp_then_map[k] = mapped_s[k];
        const float d = std::fabs(lp_then_map[k] - state_b[k]);
        if (d > worst)
        {
          worst = d;
        }
      }
    }
    CHECK(worst < 1e-4f, "T9 per-axis LPF and constant signed permutation must commute");
  }

  if (g_fail == 0)
  {
    std::printf("=== test_nav_imu_increment_select: 全部通过 ===\n");
    return 0;
  }
  std::printf("=== 失败 %d 项 ===\n", g_fail);
  return 1;
}
