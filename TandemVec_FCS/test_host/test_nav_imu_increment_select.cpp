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
 *          T9 定常带符号置换与逐轴一阶低通可交换（raw 分支与 SIL 口径等价的前提）；
 *          T10 RUB→FRD 逐轴符号（轴置换红线）；T11 选源逐位透传；
 *          T12 六轴一次性取齐（禁止逐轴混选）；T13 与改动前内联表达式逐位等价
 * @see     include/NavImuIncrementSelect.h、src/sensor_imu.cpp、include/ins_imu_health.h
 */
#include <cmath>
#include <cstdio>
#include <cstring>

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

  // T10：RUB→FRD 定常置换的逐轴符号（2026-08-07 轴置换红线：
  //      bX=-sZ、bY=+sX、bZ=-sY，加速度与角速度同一置换）。
  {
    const float unit[3][3] = {{1.0f, 0.0f, 0.0f},
                              {0.0f, 1.0f, 0.0f},
                              {0.0f, 0.0f, 1.0f}};
    // 传感器三轴单位向量 → 机体系期望像
    const float exp_acc[3][3] = {{0.0f, 1.0f, 0.0f},
                                 {0.0f, 0.0f, -1.0f},
                                 {-1.0f, 0.0f, 0.0f}};
    for (int axis = 0; axis < 3; ++axis)
    {
      const bfs::NavImuBodySample b =
          bfs::NavImuSensorRubToBodyFrd(unit[axis], unit[axis]);
      for (int k = 0; k < 3; ++k)
      {
        CHECK(b.accel_g[k] == exp_acc[axis][k],
              "T10 accel permutation must be bX=-sZ, bY=+sX, bZ=-sY");
        CHECK(b.gyro_dps[k] == exp_acc[axis][k],
              "T10 gyro must use the same permutation as accel");
      }
    }
  }

  // T11：选源必须逐位透传——Filtered 原样返回滤波值，Raw 原样返回未滤波值
  //      （不允许任何换算、混合或再映射发生在这个函数里）。
  {
    const bfs::NavImuBodySample filtered = {{1.5f, -2.25f, 3.125f},
                                            {0.5f, -0.75f, 1.875f}};
    const bfs::NavImuBodySample raw = {{-9.5f, 8.25f, -7.125f},
                                       {-6.5f, 5.75f, -4.875f}};
    const bfs::NavImuBodySample f = bfs::NavImuSelectIncrement(
        filtered, raw, bfs::NavImuSource::Filtered);
    const bfs::NavImuBodySample r =
        bfs::NavImuSelectIncrement(filtered, raw, bfs::NavImuSource::Raw);
    CHECK(std::memcmp(&f, &filtered, sizeof(f)) == 0,
          "T11 Filtered source must be bit-identical to the filtered sample");
    CHECK(std::memcmp(&r, &raw, sizeof(r)) == 0,
          "T11 Raw source must be bit-identical to the raw sample");
  }

  // T12：六轴一次性取齐——raw 与 filtered 的每个分量都不同，任何"逐轴混选"
  //      都会让结果与 raw 整结构逐位比较失败（这是接线级 bug 的机检形式）。
  {
    const bfs::NavImuBodySample filtered = {{1.0f, 2.0f, 3.0f}, {4.0f, 5.0f, 6.0f}};
    const bfs::NavImuBodySample raw = {{-1.0f, -2.0f, -3.0f}, {-4.0f, -5.0f, -6.0f}};
    const bfs::NavImuBodySample r =
        bfs::NavImuSelectIncrement(filtered, raw, bfs::NavImuSource::Raw);
    CHECK(std::memcmp(&r, &raw, sizeof(r)) == 0,
          "T12 raw selection must not mix per-axis with filtered");
    bool any_leak = false;
    for (int k = 0; k < 3; ++k)
    {
      any_leak = any_leak || (r.accel_g[k] == filtered.accel_g[k]) ||
                 (r.gyro_dps[k] == filtered.gyro_dps[k]);
    }
    CHECK(!any_leak, "T12 no axis may come from the other source");
  }

  // T13：与改动前内联表达式逐位等价——映射 + SI 换算的乘序不变，
  //      否则"默认关时行为不变"这句话不成立。
  {
    // 与固件同值（G_TO_MS2 ← kDefaultTandemVecParams.g；DEG_TO_RAD ← Arduino 核心）。
    // 断言对常数取值不敏感：只验证乘法次序与映射一致，不比物理量。
    const float kG = 9.79f;
    const float kDegToRad = 0.017453292519943295f;
    int mismatch = 0;
    for (int i = 0; i < 256; ++i)
    {
      const float ph = 0.017f * static_cast<float>(i);
      const float af[3] = {1.7f * static_cast<float>(std::sin(ph)) + 0.3f,
                           -1.1f * static_cast<float>(std::cos(0.7f * ph)) + 9.6f,
                           0.9f * static_cast<float>(std::sin(0.3f * ph)) - 0.4f};
      const float gf[3] = {12.0f * static_cast<float>(std::sin(0.11f * ph)),
                           -7.0f * static_cast<float>(std::cos(0.13f * ph)),
                           3.5f * static_cast<float>(std::sin(0.05f * ph))};
      const float ar[3] = {af[0] + 0.4f, af[1] - 0.2f, af[2] + 0.7f};
      const float gr[3] = {gf[0] - 3.0f, gf[1] + 2.0f, gf[2] - 1.0f};

      // 改动前的内联写法（传感器系 RUB → 机体系 FRD + SI 换算）
      const float legacy_f[6] = {-af[2] * kG, af[0] * kG, -af[1] * kG,
                                 -gf[2] * kDegToRad, gf[0] * kDegToRad,
                                 -gf[1] * kDegToRad};
      const float legacy_r[6] = {-ar[2] * kG, ar[0] * kG, -ar[1] * kG,
                                 -gr[2] * kDegToRad, gr[0] * kDegToRad,
                                 -gr[1] * kDegToRad};

      const bfs::NavImuBodySample fb = bfs::NavImuSensorRubToBodyFrd(af, gf);
      const bfs::NavImuBodySample rb = bfs::NavImuSensorRubToBodyFrd(ar, gr);
      const float now_f[6] = {fb.accel_g[0] * kG, fb.accel_g[1] * kG,
                              fb.accel_g[2] * kG, fb.gyro_dps[0] * kDegToRad,
                              fb.gyro_dps[1] * kDegToRad, fb.gyro_dps[2] * kDegToRad};
      const float now_r[6] = {rb.accel_g[0] * kG, rb.accel_g[1] * kG,
                              rb.accel_g[2] * kG, rb.gyro_dps[0] * kDegToRad,
                              rb.gyro_dps[1] * kDegToRad, rb.gyro_dps[2] * kDegToRad};
      mismatch += (std::memcmp(legacy_f, now_f, sizeof(now_f)) != 0) ? 1 : 0;
      mismatch += (std::memcmp(legacy_r, now_r, sizeof(now_r)) != 0) ? 1 : 0;
    }
    CHECK(mismatch == 0, "T13 helper must be bit-identical to the legacy inline mapping");
  }

  if (g_fail == 0)
  {
    std::printf("=== test_nav_imu_increment_select: 全部通过 ===\n");
    return 0;
  }
  std::printf("=== 失败 %d 项 ===\n", g_fail);
  return 1;
}
