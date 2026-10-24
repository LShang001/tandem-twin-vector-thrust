/**
 * @file    test_thrust_allocation.cpp
 * @module  固件/宿主回归测试（fw-tests：垂直推力机型映射）
 * @brief   验证期望天向比力 + cos_tilt → 总推力/油门的机型映射、倾角补偿、
 *          上下饱和、非有限输入与旧 mapFloat 口径一致性
 * @details 被测：`include/ThrustAllocation.h`。
 *          输入：合成比力/cos_tilt/质量/最大推力；
 *          处理：allocateLinearThrustThrottle；
 *          输出：PASS/FAIL 计数，main 返回非零即失败。
 *          编译：g++ -std=c++17 -Iinclude test_host/test_thrust_allocation.cpp
 *                -o bin/talloc
 * @theory  F_total = m·a_up / max(|cos_tilt|, 0.5)；throttle = F·100/F_max
 *          @ref CTL-001 §8.4；CTL-003 §2；PROP-001 §2
 * @frame   NED 天向；cos_tilt = |R33|
 * @units   比力 m/s²、质量 kg、推力 N、油门 %
 * @note    本测试锁定的是当前 TandemVec **线性油门口径**，不是 ESC 的 kT·ω²
 *          物理逆；换推力曲线必须另立机型曲线并重跑 SIL。
 * @test    T1 悬停换算；T2 倾角补偿；T3 cos 下限；T4 负 cos 取绝对值；
 *          T5 高饱和；T6 低饱和；T7 非有限/非法参数；T8 旧 mapFloat 口径
 * @see     include/ThrustAllocation.h、src/flight_control.cpp
 */
#include <cmath>
#include <cstdio>

#include "ThrustAllocation.h"
#include "TandemVec_Config.h"

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

static bool nearf(const float a, const float b, const float tol = 1e-4f)
{
  return std::fabs(a - b) <= tol;
}

static bfs::ThrustAllocationInput makeInput(const float specific_force,
                                            const float cos_tilt)
{
  bfs::ThrustAllocationInput in;
  in.specific_force_up_mps2 = specific_force;
  in.thrust_axis_up_cos = cos_tilt;
  in.mass_kg = kDefaultTandemVecParams.m;
  in.max_total_thrust_n = 27.5f;
  return in;
}

int main()
{
  const float m = kDefaultTandemVecParams.m;
  const float fmax = 27.5f;

  // T1 悬停：a_up=g，cos=1 → F=m·g，油门=F·100/Fmax。
  {
    const auto out = bfs::allocateLinearThrustThrottle(makeInput(9.81f, 1.0f));
    const float expect_f = m * 9.81f;
    const float expect_thr = expect_f * 100.0f / fmax;
    CHECK(out.valid, "T1 有效");
    CHECK(nearf(out.total_thrust_n, expect_f, 1e-5f), "T1 总推力=m·g");
    CHECK(nearf(out.throttle_pct, expect_thr, 1e-4f), "T1 油门换算");
    CHECK(!out.saturated_high && !out.saturated_low, "T1 无饱和");
  }

  // T2 倾角补偿：cos=0.5 → 总推力翻倍。
  {
    const auto base = bfs::allocateLinearThrustThrottle(makeInput(9.81f, 1.0f));
    const auto tilted = bfs::allocateLinearThrustThrottle(makeInput(9.81f, 0.5f));
    CHECK(tilted.valid, "T2 有效");
    CHECK(nearf(tilted.total_thrust_n, base.total_thrust_n * 2.0f, 1e-4f),
          "T2 cos=0.5 推力翻倍");
    CHECK(nearf(tilted.throttle_pct, base.throttle_pct * 2.0f, 1e-4f),
          "T2 cos=0.5 油门翻倍");
  }

  // T3 cos 下限：0.2 与 0.1 都按 0.5 处理（最多补偿到 60°）。
  {
    const auto at_limit = bfs::allocateLinearThrustThrottle(makeInput(9.81f, 0.5f));
    const auto below = bfs::allocateLinearThrustThrottle(makeInput(9.81f, 0.2f));
    CHECK(nearf(below.total_thrust_n, at_limit.total_thrust_n, 1e-4f),
          "T3 cos<0.5 钳到下限");
  }

  // T4 负 cos（机头朝下/朝上）取绝对值，几何关系不变。
  {
    const auto pos = bfs::allocateLinearThrustThrottle(makeInput(9.81f, 0.7f));
    const auto neg = bfs::allocateLinearThrustThrottle(makeInput(9.81f, -0.7f));
    CHECK(nearf(neg.total_thrust_n, pos.total_thrust_n, 1e-4f),
          "T4 负 cos 取绝对值");
  }

  // T5 高饱和：需求超过最大推力 → 100% 且置 saturated_high。
  {
    const auto out = bfs::allocateLinearThrustThrottle(makeInput(1000.0f, 1.0f));
    CHECK(out.valid, "T5 有效");
    CHECK(nearf(out.total_thrust_n, fmax, 1e-4f), "T5 总推力钳到 Fmax");
    CHECK(nearf(out.throttle_pct, 100.0f, 1e-4f), "T5 油门 100%");
    CHECK(out.saturated_high, "T5 高饱和标记");
    CHECK(!out.saturated_low, "T5 低饱和不置位");
  }

  // T6 低饱和：负比力（下降加速超过 1g）→ 0% 且置 saturated_low。
  {
    const auto out = bfs::allocateLinearThrustThrottle(makeInput(-1.0f, 1.0f));
    CHECK(out.valid, "T6 有效");
    CHECK(nearf(out.total_thrust_n, 0.0f, 1e-6f), "T6 总推力钳到 0");
    CHECK(nearf(out.throttle_pct, 0.0f, 1e-6f), "T6 油门 0%");
    CHECK(out.saturated_low, "T6 低饱和标记");
    CHECK(!out.saturated_high, "T6 高饱和不置位");
  }

  // T7 非有限/非法参数：valid=false，输出回零（不得把 NaN 写进油门）。
  {
    auto nan_force = makeInput(9.81f, 1.0f);
    nan_force.specific_force_up_mps2 = std::nanf("");
    CHECK(!bfs::allocateLinearThrustThrottle(nan_force).valid,
          "T7 NaN 比力无效");

    auto nan_cos = makeInput(9.81f, 1.0f);
    nan_cos.thrust_axis_up_cos = std::nanf("");
    CHECK(!bfs::allocateLinearThrustThrottle(nan_cos).valid,
          "T7 NaN cos 无效");

    auto bad_max = makeInput(9.81f, 1.0f);
    bad_max.max_total_thrust_n = 0.0f;
    const auto out_bad_max = bfs::allocateLinearThrustThrottle(bad_max);
    CHECK(!out_bad_max.valid && nearf(out_bad_max.throttle_pct, 0.0f, 1e-6f),
          "T7 Fmax=0 无效且油门回零");

    auto bad_mass = makeInput(9.81f, 1.0f);
    bad_mass.mass_kg = -1.0f;
    CHECK(!bfs::allocateLinearThrustThrottle(bad_mass).valid,
          "T7 负质量无效");

    auto bad_min_cos = makeInput(9.81f, 1.0f);
    bad_min_cos.min_cos_tilt = 0.0f;
    CHECK(!bfs::allocateLinearThrustThrottle(bad_min_cos).valid,
          "T7 min_cos=0 无效");
  }

  // T8 旧 mapFloat 口径：throttle = (F - 0)·(100 - 0)/(Fmax - 0) + 0。
  {
    const float force = m * 9.81f;
    const float legacy = (force - 0.0f) * (100.0f - 0.0f) / (fmax - 0.0f) + 0.0f;
    const auto out = bfs::allocateLinearThrustThrottle(makeInput(9.81f, 1.0f));
    CHECK(nearf(out.throttle_pct, legacy, 1e-5f),
          "T8 与旧 mapFloat 口径逐值一致");
  }

  if (g_fail == 0)
  {
    std::printf("=== test_thrust_allocation: 全部通过 ===\n");
    return 0;
  }
  std::printf("=== test_thrust_allocation: %d 项失败 ===\n", g_fail);
  return 1;
}
