/**
 * @file    test_liquid_rocket_model.cpp
 * @module  固件/宿主回归测试（fw-tests：液体火箭变质量推进与喷管几何）
 * @brief   验证点火/解锁门控、阀门与推力速率限幅、最小推力、比冲质量流、
 *          推进剂耗尽关机、非法参数与双轴喷管几何逆解
 * @details 被测：`include/LiquidRocketModel.h`。
 *          输入：CCAEGT 示例参数（Tmax=200 N、Tmin=80 N、Isp=200 s、干重 13.5 kg）；
 *          处理：LiquidRocketPropulsion::update / mapNozzleGimbal；
 *          输出：PASS/FAIL 计数，main 返回非零即失败。
 *          编译：g++ -std=c++17 -Iinclude test_host/test_liquid_rocket_model.cpp
 *                -o bin/lrm
 * @theory  T=u·Tmax；m_dot=T/(g0·Isp)；m=m_dry+m_prop；θx/θy 由 asin 几何分解
 *          @ref CTL-003 §3.3；PROP-001 §2；THY-003 §2
 * @frame   N/A（推进标量/喷管两轴角）
 * @units   N、kg、s、kg/s、deg
 * @note    参数来自外部液体火箭工程 CCAEGT 示例，不是本机标定值；本测试只锁
 *          模型逻辑与安全边界，不外推实机推力/质量。
 * @test    T1 初始质量；T2 点火/解锁互锁；T3 油门→推力；T4 速率限幅；
 *          T5 最小推力；T6 比冲质量流；T7 推进剂耗尽；T8 非法输入；
 *          T9 喷管几何
 * @see     include/LiquidRocketModel.h、CTL-003 §3.3、外部 VTVL_LiquidRocket_FC
 */
#include <cmath>
#include <cstdio>

#include "LiquidRocketModel.h"

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

static bool nearf(const float a, const float b, const float tol = 1e-3f)
{
  return std::fabs(a - b) <= tol;
}

static bfs::LiquidRocketCommand cmd(float throttle, bool ignition, bool unlocked,
                                    float dt = 0.005f)
{
  bfs::LiquidRocketCommand c;
  c.throttle = throttle;
  c.ignition_enabled = ignition;
  c.unlocked = unlocked;
  c.dt_s = dt;
  return c;
}

int main()
{
  const bfs::LiquidRocketParams params{};

  // T1 初始状态：m = m_dry + m_prop，阀门/推力 0。
  {
    bfs::LiquidRocketPropulsion engine;
    CHECK(engine.init(params), "T1 初始化");
    const auto &s = engine.state();
    CHECK(nearf(s.mass_kg, params.dry_mass_kg + params.initial_propellant_kg),
          "T1 初始质量");
    CHECK(nearf(s.propellant_kg, params.initial_propellant_kg), "T1 初始推进剂");
    CHECK(!s.ignited && nearf(s.thrust_n, 0.0f), "T1 未点火推力 0");
  }

  // T2 点火/解锁互锁：未解锁或未使能不点火；解锁+使能才锁存。
  {
    bfs::LiquidRocketPropulsion engine;
    engine.init(params);
    auto out = engine.update(cmd(0.8f, true, false));
    CHECK(!out.ignited && nearf(out.thrust_n, 0.0f), "T2 未解锁不点火");
    out = engine.update(cmd(0.8f, false, true));
    CHECK(!out.ignited && nearf(out.thrust_n, 0.0f), "T2 未使能不点火");
    out = engine.update(cmd(0.0f, true, true));
    CHECK(out.ignited && nearf(out.thrust_n, 0.0f), "T2 点火锁存但零油门无推力");
    out = engine.update(cmd(0.8f, true, true));
    CHECK(out.ignited && out.thrust_n > 0.0f, "T2 点火后油门产生推力");
  }

  // T3 油门→推力：稳态约 u·Tmax（0.8 → 160 N）。
  {
    bfs::LiquidRocketPropulsion engine;
    engine.init(params);
    bfs::LiquidRocketOutput out;
    for (int i = 0; i < 2000; ++i)
    {
      out = engine.update(cmd(0.8f, true, true));
    }
    CHECK(nearf(out.valve_position, 0.8f, 1e-3f), "T3 阀门到 0.8");
    CHECK(nearf(out.thrust_n, 0.8f * params.max_thrust_n, 0.5f), "T3 稳态推力");
    CHECK(out.mass_kg < params.dry_mass_kg + params.initial_propellant_kg,
          "T3 质量下降");
  }

  // T4 速率限幅：单步阀门/推力增量不超过配置速率。
  {
    bfs::LiquidRocketPropulsion engine;
    engine.init(params);
    (void)engine.update(cmd(0.0f, true, true));
    const auto out = engine.update(cmd(1.0f, true, true));
    CHECK(out.valve_position <= params.valve_rate_limit_per_s * 0.005f + 1e-6f,
          "T4 阀门速率限幅");
    CHECK(out.thrust_n <= params.thrust_rate_limit_nps * 0.005f + 1e-3f,
          "T4 推力速率限幅");
  }

  // T5 最小稳定推力：低油门请求被抬到 Tmin（泵压/燃烧稳定下限）。
  {
    bfs::LiquidRocketPropulsion engine;
    engine.init(params);
    for (int i = 0; i < 1000; ++i)
    {
      (void)engine.update(cmd(0.1f, true, true));
    }
    const auto out = engine.update(cmd(0.1f, true, true));
    CHECK(nearf(out.thrust_n, params.min_thrust_n, 0.5f), "T5 最小推力");
    CHECK(!out.flameout, "T5 未耗尽不熄火");
  }

  // T6 比冲质量流：m_dot = T/(g0·Isp)，dt 内质量损失一致。
  {
    bfs::LiquidRocketPropulsion engine;
    engine.init(params);
    for (int i = 0; i < 2000; ++i) (void)engine.update(cmd(0.8f, true, true));
    const float m_before = engine.state().mass_kg;
    const auto out = engine.update(cmd(0.8f, true, true));
    const float expect_mdot = out.thrust_n / (params.g0_mps2 * params.isp_s);
    const float expect_dm = expect_mdot * 0.005f;
    CHECK(nearf(out.mass_flow_kgps, expect_mdot, 1e-5f), "T6 质量流率公式");
    CHECK(nearf(m_before - out.mass_kg, expect_dm, 1e-5f), "T6 质量损失");
  }

  // T7 推进剂耗尽：推力/阀门归零、flameout、质量=干重。
  {
    bfs::LiquidRocketPropulsion engine;
    bfs::LiquidRocketParams small = params;
    small.initial_propellant_kg = 0.05f;
    engine.init(small);
    bfs::LiquidRocketOutput out;
    for (int i = 0; i < 200000 && !out.flameout; ++i)
    {
      out = engine.update(cmd(1.0f, true, true));
    }
    CHECK(out.flameout, "T7 推进剂耗尽熄火");
    CHECK(nearf(out.propellant_kg, 0.0f, 1e-6f), "T7 推进剂归零");
    CHECK(nearf(out.mass_kg, small.dry_mass_kg, 1e-5f), "T7 质量=干重");
    CHECK(nearf(out.thrust_n, 0.0f, 1e-4f), "T7 耗尽推力 0");
    CHECK(nearf(out.valve_position, 0.0f, 1e-4f), "T7 耗尽阀门 0");
  }

  // T8 非法输入：参数/油门/dt 非有限或越界 → valid=false，不写硬件量。
  {
    bfs::LiquidRocketPropulsion engine;
    bfs::LiquidRocketParams bad = params;
    bad.max_thrust_n = 0.0f;
    CHECK(!engine.init(bad), "T8 非法参数初始化失败");

    engine.init(params);
    auto out = engine.update(cmd(std::nanf(""), true, true));
    CHECK(!out.valid, "T8 NaN 油门无效");
    out = engine.update(cmd(1.2f, true, true));
    CHECK(!out.valid, "T8 油门越界无效");
    out = engine.update(cmd(0.5f, true, true, 0.0f));
    CHECK(!out.valid, "T8 dt=0 无效");
  }

  // T9 双轴喷管几何（外部工程 asin 分解口径）：
  //    小 roll=0.1 纯滚 → x≈4.05°、y≈4.06°；纯 pitch=0.1 → x≈-4.05°、y≈4.06°；
  //    零输入→0；满 roll=1 在 60° 限幅下 x=45°、y 被限到 60°并饱和；
  //    零限幅/NaN → 无效。
  {
    const float small = 0.1f;
    const float expect_x = std::asin(small * 0.70710678f) * 57.2957795f;
    const float expect_y = std::asin(
        (small * 0.70710678f) /
        std::cos(std::asin(small * 0.70710678f))) * 57.2957795f;
    auto g = bfs::mapNozzleGimbal(small, 0.0f, 60.0f);
    CHECK(g.valid, "T9 小 roll 有效");
    CHECK(nearf(g.x_deg, expect_x, 1e-3f) && nearf(g.y_deg, expect_y, 1e-3f),
          "T9 小 roll 几何分解");
    g = bfs::mapNozzleGimbal(0.0f, small, 60.0f);
    CHECK(nearf(g.x_deg, -expect_x, 1e-3f) && nearf(g.y_deg, expect_y, 1e-3f),
          "T9 小 pitch 符号");
    g = bfs::mapNozzleGimbal(0.0f, 0.0f, 60.0f);
    CHECK(nearf(g.x_deg, 0.0f, 1e-6f) && nearf(g.y_deg, 0.0f, 1e-6f),
          "T9 零输入归零");
    g = bfs::mapNozzleGimbal(1.0f, 0.0f, 60.0f);
    CHECK(g.saturated && nearf(g.x_deg, 45.0f, 1e-3f) &&
              nearf(g.y_deg, 60.0f, 1e-3f),
          "T9 满 roll 限幅/饱和");
    g = bfs::mapNozzleGimbal(0.2f, 0.1f, 0.0f);
    CHECK(!g.valid, "T9 零限幅无效");
    g = bfs::mapNozzleGimbal(std::nanf(""), 0.0f, 30.0f);
    CHECK(!g.valid, "T9 NaN 无效");
  }

  if (g_fail == 0)
  {
    std::printf("=== test_liquid_rocket_model: 全部通过 ===\n");
    return 0;
  }
  std::printf("=== test_liquid_rocket_model: %d 项失败 ===\n", g_fail);
  return 1;
}
