/**
 * @file    test_liquid_ccaegt_reference.cpp
 * @module  固件/宿主参考测试（fw-tests：液体火箭 CCAEGT 实飞制导 + 本机推进模型）
 * @brief   用外部实飞工程 `VTVL_LiquidRocket_FC` 的 CCAEGT 制导库驱动
 *          `LiquidRocketPropulsion`，验证 30 m 垂直回收剖面与触地速度
 * @details 被测：`test_host/reference/ccaegt/`（逐字复制、仅补充注释字段的外部参考库）
 *          + `include/LiquidRocketModel.h`（本仓库推进/质量/喷管模型）。
 *          本测试不把 CCAEGT 接入固件飞行模式，只做参考制导与推进模型的
 *          组合闭环；用于回答“实飞制导假设与本机模型能否闭合”。
 *          编译：g++ -std=c++17 -Iinclude -Itest_host/reference/ccaegt \
 *                test_host/test_liquid_ccaegt_reference.cpp \
 *                test_host/reference/ccaegt/CCAEGT.cpp -o bin/lrccaegt
 * @theory  CCAEGT：预测捕获高度 + 速度剖面 + 前馈/反馈推力；本机：
 *          T=u·Tmax、m_dot=T/(g0·Isp)、变质量
 *          @ref CTL-003 §3.3；外部 CCAEGT_Config.h
 * @frame   高度向上为正；NED 位置仅用于测试内部换算
 * @units   m、m/s、N、kg、s
 * @note    ★ CCAEGT 是外部实飞工程的参考实现，未做本仓库代码审查；
 *          参数 M0=16.5 kg/M_DRY=12.5 kg 与外部固件 DRY_MASS=13.5 kg 存在
 *          已知不一致。本测试只验证组合闭环趋势，不证明实机着陆安全。
 * @test    R1 点火/初始上升段；R2 进入制动段；R3 触地；R4 接地速度有界
 * @see     include/LiquidRocketModel.h、test_liquid_rocket_sil.cpp、VER-002
 */
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "CCAEGT.h"
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

int main(int argc, char **argv)
{
  double h0 = 30.0;
  double prop0 = 4.0; // 对齐 CCAEGT_M0=16.5 kg（干重 12.5）
  for (int i = 1; i + 1 < argc; i += 2)
  {
    if (std::strcmp(argv[i], "--h0") == 0) h0 = std::atof(argv[i + 1]);
    else if (std::strcmp(argv[i], "--prop") == 0) prop0 = std::atof(argv[i + 1]);
  }

  bfs::LiquidRocketParams params;
  params.dry_mass_kg = 12.5f;   // 对齐 CCAEGT_M_DRY
  params.initial_propellant_kg = static_cast<float>(prop0);
  params.max_thrust_n = 200.0f;
  params.min_thrust_n = 80.0f;
  params.isp_s = 200.0f;

  bfs::LiquidRocketPropulsion engine;
  CHECK(engine.init(params), "R1 推进模型初始化");
  CCAEGT_State_t state = {0};
  CCAEGT_Output_t out = {0};
  CHECK(CCAEGT_Init(&state) == 0, "R1 CCAEGT 初始化");
  CCAEGT_SetPhase(&state, CCAEGT_PHASE_ASCENT);
  CCAEGT_ResetIntegrator(&state);

  const double dt = 0.005;
  const double g = params.g0_mps2;
  double h = h0;
  double v_up = 0.0;
  bool landed = false;
  bool brake_seen = false;
  double landing_v = 0.0;

  for (int i = 0; i < 8000; ++i)
  {
    const double t = i * dt;
    state.h = static_cast<float>(h);
    state.v = static_cast<float>(v_up);
    state.m = engine.state().mass_kg;
    state.prop = engine.state().propellant_kg;
    state.T = engine.state().thrust_n;
    state.t = static_cast<float>(t);
    CCAEGT_Compute(&state, &out);
    const CCAEGT_Phase_t phase = CCAEGT_GetPhase(&state);
    brake_seen = brake_seen || (phase == CCAEGT_PHASE_BRAKE);

    // 对齐外部 flight_control 的执行轨：ASCENT/FREEFALL 用 Tmin 直通，
    // BRAKE 用 T_ff + 速度反馈，LANDED/CRASHED 关机。
    double throttle = 0.0;
    if (phase == CCAEGT_PHASE_ASCENT || phase == CCAEGT_PHASE_FREEFALL)
    {
      throttle = params.min_thrust_n / params.max_thrust_n;
    }
    else if (phase == CCAEGT_PHASE_BRAKE)
    {
      // 参考库自身已做前馈+反馈+限幅；直接用 T_cmd，避免叠加第二套反馈。
      throttle = out.T_cmd / params.max_thrust_n;
    }
    throttle = std::fmax(0.0, std::fmin(1.0, throttle));

    bfs::LiquidRocketCommand cmd;
    cmd.throttle = static_cast<float>(throttle);
    cmd.ignition_enabled = true;
    cmd.unlocked = true;
    cmd.dt_s = static_cast<float>(dt);
    const auto eng = engine.update(cmd);
    if (!eng.valid) { std::printf("FAIL: R1 模型无效\n"); g_fail++; break; }

    const double acc = eng.thrust_n / eng.mass_kg - g;
    v_up += acc * dt;
    h += v_up * dt;

    if (h <= 0.0)
    {
      h = 0.0;
      landing_v = -v_up; // NED down 正
      landed = true;
      break;
    }
  }

  std::printf("CCAEGT-SIL landed=%d h=%.2f v=%.2f phase=%d "
              "m=%.2f prop=%.2f brake=%d\n",
              landed ? 1 : 0, h, landing_v,
              static_cast<int>(CCAEGT_GetPhase(&state)),
              engine.state().mass_kg, engine.state().propellant_kg,
              brake_seen ? 1 : 0);

  CHECK(CCAEGT_GetPhase(&state) == CCAEGT_PHASE_ASCENT ||
            CCAEGT_GetPhase(&state) == CCAEGT_PHASE_FREEFALL ||
            CCAEGT_GetPhase(&state) == CCAEGT_PHASE_BRAKE ||
            CCAEGT_GetPhase(&state) == CCAEGT_PHASE_LANDED,
        "R1 相位有限");
  CHECK(engine.state().ignited, "R1 点火");
  CHECK(brake_seen, "R2 进入制动段");
  CHECK(landed, "R3 触地");
  CHECK(landing_v > 0.0 && landing_v < 5.0, "R4 接地速度有界");
  CHECK(engine.state().mass_kg > params.dry_mass_kg,
        "R4 推进剂未提前耗尽");

  if (g_fail == 0)
  {
    std::printf("=== test_liquid_ccaegt_reference: 全部通过 ===\n");
    return 0;
  }
  std::printf("=== test_liquid_ccaegt_reference: %d 项失败 ===\n", g_fail);
  return 1;
}
