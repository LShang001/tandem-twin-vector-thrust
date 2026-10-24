/**
 * @file    test_adrc_controller.cpp
 * @module  固件/宿主回归测试（fw-tests：一阶 ADRC/ESO）
 * @brief   验证 ADRC 跟踪、常值扰动抑制、输出限幅、非有限输入与复位
 * @details 被测：`include/AdrcController.h`；用一阶积分 plant
 *          `dy/dt=b0·u+d` 闭环，检查稳态/扰动恢复。
 * @theory  一阶 ADRC/ESO；wo≈3–5×wc；z2 估计常值扰动并前馈补偿
 *          @ref CTL-001 §8.6；VER-003 §3.10
 * @frame   N/A
 * @units   s、无量纲
 * @note    默认参数不是飞控默认；仅验证模块数学行为。
 * @test    A1 阶跃跟踪；A2 常值扰动抑制；A3 限幅；A4 非有限；A5 复位
 * @see     include/AdrcController.h、simulations/three_vehicle_mc/fast_mc.py
 */
#include <cmath>
#include <cstdio>

#include "AdrcController.h"

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
  const float dt = 0.005f;
  bfs::AdrcParams p;
  p.wc = 16.0f; p.wo = 50.0f; p.b0 = 1.0f; p.dt = dt;
  p.out_min = -50.0f; p.out_max = 50.0f;

  // A1 阶跃跟踪：plant dy/dt=u，2 s 后接近 1。
  {
    bfs::FirstOrderAdrc a; a.setParams(p);
    float y = 0.0f, u = 0.0f;
    for (int i = 0; i < 400; ++i)
    {
      u = a.update(1.0f, y);
      y += u * dt;
    }
    CHECK(std::fabs(y - 1.0f) < 0.05f, "A1 阶跃跟踪");
    CHECK(std::fabs(a.z2()) < 0.5f, "A1 无扰动时 z2 接近 0");
  }

  // A2 常值扰动：1 s 后加 d=2，ADRC 应把稳态误差拉回小值。
  {
    bfs::FirstOrderAdrc a; a.setParams(p);
    float y = 0.0f, u = 0.0f;
    for (int i = 0; i < 400; ++i)
    {
      u = a.update(1.0f, y);
      const float d = (i * dt >= 1.0f) ? 2.0f : 0.0f;
      y += (u + d) * dt;
    }
    CHECK(std::fabs(y - 1.0f) < 0.15f, "A2 常值扰动抑制");
    CHECK(std::fabs(a.z2() - 2.0f) < 0.5f, "A2 z2 估计扰动");
  }

  // A3 输出限幅：大误差下 u 不超过配置限幅。
  {
    bfs::AdrcParams q = p; q.out_min = -0.2f; q.out_max = 0.2f;
    bfs::FirstOrderAdrc a; a.setParams(q);
    float y = 0.0f;
    for (int i = 0; i < 100; ++i) y += a.update(10.0f, y) * dt;
    CHECK(y <= 0.2f * 0.5f + 1e-3f, "A3 输出限幅");
  }

  // A4 非有限输入：返回 0 且不崩溃。
  {
    bfs::FirstOrderAdrc a; a.setParams(p);
    CHECK(a.update(std::nanf(""), 0.0f) == 0.0f, "A4 非有限 setpoint");
    CHECK(a.update(0.0f, std::nanf("")) == 0.0f, "A4 非有限 measurement");
  }

  // A5 复位：状态与输出历史清零。
  {
    bfs::FirstOrderAdrc a; a.setParams(p);
    float y = 0.0f;
    for (int i = 0; i < 200; ++i) y += a.update(1.0f, y) * dt;
    a.reset();
    CHECK(std::fabs(a.z1()) < 1e-6f && std::fabs(a.z2()) < 1e-6f, "A5 复位");
  }

  if (g_fail == 0)
  {
    std::printf("=== test_adrc_controller: 全部通过 ===\n");
    return 0;
  }
  std::printf("=== test_adrc_controller: %d 项失败 ===\n", g_fail);
  return 1;
}
