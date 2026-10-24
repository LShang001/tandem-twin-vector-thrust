/**
 * @file    test_rate_phase_lead.cpp
 * @module  固件/宿主回归测试（速率相位超前）
 * @brief   验证稳态增益 1、阶跃超调、复位、非有限输入与参数
 * @details 被测：`include/RatePhaseLead.h`。
 *          编译：g++ -std=c++17 -Iinclude test_host/test_rate_phase_lead.cpp
 *                -o bin/rpl
 * @theory  一阶低通分离慢分量；K·高通提供相位超前，DC 不改变稳态。
 *          @ref VER-003 §3.11
 * @frame   N/A
 * @units   deg/s、s、无量纲
 * @note    默认 K=0.5、τ=0.02 s；本模块未接入默认飞控路径。
 * @test    P1 稳态直通；P2 阶跃初期 > 输入；P3 复位；P4 非有限输入
 * @see     include/RatePhaseLead.h
 */
#include <cmath>
#include <cstdio>

#include "RatePhaseLead.h"

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

  // P1 稳态：长时间恒定输入后输出等于输入。
  {
    bfs::RatePhaseLead p;
    float out = 0.0f;
    for (int i = 0; i < 2000; ++i) out = p.update(25.0f, dt);
    CHECK(std::fabs(out - 25.0f) < 1e-3f, "P1 稳态直通");
  }

  // P2 阶跃：初期高通分量为正，输出 > 输入；随后回落。
  {
    bfs::RatePhaseLead p;
    for (int i = 0; i < 100; ++i) (void)p.update(0.0f, dt);
    const float out = p.update(100.0f, dt);
    CHECK(out > 100.0f, "P2 阶跃初期超前");
  }

  // P3 复位：首拍直通，不保留旧状态。
  {
    bfs::RatePhaseLead p;
    for (int i = 0; i < 100; ++i) (void)p.update(50.0f, dt);
    p.reset();
    const float out = p.update(10.0f, dt);
    CHECK(std::fabs(out - 10.0f) < 1e-6f, "P3 复位首拍直通");
  }

  // P4 非有限输入：原样返回（调用方 PID 有独立保护）。
  {
    bfs::RatePhaseLead p;
    (void)p.update(1.0f, dt);
    const float out = p.update(std::nanf(""), dt);
    CHECK(std::isnan(out), "P4 非有限输入原样返回");
  }

  if (g_fail == 0)
  {
    std::printf("=== test_rate_phase_lead: 全部通过 ===\n");
    return 0;
  }
  std::printf("=== test_rate_phase_lead: %d 项失败 ===\n", g_fail);
  return 1;
}
