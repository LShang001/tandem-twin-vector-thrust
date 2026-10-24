/**
 * @file    test_yaw_oscillation_detector.cpp
 * @module  固件/宿主回归测试（偏航振荡检测）
 * @brief   验证持续高偏航角速度触发、安静恢复、短脉冲不触发、复位与非有限输入
 * @details 被测：`include/YawOscillationDetector.h`。
 *          编译：g++ -std=c++17 -Iinclude test_host/test_yaw_oscillation_detector.cpp
 *                -o bin/yod
 * @theory  位置保持时航向指令≈0；持续 |r|>30°/s 说明航向环振荡/航向翻转。
 *          @ref VER-003 §3.14
 * @frame   NFRD 机体系 z 轴角速度
 * @units   deg/s、s
 * @note    默认阈值 30°/s、触发 1 s、恢复 3 s；未接入默认导航路径。
 * @test    Y1 持续高率触发；Y2 安静恢复；Y3 短脉冲不触发；Y4 复位；
 *          Y5 非有限输入保持；Y6 手动减速带不触发
 * @see     include/YawOscillationDetector.h
 */
#include <cmath>
#include <cstdio>

#include "YawOscillationDetector.h"

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

  // Y1 持续 80 deg/s：1 s 后触发。
  {
    bfs::YawOscillationDetector d;
    for (int i = 0; i < 250; ++i) (void)d.update(80.0f, dt);
    CHECK(d.oscillating(), "Y1 持续高率触发");
  }

  // Y2 触发后安静 3 s 恢复。
  {
    bfs::YawOscillationDetector d;
    for (int i = 0; i < 300; ++i) (void)d.update(80.0f, dt);
    for (int i = 0; i < 700; ++i) (void)d.update(2.0f, dt);
    CHECK(!d.oscillating(), "Y2 安静后恢复");
  }

  // Y3 0.5 s 脉冲不触发。
  {
    bfs::YawOscillationDetector d;
    for (int i = 0; i < 100; ++i) (void)d.update(80.0f, dt);
    CHECK(!d.oscillating(), "Y3 短脉冲不触发");
  }

  // Y4 复位清除状态。
  {
    bfs::YawOscillationDetector d;
    for (int i = 0; i < 300; ++i) (void)d.update(80.0f, dt);
    d.reset();
    CHECK(!d.oscillating(), "Y4 复位清除振荡");
  }

  // Y5 非有限输入保持当前状态。
  {
    bfs::YawOscillationDetector d;
    for (int i = 0; i < 300; ++i) (void)d.update(80.0f, dt);
    const bool before = d.oscillating();
    (void)d.update(std::nanf(""), dt);
    CHECK(d.oscillating() == before, "Y5 非有限输入保持");
  }

  // Y6 20 deg/s 持续不触发（正常转弯/侧风修正带）。
  {
    bfs::YawOscillationDetector d;
    for (int i = 0; i < 1000; ++i) (void)d.update(20.0f, dt);
    CHECK(!d.oscillating(), "Y6 20deg/s 不触发");
  }

  if (g_fail == 0)
  {
    std::printf("=== test_yaw_oscillation_detector: 全部通过 ===\n");
    return 0;
  }
  std::printf("=== test_yaw_oscillation_detector: %d 项失败 ===\n", g_fail);
  return 1;
}
