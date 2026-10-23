/**
 * @file    test_gnss_epoch_timing.cpp
 * @module  固件/宿主回归测试（fw-tests：GNSS epoch 消费与时标映射）
 * @brief   验证 epoch 出队门控、重锚条件、iTOW→本地时间映射与 micros 回绕
 * @details 被测：`include/ins_gnss_epoch_timing.h` 的 `InsGnssShouldConsumeEpoch()`、
 *          `InsGnssShouldReanchor()`、`InsGnssMapEpochToLocalTime()`、
 *          `InsGnssEpochAgeSeconds()`。
 *          输入：手工构造的 (队列有样本, ready 标志, 质量通过, 年龄, 上限) 组合，
 *          以及 iTOW/本地微秒对、基础输出延迟；
 *          处理：逐条 require 断言布尔门控与映射后的测量时刻；
 *          输出：任一断言失败即 exit(1)。
 *          运行：g++ -std=c++17 -Iinclude test_host/test_gnss_epoch_timing.cpp -o bin/get
 * @theory  GNSS 量测时标：PVT 的 iTOW 是整秒对齐的 GPS 时，链路与输出延迟需由
 *          “最近完整 epoch 的接收时刻 + 基础延迟”反推量测时刻；年龄即当前时刻与之差，
 *          供 EKF 回放融合使用。@ref REF-121 §2.2.1 p.33（量测更新的时标假设）
 * @frame   N/A（纯时间轴）
 * @units   iTOW 与本地时间 µs，年龄 s，容差 1e-6（float 比较）
 * @note    不用 std::assert：NDEBUG 下会被编译掉、断言静默失效，故用 require 显式退出。
 *          星期回绕与 micros 回绕是历史事故点（错误延迟约 7 天 / 71.6 分钟）。
 * @test    T1 消费门控只看出队结果；T2 重锚四个前置条件与年龄上限；
 *          T3 队列旧 epoch 映射；T4 GPS 周回绕；T5 缺参考 epoch 回退；
 *          T6 时间不连续保护；T7 micros 回绕保持连续
 * @see     TandemVec_FCS/include/ins_gnss_epoch_timing.h、src/navigation_task.cpp
 */
#include "ins_gnss_epoch_timing.h"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

// 统一 check 风格（与 test_altitude_reference.cpp 一致）；
// 不用 std::assert —— NDEBUG 下会被编译掉，断言静默失效（脆弱点修复）。
namespace
{
void require(bool condition, const char *message = "assertion failed")
{
  if (!condition)
  {
    std::fprintf(stderr, "FAILED: %s\n", message);
    std::exit(1);
  }
}
} // namespace

int main()
{
  // epoch 是否可消费只能由 FIFO 出队结果决定，不能再受一次性 ready 标志门控。
  require(InsGnssShouldConsumeEpoch(true, false));
  require(!InsGnssShouldConsumeEpoch(false, true));

  // 首次真实重锚必须绑定到当前已出队且通过质量检查的 epoch。
  constexpr float kMaxReanchorAgeS = 0.15f;
  require(InsGnssShouldReanchor(true, true, true, 0.05f,
                               kMaxReanchorAgeS, true, false));
  require(!InsGnssShouldReanchor(false, true, true, 0.05f,
                                kMaxReanchorAgeS, true, false));
  require(!InsGnssShouldReanchor(true, false, true, 0.05f,
                                kMaxReanchorAgeS, true, false));
  require(!InsGnssShouldReanchor(true, true, false, 0.05f,
                                kMaxReanchorAgeS, true, false));
  require(!InsGnssShouldReanchor(true, true, true, 0.20f,
                                kMaxReanchorAgeS, true, false));
  require(!InsGnssShouldReanchor(true, true, true, 0.05f,
                                kMaxReanchorAgeS, false, false));
  require(!InsGnssShouldReanchor(true, true, true, 0.05f,
                                kMaxReanchorAgeS, true, true));

  constexpr uint32_t kWeekMs = 604800000U;
  constexpr uint32_t kBaseDelayUs = 15000U;

  // 最新 epoch 在本地 1.000 s 完整接收；队列中旧 200 ms 的 epoch 应映射到 0.785 s。
  const InsGnssEpochTimeMapping queued = InsGnssMapEpochToLocalTime(
      1000U, 800000U,
      1200U, 1000000U,
      kBaseDelayUs);
  require(queued.valid);
  require(queued.measurement_time_us == 785000U);
  require(std::fabs(InsGnssEpochAgeSeconds(queued, 1100000U) - 0.315f) < 1.0e-6f);

  // GPS 周末到下周初仍应得到 -100 ms，而不是约 7 天的错误延迟。
  const InsGnssEpochTimeMapping wrapped = InsGnssMapEpochToLocalTime(
      kWeekMs - 50U, 1900000U,
      50U, 2000000U,
      kBaseDelayUs);
  require(wrapped.valid);
  require(wrapped.measurement_time_us == 1885000U);

  // 缺少参考 epoch 时退化为该 epoch 自身接收时刻，仍保留基础输出延迟。
  const InsGnssEpochTimeMapping fallback = InsGnssMapEpochToLocalTime(
      5000U, 3000000U,
      0U, 0U,
      kBaseDelayUs);
  require(fallback.valid);
  require(fallback.measurement_time_us == 2985000U);

  const InsGnssEpochTimeMapping discontinuity = InsGnssMapEpochToLocalTime(
      20000U, 4000000U,
      500000U, 4100000U,
      kBaseDelayUs);
  require(discontinuity.valid);
  require(discontinuity.measurement_time_us == 3985000U);

  // MCU micros() 约 71.6 分钟回绕一次，映射和年龄计算必须保持连续。
  const InsGnssEpochTimeMapping micros_wrap = InsGnssMapEpochToLocalTime(
      6000U, 10000U,
      6000U, 10000U,
      kBaseDelayUs);
  require(micros_wrap.valid);
  require(std::fabs(InsGnssEpochAgeSeconds(micros_wrap, 20000U) - 0.025f) < 1.0e-6f);

  std::puts("GNSS epoch 消费与时间映射测试通过");
  return 0;
}
