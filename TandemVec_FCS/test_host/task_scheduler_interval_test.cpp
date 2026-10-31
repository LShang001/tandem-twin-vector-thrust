/**
 * @file    task_scheduler_interval_test.cpp
 * @module  固件/宿主回归测试（fw-tests：调度器小数 tick 间隔）
 * @brief   验证任务间隔换算、抖动分布与下限钳位（期望值按 kSchedulerTickHz 推导）
 * @details 被测：`src/task_scheduler_interval.h` 的
 *          `intervalMsToTicksQ8()`、`initialIntervalErrorQ8()`、
 *          `initialIntervalTicks()`、`nextIntervalTicks()`。
 *          输入：整数毫秒、半毫秒、亚 tick 间隔；
 *          处理：连续生成 8/64 个 tick 间隔；
 *          输出：`require()` 失败即 exit(1)。
 *          运行：g++ -std=c++17 -Iinclude test_host/task_scheduler_interval_test.cpp -o bin/tsi
 * @theory  小数分频（Bresenham 式误差累加）：把平均间隔抖动分摊到相邻 tick，
 *          保证长期平均等于请求值，同时单个间隔偏差 ≤1 tick。
 *          @ref AGENTS.md（`BFS_TASK_FRACTIONAL_INTERVALS` 相关约定）
 * @frame   N/A（调度时间轴）
 * @units   间隔 ms、tick 数（Q8 定点：1 tick = 256 Q8 单位）
 * @note    ★2026-11-01 时基由 2000Hz 提升到 6400Hz（为 ICM-45686 3.2kHz 采样）。
 *          本测试不再硬编码 15/16 tick，而是由 kSchedulerTickHz 推导期望值，
 *          以后调时基不会误报回归。
 * @test    T1 7.5ms 整除时不抖动；T2 7.75ms 抖动只落在相邻 tick；
 *          T3 长期平均等于请求值；T4 亚 tick 钳到 1 tick
 * @see     TandemVec_FCS/src/task_scheduler_interval.h
 */
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "../src/task_scheduler_interval.h"

namespace
{
void require(bool condition, const char *message)
{
  if (!condition)
  {
    std::fprintf(stderr, "FAILED: %s\n", message);
    std::exit(1);
  }
}

/// 毫秒 → 期望 tick 数（浮点，用于推导期望）。
double ticksFor(double interval_ms)
{
  return interval_ms * (static_cast<double>(scheduler::kSchedulerTickHz) / 1000.0);
}

std::vector<uint32_t> buildIntervals(float interval_ms, int count)
{
  const uint32_t ticks_q8 = scheduler::intervalMsToTicksQ8(interval_ms);
  uint16_t error_q8 = scheduler::initialIntervalErrorQ8(ticks_q8);
  std::vector<uint32_t> intervals;
  intervals.reserve(count);
  intervals.push_back(scheduler::initialIntervalTicks(ticks_q8));
  for (int i = 1; i < count; ++i)
  {
    intervals.push_back(scheduler::nextIntervalTicks(ticks_q8, error_q8));
  }
  return intervals;
}

double average(const std::vector<uint32_t> &values)
{
  uint32_t sum = 0;
  for (uint32_t value : values)
  {
    sum += value;
  }
  return static_cast<double>(sum) / static_cast<double>(values.size());
}

// T1：7.5ms 在当前时基下是整数 tick（6400Hz → 48），不应抖动。
void keeps_exact_intervals_unchanged()
{
  const double expected = ticksFor(7.5);
  require(std::fabs(expected - std::round(expected)) < 1e-6,
          "test precondition: 7.5ms is integer ticks at current timebase");
  const std::vector<uint32_t> intervals = buildIntervals(7.5f, 8);
  for (uint32_t interval : intervals)
  {
    require(interval == static_cast<uint32_t>(std::llround(expected)),
            "7.5ms remains constant ticks");
  }
}

// T2：7.75ms 非整数 tick，抖动只能落在 floor/ceil 两档。
void dithers_fractional_tick_intervals()
{
  const double expected = ticksFor(7.75);
  const uint32_t lo = static_cast<uint32_t>(std::floor(expected));
  const uint32_t hi = static_cast<uint32_t>(std::ceil(expected));
  require(lo != hi, "test precondition: 7.75ms is fractional at current timebase");
  const std::vector<uint32_t> intervals = buildIntervals(7.75f, 8);
  for (uint32_t interval : intervals)
  {
    require(interval == lo || interval == hi, "7.75ms dithers between adjacent ticks");
  }
}

// T3：长期平均必须等于请求值（误差 < 0.5 tick）。
void preserves_requested_average_interval()
{
  const std::vector<uint32_t> intervals = buildIntervals(7.75f, 64);
  const double expected = ticksFor(7.75);
  require(std::fabs(average(intervals) - expected) < 0.5,
          "7.75ms average matches requested interval");
}

// T4：亚 tick 正间隔被钳到 1 tick（防忙等）。
void clamps_positive_sub_tick_intervals_to_one_tick()
{
  const uint32_t ticks_q8 = scheduler::intervalMsToTicksQ8(0.01f);
  require(scheduler::initialIntervalTicks(ticks_q8) == 1U,
          "sub-tick interval clamps to one tick");
}
} // namespace

int main()
{
  keeps_exact_intervals_unchanged();
  dithers_fractional_tick_intervals();
  preserves_requested_average_interval();
  clamps_positive_sub_tick_intervals_to_one_tick();
  std::puts("task_scheduler_interval_test passed");
  return 0;
}
