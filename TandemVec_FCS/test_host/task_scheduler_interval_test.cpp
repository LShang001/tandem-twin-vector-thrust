/**
 * @file    task_scheduler_interval_test.cpp
 * @module  固件/宿主回归测试（fw-tests：调度器小数 tick 间隔）
 * @brief   验证 0.5 ms 分辨率的任务间隔换算、抖动分布与下限钳位
 * @details 被测：`src/task_scheduler_interval.h` 的
 *          `intervalMsToTicksQ8()`、`initialIntervalErrorQ8()`、
 *          `initialIntervalTicks()`、`nextIntervalTicks()`。
 *          输入：整数毫秒（7.0/10.0）、半毫秒（7.5）、亚 tick（0.2）间隔；
 *          处理：连续生成 8 个 tick 间隔；
 *          输出：`require()` 失败即 exit(1)。
 *          运行：g++ -std=c++17 -Iinclude test_host/task_scheduler_interval_test.cpp -o bin/tsi
 * @theory  小数分频（Bresenham 式误差累加）：把平均间隔抖动分摊到相邻 tick，
 *          保证长期平均等于请求值，同时单个间隔偏差 ≤1 tick。
 *          见 @ref AGENTS.md（`BFS_TASK_FRACTIONAL_INTERVALS` 相关约定）
 * @frame   N/A（调度时间轴）
 * @units   间隔 ms、tick 数（Q8 定点：1 tick = 256 Q8 单位）
 * @note    亚 tick 请求被钳到 1 tick 是防忙等的硬保护：小于调度器分辨率的周期
 *          会让任务每拍都触发、CPU 占用失控。容差按 tick 整数比较，不涉及浮点误差。
 * @test    T1 整半毫秒（7.5）不产生抖动；T2 小数间隔抖动分布；
 *          T3 长期平均等于请求值；T4 亚 tick 正间隔钳到 1 tick
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

void keeps_exact_half_millisecond_intervals_unchanged()
{
  const std::vector<uint32_t> intervals = buildIntervals(7.5f, 8);
  for (uint32_t interval : intervals)
  {
    require(interval == 15U, "7.5ms remains 15 ticks");
  }
}

void dithers_fractional_tick_intervals()
{
  const std::vector<uint32_t> intervals = buildIntervals(7.75f, 8);
  const uint32_t expected[] = {15U, 16U, 15U, 16U, 15U, 16U, 15U, 16U};
  for (int i = 0; i < 8; ++i)
  {
    require(intervals[i] == expected[i], "7.75ms alternates 15 and 16 ticks");
  }
}

void preserves_requested_average_interval()
{
  const std::vector<uint32_t> intervals = buildIntervals(7.75f, 64);
  require(std::fabs(average(intervals) - 15.5) < 0.001, "7.75ms average is 15.5 ticks");
}

void clamps_positive_sub_tick_intervals_to_one_tick()
{
  const uint32_t ticks_q8 = scheduler::intervalMsToTicksQ8(0.1f);
  require(scheduler::initialIntervalTicks(ticks_q8) == 1U, "sub-tick interval clamps to one tick");
}
} // namespace

int main()
{
  keeps_exact_half_millisecond_intervals_unchanged();
  dithers_fractional_tick_intervals();
  preserves_requested_average_interval();
  clamps_positive_sub_tick_intervals_to_one_tick();
  std::puts("task_scheduler_interval_test passed");
  return 0;
}
