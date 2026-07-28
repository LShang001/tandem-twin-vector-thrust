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
