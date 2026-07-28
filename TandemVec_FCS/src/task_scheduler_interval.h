#pragma once

#include <stdint.h>

namespace scheduler
{
constexpr uint32_t kSchedulerTickHz = 2000U;
constexpr uint32_t kTickQ8Scale = 256U;

// 2kHz 调度器的硬件 tick 为 0.5ms。DPS310 128Hz 压力输出对应 7.8125ms，
// 实测最佳任务周期为 7.735ms，无法用整数 tick 精确表达。
// 这里用 Q8 固定点保存目标 tick 数，通过误差累积在 15/16 tick 间抖动，
// 保持长期平均周期准确，同时不改变 TIM8 中断频率和其它整数周期任务。
inline uint32_t intervalMsToTicksQ8(float interval_ms)
{
  if (interval_ms <= 0.0f)
  {
    return kTickQ8Scale;
  }

  const float ticks_q8 = interval_ms * 2.0f * static_cast<float>(kTickQ8Scale);
  const uint32_t rounded = static_cast<uint32_t>(ticks_q8 + 0.5f);
  return rounded > 0U ? rounded : kTickQ8Scale;
}

inline uint32_t wholeTicksFromQ8(uint32_t ticks_q8)
{
  const uint32_t ticks = ticks_q8 / kTickQ8Scale;
  return ticks > 0U ? ticks : 1U;
}

inline uint16_t fractionalTicksFromQ8(uint32_t ticks_q8)
{
  return static_cast<uint16_t>(ticks_q8 % kTickQ8Scale);
}

inline uint32_t initialIntervalTicks(uint32_t ticks_q8)
{
  return wholeTicksFromQ8(ticks_q8);
}

inline uint16_t initialIntervalErrorQ8(uint32_t ticks_q8)
{
  return fractionalTicksFromQ8(ticks_q8);
}

inline uint32_t nextIntervalTicks(uint32_t ticks_q8, uint16_t &error_q8)
{
  uint32_t interval_ticks = wholeTicksFromQ8(ticks_q8);
  const uint16_t fraction_q8 = fractionalTicksFromQ8(ticks_q8);
  const uint16_t next_error_q8 = static_cast<uint16_t>(error_q8 + fraction_q8);

  if (next_error_q8 >= kTickQ8Scale)
  {
    interval_ticks++;
    error_q8 = static_cast<uint16_t>(next_error_q8 - kTickQ8Scale);
  }
  else
  {
    error_q8 = next_error_q8;
  }

  return interval_ticks;
}
} // namespace scheduler
