/**
 * @file    task_scheduler_interval.h
 * @module  固件/基础设施（任务调度器：分数 tick 间隔计算，纯平台无关）
 * @brief   用 Q8 定点数表达分数 tick 的任务周期，并在相邻周期上抖动取整
 * @details 输入：任务间隔（ms，浮点）；处理：ms→tick×256 的四舍五入 → 整数部分为
 *          tick 数、小数部分进误差累加器；每次触发时把误差累加，满 1 tick 就进位，
 *          使长期平均周期精确等于目标值；输出：本次应等待的 tick 数。
 *          不变量：只改“本任务的下次间隔”，不改 TIM8 中断频率，也不影响整数周期任务。
 * @theory  N/A（定点调度算法，非物理模型）
 * @ref     src/task_scheduler.cpp `timerCallback()` 的 `BFS_TASK_FRACTIONAL_INTERVALS` 分支；
 *          src/main.cpp `BFS_DPS310_TASK_INTERVAL_MS`（7.735ms 的取值来源）
 * @frame   N/A
 * @units   间隔 ms、tick 数（1 tick = 0.5ms @2kHz）、Q8 = 1/256 tick
 * @note    典型用途：DPS310 128Hz 压力输出对应 7.8125ms，实测最佳任务周期 7.735ms，
 *          无法用整数 tick 表达（15.47 tick），故在 15/16 tick 间抖动。
 * @test    test_host/task_scheduler_interval_test.cpp（长期平均与误差界）
 */
#pragma once

#include <stdint.h>

namespace scheduler
{
constexpr uint32_t kSchedulerTickHz = 2000U;  ///< TIM8 时基 2kHz（1 tick = 0.5ms）
constexpr uint32_t kTickQ8Scale = 256U;       ///< Q8 定点比例：1 tick = 256 个 1/256 单位

// 2kHz 调度器的硬件 tick 为 0.5ms。DPS310 128Hz 压力输出对应 7.8125ms，
// 实测最佳任务周期为 7.735ms，无法用整数 tick 精确表达。
// 这里用 Q8 固定点保存目标 tick 数，通过误差累积在 15/16 tick 间抖动，
// 保持长期平均周期准确，同时不改变 TIM8 中断频率和其它整数周期任务。
/**
 * @brief 任务间隔 ms → Q8 tick 数（四舍五入）；非正值按 1 tick 处理
 * @param[in] interval_ms 任务间隔 [ms]
 * @return Q8 定点 tick 数（1 tick = 0.5ms，1 tick = 256 Q8）
 * @formula ticks_q8 = round(interval_ms × 2 × 256)
 * @note 下限钳到 1 tick，避免 0/负间隔导致 ISR 内每拍都置位（CPU 打满）。
 */
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

/// 取 Q8 的整数 tick 部分（下限 1 tick）
inline uint32_t wholeTicksFromQ8(uint32_t ticks_q8)
{
  const uint32_t ticks = ticks_q8 / kTickQ8Scale;
  return ticks > 0U ? ticks : 1U;
}

/// 取 Q8 的小数部分（单位 1/256 tick），作为首次执行的误差种子
inline uint16_t fractionalTicksFromQ8(uint32_t ticks_q8)
{
  return static_cast<uint16_t>(ticks_q8 % kTickQ8Scale);
}

/// 注册任务时的首个间隔 tick 数（整数部分，不加误差校正）
inline uint32_t initialIntervalTicks(uint32_t ticks_q8)
{
  return wholeTicksFromQ8(ticks_q8);
}

/// 注册任务时的误差累加器初值（小数部分）
inline uint16_t initialIntervalErrorQ8(uint32_t ticks_q8)
{
  return fractionalTicksFromQ8(ticks_q8);
}

/**
 * @brief 计算下一次触发间隔，并在 N/N+1 tick 间抖动以保持长期平均周期
 * @param[in]     ticks_q8 目标周期（Q8 tick）
 * @param[in,out] error_q8 小数误差累加器（ISR 内每任务一份，初值见 initialIntervalErrorQ8）
 * @return 本次应等待的整数 tick 数（N 或 N+1）
 * @formula error += frac; if error ≥ 256 { ticks++; error −= 256 }
 * @pre 只在 TIM8 ISR 内调用（`timerCallback()`），单核无并发
 * @note 抖动上界 = 1 tick = 0.5ms；长期平均周期误差 < 1/256 tick，不改变 TIM8 频率。
 */
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
