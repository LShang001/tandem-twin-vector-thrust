/**
 * @file    BaroSampleStatus.h
 * @module  固件/气压采集诊断
 * @brief   记录最近的气压读取结果与成功样本时间
 * @details 采集侧每次取数后更新，调试控制台查询距最近成功取数的时间。
 *          数据只供诊断，时间戳取 MCU 完成读取的时刻。
 * @theory  N/A
 * @frame   N/A
 * @units   时间 ms，读取状态为驱动返回码
 * @see     sensor_peripheral.cpp、test_host/test_ins_baro_health.cpp
 */
#pragma once

#include <cstdint>

namespace bfs
{

/** @brief 气压读取快照，显式区分尚未尝试、已有样本和最后读取结果。 */
struct BaroSampleStatus
{
  bool attempted = false;
  bool has_sample = false;
  int16_t read_status = 0;
  uint32_t samples = 0;
  uint32_t last_sample_ms = 0;

  /** @brief 读取由驱动报告成功时刷新最后到达时间和帧计数。 */
  void recordRead(const int16_t status, const bool received, const uint32_t now_ms)
  {
    attempted = true;
    read_status = status;
    if (received)
    {
      has_sample = true;
      ++samples;
      last_sample_ms = now_ms;
    }
  }

  /** @brief 成功取数后才有年龄，使用无符号差值跨越毫秒时钟回卷。 */
  uint32_t sampleAgeMs(const uint32_t now_ms) const
  {
    return now_ms - last_sample_ms;
  }
};

} // namespace bfs
