#ifndef DPS310_FIFO_POLICY_H_INCLUDED
#define DPS310_FIFO_POLICY_H_INCLUDED

#include <stdint.h>

namespace dps310
{

namespace fifo_policy
{
#ifdef DPS__FIFO_SIZE
constexpr uint8_t kFifoSize = DPS__FIFO_SIZE;
#else
constexpr uint8_t kFifoSize = 32;
#endif

#ifdef DPS__SUCCEEDED
constexpr int16_t kSucceeded = DPS__SUCCEEDED;
#else
constexpr int16_t kSucceeded = 0;
#endif

#ifdef DPS__FAIL_UNKNOWN
constexpr int16_t kFailUnknown = DPS__FAIL_UNKNOWN;
#else
constexpr int16_t kFailUnknown = -1;
#endif

#ifdef DPS__FAIL_UNFINISHED
constexpr int16_t kFailUnfinished = DPS__FAIL_UNFINISHED;
#else
constexpr int16_t kFailUnfinished = -4;
#endif

#ifdef DPS__FAIL_OVERFLOW
constexpr int16_t kFailOverflow = DPS__FAIL_OVERFLOW;
#else
constexpr int16_t kFailOverflow = -5;
#endif
} // namespace fifo_policy

struct LatestFifoRawResult
{
  int32_t raw_temp = 0;
  int32_t raw_pressure = 0;
  bool temp_available = false;
  bool pressure_available = false;
};

struct FifoStatus
{
  bool empty = true;
  bool full = false;
  bool has_sample = false;
};

// DPS310 FIFO status 寄存器一次读出 empty/full 两个状态，避免旧实现连续
// 读 bitfield 时产生额外 I2C 事务。对 128Hz 压力输出，少一次 I2C 访问就能
// 明显降低 handleDPS310 的平均耗时和 FIFO 溢出概率。
inline FifoStatus decodeDps310FifoStatus(uint8_t fifo_status_register)
{
  FifoStatus status;
  status.empty = (fifo_status_register & 0x01U) != 0U;
  status.full = (fifo_status_register & 0x02U) != 0U;
  status.has_sample = !status.empty;
  return status;
}

inline int16_t pressureUpdateStatus(const LatestFifoRawResult &result)
{
  return result.pressure_available ? fifo_policy::kSucceeded : fifo_policy::kFailUnfinished;
}

// 正式固件采用“压力优先”的有限补读：常规只拿到一个压力样本就停止，
// 若第一个样本是低频温度样本，则允许再读一次追回最新压力。
// 这样在 7.735ms 周期下兼顾新鲜度和 I2C 占用；不要把它改回无限排空 FIFO，
// 否则偶发积压会把 200Hz 主循环阻塞到不可预测。
inline bool shouldStopFifoReadAfterSample(const LatestFifoRawResult &result, bool stop_after_pressure)
{
  return stop_after_pressure && result.pressure_available;
}

template <typename IsFifoEmpty, typename ReadFifoValue>
int16_t readLatestRawFromFifo(IsFifoEmpty is_fifo_empty,
                              ReadFifoValue read_fifo_value,
                              LatestFifoRawResult &result,
                              uint8_t max_reads = fifo_policy::kFifoSize)
{
  result = LatestFifoRawResult{};

  uint8_t reads = 0;
  while (!is_fifo_empty() && reads < max_reads)
  {
    int32_t raw_result = 0;
    const int16_t type = read_fifo_value(raw_result);
    if (type < 0)
    {
      return fifo_policy::kFailUnknown;
    }

    if (type == 0)
    {
      result.raw_temp = raw_result;
      result.temp_available = true;
    }
    else if (type == 1)
    {
      result.raw_pressure = raw_result;
      result.pressure_available = true;
    }

    reads++;
  }

  if (!result.temp_available && !result.pressure_available)
  {
    return fifo_policy::kFailUnfinished;
  }

  return fifo_policy::kSucceeded;
}

inline bool shouldFlushFifoAfterReadStatus(int16_t status)
{
  return status == fifo_policy::kFailUnknown || status == fifo_policy::kFailOverflow;
}

} // namespace dps310

#endif // DPS310_FIFO_POLICY_H_INCLUDED
