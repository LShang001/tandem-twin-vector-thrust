/**
 * @file    ImuSampleRing.h
 * @module  传感器 / IMU 采样 ring
 * @brief   ISR 单生产者 / 主循环单消费者（SPSC）的 IMU 样本环形缓冲。
 * @details 用于 P1 采样架构：2 kHz 定时器/ISR 只把 FIFO 样本写入 ring，
 *          主循环消费并做校准/滤波/增量/EKF；ISR 不做浮点处理。
 * @theory  N/A（无锁环形缓冲）
 * @frame   N/A
 * @units   acc=g、gyr=dps、timestamp=芯片 16 位计数
 * @note    仅单生产者/单消费者；生产端在中断上下文，消费端在主循环。
 *          head/tail 用 volatile + 编译器屏障；溢出只计数、不覆盖未读样本。
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

struct ImuRingSample
{
  float acc[3] = {0.0f, 0.0f, 0.0f};
  float gyr[3] = {0.0f, 0.0f, 0.0f};
  uint16_t timestamp = 0;
};

class ImuSampleRing
{
public:
  static constexpr size_t kCapacity = 64U;

  /// 生产端（ISR）：写入一个样本；满时返回 false 并累加 drop_count。
  bool push(const ImuRingSample &sample)
  {
    const size_t head = head_;
    const size_t next = (head + 1U) % kCapacity;
    __sync_synchronize();
    if (next == tail_)
    {
      ++drop_count_;
      return false;
    }
    buffer_[head] = sample;
    __sync_synchronize();
    head_ = next;
    return true;
  }

  /// 消费端（主循环）：取出一个样本；空时返回 false。
  bool pop(ImuRingSample &sample)
  {
    const size_t tail = tail_;
    __sync_synchronize();
    if (tail == head_)
    {
      return false;
    }
    sample = buffer_[tail];
    __sync_synchronize();
    tail_ = (tail + 1U) % kCapacity;
    return true;
  }

  size_t size() const
  {
    const size_t head = head_;
    const size_t tail = tail_;
    return (head >= tail) ? (head - tail) : (kCapacity - tail + head);
  }

  uint32_t drop_count() const { return drop_count_; }
  void clear()
  {
    head_ = 0U;
    tail_ = 0U;
    drop_count_ = 0U;
  }

private:
  volatile size_t head_ = 0U; ///< 生产者写
  volatile size_t tail_ = 0U; ///< 消费者写
  volatile uint32_t drop_count_ = 0U;
  ImuRingSample buffer_[kCapacity];
};
