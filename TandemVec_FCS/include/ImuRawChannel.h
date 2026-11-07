/**
 * @file    ImuRawChannel.h
 * @module  传感器 / IMU 原始值诊断通道
 * @brief   保留最近若干个"未校准、未滤波"的 IMU 原始样本，供 DBG/标定对比。
 * @details 输入：readIMUData() 在步骤 2 取到的原始六轴（acc=g、gyr=dps）；
 *          处理：覆盖式环形留档（满时丢最旧，不阻塞采集路径）；
 *          输出：latest()/byAge() 取样本给 DBG `imuraw` 打印
 *          （原始值 vs 当前处理值并排对比）。
 * @theory  N/A（纯留档，不参与任何解算；与 ins_vibration_hf 的 raw−lp 高通
 *          同源，但那一路只输出 RMS、看不到原始波形本身）
 * @frame   保持传感器原始轴序（RUB），不做任何坐标变换——变换属于处理链，
 *          本通道只负责"未经处理的证据"
 * @units   acc 为 g，gyr 为 dps，时间戳为 micros()
 * @note    ★ 覆盖式而非计数式：诊断通道只保证"最近 N 拍一定在"，不做丢帧统计
 *          （丢帧统计由 ImuSampleRing::drop_count 与 DBG `tasks`/`health` 负责）。
 *          单生产者（readIMUData 主循环）/ 单消费者（DBG 命令同样在主循环），
 *          因此不加锁；若将来从 ISR 写入，必须改成 ImuSampleRing 那套 SPSC 语义。
 * @see     src/sensor_imu.cpp（接线）、src/communication.cpp（DBG imuraw）、
 *          AGENTS.md 踩坑记录（printf/日志改变浮点路径）
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

/// 一个原始 IMU 样本（传感器轴序 RUB，未校准、未滤波）
struct ImuRawSample
{
  float acc_g[3];    ///< 加速度 [g]（X/Y/Z 原始轴序）
  float gyr_dps[3];  ///< 角速度 [dps]（X/Y/Z 原始轴序）
  uint32_t stamp_us; ///< 入档时刻 micros()
};

class ImuRawChannel
{
public:
  /// 留档深度：64 拍 × 2 kHz ≈ 32 ms，足够 DBG 打印与短窗频谱对比
  static constexpr size_t kCapacity = 64U;

  /// 生产端（readIMUData）：写入一个样本；满时覆盖最旧。
  void push(const ImuRawSample &sample)
  {
    buffer_[head_] = sample;
    head_ = (head_ + 1U) % kCapacity;
    if (count_ < kCapacity)
    {
      ++count_;
    }
    ++total_;
  }

  /// 消费端：取最新样本；空时返回 false。
  bool latest(ImuRawSample &out) const
  {
    return byAge(0U, out);
  }

  /// 消费端：按年龄取样本，age=0 为最新，age 越大越旧；越界返回 false。
  bool byAge(size_t age, ImuRawSample &out) const
  {
    if (age >= count_)
    {
      return false;
    }
    // head_ 指向下一个写入位，最新样本在 head_ 的前一格（按环回绕）
    const size_t idx = (head_ + kCapacity - 1U - age) % kCapacity;
    out = buffer_[idx];
    return true;
  }

  /// 当前已留档样本数（≤ kCapacity）
  size_t count() const { return count_; }

  /// 累计入档样本数（可用来判断采集链是否还在走）
  uint32_t total() const { return total_; }

  void clear()
  {
    head_ = 0U;
    count_ = 0U;
    total_ = 0U;
  }

private:
  ImuRawSample buffer_[kCapacity] = {};
  size_t head_ = 0U;    ///< 下一个写入位
  size_t count_ = 0U;   ///< 已留档数（饱和于 kCapacity）
  uint32_t total_ = 0U; ///< 累计入档数
};
