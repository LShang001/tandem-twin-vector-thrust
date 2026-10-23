/**
 * @file    ins_gnss_pps_latency.h
 * @module  固件/导航（GNSS 链路延时标定）
 * @brief   由 PPS 整秒边沿与 PVT 到达时刻估计“量测时刻→本地接收”的链路延时
 * @details 输入：PPS 边沿（iTOW + 本地捕获 µs）、PVT 到达（iTOW + 本地接收 µs）；
 *          处理：为每个 PVT 找最近 PPS 边沿 → 由整秒 + 分数秒推期望本地时刻 →
 *          差值得单帧延时 → 排序取中位数与中位数绝对偏差；
 *          输出：延时估计、离散度、样本数。样本 <3 或输入为空时 valid=false。
 * @theory  固定传输延时 + 调度抖动模型：延时取中位数抗野值，MAD 作离散度诊断
 *          @ref REF-121 §2.2.1 p.33（随机量的一阶统计量用于量测噪声/延时建模）
 * @frame   N/A（时间标量）
 * @units   iTOW ms，本地时刻 µs，延时 s
 * @note    延时折叠到 [0,1 s) 以兼容本地时钟回绕；PPS 只给整秒对时、
 *          不给 UTC 绝对时间，本模块估计的是相对延时，够 EKF 算量测年龄。
 *          当前固件尚未接入硬件 PPS 捕获（见 TandemVec_FCS/AGENTS.md）。
 * @see     `ins_gnss_epoch_timing.h`、`ins_gnss_delay_auto.h`
 */

#pragma once

// ============================================================
//  GNSS PPS 延时标定（纯平台无关）
//
//  输入：
//   - PPS 边沿：GPS 整秒 tow + 本地定时器捕获时刻；
//   - PVT 到达：PVT 的 tow + 本地接收时刻。
//
//  输出：
//   - 从 PVT 量测时刻（tow）到本地接收的链路延时估计；
//   - 使用中位数/截尾均值，避免单帧调度抖动污染。
//
//  说明：PPS 只提供“GPS 整秒”的本地时刻，不提供 UTC 绝对时间；
//  这里估计的是 PPS 到 PVT 到达的延时，足够用于 EKF 量测年龄。
//  没有 PPS 时本函数无法给出绝对延时。
// ============================================================

#include <cstdint>
#include <cstddef>
#include <cmath>
#include <algorithm>
#include <vector>

struct InsGnssPpsEdge
{
  uint32_t tow_ms;    // GPS 整秒的 tow（通常 %1000 == 0）
  uint32_t local_us;  // 本地定时器 PPS 捕获时刻
};

struct InsGnssPvtArrival
{
  uint32_t tow_ms;    // PVT 的 tow
  uint32_t local_us;  // 本地接收/解析完成时刻
};

struct InsGnssPpsLatencyEstimate
{
  bool valid = false;
  float latency_s = 0.0f;    // PPS 对齐后的量测到本地接收延时
  float spread_s = 0.0f;     // 中位数绝对偏差（诊断）
  uint32_t sample_count = 0;
};

inline float InsGnssNormalizeLatencyUs(const int64_t delta_us)
{
  // 折叠到 [0, 1s)，兼容本地 micros 回绕/整秒边界。
  int64_t v = delta_us % 1000000LL;
  if (v < 0)
  {
    v += 1000000LL;
  }
  return static_cast<float>(v);
}

inline InsGnssPpsLatencyEstimate InsGnssEstimatePpsLatency(
    const InsGnssPpsEdge *edges, const size_t edge_count,
    const InsGnssPvtArrival *arrivals, const size_t arrival_count,
    const uint32_t max_abs_delta_ms = 1000U)
{
  InsGnssPpsLatencyEstimate out;
  if (edges == nullptr || arrivals == nullptr || edge_count == 0U ||
      arrival_count == 0U)
  {
    return out;
  }

  std::vector<float> samples;
  samples.reserve(arrival_count);
  for (size_t i = 0; i < arrival_count; ++i)
  {
    // 找同一 GPS 秒（或最近的 PPS 整秒）的边沿。
    const uint32_t arrival_sec = arrivals[i].tow_ms / 1000U;
    const InsGnssPpsEdge *best = nullptr;
    int32_t best_delta_ms = 0;
    for (size_t j = 0; j < edge_count; ++j)
    {
      const int32_t d = static_cast<int32_t>(arrivals[i].tow_ms) -
                        static_cast<int32_t>(edges[j].tow_ms);
      if (best == nullptr || std::abs(d) < std::abs(best_delta_ms))
      {
        best = &edges[j];
        best_delta_ms = d;
      }
    }
    if (best == nullptr || std::abs(best_delta_ms) > static_cast<int32_t>(max_abs_delta_ms))
    {
      continue;
    }
    (void)arrival_sec;
    // PVT 相对 PPS 的分数秒 + 本地接收差。
    const int64_t fractional_us =
        static_cast<int64_t>(arrivals[i].tow_ms % 1000U) * 1000LL;
    const int64_t expected_local_us =
        static_cast<int64_t>(best->local_us) + fractional_us;
    const int64_t delta_us =
        static_cast<int64_t>(arrivals[i].local_us) - expected_local_us;
    const float lat_s = InsGnssNormalizeLatencyUs(delta_us) * 1.0e-6f;
    samples.push_back(lat_s);
  }

  if (samples.size() < 3U)
  {
    return out;
  }

  std::sort(samples.begin(), samples.end());
  const size_t mid = samples.size() / 2U;
  const float median = (samples.size() % 2U == 0U)
                           ? 0.5f * (samples[mid - 1U] + samples[mid])
                           : samples[mid];
  std::vector<float> dev;
  dev.reserve(samples.size());
  for (float s : samples)
  {
    dev.push_back(std::fabs(s - median));
  }
  std::sort(dev.begin(), dev.end());
  const size_t dmid = dev.size() / 2U;
  const float mad = (dev.size() % 2U == 0U)
                        ? 0.5f * (dev[dmid - 1U] + dev[dmid])
                        : dev[dmid];

  out.valid = true;
  out.latency_s = median;
  out.spread_s = mad;
  out.sample_count = static_cast<uint32_t>(samples.size());
  return out;
}
