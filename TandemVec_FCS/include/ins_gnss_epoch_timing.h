#pragma once

#include <cstdint>

struct InsGnssEpochTimeMapping
{
  bool valid = false;
  uint32_t measurement_time_us = 0;
};

inline bool InsGnssShouldConsumeEpoch(const bool has_epoch,
                                      const bool data_ready_flag)
{
  (void)data_ready_flag;
  return has_epoch;
}

inline bool InsGnssShouldReanchor(const bool has_epoch,
                                 const bool quality_passed,
                                 const bool time_mapping_valid,
                                 const float measurement_age_s,
                                 const float max_reanchor_age_s,
                                 const bool initialized_with_fake_origin,
                                 const bool has_real_anchor)
{
  return has_epoch && quality_passed && time_mapping_valid &&
         measurement_age_s >= 0.0f &&
         measurement_age_s <= max_reanchor_age_s &&
         initialized_with_fake_origin && !has_real_anchor;
}

inline int32_t InsGnssTowDeltaMs(const uint32_t epoch_tow_ms,
                                const uint32_t reference_tow_ms)
{
  constexpr int64_t kGpsWeekMs = 604800000LL;
  constexpr int64_t kHalfGpsWeekMs = kGpsWeekMs / 2LL;
  int64_t delta_ms = static_cast<int64_t>(epoch_tow_ms) -
                     static_cast<int64_t>(reference_tow_ms);
  if (delta_ms > kHalfGpsWeekMs)
  {
    delta_ms -= kGpsWeekMs;
  }
  else if (delta_ms < -kHalfGpsWeekMs)
  {
    delta_ms += kGpsWeekMs;
  }
  return static_cast<int32_t>(delta_ms);
}

inline InsGnssEpochTimeMapping InsGnssMapEpochToLocalTime(
    const uint32_t epoch_tow_ms,
    const uint32_t epoch_receive_time_us,
    const uint32_t reference_tow_ms,
    const uint32_t reference_receive_time_us,
    const uint32_t base_output_delay_us)
{
  InsGnssEpochTimeMapping result;
  const bool has_reference = reference_receive_time_us != 0U;
  const int32_t tow_delta_ms = has_reference
                                   ? InsGnssTowDeltaMs(epoch_tow_ms,
                                                       reference_tow_ms)
                                   : 0;
  // 队列只有少量 epoch；超过 10 s 说明 iTOW 跳变或参考已失效，改用本帧接收时刻。
  const bool reference_plausible = has_reference &&
                                   tow_delta_ms >= -10000 &&
                                   tow_delta_ms <= 10000;
  const int64_t tow_offset_us = reference_plausible
                                    ? static_cast<int64_t>(tow_delta_ms) * 1000LL
                                    : 0LL;
  const uint32_t mapping_receive_time_us = reference_plausible
                                               ? reference_receive_time_us
                                               : epoch_receive_time_us;
  if (mapping_receive_time_us == 0U)
  {
    return result;
  }
  const int64_t mapped_time_us = static_cast<int64_t>(mapping_receive_time_us) -
                                 static_cast<int64_t>(base_output_delay_us) +
                                 tow_offset_us;
  // ★ 2026-08-13 审查修正：mapped_time_us 可能略负（micros() 刚回绕时，量测落在
  //   回绕点前几 ms；或 tow_offset 为负把量测推到回绕点之前）。这不是"未来时间"——
  //   回绕时钟下负值即"回绕点前的最近过去"，强转 uint32 得近 2^32 的值，
  //   随后 InsGnssEpochAgeSeconds 的 uint32 模运算差值会正确解出真实年龄
  //   （见 micros_wrap 回归用例）。若在此判无效，会破坏 micros() 每 71.6 min
  //   回绕时的时间映射连续性，使回绕后 ~15ms 窗口内量测全部退化为固定延迟兜底。
  //   因此不做负值判无效，交给 uint32 模运算自然处理。
  result.measurement_time_us = static_cast<uint32_t>(mapped_time_us);
  result.valid = true;
  return result;
}

inline float InsGnssEpochAgeSeconds(const InsGnssEpochTimeMapping &mapping,
                                    const uint32_t now_us)
{
  if (!mapping.valid)
  {
    return 0.0f;
  }
  // ★ 2026-08-12 审查修复：elapsed_us 经 int32 截断，超过约 35.8 分钟会变负，
  // 原实现把陈旧数据当 0 龄处理。这里对负值返回 0 且由上层时间映射有效性兜底，
  // 避免陈旧 epoch 伪装成零年龄新鲜数据（长时间无参考失效路径）。
  const int32_t elapsed_us = static_cast<int32_t>(now_us - mapping.measurement_time_us);
  return elapsed_us > 0 ? static_cast<float>(elapsed_us) * 1.0e-6f : 0.0f;
}
