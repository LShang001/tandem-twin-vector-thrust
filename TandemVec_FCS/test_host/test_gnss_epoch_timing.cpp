#include "ins_gnss_epoch_timing.h"

#include <cassert>
#include <cmath>
#include <cstdint>
#include <iostream>

int main()
{
  // epoch 是否可消费只能由 FIFO 出队结果决定，不能再受一次性 ready 标志门控。
  assert(InsGnssShouldConsumeEpoch(true, false));
  assert(!InsGnssShouldConsumeEpoch(false, true));

  // 首次真实重锚必须绑定到当前已出队且通过质量检查的 epoch。
  constexpr float kMaxReanchorAgeS = 0.15f;
  assert(InsGnssShouldReanchor(true, true, true, 0.05f,
                               kMaxReanchorAgeS, true, false));
  assert(!InsGnssShouldReanchor(false, true, true, 0.05f,
                                kMaxReanchorAgeS, true, false));
  assert(!InsGnssShouldReanchor(true, false, true, 0.05f,
                                kMaxReanchorAgeS, true, false));
  assert(!InsGnssShouldReanchor(true, true, false, 0.05f,
                                kMaxReanchorAgeS, true, false));
  assert(!InsGnssShouldReanchor(true, true, true, 0.20f,
                                kMaxReanchorAgeS, true, false));
  assert(!InsGnssShouldReanchor(true, true, true, 0.05f,
                                kMaxReanchorAgeS, false, false));
  assert(!InsGnssShouldReanchor(true, true, true, 0.05f,
                                kMaxReanchorAgeS, true, true));

  constexpr uint32_t kWeekMs = 604800000U;
  constexpr uint32_t kBaseDelayUs = 15000U;

  // 最新 epoch 在本地 1.000 s 完整接收；队列中旧 200 ms 的 epoch 应映射到 0.785 s。
  const InsGnssEpochTimeMapping queued = InsGnssMapEpochToLocalTime(
      1000U, 800000U,
      1200U, 1000000U,
      kBaseDelayUs);
  assert(queued.valid);
  assert(queued.measurement_time_us == 785000U);
  assert(std::fabs(InsGnssEpochAgeSeconds(queued, 1100000U) - 0.315f) < 1.0e-6f);

  // GPS 周末到下周初仍应得到 -100 ms，而不是约 7 天的错误延迟。
  const InsGnssEpochTimeMapping wrapped = InsGnssMapEpochToLocalTime(
      kWeekMs - 50U, 1900000U,
      50U, 2000000U,
      kBaseDelayUs);
  assert(wrapped.valid);
  assert(wrapped.measurement_time_us == 1885000U);

  // 缺少参考 epoch 时退化为该 epoch 自身接收时刻，仍保留基础输出延迟。
  const InsGnssEpochTimeMapping fallback = InsGnssMapEpochToLocalTime(
      5000U, 3000000U,
      0U, 0U,
      kBaseDelayUs);
  assert(fallback.valid);
  assert(fallback.measurement_time_us == 2985000U);

  const InsGnssEpochTimeMapping discontinuity = InsGnssMapEpochToLocalTime(
      20000U, 4000000U,
      500000U, 4100000U,
      kBaseDelayUs);
  assert(discontinuity.valid);
  assert(discontinuity.measurement_time_us == 3985000U);

  // MCU micros() 约 71.6 分钟回绕一次，映射和年龄计算必须保持连续。
  const InsGnssEpochTimeMapping micros_wrap = InsGnssMapEpochToLocalTime(
      6000U, 10000U,
      6000U, 10000U,
      kBaseDelayUs);
  assert(micros_wrap.valid);
  assert(std::fabs(InsGnssEpochAgeSeconds(micros_wrap, 20000U) - 0.025f) < 1.0e-6f);

  std::cout << "GNSS epoch 消费与时间映射测试通过" << std::endl;
  return 0;
}
