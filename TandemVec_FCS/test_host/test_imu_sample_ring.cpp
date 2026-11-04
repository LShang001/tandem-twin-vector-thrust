/**
 * @file    test_imu_sample_ring.cpp
 * @module  测试/IMU SPSC ring
 * @brief   ImuSampleRing 的顺序、回绕、空/满/溢出语义。
 */
#include "ImuSampleRing.h"

#include <cassert>
#include <cstdio>

int main()
{
  ImuSampleRing ring;
  ImuRingSample s{};
  assert(!ring.pop(s));
  assert(ring.size() == 0U);

  for (int i = 0; i < static_cast<int>(ImuSampleRing::kCapacity) - 1; ++i)
  {
    s.acc[0] = static_cast<float>(i);
    assert(ring.push(s));
  }
  assert(ring.size() == ImuSampleRing::kCapacity - 1U);
  s.acc[0] = 999.0f;
  assert(!ring.push(s)); // 满：只计数，不覆盖
  assert(ring.drop_count() == 1U);

  for (int i = 0; i < static_cast<int>(ImuSampleRing::kCapacity) - 1; ++i)
  {
    assert(ring.pop(s));
    assert(s.acc[0] == static_cast<float>(i));
  }
  assert(!ring.pop(s));
  assert(ring.size() == 0U);

  // 回绕：连续生产/消费跨越数组末端
  for (int round = 0; round < 3; ++round)
  {
    for (int i = 0; i < 10; ++i)
    {
      s.acc[0] = static_cast<float>(round * 10 + i);
      assert(ring.push(s));
    }
    for (int i = 0; i < 10; ++i)
    {
      assert(ring.pop(s));
      assert(s.acc[0] == static_cast<float>(round * 10 + i));
    }
  }
  std::printf("=== test_imu_sample_ring: 全部通过 ===\n");
  return 0;
}
