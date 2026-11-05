/**
 * @file    test_imu_sample_ring.cpp
 * @module  测试/IMU SPSC ring
 * @brief   ImuSampleRing 的顺序、回绕、空/满/溢出语义。
 * @details 输入：无物理量——样本 acc[0] 只写入递增序号作载荷标记；处理：
 *          push/pop 的 FIFO 顺序、可用容量 kCapacity−1、满时只计数不覆盖、
 *          3 轮连续生产/消费跨越数组末端、空读返回 false；输出：断言结果与
 *          exit code。判据：全部 assert 通过（含 drop_count 恰为 1）。
 * @frame   N/A（纯 SPSC 队列语义回归，样本字段只当不透明载荷，不做坐标变换）
 * @units   N/A（测试只比较序号，不解释 acc/gyr/timestamp 的物理单位）
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
