/**
 * @file    test_imu_raw_channel.cpp
 * @module  固件/宿主测试（fw-tests：IMU 原始值诊断通道）
 * @brief   验证 ImuRawChannel 的留档、按年龄取样本、覆盖与清零语义
 * @details 被测：`include/ImuRawChannel.h`（平台无关，纯头文件）。
 *          输入：手工构造的 ImuRawSample 序列（用 stamp_us 作为唯一标识）；
 *          处理：按不同 push 次数调用 latest()/byAge()/count()/total()；
 *          输出：`require()` 失败即 exit(1)，stderr 打印失败原因。
 *          运行：g++ -std=c++17 -Iinclude test_host/test_imu_raw_channel.cpp -o bin/imraw
 * @theory  N/A（诊断通道的环形缓冲语义，不涉及控制/导航公式）
 * @frame   N/A（通道只做留档，不做坐标变换）
 * @units   acc=g、gyr=dps、stamp_us=µs（测试里只用序号当标识）
 * @note    本测试证明的是**缓冲语义**，不能证明 readIMUData 真的在正确位置入档；
 *          后者由固件编译与上板 DBG `imuraw` 观察覆盖。
 * @test    T1 空通道 latest/byAge 均失败；T2 单样本 latest==byAge(0)；
 *          T3 按年龄逆序取样本（age 越大越旧）；T4 越界年龄返回 false；
 *          T5 满容量后覆盖最旧且 count 饱和、total 继续增长；
 *          T6 clear() 重置 count/total 且 latest 失败
 * @see     TandemVec_FCS/include/ImuRawChannel.h、src/sensor_imu.cpp（接线）
 */
#include <cstdint>
#include <cstdio>
#include <cstdlib>

#include "ImuRawChannel.h"

namespace
{

void require(bool cond, const char *what)
{
  if (!cond)
  {
    std::fprintf(stderr, "[FAIL] %s\n", what);
    std::exit(1);
  }
}

/// 构造一个用 stamp_us 当唯一标识的样本
ImuRawSample make(uint32_t id)
{
  ImuRawSample s{};
  s.acc_g[0] = static_cast<float>(id) + 0.1f;
  s.acc_g[1] = static_cast<float>(id) + 0.2f;
  s.acc_g[2] = static_cast<float>(id) + 0.3f;
  s.gyr_dps[0] = static_cast<float>(id) + 0.4f;
  s.gyr_dps[1] = static_cast<float>(id) + 0.5f;
  s.gyr_dps[2] = static_cast<float>(id) + 0.6f;
  s.stamp_us = id;
  return s;
}

} // namespace

int main()
{
  constexpr size_t kCap = ImuRawChannel::kCapacity;

  // T1 空通道：latest / byAge 都必须失败，count/total 为 0
  {
    ImuRawChannel ch;
    ImuRawSample out{};
    require(!ch.latest(out), "T1 空通道 latest 应返回 false");
    require(!ch.byAge(0U, out), "T1 空通道 byAge(0) 应返回 false");
    require(ch.count() == 0U, "T1 空通道 count 应为 0");
    require(ch.total() == 0U, "T1 空通道 total 应为 0");
  }

  // T2 单样本：latest 与 byAge(0) 取到同一个
  {
    ImuRawChannel ch;
    ch.push(make(11U));
    ImuRawSample a{};
    ImuRawSample b{};
    require(ch.latest(a), "T2 单样本 latest 应成功");
    require(ch.byAge(0U, b), "T2 单样本 byAge(0) 应成功");
    require(a.stamp_us == 11U && b.stamp_us == 11U, "T2 latest 与 byAge(0) 应为同一最新样本");
    require(ch.count() == 1U && ch.total() == 1U, "T2 count/total 应为 1");
  }

  // T3 按年龄逆序：age 越大越旧
  {
    ImuRawChannel ch;
    ch.push(make(1U));
    ch.push(make(2U));
    ch.push(make(3U));
    ImuRawSample s{};
    require(ch.byAge(0U, s) && s.stamp_us == 3U, "T3 age=0 应为最新(3)");
    require(ch.byAge(1U, s) && s.stamp_us == 2U, "T3 age=1 应为 2");
    require(ch.byAge(2U, s) && s.stamp_us == 1U, "T3 age=2 应为最旧(1)");
  }

  // T4 越界年龄：age == count 起必须失败
  {
    ImuRawChannel ch;
    ch.push(make(7U));
    ch.push(make(8U));
    ImuRawSample s{};
    require(!ch.byAge(2U, s), "T4 age==count 应返回 false");
    require(!ch.byAge(99U, s), "T4 远越界应返回 false");
  }

  // T5 满容量覆盖：count 饱和于 kCapacity，total 继续增长，最旧被覆盖
  {
    ImuRawChannel ch;
    for (uint32_t i = 0; i < kCap + 10U; ++i)
    {
      ch.push(make(i));
    }
    require(ch.count() == kCap, "T5 count 应饱和于 kCapacity");
    require(ch.total() == static_cast<uint32_t>(kCap) + 10U, "T5 total 应继续累计");
    ImuRawSample s{};
    require(ch.byAge(0U, s) && s.stamp_us == static_cast<uint32_t>(kCap) + 9U,
            "T5 最新样本应为最后一次 push");
    // 最旧可见样本 = total-1-(kCap-1) = 10
    require(ch.byAge(kCap - 1U, s) && s.stamp_us == 10U,
            "T5 最旧可见样本应为第 10 个（前 10 个已被覆盖）");
    require(!ch.byAge(kCap, s), "T5 age==kCapacity 应越界失败");
  }

  // T6 clear：计数与内容一并重置
  {
    ImuRawChannel ch;
    ch.push(make(5U));
    ch.push(make(6U));
    ch.clear();
    ImuRawSample s{};
    require(ch.count() == 0U && ch.total() == 0U, "T6 clear 后 count/total 应为 0");
    require(!ch.latest(s), "T6 clear 后 latest 应失败");
  }

  std::printf("[PASS] test_imu_raw_channel: 6 组用例全部通过 (kCapacity=%u)\n",
              static_cast<unsigned>(kCap));
  return 0;
}
