/**
 * @file    test_imu_filter_math.cpp
 * @module  固件/宿主回归测试（fw-tests：ICM-42688-P AAF/notch 参数计算）
 * @brief   验证 AAF 带宽表映射与 notch 系数计算，与固件共用同一头文件。
 * @details 被测：`lib/ICM42688/src/imu_filter_math.h`
 *          （手册 DS-000347 §5.3 AAF 表、§5.2 notch 公式）。
 *          输入：典型目标带宽 50/100/200/400Hz；notch 频率 1/2/3kHz 与越界值。
 *          处理：查表最近邻、cos(2πf/32) 与 |COSWZ|>0.875 的 SEL 分支。
 *          输出：`require()` 失败即 exit(1)。
 * @theory  AAF 表是手册逐行数据，不是解析公式；notch 的 SEL 分支用于
 *          |COSWZ|>0.875 时把 9-bit 编码压回有效范围。
 * @frame   N/A
 * @units   带宽 Hz、频率 kHz、COSWZ 为 9-bit 编码。
 * @note    该测试不依赖 Arduino/STM32，可在任意宿主机 g++ 直接跑。
 * @test    T1 表完整性；T2 典型带宽映射；T3 notch 三分支；T4 越界拒绝
 * @see     TandemVec_FCS/lib/ICM42688/src/imu_filter_math.h
 */
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "../lib/ICM42688/src/imu_filter_math.h"

namespace
{
void require(bool condition, const char *message)
{
  if (!condition)
  {
    std::fprintf(stderr, "FAILED: %s\n", message);
    std::exit(1);
  }
}

void table_is_complete_and_monotonic()
{
  std::size_t n = 0;
  const imu_filter::AafEntry *t = imu_filter::aafTable(n);
  require(n == 63U, "AAF table has 63 entries");
  require(t[0].bw_hz == 42U && t[0].delt == 1U && t[0].deltsqr == 1U &&
              t[0].bitshift == 15U,
          "AAF first entry is 42Hz/1/1/15");
  require(t[n - 1].bw_hz == 3979U && t[n - 1].delt == 63U &&
              t[n - 1].deltsqr == 3968U && t[n - 1].bitshift == 3U,
          "AAF last entry is 3979Hz/63/3968/3");
  for (std::size_t i = 1; i < n; ++i)
  {
    require(t[i].bw_hz > t[i - 1].bw_hz, "AAF bandwidths strictly ascending");
    require(t[i].delt > t[i - 1].delt, "AAF DELT strictly ascending");
    require(t[i].deltsqr >= t[i - 1].deltsqr, "AAF DELTSQR non-decreasing");
  }
}

void maps_typical_bandwidths()
{
  // 50Hz 最近 42（差8）而不是 84（差34）；100Hz 最近 84。
  const auto &b50 = imu_filter::aafForBandwidth(50.0f);
  const auto &b100 = imu_filter::aafForBandwidth(100.0f);
  const auto &b200 = imu_filter::aafForBandwidth(200.0f);
  const auto &b400 = imu_filter::aafForBandwidth(400.0f);
  require(b50.bw_hz == 42U && b50.delt == 1U, "50Hz -> 42Hz/1");
  require(b100.bw_hz == 84U && b100.delt == 2U, "100Hz -> 84Hz/2");
  require(b200.bw_hz == 213U && b200.delt == 5U, "200Hz -> 213Hz/5");
  require(b400.bw_hz == 394U && b400.delt == 9U, "400Hz -> 394Hz/9");
}

void computes_notch_coefficients()
{
  uint16_t coswz = 0U;
  uint8_t sel = 0U;
  // f=1kHz：cos(2π/32)=0.980785>0.875 → SEL=1，COSWZ=round(8*(1-0.980785)*256)=39。
  require(imu_filter::notchCoefficients(1.0f, coswz, sel), "1kHz accepted");
  require(sel == 1U && coswz == 39U, "1kHz -> SEL=1 COSWZ=39");
  // f=2kHz：cos(π/8)=0.92388>0.875 → SEL=1，COSWZ=round(8*(1-0.92388)*256)=156。
  require(imu_filter::notchCoefficients(2.0f, coswz, sel), "2kHz accepted");
  require(sel == 1U && coswz == 156U, "2kHz -> SEL=1 COSWZ=156");
  // f=3kHz：cos(3π/16)=0.83147≤0.875 → SEL=0，COSWZ=round(0.83147*256)=213。
  require(imu_filter::notchCoefficients(3.0f, coswz, sel), "3kHz accepted");
  require(sel == 0U && coswz == 213U, "3kHz -> SEL=0 COSWZ=213");
}

void rejects_out_of_range_notch()
{
  uint16_t coswz = 0U;
  uint8_t sel = 0U;
  require(!imu_filter::notchCoefficients(0.5f, coswz, sel), "0.5kHz rejected");
  require(!imu_filter::notchCoefficients(3.5f, coswz, sel), "3.5kHz rejected");
}
} // namespace

int main()
{
  table_is_complete_and_monotonic();
  maps_typical_bandwidths();
  computes_notch_coefficients();
  rejects_out_of_range_notch();
  std::puts("imu_filter_math_test passed");
  return 0;
}
