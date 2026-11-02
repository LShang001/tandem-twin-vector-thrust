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

void maps_lpf_code_to_cutoff()
{
  // ODR 2000：码 0..7 → 1000/500/400/250/200/125/100 Hz（手册 §14.40）。
  const float expect2000[8] = {1000.0f, 500.0f, 400.0f, 250.0f,
                               200.0f, 125.0f, 100.0f, 50.0f};
  for (uint8_t code = 0U; code < 8U; ++code)
  {
    require(imu_filter::lpfCutoffHz(2000U, code) == expect2000[code],
            "LPF cutoff at ODR=2000");
    require(imu_filter::lpfCutoffHz(100U, code) ==
                ((code == 0U) ? 50.0f : 400.0f / ((code == 7U) ? 40.0f
                                                   : (code == 1U) ? 4.0f
                                                   : (code == 2U) ? 5.0f
                                                   : (code == 3U) ? 8.0f
                                                   : (code == 4U) ? 10.0f
                                                   : (code == 5U) ? 16.0f
                                                                   : 20.0f)),
            "LPF cutoff at ODR=100 uses 400Hz base");
  }
  require(imu_filter::lpfCutoffHz(2000U, 8U) == 0.0f, "reserved code -> 0");
  require(imu_filter::lpfCutoffHz(0U, 0U) == 0.0f, "ODR=0 -> 0");
}

void selects_lpf_code_by_target()
{
  // 目标 200Hz @2000 → 码 4（200Hz 精确）；目标 100Hz → 码 6（100Hz 精确）。
  require(imu_filter::selectLpfCode(2000U, 200.0f) == 4U, "200Hz@2000 -> code 4");
  require(imu_filter::selectLpfCode(2000U, 100.0f) == 6U, "100Hz@2000 -> code 6");
  require(imu_filter::selectLpfCode(2000U, 400.0f) == 2U, "400Hz@2000 -> code 2");
  // 100Hz ODR：400 为基，码 1=100Hz 精确，码 0=50Hz。
  require(imu_filter::selectLpfCode(100U, 100.0f) == 1U, "100Hz@100 -> code 1");
  require(imu_filter::selectLpfCode(100U, 50.0f) == 0U, "50Hz@100 -> code 0");
  require(imu_filter::selectLpfCode(0U, 200.0f) == 0U, "ODR=0 -> code 0");
  require(imu_filter::selectLpfCode(2000U, 0.0f) == 0U, "target=0 -> code 0");
}

void packs_aaf_registers()
{
  // 213Hz 档：DELT=5、DELTSQR=25、BITSHIFT=10 → STATIC5 = (10<<4)|0 = 0xA0。
  const auto &e = imu_filter::aafForBandwidth(213.0f);
  const imu_filter::AafRegisters r = imu_filter::aafRegisters(e);
  require(r.delt == 5U && r.deltsqr_low == 25U &&
              r.deltsqr_high_bitshift == 0xA0U,
          "213Hz AAF packing");
  // 3979Hz 档：DELT=63、DELTSQR=3968(0xF80)、BITSHIFT=3 → 低字节 0x80、高字节 0x3F。
  const auto &hi = imu_filter::aafForBandwidth(3979.0f);
  const imu_filter::AafRegisters rh = imu_filter::aafRegisters(hi);
  require(rh.delt == 63U && rh.deltsqr_low == 0x80U &&
              rh.deltsqr_high_bitshift == 0x3FU,
          "3979Hz AAF packing");
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
  maps_lpf_code_to_cutoff();
  selects_lpf_code_by_target();
  packs_aaf_registers();
  rejects_out_of_range_notch();
  std::puts("imu_filter_math_test passed");
  return 0;
}
