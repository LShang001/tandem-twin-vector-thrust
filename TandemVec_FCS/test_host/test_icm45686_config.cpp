/**
 * @file    test_icm45686_config.cpp
 * @module  固件/宿主回归测试（fw-tests：ICM-45686 配置结构/阈值换算）
 * @brief   验证 45686 的 WOM 阈值换算、FIFO/WOM 配置默认值与手册契约一致。
 * @details 被测：`lib/ICM45686/src/ICM45686.h` 的 `womThresholdMgToReg()`、
 *          `ICM45686FifoConfig` 与 `ICM45686WomConfig` 默认值。
 *          输入：典型阈值 0/52/500/1000 mg；默认配置。
 *          输出：`require()` 失败即 exit(1)。
 * @theory  手册 DS-000577：WOM 阈值寄存器单位 1g/256，按四舍五入换算并饱和到 255。
 * @frame   N/A
 * @units   mg、寄存器码。
 * @note    该测试只含头文件，不链接驱动 .c；SPI/Wire 用 fw_sil 的桩。
 * @test    T1 阈值换算；T2 FIFO 默认值；T3 WOM 默认值
 * @see     TandemVec_FCS/lib/ICM45686/src/ICM45686.h
 */
#include <cstdio>
#include <cstdlib>

#include "ICM45686.h"

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

void checks_wom_threshold()
{
  // 1g/256 ≈ 3.90625mg/LSB；52mg → round(52/3.90625)=13。
  require(ICM456xx::womThresholdMgToReg(0) == 0U, "0mg -> 0");
  require(ICM456xx::womThresholdMgToReg(52) == 13U, "52mg -> 13");
  require(ICM456xx::womThresholdMgToReg(500) == 128U, "500mg -> 128");
  require(ICM456xx::womThresholdMgToReg(1000) == 255U, "1000mg saturates to 255");
}

void checks_fifo_defaults()
{
  const ICM45686FifoConfig f{};
  require(f.enable_accel && f.enable_gyro, "FIFO default enables accel+gyro");
  require(!f.high_resolution_20bit, "FIFO default 16-bit");
  require(f.watermark == 10U, "FIFO default watermark 10");
  require(f.accel_decimation == 1U && f.gyro_decimation == 1U, "FIFO default no decimation");
}

void checks_wom_defaults()
{
  const ICM45686WomConfig w{};
  require(w.threshold_x_mg == 52U && w.threshold_y_mg == 52U &&
              w.threshold_z_mg == 52U,
          "WOM default thresholds 52mg");
  require(w.use_or_mode, "WOM default OR mode");
  require(w.duration_samples == 1U, "WOM default duration 1");
}
} // namespace

int main()
{
  checks_wom_threshold();
  checks_fifo_defaults();
  checks_wom_defaults();
  std::puts("icm45686_config_test passed");
  return 0;
}
