/**
 * @file    test_icm45686_fifo_mock.cpp
 * @module  固件/宿主回归测试（fw-tests：ICM-45686 FIFO/WOM/APEX，mock transport）
 * @brief   用寄存器镜像 + 间接寄存器窗口模拟，验证 FIFO 帧解析、WOM 阈值写入
 *          与 APEX pedometer 使能序列。
 * @details 被测：`inv_imu_driver.c`、`inv_imu_transport.c`、`inv_imu_edmp.c`
 *          （不经 Arduino 层）。
 *          mock 规则：
 *            - 直接寄存器 0x00-0xFF 存 `g_regs`；
 *            - 间接访问：写 `0x7C` 两/三字节 = 地址高/低(+首数据)，写/读 `0x7E`
 *              逐字节访问 `g_mreg[addr]` 且地址自增（与 transport.c 一致）。
 *          输出：`require()` 失败即 exit(1)。
 * @theory  手册 DS-000577：FIFO 16-byte 帧布局；WOM 阈值在 IPREG 间接地址
 *          0xA27E-0xA280（单位 1g/256）；APEX 使能在 EDMP_APEX_EN0(0x29)。
 * @frame   N/A
 * @units   raw LSB、mg、寄存器码。
 * @test    T1 FIFO 帧解析；T2 frame_count 字节序；T3 WOM 阈值/配置；
 *          T4 APEX pedometer 使能
 * @see     TandemVec_FCS/lib/ICM45686/src/imu/inv_imu_transport.c
 */
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

extern "C"
{
#include "imu/inv_imu_driver.h"
#include "imu/inv_imu_driver_advanced.h"
#include "imu/inv_imu_edmp.h"
}

namespace
{
uint8_t g_regs[256];
uint8_t g_mreg[0x10000];
uint16_t g_ireg_addr = 0;

int mock_read(void *, uint8_t reg, uint8_t *buf, uint32_t len)
{
  if (reg == IREG_DATA)
  {
    for (uint32_t i = 0U; i < len; ++i)
    {
      buf[i] = g_mreg[g_ireg_addr++];
    }
    return 0;
  }
  if (reg == IREG_ADDR_15_8 && len == 2U)
  {
    buf[0] = static_cast<uint8_t>(g_ireg_addr >> 8);
    buf[1] = static_cast<uint8_t>(g_ireg_addr & 0xFFU);
    return 0;
  }
  for (uint32_t i = 0U; i < len; ++i)
  {
    buf[i] = g_regs[(reg + i) & 0xFFU];
  }
  return 0;
}

int mock_write(void *, uint8_t reg, const uint8_t *buf, uint32_t len)
{
  if (reg == IREG_ADDR_15_8)
  {
    if (len >= 2U)
    {
      g_ireg_addr = static_cast<uint16_t>((buf[0] << 8) | buf[1]);
    }
    if (len == 3U)
    {
      g_mreg[g_ireg_addr++] = buf[2];
    }
    return 0;
  }
  if (reg == IREG_DATA)
  {
    for (uint32_t i = 0U; i < len; ++i)
    {
      g_mreg[g_ireg_addr++] = buf[i];
    }
    return 0;
  }
  for (uint32_t i = 0U; i < len; ++i)
  {
    g_regs[(reg + i) & 0xFFU] = buf[i];
  }
  return 0;
}

void reset_mock()
{
  std::memset(g_regs, 0, sizeof(g_regs));
  std::memset(g_mreg, 0, sizeof(g_mreg));
  g_ireg_addr = 0U;
}

inv_imu_device_t make_device()
{
  inv_imu_device_t dev{};
  dev.transport.context = g_regs;
  dev.transport.read_reg = mock_read;
  dev.transport.write_reg = mock_write;
  dev.transport.serif_type = UI_SPI4;
  dev.transport.sleep_us = [](uint32_t) {};
  dev.endianness_data = 1U; // 1 = 大端（与 FORMAT_16_BITS_DATA 一致）
  dev.fifo_frame_size = 16U;
  return dev;
}

void require(bool condition, const char *message)
{
  if (!condition)
  {
    std::fprintf(stderr, "FAILED: %s\n", message);
    std::exit(1);
  }
}

void checks_16byte_frame_big_endian()
{
  reset_mock();
  inv_imu_device_t dev = make_device();

  const uint8_t frame[16] = {0x80U,
                             0x01U, 0x02U, 0x03U, 0x04U, 0x05U, 0x06U,
                             0x07U, 0x08U, 0x09U, 0x0AU, 0x0BU, 0x0CU,
                             0x20U, 0x12U, 0x34U};
  std::memcpy(&g_regs[FIFO_DATA], frame, sizeof(frame));

  inv_imu_fifo_data_t data{};
  require(inv_imu_get_fifo_frame(&dev, &data) == 0, "get_fifo_frame returns 0");
  require(data.header.Byte == 0x80U, "header 0x80");
  require(data.byte_16.accel_data[0] == 0x0102, "accel x 0x0102");
  require(data.byte_16.accel_data[1] == 0x0304, "accel y 0x0304");
  require(data.byte_16.accel_data[2] == 0x0506, "accel z 0x0506");
  require(data.byte_16.gyro_data[0] == 0x0708, "gyro x 0x0708");
  require(data.byte_16.gyro_data[1] == 0x090A, "gyro y 0x090A");
  require(data.byte_16.gyro_data[2] == 0x0B0C, "gyro z 0x0B0C");
  require(data.byte_16.temp_data == 0x20, "temp 0x20");
  require(data.byte_16.timestamp == 0x1234, "timestamp 0x1234");
}

void checks_frame_count_endianness()
{
  reset_mock();
  inv_imu_device_t dev = make_device();
  g_regs[FIFO_COUNT_0] = 0x12U;
  g_regs[FIFO_COUNT_1] = 0x00U;
  uint16_t count = 0U;
  require(inv_imu_get_frame_count(&dev, &count) == 0, "frame_count returns 0");
  require(count == 0x1200U, "big-endian frame count 0x1200");
}

void checks_wom_config_sequence()
{
  reset_mock();
  inv_imu_device_t dev = make_device();
  require(inv_imu_adv_configure_wom(&dev, 13U, 14U, 15U,
                                    TMST_WOM_CONFIG_WOM_INT_MODE_ORED,
                                    TMST_WOM_CONFIG_WOM_INT_DUR_1_SMPL) == 0,
          "configure_wom returns 0");
  // 阈值写到间接地址 0xA27E/0xA27F/0xA280。
  require(g_mreg[0xA27E] == 13U, "WOM X threshold 13");
  require(g_mreg[0xA27F] == 14U, "WOM Y threshold 14");
  require(g_mreg[0xA280] == 15U, "WOM Z threshold 15");
  // TMST_WOM_CONFIG 应已写入（wom_mode=CMP_PREV），但 enable 前 wom_en 仍为 0。
  tmst_wom_config_t wom_cfg{};
  std::memcpy(&wom_cfg, &g_regs[TMST_WOM_CONFIG], 1);
  require(wom_cfg.wom_en == 0U, "WOM disabled after configure");
  require(inv_imu_adv_enable_wom(&dev) == 0, "enable_wom returns 0");
  std::memcpy(&wom_cfg, &g_regs[TMST_WOM_CONFIG], 1);
  require(wom_cfg.wom_en == 1U, "WOM enabled after enable_wom");
}

void checks_apex_pedometer_enable()
{
  reset_mock();
  inv_imu_device_t dev = make_device();
  // check_dmp_odr_decimation 要求 SRAM[0x460] 的抽取倍率与 APEX ODR 匹配：
  // 全零镜像下 0x460=0、DMP_EXT_SEN_ODR_CFG=0 → APEX 25Hz，组合合法。
  require(inv_imu_edmp_enable_pedometer(&dev) == 0, "enable_pedometer returns 0");
  // EDMP_APEX_EN0(0x29) 的 pedo_en 位应被置 1。
  edmp_apex_en0_t apex{};
  std::memcpy(&apex, &g_regs[EDMP_APEX_EN0], 1);
  require(apex.pedo_en == 1U, "pedometer enable bit set");
}
} // namespace

int main()
{
  checks_16byte_frame_big_endian();
  checks_frame_count_endianness();
  checks_wom_config_sequence();
  checks_apex_pedometer_enable();
  std::puts("icm45686_fifo_mock_test passed");
  return 0;
}
