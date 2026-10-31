/**
 * @file    test_icm45686_fifo_mock.cpp
 * @module  固件/宿主回归测试（fw-tests：ICM-45686 FIFO 帧解析，mock transport）
 * @brief   用寄存器镜像 + mock 读写验证 `inv_imu_get_fifo_frame()` 的 16-byte
 *          帧解析与 `inv_imu_get_frame_count()` 的字节序处理。
 * @details 被测：`lib/ICM45686/src/imu/inv_imu_driver.c`（不经 Arduino 层）。
 *          输入：构造的 16-byte 帧（header+accel+gyro+temp+timestamp）与
 *          FIFO_COUNT_0/1 寄存器值。
 *          输出：`require()` 失败即 exit(1)。
 * @theory  手册 DS-000577 §16：16-bit FIFO 帧布局为 header(1)+accel(6)+gyro(6)
 *          +temp(1)+timestamp(2)；数据按 INTF_CONFIG0.SENSOR_DATA_ENDIAN 解释。
 * @frame   N/A
 * @units   raw LSB；温度 raw/128+25°C 由上层换算。
 * @note    mock 只支持直接寄存器（0x00-0xFF）；间接寄存器（IPREG）不在本测试范围。
 * @test    T1 16-byte 帧大端解析；T2 frame_count 字节序
 * @see     TandemVec_FCS/lib/ICM45686/src/imu/inv_imu_driver.c
 */
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

extern "C"
{
#include "imu/inv_imu_driver.h"
}

namespace
{
uint8_t g_regs[256];

int mock_read(void *, uint8_t reg, uint8_t *buf, uint32_t len)
{
  for (uint32_t i = 0U; i < len; ++i)
  {
    buf[i] = g_regs[(reg + i) & 0xFFU];
  }
  return 0;
}

int mock_write(void *, uint8_t reg, const uint8_t *buf, uint32_t len)
{
  for (uint32_t i = 0U; i < len; ++i)
  {
    g_regs[(reg + i) & 0xFFU] = buf[i];
  }
  return 0;
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
  inv_imu_device_t dev{};
  dev.transport.context = g_regs;
  dev.transport.read_reg = mock_read;
  dev.transport.write_reg = mock_write;
  dev.transport.serif_type = UI_SPI4;
  dev.endianness_data = 1U; // 1 = 大端（与 FORMAT_16_BITS_DATA 的 is_big_endian 一致）
  dev.fifo_frame_size = 16U;

  // header + accel(0x0102/0x0304/0x0506) + gyro(0x0708/0x090A/0x0B0C)
  // + temp(0x20) + timestamp(0x1234)
  const uint8_t frame[16] = {0x80U,
                             0x01U, 0x02U, 0x03U, 0x04U, 0x05U, 0x06U,
                             0x07U, 0x08U, 0x09U, 0x0AU, 0x0BU, 0x0CU,
                             0x20U, 0x12U, 0x34U};
  std::memcpy(&g_regs[FIFO_DATA], frame, sizeof(frame));

  inv_imu_fifo_data_t data{};
  const int rc = inv_imu_get_fifo_frame(&dev, &data);
  require(rc == 0, "get_fifo_frame returns 0");
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
  inv_imu_device_t dev{};
  dev.transport.context = g_regs;
  dev.transport.read_reg = mock_read;
  dev.transport.write_reg = mock_write;
  dev.transport.serif_type = UI_SPI4;
  dev.endianness_data = 1U; // 大端

  g_regs[FIFO_COUNT_0] = 0x12U;
  g_regs[FIFO_COUNT_1] = 0x00U;
  uint16_t count = 0U;
  require(inv_imu_get_frame_count(&dev, &count) == 0, "frame_count returns 0");
  require(count == 0x1200U, "big-endian frame count 0x1200");
}
} // namespace

int main()
{
  checks_16byte_frame_big_endian();
  checks_frame_count_endianness();
  std::puts("icm45686_fifo_mock_test passed");
  return 0;
}
