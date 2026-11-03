/**
 * @file    icm42688_fifo_packet.h
 * @module  传感器库 / ICM-42688-P FIFO Packet 3 解析
 * @brief   按 DS-000347 §6.1/§6.2 解析 accel+gyro+timestamp 的 16 字节 FIFO 包
 * @details 现有 `ICM42688_FIFO` 驱动的 frame size=14、首字节当数据，与手册
 *          Packet 3（header + accel6 + gyro6 + temp1 + timestamp2 = 16 B）不一致；
 *          本解析器只做纯字节解析，不碰 SPI/校准，先供宿主单测和后续批量读取使用。
 *
 * @theory  手册 DS-000347 Rev1.2 §6.1 Packet 3 / §6.2 FIFO Header：
 *          byte0 header；byte1..6 accel X/Y/Z（大端 int16）；byte7..12 gyro；
 *          byte13 temperature[7:0]；byte14..15 timestamp[15:0]。
 *          Header bit7=1 表示 FIFO 空；bit6=1 含 accel；bit5=1 含 gyro；
 *          本解析器要求 header 高半字节=0x6（accel+gyro，普通 16-bit 分辨率）。
 *
 * @ref     TandemVec_FCS/lib/ICM42688/doc/DS-000347_ICM-42688-P_v1.2.md §6.1/§6.2
 * @frame   N/A（芯片原始数据）
 * @units   输出为原始 LSB；换算用驱动的 _accelScale/_gyroScale
 * @note    本文件不依赖 Arduino，宿主可直接单测。
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace icm42688_fifo
{
constexpr size_t kPacket3Size = 16U;

struct Packet3
{
  int16_t accel[3];   ///< accel X/Y/Z raw LSB（大端 int16）
  int16_t gyro[3];    ///< gyro X/Y/Z raw LSB（大端 int16）
  uint8_t temp;       ///< temperature[7:0] 原始字节
  uint16_t timestamp; ///< FIFO ODR timestamp
};

/// FIFO header 是否为一个合法的 accel+gyro 普通分辨率包。
inline bool isPacket3Header(uint8_t header)
{
  // bit7=1 -> FIFO empty；高半字节 0x6 = accel+gyro；bits3:2=10/11 含 timestamp。
  return ((header & 0x80U) == 0U) && ((header & 0xF0U) == 0x60U) &&
         (((header >> 2) & 0x03U) >= 0x02U);
}

/// 解析一个 16 字节 Packet 3。数据不足 / header 非法返回 false。
inline bool parsePacket3(const uint8_t *data, size_t len, Packet3 &out)
{
  if (data == nullptr || len < kPacket3Size || !isPacket3Header(data[0]))
  {
    return false;
  }
  auto be16 = [](const uint8_t *p) -> int16_t
  {
    return static_cast<int16_t>((static_cast<uint16_t>(p[0]) << 8) | p[1]);
  };
  out.accel[0] = be16(&data[1]);
  out.accel[1] = be16(&data[3]);
  out.accel[2] = be16(&data[5]);
  out.gyro[0] = be16(&data[7]);
  out.gyro[1] = be16(&data[9]);
  out.gyro[2] = be16(&data[11]);
  out.temp = data[13];
  out.timestamp = static_cast<uint16_t>((static_cast<uint16_t>(data[14]) << 8) | data[15]);
  return true;
}
} // namespace icm42688_fifo
