/**
 * @file    test_icm42688_fifo_packet.cpp
 * @module  传感器库 / ICM-42688-P FIFO 解析测试
 * @brief   ICM-42688-P FIFO Packet 3 解析器宿主单测
 * @details 对照 DS-000347 §6.1/§6.2：header/accel/gyro/temp/timestamp 的字节序、
 *          header 合法性、短包拒绝，以及 ±8g/±2000dps 的原始 LSB 换算示例。
 */
#include "icm42688_fifo_packet.h"

#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>

static void putBe16(uint8_t *p, int16_t v)
{
  p[0] = static_cast<uint8_t>((static_cast<uint16_t>(v) >> 8) & 0xFFU);
  p[1] = static_cast<uint8_t>(static_cast<uint16_t>(v) & 0xFFU);
}

int main()
{
  uint8_t packet[16] = {};
  packet[0] = 0x68; // 0110_10xx: accel+gyro+ODR timestamp
  putBe16(&packet[1], 0x0102);   // accel x = 258
  putBe16(&packet[3], -2);       // accel y = -2
  putBe16(&packet[5], 4096);     // accel z = 4096 LSB
  putBe16(&packet[7], 0x1234);   // gyro x = 4660
  putBe16(&packet[9], -123);     // gyro y = -123
  putBe16(&packet[11], 164);     // gyro z = 164 LSB = 10 dps @16.4
  packet[13] = 0x5A;             // temp byte
  packet[14] = 0xAB;             // timestamp hi
  packet[15] = 0xCD;             // timestamp lo

  icm42688_fifo::Packet3 out{};
  assert(icm42688_fifo::parsePacket3(packet, sizeof(packet), out));
  assert(out.accel[0] == 258);
  assert(out.accel[1] == -2);
  assert(out.accel[2] == 4096);
  assert(out.gyro[0] == 4660);
  assert(out.gyro[1] == -123);
  assert(out.gyro[2] == 164);
  assert(out.temp == 0x5A);
  assert(out.timestamp == 0xABCD);

  // ±8g: 4096 LSB/g；±2000dps: 16.4 LSB/dps
  assert(std::fabs(static_cast<float>(out.accel[2]) / 4096.0f - 1.0f) < 1e-6f);
  assert(std::fabs(static_cast<float>(out.gyro[2]) / 16.4f - 10.0f) < 1e-5f);

  // header 非法：FIFO 空 / 仅 accel / bit7=1
  uint8_t bad_header[16] = {};
  bad_header[0] = 0xE8;
  assert(!icm42688_fifo::parsePacket3(bad_header, sizeof(bad_header), out));
  bad_header[0] = 0x48;
  assert(!icm42688_fifo::parsePacket3(bad_header, sizeof(bad_header), out));

  // 短包拒绝
  assert(!icm42688_fifo::parsePacket3(packet, 15, out));
  assert(!icm42688_fifo::parsePacket3(nullptr, sizeof(packet), out));

  // 0x6C（bits3:2=11，FSYNC 时间）也是 16B 合法包
  packet[0] = 0x6C;
  assert(icm42688_fifo::parsePacket3(packet, sizeof(packet), out));

  std::printf("PASS: icm42688 fifo packet3 parse\n");
  return 0;
}
