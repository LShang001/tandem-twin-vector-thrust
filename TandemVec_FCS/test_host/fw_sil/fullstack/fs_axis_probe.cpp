/**
 * @file    fs_axis_probe.cpp
 * @module  固件/全链路数字孪生（fs：IMU 轴映射探针）
 * @brief   隔离验证「写数据寄存器 → 驱动 accX/Y/Z() 读出」的映射关系
 * @details 只做一件事：往 0x1F 起写一组可区分的 accel/gyro 值，读驱动输出，
 *          直接测出轴序（不做任何闭环、不跑 setup）。
 *          用途：L4 闭环里姿态解算明显偏轴，需要先确定这条链的轴映射真值，
 *          否则会陷入反复猜轴改代码的循环。
 * @note    本文件是一次性诊断探针，结论确认后可删或并入正式测试。
 */

#include "Arduino.h"
#include "SPI.h"
#include "ImuDevice.h"
#include "imu_factory.h"

#include <cstdio>

SPIClass SPI;
uint32_t g_sil_time_us = 0;
SilDelayHook g_sil_delay_hook = nullptr;
ImuDevice *IMU = nullptr;

int main()
{
  std::setvbuf(stdout, nullptr, _IONBF, 0);
  std::printf("== IMU 轴映射探针 ==\n");

  SPI.silResetRegs();
  SPI.silEnableRegMode(true);
  SPI.silSetReg(0x75, 0x47);  // WHO_AM_I
  SPI.silSetReg(0x2D, 0x08);  // DRDY

  ImuDevice *dev = createImuDevice(SPI, 0, 24000000U);
  if (dev == nullptr || dev->begin(ImuConfig{}) != 0)
  {
    std::printf("[FAIL] 驱动初始化失败\n");
    return 1;
  }

  // 写入可区分的值：accel X=1000, Y=2000, Z=3000 LSB；gyro X=100, Y=200, Z=300 LSB
  // 布局（ICM42688 手册 / 驱动 readRegisters(0x1F,14)）：temp2B + accel XYZ + gyro XYZ
  auto be16 = [](uint8_t *p, int32_t v) {
    p[0] = static_cast<uint8_t>((v >> 8) & 0xFF);
    p[1] = static_cast<uint8_t>(v & 0xFF);
  };
  uint8_t pkt[14] = {};
  be16(&pkt[2], 1000);   // accel X
  be16(&pkt[4], 2000);   // accel Y
  be16(&pkt[6], 3000);   // accel Z
  be16(&pkt[8], 100);    // gyro X
  be16(&pkt[10], 200);   // gyro Y
  be16(&pkt[12], 300);   // gyro Z
  for (uint8_t i = 0; i < 14; ++i) SPI.silSetReg(static_cast<uint8_t>(0x1F + i), pkt[i]);

  // 驱动读一次（dataReady 路径走 readRegisters(0x1F,14)）
  const int n = dev->dataReady();
  std::printf("  dataReady() = %d\n", n);
  std::printf("  写入 accel LSB = (1000, 2000, 3000)，gyro LSB = (100, 200, 300)\n");
  std::printf("  读出 accX=%+.5f accY=%+.5f accZ=%+.5f (g)\n",
              static_cast<double>(dev->accX()), static_cast<double>(dev->accY()),
              static_cast<double>(dev->accZ()));
  std::printf("  读出 gyrX=%+.5f gyrY=%+.5f gyrZ=%+.5f (dps)\n",
              static_cast<double>(dev->gyrX()), static_cast<double>(dev->gyrY()),
              static_cast<double>(dev->gyrZ()));

  // 标度反推：accLsb = acc_g * 4096（±8g）
  std::printf("  反推 accel LSB = (%+.0f, %+.0f, %+.0f)\n",
              static_cast<double>(dev->accX() * 4096.0f),
              static_cast<double>(dev->accY() * 4096.0f),
              static_cast<double>(dev->accZ() * 4096.0f));
  std::printf("  反推 gyro  LSB = (%+.0f, %+.0f, %+.0f)\n",
              static_cast<double>(dev->gyrX() * 16.384f),
              static_cast<double>(dev->gyrY() * 16.384f),
              static_cast<double>(dev->gyrZ() * 16.384f));
  return 0;
}