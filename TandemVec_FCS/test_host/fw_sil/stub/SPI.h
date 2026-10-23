/**
 * @file    SPI.h
 * @module  固件/宿主 SIL 桩（fw-tests：SPI 外设替身）
 * @brief   提供 SPISettings/SPIClass 的空实现，满足固件源码在宿主机编译与链接
 * @details 被测：无（替身层）。只声明固件用到的模式/位序常量与成员函数签名；
 *          `transfer()` 原样回传入参，`begin/write` 等为空操作。
 * @theory  N/A（外设替身）
 * @frame   N/A
 * @units   N/A
 * @note    与真实 SPI 的差异：无片选时序、无时钟分频、无 DMA；传感器回归数据
 *          必须由 SIL 主程序注入，不能依赖本桩“读回”任何寄存器值。
 * @see     TandemVec_FCS/test_host/fw_sil/fw_sil_stubs.cpp
 */
#pragma once
#include <cstdint>
#define SPI_MODE3 3
#define SPI_MODE0 0
#define MSBFIRST 1
#define LSBFIRST 0
// 宿主机无 SPI 控制器：以下成员函数全部为空实现/常量返回。
class SPISettings {
public:
  SPISettings() {}
  SPISettings(uint32_t, uint8_t, uint8_t) {}
};
class SPIClass {
public:
  SPIClass() {}
  SPIClass(int, int, int) {}
  void begin() {}
  void end() {}
  void beginTransaction(SPISettings) {}
  void setDataMode(uint8_t) {}
  void setBitOrder(uint8_t) {}
  void setFrequency(uint32_t) {}
  void endTransaction() {}
  uint8_t transfer(uint8_t v) { return v; }
  void transfer(void*, size_t) {}
  void write(uint8_t) {}
  void write(const uint8_t*, size_t) {}
};
extern SPIClass SPI;
