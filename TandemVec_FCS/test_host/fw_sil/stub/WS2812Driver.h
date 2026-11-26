/**
 * @file    WS2812Driver.h
 * @module  固件/宿主 SIL 桩（fw-tests：WS2812 灯效驱动替身）
 * @brief   提供真实固件 `setup()` 所需的 WS2812 接口，屏蔽 TIM+DMA 平台分支
 * @details 被测：无（替身层）。真实驱动 `lib/WS2812Driver/src/WS2812Driver.h` 的
 *          `beginDma()` 只在 `ARDUINO_ARCH_STM32 && STM32H7xx && HAL_DMA_MODULE_ENABLED`
 *          成立时声明（`WS2812_HAVE_H7_DMA`），宿主不具备这些宏，因此该成员不存在，
 *          导致 `main.cpp` 的 `ws2812Led.beginDma(TIM4, TIM_CHANNEL_4)` 无法编译。
 *          本替身用同名头覆盖（`-Itest_host/fw_sil/stub` 优先于 `lib/.../src`），
 *          只保留"能被调用且返回成功"的最小面。
 * @theory  N/A（灯效外设替身，不参与控制/导航/估计）
 * @frame   N/A
 * @units  N/A
 * @note    与真实驱动的差异：不产生 WS2812 时序、不占 DMA、不发任何字节。
 *          因此"灯效是否正确"永远不能由宿主 SIL 证明，只能上板看灯。
 *          若将来要在宿主验证灯效协议（颜色/时序字节），应改为在替身里记录
 *          写入的像素缓冲并加断言，而不是接真实 DMA 后端。
 * @see     TandemVec_FCS/lib/WS2812Driver/src/WS2812Driver.h（真实驱动）、
 *          TandemVec_FCS/src/main.cpp（beginDma/begin 调用点）
 */
#pragma once

#include <cstdint>

/// 灯效驱动替身：所有方法返回成功但不产生任何时序。
class WS2812Driver
{
public:
  /// 传输模式（真实驱动同名枚举；通信层的 DBG 命令会读写它）
  enum Mode { MODE_BITBANG = 0, MODE_TIM_DMA = 1 };

  /// 真实驱动构造：(数据引脚, 灯珠数量)
  WS2812Driver(int /*pin*/, int /*count*/) {}
  WS2812Driver() {}

  /// 真实签名 `beginDma(TIM_TypeDef*, uint32_t)`。HAL 桩已提供 TIM_TypeDef，
/// 这里保持同形参，避免把指针强转成整型（main.cpp 直接传 TIM4 定时器实例）。
  bool beginDma(TIM_TypeDef * /*tim*/, unsigned /*channel*/) { return false; }

  bool begin() { return true; }
  bool isDmaActive() const { return false; }

  // 供 `Ws2812AppStatus` 等上层使用的最小面
  bool show() { return true; }
  uint32_t framesSent() const { return 0; }
  uint32_t sendFailures() const { return 0; }
  void clear() {}
  void setPixel(int /*index*/, uint8_t /*r*/, uint8_t /*g*/, uint8_t /*b*/) {}
  // 真实驱动的像素接口名（Ws2812AppStatus 直接调用）
  void setPixelColor(int /*index*/, uint8_t /*r*/, uint8_t /*g*/, uint8_t /*b*/) {}
  void setPixelColor(int /*index*/, unsigned long /*rgb*/) {}

  // ---- DBG/状态灯命令用到的接口（communication.cpp）----
  void fill(uint8_t /*r*/, uint8_t /*g*/, uint8_t /*b*/) {}
  void fill(unsigned long /*rgb*/) {}
  int numPixels() const { return 1; }
  uint8_t getBrightness() const { return 255; }
  void setBrightness(uint8_t /*b*/) {}
  Mode getMode() const { return MODE_BITBANG; }
  void setMode(Mode /*m*/) {}
  // ---- DMA 诊断（communication.cpp 的 wsdbg 命令 dump 这些值）----
  uint32_t dmaBufAddr() const { return 0; }
  uint32_t dmaBufLen() const { return 0; }
  uint32_t dmaDstAddr() const { return 0; }
  uint32_t dmaBufAt(size_t /*i*/) const { return 0; }

  /// 静态电平测试（绕过协议直接拉高/拉低数据脚）
  void setStaticLevel(int /*level*/) {}
};