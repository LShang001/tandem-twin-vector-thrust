/**
 * @file    Wire.h
 * @module  固件/宿主 SIL 桩（fw-tests：I2C 外设替身）
 * @brief   提供 TwoWire 的空实现，满足 I2C 传感器驱动在宿主机编译与链接
 * @details 被测：无（替身层）。`endTransmission` 恒返回 0（成功），
 *          `requestFrom`/`read` 恒返回 0，`write` 为空操作。
 * @theory  N/A（外设替身）
 * @frame   N/A
 * @units   N/A
 * @note    与真实 I2C 的差异：无地址应答、无总线错误、无超时——因此“传感器在线/离线”
 *          这类判据在 SIL 中恒为在线，不能用于验证 I2C 故障路径。
 * @see     TandemVec_FCS/test_host/fw_sil/fw_sil_stubs.cpp
 */
#pragma once
// 宿主机无 I2C 控制器：以下成员函数全部为空实现/常量返回。
class TwoWire {
public:
  TwoWire() {}
  TwoWire(int, int) {}
  void begin() {}
  void beginTransmission(int) {}
  int endTransmission() { return 0; }
  int endTransmission(bool) { return 0; }
  int requestFrom(int, int) { return 0; }
  int requestFrom(uint8_t, uint8_t) { return 0; }
  int requestFrom(uint8_t, uint8_t, uint8_t) { return 0; }
  int read() { return 0; }
  void write(int) {}
};
extern TwoWire Wire;
