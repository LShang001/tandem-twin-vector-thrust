/**
 * @file    Wire.h
 * @module  固件/宿主 SIL 桩（fw-tests：I2C 外设替身）
 * @brief   提供 TwoWire 的宿主替身：默认全空，可切换为"内存寄存器文件"
 * @details 被测：无（替身层）。
 *          **默认行为与旧桩一致**：`endTransmission` 恒返回 0，`requestFrom`/`read`
 *          恒返回 0，`write` 为空操作——保证既有的 SEL/磁力计等构建行为不变。
 *          调 `silEnable()` 之后切换为寄存器文件模型：
 *            beginTransmission(addr) → write(reg) → endTransmission(false)
 *            → requestFrom(addr, n) → read() × n（寄存器地址自动递增）
 *          这是 DPS310/IST8310 这类器件的真实访问序列，用来在宿主验证
 *          "驱动真的把寄存器值解成了物理量"，而不是在测试里直接写结构体。
 * @theory  N/A（外设替身）
 * @frame   N/A
 * @units   N/A
 * @note    与真实 I2C 的差异：无地址应答（任何地址都"在线"）、无总线错误、无超时、
 *          无时钟拉伸。因此"传感器在线/离线"这类判据在 SIL 中恒为在线，
 *          不能用于验证 I2C 故障路径。
 * @see     TandemVec_FCS/test_host/fw_sil/fw_sil_stubs.cpp
 */
#pragma once
#include <cstdint>

class TwoWire {
public:
  /// 寄存器地址空间上限（DPS310 为 0x00–0x32，IST8310 为 0x00–0x0C）
  static constexpr int kRegCap = 256;

  TwoWire() {}
  TwoWire(int, int) {}
  void begin() {}
  /// Arduino STM32 Core 的总线速率设置；宿主不产生时钟，仅吞掉调用。
  void setClock(uint32_t) {}

  // ---- SIL 控制面（仅测试调用，固件侧不会用到）----
  /// 打开寄存器文件模型。未调用时保持旧桩行为（读恒 0）。
  void silEnable() { enabled_ = true; }
  void silDisable() { enabled_ = false; }
  bool silEnabled() const { return enabled_; }
  void silSetReg(uint8_t reg, uint8_t v) {
    if (reg < kRegCap) regs_[reg] = v;
  }
  uint8_t silGetReg(uint8_t reg) const { return reg < kRegCap ? regs_[reg] : 0; }
  /// 连续写一段寄存器（FIFO 数据块、系数块用）。
  void silSetBlock(uint8_t reg, const uint8_t *data, int n) {
    for (int i = 0; i < n; ++i) silSetReg(uint8_t(reg + i), data[i]);
  }
  /// 已写入的寄存器数据字节数（诊断：判断驱动配置有没有真的下发）。
  uint32_t silWriteCount() const { return writeCount_; }
  void silClearCounters() { writeCount_ = 0; readCount_ = 0; }

  // ---- Arduino I2C 面 ----
  void beginTransmission(int addr) {
    addr_ = addr;
    regPtr_ = 0;
    haveReg_ = false;
  }
  void write(int v) {
    ++writeCount_;
    if (!enabled_) return;
    if (!haveReg_) {
      regPtr_ = static_cast<uint8_t>(v);   // 首个字节是寄存器地址
      haveReg_ = true;
    } else {
      silSetReg(regPtr_, static_cast<uint8_t>(v));
      ++regPtr_;
    }
  }
  int endTransmission() { return 0; }
  int endTransmission(bool) { return 0; }

  int requestFrom(int addr, int n) {
    addr_ = addr;
    readIdx_ = 0;
    readLen_ = n;
    return enabled_ ? n : 0;
  }
  int requestFrom(uint8_t addr, uint8_t n) { return requestFrom(int(addr), int(n)); }
  int requestFrom(uint8_t addr, uint8_t n, uint8_t) { return requestFrom(int(addr), int(n)); }

  /// 读一个字节，寄存器指针自动递增（DPS310 的连续读语义）。
  int read() {
    ++readCount_;
    if (!enabled_) return 0;
    if (readIdx_ >= readLen_) return 0;
    ++readIdx_;
    const uint8_t v = silGetReg(regPtr_);
    ++regPtr_;
    return v;
  }

private:
  uint8_t regs_[kRegCap] = {0};
  int addr_ = 0;
  uint8_t regPtr_ = 0;
  int readIdx_ = 0;
  int readLen_ = 0;
  bool haveReg_ = false;
  bool enabled_ = false;
  uint32_t writeCount_ = 0;
  uint32_t readCount_ = 0;
};

extern TwoWire Wire;