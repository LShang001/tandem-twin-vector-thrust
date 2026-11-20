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
#include <cstddef>
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
  // Arduino STM32 Core 的引脚重映射接口（sensor_imu.cpp 的 initImu 会调用）。
  // 宿主无物理引脚，吞掉即可；引脚正确性只能靠上板验证。
  void setMISO(unsigned long) {}
  void setMOSI(unsigned long) {}
  void setSCLK(unsigned long) {}
  void setSS(unsigned long) {}
  void setSSEL(unsigned long) {}

  // ---- 寄存器级替身（全链路数字孪生）----
  // 默认**关闭**：transfer() 原样回传入参，保持既有 SIL 行为完全不变。
  // 开启后按真实 ILI/ICM SPI 读协议模拟一个 128 字节寄存器文件：
  //   地址字节 bit7=1 → 读，后续 transfer 返回寄存器内容并自增地址；
  //   地址字节 bit7=0 → 写，下一个字节写入该地址后回到"等地址"状态
  //   （ICM42688::writeRegister 正是"地址 + 单字节数据"两拍）。
  // 这使真实芯片驱动（含 writeRegister 的 delay+回读校验）能在宿主跑通，
  // 而不是用"假 IMU 后端"绕过驱动逻辑。
  void silEnableRegMode(bool on) { reg_mode_ = on; awaiting_addr_ = true; reading_ = false; }
  void silSetReg(uint8_t addr, uint8_t value) { regs_[addr & 0x7FU] = value; }
  uint8_t silGetReg(uint8_t addr) const { return regs_[addr & 0x7FU]; }

  // ---- FIFO 队列替身（ICM-42688 FIFO 数据口）----
  // 真实驱动 readFifoPackets() 先读 FIFO_COUNTH/L(0x2E/0x2F) 取字节数，
  // 再从 FIFO_DATA(0x30) 连续 burst 读。桩必须如实回答这两个地址，
  // 否则驱动永远认为 FIFO 是空的（injectFifo 不参与该路径）。
  void silFifoClear() { fifo_len_ = 0; fifo_pos_ = 0; }
  void silFifoPush(const uint8_t *data, size_t n) {
    for (size_t i = 0; i < n && fifo_len_ < sizeof(fifo_); ++i) fifo_[fifo_len_++] = data[i];
  }
  size_t silFifoCount() const { return fifo_len_ - fifo_pos_; }

  void silResetRegs() {
    for (auto &r : regs_) r = 0;
    addr_ = 0;
    reading_ = false;
    awaiting_addr_ = true;
    writes_ = 0;
    silFifoClear();
  }
  uint32_t silWriteCount() const { return writes_; }

  /// 读一个地址：FIFO 三个特殊地址走队列，其余走寄存器文件。
  uint8_t silReadAddr(uint8_t a) {
    if (a == 0x2EU) return static_cast<uint8_t>((silFifoCount() >> 8) & 0x0FU);
    if (a == 0x2FU) return static_cast<uint8_t>(silFifoCount() & 0xFFU);
    if (a == 0x30U) {
      if (fifo_pos_ < fifo_len_) return fifo_[fifo_pos_++];
      return 0;
    }
    return regs_[a & 0x7FU];
  }

  uint8_t transfer(uint8_t v) {
    if (!reg_mode_) return v;  // 旧行为：原样回传
    if (awaiting_addr_) {
      addr_ = v & 0x7FU;
      reading_ = (v & 0x80U) != 0;
      awaiting_addr_ = false;
      return 0;
    }
    if (reading_) {
      const uint8_t r = silReadAddr(addr_);
      addr_ = static_cast<uint8_t>((addr_ + 1U) & 0x7FU);
      // ★真实 SPI 由片选界定事务；桩没有 CS 信号，只能约定"每次事务都以地址字节
      //   开头"。因此单字节读/写完成后立刻回到等地址，否则下一次读的地址字节会被
      //   当成数据取走（ICM 驱动 whoAmI() 因此读到错值、begin() 返回 -3）。
      awaiting_addr_ = true;
      return r;
    }
    regs_[addr_ & 0x7FU] = v;
    ++writes_;
    awaiting_addr_ = true;  // 单字节写完成，回到等地址
    return 0;
  }

  void transfer(void *buf, size_t n) {
    if (!reg_mode_) return;  // 旧行为：空操作
    uint8_t *p = static_cast<uint8_t *>(buf);
    if (reading_) {
      // ★FIFO_DATA(0x30) 是数据口：硬件连续读会自动前进，不按寄存器地址递增。
      //   若沿用"地址+i"的普通路径，只有第一个字节从队列弹出、其余读到寄存器文件
      //   的 0，表现为 readFifo 已拿到帧数却解析出全 0。
      if (addr_ == 0x30U) {
        for (size_t i = 0; i < n; ++i) {
          p[i] = (fifo_pos_ < fifo_len_) ? fifo_[fifo_pos_++] : 0U;
        }
      } else {
        for (size_t i = 0; i < n; ++i) p[i] = silReadAddr(static_cast<uint8_t>((addr_ + i) & 0x7FU));
        addr_ = static_cast<uint8_t>((addr_ + n) & 0x7FU);
      }
    } else {
      for (size_t i = 0; i < n; ++i) {
        regs_[(addr_ + i) & 0x7FU] = p[i];
        ++writes_;
      }
    }
    awaiting_addr_ = true;  // 批量事务结束，等待下一个地址字节
  }

  void write(uint8_t) {}
  void write(const uint8_t *, size_t) {}

  /// Arduino STM32 Core 的全双工批量传输（同时收发）。
  /// W25N01GV 用 `transfer(nullptr, data, len)` 只收不发。宿主不回任何数据：
  /// 黑匣子内容与 Flash 状态永远不成立，只能验证"代码路径通"。
  void transfer(const void * /*tx*/, void *rx, size_t n) {
    if (rx != nullptr) {
      uint8_t *p = static_cast<uint8_t *>(rx);
      for (size_t i = 0; i < n; ++i) p[i] = 0;
    }
  }

private:
  uint8_t regs_[128] = {};
  uint8_t fifo_[512] = {};
  size_t fifo_len_ = 0;
  size_t fifo_pos_ = 0;
  uint8_t addr_ = 0;
  bool reading_ = false;
  bool awaiting_addr_ = true;
  bool reg_mode_ = false;
  uint32_t writes_ = 0;
};
extern SPIClass SPI;
