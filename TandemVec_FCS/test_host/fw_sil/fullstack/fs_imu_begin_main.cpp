/**
 * @file    fs_imu_begin_main.cpp
 * @module  固件/全链路数字孪生（fs：真实 IMU 驱动 begin 验证）
 * @brief   让真实 ICM 芯片驱动在宿主跑通 `begin()`，证明"驱动层未经改写即上线"
 * @details 被测：`src/imu_factory.cpp`（装配）+ `lib/ICM42688`（真实芯片驱动）。
 *          输入：宿主 SPI 桩的**寄存器级替身**——预置 WHO_AM_I=0x47，
 *                其余寄存器按"写后可回读"语义保存；
 *          处理：调用真实 `createImuDevice()` 与 `ImuDevice::begin(ImuConfig)`；
 *          输出：返回码、读回的型号名、实际 ODR、写寄存器次数。
 * @theory  N/A（驱动接入验证，不是滤波/控制算法）
 * @ref     lib/ICM42688/src/ICM42688.cpp（begin/reset/whoAmI/writeRegister）、
 *          lib/ImuAbstraction/src/ImuDevice.h（抽象接口与 WHO_AM_I 约定）
 * @frame   N/A
 * @units   N/A
 * @note    ★ 这条测试的价值在于：驱动内部的 `writeRegister()` 会 `delay(10)` 再
 *          **回读校验**，因此它不是"返回成功就算过"——只有寄存器文件真的保存了
 *          写入值，begin() 才能走完 FSR/ODR/LPF 的全部配置。用假后端永远测不到这段。
 *          仍不代表真实 SPI 时序/CS 建立时间/上电稳定时间。
 * @test    本文件即测试：退出码 0 = 通过。
 * @see     test_host/fw_sil/fullstack/README.md
 */

#include "Arduino.h"
#include "SPI.h"
#include "ImuDevice.h"
#include "imu_factory.h"

#include <cstdio>

namespace
{
int g_fail = 0;
void check(bool cond, const char *name)
{
  std::printf(cond ? "[PASS] %s\n" : "[FAIL] %s\n", name);
  if (!cond) ++g_fail;
}
}  // namespace

// ---- 本目标不链接 fw_sil_stubs.cpp（它会把整个 state_data 拖进来），
//      因此在这里提供该目标真正需要的三个全局定义。----
SPIClass SPI;
uint32_t g_sil_time_us = 0;
SilDelayHook g_sil_delay_hook = nullptr;
// state_data.cpp 里 IMU 的宿主替身（不链接 sensor_imu.cpp 时由本文件的桩提供）
ImuDevice *IMU = nullptr;

int main()
{
  std::printf("== 真实 IMU 驱动 begin() 验证（%s）==\n", compiledImuModelName());

  // ---- 寄存器级替身：预置身份寄存器，其余按读写语义保存 ----
  SPI.silResetRegs();
  SPI.silEnableRegMode(true);
  // ICM-42688-P：WHO_AM_I 在 bank0 0x75，期望 0x47（见 ImuDevice.h 对照表）。
  // ICM-45686 为 0x72/0xE9；当前构建若选中 45686，这里需按型号改，故先断言型号。
  const ImuModel model = compiledImuModel();
  if (model == ImuModel::ICM42688P)
  {
    SPI.silSetReg(0x75, 0x47);
  }
  else if (model == ImuModel::Icm45686)
  {
    SPI.silSetReg(0x72, 0xE9);
  }
  else
  {
    std::printf("[FAIL] 未知 IMU 型号\n");
    return 1;
  }

  ImuDevice *dev = createImuDevice(SPI, 0 /*cs 引脚编号，桩不关心*/, 24000000U);
  check(dev != nullptr, "createImuDevice 返回实例");
  if (dev == nullptr) return 1;

  // ---- 协议自检：手工按驱动同一时序读一次 WHO_AM_I ----
  // 若这里读得到而驱动读不到，说明问题在驱动的 bank/CS 语义，不在寄存器文件。
  {
    const uint8_t stored = SPI.silGetReg(0x75);
    SPI.transfer(static_cast<uint8_t>(0x75 | 0x80));
    const uint8_t manual = SPI.transfer(static_cast<uint8_t>(0));
    std::printf("  寄存器文件 0x75=0x%02X，手工按读时序得到 0x%02X\n", stored, manual);
    check(stored == 0x47 && manual == 0x47, "寄存器文件与读时序自洽（0x75=0x47）");
    // 手工读会留下 reading_ 状态，重新开一次事务避免污染驱动
    SPI.silEnableRegMode(true);
  }

  // ---- 真实 begin()：内部 reset → WHO_AM_I → PWR_MGMT0 → FSR/ODR/LPF ----
  ImuConfig cfg;  // 默认值即现役硬件参数
  const int rc = dev->begin(cfg);

  std::printf("  begin() 返回码 = %d\n", rc);
  std::printf("  型号名 = %s\n", dev->modelName());
  std::printf("  实际 accel ODR = %u Hz\n", static_cast<unsigned>(dev->actualAccelOdrHz()));
  std::printf("  SPI 写寄存器次数 = %u\n", static_cast<unsigned>(SPI.silWriteCount()));

  check(rc == 0, "真实驱动 begin() 成功（含 reset/WHO_AM_I/FSR/ODR/LPF 全流程）");
  check(dev->modelName() != nullptr && dev->modelName()[0] != '\0', "驱动报告了型号名");
  check(dev->actualAccelOdrHz() > 0, "驱动回报了实际 ODR");
  check(SPI.silWriteCount() > 0, "确实发生了寄存器写入（不是空跑）");

  // 回读校验：PWR_MGMT0 应被写成 0x0F（accel+gyro LN 模式）
  if (model == ImuModel::ICM42688P)
  {
    const uint8_t pwr = SPI.silGetReg(0x4E);  // UB0_REG_PWR_MGMT0
    std::printf("  回读 PWR_MGMT0 = 0x%02X（期望 0x0F）\n", pwr);
    check(pwr == 0x0F, "PWR_MGMT0 回读等于驱动写入值（寄存器文件真实生效）");
  }

  std::printf("\n%s（失败 %d 项）\n", g_fail == 0 ? "=== 全部通过 ===" : "=== 存在失败 ===",
              g_fail);
  return g_fail == 0 ? 0 : 1;
}