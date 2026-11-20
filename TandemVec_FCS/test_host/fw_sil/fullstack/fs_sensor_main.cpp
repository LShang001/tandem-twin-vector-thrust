/**
 * @file    fs_sensor_main.cpp
 * @module  固件/全链路数字孪生（fs：真实传感器模块 initImu 接线验证）
 * @brief   让真实 `src/sensor_imu.cpp` 的 `initImu()` 在宿主跑通
 * @details 被测：`src/sensor_imu.cpp`（initImu / initMagnetometer / readIMUData /
 *          handleImuTask），**一行不改**地链接进来；配合真实 ICM 芯片驱动与
 *          宿主 SPI 寄存器级替身。
 *          输入：寄存器文件预置 WHO_AM_I；
 *          处理：调用真实 `initImu()`，其内部走 SPI 引脚配置 → createImuDevice →
 *                芯片 begin()（reset/WHO_AM_I/FSR/ODR/LPF）→ Madgwick 初始化；
 *          输出：返回码、IMU 指针、型号名、实际 ODR。
 * @theory  N/A（驱动/模块接线验证，不评估滤波精度）
 * @ref     src/sensor_imu.cpp（initImu 180 行起）、src/imu_factory.cpp、
 *          lib/ICM42688/src/ICM42688_ImuDevice.cpp
 * @frame   N/A（本测试不解释姿态）
 * @units   Hz（ODR）
 * @note    ★ 与 fs_imu_begin_main.cpp 的区别：那条只验驱动，这条验**固件模块**
 *          ——包括 SPI 引脚重映射、型号装配、失败码透传、Madgwick 初始化顺序。
 *          仍未覆盖 FIFO 取帧与 IMU_Packet 数值换算（下一步）。
 * @test    本文件即测试：退出码 0 = 通过。
 * @see     test_host/fw_sil/fullstack/README.md
 */

#include "Arduino.h"
#include "SPI.h"
#include "sensor_imu.h"
#include "imu_factory.h"
#include "ImuDevice.h"
#include "MagCalStore.h"

#include <cstdio>
#include <unistd.h>

// 段在 main 之前崩（全局对象构造顺序）还是之后崩：用不经 stdio 的 write(2) 打点。
// 若连这行都看不到，说明崩在静态初始化阶段，而不是 initImu 里。
static void rawMark(const char *s)
{
  const char *p = s;
  size_t n = 0;
  while (p[n] != '\0') ++n;
  (void)!write(2, p, n);
}

// ---- 磁标定 Flash 持久化替身 ----
// 真实实现 lib/MagCal/MagCalStore.cpp 直接操作 STM32 FLASH 寄存器（FLASH->CR2、
// FLASH_SR_BSY 等），宿主没有这些外设。全链路目标不验证"标定参数能否掉电保存"，
// 因此在这里提供最小替身：读恒失败（没有已保存参数）、写恒成功。
// ★这意味着"标定值从 Flash 恢复"这条路径不在本目标的覆盖范围内。
namespace magcal {
bool storeSave(const MagCalParams &, uint16_t, float) { return true; }
bool storeLoad(MagCalParams &) { return false; }
}  // namespace magcal

namespace
{
int g_fail = 0;
void check(bool cond, const char *name)
{
  std::printf(cond ? "[PASS] %s\n" : "[FAIL] %s\n", name);
  if (!cond) ++g_fail;
}
}  // namespace

int main()
{
  rawMark("[fs_sensor] main entered\n");
  // 关键：被测代码若在 initImu/initMagnetometer 内部崩溃，带缓冲的 stdout 会
  // 把已经打印的内容一起丢掉，表现为"编译成功、运行无输出、退出码非 0"。
  // 这里改为无缓冲，让每一行都立刻落盘，便于区分"没跑到"和"跑到了但崩了"。
  std::setvbuf(stdout, nullptr, _IONBF, 0);

  std::printf("== 真实 sensor_imu.cpp 接线验证 ==\n");
  std::fflush(stdout);

  // ---- 寄存器级替身：预置身份寄存器 ----
  SPI.silResetRegs();
  SPI.silEnableRegMode(true);
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

  // ---- 真实 initImu() ----
  const int rc = initImu();
  std::printf("  initImu() 返回码 = %d\n", rc);
  check(rc == 0, "真实 initImu() 返回 0");

  check(IMU != nullptr, "IMU 全局指针已由 initImu 装配");
  if (IMU != nullptr)
  {
    std::printf("  型号名 = %s\n", IMU->modelName());
    std::printf("  实际 accel ODR = %u Hz\n", static_cast<unsigned>(IMU->actualAccelOdrHz()));
    check(IMU->actualAccelOdrHz() > 0, "固件模块回报了实际 ODR");
    check(compiledImuModelName() != nullptr && compiledImuModelName()[0] != '\0',
          "编译期型号名可用");
  }

  std::printf("  SPI 写寄存器次数 = %u\n", static_cast<unsigned>(SPI.silWriteCount()));
  check(SPI.silWriteCount() > 0, "配置过程确实写了寄存器");

  std::printf("\n%s（失败 %d 项）\n", g_fail == 0 ? "=== 全部通过 ===" : "=== 存在失败 ===",
              g_fail);
  return g_fail == 0 ? 0 : 1;
}