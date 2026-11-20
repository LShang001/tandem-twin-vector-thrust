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
#include <cmath>
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
    // ---- 模拟传感器"有数据就绪" ----
    // initImu() 采集 100 个样本时 `while (IMU->dataReady() != 1)`，判据是
    // INT_STATUS(0x2D) 的 bit3(DRDY)。寄存器文件若恒 0，这个循环永不退出
    // ——实测表现为 setup 卡死、30 s 超时。这里让 DRDY 常置位。
    SPI.silSetReg(0x2D, 0x08);
    // 14 字节突发数据（TEMP_DATA1=0x1F 起）：温度 2B + accel XYZ 各 2B + gyro XYZ 各 2B。
    // 静止姿态：RUB 坐标系 Y 轴朝上，故 accel Y ≈ +1g。±16g 量程下 2048 LSB/g。
    const uint8_t kFrame[14] = {
        0x00, 0x00,              // 温度
        0x00, 0x00,              // accel X
        0x08, 0x00,              // accel Y = 2048 → 1g
        0x00, 0x00,              // accel Z
        0x00, 0x00, 0x00, 0x00,  // gyro X/Y
        0x00, 0x00               // gyro Z
    };
    for (uint8_t i = 0; i < 14; ++i) SPI.silSetReg(static_cast<uint8_t>(0x1F + i), kFrame[i]);
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

  // ---------- FIFO 数值闭环：注入原始 LSB，经真实驱动换算后比对物理量 ----------
  // Packet3（16 B）：header + accel XYZ + gyro XYZ + temp + timestamp，全部大端。
  // header 判据见 lib/ICM42688/src/icm42688_fifo_packet.h::isPacket3Header。
  if (model == ImuModel::ICM42688P && IMU != nullptr)
  {
    constexpr int16_t kAccYRaw = 2048;   // ±8g 量程下 2048 LSB = 0.5 g
    constexpr int16_t kGyrXRaw = 820;    // ±2000dps 量程下 820 LSB ≈ 50 dps
    const uint8_t frame[16] = {
        0x68,                                  // header: 0x60 | bit3(timestamp) 合法
        static_cast<uint8_t>(0x00), static_cast<uint8_t>(0x00),  // accel X
        static_cast<uint8_t>((kAccYRaw >> 8) & 0xFF),
        static_cast<uint8_t>(kAccYRaw & 0xFF),                   // accel Y
        static_cast<uint8_t>(0x00), static_cast<uint8_t>(0x00),  // accel Z
        static_cast<uint8_t>((kGyrXRaw >> 8) & 0xFF),
        static_cast<uint8_t>(kGyrXRaw & 0xFF),                   // gyro X
        static_cast<uint8_t>(0x00), static_cast<uint8_t>(0x00),  // gyro Y
        static_cast<uint8_t>(0x00), static_cast<uint8_t>(0x00),  // gyro Z
        0x00,                                                    // temp
        0x00, 0x2A                                               // timestamp
    };
    SPI.silFifoClear();
    SPI.silFifoPush(frame, sizeof(frame));
    check(SPI.silFifoCount() == 16, "FIFO 替身已装入 1 帧（16 B）");

    ImuFifoSample sample{};
    const int n = IMU->readFifo(&sample, 1);
    std::printf("  readFifo 返回 %d 帧\n", n);
    check(n == 1, "真实 readFifo 读到 1 帧");

    std::printf("  acc = (%.4f, %.4f, %.4f) g\n", static_cast<double>(sample.acc[0]),
                static_cast<double>(sample.acc[1]), static_cast<double>(sample.acc[2]));
    std::printf("  gyr = (%.4f, %.4f, %.4f) dps\n", static_cast<double>(sample.gyr[0]),
                static_cast<double>(sample.gyr[1]), static_cast<double>(sample.gyr[2]));
    std::printf("  timestamp = %u\n", static_cast<unsigned>(sample.timestamp));

    // ±8g: 1/4096 g/LSB → 2048 LSB = 0.5 g
    // ±2000dps: 驱动用 32768/2000 = 16.384 LSB/dps（不是手册常见的 16.4 取整），
    //           所以 820 LSB = 50.048828 dps。期望值按驱动实际标度算，别按手册估。
    constexpr float kGyrLsbPerDps = 32768.0f / 2000.0f;
    check(std::fabs(sample.acc[1] - 0.5f) < 1e-3f, "accel Y 换算正确（2048 LSB → 0.5 g）");
    check(std::fabs(sample.acc[0]) < 1e-4f && std::fabs(sample.acc[2]) < 1e-4f,
          "accel X/Z 为零值");
    check(std::fabs(sample.gyr[0] - (820.0f / kGyrLsbPerDps)) < 5e-3f,
          "gyro X 换算正确（820 LSB → 50.0488 dps）");
    check(sample.timestamp == 0x002A, "timestamp 大端解析正确（0x002A）");
  }

  std::printf("\n%s（失败 %d 项）\n", g_fail == 0 ? "=== 全部通过 ===" : "=== 存在失败 ===",
              g_fail);
  return g_fail == 0 ? 0 : 1;
}