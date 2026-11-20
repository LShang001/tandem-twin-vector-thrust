/**
 * @file    fs_main_main.cpp
 * @module  固件/全链路数字孪生（fs：真实 main.cpp 初始化与主循环）
 * @brief   让真实 `src/main.cpp` 的 `setup()` 与 `loop()` 在宿主完整执行
 * @details 被测：`src/main.cpp`（589 行，含完整初始化顺序与真实任务注册表）、
 *          `src/navigation_task.cpp`、`src/sensor_imu.cpp`、`src/sensor_peripheral.cpp`、
 *          `src/communication.cpp`、`src/ubx_config.cpp` —— 全部**一行不改**链接进来。
 *          输入（唯一替身层）：
 *            - SPI 寄存器级替身（预置 ICM-42688-P WHO_AM_I=0x47、DRDY 常置位、
 *              一帧静止数据），否则 `initImu()` 失败会进 main.cpp:385 的
 *              `while(1){digitalToggle(LED_yellow); delay(200);}` 闪灯死循环；
 *            - `delay()` 时间推进钩子（推进虚拟时钟并触发 TIM8 tick），
 *              否则 `while(millis()-t0 < TIMEOUT)` 类等待永不退出。
 *          输出：setup/loop 是否返回、SPI 写寄存器次数、调度器统计可用性。
 * @theory  N/A（装配与初始化流程验证）
 * @ref     src/main.cpp（setup 顺序、BFS_ADD_TASK 任务注册、while(1) 故障分支）
 * @frame   N/A
 * @units   Hz（调度 tick）、µs（虚拟时间）
 * @note    ★这是全链路目标里覆盖面最广的一条：它把 14 个源文件里的 11 个一次性
 *          装配起来跑真实初始化流程。能证明"这些模块在同一进程里互不冲突、
 *          初始化顺序成立、任务表注册成功"；不能证明任何硬件时序。
 * @test    本文件即测试：退出码 0 = 通过。
 * @see     test_host/fw_sil/fullstack/README.md
 */

#include "Arduino.h"
#include "SPI.h"
#include "ImuDevice.h"
#include "task_scheduler.h"

#include <cstdio>

void setup();
void loop();

namespace
{
int g_fail = 0;
void check(bool cond, const char *name)
{
  std::printf(cond ? "[PASS] %s\n" : "[FAIL] %s\n", name);
  if (!cond) ++g_fail;
}

// 与 fs_scheduler_main.cpp 相同的时基：1 tick = 156.25 µs = 625/4 定点。
uint64_t g_abs_q4 = 0;
uint64_t g_next_tick_q4 = 625;

void advanceQ4(uint64_t dq4)
{
  g_abs_q4 += dq4;
  while (g_next_tick_q4 <= g_abs_q4)
  {
    g_sil_time_us = static_cast<uint32_t>(g_next_tick_q4 / 4ULL);
    g_next_tick_q4 += 625ULL;
    if (TaskTimer != nullptr) TaskTimer->silTick();
  }
}

/// delay() 钩子：固件里 delay 期间 TIM8 中断照常跑，这里照常推进虚拟时钟。
void delayHook(uint32_t us) { advanceQ4(static_cast<uint64_t>(us) * 4ULL); }

/// 预置 ICM-42688-P 寄存器：身份 + 数据就绪 + 一帧静止数据。
void primeImuRegisters()
{
  SPI.silResetRegs();
  SPI.silEnableRegMode(true);
  SPI.silSetReg(0x75, 0x47);  // WHO_AM_I
  SPI.silSetReg(0x2D, 0x08);  // INT_STATUS.DRDY 常置位
  // TEMP_DATA1(0x1F) 起 14 字节：温度 2B + accel XYZ + gyro XYZ（RUB 系 Y 朝上）
  const uint8_t frame[14] = {0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00,
                             0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  for (uint8_t i = 0; i < 14; ++i) SPI.silSetReg(static_cast<uint8_t>(0x1F + i), frame[i]);
}
}  // namespace

int main()
{
  std::setvbuf(stdout, nullptr, _IONBF, 0);
  std::printf("== 真实 main.cpp 初始化与主循环 ==\n");

  primeImuRegisters();
  g_sil_delay_hook = &delayHook;

  // ---------- setup() ----------
  std::printf("  执行 setup() ...\n");
  setup();
  std::printf("  setup() 已返回\n");
  check(true, "setup() 完整执行并返回（未进入故障死循环）");

  const unsigned writes = static_cast<unsigned>(SPI.silWriteCount());
  std::printf("  SPI 写寄存器次数 = %u\n", writes);
  check(writes > 0, "初始化过程确实配置了 IMU 寄存器");

  // ---------- loop() ----------
  std::printf("  执行 loop() ...\n");
  loop();
  std::printf("  loop() 已返回\n");
  check(true, "loop() 完整执行并返回");

  // ---------- 调度器可用性 ----------
  const uint32_t cpu = schedulerCpuPermille();
  std::printf("  调度器 CPU 负荷 = %u‰，窗口 = %u ms\n", static_cast<unsigned>(cpu),
              static_cast<unsigned>(schedulerWindowMs()));
  check(cpu < 1000U, "调度器统计接口可用且负荷 < 100%");

  std::printf("\n%s（失败 %d 项）\n", g_fail == 0 ? "=== 全部通过 ===" : "=== 存在失败 ===",
              g_fail);
  return g_fail == 0 ? 0 : 1;
}