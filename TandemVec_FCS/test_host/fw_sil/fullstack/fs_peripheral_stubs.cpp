/**
 * @file    fs_peripheral_stubs.cpp
 * @module  固件/全链路数字孪生（fs：外设与持久化替身）
 * @brief   只为"让 communication.cpp 单一目标链接通过"提供最底层外设符号
 * @details 被测：无（替身层）。这些符号在固件里由别的模块提供，与本次要验证的
 *          遥测帧构造无关：GNSS 配置（ubx_config.cpp）、MAVLink 计数
 *          （mavlink_bridge.cpp）、看门狗喂狗（kickIwdg）、磁标定 Flash 持久化。
 *
 *          替代原则：**如实回答硬件"没做成"**，不假装成功。
 *            - ubxAutoConfig 返回全 false（没有接收机应答）
 *            - ubxGnssSerialBegin 返回 false（没有真实串口 FIFO 可配）
 *            - storeLoad 返回 false（没有已保存的标定）
 *          这样上层走的是"外设失败"分支，而不是被伪造的成功掩盖。
 * @theory  N/A（外设替身）
 * @frame   N/A
 * @units  N/A
 * @note    ★本文件只服务 fs_telemetry 目标。该目标验证的是帧字节布局，
 *          GNSS 配置与黑匣子在宿主都不可证（见 README「明确不在覆盖范围」）。
 * @see     src/ubx_config.h、lib/MagCal/MagCalStore.h、test_host/fw_sil/fullstack/README.md
 */
#include "Arduino.h"
#include "ubx_config.h"
#include "MagCalStore.h"
#include "mag_reference.h"

// 看门狗喂狗：宿主无 IWDG（固件主程序里由 BFS_DISABLE_IWDG 绕开）。
void kickIwdg() {}

// GNSS 配置相关符号由真实 src/ubx_config.cpp 提供，本文件不再重复定义。
// MAVLink 发送计数由真实 src/mavlink_bridge.cpp 提供。\n
// 磁标定 Flash 持久化：宿主无 Flash 控制器，读恒失败、写恒"成功"。
namespace magcal
{
bool storeSave(const MagCalParams & /*p*/, uint16_t /*sampleCount*/, float /*residualRms*/)
{
  return true;
}
bool storeLoad(MagCalParams & /*p*/) { return false; }
bool storeErase() { return true; }
}  // namespace magcal
// 磁参考场：固件定义在 navigation_task.cpp:1019（= kDefaultMagReference）。
// 本目标不链接 navigation_task，但必须用**同一个默认值**，不能另编一个数，
// 否则调试台打印的磁倾角/总场与真机不一致。
namespace bfs
{
MagReferenceNed g_mag_reference = kDefaultMagReference;
}  // namespace bfs
// GNSS 数据是否新鲜：固件实现依赖导航任务内部的接收时刻与超时判据。
// 本目标不链接 navigation_task（会拖入 EKF 全链），故按**契约**返回 false：
// 宿主没有真实 GNSS 接收，数据必然不新鲜。上层因此走"GNSS 不可用"分支，
// 与真实环境一致，不会让遥测错误地报告一个假的新鲜定位。
bool isGnssDataFreshForNav() { return false; }