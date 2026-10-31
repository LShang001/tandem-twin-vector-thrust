/**
 * @file    imu_factory.cpp
 * @module  固件/传感器（IMU 装配层实现）
 * @brief   按编译期宏实例化对应 IMU 后端
 * @details 固件中唯一同时包含「抽象接口」与「具体芯片后端」的编译单元。
 *          通过 #if 只 include 被选中型号的头文件，因此换型不会把两套驱动
 *          同时拖进构建。
 *
 * @theory  N/A（装配）
 * @ref     lib/ImuAbstraction/src/ImuDevice.h、
 *          lib/ICM42688/src/ICM42688_ImuDevice.h、
 *          lib/ICM45686/src/ICM45686_ImuDevice.h
 * @frame   N/A
 * @units   N/A
 * @note    两个型号宏都未定义时默认走 ICM42688（现役硬件），保持既有行为不变；
 *          若同时定义两个宏，编译期直接 #error 报错，避免"改了宏但没生效"
 *          这类静默问题（TandemVec 曾出现"构建失败静默复用旧二进制"的教训）。
 * @see     imu_factory.h、platformio.ini
 */
#include "imu_factory.h"

#if defined(FCS_IMU_MODEL_ICM42688) && defined(FCS_IMU_MODEL_ICM45686)
#error "FCS_IMU_MODEL_ICM42688 与 FCS_IMU_MODEL_ICM45686 不能同时定义"
#endif

#if defined(FCS_IMU_MODEL_ICM45686)
#include "ICM45686_ImuDevice.h"
#else
#include "ICM42688_ImuDevice.h"
#endif

ImuModel compiledImuModel()
{
#if defined(FCS_IMU_MODEL_ICM45686)
  return ImuModel::Icm45686;
#else
  return ImuModel::ICM42688P;
#endif
}

const char *compiledImuModelName()
{
#if defined(FCS_IMU_MODEL_ICM45686)
  return "ICM-45686";
#else
  return "ICM-42688-P";
#endif
}

ImuDevice *createImuDevice(SPIClass &spi, uint8_t csPin, uint32_t spiHz)
{
  // 静态实例：嵌入式环境避免堆分配，且保证生命周期覆盖整个运行期。
  // 函数内 static 的构造在首次调用时完成（setup 阶段），此后指针恒定。
#if defined(FCS_IMU_MODEL_ICM45686)
  static ICM45686_ImuDevice dev(spi, csPin, spiHz);
#else
  static ICM42688_ImuDevice dev(spi, csPin, spiHz);
#endif
  return &dev;
}
