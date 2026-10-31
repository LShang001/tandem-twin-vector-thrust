/**
 * @file    imu_factory.h
 * @module  固件/传感器（IMU 装配层）
 * @brief   按编译期宏选择 IMU 后端并创建实例
 * @details ★ 这是固件中**唯一**同时知道「抽象接口」与「具体芯片库」的位置，
 *          其余业务代码只 include "ImuDevice.h"。这样做的原因：
 *          - 传感器驱动与寄存器知识全部留在各自芯片库内；
 *          - 未被选中的芯片库不会被本 TU 引用，其头文件不会被包含、
 *            其 .cpp 也不会因本文件而编进来（PlatformIO 按 LDF 选择性编译库）。
 *
 * @theory  N/A（装配）
 * @ref     lib/ImuAbstraction/src/ImuDevice.h
 * @frame   N/A
 * @units   N/A
 * @note    型号由 platformio.ini 的宏决定（默认 ICM42688，保持现役行为不变）：
 *            -DFCS_IMU_MODEL_ICM42688    ; 默认
 *            -DFCS_IMU_MODEL_ICM45686    ; 换型时打开
 * @see     src/imu_factory.cpp、platformio.ini
 */
#pragma once

#include <Arduino.h>
#include <SPI.h>

#include "ImuDevice.h"

/// 当前编译期选定的型号（用于日志/遥测，避免运行期才暴露配置）。
ImuModel compiledImuModel();

/// 编译期选定的型号名（静态字符串）。
const char *compiledImuModelName();

/**
 * @brief 创建编译期选定型号的 IMU 实例。
 * @param spi    已配置好引脚的 SPI 总线
 * @param csPin  片选引脚
 * @param spiHz  SPI 时钟（两款芯片上限均 24MHz；现役配置 10MHz）
 * @return 指向静态实例的指针，永不返回 nullptr
 */
ImuDevice *createImuDevice(SPIClass &spi, uint8_t csPin, uint32_t spiHz);
