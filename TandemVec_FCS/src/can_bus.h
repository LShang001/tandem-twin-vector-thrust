/**
 * @file can_bus.h
 * @brief CAN 总线通信模块
 *
 * 基于 MCP2515 (SPI2) + TJA1050 实现 CAN 2.0B 通信：
 * - 500 Kbps, 8 MHz 晶振, SPI2 (PB13/PB14/PB15), CS=PB12, INT=PD10
 * - 200Hz 周期轮发飞控状态数据 (IMU/导航/控制/系统)
 * - 主循环轮询接收 CAN 帧（不在中断中操作 SPI2）
 *
 * CAN ID 分配 (标准帧 11-bit):
 *   0x100~0x10F: IMU 原始数据
 *   0x110~0x11F: EKF 导航状态
 *   0x120~0x12F: 控制指令/输出
 *   0x130~0x13F: 系统状态
 *   0x700~0x7FF: 接收帧 (控制指令/参数/心跳)
 *
 * 详见 docs/电路拓扑参考.md §12.1
 */

#pragma once

#include <stdint.h>

// CAN 总线配置
#define CAN_BAUD_RATE        500E3      // 500 Kbps
#define CAN_CLOCK_FREQ       8E6        // MCP2515 晶振 8 MHz
#define CAN_SPI_FREQ         8E6        // SPI2 时钟 8 MHz (Mode0, MSBFIRST)

// 发送 CAN ID 定义 (本飞控 → 外部设备)
#define CAN_ID_IMU_GYRO      0x100  // 角速度 X/Y (float32 x2, rad/s)
#define CAN_ID_IMU_ACCEL     0x101  // 角速度Z + 加速度X (float32 x2)
#define CAN_ID_IMU_ACCEL_YZ  0x102  // 加速度 Y/Z (float32 x2, m/s²)
#define CAN_ID_NAV_POS       0x110  // NED位置 N/E (float32 x2, m)
#define CAN_ID_NAV_VEL       0x111  // NED速度 VN/VE (float32 x2, m/s)
#define CAN_ID_NAV_QUAT      0x112  // 预留: 四元数分帧
#define CAN_ID_CTRL_OUTPUT   0x120  // 控制输出 throttle/yaw (float32 x2)
#define CAN_ID_CTRL_OUTPUT_RP 0x121  // 控制输出 roll/pitch (float32 x2)
#define CAN_ID_CTRL_TARGET   0x122  // 预留: 目标姿态分帧
#define CAN_ID_SYS_STATUS    0x130  // 系统状态 roll/pitch (float32 x2, deg)
#define CAN_ID_SYS_HEIGHT    0x131  // 系统状态 yaw/height (float32 x2)

// 接收 CAN ID 定义 (外部设备 → 本飞控)
#define CAN_ID_RX_CMD        0x700  // 控制指令分帧起始ID (单帧最多 float32 x2)
#define CAN_ID_RX_PARAM      0x701  // 参数更新 (uint8 paramId, float32 value)
#define CAN_ID_RX_HEARTBEAT  0x7FF  // 心跳检测

/**
 * @brief 初始化 CAN 总线 (MCP2515 on SPI2)
 * @return 0=成功, -1=初始化失败
 *
 * 必须在 setup() 中所有传感器初始化之后调用，因为 SPI1 (IMU) 已在先初始化。
 */
int initCAN(void);

/**
 * @brief CAN 总线发送任务 (200Hz 周期调用)
 *
 * 轮发飞控状态数据帧。
 * 在 task_scheduler 中注册为 200Hz 任务。
 */
void handleCANBus(void);

// 诊断查询
uint32_t getCanSendErrorCount(void);
uint32_t getCanSendSuccessCount(void);
bool isCanInitialized(void);
