/**
 * @file    can_bus.h
 * @module  固件/通信（CAN 总线：MCP2515 + TJA1050）
 * @brief   声明 CAN 总线初始化、200Hz 状态广播任务与诊断查询接口
 * @details 输入：无（发送侧只读全局状态）；
 *          处理：初始化时注入独立 SPI2 实例并 500Kbps 建链；任务内先轮询接收、
 *                再按 8 帧轮发 1 帧（每帧 2×float32 = 8B 满载）；
 *          输出：11-bit 标准帧状态广播。
 *          不变量：帧内 float 为本机小端内存布局；接收只消费不解析（下行协议未定稿）。
 * @theory  N/A（总线驱动与外设配置）
 * @ref     docs/电路拓扑参考.md §12.1（引脚/SPI2/MCP2515 连接）、
 *          docs/C路径-参数集中与遥测结构化方案.md §3（消费方与缩放）
 * @frame   IMU/角速率/角加速度字段为 FRD 机体系；位置/速度为 NED 导航系；
 *          姿态角 roll/pitch 为 rad，yaw 为 [0,2π) rad
 * @units   float32 物理量直传（rad/s、m/s²、m、m/s、%），不做定点缩放；
 *          `CAN_ID_CTRL_OUTPUT` 的 yaw 分量为角加速度 rad/s²（gnc_tel 物理域）
 * @note    Flash 写页期间（`s_flashWriting` 与 50ms 窗口）整任务跳过，
 *          否则 MCP2515 超时 reset 会卡死 SPI2；详见 can_bus.cpp。
 * @see     src/can_bus.cpp、src/state_data.h（gnc_tel/icm_* 数据源）
 */

#pragma once

#include <stdint.h>

// CAN 总线配置
#define CAN_BAUD_RATE        500E3      // 500 Kbps
#define CAN_CLOCK_FREQ       8E6        // MCP2515 晶振 8 MHz
#define CAN_SPI_FREQ         8E6        // SPI2 时钟 8 MHz (Mode0, MSBFIRST)

// 发送 CAN ID 定义 (本飞控 → 外部设备)
// 字节序约定：float32 以 STM32 原生小端内存布局发送（直接 memcpy 内存字节），
// 接收端须按小端解析（如大端平台需字节交换）。
#define CAN_ID_IMU_GYRO      0x100  // 角速度 X/Y (float32 x2, rad/s)
#define CAN_ID_IMU_ACCEL     0x101  // 角速度Z + 加速度X (float32 x2)
#define CAN_ID_IMU_ACCEL_YZ  0x102  // 加速度 Y/Z (float32 x2, m/s²)
#define CAN_ID_NAV_POS       0x110  // NED位置 N/E (float32 x2, m)
#define CAN_ID_NAV_VEL       0x111  // NED速度 VN/VE (float32 x2, m/s)
#define CAN_ID_NAV_QUAT      0x112  // 预留: 四元数分帧
#define CAN_ID_CTRL_OUTPUT   0x120  // 控制输出 throttle/yaw (float32 x2)
#define CAN_ID_CTRL_OUTPUT_RP 0x121  // 控制输出 roll/pitch (float32 x2)
#define CAN_ID_CTRL_TARGET   0x122  // 预留: 目标姿态分帧
#define CAN_ID_SYS_STATUS    0x130  // 系统状态 roll/pitch (float32 x2, rad, EKF 输出)
#define CAN_ID_SYS_HEIGHT    0x131  // 系统状态 yaw/height (float32 x2, yaw: rad [0,2π), height: m)

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
