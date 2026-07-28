/**
 * @file sensor_imu.h
 * @brief IMU 传感器模块：初始化、数据采集与预处理
 *
 * 本模块负责板载 ICM42688 六轴惯性测量单元和 IST8310 磁力计的初始化与数据采集。
 * 数据处理流程：
 *   1. 读取原始传感器数据 (加速度g、角速度dps)
 *   2. IMU 自动校准（零偏补偿）
 *   3. 低通滤波降噪
 *   4. 坐标系变换 (传感器系RUB -> 机体系FRD -> Madgwick系FLU)
 *   5. Madgwick AHRS 姿态解算
 *   6. 四元数坐标系修正 (FLU/NWU -> FRD/NED)
 *   7. 打包到 IMU_Packet 和 AHRS_Packet 全局数据结构
 */
#pragma once
#include "state_data.h"

/**
 * @brief 初始化 IST8310 磁力计
 *
 * 启动 I2C 总线 (Wire1, 400kHz) 并初始化 IST8310 磁力计驱动。
 * 磁力计通过 I2C 总线 (PB7-SDA, PB6-SCL) 连接。
 *
 * @return true 初始化成功, false 未检测到传感器
 */
bool initMagnetometer();

/**
 * @brief 初始化 ICM42688 六轴 IMU
 *
 * 配置 SPI 总线并初始化 ICM42688，设置：
 * - 加速度计: ±8G 量程, 2kHz 采样率, 50Hz 低通滤波
 * - 陀螺仪: ±2000°/s 量程, 2kHz 采样率, 100Hz 低通滤波
 * - 使用 100 个加速度样本初始化 Madgwick 滤波器
 *
 * @return 0 成功, 负值为错误码 (-1001: 数据读取超时)
 */
int initICM42688();

/**
 * @brief 读取并处理 ICM42688 数据 (核心数据处理函数)
 *
 * 完整数据处理流水线：
 * 1. 计算采样时间差 (dt)
 * 2. 读取原始加速度(g)和角速度(dps)
 * 3. IMU 自动静止校准 (零偏和加速度尺度修正)
 * 4. 低通滤波 (互补滤波器)
 * 5. 传感器系RUB -> 机体系FRD 坐标变换
 * 6. 累加 IMU 增量数据供 EKF 双子样积分使用
 * 7. 机体系FRD -> Madgwick系FLU 坐标变换
 * 8. Madgwick AHRS 姿态更新 (6轴/9轴模式)
 * 9. 四元数坐标系修正: FLU/NWU -> FRD/NED
 * 10. 欧拉角提取和航向偏置修正
 * 11. 打包到 IMU_Packet 和 AHRS_Packet
 *
 * @return true 数据读取和处理成功, false 无新数据
 */
bool readIMUData();

/**
 * @brief ICM42688 任务调度入口 (2kHz)
 *
 * 被任务调度器调用，执行 readIMUData() 并根据结果控制黄灯状态。
 * IMU 读取失败时点亮黄灯作为告警指示。
 */
void handleICM42688();
