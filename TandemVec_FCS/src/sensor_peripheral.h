/**
 * @file sensor_peripheral.h
 * @brief 外设传感器模块：气压计、光流传感器、TVC角度传感器
 *
 * 本模块负责所有非 IMU 的外设传感器驱动和数据处理：
 *   - DPS310 气压计: 温度/气压采集、海拔高度计算、滤波
 *   - MTF02P 光流传感器: 测距+光流速度采集、姿态倾斜补偿、旋转补偿
 *   - LQS48 光流传感器: 高精度光流+激光测距、融合高度速度解算
 *   - TVC 角度传感器: ADC 读取、滤波、TVC 闭环反馈
 *
 * 光流速度解算物理模型：
 *   - 垂直高度: H = S * |cos(tilt)| (倾斜补偿)
 *   - 总视在速度: V_total = ω_flow * H (光流角速度 * 高度)
 *   - 旋转补偿: V_rot = H * ω_imu (IMU 角速度引起的视在速度)
 *   - 纯平移速度: V_trans = V_total - V_rot
 */
#pragma once
#include "state_data.h"

/**
 * @brief 初始化 DPS310 气压计
 *
 * 配置 I2C 总线 (IIC2, 1MHz Fm+) 并初始化 DPS310：
 * - 地址选择引脚 (PE15) 拉高选择 0x77
 * - 温度测量速率: 1Hz, 过采样率: 4x
 * - 压力测量速率: 128Hz, 过采样率: 2x
 * - 启动连续测量模式并读取初始数据
 * - 计算初始海拔高度并设置零点偏移
 *
 * @return 0 成功, 负值为错误码 (-1002: 初始数据读取超时)
 */
int initDPS310();

/**
 * @brief 读取并处理 DPS310 气压数据
 *
 * 从 DPS310 获取最新的温度和压力数据，经滤波后计算海拔高度。
 * 高度计算: baro_altitude = calculateAltitudeSimplified(pressure) - offset
 *
 * @return true 数据读取成功, false 读取失败
 */
bool readDPS310Data();

/**
 * @brief 将当前气压高度设为新的相对高度零点
 *
 * 解锁时调用，使气压高度、滤波状态和后续 EKF 气压观测都以起飞点为 0 m。
 * 仅改变气压高度参考，不修改压力、温度或导航绝对位置。
 *
 * @return true 当前压力/温度有效且零点重置成功，false 输入无效
 */
bool resetBaroAltitudeReference();

/**
 * @brief DPS310 任务调度入口
 *
 * 被任务调度器调用，执行 readDPS310Data()。
 * 读取失败时仅对 FIFO 溢出或通信异常清空 FIFO；暂无新数据不清空。
 */
void handleDPS310();

/**
 * @brief 获取经滤波的 TVC 角度传感器读数
 *
 * 读取两个 ADC 角度传感器的模拟值，转换为角度并经互补滤波器平滑。
 * 传感器2的原始读数取反以对齐安装方向。
 *
 * @param[out] angle1 滤波后的 TVC 通道1角度 (度)
 * @param[out] angle2 滤波后的 TVC 通道2角度 (度)
 */
void getFilteredTVCAngles(float &angle1, float &angle2);

/**
 * @brief TVC 角度传感器调试任务 (250Hz)
 *
 * 读取 TVC 角度传感器并通过 Serial8 打印 CSV 格式的调试数据。
 * 输出字段: 时间, 舵机1输出, 舵机2输出, 传感器1角度, 传感器2角度, TVC1目标, TVC2目标
 */
void handleAngleSensors();

/**
 * @brief MTF02P 光流传感器数据处理
 *
 * 处理 MTF02P 光流传感器的完整数据流：
 * 1. 高度处理: 斜距 -> 四元数倾斜补偿 -> 垂直高度 -> 中值+低通滤波
 * 2. 速度处理: 归一化光流速度 -> 实际速度 -> IMU旋转补偿 -> 纯平移速度 -> 滤波
 */
void handleOpticalFlow();

/**
 * @brief LQS48 光流传感器数据处理 (500Hz)
 *
 * 处理 LQS48 光流传感器数据，使用融合高度 (estimated_height) 作为距离源：
 * 1. 高度处理: LQS48 激光测距 -> 倾斜补偿 (仅用于记录)
 * 2. 速度处理: 光流角增量/dt -> 融合高度逆投影斜距 -> IMU旋转补偿 -> 纯平移速度
 */
void handleLQS48Flow();

/**
 * @brief MTF02P 光流遥测调试任务 (50Hz)
 *
 * 通过 Serial8 打印 MTF02P 光流传感器的原始和处理后数据。
 * 输出 CSV 字段: 时间, 斜距, 原始速度XY, 测距有效, 倾斜因子, 高度,
 *                 光流有效, IMU角速度XY, 旋转补偿XY, 平移速度XY
 */
void handleFlowTelemetry();

/**
 * @brief LQS48 光流遥测调试任务 (50Hz)
 *
 * 通过 Serial8 打印 LQS48 光流传感器的原始和处理后数据。
 * 输出 CSV 字段: 时间, 高度, 角增量XY, dt, 光流有效, 高度有效, 质量, 光照,
 *                 倾斜因子, 高度, IMU角速度XY, 斜距, 旋转补偿XY, 平移速度XY
 */
void handleLQS48FlowTelemetry();
