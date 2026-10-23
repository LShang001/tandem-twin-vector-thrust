/**
 * @file    sensor_peripheral.h
 * @module  固件/传感器外设（DPS310 气压计、UART7 光流、TVC 角度传感器）
 * @brief   声明非 IMU 外设的初始化、周期任务与诊断查询接口
 *
 * 本模块负责所有非 IMU 的外设传感器驱动和数据处理：
 *   - DPS310 气压计: 温度/气压采集、海拔高度计算、滤波
 *   - MTF02P 光流传感器: 测距+光流速度采集、姿态倾斜补偿、旋转补偿
 *   - LQS48 光流传感器: 高精度光流+激光测距、融合高度速度解算
 *   - TVC 角度传感器: ADC 读取、滤波、TVC 闭环反馈
 *
 * @details 输入：IIC2（DPS310）、UART7（光流，自动识别 LQS48 / MTF-02P MSP / Micolink /
 *          MAVLink-APM）、ADC（TVC 角度）；
 *          处理：按各自协议分帧 → 倾斜/旋转补偿 → 中值+低通滤波 → 写全局状态；
 *          输出：baro_altitude、flow_data（速度与测距两通道）、TVC 反馈角。
 * @theory  光流速度解算（几何投影，非线性动力学，非教材公式）：
 *          - 垂直高度: H = S·|cos(tilt)|（倾斜补偿，S=斜距）
 *          - 总视在速度: V_total = ω_flow·H（光流角速度 × 高度）
 *          - 旋转补偿: V_rot = H·ω_imu（IMU 角速度引起的视在速度）
 *          - 纯平移速度: V_trans = V_total − V_rot
 *          N/A 教材出处；实现自洽性见本文件与 src/sensor_peripheral.cpp 的同名注释。
 * @ref     AGENTS.md 踩坑记录（光流速度/测距两条独立新鲜度通道，2026-08-21）；
 *          AGENTS.md 硬件与串口（UART7 自动识别）；PROTOCOL-001 §二.3.2/§二.3.4
 *          （0x31 原始光流 / 0x34 通用测距语义，与本模块通道划分对应）
 * @frame   高度/速度先在机体系（FRD）做补偿，输出给 EKF 前由导航侧转 NED；
 *          TVC 角度为摆座系机械角（deg）
 * @units   高度 m、速度 m/s、光流角增量 rad、时间 dt s、角度 deg、气压 Pa、温度 °C
 * @note    ★ 光流速度与测距是**两条独立新鲜度通道**：测距帧不得刷新
 *          `flow_data.last_update_ms`（速度通道专属），否则光流停流后冻结速度会被
 *          EKF 当新测量反复融合；测距用 `flow_data.last_range_update_ms`。
 * @see     src/sensor_peripheral.cpp、src/state_data.h（OpticalFlowData/flow_data）
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
 * @brief UART7 光流传感器自动识别与调度任务 (500Hz)
 *
 * 上电后自动在 LQS48 (921600/115200) 与 MTF-02P (115200) 之间扫描：
 *   - LQS48: 0xFA 0xAA 自定义协议
 *   - MTF-02P Micolink: 0xEF 自定义协议
 *   - MTF-02P AUTO 版: 主动发送 ArduPilot 风格 MAVLink 心跳，解析 MAVLink
 *     OPTICAL_FLOW / DISTANCE_SENSOR 消息
 * 识别成功后锁定传感器类型，并分发到对应处理函数。该任务替换原先固定注册的
 * handleLQS48Flow，作为光流数据输入的唯一切入点。
 */
void handleOpticalFlowSensor();

// ---- 光流自动识别诊断信息（DBG `flow` 命令使用）----
uint8_t flowSensorGetDetectPhase();
uint32_t flowSensorGetDetectLoopCount();
uint32_t flowSensorGetMavFlowFrames();
uint32_t flowSensorGetMavDistFrames();
uint32_t flowSensorGetMspFlowFrames();
uint32_t flowSensorGetMspRangeFrames();
uint32_t flowSensorGetUartRxAvailSum();
uint32_t flowSensorGetUartRxReadSum();
void flowSensorDebugDump(HardwareSerial &out, uint32_t duration_ms);

/**
 * @brief MTF02P (Micolink 协议) 光流传感器数据处理
 *
 * 处理 MTF02P Micolink 数据流：
 * 1. 高度处理: 斜距 -> 四元数倾斜补偿 -> 垂直高度 -> 中值+低通滤波
 * 2. 速度处理: 归一化光流速度 -> 实际速度 -> IMU旋转补偿 -> 纯平移速度 -> 滤波
 *
 * 任务名与 LQS48 通道（handleLQS48Flow）严格区分，便于调度表/日志辨识传感器。
 */
void handleMTF02PFlow();

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
void handleMTF02PFlowTelemetry();

/**
 * @brief LQS48 光流遥测调试任务 (50Hz)
 *
 * 通过 Serial8 打印 LQS48 光流传感器的原始和处理后数据。
 * 输出 CSV 字段: 时间, 高度, 角增量XY, dt, 光流有效, 高度有效, 质量, 光照,
 *                 倾斜因子, 高度, IMU角速度XY, 斜距, 旋转补偿XY, 平移速度XY
 */
void handleLQS48FlowTelemetry();
