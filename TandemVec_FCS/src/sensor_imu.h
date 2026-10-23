/**
 * @file    sensor_imu.h
 * @module  固件/传感器（IMU 与磁力计）
 * @brief   声明 ICM42688 六轴 IMU、磁力计的初始化与高频数据采集接口
 * @details 输入：SPI/I2C 原始寄存器数据；处理：`initICM42688()` 配置滤波/量程并
 *          采集静止均值初始化滤波器，`readIMUData()` 完成校准→低通→RUB→FRD→
 *          增量累加→FLU 适配→Madgwick→四元数修正→打包；
 *          输出：`IMU_Packet`（FRD, rad/s, m/s²）、`backup_ahrs_*`、
 *          `imu_delta_*` 双子样增量缓冲。磁力计经 MagSensor 统一探测并对齐 FRD。
 * @theory  坐标变换 RUB→FRD→FLU 与四元数右乘修正 q_FRD_from_NED =
 *          q_FRD_from_FLU ⊗ q_FLU_from_NWU ⊗ q_NWU_from_NED
 *          @ref REF-120 §9.2.2 p.256（四元数与姿态阵）、@ref MOD-002 §3（分量序与右乘）；
 *          Madgwick 为梯度下降姿态滤波（工程实现，非秦书算法）
 * @frame   传感器 RUB → 机体系 FRD → Madgwick 输入 FLU/NWU → 输出 FRD/NED
 * @units   加速度 g（原始）/m/s²（输出），角速度 dps（原始）/rad/s（输出），
 *          时间 µs（时间戳）/s（积分步长），温度 °C
 * @redline 传感器轴映射（`bX=-sZ`、`bY=+sX`、`bZ=-sY`）——2026-08-07 轴置换的
 *          实机锚点，改前必须与实飞存档版逐字对照（见 TandemVec_FCS/AGENTS.md）
 * @note    本模块只维护 `backup_ahrs_*` 后台变量；`AHRS_Packet` 由导航输出桥统一
 *          写入，别在此处直接覆盖。磁力计馈入 Madgwick 的开关默认关闭。
 * @see     `sensor_imu.cpp`、`navigation_task.cpp`、`TandemVec_FCS/AGENTS.md`
 */
#pragma once
#include "state_data.h"

/**
 * @brief 初始化磁力计（IST8310 / QMC5883L 自动探测）
 *
 * 启动 I2C 总线 (Wire1, 400kHz) 并通过 MagSensor 自动探测磁力计型号：
 * 优先 IST8310 (0x0E，本机默认硬件)，未命中则尝试 QMC5883L (0x0D)。
 * 磁力计通过 I2C 总线 (PB7-SDA, PB6-SCL) 连接。
 *
 * @return true 初始化成功, false 未检测到任何受支持的磁力计
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
 * 11. 打包到 IMU_Packet（FRD, rad/s, m/s²）与 backup_ahrs_* 后台变量；
 *     AHRS_Packet 由 navigation_task.cpp 的输出桥统一写入
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
