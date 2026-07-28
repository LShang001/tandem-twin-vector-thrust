/**
 * @file navigation_task.h
 * @brief 导航与状态估计模块：EKF组合导航、垂直/水平KF、GNSS处理
 *
 * 本模块实现飞控系统的状态估计核心，包含多层级数据融合：
 *
 *   1. EKF 15状态扩展卡尔曼滤波器 (主导航滤波器)
 *      - 状态: 四元数姿态(4) + NED速度(3) + NED位置(3) + 陀螺零偏(3) + 加速度零偏(3)
 *      - 预测: IMU 双子样积分
 *      - 更新: GNSS位置/速度、AHRS姿态量测、双矢量航向融合、ZUPT零速修正
 *
 *   2. 垂直卡尔曼滤波器 (2状态: 高度+速度)
 *      - 预测: IMU 垂直加速度
 *      - 更新: 激光测距高度 / 气压计高度 (按优先级切换)
 *
 *   3. 水平卡尔曼滤波器 (2轴各2状态: 位置+速度)
 *      - 预测: NED系水平加速度
 *      - 更新: 光流速度 (旋转到NED系)
 *
 *   4. 双矢量航向融合
 *      - 利用 GPS 地速矢量 + 光流机体速度矢量 几何解算真实航向
 *      - 作为 EKF 的航向量测输入，修正陀螺积分漂移
 */
#pragma once
#include "state_data.h"

/**
 * @brief 检查 GNSS 数据是否新鲜 (用于导航)
 *
 * 判断最后一次 UBX 数据接收时间是否在 GNSS_NAV_DATA_TIMEOUT_MS (300ms) 内。
 * 用于区分"新鲜GNSS"和"GNSS缓存数据"。
 *
 * @return true GNSS 数据新鲜, false GNSS 数据过期或未接收
 */
bool isGnssDataFreshForNav();

/**
 * @brief 检查 GNSS 是否即时有效 (用于导航)
 *
 * 综合判断条件：UBX 定位类型 >= 3D Fix 且数据新鲜。
 * 用于 EKF 测量更新和静止检测等需要高置信度 GNSS 的场景。
 *
 * @return true GNSS 即时有效, false 不可用
 */
bool isGnssInstantValidForNav();

/**
 * @brief 检查 GNSS 输出是否有效 (用于导航输出)
 *
 * 允许短时间保持最近一次有效 GNSS 状态 (GNSS_NAV_DROPOUT_HOLD_MS = 1000ms)，
 * 但不会把 EKF 假原点 LLA 误判为真实 GNSS。
 * 用于决定控制输出使用 EKF 姿态还是后台 AHRS。
 *
 * @return true GNSS 输出有效 (含短时保持), false 无效
 */
bool isGnssOutputValidForNav();

/**
 * @brief 将 UBX 定位类型映射到 DETA100 枚举
 *
 * BFS UBX Fix: 0=None, 1=2D, 2=3D, 3=DGNSS, 4=RTK-Float, 5=RTK-Fixed
 * DETA100 Enum: 1=NoFix, 2=2D, 3=3D, 4=DGPS, 5=RTK_Float, 6=RTK_Fixed
 *
 * @param ubx_fix UBX 定位类型枚举值
 * @return GPSFixType DETA100 兼容的定位类型枚举值
 */
GPSFixType mapUbxFixToDeta100FixType(bfs::Ubx::Fix ubx_fix);

/**
 * @brief 设置导航系统起飞点原点 (经纬高)
 *
 * 将指定的 WGS84 弧度制经纬高设置为导航原点 (Home Point)。解锁时以
 * 当前导航位置调用，使控制输出的相对 NED 位置和相对高度从 0 开始。
 * 同步更新 degree 和 radian 两套原点变量。
 *
 * @param lat_rad 纬度 (弧度)
 * @param lon_rad 经度 (弧度)
 * @param alt_m   大地高 (米)
 */
void setNavigationOriginFromLlaRad(const double lat_rad, const double lon_rad, const double alt_m);

/**
 * @brief 双矢量航向融合算法
 *
 * 利用 GPS 地速矢量和光流机体速度矢量几何解算真实航向：
 *   真实航向 = 航迹角(GPS) - 侧滑角(光流)
 *
 * 防御逻辑：
 * - 低速保护: 速度 < 1.0 m/s 时跳过 (atan2 对噪声敏感)
 * - 大机动保护: 偏航角速度 > 0.35 rad/s 时跳过 (时间同步误差放大)
 * - 尺度一致性: GPS/光流速度比差异 > 35% 时跳过 (光流高度可能有误)
 *
 * 动态噪声模型: 飞得越快，几何解算越准，EKF 越信任该量测。
 */
void handleDualVectorYawFusion();

/**
 * @brief 更新 UBX 原始相对位置
 *
 * 使用 UBX 原始 LLA 和导航原点，通过 BFS lla2ned 计算 NED 相对位移。
 * 仅在 GNSS 即时有效且导航已初始化时执行。
 */
void updateUbxRelativePosition();

/**
 * @brief EKF 组合导航主任务 (200Hz)
 *
 * 导航系统状态机，包含：
 * 1. IMU 数据降采样和双子样积分准备
 * 2. 统一静止检测 (加速度模长+角速度+相邻帧变化, 连续50帧≈250ms)
 * 3. EKF 初始化 (静止确认后，支持有/无 GNSS 两种模式)
 * 4. EKF 运行阶段:
 *    - 时间更新(预测): 始终执行
 *    - 姿态量测更新: 统一的静止辅助 + 条件性 AHRS 辅助
 *      · 静止时: 重力方向辅助 + ZUPT (10Hz, 有无GNSS都使用)
 *      · 无GNSS+运动时: AHRS 姿态量测 (利用重力融合)
 *      · 有GNSS+运动时: 不需要AHRS辅助 (EKF自身姿态估计足够)
 *    - GNSS位置/速度更新: 有GNSS时执行
 * 5. AHRS 航向偏置慢速校正
 * 6. 将 EKF 结果注入全局状态（绝对 LLA + 起飞点相对 NED/高度/速度）
 * 7. GNSS 原始状态映射到 Status_Packet
 */
void handleNavigationSystem();

/**
 * @brief 估算垂直速度
 *
 * 多源垂直速度估算，按优先级切换：
 * 1. GNSS/INS 融合速度 (最高质量，直接使用)
 * 2. 激光测距高度差分 + 低通滤波
 * 3. 气压计高度差分 + 低通滤波
 */
void updateEstimatedVerticalVelocity();

/**
 * @brief 垂直卡尔曼滤波任务 (200Hz)
 *
 * 2状态垂直KF (高度+速度)，融合 IMU 垂直加速度和高度传感器：
 * - 预测: 机体系加速度旋转到NED系，去重力，取天向分量
 * - 更新: 激光测距高度 (优先) / 气压计高度 (降采样到50Hz)
 * - 输出: estimated_height, estimated_velocity
 */
void handleVerticalEstimation();

/**
 * @brief 水平卡尔曼滤波任务 (200Hz)
 *
 * 双轴各2状态水平KF (位置+速度)，融合 IMU 水平加速度和光流速度：
 * - 预测: NED系北向/东向加速度
 * - 更新: 光流纯平移速度旋转到NED系 (动态噪声: 速度越快噪声越大)
 * - 输出: fused_north_pos/vel, fused_east_pos/vel
 * - 无GNSS时覆盖 relative_north/east 和 INS_GNSS_Packet 速度
 */
void handleHorizontalEstimation();
