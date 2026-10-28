/**
 * @file    navigation_task.h
 * @module  固件/导航（状态估计任务入口）
 * @brief   声明 EKF 组合导航、垂直/水平 KF、双矢量航向与 GNSS 数据有效性接口
 * @details 输入：IMU/GNSS/气压/激光/光流/磁（MagSensor FRD 经门控，见 navigation_task.cpp）；
 *          处理：`handleNavigationSystem()` 主状态机（初始化→预测→辅助量测→
 *          GNSS 更新→输出桥），以及并行的 `handleVerticalEstimation()` /
 *          `handleHorizontalEstimation()` 独立滤波器；
 *          输出：`AHRS_Packet`、`INS_GNSS_Packet`、`Geodetic_Pos_Packet`、
 *          `estimated_height/velocity`、`relative_*`。
 *          不变量：EKF 初始化前只用后台 AHRS 填充姿态；DETA100 直出模式跳过
 *          EKF 输出桥；GNSS 新鲜度与质量门限分离（fresh / instant / output）。
 *
 *          滤波器组成（与实现一致的描述）：
 *          1) EKF 误差状态 **15 维**：位置(3)+速度(3)+失准角(3)+陀螺零偏(3)+
 *             加计零偏(3)；名义姿态四元数(4) 在库内单独维护，不占误差状态。
 *             预测：IMU 双子样增量；更新：GNSS 位置/速度、双矢量航向、ZUPT、
 *             重力方向、静止陀螺、AHRS 姿态、气压高度。
 *          2) 垂直 KF（2 状态：高度+速度）：预测用 IMU 天向加速度，更新**只用
 *             气压高度**（激光分支已 `if (0)` 禁用），输出 `vfk_*` 供遥测对照。
 *          3) 水平 KF（两轴各 2 状态：位置+速度）：预测用 NED 水平加速度，
 *             更新用光流速度；输出 `fused_*` 仅作后备，**不再覆盖** EKF 的
 *             `relative_*` 与 `INS_GNSS_Packet` 速度。
 * @theory  15 状态 ESKF（位置/速度/失准角/加计零偏/陀螺零偏）
 *          @ref REF-121 §2.2.1 p.33（KF 五方程）、§8.2.3 p.291（组合导航阻尼）；
 *          比力方程与速度/位置编排 @ref REF-120 §7.4 (7.4.6) p.189、§9.6 p.298；
 *          零速修正 @ref REF-120 §9.9.3 p.336（静基座观测条件）；
 *          姿态角约定 theta=-asin(R13) @ref MOD-002 §3
 * @frame   NED 导航系 / FRD 机体系；LLA 用 WGS84 弧度
 * @units   rad、rad/s、m、m/s、m/s²、ms（时间戳）、µs（`micros()`）
 * @redline 四元数 RK4/双子样姿态更新、theta=-asin(R13)、ZUPT 门限与
 *          GNSS R 地板（见行为保持红线清单与 EKF 踩坑记录）
 * @note    水平/垂直 KF 是独立并行估计器：控制律消费 EKF 输出，
 *          `vfk_*`/`fused_*` 仅作遥测对照与后备，勿反向覆盖 EKF 状态。
 * @see     `navigation_task.cpp`、`sensor_imu.h`、`lib/navigation-main/src/ekf_15_state.h`
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
 * 2. 统一静止检测 (加速度模长+角速度+相邻帧变化，进 20 帧≈100ms / 出 5 帧；
 *    GNSS 速度>0.8 m/s 或光流速度>1 m/s 可推翻，只出不进)
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

#ifdef FW_SIL_REAL_NAV
// 仅宿主 SIL 使用的内部观测点，用于解释导航初始化/静止检测状态；
// 生产固件不编译这些符号，也不参与控制/导航逻辑。
extern int g_sil_nav_debug_imu_count;
extern int g_sil_nav_debug_imu_invalid;
extern bool g_sil_nav_debug_imu_failed;
extern float g_sil_nav_debug_accel[3];
extern float g_sil_nav_debug_gyro[3];
// [0]=实际写入 EKF 的加计 Q、[1]=高通加速度 RMS [g]、[2]=估计器建议加计 Q、
// [3]=实际写入 EKF 的陀螺 Q
extern float g_sil_nav_debug_vib[5];
// 研究设施：在线改陀螺 Q 上界（0=用宏默认）
extern float g_sil_gyro_q_cap_radps;
// 研究设施：在线改自动延时应用死区 [s]（-1=用宏默认）
extern float g_sil_delay_deadband_s;
// 研究设施：在线开关 vib_high 否决（-1=用宏默认）
extern int g_sil_delay_vib_veto;
// 研究设施：vib_high 通道选择（-1=用宏默认）
extern int g_sil_vib_hf_source;
// 研究设施：在线设拟合窗最大 GNSS 缺口 [s]（-1=用宏默认）
extern float g_sil_delay_max_gap_s;
// 研究设施（§4.24ax）：应用量平滑时间常数 [s]（0=不平滑）
extern float g_sil_delay_apply_lpf_s;
// 研究设施（§4.24ax）：锁存首次应用量（1=冻结，构造零抖动对照）
extern int g_sil_delay_apply_freeze;
// 研究设施：应用侧残差门限 [m/s]（-1=用宏默认）
extern float g_sil_delay_max_resid_mps;
// 研究设施：搜索网格上界 [ms]（-1=用宏默认 150）
extern float g_sil_delay_grid_max_ms;
// 研究设施：τ̂ 与编译期先验的最大偏差门限 [ms]（-1=用宏默认）
extern float g_sil_delay_cap_ms;
extern bool g_sil_nav_debug_static;
extern float g_sil_imu_health_accel_range_mps2;
extern float g_sil_nav_debug_static_confidence;
extern uint32_t g_sil_nav_debug_static_frames;
extern bool g_sil_nav_debug_gnss_epoch;
extern bool g_sil_nav_debug_initialized;
extern bool g_sil_pre_stable;
extern bool g_sil_pre_ok;
#ifdef BFS_IMU_SATURATION_Q_INFLATE
extern float g_sil_imu_sat_q_inflate_mps2;
extern float g_sil_imu_sat_q_enter_s;
extern float g_sil_imu_sat_q_release_s;
#endif
extern float g_sil_pre_accel_norm;
extern float g_sil_pre_accel_std;
extern float g_sil_pre_gyro_norm;
extern float g_sil_pre_gyro_std;
#endif

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
 * - 更新: 气压计高度（50Hz 降采样）；激光测距分支当前被 `if (0)` 禁用，
 *   保留代码与 `LASER_NOISE_STD` 以便恢复
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
 * - 注意: 该输出仅作光流定位后备/遥测对照，**不再覆盖** EKF 的
 *   `relative_north/east` 与 `INS_GNSS_Packet` 速度（纯加速度积分会漂移）
 */
void handleHorizontalEstimation();
