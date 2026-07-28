/*
 * 本文件基于 Bolder Flight Systems navigation 库深度重构而来。
 * 原始项目：https://github.com/bolderflight/navigation
 * 当前版本与原始代码已有重大差异。
 *
 * 组合导航/EKF 算法库 —— 统一入口头文件
 * =========================================
 *
 * 本文件是 navigation-main 库的顶层汇聚头文件。库内各模块的职责如下：
 *
 *   constants.h          WGS-84 椭球几何参数、地球自转角速度、引力常数。
 *                        这些是地球物理计算的数值基础，全库共享。
 *
 *   utils.h              角度归一化 (WrapTo2Pi/WrapToPi) 与斜对称矩阵构造
 *                        (Skew)。WrapTo2Pi 用于连续正角度表达，WrapToPi 用于
 *                        EKF 新息的角度环绕处理；Skew 用于叉积的矩阵化表示，
 *                        是姿态传播和状态转移矩阵构造中最常用的基元之一。
 *
 *   earth_model.h        卯酉圈/子午圈曲率半径（用于 LLA → 距离换算）、
 *                        WGS-84 正常重力模型（含 Somigliana 公式 + 反平方高度修正）、
 *                        LLA 变化率（NED 速度 → 纬经高对时间导数）、
 *                        地球自转速率在 NED 系下的分量（用于陀螺零偏估计和旋转补偿）、
 *                        导航系运输速率 (transport rate)（载体在地球曲面移动时
 *                        NED 框架本身的转动）。
 *
 *   transforms.h         坐标系与姿态表示的完整变换工具集：
 *                        欧拉角 ⇄ DCM ⇄ 四元数（ZYX Tait-Bryan 约定）、
 *                        LLA ⇄ ECEF（大地 ↔ 地心地固）、
 *                        ECEF ⇄ NED（地固 ↔ 局部切平面）、
 *                        LLA ⇄ NED（大地 ↔ 局部导航）。
 *                        其中 ECEF→LLA 使用 Olson 迭代法，单次修正亚米级精度。
 *
 *   tilt_compass.h       由三轴加速度计和三轴磁力计做初始姿态对准：
 *                        利用重力方向得到 roll/pitch，利用磁方向得到 yaw。
 *                        是 EKF Initialize() 的调平/定向基础。
 *
 *   aid_source_status.h  观测源生命周期管理器，统一记录 ZUPT、Gravity、
 *                        GNSS、Attitude、YawGsf、StaticGyro 六类辅助观测
 *                        的可用性、融合尝试、EKF 门控结果、累计拒绝次数和
 *                        超时状态。不依赖 Arduino、IMU 硬件或串口协议，
 *                        可独立用于主机回归测试。
 *
 *   ekf_15_state.h       15 状态误差状态卡尔曼滤波器 (Ekf15State)，是本库
 *                        的核心。完整实现了：
 *                        - 捷联惯导解算（四元数姿态推进 + 速度/位置积分）
 *                        - 双子样/常规 TimeUpdate + 协方差传播
 *                        - GNSS 延迟回放量测更新（含 NIS 门控、重捕获逻辑）
 *                        - 静止零速修正 (ZUPT)、重力方向量测、静止角速度量测
 *                        - 姿态辅助、标量航向辅助
 *                        - 协方差稳定化（对称化 + Gershgorin + LDLT + 特征值投影）
 *                        - GNSS 失联/重捕获协方差膨胀
 *                        - 主机回归测试接缝 (EKF_HOST_REGRESSION)
 *
 * 坐标系约定（全库统一）：
 * - 机体系：FRD (前-右-下, Front-Right-Down)
 * - 导航系：NED (北-东-地, North-East-Down)
 * - 大地坐标：LLA (纬度, 经度, WGS-84 椭球高)，角度单位默认 RAD（弧度）；
 *   全库所有带 AngPosUnit 参数的函数默认值统一为 RAD，传度数需显式指定 DEG
 * - 欧拉角顺序：[Yaw, Pitch, Roll]（ZYX Tait-Bryan 约定，绕 Z→Y→X 旋转）
 *
 * 编译宏控制（由 platformio.ini 的 -D 在编译期传入，实现性能/精度权衡）：
 *
 *   快速传播路径（float 代替 double，牺牲数值精度换实时性）：
 *     BFS_NAVIGATION_EMBEDDED_FAST_PROPAGATION
 *
 *   一阶近似开关（降低协方差计算量，适合 200 Hz 桌面/低动态场景）：
 *     BFS_NAVIGATION_EMBEDDED_FAST_FIRST_ORDER_PHI          Phi ≈ I+Fdt
 *     BFS_NAVIGATION_EMBEDDED_FAST_FIRST_ORDER_COVARIANCE   P 一阶递推
 *     BFS_NAVIGATION_EMBEDDED_STRUCTURED_SIMPSON_QC         结构化等价 Simpson
 *
 *   协方差稳定化控制：
 *     BFS_NAVIGATION_EMBEDDED_FAST_STABILIZE       快速检查 + 周期完整 LDLT
 *     BFS_NAVIGATION_EMBEDDED_FULL_STABILIZE_PERIOD 完整检查周期（帧数）
 *     BFS_NAVIGATION_EMBEDDED_PROPAGATION_STABILIZE_DIVIDER  传播后稳定化分频
 *     BFS_NAVIGATION_EMBEDDED_STATIC_AID_STABILIZE_DIVIDER   静止辅助后稳定化分频
 *     BFS_NAVIGATION_EMBEDDED_FAST_STATIC_AID_UPDATE         简化 Joseph 更新
 *
 *   GNSS 融合控制：
 *     BFS_NAVIGATION_EMBEDDED_DISABLE_GNSS_DELAY_REPLAY  关闭延迟回放（纯惯导压测）
 *     BFS_NAVIGATION_GNSS_DELAY_S                         GNSS 回溯延迟（默认 0.015 s）
 *
 *   诊断与 profile：
 *     BFS_NAVIGATION_EMBEDDED_PROFILE_ENABLED    传播分段计时
 *     EKF_HOST_REGRESSION                        暴露协方差/状态转移矩阵供 g++ 测试
 *
 * 使用示例（典型 200 Hz INS + GNSS 组合导航调用流程）：
 *
 *   #include "navigation.h"
 *
 *   bfs::Ekf15State ekf;
 *
 *   // 1. 设置传感器噪声参数
 *   ekf.accel_std_mps2(accel_noise);
 *   ekf.gyro_std_radps(gyro_noise);
 *   // ... 其他噪声参数 ...
 *
 *   // 2. 静基座初始对准 + 状态/协方差初始化
 *   ekf.Initialize(static_accel, static_gyro, pseudo_mag,
 *                  Eigen::Vector3f::Zero(), kOriginLlaRadM);
 *
 *   // 3. 主循环：每导航周期一次
 *   ekf.TimeUpdateTwoSample(delta_theta_1, delta_theta_2,
 *                           delta_v_1, delta_v_2, dt_s);
 *   //    IMU 传播后立即融合 GNSS（如果当前周期有新 epoch）
 *   auto result = ekf.MeasurementUpdateDetailed(gnss_vel, gnss_lla, dt_s);
 *   //    静止辅助
 *   ekf.MeasurementUpdateVelocityDetailed(Vector3f::Zero(), 0.08f, 0.10f);
 *   //    读取 EKF 状态用于控制和输出
 *   auto vel = ekf.ned_vel_mps();
 *   auto pos = ekf.lla_rad_m();
 *   auto att = ekf.quat();
 */

#ifndef NAVIGATION_SRC_NAVIGATION_H_ // NOLINT
#define NAVIGATION_SRC_NAVIGATION_H_

#include "aid_source_status.h" // NOLINT  观测源状态诊断管理
#include "constants.h"         // NOLINT  WGS-84 椭球与地球物理常量
#include "ekf_15_state.h"      // NOLINT  15 状态误差状态 EKF（核心）
#include "tilt_compass.h"      // NOLINT  加速度计+磁力计初始姿态对准
#include "earth_model.h"       // NOLINT  曲率半径、重力模型、地球自转/运输速率
#include "transforms.h"        // NOLINT  姿态表示互转 + LLA/ECEF/NED 坐标变换
#include "utils.h"             // NOLINT  角度归一化 (Wrap) + 斜对称矩阵 (Skew)

#endif // NAVIGATION_SRC_NAVIGATION_H_ NOLINT
