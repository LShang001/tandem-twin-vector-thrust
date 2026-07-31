/*
 * WGS-84 地球模型
 * ===============
 *
 * 实现所有与 WGS-84 参考椭球相关的地球物理计算，是捷联惯导解算中
 * "从 NED 速度/位置反推地球响应"的基础模块。核心功能如下：
 *
 * - 卯酉圈曲率半径 Rn(φ)：东西方向弯曲程度，用于经度 → 东向距离换算。
 * - 子午圈曲率半径 Rm(φ)：南北方向弯曲程度，用于纬度 → 北向距离换算。
 * - 平均曲率半径 sqrt(Rn·Rm)：用于重力高度修正等需要"等效球面半径"的场景。
 *
 * - WGS-84 正常重力模型：先按 Somigliana 公式计算椭球面重力 g₀(φ)，
 *   再用反平方形式 g(h) = g₀ / (1 + h/Rm)² 做高度修正。正负高度走同一公式，
 *   避免海平面以下 GNSS 场景引入系统性高度通道误差。与 MATLAB gravity.m 一致。
 *
 * - LLA 变化率 (llarate)：由 NED 速度反算纬经高对时间的导数。
 *   dφ/dt = vN/(Rm+h)，dλ/dt = vE/((Rn+h)cosφ)，dh/dt = -vD。
 *
 * - 地球自转速率 (earthrate) 在 NED 下的投影：
 *   ω_N = ωᴇ cos(φ)，ω_E = 0，ω_D = -ωᴇ sin(φ)。
 *   用于捷联惯导中扣除地球自转对陀螺读数的贡献。
 *
 * - 导航系运输速率 (navrate)：载体在地球曲面移动时 NED 框架自身的转动。
 *   ω_N = vE/(Rn+h)，ω_E = -vN/(Rm+h)，ω_D = -vE·tan(φ)/(Rn+h)。
 *   与 earthrate 一起构成 ω_in^n = ω_ie^n + ω_en^n，用于姿态微分方程。
 *
 * 坐标系约定：
 * - 大地坐标 LLA = [纬度, 经度, 高度]，默认 RAD 输入（全库统一），
 *   传度数需显式指定 AngPosUnit::DEG。
 * - NED 速度 [北, 东, 地]，单位 m/s，地向为正。
 * - 所有函数为 inline 实现，在头文件中编译期内联，零调用开销。
 */

#ifndef NAVIGATION_SRC_EARTH_MODEL_H_ // NOLINT
#define NAVIGATION_SRC_EARTH_MODEL_H_

#include <cmath>
#include <array>
#include "units.h"     // NOLINT
#include "constants.h" // NOLINT
#include "eigen.h"     // NOLINT
#include "Eigen/Dense"

namespace bfs
{

// ====================================================================================
// 曲率半径计算
// ====================================================================================

/*
 * 卯酉圈曲率半径 (Radius of Curvature in Prime Vertical / Transverse Radius)
 *
 * 描述 WGS-84 椭球沿东西（东-西 / 横向）方向的弯曲程度。公式：
 *   Rn = a / sqrt(1 - e² sin²(φ))
 *
 * 其中 a 为半长轴，e² 为第一偏心率平方，φ 为大地纬度。
 * Rn 用于将经度变化转为东向距离：dx_E = (Rn + h) * cos(φ) * dλ。
 *
 * @param lat  大地纬度，单位由 ang 参数指定。
 * @param ang  纬度输入单位，默认弧度 (RAD)，可指定为度 (DEG)。
 * @return     卯酉圈曲率半径，单位 m。
 */
inline double earthrad_transverse_m(const double lat,
                                    const AngPosUnit ang = AngPosUnit::RAD)
{
  double rn = SEMI_MAJOR_AXIS_LENGTH_M / std::sqrt(1.0 - (ECC2 *
                                                          std::pow(std::sin(convang(lat, ang, AngPosUnit::RAD)), 2.0)));
  return rn;
}

/*
 * 子午圈曲率半径 (Radius of Curvature in Meridional Direction)
 *
 * 描述 WGS-84 椭球沿南北（北-南 / 纵向）方向的弯曲程度。公式：
 *   Rm = a(1 - e²) / (1 - e² sin²(φ))^(3/2)
 *
 * Rm 用于将纬度变化转为北向距离：dx_N = (Rm + h) * dφ。
 *
 * @param lat  大地纬度，单位由 ang 参数指定。
 * @param ang  纬度输入单位，默认弧度 (RAD)，可指定为度 (DEG)。
 * @return     子午圈曲率半径，单位 m。
 */
inline double earthrad_meridonal_m(const double lat,
                                   const AngPosUnit ang = AngPosUnit::RAD)
{
  double rm = SEMI_MAJOR_AXIS_LENGTH_M * (1.0 - ECC2) / std::pow(1.0 - ECC2 * std::pow(std::sin(convang(lat, ang, AngPosUnit::RAD)), 2.0), 1.5);
  return rm;
}

/*
 * 同时计算卯酉圈和子午圈曲率半径，返回 {Rn, Rm} 数组。
 * 适用于需要同时使用两个半径的场景（如 LLA 变化率计算），避免重复的正弦/平方运算。
 */
inline std::array<double, 2> earthrad_m(const double lat,
                                        const AngPosUnit ang =
                                            AngPosUnit::RAD)
{
  std::array<double, 2> ret;
  ret[0] = earthrad_transverse_m(lat, ang);
  ret[1] = earthrad_meridonal_m(lat, ang);
  return ret;
}

/*
 * 平均曲率半径：Rm_mean = sqrt(Rn * Rm)。
 * 用于正常重力高度修正、大地水准面近似等需要"等效球面半径"的场景，
 * 平衡了东-西和北-南两个方向的弯曲差异。
 */
inline double earthrad_mean_m(const double lat,
                              const AngPosUnit ang = AngPosUnit::RAD)
{
  const double rn = earthrad_transverse_m(lat, ang);
  const double rm = earthrad_meridonal_m(lat, ang);
  return std::sqrt(rn * rm);
}

// ====================================================================================
// WGS-84 正常重力模型
// ====================================================================================

/*
 * WGS-84 正常重力模型，计算指定大地坐标处的重力加速度。
 *
 * 该模型分两步：
 * 1. 先按 Somigliana 公式计算椭球面上的重力 g₀(φ)：
 *      g₀(φ) = 9.780318 * (1 + 0.0053024 sin²φ - 0.000005898 sin²(2φ))
 *    此公式仅依赖纬度，不考虑经度和高度。
 *
 * 2. 再用反平方形式做高度修正：
 *      g(h) = g₀ / (1 + h / Rm)^2
 *    其中 Rmean = sqrt(Rn * Rm) 为当地平均曲率半径。
 *
 * 高度修正的物理含义：
 * - 重力随高度增加减弱（离地心更远）。
 * - 反平方形式 g ∝ 1/r² 比简单线性的 g₀ * (1 - 2h/R) 更精确，且在 h < 0
 *   （海平面以下 GNSS 场景）时正确保持单调递增。
 * - 本公式与 MATLAB 项目 gravity.m 保持一致，确保仿真验证的数值闭环。
 *
 * @param lat   大地纬度，单位由 ang 参数指定。
 * @param alt   WGS-84 椭球高，单位 m。
 * @param ang   纬度输入单位，默认弧度 (RAD)，可指定为度 (DEG)。
 * @return      当地正常重力加速度，单位 m/s²。
 */
inline double normalgravity_mps2(const double lat, const double alt,
                                 const AngPosUnit ang = AngPosUnit::RAD)
{
  const double lat_rad = convang(lat, ang, AngPosUnit::RAD);
  const double sin_lat = std::sin(lat_rad);
  const double sin_lat2 = sin_lat * sin_lat;
  const double sin_2lat = std::sin(2.0 * lat_rad);

  // 椭球面上的重力 g₀（Somigliana 公式）
  const double g0 = 9.780318 *
                    (1.0 + 5.3024e-3 * sin_lat2 -
                     5.898e-6 * sin_2lat * sin_2lat);

  const double r0 = earthrad_mean_m(lat_rad, AngPosUnit::RAD);

  /*
   * 高度修正统一使用反平方形式：
   *   g(h) = g0 / (1 + h / R)²
   * 对 h < 0 的一阶展开应为 g0 * (1 - 2h / R)，不能写成 g0 * (1 + h / R)。
   * 这里正负高度走同一公式，避免海平面以下 GNSS 场景引入系统性高度通道误差。
   */
  const double scale = 1.0 + alt / r0;
  return g0 / (scale * scale);
}

/*
 * 便利重载：直接从 LLA 三维向量调用正常重力模型。
 * LLA 格式：[纬度(rad), 经度(rad), 高度(m)]。
 */
inline double normalgravity_mps2(const Eigen::Vector3d &lla,
                                 const AngPosUnit ang = AngPosUnit::RAD)
{
  return normalgravity_mps2(lla(0), lla(2), ang);
}

// ====================================================================================
// 大地坐标变化率（LLA Rate）
// ====================================================================================

/*
 * 由 NED 系速度计算大地坐标的变化率（纬经高对时间的导数）。
 *
 * 公式：
 *   dφ/dt = v_N / (Rm + h)          （纬度的角速率，单位 rad/s，输出时按 ang 换算）
 *   dλ/dt = v_E / ((Rn + h) cos φ)  （经度的角速率，同上）
 *   dh/dt = -v_D                     （高度变化率 = 地向速度的负值，因为向为正）
 *
 * 物理含义：
 * - 北向速度 v_N 沿子午线方向移动，对应纬度变化。
 * - 东向速度 v_E 沿卯酉圈方向移动，对应经度变化（除以 cos φ 是补偿经圈在极点附近的
 *   收缩效应：高纬度时相同的东向距离对应更大的经度跨越）。
 * - 地向速度 v_D 直接改变高度（向下为正，所以符号取反）。
 *
 * @param vn     NED 北向速度，单位 m/s。
 * @param ve     NED 东向速度，单位 m/s。
 * @param vd     NED 地向速度，单位 m/s（向下为正）。
 * @param lat    大地纬度，单位由 ang 参数指定。
 * @param alt    WGS-84 椭球高，单位 m。
 * @param ang    纬度输入单位和输出单位，默认弧度 (RAD)。
 * @return       [纬度率, 经度率, 高度率]，其中纬度和经度率单位与 ang 一致，
 *               高度率单位始终为 m/s。
 */
inline Eigen::Vector3d llarate(const double vn, const double ve,
                               const double vd, const double lat,
                               const double alt,
                               const AngPosUnit ang = AngPosUnit::RAD)
{
  Eigen::Vector3d lla_dot;
  double Rns = earthrad_meridonal_m(lat, ang);
  double Rew = earthrad_transverse_m(lat, ang);
  lla_dot(0) = convang(vn / (Rns + alt), AngPosUnit::RAD, ang);
  lla_dot(1) = convang(ve / (Rew + alt) /
                           std::cos(convang(lat, ang, AngPosUnit::RAD)),
                       AngPosUnit::RAD, ang);
  lla_dot(2) = -vd;
  return lla_dot;
}

/* 便利重载：从 NED 速度向量和 LLA 向量直接计算。 */
inline Eigen::Vector3d llarate(const Eigen::Vector3d &ned_vel,
                               const Eigen::Vector3d &lla,
                               const AngPosUnit ang = AngPosUnit::RAD)
{
  return llarate(ned_vel(0), ned_vel(1), ned_vel(2), lla(0), lla(2), ang);
}

// ====================================================================================
// 地球自转速率与导航系运输速率
// ====================================================================================

/*
 * 地球自转角速度在 NED 导航系下的分量。
 *
 * 物理图像：
 * 地球绕 Z 轴以 ωᴇ rad/s 自转。在纬度 φ 处：
 * - NED 系的北向 (N) 分量：ωᴇ cos(φ)（地球自转在该处垂直于南北方向的分量）
 * - NED 系的东向 (E) 分量：0（地球自转方向恰好与东西方向一致，在东向上不可观测）
 * - NED 系的地向 (D) 分量：-ωᴇ sin(φ)
 *
 * 此向量用于捷联惯导解算中的旋转补偿：陀螺在静止时仍会测到地球自转，
 * EKF 需要先减去地球自转，才能正确分离出纯姿态变化。
 *
 * @param lat  大地纬度，单位由 ang 指定。
 * @param ang  纬度单位，默认弧度 (RAD)。
 * @return     地球自转角速度在 NED 系下的三维分量，单位 rad/s。
 */
inline Eigen::Vector3d earthrate(const double lat,
                                 const AngPosUnit ang = AngPosUnit::RAD)
{
  Eigen::Vector3d w = Eigen::Vector3d::Zero();
  // 北向分量：ωᴇ cos(φ)
  w(0) = convang(WE_RADPS * std::cos(convang(lat, ang, AngPosUnit::RAD)),
                 AngPosUnit::RAD, ang);
  // 东向分量：0
  w(1) = 0.0;
  // 地向分量：-ωᴇ sin(φ)
  w(2) = convang(-WE_RADPS * std::sin(convang(lat, ang, AngPosUnit::RAD)),
                 AngPosUnit::RAD, ang);
  return w;
}

/*
 * 导航系运输速率 (Transport Rate / Navigation Rate)。
 *
 * 描述 NED 导航系本身相对地球固联系 (ECEF) 的转动角速度，源于载体在地球曲面
 * 上移动时，导航系的"北-东-地"方向也在变化。公式：
 *
 *   ω_N =  v_E / (Rn + h)                 （绕北轴旋转 = 东向运动产生的向下载体速率）
 *   ω_E = -v_N / (Rm + h)                 （绕东轴旋转 = 北向运动产生的向下分量，负号是
 *                                           因为北移 → NED 框架绕 -E 轴转动）
 *   ω_D = -v_E tan(φ) / (Rn + h)          （绕地向轴旋转 = 经圈收敛率，飞机向东飞时
 *                                           经圈会收拢，导致 NED 框架绕 D 轴旋转，负号
 *                                           来自右手定则）
 *
 * 此向量与 earthrate() 一起构成导航系总转动率：
 *   ω_in^n = ω_ie^n + ω_en^n   （NED 系相对惯性系的总角速度）
 *
 * 在捷联惯导中，陀螺测到的是机体系相对惯性系的角速度 ω_ib^b。
 * 导航解算需要的机体系相对导航系的角速度：
 *   ω_nb^b = ω_ib^b - C_n^b (ω_ie^n + ω_en^n)
 *
 * @param vn     NED 北向速度，单位 m/s。
 * @param ve     NED 东向速度，单位 m/s。
 * @param vd     NED 地向速度，单位 m/s（本函数不使用 vd，仅保留接口一致性）。
 * @param lat    大地纬度，单位由 ang 指定。
 * @param alt    WGS-84 椭球高，单位 m。
 * @param ang    纬度输入与输出角度的单位，默认弧度 (RAD)。
 * @return       NED 导航系运输速率，单位 rad/s。
 */
inline Eigen::Vector3d navrate(const double vn, const double ve,
                               const double vd, const double lat,
                               const double alt,
                               const AngPosUnit ang = AngPosUnit::RAD)
{
  Eigen::Vector3d w;
  double rew = earthrad_transverse_m(lat, ang);
  double rns = earthrad_meridonal_m(lat, ang);
  w(0) = convang(ve / (rew + alt), AngPosUnit::RAD, ang);
  w(1) = convang(-vn / (rns + alt), AngPosUnit::RAD, ang);
  w(2) = convang(-ve * std::tan(convang(lat, ang, AngPosUnit::RAD)) /
                     (rew + alt),
                 AngPosUnit::RAD, ang);
  return w;
}

/* 便利重载：从 NED 速度向量和 LLA 向量直接计算。 */
inline Eigen::Vector3d navrate(const Eigen::Vector3d &ned_vel,
                               const Eigen::Vector3d &lla,
                               const AngPosUnit ang = AngPosUnit::RAD)
{
  return navrate(ned_vel(0), ned_vel(1), ned_vel(2), lla(0), lla(2), ang);
}

} // namespace bfs

#endif // NAVIGATION_SRC_EARTH_MODEL_H_ NOLINT
