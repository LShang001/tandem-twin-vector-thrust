/**
 * @file    earth_model.h
 * @module  固件/导航（WGS-84 地球模型）
 * @brief   提供曲率半径、正常重力、LLA 变化率、地球自转率与导航系运输率
 * @details 输入：大地纬度/高度、NED 速度；处理：按 WGS-84 椭球计算 Rn/Rm、
 *          正常重力 g(φ,h)、ω_ie^n 与 ω_en^n；输出：标量或 3 维向量。
 *          全部 inline 实现，供 ekf_15_state.h 的传播、初始对准与零偏估计复用。
 *          不变量：所有函数无静态状态，可重入；角度默认 RAD。
 * @theory  卯酉圈/子午圈曲率半径 @ref REF-120 §7.2.1 表 7.2.1；
 *          正常重力 Somigliana 公式 (7.2.17) @ref REF-120 §7.2.4 (7.2.17) p.178；
 *          重力高度修正 (7.2.18) @ref REF-120 §7.2.4 (7.2.18) p.178；
 *          大地坐标变化率 @ref REF-120 §9.7 p.304–313（位置算法）；
 *          地球自转率/运输率 @ref REF-120 §7.4 (7.4.6) p.189；
 *          NED 速度方程 @ref REF-120 §9.8.1 (9.8.1) p.317–318
 * @frame   导航系 NED（北, 东, 地）；LLA=(纬度 rad, 经度 rad, 椭球高 m)
 * @units   长度 m，速度 m/s，角速度 rad/s，重力加速度 m/s²
 * @deviation 正常重力用级数式 g₀=9.780318(1+5.3024e-3 sin²φ−5.898e-6 sin²2φ)，
 *          与 (7.2.17) 的 Somigliana 闭式差约 1e-5 m/s²（常数项 9.780318 vs
 *          9.7803267714 差 8.7e-6，sin²2φ 系数 5.898e-6 vs 5.8e-6）。该量级远小于
 *          MEMS 加计零偏（~1e-2 m/s²），保留该实现并在核查记录登记。
 * @see     constants.h、ekf_15_state.h::ComputeEarthModelTerms()、
 *          docs/08-参考资料/notes/01-捷联惯导理论.md
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

/**
 * @brief 卯酉圈曲率半径 Rn（东西方向的弯曲半径）
 * @param[in] lat 大地纬度，单位由 ang 指定
 * @param[in] ang 纬度输入单位，默认 RAD
 * @return 卯酉圈曲率半径 [m]
 * @formula Rn = a/√(1−e²sin²φ)；东向距离换算 dx_E = (Rn+h)cosφ·dλ
 * @ref REF-120 §7.2.1 表 7.2.1（WGS-84 椭球参数）
 * @note 用于经度↔东向距离换算、ECEF 正算以及重力高度修正的平均半径。
 */
inline double earthrad_transverse_m(const double lat,
                                    const AngPosUnit ang = AngPosUnit::RAD)
{
  double rn = SEMI_MAJOR_AXIS_LENGTH_M / std::sqrt(1.0 - (ECC2 *
                                                          std::pow(std::sin(convang(lat, ang, AngPosUnit::RAD)), 2.0)));
  return rn;
}

/**
 * @brief 子午圈曲率半径 Rm（南北方向的弯曲半径）
 * @param[in] lat 大地纬度，单位由 ang 指定
 * @param[in] ang 纬度输入单位，默认 RAD
 * @return 子午圈曲率半径 [m]
 * @formula Rm = a(1−e²)/(1−e²sin²φ)^(3/2)；北向距离换算 dx_N = (Rm+h)·dφ
 * @ref REF-120 §7.2.1 表 7.2.1（WGS-84 椭球参数）
 * @note 用于纬度↔北向距离换算、重力高度修正与 LLA 变化率的纬度率。
 */
inline double earthrad_meridonal_m(const double lat,
                                   const AngPosUnit ang = AngPosUnit::RAD)
{
  double rm = SEMI_MAJOR_AXIS_LENGTH_M * (1.0 - ECC2) / std::pow(1.0 - ECC2 * std::pow(std::sin(convang(lat, ang, AngPosUnit::RAD)), 2.0), 1.5);
  return rm;
}

/**
 * @brief 同时计算 {Rn, Rm}，避免重复的 sin/平方运算
 * @param[in] lat 大地纬度，单位由 ang 指定
 * @param[in] ang 纬度输入单位，默认 RAD
 * @return {卯酉圈半径, 子午圈半径}，单位 m
 * @formula 见 earthrad_transverse_m() 与 earthrad_meridonal_m()
 * @ref REF-120 §7.2.1 表 7.2.1
 * @note 供 llarate()/EKF 的 LlaRateGeometryTerms 使用；两半径必须同源同单位。
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

/**
 * @brief 平均曲率半径 R̄ = √(Rn·Rm)
 * @param[in] lat 大地纬度，单位由 ang 指定
 * @param[in] ang 纬度输入单位，默认 RAD
 * @return 平均曲率半径 [m]
 * @formula R̄ = √(Rn·Rm)
 * @ref REF-120 §7.2.4 (7.2.18) p.178（高度修正使用的等效地球半径）
 * @note 用于正常重力高度修正与大地水准面近似，平衡东西/南北弯曲差异。
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

/**
 * @brief WGS-84 正常重力 g(φ, h)
 * @param[in] lat 大地纬度，单位由 ang 指定
 * @param[in] alt WGS-84 椭球高 [m]（海平面以下为负，同一公式适用）
 * @param[in] ang 纬度输入单位，默认 RAD
 * @return 当地正常重力加速度 [m/s²]，NED 下沿 +D（向下为正）
 * @formula g₀(φ) = 9.780318·(1 + 5.3024e−3·sin²φ − 5.898e−6·sin²2φ)；
 *          g(φ,h) = g₀/(1 + h/R̄)²，R̄ = √(Rn·Rm)
 * @ref REF-120 §7.2.4 (7.2.17) p.178（Somigliana 闭式）；
 *      REF-120 §7.2.4 (7.2.18) p.178（高度修正 g = g₀(Re/(Re+h))²）
 * @deviation 本实现用 9.780318 的级数式而非 (7.2.17) 的闭式：常数项差
 *            8.7e−6 m/s²，sin²2φ 项系数差约 1e−6 m/s²，合成量级 ~1e−5 m/s²。
 *            与 MEMS 加计零偏（~1e−2 m/s²）相比可忽略，与 MATLAB gravity.m 一致，
 *            保留并与 ekf_15_state.h::ComputeEarthModelTerms() 严格同式。
 * @note 高度修正用反平方形式而非一阶线性 g₀(1−2h/R)：h<0（海平面以下）时
 *       仍单调正确，避免高度通道系统性偏差；调用方必须传绝对椭球高，不能传
 *       相对起飞点高度。
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

/**
 * @brief 由 LLA 三维向量直接计算正常重力（LLA=[纬度,经度,高]）
 * @param[in] lla (纬度, 经度, 椭球高)
 * @param[in] ang 角度输入单位，默认 RAD
 * @return 正常重力 [m/s²]
 * @formula 见 normalgravity_mps2(lat, alt)
 * @ref REF-120 §7.2.4 (7.2.17)(7.2.18) p.178
 */
inline double normalgravity_mps2(const Eigen::Vector3d &lla,
                                 const AngPosUnit ang = AngPosUnit::RAD)
{
  return normalgravity_mps2(lla(0), lla(2), ang);
}

// ====================================================================================
// 大地坐标变化率（LLA Rate）
// ====================================================================================

/**
 * @brief 由 NED 速度计算大地坐标变化率（纬度率/经度率/高度率）
 * @param[in] vn NED 北向速度 [m/s]
 * @param[in] ve NED 东向速度 [m/s]
 * @param[in] vd NED 地向速度 [m/s]（向下为正）
 * @param[in] lat 大地纬度，单位由 ang 指定
 * @param[in] alt WGS-84 椭球高 [m]
 * @param[in] ang 角度输入/输出单位，默认 RAD
 * @return [dφ/dt, dλ/dt, dh/dt]；前两项单位随 ang（默认 rad/s），第三项 m/s
 * @formula dφ/dt = v_N/(Rm+h)；dλ/dt = v_E/((Rn+h)·cosφ)；dh/dt = −v_D
 * @ref REF-120 §9.7 p.304–313（位置算法）；REF-120 §9.8.1 (9.8.5a–c) p.317–322
 * @note 经度率除以 cosφ 补偿经圈在高纬度的收缩；纬度接近极点时该项发散，
 *       调用方须先用 IsValidLlaForEarthModel() 之类门限拒绝极区输入。
 * @note 高度率取 −v_D：NED 的 D 向下为正，而椭球高向上为正，符号必须取反。
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

/**
 * @brief 地球自转角速度在 NED 导航系下的投影 ω_ie^n
 * @param[in] lat 大地纬度，单位由 ang 指定
 * @param[in] ang 纬度单位，默认 RAD
 * @return ω_ie 的 NED 三分量 [rad/s]
 * @formula ω_ie^n = [ω_e·cosφ, 0, −ω_e·sinφ]（北, 东, 地）
 * @ref REF-120 §7.2.4 p.178（地球自转角速度）；REF-120 §7.4 (7.4.6) p.189
 *      （比力方程中的科氏项 2ω_ie×V）
 * @note 静止时陀螺仍会测到该角速度：Initialize() 把 earthrate()+navrate() 作为
 *       “物理期望角速度”从初始陀螺读数中扣除，避免把地球自转误当零偏；
 *       传播中的导航系旋转补偿用 −(ω_ie+ω_en)·dt 构造四元数增量。
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

/**
 * @brief 导航系运输速率 ω_en^n（NED 框架相对地球固连系的转动）
 * @param[in] vn NED 北向速度 [m/s]
 * @param[in] ve NED 东向速度 [m/s]
 * @param[in] vd NED 地向速度 [m/s]（不参与计算，仅为接口对称保留）
 * @param[in] lat 大地纬度，单位由 ang 指定
 * @param[in] alt WGS-84 椭球高 [m]
 * @param[in] ang 角度输入/输出单位，默认 RAD
 * @return ω_en^n 的 NED 三分量 [rad/s]
 * @formula ω_en^n = [v_E/(Rn+h), −v_N/(Rm+h), −v_E·tanφ/(Rn+h)]（北, 东, 地）
 * @ref REF-120 §7.4 (7.4.6) p.189（比力方程中的 (2ω_ie+ω_en)×V 项）；
 *      REF-120 §9.8.1 p.317–318（NED 速度误差方程的传输率项）
 * @note 与 earthrate() 共同构成 ω_in^n = ω_ie^n + ω_en^n；比力方程中的 −(2ω_ie
 *       + ω_en)×V 与姿态传播的“导航系旋转”都消费该向量。
 * @note 姿态误差线性化用的 ∂ω_en/∂v（M 矩阵）见
 *       ekf_15_state.h::PropagateStateFromIncrements() 的 F(β,V) 块：
 *       M = [0, 1/(Rn+h), 0; −1/(Rm+h), 0, 0; 0, −tanφ/(Rn+h), 0]。
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
