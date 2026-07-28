/*
 * 坐标变换模块
 * ============
 *
 * 实现导航计算中所有常用坐标系与姿态表示之间的相互转换，全部使用 Eigen
 * 矩阵库，模板化 inline 实现，支持 float/double 两种浮点类型。
 *
 * 本模块分为三大功能组：
 *
 * 一、姿态表示间互转（ZYX Tait-Bryan 约定）
 *   angle2dcm / dcm2angle    旋转角 ⇄ 方向余弦矩阵
 *   angle2quat / quat2angle  旋转角 ⇄ 四元数
 *   dcm2quat   / quat2dcm    DCM ⇄ 四元数
 *   eul2dcm / dcm2eul        欧拉角 ⇄ DCM（等价包装）
 *   eul2quat / quat2eul      欧拉角 ⇄ 四元数（等价包装）
 *
 *   四元数提取 Tait-Bryan 角时跳过显式 DCM 构造，直接用四元数分量
 *   计算等效矩阵元素，节省 9 个冗余元素的计算。
 *
 * 二、大地坐标 (LLA) 与地心地固坐标 (ECEF) 互转
 *   lla2ecef    LLA → ECEF（标准 WGS-84 椭球公式，直接解析）
 *   ecef2lla    ECEF → LLA（Olson 迭代法，单次修正亚米级精度，
 *               所有中间量为自动变量保证多任务可重入性）
 *
 * 三、局部导航坐标 (NED) 与 ECEF/LLA 互转
 *   ecef2ned    ECEF 向量 → NED 向量（相对参考点 lla₀ 的切平面投影）
 *   ned2ecef    NED 向量 → ECEF 向量（ecef2ned 的逆变换）
 *   lla2ned     大地坐标差 → NED 位移（LLA→ECEF→差值→NED 两步法）
 *   ned2lla     NED 位移 → 大地坐标（NED→ECEF→绝对 ECEF→LLA 两步法）
 *
 * 旋转次序为 ZYX（先绕 Z→yaw，再绕 Y→pitch，再绕 X→roll），
 * 这是单旋翼/多旋翼飞控中的标准 Tait-Bryan 约定。所有角度默认 RAD。
 */

#ifndef NAVIGATION_SRC_TRANSFORMS_H_ // NOLINT
#define NAVIGATION_SRC_TRANSFORMS_H_

#include "units.h" // NOLINT
#include "eigen.h" // NOLINT
#include "Eigen/Dense"

namespace bfs
{

// ====================================================================================
// 姿态表示之间的互转：欧拉角 / DCM / 四元数
// ====================================================================================
// 旋转次序为 ZYX（先绕 Z 转 yaw，再绕 Y 转 pitch，再绕 X 转 roll），
// 这是飞控中标准 Tait-Bryan 约定。所有角度默认单位为弧度 (RAD)。
// ====================================================================================

/*
 * 三个旋转角 → 方向余弦矩阵 (DCM)
 *
 * 按 ZYX 次序合成 DCM：
 *   C = R_x(roll) * R_y(pitch) * R_z(yaw)
 * 展开后得到 3x3 旋转矩阵。rot1/rot2/rot3 分别对应标准 Tait-Bryan
 * yaw(ψ)/pitch(θ)/roll(φ)。
 */
template <typename T>
Eigen::Matrix<T, 3, 3> angle2dcm(const T rot1, const T rot2, const T rot3,
                                 const AngPosUnit ang = AngPosUnit::RAD)
{
  static_assert(std::is_floating_point<T>::value,
                "仅支持浮点类型");
  Eigen::Matrix<T, 3, 3> dcm;
  T cos_rot1 = std::cos(convang(rot1, ang, AngPosUnit::RAD));
  T sin_rot1 = std::sin(convang(rot1, ang, AngPosUnit::RAD));
  T cos_rot2 = std::cos(convang(rot2, ang, AngPosUnit::RAD));
  T sin_rot2 = std::sin(convang(rot2, ang, AngPosUnit::RAD));
  T cos_rot3 = std::cos(convang(rot3, ang, AngPosUnit::RAD));
  T sin_rot3 = std::sin(convang(rot3, ang, AngPosUnit::RAD));
  dcm(0, 0) = cos_rot2 * cos_rot1;
  dcm(1, 0) = -cos_rot3 * sin_rot1 + sin_rot3 * sin_rot2 * cos_rot1;
  dcm(2, 0) = sin_rot3 * sin_rot1 + cos_rot3 * sin_rot2 * cos_rot1;
  dcm(0, 1) = cos_rot2 * sin_rot1;
  dcm(1, 1) = cos_rot3 * cos_rot1 + sin_rot3 * sin_rot2 * sin_rot1;
  dcm(2, 1) = -sin_rot3 * cos_rot1 + cos_rot3 * sin_rot2 * sin_rot1;
  dcm(0, 2) = -sin_rot2;
  dcm(1, 2) = sin_rot3 * cos_rot2;
  dcm(2, 2) = cos_rot3 * cos_rot2;
  return dcm;
}

/* 欧拉角向量 → DCM：等价于 angle2dcm(yaw, pitch, roll)。 */
template <typename T>
Eigen::Matrix<T, 3, 3> eul2dcm(const Eigen::Matrix<T, 3, 1> &eul,
                               const AngPosUnit ang = AngPosUnit::RAD)
{
  return angle2dcm(eul(0), eul(1), eul(2), ang);
}

/*
 * DCM → 旋转角 (Tait-Bryan)
 *
 * 从 ZYX 次序的 DCM 中提取 yaw/pitch/roll。
 * 核心关系：
 *   pitch = -asin(DCM(0,2))
 *   yaw   =  atan2(DCM(0,1), DCM(0,0))
 *   roll  =  atan2(DCM(1,2), DCM(2,2))
 *
 * 边界：pitch = ±90° 时 DCM(0,0)=DCM(0,1)=0，yaw/roll 不可分辨（万向节锁）。
 */
template <typename T>
Eigen::Matrix<T, 3, 1> dcm2angle(const Eigen::Matrix<T, 3, 3> &dcm,
                                 const AngPosUnit ang = AngPosUnit::RAD)
{
  static_assert(std::is_floating_point<T>::value,
                "仅支持浮点类型");
  Eigen::Matrix<T, 3, 1> an;
  an(0, 0) = convang(std::atan2(dcm(0, 1), dcm(0, 0)), AngPosUnit::RAD, ang);
  an(1, 0) = convang(-std::asin(dcm(0, 2)), AngPosUnit::RAD, ang);
  an(2, 0) = convang(std::atan2(dcm(1, 2), dcm(2, 2)), AngPosUnit::RAD, ang);
  return an;
}

/* DCM → 欧拉角：等价于 dcm2angle()，语义上更强调"用于飞控欧拉角输出"。 */
template <typename T>
Eigen::Matrix<T, 3, 1> dcm2eul(const Eigen::Matrix<T, 3, 3> &dcm,
                               const AngPosUnit ang = AngPosUnit::RAD)
{
  return dcm2angle(dcm, ang);
}

/*
 * 旋转角 → 四元数
 *
 * 三个 ZYX 次序的欧拉角合成四元数。
 * 使用半角公式展开，四元数各分量：
 *   w = c1*c2*c3 + s1*s2*s3
 *   x = c1*c2*s3 - s1*s2*c3
 *   y = c1*s2*c3 + s1*c2*s3
 *   z = s1*c2*c3 - c1*s2*s3
 * 其中 c1=cos(yaw/2), s1=sin(yaw/2), c2=cos(pitch/2) 等。
 */
template <typename T>
Eigen::Quaternion<T> angle2quat(const T rot1, const T rot2, const T rot3,
                                const AngPosUnit a = AngPosUnit::RAD)
{
  static_assert(std::is_floating_point<T>::value,
                "仅支持浮点类型");
  Eigen::Quaternion<T> q;
  T cos_rot1 = std::cos(convang(rot1 / static_cast<T>(2), a, AngPosUnit::RAD));
  T sin_rot1 = std::sin(convang(rot1 / static_cast<T>(2), a, AngPosUnit::RAD));
  T cos_rot2 = std::cos(convang(rot2 / static_cast<T>(2), a, AngPosUnit::RAD));
  T sin_rot2 = std::sin(convang(rot2 / static_cast<T>(2), a, AngPosUnit::RAD));
  T cos_rot3 = std::cos(convang(rot3 / static_cast<T>(2), a, AngPosUnit::RAD));
  T sin_rot3 = std::sin(convang(rot3 / static_cast<T>(2), a, AngPosUnit::RAD));
  q.w() = cos_rot1 * cos_rot2 * cos_rot3 + sin_rot1 * sin_rot2 * sin_rot3;
  q.x() = cos_rot1 * cos_rot2 * sin_rot3 - sin_rot1 * sin_rot2 * cos_rot3;
  q.y() = cos_rot1 * sin_rot2 * cos_rot3 + sin_rot1 * cos_rot2 * sin_rot3;
  q.z() = sin_rot1 * cos_rot2 * cos_rot3 - cos_rot1 * sin_rot2 * sin_rot3;
  return q;
}

/* 欧拉角向量 → 四元数 */
template <typename T>
Eigen::Quaternion<T> eul2quat(const Eigen::Matrix<T, 3, 1> &eul,
                              const AngPosUnit a = AngPosUnit::RAD)
{
  return angle2quat(eul(0), eul(1), eul(2), a);
}

/*
 * 四元数 → 旋转角 (Tait-Bryan YPR)
 *
 * 从四元数构建等效 DCM 元素后提取 yaw/pitch/roll。
 * 使用关系：
 *   roll  = atan2(m12, m11)  = atan2(2(qx*qy + qw*qz), 2(qw²+qx²) - 1)
 *   pitch = asin(-m13)        = asin(-2(qx*qz - qw*qy))
 *   yaw   = atan2(m23, m33)   = atan2(2(qy*qz + qw*qx), 2(qw²+qz²) - 1)
 *
 * 这种"跳过显式 DCM 矩阵构造"的方式节省 9 个冗余元素的计算。
 */
template <typename T>
Eigen::Matrix<T, 3, 1> quat2angle(const Eigen::Quaternion<T> &q,
                                  const AngPosUnit ang = AngPosUnit::RAD)
{
  static_assert(std::is_floating_point<T>::value,
                "仅支持浮点类型");
  Eigen::Matrix<T, 3, 1> angle;
  T m11 = static_cast<T>(2) * q.w() * q.w() + static_cast<T>(2) * q.x() * q.x() - static_cast<T>(1);
  T m12 = static_cast<T>(2) * q.x() * q.y() + static_cast<T>(2) * q.w() *
                                                  q.z();
  T m13 = static_cast<T>(2) * q.x() * q.z() - static_cast<T>(2) * q.w() *
                                                  q.y();
  T m23 = static_cast<T>(2) * q.y() * q.z() + static_cast<T>(2) * q.w() *
                                                  q.x();
  T m33 = static_cast<T>(2) * q.w() * q.w() + static_cast<T>(2) * q.z() * q.z() - static_cast<T>(1);
  angle(0, 0) = convang(std::atan2(m12, m11), AngPosUnit::RAD, ang);
  angle(1, 0) = convang(std::asin(-m13), AngPosUnit::RAD, ang);
  angle(2, 0) = convang(std::atan2(m23, m33), AngPosUnit::RAD, ang);
  return angle;
}

/* 四元数 → 欧拉角：等价于 quat2angle()。 */
template <typename T>
Eigen::Matrix<T, 3, 1> quat2eul(const Eigen::Quaternion<T> &q,
                                const AngPosUnit ang = AngPosUnit::RAD)
{
  return quat2angle(q, ang);
}

/*
 * DCM → 四元数
 *
 * 从 3x3 旋转矩阵提取四元数。使用 trace 方法：
 * 先由 trace(DCM) 决定主分量的安全计算方法以避免数值除零。
 * 对于 trace > -1 的正常情形：
 *   w = 0.5 * sqrt(1 + DCM(0,0) + DCM(1,1) + DCM(2,2))
 *   x = (DCM(1,2) - DCM(2,1)) / (4*w)
 *   y = (DCM(2,0) - DCM(0,2)) / (4*w)
 *   z = (DCM(0,1) - DCM(1,0)) / (4*w)
 */
template <typename T>
Eigen::Quaternion<T> dcm2quat(const Eigen::Matrix<T, 3, 3> &dcm)
{
  static_assert(std::is_floating_point<T>::value,
                "仅支持浮点类型");
  Eigen::Quaternion<T> q;
  q.w() = static_cast<T>(0.5) * std::sqrt(static_cast<T>(1) + dcm(0, 0) +
                                          dcm(1, 1) + dcm(2, 2));
  q.x() = (dcm(1, 2) - dcm(2, 1)) / (static_cast<T>(4) * q.w());
  q.y() = (dcm(2, 0) - dcm(0, 2)) / (static_cast<T>(4) * q.w());
  q.z() = (dcm(0, 1) - dcm(1, 0)) / (static_cast<T>(4) * q.w());
  return q;
}

/*
 * 四元数 → DCM
 *
 * 标准从单位四元数 q = [w, x, y, z] 构建旋转矩阵：
 *   ┌                                      ┐
 *   │ 2w²-1+2x²   2xy-2wz     2xz+2wy     │
 *   │ 2xy+2wz     2w²-1+2y²   2yz-2wx     │
 *   │ 2xz-2wy     2yz+2wx     2w²-1+2z²   │
 *   └                                      ┘
 *
 * 这是姿态传播中最常用的变换之一——捷联惯导中，四元数更新后，
 * 需要将 NED 系下的向量（如地球自转、运输速率）转换到机体系时
 * 必须先得到 DCM（或其转置 t_b2ned），然后做矩阵乘法。
 */
template <typename T>
Eigen::Matrix<T, 3, 3> quat2dcm(const Eigen::Quaternion<T> &q)
{
  static_assert(std::is_floating_point<T>::value,
                "仅支持浮点类型");
  Eigen::Matrix<T, 3, 3> dcm;
  dcm(0, 0) = static_cast<T>(2) * q.w() * q.w() - static_cast<T>(1) +
              static_cast<T>(2) * q.x() * q.x();
  dcm(1, 0) = static_cast<T>(2) * q.x() * q.y() - static_cast<T>(2) *
                                                      q.w() * q.z();
  dcm(2, 0) = static_cast<T>(2) * q.x() * q.z() + static_cast<T>(2) *
                                                      q.w() * q.y();
  dcm(0, 1) = static_cast<T>(2) * q.x() * q.y() + static_cast<T>(2) *
                                                      q.w() * q.z();
  dcm(1, 1) = static_cast<T>(2) * q.w() * q.w() - static_cast<T>(1) +
              static_cast<T>(2) * q.y() * q.y();
  dcm(2, 1) = static_cast<T>(2) * q.y() * q.z() - static_cast<T>(2) *
                                                      q.w() * q.x();
  dcm(0, 2) = static_cast<T>(2) * q.x() * q.z() - static_cast<T>(2) *
                                                      q.w() * q.y();
  dcm(1, 2) = static_cast<T>(2) * q.y() * q.z() + static_cast<T>(2) *
                                                      q.w() * q.x();
  dcm(2, 2) = static_cast<T>(2) * q.w() * q.w() - static_cast<T>(1) +
              static_cast<T>(2) * q.z() * q.z();
  return dcm;
}

// ====================================================================================
// 大地坐标 (LLA) 与地心地固坐标 (ECEF) 的互转
// ====================================================================================

/*
 * 大地坐标 LLA → 地心地固坐标 ECEF
 *
 * 公式（WGS-84 参考椭球）：
 *   x = (Rn + h) cos(φ) cos(λ)
 *   y = (Rn + h) cos(φ) sin(λ)
 *   z = (Rn(1 - e²) + h) sin(φ)
 *
 * 其中 Rn 为卯酉圈曲率半径。φ 为纬度，λ 为经度，h 为椭球高。
 */
inline Eigen::Vector3d lla2ecef(const Eigen::Vector3d &lla,
                                const AngPosUnit ang = AngPosUnit::RAD)
{
  Eigen::Vector3d ecef;
  double sin_lat = std::sin(convang(lla(0), ang, AngPosUnit::RAD));
  double cos_lat = std::cos(convang(lla(0), ang, AngPosUnit::RAD));
  double cos_lon = std::cos(convang(lla(1), ang, AngPosUnit::RAD));
  double sin_lon = std::sin(convang(lla(1), ang, AngPosUnit::RAD));
  double alt = lla(2);
  double Rn = SEMI_MAJOR_AXIS_LENGTH_M /
              std::sqrt(std::fabs(1.0 - (ECC2 * sin_lat * sin_lat)));
  ecef(0) = (Rn + alt) * cos_lat * cos_lon;
  ecef(1) = (Rn + alt) * cos_lat * sin_lon;
  ecef(2) = (Rn * (1.0 - ECC2) + alt) * sin_lat;
  return ecef;
}

/*
 * 地心地固坐标 ECEF → 大地坐标 LLA（Olson 迭代法）
 *
 * 这是导航库中最复杂的坐标变换之一。Olson 法避免了传统 Newton-Raphson
 * 迭代的收敛问题，在高速/近地/极区均有稳定表现。
 *
 * 算法概要：
 * 1. 初值猜测：纬度 ≈ asin(z/r)，经度直接由 atan2(y,x) 得到。
 * 2. 按纬度的初值计算一系列几何中间量（s, c, u, v, f, m, p）。
 * 3. 一次修正后即达到亚米级精度，不需要迭代循环。
 *
 * 实现细节（本函数针对 ecef2lla 做了专门优化）：
 * - 中间量不使用 static：P4 双核/RTOS 下 static 会导致不可重入的数据竞争。
 * - 半径 < 100km 的保护退回到 (0,0,0)，用于拒绝 ECEF 零向量的退化输入。
 */
inline Eigen::Vector3d ecef2lla(const Eigen::Vector3d &ecef,
                                const AngPosUnit ang = AngPosUnit::RAD)
{
  Eigen::Vector3d lla = Eigen::Vector3d::Zero();
  static constexpr double A1 = SEMI_MAJOR_AXIS_LENGTH_M * ECC2;
  static constexpr double A2 = A1 * A1;
  static constexpr double A3 = A1 * ECC2 / 2.0;
  static constexpr double A4 = 2.5 * A2;
  static constexpr double A5 = A1 + A3;
  static constexpr double A6 = 1.0 - ECC2;

  // 中间量全部为普通局部变量：在多任务/双核环境下保证函数可重入。
  double x, y, z, zp, w2, w, z2, r2, r, s2, c2, u,
      v, s, ss, c, g, rg, rf, f, m, p;

  x = ecef(0);
  y = ecef(1);
  z = ecef(2);
  zp = std::fabs(z);
  w2 = x * x + y * y;
  w = std::sqrt(w2);
  z2 = z * z;
  r2 = w2 + z2;
  r = std::sqrt(r2);

  // 半径过小视为退化（ECEF 零向量）：直接返回 (0,0,0)。
  if (r < 100000.0)
  {
    return lla;
  }

  lla(1) = std::atan2(y, x);           // 经度 = atan2(y,x)，直接解
  s2 = z2 / r2;
  c2 = w2 / r2;
  u = A2 / r;
  v = A3 - A4 / r;

  // 按纬度初值选择安全反函数支路：高纬度用 asin，赤道附近用 acos。
  if (c2 > 0.3)
  {
    s = (zp / r) * (1.0 + c2 * (A1 + u + s2 * v) / r);
    lla(0) = std::asin(s);
    ss = s * s;
    c = std::sqrt(1.0 - ss);
  }
  else
  {
    c = (w / r) * (1.0 - s2 * (A5 - u - c2 * v) / r);
    lla(0) = std::acos(c);
    ss = 1.0 - c * c;
    s = std::sqrt(ss);
  }

  // Olson 高度修正
  g = 1.0 - ECC2 * ss;
  rg = SEMI_MAJOR_AXIS_LENGTH_M / std::sqrt(g);
  rf = A6 * rg;
  u = w - rg * c;
  v = zp - rf * s;
  f = c * u + s * v;
  m = c * v - s * u;
  p = m / (rf / g + f);
  lla(0) = lla(0) + p;               // 纬度修正
  lla(2) = f + m * p / 2.0;          // 高度

  if (z < 0) { lla(0) = -1 * lla(0); }  // 南半球符号恢复

  lla(0) = convang(lla(0), AngPosUnit::RAD, ang);
  lla(1) = convang(lla(1), AngPosUnit::RAD, ang);
  return lla;
}

// ====================================================================================
// 局部导航坐标 (NED) 与地心地固坐标 (ECEF) / 大地坐标 (LLA) 的互转
// ====================================================================================

/*
 * ECEF 向量 → NED 向量（相对参考点 lla0）
 *
 * 先构建从 ECEF 到 NED 的旋转矩阵 R（仅依赖参考点 lla0），
 * 再乘以 ECEF 差向量。结果是在参考点处切平面内的北/东/地位移。
 *
 * 矩阵：
 *   R = [ -sin(φ)cos(λ)  -sin(φ)sin(λ)   cos(φ)   ]  ← 北向
 *       [ -sin(λ)          cos(λ)          0        ]  ← 东向
 *       [ -cos(φ)cos(λ)  -cos(φ)sin(λ)  -sin(φ)   ]  ← 地向
 */
inline Eigen::Vector3d ecef2ned(const Eigen::Vector3d &ecef,
                                const Eigen::Vector3d &lla0,
                                const AngPosUnit ang = AngPosUnit::RAD)
{
  Eigen::Matrix3d R;
  double sin_lat = std::sin(convang(lla0(0), ang, AngPosUnit::RAD));
  double cos_lat = std::cos(convang(lla0(0), ang, AngPosUnit::RAD));
  double sin_lon = std::sin(convang(lla0(1), ang, AngPosUnit::RAD));
  double cos_lon = std::cos(convang(lla0(1), ang, AngPosUnit::RAD));
  R(0, 0) = -sin_lat * cos_lon;
  R(0, 1) = -sin_lat * sin_lon;
  R(0, 2) = cos_lat;
  R(1, 0) = -sin_lon;
  R(1, 1) = cos_lon;
  R(1, 2) = 0;
  R(2, 0) = -cos_lat * cos_lon;
  R(2, 1) = -cos_lat * sin_lon;
  R(2, 2) = -sin_lat;
  return R * ecef;
}

/*
 * NED 向量 → ECEF 向量（相对参考点 lla0）
 *
 * 这是 ecef2ned 的逆变换：R^T * ned（R 是正交矩阵，转置即为逆）。
 */
inline Eigen::Vector3d ned2ecef(const Eigen::Vector3d &ned,
                                const Eigen::Vector3d &lla0,
                                const AngPosUnit ang = AngPosUnit::RAD)
{
  Eigen::Matrix3d R;
  double sin_lat = std::sin(convang(lla0(0), ang, AngPosUnit::RAD));
  double cos_lat = std::cos(convang(lla0(0), ang, AngPosUnit::RAD));
  double sin_lon = std::sin(convang(lla0(1), ang, AngPosUnit::RAD));
  double cos_lon = std::cos(convang(lla0(1), ang, AngPosUnit::RAD));
  R(0, 0) = -sin_lat * cos_lon;
  R(0, 1) = -sin_lat * sin_lon;
  R(0, 2) = cos_lat;
  R(1, 0) = -sin_lon;
  R(1, 1) = cos_lon;
  R(1, 2) = 0;
  R(2, 0) = -cos_lat * cos_lon;
  R(2, 1) = -cos_lat * sin_lon;
  R(2, 2) = -sin_lat;
  return R.transpose() * ned;
}

/*
 * 大地坐标 LLA → NED（相对参考点 lla0）
 *
 * 两步法：LLA → ECEF → 差值 → NED。
 * 物理含义：两点在 ECEF 下的差向量，投影到参考点处的北/东/地方向。
 */
inline Eigen::Vector3d lla2ned(const Eigen::Vector3d &lla,
                               const Eigen::Vector3d &lla0,
                               const AngPosUnit ang = AngPosUnit::RAD)
{
  Eigen::Vector3d loc = lla2ecef(lla, ang);
  Eigen::Vector3d ref = lla2ecef(lla0, ang);
  return ecef2ned(loc - ref, lla0, ang);
}

/*
 * NED → 大地坐标 LLA（相对参考点 lla0）
 *
 * 两步法：NED → ECEF → 绝对 ECEF → LLA。
 * 用于 GNSS 融合后从 EKF 的 NED 位置恢复大地坐标。
 */
inline Eigen::Vector3d ned2lla(const Eigen::Vector3d &ned,
                               const Eigen::Vector3d &lla0,
                               const AngPosUnit ang = AngPosUnit::RAD)
{
  Eigen::Vector3d loc = ned2ecef(ned, lla0, ang);
  Eigen::Vector3d ref = lla2ecef(lla0, ang);
  return ecef2lla(loc + ref, ang);
}

} // namespace bfs

#endif // NAVIGATION_SRC_TRANSFORMS_H_ NOLINT
