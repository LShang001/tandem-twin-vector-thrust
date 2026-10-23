/**
 * @file    transforms.h
 * @module  固件/导航（坐标系与姿态表示变换）
 * @brief   欧拉角/DCM/四元数互转与 LLA/ECEF/NED 互转的 header-only 工具集
 * @details 输入：角度、DCM、四元数、LLA/ECEF/NED 向量；处理：按 ZYX Tait-Bryan
 *          约定合成或反解旋转，按 WGS-84 椭球做“大地坐标 ↔ 地心地固 ↔ 局部切
 *          平面”两步换算；输出：对应表示的向量/矩阵。
 *          全部为 Eigen 模板化 inline 实现，支持 float/double；中间量均为普通
 *          局部变量，无静态状态，故可重入（P4 双核/RTOS 约束）。
 *          不变量：欧拉角顺序 [Yaw, Pitch, Roll]；角度默认 RAD；LLA=(纬度,经度,高)。
 * @theory  3-2-1 欧拉角与姿态阵（theta = −asin(R13)）@ref MOD-002 §3；
 *          四元数与姿态阵关系 @ref REF-120 §9.2.2 p.256–258；
 *          四元数乘法矩阵 @ref REF-120 §9.2.2 (9.2.14)；
 *          WGS-84 大地坐标/椭球几何 @ref REF-120 §7.2.1 表 7.2.1；
 *          NED 切平面与导航系定义 @ref REF-120 §9.8.1 (9.8.1) p.317–318
 * @frame   姿态：机体系 FRD → 导航系 NED；位置：LLA(WGS-84) / ECEF / NED 三系
 * @units   角度 rad（可用 AngPosUnit::DEG 显式指定）；位置 m；DCM/四元数无量纲
 * @note    `ecef2lla` 采用 Olson 迭代法单次修正（亚米级精度），半径 <100 km 视为
 *          退化输入并返回 (0,0,0)；`dcm2quat` 的 w=½√(1+tr C) 分支在 tr(C)→−1
 *          （180° 旋转）时 4w→0 会放大误差，当前库内无调用点，启用前需改
 *          trace 分支法并补测试。
 * @see     earth_model.h、ekf_15_state.h、docs/04-数学建模/MOD-002-坐标系与符号约定.md
 */

#ifndef NAVIGATION_SRC_TRANSFORMS_H_ // NOLINT
#define NAVIGATION_SRC_TRANSFORMS_H_

#include "units.h" // NOLINT
#include "eigen.h" // NOLINT
#include "Eigen/Dense"

namespace bfs
{

// ====================================================================================
// 姿态表示之间的互转：欧拉角 / DCM / 四元数（ZYX Tait-Bryan）
// ====================================================================================
// 转换关系总览（=> 表示“可由下式得到”）：
//   欧拉角 (yaw,pitch,roll) --angle2dcm--> DCM --dcm2quat--> 四元数
//   四元数 --quat2dcm--> DCM --dcm2angle--> 欧拉角
//   欧拉角 --angle2quat--> 四元数 --quat2angle--> 欧拉角
// eul2*/quat2eul/dcm2eul 为等价包装，仅为调用语义清晰。
// 全部角度默认 rad；DCM 语义为“机体系 → 导航系”（与 ekf_15_state.h 的
// t_b2ned = quat2dcm(quat_)^T 互为转置，勿混用）。
// ====================================================================================

/**
 * @brief 由 3-2-1 欧拉角合成方向余弦矩阵 C^n_b
 * @param[in] rot1 偏航角 yaw ψ，单位由 ang 指定
 * @param[in] rot2 俯仰角 pitch θ
 * @param[in] rot3 横滚角 roll φ
 * @param[in] ang  输入角度单位，默认 RAD
 * @return 3×3 旋转阵（机体系→导航系），C^b_n = C^T
 * @formula C = R_x(roll)·R_y(pitch)·R_z(yaw)，即先绕 Z 转 yaw、再 Y 转 pitch、
 *          最后 X 转 roll
 * @ref MOD-002 §3（3-2-1 顺序、theta = −asin(R13)）；REF-120 §9.2.2 p.256–258
 * @note 与 dcm2angle() 严格互逆。外部 ENU 数据导入前必须先做 ENU→NED 轴置换，
 *       否则分量次序与符号会错位（见 MOD-002 §2 的等价变换说明）。
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

/**
 * @brief 由方向余弦阵提取 3-2-1 欧拉角 [yaw, pitch, roll]
 * @param[in] dcm 旋转阵（机体系→导航系）
 * @param[in] ang 输出角度单位，默认 RAD
 * @return 列向量 {yaw, pitch, roll}，单位由 ang 指定
 * @formula pitch = −asin(C(0,2))；yaw = atan2(C(0,1), C(0,0))；
 *          roll = atan2(C(1,2), C(2,2))
 * @ref MOD-002 §3（本项目 theta = −asin(R13) 约定，R13 即 C(0,2)）；
 *      REF-120 §9.2.2 p.256–258
 * @note 与 angle2dcm() 的 rot1/rot2/rot3=(yaw,pitch,roll) 顺序对应；
 *       pitch=±90° 时 C(0,0)=C(0,1)=0，yaw/roll 不可分辨（万向节锁），
 *       此时输出依赖浮点舍入，调用方应避免该姿态作为控制参考。
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

/**
 * @brief 由 3-2-1 欧拉角构造四元数
 * @param[in] rot1 偏航角 yaw ψ，单位由 a 指定
 * @param[in] rot2 俯仰角 pitch θ
 * @param[in] rot3 横滚角 roll φ
 * @param[in] a    输入角度单位，默认 RAD
 * @return 单位四元数（Eigen 分量顺序 w,x,y,z）
 * @formula w = c1c2c3 + s1s2s3，x = c1c2s3 − s1s2c3，
 *          y = c1s2c3 + s1c2s3，z = s1c2c3 − c1s2s3；
 *          其中 c1=cos(ψ/2)、s1=sin(ψ/2)，c2/c3 分别对应 θ/2、φ/2
 * @ref REF-120 §9.2.2 p.256–258（四元数与姿态阵的等价关系）
 * @note 半角展开只对归一化四元数成立；调用方无需再归一化（正交三角恒等式
 *       保证 |q|=1），但经过外部数值处理后仍应检查。
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

/**
 * @brief 由四元数提取 3-2-1 欧拉角 [yaw, pitch, roll]
 * @param[in] q   单位四元数（w,x,y,z）
 * @param[in] ang 输出角度单位，默认 RAD
 * @return 列向量 {yaw, pitch, roll}
 * @formula 直接由四元数分量构造等效 DCM 元素：
 *          m11 = 2w²+2x²−1，m12 = 2xy+2wz，m13 = 2xz−2wy，
 *          m23 = 2yz+2wx，m33 = 2w²+2z²−1；
 *          roll = atan2(m12, m11)，pitch = −asin(m13)，yaw = atan2(m23, m33)
 * @ref MOD-002 §3（theta = −asin(R13)）；REF-120 §9.2.2 p.256–258
 * @note 跳过显式 3×3 矩阵构造，只算 5 个元素，供 200 Hz 遥测/控制路径调用；
 *       与 dcm2angle(quat2dcm(q)) 数学等价（浮点末位可能差 1 ulp）。
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

/**
 * @brief 由方向余弦阵提取四元数（trace 法）
 * @param[in] dcm 3×3 旋转阵（正交，行列式 +1）
 * @return 单位四元数（w≥0 分支）
 * @formula w = ½√(1+C(0,0)+C(1,1)+C(2,2))；x = (C(1,2)−C(2,1))/(4w)；
 *          y = (C(2,0)−C(0,2))/(4w)；z = (C(0,1)−C(1,0))/(4w)
 * @ref REF-120 §9.2.2 p.256–258（四元数与姿态阵关系）
 * @deviation 只在 trace(C) > −1 时数值稳定；接近 180° 旋转时 w→0，除法放大
 *            舍入误差。当前库内无调用点（EKF 全程用四元数传播），若启用需改为
 *            “选 trace 最大的主分量”分支法并补 180° 用例。已登记至核查记录。
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

/**
 * @brief 由单位四元数构造方向余弦阵 C^n_b
 * @param[in] q 单位四元数（w,x,y,z）
 * @return 3×3 旋转阵 C^n_b；姿态传播中的 C^b_n 为其转置
 * @formula
 *   ┌                                                   ┐
 *   │ 2w²−1+2x²   2xy−2wz     2xz+2wy                   │
 *   │ 2xy+2wz     2w²−1+2y²   2yz−2wx                   │
 *   │ 2xz−2wy     2yz+2wx     2w²−1+2z²                 │
 *   └                                                   ┘
 * @ref REF-120 §9.2.2 p.256–258（四元数 ↔ 姿态阵等价关系）；
 *      REF-120 §9.2.5 p.265（四元数规范化后使用）
 * @note 这是姿态传播最常用的变换：EKf15State 每次姿态更新后用
 *       `t_b2ned = quat2dcm(quat_).transpose()` 缓存 C^b_n，避免在 200 Hz
 *       传播逐帧重复转置。
 * @pre |q| = 1；非单位四元数会得到带尺度项的“伪 DCM”
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
// 椭球参数取自 constants.h（WGS-84）。LLA 顺序固定为 (纬度, 经度, 椭球高)，
// 纬度/经度默认 rad；高度为 WGS-84 椭球高（非海拔高、非相对起飞点高）。
// 与 ekf_15_state.h 的接口约定一致：EKF 内部始终保存绝对 LLA。
// ====================================================================================

/**
 * @brief 大地坐标 LLA → 地心地固坐标 ECEF
 * @param[in] lla (纬度, 经度, 椭球高)，角度单位由 ang 指定，高度 m
 * @param[in] ang 角度输入单位，默认 RAD
 * @return ECEF 坐标 [m]，地心为原点、Z 轴沿自转轴
 * @formula x = (Rn+h)cosφcosλ，y = (Rn+h)cosφsinλ，
 *          z = (Rn(1−e²)+h)sinφ；Rn = a/√(1−e²sin²φ)
 * @ref REF-120 §7.2.1 表 7.2.1（WGS-84 椭球参数）；
 *      REF-120 §7.2.2（天文/地理/地心纬度与垂线定义）
 * @note Rn 用 std::fabs() 保证被开方项非负；正常纬度下该保护不触发，
 *       仅在极端错误输入（|sinφ|>1 的浮点溢出）时避免 NaN 传播。
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

/**
 * @brief 地心地固坐标 ECEF → 大地坐标 LLA（Olson 单次修正法）
 * @param[in] ecef ECEF 坐标 [m]
 * @param[in] ang  输出角度单位，默认 RAD
 * @return (纬度, 经度, 椭球高)；输入半径 <100 km 时返回 (0,0,0)
 * @formula 经度 λ = atan2(y,x)；纬度由初值 s = (|z|/r)(1 + c²(A1+u+s²v)/r)
 *          作一次修正 φ ← φ + p，高度 h = f + m·p/2（系数 A1..A6 见实现）
 * @ref REF-120 §7.2.1 表 7.2.1（WGS-84 椭球）；REF-120 §7.2.2
 * @deviation 不做迭代：Olson 一次修正的标称误差为亚米级（与 MATLAB 参考实现
 *            一致），但 ECEF 点远离椭球面（|h| 数十 km）时误差增大；本工程
 *            只用飞行高度范围内的数据，故保留该实现并登记于核查记录。
 * @note 中间量全部为自动变量（非 static），保证 P4 双核/RTOS 下可重入；
 *       分支 c²>0.3 用 asin 求纬度、否则用 acos，避免极区反函数病态。
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

/**
 * @brief ECEF 向量 → NED 向量（相对参考点 lla0 的切平面投影）
 * @param[in] ecef ECEF 向量（通常是两点 ECEF 差）[m]
 * @param[in] lla0 参考点 (纬度, 经度, 高)，角度单位由 ang 指定
 * @param[in] ang  角度单位，默认 RAD
 * @return NED 向量 [m]（北, 东, 地）
 * @formula R = [−sinφcosλ, −sinφsinλ, cosφ; −sinλ, cosλ, 0;
 *              −cosφcosλ, −cosφsinλ, −sinφ]，输出 = R·ecef
 * @ref REF-120 §9.8.1 (9.8.1) p.317–318（NED 导航系定义与速度方程）；
 *      REF-120 §7.2.2（地理纬度/垂线）
 * @note R 为正交阵，故逆变换就是 R^T（见 ned2ecef）；切平面投影假设参考点
 *       附近曲率为常数，数十 km 级位移会被曲率效应放大（本工程只用
 *       起飞点附近的相对位移）。
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

/**
 * @brief NED 向量 → ECEF 向量（ecef2ned 的逆变换）
 * @param[in] ned  NED 向量 [m]
 * @param[in] lla0 参考点 (纬度, 经度, 高)
 * @param[in] ang  角度单位，默认 RAD
 * @return ECEF 向量 [m]
 * @formula ecef = R^T·ned（R 的定义见 ecef2ned；正交阵转置即逆）
 * @ref REF-120 §9.8.1 (9.8.1) p.317–318
 * @note 与 ecef2ned() 必须使用同一参考点 lla0，否则差值无物理意义。
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

/**
 * @brief 大地坐标 LLA → NED 位移（相对参考点 lla0）
 * @param[in] lla  目标点 (纬度, 经度, 高)
 * @param[in] lla0 参考点 (纬度, 经度, 高)
 * @param[in] ang  角度单位，默认 RAD
 * @return NED 位移 [m]
 * @formula Δned = ecef2ned(lla2ecef(lla) − lla2ecef(lla0), lla0)
 * @ref REF-120 §9.8.1 (9.8.1) p.317–318（NED 导航系）
 * @note 两步法（LLA→ECEF→差值→NED）比“纬度/经度差×曲率半径”的一步法在
 *       大位移/高纬度处更稳定；单次调用含 2 次 ECEF 正算 + 1 次旋转。
 * @test test_host/test_ekf_15state.cpp T4（GNSS 位置残差经本函数换算后收敛）
 */
inline Eigen::Vector3d lla2ned(const Eigen::Vector3d &lla,
                               const Eigen::Vector3d &lla0,
                               const AngPosUnit ang = AngPosUnit::RAD)
{
  Eigen::Vector3d loc = lla2ecef(lla, ang);
  Eigen::Vector3d ref = lla2ecef(lla0, ang);
  return ecef2ned(loc - ref, lla0, ang);
}

/**
 * @brief NED 位移 → 大地坐标 LLA（相对参考点 lla0）
 * @param[in] ned  NED 位移 [m]
 * @param[in] lla0 参考点 (纬度, 经度, 高)
 * @param[in] ang  角度单位，默认 RAD
 * @return 目标点 (纬度, 经度, 椭球高)
 * @formula lla = ecef2lla(ned2ecef(ned, lla0) + lla2ecef(lla0))
 * @ref REF-120 §9.8.1 (9.8.1) p.317–318
 * @note 用于 GNSS 融合后把 EKF 的 NED 位置还原成大地坐标；与 lla2ned() 在
 *       同一 lla0 下互为逆运算，往返误差由 ecef2lla 的一次修正精度决定。
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
