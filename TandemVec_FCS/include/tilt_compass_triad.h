/**
 * @file    tilt_compass_triad.h
 * @module   固件/头文件
 * @brief   TRIAD 双矢量静基座初始对准（替代 TiltCompass 欧拉角公式）。
 *
 * 动机：`TiltCompass` 用 `asin(a_x)`、`asin(-a_y/cosθ)` 求 roll/pitch，
 * 在 |pitch|→90° 时 `1/cosθ` 奇异（REF-120 §9.5.1 p.284–285）。TRIAD 双矢量
 * 由"重力方向 + 磁北方向"直接构造 C_b^n，再转四元数，无奇异点。
 *
 * 参考：REF-120 §9.9.1 (9.9.1)–(9.9.5) p.323–324（g + 磁/航向双矢量定姿）。
 *
 * 约定：输入 accel 为 FRD 比力（静止指向天顶）、mag 为 µT；输出
 * [yaw, pitch, roll] rad（与 TiltCompass 同约定），失败返回零向量。
 */
#pragma once

#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>

/**
 * @param accel_frd   加速度计比力（FRD，静止时指向天顶）
 * @param mag_uT      磁力计向量（任一架次坐标）
 * @param mag_ned_uT  参考磁矢量（NED，µT；用项目 kDefaultMagReference）
 * @return [yaw, pitch, roll] rad；输入无效时返回零向量
 */
inline Eigen::Vector3f TiltCompassTRIAD(
    const Eigen::Vector3f &accel_frd, const Eigen::Vector3f &mag_uT,
    const Eigen::Vector3f &mag_ned_uT)
{
  const float an = accel_frd.norm();
  if (!(an > 1e-3f) || !std::isfinite(an))
  {
    return Eigen::Vector3f::Zero();
  }
  const Eigen::Vector3f up_b = accel_frd / an;
  const Eigen::Vector3f up_n(0.0f, 0.0f, -1.0f);  // NED 天顶

  // 磁矢量的水平分量（去掉天顶投影），分别用于构造北向基矢
  const Eigen::Vector3f h_b = (mag_uT - up_b * mag_uT.dot(up_b));
  const Eigen::Vector3f h_n = (mag_ned_uT - up_n * mag_ned_uT.dot(up_n));
  if (!(h_b.norm() > 1e-6f) || !(h_n.norm() > 1e-6f) ||
      !std::isfinite(h_b.norm()) || !std::isfinite(h_n.norm()))
  {
    return Eigen::Vector3f::Zero();
  }
  const Eigen::Vector3f nb = h_b.normalized();
  const Eigen::Vector3f nn = h_n.normalized();
  // 右手正交基：{水平磁北, up×磁北, up}
  const Eigen::Vector3f eb = up_b.cross(nb);
  const Eigen::Vector3f en = up_n.cross(nn);
  if (!(eb.norm() > 1e-6f) || !(en.norm() > 1e-6f))
  {
    return Eigen::Vector3f::Zero();
  }
  Eigen::Matrix3f B_b, N_n;
  B_b.col(0) = nb;
  B_b.col(1) = eb.normalized();
  B_b.col(2) = up_b;
  N_n.col(0) = nn;
  N_n.col(1) = en.normalized();
  N_n.col(2) = up_n;
  // C_b^n = N_n · B_b^T（把 body 矢量转到 NED）
  const Eigen::Matrix3f C_bn = N_n * B_b.transpose();

  // ZYX 欧拉角提取（与 TiltCompass 输出同约定）
  const float pitch = std::asin(-std::fmax(-1.0f, std::fmin(1.0f, C_bn(2, 0))));
  const float roll = std::atan2(C_bn(2, 1), C_bn(2, 2));
  const float yaw = std::atan2(C_bn(1, 0), C_bn(0, 0));
  return Eigen::Vector3f(yaw, pitch, roll);
}
