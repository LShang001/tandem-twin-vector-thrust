#pragma once

#include "Eigen/Dense"

/*
 * 一阶协方差传播块展开 helper
 * ===========================
 *
 * 当前 15 状态误差模型的 F 矩阵在 fast first-order covariance 路径下具有固定稀疏结构。
 * 本 helper 精确计算：
 *
 *   (F * dt) * P
 *
 * 但不再走一次通用 15x15 稠密矩阵乘法，而是按已知非零块直接展开。
 *
 * 注意：
 * - 这里只负责返回左乘结果 fs_p，调用方仍需执行：
 *     P += fs_p + fs_p^T + Q
 * - 数学上与当前稠密实现等价，不引入新的近似。
 */
inline Eigen::Matrix<float, 15, 15> Icm45686FirstOrderCovarianceDeltaFromBlocks(
    const Eigen::Matrix<float, 15, 15> &p,
    const Eigen::Matrix3f &vel_att_coupling,
    const Eigen::Matrix3f &vel_accel_bias_coupling,
    const Eigen::Matrix3f &att_gyro_coupling,
    const Eigen::Matrix3f &accel_bias_markov,
    const Eigen::Matrix3f &gyro_bias_markov,
    float gravity_height_coupling,
    float dt_s
#if BFS_NAVIGATION_EMBEDDED_FULL_F_MATRIX
    ,
    const Eigen::Matrix3f &vel_vel_coupling,
    const Eigen::Matrix3f &pos_pos_coupling,
    const Eigen::Matrix3f &att_vel_coupling
#endif
    )
{
  Eigen::Matrix<float, 15, 15> fs_p = Eigen::Matrix<float, 15, 15>::Zero();

  // pos <- vel: 上 3 行直接等于 P 的速度误差行块。
  fs_p.block<3, 15>(0, 0) = p.block<3, 15>(3, 0);
#if BFS_NAVIGATION_EMBEDDED_FULL_F_MATRIX
  fs_p.block<3, 15>(0, 0).noalias() += pos_pos_coupling * p.block<3, 15>(0, 0);
#endif

  // vel <- att / accel_bias，再叠加 D 向对高度误差的一阶重力耦合。
  fs_p.block<3, 15>(3, 0).noalias() =
      vel_att_coupling * p.block<3, 15>(6, 0);
  fs_p.block<3, 15>(3, 0).noalias() +=
      vel_accel_bias_coupling * p.block<3, 15>(9, 0);
  fs_p.row(5).noalias() += gravity_height_coupling * p.row(2);
#if BFS_NAVIGATION_EMBEDDED_FULL_F_MATRIX
  fs_p.block<3, 15>(3, 0).noalias() += vel_vel_coupling * p.block<3, 15>(3, 0);
#endif

  // att <- att / gyro_bias。
  fs_p.block<3, 15>(6, 0).noalias() =
      att_gyro_coupling * p.block<3, 15>(6, 0);
  fs_p.block<3, 15>(6, 0).noalias() -= p.block<3, 15>(12, 0);
#if BFS_NAVIGATION_EMBEDDED_FULL_F_MATRIX
  fs_p.block<3, 15>(6, 0).noalias() += att_vel_coupling * p.block<3, 15>(3, 0);
#endif

  // accel_bias <- accel_bias。
  fs_p.block<3, 15>(9, 0).noalias() =
      accel_bias_markov * p.block<3, 15>(9, 0);

  // gyro_bias <- gyro_bias。
  fs_p.block<3, 15>(12, 0).noalias() =
      gyro_bias_markov * p.block<3, 15>(12, 0);

  fs_p *= dt_s;
  return fs_p;
}

/*
 * 高精度 Simpson Qd 路径使用的连续过程噪声 Qc 结构化展开
 * =======================================================
 *
 * 当前 EKF 的 Gs/Rw 结构固定且互不相关：
 * - accel 白噪声驱动速度误差
 * - gyro 白噪声驱动态度误差
 * - accel bias 随机游走驱动加速度零偏误差
 * - gyro bias 随机游走驱动陀螺零偏误差
 *
 * 由于速度噪声通道里出现的姿态矩阵是正交矩阵，C * sigma^2I * C^T
 * 仍然等于 sigma^2I。因此该函数与稠密的 Gs * Rw * Gs^T 完全等价，
 * 但避免了高精度路径里每帧一次 15x12/12x12/12x15 的稠密乘法。
 */
inline Eigen::Matrix<double, 15, 15> Icm45686StructuredContinuousQc(
    double accel_std_mps2,
    double gyro_std_radps,
    double accel_markov_bias_std_mps2,
    double accel_tau_s,
    double gyro_markov_bias_std_radps,
    double gyro_tau_s)
{
  Eigen::Matrix<double, 15, 15> qc =
      Eigen::Matrix<double, 15, 15>::Zero();

  /*
   * 原高精度路径先在 Ekf15State::Initialize() 中把 rw_ 保存为 float，
   * 随后在传播时执行 rw_.cast<double>()。这里也先按 float 规则得到相同方差项，
   * 再转 double，保证结构化路径复现原始数值行为，而不是悄悄改变过程噪声大小。
   */
  const float accel_std_f = static_cast<float>(accel_std_mps2);
  const float gyro_std_f = static_cast<float>(gyro_std_radps);
  const float accel_bias_std_f =
      static_cast<float>(accel_markov_bias_std_mps2);
  const float accel_tau_f = static_cast<float>(accel_tau_s);
  const float gyro_bias_std_f = static_cast<float>(gyro_markov_bias_std_radps);
  const float gyro_tau_f = static_cast<float>(gyro_tau_s);
  const float accel_var_f = accel_std_f * accel_std_f;
  const float gyro_var_f = gyro_std_f * gyro_std_f;
  const float accel_bias_var_f =
      2.0f * accel_bias_std_f * accel_bias_std_f / accel_tau_f;
  const float gyro_bias_var_f =
      2.0f * gyro_bias_std_f * gyro_bias_std_f / gyro_tau_f;

  qc.block<3, 3>(3, 3).diagonal().setConstant(
      static_cast<double>(accel_var_f));
  qc.block<3, 3>(6, 6).diagonal().setConstant(
      static_cast<double>(gyro_var_f));
  qc.block<3, 3>(9, 9).diagonal().setConstant(
      static_cast<double>(accel_bias_var_f));
  qc.block<3, 3>(12, 12).diagonal().setConstant(
      static_cast<double>(gyro_bias_var_f));
  return qc;
}
