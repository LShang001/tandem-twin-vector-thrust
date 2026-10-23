/**
 * @file    icm45686_ins_fast_covariance_blocks.h
 * @module  固件/基础设施（EKF 15 状态协方差传播的块展开加速，header-only）
 * @brief   按 F 矩阵已知稀疏结构直接展开 (F·dt)·P 与结构化过程噪声 Qc
 * @details 输入：15×15 协方差 P、各耦合块（3×3）、时间步 dt_s、噪声标准差；
 *          处理：不构造稠密 F，而按“位置←速度 / 速度←姿态、加计零偏、重力-高度 /
 *                姿态←姿态、陀螺零偏 / 零偏 Markov”逐块左乘；Qc 用 Gs·Rw·Gsᵀ 的解析等价式；
 *          输出：`(F·dt)·P` 的左乘结果 fs_p（调用方仍需 `P += fs_p + fs_pᵀ + Q`）
 *          或 15×15 连续过程噪声 Qc。
 *          不变量：与稠密实现**数学等价**，不引入新近似（块稀疏结构即 F 的结构）。
 * @theory  一阶协方差传播 P_{k+1} = (I + F·dt)·P_k·(I + F·dt)ᵀ + Q_d
 *          ≈ P + (F·dt)·P + P·(F·dt)ᵀ + Q_d；
 *          连续噪声离散化 Q_d = ∫ Φ·G·Q_c·Gᵀ·Φᵀ dτ（一阶近似 = G·Q_c·Gᵀ·dt）。
 * @ref     REF-121 §2.2.1 (2.2.4d) p.33（KF 协方差预测方程）；
 *          REF-120 §9.6（捷联误差方程 F 的分块结构）；@ref src/ekf_15_state.h（状态编号约定）
 * @frame   状态向量按 [位置(0-2) 速度(3-5) 姿态失准角(6-8) 加计零偏(9-11) 陀螺零偏(12-14)]
 *          排列；姿态为体轴右乘误差 δβ（见 AGENTS.md 导航踩坑记录）
 * @units   P 各块单位随状态量纲（m²、m²/s²、rad²、m²/s⁴…）；dt_s 秒；标准差 m/s²、rad/s
 * @note    ① 只有 `BFS_NAVIGATION_EMBEDDED_FULL_F_MATRIX` 打开时才叠加 vel-vel、pos-pos、
 *             att-vel 三个额外耦合块——默认嵌入式路径不启用；
 *          ② Qc 的等价性依赖“速度噪声通道里 C 为正交阵 ⇒ C·σ²I·Cᵀ = σ²I”，
 *             改 Gs/Rw 结构前必须重新核对该假设（见 Icm45686StructuredContinuousQc 注释）。
 * @see     lib/navigation-main/src/ekf_15_state.h（消费方）、docs/08-参考资料 notes
 */
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
