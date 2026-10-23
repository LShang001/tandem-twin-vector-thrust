/**
 * @file    utils.h
 * @module  固件/导航（通用数学基元）
 * @brief   角度归一化与斜对称矩阵构造的 header-only 模板集合
 * @details 输入：角度 [rad] / 三维向量；处理：取模归位、构造 [w]×；
 *          输出：归一化角度或 3×3 反对称矩阵。被 transforms.h、ekf_15_state.h
 *          在姿态误差线性化、新息环绕和状态转移矩阵构造中复用。
 *          不变量：全部 inline 模板，无静态状态，可重入。
 * @theory  叉积的矩阵表示 [w]×v = w×v @ref REF-120 §9.2 p.252–266（矢量叉积与
 *          姿态矩阵的等价关系）；角度环绕用于滤波器新息，避免线性化失效
 *          @ref REF-121 §2.2.1 p.33
 * @frame   与输入向量同坐标系（约定由调用方保证，本文件不隐含 NED/FRD）
 * @units   角度 rad（模板 T 决定精度）；Skew 输入单位随调用方（rad/s、rad、m/s² 等）
 * @note    `WrapToPi` 只做一次 2π 归位：输入在 (−3π, 3π) 内可回到 (−π, π]；
 *          更大角度请先 `std::fmod`（ekf_15_state.h::WrapAngleRad() 采用该做法，
 *          保证耗时确定）。本文件不改变该行为，仅说明适用边界。
 * @see     transforms.h、ekf_15_state.h、docs/08-参考资料/notes/01-捷联惯导理论.md
 */

#ifndef NAVIGATION_SRC_UTILS_H_ // NOLINT
#define NAVIGATION_SRC_UTILS_H_

#include "eigen.h" // NOLINT
#include "Eigen/Dense"

namespace bfs
{

/**
 * @brief 将角度归一化到 [0, 2π)
 * @param[in] ang 输入角度 [rad]（任意实数）
 * @return 等价角度，范围 [0, 2π)
 * @formula ang ← fmod(ang, 2π)；若 ang < 0 则 ang ← ang + 2π
 * @ref REF-121 §2.2.1 p.33（滤波量测/新息方程的周期量处理前提）
 * @note 与 WrapToPi() 互为同一 2π 周期间隔的互补映射；用于欧拉角/航向的
 *       连续正角度表达（如遥测、日志），不用于滤波内部的新息计算。
 */
template <typename T>
T WrapTo2Pi(T ang)
{
  static_assert(std::is_floating_point<T>::value,
                "仅支持浮点类型");
  ang = std::fmod(ang, static_cast<T>(2 * M_PI));
  if (ang < static_cast<T>(0))
  {
    ang += static_cast<T>(2 * M_PI);
  }
  return ang;
}

/**
 * @brief 将角度归一化到 (−π, π]
 * @param[in] ang 输入角度 [rad]
 * @return 等价角度，正常输入下范围 (−π, π]
 * @formula 若 ang > π 则 ang ← ang − 2π；若 ang < −π 则 ang ← ang + 2π（单次归位）
 * @ref REF-121 §2.2.1 p.33（卡尔曼新息必须落在主值区间，否则线性化假设失效）
 * @note 只做一次 2π 归位，因此仅对 (−3π, 3π) 的输入保证结果在主值区间；
 *       更大角度须先 fmod。ekf_15_state.h::WrapAngleRad() 使用 fmod+单次归位，
 *       对任意大输入耗时确定，二者数学意图相同、适用边界不同。
 */
template <typename T>
T WrapToPi(T ang)
{
  static_assert(std::is_floating_point<T>::value,
                "仅支持浮点类型");
  if (ang > static_cast<T>(M_PI))
  {
    ang -= static_cast<T>(2 * M_PI);
  }
  if (ang < -static_cast<T>(M_PI))
  {
    ang += static_cast<T>(2 * M_PI);
  }
  return ang;
}

/**
 * @brief 由三维向量构造斜对称矩阵 [w]×（满足 [w]× v = w × v）
 * @param[in] w 三维向量，单位随调用方（rad/s、rad、m/s² 等）
 * @return 3×3 斜对称矩阵
 * @formula [w]× = [[0, −w_z, w_y], [w_z, 0, −w_x], [−w_y, w_x, 0]]
 * @ref REF-120 §9.2.3 (9.2.47) p.261–262（角速度阵与四元数微分方程）；
 *      REF-120 §9.8.2 (9.8.13)（姿态误差方程中的 φ×ω 项）
 * @note 叉积矩阵化是姿态传播、F 矩阵交叉块（F(V,β)=−C^n_b[f^b]×、
 *       F(β,β)=−[ω^b]×）以及 Φ 展开的公共基元；本函数不做单位换算。
 */
template <typename T>
Eigen::Matrix<T, 3, 3> Skew(const Eigen::Matrix<T, 3, 1> &w)
{
  Eigen::Matrix<T, 3, 3> c;
  c(0, 0) = 0.0;
  c(0, 1) = -w(2);
  c(0, 2) = w(1);
  c(1, 0) = w(2);
  c(1, 1) = 0.0;
  c(1, 2) = -w(0);
  c(2, 0) = -w(1);
  c(2, 1) = w(0);
  c(2, 2) = 0.0;
  return c;
}

} // namespace bfs

#endif // NAVIGATION_SRC_UTILS_H_ NOLINT
