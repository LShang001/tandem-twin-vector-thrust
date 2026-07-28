/*
 * 通用数学工具函数
 * ================
 *
 * 提供导航计算中最常用的数学基元，所有函数均为模板化 inline 实现，
 * 支持 float / double 两种浮点类型，零运行时间接调用开销：
 *
 * - WrapTo2Pi：将角度从 [-π, +π] 映射到 [0, 2π)，用于连续正角度表达。
 * - WrapToPi：将角度从 [0, 2π) 映射回 [-π, +π]，用于 EKF 新息的角度环绕处理。
 * - Skew：构造三维向量的斜对称矩阵 [w]×，满足 [w]× v = w × v，
 *   广泛用于姿态运动学 (dC/dt)、状态转移矩阵 Fs 的交叉耦合块、
 *   Phi 矩阵展开等需叉积矩阵化表示的场景。
 */

#ifndef NAVIGATION_SRC_UTILS_H_ // NOLINT
#define NAVIGATION_SRC_UTILS_H_

#include "eigen.h" // NOLINT
#include "Eigen/Dense"

namespace bfs
{

/*
 * 将角度从 [-π, +π] 区间归一化到 [0, 2π) 区间。
 *
 * 应用场景：
 * - 欧拉角/航向角的连续正角度表达。
 * - 先 fmod 取余，再处理负值使其落入 [0, 2π)。
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

/*
 * 将角度从 [0, 2π) 区间归一化到 [-π, +π] 区间。
 *
 * 应用场景：
 * - 卡尔曼滤波的新息（Innovation）必须在此区间内，否则线性化假设失效。
 * - 角度大于 +π 则减去 2π，小于 -π 则加上 2π。
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

/*
 * 根据三维向量 w 构造斜对称矩阵 (Skew-Symmetric Matrix)。
 *
 * 矩阵形式：
 *         ┌          ┐
 *         │ 0  -wz  wy│
 *   [w]× =│ wz   0  -wx│
 *         │-wy  wx   0 │
 *         └          ┘
 *
 * 核心性质：[w]× v = w × v，即斜对称矩阵乘向量等价于叉积。
 *
 * 应用场景：
 * - 姿态传播：NED 角速度 → 斜对称形式 → DCM 更新微分方程。
 * - 状态转移矩阵 Fs：加速度比力的斜对称块参与姿态误差传播。
 * - Phi 矩阵展开：Fs 中加速度和角速度的交叉耦合项。
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
